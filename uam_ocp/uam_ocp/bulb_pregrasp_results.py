"""Metrics, files, and comparison plots for P2.7 strategies."""

import csv
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pinocchio as pin
import yaml

from .actuation import UamActuation
from .bulb_pregrasp_planner import BulbPregraspPlanner, BulbStrategySolution
from .model_loader import MODULE_ROOT, UamModel, load_yaml
from .static_trim import StaticTrimSolver
from .terminal_rest import metrics_from_state, terminal_rest_config
from .visualization import save_plots


def evaluate_solution(robot: UamModel, actuation: UamActuation,
                      planner: BulbPregraspPlanner,
                      solution: BulbStrategySolution) -> Tuple[Dict[str, Any], Dict[str, np.ndarray]]:
    """Evaluate terminal task, attitude, bounds, margins, and smoothness."""
    positions=[]; rotations=[]; linear=[]; angular=[]; rpy=[]; poserr=[]; roterr=[]
    for state in solution.states:
        q=state[:robot.model.nq]; v=state[robot.model.nq:]; data=robot.model.createData()
        pin.forwardKinematics(robot.model,data,q,v); pin.updateFramePlacements(robot.model,data)
        ee=data.oMf[robot.end_effector_frame_id]
        velocity=pin.getFrameVelocity(robot.model,data,robot.end_effector_frame_id,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        positions.append(ee.translation.copy()); rotations.append(ee.rotation.copy())
        linear.append(velocity.linear.copy()); angular.append(velocity.angular.copy())
        rpy.append(pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).matrix()))
        poserr.append(np.linalg.norm(ee.translation-solution.target_pose.translation))
        roterr.append(np.linalg.norm(pin.log3(solution.target_pose.rotation.T@ee.rotation)))
    positions=np.asarray(positions); rotations=np.asarray(rotations); linear=np.asarray(linear); angular=np.asarray(angular); rpy=np.asarray(rpy)
    lower,upper=actuation.control_bounds(); controls=solution.controls
    rotor=controls[:,:actuation.n_rotors]; joint=controls[:,actuation.n_rotors:]
    joint_limits=np.maximum(np.abs(lower[actuation.n_rotors:]),np.abs(upper[actuation.n_rotors:]))
    terminal_trim=StaticTrimSolver(robot,actuation).solve_trim(planner.q_seed)
    rest=terminal_rest_config(solution.scenario)
    rest_metrics=metrics_from_state(robot,solution.states[-1],rest)
    static_scenarios = load_yaml(MODULE_ROOT / "config" / "static_trim_scenarios.yaml")
    fully = robot.neutral_configuration(static_scenarios["scenarios"]["fully_extended"]["joints"])
    fully[:3] = np.asarray(static_scenarios["base"]["position"], dtype=float)
    fully[3:7] = np.asarray(static_scenarios["base"]["quaternion_xyzw"], dtype=float)
    fully = pin.normalize(robot.model, fully)
    arm_indices=[item.idx_q for item in robot.arm_joints]
    min_distance_fully=float(np.min([np.linalg.norm(state[:robot.model.nq][arm_indices]-fully[arm_indices]) for state in solution.states]))
    metrics={
        "pass": bool(solution.converged and poserr[-1]<solution.scenario["terminal_position_tolerance_m"] and roterr[-1]<solution.scenario["terminal_orientation_tolerance_rad"] and np.linalg.norm(linear[-1])<solution.scenario["terminal_ee_linear_velocity_tolerance_mps"] and np.linalg.norm(angular[-1])<solution.scenario["terminal_ee_angular_velocity_tolerance_radps"] and rest_metrics["terminal_rest_pass"]),
        "fddp_converged":solution.converged,"iterations":solution.iterations,
        "final_cost":float(solution.costs[-1]) if solution.costs else None,
        "pass_1_iterations":solution.iterations_pass_1,
        "pass_2_iterations":solution.iterations_pass_2,
        "total_fddp_iterations":solution.total_iterations,
        "pass_1_converged":solution.converged_pass_1,
        "pass_2_converged":solution.converged_pass_2,
        "rollout_error":solution.rollout_error,
        "terminal_position_error_m":float(poserr[-1]),"terminal_orientation_error_rad":float(roterr[-1]),
        "terminal_ee_linear_velocity_mps":float(np.linalg.norm(linear[-1])),"terminal_ee_angular_velocity_radps":float(np.linalg.norm(angular[-1])),
        "maximum_base_tilt_rad":float(np.max(np.linalg.norm(rpy[:,:2],axis=1))),
        "maximum_rotor_occupancy":float(np.max(rotor/upper[:actuation.n_rotors])),
        "maximum_joint_torque_occupancy":float(np.max(np.abs(joint)/joint_limits)),
        "minimum_rotor_margin_N":float(np.min(np.minimum(rotor-lower[:4],upper[:4]-rotor))),
        "minimum_joint_torque_margin_Nm":float(np.min(joint_limits-np.abs(joint))),
        "delta_u_cost":float(np.sum(np.diff(controls,axis=0)**2)),
        "input_saturation":bool(np.any(controls<=lower+1e-9) or np.any(controls>=upper-1e-9)),
        "control_bounds_satisfied":bool(np.all(controls>=lower-1e-10) and np.all(controls<=upper+1e-10)),
        "terminal_reference_trim_strict":terminal_trim.strict_feasible,
        "minimum_arm_distance_to_fully_extended_rad":min_distance_fully,
        "passed_near_fully_extended":bool(min_distance_fully<0.15),
        "pose_source":solution.bulb_diagnostics["pose_source"],
    }
    metrics.update(rest_metrics)
    arrays={"ee_position":positions,"ee_rotation":rotations,"ee_linear":linear,"ee_angular":angular,"base_rpy":rpy,"position_error":np.asarray(poserr),"orientation_error":np.asarray(roterr)}
    return metrics,arrays


def save_strategy(robot: UamModel, actuation: UamActuation, solution: BulbStrategySolution,
                  metrics: Dict[str, Any], arrays: Dict[str,np.ndarray], output: Path) -> None:
    """Save required per-strategy trajectory artifacts."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    output.mkdir(parents=True,exist_ok=True); dt=float(solution.scenario["dt"])
    qnames=["base_x","base_y","base_z","base_qx","base_qy","base_qz","base_qw"]+[j.name for j in robot.arm_joints]
    vnames=["base_vx_body","base_vy_body","base_vz_body","base_wx_body","base_wy_body","base_wz_body"]+[j.name+"_velocity" for j in robot.arm_joints]
    unames=[f"rotor_{r['id']}_thrust_N" for r in actuation.rotors]+[n+"_torque_Nm" for n in actuation.joint_names]
    metadata = {
        "model_variant": robot.config.get("model_variant", "unknown"),
        "arm_dof": robot.n_arm,
        "active_arm_joint_names": [j.name for j in robot.arm_joints],
        "locked_joint_names": list(robot.config.get("locked_joint_names", [])),
        "canonical_launch": robot.config.get("canonical_launch"),
        "lock_shoulder_pan": bool(robot.config.get("lock_shoulder_pan", False)),
    }
    np.savez_compressed(output/"trajectory.npz",
        time_s=np.arange(len(solution.states))*dt,dt_s=np.asarray([dt]),
        states=solution.states,controls=solution.controls,solver_states=solution.states,
        trim_references=solution.trim_references,reference_states=solution.reference_states,
        costs=np.asarray(solution.costs),costs_pass_1=np.asarray(solution.costs_pass_1),
        costs_pass_2=np.asarray(solution.costs_pass_2),
        objective_costs_pass_1=np.asarray(solution.diagnostics_pass_1["objective_cost"]),
        objective_costs_pass_2=np.asarray(solution.diagnostics_pass_2["objective_cost"]),
        dynamics_gap_max_pass_1=np.asarray(solution.diagnostics_pass_1["dynamics_gap_max"]),
        dynamics_gap_max_pass_2=np.asarray(solution.diagnostics_pass_2["dynamics_gap_max"]),
        dynamics_gap_sum_squares_pass_1=np.asarray(solution.diagnostics_pass_1["dynamics_gap_sum_squares"]),
        dynamics_gap_sum_squares_pass_2=np.asarray(solution.diagnostics_pass_2["dynamics_gap_sum_squares"]),
        dynamics_gap_penalty_pass_1=np.asarray(solution.diagnostics_pass_1["dynamics_gap_penalty"]),
        dynamics_gap_penalty_pass_2=np.asarray(solution.diagnostics_pass_2["dynamics_gap_penalty"]),
        terminal_ee_position_error_pass_1=np.asarray(solution.diagnostics_pass_1["terminal_ee_position_error_m"]),
        terminal_ee_position_error_pass_2=np.asarray(solution.diagnostics_pass_2["terminal_ee_position_error_m"]),
        terminal_ee_orientation_error_pass_1=np.asarray(solution.diagnostics_pass_1["terminal_ee_orientation_error_rad"]),
        terminal_ee_orientation_error_pass_2=np.asarray(solution.diagnostics_pass_2["terminal_ee_orientation_error_rad"]),
        terminal_base_linear_velocity_norm_pass_1=np.asarray(solution.diagnostics_pass_1["terminal_base_linear_velocity_norm_mps"]),
        terminal_base_linear_velocity_norm_pass_2=np.asarray(solution.diagnostics_pass_2["terminal_base_linear_velocity_norm_mps"]),
        terminal_base_angular_velocity_norm_pass_1=np.asarray(solution.diagnostics_pass_1["terminal_base_angular_velocity_norm_radps"]),
        terminal_base_angular_velocity_norm_pass_2=np.asarray(solution.diagnostics_pass_2["terminal_base_angular_velocity_norm_radps"]),
        terminal_max_arm_joint_velocity_pass_1=np.asarray(solution.diagnostics_pass_1["terminal_max_arm_joint_velocity_radps"]),
        terminal_max_arm_joint_velocity_pass_2=np.asarray(solution.diagnostics_pass_2["terminal_max_arm_joint_velocity_radps"]),
        dynamics_gap_penalty_weight=np.asarray([solution.dynamics_gap_penalty_weight]),
        iterations_pass_1=np.asarray([solution.iterations_pass_1]),
        iterations_pass_2=np.asarray([solution.iterations_pass_2]),
        total_iterations=np.asarray([solution.total_iterations]),
        converged_pass_1=np.asarray([solution.converged_pass_1]),
        converged_pass_2=np.asarray([solution.converged_pass_2]),
        q_names=np.asarray(qnames),v_names=np.asarray(vnames),
        control_names=np.asarray(unames),world_frame=np.asarray(["Gazebo ENU / Pinocchio world"]),
        body_velocity_frame=np.asarray(["body frame"]),base_angular_velocity_frame=np.asarray(["body frame"]),
        terminal_rest_config=np.asarray([terminal_rest_config(solution.scenario)],dtype=object),
        model_variant=np.asarray([metadata["model_variant"]]),
        arm_dof=np.asarray([metadata["arm_dof"]]),
        active_arm_joint_names=np.asarray(metadata["active_arm_joint_names"]),
        locked_joint_names=np.asarray(metadata["locked_joint_names"]),
        canonical_launch=np.asarray([metadata["canonical_launch"] or ""]),
        lock_shoulder_pan=np.asarray([metadata["lock_shoulder_pan"]]),
        **arrays)
    _csv(output/"states.csv",["time_s"]+qnames+vnames,[dict([("time_s",i*dt)]+list(zip(qnames+vnames,x))) for i,x in enumerate(solution.states)])
    _csv(output/"controls.csv",["time_s"]+unames,[dict([("time_s",i*dt)]+list(zip(unames,u))) for i,u in enumerate(solution.controls)])
    _csv(output/"trim_reference.csv",["time_s"]+unames,[dict([("time_s",i*dt)]+list(zip(unames,u))) for i,u in enumerate(solution.trim_references)])
    rows=[]
    for i,(p,R,v,w) in enumerate(zip(arrays["ee_position"],arrays["ee_rotation"],arrays["ee_linear"],arrays["ee_angular"])):
        quat=pin.Quaternion(R).coeffs(); vals=np.r_[p,quat,v,w,arrays["position_error"][i],arrays["orientation_error"][i]]
        keys=["x","y","z","qx","qy","qz","qw","vx","vy","vz","wx","wy","wz","position_error","orientation_error"]
        rows.append(dict([("time_s",i*dt)]+list(zip(keys,vals))))
    _csv(output/"ee_pose.csv",["time_s"]+keys,rows)
    summary={
        "strategy":solution.report_name,
        "metadata":metadata,
        "metrics":metrics,
        "fddp_pass_1":{
            "iterations":solution.iterations_pass_1,
            "converged":solution.converged_pass_1,
            "initial_cost":solution.costs_pass_1[0] if solution.costs_pass_1 else None,
            "final_cost":solution.costs_pass_1[-1] if solution.costs_pass_1 else None,
            "initial_objective_cost":solution.diagnostics_pass_1["objective_cost"][0] if solution.diagnostics_pass_1["objective_cost"] else None,
            "final_objective_cost":solution.diagnostics_pass_1["objective_cost"][-1] if solution.diagnostics_pass_1["objective_cost"] else None,
            "initial_dynamics_gap_max":solution.diagnostics_pass_1["dynamics_gap_max"][0] if solution.diagnostics_pass_1["dynamics_gap_max"] else None,
            "final_dynamics_gap_max":solution.diagnostics_pass_1["dynamics_gap_max"][-1] if solution.diagnostics_pass_1["dynamics_gap_max"] else None,
        },
        "fddp_pass_2":{
            "iterations":solution.iterations_pass_2,
            "converged":solution.converged_pass_2,
            "initial_cost":solution.costs_pass_2[0] if solution.costs_pass_2 else None,
            "final_cost":solution.costs_pass_2[-1] if solution.costs_pass_2 else None,
            "initial_objective_cost":solution.diagnostics_pass_2["objective_cost"][0] if solution.diagnostics_pass_2["objective_cost"] else None,
            "final_objective_cost":solution.diagnostics_pass_2["objective_cost"][-1] if solution.diagnostics_pass_2["objective_cost"] else None,
            "initial_dynamics_gap_max":solution.diagnostics_pass_2["dynamics_gap_max"][0] if solution.diagnostics_pass_2["dynamics_gap_max"] else None,
            "final_dynamics_gap_max":solution.diagnostics_pass_2["dynamics_gap_max"][-1] if solution.diagnostics_pass_2["dynamics_gap_max"] else None,
        },
        "total_fddp_iterations":solution.total_iterations,
        "dynamics_gap_penalty_weight":solution.dynamics_gap_penalty_weight,
        "cost_curve_note":"costs and optimization_cost_convergence.png contain pass 2 diagnostic merit cost only; pass 1 uses a different proximal objective. objective_costs_pass_* store raw Crocoddyl objective costs.",
        "diagnostic_total_cost":"objective_cost + dynamics_gap_penalty_weight * sum_k ||x[k+1] - f(x[k], u[k])||^2; diagnostic only, not a replacement for BoxFDDP shooting dynamics.",
        "delta_u_implementation":"two-pass proximal previous-control reference; exact cross-node delta-u requires control-state augmentation",
    }
    (output/"optimization_summary.yaml").write_text(yaml.safe_dump(summary,sort_keys=False))
    _save_two_pass_cost_plots(solution, output)
    fig,axes=plt.subplots(2,2,figsize=(11,8));t=np.arange(len(solution.states))*dt
    axes[0,0].plot(t,arrays["ee_position"]);axes[0,0].set_ylabel("EE position [m]")
    axes[0,1].plot(t,arrays["base_rpy"]);axes[0,1].set_ylabel("base RPY [rad]")
    axes[1,0].plot(t,arrays["position_error"],label="position");axes[1,0].plot(t,arrays["orientation_error"],label="orientation");axes[1,0].legend()
    axes[1,1].plot(t[:-1],solution.controls[:,:4]);axes[1,1].set_ylabel("rotor thrust [N]")
    fig.tight_layout();fig.savefig(output/"state_control_timeseries.png",dpi=160);plt.close(fig)
    save_plots(robot,actuation,solution,output,solution.report_name)


def _save_two_pass_cost_plots(solution: BulbStrategySolution, output: Path) -> None:
    """Save separate cost plots for the two different proximal objectives."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    title = f"{solution.scenario.get('scenario_name','scene_bulb_pregrasp')} | {solution.report_name}"
    for label, costs, objective, iterations, converged, filename in [
        ("Pass 1", solution.costs_pass_1, solution.diagnostics_pass_1["objective_cost"], solution.iterations_pass_1,
         solution.converged_pass_1, "optimization_cost_convergence_pass1.png"),
        ("Pass 2", solution.costs_pass_2, solution.diagnostics_pass_2["objective_cost"], solution.iterations_pass_2,
         solution.converged_pass_2, "optimization_cost_convergence_pass2.png"),
    ]:
        fig, ax = plt.subplots(figsize=(7,4))
        x = np.arange(len(costs))
        ax.semilogy(x, costs, marker="o", label="diagnostic total")
        ax.semilogy(np.arange(len(objective)), objective, marker=".", ls="--", label="objective")
        if costs:
            ax.scatter([0, len(costs)-1], [costs[0], costs[-1]], color="r", zorder=3)
        ax.set(
            xlabel=f"{label} BoxFDDP iteration",
            ylabel=f"{label} cost",
            title=f"BoxFDDP {label.lower()} diagnostic cost convergence\n{title}",
        )
        ax.text(0.02,0.03,f"{label.lower()} iterations={iterations}\n{label.lower()} converged={converged}",
                transform=ax.transAxes,fontsize=8,va="bottom")
        ax.legend(fontsize=8)
        ax.grid(True)
        fig.tight_layout(); fig.savefig(output/filename,dpi=160); plt.close(fig)

    fig, axes = plt.subplots(1,2,figsize=(12,4))
    for ax, label, costs, iterations, converged in [
        (axes[0], "Pass 1", solution.costs_pass_1, solution.iterations_pass_1, solution.converged_pass_1),
        (axes[1], "Pass 2", solution.costs_pass_2, solution.iterations_pass_2, solution.converged_pass_2),
    ]:
        x = np.arange(len(costs))
        ax.semilogy(x, costs, marker="o")
        if costs:
            ax.scatter([0, len(costs)-1], [costs[0], costs[-1]], color="r", zorder=3)
        ax.set(xlabel=f"{label} iteration", ylabel=f"{label} cost",
               title=f"{label}: iterations={iterations}, converged={converged}")
        ax.grid(True)
    fig.suptitle(f"BoxFDDP two-pass cost convergence\n{title}")
    fig.tight_layout(); fig.savefig(output/"optimization_cost_convergence_two_passes.png",dpi=160); plt.close(fig)
    _save_pass_diagnostics(solution, output)


def _save_pass_diagnostics(solution: BulbStrategySolution, output: Path) -> None:
    """Plot objective, dynamics gap, terminal task error, and rest metrics."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    def positive(values: List[float]) -> np.ndarray:
        return np.maximum(np.asarray(values, dtype=float), 1.0e-16)
    title = f"{solution.scenario.get('scenario_name','scene_bulb_pregrasp')} | {solution.report_name}"
    rest = terminal_rest_config(solution.scenario)
    for label, diagnostics, filename in [
        ("Pass 1", solution.diagnostics_pass_1, "optimization_pass1_diagnostics.png"),
        ("Pass 2", solution.diagnostics_pass_2, "optimization_pass2_diagnostics.png"),
    ]:
        x = np.arange(len(diagnostics["diagnostic_total_cost"]))
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes[0, 0].semilogy(x, positive(diagnostics["diagnostic_total_cost"]), marker="o", label="diagnostic total")
        axes[0, 0].semilogy(x, positive(diagnostics["objective_cost"]), marker=".", ls="--", label="objective")
        axes[0, 0].set_ylabel("cost")
        axes[0, 0].legend(fontsize=8)
        axes[0, 1].semilogy(x, positive(diagnostics["dynamics_gap_max"]), marker="o", label="max gap")
        axes[0, 1].semilogy(x, positive(diagnostics["dynamics_gap_sum_squares"]), marker=".", ls="--", label="sum gap^2")
        axes[0, 1].set_ylabel("dynamics defect")
        axes[0, 1].legend(fontsize=8)
        axes[1, 0].semilogy(x, positive(diagnostics["terminal_ee_position_error_m"]), marker="o", label="EE position [m]")
        axes[1, 0].semilogy(x, positive(diagnostics["terminal_ee_orientation_error_rad"]), marker=".", label="EE orientation [rad]")
        axes[1, 0].set_xlabel(f"{label} BoxFDDP iteration")
        axes[1, 0].set_ylabel("terminal task error")
        axes[1, 0].legend(fontsize=8)
        axes[1, 1].semilogy(x, positive(diagnostics["terminal_base_linear_velocity_norm_mps"]), marker="o", label="|v_B^B|")
        axes[1, 1].semilogy(x, positive(diagnostics["terminal_base_angular_velocity_norm_radps"]), marker=".", label="|omega_B^B|")
        axes[1, 1].semilogy(x, positive(diagnostics["terminal_max_arm_joint_velocity_radps"]), marker="s", label="|dq_a|_inf")
        axes[1, 1].axhline(rest["pass_base_linear_velocity_norm_mps"], color="r", ls=":", linewidth=0.8)
        axes[1, 1].axhline(rest["pass_base_angular_velocity_norm_radps"], color="r", ls=":", linewidth=0.8)
        axes[1, 1].axhline(rest["pass_arm_joint_velocity_inf_radps"], color="r", ls=":", linewidth=0.8)
        axes[1, 1].set_xlabel(f"{label} BoxFDDP iteration")
        axes[1, 1].set_ylabel("terminal rest metric")
        axes[1, 1].legend(fontsize=8)
        for ax in axes.flat:
            ax.grid(True)
        fig.suptitle(
            f"{label} diagnostics: objective, dynamics gap, task error, terminal rest\n"
            f"{title} | gap weight={solution.dynamics_gap_penalty_weight:g}")
        fig.tight_layout()
        fig.savefig(output/filename, dpi=160)
        plt.close(fig)


def save_comparison(solutions: List[BulbStrategySolution], evaluations: Dict[str,Tuple[Dict[str,Any],Dict[str,np.ndarray]]], output: Path) -> None:
    """Save scenario resolution, comparison tables/plots, and feasibility report."""
    import matplotlib;matplotlib.use("Agg");import matplotlib.pyplot as plt
    output.mkdir(parents=True,exist_ok=True); first=solutions[0]
    (output/"scenario_resolved.yaml").write_text(yaml.safe_dump(first.scenario,sort_keys=False))
    target={"bulb_pose":{"translation":first.bulb_diagnostics,"position":first.scenario["bulb_pose_world"]["position"]},"pregrasp":{"position":first.target_pose.translation.tolist(),"rotation":first.target_pose.rotation.tolist()},"diagnostics":first.target_diagnostics}
    (output/"target_frames.yaml").write_text(yaml.safe_dump(target,sort_keys=False))
    (output/"ik_seed_report.yaml").write_text(yaml.safe_dump(first.ik_report,sort_keys=False))
    rows=[]
    for sol in solutions:
        m=evaluations[sol.strategy][0];rows.append({"strategy":sol.report_name,**m})
    fields=list(rows[0]);_csv(output/"comparison_summary.csv",fields,rows)
    (output/"comparison_summary.yaml").write_text(yaml.safe_dump({"strategies":rows,"soft_strategy_note":"Soft-constraint comparison, not strict DOF locking."},sort_keys=False))
    dt=float(first.scenario["dt"]);colors={s.strategy:None for s in solutions}
    fig=plt.figure(figsize=(8,6));ax=fig.add_subplot(111,projection="3d")
    for s in solutions:
        p=evaluations[s.strategy][1]["ee_position"];ax.plot(p[:,0],p[:,1],p[:,2],label=s.report_name)
    ax.scatter(*first.target_pose.translation,marker="x");ax.legend(fontsize=7);fig.tight_layout();fig.savefig(output/"trajectory_comparison_3d.png",dpi=160);plt.close(fig)
    for key,file,ylabel in [("position_error","ee_position_error_comparison.png","position error [m]"),("orientation_error","ee_orientation_error_comparison.png","orientation error [rad]")]:
        fig,ax=plt.subplots();
        for s in solutions:
            y=evaluations[s.strategy][1][key];ax.semilogy(np.arange(len(y))*dt,y,label=s.report_name)
        ax.set(xlabel="time [s]",ylabel=ylabel);ax.legend(fontsize=7);fig.tight_layout();fig.savefig(output/file,dpi=160);plt.close(fig)
    fig,ax=plt.subplots();
    for s in solutions:ax.plot(np.arange(len(s.states))*dt,evaluations[s.strategy][1]["base_rpy"][:,:2],label=s.report_name)
    ax.set_ylabel("roll/pitch [rad]");fig.tight_layout();fig.savefig(output/"base_attitude_comparison.png",dpi=160);plt.close(fig)
    names=[s.report_name for s in solutions];m=[evaluations[s.strategy][0] for s in solutions];x=np.arange(3)
    fig,ax=plt.subplots();ax.bar(x-.2,[v["minimum_rotor_margin_N"] for v in m],.4,label="rotor N");ax.bar(x+.2,[v["minimum_joint_torque_margin_Nm"] for v in m],.4,label="joint Nm");ax.set_xticks(x,names,rotation=20);ax.legend();fig.tight_layout();fig.savefig(output/"thrust_torque_margin_comparison.png",dpi=160);plt.close(fig)
    fig,ax=plt.subplots();ax.bar(x,[v["delta_u_cost"] for v in m]);ax.set_xticks(x,names,rotation=20);ax.set_ylabel("sum ||du||^2");fig.tight_layout();fig.savefig(output/"control_smoothness_comparison.png",dpi=160);plt.close(fig)
    passed=all(v["pass"] for v in m)
    (output/"feasibility_report.md").write_text("# P2.7 feasibility\n\n"+f"Overall: **{'PASS' if passed else 'FAIL'}**\n\nPose source: `{first.bulb_diagnostics['pose_source']}`. IK seed: `{'PASS' if first.ik_report['success'] else 'FAIL'}`. This is free-flight pregrasp only; no contact or screwing.\n")


def _csv(path:Path,fields:List[str],rows:List[Dict[str,Any]])->None:
    with path.open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
