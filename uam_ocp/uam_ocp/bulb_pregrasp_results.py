"""Metrics, files, and comparison plots for P2.7 strategies."""

import csv
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pinocchio as pin
import yaml

from .actuation import UamActuation
from .bulb_pregrasp_planner import BulbPregraspPlanner, BulbStrategySolution
from .model_loader import UamModel
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
    trim_results=yaml.safe_load((Path(__file__).resolve().parents[1]/"results/static_trim/trim_results.yaml").read_text())
    fully_entry=next(entry for entry in trim_results["entries"] if entry["name"]=="fully_extended")
    fully=np.asarray(fully_entry["result"]["q"])
    arm_indices=[item.idx_q for item in robot.arm_joints]
    min_distance_fully=float(np.min([np.linalg.norm(state[:robot.model.nq][arm_indices]-fully[arm_indices]) for state in solution.states]))
    metrics={
        "pass": bool(solution.converged and poserr[-1]<solution.scenario["terminal_position_tolerance_m"] and roterr[-1]<solution.scenario["terminal_orientation_tolerance_rad"] and np.linalg.norm(linear[-1])<solution.scenario["terminal_ee_linear_velocity_tolerance_mps"] and np.linalg.norm(angular[-1])<solution.scenario["terminal_ee_angular_velocity_tolerance_radps"] and rest_metrics["terminal_rest_pass"]),
        "fddp_converged":solution.converged,"iterations":solution.iterations,
        "final_cost":float(solution.costs[-1]) if solution.costs else None,
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
        "left_knuckle_max_abs_torque_Nm":float(np.max(np.abs(joint[:,actuation.joint_names.index("left_knuckle_joint")]))),
        "left_knuckle_static_margin_Nm":0.19984637956497772,
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
    np.savez_compressed(output/"trajectory.npz",
        time_s=np.arange(len(solution.states))*dt,dt_s=np.asarray([dt]),
        states=solution.states,controls=solution.controls,solver_states=solution.states,
        trim_references=solution.trim_references,reference_states=solution.reference_states,
        costs=np.asarray(solution.costs),q_names=np.asarray(qnames),v_names=np.asarray(vnames),
        control_names=np.asarray(unames),world_frame=np.asarray(["Gazebo ENU / Pinocchio world"]),
        body_velocity_frame=np.asarray(["body frame"]),base_angular_velocity_frame=np.asarray(["body frame"]),
        terminal_rest_config=np.asarray([terminal_rest_config(solution.scenario)],dtype=object),**arrays)
    _csv(output/"states.csv",["time_s"]+qnames+vnames,[dict([("time_s",i*dt)]+list(zip(qnames+vnames,x))) for i,x in enumerate(solution.states)])
    _csv(output/"controls.csv",["time_s"]+unames,[dict([("time_s",i*dt)]+list(zip(unames,u))) for i,u in enumerate(solution.controls)])
    _csv(output/"trim_reference.csv",["time_s"]+unames,[dict([("time_s",i*dt)]+list(zip(unames,u))) for i,u in enumerate(solution.trim_references)])
    rows=[]
    for i,(p,R,v,w) in enumerate(zip(arrays["ee_position"],arrays["ee_rotation"],arrays["ee_linear"],arrays["ee_angular"])):
        quat=pin.Quaternion(R).coeffs(); vals=np.r_[p,quat,v,w,arrays["position_error"][i],arrays["orientation_error"][i]]
        keys=["x","y","z","qx","qy","qz","qw","vx","vy","vz","wx","wy","wz","position_error","orientation_error"]
        rows.append(dict([("time_s",i*dt)]+list(zip(keys,vals))))
    _csv(output/"ee_pose.csv",["time_s"]+keys,rows)
    (output/"optimization_summary.yaml").write_text(yaml.safe_dump({"strategy":solution.report_name,"metrics":metrics,"delta_u_implementation":"two-pass proximal previous-control reference; exact cross-node delta-u requires control-state augmentation"},sort_keys=False))
    fig,ax=plt.subplots();ax.semilogy(solution.costs);ax.set(xlabel="iteration",ylabel="cost");fig.tight_layout();fig.savefig(output/"cost_convergence.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,8));t=np.arange(len(solution.states))*dt
    axes[0,0].plot(t,arrays["ee_position"]);axes[0,0].set_ylabel("EE position [m]")
    axes[0,1].plot(t,arrays["base_rpy"]);axes[0,1].set_ylabel("base RPY [rad]")
    axes[1,0].plot(t,arrays["position_error"],label="position");axes[1,0].plot(t,arrays["orientation_error"],label="orientation");axes[1,0].legend()
    axes[1,1].plot(t[:-1],solution.controls[:,:4]);axes[1,1].set_ylabel("rotor thrust [N]")
    fig.tight_layout();fig.savefig(output/"state_control_timeseries.png",dpi=160);plt.close(fig)
    save_plots(robot,actuation,solution,output,solution.report_name)


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
