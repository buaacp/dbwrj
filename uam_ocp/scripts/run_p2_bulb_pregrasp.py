#!/usr/bin/env python3
"""Run all P2.7 soft coordination strategies and export comparisons."""

import argparse
from datetime import datetime
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from uam_ocp.actuation import UamActuation
from uam_ocp.bulb_pregrasp_planner import BulbPregraspPlanner
from uam_ocp.bulb_pregrasp_results import evaluate_solution,save_comparison,save_strategy
from uam_ocp.model_loader import load_uam_model
from uam_ocp.prediction_model import UAMPredictionModel

def _unique_output(base: Path, run_id: str) -> Path:
    candidate = base / "runs" / run_id
    if not candidate.exists():
        candidate.mkdir(parents=True)
        return candidate
    for index in range(1, 1000):
        suffixed = base / "runs" / f"{run_id}_{index:03d}"
        if not suffixed.exists():
            suffixed.mkdir(parents=True)
            return suffixed
    raise RuntimeError(f"Could not allocate unique output directory under {base / 'runs'}")

def main()->int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--run-id",default=datetime.now().strftime("%Y%m%d_%H%M%S"),
                        help="Unique result directory name under results/p2_bulb_pregrasp/runs")
    args=parser.parse_args()
    robot=load_uam_model();actuation=UamActuation(robot);prediction=UAMPredictionModel(robot,actuation)
    planner=BulbPregraspPlanner(robot,actuation,prediction)
    output=_unique_output(ROOT/"results"/"p2_bulb_pregrasp",args.run_id)
    solutions=[];evaluations={}
    for name in ("arm_dominant","uav_dominant","whole_body"):
        solution=planner.solve_strategy(name);metrics,arrays=evaluate_solution(robot,actuation,planner,solution)
        save_strategy(robot,actuation,solution,metrics,arrays,output/name)
        solutions.append(solution);evaluations[name]=(metrics,arrays)
        print(name,metrics)
    save_comparison(solutions,evaluations,output)
    passed=planner.ik_report["success"] and all(evaluations[s.strategy][0]["pass"] for s in solutions)
    print("P2.7:","PASS" if passed else "FAIL")
    print("results:",output)
    return 0 if passed else 1
if __name__=="__main__":sys.exit(main())
