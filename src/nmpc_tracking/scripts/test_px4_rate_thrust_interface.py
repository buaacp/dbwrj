#!/usr/bin/env python3
import argparse

from nmpc_tracking.frames import enu_to_ned_position, flu_to_frd_body_rate
from nmpc_tracking.px4_rate_thrust_adapter import ThrustMapper


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="publish only when explicitly requested")
    parser.add_argument("--mass", type=float, default=1.5)
    parser.add_argument("--hover-thrust", type=float, default=0.5)
    args = parser.parse_args()
    mapper = ThrustMapper(args.mass, 9.81, args.hover_thrust)
    print("ENU [1,2,3] -> NED", enu_to_ned_position([1, 2, 3]).tolist())
    print("FLU rate [1,2,3] -> FRD", flu_to_frd_body_rate([1, 2, 3]).tolist())
    print("hover normalized thrust", mapper.force_to_normalized(args.mass * 9.81))
    if not args.execute:
        print("dry-run: no ROS publisher created")
        return
    raise SystemExit("explicit execution publishing is not implemented in this validation scaffold")


if __name__ == "__main__":
    main()
