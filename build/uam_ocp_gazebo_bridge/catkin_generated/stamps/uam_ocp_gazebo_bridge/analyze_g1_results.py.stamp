#!/usr/bin/env python3
import argparse
from uam_ocp_gazebo_bridge.offline_g1_analyzer import analyze_run


def main():
    parser = argparse.ArgumentParser(description="Offline G1 telemetry analyzer")
    parser.add_argument("--input", required=True); parser.add_argument("--output", required=True)
    args = parser.parse_args(); summary = analyze_run(args.input, args.output)
    print("G1 offline analysis: %s" % summary["status"])
    print("output: %s" % args.output)


if __name__ == "__main__": main()
