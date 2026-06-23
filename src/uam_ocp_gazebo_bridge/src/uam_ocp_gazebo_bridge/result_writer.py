"""Persist G1 telemetry, metadata, summaries, and comparison plots."""

import csv
from pathlib import Path
import numpy as np
import yaml


def yaml_safe(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return dict((k,yaml_safe(v)) for k,v in value.items())
    if isinstance(value, (list,tuple)): return [yaml_safe(v) for v in value]
    return value


class ResultWriter(object):
    def __init__(self, output, joint_names):
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True);self.joint_names=list(joint_names)

    def checkpoint(self, rows, arm_commands):
        """Flush current CSV records without performing any runtime analysis."""
        if rows:
            self._csv(self.output/"telemetry.csv",list(rows[0]),rows)
        if arm_commands:
            self._csv(self.output/"commanded_arm_trajectory.csv",list(arm_commands[0]),arm_commands)

    def write_run(self, rows, arm_commands, target, trim, metadata, status, metrics, abort, topic_rates, startup_telemetry):
        directory=self.output
        self._yaml(directory/"resolved_target_configuration.yaml",target)
        self._yaml(directory/"static_trim_reference.yaml",trim)
        self._yaml(directory/"run_metadata.yaml",metadata)
        result={"status":status,"metrics":metrics,"abort":abort if status=="ABORTED" else None,
                "interface_failure":abort if status in ("NOT_RUN_INTERFACE_UNAVAILABLE","NOT_RUN_ENVIRONMENT_UNAVAILABLE","CONFIGURATION_UNRESOLVED") else None,
                "startup_telemetry":startup_telemetry,
                "joint_failure":abort.get("joint_failure") if abort else None}
        self._yaml(directory/"result.yaml",result)
        self._yaml(directory/"topic_rates.yaml",{"topic_rates_hz":topic_rates})
        fields=list(rows[0].keys()) if rows else self._telemetry_fields()
        self._csv(directory/"telemetry.csv",fields,rows)
        command_fields=list(arm_commands[0].keys()) if arm_commands else ["sim_time_s"]
        self._csv(directory/"commanded_arm_trajectory.csv",command_fields,arm_commands)
        arrays={key:np.asarray([row[key] for row in rows]) for key in fields} if rows else {}
        np.savez_compressed(str(directory/"telemetry.npz"),**arrays)

    def _telemetry_fields(self):
        fields=["sim_time_s","wall_time_s","state","active_reference_kind","active_joint_reference_kind","ref_px","ref_py","ref_pz","px","py","pz","vx","vy","vz","roll_rad","pitch_rad","yaw_rad","wx","wy","wz","position_error_m","position_error_to_active_reference_m","position_error_to_final_hold_m","position_error_to_initial_m","takeoff_transition_error_m","joint_error_to_active_reference_deg","joint_error_to_neutral_deg","max_joint_error_to_active_reference_deg","max_joint_error_to_neutral_deg","speed_mps","yaw_error_rad","max_joint_error_rad","offboard","armed"]
        for name in self.joint_names: fields += ["q_"+name,"qd_"+name,"qcmd_"+name,"qref_"+name,"effort_"+name]
        return fields

    @staticmethod
    def _yaml(path,data):path.write_text(yaml.safe_dump(yaml_safe(data),sort_keys=False))
    @staticmethod
    def _csv(path,fields,rows):
        with path.open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
