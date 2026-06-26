from typing import Dict

import numpy as np


class SafetyMonitor:
    def __init__(self, config: Dict):
        self.config = dict(config)
        self.interface_validated = False

    def mark_interface_validated(self) -> None:
        self.interface_validated = True

    def require_track_allowed(self) -> None:
        if not self.interface_validated:
            raise RuntimeError("TRACK is blocked until PX4 and arm interface validation passes")

    def check_finite_command(self, values) -> None:
        if not np.all(np.isfinite(np.asarray(values, dtype=float))):
            raise RuntimeError("refusing to publish non-finite command")
