import csv
import os
from typing import Dict, Iterable

import numpy as np
import yaml


class ResultLogger:
    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        self.rows = []

    def log(self, row: Dict) -> None:
        self.rows.append(dict(row))

    def save(self) -> None:
        os.makedirs(self.output_dir, exist_ok=True)
        if self.rows:
            keys = sorted(self.rows[0].keys())
            with open(os.path.join(self.output_dir, "tracking_log.csv"), "w") as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(self.rows)
            np.savez_compressed(os.path.join(self.output_dir, "tracking_log.npz"),
                                rows=np.asarray(self.rows, dtype=object))
        summary = {"samples": len(self.rows)}
        with open(os.path.join(self.output_dir, "summary.yaml"), "w") as f:
            yaml.safe_dump(summary, f, sort_keys=False)
