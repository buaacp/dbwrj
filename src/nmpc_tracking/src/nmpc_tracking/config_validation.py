from typing import Dict

from .acados_model import controller_dimensions
from .joint_mapping import mapping_from_config
from .px4_rate_thrust_adapter import ThrustMapper
from .robot_layout import assert_layout_consistency, load_robot_layout


def validate_config(config: Dict) -> Dict[str, int]:
    for key in ["runtime", "topics", "trajectory", "planner", "controller", "vehicle", "arm"]:
        if key not in config:
            raise ValueError("missing config section: %s" % key)
    rate = float(config["controller"]["rate_hz"])
    dt = float(config["controller"]["dt"])
    if abs(rate * dt - 1.0) > 1e-6:
        raise ValueError("controller.rate_hz and controller.dt are inconsistent")
    if int(config["controller"]["horizon_steps"]) <= 0:
        raise ValueError("controller.horizon_steps must be positive")
    layout = load_robot_layout(config)
    vehicle = config["vehicle"]
    ThrustMapper(
        layout.effective_mass_kg, vehicle["gravity_mps2"], vehicle["hover_thrust_norm"],
        vehicle.get("thrust_norm_min", 0.0), vehicle.get("thrust_norm_max", 1.0),
    )
    dims = controller_dimensions(config, layout)
    assert_layout_consistency(layout, dims["state_dim"], dims["command_dim"], dims["control_rate_dim"])
    return dims
