#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""Translational L1 adaptive augmentation.

This module is adapted from eagle-mpc-python's L1 implementation, but keeps
only the model-light translational estimator.  It is intentionally independent
of ROS, Pinocchio, acados, and the vehicle/arm geometry.
"""

import math
from typing import Optional

import numpy as np


GRAVITY = 9.81


class L1Params:
    def __init__(
        self,
        enabled=False,
        as_gain=8.0,
        wc_xy=6.0,
        wc_z=6.0,
        max_accel_xy=2.0,
        max_accel_z=1.0,
        max_sigma=8.0,
        use_pos_feedback=False,
        k_pos_i_xy=0.4,
        k_pos_i_z=0.3,
        k_pos_p_xy=0.0,
        k_pos_p_z=0.0,
        max_pos_integral_xy=1.0,
        max_pos_integral_z=0.8,
    ):
        self.enabled = bool(enabled)
        self.as_gain = as_gain
        self.wc_xy = wc_xy
        self.wc_z = wc_z
        self.max_accel_xy = max_accel_xy
        self.max_accel_z = max_accel_z
        self.max_sigma = max_sigma
        self.use_pos_feedback = bool(use_pos_feedback)
        self.k_pos_i_xy = k_pos_i_xy
        self.k_pos_i_z = k_pos_i_z
        self.k_pos_p_xy = k_pos_p_xy
        self.k_pos_p_z = k_pos_p_z
        self.max_pos_integral_xy = max_pos_integral_xy
        self.max_pos_integral_z = max_pos_integral_z

    def sanitize(self) -> "L1Params":
        self.as_gain = float(max(1e-3, self.as_gain))
        self.wc_xy = float(max(0.0, self.wc_xy))
        self.wc_z = float(max(0.0, self.wc_z))
        self.max_accel_xy = float(max(0.0, self.max_accel_xy))
        self.max_accel_z = float(max(0.0, self.max_accel_z))
        self.max_sigma = float(max(0.0, self.max_sigma))
        self.k_pos_i_xy = float(max(0.0, self.k_pos_i_xy))
        self.k_pos_i_z = float(max(0.0, self.k_pos_i_z))
        self.k_pos_p_xy = float(max(0.0, self.k_pos_p_xy))
        self.k_pos_p_z = float(max(0.0, self.k_pos_p_z))
        self.max_pos_integral_xy = float(max(0.0, self.max_pos_integral_xy))
        self.max_pos_integral_z = float(max(0.0, self.max_pos_integral_z))
        return self


class L1AdaptiveAugmentation:
    """Velocity-channel disturbance estimator plus bounded acceleration output.

    The estimator assumes the measured world-frame velocity follows

        v_dot = a_model + sigma

    where sigma is an unknown acceleration disturbance.  Callers decide how to
    map the returned acceleration compensation to their command interface.
    """

    def __init__(self, params: Optional[L1Params] = None):
        self.params = (params or L1Params()).sanitize()
        self.v_hat = None
        self.sigma_hat = np.zeros(3, dtype=float)
        self.a_l1 = np.zeros(3, dtype=float)
        self.a_pos = np.zeros(3, dtype=float)
        self.a_ac = np.zeros(3, dtype=float)
        self.pos_integral = np.zeros(3, dtype=float)
        self._initialized = False

    @property
    def enabled(self) -> bool:
        return bool(self.params.enabled)

    def set_enabled(self, flag: bool) -> None:
        self.params.enabled = bool(flag)
        if not flag:
            self.reset()

    def reset(self, v_world: Optional[np.ndarray] = None) -> None:
        self.sigma_hat[:] = 0.0
        self.a_l1[:] = 0.0
        self.a_pos[:] = 0.0
        self.a_ac[:] = 0.0
        self.pos_integral[:] = 0.0
        if v_world is None:
            self.v_hat = None
            self._initialized = False
        else:
            self.v_hat = np.asarray(v_world, dtype=float).reshape(3).copy()
            self._initialized = True

    def step(
        self,
        dt: float,
        v_world: np.ndarray,
        a_model_world: np.ndarray,
        pos_err_world: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        p = self.params
        v = np.asarray(v_world, dtype=float).reshape(3)
        a_model = np.asarray(a_model_world, dtype=float).reshape(3)

        if dt <= 0.0:
            return self.a_ac.copy()

        if not p.enabled:
            self.a_l1[:] = 0.0
            self.sigma_hat[:] = 0.0
            return self._compose_a_ac(dt, pos_err_world)

        if not self._initialized or self.v_hat is None:
            self.v_hat = v.copy()
            self._initialized = True
            return self.a_ac.copy()

        a_s = p.as_gain
        v_tilde = self.v_hat - v
        exp_term = math.exp(-a_s * dt)
        denom = max(1.0 - exp_term, 1e-9)
        k_pc = a_s * exp_term / denom
        self.sigma_hat = self.sigma_hat - k_pc * v_tilde
        if p.max_sigma > 0.0:
            self.sigma_hat = np.clip(self.sigma_hat, -p.max_sigma, p.max_sigma)

        v_hat_dot = a_model + self.sigma_hat - a_s * v_tilde
        self.v_hat = self.v_hat + dt * v_hat_dot

        alpha_xy = 1.0 - math.exp(-p.wc_xy * dt) if p.wc_xy > 0.0 else 0.0
        alpha_z = 1.0 - math.exp(-p.wc_z * dt) if p.wc_z > 0.0 else 0.0
        target = -self.sigma_hat
        self.a_l1[0] += alpha_xy * (target[0] - self.a_l1[0])
        self.a_l1[1] += alpha_xy * (target[1] - self.a_l1[1])
        self.a_l1[2] += alpha_z * (target[2] - self.a_l1[2])
        return self._compose_a_ac(dt, pos_err_world)

    def _compose_a_ac(self, dt: float, pos_err_world: Optional[np.ndarray]) -> np.ndarray:
        p = self.params
        self.a_pos[:] = 0.0

        if p.use_pos_feedback and pos_err_world is not None:
            err = np.asarray(pos_err_world, dtype=float).reshape(3)
            self.pos_integral += dt * err
            if p.max_pos_integral_xy > 0.0:
                self.pos_integral[0] = float(np.clip(self.pos_integral[0], -p.max_pos_integral_xy, p.max_pos_integral_xy))
                self.pos_integral[1] = float(np.clip(self.pos_integral[1], -p.max_pos_integral_xy, p.max_pos_integral_xy))
            if p.max_pos_integral_z > 0.0:
                self.pos_integral[2] = float(np.clip(self.pos_integral[2], -p.max_pos_integral_z, p.max_pos_integral_z))

            self.a_pos[0] = -p.k_pos_i_xy * self.pos_integral[0] - p.k_pos_p_xy * err[0]
            self.a_pos[1] = -p.k_pos_i_xy * self.pos_integral[1] - p.k_pos_p_xy * err[1]
            self.a_pos[2] = -p.k_pos_i_z * self.pos_integral[2] - p.k_pos_p_z * err[2]

        self.a_ac = self.a_l1 + self.a_pos
        if p.max_accel_xy > 0.0:
            self.a_ac[0] = float(np.clip(self.a_ac[0], -p.max_accel_xy, p.max_accel_xy))
            self.a_ac[1] = float(np.clip(self.a_ac[1], -p.max_accel_xy, p.max_accel_xy))
        if p.max_accel_z > 0.0:
            self.a_ac[2] = float(np.clip(self.a_ac[2], -p.max_accel_z, p.max_accel_z))
        return self.a_ac.copy()

    def disturbance_force_world(self, mass: float) -> np.ndarray:
        return float(mass) * self.sigma_hat.copy()

    def estimated_added_mass(self, mass: float) -> float:
        return float(-mass * self.sigma_hat[2] / GRAVITY) if GRAVITY > 0.0 else 0.0
