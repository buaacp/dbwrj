#!/usr/bin/env bash
# Source this file before running acados/acados_template code generation.

export ACADOS_SOURCE_DIR="/home/zlhq/acados"
export ACADOS_PYTHON="/home/zlhq/micromamba-root/envs/eagle_mpc/bin/python"
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH}"

# This local acados build links OSQP dynamically; preloading OSQP resolves
# LINSYS_SOLVER_NAME when libacados.so is loaded through Python ctypes.
export LD_PRELOAD="${ACADOS_SOURCE_DIR}/lib/libosqp.so${LD_PRELOAD:+:${LD_PRELOAD}}"

export PATH="/home/zlhq/micromamba-root/envs/eagle_mpc/bin:${PATH}"

export PYTHONPATH="/opt/ros/melodic/lib/python2.7/dist-packages:/home/zlhq/px4_fly_ws/devel/lib/python3/dist-packages:/home/zlhq/px4_fly_ws/uam_ocp:/home/zlhq/px4_fly_ws/src/nmpc_tracking/src:${PYTHONPATH}"
