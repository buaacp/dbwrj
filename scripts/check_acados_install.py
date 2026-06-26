#!/usr/bin/env python3
import ctypes
import os
import sys


def main():
    source_dir = os.environ.get("ACADOS_SOURCE_DIR", "/home/zlhq/acados")
    lib_dir = os.path.join(source_dir, "lib")
    os.environ["ACADOS_SOURCE_DIR"] = source_dir
    os.environ["LD_LIBRARY_PATH"] = lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")

    osqp = os.path.join(lib_dir, "libosqp.so")
    acados = os.path.join(lib_dir, "libacados.so")
    if os.path.exists(osqp):
        ctypes.CDLL(osqp, mode=ctypes.RTLD_GLOBAL)
    ctypes.CDLL(acados, mode=ctypes.RTLD_GLOBAL)

    import acados_template
    import casadi
    import numpy
    import scipy

    print("python", sys.version.split()[0], sys.executable)
    print("ACADOS_SOURCE_DIR", source_dir)
    print("acados_template", getattr(acados_template, "__file__", "OK"))
    print("casadi", casadi.__version__)
    print("numpy", numpy.__version__)
    print("scipy", scipy.__version__)
    print("libacados load OK")


if __name__ == "__main__":
    main()
