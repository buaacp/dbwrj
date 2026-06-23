#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/zlhq/px4_fly_ws/src/uav_bulb_camera_pose"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zlhq/px4_fly_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zlhq/px4_fly_ws/install/lib/python2.7/dist-packages:/home/zlhq/px4_fly_ws/build/uav_bulb_camera_pose/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zlhq/px4_fly_ws/build/uav_bulb_camera_pose" \
    "/usr/bin/python2" \
    "/home/zlhq/px4_fly_ws/src/uav_bulb_camera_pose/setup.py" \
     \
    build --build-base "/home/zlhq/px4_fly_ws/build/uav_bulb_camera_pose" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zlhq/px4_fly_ws/install" --install-scripts="/home/zlhq/px4_fly_ws/install/bin"
