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

echo_and_run cd "/home/sebalchi/catkin_ws/src/project_g1/src/gazebo_ros_pkgs/gazebo_plugins"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sebalchi/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sebalchi/catkin_ws/install/lib/python3/dist-packages:/home/sebalchi/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sebalchi/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/sebalchi/catkin_ws/src/project_g1/src/gazebo_ros_pkgs/gazebo_plugins/setup.py" \
     \
    build --build-base "/home/sebalchi/catkin_ws/build/project_g1/src/gazebo_ros_pkgs/gazebo_plugins" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sebalchi/catkin_ws/install" --install-scripts="/home/sebalchi/catkin_ws/install/bin"
