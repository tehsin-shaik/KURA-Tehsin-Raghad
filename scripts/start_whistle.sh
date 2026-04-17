#!/bin/bash
echo "[START WHISTLE DETECTOR]"

cd `dirname $0`
cd ..

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 launch whistle_detector launch.py "$@"
