#!/bin/bash

source "${ICI_SRC_PATH}/workspace.sh"
source "${ICI_SRC_PATH}/util.sh"

echo "Generating coverage for 'cnr_control_toolbox'"

ws=~/target_ws
extend="/opt/ros/$ROS_DISTRO"
ici_exec_in_workspace "$extend" "$ws" catkin build state_space_systems     -v --no-deps --catkin-make-args coverage_report
ici_exec_in_workspace "$extend" "$ws" catkin build state_space_controllers -v --no-deps --catkin-make-args coverage_report
ici_exec_in_workspace "$extend" "$ws" catkin build state_space_filters     -v --no-deps --catkin-make-args coverage_report
ici_exec_in_workspace "$extend" "$ws" catkin build state_space_ros         -v --no-deps --catkin-make-args coverage_report

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "$ws/build/state_space_systems/coverage_report.info.cleaned"
rm "$ws/build/state_space_systems/coverage_report.info.removed"

rm "$ws/build/state_space_controllers/coverage_report.info.cleaned"
rm "$ws/build/state_space_controllers/coverage_report.info.removed"

rm "$ws/build/state_space_filters/coverage_report.info.cleaned"
rm "$ws/build/state_space_filters/coverage_report.info.removed"

rm "$ws/build/state_space_ros/coverage_report.info.cleaned"
rm "$ws/build/state_space_ros/coverage_report.info.removed"

# Actually upload coverage informationstate_space_controllers
bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_systems/"
bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_controllers/"
bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_filters/"
bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_ros/"