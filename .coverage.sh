#!/bin/bash


echo "Generating coverage for 'cnr_control_toolbox'"

ws=~/target_ws
cd "$ws"
catkin build state_space_systems     -v --no-deps --catkin-make-args coverage_report  --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS=ON -DENABLE_TESTING=ON
catkin build state_space_controllers -v --no-deps --catkin-make-args coverage_report  --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS=ON -DENABLE_TESTING=ON
catkin build state_space_filters     -v --no-deps --catkin-make-args coverage_report  --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS=ON -DENABLE_TESTING=ON
catkin build state_space_ros         -v --no-deps --catkin-make-args coverage_report  --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS=ON -DENABLE_TESTING=ON

#echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "$ws/build/state_space_systems/coverage_report.info.cleaned"
rm "$ws/build/state_space_systems/coverage_report.info.removed"

rm "$ws/build/state_space_controllers/coverage_report.info.cleaned"
rm "$ws/build/state_space_controllers/coverage_report.info.removed"

rm "$ws/build/state_space_filters/coverage_report.info.cleaned"
rm "$ws/build/state_space_filters/coverage_report.info.removed"

rm "$ws/build/state_space_ros/coverage_report.info.cleaned"
rm "$ws/build/state_space_ros/coverage_report.info.removed"

# # Actually upload coverage informationstate_space_controllers
# bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_systems/"
# bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_controllers/"
# bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_filters/"
# bash <(curl -s https://codecov.io/bash) -s "$ws/build/state_space_ros/"