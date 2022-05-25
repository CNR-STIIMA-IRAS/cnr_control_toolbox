#!/bin/bash

declare -a StringArray=("kinematics_filters" "state_space_systems" "state_space_controllers" "state_space_filters" "state_space_ros")

ws=~/target_ws

cp /home/runner/work/cnr_control_toolbox/cnr_control_toolbox/codecov.yml "$ws"/

cd "$ws"

source ./devel/setup.bash

for val in ${StringArray[@]}; do

    echo "Generating coverage for '$val'"

    catkin build "$val" --no-deps --catkin-make-args run_tests 
    catkin build "$val" --no-deps --catkin-make-args coverage_report --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DCATKIN_ENABLE_TESTING=ON
done

# Remove duplicated information
find "$ws" -name \*.info.cleaned -exec rm {} \;
find "$ws" -name \*.info.removed -exec rm {} \;
