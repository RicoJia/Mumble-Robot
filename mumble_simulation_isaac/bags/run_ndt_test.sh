#!/bin/bash

# Usage: ./run_ndt_test.sh 10 42

START_IDX=$1
STOP_IDX=$2

# Optional: check if arguments were passed
if [ -z "$START_IDX" ] || [ -z "$STOP_IDX" ]; then
  echo "Usage: $0 <start_msg_index> <stopping_msg_index>"
  exit 1
fi

rm -rf /tmp/test_halo_3d_slam.pcd

# ./build/mumble_onboard/halo/test_halo_3d_slam --bag_path bags/mojave_room5 --start_msg_index=130 --stopping_msg_index 140 --yaml_config_path="src/mumble_onboard/configs/slam3d_configs/test_slam_3d.yaml" && python3 /home/mumble_robot/src/mumble_onboard/halo/scripts/visualize_pcd.py -p /tmp/test_halo_3d_slam.pcd

/home/mumble_robot/build/mumble_onboard/halo/test_halo_3d_slam \
  --bag_path /home/mumble_robot/bags/mojave_apt1 \
  --start_msg_index="$START_IDX" \
  --stopping_msg_index="$STOP_IDX" \
  --yaml_config_path="/home/mumble_robot/src/mumble_onboard/configs/slam3d_configs/test_slam_3d.yaml" \
  && ros2 run mumble_onboard pcd_sequence_viewer.py /tmp/test_halo_3d_slam.pcd
