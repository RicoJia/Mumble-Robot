#!/bin/bash

# Usage: ./run_ndt_test.sh 10 42

START_IDX=$1
STOP_IDX=$2

# Optional: check if arguments were passed
if [ -z "$START_IDX" ] || [ -z "$STOP_IDX" ]; then
  echo "Usage: $0 <start_msg_index> <stopping_msg_index>"
  exit 1
fi

rm -rf /tmp/test_incremental_3d_ndt.pcd

/home/mumble_robot/build/mumble_onboard/halo/test_direct_3d_ndt_lo \
  --bag_path /home/mumble_robot/bags/mojave_room5 \
  --start_msg_index="$START_IDX" \
  --stopping_msg_index="$STOP_IDX" \
  --yaml_config_path="/home/mumble_robot/src/mumble_onboard/configs/slam3d_configs/test_direct_3d_ndt_lo.yaml" \
  && python3 /home/mumble_robot/src/mumble_onboard/halo/scripts/visualize_pcd.py \
    -p /tmp/test_incremental_3d_ndt.pcd -s
