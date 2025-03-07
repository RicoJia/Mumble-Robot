colcon_build_source(){
    colcon build --symlink-install
    source install/setup.bash
}

sudo_ros_preserve_env(){ 
    # when passing args from one func to another, use $@ expansion 
    local cmd="$@" 
    # -E preserves all current user's env variables. 
    # - `source` vs `./`: `./` will createa a copy of the env of the current session for the new script. 
    # when the script is done, the new env is gone. `source` and `.` will execute the script right on the spot 
    echo "Using sudo_ros_preserve_env..."
    sudo -E /bin/bash -c " source ${WORKDIRECTORY}/install/setup.bash; export PYTHONPATH=\$PYTHONPATH; $cmd" 
} 

run_bag_recorder(){
    sudo_ros_preserve_env ros2 run mumble_onboard mumble_bag_recorder.py
}

bag_replay_sudo(){
    sudo_ros_preserve_env ros2 bag play $1
}

if [ ! -d "/home/mumble_robot/build/" ]; then
    echo "First container launch: running colcon build..."
    colcon_build_source
    cd /home/mumble_robot/src/mumble_physical_runtime
fi