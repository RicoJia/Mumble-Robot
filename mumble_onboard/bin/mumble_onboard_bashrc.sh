# clear && MAKEFLAGS="-j 4" colcon build --symlink-install 
# Can do: clear && colcon_build_source --parallel-workers 2 --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
# For profiling:
#   --cmake-args -DENABLE_GPROF=ON
#   gprof ./build/mumble_onboard/halo/test_direct_3d_ndt_lo gmon.out > profile.txt
colcon_build_source(){
    clear && MAKEFLAGS="-j 6" colcon build --symlink-install  "$@"
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

rm_build(){
    rm -rf /home/mumble_robot/build/ /home/mumble_robot/install/ /home/mumble_robot/log/
}

if [ ! -d "/home/mumble_robot/build/" ]; then
    echo "First container launch: running colcon build..."
    colcon_build_source
    sudo ln -s /home/mumble_robot/src/mumble_onboard/bin/perf /usr/bin/perf
    # Run this on the host, too
    sudo sysctl -w kernel.perf_event_paranoid=-1
    sudo sysctl -w kernel.kptr_restrict=0
    cd /home/mumble_robot/src/mumble_physical_runtime
fi