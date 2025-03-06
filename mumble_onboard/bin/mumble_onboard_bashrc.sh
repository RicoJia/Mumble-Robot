colcon_build_source(){
    colcon build --symlink-install
    source install/setup.bash
}

if [ ! -d "/home/mumble_robot/build/" ]; then
    echo "First container launch: running colcon build..."
    colcon_build_source
    cd /home/mumble_robot/src/mumble_physical_runtime
fi