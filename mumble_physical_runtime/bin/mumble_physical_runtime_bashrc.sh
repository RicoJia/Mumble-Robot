colcon_build(){
    colcon build --symlink-install
}
source_setup(){
    source install/setup.bash
}
print_opening_msg_mumble_physical_runtime(){
# NO LEADING spacing or tabs
echo -e "$(cat << 'EOF'
\e[31mYum yum  - this is a friendly message from mumble! \e[0m
EOF
)"
}

sudo_ros_preserve_env(){ 
    # when passing args from one func to another, use $@ expansion 
    local cmd="$@" 
    # -E preserves all current user's env variables. 
    # - `source` vs `./`: `./` will createa a copy of the env of the current session for the new script. 
    # when the script is done, the new env is gone. `source` and `.` will execute the script right on the spot 
    echo "Using sudo_ros_preserve_env..."
    sudo -E /bin/bash -c "source ${WORKDIRECTORY}/install/setup.bash; $cmd" 
} 

alias ros2_sudo='sudo_ros_preserve_env ros2'

TODO
print_opening_msg_mumble_physical_runtime
colcon_build
source_setup