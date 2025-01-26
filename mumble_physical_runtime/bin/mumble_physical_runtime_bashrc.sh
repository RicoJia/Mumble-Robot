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

print_opening_msg_mumble_physical_runtime