#!/bin/bash

# This script is to launch the Mumble-Robot docker containers based on architectures.
# If we are on rpi, we will launch the arm container. (Real robot)
# If we are on x86, we will launch the amd64 container. (Simulation)

# Usage:
# ./launch_mumble.sh [build_rpi] [build_onboard]
#   build_rpi: Optional argument to build the Raspberry Pi container on x86_64 architecture.

check_sudo() {
  if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root. Please use 'sudo'." >&2
    exit 1
  fi
}

launch_runtime() {
    CURRENT_DIR=$SCRIPT_DIR/mumble_physical_runtime
    cd $CURRENT_DIR
    
    if [ "$ARCH" = "aarch64" ]; then
        # CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up --build -d
        CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up -d
    fi

    if [ "$ARCH" = "x86_64" ]; then
        if [ "$1" = "build_rpi_runtime" ]; then
            docker buildx build \
            --platform linux/arm64/v8 \
            --build-arg WORKDIRECTORY=/home/mumble_physical_runtime \
            -t ricojia/mumble_physical_runtime_rpi:latest \
            -f ./Dockerfile_mumble_physical_runtime \
            ./mumble_physical_runtime \
            --push
        else
            CURRENT_DIR=$CURRENT_DIR docker compose --profile amd up --build  -d # TODO: to change to arm
        fi
    fi
    cd $SCRIPT_DIR
}

launch_onboard() {
    CURRENT_DIR=$SCRIPT_DIR/mumble_onboard
    cd $CURRENT_DIR
    if [ "$ARCH" = "aarch64" ]; then
        # CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up --build -d
        CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up -d
    else
        CURRENT_DIR=$CURRENT_DIR docker compose --profile amd up --build  -d # TODO: to change to arm
    fi
    if [ "$1" = "build_rpi_onboard" ]; then 
        docker buildx build \
        --platform linux/arm64/v8 \
        --build-arg WORKDIRECTORY=/home/mumble_onboard \
        --build-arg BUILD_PCL_FROM_SOURCE=0 \
        -t ricojia/mumble_onboard_rpi:latest \
        -f ./Dockerfile_mumble_onboard \
        ./mumble_onboard \
        --push
    fi
    cd $SCRIPT_DIR
    # docker compose --profile amd64 up -d
    echo "mumble"

}

launch_isaac_sim(){
    CURRENT_DIR=$SCRIPT_DIR/mumble_simulation_isaac
    cd $CURRENT_DIR
    if [ "$ARCH" = "aarch64" ]; then
        echo "Isaac sim can only be run on aarch 64"
    else
        CURRENT_DIR=$CURRENT_DIR docker compose --profile amd up -d
    fi
    cd $SCRIPT_DIR
}

terminate_all_containers(){
    echo "Stopping containers ... "
    docker stop mumble_onboard_container mumble_physical_runtime_container mumble_simulation_isaac_container
}

if [ "$1" = "kill" ]; then 
    terminate_all_containers
else
    ARCH=$(uname -m)
    SCRIPT_DIR=$(dirname $(realpath $0))
    check_sudo
    launch_runtime $1
    launch_onboard $1
    # launch_isaac_sim $1
fi
