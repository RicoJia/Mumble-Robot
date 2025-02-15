#!/bin/bash

# This script is to launch the Mumble-Robot docker containers based on architectures.
# If we are on rpi, we will launch the arm container. (Real robot)
# If we are on x86, we will launch the amd64 container. (Simulation)

# Usage:
# ./launch_mumble.sh [build_rpi]
#   build_rpi: Optional argument to build the Raspberry Pi container on x86_64 architecture.

check_sudo() {
  if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root. Please use 'sudo'." >&2
    exit 1
  fi
}
check_sudo

ARCH=$(uname -m)
CURRENT_DIR=$(dirname $(realpath docker-compose.yml))
if [ "$ARCH" = "aarch64" ]; then
    # CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up --build -d
    CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up -d
fi

if [ "$ARCH" = "x86_64" ]; then
    if [ "$1" = "build_rpi" ]; then
        docker buildx build \
        --platform linux/arm64/v8 \
        --build-arg WORKDIRECTORY=/home/mumble_physical_runtime \
        -t ricojia/mumble_physical_runtime_rpi:latest \
        -f ./mumble_physical_runtime/Dockerfile_mumble_physical_runtime \
        ./mumble_physical_runtime \
        --push
    else
        CURRENT_DIR=$CURRENT_DIR docker compose --profile amd up --build  # TODO: to change to arm
    fi
    # docker compose --profile amd64 up -d
fi