#!/bin/bash

# This script is to launch the Mumble-Robot docker containers based on architectures.
# If we are on rpi, we will launch the arm container. (Real robot)
# If we are on x86, we will launch the amd64 container. (Simulation)

# TODO: not sure if aarch 64 is what we need
ARCH=$(uname -m)
CURRENT_DIR=$(dirname $(realpath docker-compose.yml))
if [ "$ARCH" = "aarch64" ]; then
    CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up --build -d
fi

if [ "$ARCH" = "x86_64" ]; then
    # docker compose --profile amd64 up -d
    CURRENT_DIR=$CURRENT_DIR docker compose --profile arm up --build  # TODO: to change to arm
fi