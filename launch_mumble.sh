#!/bin/bash

# This script is to launch the Mumble-Robot docker containers based on architectures.
# If we are on rpi, we will launch the arm container. (Real robot)
# If we are on x86, we will launch the amd64 container. (Simulation)

# TODO: not sure if aarch 64 is what we need
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    docker compose --profile arm up
fi

if [ "$ARCH" = "x86_64" ]; then
    docker compose --profile amd64 up
fi