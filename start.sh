#!/bin/bash

# Change this to "true" to enable persistence.
# When enabled, the container will not be deleted after stopping, and the built workspace and logs will be saved.
# This allows you to keep changes made inside the container, such as installing new software or modifying code, for the next startup.
ENABLE_PERSISTENCE="false"

# Change this to "true" to enable auto-start on system boot.
# When enabled, the container will automatically restart with the system.
ENABLE_AUTOSTART="true"

# Please modify this to the correct device name you confirmed with dmesg
HOST_DEVICE="/dev/ttyAMA0"


# Check if the device exists
if [ ! -e "$HOST_DEVICE" ]; then
    echo "Error: Device $HOST_DEVICE does not exist!"
    echo "Please reconnect your robot mainboard and use 'dmesg -w' to confirm the correct device name, then modify this script."
    exit 1
fi

set -e

echo "--- Building Docker image... ---"
docker build -t amadeus:minimal .

# Kill and remove old container if it exists (to avoid name conflict)
if docker ps -a --format '{{.Names}}' | grep -Eq "^amadeus_control\$"; then
    echo "--- Removing old container 'amadeus_control' ---"
    docker rm -f amadeus_control
fi

# Choose different docker run options based on persistence and autostart
DOCKER_RUN_OPTIONS=""

if [ "$ENABLE_PERSISTENCE" = "true" ]; then
    echo "--- Persistence enabled ---"
    DOCKER_RUN_OPTIONS="-d -v amadeus_ws_data:/ros2_ws --name amadeus_control"
else
    echo "--- Persistence disabled ---"
    if [ "$ENABLE_AUTOSTART" = "true" ]; then
        # ⚠️ auto-start requires container to be kept, so cannot use --rm
        DOCKER_RUN_OPTIONS="-d --name amadeus_control"
        echo "--- Container will NOT be auto-removed, since auto-start is enabled ---"
    else
        # safe to auto-remove when not autostart
        DOCKER_RUN_OPTIONS="-d --rm --name amadeus_control"
        echo "--- Container will be automatically deleted after stopping ---"
    fi
fi


# Choose restart policy based on ENABLE_AUTOSTART
RESTART_POLICY=""
if [ "$ENABLE_AUTOSTART" = "true" ]; then
    RESTART_POLICY="--restart=always"
    echo "--- Auto-start enabled ---"
else
    RESTART_POLICY="--restart=no"
    echo "--- Auto-start disabled ---"
fi

# --- Mapping port 9000 to container port 8000 ---
# Port 9000 is used to avoid conflicts with common ports
PORT_MAPPING="-p 9000:8000"

echo "--- Starting container 'amadeus_control' in privileged mode... ---"
echo "--- Mapping host device '$HOST_DEVICE' to '/dev/rrc' in container... ---"
echo "--- Mapping host port 9000 to container port 8000 (WebSocket)... ---"

# Using --privileged=true option to solve all potential device permission issues
docker run $DOCKER_RUN_OPTIONS $PORT_MAPPING --device=$HOST_DEVICE:/dev/rrc --privileged=true $RESTART_POLICY amadeus:minimal