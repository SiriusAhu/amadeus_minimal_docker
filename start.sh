#!/bin/bash

# Change this to "true" to enable persistence.
# When enabled, the container will not be deleted after stopping, and the built workspace and logs will be saved.
# This allows you to keep changes made inside the container, such as installing new software or modifying code, for the next startup.
ENABLE_PERSISTENCE="false"

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

# Choose different docker run options based on whether persistence is enabled
DOCKER_RUN_OPTIONS=""
if [ "$ENABLE_PERSISTENCE" = "true" ]; then
    echo "--- Persistence enabled ---"
    # -v creates a data volume named amadeus_ws_data and mounts it to /ros2_ws in the container
    # This will save the build/, install/, log/ folders
    # Omit the --rm option so the container is not deleted after stopping
    DOCKER_RUN_OPTIONS="-it -v amadeus_ws_data:/ros2_ws --name amadeus_control"
else
    echo "--- Persistence disabled (container will be automatically deleted after stopping) ---"
    # --rm option will automatically delete the container when it stops
    DOCKER_RUN_OPTIONS="--rm -it --name amadeus_control"
fi

echo "--- Starting container 'amadeus_control' in privileged mode... ---"
echo "--- Mapping host device '$HOST_DEVICE' to '/dev/rrc' in container... ---"

# Using --privileged=true option to solve all potential device permission issues
docker run $DOCKER_RUN_OPTIONS --device=$HOST_DEVICE:/dev/rrc --privileged=true amadeus:minimal