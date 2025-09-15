#!/bin/bash

echo "--- Entering container 'amadeus_control'... ---"
echo "--- You are now inside the container. Type 'exit' to leave. ---"

# Enter the container using exec and start a bash shell
# bash -c '...' means to ensure that after entering, we first source the environment, then start a new interactive bash
docker exec -it amadeus_control bash -c "source /ros2_ws/install/setup.bash && bash"
