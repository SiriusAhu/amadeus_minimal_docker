#!/bin/bash

echo "--- Running final hardware MOTOR test via SDK... ---"

# This command will execute a small piece of Python code inside the container
# It will directly command the motors to rotate at 50% duty cycle for 1 second, then stop
docker exec -it amadeus_control bash -c "
source /ros2_ws/install/setup.bash && \
python3 -c \"
import time
from ros_robot_controller.ros_robot_controller_sdk import Board
print('--- Python script started inside container ---')
try:
    print('Initializing board...')
    board = Board()
    board.enable_reception()
    print('Board initialized.')
    
    print('Sending MOTOR command (all motors at 50 duty)...')
    # Directly call the SDK function to make all motors rotate forward
    board.set_motor_duty([[1, 50], [2, 50], [3, 50], [4, 50]])
    
    # Keep rotating for 1 second
    time.sleep(1)
    
    print('Sending STOP command...')
    # Stop all motors
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])
    
    print('--- Test finished ---')

except Exception as e:
    print('!!! TEST FAILED !!!')
    print(f'An error occurred: {e}')
\"
"
