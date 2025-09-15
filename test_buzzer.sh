#!/bin/bash

echo "--- Running final hardware test via SDK with correct baud rate... ---"

docker exec -it amadeus_control bash -c "
source /ros2_ws/install/setup.bash && \
python3 -c \"
import time
from ros_robot_controller.ros_robot_controller_sdk import Board
print('--- Python script started inside container ---')
try:
    print('Initializing board with correct baud rate (1000000)...')
    # Initialize with same code
    board = Board()
    board.enable_reception()
    print('Board initialized.')
    
    print('Sending BUZZER command...')
    # Send a buzzer command that lasts for 0.2 seconds
    board.set_buzzer(1000, 0.2, 0, 1)
    print('Command sent. Listen for the beep!')
    
    time.sleep(0.5)
    print('--- Test finished ---')

except Exception as e:
    print('!!! TEST FAILED !!!')
    print(f'An error occurred: {e}')
\"
"
