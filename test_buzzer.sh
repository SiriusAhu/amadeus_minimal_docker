#!/bin/bash

echo "--- Running final hardware test via SDK with correct baud rate... ---"

docker exec -it amadeus_control bash -lc '
source /ros2_ws/install/setup.bash

python3 - << "PY"
import time
from ros_robot_controller.ros_robot_controller_sdk import Board

print("--- Python script started inside container ---")
try:
    board = Board()
    board.enable_reception()

    print("Sending BUZZER command...")
    board.set_buzzer(1000, 0.2, 0, 1)   # 1000Hz, 0.2s
    time.sleep(0.3)

    print("Stopping BUZZER...")
    if hasattr(board, "stop_buzzer"):
        board.stop_buzzer()
    else:
        board.set_buzzer(0, 0, 0, 0)    # fallback stop

    print("--- Test finished ---")
except Exception as e:
    print("Test failed:", e)
PY
'
