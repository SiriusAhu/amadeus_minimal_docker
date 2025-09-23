#!/bin/bash

echo "--- [Correct Way] Testing ROS 2 Mecanum Logic via /cmd_vel topic ---"
echo "This script will send a command to move FORWARD for 1 second, then STOP."
echo "This WILL execute the code in mecanum.py."

# 在运行中的容器内部执行ROS2命令
docker exec -it amadeus_control bash -c "
  # 加载ROS2环境
  source /ros2_ws/install/setup.bash

  # 指令1: 发布一个前进指令到 /cmd_vel 话题
  echo '>>> Sending MOVE FORWARD command (linear.x = 0.3)...'
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

  # 等待1秒，让机器人运动
  echo '--- Moving for 1 second... ---'
  sleep 1

  # 指令2: 发布一个停止指令
  echo '>>> Sending STOP command...'
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

  echo '--- Test finished ---'
"