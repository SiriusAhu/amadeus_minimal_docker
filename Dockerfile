# Dockerfile

# 1. 设置基础镜像
FROM ros:humble-ros-base

# 2. 设置工作环境
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 3. 安装核心依赖和便利工具 (包括常用文本编辑器Vim, Nano和作者爱用的Neovim)
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-rosdep \
    vim \
    nano \
    neovim \
    && rm -rf /var/lib/apt/lists/*

# 4. 配置 rosdep 并安装所需的Python库（清华大学镜像源）
RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc pyserial fastapi "uvicorn[standard]" websockets && \
    rosdepc init && \
    rosdepc update

# 5. 创建 ROS2 工作区
WORKDIR /ros2_ws

# 6. 复制源码
COPY ./src ./src

# 7. 安装 ROS 依赖
RUN rosdepc install --from-paths src -y --ignore-src

# 8. 编译工作区
RUN . /opt/ros/humble/setup.bash && colcon build

# 9. Set the executable permission on our script
RUN chmod +x /ros2_ws/install/web_controller/lib/web_controller/web_api_server

# 10. 创建符号链接以解决硬编码路径问题
RUN mkdir -p /home/ubuntu/software/Servo_upper_computer && \
    ln -s /ros2_ws/install/minimal_bringup/share/minimal_bringup/config/servo_config.yaml /home/ubuntu/software/Servo_upper_computer/servo_config.yaml

# 11. 设置容器启动命令 (Entrypoint)
COPY <<'EOF' /ros_entrypoint.sh
#!/bin/bash
set -e

# 在脚本最开始，加载一次ROS环境
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"

# 1. 在后台启动机器人运动控制
echo "--- Starting ROS 2 motion control nodes in background... ---"
ros2 launch minimal_bringup minimal_motion.launch.py &

# 等待几秒钟
sleep 5

# 2. 在前台启动Web API服务
echo "--- Starting Web API controller node in foreground... ---"
exec ros2 launch web_controller web_controller.launch.py
EOF

# 修复换行符问题并赋予执行权限
RUN sed -i 's/\r$//' /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# 设置容器的入口点
ENTRYPOINT ["/ros_entrypoint.sh"]