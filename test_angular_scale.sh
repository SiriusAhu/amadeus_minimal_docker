#!/bin/bash
# =========================================
# 角速度缩放因子测试脚本
# =========================================
# 用法: ./test_angular_scale.sh <scale_value>
# 例如: ./test_angular_scale.sh 0.5
#       ./test_angular_scale.sh 0.8
#       ./test_angular_scale.sh 1.0
#
# 此脚本会：
# 1. 设置环境变量 ANGULAR_SPEED_SCALE
# 2. 重启 Docker 容器
# 3. 你可以立即测试旋转效果
# =========================================

SCALE=${1:-0.5}

echo "========================================="
echo "设置角速度缩放因子: $SCALE"
echo "========================================="

# 停止现有容器
./stop.sh

# 用新的环境变量启动容器
docker run -d --rm \
    --name ros2-minimal \
    --network host \
    --privileged \
    -v /dev:/dev \
    -e ANGULAR_SPEED_SCALE=$SCALE \
    ros2-minimal

echo ""
echo "容器已启动，角速度缩放因子: $SCALE"
echo "请测试旋转效果，如需调整请运行:"
echo "  ./test_angular_scale.sh <新值>"
echo ""
echo "推荐测试值: 0.3, 0.5, 0.7, 0.8, 1.0"
echo "========================================="

