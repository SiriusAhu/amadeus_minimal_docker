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

# 停止并删除所有相关容器
echo "停止现有容器..."
docker stop amadeus_control ros2-minimal 2>/dev/null
docker rm amadeus_control ros2-minimal 2>/dev/null

# 用新的环境变量启动容器
echo "启动新容器..."
docker run -d --rm \
    --name amadeus_control \
    -p 9000:8000 \
    --privileged \
    -v /dev:/dev \
    -e ANGULAR_SPEED_SCALE=$SCALE \
    amadeus:minimal

sleep 2

# 检查容器是否启动成功
if docker ps | grep -q amadeus_control; then
    echo ""
    echo "✓ 容器已启动，角速度缩放因子: $SCALE"
    echo ""
    echo "查看日志确认缩放因子:"
    docker logs amadeus_control 2>&1 | grep -i "角速度" || echo "(等待节点启动...)"
    echo ""
    echo "请测试旋转效果，如需调整请运行:"
    echo "  ./test_angular_scale.sh <新值>"
    echo ""
    echo "推荐测试值: 0.5, 0.7, 0.8, 1.0, 1.2"
    echo "========================================="
else
    echo ""
    echo "✗ 容器启动失败！"
    docker logs amadeus_control 2>&1 | tail -20
fi
