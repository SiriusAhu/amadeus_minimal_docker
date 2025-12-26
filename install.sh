#!/bin/bash
# ============================================================
# Amadeus Docker 安装脚本
# 用于在新设备上快速安装和配置 Docker 容器
# ============================================================

set -e

echo "======================================================"
echo "  Amadeus Docker 安装脚本"
echo "======================================================"

# 检查是否在正确的目录
if [ ! -f "Dockerfile" ]; then
    echo "❌ 错误: 请在 amadeus_minimal_docker 目录下运行此脚本"
    exit 1
fi

# 检查是否安装了 Docker
if ! command -v docker &> /dev/null; then
    echo "📦 Docker 未安装，开始安装..."
    
    # 使用官方脚本安装 Docker（自动检测系统）
    curl -fsSL https://get.docker.com | sh
    
    # 添加当前用户到 docker 组
    sudo usermod -aG docker $USER
    
    echo "✅ Docker 安装完成"
    echo "⚠️  请注意：需要重新登录才能使用 docker 命令"
    echo "   运行 'newgrp docker' 或重新登录后再次运行此脚本"
    exit 0
fi

echo "✅ Docker 已安装: $(docker --version)"

# 检查设备
HOST_DEVICE="/dev/ttyAMA0"
if [ ! -e "$HOST_DEVICE" ]; then
    echo "⚠️  警告: 设备 $HOST_DEVICE 不存在"
    echo "   请确认机器人主板已连接"
    echo "   使用 'dmesg -w' 确认正确的设备名"
    echo ""
    read -p "是否继续安装（仅构建镜像）？[y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 构建 Docker 镜像
echo ""
echo "📦 构建 Docker 镜像..."
docker build -t amadeus:minimal .

echo ""
echo "✅ 镜像构建完成"

# 询问是否启动容器
echo ""
read -p "是否现在启动容器？[Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then
    echo "跳过容器启动"
    echo ""
    echo "稍后可以使用以下命令启动容器:"
    echo "  ./start.sh"
else
    echo ""
    echo "🚀 启动容器..."
    ./start.sh
fi

echo ""
echo "======================================================"
echo "  安装完成！"
echo "======================================================"
echo ""
echo "常用命令："
echo "  ./start.sh    - 启动容器"
echo "  ./stop.sh     - 停止容器"
echo "  ./enter.sh    - 进入容器终端"
echo ""
echo "WebSocket API 端口: 9000"
echo "示例: ws://<IP>:9000/ws"
echo ""

