#!/bin/bash
# Amadeus Docker 一键安装脚本
# 适用于 Raspberry Pi (Debian/Ubuntu)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================"
echo "   Amadeus Docker 安装脚本"
echo "============================================"
echo ""

# 检查是否为 root
if [ "$EUID" -eq 0 ]; then
    echo "⚠️  请不要使用 root 用户运行此脚本"
    exit 1
fi

# 检测系统
echo "📋 检测系统环境..."
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
else
    OS=$(uname -s)
    VER=$(uname -r)
fi
echo "   系统: $OS $VER"

# 配置国内 Docker 镜像源
configure_docker_mirrors() {
    echo ""
    echo "📦 配置 Docker 镜像源..."
    
    sudo mkdir -p /etc/docker
    
    # 检查是否已配置
    if [ -f /etc/docker/daemon.json ] && grep -q "registry-mirrors" /etc/docker/daemon.json; then
        echo "   ✅ Docker 镜像源已配置"
        return
    fi
    
    # 配置国内镜像
    cat > /tmp/daemon.json << 'EOF'
{
    "registry-mirrors": [
        "https://mirror.ccs.tencentyun.com",
        "https://docker.mirrors.ustc.edu.cn"
    ]
}
EOF
    sudo mv /tmp/daemon.json /etc/docker/daemon.json
    
    # 重启 Docker
    if systemctl is-active --quiet docker; then
        sudo systemctl restart docker
    fi
    
    echo "   ✅ Docker 镜像源已配置 (腾讯云 + USTC)"
}

# 安装 Docker
install_docker() {
    echo ""
    echo "🐳 检查/安装 Docker..."
    
    if command -v docker &> /dev/null; then
        DOCKER_VERSION=$(docker --version 2>/dev/null || echo "unknown")
        echo "   ✅ Docker 已安装: $DOCKER_VERSION"
    else
        echo "   📥 安装 Docker..."
        
        # 使用官方脚本安装
        curl -fsSL https://get.docker.com | sh
        
        # 添加当前用户到 docker 组
        sudo usermod -aG docker $USER
        
        echo "   ✅ Docker 安装完成"
        echo "   ⚠️  请注销并重新登录以使 docker 组生效"
    fi
    
    # 确保 Docker 服务运行
    if ! systemctl is-active --quiet docker; then
        sudo systemctl start docker
        sudo systemctl enable docker
    fi
}

# 检测串口设备
detect_device() {
    echo ""
    echo "🔌 检测串口设备..."
    
    # 常见设备路径
    DEVICES=("/dev/ttyAMA0" "/dev/ttyUSB0" "/dev/ttyACM0" "/dev/serial0")
    
    DETECTED_DEVICE=""
    for dev in "${DEVICES[@]}"; do
        if [ -e "$dev" ]; then
            echo "   找到设备: $dev"
            DETECTED_DEVICE="$dev"
            break
        fi
    done
    
    if [ -z "$DETECTED_DEVICE" ]; then
        echo "   ⚠️  未检测到串口设备"
        echo "   请确认机器人主板已连接，使用 'dmesg -w' 查看设备"
        DETECTED_DEVICE="/dev/ttyAMA0"
    fi
    
    echo "   使用设备: $DETECTED_DEVICE"
}

# 构建 Docker 镜像
build_image() {
    echo ""
    echo "🏗️  构建 Docker 镜像..."
    
    cd "$PROJECT_DIR"
    
    docker build -t amadeus:minimal .
    
    echo "   ✅ 镜像构建完成"
}

# 启动容器
start_container() {
    echo ""
    echo "🚀 启动容器..."
    
    cd "$PROJECT_DIR"
    
    # 停止旧容器
    if docker ps -a --format '{{.Names}}' | grep -Eq "^amadeus_control$"; then
        echo "   停止旧容器..."
        docker rm -f amadeus_control
    fi
    
    # 启动新容器
    docker run -d \
        --name amadeus_control \
        -p 9000:8000 \
        --device=$DETECTED_DEVICE:/dev/rrc \
        --privileged=true \
        --restart=always \
        amadeus:minimal
    
    sleep 3
    
    if docker ps --format '{{.Names}}' | grep -Eq "^amadeus_control$"; then
        echo "   ✅ 容器启动成功"
    else
        echo "   ❌ 容器启动失败"
        docker logs amadeus_control
        exit 1
    fi
}

# 安装配网服务
install_provisioning() {
    echo ""
    echo "📡 安装配网服务..."
    
    if [ -f "$PROJECT_DIR/provisioning/install.sh" ]; then
        cd "$PROJECT_DIR/provisioning"
        bash install.sh
        echo "   ✅ 配网服务已安装"
    else
        echo "   ⚠️  配网服务脚本不存在，跳过"
    fi
}

# 显示完成信息
show_complete() {
    echo ""
    echo "============================================"
    echo "   ✅ Amadeus Docker 安装完成!"
    echo "============================================"
    echo ""
    echo "📌 常用命令:"
    echo "   查看容器: docker ps"
    echo "   查看日志: docker logs -f amadeus_control"
    echo "   重启容器: docker restart amadeus_control"
    echo "   进入容器: docker exec -it amadeus_control bash"
    echo ""
    echo "🌐 服务端口:"
    echo "   WebSocket: ws://localhost:9000/ws"
    echo ""
}

# 主流程
main() {
    install_docker
    configure_docker_mirrors
    detect_device
    build_image
    start_container
    install_provisioning
    show_complete
}

main "$@"

