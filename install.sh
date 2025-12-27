#!/bin/bash
#
# Amadeus Docker 安装脚本
# 适用于树莓派和其他 Linux 系统
#
# 功能:
# 1. 安装 Docker (如未安装)
# 2. 构建 Docker 镜像
# 3. 配置 udev 规则 (用于串口权限)
# 4. 配置开机自启服务
# 5. 安装并配置 WiFi 配网服务
#
# 用法: sudo bash install.sh
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# 检查 root 权限
check_root() {
    if [ "$EUID" -ne 0 ]; then
        error "请使用 sudo 运行此脚本"
    fi
}

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 获取实际用户 (非 root)
ACTUAL_USER="${SUDO_USER:-$USER}"
ACTUAL_HOME=$(getent passwd "$ACTUAL_USER" | cut -d: -f6)

info "========================================"
info "  Amadeus Docker 安装脚本"
info "========================================"
echo

# 配置国内镜像源
configure_mirrors() {
    info "配置国内镜像源..."
    
    # 配置 Docker 镜像
    mkdir -p /etc/docker
    cat > /etc/docker/daemon.json << 'EOF'
{
    "registry-mirrors": [
        "https://docker.mirrors.ustc.edu.cn",
        "https://hub-mirror.c.163.com",
        "https://mirror.ccs.tencentyun.com"
    ]
}
EOF
    
    success "Docker 镜像源已配置"
}

# 安装 Docker
install_docker() {
    if command -v docker &> /dev/null; then
        success "Docker 已安装: $(docker --version)"
        return
    fi
    
    info "安装 Docker..."
    
    # 使用清华源安装
    apt update
    apt install -y apt-transport-https ca-certificates curl gnupg lsb-release
    
    # 添加 Docker 官方 GPG 密钥
    curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # 添加 Docker 仓库
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/debian $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    apt update
    apt install -y docker-ce docker-ce-cli containerd.io
    
    # 将用户添加到 docker 组
    usermod -aG docker "$ACTUAL_USER"
    
    success "Docker 安装成功"
}

# 配置 udev 规则
setup_udev() {
    info "配置 udev 规则..."
    
    # 串口权限规则
    cat > /etc/udev/rules.d/99-amadeus.rules << 'EOF'
# Amadeus Robot Controller
KERNEL=="ttyACM*", MODE="0666"
KERNEL=="ttyUSB*", MODE="0666"
KERNEL=="ttyAMA*", MODE="0666"
# 摄像头
SUBSYSTEM=="video4linux", MODE="0666"
EOF
    
    udevadm control --reload-rules
    udevadm trigger
    
    success "udev 规则已配置"
}

# 构建 Docker 镜像
build_image() {
    info "构建 Docker 镜像 (这可能需要 10-30 分钟)..."
    
    # 使用与 start.sh 一致的镜像名称
    docker build -t amadeus:minimal .
    
    success "Docker 镜像构建完成"
}

# 配置 Docker 服务
setup_docker_service() {
    info "配置 Docker 开机自启服务..."
    
    # 使用与 start.sh 一致的配置：
    # - 镜像名称: amadeus:minimal
    # - 端口映射: 9000:8000 (App 连接 9000 端口)
    # - 设备映射: /dev/ttyAMA0:/dev/rrc
    cat > /etc/systemd/system/amadeus-docker.service << EOF
[Unit]
Description=Amadeus Docker Container - 车辆控制容器
After=docker.service
Requires=docker.service

[Service]
Type=simple
User=$ACTUAL_USER
Restart=on-failure
RestartSec=10
ExecStartPre=-/usr/bin/docker stop amadeus_control
ExecStartPre=-/usr/bin/docker rm amadeus_control
ExecStart=/usr/bin/docker run --rm \\
    --name amadeus_control \\
    -p 9000:8000 \\
    --device=/dev/ttyAMA0:/dev/rrc \\
    --privileged=true \\
    amadeus:minimal
ExecStop=/usr/bin/docker stop amadeus_control

[Install]
WantedBy=multi-user.target
EOF
    
    systemctl daemon-reload
    systemctl enable amadeus-docker.service
    
    success "Docker 服务已配置: amadeus-docker.service"
}

# 安装配网服务
install_provisioning() {
    info "安装 WiFi 配网服务..."
    
    if [ -d "provisioning" ]; then
        cd provisioning
        bash install.sh
        cd ..
        success "配网服务安装完成"
    else
        warn "未找到 provisioning 目录，跳过配网服务安装"
    fi
}

# 安装摄像头服务
install_camera() {
    info "安装摄像头服务 (mjpg-streamer)..."
    
    # 检查是否已安装
    if command -v mjpg_streamer &> /dev/null; then
        success "mjpg-streamer 已安装"
        return
    fi
    
    # 安装依赖
    apt install -y cmake libjpeg-dev
    
    # 克隆并构建
    cd /tmp
    if [ ! -d "mjpg-streamer" ]; then
        git clone https://github.com/jacksonliam/mjpg-streamer.git
    fi
    
    cd mjpg-streamer/mjpg-streamer-experimental
    make
    make install
    
    cd "$SCRIPT_DIR"
    
    # 创建摄像头服务
    cat > /etc/systemd/system/amadeus-camera.service << 'EOF'
[Unit]
Description=Amadeus Camera Stream (MJPG-Streamer)
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 640x480 -f 15" -o "output_http.so -p 8081 -w /usr/local/share/mjpg-streamer/www"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
    
    systemctl daemon-reload
    systemctl enable amadeus-camera.service
    
    success "摄像头服务已配置: amadeus-camera.service"
}

# 创建辅助脚本
create_scripts() {
    info "创建辅助脚本..."
    
    # 测试脚本
    cat > test.sh << 'EOF'
#!/bin/bash
echo "===== Amadeus Docker 状态检查 ====="
echo

# 检查 Docker 容器
if docker ps | grep -q amadeus_control; then
    echo "✅ Docker 容器运行中"
else
    echo "❌ Docker 容器未运行"
fi

# 检查 WebSocket API (端口 9000 映射到容器内 8000)
if curl -s --max-time 2 http://localhost:9000/ 2>/dev/null; then
    echo "✅ WebSocket API 正常 (端口 9000)"
else
    echo "⚠️  WebSocket API 不可用 (端口 9000)"
fi

# 检查摄像头
if curl -s --max-time 2 -o /dev/null -w "%{http_code}" "http://localhost:8081/?action=snapshot" | grep -q "200"; then
    echo "✅ 摄像头服务正常 (端口 8081)"
else
    echo "⚠️  摄像头服务不可用"
fi

echo
echo "服务端口:"
echo "  WebSocket API: 9000"
echo "  摄像头: 8081"
echo
EOF
    chmod +x test.sh
    
    success "辅助脚本已创建"
}

# 主流程
main() {
    check_root
    configure_mirrors
    install_docker
    setup_udev
    build_image
    setup_docker_service
    install_provisioning
    install_camera
    create_scripts
    
    echo
    info "========================================"
    success "安装完成!"
    info "========================================"
    echo
    echo "使用方法:"
    echo "  启动容器: sudo systemctl start amadeus-docker"
    echo "  停止容器: sudo systemctl stop amadeus-docker"
    echo "  查看日志: sudo docker logs -f amadeus_control"
    echo "  测试状态: ./test.sh"
    echo
    echo "  启动摄像头: sudo systemctl start amadeus-camera"
    echo "  配网模式: sudo systemctl start amadeus-provisioning"
    echo
    echo "服务端口:"
    echo "  WebSocket API: 9000"
    echo "  摄像头: 8081"
    echo
    echo "请重新登录以使 docker 组权限生效"
    echo
}

main "$@"

