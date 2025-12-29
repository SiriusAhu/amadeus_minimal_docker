#!/bin/bash
# Amadeus WiFi Provisioning 安装脚本
# 在树莓派上运行: chmod +x install.sh && sudo ./install.sh

set -e

echo "======================================"
echo "  Amadeus WiFi Provisioning 安装"
echo "======================================"

# 检查是否为 root
if [ "$EUID" -ne 0 ]; then
    echo "请使用 sudo 运行此脚本"
    exit 1
fi

# 安装依赖
echo "[1/5] 安装 Python 依赖..."
# 创建虚拟环境
VENV_DIR="/opt/amadeus/venv"
python3 -m venv $VENV_DIR || {
    echo "创建虚拟环境失败，尝试安装 python3-venv..."
    apt-get update && apt-get install -y python3-venv python3-full
    python3 -m venv $VENV_DIR
}
$VENV_DIR/bin/pip install fastapi uvicorn pydantic

# 创建目录
echo "[2/5] 创建目录..."
mkdir -p /opt/amadeus

# 复制配网服务器
echo "[3/5] 复制配网服务器..."
cp provisioning_server.py /opt/amadeus/
chmod +x /opt/amadeus/provisioning_server.py

# 创建 NetworkManager AP 配置
echo "[4/5] 配置 WiFi AP..."
DEVICE_ID=$(cat /sys/class/net/wlan0/address 2>/dev/null | tail -c 6 | tr -d ':' | tr '[:lower:]' '[:upper:]' || echo "0000")
AP_SSID="Amadeus-${DEVICE_ID}"

cat > /etc/NetworkManager/system-connections/Amadeus-AP.nmconnection << EOF
[connection]
id=Amadeus-AP
type=wifi
autoconnect=false

[wifi]
mode=ap
ssid=${AP_SSID}
band=bg
channel=6

[wifi-security]
key-mgmt=wpa-psk
psk=amadeus

[ipv4]
method=shared
address1=192.168.4.1/24

[ipv6]
method=ignore
EOF

chmod 600 /etc/NetworkManager/system-connections/Amadeus-AP.nmconnection
nmcli connection reload || true

# 创建 WiFi 管理脚本
cat > /usr/local/bin/amadeus-wifi-manager.sh << 'SCRIPT'
#!/bin/bash
# Amadeus WiFi 管理脚本
# 功能：检查 WiFi 连接状态，如果未连接则启动 AP 模式

check_wifi_connection() {
    # 检查是否有活动的 WiFi 连接
    # 排除 AP 模式本身的连接
    nmcli -t -f ACTIVE,SSID,TYPE con show --active 2>/dev/null | grep -E 'wifi' | grep -v "Amadeus-AP" | grep -q '^yes'
    return $?
}

start_ap_mode() {
    echo "[Amadeus] 未检测到 WiFi 连接，启动 AP 模式..."
    # 停止可能导致冲突的 WiFi 连接
    nmcli -t -f NAME,TYPE con show --active 2>/dev/null | grep 'wireless' | while read line; do
        name=$(echo "$line" | cut -d: -f1)
        if [ "$name" != "Amadeus-AP" ]; then
            echo "[Amadeus] 断开 WiFi 连接: $name"
            nmcli con down "$name" 2>/dev/null || true
        fi
    done
    # 启动 AP 模式
    nmcli con up Amadeus-AP 2>/dev/null || true
    # 启动配网服务
    systemctl start amadeus-provisioning.service 2>/dev/null || true
    echo "[Amadeus] AP 模式已启动"
}

stop_ap_mode() {
    echo "[Amadeus] 停止 AP 模式..."
    systemctl stop amadeus-provisioning.service 2>/dev/null || true
    nmcli con down Amadeus-AP 2>/dev/null || true
}

# 主逻辑
if ! check_wifi_connection; then
    start_ap_mode
else
    echo "[Amadeus] WiFi 已连接，无需 AP 模式"
    stop_ap_mode
fi
SCRIPT

chmod +x /usr/local/bin/amadeus-wifi-manager.sh

# 创建持续监控脚本（用于在 WiFi 断开时自动启动 AP）
cat > /usr/local/bin/amadeus-wifi-monitor.sh << 'SCRIPT'
#!/bin/bash
# Amadeus WiFi 持续监控脚本
# 功能：持续监控 WiFi 连接状态，断开时自动启动 AP

LOG_FILE="/var/log/amadeus-wifi-monitor.log"
INTERVAL=10  # 检查间隔（秒）

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}

check_wifi_connection() {
    nmcli -t -f ACTIVE,SSID,TYPE con show --active 2>/dev/null | grep -E 'wifi' | grep -v "Amadeus-AP" | grep -q '^yes'
    return $?
}

start_ap_mode() {
    log "WiFi 已断开，启动 AP 模式..."
    # 停止可能导致冲突的 WiFi 连接
    nmcli -t -f NAME,TYPE con show --active 2>/dev/null | grep 'wireless' | while read line; do
        name=$(echo "$line" | cut -d: -f1)
        if [ "$name" != "Amadeus-AP" ]; then
            nmcli con down "$name" 2>/dev/null || true
        fi
    done
    # 启动 AP 模式
    nmcli con up Amadeus-AP 2>/dev/null || true
    systemctl start amadeus-provisioning.service 2>/dev/null || true
    log "AP 模式已启动"
}

stop_ap_mode() {
    log "WiFi 已连接，停止 AP 模式..."
    systemctl stop amadeus-provisioning.service 2>/dev/null || true
    nmcli con down Amadeus-AP 2>/dev/null || true
}

log "WiFi 监控服务启动"

while true; do
    if check_wifi_connection; then
        # WiFi 已连接，检查 AP 是否在运行，如果在则停止
        if nmcli con show --active | grep -q "Amadeus-AP"; then
            stop_ap_mode
            log "WiFi 已恢复，AP 已停止"
        fi
    else
        # WiFi 未连接，检查 AP 是否在运行，如果不在则启动
        if ! nmcli con show --active | grep -q "Amadeus-AP"; then
            start_ap_mode
        fi
    fi
    sleep $INTERVAL
done
SCRIPT

chmod +x /usr/local/bin/amadeus-wifi-monitor.sh

# 创建 systemd 服务
echo "[5/5] 创建 systemd 服务..."

cat > /etc/systemd/system/amadeus-provisioning.service << EOF
[Unit]
Description=Amadeus WiFi Provisioning Service
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/opt/amadeus
ExecStart=/opt/amadeus/venv/bin/python /opt/amadeus/provisioning_server.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 启动时检查 WiFi 连接
cat > /etc/systemd/system/amadeus-wifi-check.service << EOF
[Unit]
Description=Amadeus WiFi Connection Check (One-time)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/amadeus-wifi-manager.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# 持续监控 WiFi 连接状态（断线时自动启动 AP）
cat > /etc/systemd/system/amadeus-wifi-monitor.service << EOF
[Unit]
Description=Amadeus WiFi Connection Monitor
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/local/bin/amadeus-wifi-monitor.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# 重新加载 systemd
systemctl daemon-reload

# 启用 WiFi 检查服务和持续监控服务
systemctl enable amadeus-wifi-check.service
systemctl enable amadeus-wifi-monitor.service

echo ""
echo "======================================"
echo "  安装完成!"
echo "======================================"
echo ""
echo "AP SSID: ${AP_SSID}"
echo "AP 密码: amadeus"
echo "配网地址: http://192.168.4.1"
echo ""
echo "功能说明:"
echo "  - 启动时检查 WiFi：如果未连接自动启动 AP"
echo "  - 持续监控 WiFi：断线时自动重新启动 AP"
echo ""
echo "使用方法:"
echo "  1. 启动 AP 模式:"
echo "     sudo /usr/local/bin/amadeus-wifi-manager.sh"
echo ""
echo "  2. 手动启动配网服务:"
echo "     sudo systemctl start amadeus-provisioning"
echo ""
echo "  3. 查看 WiFi 监控日志:"
echo "     tail -f /var/log/amadeus-wifi-monitor.log"
echo ""
echo "  4. 查看配网服务日志:"
echo "     journalctl -u amadeus-provisioning -f"
echo ""

