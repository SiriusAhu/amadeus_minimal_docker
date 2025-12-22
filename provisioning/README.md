# WiFi 配网服务

本目录包含 Amadeus 智能车的 WiFi 配网功能。

## 📦 文件说明

| 文件 | 说明 |
|------|------|
| `provisioning_server.py` | 配网 Web 服务器 (FastAPI) |
| `install.sh` | 一键安装脚本 |

## 🚀 安装

```bash
# SSH 到小车
ssh amadeus@小车IP

# 进入目录
cd amadeus_minimal_docker/provisioning

# 运行安装
chmod +x install.sh
sudo ./install.sh
```

## 📋 安装后配置

安装成功后会显示：

```
AP SSID: Amadeus-XXXX
AP 密码: amadeus
配网地址: http://192.168.4.1
```

## 🔧 使用方法

### 自动模式

小车开机后，如果无法连接已知 WiFi，会自动启动 AP 模式。

### 手动模式

```bash
# 启动 AP 模式
sudo /usr/local/bin/amadeus-wifi-manager.sh

# 启动配网服务
sudo systemctl start amadeus-provisioning

# 查看日志
journalctl -u amadeus-provisioning -f
```

## 📱 配网流程

1. 手机搜索 WiFi，连接 `Amadeus-XXXX` (密码: `amadeus`)
2. 浏览器访问 `http://192.168.4.1`
3. 选择目标 WiFi，输入密码
4. 等待连接成功，记录新 IP
5. 手机切换回原 WiFi
6. 在 App 中使用新 IP 连接

## 🔒 安全说明

- AP 默认密码: `amadeus`
- 建议首次配网后修改密码
- 配网成功后 AP 模式会关闭

## 🛠️ 故障排除

### 找不到热点

1. 确保小车已开机
2. 等待 30 秒让系统启动
3. 检查 LED 指示灯
4. 手动运行 `sudo /usr/local/bin/amadeus-wifi-manager.sh`

### 连接显示"网络切换中"

这是正常现象！小车成功连接新 WiFi 后，AP 模式会断开，导致配网页面无法收到成功响应。

**解决方法**：
1. 切换到你的 WiFi 网络
2. 在 App 中输入小车的新 IP（通常与路由器分配一致）
3. 或者在路由器管理页面查看小车的 IP

### 密码错误

请确认密码区分大小写，WiFi 名称正确无误。

