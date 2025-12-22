#!/usr/bin/env python3
"""
Amadeus WiFi Provisioning Server
提供 WiFi 配网的 Web API

部署到树莓派后运行：
  sudo python3 provisioning_server.py

API 端点：
  GET  /           - 设备信息
  GET  /api/scan   - 扫描 WiFi 列表
  POST /api/connect - 连接到 WiFi
  GET  /api/status - 获取当前网络状态
  POST /api/reset  - 重置网络配置
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import subprocess
import time
import uvicorn
import socket

app = FastAPI(title="Amadeus Provisioning API", version="1.0.0")

# 允许跨域
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class ConnectRequest(BaseModel):
    ssid: str
    password: str

def get_device_id():
    """获取设备 ID (MAC 地址后 4 位)"""
    try:
        result = subprocess.run(
            ['cat', '/sys/class/net/wlan0/address'],
            capture_output=True, text=True
        )
        mac = result.stdout.strip()
        return mac[-5:].replace(':', '').upper()
    except:
        return "0000"

@app.get("/", response_class=HTMLResponse)
async def root():
    """配网首页"""
    device_id = get_device_id()
    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Amadeus 配网</title>
        <style>
            * {{ box-sizing: border-box; }}
            body {{ 
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
                background: #1a1a1a; color: #fff;
                margin: 0; padding: 20px;
                min-height: 100vh;
            }}
            .container {{ max-width: 400px; margin: 0 auto; }}
            h1 {{ text-align: center; margin-bottom: 30px; }}
            .card {{ 
                background: #2a2a2a; border-radius: 12px; 
                padding: 20px; margin-bottom: 16px;
            }}
            .wifi-item {{ 
                display: flex; justify-content: space-between; align-items: center;
                padding: 12px; border-radius: 8px; cursor: pointer;
                transition: background 0.2s;
            }}
            .wifi-item:hover {{ background: #3a3a3a; }}
            .wifi-name {{ font-weight: 500; }}
            .wifi-signal {{ color: #888; font-size: 14px; }}
            input {{ 
                width: 100%; padding: 12px; border-radius: 8px;
                border: 1px solid #444; background: #333; color: #fff;
                font-size: 16px; margin-bottom: 12px;
            }}
            button {{ 
                width: 100%; padding: 14px; border-radius: 8px;
                border: none; background: #0066ff; color: #fff;
                font-size: 16px; font-weight: 600; cursor: pointer;
                transition: opacity 0.2s;
            }}
            button:hover {{ opacity: 0.9; }}
            button:disabled {{ background: #555; cursor: not-allowed; }}
            .status {{ text-align: center; padding: 20px; color: #888; }}
            .success {{ color: #4CAF50; }}
            .error {{ color: #f44336; }}
            #wifiList {{ max-height: 300px; overflow-y: auto; }}
            #passwordModal {{ 
                display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0;
                background: rgba(0,0,0,0.8); padding: 20px;
                align-items: center; justify-content: center;
            }}
            #passwordModal.show {{ display: flex; }}
            .modal-content {{ 
                background: #2a2a2a; border-radius: 12px; padding: 24px;
                width: 100%; max-width: 360px;
            }}
            .modal-title {{ font-size: 18px; font-weight: 600; margin-bottom: 16px; }}
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🚗 Amadeus-{device_id}</h1>
            
            <div class="card">
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 16px;">
                    <span style="font-weight: 600;">选择 WiFi 网络</span>
                    <button onclick="scanWifi()" style="width: auto; padding: 8px 16px;">刷新</button>
                </div>
                <div id="wifiList">
                    <div class="status">点击刷新扫描 WiFi...</div>
                </div>
            </div>
            
            <div class="card">
                <div id="statusText" class="status">等待配网...</div>
            </div>
        </div>
        
        <div id="passwordModal">
            <div class="modal-content">
                <div class="modal-title">连接到: <span id="selectedSSID"></span></div>
                <input type="password" id="passwordInput" placeholder="输入 WiFi 密码">
                <button onclick="connectWifi()">连接</button>
                <button onclick="closeModal()" style="background: #555; margin-top: 8px;">取消</button>
            </div>
        </div>
        
        <script>
            let selectedSSID = '';
            
            async function scanWifi() {{
                document.getElementById('wifiList').innerHTML = '<div class="status">扫描中...</div>';
                try {{
                    const res = await fetch('/api/scan');
                    const data = await res.json();
                    if (data.networks.length === 0) {{
                        document.getElementById('wifiList').innerHTML = '<div class="status">未找到 WiFi</div>';
                        return;
                    }}
                    document.getElementById('wifiList').innerHTML = data.networks.map(n => `
                        <div class="wifi-item" onclick="selectWifi('${{n.ssid}}')">
                            <span class="wifi-name">${{n.ssid}}</span>
                            <span class="wifi-signal">${{n.signal}}% ${{n.security ? '🔒' : ''}}</span>
                        </div>
                    `).join('');
                }} catch (e) {{
                    document.getElementById('wifiList').innerHTML = '<div class="status error">扫描失败</div>';
                }}
            }}
            
            function selectWifi(ssid) {{
                selectedSSID = ssid;
                document.getElementById('selectedSSID').textContent = ssid;
                document.getElementById('passwordInput').value = '';
                document.getElementById('passwordModal').classList.add('show');
            }}
            
            function closeModal() {{
                document.getElementById('passwordModal').classList.remove('show');
            }}
            
            async function connectWifi() {{
                const password = document.getElementById('passwordInput').value;
                const statusEl = document.getElementById('statusText');
                
                statusEl.className = 'status';
                statusEl.textContent = '正在连接...';
                closeModal();
                
                try {{
                    const res = await fetch('/api/connect', {{
                        method: 'POST',
                        headers: {{ 'Content-Type': 'application/json' }},
                        body: JSON.stringify({{ ssid: selectedSSID, password }})
                    }});
                    const data = await res.json();
                    
                    if (data.success) {{
                        statusEl.className = 'status success';
                        statusEl.innerHTML = `✅ 配网成功！<br>设备 IP: ${{data.ip_address}}<br><br>请切换到您的 WiFi 网络<br>然后使用 App 连接设备`;
                    }} else {{
                        statusEl.className = 'status error';
                        statusEl.textContent = '❌ ' + data.message;
                    }}
                }} catch (e) {{
                    // 网络切换时可能会超时，这其实可能是成功的
                    statusEl.className = 'status';
                    statusEl.innerHTML = `⏳ 网络切换中...<br><br>如果小车已成功连接新 WiFi：<br>1. 请切换到您的 WiFi 网络<br>2. 使用 App 扫描或手动输入小车 IP<br><br>如果连接失败，请刷新页面重试`;
                }}
            }}
            
            // 页面加载时扫描
            scanWifi();
        </script>
    </body>
    </html>
    """

@app.get("/api/info")
async def get_info():
    """获取设备信息"""
    return {
        "device": "Amadeus Smart Car",
        "device_id": get_device_id(),
        "version": "1.0.0",
        "hostname": socket.gethostname(),
        "status": "provisioning_mode"
    }

@app.get("/api/scan")
async def scan_wifi():
    """扫描附近的 WiFi 网络"""
    try:
        # 使用 nmcli 扫描
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'SSID,SIGNAL,SECURITY', 'dev', 'wifi', 'list', '--rescan', 'yes'],
            capture_output=True,
            text=True,
            timeout=15
        )
        
        if result.returncode != 0:
            raise HTTPException(status_code=500, detail="WiFi scan failed")
        
        networks = []
        seen_ssids = set()
        
        for line in result.stdout.strip().split('\n'):
            if not line:
                continue
            
            parts = line.split(':')
            if len(parts) >= 2:
                ssid = parts[0]
                signal = int(parts[1]) if parts[1].isdigit() else 0
                security = parts[2] if len(parts) > 2 else ""
                
                # 去重（保留信号最强的），跳过空 SSID
                if ssid and ssid not in seen_ssids:
                    seen_ssids.add(ssid)
                    networks.append({
                        "ssid": ssid,
                        "signal": signal,
                        "security": security
                    })
        
        # 按信号强度排序
        networks.sort(key=lambda x: x['signal'], reverse=True)
        
        return {"networks": networks}
    
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=504, detail="Scan timeout")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/connect")
async def connect_wifi(request: ConnectRequest):
    """连接到指定的 WiFi 网络"""
    try:
        # 检查是否已存在该连接
        check_cmd = ['nmcli', 'con', 'show', request.ssid]
        check_result = subprocess.run(check_cmd, capture_output=True)
        
        if check_result.returncode == 0:
            # 连接已存在，删除旧的
            subprocess.run(['nmcli', 'con', 'delete', request.ssid])
        
        # 创建新连接
        connect_cmd = [
            'nmcli', 'dev', 'wifi', 'connect',
            request.ssid,
            'password', request.password
        ]
        
        result = subprocess.run(
            connect_cmd,
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            # 连接成功，等待获取 IP
            time.sleep(3)
            
            # 获取新 IP 地址
            ip_result = subprocess.run(
                ['hostname', '-I'],
                capture_output=True,
                text=True
            )
            ips = ip_result.stdout.strip().split()
            # 过滤掉 AP 模式的 IP (192.168.4.x)
            new_ip = "unknown"
            for ip in ips:
                if not ip.startswith("192.168.4."):
                    new_ip = ip
                    break
            
            return {
                "success": True,
                "message": "WiFi connected successfully",
                "ip_address": new_ip
            }
        else:
            error_msg = result.stderr.strip() or "Unknown error"
            return {
                "success": False,
                "message": f"Connection failed: {error_msg}"
            }
    
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=504, detail="Connection timeout")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/status")
async def get_status():
    """获取当前网络状态"""
    try:
        # 检查是否已连接 WiFi
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi'],
            capture_output=True,
            text=True
        )
        
        for line in result.stdout.strip().split('\n'):
            if line.startswith('yes:'):
                ssid = line.split(':')[1]
                # 获取 IP
                ip_result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
                ips = ip_result.stdout.strip().split()
                client_ip = next((ip for ip in ips if not ip.startswith("192.168.4.")), "unknown")
                
                return {
                    "connected": True,
                    "ssid": ssid,
                    "ip_address": client_ip,
                    "mode": "client"
                }
        
        return {
            "connected": False,
            "mode": "ap"
        }
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/reset")
async def reset_network():
    """重置网络配置（删除所有已保存的 WiFi）"""
    try:
        # 获取所有 WiFi 连接
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'NAME,TYPE', 'con', 'show'],
            capture_output=True,
            text=True
        )
        
        deleted = []
        for line in result.stdout.strip().split('\n'):
            if ':802-11-wireless' in line or ':wifi' in line:
                name = line.split(':')[0]
                # 保留 AP 配置
                if not name.startswith('Amadeus'):
                    subprocess.run(['nmcli', 'con', 'delete', name])
                    deleted.append(name)
        
        return {
            "success": True,
            "message": f"Deleted {len(deleted)} WiFi configurations",
            "deleted": deleted
        }
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    print("=" * 50)
    print("  Amadeus WiFi Provisioning Server")
    print("  访问 http://192.168.4.1 进行配网")
    print("=" * 50)
    uvicorn.run(app, host="0.0.0.0", port=80, log_level="info")

