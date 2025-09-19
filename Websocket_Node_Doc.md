# Web Controller - 远程控制节点

本节点提供了一个 `WebSocket API`，用于通过局域网实时远程控制机器人。
## 功能
    - 实时运动控制: 通过 `WebSocket` 发送指令，控制机器人的前进、后退、平移和旋转。

    - 设备控制: 控制机器人上的其他设备，如蜂鸣器。

    - 断线保护: 当客户端（如浏览器页面）断开连接时，机器人会自动停止运动，确保安全。

## 如何使用
### 1. 启动服务

首先，确保整个 Docker 项目已通过根目录的 `start.sh` 脚本成功启动。

该脚本会自动启动 `web_controller` 节点，并在树莓派主机的 8000 端口上开启 WebSocket 服务。
### 2. 连接到 WebSocket

在任何位于同一局域网下的客户端设备（如你的电脑或手机）上，你可以通过编写简单的客户端程序（如 Python 或 JavaScript）来连接到服务。

    - WebSocket 地址: `ws://<树莓派的IP地址>:8000/ws`

### 3. 发送控制指令

连接成功后，客户端可以向服务器发送 JSON 格式 的文本消息来控制机器人。所有指令都通过一个统一的 `action` 字段来区分。
#### 指令格式

##### a) 移动 (`move`)

控制机器人的线速度和角速度。
```
{
  "action": "move",
  "linear_x": 0.5,  // 前后速度 (m/s), 正为前, 负为后
  "linear_y": -0.2, // 左右平移速度 (m/s), 正为左, 负为右
  "angular_z": 0.8  // 原地旋转速度 (rad/s), 正为逆时针, 负为顺时针
}
```
提示：你可以只提供需要控制的参数，其他参数会默认为0。

##### b) 停止 (`stop`)
```
立即停止所有运动。这是最常用的安全指令。

{
  "action": "stop"
}
```
##### c) 鸣笛 (`beep`)
```
让蜂鸣器响一声。

{
  "action": "beep"
}
```
### 4. 示例 (Python 客户端)

这是一个简单的 Python 测试脚本，可以用来验证服务是否正常工作。
```Python
import asyncio
import websockets
import json

async def control_robot():
    # 替换为你的树莓派IP地址
    uri = "ws://192.168.1.100:8000/ws"
    
    async with websockets.connect(uri) as websocket:
        print("--- 已连接到机器人 ---")

        # 示例1: 前进1秒
        print("向前移动...")
        await websocket.send(json.dumps({"action": "move", "linear_x": 0.3}))
        await asyncio.sleep(1)

        # 示例2: 停止
        print("停止...")
        await websocket.send(json.dumps({"action": "stop"}))
        await asyncio.sleep(1)

        # 示例3: 鸣笛
        print("鸣笛...")
        await websocket.send(json.dumps({"action": "beep"}))
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(control_robot())
```
将以上代码保存为 `test_client.py`，修改IP地址后运行 `python test_client.py` 即可看到效果。