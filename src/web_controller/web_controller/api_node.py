# api_node.py
# 最小化的 WebSocket API 控制器
# 只提供基本的底层控制指令，高级逻辑（如表演动作）由外部实现

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from std_msgs.msg import UInt16
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
import threading
import json
import time

# 电池电压范围 (mV) - 2S 锂电池
BATTERY_MIN_MV = 6400   # 3.2V x 2 = 6.4V (空电)
BATTERY_MAX_MV = 8400   # 4.2V x 2 = 8.4V (满电)
BATTERY_LOW_MV = 6800   # 低电量警告阈值


class RosBridgeNode(Node):
    """ROS 2 节点：桥接 WebSocket 和 ROS 话题"""
    
    def __init__(self):
        super().__init__('web_api_controller')
        
        # 发布者
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buzzer_publisher = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        
        # 订阅电池状态
        self.battery_voltage_mv = 0
        self.battery_last_update = 0
        self.create_subscription(UInt16, '/ros_robot_controller/battery', self._battery_callback, 10)
        
        # 蜂鸣器定时器管理
        self._beep_timers = set()
        
        self.get_logger().info('Web API Controller Node started.')
    
    def _battery_callback(self, msg: UInt16):
        """电池电压回调"""
        self.battery_voltage_mv = msg.data
        self.battery_last_update = time.time()
    
    def get_battery_info(self) -> dict:
        """获取电池信息"""
        voltage_mv = self.battery_voltage_mv
        voltage_v = voltage_mv / 1000.0
        
        # 计算百分比
        if voltage_mv <= BATTERY_MIN_MV:
            percentage = 0
        elif voltage_mv >= BATTERY_MAX_MV:
            percentage = 100
        else:
            percentage = int((voltage_mv - BATTERY_MIN_MV) / (BATTERY_MAX_MV - BATTERY_MIN_MV) * 100)
        
        # 判断状态
        if voltage_mv == 0:
            status = "unknown"
        elif voltage_mv < BATTERY_LOW_MV:
            status = "low"
        elif voltage_mv > BATTERY_MAX_MV - 200:
            status = "full"
        else:
            status = "normal"
        
        return {
            "voltage_mv": voltage_mv,
            "voltage_v": round(voltage_v, 2),
            "percentage": percentage,
            "status": status,
            "last_update": self.battery_last_update
        }

    def move(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """发布运动指令"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.twist_publisher.publish(msg)
        self.get_logger().debug(f'Twist: x={linear_x:.2f}, y={linear_y:.2f}, z={angular_z:.2f}')
    
    def stop(self):
        """停止运动"""
        self.move(0.0, 0.0, 0.0)

    def beep(self, freq=2000, duration=0.2):
        """蜂鸣器响一次"""
        msg = BuzzerState()
        msg.freq = int(freq)
        msg.on_time = float(duration)
        msg.off_time = 0.0
        msg.repeat = 1
        self.buzzer_publisher.publish(msg)
        
        # 兜底：duration 后强制停止
        def _stop_once():
            try:
                self.stop_beep()
            finally:
                timer.cancel()
                self._beep_timers.discard(timer)
        
        timer = self.create_timer(float(duration), _stop_once)
        self._beep_timers.add(timer)

    def stop_beep(self):
        """停止蜂鸣器"""
        msg = BuzzerState()
        msg.freq = 0
        msg.on_time = 0.0
        msg.off_time = 0.0
        msg.repeat = 0
        self.buzzer_publisher.publish(msg)


class WebApiServer:
    """WebSocket API 服务器"""
    
    def __init__(self, ros_node: RosBridgeNode):
        self.app = FastAPI(title="Amadeus Vehicle API", version="1.0.0")
        self.ros_node = ros_node
        self._setup_routes()

    def _setup_routes(self):
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.ros_node.get_logger().info("Client connected.")
            
            try:
                while True:
                    data = await websocket.receive_text()
                    cmd = json.loads(data)
                    action = cmd.get("action")
                    timestamp = cmd.get("timestamp", 0)
                    
                    # ====== 基础指令 ======
                    
                    if action == "ping":
                        # 心跳
                        await websocket.send_json({"action": "pong", "timestamp": timestamp})
                    
                    elif action == "move":
                        # 运动控制
                        self.ros_node.move(
                            linear_x=cmd.get("linear_x", 0.0),
                            linear_y=cmd.get("linear_y", 0.0),
                            angular_z=cmd.get("angular_z", 0.0)
                        )
                    
                    elif action == "stop":
                        # 紧急停止
                        self.ros_node.stop()
                        self.ros_node.stop_beep()
                        await websocket.send_json({"action": "stop_ack", "timestamp": timestamp})
                    
                    elif action == "beep":
                        # 蜂鸣器
                        self.ros_node.beep(
                            freq=cmd.get("frequency", 2000),
                            duration=cmd.get("duration", 0.2)
                        )
                    
                    elif action == "stop_beep":
                        # 停止蜂鸣器
                        self.ros_node.stop_beep()
                    
                    elif action == "get_status":
                        # 获取状态
                        await websocket.send_json({
                            "action": "status",
                            "connected": True,
                            "battery": self.ros_node.get_battery_info(),
                            "timestamp": timestamp
                        })
                    
                    elif action == "get_battery":
                        # 获取电池状态
                        await websocket.send_json({
                            "action": "battery",
                            **self.ros_node.get_battery_info(),
                            "timestamp": timestamp
                        })
                    
                    else:
                        # 未知指令
                        await websocket.send_json({
                            "action": "error",
                            "error": f"Unknown action: {action}",
                            "timestamp": timestamp
                        })
            
            except WebSocketDisconnect:
                self.ros_node.get_logger().info("Client disconnected.")
            finally:
                # 断开连接时停止
                self.ros_node.stop()
                self.ros_node.stop_beep()

    def run(self):
        uvicorn.run(self.app, host="0.0.0.0", port=8000)


def main(args=None):
    rclpy.init(args=args)
    ros_node = RosBridgeNode()
    server = WebApiServer(ros_node)
    
    # ROS spin 在后台线程
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # 启动 Web 服务器
    server.run()
    
    # 清理
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
