# api_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
import threading
import json

# 定义ROS 2节点
class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('web_api_controller')
        self.twist_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buzzer_publisher_ = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.get_logger().info('Web API Controller Node has been started.')

        # 用来保存一次性定时器，避免被 GC
        self._beep_timers = set()

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.twist_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Twist: x={linear_x}, y={linear_y}, z={angular_z}')
    
    def stop_robot(self):
        self.publish_twist(0.0, 0.0, 0.0)
        self.get_logger().info('Sent stop command.')

    def beep(self, freq=2000, duration=0.2):
        """开始蜂鸣（一次），并在 duration 后兜底停止。"""
        msg = BuzzerState()
        msg.freq = int(freq)
        msg.on_time = float(duration)
        msg.off_time = 0.0
        msg.repeat = 1
        self.buzzer_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Buzzer: freq={freq}, duration={duration}')

        # —— 兜底：duration 后强制停止（一次性定时器）——
        def _stop_once():
            try:
                self.stop_beep()
            finally:
                # 取消并移除这个定时器
                timer.cancel()
                self._beep_timers.discard(timer)

        # create_timer 是周期性定时器，这里我们用一次回调后立即 cancel
        timer = self.create_timer(float(duration), _stop_once)
        self._beep_timers.add(timer)

    def stop_beep(self):
        """显式停止蜂鸣（幂等）。不同驱动可能要求 freq=0 或 repeat=0。"""
        msg = BuzzerState()
        msg.freq = 0
        msg.on_time = 0.0
        msg.off_time = 0.0
        msg.repeat = 0
        self.buzzer_publisher_.publish(msg)
        self.get_logger().info('Publishing Buzzer STOP')

class WebApiServer:
    def __init__(self, ros_node):
        self.app = FastAPI()
        self.ros_node = ros_node

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.ros_node.get_logger().info("WebSocket client connected.")
            try:
                while True:
                    data = await websocket.receive_text()
                    cmd = json.loads(data)

                    action = cmd.get("action")
                    if action == "move":
                        self.ros_node.publish_twist(
                            linear_x=cmd.get("linear_x", 0.0),
                            linear_y=cmd.get("linear_y", 0.0),
                            angular_z=cmd.get("angular_z", 0.0)
                        )

                    elif action == "stop":
                        # 只停底盘
                        self.ros_node.stop_robot()

                    elif action == "beep":
                        # 支持客户端传入 frequency/duration
                        freq = cmd.get("frequency", 2000)
                        duration = cmd.get("duration", 0.2)
                        self.ros_node.beep(freq=freq, duration=duration)

                    elif action in ("stop_beep", "buzzer_off"):
                        # 显式停蜂鸣（客户端保险调用）
                        self.ros_node.stop_beep()

            except WebSocketDisconnect:
                self.ros_node.get_logger().warn("WebSocket client disconnected.")
            finally:
                # 兜底：断开时同时停车+停蜂鸣
                self.ros_node.stop_robot()
                self.ros_node.stop_beep()

    def run_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=8000)
        
# 3. 主程序入口
def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)

    # 创建ROS 2节点实例
    ros_bridge_node = RosBridgeNode()

    # 创建Web API服务器实例，并传入ROS节点
    web_api_server = WebApiServer(ros_bridge_node)

    # 将ROS 2的spin()放到一个独立的后台线程中运行
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_bridge_node,))
    ros_thread.daemon = True
    ros_thread.start()

    # 在主线程中启动FastAPI/Uvicorn服务器
    web_api_server.run_server()
    
    # 清理
    ros_bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()