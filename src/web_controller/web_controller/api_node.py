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

    # --- 新增3: 一个发送蜂鸣器指令的新方法 ---
    def beep(self, freq=2000, duration=0.2):
        msg = BuzzerState()
        msg.freq = freq
        msg.on_time = duration
        msg.off_time = 0.0  # 响一次即可
        msg.repeat = 1
        self.buzzer_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Buzzer: freq={freq}, duration={duration}')

class WebApiServer:
    def __init__(self, ros_node):
        self.app = FastAPI()
        self.ros_node = ros_node

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.ros_node.get_logger().info(f"WebSocket client connected.")
            try:
                while True:
                    data = await websocket.receive_text()
                    cmd = json.loads(data)
                    
                    if cmd.get("action") == "move":
                        self.ros_node.publish_twist(
                            linear_x=cmd.get("linear_x", 0.0),
                            linear_y=cmd.get("linear_y", 0.0),
                            angular_z=cmd.get("angular_z", 0.0)
                        )
                    elif cmd.get("action") == "stop":
                        self.ros_node.stop_robot()
                    elif cmd.get("action") == "beep":
                        self.ros_node.beep()

            except WebSocketDisconnect:
                self.ros_node.get_logger().warn(f"WebSocket client disconnected.")
            finally:
                self.ros_node.stop_robot()

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