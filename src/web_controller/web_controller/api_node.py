# api_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from std_msgs.msg import UInt16
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
import threading
import asyncio
import json
import time
import math

# 电池电压范围 (mV) - 2S 锂电池
BATTERY_MIN_MV = 6400   # 3.2V x 2 = 6.4V (空电)
BATTERY_MAX_MV = 8400   # 4.2V x 2 = 8.4V (满电)
BATTERY_LOW_MV = 6800   # 低电量警告阈值

# 定义ROS 2节点
class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('web_api_controller')
        self.twist_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buzzer_publisher_ = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        
        # 订阅电池状态
        self.battery_voltage_mv = 0
        self.battery_last_update = 0
        self.battery_subscription = self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self._battery_callback,
            10
        )
        
        self.get_logger().info('Web API Controller Node has been started.')

        # 用来保存一次性定时器，避免被 GC
        self._beep_timers = set()
    
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


# ============ 表演动作定义 ============
# 这些动作可以通过 WebSocket API 调用，也可以通过 MCP 被 LLM 调用

PERFORMANCE_ACTIONS = {
    "spin": {
        "name": "转圈",
        "name_en": "Spin",
        "description": "原地旋转一圈或多圈",
        "params": {
            "circles": {"type": "float", "default": 1.0, "min": 0.5, "max": 5.0, "desc": "圈数"},
            "speed": {"type": "int", "default": 50, "min": 10, "max": 100, "desc": "速度百分比"},
            "direction": {"type": "str", "default": "left", "options": ["left", "right"], "desc": "旋转方向"}
        }
    },
    "shake": {
        "name": "摇头",
        "name_en": "Shake",
        "description": "左右摇头表示否定或可爱",
        "params": {
            "times": {"type": "int", "default": 3, "min": 1, "max": 10, "desc": "摇头次数"},
            "speed": {"type": "int", "default": 60, "min": 20, "max": 100, "desc": "速度百分比"}
        }
    },
    "figure8": {
        "name": "画8字",
        "name_en": "Figure 8",
        "description": "画8字形轨迹",
        "params": {
            "size": {"type": "float", "default": 1.0, "min": 0.5, "max": 2.0, "desc": "大小倍数"},
            "speed": {"type": "int", "default": 40, "min": 20, "max": 80, "desc": "速度百分比"}
        }
    },
    "celebrate": {
        "name": "庆祝",
        "name_en": "Celebrate",
        "description": "庆祝动作：转圈+鸣笛",
        "params": {
            "intensity": {"type": "int", "default": 50, "min": 20, "max": 100, "desc": "强度百分比"}
        }
    },
    "greet": {
        "name": "打招呼",
        "name_en": "Greet",
        "description": "鸣笛打招呼",
        "params": {}
    }
}


class PerformanceExecutor:
    """表演动作执行器"""
    
    def __init__(self, ros_node: RosBridgeNode):
        self.ros_node = ros_node
        self._running = False
        self._cancel_event = asyncio.Event()
        self._lock = asyncio.Lock()  # 防止竞争条件
    
    def cancel(self):
        """取消当前表演"""
        self._cancel_event.set()
        # 不直接设置 _running = False，让 execute() 的 finally 块处理
        # 立即停止机器人
        self.ros_node.stop_robot()
        self.ros_node.stop_beep()
    
    def is_cancelled(self) -> bool:
        """检查是否被取消"""
        return self._cancel_event.is_set()
    
    def force_reset(self):
        """强制重置状态（仅在确定没有执行时调用）"""
        self._running = False
        self._cancel_event.clear()
    
    async def _sleep(self, duration: float) -> bool:
        """可取消的 sleep，返回 False 表示被取消"""
        if self._cancel_event.is_set():
            return False
        try:
            await asyncio.wait_for(self._cancel_event.wait(), timeout=duration)
            self.ros_node.stop_robot()  # 取消时立即停止
            return False  # 被取消
        except asyncio.TimeoutError:
            return True  # 正常完成
    
    async def execute(self, action: str, params: dict) -> dict:
        """执行表演动作"""
        if action not in PERFORMANCE_ACTIONS:
            return {"success": False, "error": f"未知动作: {action}"}
        
        # 使用锁防止竞争条件
        if self._running:
            return {"success": False, "error": "另一个表演正在进行中"}
        
        async with self._lock:
            if self._running:
                return {"success": False, "error": "另一个表演正在进行中"}
            
            self._running = True
            self._cancel_event.clear()
        
        try:
            # 获取动作定义并合并默认参数
            action_def = PERFORMANCE_ACTIONS[action]
            full_params = {}
            for k, v in action_def.get("params", {}).items():
                full_params[k] = params.get(k, v["default"])
            
            # 执行对应动作
            method = getattr(self, f"_do_{action}", None)
            if method:
                await method(full_params)
                if self.is_cancelled():
                    return {"success": False, "error": "动作被取消", "cancelled": True}
                self.ros_node.stop_robot()  # 确保动作结束后停止
                return {"success": True, "action": action, "name": action_def["name"]}
            else:
                return {"success": False, "error": f"动作 {action} 未实现"}
        except asyncio.CancelledError:
            return {"success": False, "error": "动作被取消", "cancelled": True}
        finally:
            self.ros_node.stop_robot()
            self._running = False
    
    async def _do_spin(self, params: dict):
        """转圈 - 原地旋转指定圈数"""
        circles = float(params.get("circles", 1.0))
        speed = float(params.get("speed", 50)) / 100.0
        direction = params.get("direction", "left")
        
        # 角速度 1.5 rad/s 是一个合适的旋转速度
        angular_speed = 1.5 * max(0.3, speed)  # 最低30%速度
        if direction == "right":
            angular_speed = -angular_speed
        
        # 计算旋转时间: 一圈 = 2π rad
        duration = circles * (2 * math.pi / abs(angular_speed))
        
        self.ros_node.publish_twist(0, 0, angular_speed)
        await self._sleep(duration)
        self.ros_node.stop_robot()
    
    async def _do_shake(self, params: dict):
        """摇头 - 左右快速转动"""
        times = int(params.get("times", 3))
        speed = float(params.get("speed", 60)) / 100.0
        
        # 使用较大的角速度确保能动
        angular_speed = 2.0 * max(0.4, speed)  # 最低40%速度
        shake_duration = 0.3  # 每个方向0.3秒
        
        for i in range(times):
            if self.is_cancelled():
                break
            # 左转
            self.ros_node.publish_twist(0, 0, angular_speed)
            if not await self._sleep(shake_duration):
                break
            # 右转
            self.ros_node.publish_twist(0, 0, -angular_speed)
            if not await self._sleep(shake_duration * 2):
                break
            # 回正
            self.ros_node.publish_twist(0, 0, angular_speed)
            if not await self._sleep(shake_duration):
                break
            # 短暂停顿
            self.ros_node.stop_robot()
            if not await self._sleep(0.15):
                break
        
        self.ros_node.stop_robot()
    
    async def _do_figure8(self, params: dict):
        """画8字 - 两个连续的圆弧"""
        size = float(params.get("size", 1.0))
        speed = float(params.get("speed", 40)) / 100.0
        
        linear_speed = 0.12 * max(0.4, speed) * size
        angular_speed = 1.2 * max(0.4, speed)
        duration_per_circle = 2.5  # 每个圆弧2.5秒
        
        # 第一个圈 (左转)
        if self.is_cancelled():
            return
        self.ros_node.publish_twist(linear_speed, 0, angular_speed)
        if not await self._sleep(duration_per_circle):
            self.ros_node.stop_robot()
            return
        
        # 第二个圈 (右转)
        if self.is_cancelled():
            return
        self.ros_node.publish_twist(linear_speed, 0, -angular_speed)
        if not await self._sleep(duration_per_circle):
            self.ros_node.stop_robot()
            return
        
        self.ros_node.stop_robot()
    
    async def _do_celebrate(self, params: dict):
        """庆祝 - 简化版本，确保能正常结束"""
        intensity = float(params.get("intensity", 50)) / 100.0
        intensity = max(0.4, intensity)  # 最低40%强度
        
        # 1. 鸣笛
        self.ros_node.beep(freq=1500, duration=0.3)
        if not await self._sleep(0.4):
            return
        
        # 2. 快速转半圈
        if self.is_cancelled():
            return
        angular_speed = 2.0 * intensity
        self.ros_node.publish_twist(0, 0, angular_speed)
        if not await self._sleep(1.5):  # 约半圈
            self.ros_node.stop_robot()
            return
        self.ros_node.stop_robot()
        
        # 3. 鸣笛两声
        if self.is_cancelled():
            return
        self.ros_node.beep(freq=2000, duration=0.15)
        if not await self._sleep(0.25):
            return
        self.ros_node.beep(freq=2500, duration=0.15)
        if not await self._sleep(0.25):
            return
        
        # 4. 左右晃两下
        if self.is_cancelled():
            return
        shake_speed = 1.8 * intensity
        self.ros_node.publish_twist(0, 0, shake_speed)
        if not await self._sleep(0.3):
            self.ros_node.stop_robot()
            return
        self.ros_node.publish_twist(0, 0, -shake_speed)
        if not await self._sleep(0.3):
            self.ros_node.stop_robot()
            return
        
        self.ros_node.stop_robot()
    
    async def _do_greet(self, params: dict):
        """打招呼 - 前进后退+鸣笛"""
        # 鸣笛
        self.ros_node.beep(freq=1000, duration=0.2)
        if not await self._sleep(0.3):
            return
        
        # 前进一点
        if self.is_cancelled():
            return
        self.ros_node.publish_twist(0.12, 0, 0)  # 稍快的速度
        if not await self._sleep(0.5):
            self.ros_node.stop_robot()
            return
        
        # 后退回来
        if self.is_cancelled():
            return
        self.ros_node.publish_twist(-0.12, 0, 0)
        if not await self._sleep(0.5):
            self.ros_node.stop_robot()
            return
        
        self.ros_node.stop_robot()
        
        # 再鸣笛
        if self.is_cancelled():
            return
        self.ros_node.beep(freq=1200, duration=0.15)
        if not await self._sleep(0.2):
            return
        self.ros_node.beep(freq=1500, duration=0.15)
        if not await self._sleep(0.2):
            return
        
        self.ros_node.stop_robot()

class WebApiServer:
    def __init__(self, ros_node):
        self.app = FastAPI()
        self.ros_node = ros_node
        self.performance_executor = PerformanceExecutor(ros_node)

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.ros_node.get_logger().info("WebSocket client connected.")
            try:
                while True:
                    data = await websocket.receive_text()
                    cmd = json.loads(data)

                    action = cmd.get("action")
                    
                    # 心跳响应
                    if action == "ping":
                        await websocket.send_json({
                            "action": "pong",
                            "timestamp": cmd.get("timestamp", 0)
                        })
                        continue

                    if action == "move":
                        self.ros_node.publish_twist(
                            linear_x=cmd.get("linear_x", 0.0),
                            linear_y=cmd.get("linear_y", 0.0),
                            angular_z=cmd.get("angular_z", 0.0)
                        )

                    elif action == "stop":
                        # 停底盘 + 取消表演
                        self.performance_executor.cancel()
                        self.ros_node.stop_robot()
                        # 发送确认响应
                        await websocket.send_json({
                            "action": "stop_ack",
                            "cancelled": self.performance_executor.is_cancelled(),
                            "timestamp": cmd.get("timestamp", 0)
                        })

                    elif action == "beep":
                        # 支持客户端传入 frequency/duration
                        freq = cmd.get("frequency", 2000)
                        duration = cmd.get("duration", 0.2)
                        self.ros_node.beep(freq=freq, duration=duration)

                    elif action in ("stop_beep", "buzzer_off"):
                        # 显式停蜂鸣（客户端保险调用）
                        self.ros_node.stop_beep()
                    
                    elif action == "get_status":
                        # 返回车辆状态（包含电池信息）
                        battery_info = self.ros_node.get_battery_info()
                        await websocket.send_json({
                            "action": "status",
                            "connected": True,
                            "battery": battery_info,
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    elif action == "get_battery":
                        # 单独获取电池状态
                        battery_info = self.ros_node.get_battery_info()
                        await websocket.send_json({
                            "action": "battery",
                            **battery_info,
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    # ====== 表演动作 ======
                    elif action == "perform":
                        # 执行表演动作
                        perform_action = cmd.get("perform_action", "")
                        params = cmd.get("params", {})
                        timestamp = cmd.get("timestamp", 0)
                        
                        # 检查是否可以执行
                        if self.performance_executor._running:
                            await websocket.send_json({
                                "action": "perform_result",
                                "perform_action": perform_action,
                                "success": False,
                                "error": "另一个表演正在进行中",
                                "timestamp": timestamp
                            })
                        else:
                            # 立即回复已开始
                            await websocket.send_json({
                                "action": "perform_started",
                                "perform_action": perform_action,
                                "timestamp": timestamp
                            })
                            
                            # 在后台执行表演，完成后发送结果
                            async def run_perform():
                                result = await self.performance_executor.execute(perform_action, params)
                                try:
                                    await websocket.send_json({
                                        "action": "perform_result",
                                        "perform_action": result.get("action", perform_action),
                                        "success": result.get("success", False),
                                        "name": result.get("name", ""),
                                        "error": result.get("error", ""),
                                        "cancelled": result.get("cancelled", False),
                                        "timestamp": timestamp
                                    })
                                except Exception:
                                    pass  # WebSocket 可能已关闭
                            
                            asyncio.create_task(run_perform())
                    
                    elif action == "cancel_perform":
                        # 取消表演
                        self.performance_executor.cancel()
                        await websocket.send_json({
                            "action": "perform_cancelled",
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    elif action == "list_performances":
                        # 列出所有可用表演动作
                        performances = []
                        for key, val in PERFORMANCE_ACTIONS.items():
                            performances.append({
                                "id": key,
                                "name": val["name"],
                                "name_en": val["name_en"],
                                "description": val["description"],
                                "params": val["params"]
                            })
                        await websocket.send_json({
                            "action": "performances_list",
                            "performances": performances,
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    # ====== 视觉功能 (预留接口) ======
                    elif action == "vision_status":
                        # 获取视觉功能状态
                        await websocket.send_json({
                            "action": "vision_status",
                            "yolo_available": False,  # 待实现
                            "follow_available": False,  # 待实现
                            "follow_state": "idle",
                            "message": "视觉功能需要安装 ultralytics: pip install ultralytics",
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    elif action == "start_follow":
                        # 开始人体跟随 (预留)
                        await websocket.send_json({
                            "action": "follow_result",
                            "success": False,
                            "error": "人体跟随功能尚未启用，需要安装 YOLO",
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    elif action == "stop_follow":
                        # 停止人体跟随 (预留)
                        await websocket.send_json({
                            "action": "follow_result",
                            "success": True,
                            "state": "idle",
                            "timestamp": cmd.get("timestamp", 0)
                        })
                    
                    elif action == "detect_objects":
                        # 目标检测 (预留)
                        await websocket.send_json({
                            "action": "detection_result",
                            "success": False,
                            "error": "YOLO 检测功能尚未启用",
                            "detections": [],
                            "timestamp": cmd.get("timestamp", 0)
                        })

            except WebSocketDisconnect:
                self.ros_node.get_logger().warn("WebSocket client disconnected.")
            finally:
                # 兜底：断开时同时停车+停蜂鸣+取消表演
                self.performance_executor.cancel()
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