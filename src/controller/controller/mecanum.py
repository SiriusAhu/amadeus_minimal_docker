import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorSpeedControl, MotorsSpeedControl  


# ========================================================================
# 角速度缩放因子配置
# ------------------------------------------------------------------------
# 原始值: 1.0 (无缩放)
# 默认值: 1.0 (可通过环境变量 ANGULAR_SPEED_SCALE 调整)
# 修改原因: 麦克纳姆轮四轮同时发力进行纯旋转时，实际角速度过快，导致：
#   1. 摄像头画面模糊，影响 YOLO 检测和人体跟随
#   2. 用户手动控制时旋转过于灵敏，难以精确操控
# 此缩放因子将输入的角速度缩放后再用于电机计算，降低实际旋转速度
#
# 使用方法：设置环境变量来调整，例如：
#   export ANGULAR_SPEED_SCALE=0.8
# ========================================================================
def get_angular_speed_scale() -> float:
    """从环境变量读取角速度缩放因子"""
    try:
        scale = float(os.environ.get('ANGULAR_SPEED_SCALE', '1.0'))
        # 限制范围在 0.1 到 2.0 之间
        return max(0.1, min(2.0, scale))
    except ValueError:
        return 1.0


class MecanumChassis(Node):
    ANGULAR_SPEED_SCALE = get_angular_speed_scale()  # 从环境变量读取，默认 0.5
    
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065, max_linear_speed=1.0, max_angular_speed=1.0):
        super().__init__('mecanum_chassis')
        
        # 启动时打印当前角速度缩放因子
        self.get_logger().info(f'角速度缩放因子: {self.ANGULAR_SPEED_SCALE}')

        # 麦克纳姆轮的物理参数
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
        self.max_linear_speed = max_linear_speed  # 最大线性速度 (m/s)
        self.max_angular_speed = max_angular_speed  # 最大角速度 (rad/s)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(MotorsSpeedControl, '/ros_robot_controller/set_motor_speeds', 10)

    def speed_convert(self, speed, max_speed):
        """
        将速度从 m/s 转换为电机速度，并映射到 -100 到 100 范围。
        :param speed: 速度（m/s）
        :param max_speed: 最大速度（m/s）
        :return: 电机控制速度 (-100 到 100)
        """
        # 按比例将速度转换为电机控制速度
        motor_speed = (speed / max_speed) * 100
        motor_speed = max(-100, min(100, motor_speed))  # 限制在 -100 到 100 之间
        return motor_speed
    # def speed_convert(self, speed, max_speed, angular_speed_factor=10.0):
    #     """
    #     将速度从 m/s 转换为电机速度，并映射到 -100 到 100 范围。
    #     :param speed: 速度（m/s）或角速度（rad/s）
    #     :param max_speed: 最大速度（m/s）或角速度
    #     :param angular_speed_factor: 角速度的比例因子，控制角速度的映射范围
    #     :return: 电机控制速度 (-100 到 100)
    #     """
    #     # 对角速度的映射进行调整
    #     if speed == 0:
    #         motor_speed = 0
    #     else:
    #         if abs(speed) <= max_speed:
    #             motor_speed = (speed / max_speed) * 100
    #         else:
    #             motor_speed = angular_speed_factor * speed
    #             motor_speed = max(-100, min(100, motor_speed))  # 限制在 -100 到 100 之间
        
    #     return motor_speed


    def cmd_vel_callback(self, msg: Twist):
        """
        订阅/cmd_vel话题，获取线性速度和角速度，计算并发布四个电机的速度。
        :param msg: 订阅的Twist消息
        """
        linear_x = msg.linear.x  # 线性速度（前后）
        linear_y = msg.linear.y  # 线性速度（左右）
        # 应用角速度缩放因子，降低实际旋转速度
        # 原始代码: angular_z = msg.angular.z
        angular_z = msg.angular.z * self.ANGULAR_SPEED_SCALE

        # 计算四个电机的速度
        motor_msg = self.set_velocity(linear_x, linear_y, angular_z)

        # 发布电机速度
        self.publisher.publish(motor_msg)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        根据线性速度和角速度计算每个电机的控制速度
        :param linear_x: x方向线性速度（m/s）
        :param linear_y: y方向线性速度（m/s）
        :param angular_z: 角速度（rad/s）
        :return: MotorsSpeedControl 消息，包含四个电机的速度信息
        """
        # 计算四个电机的速度
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

        # 映射电机速度到 -100 到 100 范围
        motor_speeds = [self.speed_convert(v, self.max_linear_speed) for v in [-motor1, motor3, -motor2, motor4]]

        msg = MotorsSpeedControl()
        msg.data = []
        for i in range(4):
            motor_msg = MotorSpeedControl()
            motor_msg.id = i + 1  # 电机编号
            motor_msg.speed = float(motor_speeds[i])  # 电机速度
            msg.data.append(motor_msg)

        return msg
    # def set_velocity(self, linear_x, linear_y, angular_z):
    #     """
    #     根据线性速度和角速度计算每个电机的控制速度
    #     :param linear_x: x方向线性速度（m/s）
    #     :param linear_y: y方向线性速度（m/s）
    #     :param angular_z: 角速度（rad/s）
    #     :return: MotorsSpeedControl 消息，包含四个电机的速度信息
    #     """
    #     motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
    #     motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
    #     motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
    #     motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

    #     # 使用调整过的speed_convert
    #     motor_speeds = [self.speed_convert(v, self.max_linear_speed, angular_speed_factor=10.0) for v in [-motor1, motor2, -motor3, motor4]]

    #     msg = MotorsSpeedControl()
    #     msg.data = []

    #     for i in range(4):
    #         motor_msg = MotorSpeedControl()
    #         motor_msg.id = i + 1  # 电机编号
    #         motor_msg.speed = float(motor_speeds[i])  # 电机速度
    #         msg.data.append(motor_msg)

    #     return msg

def main(args=None):
    # 初始化rclpy库
    rclpy.init(args=args)

    # 创建MecanumChassis节点
    mecanum_chassis = MecanumChassis()

    # 保持节点运行
    rclpy.spin(mecanum_chassis)

    # 销毁节点
    mecanum_chassis.destroy_node()

    # 关闭rclpy库
    rclpy.shutdown()


if __name__ == '__main__':
    main()
