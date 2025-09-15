from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- 参数声明部分 (保持不变) ---
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value='imu_link')
    car_type_arg = DeclareLaunchArgument('car_type', default_value='mecanum')

    # --- ros_robot_controller 节点 (最终修正版) ---
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
        parameters=[{'imu_frame': LaunchConfiguration('imu_frame')}],
        # --- 这是最关键的修正 (1/2) ---
        # 告诉这个节点，把它对 '~/set_motor_speeds' 的订阅
        # 重定向到我们实际发布指令的 '/motors_speed' 话题
        remappings=[
            ('~/set_motor_speeds', '/motors_speed')
        ]
    )

    # --- mecanum_chassis 节点 (保持不变) ---
    mecanum_chassis_node = Node(
        package='controller',
        executable='mecanum',
        name='mecanum_chassis_node',
        output='screen',
        parameters=[{
            'car_type': LaunchConfiguration('car_type')
        }],
        # --- 这是最关键的修正 (2/2) ---
        # 这里的重映射是确保 mecanum 节点的输出也是正确的，以防万一
        remappings=[
            ('/ros_robot_controller/set_motor_speeds', '/motors_speed')
        ]
    )

    # --- 返回完整的启动描述 ---
    return LaunchDescription([
        imu_frame_arg,
        car_type_arg,
        ros_robot_controller_node,
        mecanum_chassis_node
    ])