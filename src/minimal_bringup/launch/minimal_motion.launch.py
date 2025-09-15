import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- NEW ---
    # 1. 声明一个名为 'car_type' 的启动参数，并设置默认值为 'mecanum'
    #    这是解决问题的关键
    car_type_arg = DeclareLaunchArgument(
        'car_type',
        default_value='mecanum',
        description='Type of the car chassis'
    )

    # 2. 获取 controller 功能包的路径
    controller_package_path = get_package_share_directory('controller')

    # 3. 包含并启动 controller 包中的 launch 文件
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
        # --- MODIFIED ---
        # 4. 将我们声明的 'car_type' 参数传递给 controller.launch.py
        launch_arguments={'car_type': LaunchConfiguration('car_type')}.items()
    )

    # 5. 返回 LaunchDescription 对象
    return LaunchDescription([
        car_type_arg,
        controller_launch
    ])