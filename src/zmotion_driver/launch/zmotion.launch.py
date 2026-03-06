from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取msgs_bridge包的路径
    msgs_bridge_share_dir = get_package_share_directory('msgs_bridge')
    # 构建msgs_converter.launch.py的路径
    converter_launch_path = os.path.join(msgs_bridge_share_dir, 'launch', 'msgs_converter.launch.py')
    
    # 获取zmotion_driver包的路径
    zmotion_driver_share_dir = get_package_share_directory('zmotion_driver')
    # 构建参数文件路径
    params_file = os.path.join(zmotion_driver_share_dir, 'config', 'zmotion_params.yaml')
    
    # 包含msgs_converter.launch.py
    include_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(converter_launch_path)
    )
    
    # 启动zmotion_node
    zmotion_node = Node(
        package='zmotion_driver',
        executable='zmotion_node',
        name='zmotion_controller',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        include_converter,
        zmotion_node
    ])