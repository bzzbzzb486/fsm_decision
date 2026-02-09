from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的路径
    pkg_share = get_package_share_directory('fsm_decision')
    
    # YAML 配置文件路径
    params_file = os.path.join(pkg_share, 'config', 'decision_params.yaml')
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file_config = LaunchConfiguration('params_file')
    
    # 决策节点
    fsm_decision_node = Node(
        package='fsm_decision',
        executable='fsm_decision_node',
        name='fsm_decision',
        output='screen',
        parameters=[
            params_file_config,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        fsm_decision_node,
    ])