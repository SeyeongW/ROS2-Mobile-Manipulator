import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. MoveIt 설정
    moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="open_manipulator_x_moveit_config").to_moveit_configs()

    # 2. RealSense 실행 (Alignment 켬)
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'align_depth.enable': 'true'}.items()
    )

    # 4. 노드들
    aruco_node = Node(package="omx_real_pick", executable="aruco_realsense.py")
    
    control_node = Node(
        package="omx_real_pick", 
        executable="real_pick_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False}
        ]
    )

    return LaunchDescription([
        rs_launch,
        TimerAction(period=5.0, actions=[aruco_node]),
        TimerAction(period=8.0, actions=[control_node])
    ])
    
