import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder   # ✅ 추가


def generate_launch_description():
    # -----------------------
    # Launch arguments
    # -----------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    marker_size = LaunchConfiguration("marker_size")
    camera_frame = LaunchConfiguration("camera_frame")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    # -----------------------
    # 0) Arguments 선언
    # -----------------------
    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("marker_size", default_value="0.06"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
    ]

    # -----------------------
    # 1) MoveIt Config Builder (🔥 핵심 추가)
    # -----------------------
    moveit_config = MoveItConfigsBuilder(
        "open_manipulator_x",
        package_name="open_manipulator_x_moveit_config"
    ).to_moveit_configs()

    # -----------------------
    # 2) MoveIt move_group
    # -----------------------
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("open_manipulator_x_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        )
    )

    # -----------------------
    # 3) RealSense
    # -----------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "align_depth.enable": "true",
        }.items(),
    )

    # -----------------------
    # 4) ArUco node
    # -----------------------
    aruco_node = Node(
        package="omx_real_pick",
        executable="aruco_realsense.py",
        name="aruco_subscriber_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "marker_size": marker_size,
            "frame_id": camera_frame,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
        }],
    )

    # -----------------------
    # 5) Pick node (🔥 MoveIt 파라미터 전달)
    # -----------------------
    pick_node = Node(
        package="omx_real_pick",
        executable="real_pick_node",
        name="real_pick_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,  # 🔥 이게 없어서 IK 경고 떴던 것
            {"use_sim_time": use_sim_time},
        ],
    )

    # -----------------------
    # 순서 보장용 Timer
    # -----------------------
    delayed_aruco = TimerAction(period=6.0, actions=[aruco_node])
    delayed_pick  = TimerAction(period=10.0, actions=[pick_node])

    return LaunchDescription(
        declare_args
        + [
            move_group_launch,
            realsense_launch,
            delayed_aruco,
            delayed_pick,
        ]
    )
