import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    # 3) RealSense (align depth enable)
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
    #
    # ⚠️ 중요:
    # Node(...)는 "설치된 실행파일"만 실행 가능.
    # 즉, omx_real_pick 패키지에 aruco_realsense.py가
    # console_scripts(또는 install/lib 아래 실행파일)로 존재해야 함.
    # -----------------------
    aruco_node = Node(
        package="omx_real_pick",
        executable="aruco_realsense.py",   # 너 패키지에서 실제 실행 가능한 이름이어야 함
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
    # 5) Pick control node (MoveIt interface)
    # -----------------------
    pick_node = Node(
        package="omx_real_pick",
        executable="real_pick_node",  # C++ 노드 실행파일
        name="real_pick_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )

    # -----------------------
    # 순서 보장용 Timer
    # (하드웨어/MoveIt/카메라가 먼저 안정화된 뒤 aruco/pick 실행)
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
