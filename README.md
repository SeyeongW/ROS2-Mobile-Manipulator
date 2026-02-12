# ROS2-Indoor-Mobile-Manipulator
Autonomous Indoor Mobile Manipulator based on ROS 2 Humble. Integrated with Nav2, MoveIt 2, and AI Perception (YOLO &amp; Audio SSL) for Pet Care Services. (ROS 2 Humble 기반의 자율주행 모바일 매니퓰레이터. 펫 케어 서비스를 위해 Nav2, MoveIt 2, AI 인식을 통합함.)

## Execution Steps

1. Launch Robot Controller (U2D2)
Open a new terminal and run the hardware driver.

```bash
ros2 launch open_manipulator_x_bringup hardware.launch.py
```
2. Launch MoveIt MoveGroup
Open a new terminal and run the MoveIt 2 path planning node.

```bash
ros2 launch open_manipulator_x_moveit_config move_group.launch.py
```
3. Run Application Node
Open a new terminal and run the main application (Camera + Tracking + Control).

```bash
ros2 launch omx_real_pick hardware.launch.py
```
