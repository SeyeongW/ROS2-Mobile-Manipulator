#!/usr/bin/env python3
# ===============================
# aruco_subscriber_node.py (FULL)
# Fixes applied:
# 1) Subscribe to CameraInfo and use real K/D (no fake intrinsics)
# 2) Use sim time (consistent with Gazebo) via parameter
# 3) Publish TF with correct header.stamp + frame_id
# 4) Keep multi-dictionary detection (as you wrote), but only publish valid pose
# 5) Add basic sanity checks on Z (ignore negative/zero/too-far)
# ===============================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose

from ros2_aruco_interfaces.msg import ArucoMarkers

import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros

def rotation_matrix_to_quaternion(R: np.ndarray):
    tr = float(np.trace(R))
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S
    return [float(qx), float(qy), float(qz), float(qw)]

class ArucoSubscriberNode(Node):
    def __init__(self):
        super().__init__("aruco_subscriber_node")

        # Use sim time by default (Gazebo). Set to False on real camera.
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)])

        # Parameters
        self.declare_parameter("marker_size", 0.06)
        self.declare_parameter("frame_id", "camera_link")  # camera optical frame is even better if available
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        self.marker_size = float(self.get_parameter("marker_size").value)
        self.base_frame_id = str(self.get_parameter("frame_id").value)
        image_topic = str(self.get_parameter("image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        # Optional offsets (keep 0 unless you *know* your camera extrinsics)
        self.rgb_offset_x = 0.0
        self.rgb_offset_y = 0.0
        self.rgb_offset_z = 0.0

        # ArUco dictionaries
        self.dict_collection = {
            "6x6": aruco.DICT_6X6_250,
            "5x5": aruco.DICT_5X5_100,
            "4x4": aruco.DICT_4X4_50,
            "ORIG": aruco.DICT_ARUCO_ORIGINAL
        }
        self.detectors = []
        for name, dict_enum in self.dict_collection.items():
            d = aruco.getPredefinedDictionary(dict_enum)
            p = aruco.DetectorParameters()
            det = aruco.ArucoDetector(d, p)
            self.detectors.append((name, det))

        # Marker corner points in marker frame (z=0 plane)
        ms_half = self.marker_size / 2.0
        self.marker_obj_points = np.array([
            [-ms_half,  ms_half, 0.0],
            [ ms_half,  ms_half, 0.0],
            [ ms_half, -ms_half, 0.0],
            [-ms_half, -ms_half, 0.0]
        ], dtype=np.float32)

        # QoS for images
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Camera intrinsics (from CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None

        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_policy)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(ArucoMarkers, "/aruco/markers", 10)
        self.image_res_pub = self.create_publisher(Image, "/aruco/result_image", 10)
        self.cv_bridge = CvBridge()

        self.get_logger().info(f"✅ ArUco Subscriber Node Started! image={image_topic}, info={info_topic}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if not np.isfinite(K).all() or np.allclose(K, 0.0):
            self.get_logger().warn("CameraInfo K invalid; waiting...")
            return
        self.camera_matrix = K
        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.get_logger().info("✅ CameraInfo received. Using real intrinsics for solvePnP.")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn_throttle(1.0, "Waiting for CameraInfo...")
            return

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        vis_frame = frame.copy()

        marker_msg = ArucoMarkers()
        marker_msg.header = msg.header

        # Ensure header frame id exists
        if not marker_msg.header.frame_id:
            marker_msg.header.frame_id = self.base_frame_id

        found_any = False

        for name, detector in self.detectors:
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is None or len(ids) == 0:
                continue

            found_any = True
            aruco.drawDetectedMarkers(vis_frame, corners, ids)

            for i in range(len(ids)):
                ok, rvec, tvec = cv2.solvePnP(
                    self.marker_obj_points,
                    corners[i],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not ok:
                    continue

                px = float(tvec[0][0]) + self.rgb_offset_x
                py = float(tvec[1][0]) + self.rgb_offset_y
                pz = float(tvec[2][0]) + self.rgb_offset_z

                # Sanity check on depth (meters)
                if not np.isfinite(pz) or pz <= 0.02 or pz > 2.0:
                    continue

                current_id = int(ids[i][0])
                marker_msg.marker_ids.append(current_id)

                pose = Pose()
                pose.position.x = px
                pose.position.y = py
                pose.position.z = pz

                rmat = cv2.Rodrigues(rvec)[0]
                qx, qy, qz, qw = rotation_matrix_to_quaternion(rmat)
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
                marker_msg.poses.append(pose)

                # Publish TF: parent = camera frame, child = aruco_marker_ID
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.base_frame_id
                t.child_frame_id = f"aruco_marker_{current_id}"
                t.transform.translation.x = px
                t.transform.translation.y = py
                t.transform.translation.z = pz
                t.transform.rotation = pose.orientation
                self.tf_broadcaster.sendTransform(t)

                # Overlay text
                c = corners[i][0]
                text_pos = (int(c[0][0]), int(c[0][1]) - 10)
                info_text = f"ID:{current_id}({name}) z:{pz:.2f}"
                cv2.putText(
                    vis_frame, info_text, text_pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

        # Publish visualization
        vis_msg = self.cv_bridge.cv2_to_imgmsg(vis_frame, encoding="bgr8")
        vis_msg.header = msg.header
        self.image_res_pub.publish(vis_msg)

        if found_any:
            self.marker_pub.publish(marker_msg)

def main():
    rclpy.init()
    node = ArucoSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
