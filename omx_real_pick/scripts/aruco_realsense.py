#!/usr/bin/env python3
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
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return [float(qx), float(qy), float(qz), float(qw)]


class ArucoSubscriberNode(Node):
    def __init__(self):
        super().__init__("aruco_subscriber_node")

        # Params
        self.declare_parameter("marker_size", 0.06)
        self.declare_parameter("frame_id", "camera_color_optical_frame")  # 비워두면 msg.header.frame_id 사용
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        # ✅ depth (aligned)
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("depth_unit_scale", 0.001)  # 16UC1(mm) -> m
        self.declare_parameter("depth_median_ksize", 3)    # 3 or 5 권장

        self.marker_size = float(self.get_parameter("marker_size").value)
        self.base_frame_id = str(self.get_parameter("frame_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)
        self.depth_median_ksize = int(self.get_parameter("depth_median_ksize").value)

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

        # Marker object points
        ms_half = self.marker_size / 2.0
        self.marker_obj_points = np.array([
            [-ms_half,  ms_half, 0.0],
            [ ms_half,  ms_half, 0.0],
            [ ms_half, -ms_half, 0.0],
            [-ms_half, -ms_half, 0.0]
        ], dtype=np.float32)

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_matrix = None
        self.dist_coeffs = None

        self.latest_depth = None
        self.latest_depth_stamp = None

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, qos_img)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_img)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(ArucoMarkers, "/aruco/markers", 10)
        self.image_res_pub = self.create_publisher(Image, "/aruco/result_image", 10)
        self.cv_bridge = CvBridge()

        self.get_logger().info(
            f"✅ ArUco+Depth Node Started!\n"
            f"  color={self.image_topic}\n"
            f"  info ={self.info_topic}\n"
            f"  depth={self.depth_topic}\n"
            f"  frame_id(param)='{self.base_frame_id}' (empty => use msg.header.frame_id)"
        )

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
        self.get_logger().info("✅ CameraInfo received. Using real intrinsics.")

    def depth_callback(self, msg: Image):
        # depth는 16UC1(mm)인 경우가 일반적
        try:
            # encoding을 강제하지 않고 그대로 받기 (16UC1)
            depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth is None:
                return
            self.latest_depth = depth
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def _depth_median_at(self, u: int, v: int):
        if self.latest_depth is None:
            return None

        h, w = self.latest_depth.shape[:2]
        k = self.depth_median_ksize
        if k < 1:
            k = 1
        if k % 2 == 0:
            k += 1
        r = k // 2

        u0 = max(0, u - r)
        u1 = min(w - 1, u + r)
        v0 = max(0, v - r)
        v1 = min(h - 1, v + r)

        roi = self.latest_depth[v0:v1+1, u0:u1+1].astype(np.float32)

        # 0(무효), NaN 제거
        roi = roi[np.isfinite(roi)]
        roi = roi[roi > 0.0]
        if roi.size == 0:
            return None

        d_raw = float(np.median(roi))
        d_m = d_raw * self.depth_unit_scale
        return d_m

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

        # parent frame: param이 비었으면 이미지 frame_id 사용
        parent_frame = self.base_frame_id.strip() if self.base_frame_id.strip() else msg.header.frame_id
        if not parent_frame:
            parent_frame = "camera_link"

        found_any = False

        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])

        for name, detector in self.detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                continue

            found_any = True
            aruco.drawDetectedMarkers(vis_frame, corners, ids)

            for i in range(len(ids)):
                current_id = int(ids[i][0])

                # 마커 중심 픽셀 (u,v)
                c = corners[i][0]  # (4,2)
                u = int(np.mean(c[:, 0]))
                v = int(np.mean(c[:, 1]))

                # 1) rotation은 solvePnP로 구함
                ok, rvec, tvec = cv2.solvePnP(
                    self.marker_obj_points,
                    corners[i],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not ok:
                    continue

                rmat = cv2.Rodrigues(rvec)[0]
                qx, qy, qz, qw = rotation_matrix_to_quaternion(rmat)

                # 2) translation은 depth 기반으로 (X,Y,Z)
                d_m = self._depth_median_at(u, v)
                if d_m is None or not np.isfinite(d_m) or d_m <= 0.02 or d_m > 2.0:
                    # depth가 없으면 solvePnP translation fallback
                    px = float(tvec[0][0])
                    py = float(tvec[1][0])
                    pz = float(tvec[2][0])
                    use_depth = False
                else:
                    # deproject pixel -> camera XYZ
                    px = (u - cx) / fx * d_m
                    py = (v - cy) / fy * d_m
                    pz = d_m
                    use_depth = True

                # publish /aruco/markers
                marker_msg.marker_ids.append(current_id)

                pose = Pose()
                pose.position.x = float(px)
                pose.position.y = float(py)
                pose.position.z = float(pz)
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
                marker_msg.poses.append(pose)

                # publish TF
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = parent_frame
                t.child_frame_id = f"aruco_marker_{current_id}"
                t.transform.translation.x = float(px)
                t.transform.translation.y = float(py)
                t.transform.translation.z = float(pz)
                t.transform.rotation = pose.orientation
                self.tf_broadcaster.sendTransform(t)

                # overlay
                text_pos = (int(c[0][0]), int(c[0][1]) - 10)
                src = "D" if use_depth else "PnP"
                info_text = f"ID:{current_id}({name}) {src} z:{pz:.2f}"
                cv2.putText(vis_frame, info_text, text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # result image publish
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