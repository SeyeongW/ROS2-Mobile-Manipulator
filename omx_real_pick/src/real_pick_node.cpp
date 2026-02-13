#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using ArucoMarkers   = ros2_aruco_interfaces::msg::ArucoMarkers;

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static bool isFinite(double v) { return std::isfinite(v); }

static double dist3(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static void operateGripper(rclcpp::Node::SharedPtr node,
                           rclcpp_action::Client<GripperCommand>::SharedPtr client,
                           double pos)
{
  if (!client->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return;
  }
  auto goal = GripperCommand::Goal();
  goal.command.position = pos;
  goal.command.max_effort = 0.5;
  client->async_send_goal(goal);
  rclcpp::sleep_for(800ms);
}

enum class FSM {
  HOME_INIT,
  WAIT_MARKER,
  CENTER_ON_MARKER,  // ✅ 화면 중앙 정렬(visual servo)
  PREGRASP,
  APPROACH,
  GRASP,
  LIFT_HOME
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("real_pick_node");

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&exec](){ exec->spin(); });

  // ---------------- TF ----------------
  auto tf_buffer  = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // --------------- MoveIt -------------
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.20);
  arm.setMaxAccelerationScalingFactor(0.20);
  arm.setPlanningTime(3.0);

  // Position 중심 (orientation 강제 X)
  arm.setGoalPositionTolerance(0.01);
  arm.setGoalOrientationTolerance(3.14);

  // -------------- Gripper -------------
  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  // ---------- SETTINGS ----------
  const std::string base_frame    = "link1";
  const std::string camera_frame  = "camera_link";     // ✅ aruco 노드 parent frame
  const std::string markers_topic = "/aruco/markers";

  const double GRIP_OPEN  = 0.019;
  const double GRIP_CLOSE = -0.001;

  // TF freshness + stabilization
  const double max_tf_age_sec = 1.0;

  // Marker selection bounds (from /aruco/markers camera-depth)
  const double marker_z_min = 0.05;
  const double marker_z_max = 2.00;

  // Centering (visual servo) gains
  const double K_yaw   = 0.8;   // joint1 보정 gain
  const double K_pitch = 0.8;   // joint2 보정 gain
  const double max_step_rad = 0.20; // 한 번에 최대 보정(라디안) 제한

  // Centering threshold
  const double yaw_tol_rad   = 3.0 * M_PI / 180.0;   // 3 deg
  const double pitch_tol_rad = 3.0 * M_PI / 180.0;   // 3 deg
  const int center_need = 5;
  int center_count = 0;

  // Joint limits (대략 OM-X 기본 제한에 맞춘 보수값; 필요시 조절)
  const double j1_min = -M_PI, j1_max = M_PI;
  const double j2_min = -1.5,  j2_max = 1.5;

  // Grasp pipeline tuning
  const double pregrasp_dist = 0.12;
  const double grasp_dist    = 0.03;
  const double lift_dist     = 0.10;

  // Cartesian
  const double cart_step     = 0.01;
  const double cart_jump_th  = 0.0;
  const double min_cart_frac = 0.85;

  // Reached
  const double grasp_pos_tol = 0.02;

  // Workspace: Z 고정 (카메라 EE 장착 구조 안정화)
  const double z_min = 0.05;
  const double z_max = 0.35;
  const double z_work = 0.22;
  const double xy_max = 0.30;

  // Failure handling
  int fail_count = 0;
  const int max_fail = 3;

  // ---------- /aruco/markers cache ----------
  std::mutex mk_mtx;
  ArucoMarkers latest;
  bool have_markers = false;

  auto sub = node->create_subscription<ArucoMarkers>(
    markers_topic, 10,
    [&](const ArucoMarkers::SharedPtr msg){
      std::lock_guard<std::mutex> lk(mk_mtx);
      latest = *msg;
      have_markers = true;
    }
  );

  auto selectClosestMarkerId = [&]() -> int {
    std::lock_guard<std::mutex> lk(mk_mtx);
    if (!have_markers) return -1;
    if (latest.marker_ids.size() != latest.poses.size()) return -1;
    if (latest.marker_ids.empty()) return -1;

    int best_id = -1;
    double best_z = 1e9;

    for (size_t i = 0; i < latest.marker_ids.size(); i++) {
      const double z = latest.poses[i].position.z;
      if (!isFinite(z) || z < marker_z_min || z > marker_z_max) continue;
      if (z < best_z) {
        best_z = z;
        best_id = latest.marker_ids[i];
      }
    }
    return best_id;
  };

  auto getFreshTF = [&](const std::string& target, const std::string& source,
                        geometry_msgs::msg::TransformStamped& out) -> bool
  {
    // want: source -> target (lookupTransform(target_frame, source_frame))
    try {
      if (!tf_buffer->canTransform(target, source, tf2::TimePointZero, 50ms))
        return false;

      auto t = tf_buffer->lookupTransform(target, source, tf2::TimePointZero);

      rclcpp::Time now = node->get_clock()->now();
      rclcpp::Time stamp = rclcpp::Time(t.header.stamp);
      const double age = (now - stamp).seconds();
      if (age > max_tf_age_sec) return false;

      const auto &tr = t.transform.translation;
      if (!isFinite(tr.x) || !isFinite(tr.y) || !isFinite(tr.z)) return false;

      out = t;
      return true;
    } catch (...) {
      return false;
    }
  };

  // ---------- FSM ----------
  FSM state = FSM::HOME_INIT;

  int target_id = -1;
  std::string target_frame;

  geometry_msgs::msg::Pose pregrasp_pose;
  geometry_msgs::msg::Pose final_pose;

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    switch (state) {

      case FSM::HOME_INIT: {
        RCLCPP_INFO(node->get_logger(), ">> HOME_INIT: go 'home', gripper open");
        arm.setNamedTarget("home");
        arm.move();
        rclcpp::sleep_for(500ms);

        operateGripper(node, gripper, GRIP_OPEN);

        fail_count = 0;
        center_count = 0;
        target_id = -1;
        target_frame.clear();

        state = FSM::WAIT_MARKER;
        break;
      }

      case FSM::WAIT_MARKER: {
        const int best = selectClosestMarkerId();
        if (best < 0) {
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAIT_MARKER: no marker yet");
          break;
        }

        if (best != target_id) {
          target_id = best;
          target_frame = "aruco_marker_" + std::to_string(target_id);
          center_count = 0;
          RCLCPP_INFO(node->get_logger(), ">> Target marker set to ID=%d (%s)",
                      target_id, target_frame.c_str());
        }

        // marker가 camera 프레임에서라도 보이는지(신선한 TF)
        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          center_count = 0;
          break;
        }

        // 보이면 CENTER 단계로
        state = FSM::CENTER_ON_MARKER;
        break;
      }

      case FSM::CENTER_ON_MARKER: {
        if (target_frame.empty()) {
          state = FSM::WAIT_MARKER;
          break;
        }

        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          center_count = 0;
          RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                               ">> CENTER: marker TF not fresh");
          state = FSM::WAIT_MARKER;
          break;
        }

        const double x = t_cam_marker.transform.translation.x;
        const double y = t_cam_marker.transform.translation.y;
        const double z = t_cam_marker.transform.translation.z;

        // 화면 중심 각도 오차 (카메라 기준)
        const double yaw_err   = std::atan2(x, z);
        const double pitch_err = std::atan2(-y, z);

        RCLCPP_INFO(node->get_logger(),
                    ">> CENTER: yaw_err=%.2fdeg pitch_err=%.2fdeg (x=%.3f y=%.3f z=%.3f)",
                    yaw_err * 180.0 / M_PI, pitch_err * 180.0 / M_PI, x, y, z);

        // 목표 중심 도달 판정
        if (std::abs(yaw_err) < yaw_tol_rad && std::abs(pitch_err) < pitch_tol_rad) {
          center_count++;
          RCLCPP_INFO(node->get_logger(), ">> CENTER OK %d/%d", center_count, center_need);
        } else {
          center_count = 0;
        }

        if (center_count >= center_need) {
          // 이제 base->marker로 pregrasp/final 생성하고 그랩 시도
          geometry_msgs::msg::TransformStamped t_base_marker;
          if (!getFreshTF(base_frame, target_frame, t_base_marker)) {
            center_count = 0;
            state = FSM::WAIT_MARKER;
            break;
          }

          // 중심점(마커) 기반 pregrasp/final: x/y는 marker 기준, z는 작업높이 고정
          const double mx = t_base_marker.transform.translation.x;
          const double my = t_base_marker.transform.translation.y;

          // 간단히 base x축 방향으로 앞/뒤 오프셋 주는 대신,
          // 마커 법선 기반까지 하려면 orientation이 필요하지만 OM-X에선 IK가 빡세서
          // 일단 "센터링 후"에는 위치만으로 접근(안정).
          pregrasp_pose.position.x = clamp(mx + pregrasp_dist, -xy_max, xy_max);
          pregrasp_pose.position.y = clamp(my,              -xy_max, xy_max);
          pregrasp_pose.position.z = clamp(z_work, z_min, z_max);

          final_pose.position.x = clamp(mx + grasp_dist, -xy_max, xy_max);
          final_pose.position.y = clamp(my,            -xy_max, xy_max);
          final_pose.position.z = clamp(z_work, z_min, z_max);

          // orientation은 현재 EE 유지
          auto ee_now = arm.getCurrentPose().pose;
          pregrasp_pose.orientation = ee_now.orientation;
          final_pose.orientation    = ee_now.orientation;

          RCLCPP_INFO(node->get_logger(),
                      ">> CENTER DONE. pre=(%.3f %.3f %.3f) final=(%.3f %.3f %.3f)",
                      pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z,
                      final_pose.position.x, final_pose.position.y, final_pose.position.z);

          state = FSM::PREGRASP;
          break;
        }

        // 아직 중심이 아니면: joint1/joint2를 조금씩 보정
        std::vector<double> joints = arm.getCurrentJointValues();
        if (joints.size() < 2) {
          RCLCPP_ERROR(node->get_logger(), "Joint vector too small");
          state = FSM::WAIT_MARKER;
          break;
        }

        double dyaw   = clamp(-K_yaw   * yaw_err,   -max_step_rad, max_step_rad);
        double dpitch = clamp(K_pitch * pitch_err, -max_step_rad, max_step_rad);

        // joint1: yaw, joint2: pitch(어깨)
        joints[0] = clamp(joints[0] + dyaw,   j1_min, j1_max);
        joints[1] = clamp(joints[1] + dpitch, j2_min, j2_max);

        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);

        auto res = arm.move();
        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          RCLCPP_ERROR(node->get_logger(), "CENTER move failed (%d/%d)", fail_count, max_fail);
          if (fail_count >= max_fail) state = FSM::HOME_INIT;
          else state = FSM::WAIT_MARKER;
        } else {
          fail_count = 0;
        }
        break;
      }

      case FSM::PREGRASP: {
        RCLCPP_INFO(node->get_logger(), ">> PREGRASP: move to pregrasp (position-only)");
        arm.setStartStateToCurrentState();
        arm.setPositionTarget(pregrasp_pose.position.x,
                              pregrasp_pose.position.y,
                              pregrasp_pose.position.z);

        auto res = arm.move();
        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          RCLCPP_ERROR(node->get_logger(), "PREGRASP failed (%d/%d)", fail_count, max_fail);
          if (fail_count >= max_fail) state = FSM::HOME_INIT;
          else state = FSM::WAIT_MARKER;
          break;
        }

        fail_count = 0;
        state = FSM::APPROACH;
        break;
      }

      case FSM::APPROACH: {
        RCLCPP_INFO(node->get_logger(), ">> APPROACH: cartesian to final");
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(final_pose);

        moveit_msgs::msg::RobotTrajectory traj;
        arm.setStartStateToCurrentState();
        const double frac = arm.computeCartesianPath(waypoints, cart_step, cart_jump_th, traj);

        RCLCPP_INFO(node->get_logger(), ">> Cartesian fraction=%.2f", frac);

        if (frac < min_cart_frac) {
          fail_count++;
          RCLCPP_ERROR(node->get_logger(), "Cartesian too low (%.2f) (%d/%d)", frac, fail_count, max_fail);
          if (fail_count >= max_fail) state = FSM::HOME_INIT;
          else state = FSM::WAIT_MARKER;
          break;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;

        auto exec_res = arm.execute(plan);
        if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          RCLCPP_ERROR(node->get_logger(), "Cartesian execute failed (%d/%d)", fail_count, max_fail);
          if (fail_count >= max_fail) state = FSM::HOME_INIT;
          else state = FSM::WAIT_MARKER;
          break;
        }

        // grasp 위치 도달 판단(포지션 오차)
        auto ee = arm.getCurrentPose().pose;
        const double err = dist3(ee.position, final_pose.position);
        RCLCPP_INFO(node->get_logger(), ">> EE pos err=%.3f m", err);

        if (err <= grasp_pos_tol) state = FSM::GRASP;
        else state = FSM::WAIT_MARKER;

        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASP: close gripper");
        operateGripper(node, gripper, GRIP_CLOSE);
        state = FSM::LIFT_HOME;
        break;
      }

      case FSM::LIFT_HOME: {
        RCLCPP_INFO(node->get_logger(), ">> LIFT_HOME: lift and home");
        auto ee = arm.getCurrentPose().pose;

        geometry_msgs::msg::Pose lift_pose = ee;
        lift_pose.position.z = clamp(lift_pose.position.z + lift_dist, z_min, z_max);

        arm.setStartStateToCurrentState();
        arm.setPoseTarget(lift_pose);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        center_count = 0;
        fail_count = 0;
        state = FSM::WAIT_MARKER;
        break;
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
