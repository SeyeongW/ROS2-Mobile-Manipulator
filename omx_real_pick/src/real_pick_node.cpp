// real_pick_node.cpp (FULL, 안정화 버전)
// 핵심 변경점:
// 1) HOME -> gripper open -> WAIT
// 2) marker TF로 pregrasp/final 위치는 만들되
// 3) OM-X에서 IK가 잘 풀리도록 "orientation은 강제하지 않음"
//    - GoalOrientationTolerance = 3.14 (사실상 무시)
//    - pregrasp/final orientation = 현재 end-effector orientation 유지
// 4) PREGRASP는 setPoseTarget() 대신 setPositionTarget() (position-only)
// 5) APPROACH는 computeCartesianPath로 final 위치까지 직선 접근 (orientation 유지)
// 6) Z는 camera-on-EE 구조에서 튀기 쉬움 -> z_work로 고정 (권장)

// 필요 헤더
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

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
  PREGRASP,
  APPROACH,
  GRASP,
  LIFT_HOME
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("real_pick_node");

  // Spin background
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&exec](){ exec->spin(); });

  // ---------------- TF ----------------
  auto tf_buffer  = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // --------------- MoveIt -------------
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setPlanningTime(5.0);

  // ✅ Position 중심. Orientation은 강제하지 않음(OM-X IK 안정)
  arm.setGoalPositionTolerance(0.01);     // 1 cm
  arm.setGoalOrientationTolerance(3.14);  // ✅ 사실상 무시

  // -------------- Gripper -------------
  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  // ---------- SETTINGS ----------
  const std::string base_frame    = "link1";
  const std::string markers_topic = "/aruco/markers";

  const double GRIP_OPEN  = 0.019;
  const double GRIP_CLOSE = -0.001;

  // TF freshness + stabilization
  const double max_tf_age_sec = 1.0;
  const int stable_need = 5;
  int stable_count = 0;

  // Marker selection bounds (from /aruco/markers camera-depth)
  const double marker_z_min = 0.05;
  const double marker_z_max = 2.00;

  // Approach tuning
  const double pregrasp_dist = 0.12;   // 12 cm away (normal direction)
  const double grasp_dist    = 0.03;   // 3 cm away
  const double lift_dist     = 0.10;

  // Cartesian
  const double cart_step     = 0.01;   // 1 cm step
  const double cart_jump_th  = 0.0;
  const double min_cart_frac = 0.85;

  // "Reached grasp pose?" 판단
  const double grasp_pos_tol = 0.02;   // 2 cm

  // Failure handling
  int fail_count = 0;
  const int max_fail = 3;

  // ---- Workspace clamp (IMPORTANT) ----
  // camera-on-EE 구조에서는 marker(base).z가 작업 평면 높이와 무관하게 튈 수 있음.
  // 그래서 z를 고정(z_work) 또는 clamp.
  const double z_min = 0.05;
  const double z_max = 0.35;
  const double z_work = 0.22;          // ✅ 네 환경에 맞게 튜닝(테이블 위 높이)
  const bool   use_fixed_z = true;     // ✅ true 추천
  const double xy_max = 0.30;

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

  auto getFreshMarkerTF = [&](const std::string& target_marker_frame,
                              geometry_msgs::msg::TransformStamped& out) -> bool
  {
    try {
      if (!tf_buffer->canTransform(base_frame, target_marker_frame, tf2::TimePointZero, 50ms))
        return false;

      auto t = tf_buffer->lookupTransform(base_frame, target_marker_frame, tf2::TimePointZero);

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

  auto tfToPose = [&](const geometry_msgs::msg::TransformStamped& t) -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose p;
    p.position.x = t.transform.translation.x;
    p.position.y = t.transform.translation.y;
    p.position.z = t.transform.translation.z;
    p.orientation = t.transform.rotation;
    return p;
  };

  // marker frame z-axis in base frame: R(q_marker) * (0,0,1)
  auto markerZAxisInBase = [&](const geometry_msgs::msg::Pose& marker_pose) -> tf2::Vector3 {
    tf2::Quaternion q;
    tf2::fromMsg(marker_pose.orientation, q);
    tf2::Matrix3x3 R(q);
    return R * tf2::Vector3(0,0,1);
  };

  // ---------- FSM state ----------
  FSM state = FSM::HOME_INIT;

  int target_id = -1;
  std::string target_frame;

  geometry_msgs::msg::Pose pregrasp_pose;
  geometry_msgs::msg::Pose final_pose;

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    switch (state) {

      case FSM::HOME_INIT: {
        RCLCPP_INFO(node->get_logger(), ">> HOME_INIT: moving to 'home' and opening gripper");
        arm.setNamedTarget("home");
        arm.move();
        rclcpp::sleep_for(500ms);

        operateGripper(node, gripper, GRIP_OPEN);

        stable_count = 0;
        fail_count = 0;
        target_id = -1;
        target_frame.clear();

        state = FSM::WAIT_MARKER;
        break;
      }

      case FSM::WAIT_MARKER: {
        const int best = selectClosestMarkerId();
        if (best < 0) {
          stable_count = 0;
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAIT_MARKER: no marker yet");
          break;
        }

        if (best != target_id) {
          target_id = best;
          target_frame = "aruco_marker_" + std::to_string(target_id);
          stable_count = 0;
          RCLCPP_INFO(node->get_logger(), ">> Target marker set to ID=%d (%s)",
                      target_id, target_frame.c_str());
        }

        geometry_msgs::msg::TransformStamped t_base_marker;
        if (!getFreshMarkerTF(target_frame, t_base_marker)) {
          stable_count = 0;
          break;
        }

        stable_count++;
        auto marker_pose = tfToPose(t_base_marker);

        RCLCPP_INFO(node->get_logger(), ">> STABLE %d/%d  marker(base)=(%.3f %.3f %.3f)",
                    stable_count, stable_need,
                    marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);

        if (stable_count < stable_need) break;

        // ✅ orientation은 현재 EE orientation 유지(OM-X IK 안정)
        auto ee_now = arm.getCurrentPose().pose;

        // 위치는 marker normal 기반으로 생성(접근 방향성 유지)
        tf2::Vector3 z_axis = markerZAxisInBase(marker_pose);

        // 접근이 반대로 가면 이 줄을 켜라:
        // z_axis = -z_axis;

        // Final pose
        final_pose = marker_pose;
        final_pose.orientation = ee_now.orientation;   // ✅ 유지
        final_pose.position.x += z_axis.x() * grasp_dist;
        final_pose.position.y += z_axis.y() * grasp_dist;
        final_pose.position.z += z_axis.z() * grasp_dist;

        // Pregrasp pose
        pregrasp_pose = marker_pose;
        pregrasp_pose.orientation = ee_now.orientation; // ✅ 유지
        pregrasp_pose.position.x += z_axis.x() * pregrasp_dist;
        pregrasp_pose.position.y += z_axis.y() * pregrasp_dist;
        pregrasp_pose.position.z += z_axis.z() * pregrasp_dist;

        // ---- Z FIX/CLAMP ----
        if (use_fixed_z) {
          pregrasp_pose.position.z = z_work;
          final_pose.position.z    = z_work;   // 필요하면 z_work - 0.02
        } else {
          pregrasp_pose.position.z = clamp(pregrasp_pose.position.z, z_min, z_max);
          final_pose.position.z    = clamp(final_pose.position.z,    z_min, z_max);
        }
        pregrasp_pose.position.z = clamp(pregrasp_pose.position.z, z_min, z_max);
        final_pose.position.z    = clamp(final_pose.position.z,    z_min, z_max);

        // XY clamp
        pregrasp_pose.position.x = clamp(pregrasp_pose.position.x, -xy_max, xy_max);
        pregrasp_pose.position.y = clamp(pregrasp_pose.position.y, -xy_max, xy_max);
        final_pose.position.x    = clamp(final_pose.position.x,    -xy_max, xy_max);
        final_pose.position.y    = clamp(final_pose.position.y,    -xy_max, xy_max);

        RCLCPP_INFO(node->get_logger(),
                    ">> Built poses(Z fixed=%s): pre(%.3f %.3f %.3f) final(%.3f %.3f %.3f)",
                    use_fixed_z ? "true" : "false",
                    pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z,
                    final_pose.position.x,  final_pose.position.y,  final_pose.position.z);

        state = FSM::PREGRASP;
        break;
      }

      case FSM::PREGRASP: {
        RCLCPP_INFO(node->get_logger(), ">> PREGRASP: planning to pregrasp (position-only)");
        arm.setStartStateToCurrentState();

        // ✅ pose target 대신 position-only target (IK 성공률 ↑)
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
        RCLCPP_INFO(node->get_logger(), ">> APPROACH: cartesian to final (orientation 유지)");
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(final_pose);

        moveit_msgs::msg::RobotTrajectory traj;
        arm.setStartStateToCurrentState();
        const double frac = arm.computeCartesianPath(waypoints, cart_step, cart_jump_th, traj);

        RCLCPP_INFO(node->get_logger(), ">> Cartesian fraction=%.2f", frac);

        if (frac < min_cart_frac) {
          fail_count++;
          RCLCPP_ERROR(node->get_logger(), "Cartesian path too low (%.2f) (%d/%d)", frac, fail_count, max_fail);
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

        auto ee = arm.getCurrentPose().pose;
        const double err = dist3(ee.position, final_pose.position);
        RCLCPP_INFO(node->get_logger(), ">> EE position error to final: %.3f m", err);

        if (err <= grasp_pos_tol) {
          state = FSM::GRASP;
        } else {
          RCLCPP_WARN(node->get_logger(), ">> Not close enough to grasp. Back to WAIT_MARKER");
          state = FSM::WAIT_MARKER;
        }
        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASP: closing gripper");
        operateGripper(node, gripper, GRIP_CLOSE);
        state = FSM::LIFT_HOME;
        break;
      }

      case FSM::LIFT_HOME: {
        RCLCPP_INFO(node->get_logger(), ">> LIFT_HOME: lifting and returning home");

        auto ee = arm.getCurrentPose().pose;
        geometry_msgs::msg::Pose lift_pose = ee;
        lift_pose.position.z = clamp(lift_pose.position.z + lift_dist, z_min, z_max);

        arm.setStartStateToCurrentState();
        arm.setPoseTarget(lift_pose);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        stable_count = 0;
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