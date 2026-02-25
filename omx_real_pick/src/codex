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
#include <future>

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

// [ADDED] xyz 기반 거리
static double distXYZ(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

static bool operateGripper(rclcpp::Node::SharedPtr node,
                           rclcpp_action::Client<GripperCommand>::SharedPtr client,
                           double pos,
                           double effort = 0.5,
                           std::chrono::milliseconds send_timeout = 3000ms,
                           std::chrono::milliseconds result_timeout = 5000ms)
{
  if (!client->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = effort;

  auto send_fut = client->async_send_goal(goal);

  if (send_fut.wait_for(send_timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Gripper send_goal timeout");
    return false;
  }

  auto gh = send_fut.get();
  if (!gh) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
    return false;
  }

  auto res_fut = client->async_get_result(gh);
  if (res_fut.wait_for(result_timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Gripper result timeout");
    return false;
  }

  auto res = res_fut.get();
  if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper failed (code=%d)", (int)res.code);
    return false;
  }
  return true;
}

enum class FSM {
  HOME_INIT,
  WAIT_MARKER,
  CENTER_ON_MARKER,
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

  auto tf_buffer  = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");

  const std::string base_frame    = "link1";
  const std::string camera_frame  = "camera_link";
  const std::string markers_topic = "/aruco/markers";

  arm.setPoseReferenceFrame(base_frame);

  arm.setMaxVelocityScalingFactor(0.20);
  arm.setMaxAccelerationScalingFactor(0.20);
  arm.setPlanningTime(3.0);

  arm.setGoalPositionTolerance(0.01);
  arm.setGoalOrientationTolerance(3.14);

  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  const double GRIP_OPEN  = 0.019;
  const double GRIP_CLOSE = 0.0;

  const double max_tf_age_sec = 1.0;

  const double marker_z_min = 0.05;
  const double marker_z_max = 2.00;

  const double K_yaw   = 0.6;
  const double K_pitch = 0.6;
  const double max_step_rad = 0.12;

  // Runtime-tunable control direction signs.
  // Use ros2 params to flip when camera/joint axis conventions differ.
  node->declare_parameter<int>("yaw_cmd_sign", -1);
  node->declare_parameter<int>("pitch_cmd_sign", +1);

  const double yaw_tol_rad   = 3.0 * M_PI / 180.0;
  const double pitch_tol_rad = 3.0 * M_PI / 180.0;
  const int center_need = 5;
  int center_count = 0;

  const double j1_min = -M_PI, j1_max = M_PI;
  const double j2_min = -1.5,  j2_max = 1.5;

  const double pregrasp_dx = 0.12;
  const double final_dx    = 0.03;
  const double lift_dist   = 0.10;

  const double cart_step     = 0.01;
  const double cart_jump_th  = 0.0;
  const double min_cart_frac = 0.85;

  const double grasp_pos_tol = 0.03;

  const double z_min  = 0.05;
  const double z_max  = 0.35;
  const double z_work = 0.22;
  const double xy_max = 0.30;

  int fail_count = 0;
  const int max_fail = 3;

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


  FSM state = FSM::HOME_INIT;

  int target_id = -1;
  std::string target_frame;

  geometry_msgs::msg::Pose pregrasp_pose;
  geometry_msgs::msg::Pose final_pose;

  bool opened_on_detect = false;

  double latch_x = 0.0, latch_y = 0.0;
  bool have_latch = false;

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    switch (state) {

      case FSM::HOME_INIT: {
        RCLCPP_INFO(node->get_logger(), ">> HOME_INIT: go 'home', gripper open");
        arm.setNamedTarget("home");
        arm.move();
        rclcpp::sleep_for(500ms);

        operateGripper(node, gripper, GRIP_OPEN, 5.0);

        fail_count = 0;
        center_count = 0;
        target_id = -1;
        target_frame.clear();
        opened_on_detect = false;
        have_latch = false;

        state = FSM::WAIT_MARKER;
        break;
      }

      case FSM::WAIT_MARKER: {
        const int best = selectClosestMarkerId();
        if (best < 0) {
          center_count = 0;
          opened_on_detect = false;
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAIT_MARKER: no marker yet");
          break;
        }

        if (best != target_id) {
          target_id = best;
          target_frame = "aruco_marker_" + std::to_string(target_id);
          center_count = 0;
          opened_on_detect = false;
          have_latch = false;
          RCLCPP_INFO(node->get_logger(), ">> Target marker set to ID=%d (%s)",
                      target_id, target_frame.c_str());
        }

        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          center_count = 0;
          break;
        }

        // [ADDED] camera 기준 마커 xyz + 거리 로그
        {
          const double mx = t_cam_marker.transform.translation.x;
          const double my = t_cam_marker.transform.translation.y;
          const double mz = t_cam_marker.transform.translation.z;
          const double md = distXYZ(mx, my, mz);
          RCLCPP_INFO(node->get_logger(),
                      ">> MARKER (camera->%s): xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                      target_frame.c_str(), mx, my, mz, md);
        }

        if (!opened_on_detect) {
          RCLCPP_INFO(node->get_logger(), ">> Marker detected! Open gripper now.");
          operateGripper(node, gripper, GRIP_OPEN, 5.0);
          opened_on_detect = true;
        }

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
          state = FSM::WAIT_MARKER;
          break;
        }

        const double x = t_cam_marker.transform.translation.x;
        const double y = t_cam_marker.transform.translation.y;
        const double z = t_cam_marker.transform.translation.z;

        // [ADDED] camera 기준 마커 xyz + 거리 로그
        const double d = distXYZ(x, y, z);
        RCLCPP_INFO(node->get_logger(),
                    ">> MARKER (camera->%s): xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                    target_frame.c_str(), x, y, z, d);

        const double yaw_err   = std::atan2(x, z);
        const double pitch_err = std::atan2(-y, z);

        RCLCPP_INFO(node->get_logger(),
                    ">> CENTER: yaw_err=%.2fdeg pitch_err=%.2fdeg (x=%.3f y=%.3f z=%.3f)",
                    yaw_err * 180.0 / M_PI, pitch_err * 180.0 / M_PI, x, y, z);

        if (std::abs(yaw_err) < yaw_tol_rad && std::abs(pitch_err) < pitch_tol_rad) {
          center_count++;
          RCLCPP_INFO(node->get_logger(), ">> CENTER OK %d/%d", center_count, center_need);
        } else {
          center_count = 0;
        }

        if (center_count >= center_need) {
          geometry_msgs::msg::TransformStamped t_base_marker;
          if (!getFreshTF(base_frame, target_frame, t_base_marker)) {
            center_count = 0;
            state = FSM::WAIT_MARKER;
            break;
          }

          // [ADDED] base(link1) 기준 마커 xyz + 거리 로그
          {
            const double bx = t_base_marker.transform.translation.x;
            const double by = t_base_marker.transform.translation.y;
            const double bz = t_base_marker.transform.translation.z;
            const double bd = distXYZ(bx, by, bz);
            RCLCPP_INFO(node->get_logger(),
                        ">> MARKER (base[%s]->%s): xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                        base_frame.c_str(), target_frame.c_str(), bx, by, bz, bd);
          }

          latch_x = t_base_marker.transform.translation.x;
          latch_y = t_base_marker.transform.translation.y;
          have_latch = true;

          pregrasp_pose.position.x = clamp(latch_x, -xy_max, xy_max);
          pregrasp_pose.position.y = clamp(latch_y, -xy_max, xy_max);
          pregrasp_pose.position.z = clamp(z_work,  z_min, z_max);

          final_pose.position.x = clamp(latch_x, -xy_max, xy_max);
          final_pose.position.y = clamp(latch_y, -xy_max, xy_max);
          final_pose.position.z = clamp(z_work,  z_min, z_max);

          pregrasp_pose.position.x = clamp(latch_x + pregrasp_dx, -xy_max, xy_max);
          final_pose.position.x    = clamp(latch_x + final_dx,    -xy_max, xy_max);

          auto ee_now = arm.getCurrentPose().pose;
          pregrasp_pose.orientation = ee_now.orientation;
          final_pose.orientation    = ee_now.orientation;

          RCLCPP_INFO(node->get_logger(),
                      ">> CENTER DONE (latched). pre=(%.3f %.3f %.3f) final=(%.3f %.3f %.3f)",
                      pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z,
                      final_pose.position.x, final_pose.position.y, final_pose.position.z);

          state = FSM::PREGRASP;
          break;
        }

        std::vector<double> joints = arm.getCurrentJointValues();
        if (joints.size() < 2) {
          state = FSM::WAIT_MARKER;
          break;
        }

        int yaw_cmd_sign = -1;
        int pitch_cmd_sign = +1;
        node->get_parameter("yaw_cmd_sign", yaw_cmd_sign);
        node->get_parameter("pitch_cmd_sign", pitch_cmd_sign);

        yaw_cmd_sign = (yaw_cmd_sign >= 0) ? 1 : -1;
        pitch_cmd_sign = (pitch_cmd_sign >= 0) ? 1 : -1;

        double dyaw   = clamp(yaw_cmd_sign * K_yaw * yaw_err,   -max_step_rad, max_step_rad);
        double dpitch = clamp(pitch_cmd_sign * K_pitch * pitch_err, -max_step_rad, max_step_rad);

        RCLCPP_DEBUG(node->get_logger(),
                     ">> CTRL: yaw_cmd_sign=%d pitch_cmd_sign=%d dyaw=%.4f dpitch=%.4f",
                     yaw_cmd_sign, pitch_cmd_sign, dyaw, dpitch);

        joints[0] = clamp(joints[0] + dyaw,   j1_min, j1_max);
        joints[1] = clamp(joints[1] + dpitch, j2_min, j2_max);

        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);

        auto res = arm.move();
        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
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
          rclcpp::sleep_for(800ms);
          auto ee_now = arm.getCurrentPose().pose;
          double err_now = dist3(ee_now.position, final_pose.position);
          if (err_now <= grasp_pos_tol) {
            RCLCPP_WARN(node->get_logger(),
                        ">> Execute reported failure but EE is close enough (err=%.3f). Treat as success.",
                        err_now);
          } else {
            fail_count++;
            RCLCPP_ERROR(node->get_logger(), "Cartesian execute failed (%d/%d)", fail_count, max_fail);
            if (fail_count >= max_fail) state = FSM::HOME_INIT;
            else state = FSM::WAIT_MARKER;
            break;
          }
        }

        auto ee = arm.getCurrentPose().pose;
        const double err = dist3(ee.position, final_pose.position);
        RCLCPP_INFO(node->get_logger(), ">> EE pos err=%.3f m", err);

        if (err <= grasp_pos_tol) state = FSM::GRASP;
        else state = FSM::WAIT_MARKER;

        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASP: close gripper NOW");
        operateGripper(node, gripper, GRIP_CLOSE, 10.0);
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
        opened_on_detect = false;
        have_latch = false;
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
