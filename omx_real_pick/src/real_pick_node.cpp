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

static double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static bool isFinite(double v) { return std::isfinite(v); }

static double distXYZ(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

static double stepToward(double cur, double tgt, double max_step) {
  const double d = tgt - cur;
  if (std::abs(d) <= max_step) return tgt;
  return cur + (d > 0 ? max_step : -max_step);
}

static bool operateGripper(
  rclcpp::Node * node,
  const rclcpp_action::Client<GripperCommand>::SharedPtr & client,
  double pos,
  double effort = 10.0,
  std::chrono::milliseconds server_wait = 2000ms,
  std::chrono::milliseconds goal_wait   = 8000ms,
  std::chrono::milliseconds result_wait = 12000ms
)
{
  if (!client->wait_for_action_server(server_wait)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = effort;

  rclcpp_action::Client<GripperCommand>::SendGoalOptions opt;
  opt.goal_response_callback =
    [node](const rclcpp_action::ClientGoalHandle<GripperCommand>::SharedPtr & gh) {
      if (!gh) RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected (goal_response_cb)");
      else     RCLCPP_INFO (node->get_logger(), "Gripper goal accepted");
    };

  auto fut_goal = client->async_send_goal(goal, opt);
  if (fut_goal.wait_for(goal_wait) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Gripper send_goal timeout");
    return false;
  }

  auto gh = fut_goal.get();
  if (!gh) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
    return false;
  }

  auto fut_res = client->async_get_result(gh);
  if (fut_res.wait_for(result_wait) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Gripper result timeout");
    return false;
  }

  auto wrapped = fut_res.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper failed (code=%d)", (int)wrapped.code);
    return false;
  }

  return true;
}

enum class FSM {
  HOME_INIT,
  WAIT_MARKER,
  CENTER_ON_MARKER,
  APPROACH_CONTINUOUS,
  GRASP,
  LIFT_HOME
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("real_pick_node", opts);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&exec](){ exec->spin(); });

  auto tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");

  const std::string base_frame    = node->declare_parameter<std::string>("base_frame", "link1");
  const std::string camera_frame  = node->declare_parameter<std::string>("camera_frame", "camera_link");
  const std::string markers_topic = node->declare_parameter<std::string>("markers_topic", "/aruco/markers");

  // [수정 1] pose reference frame을 base_frame으로 명시
  arm.setPoseReferenceFrame(base_frame);

  arm.setMaxVelocityScalingFactor(node->declare_parameter<double>("vel_scale", 0.20));
  arm.setMaxAccelerationScalingFactor(node->declare_parameter<double>("acc_scale", 0.20));
  arm.setPlanningTime(node->declare_parameter<double>("planning_time", 3.0));
  arm.setGoalPositionTolerance(node->declare_parameter<double>("pos_tol", 0.01));
  arm.setGoalOrientationTolerance(node->declare_parameter<double>("ori_tol", 3.14));

  auto gripper_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd", gripper_cb_group);

  const double GRIP_OPEN  = node->declare_parameter<double>("grip_open", 0.019);
  const double GRIP_CLOSE = node->declare_parameter<double>("grip_close", 0.0);
  const double GRIP_EFFORT_OPEN  = node->declare_parameter<double>("grip_effort_open", 10.0);
  const double GRIP_EFFORT_CLOSE = node->declare_parameter<double>("grip_effort_close", 30.0);

  const double max_tf_age_sec = node->declare_parameter<double>("max_tf_age_sec", 1.0);

  const double marker_z_min = node->declare_parameter<double>("marker_z_min", 0.05);
  const double marker_z_max = node->declare_parameter<double>("marker_z_max", 2.00);

  const double K_yaw        = node->declare_parameter<double>("K_yaw", 0.6);
  const double K_pitch      = node->declare_parameter<double>("K_pitch", 0.6);
  const double max_step_rad = node->declare_parameter<double>("max_step_rad", 0.12);

  node->declare_parameter<int>("yaw_cmd_sign", -1);
  node->declare_parameter<int>("pitch_cmd_sign", +1);

  const double yaw_tol_rad   = node->declare_parameter<double>("yaw_tol_deg", 3.0)   * M_PI / 180.0;
  const double pitch_tol_rad = node->declare_parameter<double>("pitch_tol_deg", 3.0) * M_PI / 180.0;
  const int center_need      = node->declare_parameter<int>("center_need", 5);

  const double j1_min = node->declare_parameter<double>("j1_min", -M_PI);
  const double j1_max = node->declare_parameter<double>("j1_max",  M_PI);
  const double j2_min = node->declare_parameter<double>("j2_min", -1.5);
  const double j2_max = node->declare_parameter<double>("j2_max",  1.5);

  const double xy_max      = node->declare_parameter<double>("xy_max", 0.30);
  const double z_min       = node->declare_parameter<double>("z_min", 0.05);
  const double z_max       = node->declare_parameter<double>("z_max", 0.35);

  const double approach_dx  = node->declare_parameter<double>("approach_dx", 0.06);
  const double max_step_m   = node->declare_parameter<double>("max_step_m", 0.05);
  const double close_dist_m = node->declare_parameter<double>("close_dist_m", 0.10);
  const double lift_dist    = node->declare_parameter<double>("lift_dist", 0.10);

  const double approach_arrive_tol = node->declare_parameter<double>("approach_arrive_tol", 0.01);
  const double approach_goal_eps   = node->declare_parameter<double>("approach_goal_eps", 0.005);
  const int close_need             = node->declare_parameter<int>("close_need", 3);

  int fail_count = 0;
  const int max_fail = node->declare_parameter<int>("max_fail", 3);

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

  auto getFreshTF = [&](const std::string& target,
                        const std::string& source,
                        geometry_msgs::msg::TransformStamped& out) -> bool
  {
    try {
      if (!tf_buffer->canTransform(target, source, tf2::TimePointZero, 80ms))
        return false;

      auto t = tf_buffer->lookupTransform(target, source, tf2::TimePointZero);

      if (!(t.header.stamp.sec == 0 && t.header.stamp.nanosec == 0)) {
        const rclcpp::Time now = node->get_clock()->now();
        const rclcpp::Time stamp = rclcpp::Time(t.header.stamp);
        const double age = (now - stamp).seconds();
        if (age < 0.0 || age > max_tf_age_sec) return false;
      }

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

  bool opened_on_detect = false;
  int center_count = 0;
  int close_count  = 0;

  bool have_prev_approach_target = false;
  double prev_tgt_x = 0.0;
  double prev_tgt_y = 0.0;
  double prev_tgt_z = 0.0;

  RCLCPP_INFO(node->get_logger(), "MoveIt planning_frame=%s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "MoveIt pose_ref_frame=%s", arm.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "MoveIt eef_link=%s", arm.getEndEffectorLink().c_str());

  // [수정 2] joint 이름/인덱스 확인 로그
  {
    const auto joint_names = arm.getJointNames();
    for (size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "MoveIt joint[%zu] = %s", i, joint_names[i].c_str());
    }
  }

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    switch (state) {

      case FSM::HOME_INIT: {
        RCLCPP_INFO(node->get_logger(), ">> HOME_INIT: home + open gripper");
        arm.setNamedTarget("home");
        arm.move();
        rclcpp::sleep_for(500ms);

        (void)operateGripper(node.get(), gripper, GRIP_OPEN, GRIP_EFFORT_OPEN);

        fail_count = 0;
        center_count = 0;
        close_count = 0;
        target_id = -1;
        target_frame.clear();
        opened_on_detect = false;
        have_prev_approach_target = false;

        state = FSM::WAIT_MARKER;
        break;
      }

      case FSM::WAIT_MARKER: {
        const int best = selectClosestMarkerId();
        if (best < 0) {
          center_count = 0;
          close_count = 0;
          opened_on_detect = false;
          have_prev_approach_target = false;
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAIT_MARKER: no marker yet");
          break;
        }

        if (best != target_id) {
          target_id = best;
          target_frame = "aruco_marker_" + std::to_string(target_id);
          center_count = 0;
          close_count = 0;
          opened_on_detect = false;
          have_prev_approach_target = false;
          RCLCPP_INFO(node->get_logger(), ">> Target marker ID=%d (%s)",
                      target_id, target_frame.c_str());
        }

        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          center_count = 0;
          close_count = 0;
          break;
        }

        const double mx = t_cam_marker.transform.translation.x;
        const double my = t_cam_marker.transform.translation.y;
        const double mz = t_cam_marker.transform.translation.z;
        const double md = distXYZ(mx, my, mz);

        RCLCPP_INFO(node->get_logger(),
                    ">> MARKER(camera->%s): xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                    target_frame.c_str(), mx, my, mz, md);

        if (!opened_on_detect) {
          RCLCPP_INFO(node->get_logger(), ">> Marker detected! Open gripper.");
          (void)operateGripper(node.get(), gripper, GRIP_OPEN, GRIP_EFFORT_OPEN);
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
          close_count = 0;
          have_prev_approach_target = false;
          state = FSM::WAIT_MARKER;
          break;
        }

        const double x = t_cam_marker.transform.translation.x;
        const double y = t_cam_marker.transform.translation.y;
        const double z = t_cam_marker.transform.translation.z;
        const double d = distXYZ(x, y, z);

        RCLCPP_INFO(node->get_logger(),
                    ">> CENTER: cam xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                    x, y, z, d);

        if (d <= close_dist_m) {
          close_count++;
          RCLCPP_INFO(node->get_logger(), ">> CENTER close OK %d/%d", close_count, close_need);
          if (close_count >= close_need) {
            RCLCPP_INFO(node->get_logger(), ">> Within %.2fm -> GRASP", close_dist_m);
            state = FSM::GRASP;
            break;
          }
        } else {
          close_count = 0;
        }

        const double yaw_err   = std::atan2(x, z);
        const double pitch_err = std::atan2(-y, z);

        if (std::abs(yaw_err) < yaw_tol_rad && std::abs(pitch_err) < pitch_tol_rad) {
          center_count++;
          RCLCPP_INFO(node->get_logger(), ">> CENTER OK %d/%d", center_count, center_need);
        } else {
          center_count = 0;
        }

        std::vector<double> joints = arm.getCurrentJointValues();
        if (joints.size() < 2) {
          close_count = 0;
          state = FSM::WAIT_MARKER;
          break;
        }

        int yaw_cmd_sign = -1;
        int pitch_cmd_sign = +1;
        node->get_parameter("yaw_cmd_sign", yaw_cmd_sign);
        node->get_parameter("pitch_cmd_sign", pitch_cmd_sign);
        yaw_cmd_sign   = (yaw_cmd_sign >= 0) ? 1 : -1;
        pitch_cmd_sign = (pitch_cmd_sign >= 0) ? 1 : -1;

        double dyaw   = clampd(yaw_cmd_sign * K_yaw * yaw_err, -max_step_rad, max_step_rad);
        double dpitch = clampd(-pitch_cmd_sign * K_pitch * pitch_err, -max_step_rad, max_step_rad);

        // [수정 3] CENTER 단계 디버그 로그 추가
        RCLCPP_INFO(node->get_logger(),
          ">> CENTER ERR: yaw_err=%.4f pitch_err=%.4f dyaw=%.4f dpitch=%.4f j0_before=%.4f j1_before=%.4f",
          yaw_err, pitch_err, dyaw, dpitch, joints[0], joints[1]);

        joints[0] = clampd(joints[0] + dyaw,   j1_min, j1_max);
        joints[1] = clampd(joints[1] + dpitch, j2_min, j2_max);

        RCLCPP_INFO(node->get_logger(),
          ">> CENTER CMD: j0_after=%.4f j1_after=%.4f",
          joints[0], joints[1]);

        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);

        auto res = arm.move();
        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          close_count = 0;
          RCLCPP_WARN(node->get_logger(), "CENTER move failed (%d/%d) -> back to WAIT",
                      fail_count, max_fail);
          state = FSM::WAIT_MARKER;
          break;
        } else {
          fail_count = 0;
        }

        if (center_count >= center_need) {
          close_count = 0;
          have_prev_approach_target = false;
          RCLCPP_INFO(node->get_logger(), ">> CENTER DONE -> APPROACH_CONTINUOUS");
          state = FSM::APPROACH_CONTINUOUS;
        }

        break;
      }

      case FSM::APPROACH_CONTINUOUS: {
        if (target_frame.empty()) {
          state = FSM::WAIT_MARKER;
          break;
        }

        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          close_count = 0;
          have_prev_approach_target = false;
          state = FSM::WAIT_MARKER;
          break;
        }

        const double cx = t_cam_marker.transform.translation.x;
        const double cy = t_cam_marker.transform.translation.y;
        const double cz = t_cam_marker.transform.translation.z;
        const double cd = distXYZ(cx, cy, cz);

        RCLCPP_INFO(node->get_logger(),
                    ">> TRACK(cam): xyz=(%.3f %.3f %.3f) dist=%.3f m",
                    cx, cy, cz, cd);

        RCLCPP_INFO(node->get_logger(),
                    ">> APPROACH CHECK: cd=%.3f, close_dist_m=%.3f, target_frame=%s",
                    cd, close_dist_m, target_frame.c_str());

        if (cd <= close_dist_m) {
          close_count++;
          RCLCPP_INFO(node->get_logger(), ">> APPROACH close OK %d/%d", close_count, close_need);
          if (close_count >= close_need) {
            RCLCPP_INFO(node->get_logger(), ">> Within %.2fm -> GRASP", close_dist_m);
            state = FSM::GRASP;
            break;
          }
        } else {
          close_count = 0;
        }

        geometry_msgs::msg::TransformStamped t_base_marker;
        if (!getFreshTF(base_frame, target_frame, t_base_marker)) {
          close_count = 0;
          have_prev_approach_target = false;
          state = FSM::WAIT_MARKER;
          break;
        }

        const double bx = t_base_marker.transform.translation.x;
        const double by = t_base_marker.transform.translation.y;
        const double bz = t_base_marker.transform.translation.z;

        const double goal_x = clampd(bx + approach_dx, -xy_max, xy_max);
        const double goal_y = clampd(by,              -xy_max, xy_max);
        const double goal_z = clampd(bz,               z_min,   z_max);

        auto ee_now = arm.getCurrentPose().pose;

        // [수정 4] APPROACH 단계 frame 확인 로그 추가
        RCLCPP_INFO(node->get_logger(),
          ">> APPROACH FRAME CHECK: base_goal=(%.3f %.3f %.3f), ee_now=(%.3f %.3f %.3f), pose_ref=%s",
          goal_x, goal_y, goal_z,
          ee_now.position.x, ee_now.position.y, ee_now.position.z,
          arm.getPoseReferenceFrame().c_str());

        const double err_x = goal_x - ee_now.position.x;
        const double err_y = goal_y - ee_now.position.y;
        const double err_z = goal_z - ee_now.position.z;
        const double goal_err = distXYZ(err_x, err_y, err_z);

        const double tgt_x = stepToward(ee_now.position.x, goal_x, max_step_m);
        const double tgt_y = stepToward(ee_now.position.y, goal_y, max_step_m);
        const double tgt_z = stepToward(ee_now.position.z, goal_z, max_step_m);

        RCLCPP_INFO(node->get_logger(),
          ">> APPROACH DBG: EE=(%.3f %.3f %.3f) goal_raw=(%.3f %.3f %.3f) tgt=(%.3f %.3f %.3f) goal_err=%.4f",
          ee_now.position.x, ee_now.position.y, ee_now.position.z,
          goal_x, goal_y, goal_z,
          tgt_x, tgt_y, tgt_z,
          goal_err
        );

        if (goal_err <= approach_arrive_tol) {
          RCLCPP_INFO(node->get_logger(),
                      ">> APPROACH SKIP: already near goal (goal_err=%.4f <= %.4f)",
                      goal_err, approach_arrive_tol);
          break;
        }

        if (have_prev_approach_target) {
          const double dt = distXYZ(tgt_x - prev_tgt_x, tgt_y - prev_tgt_y, tgt_z - prev_tgt_z);
          if (dt <= approach_goal_eps) {
            RCLCPP_INFO(node->get_logger(),
                        ">> APPROACH SKIP: target change too small (dt=%.4f <= %.4f)",
                        dt, approach_goal_eps);
            break;
          }
        }

        arm.setStartStateToCurrentState();
        arm.setPositionTarget(tgt_x, tgt_y, tgt_z);

        auto res = arm.move();
        arm.clearPoseTargets();

        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          close_count = 0;
          RCLCPP_WARN(node->get_logger(), "APPROACH move failed (%d/%d) -> back to CENTER",
                      fail_count, max_fail);

          if (fail_count >= max_fail) {
            fail_count = 0;
            center_count = 0;
            have_prev_approach_target = false;
            state = FSM::WAIT_MARKER;
          } else {
            center_count = 0;
            have_prev_approach_target = false;
            state = FSM::CENTER_ON_MARKER;
          }
        } else {
          fail_count = 0;
          prev_tgt_x = tgt_x;
          prev_tgt_y = tgt_y;
          prev_tgt_z = tgt_z;
          have_prev_approach_target = true;
        }

        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASP: close gripper");
        (void)operateGripper(node.get(), gripper, GRIP_CLOSE, GRIP_EFFORT_CLOSE);
        state = FSM::LIFT_HOME;
        break;
      }

      case FSM::LIFT_HOME: {
        RCLCPP_INFO(node->get_logger(), ">> LIFT_HOME: lift and home");
        auto ee = arm.getCurrentPose().pose;

        geometry_msgs::msg::Pose lift_pose = ee;
        lift_pose.position.z = clampd(lift_pose.position.z + lift_dist, z_min, z_max);

        arm.setStartStateToCurrentState();
        arm.setPoseTarget(lift_pose);
        arm.move();
        arm.clearPoseTargets();

        arm.setNamedTarget("home");
        arm.move();

        center_count = 0;
        close_count = 0;
        fail_count = 0;
        opened_on_detect = false;
        have_prev_approach_target = false;
        state = FSM::WAIT_MARKER;
        break;
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return 0;
}