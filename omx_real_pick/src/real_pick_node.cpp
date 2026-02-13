#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include <deque>
#include <algorithm>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using ArucoMarkers   = ros2_aruco_interfaces::msg::ArucoMarkers;

static bool isFinite(double v) { return std::isfinite(v); }

void operateGripper(rclcpp::Node::SharedPtr node,
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
  WAIT,          // 가만히 대기 (마커 보이면 STABILIZE)
  STABILIZE,     // 동일 ID를 N번 연속으로 "fresh TF"로 확인
  APPROACH,      // step-down 접근
  GRASP,         // 집기
  LIFT_AND_HOME  // 들어올리고 홈
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("omx_real_picker_any_id_wait");
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

  // position only
  arm.setGoalPositionTolerance(0.03);
  arm.setGoalOrientationTolerance(3.14);

  // -------------- Gripper -------------
  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  // ---------- USER SETTINGS ----------
  std::string base_frame = "link1";          // 로봇 기준 프레임(네 환경에 맞게)
  std::string markers_topic = "/aruco/markers";

  // Gripper positions (환경마다 다를 수 있음)
  double GRIP_OPEN  = 0.019;
  double GRIP_CLOSE = -0.001;

  // APPROACH params
  double hover_offset = 0.15;
  double step         = 0.03;
  double final_min_z  = 0.04;

  // TF 신선도 기준 (카메라 FPS/검출 끊김 고려해서 넉넉히)
  double max_tf_age_sec = 1.5;

  // 같은 ID가 연속 N번 "fresh TF"로 잡혀야 안정화 완료
  int stable_need = 5;

  // move 실패 허용
  int max_fail_before_wait = 3;

  // filtering window (median)
  const size_t FILTER_N = 7;
  std::deque<double> fx, fy, fz;

  auto push_and_median = [&](std::deque<double>& dq, double v) -> double {
    dq.push_back(v);
    if (dq.size() > FILTER_N) dq.pop_front();
    std::vector<double> tmp(dq.begin(), dq.end());
    std::sort(tmp.begin(), tmp.end());
    return tmp[tmp.size()/2];
  };

  // ---------- /aruco/markers 최신값 저장 ----------
  std::mutex mk_mtx;
  ArucoMarkers latest_markers;
  bool have_markers = false;

  auto markers_sub = node->create_subscription<ArucoMarkers>(
    markers_topic, 10,
    [&](const ArucoMarkers::SharedPtr msg){
      std::lock_guard<std::mutex> lk(mk_mtx);
      latest_markers = *msg;
      have_markers = true;
    }
  );

  // ---------- "현재 보이는 마커 중 가장 가까운 ID" 선택 ----------
  auto selectClosestId = [&]() -> int {
    std::lock_guard<std::mutex> lk(mk_mtx);
    if (!have_markers) return -1;
    if (latest_markers.marker_ids.size() != latest_markers.poses.size()) return -1;
    if (latest_markers.marker_ids.empty()) return -1;

    int best_id = -1;
    double best_z = 1e9;

    for (size_t i=0; i<latest_markers.marker_ids.size(); i++) {
      const auto &p = latest_markers.poses[i].position;
      double z = p.z;
      if (!isFinite(z) || z <= 0.02 || z > 2.0) continue;
      if (z < best_z) {
        best_z = z;
        best_id = latest_markers.marker_ids[i];
      }
    }
    return best_id;
  };

  // ---------- 선택된 마커 TF를 base_frame 기준으로 "fresh"하게 얻기 ----------
  auto getFreshMarkerTF = [&](const std::string& target_marker_frame,
                              double& ox, double& oy, double& oz) -> bool
  {
    try {
      if (!tf_buffer->canTransform(base_frame, target_marker_frame, tf2::TimePointZero, 50ms)) {
        return false;
      }
      auto t = tf_buffer->lookupTransform(base_frame, target_marker_frame, tf2::TimePointZero);

      rclcpp::Time now = node->get_clock()->now();
      rclcpp::Time stamp = t.header.stamp;
      double age = (now - stamp).seconds();

      if (age > max_tf_age_sec) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "TF too old: %.2fs (ignore) frame=%s",
                             age, target_marker_frame.c_str());
        return false;
      }

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;

      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) return false;

      ox = push_and_median(fx, x);
      oy = push_and_median(fy, y);
      oz = push_and_median(fz, z);
      return true;
    } catch (...) {
      return false;
    }
  };

  // ---------- INIT ----------
  RCLCPP_INFO(node->get_logger(), ">> HOME");
  arm.setNamedTarget("home");
  arm.move();
  rclcpp::sleep_for(800ms);

  operateGripper(node, gripper, GRIP_OPEN);

  FSM state = FSM::WAIT;

  int stable_count = 0;
  int approach_fail = 0;

  double current_offset = hover_offset;

  // 목표 마커 ID/프레임
  int target_id = -1;
  std::string target_marker_frame;

  // filtered marker pose (base_frame 기준)
  double mx=0, my=0, mz=0;

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {

    // 1) 현재 보이는 마커 중 가장 가까운 ID 선택
    int best_id = selectClosestId();

    // 2) 타겟 ID가 바뀌면 필터/카운트 리셋
    if (best_id != target_id) {
      target_id = best_id;
      stable_count = 0;
      fx.clear(); fy.clear(); fz.clear();
      if (target_id >= 0) {
        target_marker_frame = "aruco_marker_" + std::to_string(target_id);
        RCLCPP_INFO(node->get_logger(), ">> Target marker set to ID=%d (%s)",
                    target_id, target_marker_frame.c_str());
      } else {
        target_marker_frame.clear();
      }
    }

    // 3) target_marker_frame이 있으면 TF로 pose 얻기
    bool visible = false;
    if (!target_marker_frame.empty()) {
      visible = getFreshMarkerTF(target_marker_frame, mx, my, mz);
    }

    switch (state) {
      case FSM::WAIT: {
        stable_count = 0;
        fx.clear(); fy.clear(); fz.clear();

        if (visible) {
          RCLCPP_INFO(node->get_logger(), ">> Marker seen (ID=%d). Switching to STABILIZE", target_id);
          state = FSM::STABILIZE;
        } else {
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAITING... (no fresh marker TF)");
        }
        break;
      }

      case FSM::STABILIZE: {
        if (!visible) {
          stable_count = 0;
          RCLCPP_WARN(node->get_logger(), ">> Lost marker TF. Back to WAIT");
          state = FSM::WAIT;
          break;
        }

        stable_count++;
        RCLCPP_INFO(node->get_logger(), ">> STABILIZE %d/%d (ID=%d x=%.3f y=%.3f z=%.3f)",
                    stable_count, stable_need, target_id, mx, my, mz);

        if (stable_count >= stable_need) {
          current_offset = hover_offset;
          approach_fail = 0;
          RCLCPP_INFO(node->get_logger(), ">> STABILIZE done. Switching to APPROACH");
          state = FSM::APPROACH;
        }
        break;
      }

      case FSM::APPROACH: {
        if (!visible) {
          RCLCPP_WARN(node->get_logger(), ">> Lost marker during approach. Back to STABILIZE");
          stable_count = 0;
          state = FSM::STABILIZE;
          break;
        }

        double target_z = std::max(final_min_z, mz + current_offset);

        RCLCPP_INFO(node->get_logger(), ">> APPROACH ID=%d offset=%.3f target=(%.3f, %.3f, %.3f)",
                    target_id, current_offset, mx, my, target_z);

        arm.setStartStateToCurrentState();
        arm.setPositionTarget(mx, my, target_z);

        auto result = arm.move();

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          approach_fail = 0;
          current_offset -= step;

          if (current_offset <= 0.01) {
            RCLCPP_INFO(node->get_logger(), ">> Reached near target. Switching to GRASP");
            state = FSM::GRASP;
          }
        } else {
          approach_fail++;
          RCLCPP_ERROR(node->get_logger(), "Move failed (%d/%d).", approach_fail, max_fail_before_wait);

          if (approach_fail >= max_fail_before_wait) {
            RCLCPP_WARN(node->get_logger(), ">> Too many failures. Back to WAIT");
            state = FSM::WAIT;
          } else {
            current_offset = std::min(hover_offset, current_offset + 0.05);
          }
        }
        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASPING! (ID=%d)", target_id);
        operateGripper(node, gripper, GRIP_CLOSE);
        state = FSM::LIFT_AND_HOME;
        break;
      }

      case FSM::LIFT_AND_HOME: {
        // lift는 마지막 mx,my,mz 기준
        double lift_z = std::max(final_min_z + 0.10, mz + hover_offset);
        RCLCPP_INFO(node->get_logger(), ">> LIFT to z=%.3f then HOME", lift_z);

        arm.setStartStateToCurrentState();
        arm.setPositionTarget(mx, my, lift_z);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        operateGripper(node, gripper, GRIP_OPEN);

        RCLCPP_INFO(node->get_logger(), ">> Done. Back to WAIT");
        state = FSM::WAIT;
        break;
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
