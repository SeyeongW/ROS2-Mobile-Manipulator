#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include <deque>
#include <algorithm>

using GripperCommand = control_msgs::action::GripperCommand;
using namespace std::chrono_literals;

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
  WAIT,          // 가만히 대기
  STABILIZE,     // N번 연속 fresh TF 확보 + 필터
  APPROACH,      // step-down 접근
  GRASP,         // 집기
  LIFT_AND_HOME  // 들어올리고 홈
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("omx_real_picker_static_wait");
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&exec](){ exec->spin(); });

  // TF
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // MoveIt
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setPlanningTime(5.0);

  // position only
  arm.setGoalPositionTolerance(0.03);      // 3cm
  arm.setGoalOrientationTolerance(3.14);   // ignore

  // Gripper
  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  // ---------- SETTINGS ----------
  std::string base_frame = "link1";
  std::string target_marker = "aruco_marker_0";

  double GRIP_OPEN  = 0.019;
  double GRIP_CLOSE = -0.001;

  // APPROACH params
  double hover_offset = 0.15;     // 위에서 시작
  double step = 0.03;             // 접근 step
  double final_min_z = 0.04;      // 바닥 박힘 방지
  double max_tf_age_sec = 1.5;    // ✅ 조금 넉넉히 (카메라/검출 끊김 허용)
  int    stable_need = 5;         // 연속 N회 안정화
  int    max_fail_before_wait = 3;

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

  // filtered marker
  double mx=0, my=0, mz=0;

  auto getFreshMarker = [&](double& ox, double& oy, double& oz) -> bool {
    try {
      if (!tf_buffer->canTransform(base_frame, target_marker, tf2::TimePointZero, 50ms)) {
        return false;
      }
      auto t = tf_buffer->lookupTransform(base_frame, target_marker, tf2::TimePointZero);

      rclcpp::Time now = node->get_clock()->now();
      rclcpp::Time stamp = t.header.stamp;
      double age = (now - stamp).seconds();

      if (age > max_tf_age_sec) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "TF too old: %.2fs (ignore)", age);
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

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    bool visible = getFreshMarker(mx, my, mz);

    switch (state) {
      case FSM::WAIT: {
        // ✅ 가만히 대기
        stable_count = 0;
        fx.clear(); fy.clear(); fz.clear();

        if (visible) {
          RCLCPP_INFO(node->get_logger(), ">> Marker seen. Switching to STABILIZE");
          state = FSM::STABILIZE;
        } else {
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAITING... (no marker)");
        }
        break;
      }

      case FSM::STABILIZE: {
        if (!visible) {
          stable_count = 0;
          RCLCPP_WARN(node->get_logger(), ">> Lost marker. Back to WAIT");
          state = FSM::WAIT;
          break;
        }

        stable_count++;
        RCLCPP_INFO(node->get_logger(), ">> STABILIZE %d/%d (x=%.3f y=%.3f z=%.3f)",
                    stable_count, stable_need, mx, my, mz);

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

        RCLCPP_INFO(node->get_logger(), ">> APPROACH offset=%.3f target=(%.3f, %.3f, %.3f)",
                    current_offset, mx, my, target_z);

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
        RCLCPP_INFO(node->get_logger(), ">> GRASPING!");
        operateGripper(node, gripper, GRIP_CLOSE);
        state = FSM::LIFT_AND_HOME;
        break;
      }

      case FSM::LIFT_AND_HOME: {
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
