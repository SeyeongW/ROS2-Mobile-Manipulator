// omx_real_picker_tf_only.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <algorithm>
#include <future>
#include <thread>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

static bool isFinite(double v){ return std::isfinite(v); }

static double medianPush(std::deque<double>& dq, double v, size_t N){
  dq.push_back(v);
  if(dq.size() > N) dq.pop_front();
  std::vector<double> tmp(dq.begin(), dq.end());
  std::sort(tmp.begin(), tmp.end());
  return tmp[tmp.size()/2];
}

static bool operateGripperBlocking(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<GripperCommand>::SharedPtr client,
  double pos, double effort,
  std::chrono::milliseconds server_wait = 1500ms,
  std::chrono::milliseconds result_wait = 4000ms
){
  if(!client->wait_for_action_server(server_wait)){
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = effort;

  auto fut_goal = client->async_send_goal(goal);
  if(fut_goal.wait_for(2s) != std::future_status::ready){
    RCLCPP_ERROR(node->get_logger(), "Gripper send_goal timeout");
    return false;
  }

  auto gh = fut_goal.get();
  if(!gh){
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
    return false;
  }

  auto fut_res = client->async_get_result(gh);
  if(fut_res.wait_for(result_wait) != std::future_status::ready){
    RCLCPP_ERROR(node->get_logger(), "Gripper result timeout");
    return false;
  }

  return true;
}

enum class FSM { SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME };

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("omx_real_picker_tf_only");

  // Executor (필수: TF/Action 콜백 안정)
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&](){ exec->spin(); });

  // TF
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // MoveIt
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setPlanningTime(5.0);
  arm.setGoalPositionTolerance(0.02);   // 조금 타이트하게
  arm.setGoalOrientationTolerance(3.14);

  // Frames
  std::string base_frame   = node->declare_parameter<std::string>("base_frame", "link1");
  std::string marker_frame = node->declare_parameter<std::string>("marker_frame", "aruco_marker_23");

  arm.setPoseReferenceFrame(base_frame);
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "Pose reference frame: %s", base_frame.c_str());
  RCLCPP_INFO(node->get_logger(), "Marker frame: %s", marker_frame.c_str());

  // Gripper
  auto gripper = rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd");
  double GRIP_OPEN   = node->declare_parameter<double>("grip_open", 0.019);
  double GRIP_CLOSE  = node->declare_parameter<double>("grip_close", 0.0);
  double GRIP_EFFORT = node->declare_parameter<double>("grip_effort", 0.2);

  // SEARCH waypoints
  std::vector<std::vector<double>> waypoints = {
    { 0.00, -0.20,  0.20,  0.80},
    { 1.00, -0.20,  0.20,  0.80},
    {-1.00, -0.20,  0.20,  0.80},
    { 0.00, -0.60,  0.30,  1.20}
  };

  // TF freshness
  double max_tf_age_sec = node->declare_parameter<double>("max_tf_age_sec", 1.0);
  int stable_need       = node->declare_parameter<int>("stable_need", 5);

  // Safety clamp (오프셋 아님: 로봇이 갈 수 없는 목표를 걸러주는 안전장치)
  double min_z = node->declare_parameter<double>("min_z", 0.02);
  double max_z = node->declare_parameter<double>("max_z", 0.40);
  double max_xy = node->declare_parameter<double>("max_xy", 0.35);

  const size_t FILTER_N = 7;
  std::deque<double> fx, fy, fz;

  // init
  RCLCPP_INFO(node->get_logger(), ">> HOME");
  arm.setNamedTarget("home");
  arm.move();
  rclcpp::sleep_for(600ms);

  (void)operateGripperBlocking(node, gripper, GRIP_OPEN, GRIP_EFFORT);

  FSM state = FSM::SEARCH;
  int wp_idx = 0;
  int stable_count = 0;

  double mx=0, my=0, mz=0;

  auto getFreshMarkerFromTF = [&]() -> bool {
    if(!tf_buffer.canTransform(base_frame, marker_frame, tf2::TimePointZero, tf2::durationFromSec(0.15))){
      return false;
    }
    geometry_msgs::msg::TransformStamped tf = tf_buffer.lookupTransform(
      base_frame, marker_frame, tf2::TimePointZero);

    const rclcpp::Time now = node->get_clock()->now();
    const rclcpp::Time st(tf.header.stamp);
    const double age = (now - st).seconds();
    if(age > max_tf_age_sec) return false;

    const double x = tf.transform.translation.x;
    const double y = tf.transform.translation.y;
    const double z = tf.transform.translation.z;
    if(!isFinite(x)||!isFinite(y)||!isFinite(z)) return false;

    mx = medianPush(fx, x, FILTER_N);
    my = medianPush(fy, y, FILTER_N);
    mz = medianPush(fz, z, FILTER_N);
    return true;
  };

  rclcpp::Rate rate(10);

  while(rclcpp::ok()){
    const bool visible = getFreshMarkerFromTF();

    switch(state){
      case FSM::SEARCH: {
        stable_count = 0;
        fx.clear(); fy.clear(); fz.clear();

        if(visible){
          RCLCPP_INFO(node->get_logger(), ">> Target seen -> STABILIZE");
          state = FSM::STABILIZE;
          break;
        }

        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1500,
                             ">> SEARCH waypoint %d", wp_idx);
        arm.setJointValueTarget(waypoints[wp_idx]);
        arm.move();
        wp_idx = (wp_idx + 1) % waypoints.size();
        rclcpp::sleep_for(200ms);
      } break;

      case FSM::STABILIZE: {
        if(!visible){
          RCLCPP_WARN(node->get_logger(), ">> Lost target -> SEARCH");
          state = FSM::SEARCH;
          break;
        }

        stable_count++;
        RCLCPP_INFO(node->get_logger(), ">> STABILIZE %d/%d  raw=(%.3f %.3f %.3f)",
                    stable_count, stable_need, mx, my, mz);

        if(stable_count >= stable_need){
          state = FSM::APPROACH;
          RCLCPP_INFO(node->get_logger(), ">> STABILIZE done -> APPROACH");
        }
      } break;

      case FSM::APPROACH: {
        if(!visible){
          RCLCPP_WARN(node->get_logger(), ">> Lost target -> STABILIZE");
          stable_count = 0;
          state = FSM::STABILIZE;
          break;
        }

        // ✅ 오로지 TF값(mx,my,mz)로 목표 생성 (offset 없음)
        // 단, 안전 클램프만 적용
        const double tx = std::clamp(mx, -max_xy, max_xy);
        const double ty = std::clamp(my, -max_xy, max_xy);
        const double tz = std::clamp(mz,  min_z,  max_z);

        RCLCPP_INFO(node->get_logger(), ">> APPROACH tf=(%.3f %.3f %.3f) clamped=(%.3f %.3f %.3f)",
                    mx, my, mz, tx, ty, tz);

        arm.setStartStateToCurrentState();
        arm.setPositionTarget(tx, ty, tz);

        auto res = arm.move();
        if(res != moveit::core::MoveItErrorCode::SUCCESS){
          RCLCPP_WARN(node->get_logger(), ">> Move failed (Invalid goal?). Back to STABILIZE");
          stable_count = 0;
          state = FSM::STABILIZE;
          break;
        }

        state = FSM::GRASP;
        RCLCPP_INFO(node->get_logger(), ">> Arrived -> GRASP");
      } break;

      case FSM::GRASP: {
        (void)operateGripperBlocking(node, gripper, GRIP_CLOSE, GRIP_EFFORT);
        state = FSM::LIFT_AND_HOME;
      } break;

      case FSM::LIFT_AND_HOME: {
        auto cur = arm.getCurrentPose();
        cur.header.frame_id = base_frame;
        cur.pose.position.z += 0.10;

        arm.setStartStateToCurrentState();
        arm.setPoseTarget(cur);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        (void)operateGripperBlocking(node, gripper, GRIP_OPEN, GRIP_EFFORT);
        state = FSM::SEARCH;
      } break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  exec->cancel();
  if(spinner.joinable()) spinner.join();
  return 0;
}