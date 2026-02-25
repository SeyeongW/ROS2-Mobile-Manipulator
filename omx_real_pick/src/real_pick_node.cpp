// omx_real_picker_alt_tf_direct.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <algorithm>

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

  // 결과 상세가 필요하면 fut_res.get() 파싱 가능
  return true;
}

enum class FSM { SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME };

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("omx_real_picker_alt_tf_direct");

  node->declare_parameter<bool>("use_sim_time", true);

  // TF
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // MoveIt
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setPlanningTime(5.0);
  arm.setGoalPositionTolerance(0.03);
  arm.setGoalOrientationTolerance(3.14);

  // Frames
  std::string base_frame   = node->declare_parameter<std::string>("base_frame", "link1");
  std::string marker_frame = node->declare_parameter<std::string>("marker_frame", "aruco_marker_23");

  arm.setPoseReferenceFrame(base_frame);
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "Pose reference frame: %s", base_frame.c_str());

  // Gripper
  auto gripper = rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd");
  double GRIP_OPEN  = node->declare_parameter<double>("grip_open", 0.019);
  double GRIP_CLOSE = node->declare_parameter<double>("grip_close", 0.0);   // -0.001 대신 0.0 권장
  double GRIP_EFFORT= node->declare_parameter<double>("grip_effort", 30.0); // 0.5는 너무 낮을 가능성 큼

  // SEARCH waypoints
  std::vector<std::vector<double>> waypoints = {
    { 0.00, -0.20,  0.20,  0.80},
    { 1.00, -0.20,  0.20,  0.80},
    {-1.00, -0.20,  0.20,  0.80},
    { 0.00, -0.60,  0.30,  1.20}
  };

  // Approach params
  double hover_offset = node->declare_parameter<double>("hover_offset", 0.15);
  double step         = node->declare_parameter<double>("step", 0.03);
  double final_min_z  = node->declare_parameter<double>("final_min_z", 0.04);

  double max_tf_age_sec = node->declare_parameter<double>("max_tf_age_sec", 0.7);
  int stable_need       = node->declare_parameter<int>("stable_need", 5);

  const size_t FILTER_N = 7;
  std::deque<double> fx, fy, fz;

  // init
  RCLCPP_INFO(node->get_logger(), ">> HOME");
  arm.setNamedTarget("home");
  arm.move();
  rclcpp::sleep_for(600ms);

  if(!operateGripperBlocking(node, gripper, GRIP_OPEN, GRIP_EFFORT)){
    RCLCPP_ERROR(node->get_logger(), "Gripper open failed");
  }

  FSM state = FSM::SEARCH;
  int wp_idx = 0;
  int stable_count = 0;

  double current_offset = hover_offset;
  double mx=0, my=0, mz=0;

  auto getFreshMarkerFromTF = [&]() -> bool {
    if(!tf_buffer.canTransform(base_frame, marker_frame, tf2::TimePointZero, tf2::durationFromSec(0.15))){
      return false;
    }
    geometry_msgs::msg::TransformStamped tf = tf_buffer.lookupTransform(base_frame, marker_frame, tf2::TimePointZero);

    // TF age check
    const rclcpp::Time now = node->get_clock()->now();
    const rclcpp::Time st(tf.header.stamp);
    const double age = (now - st).seconds();
    if(age > max_tf_age_sec) return false;

    double x=tf.transform.translation.x;
    double y=tf.transform.translation.y;
    double z=tf.transform.translation.z;
    if(!isFinite(x)||!isFinite(y)||!isFinite(z)) return false;

    mx = medianPush(fx, x, FILTER_N);
    my = medianPush(fy, y, FILTER_N);
    mz = medianPush(fz, z, FILTER_N);
    return true;
  };

  rclcpp::Rate rate(10);

  while(rclcpp::ok()){
    bool visible = getFreshMarkerFromTF();

    switch(state){
      case FSM::SEARCH: {
        stable_count = 0;
        fx.clear(); fy.clear(); fz.clear();

        if(visible){
          RCLCPP_INFO(node->get_logger(), ">> Target seen -> STABILIZE");
          state = FSM::STABILIZE;
          break;
        }

        RCLCPP_INFO(node->get_logger(), ">> SEARCH waypoint %d", wp_idx);
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
        RCLCPP_INFO(node->get_logger(), ">> STABILIZE %d/%d  (%.3f %.3f %.3f)",
                    stable_count, stable_need, mx, my, mz);

        if(stable_count >= stable_need){
          current_offset = hover_offset;
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

        // target pose in base_frame
        geometry_msgs::msg::PoseStamped tgt;
        tgt.header.frame_id = base_frame;
        tgt.header.stamp = node->get_clock()->now();
        tgt.pose.position.x = mx;
        tgt.pose.position.y = my;
        tgt.pose.position.z = std::max(final_min_z, mz + current_offset);
        tgt.pose.orientation = arm.getCurrentPose().pose.orientation; // 4DOF면 orientation 고정이 더 안전

        RCLCPP_INFO(node->get_logger(), ">> APPROACH off=%.3f  tgt=(%.3f %.3f %.3f)",
                    current_offset, tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z);

        arm.setPoseTarget(tgt);
        arm.move();

        current_offset = std::max(0.0, current_offset - step);

        if(current_offset <= 0.0001){
          state = FSM::GRASP;
          RCLCPP_INFO(node->get_logger(), ">> Reached -> GRASP");
        }
      } break;

      case FSM::GRASP: {
        if(!operateGripperBlocking(node, gripper, GRIP_CLOSE, GRIP_EFFORT)){
          RCLCPP_ERROR(node->get_logger(), "Gripper close failed");
        }
        state = FSM::LIFT_AND_HOME;
      } break;

      case FSM::LIFT_AND_HOME: {
        auto cur = arm.getCurrentPose();
        cur.header.frame_id = base_frame;
        cur.pose.position.z += 0.10;
        arm.setPoseTarget(cur);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();
        operateGripperBlocking(node, gripper, GRIP_OPEN, GRIP_EFFORT);
        state = FSM::SEARCH;
      } break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
 // MoveIt pose 기준 프레임을 link1로 강제

// 타겟 입력을 topic Pose 변환 대신 TF 직접 조회로 변경

// 그리퍼를 블로킹+결과 확인으로 바꾸고 effort 상향

// 시간/신선도 판단을 TF 기준으로 통일

// Planning frame: ... 출력이 뭐로 뜨는지

// ros2 run tf2_ros tf2_echo link1 aruco_marker_23가 끊김 없이 나오는지

// ros2 action list | grep gripper로 액션 이름이 /gripper_controller/gripper_cmd가 맞는지