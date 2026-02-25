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
#include <future>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using ArucoMarkers   = ros2_aruco_interfaces::msg::ArucoMarkers;

static double clamp(double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); }

static bool operateGripperWait(const rclcpp::Node::SharedPtr& node,
                               const rclcpp_action::Client<GripperCommand>::SharedPtr& client,
                               double pos, double effort)
{
  if(!client->wait_for_action_server(2s)){
    RCLCPP_ERROR(node->get_logger(), "gripper server not ready");
    return false;
  }
  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = effort;

  auto send_fut = client->async_send_goal(goal);
  if(send_fut.wait_for(3s) != std::future_status::ready){
    RCLCPP_ERROR(node->get_logger(), "send goal timeout");
    return false;
  }
  auto gh = send_fut.get();
  if(!gh){
    RCLCPP_ERROR(node->get_logger(), "goal rejected");
    return false;
  }
  auto res_fut = client->async_get_result(gh);
  if(res_fut.wait_for(5s) != std::future_status::ready){
    RCLCPP_ERROR(node->get_logger(), "result timeout");
    return false;
  }
  return res_fut.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

enum class FSM { HOME_INIT, WAIT_MARKER, PREGRASP, APPROACH, GRASP, LIFT_HOME };

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_node");

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spin_thread([&](){ exec->spin(); });

  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  const std::string base_frame = "link1";
  arm.setPoseReferenceFrame(base_frame);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  RCLCPP_INFO(node->get_logger(), "planning frame=%s", arm.getPlanningFrame().c_str());

  auto gripper = rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd");
  const double GRIP_OPEN = 0.019;
  const double GRIP_CLOSE = 0.0;

  std::mutex mk_mtx;
  ArucoMarkers latest;
  bool have_marker = false;

  auto sub = node->create_subscription<ArucoMarkers>(
    "/aruco/markers", 10,
    [&](const ArucoMarkers::SharedPtr msg){
      std::lock_guard<std::mutex> lk(mk_mtx);
      latest = *msg;
      have_marker = true;
    }
  );

  FSM state = FSM::HOME_INIT;
  geometry_msgs::msg::Pose pregrasp_pose, final_pose;
  double latch_x = 0.0, latch_y = 0.0;
  rclcpp::Rate rate(10);

  while(rclcpp::ok()){
    switch(state){
      case FSM::HOME_INIT: {
        arm.setNamedTarget("home");
        arm.move();
        operateGripperWait(node, gripper, GRIP_OPEN, 5.0);
        state = FSM::WAIT_MARKER;
      } break;

      case FSM::WAIT_MARKER: {
        ArucoMarkers snap;
        { std::lock_guard<std::mutex> lk(mk_mtx);
          if(!have_marker || latest.marker_ids.empty()) break;
          snap = latest;
        }

        const std::string target_frame = "aruco_marker_" + std::to_string(snap.marker_ids[0]);
        geometry_msgs::msg::TransformStamped tf;
        try{
          tf = tf_buffer->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
        }catch(...){
          break;
        }

        latch_x = tf.transform.translation.x;
        latch_y = tf.transform.translation.y;

        pregrasp_pose.position.x = latch_x - 0.12;
        pregrasp_pose.position.y = latch_y;
        pregrasp_pose.position.z = 0.22;

        final_pose.position.x = latch_x - 0.03;
        final_pose.position.y = latch_y;
        final_pose.position.z = 0.22;

        auto ee = arm.getCurrentPose().pose;
        pregrasp_pose.orientation = ee.orientation;
        final_pose.orientation = ee.orientation;

        state = FSM::PREGRASP;
      } break;

      case FSM::PREGRASP: {
        arm.setPoseTarget(pregrasp_pose);
        if(arm.move() != moveit::core::MoveItErrorCode::SUCCESS){
          state = FSM::WAIT_MARKER;
          break;
        }
        state = FSM::APPROACH;
      } break;

      case FSM::APPROACH: {
        arm.setPoseTarget(final_pose);
        if(arm.move() != moveit::core::MoveItErrorCode::SUCCESS){
          state = FSM::WAIT_MARKER;
          break;
        }
        state = FSM::GRASP;
      } break;

      case FSM::GRASP: {
        operateGripperWait(node, gripper, GRIP_CLOSE, 10.0);
        std::this_thread::sleep_for(1s);
        state = FSM::LIFT_HOME;
      } break;

      case FSM::LIFT_HOME: {
        auto pose = arm.getCurrentPose().pose;
        pose.position.z = clamp(pose.position.z + 0.1, -10.0, 10.0); // clamp는 안전용(원하면 z범위로 바꿔)
        arm.setPoseTarget(pose);
        arm.move();
        arm.setNamedTarget("home");
        arm.move();
        state = FSM::WAIT_MARKER;
      } break;
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}