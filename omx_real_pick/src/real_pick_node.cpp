// omx_real_picker_tf_stop_pick.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <angles/angles.h>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <algorithm>
#include <memory>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static bool isFinite(double v){ return std::isfinite(v); }

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
  goal.command.position   = pos;
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

class OmxPickTfStop : public rclcpp::Node
{
public:
  OmxPickTfStop(rclcpp::Node::SharedPtr move_group_arm_node)
  : Node("omx_pick_tf_stop")
  {
    // ---- params (declare) ----
    // use_sim_time: 중복 declare 방지
    if(!this->has_parameter("use_sim_time")){
      this->declare_parameter<bool>("use_sim_time", true);
    }
    bool use_sim_time=true;
    this->get_parameter("use_sim_time", use_sim_time);
    RCLCPP_INFO(get_logger(), "use_sim_time=%s", use_sim_time ? "true" : "false");

    base_frame_   = this->declare_parameter<std::string>("base_frame", "link1");
    marker_frame_ = this->declare_parameter<std::string>("marker_frame", "aruco_marker_23");

    // “성공 코드 방식”의 정지 판정 파라미터
    filtering_time_      = this->declare_parameter<double>("filtering_time_sec", 2.0);   // 최근 TF만 사용
    stop_time_threshold_ = this->declare_parameter<double>("stop_time_sec", 3.0);        // 3초 정지면 pick
    distance_threshold_  = this->declare_parameter<double>("distance_threshold_m", 0.01);// 1cm 이하 이동이면 정지로 간주
    target_z_min_        = this->declare_parameter<double>("target_z_min", 0.04);

    // pick 동작 오프셋(성공 예제 그대로 기본값)
    hover_z_ = this->declare_parameter<double>("hover_z", 0.12);
    grasp_z_ = this->declare_parameter<double>("grasp_z", 0.07);
    lift_z_  = this->declare_parameter<double>("lift_z",  0.12);

    // place 위치
    place_x_ = this->declare_parameter<double>("place_x", 0.10);
    place_y_ = this->declare_parameter<double>("place_y", 0.20);
    place_z_ = this->declare_parameter<double>("place_z", 0.13);

    // MoveIt 속도
    vel_scale_ = this->declare_parameter<double>("vel_scale", 0.15);
    acc_scale_ = this->declare_parameter<double>("acc_scale", 0.15);

    // gripper params (너 방식 유지)
    grip_open_   = this->declare_parameter<double>("grip_open", 0.019);
    grip_close_  = this->declare_parameter<double>("grip_close", 0.0);
    grip_effort_ = this->declare_parameter<double>("grip_effort", 30.0);

    // ---- TF ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- MoveIt arm ----
    arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    arm_->setPoseReferenceFrame(base_frame_);
    arm_->setMaxVelocityScalingFactor(vel_scale_);
    arm_->setMaxAccelerationScalingFactor(acc_scale_);
    arm_->setPlanningTime(5.0);
    arm_->setGoalPositionTolerance(0.03);
    arm_->setGoalOrientationTolerance(3.14);

    RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "Pose reference frame: %s", base_frame_.c_str());

    // ---- Gripper action client (this node) ----
    gripper_ = rclcpp_action::create_client<GripperCommand>(shared_from_this(),
                                                            "/gripper_controller/gripper_cmd");

    // ---- init pose ----
    RCLCPP_INFO(get_logger(), ">> HOME");
    arm_->setNamedTarget("home");
    arm_->move();
    rclcpp::sleep_for(500ms);
    operateGripperBlocking(shared_from_this(), gripper_, grip_open_, grip_effort_);

    // 과거 TF 초기화 플래그
    have_past_ = false;
    busy_ = false;

    // ---- timer ----
    timer_ = this->create_wall_timer(200ms, std::bind(&OmxPickTfStop::on_timer, this));
  }

private:
  void on_timer()
  {
    if(busy_) return;  // pick 실행 중 재진입 방지

    geometry_msgs::msg::TransformStamped tf_msg;
    try{
      tf_msg = tf_buffer_->lookupTransform(base_frame_, marker_frame_, tf2::TimePointZero);
    }catch(const tf2::TransformException& ex){
      // TF가 없으면 그냥 대기
      RCLCPP_DEBUG(get_logger(), "TF unavailable: %s", ex.what());
      have_past_ = false;  // 끊기면 과거 비교 리셋하는 게 안전
      return;
    }

    // tf2::Transform으로 변환해 stamp 포함해서 처리 (성공 코드 방식)
    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);

    const rclcpp::Time now = this->get_clock()->now();
    if(now.nanoseconds() == 0){
      // sim time인데 /clock 아직 안 들어오면 now=0일 수 있음
      return;
    }

    const auto elapsed_ns =
      now.nanoseconds() - tf.stamp_.time_since_epoch().count();

    const auto filtering_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(filtering_time_)
      ).count();

    if(elapsed_ns > filtering_ns){
      // 오래된 TF는 버림
      have_past_ = false;
      return;
    }

    // 첫 샘플이면 past 세팅하고 끝
    if(!have_past_){
      tf_past_ = tf;
      stop_start_time_ = now;
      have_past_ = true;
      return;
    }

    // 이동량 계산
    const double diff = (tf_past_.getOrigin() - tf.getOrigin()).length();

    if(diff < distance_threshold_){
      // 거의 움직이지 않음 → stop 유지시간 체크
      const double stop_sec = (now - stop_start_time_).seconds();
      if(stop_sec >= stop_time_threshold_){
        // pick 트리거
        tf2::Vector3 p = tf.getOrigin();
        if(p.z() < target_z_min_) p.setZ(target_z_min_);
        busy_ = true;
        picking(p);
        busy_ = false;

        // pick 후에는 과거 상태 리셋
        have_past_ = false;
      }
    }else{
      // 움직이면 past 갱신 + stop timer 리셋
      tf_past_ = tf;
      stop_start_time_ = now;
    }
  }

  void move_to_xyz_keep_orientation(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped tgt;
    tgt.header.frame_id = base_frame_;
    tgt.header.stamp = this->get_clock()->now();

    tgt.pose.position.x = x;
    tgt.pose.position.y = y;
    tgt.pose.position.z = z;

    // 너 코드처럼 orientation 고정(현재자세 유지)
    tgt.pose.orientation = arm_->getCurrentPose().pose.orientation;

    arm_->setPoseTarget(tgt);
    arm_->move();
  }

  void picking(const tf2::Vector3& target_position)
  {
    RCLCPP_INFO(get_logger(), ">> PICK start (%.3f %.3f %.3f)",
                target_position.x(), target_position.y(), target_position.z());

    // (선택) 혹시 뭔가 잡고 있던 상황 대비: 열고-닫기-열기
    operateGripperBlocking(shared_from_this(), gripper_, grip_open_,  grip_effort_);
    rclcpp::sleep_for(150ms);
    operateGripperBlocking(shared_from_this(), gripper_, grip_close_, grip_effort_);
    rclcpp::sleep_for(150ms);
    operateGripperBlocking(shared_from_this(), gripper_, grip_open_,  grip_effort_);
    rclcpp::sleep_for(150ms);

    // 1) hover
    move_to_xyz_keep_orientation(
      target_position.x(),
      target_position.y(),
      target_position.z() + hover_z_);

    // 2) open
    operateGripperBlocking(shared_from_this(), gripper_, grip_open_, grip_effort_);

    // 3) grasp down
    move_to_xyz_keep_orientation(
      target_position.x(),
      target_position.y(),
      target_position.z() + grasp_z_);

    // 4) close
    operateGripperBlocking(shared_from_this(), gripper_, grip_close_, grip_effort_);

    // 5) lift
    move_to_xyz_keep_orientation(
      target_position.x(),
      target_position.y(),
      target_position.z() + lift_z_);

    // 6) move to place (위로 들고 이동)
    move_to_xyz_keep_orientation(place_x_, place_y_, 0.20);

    // 7) 내려놓기
    move_to_xyz_keep_orientation(place_x_, place_y_, place_z_);

    // 8) open
    operateGripperBlocking(shared_from_this(), gripper_, grip_open_, grip_effort_);

    // 9) 살짝 상승
    move_to_xyz_keep_orientation(place_x_, place_y_, 0.20);

    // 10) home
    arm_->setNamedTarget("home");
    arm_->move();

    // 11) 기본 상태(열어둠)
    operateGripperBlocking(shared_from_this(), gripper_, grip_open_, grip_effort_);

    RCLCPP_INFO(get_logger(), ">> PICK done");
  }

private:
  // params
  std::string base_frame_;
  std::string marker_frame_;

  double filtering_time_;
  double stop_time_threshold_;
  double distance_threshold_;
  double target_z_min_;

  double hover_z_;
  double grasp_z_;
  double lift_z_;

  double place_x_, place_y_, place_z_;
  double vel_scale_, acc_scale_;

  double grip_open_, grip_close_, grip_effort_;

  // components
  std::shared_ptr<MoveGroupInterface> arm_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  // state
  bool have_past_{false};
  bool busy_{false};
  tf2::Stamped<tf2::Transform> tf_past_;
  rclcpp::Time stop_start_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // MoveIt node는 성공 예제처럼 따로 두는 게 종종 안정적
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);

  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", opts);
  auto picker_node = std::make_shared<OmxPickTfStop>(move_group_arm_node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(picker_node);
  exec.add_node(move_group_arm_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}