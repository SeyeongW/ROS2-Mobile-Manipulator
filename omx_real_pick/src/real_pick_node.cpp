// omx_real_picker_tf_stop_improved_keep_values.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <algorithm>
#include <memory>

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

class OmxRealPickerStopBased : public rclcpp::Node
{
public:
  OmxRealPickerStopBased()
  : Node("omx_real_picker_tf_stop_based")
  {
    // -----------------------------
    // use_sim_time: 중복 선언 방지
    // -----------------------------
    if(!this->has_parameter("use_sim_time")){
      this->declare_parameter<bool>("use_sim_time", true);
    }
    bool use_sim_time=true;
    this->get_parameter("use_sim_time", use_sim_time);
    RCLCPP_INFO(get_logger(), "use_sim_time=%s", use_sim_time ? "true" : "false");

    // -----------------------------
    // TF
    // -----------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // -----------------------------
    // MoveIt (네 코드 값 유지)
    // -----------------------------
    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");

    arm_->setMaxVelocityScalingFactor(0.15);
    arm_->setMaxAccelerationScalingFactor(0.15);
    arm_->setPlanningTime(5.0);
    arm_->setGoalPositionTolerance(0.03);
    arm_->setGoalOrientationTolerance(3.14);

    // Frames (네 코드 값 유지)
    base_frame_   = this->declare_parameter<std::string>("base_frame", "link1");
    marker_frame_ = this->declare_parameter<std::string>("marker_frame", "aruco_marker_23");
    arm_->setPoseReferenceFrame(base_frame_);

    RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "Pose reference frame: %s", base_frame_.c_str());

    // -----------------------------
    // Gripper (네 코드 값 유지)
    // -----------------------------
    gripper_ = rclcpp_action::create_client<GripperCommand>(
      shared_from_this(), "/gripper_controller/gripper_cmd");

    GRIP_OPEN_   = this->declare_parameter<double>("grip_open", 0.019);
    GRIP_CLOSE_  = this->declare_parameter<double>("grip_close", 0.0);
    GRIP_EFFORT_ = this->declare_parameter<double>("grip_effort", 30.0);

    // -----------------------------
    // SEARCH waypoints (네 코드 값 유지)
    // -----------------------------
    waypoints_ = {
      { 0.00, -0.20,  0.20,  0.80},
      { 1.00, -0.20,  0.20,  0.80},
      {-1.00, -0.20,  0.20,  0.80},
      { 0.00, -0.60,  0.30,  1.20}
    };

    // -----------------------------
    // Approach params (네 코드 값 유지)
    // -----------------------------
    hover_offset_ = this->declare_parameter<double>("hover_offset", 0.15);
    step_         = this->declare_parameter<double>("step", 0.03);
    final_min_z_  = this->declare_parameter<double>("final_min_z", 0.04);

    // TF age (네 코드 유지)
    max_tf_age_sec_ = this->declare_parameter<double>("max_tf_age_sec", 0.7);

    // -----------------------------
    // “성공 코드 방식” 정지 판정 파라미터 (추가)
    // -----------------------------
    filtering_time_sec_      = this->declare_parameter<double>("filtering_time_sec", 2.0);
    stop_time_threshold_sec_ = this->declare_parameter<double>("stop_time_threshold_sec", 1.0);
    // ↑ 기본값 1.0초로 둔 이유:
    //   CRANE는 3초 정지 후 pick인데, 네 환경에서 너무 느려질 수 있어서 기본은 1초.
    //   필요하면 launch에서 2~3초로 올려.
    distance_threshold_m_    = this->declare_parameter<double>("distance_threshold_m", 0.01);

    // Median filter (네 코드 유지)
    FILTER_N_ = (size_t)this->declare_parameter<int>("median_filter_N", 7);

    // -----------------------------
    // Init (네 코드 유지)
    // -----------------------------
    RCLCPP_INFO(get_logger(), ">> HOME");
    arm_->setNamedTarget("home");
    arm_->move();
    rclcpp::sleep_for(600ms);

    operateGripperBlocking(shared_from_this(), gripper_, GRIP_OPEN_, GRIP_EFFORT_);

    // state
    state_ = FSM::SEARCH;
    wp_idx_ = 0;
    busy_ = false;

    have_prev_tf_ = false;
    stop_start_time_ = this->get_clock()->now();

    current_offset_ = hover_offset_;

    // timer (성공 코드 방식)
    timer_ = this->create_wall_timer(200ms, std::bind(&OmxRealPickerStopBased::tick, this));
  }

private:
  enum class FSM { SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME };

  bool getMarkerFiltered(double& out_x, double& out_y, double& out_z, bool& out_fresh)
  {
    out_fresh = false;

    geometry_msgs::msg::TransformStamped tf_msg;
    try{
      // 최신값(TimePointZero) 사용
      tf_msg = tf_buffer_->lookupTransform(base_frame_, marker_frame_, tf2::TimePointZero);
    }catch(const tf2::TransformException&){
      return false;
    }

    const rclcpp::Time now = this->get_clock()->now();
    if(now.nanoseconds() == 0){
      // sim time인데 /clock 아직 안 온 상태
      return false;
    }

    const rclcpp::Time st(tf_msg.header.stamp);
    const double age = (now - st).seconds();
    if(age > max_tf_age_sec_) return false;

    // filtering_time_sec_ 내 TF만 인정 (성공 코드 방식)
    if(age > filtering_time_sec_) return false;

    const double x = tf_msg.transform.translation.x;
    const double y = tf_msg.transform.translation.y;
    const double z = tf_msg.transform.translation.z;
    if(!isFinite(x)||!isFinite(y)||!isFinite(z)) return false;

    out_x = medianPush(fx_, x, FILTER_N_);
    out_y = medianPush(fy_, y, FILTER_N_);
    out_z = medianPush(fz_, z, FILTER_N_);

    out_fresh = true;
    return true;
  }

  bool isStoppedForLongEnough(double x, double y, double z)
  {
    const rclcpp::Time now = this->get_clock()->now();

    if(!have_prev_tf_){
      prev_x_ = x; prev_y_ = y; prev_z_ = z;
      stop_start_time_ = now;
      have_prev_tf_ = true;
      return false;
    }

    const double dx = x - prev_x_;
    const double dy = y - prev_y_;
    const double dz = z - prev_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if(dist < distance_threshold_m_){
      const double stopped_sec = (now - stop_start_time_).seconds();
      return (stopped_sec >= stop_time_threshold_sec_);
    }else{
      prev_x_ = x; prev_y_ = y; prev_z_ = z;
      stop_start_time_ = now;
      return false;
    }
  }

  void setPoseXYZKeepOri(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped tgt;
    tgt.header.frame_id = base_frame_;
    tgt.header.stamp = this->get_clock()->now();
    tgt.pose.position.x = x;
    tgt.pose.position.y = y;
    tgt.pose.position.z = z;

    tgt.pose.orientation = arm_->getCurrentPose().pose.orientation; // 네 방식 유지
    arm_->setPoseTarget(tgt);
    arm_->move();
  }

  void tick()
  {
    if(busy_) return;           // pick 동작 중 재진입 방지
    rclcpp::spin_some(shared_from_this());

    double mx=0,my=0,mz=0;
    bool fresh=false;
    bool ok = getMarkerFiltered(mx,my,mz,fresh);

    const bool visible = ok && fresh;

    switch(state_){
      case FSM::SEARCH: {
        // 필터 큐 리셋
        fx_.clear(); fy_.clear(); fz_.clear();
        have_prev_tf_ = false;

        if(visible){
          state_ = FSM::STABILIZE;
          RCLCPP_INFO(get_logger(), ">> Seen -> STABILIZE(stop-check)");
          break;
        }

        RCLCPP_INFO(get_logger(), ">> SEARCH waypoint %d", wp_idx_);
        arm_->setJointValueTarget(waypoints_[wp_idx_]);
        arm_->move();
        wp_idx_ = (wp_idx_ + 1) % (int)waypoints_.size();
      } break;

      case FSM::STABILIZE: {
        if(!visible){
          state_ = FSM::SEARCH;
          RCLCPP_WARN(get_logger(), ">> Lost -> SEARCH");
          break;
        }

        // 성공 코드 방식: “정지 시간” 만족하면 접근 시작
        const bool stopped = isStoppedForLongEnough(mx,my,mz);
        RCLCPP_INFO(get_logger(), ">> STABILIZE stop=%s (%.3f %.3f %.3f)",
                    stopped ? "YES" : "no", mx,my,mz);

        if(stopped){
          current_offset_ = hover_offset_;
          state_ = FSM::APPROACH;
          RCLCPP_INFO(get_logger(), ">> Stop confirmed -> APPROACH");
        }
      } break;

      case FSM::APPROACH: {
        if(!visible){
          state_ = FSM::STABILIZE;
          have_prev_tf_ = false;
          RCLCPP_WARN(get_logger(), ">> Lost -> STABILIZE");
          break;
        }

        // 네 기존 접근 방식 유지
        const double target_z = std::max(final_min_z_, mz + current_offset_);

        RCLCPP_INFO(get_logger(), ">> APPROACH off=%.3f tgt=(%.3f %.3f %.3f)",
                    current_offset_, mx, my, target_z);

        setPoseXYZKeepOri(mx, my, target_z);

        current_offset_ = std::max(0.0, current_offset_ - step_);

        if(current_offset_ <= 1e-4){
          state_ = FSM::GRASP;
          RCLCPP_INFO(get_logger(), ">> Reached -> GRASP");
        }
      } break;

      case FSM::GRASP: {
        busy_ = true;
        if(!operateGripperBlocking(shared_from_this(), gripper_, GRIP_CLOSE_, GRIP_EFFORT_)){
          RCLCPP_ERROR(get_logger(), "Gripper close failed");
        }
        busy_ = false;
        state_ = FSM::LIFT_AND_HOME;
      } break;

      case FSM::LIFT_AND_HOME: {
        busy_ = true;

        auto cur = arm_->getCurrentPose();
        cur.header.frame_id = base_frame_;
        cur.pose.position.z += 0.10;
        arm_->setPoseTarget(cur);
        arm_->move();

        arm_->setNamedTarget("home");
        arm_->move();

        operateGripperBlocking(shared_from_this(), gripper_, GRIP_OPEN_, GRIP_EFFORT_);

        busy_ = false;

        state_ = FSM::SEARCH;
      } break;
    }
  }

private:
  // MoveIt/TF
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // frames
  std::string base_frame_;
  std::string marker_frame_;

  // params (keep)
  double hover_offset_;
  double step_;
  double final_min_z_;
  double max_tf_age_sec_;

  // stop-based params (new)
  double filtering_time_sec_;
  double stop_time_threshold_sec_;
  double distance_threshold_m_;

  // gripper params (keep)
  double GRIP_OPEN_;
  double GRIP_CLOSE_;
  double GRIP_EFFORT_;

  // search
  std::vector<std::vector<double>> waypoints_;
  int wp_idx_{0};

  // median filter queues (keep)
  size_t FILTER_N_;
  std::deque<double> fx_, fy_, fz_;

  // stop detect state
  bool have_prev_tf_{false};
  double prev_x_{0}, prev_y_{0}, prev_z_{0};
  rclcpp::Time stop_start_time_;

  // FSM state
  enum class FSM { SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME };
  FSM state_{FSM::SEARCH};

  bool busy_{false};
  double current_offset_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OmxRealPickerStopBased>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}