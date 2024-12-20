#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp> // 操作角による速度制限
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> // by ChatGPT
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::PoseWithCovarianceStamped;  //by ChatGPT
using autoware_auto_vehicle_msgs::msg::SteeringReport;  // 操作角による速度制限
using nav_msgs::msg::Odometry;

class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_pose_with_covariance_;//by ChatGPT
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_status_;  // 操作角による速度制限
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;  

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;
  PoseWithCovarianceStamped::SharedPtr pose_with_covariance_;//by ChatGPT
  SteeringReport::SharedPtr steering_status_;  // 操作角による速度制限
  OnSetParametersCallbackHandle::SharedPtr  param_handler_;  // RTPC

  // pure pursuit parameters
  const double wheel_base_;
  double lookahead_gain_;
  double lookahead_gain2_; // 先読み用
  double lookahead_min_distance_;
  double lookahead_min_distance2_;  // 先読み用
  const double speed_proportional_gain_;
  double acceleration_offset_;  //  速度維持用オフセット
  const double steering_diff_gain_;  // 操舵制御用 未使用
  bool use_external_target_vel_;
  double map_vel_gain_;
  double external_target_vel_;  // RTPC
  bool use_steer_angle_v_limit_;
  const double predict_time_v_limit_; //  速度制限用先読み時間
  double v_limit_angle_;
  const double v_limit_angle2_; // 未使用
  double angle_limit_v_;
  const double angle_limit_v2_; // 未使用
  double predict_time_;
  double steering_tire_angle_gain_;


 private:
  void onTimer();
  bool subscribeMessageAvailable();
//  std::shared_ptr<rclcpp::ParameterEventHandler>  param_subscriber_;  //  RTPC
//  std::shared_ptr<rclcpp::ParameterCallbackHandle>  cb_handle_;       //  RTPC
//  double last_steering_angle; //  微分操舵制御用
  int dbg_cnt;  //  テスト用
  double test_x;
  double test_y;
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
