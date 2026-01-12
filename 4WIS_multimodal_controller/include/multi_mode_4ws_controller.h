#ifndef MULTI_MODE_4WS_CONTROLLER_H
#define MULTI_MODE_4WS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tfMessage.h>
#include <std_msgs/UInt8.h>

#include <boost/circular_buffer.hpp>
#include <ros/ros.h>

namespace multi_mode_4ws_controller
{

/**
 * @brief 运动模态枚举
 */
enum MotionMode
{
  MODE_ACKERMANN = 0,        // 前后阿克曼
  MODE_LATERAL_ACKERMANN = 1,          // 四轮同向平移
  MODE_PARALLEL = 2,// 横向阿克曼
  MODE_SPIN = 3              // 原地自转
};

/**
 * @brief 速度限制器
 */
class SpeedLimiter
{
public:
  bool has_velocity_limits;
  bool has_acceleration_limits;
  double min_velocity;
  double max_velocity;
  double min_acceleration;
  double max_acceleration;
  

  SpeedLimiter()
    : has_velocity_limits(false)
    , has_acceleration_limits(false)
    , min_velocity(0.0)
    , max_velocity(0.0)
    , min_acceleration(0.0)
    , max_acceleration(0.0)
  {}

  double limit(double& v, double v0, double dt);
  double limit_velocity(double& v);
  double limit_acceleration(double& v, double v0, double dt);
};

/**
 * @brief 里程计类
 */
class Odometry
{
public:
  Odometry(size_t velocity_rolling_window_size = 10);
  
  void init(const ros::Time& time);
  bool update(double fl_vel, double fr_vel, double rl_vel, double rr_vel,
              double front_steering, double rear_steering, const ros::Time& time);
  
  void setWheelParams(double track, double wheel_radius, double wheel_base);
  
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

private:
  void integrateXY(double linear_x, double linear_y, double angular);
  void integrateRungeKutta2(double linear, double angular);
  
  ros::Time timestamp_;
  double x_;
  double y_;
  double heading_;
  double linear_;
  double linear_x_;
  double linear_y_;
  double angular_;
  
  double track_;
  double wheel_radius_;
  double wheel_base_;
  
  size_t velocity_rolling_window_size_;
  boost::circular_buffer<double> linear_acc_;
  boost::circular_buffer<double> angular_acc_;
};

/**
 * @brief 多模态控制命令
 */
struct MultiModeCommand
{
  uint8_t mode;
  double vx;
  double wz;
  ros::Time stamp;
  
  MultiModeCommand() 
    : mode(MODE_ACKERMANN), vx(0.0), wz(0.0), stamp(0.0) {}
};

/**
 * @brief 多模态四轮转向控制器
 * 
 * 注意：这个控制器需要两种硬件接口：
 * - VelocityJointInterface: 用于驱动轮
 * - PositionJointInterface: 用于转向轮
 */
class MultiMode4WSController 
  : public controller_interface::MultiInterfaceController<
      hardware_interface::VelocityJointInterface,
      hardware_interface::PositionJointInterface>
{
public:
  MultiMode4WSController();
  ~MultiMode4WSController();

  bool init(hardware_interface::RobotHW* hw, 
            ros::NodeHandle& root_nh, 
            ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  // 回调函数
  void cmdVelCallback(const geometry_msgs::Twist& msg);
  void modeSelectCallback(const std_msgs::UInt8& msg);
  
  // 辅助函数
  bool getWheelNames(ros::NodeHandle& nh, const std::string& param,
                     std::vector<std::string>& names);
  void brake();
  void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  
  // 模态运动学计算
  void computeWheelCommands(const MultiModeCommand& cmd,
                           double& vl_f, double& vr_f, double& vl_r, double& vr_r,
                           double& sl_f, double& sr_f, double& sl_r, double& sr_r);
  
  void modeAckermann(const MultiModeCommand& cmd,
                    double& vl_f, double& vr_f, double& vl_r, double& vr_r,
                    double& sl_f, double& sr_f, double& sl_r, double& sr_r);
  
  void modeParallel(const MultiModeCommand& cmd,
                  double& vl_f, double& vr_f, double& vl_r, double& vr_r,
                  double& sl_f, double& sr_f, double& sl_r, double& sr_r);
  
  void modeLateralAckermann(const MultiModeCommand& cmd,
                           double& vl_f, double& vr_f, double& vl_r, double& vr_r,
                           double& sl_f, double& sr_f, double& sl_r, double& sr_r);
  
  void modeSpin(const MultiModeCommand& cmd,
               double& vl_f, double& vr_f, double& vl_r, double& vr_r,
               double& sl_f, double& sr_f, double& sl_r, double& sr_r);
  
  void updateOdometry(const ros::Time& time);
  void publishOdometry(const ros::Time& time);

  // 硬件接口
  std::vector<hardware_interface::JointHandle> front_wheel_joints_;
  std::vector<hardware_interface::JointHandle> rear_wheel_joints_;
  
  // 需要位置接口的转向关节
  std::vector<hardware_interface::JointHandle> front_steer_joints_;
  std::vector<hardware_interface::JointHandle> rear_steer_joints_;
  
  // ROS通信
  ros::Subscriber sub_command_;
  ros::Subscriber sub_mode_;
  
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tf_odom_pub_;
  
  // 命令缓冲
  realtime_tools::RealtimeBuffer<MultiModeCommand> command_;
  MultiModeCommand command_struct_;
  
  // 里程计
  Odometry odometry_;
  ros::Time last_odom_publish_time_;
  ros::Duration publish_period_;
  
  // 速度限制器
  SpeedLimiter limiter_lin_1;
  SpeedLimiter limiter_ang_1;
  SpeedLimiter limiter_lin_2;
  SpeedLimiter limiter_ang_2;
  SpeedLimiter limiter_lin_3;
  SpeedLimiter limiter_ang_3;
  SpeedLimiter limiter_lin_4;
  SpeedLimiter limiter_ang_4;
  MultiModeCommand last_cmd_;

  // 车辆参数
  double track_;           // 轮距
  double wheel_radius_;    // 轮子半径
  double wheel_base_;      // 轴距

  // 控制参数
  double cmd_vel_timeout_;
  std::string base_frame_id_;
  bool enable_odom_tf_;
  uint8_t current_mode_;

  
  std::string name_;
  bool open_loop_;
};

} // namespace multi_mode_4ws_controller

#endif // MULTI_MODE_4WS_CONTROLLER_H