#include <multi_mode_4ws_controller.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>

namespace multi_mode_4ws_controller
{

// ============================================================================
// SpeedLimiter Implementation
// ============================================================================

double SpeedLimiter::limit(double& v, double v0, double dt)
{
  const double tmp = v;
  limit_acceleration(v, v0, dt);
  limit_velocity(v);
  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_velocity(double& v)
{
  const double tmp = v;
  if (has_velocity_limits)
  {
    v = std::max(std::min(v, max_velocity), min_velocity);
  }
  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
{
  const double tmp = v;
  if (has_acceleration_limits)
  {
    const double dv_min = min_acceleration * dt;
    const double dv_max = max_acceleration * dt;
    const double dv = std::max(std::min(v - v0, dv_max), dv_min);
    v = v0 + dv;
  }
  return tmp != 0.0 ? v / tmp : 1.0;
}

// ============================================================================
// Odometry Implementation
// ============================================================================

Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , track_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(velocity_rolling_window_size)
  , angular_acc_(velocity_rolling_window_size)
{
}

void Odometry::init(const ros::Time& time)
{
  timestamp_ = time;
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  linear_ = 0.0;
  linear_x_ = 0.0;
  linear_y_ = 0.0;
  angular_ = 0.0;
}

void Odometry::setWheelParams(double track, double wheel_radius, double wheel_base)
{
  track_ = track;
  wheel_radius_ = wheel_radius;
  wheel_base_ = wheel_base;
}

bool Odometry::update(double fl_vel, double fr_vel, double rl_vel, double rr_vel,
                      double front_steering, double rear_steering, const ros::Time& time)
{
  const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001)
    return false;
  
  timestamp_ = time;
  
  // 简化的运动学模型：基于轮速和转向角估算车体速度
  // 前轮平均速度
  const double front_vel = (fl_vel + fr_vel) / 2.0 * wheel_radius_;
  const double rear_vel = (rl_vel + rr_vel) / 2.0 * wheel_radius_;
  
  // 估算车体速度
  linear_x_ = (front_vel * cos(front_steering) + rear_vel * cos(rear_steering)) / 2.0;
  linear_y_ = (front_vel * sin(front_steering) + rear_vel * sin(rear_steering)) / 2.0;
  
  // 计算角速度（简化）
  if (fabs(front_steering - rear_steering) > 0.001)
  {
    angular_ = (front_vel * tan(front_steering) - rear_vel * tan(rear_steering)) / wheel_base_;
  }
  else
  {
    angular_ = 0.0;
  }
  
  linear_ = sqrt(linear_x_ * linear_x_ + linear_y_ * linear_y_);
  
  // 积分位姿
  integrateXY(linear_x_ * dt, linear_y_ * dt, angular_ * dt);
  
  return true;
}

void Odometry::integrateXY(double linear_x, double linear_y, double angular)
{
  const double delta_x = linear_x * cos(heading_) - linear_y * sin(heading_);
  const double delta_y = linear_x * sin(heading_) + linear_y * cos(heading_);
  
  x_ += delta_x;
  y_ += delta_y;
  heading_ += angular;
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

// ============================================================================
// MultiMode4WSController Implementation
// ============================================================================

MultiMode4WSController::MultiMode4WSController()
  : odometry_(10)
  , track_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , cmd_vel_timeout_(0.5)
  , base_frame_id_("base_link")
  , enable_odom_tf_(true)
  , current_mode_(MODE_ACKERMANN)
  , open_loop_(false)
{
}

MultiMode4WSController::~MultiMode4WSController()
{
  sub_command_.shutdown();
  sub_mode_.shutdown();
}

bool MultiMode4WSController::getWheelNames(ros::NodeHandle& nh, 
                                           const std::string& param,
                                           std::vector<std::string>& names)
{
  XmlRpc::XmlRpcValue wheel_list;
  if (!nh.getParam(param, wheel_list))
  {
    ROS_ERROR_STREAM("Failed to get parameter: " << param);
    return false;
  }
  
  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (wheel_list.size() == 0)
    {
      ROS_ERROR_STREAM("Parameter " << param << " is empty");
      return false;
    }
    
    names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); ++i)
    {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Parameter " << param << "[" << i << "] is not a string");
        return false;
      }
      names[i] = static_cast<std::string>(wheel_list[i]);
    }
  }
  else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    names.push_back(wheel_list);
  }
  else
  {
    ROS_ERROR_STREAM("Parameter " << param << " has wrong type");
    return false;
  }
  
  return true;
}

bool MultiMode4WSController::init(hardware_interface::RobotHW* hw,
                                   ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);
  
  // 获取关节名称
  std::vector<std::string> front_wheel_names, rear_wheel_names;
  std::vector<std::string> front_steer_names, rear_steer_names;
  
  if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
      !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names))
  {
    return false;
  }
  
  if (!getWheelNames(controller_nh, "front_steering", front_steer_names) ||
      !getWheelNames(controller_nh, "rear_steering", rear_steer_names))
  {
    return false;
  }
  
  if (front_wheel_names.size() != 2 || rear_wheel_names.size() != 2 ||
      front_steer_names.size() != 2 || rear_steer_names.size() != 2)
  {
    ROS_ERROR_STREAM(name_ << ": Need exactly 2 wheels and 2 steering joints per axle");
    return false;
  }
  
  // 获取车辆参数
  if (!controller_nh.getParam("wheel_base", wheel_base_))
  {
    ROS_ERROR_STREAM(name_ << ": wheel_base parameter required");
    return false;
  }
  
  if (!controller_nh.getParam("track", track_))
  {
    ROS_ERROR_STREAM(name_ << ": track parameter required");
    return false;
  }
  
  if (!controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR_STREAM(name_ << ": wheel_radius parameter required");
    return false;
  }
  
  // 其他参数
  double publish_rate = 50.0;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  publish_period_ = ros::Duration(1.0 / publish_rate);
  
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  controller_nh.param("open_loop", open_loop_, open_loop_);

  int velocity_rolling_window_size = 10;
  controller_nh.param("velocity_rolling_window_size", 
                     velocity_rolling_window_size, velocity_rolling_window_size);
  
  // 速度限制
  controller_nh.param("linear/MODE_ACKERMANN/x/has_velocity_limits", limiter_lin_1.has_velocity_limits, false);
  controller_nh.param("linear/MODE_LATERAL_ACKERMANN/x/has_velocity_limits", limiter_lin_2.has_velocity_limits, false);
  controller_nh.param("linear/MODE_PARALLEL/x/has_velocity_limits", limiter_lin_3.has_velocity_limits, false);
  controller_nh.param("linear/MODE_SPIN/x/has_velocity_limits", limiter_lin_4.has_velocity_limits, false);


  controller_nh.param("linear/MODE_ACKERMANN/x/has_acceleration_limits", limiter_lin_1.has_acceleration_limits, false);
  controller_nh.param("linear/MODE_LATERAL_ACKERMANN/x/has_acceleration_limits", limiter_lin_2.has_acceleration_limits, false);
  controller_nh.param("linear/MODE_PARALLEL/x/has_acceleration_limits", limiter_lin_3.has_acceleration_limits, false);
  controller_nh.param("linear/MODE_SPIN/x/has_acceleration_limits", limiter_lin_4.has_acceleration_limits, false);


  controller_nh.param("linear/MODE_ACKERMANN/x/max_velocity", limiter_lin_1.max_velocity, 1.0);
  controller_nh.param("linear/MODE_ACKERMANN/x/min_velocity", limiter_lin_1.min_velocity, -1.0);
  controller_nh.param("linear/MODE_LATERAL_ACKERMANN/x/max_velocity", limiter_lin_2.max_velocity, 1.0);
  controller_nh.param("linear/MODE_LATERAL_ACKERMANN/x/min_velocity", limiter_lin_2.min_velocity, -1.0);
  controller_nh.param("linear/MODE_PARALLEL/x/max_velocity", limiter_lin_3.max_velocity, 1.0);
  controller_nh.param("linear/MODE_PARALLEL/x/min_velocity", limiter_lin_3.min_velocity, -1.0);
  controller_nh.param("linear/MODE_SPIN/x/max_velocity", limiter_lin_4.max_velocity, 1.0);
  controller_nh.param("linear/MODE_SPIN/x/min_velocity", limiter_lin_4.min_velocity, -1.0);


  controller_nh.param("linear/MODE_ACKERMANN/x/max_acceleration", limiter_lin_1.max_acceleration, 1.0);
  controller_nh.param("linear/MODE_ACKERMANN/x/min_acceleration", limiter_lin_1.min_acceleration, -1.0);
  controller_nh.param("linear/MODE_LATERAL_ACKERMANN/x/max_acceleration", limiter_lin_2.max_acceleration, 1.0);
  controller_nh.param("linear/MODE_LATERAL_ACKERMANN/x/min_acceleration", limiter_lin_2.min_acceleration, -1.0);
  controller_nh.param("linear/MODE_PARALLEL/x/max_acceleration", limiter_lin_3.max_acceleration, 1.0);
  controller_nh.param("linear/MODE_PARALLEL/x/min_acceleration", limiter_lin_3.min_acceleration, -1.0);
  controller_nh.param("linear/MODE_SPIN/x/max_acceleration", limiter_lin_4.max_acceleration, 1.0);
  controller_nh.param("linear/MODE_SPIN/x/min_acceleration", limiter_lin_4.min_acceleration, -1.0);


  // 转角限制
  controller_nh.param("angular/MODE_ACKERMANN/z/has_velocity_limits", limiter_ang_1.has_velocity_limits, false);
  controller_nh.param("angular/MODE_LATERAL_ACKERMANN/z/has_velocity_limits", limiter_ang_2.has_velocity_limits, false);
  controller_nh.param("angular/MODE_PARALLEL/z/has_velocity_limits", limiter_ang_3.has_velocity_limits, false);
  controller_nh.param("angular/MODE_SPIN/z/has_velocity_limits", limiter_ang_4.has_velocity_limits, false);


  controller_nh.param("angular/MODE_ACKERMANN/z/has_acceleration_limits", limiter_ang_1.has_acceleration_limits, false);
  controller_nh.param("angular/MODE_LATERAL_ACKERMANN/z/has_acceleration_limits", limiter_ang_2.has_acceleration_limits, false);
  controller_nh.param("angular/MODE_PARALLEL/z/has_acceleration_limits", limiter_ang_3.has_acceleration_limits, false);
  controller_nh.param("angular/MODE_SPIN/z/has_acceleration_limits", limiter_ang_4.has_acceleration_limits, false);


  controller_nh.param("angular/MODE_ACKERMANN/z/max_velocity", limiter_ang_1.max_velocity, 1.0);
  controller_nh.param("angular/MODE_ACKERMANN/z/min_velocity", limiter_ang_1.min_velocity, -1.0);
  controller_nh.param("angular/MODE_LATERAL_ACKERMANN/z/max_velocity", limiter_ang_2.max_velocity, 1.0);
  controller_nh.param("angular/MODE_LATERAL_ACKERMANN/z/min_velocity", limiter_ang_2.min_velocity, -1.0);
  controller_nh.param("angular/MODE_PARALLEL/z/max_velocity", limiter_ang_3.max_velocity, 1.0);
  controller_nh.param("angular/MODE_PARALLEL/z/min_velocity", limiter_ang_3.min_velocity, -1.0);
  controller_nh.param("angular/MODE_SPIN/z/max_velocity", limiter_ang_4.max_velocity, 1.0);
  controller_nh.param("angular/MODE_SPIN/z/min_velocity", limiter_ang_4.min_velocity, -1.0);


  controller_nh.param("angular/MODE_ACKERMANN/z/max_acceleration", limiter_ang_1.max_acceleration, 1.0);
  controller_nh.param("angular/MODE_ACKERMANN/z/min_acceleration", limiter_ang_1.min_acceleration, -1.0);
  controller_nh.param("angular/MODE_LATERAL_ACKERMANN/z/max_acceleration", limiter_ang_2.max_acceleration, 1.0);
  controller_nh.param("angular/MODE_LATERAL_ACKERMANN/z/min_acceleration", limiter_ang_2.min_acceleration, -1.0);
  controller_nh.param("angular/MODE_PARALLEL/z/max_acceleration", limiter_ang_3.max_acceleration, 1.0);
  controller_nh.param("angular/MODE_PARALLEL/z/min_acceleration", limiter_ang_3.min_acceleration, -1.0);
  controller_nh.param("angular/MODE_SPIN/z/max_acceleration", limiter_ang_4.max_acceleration, 1.0);
  controller_nh.param("angular/MODE_SPIN/z/min_acceleration", limiter_ang_4.min_acceleration, -1.0);

 
  
  // 设置里程计参数
  odometry_.setWheelParams(track_, wheel_radius_, wheel_base_);
  
  // 获取硬件接口
  hardware_interface::VelocityJointInterface* vel_joint_hw = 
    hw->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::PositionJointInterface* pos_joint_hw = 
    hw->get<hardware_interface::PositionJointInterface>();
  
  if (!vel_joint_hw)
  {
    ROS_ERROR_STREAM(name_ << ": Could not get VelocityJointInterface");
    return false;
  }
  
  if (!pos_joint_hw)
  {
    ROS_ERROR_STREAM(name_ << ": Could not get PositionJointInterface");
    return false;
  }
  
  front_wheel_joints_.resize(2);
  rear_wheel_joints_.resize(2);
  front_steer_joints_.resize(2);
  rear_steer_joints_.resize(2);
  
  try
  {
    for (size_t i = 0; i < 2; ++i)
    {
      // 驱动轮使用速度接口
      front_wheel_joints_[i] = vel_joint_hw->getHandle(front_wheel_names[i]);
      rear_wheel_joints_[i] = vel_joint_hw->getHandle(rear_wheel_names[i]);
      
      // 转向轮使用位置接口
      front_steer_joints_[i] = pos_joint_hw->getHandle(front_steer_names[i]);
      rear_steer_joints_[i] = pos_joint_hw->getHandle(rear_steer_names[i]);
    }
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM(name_ << ": " << e.what());
    return false;
  }
  
  // 设置发布器
  setOdomPubFields(root_nh, controller_nh);
  
  // 订阅命令
  sub_command_ = controller_nh.subscribe("cmd_vel", 1, 
                                        &MultiMode4WSController::cmdVelCallback, this);
  sub_mode_ = controller_nh.subscribe("mode_select", 1,
                                     &MultiMode4WSController::modeSelectCallback, this);
  
  ROS_INFO_STREAM(name_ << ": Initialized successfully");
  ROS_INFO_STREAM("  Wheel base: " << wheel_base_);
  ROS_INFO_STREAM("  Track: " << track_);
  ROS_INFO_STREAM("  Wheel radius: " << wheel_radius_);
  ROS_INFO_STREAM("  Supported modes: 0=Ackermann, 1=Lateral, 2=Lateral-Ack, 3=Spin");
  
  return true;
}

void MultiMode4WSController::starting(const ros::Time& time)
{
  brake();
  odometry_.init(time);
  last_odom_publish_time_ = time;
  
  command_struct_ = MultiModeCommand();
  command_.writeFromNonRT(command_struct_);
}

void MultiMode4WSController::stopping(const ros::Time& /*time*/)
{
  brake();
}

void MultiMode4WSController::update(const ros::Time& time, const ros::Duration& period)
{
  // 更新里程计
  updateOdometry(time);
  
  // 获取当前命令
  MultiModeCommand cmd = *(command_.readFromRT());
  
  // 检查超时
  const double dt = (time - cmd.stamp).toSec();
  if (dt > cmd_vel_timeout_)
  {
    cmd.vx = 0.0;
    cmd.wz = 0.0;
  }

  const double cmd_dt(period.toSec());
  


  switch (cmd.mode)
  {
    case MODE_ACKERMANN:
    {
      limiter_lin_1.limit(cmd.vx, last_cmd_.vx, cmd_dt);
      limiter_ang_1.limit(cmd.wz, last_cmd_.wz, cmd_dt);
      last_cmd_ = cmd;
      break;
    }
    case MODE_LATERAL_ACKERMANN:
    {
      limiter_lin_2.limit(cmd.vx, last_cmd_.vx, cmd_dt);
      limiter_ang_2.limit(cmd.wz, last_cmd_.wz, cmd_dt);
      last_cmd_ = cmd;
      break;
    }
    case MODE_PARALLEL:
    {
      limiter_lin_3.limit(cmd.vx, last_cmd_.vx, cmd_dt);
      double theta_eq = last_cmd_.wz + std::atan2(std::sin(cmd.wz - last_cmd_.wz), std::cos(cmd.wz - last_cmd_.wz)); 
      limiter_ang_3.limit(theta_eq, last_cmd_.wz, cmd_dt);
      cmd.wz = theta_eq;
      last_cmd_ = cmd;
      break;
    }
    case MODE_SPIN:
    {
      limiter_lin_4.limit(cmd.vx, last_cmd_.vx, cmd_dt);
      limiter_ang_4.limit(cmd.wz, last_cmd_.wz, cmd_dt);
      last_cmd_ = cmd;
      break;
    }
  }


  // 计算轮速和转角
  double vl_f, vr_f, vl_r, vr_r;
  double sl_f, sr_f, sl_r, sr_r;
  computeWheelCommands(cmd, vl_f, vr_f, vl_r, vr_r, sl_f, sr_f, sl_r, sr_r);
  
  // 发送命令到硬件
  front_wheel_joints_[0].setCommand(vl_f);
  front_wheel_joints_[1].setCommand(vr_f);
  rear_wheel_joints_[0].setCommand(vl_r);
  rear_wheel_joints_[1].setCommand(vr_r);

  // 实际应该用PositionJointInterface
  front_steer_joints_[0].setCommand(sl_f);
  front_steer_joints_[1].setCommand(sr_f);
  rear_steer_joints_[0].setCommand(sl_r);
  rear_steer_joints_[1].setCommand(sr_r);
  
  // 发布里程计
  publishOdometry(time);
}

void MultiMode4WSController::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  if (std::isnan(msg.linear.x) || std::isnan(msg.linear.y) || std::isnan(msg.angular.z))
  {
    ROS_WARN_THROTTLE(1.0, "Received NaN in cmd_vel, ignoring");
    return;
  }
  
  command_struct_.vx = msg.linear.x;
  command_struct_.wz = msg.angular.z;
  command_struct_.mode = current_mode_;
  command_struct_.stamp = ros::Time::now();
  command_.writeFromNonRT(command_struct_);
}

void MultiMode4WSController::modeSelectCallback(const std_msgs::UInt8& msg)
{
  if (msg.data > MODE_SPIN)
  {
    ROS_WARN_STREAM("Invalid mode " << (int)msg.data << ", ignoring");
    return;
  }
  
  if (msg.data != current_mode_)
  {
    ROS_INFO_STREAM(name_ << ": Switching from mode " << (int)current_mode_ 
                    << " to " << (int)msg.data);
    current_mode_ = msg.data;
  }
}

void MultiMode4WSController::computeWheelCommands(const MultiModeCommand& cmd,
                                                  double& vl_f, double& vr_f,
                                                  double& vl_r, double& vr_r,
                                                  double& sl_f, double& sr_f,
                                                  double& sl_r, double& sr_r)
{
  
  switch (cmd.mode)
  {
    case MODE_ACKERMANN:
    {
      modeAckermann(cmd, vl_f, vr_f, vl_r, vr_r, sl_f, sr_f, sl_r, sr_r);
      break;
    }
    case MODE_PARALLEL:
    {
      modeParallel(cmd, vl_f, vr_f, vl_r, vr_r, sl_f, sr_f, sl_r, sr_r);
      break;
    }
    case MODE_LATERAL_ACKERMANN:
    {
      modeLateralAckermann(cmd, vl_f, vr_f, vl_r, vr_r, sl_f, sr_f, sl_r, sr_r);
      break;
    }
    case MODE_SPIN:
    {
      modeSpin(cmd, vl_f, vr_f, vl_r, vr_r, sl_f, sr_f, sl_r, sr_r);
      break;
    }
    default:
    {
      brake();
      vl_f = vr_f = vl_r = vr_r = 0.0;
      sl_f = sr_f = sl_r = sr_r = 0.0;
    }
  }
}

void MultiMode4WSController::modeAckermann(const MultiModeCommand& cmd,
                                           double& vl_f, double& vr_f,
                                           double& vl_r, double& vr_r,
                                           double& sl_f, double& sr_f,
                                           double& sl_r, double& sr_r)
{
  if (fabs(cmd.wz) < 1e-4)
  {
    // 直线
    const double v = cmd.vx / wheel_radius_;
    vl_f = vr_f = vl_r = vr_r = v;
    sl_f = sr_f = sl_r = sr_r = 0;
  }
  
  else
  {
    double delta = cmd.wz;
    double v     = cmd.vx;

    // 1. cot 分解（注意 track_/2）
    double cot_d = 1.0 / tan(delta);

    double cot_fl = cot_d - (track_) / wheel_base_;
    double cot_fr = cot_d + (track_) / wheel_base_;

    double delta_fl = atan(1.0 / cot_fl);
    double delta_fr = atan(1.0 / cot_fr);

    // 2. 后轮（SNS）
    double delta_rl = -delta_fl;
    double delta_rr = -delta_fr;

    // 3. 轮速（严格按论文）
    double vfl = v * tan(delta) / sin(delta_fl);
    double vfr = v * tan(delta) / sin(delta_fr);
    double vrl = v * tan(delta) / sin(delta_fl);
    double vrr = v * tan(delta) / sin(delta_fr);

    // 4. 输出
    sl_f = delta_fl;
    sr_f = delta_fr;
    sl_r = delta_rl;
    sr_r = delta_rr;

    vl_f = vfl / wheel_radius_;
    vr_f = vfr / wheel_radius_;
    vl_r = vrl / wheel_radius_;
    vr_r = vrr / wheel_radius_;
    // ROS_INFO_STREAM(delta_fl << "  "  << delta_fr << "  " <<  delta_rl << "  " << delta_rr);
  }
}


void MultiMode4WSController::modeLateralAckermann(const MultiModeCommand& cmd,
                                                  double& vl_f, double& vr_f,
                                                  double& vl_r, double& vr_r,
                                                  double& sl_f, double& sr_f,
                                                  double& sl_r, double& sr_r)
{
  // 以左侧为正方向
  if (fabs(cmd.wz) < 1e-4)
  {
    // 直线
    const double v = cmd.vx / wheel_radius_;
    vl_f = vr_f = vl_r = vr_r = v;
    sl_f = sr_f = sl_r = sr_r = M_PI/2;
  }
  
  else
  {
    double delta = cmd.wz;
    double v     = cmd.vx;

    // 1. cot 分解（注意 track_/2）
    double cot_d = 1.0 / tan(delta);


    double cot_fr = cot_d - (wheel_base_) / track_;
    double cot_rr = cot_d + (wheel_base_) / track_;
    double delta_fr = atan(1.0 / cot_fr);
    double delta_rr = atan(1.0 / cot_rr);

    // 2. 右轮（SNS）
    double delta_fl = -delta_fr;
    double delta_rl = -delta_rr;

    // 3. 轮速
    double vfl = -v * tan(delta) / sin(delta_fl);
    double vfr = -v * tan(delta) / sin(delta_fl);
    double vrl = v * tan(delta) / sin(delta_rr);
    double vrr = v * tan(delta) / sin(delta_rr);

    // 4. 输出
    sl_f = delta_fl + M_PI/2;
    sr_f = delta_fr + M_PI/2;
    sl_r = delta_rl + M_PI/2;
    sr_r = delta_rr + M_PI/2;

    vl_f = vfl / wheel_radius_;
    vr_f = vfr / wheel_radius_;
    vl_r = vrl / wheel_radius_;
    vr_r = vrr / wheel_radius_;

    // ROS_INFO_STREAM(delta_fl << "  "  << delta_fr << "  " <<  delta_rl << "  " << delta_rr);
    // ROS_INFO_STREAM(vfl << "  "  << vfr << "  " <<  vrl << "  " << vrr);
  }
}

void MultiMode4WSController::modeParallel(const MultiModeCommand& cmd,
                                      double& vl_f, double& vr_f,
                                      double& vl_r, double& vr_r,
                                      double& sl_f, double& sr_f,
                                      double& sl_r, double& sr_r)
{

    double delta = cmd.wz;
    double v     = cmd.vx;

    
    // 转向
    sl_f = sr_f = sl_r = sr_r = delta;
    // 轮速
    vl_f = vr_f = vl_r = vr_r = v;

    vl_f = vl_f / wheel_radius_;
    vr_f = vr_f / wheel_radius_;
    vl_r = vl_r / wheel_radius_;
    vr_r = vr_r / wheel_radius_;
  
}


void MultiMode4WSController::modeSpin(const MultiModeCommand& cmd,
                                      double& vl_f, double& vr_f,
                                      double& vl_r, double& vr_r,
                                      double& sl_f, double& sr_f,
                                      double& sl_r, double& sr_r)
{
  const double angle = atan2(track_, wheel_base_);
  
  // 车轮指向
  sl_f = -(M_PI_2 - angle);
  sr_f = M_PI_2 - angle;
  sl_r = M_PI_2 - angle;
 
  sr_r = -(M_PI_2 - angle);
  
  const double v = cmd.vx / wheel_radius_;
  vr_f = vr_r = v;
  vl_f = vl_r = -v;
}

void MultiMode4WSController::updateOdometry(const ros::Time& time)
{
  if (open_loop_)
    return;
  
  const double fl_vel = front_wheel_joints_[0].getVelocity();
  const double fr_vel = front_wheel_joints_[1].getVelocity();
  const double rl_vel = rear_wheel_joints_[0].getVelocity();
  const double rr_vel = rear_wheel_joints_[1].getVelocity();
  
  if (std::isnan(fl_vel) || std::isnan(fr_vel) || std::isnan(rl_vel) || std::isnan(rr_vel))
    return;
  
  const double fl_steer = front_steer_joints_[0].getPosition();
  const double fr_steer = front_steer_joints_[1].getPosition();
  const double rl_steer = rear_steer_joints_[0].getPosition();
  const double rr_steer = rear_steer_joints_[1].getPosition();
  
  if (std::isnan(fl_steer) || std::isnan(fr_steer) || std::isnan(rl_steer) || std::isnan(rr_steer))
    return;
  
  double front_steer = 0.0;
  if (fabs(fl_steer) > 0.001 || fabs(fr_steer) > 0.001)
  {
    front_steer = atan(2.0 * tan(fl_steer) * tan(fr_steer) / 
                      (tan(fl_steer) + tan(fr_steer)));
  }
  
  double rear_steer = 0.0;
  if (fabs(rl_steer) > 0.001 || fabs(rr_steer) > 0.001)
  {
    rear_steer = atan(2.0 * tan(rl_steer) * tan(rr_steer) /
                     (tan(rl_steer) + tan(rr_steer)));
  }
  
  odometry_.update(fl_vel, fr_vel, rl_vel, rr_vel, front_steer, rear_steer, time);
}

void MultiMode4WSController::publishOdometry(const ros::Time& time)
{
  if (last_odom_publish_time_ + publish_period_ > time)
    return;
  
  last_odom_publish_time_ += publish_period_;
  
  const geometry_msgs::Quaternion orientation = 
    tf::createQuaternionMsgFromYaw(odometry_.getHeading());
  
  // 发布里程计
  if (odom_pub_->trylock())
  {
    odom_pub_->msg_.header.stamp = time;
    odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
    odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
    odom_pub_->msg_.pose.pose.orientation = orientation;
    odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinearX();
    odom_pub_->msg_.twist.twist.linear.y = odometry_.getLinearY();
    odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
    odom_pub_->unlockAndPublish();
  }
  
  // 发布TF
  if (enable_odom_tf_ && tf_odom_pub_->trylock())
  {
    geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
    odom_frame.header.stamp = time;
    odom_frame.transform.translation.x = odometry_.getX();
    odom_frame.transform.translation.y = odometry_.getY();
    odom_frame.transform.rotation = orientation;
    tf_odom_pub_->unlockAndPublish();
  }
}

void MultiMode4WSController::brake()
{
  for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
  {
    front_wheel_joints_[i].setCommand(0.0);
    rear_wheel_joints_[i].setCommand(0.0);
    front_steer_joints_[i].setCommand(0.0);
    rear_steer_joints_[i].setCommand(0.0);
  }
}

void MultiMode4WSController::setOdomPubFields(ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh)
{
  // 获取协方差参数
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  
  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  
  // 创建里程计发布器
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
    controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  
  // 设置协方差
  if (pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_cov_list.size() == 6)
  {
    odom_pub_->msg_.pose.covariance = {
      static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])
    };
  }
  
  if (twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray && twist_cov_list.size() == 6)
  {
    odom_pub_->msg_.twist.covariance = {
      static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])
    };
  }
  
  // 创建TF发布器
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(
    root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}

} // namespace multi_mode_4ws_controller

PLUGINLIB_EXPORT_CLASS(multi_mode_4ws_controller::MultiMode4WSController,
                       controller_interface::ControllerBase)