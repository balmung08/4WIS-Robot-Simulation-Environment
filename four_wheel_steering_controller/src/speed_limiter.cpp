#include <algorithm>

#include <four_wheel_steering_controller/speed_limiter.h>

namespace four_wheel_steering_controller
{

  SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits, // 是否启用速度上下限
    bool has_acceleration_limits,  // 是否启用加速度上下限
    bool has_jerk_limits,  // 是否启用加加速度上下限
    double min_velocity, 
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_jerk,
    double max_jerk
  )
  : has_velocity_limits(has_velocity_limits)
  , has_acceleration_limits(has_acceleration_limits)
  , has_jerk_limits(has_jerk_limits)
  , min_velocity(min_velocity)
  , max_velocity(max_velocity)
  , min_acceleration(min_acceleration)
  , max_acceleration(max_acceleration)
  , min_jerk(min_jerk)
  , max_jerk(max_jerk)
  {
  }

  // 控制量限制函数总入口，串联三种限制。加加速度限制通常不启动
  double SpeedLimiter::limit(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);
    // 返回原本的速度被限制到的百分比，可能有用
    return tmp != 0.0 ? v / tmp : 1.0;
  }


  // 限制速度 clip(v, vmin​, vmax​)，同样返回了比例
  double SpeedLimiter::limit_velocity(double& v)
  {
    const double tmp = v;

    if (has_velocity_limits)
    {
      v = clamp(v, min_velocity, max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  // 从加速度角度上限制速度 v←v0+clip(v−v0​, Δvmin​, Δvmax​)
  double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
  {
    const double tmp = v;

    if (has_acceleration_limits)
    {
      const double dv_min = min_acceleration * dt;
      const double dv_max = max_acceleration * dt;

      const double dv = clamp(v - v0, dv_min, dv_max);

      v = v0 + dv;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  // 从加加速度角度上限制速度 v←v0​+dv0​+da
  double SpeedLimiter::limit_jerk(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    if (has_jerk_limits)
    {
      const double dv  = v  - v0;
      const double dv0 = v0 - v1;

      const double dt2 = 2. * dt * dt;

      const double da_min = min_jerk * dt2;
      const double da_max = max_jerk * dt2;

      const double da = clamp(dv - dv0, da_min, da_max);

      v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

} // namespace four_wheel_steering_controller
