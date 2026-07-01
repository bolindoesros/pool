/*!******************************************************************************
 *  \file       pid.hpp
 *  \brief      PID 1D Controller definition
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * adapted for loki auv, removed templates and Eigen dependency
 ********************************************************************************/

#ifndef LOKI_CONTROL__PID_HPP_
#define LOKI_CONTROL__PID_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace loki
{

struct PIDParams
{
  double Kp_gains = 0.0;
  double Ki_gains = 0.0;
  double Kd_gains = 0.0;

  double antiwindup_cte    = 0.0;
  double alpha             = 1.0;
  bool   reset_integral_flag = false;

  double upper_output_saturation = 0.0;
  double lower_output_saturation = 0.0;
};

class PID
{
public:
  explicit PID(const PIDParams & pid_params = PIDParams(), const bool & verbose = false);
  ~PID();

  void update_params(const PIDParams & params);
  void reset_controller();

  void set_output_saturation(double upper_saturation, double lower_saturation);
  void disable_output_saturation();

  static double get_error(double state, double reference);
  static void   get_error(double state, double reference,
                          double state_dot, double reference_dot,
                          double & proportional_error, double & derivative_error);

  double compute_control(double dt, double proportional_error);
  double compute_control(double dt, double proportional_error, double derivative_error);

  static double saturate_output(double output, double upper_limits, double lower_limits);

  PIDParams get_params() const;

  void   set_gains(double kp, double ki, double kd);
  void   get_gains(double & kp, double & ki, double & kd) const;
  void   set_kp(double kp);
  double get_kp() const;
  void   set_ki(double ki);
  double get_ki() const;
  void   set_kd(double kd);
  double get_kd() const;
  void   set_anti_windup(double anti_windup);
  double get_anti_windup() const;
  void   set_alpha(double alpha);
  double get_alpha() const;
  void   set_reset_integral_saturation_flag(bool flag);
  bool   get_reset_integral_saturation_flag() const;
  void   get_saturation_limits(double & upper_limit, double & lower_limit) const;
  bool   get_output_saturation_flag() const;
  double get_proportional_error() const;
  double get_derivative_error() const;
  double get_proportional_error_contribution() const;
  double get_integral_error_contribution() const;
  double get_derivative_error_contribution() const;
  double get_output() const;

private:
  bool verbose_ = false;

  double Kp_ = 0.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  double antiwindup_cte_    = 0.0;
  double alpha_             = 1.0;
  bool   reset_integral_flag_ = false;

  bool   saturation_flag_           = false;
  double upper_output_saturation_   = 0.0;
  double lower_output_saturation_   = 0.0;

  bool   first_run_                 = true;
  double integral_accum_error_      = 0.0;
  double filtered_derivate_error_   = 0.0;

  double proportional_error_              = 0.0;
  double derivative_error_                = 0.0;
  double proportional_error_contribution_ = 0.0;
  double integral_error_contribution_     = 0.0;
  double derivate_error_contribution_     = 0.0;
  double output_                          = 0.0;

protected:
  double compute_integral_contribution(double dt, double proportional_error);
  double compute_derivative_contribution_by_deriving(double dt, double proportional_error);
  double compute_derivative_contribution(double derivate_error);
};

}  // namespace loki

#endif  // LOKI_CONTROL__PID_HPP_