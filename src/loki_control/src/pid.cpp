/*!*******************************************************************************************
 *  \file       pid.cpp
 *  \brief      PID 1D Controller implementation
 *  \authors    Rafael Pérez Seguí
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * adapted for loki auv removed templates and Eigen dependency
 ********************************************************************************/

#include "loki_control/pid.hpp"

namespace loki
{

PID::PID(const PIDParams & pid_params, const bool & verbose)
    : verbose_(verbose)
{
  update_params(pid_params);
  reset_controller();
}

PID::~PID() {}

void PID::update_params(const PIDParams & params)
{
  set_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
  set_anti_windup(params.antiwindup_cte);
  set_alpha(params.alpha);
  set_reset_integral_saturation_flag(params.reset_integral_flag);

  if ((params.upper_output_saturation != 0.0) || (params.lower_output_saturation != 0.0)) {
    set_output_saturation(params.upper_output_saturation, params.lower_output_saturation);
  } else {
    disable_output_saturation();
  }
}

void PID::reset_controller()
{
  first_run_ = true;
}

void PID::set_output_saturation(double upper_saturation, double lower_saturation)
{
  if (std::abs(upper_saturation - lower_saturation) < std::numeric_limits<double>::epsilon()) {
    std::cerr << "Upper and lower saturation are equal. Saturation is disabled" << std::endl;
    disable_output_saturation();
    return;
  }
  if (upper_saturation < lower_saturation) {
    std::cerr << "Upper saturation is lower than lower saturation. Saturation is disabled" << std::endl;
    disable_output_saturation();
    return;
  }
  upper_output_saturation_ = upper_saturation;
  lower_output_saturation_ = lower_saturation;
  saturation_flag_         = true;
}

void PID::disable_output_saturation()
{
  saturation_flag_ = false;
}

double PID::get_error(double state, double reference)
{
  return reference - state;
}

void PID::get_error(double state, double reference,
                    double state_dot, double reference_dot,
                    double & proportional_error, double & derivative_error)
{
  proportional_error = reference - state;
  derivative_error   = reference_dot - state_dot;
}

double PID::compute_control(double dt, double proportional_error)
{
  if (first_run_) {
    first_run_               = false;
    integral_accum_error_    = 0.0;
    proportional_error_      = proportional_error;
    filtered_derivate_error_ = 0.0;
  }

  proportional_error_contribution_ = Kp_ * proportional_error;
  integral_error_contribution_     = compute_integral_contribution(dt, proportional_error);
  derivate_error_contribution_     = compute_derivative_contribution_by_deriving(dt, proportional_error);

  output_ = proportional_error_contribution_ +
            integral_error_contribution_     +
            derivate_error_contribution_;

  if (saturation_flag_) {
    output_ = saturate_output(output_, upper_output_saturation_, lower_output_saturation_);
  }

  proportional_error_ = proportional_error;
  return output_;
}

double PID::compute_control(double dt, double proportional_error, double derivative_error)
{
  if (first_run_) {
    first_run_               = false;
    integral_accum_error_    = 0.0;
    proportional_error_      = proportional_error;
    filtered_derivate_error_ = 0.0;
  }

  proportional_error_contribution_ = Kp_ * proportional_error;
  integral_error_contribution_     = compute_integral_contribution(dt, proportional_error);
  derivate_error_contribution_     = compute_derivative_contribution(derivative_error);

  output_ = proportional_error_contribution_ +
            integral_error_contribution_     +
            derivate_error_contribution_;

  if (saturation_flag_) {
    output_ = saturate_output(output_, upper_output_saturation_, lower_output_saturation_);
  }

  proportional_error_ = proportional_error;
  derivative_error_   = derivative_error;
  return output_;
}

double PID::saturate_output(double output, double upper_limits, double lower_limits)
{
  return std::clamp(output, lower_limits, upper_limits);
}

PIDParams PID::get_params() const
{
  PIDParams params;
  get_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
  params.antiwindup_cte      = get_anti_windup();
  params.alpha               = get_alpha();
  params.reset_integral_flag = get_reset_integral_saturation_flag();
  get_saturation_limits(params.upper_output_saturation, params.lower_output_saturation);
  return params;
}

void   PID::set_gains(double kp, double ki, double kd) { Kp_ = kp; Ki_ = ki; Kd_ = kd; }
void   PID::get_gains(double & kp, double & ki, double & kd) const { kp = Kp_; ki = Ki_; kd = Kd_; }
void   PID::set_kp(double kp) { Kp_ = kp; }
double PID::get_kp() const { return Kp_; }
void   PID::set_ki(double ki) { Ki_ = ki; }
double PID::get_ki() const { return Ki_; }
void   PID::set_kd(double kd) { Kd_ = kd; }
double PID::get_kd() const { return Kd_; }
void   PID::set_anti_windup(double anti_windup) { antiwindup_cte_ = anti_windup; }
double PID::get_anti_windup() const { return antiwindup_cte_; }
void   PID::set_alpha(double alpha) { alpha_ = alpha; }
double PID::get_alpha() const { return alpha_; }
void   PID::set_reset_integral_saturation_flag(bool flag) { reset_integral_flag_ = flag; }
bool   PID::get_reset_integral_saturation_flag() const { return reset_integral_flag_; }
void   PID::get_saturation_limits(double & upper_limit, double & lower_limit) const {
  upper_limit = upper_output_saturation_;
  lower_limit = lower_output_saturation_;
}
bool   PID::get_output_saturation_flag() const { return saturation_flag_; }
double PID::get_proportional_error() const { return proportional_error_; }
double PID::get_derivative_error() const { return derivative_error_; }
double PID::get_proportional_error_contribution() const { return proportional_error_contribution_; }
double PID::get_integral_error_contribution() const { return integral_error_contribution_; }
double PID::get_derivative_error_contribution() const { return derivate_error_contribution_; }
double PID::get_output() const { return output_; }

double PID::compute_integral_contribution(double dt, double proportional_error)
{
  if (reset_integral_flag_ != 0) {
    if (std::abs(integral_accum_error_) > antiwindup_cte_) {
      if (std::signbit(integral_accum_error_) != std::signbit(proportional_error)) {
        integral_accum_error_ = 0.0;
      }
    }
  }

  integral_accum_error_ += proportional_error * dt;

  if (antiwindup_cte_ != 0.0) {
    integral_accum_error_ =
        saturate_output(integral_accum_error_, antiwindup_cte_, -1.0 * antiwindup_cte_);
  }

  return Ki_ * integral_accum_error_;
}

double PID::compute_derivative_contribution_by_deriving(double dt, double proportional_error)
{
  double derivate_proportional_error_increment = (proportional_error - proportional_error_) / dt;

  filtered_derivate_error_ =
      alpha_ * derivate_proportional_error_increment + (1.0 - alpha_) * filtered_derivate_error_;

  return compute_derivative_contribution(filtered_derivate_error_);
}

double PID::compute_derivative_contribution(double derivate_error)
{
  return Kd_ * derivate_error;
}

}  // namespace loki