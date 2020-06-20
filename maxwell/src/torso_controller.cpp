/*
 * Copyright 2020 Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>

#include "etherbotix/dynamixel.hpp"
#include "etherbotix/etherbotix.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "maxwell/calibrated_interpolation.hpp"
#include "robot_controllers_interface/controller.h"
#include "robot_controllers_interface/controller_manager.h"
#include "robot_controllers_interface/joint_handle.h"
#include "std_srvs/srv/empty.hpp"

// Interface mainly for testing
#include "geometry_msgs/msg/point.hpp"

using etherbotix::Etherbotix;
using std::placeholders::_1;
using std::placeholders::_2;

#include <iostream>

namespace maxwell
{

void CalibratedInterpolation::push_back(int key, double value)
{
  data.emplace_back(key, value);
}

int CalibratedInterpolation::min_key()
{
  return data.front().first;
}

int CalibratedInterpolation::max_key()
{
  return data.back().first;
}

double CalibratedInterpolation::min_value()
{
  return data.front().second;
}

double CalibratedInterpolation::max_value()
{
  return data.back().second;
}

double CalibratedInterpolation::convert(int reading)
{
  // Check limits
  if (reading <= min_key())
  {
    return min_value();
  }
  else if (reading >= max_key())
  {
    return max_value();
  }

  // Convert raw count into position using calibration data
  CalEntry low = data.front();
  CalEntry high = data.back();
  auto cal_iterator = data.begin();
  while (reading > cal_iterator->first && cal_iterator != data.end())
  {
    low = *cal_iterator;
    ++cal_iterator;
  }
  while (reading < cal_iterator->first && cal_iterator != data.end())
  {
    high = *cal_iterator;
    ++cal_iterator;
  }
  int x = high.first - low.first;
  double y = high.second - low.second;
  int x1 = reading - low.first;
  double y1 = y * (static_cast<double>(x1) / static_cast<double>(x));
  return low.second + y1;
}

class LinearJointHandle : public robot_controllers_interface::JointHandle
{
  // Loop runs at 100hz, stop zeroing when no movement for 1s
  static constexpr int DEFAULT_ZERO_CYCLES = 100;

public:
  LinearJointHandle(const std::string& name,
                    rclcpp::Node::SharedPtr node) :
    name_(name),
    position_raw_(0),
    last_raw_(-1),
    position_(0.0),
    velocity_(0.0),
    has_command_(false),
    desired_position_(0.0),
    goal_tolerance_(0.002),
    travel_direction_(0),
    has_zero_command_(false),
    zero_cycles_(DEFAULT_ZERO_CYCLES)
  {
    eth_ = std::make_unique<Etherbotix>();

    a_ = node->declare_parameter<int>(name + ".motor_a", 0);
    b_ = node->declare_parameter<int>(name + ".motor_b", 1);
    pwm_ = node->declare_parameter<int>(name + ".motor_pwm", 7);
  
    // TODO: verify a/b/pwm range

    max_vel_ = node->declare_parameter<double>(name + ".max_vel", 0.0);

    std::vector<double> calibration_data =
      node->declare_parameter<std::vector<double>>(name + ".calibration_data",
        std::vector<double>());
    if (calibration_data.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "Calibration data not set!");
    }
    for (size_t i = 0; i < calibration_data.size(); i += 2)
    {

      int key = static_cast<int>(calibration_data[i]);
      double value = calibration_data[i + 1];
      cal_.push_back(key, value);
    }

    // Update thread
    std::thread{std::bind(&LinearJointHandle::update, this)}.detach();
  }

  virtual ~LinearJointHandle()
  {
    // Stop motion
    setVelocity(0, 0);
  }

  virtual void setPosition(double position, double /*velocity*/, double /*effort*/)
  {
    if (has_zero_command_)
    {
      // Cannot control when zeroing - shouldn't really hit this though
      return;
    }
    else if (position <= getPositionMax() and position >= getPositionMin())
    {
      desired_position_ = position;
      has_command_ = true;
    }
  }

  virtual void setVelocity(double velocity, double /*effort*/)
  {
    if (velocity > 0.0)
    {
      // Up
      eth_->set_digital_pin(a_, 1);
      eth_->set_digital_pin(b_, 0);
      eth_->set_digital_pin(pwm_, 1);
      travel_direction_ = 1;
    }
    else if (velocity < 0.0)
    {
      // Down
      eth_->set_digital_pin(a_, 0);
      eth_->set_digital_pin(b_, 1);
      eth_->set_digital_pin(pwm_, 1);
      travel_direction_ = -1;
    }
    else
    {
      eth_->set_digital_pin(pwm_, 0);
      travel_direction_ = 0;
    }
  }

  virtual void setEffort(double effort)
  {
    (void) effort;
    throw std::runtime_error("effort control mode is not supported");
  }

  virtual double getPosition()
  {
    return position_;
  }

  virtual double getVelocity()
  {
    return velocity_;
  }

  virtual double getEffort()
  {
    return 0.0;
  }

  virtual bool isContinuous()
  {
    return false;
  }

  virtual double getPositionMin()
  {
    return cal_.min_value();
  }

  virtual double getPositionMax()
  {
    return cal_.max_value();
  }

  virtual double getVelocityMax()
  {
    return max_vel_;
  }

  virtual double getEffortMax()
  {
    return 0.0;
  }

  virtual std::string getName()
  {
    return name_;
  }

  virtual void reset()
  {
    has_command_ = false;
  }

  bool zero()
  {
    if (!has_zero_command_)
    {
      zero_cycles_ = DEFAULT_ZERO_CYCLES;
      has_zero_command_ = true;
    }
    else if (zero_cycles_ == 0)
    {
      position_raw_ = 0;
      has_zero_command_ = false;
      return true;
    }
    return false;
  }

private:
  void update()
  {
    // Enable timer 12
    eth_->set_tim12_mode(1);
        
    while (rclcpp::ok())
    {
      // Update from etherbotix
      uint8_t buffer[256];
      uint8_t len = dynamixel::get_read_packet(buffer, Etherbotix::ETHERBOTIX_ID, 0, 128);
      eth_->send(buffer, len);

      int raw = eth_->get_tim12_count();
      if (raw == -1)
      {
        // We have not yet read the value
        continue;
      }
      if (last_raw_  == -1)
      {
        // This is our first read
        last_raw_ = raw;
      }

      // The raw value only ever goes up, regardless of direction of
      // motion. We therefore have to apply our own diretion value
      // to the measured difference in position.
      int diff = raw - last_raw_;
      position_raw_ += travel_direction_ * diff;
      last_raw_ = raw;
      // Convert to meters
      double p = cal_.convert(position_raw_);
      // Estimate velocity
      velocity_ = (p - position_) / 0.02;
      position_ = p;

      // Update control output
      if (has_zero_command_)
      {
        if (diff != 0)
        {
          // We moved, reset the cycle counter
          zero_cycles_ = DEFAULT_ZERO_CYCLES;
        }
        else if (zero_cycles_ > 0)
        {
          --zero_cycles_;
        }
        setVelocity(1, 0);
      }
      else if (has_command_)
      {
        if (desired_position_ > position_ + goal_tolerance_)
        {
          setVelocity(1, 0);
        }
        else if (desired_position_ < position_ - goal_tolerance_)
        {
          setVelocity(-1, 0);
        }
        else
        {
          setVelocity(0, 0);
        }
      }
      else
      {
        // Clear any possible command
        setVelocity(0, 0);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  // No copy
  LinearJointHandle(const LinearJointHandle&) = delete;
  LinearJointHandle& operator=(const LinearJointHandle&) = delete;

  std::unique_ptr<Etherbotix> eth_;

  std::string name_;

  // Etherbotix pins to use
  int a_, b_, pwm_;

  // Actual measurements
  int position_raw_, last_raw_;
  double position_;  // meters
  double velocity_;  // meters/sec

  // Limits
  double max_vel_;

  // Commands
  bool has_command_;
  double desired_position_;
  double goal_tolerance_;
  int travel_direction_;
  bool has_zero_command_;
  int zero_cycles_;

  // Mapping 
  CalibratedInterpolation cal_;
};

/**
 * @brief Torso controller creates the torso joint and also provides
 *        a service which can be used to zero the joint.
 */
class TorsoController : public robot_controllers_interface::Controller
{
public:
  TorsoController()
  {
  }

  virtual ~TorsoController() = default;

  int init(const std::string& name,
           rclcpp::Node::SharedPtr node,
           robot_controllers_interface::ControllerManagerPtr manager)
  {
    robot_controllers_interface::Controller::init(name, node, manager);
    manager_ = manager;

    // Parameters
    std::string joint_name = node->declare_parameter<std::string>(name + ".joint",
                                                                  "arm_lift_joint");

    // Create linear joint handle
    joint_ = std::make_shared<LinearJointHandle>(joint_name, node);
    auto j = std::static_pointer_cast<robot_controllers_interface::JointHandle>(joint_);
    manager->addJointHandle(j);

    // Service to zero the joint
    server_ = node->create_service<std_srvs::srv::Empty>(
      name + "/zero", std::bind(&TorsoController::callback, this, _1, _2));

    // Start and zero ourselves
    has_zero_command_ = true;
    manager->requestStart(getName());

    return 0;
  }

  bool start()
  {
    return has_zero_command_;
  }

  bool stop(bool force)
  {
    return true;
  }

  bool reset()
  {
    return true;
  }

  void update(const rclcpp::Time& now, const rclcpp::Duration& dt)
  {
    if (has_zero_command_)
    {
      if (joint_->zero())
      {
        // Done
        has_zero_command_ = false;
        manager_->requestStop(getName());
      }
    }
  }

  std::vector<std::string> getCommandedNames()
  {
    return {joint_->getName()};
  }

  std::vector<std::string> getClaimedNames()
  {
    // Claimed == Commanded
    return getCommandedNames();
  }

private:
  void callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    // Start zeroing
    has_zero_command_ = true;
    manager_->requestStart(getName());
  }

  robot_controllers_interface::ControllerManagerPtr manager_;
  std::shared_ptr<LinearJointHandle> joint_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
  bool has_zero_command_;
};

}  // namespace maxwell

PLUGINLIB_EXPORT_CLASS(maxwell::TorsoController,
                       robot_controllers_interface::Controller)
