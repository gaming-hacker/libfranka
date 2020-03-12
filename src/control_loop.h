// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

#include <cmath>
#include <functional>

#include "robot_control.h"

namespace franka {

template <typename T>
class ControlLoop {
 public:
  static constexpr research_interface::robot::Move::Deviation kDefaultDeviation{10.0, 3.12,
                                                              6.2831853071795865};

  using ControlCallback = std::function<Torques(const RobotState&, franka::Duration)>;
  using MotionGeneratorCallback = std::function<T(const RobotState&, franka::Duration)>;

  ControlLoop(RobotControl& robot,
              ControlCallback control_callback,
              MotionGeneratorCallback motion_callback,
              bool limit_rate,
              double cutoff_frequency);
  ControlLoop(RobotControl& robot,
              ControllerMode controller_mode,
              MotionGeneratorCallback motion_callback,
              bool limit_rate,
              double cutoff_frequency);

  void operator()();

 protected:
  ControlLoop(RobotControl& robot,
              MotionGeneratorCallback motion_callback,
              ControlCallback control_callback,
              bool limit_rate,
              double cutoff_frequency);

  auto spinControl(const RobotState& robot_state,
                   franka::Duration time_step,
                   research_interface::robot::ControllerCommand* command)-> bool;
  auto spinMotion(const RobotState& robot_state,
                  franka::Duration time_step,
                  research_interface::robot::MotionGeneratorCommand* command) -> bool;

 private:
  RobotControl& robot_;
  const MotionGeneratorCallback motion_callback_;
  const ControlCallback control_callback_;
  const bool limit_rate_;
  const double cutoff_frequency_;
  uint32_t motion_id_ = 0;

  void convertMotion(const T& motion,
                     const RobotState& robot_state,
                     research_interface::robot::MotionGeneratorCommand* command);
};

}  // namespace franka
