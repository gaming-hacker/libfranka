// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <franka/log.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

#include <deque>
#include <string>
#include <vector>

namespace franka {

class Logger {
 public:
  explicit Logger(size_t log_size);

  void log(const RobotState& state, const research_interface::robot::RobotCommand& command);

  auto flush() -> std::vector<franka::Record>;

 private:
  std::vector<RobotState> states_;
  std::vector<research_interface::robot::RobotCommand> commands_;
  size_t ring_front_{0};
  size_t ring_size_{0};

  const size_t log_size_;
};

}  // namespace franka
