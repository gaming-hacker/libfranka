// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/vacuum_gripper.h>
#include <research_interface/vacuum_gripper/types.h>
#include <sstream>
#include "network.h"

namespace franka {

namespace {

template <typename T, typename... TArgs>
auto executeCommand(Network& network, TArgs&&... args) -> bool {
  uint32_t command_id = network.tcpSendRequest<T>(std::forward<TArgs>(args)...);
  typename T::Response response = network.tcpBlockingReceiveResponse<T>(command_id);

  switch (response.status) {
    case T::Status::kSuccess:
      return true;
    case T::Status::kFail:
      throw CommandException("libfranka vacuum gripper: Command failed!");
    case T::Status::kUnsuccessful:
      return false;
    case T::Status::kAborted:
      throw CommandException("libfranka vacuum gripper: Command aborted!");
    default:
      throw ProtocolException(
          "libfranka vacuum gripper: Unexpected response while handling command!");
  }
}

auto convertVacuumGripperState(
    const research_interface::vacuum_gripper::VacuumGripperState& vacuum_gripper_state) noexcept -> VacuumGripperState {
  VacuumGripperState converted{};
  converted.in_control_range = vacuum_gripper_state.in_control_range;
  converted.part_detached = vacuum_gripper_state.part_detached;
  converted.part_present = vacuum_gripper_state.part_present;
  switch (vacuum_gripper_state.device_status) {
    case research_interface::vacuum_gripper::DeviceStatus::kGreen:
      converted.device_status = VacuumGripperDeviceStatus::kGreen;
      break;
    case research_interface::vacuum_gripper::DeviceStatus::kYellow:
      converted.device_status = VacuumGripperDeviceStatus::kYellow;
      break;
    case research_interface::vacuum_gripper::DeviceStatus::kOrange:
      converted.device_status = VacuumGripperDeviceStatus::kOrange;
      break;
    case research_interface::vacuum_gripper::DeviceStatus::kRed:
      converted.device_status = VacuumGripperDeviceStatus::kRed;
      break;
  }
  converted.actual_power = vacuum_gripper_state.actual_power;
  converted.vacuum = vacuum_gripper_state.vacuum;
  converted.time = Duration(vacuum_gripper_state.message_id);
  return converted;
}

}  // anonymous namespace

VacuumGripper::VacuumGripper(const std::string& franka_address)
    : network_{std::make_unique<Network>(franka_address,
                                         research_interface::vacuum_gripper::kCommandPort)} {
  connect<research_interface::vacuum_gripper::Connect,
          research_interface::vacuum_gripper::kVersion>(*network_, &ri_version_);
}

VacuumGripper::~VacuumGripper() noexcept = default;
VacuumGripper::VacuumGripper(VacuumGripper&&) noexcept = default;
auto VacuumGripper::operator=(VacuumGripper&&) noexcept -> VacuumGripper& = default;

auto VacuumGripper::serverVersion() const noexcept -> VacuumGripper::ServerVersion {
  return ri_version_;
}

auto VacuumGripper::vacuum(uint8_t vacuum,
                           std::chrono::milliseconds timeout,
                           ProductionSetupProfile profile) const -> bool {
  research_interface::vacuum_gripper::Profile converted_profile;
  switch (profile) {
    case ProductionSetupProfile::kP0:
      converted_profile = research_interface::vacuum_gripper::Profile::kP0;
      break;
    case ProductionSetupProfile::kP1:
      converted_profile = research_interface::vacuum_gripper::Profile::kP1;
      break;
    case ProductionSetupProfile::kP2:
      converted_profile = research_interface::vacuum_gripper::Profile::kP2;
      break;
    case ProductionSetupProfile::kP3:
      converted_profile = research_interface::vacuum_gripper::Profile::kP3;
      break;
    default:
      throw CommandException("Vacuum Gripper: Vacuum profile not defined!");
      break;
  }
  return executeCommand<research_interface::vacuum_gripper::Vacuum>(*network_, vacuum,
                                                                    converted_profile, timeout);
}

auto VacuumGripper::dropOff(std::chrono::milliseconds timeout) const -> bool {
  return executeCommand<research_interface::vacuum_gripper::DropOff>(*network_, timeout);
}

auto VacuumGripper::stop() const -> bool {
  return executeCommand<research_interface::vacuum_gripper::Stop>(*network_);
}

auto VacuumGripper::readOnce() const -> VacuumGripperState {
  research_interface::vacuum_gripper::VacuumGripperState vacuum_gripper_state{};
  // Delete old data from the UDP buffer.
  while (network_->udpReceive<decltype(vacuum_gripper_state)>(&vacuum_gripper_state)) {
  }

  vacuum_gripper_state = network_->udpBlockingReceive<decltype(vacuum_gripper_state)>();
  return convertVacuumGripperState(vacuum_gripper_state);
}

}  // namespace franka
