// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <array>
#include <memory>

/**
 * @file model.h
 * Contains model library types.
 */

namespace franka {

/**
 * Enumerates the seven joints, the flange, and the end effector of a robot.
 */
enum class Frame {
  kJoint1,
  kJoint2,
  kJoint3,
  kJoint4,
  kJoint5,
  kJoint6,
  kJoint7,
  kFlange,
  kEndEffector,
  kStiffness
};

/**
 * Post-increments the given Frame by one.
 *
 * For example, Frame::kJoint2++ results in Frame::kJoint3.
 *
 * @param[in] frame Frame to increment.
 *
 * @return Original Frame.
 */
auto operator++(Frame& frame, int /* dummy */) noexcept -> Frame ;

class ModelLibrary;
class Network;

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class Model {
 public:
  /**
   * Creates a new Model instance.
   *
   * This constructor is for internal use only.
   *
   * @see Robot::loadModel
   *
   * @param[in] network For internal use.
   *
   * @throw ModelException if the model library cannot be loaded.
   */
  explicit Model(franka::Network& network);

  /**
   * Move-constructs a new Model instance.
   *
   * @param[in] model Other Model instance.
   */
  Model(Model&& model) noexcept;

  /**
   * Move-assigns this Model from another Model instance.
   *
   * @param[in] model Other Model instance.
   *
   * @return Model instance.
   */
  auto operator=(Model&& model) noexcept -> Model&;

  /**
   * Unloads the model library.
   */
  ~Model() noexcept;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  [[nodiscard]] auto pose(Frame frame, const franka::RobotState& robot_state) const -> std::array<double, 16>;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  [[nodiscard]] auto pose(
      Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,
      const std::array<double, 16>& EE_T_K)
      const -> std::array<double, 16>;

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] auto  bodyJacobian(Frame frame, const franka::RobotState& robot_state) const -> std::array<double, 42>;

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] auto bodyJacobian(
      Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,
      const std::array<double, 16>& EE_T_K)
      const -> std::array<double, 42>;

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] auto zeroJacobian(Frame frame, const franka::RobotState& robot_state) const -> std::array<double, 42>;

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] auto zeroJacobian(
      Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,
      const std::array<double, 16>& EE_T_K)
      const -> std::array<double, 42>;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  [[nodiscard]] auto mass(const franka::RobotState& robot_state) const noexcept -> std::array<double, 49>;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  [[nodiscard]] auto mass(
      const std::array<double, 7>& q,
      const std::array<double, 9>& I_total,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)
      const noexcept -> std::array<double, 49>;

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the Coriolis force vector should be calculated.
   *
   * @return Coriolis force vector.
   */
  [[nodiscard]] auto  coriolis(const franka::RobotState& robot_state) const noexcept -> std::array<double, 7>;

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   */
  [[nodiscard]] auto coriolis(
      const std::array<double, 7>& q,
      const std::array<double, 7>& dq,
      const std::array<double, 9>& I_total,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)
      const noexcept -> std::array<double, 7>;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   */
  [[nodiscard]] auto gravity(
      const std::array<double, 7>& q,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const noexcept -> std::array<double, 7>;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   */
  [[nodiscard]] auto gravity(const franka::RobotState& robot_state,
                                const std::array<double, 3>& gravity_earth = {
                                    {0., 0., -9.81}}) const noexcept -> std::array<double, 7>;

  /// @cond DO_NOT_DOCUMENT
  Model(const Model&) = delete;
  auto operator=(const Model&) -> Model& = delete;
  /// @endcond

 private:
  std::unique_ptr<ModelLibrary> library_;
};

}  // namespace franka
