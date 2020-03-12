// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "load_calculations.h"

namespace franka {

auto combineCenterOfMass(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,
    double m_load,
    const std::array<double, 3>& F_x_Cload) -> std::array<double, 3> {
  std::array<double, 3> F_x_Ctotal{};
  if ((m_ee + m_load) > 0) {
    for (size_t i = 0; i < F_x_Ctotal.size(); i++) {
      F_x_Ctotal[i] = (m_ee * F_x_Cee[i] + m_load * F_x_Cload[i]) / (m_ee + m_load);
    }
  }

  return F_x_Ctotal;
}

auto skewSymmetricMatrixFromVector(const Eigen::Vector3d& input)  -> Eigen::Matrix3d {
  Eigen::Matrix3d input_hat;
  input_hat << 0, -input(2), input(1), input(2), 0, -input(0), -input(1), input(0), 0;
  return input_hat;
}

auto combineInertiaTensor(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,
    const std::array<double, 9>& I_ee,
    double m_load,
    const std::array<double, 3>& F_x_Cload,
    const std::array<double, 9>& I_load,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) -> std::array<double, 9> {
  // If the combined mass equals to zero, the combined inertia is also zero.
  if (m_total == 0) {
    return std::array<double, 9>{};
  }

  Eigen::Vector3d center_of_mass_ee(F_x_Cee.data());
  Eigen::Vector3d center_of_mass_load(F_x_Cload.data());
  Eigen::Vector3d center_of_mass_total(F_x_Ctotal.data());

  Eigen::Matrix3d inertia_ee(I_ee.data());
  Eigen::Matrix3d inertia_ee_flange = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d inertia_load(I_load.data());
  Eigen::Matrix3d inertia_load_flange = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d inertia_total_flange = Eigen::Matrix3d::Zero();

  // Check if the mass equals zero, the inertia should then be zero as well.
  if (m_ee == 0) {
    inertia_ee = Eigen::Matrix3d::Zero();
  }
  if (m_load == 0) {
    inertia_load = Eigen::Matrix3d::Zero();
  }

  // Calculate inertia tensor of EE and load in flange coordinates.
  inertia_ee_flange = inertia_ee - m_ee * (skewSymmetricMatrixFromVector(center_of_mass_ee) *
                                           skewSymmetricMatrixFromVector(center_of_mass_ee));
  inertia_load_flange =
      inertia_load - m_load * (skewSymmetricMatrixFromVector(center_of_mass_load) *
                               skewSymmetricMatrixFromVector(center_of_mass_load));

  // Calculate combined inertia tensor in flange coordinate.
  inertia_total_flange = inertia_ee_flange + inertia_load_flange;

  // Calculate combined inertia tensor in combined body center of mass coordinate.
  std::array<double, 9> I_total;
  Eigen::Map<Eigen::Matrix3d> inertia_total(I_total.data(), 3, 3);
  inertia_total =
      inertia_total_flange + m_total * (skewSymmetricMatrixFromVector(center_of_mass_total) *
                                        skewSymmetricMatrixFromVector(center_of_mass_total));

  return I_total;
}

}  // namespace franka
