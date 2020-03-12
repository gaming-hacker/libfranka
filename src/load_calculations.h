// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <Eigen/Core>
#include <array>

namespace franka {

auto combineCenterOfMass(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,
    double m_load,
    const std::array<double, 3>& F_x_Cload) -> std::array<double, 3>;

auto skewSymmetricMatrixFromVector(const Eigen::Vector3d& input) -> Eigen::Matrix3d;

auto combineInertiaTensor(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,
    const std::array<double, 9>& I_ee,
    double m_load,
    const std::array<double, 3>& F_x_Cload,
    const std::array<double, 9>& I_load,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) -> std::array<double, 9>;

}  // namespace franka
