// SPDX-License-Identifier: BUSL-1.1
// docs/PlatformEdgeTest-rationale.md

#pragma once

#include "glm/vec3.hpp"

#include <array>

namespace quarterCylinderPlatform
{

struct ImportedHull
{
    const char* actor;
    std::array<float, 3> translation;
    std::array<float, 4> rotationWxyz;
    std::array<glm::vec3, 10> points;
};

inline constexpr std::array<ImportedHull, 2> kHulls = { {
    { "SM_QuarterCylinder21",
        { 19.1150322f, 1.42316413f, -13.7344723f },
        { 0.965925813f, 0.258819044f, -0.0f, 0.0f },
        { {
            glm::vec3{ 4.15500069f, -0.000114604991f, -10.0199995f },
            glm::vec3{ -0.0224998482f, -0.000125000006f, -10.0199995f },
            glm::vec3{ -0.0224996097f, 0.100125f, -10.0199995f },
            glm::vec3{ 4.15500021f, 0.100125f, -10.0199995f },
            glm::vec3{ 10.0249996f, -0.000125000006f, 0.0200016387f },
            glm::vec3{ 10.0249996f, 0.100125f, 0.0200016387f },
            glm::vec3{ -0.0225000866f, 0.100125f, 0.0200016387f },
            glm::vec3{ -0.0224998482f, -9.99984768e-05f, 0.0200016387f },
            glm::vec3{ 10.0199966f, -0.000110393492f, -4.15500307f },
            glm::vec3{ 10.0199966f, 0.100125f, -4.15500355f }
        } } },
    { "SM_QuarterCylinder22",
        { 19.1150322f, 1.42316413f, -13.7344723f },
        { 0.683012724f, 0.183012694f, 0.683012724f, 0.183012694f },
        { {
            glm::vec3{ 4.15500069f, -0.000114604991f, -10.0199995f },
            glm::vec3{ -0.0224998482f, -0.000125000006f, -10.0199995f },
            glm::vec3{ -0.0224996097f, 0.100125f, -10.0199995f },
            glm::vec3{ 4.15500021f, 0.100125f, -10.0199995f },
            glm::vec3{ 10.0249996f, -0.000125000006f, 0.0200016387f },
            glm::vec3{ 10.0249996f, 0.100125f, 0.0200016387f },
            glm::vec3{ -0.0225000866f, 0.100125f, 0.0200016387f },
            glm::vec3{ -0.0224998482f, -9.99984768e-05f, 0.0200016387f },
            glm::vec3{ 10.0199966f, -0.000110393492f, -4.15500307f },
            glm::vec3{ 10.0199966f, 0.100125f, -4.15500355f }
        } } },
} };

} // namespace quarterCylinderPlatform
