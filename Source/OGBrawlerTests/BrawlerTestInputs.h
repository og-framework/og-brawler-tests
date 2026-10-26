#pragma once
// SPDX-License-Identifier: BUSL-1.1

#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationFrame.h"

#include <cstdint>

#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

namespace brawlerTestInputs
{

struct Fields
{
    glm::vec3 aimDirection       = simFrame::kUp;
    bool      attackLeft         = false;
    bool      attackRight        = false;
    glm::vec2 moveStick          = glm::vec2(0.f);
    glm::vec3 moveDirectionWorld = glm::vec3(0.f);
    uint32_t  triggeredActionId  = 0u;
    uint8_t   flags              = 0u;
};

inline simulatableBrawler::PlayerInput make(const Fields& f)
{
    return simulatableBrawler::PlayerInput{
        .aimDirection       = f.aimDirection,
        .attackLeft         = f.attackLeft,
        .attackRight        = f.attackRight,
        .moveStick          = f.moveStick,
        .moveDirectionWorld = f.moveDirectionWorld,
        .triggeredActionId  = f.triggeredActionId,
        .flags              = f.flags };
}

} // namespace brawlerTestInputs
