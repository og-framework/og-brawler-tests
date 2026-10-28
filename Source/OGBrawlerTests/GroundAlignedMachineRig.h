#pragma once
// SPDX-License-Identifier: BUSL-1.1

#include <cstdint>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/SimulationDependencies.h"

#include "BrawlerSimulationMocks.h"

namespace groundAlignedMachineRig
{
    inline constexpr float kDt = 1.f / 60.f;
    inline constexpr std::uint32_t kCapsuleBodyIndex = 1u;

    struct MachineTickResult
    {
        dAttackMachineSimulation::State machine;
        dAttackRadialSimulation::InitialConditions radialIC;
        brawlerProjectileSimulation::InitialConditions projectileIC;
    };

    inline MachineTickResult integrateOnce(const dAttackMachineSimulation::PlayerInputView& playerInput,
        const glm::vec3& capsulePosition,
        const brawlerGround::GroundSample& ground)
    {
        const simulatableBrawler::StaticData staticData;
        brawlerTestMocks::MockPhysicsAdapter physics{4u};
        physics.bodies[kCapsuleBodyIndex].transform = glm::translate(glm::mat4(1.f), capsulePosition);

        const dAttackMachineSimulation::IntegrationUtils<brawlerTestMocks::MockPhysicsAdapter> utils{
            kDt, 0u, staticData.m_attackSequences, physics, staticData.m_projectileStaticData};
        const dAttackMachineSimulation::AllInput<brawlerTestMocks::MockPhysicsAdapter> allInput{playerInput, utils};

        simulatableBrawler::State state{};
        auto deps = makeDependencies<dAttackMachineSimulation::Dependencies>(state);
        const simulatableBrawler::CharacterBindings bindings{BodyId{kCapsuleBodyIndex}};
        const brawlerInboundHit::DerivedState inboundHit{};

        dAttackMachineSimulation::integrate3(kDt, allInput, deps, bindings, inboundHit, ground);

        return MachineTickResult{state.get<dAttackMachineSimulation::State>(),
            state.get<dAttackRadialSimulation::InitialConditions>(),
            state.get<brawlerProjectileSimulation::InitialConditions>()};
    }

    inline MachineTickResult swingStart(const glm::vec3& aimDirection, const brawlerGround::GroundSample& ground)
    {
        dAttackMachineSimulation::PlayerInputView playerInput{};
        playerInput.aimDirection = aimDirection;
        playerInput.attackLeft = true;
        return integrateOnce(playerInput, glm::vec3(0.f), ground);
    }

    inline MachineTickResult hadoukenLaunch(const glm::vec3& aimDirection,
        const glm::vec3& capsulePosition,
        const brawlerGround::GroundSample& ground)
    {
        dAttackMachineSimulation::PlayerInputView playerInput{};
        playerInput.aimDirection = aimDirection;
        playerInput.triggeredActionId = inputSequence::kHadoukenActionId;
        return integrateOnce(playerInput, capsulePosition, ground);
    }

    inline glm::mat3 swingFrame(const dAttackRadialSimulation::InitialConditions& ic)
    {
        return glm::mat3(glm::rotate(glm::mat4(1.f), ic.initialAimAngle, ic.initialAimRotationAxis));
    }
} // namespace groundAlignedMachineRig
