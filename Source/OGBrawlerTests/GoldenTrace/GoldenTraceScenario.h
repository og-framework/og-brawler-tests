#pragma once
// SPDX-License-Identifier: BUSL-1.1
// docs/GoldenTraceScenario-rationale.md

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "OGSimulation/OGAssert.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

namespace goldenTraceScenario
{

using Frame = simFrame;

inline constexpr uint32_t kScenarioTicks = 600u;
inline constexpr uint32_t kDenseThroughTick = 300u;
inline constexpr uint32_t kSparseEveryTicks = 5u;
inline constexpr float kDtSeconds = 0.016667f;

inline constexpr unsigned int kCharacterA = 1u;
inline constexpr unsigned int kCharacterB = 2u;
inline constexpr uint32_t kSlotA = 0u;
inline constexpr uint32_t kSlotB = 1u;

constexpr bool isSampleTick(uint32_t tick)
{
    return tick <= kDenseThroughTick || tick % kSparseEveryTicks == 0u;
}

inline constexpr glm::vec3 kSpawnA{ 0.f, 1.5f, -20.f };
inline constexpr glm::vec3 kSpawnB{ 1.8f, 1.5f, -20.f };
inline constexpr glm::vec3 kRespawnB{ 1.85f, 1.5f, -22.415f };

inline constexpr float kFloorStart = 19.f;
inline constexpr glm::vec3 kFloorCentre{ 0.75f, -0.25f, -22.f };
inline constexpr glm::vec3 kFloorHalfExtents{ 2.25f, 0.25f, 3.f };

inline constexpr float kRampFoot = 20.5f;
inline constexpr float kRampTop = 21.324244f;
inline constexpr float kRampAngleDeg = 20.f;
inline constexpr glm::vec3 kRampCentre{ 0.f, 0.05603074f, -20.946323f };
inline constexpr glm::vec3 kRampHalfExtents{ 0.6f, 0.1f, 0.43857065f };

inline constexpr float kStepHeight = 0.3f;
inline constexpr float kStepEdge = 21.8f;
inline constexpr glm::vec3 kPlateauCentre{ 0.f, 0.15f, -21.562122f };
inline constexpr glm::vec3 kPlateauHalfExtents{ 0.6f, 0.15f, 0.23787838f };

inline constexpr float kStaticFriction = 0.7f;
inline constexpr float kStaticRestitution = 0.3f;

inline constexpr brawlerMovementSimulation::MovementModel kMovementModel = brawlerMovementSimulation::MovementModel::ContinuousAccelBrake;
inline constexpr float kMaxWalkSpeed = 1.f;
inline constexpr uint32_t kStepPeriodTicks = 20u;
inline constexpr float kStepSpeed = 1.f;
inline constexpr float kGravity = -9.8f;

inline simulatableBrawler::StaticData scenarioStaticData(float gravityScale)
{
    return simulatableBrawler::StaticData(kMovementModel, kMaxWalkSpeed, kStepPeriodTicks, kStepSpeed, kGravity * gravityScale);
}

inline StaticShapeDescriptor staticBox(const glm::vec3& centre, const glm::quat& rotation, const glm::vec3& halfExtents, uint64_t stableKey)
{
    StaticShapeDescriptor descriptor;
    descriptor.shape = StaticBox{ halfExtents };
    descriptor.localToWorld = glm::translate(glm::mat4(1.f), centre) * glm::mat4_cast(rotation);
    descriptor.categories = CollisionCategories::single(collisionCategory::world);
    descriptor.blockingCategories = collisionCategory::bodyGuardProjectile | collisionCategory::worldAndCharacter
        | CollisionCategories::single(collisionCategory::queryRouting);
    descriptor.friction = kStaticFriction;
    descriptor.restitution = kStaticRestitution;
    descriptor.stableKey = stableKey;
    return descriptor;
}

inline StaticWorldDescription arenaStatics()
{
    const glm::quat identity = glm::quat(1.f, -0.f, -0.f, 0.f);
    StaticWorldDescription description;

    description.shapes.push_back(staticBox(kFloorCentre, identity, kFloorHalfExtents, 1u));

    const double angle = double(kRampAngleDeg) * 3.14159265358979323846 / 180.0;
    description.shapes.push_back(staticBox(kRampCentre,
        glm::quat(float(std::cos(0.5 * angle)), float(std::sin(0.5 * angle)), -0.f, 0.f), kRampHalfExtents, 2u));

    description.shapes.push_back(staticBox(kPlateauCentre, identity, kPlateauHalfExtents, 3u));

    return description;
}

struct InputRow
{
    unsigned int characterId = 0u;
    uint32_t firstTick = 0u;
    uint32_t lastTick = 0u;
    glm::vec3 aim = glm::vec3(0.f);
    glm::vec3 move = glm::vec3(0.f);
    float stickX = 0.f;
    float stickY = 0.f;
    bool attackLeft = false;
    bool attackRight = false;
    uint32_t actionId = 0u;
    bool holdGuard = false;
};

inline constexpr glm::vec3 kTowardB{ 1.f, 0.f, -0.f };
inline constexpr glm::vec3 kTowardA{ -1.f, 0.f, -0.f };
inline constexpr glm::vec3 kAlongTheArena{ 0.f, 0.f, -1.f };
inline constexpr glm::vec3 kBackAlongTheArena{ 0.f, 0.f, 1.f };
inline constexpr glm::vec3 kStill{ 0.f, 0.f, -0.f };
inline constexpr float kGuardOffsetSin = 0.5646425f;
inline constexpr float kGuardOffsetCos = 0.8253356f;
inline constexpr glm::vec3 kGuardBlocksRightSwing{ -kGuardOffsetCos, 0.f, -kGuardOffsetSin };
inline constexpr glm::vec3 kGuardBlocksLeftSwing{ -kGuardOffsetCos, 0.f, kGuardOffsetSin };

inline constexpr uint32_t kRightSwingTick = 40u;
inline constexpr uint32_t kLeftSwingTick = 90u;
inline constexpr uint32_t kKnockbackSwingTick = 140u;
inline constexpr uint32_t kWalkFirstTick = 185u;
inline constexpr uint32_t kWalkLastTick = 334u;
inline constexpr uint32_t kOverheadTick = 390u;
inline constexpr uint32_t kBlockedProjectileTick = 462u;
inline constexpr uint32_t kSecondOverheadTick = 518u;
inline constexpr uint32_t kHitProjectileTick = 522u;

inline constexpr InputRow kInputScript[] = {
    { kCharacterA, 1u, kRightSwingTick - 1u, kTowardB, kStill },
    { kCharacterA, kRightSwingTick, kRightSwingTick, kTowardB, kAlongTheArena, 1.f, 0.f, true },
    { kCharacterA, kRightSwingTick + 1u, kLeftSwingTick - 1u, kTowardB, kStill },
    { kCharacterA, kLeftSwingTick, kLeftSwingTick, kTowardB, kBackAlongTheArena, -1.f, 0.f, true },
    { kCharacterA, kLeftSwingTick + 1u, kKnockbackSwingTick - 1u, kTowardB, kStill },
    { kCharacterA, kKnockbackSwingTick, kKnockbackSwingTick, kTowardB, kAlongTheArena, 1.f, 0.f, true },
    { kCharacterA, kKnockbackSwingTick + 1u, kWalkFirstTick - 1u, kTowardB, kStill },
    { kCharacterA, kWalkFirstTick, kWalkLastTick, kAlongTheArena, kAlongTheArena, 0.f, 1.f },
    { kCharacterA, kWalkLastTick + 1u, kOverheadTick - 1u, kTowardB, kStill },
    { kCharacterA, kOverheadTick, kOverheadTick, kTowardB, kStill, 0.f, 0.f, true },
    { kCharacterA, kOverheadTick + 1u, kBlockedProjectileTick - 3u, kTowardB, kStill },
    { kCharacterA, kBlockedProjectileTick - 2u, kBlockedProjectileTick + 20u, kTowardB, kStill, 0.f, 0.f, false, false, 0u, true },
    { kCharacterA, kBlockedProjectileTick + 21u, kSecondOverheadTick - 1u, kTowardB, kStill },
    { kCharacterA, kSecondOverheadTick, kSecondOverheadTick, kTowardB, kStill, 0.f, 0.f, true },
    { kCharacterA, kSecondOverheadTick + 1u, kScenarioTicks, kTowardB, kStill },

    { kCharacterB, 1u, kRightSwingTick - 6u, kTowardA, kStill },
    { kCharacterB, kRightSwingTick - 5u, kLeftSwingTick - 6u, kGuardBlocksRightSwing, kStill, 0.f, 0.f, false, false, 0u, true },
    { kCharacterB, kLeftSwingTick - 5u, kKnockbackSwingTick - 6u, kGuardBlocksLeftSwing, kStill, 0.f, 0.f, false, false, 0u, true },
    { kCharacterB, kKnockbackSwingTick - 5u, kOverheadTick + 60u, kTowardB, kStill },
    { kCharacterB, kOverheadTick + 61u, kBlockedProjectileTick - 1u, kTowardA, kStill },
    { kCharacterB, kBlockedProjectileTick, kBlockedProjectileTick, kTowardA, kStill, 0.f, 0.f, false, false,
        inputSequence::kHadoukenActionId },
    { kCharacterB, kBlockedProjectileTick + 1u, kHitProjectileTick - 1u, kTowardA, kStill },
    { kCharacterB, kHitProjectileTick, kHitProjectileTick, kTowardA, kStill, 0.f, 0.f, false, false,
        inputSequence::kHadoukenActionId },
    { kCharacterB, kHitProjectileTick + 1u, kScenarioTicks, kTowardA, kStill },
};

inline std::optional<InputRow> scriptRowAt(unsigned int characterId, uint32_t tick)
{
    std::optional<InputRow> found;
    for (const InputRow& row : kInputScript)
    {
        if (row.characterId == characterId && tick >= row.firstTick && tick <= row.lastTick)
        {
            OG_CHECK(!found.has_value(), "goldenTraceScenario::scriptRowAt: two input rows cover the same character and tick");
            found = row;
        }
    }
    return found;
}

inline simulatableBrawler::PlayerInput inputAt(unsigned int characterId, uint32_t tick)
{
    const std::optional<InputRow> row = scriptRowAt(characterId, tick);
    OG_CHECK(row.has_value(), "goldenTraceScenario::inputAt: every character needs an input row on every scenario tick");
    simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
    input.aimDirection = row->aim;
    input.attackLeft = row->attackLeft;
    input.attackRight = row->attackRight;
    input.moveStick = glm::vec2(row->stickX, row->stickY);
    input.moveDirectionWorld = row->move;
    input.triggeredActionId = row->actionId;
    input.flags = row->holdGuard ? brawlerMovementSimulation::kInputFlagHoldGuard : uint8_t(0u);
    return input;
}

} // namespace goldenTraceScenario
