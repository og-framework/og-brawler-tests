#pragma once
// SPDX-License-Identifier: BUSL-1.1
// docs/GoldenTraceSlopeScenario-rationale.md

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "glm/gtc/quaternion.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "GoldenTraceScenario.h"
#include "GoldenTraceScene.h"

#include "OGSimulation/OGAssert.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

namespace goldenTraceSlopeScenario
{

using goldenTraceScenario::InputRow;
using goldenTraceScenario::kDtSeconds;
using goldenTraceScenario::kGuardOffsetCos;
using goldenTraceScenario::kGuardOffsetSin;

inline constexpr unsigned int kCharacterA = 1u;
inline constexpr unsigned int kCharacterB = 2u;
inline constexpr unsigned int kCharacterC = 3u;
inline constexpr unsigned int kCharacterD = 4u;
inline constexpr unsigned int kCharacterE = 5u;
inline constexpr unsigned int kCharacterF = 6u;
inline constexpr unsigned int kCharacterG = 7u;

inline constexpr float kSlopeAngleDeg = 30.f;
inline constexpr float kHalfAngleCos = 0.9659258f;
inline constexpr float kHalfAngleSin = 0.25881904f;
inline constexpr glm::vec3 kFaceNormal{ 0.f, 0.8660254f, 0.5f };
inline constexpr glm::vec3 kUpTheFallLine{ 0.f, 0.5f, -0.8660254f };
inline constexpr glm::vec3 kFacePoint{ 0.f, 8.f, 0.f };
inline constexpr glm::vec3 kSlopeCentre{ 0.f, 7.7834935f, -0.125f };
inline constexpr glm::vec3 kSlopeHalfExtents{ 6.f, 0.25f, 12.f };

inline constexpr float kLowerCreaseZ = 10.392304f;
inline constexpr float kLowerCreaseHeight = 2.f;
inline constexpr float kUpperCreaseZ = -10.392304f;
inline constexpr float kUpperCreaseHeight = 14.f;
inline constexpr float kHalfWidth = 6.f;

inline constexpr glm::vec3 kLandingCentre{ 0.f, 1.75f, 15.892304f };
inline constexpr glm::vec3 kLandingHalfExtents{ 6.f, 0.25f, 6.5f };
inline constexpr glm::vec3 kPlateauCentre{ 0.f, 13.75f, -14.392304f };
inline constexpr glm::vec3 kPlateauHalfExtents{ 6.f, 0.25f, 4.f };

inline constexpr glm::vec3 kFaceA{ 0.f, 8.f, 0.f };
inline constexpr glm::vec3 kFaceB{ 0.f, 9.f, -1.7320508f };
inline constexpr glm::vec3 kFaceC{ 0.f, 7.f, 1.7320508f };
inline constexpr glm::vec3 kFaceD{ -2.f, 8.f, 0.f };
inline constexpr glm::vec3 kFaceE{ 2.f, 8.f, 0.f };
inline constexpr glm::vec3 kLowerF{ -1.f, 2.15f, 10.132497f };
inline constexpr glm::vec3 kLowerG{ 1.f, 2.f, 10.692305f };
inline constexpr glm::vec3 kUpperF{ -1.f, 13.95f, -10.305702f };
inline constexpr glm::vec3 kUpperG{ 1.f, 14.f, -10.692305f };
inline constexpr glm::vec3 kCreaseWalkStart{ -4.5f, 2.5f, 9.526279f };

inline constexpr float kSpawnHeight = 1.5f;
inline constexpr float kTeleportHeight = 1.2f;

constexpr glm::vec3 raised(const glm::vec3& facePoint, float height)
{
    return glm::vec3(facePoint.x, facePoint.y + height, facePoint.z);
}

inline constexpr glm::vec3 kTowardUpSlope{ 0.f, 0.f, -1.f };
inline constexpr glm::vec3 kTowardDownSlope{ 0.f, 0.f, 1.f };
inline constexpr glm::vec3 kTowardMinusX{ -1.f, 0.f, 0.f };
inline constexpr glm::vec3 kTowardPlusX{ 1.f, 0.f, 0.f };
inline constexpr glm::vec3 kStill{ 0.f, 0.f, 0.f };
inline constexpr glm::vec3 kLowerFToG{ 0.9629881f, 0.f, 0.26954404f };
inline constexpr glm::vec3 kLowerGToF{ -0.9629881f, 0.f, -0.26954404f };
inline constexpr glm::vec3 kUpperFToG{ 0.9818251f, 0.f, -0.18978804f };
inline constexpr glm::vec3 kUpperGToF{ -0.9818251f, 0.f, 0.18978804f };
inline constexpr glm::vec3 kCGuardBlocksRightSwing{ kGuardOffsetSin, 0.f, -kGuardOffsetCos };
inline constexpr glm::vec3 kCGuardBlocksLeftSwing{ -kGuardOffsetSin, 0.f, -kGuardOffsetCos };

inline constexpr uint32_t kIdleTicks = 60u;
inline constexpr uint32_t kSettleTicks = 35u;
inline constexpr uint32_t kSlideSlotTicks = 110u;
inline constexpr uint32_t kLaunchSlotTicks = 180u;
inline constexpr uint32_t kShotSlotTicks = 95u;
inline constexpr uint32_t kCreaseWalkSlotTicks = 165u;
inline constexpr uint32_t kSlotTicks[] = { kLaunchSlotTicks, kSlideSlotTicks, kSlideSlotTicks, kLaunchSlotTicks, kLaunchSlotTicks,
    kSlideSlotTicks, kSlideSlotTicks, kLaunchSlotTicks, kShotSlotTicks, kCreaseWalkSlotTicks };
inline constexpr std::size_t kSlotCount = std::size(kSlotTicks);

constexpr uint32_t slotStart(std::size_t slot)
{
    uint32_t start = kIdleTicks + 1u;
    for (std::size_t index = 0; index + 1u < slot; ++index)
        start += kSlotTicks[index];
    return start;
}

constexpr uint32_t actionTick(std::size_t slot) { return slotStart(slot) + kSettleTicks; }

inline constexpr uint32_t kScenarioTicks = slotStart(kSlotCount + 1u) - 1u;
static_assert(kScenarioTicks <= 1500u, "the slope scenario stays within its 1500-tick budget");

constexpr bool isSampleTick(uint32_t tick)
{
    return tick <= goldenTraceScenario::kDenseThroughTick || tick % goldenTraceScenario::kSparseEveryTicks == 0u;
}

inline constexpr uint32_t kCreaseWalkTeleportTick = slotStart(10u);
inline constexpr uint32_t kCreaseWalkFirstTick = kCreaseWalkTeleportTick + 25u;
inline constexpr uint32_t kCreaseWalkLastTick = kCreaseWalkFirstTick + 59u;
inline constexpr uint32_t kCreaseSwingTick = kCreaseWalkLastTick + 26u;

inline simulatableBrawler::StaticData slopeStaticData(float gravityScale)
{
    return goldenTraceScenario::scenarioStaticData(gravityScale);
}

inline StaticWorldDescription slopeStatics()
{
    const glm::quat identity = glm::quat(1.f, 0.f, 0.f, 0.f);
    StaticWorldDescription description;
    description.shapes.push_back(goldenTraceScenario::staticBox(kSlopeCentre, glm::quat(kHalfAngleCos, kHalfAngleSin, -0.f, 0.f),
        kSlopeHalfExtents, 1u));
    description.shapes.push_back(goldenTraceScenario::staticBox(kLandingCentre, identity, kLandingHalfExtents, 2u));
    description.shapes.push_back(goldenTraceScenario::staticBox(kPlateauCentre, identity, kPlateauHalfExtents, 3u));
    return description;
}

enum class Swing
{
    Right,
    Left,
    Overhead,
};

constexpr InputRow still(unsigned int id, uint32_t firstTick, uint32_t lastTick, const glm::vec3& aim)
{
    return InputRow{ id, firstTick, lastTick, aim, kStill };
}

constexpr InputRow guard(unsigned int id, uint32_t firstTick, uint32_t lastTick, const glm::vec3& aim)
{
    return InputRow{ id, firstTick, lastTick, aim, kStill, 0.f, 0.f, false, false, 0u, true };
}

constexpr InputRow walk(unsigned int id, uint32_t firstTick, uint32_t lastTick, const glm::vec3& direction)
{
    return InputRow{ id, firstTick, lastTick, direction, direction, 0.f, 1.f };
}

constexpr InputRow shoot(unsigned int id, uint32_t tick, const glm::vec3& aim)
{
    return InputRow{ id, tick, tick, aim, kStill, 0.f, 0.f, false, false, inputSequence::kHadoukenActionId };
}

constexpr InputRow swing(unsigned int id, uint32_t tick, const glm::vec3& aim, Swing side)
{
    if (side == Swing::Overhead)
        return InputRow{ id, tick, tick, aim, kStill, 0.f, 0.f, true };
    if (side == Swing::Right)
        return InputRow{ id, tick, tick, aim, glm::vec3(aim.z, 0.f, -aim.x), 1.f, 0.f, true };
    return InputRow{ id, tick, tick, aim, glm::vec3(-aim.z, 0.f, aim.x), -1.f, 0.f, true };
}

inline constexpr InputRow kInputScript[] = {
    still(kCharacterA, 1u, actionTick(1u) - 1u, kTowardUpSlope),
    swing(kCharacterA, actionTick(1u), kTowardUpSlope, Swing::Right),
    still(kCharacterA, actionTick(1u) + 1u, slotStart(2u) - 1u, kTowardUpSlope),
    still(kCharacterA, slotStart(2u), actionTick(2u) - 1u, kTowardDownSlope),
    swing(kCharacterA, actionTick(2u), kTowardDownSlope, Swing::Right),
    still(kCharacterA, actionTick(2u) + 1u, slotStart(3u) - 1u, kTowardDownSlope),
    still(kCharacterA, slotStart(3u), actionTick(3u) - 1u, kTowardMinusX),
    swing(kCharacterA, actionTick(3u), kTowardMinusX, Swing::Left),
    still(kCharacterA, actionTick(3u) + 1u, slotStart(4u) - 1u, kTowardMinusX),
    still(kCharacterA, slotStart(4u), actionTick(4u) - 1u, kTowardPlusX),
    swing(kCharacterA, actionTick(4u), kTowardPlusX, Swing::Left),
    still(kCharacterA, actionTick(4u) + 1u, slotStart(5u) - 1u, kTowardPlusX),
    still(kCharacterA, slotStart(5u), actionTick(5u) - 1u, kTowardUpSlope),
    swing(kCharacterA, actionTick(5u), kTowardUpSlope, Swing::Overhead),
    still(kCharacterA, actionTick(5u) + 1u, slotStart(6u) - 1u, kTowardUpSlope),
    still(kCharacterA, slotStart(6u), actionTick(6u) - 1u, kTowardDownSlope),
    swing(kCharacterA, actionTick(6u), kTowardDownSlope, Swing::Right),
    still(kCharacterA, actionTick(6u) + 1u, actionTick(7u) - 1u, kTowardDownSlope),
    swing(kCharacterA, actionTick(7u), kTowardDownSlope, Swing::Left),
    still(kCharacterA, actionTick(7u) + 1u, actionTick(9u) - 1u, kTowardDownSlope),
    shoot(kCharacterA, actionTick(9u), kTowardDownSlope),
    still(kCharacterA, actionTick(9u) + 1u, kCreaseWalkFirstTick - 1u, kTowardDownSlope),
    walk(kCharacterA, kCreaseWalkFirstTick, kCreaseWalkLastTick, kTowardDownSlope),
    still(kCharacterA, kCreaseWalkLastTick + 1u, kCreaseSwingTick - 1u, kTowardDownSlope),
    swing(kCharacterA, kCreaseSwingTick, kTowardDownSlope, Swing::Right),
    still(kCharacterA, kCreaseSwingTick + 1u, kScenarioTicks, kTowardDownSlope),

    still(kCharacterB, 1u, slotStart(5u) - 1u, kTowardDownSlope),
    guard(kCharacterB, slotStart(5u), slotStart(6u) - 1u, kTowardDownSlope),
    still(kCharacterB, slotStart(6u), actionTick(8u) - 1u, kTowardDownSlope),
    shoot(kCharacterB, actionTick(8u), kTowardDownSlope),
    still(kCharacterB, actionTick(8u) + 1u, kScenarioTicks, kTowardDownSlope),

    still(kCharacterC, 1u, slotStart(6u) - 1u, kTowardUpSlope),
    guard(kCharacterC, slotStart(6u), slotStart(7u) - 1u, kCGuardBlocksRightSwing),
    guard(kCharacterC, slotStart(7u), slotStart(8u) - 1u, kCGuardBlocksLeftSwing),
    still(kCharacterC, slotStart(8u), slotStart(9u) - 1u, kTowardUpSlope),
    guard(kCharacterC, slotStart(9u), slotStart(10u) - 1u, kTowardUpSlope),
    still(kCharacterC, slotStart(10u), kScenarioTicks, kTowardUpSlope),

    still(kCharacterD, 1u, kScenarioTicks, kTowardPlusX),
    still(kCharacterE, 1u, kScenarioTicks, kTowardMinusX),

    still(kCharacterF, 1u, actionTick(1u) - 1u, kLowerFToG),
    swing(kCharacterF, actionTick(1u), kLowerFToG, Swing::Left),
    still(kCharacterF, actionTick(1u) + 1u, actionTick(2u) - 1u, kLowerFToG),
    swing(kCharacterF, actionTick(2u), kLowerFToG, Swing::Right),
    still(kCharacterF, actionTick(2u) + 1u, slotStart(5u) - 1u, kLowerFToG),
    still(kCharacterF, slotStart(5u), actionTick(5u) - 1u, kUpperFToG),
    swing(kCharacterF, actionTick(5u), kUpperFToG, Swing::Left),
    still(kCharacterF, actionTick(5u) + 1u, actionTick(6u) - 1u, kUpperFToG),
    swing(kCharacterF, actionTick(6u), kUpperFToG, Swing::Right),
    still(kCharacterF, actionTick(6u) + 1u, kScenarioTicks, kUpperFToG),

    still(kCharacterG, 1u, actionTick(3u) - 1u, kLowerGToF),
    swing(kCharacterG, actionTick(3u), kLowerGToF, Swing::Right),
    still(kCharacterG, actionTick(3u) + 1u, actionTick(4u) - 1u, kLowerGToF),
    swing(kCharacterG, actionTick(4u), kLowerGToF, Swing::Left),
    still(kCharacterG, actionTick(4u) + 1u, slotStart(5u) - 1u, kLowerGToF),
    still(kCharacterG, slotStart(5u), actionTick(7u) - 1u, kUpperGToF),
    swing(kCharacterG, actionTick(7u), kUpperGToF, Swing::Right),
    still(kCharacterG, actionTick(7u) + 1u, actionTick(8u) - 1u, kUpperGToF),
    swing(kCharacterG, actionTick(8u), kUpperGToF, Swing::Left),
    still(kCharacterG, actionTick(8u) + 1u, kScenarioTicks, kUpperGToF),
};

inline constexpr goldenTraceScene::Character kCharacters[] = {
    { kCharacterA, 0u, "A.", raised(kFaceA, kSpawnHeight) },
    { kCharacterB, 1u, "B.", raised(kFaceB, kSpawnHeight) },
    { kCharacterC, 2u, "C.", raised(kFaceC, kSpawnHeight) },
    { kCharacterD, 3u, "D.", raised(kFaceD, kSpawnHeight) },
    { kCharacterE, 4u, "E.", raised(kFaceE, kSpawnHeight) },
    { kCharacterF, 5u, "F.", raised(kLowerF, kSpawnHeight) },
    { kCharacterG, 6u, "G.", raised(kLowerG, kSpawnHeight) },
};

inline constexpr goldenTraceScene::SpawnPoint kSpawnTable[] = {
    { 0u, raised(kFaceA, kSpawnHeight) },
    { 1u, raised(kFaceB, kSpawnHeight) },
    { 2u, raised(kFaceC, kSpawnHeight) },
    { 3u, raised(kFaceD, kSpawnHeight) },
    { 4u, raised(kFaceE, kSpawnHeight) },
    { 5u, raised(kLowerF, kSpawnHeight) },
    { 6u, raised(kLowerG, kSpawnHeight) },
};

inline constexpr goldenTraceScene::Teleport kTeleports[] = {
    { kCharacterF, slotStart(2u), raised(kLowerF, kTeleportHeight) },
    { kCharacterG, slotStart(2u), raised(kLowerG, kTeleportHeight) },
    { kCharacterF, slotStart(3u), raised(kLowerF, kTeleportHeight) },
    { kCharacterG, slotStart(3u), raised(kLowerG, kTeleportHeight) },
    { kCharacterF, slotStart(4u), raised(kLowerF, kTeleportHeight) },
    { kCharacterG, slotStart(4u), raised(kLowerG, kTeleportHeight) },
    { kCharacterF, slotStart(5u), raised(kUpperF, kTeleportHeight) },
    { kCharacterG, slotStart(5u), raised(kUpperG, kTeleportHeight) },
    { kCharacterF, slotStart(6u), raised(kUpperF, kTeleportHeight) },
    { kCharacterG, slotStart(6u), raised(kUpperG, kTeleportHeight) },
    { kCharacterF, slotStart(7u), raised(kUpperF, kTeleportHeight) },
    { kCharacterG, slotStart(7u), raised(kUpperG, kTeleportHeight) },
    { kCharacterF, slotStart(8u), raised(kUpperF, kTeleportHeight) },
    { kCharacterG, slotStart(8u), raised(kUpperG, kTeleportHeight) },
    { kCharacterA, kCreaseWalkTeleportTick, raised(kCreaseWalkStart, kTeleportHeight) },
};

inline std::optional<InputRow> scriptRowAt(unsigned int characterId, uint32_t tick)
{
    std::optional<InputRow> found;
    for (const InputRow& row : kInputScript)
    {
        if (row.characterId == characterId && tick >= row.firstTick && tick <= row.lastTick)
        {
            OG_CHECK(!found.has_value(), "goldenTraceSlopeScenario::scriptRowAt: two input rows cover the same character and tick");
            found = row;
        }
    }
    return found;
}

inline simulatableBrawler::PlayerInput inputAt(unsigned int characterId, uint32_t tick)
{
    const std::optional<InputRow> row = scriptRowAt(characterId, tick);
    OG_CHECK(row.has_value(), "goldenTraceSlopeScenario::inputAt: every character needs an input row on every scenario tick");
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

struct Scenario
{
    static constexpr const char* kGenerator = "BrawlerJoltSlopeGoldenTrace.Record";
    static constexpr float kDtSeconds = goldenTraceSlopeScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = goldenTraceSlopeScenario::kScenarioTicks;

    static bool isSampleTick(uint32_t tick) { return goldenTraceSlopeScenario::isSampleTick(tick); }
    static StaticWorldDescription statics() { return slopeStatics(); }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return slopeStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int characterId, uint32_t tick)
    {
        return goldenTraceSlopeScenario::inputAt(characterId, tick);
    }
    static std::span<const goldenTraceScene::Character> characters() { return kCharacters; }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable() { return kSpawnTable; }
    static std::span<const goldenTraceScene::Teleport> teleports() { return kTeleports; }
};

static_assert(goldenTraceScene::Scenario<Scenario>, "the slope scenario provides everything goldenTraceScene::Scene reads");
static_assert(std::size(kCharacters) <= kMaxSimulatableSlots, "the slope scenario's characters fit the world's body slots");

} // namespace goldenTraceSlopeScenario
