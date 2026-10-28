// SPDX-License-Identifier: BUSL-1.1
// docs/BrawlerJoltSlopeGoldenTraceTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceFormat.h"
#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"

#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/HitReaction.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#if __has_include("BrawlerJoltSlopeGolden.inl") && __has_include("BrawlerJoltSlopeGoldenEnvelope.inl")
namespace slopeGoldenData
{
#include "BrawlerJoltSlopeGolden.inl"
#include "BrawlerJoltSlopeGoldenEnvelope.inl"
} // namespace slopeGoldenData

namespace
{
inline constexpr bool kSlopeGoldenRecorded = true;

goldenTrace::GoldenView recordedSlopeGolden()
{
    using namespace slopeGoldenData;
    static_assert(goldenTrace::everyFieldHasABound(goldenTraceData::kGoldenFields),
        "BrawlerJoltSlopeGolden.inl: every recorded field has a bound class (a 'p' field's last name segment is position, "
        "linearVelocity or velocity, and no field is 'd')");
    return goldenTrace::GoldenView{ &goldenTraceData::kGoldenHeader, goldenTraceData::kGoldenFields,
        std::size(goldenTraceData::kGoldenFields), goldenTraceData::kGoldenTicks, std::size(goldenTraceData::kGoldenTicks),
        goldenTraceData::kGoldenValues, std::size(goldenTraceData::kGoldenValues) };
}

goldenTrace::EnvelopeView recordedSlopeEnvelope()
{
    using namespace slopeGoldenData;
    static_assert(sizeof(goldenTraceEnvelopeData::kEnvelopeTagOrder) - 1u == goldenTrace::kTagCount,
        "BrawlerJoltSlopeGoldenEnvelope.inl stores one envelope column per golden tag");
    return goldenTrace::EnvelopeView{ &goldenTraceEnvelopeData::kEnvelopeHeader, goldenTraceEnvelopeData::kEnvelopeNudges,
        std::size(goldenTraceEnvelopeData::kEnvelopeNudges), goldenTraceEnvelopeData::kEnvelopeTicks,
        std::size(goldenTraceEnvelopeData::kEnvelopeTicks), goldenTraceEnvelopeData::kEnvelopeValues,
        std::size(goldenTraceEnvelopeData::kEnvelopeValues) };
}
} // namespace
#else
namespace
{
inline constexpr bool kSlopeGoldenRecorded = false;
goldenTrace::GoldenView recordedSlopeGolden() { return {}; }
goldenTrace::EnvelopeView recordedSlopeEnvelope() { return {}; }
} // namespace
#endif

namespace
{

using SlopeScenario = goldenTraceSlopeScenario::Scenario;
using SlopeScene = goldenTraceScene::Scene<SlopeScenario>;

inline constexpr double kNormalTolerance = 1e-6;
inline constexpr double kFaceInteriorMargin = 0.5;
inline constexpr double kOnFaceDistance = 1e-3;
inline constexpr double kStillDisplacement = 1e-3;
inline constexpr uint32_t kStillTicksBeforeAnAction = 20u;

struct Rectangle
{
    glm::dvec3 centre;
    glm::dvec3 axisU;
    double halfU;
    glm::dvec3 axisV;
    double halfV;
};

glm::dvec3 analyticFaceNormal()
{
    const double angle = double(goldenTraceSlopeScenario::kSlopeAngleDeg) * 3.14159265358979323846 / 180.0;
    return glm::dvec3(0.0, std::cos(angle), std::sin(angle));
}

glm::dvec3 analyticUpTheFallLine()
{
    const double angle = double(goldenTraceSlopeScenario::kSlopeAngleDeg) * 3.14159265358979323846 / 180.0;
    return glm::dvec3(0.0, std::sin(angle), -std::cos(angle));
}

Rectangle slopeFace()
{
    return Rectangle{ glm::dvec3(goldenTraceSlopeScenario::kFacePoint), glm::dvec3(1.0, 0.0, 0.0),
        double(goldenTraceSlopeScenario::kSlopeHalfExtents.x), analyticUpTheFallLine(),
        double(goldenTraceSlopeScenario::kSlopeHalfExtents.z) };
}

std::vector<Rectangle> exposedGround()
{
    const double halfWidth = double(goldenTraceSlopeScenario::kHalfWidth);
    const double lowerCrease = double(goldenTraceSlopeScenario::kLowerCreaseZ);
    const double landingFar = double(goldenTraceSlopeScenario::kLandingCentre.z + goldenTraceSlopeScenario::kLandingHalfExtents.z);
    const double upperCrease = double(goldenTraceSlopeScenario::kUpperCreaseZ);
    const double plateauFar = double(goldenTraceSlopeScenario::kPlateauCentre.z - goldenTraceSlopeScenario::kPlateauHalfExtents.z);
    return { slopeFace(),
        Rectangle{ glm::dvec3(0.0, double(goldenTraceSlopeScenario::kLowerCreaseHeight), 0.5 * (lowerCrease + landingFar)),
            glm::dvec3(1.0, 0.0, 0.0), halfWidth, glm::dvec3(0.0, 0.0, 1.0), 0.5 * (landingFar - lowerCrease) },
        Rectangle{ glm::dvec3(0.0, double(goldenTraceSlopeScenario::kUpperCreaseHeight), 0.5 * (upperCrease + plateauFar)),
            glm::dvec3(1.0, 0.0, 0.0), halfWidth, glm::dvec3(0.0, 0.0, 1.0), 0.5 * (upperCrease - plateauFar) } };
}

double distanceToRectangle(const glm::dvec3& point, const Rectangle& rectangle)
{
    const glm::dvec3 offset = point - rectangle.centre;
    const double u = std::clamp(glm::dot(offset, rectangle.axisU), -rectangle.halfU, rectangle.halfU);
    const double v = std::clamp(glm::dot(offset, rectangle.axisV), -rectangle.halfV, rectangle.halfV);
    return glm::length(offset - u * rectangle.axisU - v * rectangle.axisV);
}

double clearanceOf(const glm::dvec3& point, const std::vector<Rectangle>& ground)
{
    double nearest = std::numeric_limits<double>::infinity();
    for (const Rectangle& rectangle : ground)
        nearest = std::min(nearest, distanceToRectangle(point, rectangle));
    return nearest;
}

bool isOnTheFaceInterior(const glm::dvec3& point)
{
    const Rectangle face = slopeFace();
    const glm::dvec3 offset = point - face.centre;
    return std::fabs(glm::dot(offset, analyticFaceNormal())) < kOnFaceDistance
        && std::fabs(glm::dot(offset, face.axisU)) < face.halfU - kFaceInteriorMargin
        && std::fabs(glm::dot(offset, face.axisV)) < face.halfV - kFaceInteriorMargin;
}

struct CharacterTick
{
    glm::dvec3 position;
    bool supported;
    bool hitFlinch;
    bool knockedBack;
    bool inboundHit;
    bool inboundBlocked;
    glm::dvec2 hitDirection;
    glm::dvec3 surfaceNormal;
    glm::dvec3 probePoint;
    unsigned int meleeTargetThisTick;
};

CharacterTick sampleOf(const SlopeScene& scene, unsigned int id, uint32_t tick)
{
    const goldenTraceScene::DetectionLog& log = scene.detections.at(id);
    const SimulatableBrawler& brawler = scene.storage.get<SimulatableBrawler>(id);
    const auto& state = brawler.getAllState().getState();
    const auto& derived = brawler.getAllState().getDerivedState();
    const auto& movement = state.get<brawlerMovementSimulation::State>();
    const auto& movementDerived = derived.get<brawlerMovementSimulation::DerivedState>();
    const auto& machine = state.get<dAttackMachineSimulation::State>();
    const auto& inbound = derived.get<brawlerInboundHit::DerivedState>();
    const uint32_t support = (movement.flags & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift;
    return CharacterTick{ glm::dvec3(movement.bodyState.position),
        support == static_cast<uint32_t>(brawlerMovementSimulation::SupportState::Supported),
        machine.m_currentState == DAttackState::HitFlinch,
        inbound.wasHitThisTick && inbound.reactionKind == HitReactionKind::Knockback, inbound.wasHitThisTick,
        inbound.wasGuardBlockedThisTick, glm::dvec2(inbound.hitDirectionXY), glm::dvec3(movementDerived.surfaceNormal),
        glm::dvec3(movementDerived.lastProbePoint),
        log.meleeHits > 0u && log.lastMeleeHitTick == tick ? static_cast<unsigned int>(log.lastMeleeTarget) : 0u };
}

struct Knockback
{
    unsigned int attacker = 0u;
    unsigned int target = 0u;
    uint32_t hitTick = 0u;
    uint32_t windowEndTick = 0u;
    glm::dvec2 direction{ 0.0 };
    double travel = 0.0;
    uint32_t unsupportedTicks = 0u;
    double apex = 0.0;
    double airtime = 0.0;
    uint32_t lockoutEndTick = 0u;
    double minClearance = 0.0;
    double restClearance = 0.0;
};

struct SlopeMeasurements
{
    double idleDriftA = 0.0;
    uint32_t slopeTicks = 0u;
    uint32_t normalDiffersTicks = 0u;
    double maxNormalDifference = 0.0;
    uint32_t unsettledPhaseStarts = 0u;
    uint32_t creaseSwings = 0u;
    uint32_t creaseHits = 0u;
    std::vector<Knockback> knockbacks;
    std::unordered_map<unsigned int, uint32_t> inboundHits;
    std::unordered_map<unsigned int, uint32_t> inboundBlocks;
};

uint32_t nextResetTick(unsigned int id, uint32_t afterTick)
{
    uint32_t next = SlopeScenario::kScenarioTicks + 1u;
    for (const goldenTraceScene::Teleport& teleport : SlopeScenario::teleports())
        if (teleport.characterId == id && teleport.tick > afterTick)
            next = std::min(next, teleport.tick);
    return next;
}

std::vector<uint32_t> actionTicks()
{
    std::vector<uint32_t> ticks;
    for (std::size_t slot = 1u; slot < goldenTraceSlopeScenario::kSlotCount; ++slot)
        ticks.push_back(goldenTraceSlopeScenario::actionTick(slot));
    ticks.push_back(goldenTraceSlopeScenario::kCreaseSwingTick);
    return ticks;
}

bool isACreaseCharacter(unsigned int id)
{
    return id == goldenTraceSlopeScenario::kCharacterF || id == goldenTraceSlopeScenario::kCharacterG;
}

SlopeMeasurements measureTheSlope()
{
    SlopeScene scene(std::nullopt, 1.f);
    const auto characters = SlopeScenario::characters();
    const std::vector<Rectangle> ground = exposedGround();
    const glm::dvec3 faceNormal = analyticFaceNormal();

    std::vector<std::vector<CharacterTick>> ticks(characters.size());
    for (std::size_t index = 0; index < characters.size(); ++index)
        ticks[index].push_back(sampleOf(scene, characters[index].id, 0u));
    for (uint32_t tick = 1u; tick <= SlopeScenario::kScenarioTicks; ++tick)
    {
        scene.step(tick);
        for (std::size_t index = 0; index < characters.size(); ++index)
            ticks[index].push_back(sampleOf(scene, characters[index].id, tick));
    }

    SlopeMeasurements measured;
    const std::vector<CharacterTick>& tickOfA = ticks[0];
    std::optional<glm::dvec3> restA;
    for (uint32_t tick = 1u; tick <= goldenTraceSlopeScenario::kIdleTicks; ++tick)
    {
        if (!tickOfA[tick].supported)
            continue;
        if (!restA.has_value())
            restA = tickOfA[tick].position;
        const glm::dvec3 drift = tickOfA[tick].position - *restA;
        measured.idleDriftA = std::max(measured.idleDriftA, std::hypot(drift.x, drift.z));
    }

    for (std::size_t index = 0; index < characters.size(); ++index)
    {
        const unsigned int id = characters[index].id;
        for (uint32_t tick = 1u; tick <= SlopeScenario::kScenarioTicks; ++tick)
        {
            const CharacterTick& sample = ticks[index][tick];
            measured.inboundHits[id] += sample.inboundHit ? 1u : 0u;
            measured.inboundBlocks[id] += sample.inboundBlocked ? 1u : 0u;
            if (sample.supported && isOnTheFaceInterior(sample.probePoint))
            {
                ++measured.slopeTicks;
                const glm::dvec3 difference = glm::abs(sample.surfaceNormal - faceNormal);
                const double largest = std::max({ difference.x, difference.y, difference.z });
                measured.maxNormalDifference = std::max(measured.maxNormalDifference, largest);
                measured.normalDiffersTicks += largest > kNormalTolerance ? 1u : 0u;
            }
            if (!sample.knockedBack)
                continue;

            Knockback knockback;
            knockback.target = id;
            knockback.hitTick = tick;
            knockback.direction = sample.hitDirection;
            for (std::size_t other = 0; other < characters.size(); ++other)
                if (ticks[other][tick].meleeTargetThisTick == id)
                    knockback.attacker = characters[other].id;
            uint32_t windowEnd = nextResetTick(id, tick) - 1u;
            for (uint32_t later = tick + 1u; later <= windowEnd; ++later)
                if (ticks[index][later].knockedBack)
                {
                    windowEnd = later - 1u;
                    break;
                }
            knockback.windowEndTick = windowEnd;
            const glm::dvec3 launchPoint = ticks[index][tick - 1u].position;
            knockback.restClearance = clearanceOf(launchPoint, ground);
            knockback.minClearance = knockback.restClearance;
            std::optional<uint32_t> firstUnsupported;
            std::optional<uint32_t> landed;
            for (uint32_t later = tick; later <= windowEnd; ++later)
            {
                const CharacterTick& during = ticks[index][later];
                knockback.unsupportedTicks += during.supported ? 0u : 1u;
                knockback.apex = std::max(knockback.apex, during.position.y - launchPoint.y);
                knockback.minClearance = std::min(knockback.minClearance, clearanceOf(during.position, ground));
                if (!during.supported && !firstUnsupported.has_value())
                    firstUnsupported = later;
                if (during.supported && firstUnsupported.has_value() && !landed.has_value())
                    landed = later;
                if (!during.hitFlinch && knockback.lockoutEndTick == 0u)
                    knockback.lockoutEndTick = later;
            }
            if (firstUnsupported.has_value())
                knockback.airtime = double(landed.value_or(windowEnd + 1u) - *firstUnsupported) * double(SlopeScenario::kDtSeconds);
            knockback.travel = glm::length(ticks[index][windowEnd].position - launchPoint);
            measured.knockbacks.push_back(knockback);
        }
    }

    for (const uint32_t action : actionTicks())
        for (std::size_t index = 0; index < characters.size(); ++index)
            for (uint32_t tick = action - kStillTicksBeforeAnAction; tick < action; ++tick)
            {
                const CharacterTick& sample = ticks[index][tick];
                const glm::dvec3 moved = sample.position - ticks[index][tick - 1u].position;
                if (!sample.supported || std::hypot(moved.x, moved.z) > kStillDisplacement)
                {
                    ++measured.unsettledPhaseStarts;
                    break;
                }
            }

    for (const goldenTraceScenario::InputRow& row : goldenTraceSlopeScenario::kInputScript)
        measured.creaseSwings += isACreaseCharacter(row.characterId) && row.attackLeft ? 1u : 0u;
    for (const Knockback& knockback : measured.knockbacks)
        measured.creaseHits += isACreaseCharacter(knockback.target) && isACreaseCharacter(knockback.attacker) ? 1u : 0u;
    std::sort(measured.knockbacks.begin(), measured.knockbacks.end(),
        [](const Knockback& first, const Knockback& second) { return first.hitTick < second.hitTick; });
    return measured;
}

const char* prefixOf(unsigned int id)
{
    for (const goldenTraceScene::Character& character : SlopeScenario::characters())
        if (character.id == id)
            return character.prefix;
    return "?";
}

void printSlopeMeasurements(const SlopeMeasurements& measured)
{
    std::string line = "[GoldenTrace.slope]";
    char buffer[512];
    std::snprintf(buffer, sizeof(buffer), " idleDriftAM=%.9g slopeTicks=%u normalDiffersTicks=%u maxNormalDifference=%.3g"
                                          " unsettledPhaseStarts=%u creaseHits=%u/%u knockbacks=%zu",
        measured.idleDriftA, measured.slopeTicks, measured.normalDiffersTicks, measured.maxNormalDifference,
        measured.unsettledPhaseStarts, measured.creaseHits, measured.creaseSwings, measured.knockbacks.size());
    line += buffer;
    line += " inbound=";
    for (const goldenTraceScene::Character& character : SlopeScenario::characters())
    {
        std::snprintf(buffer, sizeof(buffer), "%s%u/%u ", character.prefix, measured.inboundHits.at(character.id),
            measured.inboundBlocks.at(character.id));
        line += buffer;
    }
    for (const Knockback& knockback : measured.knockbacks)
    {
        std::snprintf(buffer, sizeof(buffer),
            "| %s>%stick=%u dir=(%.4f,%.4f) travelM=%.4f unsupportedTicks=%u apexM=%.4f airtimeS=%.4f lockoutEndTick=%u"
            " minClearanceM=%.4f restClearanceM=%.4f windowEndTick=%u ",
            prefixOf(knockback.attacker), prefixOf(knockback.target), knockback.hitTick, knockback.direction.x, knockback.direction.y,
            knockback.travel, knockback.unsupportedTicks, knockback.apex, knockback.airtime, knockback.lockoutEndTick, knockback.minClearance,
            knockback.restClearance, knockback.windowEndTick);
        line += buffer;
    }
    std::printf("%s\n", line.c_str());
}

} // namespace

TEST_CASE("BrawlerJoltSlopeGoldenTrace.ScriptCoversEveryTickOnce", "[GoldenTrace]")
{
    uint32_t coveredTicks = 0;
    for (const goldenTraceScene::Character& character : SlopeScenario::characters())
        for (uint32_t tick = 1u; tick <= SlopeScenario::kScenarioTicks; ++tick)
            coveredTicks += goldenTraceSlopeScenario::scriptRowAt(character.id, tick).has_value() ? 1u : 0u;
    REQUIRE(coveredTicks == uint32_t(SlopeScenario::characters().size()) * SlopeScenario::kScenarioTicks);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.MatchesTheRecordedTrace", "[GoldenTrace]")
{
    goldenTraceScene::requireMatchesTheRecordedTrace<SlopeScenario>(kSlopeGoldenRecorded, recordedSlopeGolden(),
        recordedSlopeEnvelope());
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.PrintsTheSlopeMeasurements", "[GoldenTrace]")
{
    const SlopeMeasurements measured = measureTheSlope();
    printSlopeMeasurements(measured);
    REQUIRE(measured.slopeTicks > 0u);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.Record", "[.GoldenRecord]")
{
    goldenTraceScene::recordGoldenAndEnvelope<SlopeScenario>("BrawlerJoltSlopeGolden.inl", "BrawlerJoltSlopeGoldenEnvelope.inl");
}

#endif
