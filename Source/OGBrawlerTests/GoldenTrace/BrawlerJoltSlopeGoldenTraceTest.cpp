// SPDX-License-Identifier: BUSL-1.1
// docs/BrawlerJoltSlopeGoldenTraceTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceFormat.h"
#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"

#include "OGSimulation/SimulationFrame.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/HitReaction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
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

struct CountingQuery
{
    JoltSpatialQueryAdapter& inner;
    uint32_t sweeps = 0u;

    template <typename... Args>
    decltype(auto) sweep(Args&&... args)
    {
        ++sweeps;
        return inner.sweep(std::forward<Args>(args)...);
    }
};

struct GroundTick
{
    glm::dvec3 attackGround{ 0.0, 1.0, 0.0 };
    glm::dvec3 movementGround{ 0.0, 1.0, 0.0 };
};

GroundTick groundOf(SlopeScene& scene, unsigned int id, CountingQuery& shipped)
{
    const SimulatableBrawler& brawler = scene.storage.get<SimulatableBrawler>(id);
    const auto& movement = brawler.getAllState().getState().get<brawlerMovementSimulation::State>();
    const auto& bindings = brawler.getPhysicsComposite().get<brawlerMovementSimulation::PhysicsDeclaration>().bindings;
    const brawlerMovementSimulation::StaticData& sd = scene.staticData.m_movementStaticData;
    (void)brawlerMovementSimulation::sampleAttackGround(movement, sd, bindings, shipped);
    const brawlerGround::GroundSample sample = brawlerMovementSimulation::sampleAttackGround<true>(movement, sd, bindings, scene.query);
    const brawlerMovementSimulation::GroundProbe probe = brawlerMovementSimulation::probeGround(movement, sd, bindings, scene.query);
    const glm::vec3 face = brawlerMovementSimulation::attackGroundNormal(probe.hit, sd);
    return GroundTick{ glm::dvec3(sample.normal), glm::dvec3(probe.walkable ? face : simFrame::kUp) };
}

struct ShotTick
{
    uint32_t spawnTick = 0u;
    bool alive = false;
    glm::dvec3 position{ 0.0 };
};

struct CharacterTick
{
    glm::dvec3 position;
    bool supported;
    bool hitFlinch;
    bool knockedBack;
    bool stunned;
    bool inboundHit;
    bool inboundBlocked;
    glm::dvec2 hitDirection;
    glm::dvec3 hitDirectionWorld;
    double knockbackSpeed;
    double speed;
    glm::dvec3 surfaceNormal;
    glm::dvec3 probePoint;
    unsigned int meleeTargetThisTick;
    unsigned int radialSequence;
    float initialAimAngle;
    glm::vec3 initialAimRotationAxis;
    glm::quat weaponRotation;
    goldenTraceScene::DetectionLog log;
    std::vector<ShotTick> shots;
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
    const auto& radialIc = state.get<dAttackRadialSimulation::InitialConditions>();
    const auto& radial = state.get<dAttackRadialSimulation::State>();
    const auto& projectiles = state.get<brawlerProjectileSimulation::State>();
    const uint32_t support = (movement.flags & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift;
    std::vector<ShotTick> shots;
    for (const brawlerProjectileSimulation::ProjectileSlot& slot : projectiles.slots)
        shots.push_back(ShotTick{ slot.spawnTick, slot.isAlive(tick), glm::dvec3(slot.bodyState.position) });
    return CharacterTick{ glm::dvec3(movement.bodyState.position),
        support == static_cast<uint32_t>(brawlerMovementSimulation::SupportState::Supported),
        machine.m_currentState == DAttackState::HitFlinch,
        inbound.wasHitThisTick && inbound.reactionKind == HitReactionKind::Knockback,
        inbound.wasHitThisTick && inbound.reactionKind == HitReactionKind::Stun, inbound.wasHitThisTick,
        inbound.wasGuardBlockedThisTick, glm::dvec2(inbound.hitDirectionXY), glm::dvec3(inbound.hitDirectionWorld),
        double(inbound.knockbackSpeed), double(glm::length(movement.velocity)),
        glm::dvec3(movementDerived.surfaceNormal), glm::dvec3(movementDerived.lastProbePoint),
        log.meleeHits > 0u && log.lastMeleeHitTick == tick ? static_cast<unsigned int>(log.lastMeleeTarget) : 0u,
        radial.currenSequenceId, radialIc.initialAimAngle, radialIc.initialAimRotationAxis,
        dAttackRadialSimulation::PhysicsDeclaration::bodyStateOf(radial).rotation, log, std::move(shots) };
}

enum class GroundCrossing
{
    Leaves,
    InPlane,
    Enters,
};

const char* nameOf(GroundCrossing crossing)
{
    return crossing == GroundCrossing::Leaves ? "leaves" : crossing == GroundCrossing::Enters ? "enters" : "inPlane";
}

struct Knockback
{
    unsigned int attacker = 0u;
    unsigned int target = 0u;
    uint32_t hitTick = 0u;
    uint32_t windowEndTick = 0u;
    glm::dvec2 direction{ 0.0 };
    glm::dvec3 directionWorld{ 0.0 };
    double dotTargetGround = 0.0;
    GroundCrossing crossing = GroundCrossing::InPlane;
    double travel = 0.0;
    uint32_t unsupportedTicks = 0u;
    uint32_t firstUnsupportedTick = 0u;
    double knockbackSpeed = 0.0;
    double speedIntoFirstUnsupported = 0.0;
    double apex = 0.0;
    double peakClearanceRise = 0.0;
    double airtime = 0.0;
    uint32_t lockoutEndTick = 0u;
    double minClearance = 0.0;
    double restClearance = 0.0;
};

struct SwingRecord
{
    unsigned int id = 0u;
    uint32_t startTick = 0u;
    unsigned int sequence = 0u;
    uint32_t ticks = 0u;
    bool tilted = false;
    double upError = 0.0;
    double outOfPlane = 0.0;
};

struct Shot
{
    unsigned int shooter = 0u;
    uint32_t spawnTick = 0u;
    uint32_t aliveTicks = 0u;
    uint32_t overFaceTicks = 0u;
    double firstClearance = 0.0;
    double clearanceDeviation = 0.0;
    uint32_t hits = 0u;
    uint32_t blocks = 0u;
    unsigned int lastTarget = 0u;
};

struct Event
{
    unsigned int attacker = 0u;
    unsigned int target = 0u;
    uint32_t tick = 0u;
};

struct Stun
{
    unsigned int attacker = 0u;
    unsigned int target = 0u;
    uint32_t hitTick = 0u;
    uint32_t windowEndTick = 0u;
    uint32_t unsupportedTicks = 0u;
    double horizontalTravel = 0.0;
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
    uint32_t extraCasts = 0u;
    std::vector<Knockback> knockbacks;
    std::vector<SwingRecord> swings;
    std::vector<Shot> shots;
    std::vector<Event> guardBlocks;
    std::vector<Stun> stuns;
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

double largestComponent(const glm::dvec3& value)
{
    const glm::dvec3 magnitude = glm::abs(value);
    return std::max({ magnitude.x, magnitude.y, magnitude.z });
}

bool isOverTheFaceInterior(const glm::dvec3& point)
{
    const Rectangle face = slopeFace();
    const glm::dvec3 offset = point - face.centre;
    return std::fabs(glm::dot(offset, face.axisU)) < face.halfU - kFaceInteriorMargin
        && std::fabs(glm::dot(offset, face.axisV)) < face.halfV - kFaceInteriorMargin;
}

double bladeOutOfPlane(const glm::quat& weaponRotation, const glm::vec3& localAxis, const glm::dvec3& worldAxis)
{
    const glm::dvec3 axis = glm::normalize(glm::dvec3(localAxis));
    const glm::dvec3 reference = std::fabs(axis.z) < 0.9 ? glm::dvec3(0.0, 0.0, 1.0) : glm::dvec3(1.0, 0.0, 0.0);
    const glm::dvec3 first = glm::normalize(glm::cross(axis, reference));
    const glm::dvec3 second = glm::cross(axis, first);
    const glm::dmat3 rotation = glm::mat3_cast(glm::dquat(weaponRotation));
    return std::max(std::fabs(glm::dot(rotation * first, worldAxis)), std::fabs(glm::dot(rotation * second, worldAxis)));
}

std::size_t indexOf(unsigned int id)
{
    const auto characters = SlopeScenario::characters();
    for (std::size_t index = 0; index < characters.size(); ++index)
        if (characters[index].id == id)
            return index;
    return characters.size();
}

SlopeMeasurements measureTheSlope()
{
    SlopeScene scene(std::nullopt, 1.f);
    CountingQuery shipped{ scene.query };
    const auto characters = SlopeScenario::characters();
    const std::vector<Rectangle> ground = exposedGround();
    const glm::dvec3 faceNormal = analyticFaceNormal();
    const glm::dvec3 facePoint = glm::dvec3(goldenTraceSlopeScenario::kFacePoint);

    std::vector<std::vector<CharacterTick>> ticks(characters.size());
    std::vector<std::vector<GroundTick>> grounds(characters.size());
    for (std::size_t index = 0; index < characters.size(); ++index)
    {
        ticks[index].push_back(sampleOf(scene, characters[index].id, 0u));
        grounds[index].push_back(GroundTick{});
    }
    for (uint32_t tick = 1u; tick <= SlopeScenario::kScenarioTicks; ++tick)
    {
        for (std::size_t index = 0; index < characters.size(); ++index)
            grounds[index].push_back(groundOf(scene, characters[index].id, shipped));
        scene.step(tick);
        for (std::size_t index = 0; index < characters.size(); ++index)
            ticks[index].push_back(sampleOf(scene, characters[index].id, tick));
    }

    SlopeMeasurements measured;
    measured.extraCasts = shipped.sweeps;
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

    const auto& sequences = scene.staticData.m_attackSimulationStaticData.getAttackSequences();
    for (std::size_t index = 0; index < characters.size(); ++index)
    {
        const unsigned int id = characters[index].id;
        for (uint32_t tick = 1u; tick <= SlopeScenario::kScenarioTicks; ++tick)
        {
            const CharacterTick& sample = ticks[index][tick];
            const CharacterTick& before = ticks[index][tick - 1u];
            measured.inboundHits[id] += sample.inboundHit ? 1u : 0u;
            measured.inboundBlocks[id] += sample.inboundBlocked ? 1u : 0u;
            if (sample.log.guardBlocks > before.log.guardBlocks)
                measured.guardBlocks.push_back(Event{ id, 0u, tick });
            if (sample.supported && isOnTheFaceInterior(sample.probePoint))
            {
                ++measured.slopeTicks;
                const double largest = largestComponent(sample.surfaceNormal - faceNormal);
                measured.maxNormalDifference = std::max(measured.maxNormalDifference, largest);
                measured.normalDiffersTicks += largest > kNormalTolerance ? 1u : 0u;
            }

            if (isRealAttackSequence(sample.radialSequence) && sample.radialSequence != before.radialSequence)
            {
                SwingRecord swing;
                swing.id = id;
                swing.startTick = tick;
                swing.sequence = sample.radialSequence;
                const glm::mat3 initialAim = glm::mat3(glm::rotate(glm::mat4(1.f), sample.initialAimAngle, sample.initialAimRotationAxis));
                const glm::vec3 localAxis = sequences[swing.sequence].getRotationAxis();
                const glm::dvec3 worldAxis = glm::dvec3(initialAim * localAxis);
                const glm::dvec3& normal = grounds[index][tick].attackGround;
                swing.upError = largestComponent(glm::dvec3(initialAim * simFrame::kUp) - normal);
                swing.tilted = largestComponent(normal - glm::dvec3(simFrame::kUp)) > 1e-3;
                for (uint32_t during = tick; during <= SlopeScenario::kScenarioTicks
                     && ticks[index][during].radialSequence == swing.sequence; ++during)
                {
                    ++swing.ticks;
                    swing.outOfPlane = std::max(swing.outOfPlane,
                        bladeOutOfPlane(ticks[index][during].weaponRotation, localAxis, worldAxis));
                }
                measured.swings.push_back(swing);
            }

            for (std::size_t slot = 0; slot < sample.shots.size(); ++slot)
            {
                const ShotTick& shotTick = sample.shots[slot];
                if (shotTick.spawnTick == 0u || !shotTick.alive)
                    continue;
                auto found = std::find_if(measured.shots.begin(), measured.shots.end(), [&](const Shot& shot)
                    { return shot.shooter == id && shot.spawnTick == shotTick.spawnTick; });
                if (found == measured.shots.end())
                {
                    measured.shots.push_back(Shot{ id, shotTick.spawnTick });
                    found = std::prev(measured.shots.end());
                }
                ++found->aliveTicks;
                if (!isOverTheFaceInterior(shotTick.position))
                    continue;
                const double clearance = glm::dot(shotTick.position - facePoint, faceNormal);
                if (found->overFaceTicks == 0u)
                    found->firstClearance = clearance;
                ++found->overFaceTicks;
                found->clearanceDeviation = std::max(found->clearanceDeviation, std::fabs(clearance - found->firstClearance));
            }
            const bool shotHit = sample.log.projectileHits > before.log.projectileHits;
            const bool shotBlocked = sample.log.projectileBlocks > before.log.projectileBlocks;
            if (shotHit || shotBlocked)
            {
                Shot* latest = nullptr;
                for (Shot& shot : measured.shots)
                    if (shot.shooter == id && shot.spawnTick <= tick && (latest == nullptr || shot.spawnTick > latest->spawnTick))
                        latest = &shot;
                if (latest != nullptr)
                {
                    latest->hits += shotHit ? 1u : 0u;
                    latest->blocks += shotBlocked ? 1u : 0u;
                    latest->lastTarget = sample.log.lastProjectileTarget;
                }
            }

            if (!sample.knockedBack && !sample.stunned)
                continue;
            unsigned int attacker = 0u;
            for (std::size_t other = 0; other < characters.size(); ++other)
                if (ticks[other][tick].meleeTargetThisTick == id)
                    attacker = characters[other].id;
            uint32_t windowEnd = nextResetTick(id, tick) - 1u;
            for (uint32_t later = tick + 1u; later <= windowEnd; ++later)
                if (ticks[index][later].knockedBack || ticks[index][later].stunned)
                {
                    windowEnd = later - 1u;
                    break;
                }
            const glm::dvec3 launchPoint = ticks[index][tick - 1u].position;

            if (sample.stunned)
            {
                if (attacker == 0u)
                    continue;
                Stun stun{ attacker, id, tick, windowEnd };
                for (uint32_t later = tick; later <= windowEnd; ++later)
                {
                    stun.unsupportedTicks += ticks[index][later].supported ? 0u : 1u;
                    const glm::dvec3 moved = ticks[index][later].position - launchPoint;
                    stun.horizontalTravel = std::max(stun.horizontalTravel, std::hypot(moved.x, moved.z));
                }
                measured.stuns.push_back(stun);
                continue;
            }

            Knockback knockback;
            knockback.attacker = attacker;
            knockback.target = id;
            knockback.hitTick = tick;
            knockback.direction = sample.hitDirection;
            knockback.directionWorld = sample.hitDirectionWorld;
            knockback.knockbackSpeed = sample.knockbackSpeed;
            knockback.windowEndTick = windowEnd;
            const glm::dvec3 targetGround = grounds[index][tick].movementGround;
            const double length = glm::length(sample.hitDirectionWorld);
            knockback.dotTargetGround = length > 0.0 ? glm::dot(sample.hitDirectionWorld, targetGround) / length : 0.0;
            const double launchSine = double(brawlerMovementSimulation::kMinLaunchSine);
            knockback.crossing = knockback.dotTargetGround > launchSine ? GroundCrossing::Leaves
                : knockback.dotTargetGround < -launchSine                 ? GroundCrossing::Enters
                                                                           : GroundCrossing::InPlane;
            knockback.restClearance = clearanceOf(launchPoint, ground);
            knockback.minClearance = knockback.restClearance;
            std::optional<uint32_t> firstUnsupported;
            std::optional<uint32_t> landed;
            for (uint32_t later = tick; later <= windowEnd; ++later)
            {
                const CharacterTick& during = ticks[index][later];
                knockback.unsupportedTicks += during.supported ? 0u : 1u;
                knockback.apex = std::max(knockback.apex, during.position.y - launchPoint.y);
                const double clearance = clearanceOf(during.position, ground);
                knockback.minClearance = std::min(knockback.minClearance, clearance);
                knockback.peakClearanceRise = std::max(knockback.peakClearanceRise, clearance - knockback.restClearance);
                if (!during.supported && !firstUnsupported.has_value())
                    firstUnsupported = later;
                if (during.supported && firstUnsupported.has_value() && !landed.has_value())
                    landed = later;
                if (!during.hitFlinch && knockback.lockoutEndTick == 0u)
                    knockback.lockoutEndTick = later;
            }
            knockback.firstUnsupportedTick = firstUnsupported.value_or(0u);
            if (firstUnsupported.has_value())
                knockback.speedIntoFirstUnsupported = ticks[index][*firstUnsupported - 1u].speed;
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
    std::sort(measured.swings.begin(), measured.swings.end(),
        [](const SwingRecord& first, const SwingRecord& second) { return first.startTick < second.startTick; });
    return measured;
}

const SlopeMeasurements& theSlope()
{
    static const SlopeMeasurements measured = measureTheSlope();
    return measured;
}

const char* prefixOf(unsigned int id)
{
    for (const goldenTraceScene::Character& character : SlopeScenario::characters())
        if (character.id == id)
            return character.prefix;
    return "?";
}

struct SlotWindow
{
    uint32_t first;
    uint32_t last;
    bool contains(uint32_t tick) const { return tick >= first && tick <= last; }
};

SlotWindow slotWindow(std::size_t slot)
{
    return SlotWindow{ goldenTraceSlopeScenario::slotStart(slot), goldenTraceSlopeScenario::slotStart(slot + 1u) - 1u };
}

const Knockback* knockbackIn(const SlopeMeasurements& measured, unsigned int attacker, unsigned int target, const SlotWindow& window)
{
    for (const Knockback& knockback : measured.knockbacks)
        if (knockback.attacker == attacker && knockback.target == target && window.contains(knockback.hitTick))
            return &knockback;
    return nullptr;
}

uint32_t guardBlocksIn(const SlopeMeasurements& measured, unsigned int attacker, const SlotWindow& window)
{
    uint32_t count = 0u;
    for (const Event& block : measured.guardBlocks)
        count += block.attacker == attacker && window.contains(block.tick) ? 1u : 0u;
    return count;
}

double largestSwingUpError(const SlopeMeasurements& measured)
{
    double largest = 0.0;
    for (const SwingRecord& swing : measured.swings)
        largest = std::max(largest, swing.upError);
    return largest;
}

double largestOutOfPlane(const SlopeMeasurements& measured)
{
    double largest = 0.0;
    for (const SwingRecord& swing : measured.swings)
        largest = std::max(largest, swing.outOfPlane);
    return largest;
}

void printSlopeMeasurements(const SlopeMeasurements& measured)
{
    std::string line = "[GoldenTrace.slope]";
    char buffer[768];
    uint32_t tiltedSwings = 0u;
    for (const SwingRecord& swing : measured.swings)
        tiltedSwings += swing.tilted ? 1u : 0u;
    std::snprintf(buffer, sizeof(buffer), " idleDriftAM=%.9g slopeTicks=%u normalDiffersTicks=%u maxNormalDifference=%.3g"
                                          " unsettledPhaseStarts=%u creaseHits=%u/%u knockbacks=%zu swings=%zu tiltedSwings=%u"
                                          " maxSwingUpError=%.3g weaponOutOfPlaneMax=%.3g extraCasts=%u guardBlocks=%zu",
        measured.idleDriftA, measured.slopeTicks, measured.normalDiffersTicks, measured.maxNormalDifference,
        measured.unsettledPhaseStarts, measured.creaseHits, measured.creaseSwings, measured.knockbacks.size(),
        measured.swings.size(), tiltedSwings, largestSwingUpError(measured), largestOutOfPlane(measured), measured.extraCasts,
        measured.guardBlocks.size());
    line += buffer;
    line += " inbound=";
    for (const goldenTraceScene::Character& character : SlopeScenario::characters())
    {
        std::snprintf(buffer, sizeof(buffer), "%s%u/%u ", character.prefix, measured.inboundHits.at(character.id),
            measured.inboundBlocks.at(character.id));
        line += buffer;
    }
    for (const Shot& shot : measured.shots)
    {
        std::snprintf(buffer, sizeof(buffer), "| %sshot.spawnTick=%u aliveTicks=%u overFaceTicks=%u clearanceM=%.6f"
                                              " clearanceDeviationM=%.3g hits=%u blocks=%u target=%s ",
            prefixOf(shot.shooter), shot.spawnTick, shot.aliveTicks, shot.overFaceTicks, shot.firstClearance, shot.clearanceDeviation,
            shot.hits, shot.blocks, shot.lastTarget == 0u ? "none" : prefixOf(shot.lastTarget));
        line += buffer;
    }
    for (const Stun& stun : measured.stuns)
    {
        std::snprintf(buffer, sizeof(buffer), "| %s>%sstun.tick=%u unsupportedTicks=%u horizontalTravelM=%.6f windowEndTick=%u ",
            prefixOf(stun.attacker), prefixOf(stun.target), stun.hitTick, stun.unsupportedTicks, stun.horizontalTravel,
            stun.windowEndTick);
        line += buffer;
    }
    for (const Knockback& knockback : measured.knockbacks)
    {
        std::snprintf(buffer, sizeof(buffer),
            "| %s>%stick=%u dir=(%.4f,%.4f) dirWorld=(%.4f,%.4f,%.4f) %s=%.4f travelM=%.4f unsupportedTicks=%u"
            " firstUnsupportedTick=%u speedIntoItMS=%.4f apexM=%.4f peakClearanceRiseM=%.4f airtimeS=%.4f lockoutEndTick=%u"
            " minClearanceM=%.4f restClearanceM=%.4f windowEndTick=%u ",
            prefixOf(knockback.attacker), prefixOf(knockback.target), knockback.hitTick, knockback.direction.x, knockback.direction.y,
            knockback.directionWorld.x, knockback.directionWorld.y, knockback.directionWorld.z, nameOf(knockback.crossing),
            knockback.dotTargetGround, knockback.travel, knockback.unsupportedTicks, knockback.firstUnsupportedTick,
            knockback.speedIntoFirstUnsupported, knockback.apex,
            knockback.peakClearanceRise, knockback.airtime, knockback.lockoutEndTick, knockback.minClearance, knockback.restClearance,
            knockback.windowEndTick);
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
    const SlopeMeasurements& measured = theSlope();
    printSlopeMeasurements(measured);
    REQUIRE(measured.slopeTicks > 0u);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.SwingsFollowTheGround", "[GoldenTrace]")
{
    const SlopeMeasurements& measured = theSlope();
    uint32_t tiltedSwingsOfA = 0u;
    for (const SwingRecord& swing : measured.swings)
    {
        INFO(prefixOf(swing.id) << " swing seq " << swing.sequence << " from tick " << swing.startTick << " (" << swing.ticks
             << " ticks): |R_ic kUp - n| = " << swing.upError << ", blade out of plane " << swing.outOfPlane);
        CHECK(swing.upError <= 1e-6);
        CHECK(swing.outOfPlane <= 1e-5);
        tiltedSwingsOfA += swing.id == goldenTraceSlopeScenario::kCharacterA && swing.tilted ? 1u : 0u;
    }
    CHECK(tiltedSwingsOfA >= 7u);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.SideSwingsHitUpAndDownTheSlope", "[GoldenTrace]")
{
    const SlopeMeasurements& measured = theSlope();
    CHECK(knockbackIn(measured, goldenTraceSlopeScenario::kCharacterA, goldenTraceSlopeScenario::kCharacterB, slotWindow(1u)) != nullptr);
    CHECK(knockbackIn(measured, goldenTraceSlopeScenario::kCharacterA, goldenTraceSlopeScenario::kCharacterC, slotWindow(2u)) != nullptr);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.GuardsBlockInTheGroundPlane", "[GoldenTrace]")
{
    const SlopeMeasurements& measured = theSlope();
    for (const std::size_t slot : { 5u, 6u, 7u })
    {
        const unsigned int guard = slot == 5u ? goldenTraceSlopeScenario::kCharacterB : goldenTraceSlopeScenario::kCharacterC;
        INFO("slot " << slot << ": A's swing into " << prefixOf(guard) << "'s guard");
        CHECK(guardBlocksIn(measured, goldenTraceSlopeScenario::kCharacterA, slotWindow(slot)) >= 1u);
        CHECK(knockbackIn(measured, goldenTraceSlopeScenario::kCharacterA, guard, slotWindow(slot)) == nullptr);
    }
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.ProjectilesFollowTheSlope", "[GoldenTrace]")
{
    const SlopeMeasurements& measured = theSlope();
    REQUIRE(measured.shots.size() == 2u);
    bool blockedByC = false;
    for (const Shot& shot : measured.shots)
    {
        INFO(prefixOf(shot.shooter) << " shot from tick " << shot.spawnTick << ": " << shot.overFaceTicks
             << " ticks over the face, clearance " << shot.firstClearance << " m, deviation " << shot.clearanceDeviation << " m");
        CHECK(shot.overFaceTicks >= 3u);
        CHECK(shot.clearanceDeviation <= 1e-4);
        blockedByC = blockedByC
            || (shot.shooter == goldenTraceSlopeScenario::kCharacterA && shot.blocks >= 1u
                && shot.lastTarget == goldenTraceSlopeScenario::kCharacterC);
    }
    CHECK(blockedByC);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.KnockbacksFollowTheAttackersSwing", "[GoldenTrace]")
{
    const SlopeMeasurements& measured = theSlope();
    for (const unsigned int target : { goldenTraceSlopeScenario::kCharacterD, goldenTraceSlopeScenario::kCharacterE })
    {
        const SlotWindow window = slotWindow(target == goldenTraceSlopeScenario::kCharacterD ? 3u : 4u);
        const Knockback* knockback = knockbackIn(measured, goldenTraceSlopeScenario::kCharacterA, target, window);
        REQUIRE(knockback != nullptr);
        INFO(prefixOf(target) << " same-face fall-line knockback: " << knockback->unsupportedTicks << " Unsupported ticks, the first at "
             << knockback->firstUnsupportedTick << " entered at " << knockback->speedIntoFirstUnsupported << " m/s");
        CHECK(knockback->crossing == GroundCrossing::InPlane);
        CHECK((knockback->unsupportedTicks == 0u
               || (knockback->firstUnsupportedTick > knockback->hitTick
                   && knockback->speedIntoFirstUnsupported <= 0.15 * knockback->knockbackSpeed)));
    }

    std::array<uint32_t, 2> longLaunches{};
    std::array<uint32_t, 2> entries{};
    std::array<uint32_t, 2> inPlane{};
    std::array<uint32_t, 2> crossings{};
    uint32_t convexLongDetaches = 0u;
    for (const Knockback& knockback : measured.knockbacks)
    {
        if (!isACreaseCharacter(knockback.attacker) || !isACreaseCharacter(knockback.target))
            continue;
        const std::size_t crease = knockback.hitTick < goldenTraceSlopeScenario::slotStart(5u) ? 0u : 1u;
        INFO(prefixOf(knockback.attacker) << ">" << prefixOf(knockback.target) << " crease hit at " << knockback.hitTick << " ("
             << (crease == 0u ? "concave" : "convex") << "): " << nameOf(knockback.crossing) << " (" << knockback.dotTargetGround
             << "), " << knockback.unsupportedTicks << " Unsupported ticks from " << knockback.firstUnsupportedTick);
        if (knockback.crossing == GroundCrossing::Leaves)
        {
            CHECK(knockback.firstUnsupportedTick == knockback.hitTick);
            longLaunches[crease] += knockback.unsupportedTicks >= 10u ? 1u : 0u;
            ++crossings[crease];
        }
        else if (knockback.crossing == GroundCrossing::Enters)
        {
            CHECK(knockback.firstUnsupportedTick != knockback.hitTick);
            ++entries[crease];
            ++crossings[crease];
        }
        else
        {
            ++inPlane[crease];
            if (crease == 1u)
            {
                CHECK(knockback.firstUnsupportedTick == knockback.hitTick);
                convexLongDetaches += knockback.unsupportedTicks >= 10u ? 1u : 0u;
            }
        }
    }
    CHECK(longLaunches[0] >= 1u);
    CHECK(entries[0] >= 1u);
    CHECK(crossings[1] == 0u);
    CHECK(inPlane[1] == 4u);
    CHECK(convexLongDetaches == 4u);

    REQUIRE(measured.stuns.size() == 1u);
    const Stun& stun = measured.stuns.front();
    INFO(prefixOf(stun.attacker) << "'s overhead on " << prefixOf(stun.target) << " at " << stun.hitTick << ": "
         << stun.unsupportedTicks << " Unsupported ticks, " << stun.horizontalTravel << " m horizontal travel");
    CHECK(stun.attacker == goldenTraceSlopeScenario::kCharacterC);
    CHECK(stun.target == goldenTraceSlopeScenario::kCharacterD);
    CHECK(stun.unsupportedTicks == 0u);
    CHECK(stun.horizontalTravel < 0.01);
}

TEST_CASE("BrawlerJoltSlopeGoldenTrace.Record", "[.GoldenRecord]")
{
    goldenTraceScene::recordGoldenAndEnvelope<SlopeScenario>("BrawlerJoltSlopeGolden.inl", "BrawlerJoltSlopeGoldenEnvelope.inl");
}

#endif
