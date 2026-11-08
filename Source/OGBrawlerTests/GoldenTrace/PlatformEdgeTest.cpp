// SPDX-License-Identifier: BUSL-1.1
// docs/PlatformEdgeTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceScenario.h"
#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"
#include "QuarterCylinderPlatform.inl"

#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/trigonometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace platformEdgeTests
{

using goldenTraceSlopeScenario::Swing;

inline constexpr unsigned int kTarget = 1u;
inline constexpr unsigned int kAttacker = 2u;
inline constexpr uint32_t kSwingTick = 61u;
inline constexpr uint32_t kRunTicks = 200u;
inline constexpr uint32_t kAfterDepartureTicks = 5u;
inline constexpr uint32_t kStayPrintTicks = 45u;
inline constexpr float kAttackerRadius = 1.6f;
inline constexpr float kAttackerCloseRadius = 1.2f;
inline constexpr float kAttackerMargin = 0.25f;
inline constexpr std::array<float, 3u> kAttackerInwardShifts = { 0.f, 0.45f, 0.9f };
inline constexpr float kEventThreshold = 0.25f;
inline constexpr float kMirrorTolerance = 1e-4f;
inline constexpr float kResidualTolerance = 1e-3f;
inline constexpr float kFaceMatchCos = 0.9999f;
inline constexpr float kDirectionCos = 0.82f;
inline constexpr float kGoodDirectionCos = 0.99f;
inline constexpr float kTopS = 10.02f;
inline constexpr float kLowS = -0.02f;
inline constexpr float kTipPlusW = 10.025f;
inline constexpr float kTipMinusW = -10.02f;
inline constexpr float kChamferSum = 14.175f;
inline constexpr float kSeamOverlap = 0.0225f;
inline constexpr float kHalfSqrt2 = 0.70710678f;
inline constexpr glm::vec3 kFloorCentre{ 19.115f, -0.25f, -16.f };
inline constexpr glm::vec3 kFloorHalfExtents{ 30.f, 0.25f, 30.f };
inline constexpr std::array<float, 6u> kDistances = { -0.15f, 0.3f, 1.5f, 3.5f, 4.5f, 4.9f };
inline constexpr float kReachableFromDistance = 1.f;
inline constexpr float kLipHitDistance = kDistances.front();
inline constexpr float kFloorContactHeight = 0.05f;
inline constexpr float kRimOnBoundaryM = 1e-3f;
inline constexpr std::array<float, 3u> kObliqueDeg = { 0.f, 30.f, -30.f };

struct PlatformFrame
{
    glm::vec3 origin{ 0.f };
    glm::vec3 along{ 0.f };
    glm::vec3 lateral{ 0.f };
    glm::vec3 normal{ 0.f };
    float top = 0.f;

    glm::vec3 surface(const glm::vec2& sw) const { return origin + along * sw.x + lateral * sw.y + normal * top; }
    glm::vec2 chart(const glm::vec3& p) const
    {
        const glm::vec3 below = p - glm::vec3(0.f, glm::dot(p - surface(glm::vec2(0.f)), normal) / normal.y, 0.f);
        return glm::vec2(glm::dot(below - origin, along), glm::dot(below - origin, lateral));
    }
    glm::vec3 inPlane(const glm::vec2& d) const { return along * d.x + lateral * d.y; }
};

glm::quat rotationOf(const quarterCylinderPlatform::ImportedHull& hull)
{
    return glm::quat(hull.rotationWxyz[0], hull.rotationWxyz[1], hull.rotationWxyz[2], hull.rotationWxyz[3]);
}

PlatformFrame platformFrame()
{
    const quarterCylinderPlatform::ImportedHull& hull = quarterCylinderPlatform::kHulls[0];
    const glm::quat rotation = rotationOf(hull);
    PlatformFrame frame;
    frame.origin = glm::vec3(hull.translation[0], hull.translation[1], hull.translation[2]);
    frame.along = rotation * glm::vec3(0.f, 0.f, -1.f);
    frame.lateral = rotation * glm::vec3(1.f, 0.f, 0.f);
    frame.normal = rotation * glm::vec3(0.f, 1.f, 0.f);
    for (const glm::vec3& point : hull.points)
        frame.top = std::max(frame.top, point.y);
    return frame;
}

StaticWorldDescription platformStatics()
{
    StaticWorldDescription description;
    uint64_t stableKey = 1u;
    for (const quarterCylinderPlatform::ImportedHull& hull : quarterCylinderPlatform::kHulls)
    {
        StaticShapeDescriptor descriptor = goldenTraceScenario::staticBox(glm::vec3(0.f), glm::quat(1.f, 0.f, 0.f, 0.f),
            glm::vec3(1.f), stableKey++);
        descriptor.shape = StaticConvexHull{ std::vector<glm::vec3>(hull.points.begin(), hull.points.end()) };
        descriptor.localToWorld = glm::translate(glm::mat4(1.f), glm::vec3(hull.translation[0], hull.translation[1], hull.translation[2]))
            * glm::mat4_cast(rotationOf(hull));
        description.shapes.push_back(std::move(descriptor));
    }
    description.shapes.push_back(goldenTraceScenario::staticBox(kFloorCentre, glm::quat(1.f, 0.f, 0.f, 0.f), kFloorHalfExtents, stableKey));
    return description;
}

struct RimSite
{
    const char* name;
    glm::vec2 rim;
    glm::vec2 outward;
};

inline constexpr std::array<RimSite, 15u> kSites = { {
    { "arcTop", { kTopS, 1.5f }, { 1.f, 0.f } },
    { "arcTopSeam", { kTopS, 0.f }, { 1.f, 0.f } },
    { "chamferPlus", { 0.5f * kChamferSum, 0.5f * kChamferSum }, { kHalfSqrt2, kHalfSqrt2 } },
    { "chamferMinus", { 0.5f * kChamferSum, -0.5f * kChamferSum }, { kHalfSqrt2, -kHalfSqrt2 } },
    { "tipPlusLow", { 0.5f, kTipPlusW }, { 0.f, 1.f } },
    { "tipPlusMid", { 2.5f, kTipPlusW }, { 0.f, 1.f } },
    { "tipMinusLow", { 0.5f, kTipMinusW }, { 0.f, -1.f } },
    { "tipMinusMid", { 2.5f, kTipMinusW }, { 0.f, -1.f } },
    { "tipCornerPlus", { kLowS, kTipPlusW }, { -kHalfSqrt2, kHalfSqrt2 } },
    { "tipCornerMinus", { kLowS, kTipMinusW }, { -kHalfSqrt2, -kHalfSqrt2 } },
    { "lowEdgePlus", { kLowS, 3.f }, { -1.f, 0.f } },
    { "lowEdgeMinus", { kLowS, -3.f }, { -1.f, 0.f } },
    { "lowEdgeSeam", { kLowS, 0.f }, { -1.f, 0.f } },
    { "seamCrossPlus", { 5.f, 0.f }, { 0.f, 1.f } },
    { "seamCrossMinus", { 5.f, 0.f }, { 0.f, -1.f } },
} };

float insideMargin(const glm::vec2& sw)
{
    return std::min({ sw.x - kLowS, kTopS - sw.x, kTipPlusW - sw.y, sw.y - kTipMinusW,
        (kChamferSum - sw.x - std::abs(sw.y)) * kHalfSqrt2 });
}

glm::vec2 rotated(const glm::vec2& d, float degrees)
{
    const float c = std::cos(glm::radians(degrees));
    const float s = std::sin(glm::radians(degrees));
    return glm::vec2(c * d.x - s * d.y, s * d.x + c * d.y);
}

glm::vec3 horizontalUnit(const glm::vec3& v)
{
    return glm::normalize(glm::vec3(v.x, 0.f, v.z));
}

struct Placement
{
    glm::vec3 targetSpawn{ 0.f };
    glm::vec3 attackerSpawn{ 0.f };
    glm::vec3 attackerAim{ 0.f };
    glm::vec3 targetAim{ 0.f };
    Swing side = Swing::Right;
};

simulatableBrawler::PlayerInput inputOf(const goldenTraceScenario::InputRow& row)
{
    simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
    input.aimDirection = row.aim;
    input.attackLeft = row.attackLeft;
    input.attackRight = row.attackRight;
    input.moveStick = glm::vec2(row.stickX, row.stickY);
    input.moveDirectionWorld = row.move;
    input.triggeredActionId = row.actionId;
    return input;
}

struct EdgeScenario
{
    static constexpr const char* kGenerator = "PlatformEdge.run";
    static constexpr float kDtSeconds = goldenTraceSlopeScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = kRunTicks;

    static inline Placement placement{};
    static inline std::array<goldenTraceScene::Character, 2u> characterTable{};
    static inline std::array<goldenTraceScene::SpawnPoint, 2u> spawnPoints{};

    static void set(const Placement& p)
    {
        placement = p;
        characterTable = { goldenTraceScene::Character{ kTarget, 0u, "T.", p.targetSpawn },
            goldenTraceScene::Character{ kAttacker, 1u, "A.", p.attackerSpawn } };
        spawnPoints = { goldenTraceScene::SpawnPoint{ 0u, p.targetSpawn }, goldenTraceScene::SpawnPoint{ 1u, p.attackerSpawn } };
    }

    static bool isSampleTick(uint32_t) { return false; }
    static StaticWorldDescription statics() { return platformStatics(); }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return goldenTraceScenario::scenarioStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int id, uint32_t tick)
    {
        if (id == kAttacker && tick == kSwingTick)
            return inputOf(goldenTraceSlopeScenario::swing(kAttacker, tick, placement.attackerAim, placement.side));
        return inputOf(goldenTraceSlopeScenario::still(id, tick, tick, id == kAttacker ? placement.attackerAim : placement.targetAim));
    }
    static std::span<const goldenTraceScene::Character> characters() { return characterTable; }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable() { return spawnPoints; }
    static std::span<const goldenTraceScene::Teleport> teleports() { return {}; }
};

static_assert(goldenTraceScene::Scenario<EdgeScenario>, "the platform-edge scenario provides everything goldenTraceScene::Scene reads");

struct TickRow
{
    uint32_t tick = 0u;
    glm::vec3 position{ 0.f };
    float pastRim = 0.f;
    uint8_t support = 0u;
    float clearance = 0.f;
    bool blocked = false;
    bool walkable = false;
    bool faceWalkable = false;
    brawlerMovementSimulation::SupportKind kind = brawlerMovementSimulation::SupportKind::None;
    int hull = -1;
    float impactHeight = 0.f;
    glm::vec3 gjk{ 0.f };
    glm::vec3 exact{ 0.f };
    glm::vec3 attackGround{ 0.f };
    glm::vec3 n{ 0.f };
    glm::vec3 vRaw{ 0.f };
    glm::vec3 vWritten{ 0.f };
    glm::vec3 vActual{ 0.f };
    glm::vec3 vMirror{ 0.f };
    bool committed = false;
    bool hitTick = false;
    bool written = false;
    bool detached = false;
    bool mirrored = false;
    float servoDv = 0.f;
    float stiffDv = 0.f;
    float dampDv = 0.f;
    float channelUp = 0.f;
    float uvDy = 0.f;
    float pushDy = 0.f;
    float pushDv = 0.f;
    float writeDy = 0.f;
    float redirectDy = 0.f;
    float residualDy = 0.f;
    float residualDv = 0.f;
    float vDotUp = 0.f;
};

struct Mirror
{
    glm::vec3 vPush{ 0.f };
    glm::vec3 vWritten{ 0.f };
    float redirectDy = 0.f;
    bool written = false;
    bool detached = false;
    bool supported = false;
    float servoDv = 0.f;
    float stiffDv = 0.f;
    float dampDv = 0.f;
    float channelUp = 0.f;
    float uvDy = 0.f;
    glm::vec3 residual{ 0.f };
    float vDotUp = 0.f;
    glm::vec3 vOut{ 0.f };
};

Mirror mirrorIntegrate(const brawlerMovementSimulation::State& pre, const brawlerMovementSimulation::StaticData& sd,
    const brawlerMovementSimulation::GroundSupport& ground, bool supported, bool committed,
    const brawlerInboundHit::DerivedState& inbound, uint32_t tick, float dt)
{
    using namespace brawlerMovementSimulation;
    Mirror m;
    brawlerMovementSimulation::State state = pre;
    if ((state.flags & kFlagHasCommand) != 0u)
    {
        const glm::vec3 predicted = state.positionCmd + state.velocity * dt;
        const glm::vec3 pushOut = state.bodyState.position - predicted;
        if (glm::dot(pushOut, pushOut) > kPushOutEps * kPushOutEps)
        {
            const glm::vec3 contactNormal = glm::normalize(pushOut);
            if (glm::dot(contactNormal, kWorldUp) >= sd.cosMaxSlope)
                state.velocity -= glm::dot(state.velocity, kWorldUp) * kWorldUp;
            else
                state.velocity -= glm::dot(state.velocity, contactNormal) * contactNormal;
        }
    }
    m.vPush = state.velocity;
    m.written = writesKnockbackVelocity(inbound, committed);
    if (m.written)
        state.velocity = inbound.hitDirectionWorld * inbound.knockbackSpeed;
    m.vWritten = state.velocity;
    const SupportDecision decision = classifySupport(state, sd, ground, committed, m.written, inbound.hitDirectionWorld);
    m.detached = decision.detached;
    m.supported = decision.support != SupportState::Unsupported;
    if (redirectsIntoFace(state, sd, ground, decision.support, committed, m.written))
    {
        const float before = state.velocity.y;
        state.velocity -= ground.faceNormal * glm::dot(state.velocity, ground.faceNormal);
        m.redirectDy = state.velocity.y - before;
    }

    glm::vec3 u(simFrame::kForward);
    glm::vec3 v(simFrame::kRight);
    buildTangentFrame(decision.frameNormal, u, v);
    const VelocityChannels channels = decomposeVelocity(state.velocity, u, v, kWorldUp);
    m.residual = (u * channels.uv.x + v * channels.uv.y + kWorldUp * channels.up) - state.velocity;
    m.vDotUp = glm::dot(v, kWorldUp);
    glm::vec2 velocityUV(0.f);
    if (committed)
    {
        if (inbound.wasHitThisTick)
            velocityUV = channels.uv;
        else
            velocityUV = moveTowards(channels.uv, glm::vec2(0.f), sd.launchDecel * dt);
    }
    else
    {
        velocityUV = computeDesiredVelocityUV_ContinuousAccelBrake(sd, state, tick, glm::vec2(0.f), channels.uv, dt);
    }
    m.channelUp = channels.up;
    float accelUp = sd.gravity;
    if (supported)
    {
        const float e = sd.rideHeight - ground.probe.clearance;
        const float servo = (e >= 0.f) ? sd.hoverStiffness * e - sd.hoverDamping * channels.up - sd.gravity
                                       : glm::max(sd.hoverStiffness * e, -sd.hoverPullDownAccel);
        const float clamped = glm::clamp(servo, -sd.hoverMaxAccel, sd.hoverMaxAccel);
        accelUp += clamped;
        m.servoDv = clamped * dt;
        m.stiffDv = (e >= 0.f ? sd.hoverStiffness * e : glm::max(sd.hoverStiffness * e, -sd.hoverPullDownAccel)) * dt;
        m.dampDv = (e >= 0.f ? -sd.hoverDamping * channels.up : 0.f) * dt;
    }
    const float velocityUp = glm::max(channels.up + accelUp * dt, -sd.terminalFallSpeed);
    m.vOut = u * velocityUV.x + v * velocityUV.y + kWorldUp * velocityUp;
    m.uvDy = (u.y * velocityUV.x + v.y * velocityUV.y) - (u.y * channels.uv.x + v.y * channels.uv.y);
    return m;
}

struct RunResult
{
    bool rested = false;
    bool hit = false;
    uint32_t hitTick = 0u;
    glm::vec3 hitDirection{ 0.f };
    std::vector<TickRow> rows;
    uint32_t mirrorMismatches = 0u;
    uint32_t supportMismatches = 0u;
    uint32_t pushOutMismatches = 0u;
    TickRow rest;
    glm::vec3 attackerAtSwing{ 0.f };
    uint32_t attackerHits = 0u;
};

int hullSideOf(float lateral)
{
    if (lateral > kSeamOverlap)
        return 0;
    if (lateral < -kSeamOverlap)
        return 1;
    return 2;
}

RunResult runOnce(const Placement& placement, const PlatformFrame& frame, const RimSite& site)
{
    EdgeScenario::set(placement);
    goldenTraceScene::Scene<EdgeScenario> scene(std::nullopt, 1.f);
    const brawlerMovementSimulation::StaticData& sd = scene.staticData.m_movementStaticData;
    const float dt = EdgeScenario::kDtSeconds;
    RunResult result;
    for (uint32_t tick = 1u; tick <= kRunTicks; ++tick)
    {
        SimulatableBrawler& target = scene.storage.template get<SimulatableBrawler>(kTarget);
        const brawlerMovementSimulation::State pre = target.getAllState().getState().template get<brawlerMovementSimulation::State>();
        const bool teleporting = target.getAllState().getState().template get<brawlerMovementSimulation::InitialConditions>().teleportPending != 0u;
        const auto& bindings = target.getPhysicsComposite().template get<brawlerMovementSimulation::PhysicsDeclaration>().bindings;
        const brawlerMovementSimulation::GroundSupport ground = brawlerMovementSimulation::estimateSupport(pre, sd, bindings, scene.query);
        const brawlerMovementSimulation::GroundProbe& probe = ground.probe;

        scene.step(tick);

        const SimulatableBrawler& after = scene.storage.template get<SimulatableBrawler>(kTarget);
        const auto& post = after.getAllState().getState().template get<brawlerMovementSimulation::State>();
        const auto& derived = after.getAllState().getDerivedState().template get<brawlerMovementSimulation::DerivedState>();
        const auto& inbound = after.getAllState().getDerivedState().template get<brawlerInboundHit::DerivedState>();
        const auto& machine = after.getAllState().getState().template get<dAttackMachineSimulation::State>();

        TickRow row;
        row.tick = tick;
        row.position = pre.bodyState.position;
        row.pastRim = glm::dot(frame.chart(pre.bodyState.position) - site.rim, site.outward);
        row.support = static_cast<uint8_t>((post.flags & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift);
        row.clearance = probe.clearance;
        row.blocked = probe.hit.blocked;
        row.walkable = probe.walkable;
        row.kind = ground.kind;
        row.faceWalkable = probe.hit.blocked && glm::dot(probe.hit.surfaceNormal, brawlerMovementSimulation::kWorldUp) >= sd.cosMaxSlope;
        row.hull = probe.hit.blocked ? hullSideOf(frame.chart(probe.hit.impactPoint).y) : -1;
        row.impactHeight = probe.hit.impactPoint.y;
        row.gjk = probe.hit.normal;
        row.exact = probe.hit.surfaceNormal;
        row.attackGround = ground.faceNormal;
        row.n = derived.surfaceNormal;
        row.vRaw = pre.velocity;
        row.vActual = post.velocity;
        row.committed = brawlerMovementSimulation::machineLaunchesMovement(machine);
        row.hitTick = inbound.wasHitThisTick && inbound.reactionKind == HitReactionKind::Knockback;
        if (row.hitTick && !result.hit)
        {
            result.hit = true;
            result.hitTick = tick;
            result.hitDirection = inbound.hitDirectionWorld;
        }
        if (!teleporting)
        {
            const bool supported = row.support != 0u;
            const Mirror m = mirrorIntegrate(pre, sd, ground, supported, row.committed, inbound, tick, dt);
            row.mirrored = true;
            row.vMirror = m.vOut;
            row.vWritten = m.vWritten;
            row.written = m.written;
            row.detached = m.detached;
            row.servoDv = m.servoDv;
            row.stiffDv = m.stiffDv;
            row.dampDv = m.dampDv;
            row.channelUp = m.channelUp;
            row.uvDy = m.uvDy;
            row.pushDy = m.vPush.y - pre.velocity.y;
            row.pushDv = glm::length(m.vPush - pre.velocity);
            row.residualDy = m.residual.y;
            row.residualDv = glm::length(m.residual);
            row.vDotUp = m.vDotUp;
            row.writeDy = m.vWritten.y - m.vPush.y;
            row.redirectDy = m.redirectDy;
            if (glm::length(m.vOut - post.velocity) > kMirrorTolerance * std::max(1.f, glm::length(post.velocity)))
                ++result.mirrorMismatches;
            if (supported != m.supported)
                ++result.supportMismatches;
            if (glm::length((pre.bodyState.position - (pre.positionCmd + pre.velocity * dt)) - derived.lastPushOut) > 1e-5f
                && (pre.flags & brawlerMovementSimulation::kFlagHasCommand) != 0u)
                ++result.pushOutMismatches;
        }
        if (tick == kSwingTick)
        {
            result.attackerAtSwing = scene.storage.template get<SimulatableBrawler>(kAttacker).getAllState().getState()
                                         .template get<brawlerMovementSimulation::State>().bodyState.position;
            result.rest = row;
            result.rested = row.support == uint8_t(brawlerMovementSimulation::SupportState::Supported)
                && glm::length(pre.velocity) < 0.05f;
        }
        result.rows.push_back(row);
    }
    result.attackerHits = scene.detections.at(kAttacker).meleeHits;
    return result;
}

std::optional<Placement> placementFor(const PlatformFrame& frame, const RimSite& site, float distance, const glm::vec2& wanted, Swing side,
    bool flipAttacker, float shift)
{
    const glm::vec2 targetSw = site.rim - site.outward * distance;
    const glm::vec2 perpendicular(-wanted.y, wanted.x);
    const float margin = std::min(kAttackerMargin, distance - 0.01f);
    std::optional<glm::vec2> best;
    for (const float radius : { kAttackerRadius, kAttackerCloseRadius })
    {
        std::vector<glm::vec2> inside;
        for (const float sign : { 1.f, -1.f })
        {
            const glm::vec2 candidate = targetSw + perpendicular * (radius * sign) - site.outward * shift;
            if (insideMargin(candidate) >= margin)
                inside.push_back(candidate);
        }
        if (inside.empty())
            continue;
        std::sort(inside.begin(), inside.end(), [&](const glm::vec2& a, const glm::vec2& b)
            { return glm::dot(a - targetSw, site.outward) < glm::dot(b - targetSw, site.outward); });
        if (flipAttacker && inside.size() < 2u)
            return std::nullopt;
        best = inside[flipAttacker ? 1u : 0u];
        break;
    }
    if (!best.has_value())
        return std::nullopt;
    Placement placement;
    const glm::vec3 targetFace = frame.surface(targetSw);
    const glm::vec3 attackerFace = frame.surface(*best);
    placement.targetSpawn = goldenTraceSlopeScenario::raised(targetFace, goldenTraceSlopeScenario::kTeleportHeight);
    placement.attackerSpawn = goldenTraceSlopeScenario::raised(attackerFace, goldenTraceSlopeScenario::kTeleportHeight);
    placement.attackerAim = horizontalUnit(targetFace - attackerFace);
    placement.targetAim = horizontalUnit(attackerFace - targetFace);
    placement.side = side;
    return placement;
}


struct Event
{
    const char* site = "";
    float distance = 0.f;
    float obliqueDeg = 0.f;
    Swing side = Swing::Right;
    bool rested = false;
    TickRow rest;
    float directionCos = 0.f;
    float actualObliqueDeg = 0.f;
    uint32_t hitTick = 0u;
    bool departs = false;
    bool edgeAtHit = false;
    uint32_t edgeTick = 0u;
    uint32_t departTick = 0u;
    uint32_t reportTick = 0u;
    uint32_t landingTick = 0u;
    float speedAtEdge = 0.f;
    float vyAtEdge = 0.f;
    float anomaly = 0.f;
    float heightDeviation = 0.f;
    float servo = 0.f;
    float stiff = 0.f;
    float damp = 0.f;
    float uvSupportedTilted = 0.f;
    float uvDetachedTilted = 0.f;
    float uvOther = 0.f;
    float push = 0.f;
    float pushSpeedLoss = 0.f;
    float residualDy = 0.f;
    float residualDv = 0.f;
    uint32_t supportFlips = 0u;
    uint32_t detachedWalkableTicks = 0u;
    uint32_t disagreeTicks = 0u;
    uint32_t lipFaceTicks = 0u;
    uint32_t hullSwitches = 0u;
    float minNTiltDeg = 0.f;
    float maxNTiltDeg = 0.f;
    std::string label;
    bool estimateEdgeAtHit = false;
    uint32_t estimateEdgeTick = 0u;
    float estimateAnomaly = 0.f;
    float estimateServo = 0.f;
    float estimateRedirect = 0.f;
    std::string estimateLabel;
};

float tiltDeg(const glm::vec3& normal, const glm::vec3& outwardHorizontal)
{
    return glm::degrees(std::atan2(glm::dot(normal, outwardHorizontal), normal.y));
}

bool exactIsTop(const TickRow& row, const PlatformFrame& frame)
{
    return row.blocked && glm::dot(row.exact, frame.normal) > kFaceMatchCos;
}

bool isPureFace(const TickRow& row, const PlatformFrame& frame)
{
    return exactIsTop(row, frame) && row.support != 0u && glm::dot(row.gjk, frame.normal) > kFaceMatchCos;
}

bool isEstimateFace(const TickRow& row, const PlatformFrame& frame)
{
    return row.kind == brawlerMovementSimulation::SupportKind::Face && row.support != 0u
        && glm::dot(row.attackGround, frame.normal) > kFaceMatchCos;
}

const char* classOf(float anomaly)
{
    return anomaly > kEventThreshold ? "lift" : (anomaly < -kEventThreshold ? "downPull" : "none");
}

bool onFloor(const TickRow& row)
{
    return row.support != 0u && row.blocked && row.impactHeight < kFloorContactHeight;
}

Event analyse(const RunResult& run, const PlatformFrame& frame, const RimSite& site, float distance, float obliqueDeg, Swing side,
    const glm::vec2& wanted)
{
    Event e;
    e.site = site.name;
    e.distance = distance;
    e.obliqueDeg = obliqueDeg;
    e.side = side;
    e.rested = run.rested;
    e.rest = run.rest;
    if (!run.hit)
    {
        e.label = "noHit";
        return e;
    }
    const glm::vec3 wantedWorld = glm::normalize(frame.inPlane(wanted));
    e.directionCos = glm::dot(glm::normalize(run.hitDirection), wantedWorld);
    const glm::vec2 directionChart(glm::dot(run.hitDirection, frame.along), glm::dot(run.hitDirection, frame.lateral));
    e.actualObliqueDeg = glm::degrees(std::atan2(site.outward.x * directionChart.y - site.outward.y * directionChart.x,
        glm::dot(site.outward, directionChart)));
    e.hitTick = run.hitTick;

    std::size_t hitIndex = 0u;
    while (run.rows[hitIndex].tick != run.hitTick)
        ++hitIndex;
    std::size_t floorIndex = hitIndex;
    while (floorIndex < run.rows.size() && !onFloor(run.rows[floorIndex]))
        ++floorIndex;
    e.landingTick = floorIndex < run.rows.size() ? run.rows[floorIndex].tick : 0u;
    std::size_t departIndex = floorIndex;
    while (departIndex > hitIndex && run.rows[departIndex - 1u].support == 0u)
        --departIndex;
    if (departIndex == floorIndex)
    {
        e.label = "staysOnPlatform";
        return e;
    }
    std::size_t edgeIndex = hitIndex;
    for (std::size_t index = hitIndex; index < departIndex; ++index)
        if (isPureFace(run.rows[index], frame))
            edgeIndex = index + 1u;
    e.departs = true;
    e.edgeAtHit = edgeIndex == hitIndex;
    e.edgeTick = run.rows[edgeIndex].tick;
    e.departTick = run.rows[departIndex].tick;
    const std::size_t reportIndex = std::min(departIndex + kAfterDepartureTicks, floorIndex - 1u);
    e.reportTick = run.rows[reportIndex].tick;
    const glm::vec3 reference = e.edgeAtHit ? run.rows[hitIndex].vWritten : run.rows[edgeIndex].vRaw;
    e.speedAtEdge = glm::length(reference);
    e.vyAtEdge = reference.y;

    const float g = goldenTraceScenario::kGravity;
    const float dt = EdgeScenario::kDtSeconds;
    const glm::vec3 outwardHorizontal = horizontalUnit(frame.inPlane(site.outward));
    int previousHull = -2;
    for (std::size_t index = edgeIndex; index <= reportIndex; ++index)
    {
        const TickRow& row = run.rows[index];
        const float anomaly = row.vActual.y - (reference.y + g * dt * float(index - edgeIndex + 1u));
        e.heightDeviation += anomaly * dt;
        e.anomaly = anomaly;
        e.servo += row.servoDv;
        e.stiff += row.stiffDv;
        e.damp += row.dampDv;
        e.push += index == hitIndex ? 0.f : row.pushDy;
        e.pushSpeedLoss += index == hitIndex ? 0.f : row.pushDv;
        e.residualDy += row.residualDy;
        e.residualDv += row.residualDv;
        const bool tilted = glm::dot(row.n, brawlerMovementSimulation::kWorldUp) < kFaceMatchCos
            && glm::dot(row.n, frame.normal) < kFaceMatchCos;
        if (tilted && row.support != 0u)
            e.uvSupportedTilted += row.uvDy;
        else if (tilted)
            e.uvDetachedTilted += row.uvDy;
        else
            e.uvOther += row.uvDy;
        if (index > edgeIndex && row.support != run.rows[index - 1u].support)
            ++e.supportFlips;
        if (row.support == 0u && row.walkable)
            ++e.detachedWalkableTicks;
        if (row.blocked && row.walkable != row.faceWalkable)
            ++e.disagreeTicks;
        if (row.blocked && !exactIsTop(row, frame))
            ++e.lipFaceTicks;
        if (row.blocked && previousHull != -2 && row.hull != previousHull)
            ++e.hullSwitches;
        if (row.blocked)
        {
            previousHull = row.hull;
            e.minNTiltDeg = std::min(e.minNTiltDeg, tiltDeg(row.n, outwardHorizontal));
            e.maxNTiltDeg = std::max(e.maxNTiltDeg, tiltDeg(row.n, outwardHorizontal));
        }
    }
    e.label = classOf(e.anomaly);

    std::size_t estimateEdgeIndex = hitIndex;
    for (std::size_t index = hitIndex; index < departIndex; ++index)
        if (isEstimateFace(run.rows[index], frame))
            estimateEdgeIndex = index + 1u;
    e.estimateEdgeAtHit = estimateEdgeIndex == hitIndex;
    e.estimateEdgeTick = run.rows[estimateEdgeIndex].tick;
    const glm::vec3 estimateReference = e.estimateEdgeAtHit ? run.rows[hitIndex].vWritten : run.rows[estimateEdgeIndex].vRaw;
    for (std::size_t index = estimateEdgeIndex; index <= reportIndex; ++index)
    {
        const TickRow& row = run.rows[index];
        e.estimateAnomaly = row.vActual.y - (estimateReference.y + g * dt * float(index - estimateEdgeIndex + 1u));
        e.estimateServo += row.servoDv;
        e.estimateRedirect += row.redirectDy;
    }
    e.estimateLabel = classOf(e.estimateAnomaly);
    return e;
}

struct SupportCounters
{
    uint32_t flips = 0u;
    bool rimHang = false;
    uint32_t lipTicks = 0u;
};

SupportCounters supportCountersOf(const RunResult& run, const Event& e, const RimSite& site)
{
    SupportCounters counters;
    std::size_t hitIndex = 0u;
    while (run.rows[hitIndex].tick != run.hitTick)
        ++hitIndex;
    std::size_t floorIndex = hitIndex;
    while (floorIndex < run.rows.size() && !onFloor(run.rows[floorIndex]))
        ++floorIndex;
    for (std::size_t index = hitIndex; index < floorIndex; ++index)
    {
        const TickRow& row = run.rows[index];
        if (index > 0u && row.support != run.rows[index - 1u].support)
            ++counters.flips;
        if (row.blocked && row.walkable && !row.faceWalkable)
            ++counters.lipTicks;
    }
    counters.rimHang = std::abs(insideMargin(site.rim)) < kRimOnBoundaryM && e.label == "staysOnPlatform" && run.rows.back().pastRim > 0.f;
    return counters;
}


const char* kindField(brawlerMovementSimulation::SupportKind kind)
{
    if (kind == brawlerMovementSimulation::SupportKind::Face)
        return " kind=Face";
    return kind == brawlerMovementSimulation::SupportKind::Lip ? " kind=Lip" : " kind=None";
}

std::string estimateField(const Event& e)
{
    if (!e.departs)
        return "";
    char buffer[200];
    std::snprintf(buffer, sizeof(buffer), " estimateEdge=%u estimateEdgeAtHit=%d estimateAnomaly=%.4f estimateServo=%.4f estimateRedirect=%.4f estimateClass=%s",
        e.estimateEdgeTick, e.estimateEdgeAtHit ? 1 : 0, double(e.estimateAnomaly), double(e.estimateServo), double(e.estimateRedirect),
        e.estimateLabel.c_str());
    return buffer;
}

const char* sideName(Swing side) { return side == Swing::Right ? "R" : "L"; }
const char* supportName(uint8_t support) { return support == 0u ? "U" : (support == 1u ? "S" : "St"); }

void printTicks(const RunResult& run, const Event& e, const PlatformFrame& frame, const RimSite& site)
{
    const glm::vec3 outwardHorizontal = horizontalUnit(frame.inPlane(site.outward));
    const uint32_t last = e.departs ? std::max({ e.hitTick, e.reportTick, e.edgeTick }) + 3u : e.hitTick + kStayPrintTicks;
    for (const TickRow& row : run.rows)
    {
        if (row.tick + 2u < e.hitTick || row.tick > last)
            continue;
        std::printf("[PlatformEdge.tick] law=v2 site=%s d=%.2f obl=%+.0f t=%u rel=%d past=%.3f y=%.4f sup=%s clr=%.4f blk=%d hull=%d "
                    "walk=%d faceWalk=%d gjkTilt=%.2f exactTilt=%.2f exactDotTop=%.5f groundTop=%d nTilt=%.2f commit=%d hitTick=%d "
                    "written=%d detached=%d vyRaw=%.4f vyWritten=%.4f vy=%.4f |v|=%.4f vN=%.4f chanUp=%.4f servo=%.4f stiff=%.4f "
                    "damp=%.4f uvDy=%.4f pushDy=%.4f pushDv=%.4f residualDy=%.4f residualDv=%.4f vDotUp=%.4f mirrorErr=%.2e%s\n",
            site.name, double(e.distance), double(e.obliqueDeg), row.tick, int(row.tick) - int(e.hitTick), double(row.pastRim),
            double(row.position.y), supportName(row.support), double(row.clearance), row.blocked ? 1 : 0, row.hull, row.walkable ? 1 : 0,
            row.faceWalkable ? 1 : 0, double(tiltDeg(row.gjk, outwardHorizontal)), double(tiltDeg(row.exact, outwardHorizontal)),
            double(glm::dot(row.exact, frame.normal)), glm::dot(row.attackGround, frame.normal) > kFaceMatchCos ? 1 : 0,
            double(tiltDeg(row.n, outwardHorizontal)), row.committed ? 1 : 0, row.hitTick ? 1 : 0, row.written ? 1 : 0,
            row.detached ? 1 : 0, double(row.vRaw.y), double(row.vWritten.y), double(row.vActual.y), double(glm::length(row.vActual)),
            double(glm::dot(row.vActual, frame.normal)), double(row.channelUp), double(row.servoDv), double(row.stiffDv),
            double(row.dampDv), double(row.uvDy), double(row.pushDy), double(row.pushDv), double(row.residualDy),
            double(row.residualDv), double(row.vDotUp), double(row.mirrored ? glm::length(row.vMirror - row.vActual) : 0.f),
            kindField(row.kind));
    }
}

void printRun(const Event& e, const PlatformFrame& frame, const RimSite& site)
{
    const glm::vec3 outwardHorizontal = horizontalUnit(frame.inPlane(site.outward));
    std::printf("[PlatformEdge.run] law=v2 site=%s d=%.2f obl=%+.0f actualObl=%+.1f side=%s rested=%d restSup=%s restClr=%.4f restGjkTilt=%.2f "
                "restExactTop=%d hit=%u dirCos=%.4f outcome=%s edgeAtHit=%d edge=%u depart=%u report=%u landing=%u speed=%.3f "
                "vyEdge=%.4f anomaly=%.4f dy=%.4f servo=%.4f stiff=%.4f damp=%.4f uvSupTilted=%.4f uvDetTilted=%.4f uvOther=%.4f "
                "push=%.4f pushDv=%.4f residualDy=%.4f residualDv=%.4f flips=%u detachedWalkable=%u disagree=%u lipFace=%u hullSwitch=%u nTilt=[%.2f,%.2f] class=%s%s\n",
        site.name, double(e.distance), double(e.obliqueDeg), double(e.actualObliqueDeg), sideName(e.side), e.rested ? 1 : 0,
        supportName(e.rest.support), double(e.rest.clearance), double(tiltDeg(e.rest.gjk, outwardHorizontal)),
        exactIsTop(e.rest, frame) ? 1 : 0, e.hitTick, double(e.directionCos), e.departs ? "departs" : "stays", e.edgeAtHit ? 1 : 0,
        e.edgeTick, e.departTick, e.reportTick, e.landingTick, double(e.speedAtEdge), double(e.vyAtEdge), double(e.anomaly),
        double(e.heightDeviation), double(e.servo), double(e.stiff), double(e.damp), double(e.uvSupportedTilted),
        double(e.uvDetachedTilted), double(e.uvOther), double(e.push), double(e.pushSpeedLoss), double(e.residualDy),
        double(e.residualDv), e.supportFlips, e.detachedWalkableTicks, e.disagreeTicks,
        e.lipFaceTicks, e.hullSwitches, double(e.minNTiltDeg), double(e.maxNTiltDeg), e.label.c_str(), estimateField(e).c_str());
}

void knockbacksAcrossTheRims()
{
    const PlatformFrame frame = platformFrame();
    {
        INFO("the imported platform is the user's 30 degree, 0.1 m slab");
        REQUIRE(std::abs(glm::degrees(std::acos(frame.normal.y)) - 30.f) < 1e-3f);
        REQUIRE(std::abs(frame.top - 0.100125f) < 1e-6f);
        const quarterCylinderPlatform::ImportedHull& other = quarterCylinderPlatform::kHulls[1];
        REQUIRE(glm::dot(rotationOf(other) * glm::vec3(0.f, 1.f, 0.f), frame.normal) > kFaceMatchCos);
    }
    const bool printEveryTick = goldenTraceScene::environmentValue("OG_PLATFORM_EDGE_TICKS").value_or("") == "1";
    const std::string onlySite = goldenTraceScene::environmentValue("OG_PLATFORM_EDGE_SITE").value_or("");
    const bool printRuns = goldenTraceScene::environmentValue("OG_SUPPORT_LAW_RUNS").value_or("") == "1";
    uint32_t supportFlips = 0u;
    uint32_t rimHangs = 0u;
    uint32_t lipTicks = 0u;

    std::vector<Event> events;
    uint32_t runs = 0u;
    uint32_t mirrorMismatches = 0u;
    uint32_t supportMismatches = 0u;
    uint32_t pushOutMismatches = 0u;
    uint32_t mirroredTicks = 0u;
    uint32_t unreachable = 0u;
    uint32_t lossyTicks = 0u;
    uint32_t lossyOnPureFace = 0u;
    std::array<uint32_t, 3u> kindTicks = { 0u, 0u, 0u };
    float largestResidual = 0.f;
    for (const RimSite& site : kSites)
    {
        if (!onlySite.empty() && onlySite != site.name)
            continue;
        for (const float distance : kDistances)
            for (const float oblique : kObliqueDeg)
            {
                const glm::vec2 wanted = rotated(site.outward, oblique);
                std::optional<RunResult> chosen;
                Event chosenEvent;
                for (const float shift : kAttackerInwardShifts)
                    for (const bool flipAttacker : { false, true })
                        for (const Swing side : { Swing::Right, Swing::Left })
                        {
                            if (chosen.has_value() && chosenEvent.directionCos >= kGoodDirectionCos)
                                break;
                            const std::optional<Placement> placement = placementFor(frame, site, distance, wanted, side, flipAttacker, shift);
                            if (!placement.has_value())
                                continue;
                            RunResult run = runOnce(*placement, frame, site);
                            ++runs;
                            mirrorMismatches += run.mirrorMismatches;
                            supportMismatches += run.supportMismatches;
                            pushOutMismatches += run.pushOutMismatches;
                            for (const TickRow& row : run.rows)
                                mirroredTicks += row.mirrored ? 1u : 0u;
                            const Event event = analyse(run, frame, site, distance, oblique, side, wanted);
                            if (printEveryTick)
                            {
                                const glm::vec3 offset = run.rest.position - run.attackerAtSwing;
                                std::printf("[PlatformEdge.attempt] law=v2 site=%s d=%.2f obl=%+.0f shift=%.2f flip=%d side=%s rested=%d hit=%d "
                                            "dirCos=%.4f meleeHits=%u inPlaneM=%.3f alongNormalM=%.3f targetPast=%.3f\n",
                                    site.name, double(distance), double(oblique), double(shift), flipAttacker ? 1 : 0, sideName(side),
                                    run.rested ? 1 : 0, run.hit ? 1 : 0, double(event.directionCos), run.attackerHits,
                                    double(glm::length(offset - frame.normal * glm::dot(offset, frame.normal))),
                                    double(glm::dot(offset, frame.normal)), double(run.rest.pastRim));
                            }
                            if (run.hit && event.directionCos >= kDirectionCos
                                && (!chosen.has_value() || event.directionCos > chosenEvent.directionCos))
                            {
                                chosen = std::move(run);
                                chosenEvent = event;
                            }
                        }
                INFO("site " << site.name << " distance " << distance << " oblique " << oblique);
                CHECK((chosen.has_value() || distance < kReachableFromDistance));
                if (!chosen.has_value())
                {
                    ++unreachable;
                    if (printRuns)
                        std::printf("[PlatformEdge.unreachable] law=v2 site=%s d=%.2f obl=%+.0f\n", site.name, double(distance), double(oblique));
                    continue;
                }
                const SupportCounters counters = supportCountersOf(*chosen, chosenEvent, site);
                supportFlips += counters.flips;
                rimHangs += counters.rimHang ? 1u : 0u;
                lipTicks += counters.lipTicks;
                if (printRuns)
                {
                    printRun(chosenEvent, frame, site);
                    std::printf("[PlatformEdge.support] law=v2 site=%s d=%.2f obl=%+.0f flips=%u rimHang=%d lipTicks=%u\n", site.name,
                        double(distance), double(oblique), counters.flips, counters.rimHang ? 1 : 0, counters.lipTicks);
                }
                for (const TickRow& row : chosen->rows)
                {
                    if (row.mirrored)
                        ++kindTicks[static_cast<std::size_t>(row.kind)];
                    if (!row.mirrored || row.residualDv <= kResidualTolerance)
                        continue;
                    ++lossyTicks;
                    lossyOnPureFace += isPureFace(row, frame) ? 1u : 0u;
                    largestResidual = std::max(largestResidual, row.residualDv);
                }
                if (printEveryTick)
                    printTicks(*chosen, chosenEvent, frame, site);
                events.push_back(chosenEvent);
            }
    }

    uint32_t lifts = 0u;
    uint32_t downPulls = 0u;
    uint32_t stays = 0u;
    uint32_t estimateLifts = 0u;
    uint32_t estimateDownPulls = 0u;
    for (const RimSite& site : kSites)
    {
        uint32_t siteLifts = 0u;
        uint32_t siteDown = 0u;
        uint32_t siteEstimateLifts = 0u;
        uint32_t siteEstimateDown = 0u;
        float largestEstimateLift = 0.f;
        float largestEstimateDown = 0.f;
        uint32_t siteStays = 0u;
        uint32_t siteRuns = 0u;
        float largestLift = 0.f;
        float largestDown = 0.f;
        for (const Event& e : events)
        {
            if (std::string(e.site) != site.name)
                continue;
            ++siteRuns;
            siteLifts += e.label == "lift" ? 1u : 0u;
            siteDown += e.label == "downPull" ? 1u : 0u;
            siteStays += e.label == "staysOnPlatform" ? 1u : 0u;
            largestLift = std::max(largestLift, e.anomaly);
            largestDown = std::min(largestDown, e.anomaly);
            siteEstimateLifts += e.departs && e.estimateLabel == "lift" ? 1u : 0u;
            siteEstimateDown += e.departs && e.estimateLabel == "downPull" ? 1u : 0u;
            if (e.departs)
            {
                largestEstimateLift = std::max(largestEstimateLift, e.estimateAnomaly);
                largestEstimateDown = std::min(largestEstimateDown, e.estimateAnomaly);
            }
        }
        if (siteRuns == 0u)
            continue;
        lifts += siteLifts;
        downPulls += siteDown;
        stays += siteStays;
        estimateLifts += siteEstimateLifts;
        estimateDownPulls += siteEstimateDown;
        std::printf("[PlatformEdge.site] law=v2 site=%s runs=%u lifts=%u downPulls=%u staysOnPlatform=%u largestLift=%.4f largestDownPull=%.4f\n",
            site.name, siteRuns, siteLifts, siteDown, siteStays, double(largestLift), double(largestDown));
        std::printf("[PlatformEdge.estimateSite] law=v2 site=%s runs=%u lifts=%u downPulls=%u largestLift=%.4f largestDownPull=%.4f\n",
            site.name, siteRuns, siteEstimateLifts, siteEstimateDown, double(largestEstimateLift), double(largestEstimateDown));
    }
    std::printf("[PlatformEdge.summary] law=v2 runs=%u placements=%zu unreachable=%u lifts=%u downPulls=%u staysOnPlatform=%u mirroredTicks=%u "
                "mirrorMismatches=%u supportMismatches=%u pushOutMismatches=%u\n",
        runs, events.size(), unreachable, lifts, downPulls, stays, mirroredTicks, mirrorMismatches, supportMismatches, pushOutMismatches);

    std::printf("[PlatformEdge.decomposition] law=v2 lossyTicks=%u lossyOnPureFace=%u largestResidualMps=%.4f\n", lossyTicks, lossyOnPureFace,
        double(largestResidual));
    std::printf("[SupportLaw.scorecard] law=v2 rig=platformEdge runs=%zu flips=%u lifts=%u downPulls=%u rimHangs=%u lipTicks=%u\n",
        events.size(), supportFlips, lifts, downPulls, rimHangs, lipTicks);
    std::printf("[PlatformEdge.estimate] law=v2 noneTicks=%u faceTicks=%u lipTicks=%u\n", kindTicks[0], kindTicks[1], kindTicks[2]);
    std::printf("[PlatformEdge.estimateSummary] law=v2 lifts=%u downPulls=%u\n", estimateLifts, estimateDownPulls);
    {
        uint32_t lipHitPlacements = 0u;
        float largestLipHitServo = 0.f;
        for (const Event& e : events)
        {
            if (e.distance != kLipHitDistance || std::string(e.site).rfind("tip", 0u) != 0u)
                continue;
            ++lipHitPlacements;
            largestLipHitServo = std::max(largestLipHitServo, e.servo);
            INFO("Lip-hit placement " << e.site << " oblique " << e.obliqueDeg << " servo " << e.servo);
            CHECK(e.servo <= kEventThreshold);
        }
        std::printf("[PlatformEdge.lipHits] law=v2 placements=%u largestServo=%.4f\n", lipHitPlacements, double(largestLipHitServo));
        if (onlySite.empty())
            CHECK(lipHitPlacements == 16u);
        if (onlySite.empty())
        {
            CHECK(estimateLifts <= 1u);
            CHECK(estimateDownPulls == 0u);
        }
        for (const Event& e : events)
        {
            if (!e.departs || e.estimateLabel != "lift")
                continue;
            INFO("estimate lift " << e.site << " d " << e.distance << " oblique " << e.obliqueDeg << " anomaly " << e.estimateAnomaly
                 << " servo " << e.estimateServo << " redirect " << e.estimateRedirect);
            CHECK(std::string(e.site) == "tipCornerPlus");
            CHECK(e.distance == kDistances.back());
            CHECK(e.obliqueDeg == 0.f);
            CHECK(e.estimateRedirect == 0.f);
        }
    }
    if (onlySite.empty())
    {
        CHECK(supportFlips == 214u);
        CHECK(rimHangs == 17u);
        CHECK(lipTicks == 308u);
    }

    CHECK(mirroredTicks > 0u);
    CHECK(mirrorMismatches == 0u);
    CHECK(supportMismatches == 0u);
    CHECK(pushOutMismatches == 0u);
}

TEST_CASE("PlatformEdge.KnockbacksAcrossTheImportedPlatformRimsUnderV2", "[GoldenTrace]")
{
    knockbacksAcrossTheRims();
}

} // namespace platformEdgeTests

#endif
