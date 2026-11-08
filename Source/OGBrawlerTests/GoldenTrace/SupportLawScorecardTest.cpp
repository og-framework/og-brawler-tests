// SPDX-License-Identifier: BUSL-1.1
// docs/SupportLawScorecardTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceScenario.h"
#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"

#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include "glm/geometric.hpp"
#include "glm/mat4x4.hpp"
#include "glm/trigonometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace supportLawScorecardTests
{

using goldenTraceScenario::InputRow;
using goldenTraceSlopeScenario::Swing;

inline constexpr unsigned int kTarget = 1u;
inline constexpr unsigned int kStunner = 2u;
inline constexpr unsigned int kShooter = 3u;
inline constexpr uint32_t kSwingTick = 61u;
inline constexpr uint32_t kSwingToHitTicks = 24u;
inline constexpr uint32_t kRunTicks = 240u;
inline constexpr uint32_t kWalkFirstTick = 30u;
inline constexpr uint32_t kMidSlideDelayTicks = 18u;
inline constexpr uint32_t kMidSlideDelaySlackTicks = 4u;
inline constexpr uint32_t kShotDelayTicks = 12u;
inline constexpr uint32_t kCreaseWalkTicks = 120u;
inline constexpr uint32_t kFlatDirections = 20u;
inline constexpr float kFlatStepDeg = 18.f;
inline constexpr std::array<float, 2u> kAttackerRadii = { 1.6f, 1.2f };
inline constexpr std::array<float, 3u> kAttackerShifts = { 0.f, 0.45f, -0.45f };
inline constexpr std::array<uint32_t, 4u> kMidSlideSwingDelays = { 0u, 3u, 6u, 9u };
inline constexpr float kStunnerRadius = 1.6f;
inline constexpr float kShooterBehindM = 2.f;
inline constexpr float kPathClearanceM = 1.f;
inline constexpr float kDirectionCos = 0.82f;
inline constexpr float kGoodDirectionCos = 0.99f;
inline constexpr float kClearanceSweepM = 20.f;
inline constexpr float kHopApexM = 0.05f;
inline constexpr float kSlideEndFraction = 0.15f;
inline constexpr float kPinTolerance = 5e-4f;
inline constexpr float kRedirectSpeedTolerance = 1e-3f;
inline constexpr float kSameFaceCos = 0.99985f;
inline constexpr std::array<float, 4u> kCreaseDistances = { 0.3f, 1.5f, 3.5f, 4.5f };
inline constexpr std::array<float, 3u> kCreaseObliqueDeg = { 0.f, 30.f, -30.f };
inline constexpr float kProjectileDistance = 0.3f;
inline constexpr float kCreaseWalkStartM = 1.f;
inline constexpr glm::vec3 kSlabCentre{ 0.f, -0.25f, 0.f };
inline constexpr glm::vec3 kSlabHalfExtents{ 30.f, 0.25f, 30.f };
inline constexpr glm::vec3 kFirstKnockback{ 1.f, 0.f, 0.f };
inline constexpr glm::vec3 kFirstAttackerParking{ -25.f, 0.f, -25.f };
inline constexpr uint32_t kFirstAttackerLeavesTicks = 2u;

bool printsRuns()
{
    return goldenTraceScene::environmentValue("OG_SUPPORT_LAW_RUNS").value_or("") == "1";
}

enum class World
{
    Flat,
    Slope,
};

StaticWorldDescription flatStatics()
{
    StaticWorldDescription description;
    description.shapes.push_back(goldenTraceScenario::staticBox(kSlabCentre, glm::quat(1.f, 0.f, 0.f, 0.f), kSlabHalfExtents, 1u));
    return description;
}

simulatableBrawler::StaticData scorecardStaticData(float gravityScale)
{
    return goldenTraceScenario::scenarioStaticData(gravityScale);
}

simulatableBrawler::PlayerInput inputOf(const InputRow& row)
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

struct RigActor
{
    unsigned int id = 0u;
    glm::vec3 spawn{ 0.f };
    glm::vec3 aim{ 0.f };
};

struct RigScenario
{
    static constexpr const char* kGenerator = "SupportLawScorecard.run";
    static constexpr float kDtSeconds = goldenTraceScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = kRunTicks;

    static inline World world = World::Flat;
    static inline std::vector<RigActor> actors{};
    static inline std::vector<InputRow> script{};
    static inline std::vector<goldenTraceScene::Character> characterTable{};
    static inline std::vector<goldenTraceScene::SpawnPoint> spawnPoints{};
    static inline std::vector<goldenTraceScene::Teleport> teleportTable{};

    static void set(World w, const std::vector<RigActor>& a, const std::vector<InputRow>& s, const std::vector<goldenTraceScene::Teleport>& t)
    {
        world = w;
        actors = a;
        script = s;
        teleportTable = t;
        characterTable.clear();
        spawnPoints.clear();
        for (uint32_t slot = 0u; slot < actors.size(); ++slot)
        {
            characterTable.push_back(goldenTraceScene::Character{ actors[slot].id, slot, "", actors[slot].spawn });
            spawnPoints.push_back(goldenTraceScene::SpawnPoint{ slot, actors[slot].spawn });
        }
    }

    static bool isSampleTick(uint32_t) { return false; }
    static StaticWorldDescription statics() { return world == World::Flat ? flatStatics() : goldenTraceSlopeScenario::slopeStatics(); }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return scorecardStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int id, uint32_t tick)
    {
        for (const InputRow& row : script)
            if (row.characterId == id && tick >= row.firstTick && tick <= row.lastTick)
                return inputOf(row);
        for (const RigActor& actor : actors)
            if (actor.id == id)
                return inputOf(goldenTraceSlopeScenario::still(id, tick, tick, actor.aim));
        return simulatableBrawler::getZeroPlayerInput();
    }
    static std::span<const goldenTraceScene::Character> characters() { return characterTable; }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable() { return spawnPoints; }
    static std::span<const goldenTraceScene::Teleport> teleports() { return teleportTable; }
};

static_assert(goldenTraceScene::Scenario<RigScenario>, "the scorecard scenario provides everything goldenTraceScene::Scene reads");

struct TickRow
{
    uint32_t tick = 0u;
    glm::vec3 position{ 0.f };
    glm::vec3 velocity{ 0.f };
    uint8_t support = 0u;
    uint8_t derivedSupport = 0u;
    bool blocked = false;
    float clearance = 0.f;
    bool hitFlinch = false;
    bool hit = false;
    HitReactionKind reaction = HitReactionKind::Stun;
    float knockbackSpeed = 0.f;
    glm::vec3 hitDirection{ 0.f };
    bool committed = false;
    glm::vec3 faceNormal{ 0.f };
    float probeClearance = 0.f;
    glm::vec3 velocityIn{ 0.f };
};

struct RigRun
{
    std::vector<TickRow> rows;
    uint32_t supportSourceMismatches = 0u;
    float rideHeight = 0.f;
    float probeLength = 0.f;
    float slideEndSpeed = 0.f;
    float gravityStep = 0.f;
    float decelStep = 0.f;
};

glm::vec3 velocityEnteringTheRedirect(const brawlerMovementSimulation::State& pre, const brawlerMovementSimulation::StaticData& sd,
    const brawlerInboundHit::DerivedState& inbound, bool committed, float dt)
{
    using namespace brawlerMovementSimulation;
    glm::vec3 velocity = pre.velocity;
    if ((pre.flags & kFlagHasCommand) != 0u)
    {
        const glm::vec3 pushOut = pre.bodyState.position - (pre.positionCmd + pre.velocity * dt);
        if (glm::dot(pushOut, pushOut) > kPushOutEps * kPushOutEps)
        {
            const glm::vec3 contactNormal = glm::normalize(pushOut);
            if (glm::dot(contactNormal, kWorldUp) >= sd.cosMaxSlope)
                velocity -= glm::dot(velocity, kWorldUp) * kWorldUp;
            else
                velocity -= glm::dot(velocity, contactNormal) * contactNormal;
        }
    }
    if (writesKnockbackVelocity(inbound, committed))
        velocity = inbound.hitDirectionWorld * inbound.knockbackSpeed;
    return velocity;
}

RigRun runRig(World world, const std::vector<RigActor>& actors, const std::vector<InputRow>& script,
    const std::vector<goldenTraceScene::Teleport>& teleports = {})
{
    RigScenario::set(world, actors, script, teleports);
    goldenTraceScene::Scene<RigScenario> scene(std::nullopt, 1.f);
    const brawlerMovementSimulation::StaticData& sd = scene.staticData.m_movementStaticData;
    RigRun run;
    run.rideHeight = sd.rideHeight;
    run.probeLength = sd.rideHeight + sd.snapDistance;
    run.slideEndSpeed = kSlideEndFraction * scene.staticData.m_hitReactions[0].knockbackSpeed;
    run.gravityStep = sd.gravity * RigScenario::kDtSeconds;
    run.decelStep = sd.launchDecel * RigScenario::kDtSeconds;
    for (uint32_t tick = 1u; tick <= kRunTicks; ++tick)
    {
        const SimulatableBrawler& before = scene.storage.template get<SimulatableBrawler>(kTarget);
        const glm::vec3 position = before.getAllState().getState().template get<brawlerMovementSimulation::State>().bodyState.position;
        const auto& bindings = before.getPhysicsComposite().template get<brawlerMovementSimulation::PhysicsDeclaration>().bindings;
        glm::mat4 pose(1.f);
        pose[3] = glm::vec4(position, 1.f);
        const SweepHit below = scene.query.sweep(bindings.queryVolumeIds[0], pose, -brawlerMovementSimulation::kWorldUp * kClearanceSweepM);
        const brawlerMovementSimulation::State pre = before.getAllState().getState().template get<brawlerMovementSimulation::State>();
        const brawlerMovementSimulation::GroundSupport ground = brawlerMovementSimulation::estimateSupport(pre, sd, bindings, scene.query);

        scene.step(tick);

        const SimulatableBrawler& after = scene.storage.template get<SimulatableBrawler>(kTarget);
        const auto& post = after.getAllState().getState().template get<brawlerMovementSimulation::State>();
        const auto& derived = after.getAllState().getDerivedState().template get<brawlerMovementSimulation::DerivedState>();
        const auto& inbound = after.getAllState().getDerivedState().template get<brawlerInboundHit::DerivedState>();
        const auto& machine = after.getAllState().getState().template get<dAttackMachineSimulation::State>();

        TickRow row;
        row.tick = tick;
        row.position = position;
        row.velocity = post.velocity;
        row.support = static_cast<uint8_t>((post.flags & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift);
        row.derivedSupport = derived.lastSupportState;
        row.blocked = below.blocked;
        row.clearance = below.blocked ? below.fraction * kClearanceSweepM : kClearanceSweepM;
        row.hitFlinch = machine.m_currentState == DAttackState::HitFlinch;
        row.hit = inbound.wasHitThisTick;
        row.reaction = inbound.reactionKind;
        row.knockbackSpeed = inbound.knockbackSpeed;
        row.hitDirection = inbound.hitDirectionWorld;
        row.committed = brawlerMovementSimulation::machineLaunchesMovement(machine);
        row.faceNormal = ground.faceNormal;
        row.probeClearance = ground.probe.clearance;
        row.velocityIn = velocityEnteringTheRedirect(pre, sd, inbound, row.committed, RigScenario::kDtSeconds);
        if (row.support != row.derivedSupport)
            ++run.supportSourceMismatches;
        run.rows.push_back(row);
    }
    return run;
}

std::optional<std::size_t> firstHitIndex(const RigRun& run, uint32_t fromTick, std::optional<HitReactionKind> reaction)
{
    for (std::size_t index = 0u; index < run.rows.size(); ++index)
    {
        const TickRow& row = run.rows[index];
        if (row.tick >= fromTick && row.hit && (!reaction.has_value() || row.reaction == *reaction))
            return index;
    }
    return std::nullopt;
}

std::size_t indexOfTick(const RigRun& run, uint32_t tick)
{
    return std::min<std::size_t>(tick - 1u, run.rows.size() - 1u);
}

std::size_t lockoutEndIndex(const RigRun& run, std::size_t hitIndex)
{
    std::size_t index = hitIndex + 1u;
    while (index < run.rows.size() && run.rows[index].hitFlinch)
        ++index;
    return std::min(index, run.rows.size() - 1u);
}

uint32_t transitionsIn(const RigRun& run, std::size_t first, std::size_t end)
{
    uint32_t transitions = 0u;
    for (std::size_t index = std::max<std::size_t>(first, 1u); index < end && index < run.rows.size(); ++index)
        transitions += run.rows[index].support != run.rows[index - 1u].support ? 1u : 0u;
    return transitions;
}

uint32_t unsupportedIn(const RigRun& run, std::size_t first, std::size_t end)
{
    uint32_t ticks = 0u;
    for (std::size_t index = first; index < end && index < run.rows.size(); ++index)
        ticks += run.rows[index].support == 0u ? 1u : 0u;
    return ticks;
}

uint32_t slideEndTicksIn(const RigRun& run, std::size_t first, std::size_t end)
{
    uint32_t ticks = 0u;
    for (std::size_t index = first; index < end && index < run.rows.size(); ++index)
    {
        const TickRow& row = run.rows[index];
        ticks += (row.support == 0u && row.blocked && row.clearance <= run.probeLength && glm::length(row.velocity) < run.slideEndSpeed) ? 1u : 0u;
    }
    return ticks;
}

float apexIn(const RigRun& run, std::size_t first, std::size_t end)
{
    float apex = -run.rideHeight;
    for (std::size_t index = first; index < end && index < run.rows.size(); ++index)
        if (run.rows[index].blocked)
            apex = std::max(apex, run.rows[index].clearance - run.rideHeight);
    return apex;
}

glm::vec3 horizontalUnit(const glm::vec3& v)
{
    return glm::normalize(glm::vec3(v.x, 0.f, v.z));
}

glm::vec3 headingRotated(const glm::vec3& heading, float degrees)
{
    const float c = std::cos(glm::radians(degrees));
    const float s = std::sin(glm::radians(degrees));
    return glm::vec3(c * heading.x - s * heading.z, 0.f, s * heading.x + c * heading.z);
}

glm::vec3 inPlane(const glm::vec3& heading, const glm::vec3& normal)
{
    return glm::normalize(glm::vec3(heading.x, -(normal.x * heading.x + normal.z * heading.z) / normal.y, heading.z));
}

glm::vec3 lateralOf(const glm::vec3& heading) { return glm::vec3(-heading.z, 0.f, heading.x); }

float horizontalDistanceToSegment(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b)
{
    const glm::vec2 p(point.x, point.z);
    const glm::vec2 s(a.x, a.z);
    const glm::vec2 along = glm::vec2(b.x, b.z) - s;
    const float lengthSq = glm::dot(along, along);
    const float t = lengthSq > 0.f ? std::clamp(glm::dot(p - s, along) / lengthSq, 0.f, 1.f) : 0.f;
    return glm::length(p - (s + along * t));
}

const char* supportName(uint8_t support) { return support == 0u ? "U" : (support == 1u ? "S" : "St"); }

glm::vec3 flatSurface(float x, float z) { return glm::vec3(x, 0.f, z); }

glm::vec3 slopeSurface(float x, float z)
{
    using namespace goldenTraceSlopeScenario;
    if (z >= kLowerCreaseZ)
        return glm::vec3(x, kLowerCreaseHeight, z);
    if (z <= kUpperCreaseZ)
        return glm::vec3(x, kUpperCreaseHeight, z);
    return glm::vec3(x, kLowerCreaseHeight + (kLowerCreaseZ - z) * (kUpTheFallLine.y / -kUpTheFallLine.z), z);
}

glm::vec3 slopeNormal(float z)
{
    using namespace goldenTraceSlopeScenario;
    return (z < kLowerCreaseZ && z > kUpperCreaseZ) ? kFaceNormal : brawlerMovementSimulation::kWorldUp;
}

glm::vec3 surfaceOf(World world, float x, float z) { return world == World::Flat ? flatSurface(x, z) : slopeSurface(x, z); }

glm::vec3 normalOf(World world, float z) { return world == World::Flat ? brawlerMovementSimulation::kWorldUp : slopeNormal(z); }

RigActor actorAt(unsigned int id, const glm::vec3& face, const glm::vec3& aim)
{
    return RigActor{ id, goldenTraceSlopeScenario::raised(face, goldenTraceSlopeScenario::kTeleportHeight), aim };
}

struct MeleeChoice
{
    RigRun run;
    std::vector<RigActor> actors;
    std::vector<InputRow> script;
    std::vector<goldenTraceScene::Teleport> teleports;
    std::size_t hitIndex = 0u;
    float directionCos = -2.f;
};

struct MeleeSetup
{
    World world = World::Flat;
    glm::vec3 targetAt{ 0.f };
    glm::vec3 heading{ 0.f };
    std::vector<RigActor> actors;
    std::vector<InputRow> script;
    std::vector<goldenTraceScene::Teleport> teleports;
    uint32_t swingTick = kSwingTick;
    uint32_t measureFromTick = kSwingTick;
    std::optional<std::pair<glm::vec3, glm::vec3>> avoidPath;
    std::optional<std::pair<uint32_t, uint32_t>> hitTickRange;
    std::vector<uint32_t> swingDelays{ 0u };
};

struct MeleeCandidate
{
    float radius = 0.f;
    float sign = 0.f;
    Swing side = Swing::Right;
    float shift = 0.f;
    uint32_t swingDelay = 0u;
};

std::optional<MeleeChoice> meleeAttempt(const MeleeSetup& setup, const MeleeCandidate& candidate)
{
    const float radius = candidate.radius;
    const float sign = candidate.sign;
    const Swing side = candidate.side;
    const uint32_t swingTick = setup.swingTick + candidate.swingDelay;
    const glm::vec3 offset = lateralOf(setup.heading) * (radius * sign) + setup.heading * candidate.shift;
    const glm::vec3 attackerFace = surfaceOf(setup.world, setup.targetAt.x + offset.x, setup.targetAt.z + offset.z);
    if (setup.avoidPath.has_value() && horizontalDistanceToSegment(attackerFace, setup.avoidPath->first, setup.avoidPath->second) < kPathClearanceM)
        return std::nullopt;
    const glm::vec3 aim = horizontalUnit(setup.targetAt - attackerFace);
    const unsigned int id = static_cast<unsigned int>(setup.actors.size() + 1u);
    MeleeChoice choice;
    choice.actors = setup.actors;
    choice.actors.push_back(actorAt(id, attackerFace, aim));
    choice.script = setup.script;
    choice.script.push_back(goldenTraceSlopeScenario::swing(id, swingTick, aim, side));
    choice.teleports = setup.teleports;
    choice.run = runRig(setup.world, choice.actors, choice.script, choice.teleports);
    const std::optional<std::size_t> hitIndex = firstHitIndex(choice.run, setup.measureFromTick, HitReactionKind::Knockback);
    const uint32_t hitTick = hitIndex.has_value() ? choice.run.rows[*hitIndex].tick : 0u;
    const bool inRange = !setup.hitTickRange.has_value() || (hitTick >= setup.hitTickRange->first && hitTick <= setup.hitTickRange->second);
    const glm::vec3 wanted = inPlane(setup.heading, normalOf(setup.world, setup.targetAt.z));
    choice.directionCos = hitIndex.has_value() ? glm::dot(glm::normalize(choice.run.rows[*hitIndex].hitDirection), wanted) : -2.f;
    if (printsRuns())
    {
        const std::optional<std::size_t> anyHit = firstHitIndex(choice.run, setup.measureFromTick, std::nullopt);
        std::printf("[SupportLaw.attempt] law=v2 at=(%.2f,%.2f,%.2f) heading=(%.3f,%.3f) radius=%.1f sign=%+.0f side=%s shift=%+.2f swing=%u hit=%u "
                    "inRange=%d dirCos=%.4f firstHit=%u firstReaction=%u\n",
            double(setup.targetAt.x), double(setup.targetAt.y), double(setup.targetAt.z), double(setup.heading.x), double(setup.heading.z),
            double(radius), double(sign), side == Swing::Right ? "R" : "L", double(candidate.shift), swingTick, hitTick, inRange ? 1 : 0,
            double(choice.directionCos), anyHit.has_value() ? choice.run.rows[*anyHit].tick : 0u,
            anyHit.has_value() ? unsigned(choice.run.rows[*anyHit].reaction) : 9u);
    }
    if (!hitIndex.has_value() || !inRange)
        return std::nullopt;
    choice.hitIndex = *hitIndex;
    return choice;
}

std::optional<MeleeChoice> bestMelee(const MeleeSetup& setup)
{
    std::optional<MeleeChoice> best;
    for (const uint32_t swingDelay : setup.swingDelays)
        for (const float shift : kAttackerShifts)
            for (const float radius : kAttackerRadii)
                for (const float sign : { 1.f, -1.f })
                    for (const Swing side : { Swing::Right, Swing::Left })
                    {
                        if (best.has_value() && best->directionCos >= kGoodDirectionCos)
                            return best;
                        std::optional<MeleeChoice> tried = meleeAttempt(setup, MeleeCandidate{ radius, sign, side, shift, swingDelay });
                        if (tried.has_value() && tried->directionCos >= kDirectionCos
                            && (!best.has_value() || tried->directionCos > best->directionCos))
                            best = std::move(tried);
                    }
    return best;
}

template <typename T>
void checkPin(const char* rig, const char* field, T measured, T pinned)
{
    INFO("rig " << rig << " field " << field << " measured " << measured << " pinned " << pinned);
    if constexpr (std::is_floating_point_v<T>)
        CHECK(std::abs(measured - pinned) <= kPinTolerance);
    else
        CHECK(measured == pinned);
}

enum class FlatState
{
    Rest,
    Walk,
    MidSlide,
};

const char* flatStateName(FlatState state)
{
    return state == FlatState::Rest ? "rest" : (state == FlatState::Walk ? "walk" : "midSlide");
}

struct FlatRun
{
    FlatState state = FlatState::Rest;
    float headingDeg = 0.f;
    uint32_t hitTick = 0u;
    uint32_t delayTicks = 0u;
    float directionCos = 0.f;
    float hitClearance = 0.f;
    float preHitVy = 0.f;
    uint32_t exitTick = 0u;
    uint32_t flips = 0u;
    uint32_t unsupported = 0u;
    uint32_t slideEndTicks = 0u;
    float apexM = 0.f;
    float slideM = 0.f;
};

struct FlatTotals
{
    uint32_t runs = 0u;
    uint32_t flips = 0u;
    uint32_t flickerRuns = 0u;
    uint32_t unsupported = 0u;
    uint32_t slideEndTicks = 0u;
    float apexMaxM = 0.f;
    float slideMinM = 0.f;
    float slideMaxM = 0.f;
    float apexSumM = 0.f;
    float slideSumM = 0.f;
};

FlatRun measureFlat(FlatState state, float headingDeg, const MeleeChoice& choice, uint32_t referenceTick)
{
    const RigRun& run = choice.run;
    const std::size_t hitIndex = choice.hitIndex;
    FlatRun r;
    r.state = state;
    r.headingDeg = headingDeg;
    r.hitTick = run.rows[hitIndex].tick;
    r.delayTicks = r.hitTick - referenceTick;
    r.directionCos = choice.directionCos;
    r.hitClearance = run.rows[hitIndex].clearance;
    r.preHitVy = hitIndex > 0u ? run.rows[hitIndex - 1u].velocity.y : 0.f;
    const std::size_t exitIndex = lockoutEndIndex(run, hitIndex);
    r.exitTick = run.rows[exitIndex].tick;
    r.flips = transitionsIn(run, hitIndex, exitIndex + 1u);
    r.unsupported = unsupportedIn(run, hitIndex, exitIndex + 1u);
    r.slideEndTicks = slideEndTicksIn(run, hitIndex, exitIndex + 1u);
    r.apexM = apexIn(run, hitIndex, exitIndex + 1u);
    const glm::vec3 moved = run.rows[exitIndex].position - run.rows[hitIndex].position;
    r.slideM = std::hypot(moved.x, moved.z);
    return r;
}

void printFlatRun(const FlatRun& r, const RigRun& run)
{
    std::printf("[SupportLaw.flat.run] law=v2 state=%s dir=%.0f hit=%u delay=%u dirCos=%.4f hitClr=%.5f preHitVy=%.3e exit=%u flips=%u "
                "unsupported=%u apexM=%.4f slideM=%.4f slideEndTicks=%u\n",
        flatStateName(r.state), double(r.headingDeg), r.hitTick, r.delayTicks, double(r.directionCos), double(r.hitClearance),
        double(r.preHitVy), r.exitTick, r.flips, r.unsupported, double(r.apexM), double(r.slideM), r.slideEndTicks);
    for (const TickRow& row : run.rows)
    {
        if (row.tick + 2u < r.hitTick || row.tick > r.exitTick + 2u)
            continue;
        std::printf("[SupportLaw.flat.tick] law=v2 state=%s dir=%.0f t=%u rel=%d sup=%s clr=%.5f vy=%.5f |v|=%.4f flinch=%d hit=%d\n",
            flatStateName(r.state), double(r.headingDeg), row.tick, int(row.tick) - int(r.hitTick), supportName(row.support),
            double(row.clearance), double(row.velocity.y), double(glm::length(row.velocity)), row.hitFlinch ? 1 : 0, row.hit ? 1 : 0);
    }
}

FlatTotals runFlatRig()
{
    const glm::vec3 origin = flatSurface(0.f, 0.f);
    const RigActor target = actorAt(kTarget, origin, goldenTraceSlopeScenario::kTowardPlusX);
    const bool printRuns = printsRuns();

    const std::vector<InputRow> walkScript = { goldenTraceSlopeScenario::walk(kTarget, kWalkFirstTick, kSwingTick + kSwingToHitTicks + 6u,
        goldenTraceSlopeScenario::kTowardPlusX) };
    const RigRun walkDry = runRig(World::Flat, { target }, walkScript);
    const glm::vec3 walkAtHit = walkDry.rows[indexOfTick(walkDry, kSwingTick + kSwingToHitTicks)].position;

    MeleeSetup firstSetup;
    firstSetup.targetAt = origin;
    firstSetup.heading = kFirstKnockback;
    firstSetup.actors = { target };
    const std::optional<MeleeChoice> first = bestMelee(firstSetup);
    REQUIRE(first.has_value());
    const uint32_t firstHitTick = first->run.rows[first->hitIndex].tick;
    const glm::vec3 firstHitAt = first->run.rows[first->hitIndex].position;
    const glm::vec3 midSlideAt = first->run.rows[indexOfTick(first->run, firstHitTick + kMidSlideDelayTicks)].position;

    FlatTotals totals;
    std::vector<FlatRun> results;
    uint32_t sourceMismatches = 0u;
    for (const FlatState state : { FlatState::Rest, FlatState::Walk, FlatState::MidSlide })
        for (uint32_t step = 0u; step < kFlatDirections; ++step)
        {
            const float headingDeg = kFlatStepDeg * float(step);
            MeleeSetup setup;
            setup.heading = headingRotated(kFirstKnockback, headingDeg);
            setup.actors = { target };
            uint32_t referenceTick = kSwingTick;
            if (state == FlatState::Rest)
            {
                setup.targetAt = origin;
            }
            else if (state == FlatState::Walk)
            {
                setup.targetAt = walkAtHit;
                setup.script = walkScript;
                setup.avoidPath = std::make_pair(origin, walkAtHit);
            }
            else
            {
                setup.targetAt = midSlideAt;
                setup.actors = first->actors;
                setup.script = first->script;
                setup.teleports = { goldenTraceScene::Teleport{ first->actors.back().id, firstHitTick + kFirstAttackerLeavesTicks,
                    goldenTraceSlopeScenario::raised(kFirstAttackerParking, goldenTraceSlopeScenario::kTeleportHeight) } };
                setup.swingTick = firstHitTick + kMidSlideDelayTicks - kSwingToHitTicks;
                setup.measureFromTick = firstHitTick + 1u;
                setup.avoidPath = std::make_pair(firstHitAt, midSlideAt);
                setup.hitTickRange = std::make_pair(firstHitTick + kMidSlideDelayTicks - kMidSlideDelaySlackTicks,
                    firstHitTick + kMidSlideDelayTicks + kMidSlideDelaySlackTicks);
                setup.swingDelays.assign(kMidSlideSwingDelays.begin(), kMidSlideSwingDelays.end());
                referenceTick = firstHitTick;
            }
            const std::optional<MeleeChoice> choice = bestMelee(setup);
            INFO("flat state " << flatStateName(state) << " heading " << headingDeg);
            CHECK(choice.has_value());
            if (!choice.has_value())
            {
                std::printf("[SupportLaw.flat.unreachable] law=v2 state=%s dir=%.0f\n", flatStateName(state), double(headingDeg));
                continue;
            }
            sourceMismatches += choice->run.supportSourceMismatches;
            results.push_back(measureFlat(state, headingDeg, *choice, referenceTick));
            if (printRuns)
                printFlatRun(results.back(), choice->run);
        }

    bool anyRun = false;
    for (const FlatState state : { FlatState::Rest, FlatState::Walk, FlatState::MidSlide })
    {
        FlatTotals part;
        bool any = false;
        for (const FlatRun& r : results)
        {
            if (r.state != state)
                continue;
            part.slideMinM = any ? std::min(part.slideMinM, r.slideM) : r.slideM;
            part.slideMaxM = any ? std::max(part.slideMaxM, r.slideM) : r.slideM;
            part.apexMaxM = any ? std::max(part.apexMaxM, r.apexM) : r.apexM;
            any = true;
            ++part.runs;
            part.flips += r.flips;
            part.flickerRuns += r.flips >= 2u ? 1u : 0u;
            part.unsupported += r.unsupported;
            part.slideEndTicks += r.slideEndTicks;
        }
        std::printf("[SupportLaw.flat.state] law=v2 state=%s runs=%u flips=%u flickerRuns=%u unsupported=%u apexMaxM=%.4f slideMinM=%.4f "
                    "slideMaxM=%.4f slideEndTicks=%u\n",
            flatStateName(state), part.runs, part.flips, part.flickerRuns, part.unsupported, double(part.apexMaxM),
            double(part.slideMinM), double(part.slideMaxM), part.slideEndTicks);
        if (!any)
            continue;
        totals.slideMinM = anyRun ? std::min(totals.slideMinM, part.slideMinM) : part.slideMinM;
        totals.slideMaxM = anyRun ? std::max(totals.slideMaxM, part.slideMaxM) : part.slideMaxM;
        totals.apexMaxM = anyRun ? std::max(totals.apexMaxM, part.apexMaxM) : part.apexMaxM;
        anyRun = true;
        totals.runs += part.runs;
        totals.flips += part.flips;
        totals.flickerRuns += part.flickerRuns;
        totals.unsupported += part.unsupported;
        totals.slideEndTicks += part.slideEndTicks;
    }
    for (const FlatRun& r : results)
    {
        totals.apexSumM += r.apexM;
        totals.slideSumM += r.slideM;
    }
    std::printf("[SupportLaw.scorecard] law=v2 rig=flat runs=%u flips=%u apexMaxM=%.4f slideEndTicks=%u\n", totals.runs,
        totals.flips, double(totals.apexMaxM), totals.slideEndTicks);
    CHECK(sourceMismatches == 0u);
    return totals;
}

struct Crossing
{
    const char* name;
    float creaseZ;
    float travelZ;
    bool startsOnSlope;
    bool convex;
};

inline constexpr std::array<Crossing, 4u> kCrossings = { {
    { "flatToSlope", goldenTraceSlopeScenario::kLowerCreaseZ, -1.f, false, false },
    { "slopeToFlat", goldenTraceSlopeScenario::kLowerCreaseZ, 1.f, true, false },
    { "slopeToPlateau", goldenTraceSlopeScenario::kUpperCreaseZ, -1.f, true, true },
    { "plateauToSlope", goldenTraceSlopeScenario::kUpperCreaseZ, 1.f, false, true },
} };

glm::vec3 creaseStart(const Crossing& crossing, float distance)
{
    const float alongZ = crossing.startsOnSlope ? distance * -goldenTraceSlopeScenario::kUpTheFallLine.z : distance;
    return slopeSurface(0.f, crossing.creaseZ - crossing.travelZ * alongZ);
}

glm::vec3 destinationNormal(const Crossing& crossing)
{
    return crossing.startsOnSlope ? brawlerMovementSimulation::kWorldUp : goldenTraceSlopeScenario::kFaceNormal;
}

enum class CreaseKind
{
    Melee,
    Projectile,
    Walk,
};

const char* creaseKindName(CreaseKind kind)
{
    return kind == CreaseKind::Melee ? "melee" : (kind == CreaseKind::Projectile ? "projectile" : "walk");
}

struct CreaseRun
{
    const Crossing* crossing = nullptr;
    CreaseKind kind = CreaseKind::Melee;
    float distance = 0.f;
    float obliqueDeg = 0.f;
    uint32_t startTick = 0u;
    float directionCos = 0.f;
    float knockbackSpeed = 0.f;
    uint32_t windowEndTick = 0u;
    bool crossed = false;
    uint32_t crossTick = 0u;
    uint32_t unsupportedAfter = 0u;
    uint32_t flipsAfter = 0u;
    float apexM = 0.f;
    bool hop = false;
    bool offWorld = false;
    float speedBefore = 0.f;
    float inPlaneAfter = 0.f;
    float vyDepart = 0.f;
    float vSinTheta = 0.f;
    uint32_t slideEndTicks = 0u;
    bool committedCrossing = false;
    uint32_t departTick = 0u;
    float ballisticErrMps = 0.f;
    uint32_t redirectTick = 0u;
    float redirectSpeedIn = 0.f;
    float redirectThetaDeg = 0.f;
    float redirectExpectedMps = 0.f;
    float redirectInPlaneMps = 0.f;
    float minProbeClearanceM = 0.f;
};

struct CreaseTotals
{
    uint32_t runs = 0u;
    uint32_t crossed = 0u;
    uint32_t flips = 0u;
    uint32_t hops = 0u;
    uint32_t unsupported = 0u;
    uint32_t slideEndTicks = 0u;
    uint32_t offWorld = 0u;
    float apexMaxM = 0.f;
    float inPlaneMinMps = 0.f;
    float inPlaneMaxMps = 0.f;
    float convexVyErrMaxMps = 0.f;
    float apexSumM = 0.f;
    float inPlaneSumMps = 0.f;
    float convexVyErrSumMps = 0.f;
};

CreaseRun measureCrease(const Crossing& crossing, CreaseKind kind, float distance, float obliqueDeg, const RigRun& run,
    std::size_t startIndex, std::size_t windowEnd, float directionCos)
{
    CreaseRun r;
    r.crossing = &crossing;
    r.kind = kind;
    r.distance = distance;
    r.obliqueDeg = obliqueDeg;
    r.startTick = run.rows[startIndex].tick;
    r.directionCos = directionCos;
    r.knockbackSpeed = run.rows[startIndex].knockbackSpeed;
    r.windowEndTick = run.rows[std::min(windowEnd, run.rows.size() - 1u)].tick;
    if (kind != CreaseKind::Walk)
        r.slideEndTicks = slideEndTicksIn(run, startIndex, windowEnd);
    std::size_t crossIndex = startIndex;
    while (crossIndex < run.rows.size() && (run.rows[crossIndex].position.z - crossing.creaseZ) * crossing.travelZ < 0.f)
        ++crossIndex;
    if (crossIndex >= run.rows.size())
        return r;
    r.crossed = true;
    r.crossTick = run.rows[crossIndex].tick;
    r.unsupportedAfter = unsupportedIn(run, crossIndex, windowEnd);
    r.flipsAfter = transitionsIn(run, crossIndex, windowEnd);
    r.apexM = apexIn(run, crossIndex, run.rows.size());
    for (std::size_t index = crossIndex; index < run.rows.size(); ++index)
    {
        const TickRow& row = run.rows[index];
        r.offWorld = r.offWorld || !row.blocked;
        r.hop = r.hop || (row.support == 0u && row.blocked && row.clearance - run.rideHeight > kHopApexM);
    }
    const glm::vec3 after = run.rows[crossIndex].velocity;
    const glm::vec3 n = destinationNormal(crossing);
    r.inPlaneAfter = glm::length(after - n * glm::dot(after, n));
    r.speedBefore = crossIndex > 0u ? glm::length(run.rows[crossIndex - 1u].velocity) : 0.f;
    r.vyDepart = after.y;
    r.vSinTheta = r.speedBefore * goldenTraceSlopeScenario::kUpTheFallLine.y;
    r.committedCrossing = kind != CreaseKind::Walk && crossIndex < windowEnd;
    if (!crossing.convex && r.committedCrossing)
    {
        std::size_t redirectIndex = startIndex;
        while (redirectIndex < windowEnd
               && !(run.rows[redirectIndex].support != 0u && glm::dot(run.rows[redirectIndex].faceNormal, n) > kSameFaceCos))
            ++redirectIndex;
        if (redirectIndex < windowEnd)
        {
            const TickRow& row = run.rows[redirectIndex];
            const float intoFace = glm::dot(row.velocityIn, n);
            const float speedIn = glm::length(row.velocityIn);
            const float inPlaneIn = std::sqrt(std::max(speedIn * speedIn - intoFace * intoFace, 0.f));
            const glm::vec3 inPlaneOut = row.velocity - brawlerMovementSimulation::kWorldUp * (glm::dot(row.velocity, n) / n.y);
            r.redirectTick = row.tick;
            r.redirectSpeedIn = speedIn;
            r.redirectThetaDeg = glm::degrees(std::atan2(-intoFace, inPlaneIn));
            r.redirectExpectedMps = row.hit ? inPlaneIn : std::max(inPlaneIn - run.decelStep, 0.f);
            r.redirectInPlaneMps = glm::length(inPlaneOut);
            r.minProbeClearanceM = row.probeClearance;
            for (std::size_t index = redirectIndex; index < windowEnd; ++index)
                r.minProbeClearanceM = std::min(r.minProbeClearanceM, run.rows[index].probeClearance);
        }
    }
    if (crossing.convex && r.committedCrossing)
    {
        std::size_t departIndex = crossIndex;
        while (departIndex < windowEnd && run.rows[departIndex].support != 0u)
            ++departIndex;
        if (departIndex < windowEnd && departIndex > 0u)
        {
            r.departTick = run.rows[departIndex].tick;
            r.ballisticErrMps = run.rows[departIndex].velocity.y - (run.rows[departIndex - 1u].velocity.y + run.gravityStep);
        }
    }
    return r;
}

void printCreaseRun(const CreaseRun& r, const RigRun& run)
{
    std::printf("[SupportLaw.crease.run] law=v2 crossing=%s kind=%s d=%.1f obl=%+.0f start=%u dirCos=%.4f speed=%.2f windowEnd=%u crossed=%d "
                "cross=%u unsupported=%u flips=%u apexM=%.4f hop=%d offWorld=%d speedBefore=%.4f inPlaneAfter=%.4f vyDepart=%.4f "
                "vSinTheta=%.4f slideEndTicks=%u\n",
        r.crossing->name, creaseKindName(r.kind), double(r.distance), double(r.obliqueDeg), r.startTick, double(r.directionCos),
        double(r.knockbackSpeed), r.windowEndTick, r.crossed ? 1 : 0, r.crossTick, r.unsupportedAfter, r.flipsAfter, double(r.apexM),
        r.hop ? 1 : 0, r.offWorld ? 1 : 0, double(r.speedBefore), double(r.inPlaneAfter), double(r.vyDepart), double(r.vSinTheta),
        r.slideEndTicks);
    if (r.crossing->convex && r.committedCrossing)
        std::printf("[SupportLaw.crease.departure] law=v2 crossing=%s kind=%s d=%.1f obl=%+.0f cross=%u depart=%u ballisticErrMps=%.4f\n",
            r.crossing->name, creaseKindName(r.kind), double(r.distance), double(r.obliqueDeg), r.crossTick, r.departTick,
            double(r.ballisticErrMps));
    if (!r.crossing->convex && r.committedCrossing)
        std::printf("[SupportLaw.crease.redirect] law=v2 crossing=%s kind=%s d=%.1f obl=%+.0f cross=%u redirect=%u speedIn=%.4f thetaDeg=%.4f "
                    "expectedMps=%.4f inPlaneMps=%.4f errMps=%.6f minProbeClearanceM=%.5f unsupported=%u\n",
            r.crossing->name, creaseKindName(r.kind), double(r.distance), double(r.obliqueDeg), r.crossTick, r.redirectTick,
            double(r.redirectSpeedIn), double(r.redirectThetaDeg), double(r.redirectExpectedMps), double(r.redirectInPlaneMps),
            double(r.redirectInPlaneMps - r.redirectExpectedMps), double(r.minProbeClearanceM), r.unsupportedAfter);
    const uint32_t last = r.crossed ? std::max(r.windowEndTick, r.crossTick + 60u) : r.windowEndTick;
    for (const TickRow& row : run.rows)
    {
        if (row.tick + 2u < r.startTick || row.tick > last)
            continue;
        std::printf("[SupportLaw.crease.tick] law=v2 crossing=%s kind=%s d=%.1f obl=%+.0f t=%u rel=%d z=%.4f y=%.4f sup=%s clr=%.5f vy=%.5f "
                    "|v|=%.4f flinch=%d hit=%d\n",
            r.crossing->name, creaseKindName(r.kind), double(r.distance), double(r.obliqueDeg), row.tick, int(row.tick) - int(r.startTick),
            double(row.position.z), double(row.position.y), supportName(row.support), double(row.clearance), double(row.velocity.y),
            double(glm::length(row.velocity)), row.hitFlinch ? 1 : 0, row.hit ? 1 : 0);
    }
}

void checkV2Crease(const std::vector<CreaseRun>& results)
{
    const float gravityStep = std::abs(goldenTraceScenario::kGravity) * RigScenario::kDtSeconds;
    uint32_t departures = 0u;
    uint32_t concaveHops = 0u;
    float worstErr = 0.f;
    uint32_t concaveCrossings = 0u;
    uint32_t concaveUnsupported = 0u;
    uint32_t faceAfterTheLockout = 0u;
    float worstRedirectErrOverV = 0.f;
    float minProbeClearance = std::numeric_limits<float>::max();
    uint32_t slopeToPlateauCrossed = 0u;
    for (const CreaseRun& r : results)
    {
        concaveHops += !r.crossing->convex && r.hop ? 1u : 0u;
        slopeToPlateauCrossed += r.crossed && std::string(r.crossing->name) == "slopeToPlateau" ? 1u : 0u;
        if (!r.crossing->convex && r.committedCrossing)
        {
            INFO("concave crossing " << r.crossing->name << " " << creaseKindName(r.kind) << " d " << r.distance << " obl " << r.obliqueDeg
                 << " cross " << r.crossTick << " redirect " << r.redirectTick << " speedIn " << r.redirectSpeedIn << " theta "
                 << r.redirectThetaDeg << " expected " << r.redirectExpectedMps << " inPlane " << r.redirectInPlaneMps);
            CHECK(r.unsupportedAfter == 0u);
            concaveUnsupported += r.unsupportedAfter;
            if (r.redirectTick == 0u)
            {
                ++faceAfterTheLockout;
                continue;
            }
            CHECK(std::abs(r.redirectInPlaneMps - r.redirectExpectedMps) <= kRedirectSpeedTolerance * r.redirectSpeedIn);
            ++concaveCrossings;
            if (r.redirectSpeedIn > 0.f)
                worstRedirectErrOverV = std::max(worstRedirectErrOverV, std::abs(r.redirectInPlaneMps - r.redirectExpectedMps) / r.redirectSpeedIn);
            minProbeClearance = std::min(minProbeClearance, r.minProbeClearanceM);
        }
        if (!r.crossing->convex || !r.committedCrossing)
            continue;
        INFO("crossing " << r.crossing->name << " " << creaseKindName(r.kind) << " d " << r.distance << " obl " << r.obliqueDeg
             << " cross " << r.crossTick << " depart " << r.departTick << " ballisticErr " << r.ballisticErrMps);
        CHECK(r.departTick != 0u);
        CHECK(r.departTick <= r.crossTick + 1u);
        CHECK(std::abs(r.ballisticErrMps) <= gravityStep);
        worstErr = std::max(worstErr, std::abs(r.ballisticErrMps));
        ++departures;
    }
    std::printf("[SupportLaw.crease.departures] law=v2 convexDepartures=%u worstBallisticErrMps=%.4f gravityStepMps=%.4f concaveHops=%u\n",
        departures, double(worstErr), double(gravityStep), concaveHops);
    CHECK(departures == 22u);
    CHECK(concaveHops == 0u);
    std::printf("[SupportLaw.crease.redirects] law=v2 concaveCrossings=%u faceAfterTheLockout=%u unsupported=%u worstErrOverV=%.3g "
                "minProbeClearanceM=%.5f\n",
        concaveCrossings, faceAfterTheLockout, concaveUnsupported, double(worstRedirectErrOverV), double(minProbeClearance));
    CHECK(concaveCrossings == 22u);
    CHECK(faceAfterTheLockout == 2u);
    checkPin("crease", "slopeToPlateau.crossed", slopeToPlateauCrossed, 14u);
}

struct ProjectileChoice
{
    RigRun run;
    std::size_t hitIndex = 0u;
    float directionCos = 0.f;
};

std::optional<ProjectileChoice> projectileOnFlinch(const glm::vec3& targetAt, const glm::vec3& heading)
{
    const RigActor target = actorAt(kTarget, targetAt, heading);
    const glm::vec3 shooterFace = slopeSurface(targetAt.x - heading.x * kShooterBehindM, targetAt.z - heading.z * kShooterBehindM);
    const RigActor shooter = actorAt(kShooter, shooterFace, heading);
    for (const float sign : { 1.f, -1.f })
    {
        const glm::vec3 offset = lateralOf(heading) * (kStunnerRadius * sign);
        const glm::vec3 stunnerFace = slopeSurface(targetAt.x + offset.x, targetAt.z + offset.z);
        const glm::vec3 stunnerAim = horizontalUnit(targetAt - stunnerFace);
        const std::vector<RigActor> actors = { target, actorAt(kStunner, stunnerFace, stunnerAim), shooter };
        const InputRow overhead = goldenTraceSlopeScenario::swing(kStunner, kSwingTick, stunnerAim, Swing::Overhead);

        const RigRun stunDry = runRig(World::Slope, actors, { overhead });
        const std::optional<std::size_t> stunIndex = firstHitIndex(stunDry, kSwingTick, HitReactionKind::Stun);
        if (!stunIndex.has_value())
            continue;
        const RigRun shotDry = runRig(World::Slope, actors, { goldenTraceSlopeScenario::shoot(kShooter, kSwingTick, heading) });
        const std::optional<std::size_t> shotIndex = firstHitIndex(shotDry, kSwingTick, std::nullopt);
        if (!shotIndex.has_value())
            continue;
        const uint32_t stunTick = stunDry.rows[*stunIndex].tick;
        const uint32_t flight = shotDry.rows[*shotIndex].tick - kSwingTick;
        const uint32_t fireTick = stunTick + kShotDelayTicks - flight;

        ProjectileChoice choice;
        choice.run = runRig(World::Slope, actors, { overhead, goldenTraceSlopeScenario::shoot(kShooter, fireTick, heading) });
        const std::optional<std::size_t> hitIndex = firstHitIndex(choice.run, stunTick + 1u, HitReactionKind::Knockback);
        if (!hitIndex.has_value())
            continue;
        choice.hitIndex = *hitIndex;
        choice.directionCos = glm::dot(glm::normalize(choice.run.rows[*hitIndex].hitDirection), inPlane(heading, slopeNormal(targetAt.z)));
        return choice;
    }
    return std::nullopt;
}

CreaseTotals runCreaseRig()
{
    const bool printRuns = printsRuns();
    CreaseTotals totals;
    std::vector<CreaseRun> results;
    uint32_t sourceMismatches = 0u;
    for (const Crossing& crossing : kCrossings)
    {
        const glm::vec3 fallLine(0.f, 0.f, crossing.travelZ);
        for (const float distance : kCreaseDistances)
            for (const float oblique : kCreaseObliqueDeg)
            {
                MeleeSetup setup;
                setup.world = World::Slope;
                setup.targetAt = creaseStart(crossing, distance);
                setup.heading = headingRotated(fallLine, oblique);
                setup.actors = { actorAt(kTarget, setup.targetAt, setup.heading) };
                const std::optional<MeleeChoice> choice = bestMelee(setup);
                INFO("crossing " << crossing.name << " distance " << distance << " oblique " << oblique);
                CHECK(choice.has_value());
                if (!choice.has_value())
                {
                    std::printf("[SupportLaw.crease.unreachable] law=v2 crossing=%s kind=melee d=%.1f obl=%+.0f\n", crossing.name, double(distance),
                        double(oblique));
                    continue;
                }
                sourceMismatches += choice->run.supportSourceMismatches;
                results.push_back(measureCrease(crossing, CreaseKind::Melee, distance, oblique, choice->run, choice->hitIndex,
                    lockoutEndIndex(choice->run, choice->hitIndex), choice->directionCos));
                if (printRuns)
                    printCreaseRun(results.back(), choice->run);
            }

        const glm::vec3 shotAt = creaseStart(crossing, kProjectileDistance);
        const std::optional<ProjectileChoice> shot = projectileOnFlinch(shotAt, fallLine);
        {
            INFO("crossing " << crossing.name << " projectile on flinch");
            CHECK(shot.has_value());
        }
        if (shot.has_value())
        {
            sourceMismatches += shot->run.supportSourceMismatches;
            results.push_back(measureCrease(crossing, CreaseKind::Projectile, kProjectileDistance, 0.f, shot->run, shot->hitIndex,
                lockoutEndIndex(shot->run, shot->hitIndex), shot->directionCos));
            if (printRuns)
                printCreaseRun(results.back(), shot->run);
        }
        else
        {
            std::printf("[SupportLaw.crease.unreachable] law=v2 crossing=%s kind=projectile d=%.1f obl=+0\n", crossing.name, double(kProjectileDistance));
        }

        const glm::vec3 walkAt = creaseStart(crossing, kCreaseWalkStartM);
        const RigRun walk = runRig(World::Slope, { actorAt(kTarget, walkAt, fallLine) },
            { goldenTraceSlopeScenario::walk(kTarget, kSwingTick, kSwingTick + kCreaseWalkTicks - 1u, fallLine) });
        sourceMismatches += walk.supportSourceMismatches;
        results.push_back(measureCrease(crossing, CreaseKind::Walk, kCreaseWalkStartM, 0.f, walk, indexOfTick(walk, kSwingTick),
            indexOfTick(walk, kSwingTick + kCreaseWalkTicks), 1.f));
        if (printRuns)
            printCreaseRun(results.back(), walk);
    }

    bool anyRun = false;
    for (const Crossing& crossing : kCrossings)
    {
        CreaseTotals part;
        bool anyInPlane = false;
        float worstDepartureErr = 0.f;
        for (const CreaseRun& r : results)
        {
            if (r.crossing != &crossing)
                continue;
            ++part.runs;
            part.slideEndTicks += r.slideEndTicks;
            if (!r.crossed)
                continue;
            ++part.crossed;
            part.flips += r.flipsAfter;
            part.hops += r.hop ? 1u : 0u;
            part.unsupported += r.unsupportedAfter;
            part.offWorld += r.offWorld ? 1u : 0u;
            part.apexMaxM = anyInPlane ? std::max(part.apexMaxM, r.apexM) : r.apexM;
            part.inPlaneMinMps = anyInPlane ? std::min(part.inPlaneMinMps, r.inPlaneAfter) : r.inPlaneAfter;
            part.inPlaneMaxMps = anyInPlane ? std::max(part.inPlaneMaxMps, r.inPlaneAfter) : r.inPlaneAfter;
            anyInPlane = true;
            totals.apexSumM += r.apexM;
            totals.inPlaneSumMps += r.inPlaneAfter;
            if (crossing.convex)
            {
                const float departureErr = std::abs(r.vyDepart - crossing.travelZ * -1.f * r.vSinTheta);
                worstDepartureErr = std::max(worstDepartureErr, departureErr);
                totals.convexVyErrSumMps += departureErr;
            }
        }
        totals.convexVyErrMaxMps = std::max(totals.convexVyErrMaxMps, worstDepartureErr);
        std::printf("[SupportLaw.crease.crossing] law=v2 crossing=%s convex=%d runs=%u crossed=%u hops=%u unsupported=%u flips=%u apexMaxM=%.4f "
                    "offWorld=%u inPlaneMinMps=%.4f inPlaneMaxMps=%.4f convexVyErrMaxMps=%.4f slideEndTicks=%u\n",
            crossing.name, crossing.convex ? 1 : 0, part.runs, part.crossed, part.hops, part.unsupported, part.flips,
            double(part.apexMaxM), part.offWorld, double(part.inPlaneMinMps), double(part.inPlaneMaxMps), double(worstDepartureErr),
            part.slideEndTicks);
        totals.runs += part.runs;
        totals.crossed += part.crossed;
        totals.flips += part.flips;
        totals.hops += part.hops;
        totals.unsupported += part.unsupported;
        totals.slideEndTicks += part.slideEndTicks;
        totals.offWorld += part.offWorld;
        if (!anyInPlane)
            continue;
        totals.apexMaxM = anyRun ? std::max(totals.apexMaxM, part.apexMaxM) : part.apexMaxM;
        totals.inPlaneMinMps = anyRun ? std::min(totals.inPlaneMinMps, part.inPlaneMinMps) : part.inPlaneMinMps;
        totals.inPlaneMaxMps = anyRun ? std::max(totals.inPlaneMaxMps, part.inPlaneMaxMps) : part.inPlaneMaxMps;
        anyRun = true;
    }
    std::printf("[SupportLaw.scorecard] law=v2 rig=crease runs=%u flips=%u hops=%u apexMaxM=%.4f slideEndTicks=%u\n", totals.runs,
        totals.flips, totals.hops, double(totals.apexMaxM), totals.slideEndTicks);
    CHECK(sourceMismatches == 0u);
    checkV2Crease(results);
    return totals;
}

void flatKnockbacks()
{
    const FlatTotals totals = runFlatRig();
    CHECK(totals.runs == 3u * kFlatDirections);
    if (printsRuns())
        std::printf("[SupportLaw.pin] law=v2 rig=flat flips=%u flickerRuns=%u unsupported=%u slideEndTicks=%u apexMaxM=%.4f apexSumM=%.4f slideMinM=%.4f "
                    "slideMaxM=%.4f slideSumM=%.4f\n",
            totals.flips, totals.flickerRuns, totals.unsupported, totals.slideEndTicks, double(totals.apexMaxM), double(totals.apexSumM),
            double(totals.slideMinM), double(totals.slideMaxM), double(totals.slideSumM));
    checkPin("flat", "flips", totals.flips, 0u);
    checkPin("flat", "flickerRuns", totals.flickerRuns, 0u);
    checkPin("flat", "unsupported", totals.unsupported, 0u);
    checkPin("flat", "slideEndTicks", totals.slideEndTicks, 0u);
}

void creaseCrossings()
{
    const CreaseTotals totals = runCreaseRig();
    CHECK(totals.runs == uint32_t(kCrossings.size() * (kCreaseDistances.size() * kCreaseObliqueDeg.size() + 2u)));
    if (printsRuns())
        std::printf("[SupportLaw.pin] law=v2 rig=crease crossed=%u flips=%u hops=%u unsupported=%u slideEndTicks=%u offWorld=%u apexMaxM=%.4f apexSumM=%.4f "
                    "inPlaneMinMps=%.4f inPlaneMaxMps=%.4f inPlaneSumMps=%.4f convexVyErrMaxMps=%.4f convexVyErrSumMps=%.4f\n",
            totals.crossed, totals.flips, totals.hops, totals.unsupported, totals.slideEndTicks, totals.offWorld, double(totals.apexMaxM),
            double(totals.apexSumM), double(totals.inPlaneMinMps), double(totals.inPlaneMaxMps), double(totals.inPlaneSumMps),
            double(totals.convexVyErrMaxMps), double(totals.convexVyErrSumMps));
}

TEST_CASE("SupportLawScorecard.FlatKnockbacksUnderV2", "[GoldenTrace]")
{
    flatKnockbacks();
}

TEST_CASE("SupportLawScorecard.CreaseCrossingsUnderV2", "[GoldenTrace]")
{
    creaseCrossings();
}

} // namespace supportLawScorecardTests

#endif
