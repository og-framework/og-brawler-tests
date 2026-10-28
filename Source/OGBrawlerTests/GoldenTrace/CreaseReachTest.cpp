// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceScenario.h"
#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"
#include "../GroundAlignedMachineRig.h"

#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat4x4.hpp"
#include "glm/trigonometric.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace creaseReachTests
{

using namespace goldenTraceSlopeScenario;

enum class Crease : uint32_t
{
    Concave,
    Convex,
};

enum class Side : uint32_t
{
    Slope,
    Flat,
};

enum class Mode : uint32_t
{
    Today,
    Flip,
};

inline constexpr uint32_t kDistanceSteps = 20u;
inline constexpr float kDistanceStepM = 0.05f;
inline constexpr uint32_t kSweepPlacements = 2u * 2u * kDistanceSteps;
inline constexpr uint32_t kColumns = 8u;
inline constexpr uint32_t kSweepBatches = kSweepPlacements / kColumns;
inline constexpr uint32_t kScenarioBatch = kSweepBatches;
inline constexpr uint32_t kSettleTicks = 60u;
inline constexpr float kSlopeSideX = -1.f;
inline constexpr float kFlatSideX = 1.f;
inline constexpr float kStillPerTickM = 0.001f;
inline const float kDesignLaunchSine = std::sin(glm::radians(2.f));
inline constexpr uint32_t kScenarioConcaveToday = 2u;
inline constexpr uint32_t kScenarioConcaveFlip = 4u;
inline constexpr uint32_t kScenarioConvexToday = 4u;
inline constexpr uint32_t kScenarioConvexFlip = 4u;

static_assert(kSweepPlacements % kColumns == 0u, "the sweep fills whole scenes of kColumns characters");
static_assert(kColumns <= kMaxSimulatableSlots, "one settle scene holds at most the world's body slots");

constexpr float distanceAt(uint32_t step) { return kDistanceStepM * float(step + 1u); }

constexpr glm::vec3 facePointAt(Crease crease, Side side, float distance, float x)
{
    if (crease == Crease::Concave)
    {
        if (side == Side::Slope)
            return glm::vec3(x, kLowerCreaseHeight + distance * kUpTheFallLine.y, kLowerCreaseZ + distance * kUpTheFallLine.z);
        return glm::vec3(x, kLowerCreaseHeight, kLowerCreaseZ + distance);
    }
    if (side == Side::Slope)
        return glm::vec3(x, kUpperCreaseHeight - distance * kUpTheFallLine.y, kUpperCreaseZ - distance * kUpTheFallLine.z);
    return glm::vec3(x, kUpperCreaseHeight, kUpperCreaseZ - distance);
}

struct Placement
{
    Crease crease = Crease::Concave;
    Side side = Side::Slope;
    float distance = 0.f;
    glm::vec3 facePoint = glm::vec3(0.f);
};

constexpr float columnX(uint32_t column) { return -5.25f + 1.5f * float(column); }

constexpr Placement sweepPlacement(uint32_t index)
{
    const Crease crease = index / (2u * kDistanceSteps) == 0u ? Crease::Concave : Crease::Convex;
    const Side side = (index / kDistanceSteps) % 2u == 0u ? Side::Slope : Side::Flat;
    const float distance = distanceAt(index % kDistanceSteps);
    return Placement{ crease, side, distance, facePointAt(crease, side, distance, columnX(index % kColumns)) };
}

inline constexpr std::array<Placement, 4u> kScenarioPlacements = {
    Placement{ Crease::Concave, Side::Slope, 0.f, kLowerF },
    Placement{ Crease::Concave, Side::Flat, 0.f, kLowerG },
    Placement{ Crease::Convex, Side::Slope, 0.f, kUpperF },
    Placement{ Crease::Convex, Side::Flat, 0.f, kUpperG },
};

constexpr uint32_t charactersIn(uint32_t batch) { return batch == kScenarioBatch ? uint32_t(kScenarioPlacements.size()) : kColumns; }

constexpr Placement placementOf(uint32_t batch, uint32_t column)
{
    if (batch == kScenarioBatch)
        return kScenarioPlacements[column];
    return sweepPlacement(batch * kColumns + column);
}

template <uint32_t Batch>
struct SettleScenario
{
    static constexpr const char* kGenerator = "CreaseReach.settle";
    static constexpr float kDtSeconds = goldenTraceSlopeScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = kSettleTicks;

    static constexpr std::array<goldenTraceScene::Character, kColumns> makeCharacters()
    {
        std::array<goldenTraceScene::Character, kColumns> characters{};
        for (uint32_t column = 0u; column < charactersIn(Batch); ++column)
            characters[column] = goldenTraceScene::Character{ column + 1u, column, "",
                raised(placementOf(Batch, column).facePoint, kTeleportHeight) };
        return characters;
    }

    static constexpr std::array<goldenTraceScene::SpawnPoint, kColumns> makeSpawnTable()
    {
        std::array<goldenTraceScene::SpawnPoint, kColumns> spawns{};
        for (uint32_t column = 0u; column < charactersIn(Batch); ++column)
            spawns[column] = goldenTraceScene::SpawnPoint{ column, raised(placementOf(Batch, column).facePoint, kSpawnHeight) };
        return spawns;
    }

    static constexpr std::array<goldenTraceScene::Character, kColumns> kCharacterTable = makeCharacters();
    static constexpr std::array<goldenTraceScene::SpawnPoint, kColumns> kSpawnTable = makeSpawnTable();

    static bool isSampleTick(uint32_t) { return false; }
    static StaticWorldDescription statics() { return slopeStatics(); }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return slopeStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int, uint32_t)
    {
        simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
        input.aimDirection = kTowardPlusX;
        return input;
    }
    static std::span<const goldenTraceScene::Character> characters()
    {
        return std::span<const goldenTraceScene::Character>(kCharacterTable.data(), charactersIn(Batch));
    }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable()
    {
        return std::span<const goldenTraceScene::SpawnPoint>(kSpawnTable.data(), charactersIn(Batch));
    }
    static std::span<const goldenTraceScene::Teleport> teleports() { return {}; }
};

static_assert(goldenTraceScene::Scenario<SettleScenario<0u>>, "the settle scenario provides everything goldenTraceScene::Scene reads");

struct Rest
{
    Placement placement;
    glm::vec3 capsuleOffset = glm::vec3(0.f);
    brawlerGround::GroundSample ground;
    glm::vec3 movementFace = glm::vec3(0.f);
    bool supported = false;
    float lastTickTravelM = 0.f;
};

template <uint32_t Batch>
void settleBatch(std::vector<Rest>& rests)
{
    goldenTraceScene::Scene<SettleScenario<Batch>> scene(std::nullopt, 1.f);
    std::array<glm::vec3, kColumns> previous{};
    for (uint32_t tick = 1u; tick <= kSettleTicks; ++tick)
    {
        if (tick == kSettleTicks)
            for (const goldenTraceScene::Character& character : SettleScenario<Batch>::characters())
                previous[character.slot] = scene.storage.template get<SimulatableBrawler>(character.id)
                                               .getAllState().getState().template get<brawlerMovementSimulation::State>()
                                               .bodyState.position;
        scene.step(tick);
    }
    for (const goldenTraceScene::Character& character : SettleScenario<Batch>::characters())
    {
        const SimulatableBrawler& brawler = scene.storage.template get<SimulatableBrawler>(character.id);
        const auto& movement = brawler.getAllState().getState().template get<brawlerMovementSimulation::State>();
        const auto& bindings = brawler.getPhysicsComposite().template get<brawlerMovementSimulation::PhysicsDeclaration>().bindings;
        const Placement placement = placementOf(Batch, character.slot);
        const glm::vec3 position = movement.bodyState.position;
        const glm::vec3 step = position - previous[character.slot];
        const auto support = static_cast<brawlerMovementSimulation::SupportState>(
            (movement.flags & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift);
        const brawlerMovementSimulation::GroundProbe probe =
            brawlerMovementSimulation::probeGround(movement, scene.staticData.m_movementStaticData, bindings, scene.query);
        rests.push_back(Rest{ placement, position - placement.facePoint,
            brawlerMovementSimulation::sampleAttackGround<true>(movement, scene.staticData.m_movementStaticData, bindings, scene.query),
            probe.walkable ? probe.hit.normal : brawlerMovementSimulation::kWorldUp,
            support == brawlerMovementSimulation::SupportState::Supported, glm::length(glm::vec3(step.x, 0.f, step.z)) });
    }
}

template <uint32_t... Batches>
std::vector<Rest> settleAll(std::integer_sequence<uint32_t, Batches...>)
{
    std::vector<Rest> rests;
    (settleBatch<Batches>(rests), ...);
    return rests;
}

struct Reach
{
    bool started = false;
    bool connects = false;
    bool segmentLastsATick = true;
    bool leavesTargetGround = false;
    float tangentDotTargetGround = 0.f;
    float alongAxisM = 0.f;
    float onPlaneM = 0.f;
};

dAttackMachineSimulation::PlayerInputView swingInput(const glm::vec3& aim, Swing side)
{
    const goldenTraceScenario::InputRow row = swing(1u, 1u, aim, side);
    simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
    input.aimDirection = row.aim;
    input.attackLeft = row.attackLeft;
    input.attackRight = row.attackRight;
    input.moveStick = glm::vec2(row.stickX, row.stickY);
    input.moveDirectionWorld = row.move;
    input.triggeredActionId = row.actionId;
    return dAttackMachineSimulation::PlayerInputView::from(input);
}

Reach reachOf(const simulatableBrawler::StaticData& staticData, const glm::vec3& attacker, const glm::vec3& target,
    const glm::vec3& aim, Swing side, const brawlerGround::GroundSample& ground, const glm::vec3& targetMovementFace)
{
    const groundAlignedMachineRig::MachineTickResult tick = groundAlignedMachineRig::integrateOnce(swingInput(aim, side), attacker, ground);
    const dAttackRadialSimulation::InitialConditions& initialConditions = tick.radialIC;
    const unsigned int expectedSequence = side == Swing::Right ? dAttackDirection::kRightSequenceId : dAttackDirection::kLeftSequenceId;
    Reach reach;
    reach.started = tick.machine.m_currentState == DAttackState::Attacking
        && initialConditions.activeAttackSequence == expectedSequence;
    if (!reach.started)
        return reach;

    const dAttackRadialSimulation::StaticData& radial = staticData.m_attackSimulationStaticData;
    const glm::vec3 rootTranslation = attacker + dAttackRadialSimulation::PhysicsDeclaration::attachmentOffset(radial);
    const auto& activeAttackSequence = radial.getAttackSequences()[initialConditions.activeAttackSequence];
    const glm::mat4 initialRotation = glm::rotate(glm::mat4(1.f), initialConditions.initialAimAngle, initialConditions.initialAimRotationAxis);
    const glm::vec3 worldSequenceRotationAxis = glm::vec3(initialRotation * glm::vec4(activeAttackSequence.getRotationAxis(), 0.f));

    const glm::vec3 hitDirection = target - rootTranslation;
    const float signedDistanceAlongRotationAxis = glm::dot(hitDirection, worldSequenceRotationAxis);
    const float lengthAlongRotationAxis = glm::abs(signedDistanceAlongRotationAxis);
    const glm::vec3 hitDirectionOnRotationPlane = hitDirection - signedDistanceAlongRotationAxis * worldSequenceRotationAxis;
    const float hitDistance = glm::length(hitDirectionOnRotationPlane);
    const bool hitIsInCircle = hitDistance > radial.getAttackCircle().getInnerRadius()
        && hitDistance < radial.getAttackCircle().getOuterRadius()
        && lengthAlongRotationAxis < radial.getAttackCircle().getHalfThickness();
    reach.alongAxisM = signedDistanceAlongRotationAxis;
    reach.onPlaneM = hitDistance;
    if (!hitIsInCircle)
        return reach;
    const glm::vec3 normalizedHitDirection = glm::normalize(hitDirectionOnRotationPlane);
    const DAttackSegment hitAttackSegment = dAttackRadialSimulation::getAttackSegment(initialConditions, radial, normalizedHitDirection);
    reach.connects = hitAttackSegment.state == DAttackRadialSequenceState::Damaging;
    if (!reach.connects)
        return reach;

    const float segmentStart = activeAttackSequence.getAttackPointTime(hitAttackSegment.index);
    const float segmentEnd = activeAttackSequence.getAttackPointTime(hitAttackSegment.index + 1u);
    reach.segmentLastsATick = segmentEnd - segmentStart >= kDtSeconds;
    const float authoredAngularVelocity = activeAttackSequence.getAngularVelocity(0.5f * (segmentStart + segmentEnd));
    const glm::vec3 tangent = glm::cross(worldSequenceRotationAxis, normalizedHitDirection);
    const glm::vec3 knockbackDirection = glm::normalize(authoredAngularVelocity < 0.f ? -tangent : tangent);
    reach.tangentDotTargetGround = glm::dot(knockbackDirection, targetMovementFace);
    reach.leavesTargetGround = reach.tangentDotTargetGround > kDesignLaunchSine;
    return reach;
}

struct Pair
{
    glm::vec3 slopeCapsule;
    glm::vec3 flatCapsule;
    glm::vec3 slopeFace;
    glm::vec3 flatFace;
    brawlerGround::GroundSample slopeGround;
    brawlerGround::GroundSample flatGround;
    glm::vec3 slopeMovementFace;
    glm::vec3 flatMovementFace;
};

enum class Role : uint32_t
{
    SlopeAttacker,
    FlatAttacker,
};

struct Sense
{
    const char* name;
    Swing slopeAttackerSide;
    Swing flatAttackerSide;
};

inline constexpr std::array<Sense, 2u> kSenses = {
    Sense{ "up", Swing::Left, Swing::Right },
    Sense{ "down", Swing::Right, Swing::Left },
};

glm::vec3 horizontalAim(const glm::vec3& from, const glm::vec3& to) { return glm::normalize(glm::vec3(to.x - from.x, 0.f, to.z - from.z)); }

Reach pairReach(const simulatableBrawler::StaticData& staticData, const Pair& pair, Role role, const Sense& sense, Mode mode,
    std::optional<glm::vec3> aimOverride = std::nullopt)
{
    const bool slopeAttacks = role == Role::SlopeAttacker;
    const glm::vec3 attacker = slopeAttacks ? pair.slopeCapsule : pair.flatCapsule;
    const glm::vec3 target = slopeAttacks ? pair.flatCapsule : pair.slopeCapsule;
    const glm::vec3 aim = aimOverride.value_or(slopeAttacks ? horizontalAim(pair.slopeFace, pair.flatFace)
                                                            : horizontalAim(pair.flatFace, pair.slopeFace));
    const brawlerGround::GroundSample ground = mode == Mode::Today ? brawlerGround::GroundSample{}
                                                                    : (slopeAttacks ? pair.slopeGround : pair.flatGround);
    return reachOf(staticData, attacker, target, aim, slopeAttacks ? sense.slopeAttackerSide : sense.flatAttackerSide, ground,
        slopeAttacks ? pair.flatMovementFace : pair.slopeMovementFace);
}

const Rest& restAt(const std::vector<Rest>& rests, Crease crease, Side side, uint32_t step)
{
    const uint32_t index = uint32_t(crease) * 2u * kDistanceSteps + uint32_t(side) * kDistanceSteps + step;
    return rests[index];
}

Pair sweepPair(const std::vector<Rest>& rests, Crease crease, uint32_t slopeStep, uint32_t flatStep)
{
    const Rest& slope = restAt(rests, crease, Side::Slope, slopeStep);
    const Rest& flat = restAt(rests, crease, Side::Flat, flatStep);
    const glm::vec3 slopeFace = facePointAt(crease, Side::Slope, slope.placement.distance, kSlopeSideX);
    const glm::vec3 flatFace = facePointAt(crease, Side::Flat, flat.placement.distance, kFlatSideX);
    return Pair{ slopeFace + slope.capsuleOffset, flatFace + flat.capsuleOffset, slopeFace, flatFace, slope.ground, flat.ground,
        slope.movementFace, flat.movementFace };
}

const char* creaseName(Crease crease) { return crease == Crease::Concave ? "concave" : "convex"; }
const char* roleName(Role role) { return role == Role::SlopeAttacker ? "slopeAttacker" : "flatAttacker"; }
const char* modeName(Mode mode) { return mode == Mode::Today ? "today" : "flip"; }

std::string windowsOf(const std::array<std::array<bool, kDistanceSteps>, kDistanceSteps>& grid, uint32_t& count)
{
    std::string text;
    count = 0u;
    char buffer[64];
    for (uint32_t slopeStep = 0u; slopeStep < kDistanceSteps; ++slopeStep)
    {
        std::string row;
        uint32_t flatStep = 0u;
        while (flatStep < kDistanceSteps)
        {
            if (!grid[slopeStep][flatStep])
            {
                ++flatStep;
                continue;
            }
            const uint32_t first = flatStep;
            while (flatStep < kDistanceSteps && grid[slopeStep][flatStep])
                ++flatStep;
            count += flatStep - first;
            std::snprintf(buffer, sizeof(buffer), "%s%.2f-%.2f", row.empty() ? "" : ",", double(distanceAt(first)),
                double(distanceAt(flatStep - 1u)));
            row += buffer;
        }
        if (!row.empty())
        {
            std::snprintf(buffer, sizeof(buffer), " s%.2f:f", double(distanceAt(slopeStep)));
            text += buffer + row;
        }
    }
    return text.empty() ? std::string(" none") : text;
}

using Grid = std::array<std::array<bool, kDistanceSteps>, kDistanceSteps>;

struct SwingGrids
{
    std::array<Grid, 2u> connects{};
    Grid flipLaunches{};
};

using CreaseGrids = std::array<std::array<SwingGrids, kSenses.size()>, 2u>;

TEST_CASE("BrawlerJoltSlopeCreaseReach.PrintsTheConnectingWindows", "[GoldenTrace]")
{
    const std::vector<Rest> rests = settleAll(std::make_integer_sequence<uint32_t, kSweepBatches + 1u>{});
    REQUIRE(rests.size() == kSweepPlacements + kScenarioPlacements.size());
    for (const Rest& rest : rests)
    {
        INFO("crease " << creaseName(rest.placement.crease) << " side " << uint32_t(rest.placement.side) << " distance "
                       << rest.placement.distance << " travel " << rest.lastTickTravelM);
        REQUIRE(rest.supported);
        REQUIRE(rest.lastTickTravelM <= kStillPerTickM);
    }

    const simulatableBrawler::StaticData staticData = slopeStaticData(1.f);

    for (const Crease crease : { Crease::Concave, Crease::Convex })
        for (const Side side : { Side::Slope, Side::Flat })
        {
            std::string heights;
            char buffer[96];
            for (uint32_t step = 0u; step < kDistanceSteps; ++step)
            {
                const Rest& rest = restAt(rests, crease, side, step);
                std::snprintf(buffer, sizeof(buffer), " %.2f:%.4f/%.4f/%.4f", double(rest.placement.distance), double(rest.capsuleOffset.y),
                    double(glm::dot(rest.ground.normal, kFaceNormal)), double(glm::dot(rest.movementFace, kFaceNormal)));
                heights += buffer;
            }
            std::printf("[GoldenTrace.creaseRest] crease=%s side=%s distance:capsuleAboveFaceM/attackSampleDotFace/movementFaceDotFace%s\n",
                creaseName(crease), side == Side::Slope ? "slope" : "flat", heights.c_str());
        }

    std::array<CreaseGrids, 2u> grids{};
    for (const Crease crease : { Crease::Concave, Crease::Convex })
        for (uint32_t slopeStep = 0u; slopeStep < kDistanceSteps; ++slopeStep)
            for (uint32_t flatStep = 0u; flatStep < kDistanceSteps; ++flatStep)
            {
                const Pair pair = sweepPair(rests, crease, slopeStep, flatStep);
                for (const Role role : { Role::SlopeAttacker, Role::FlatAttacker })
                    for (std::size_t sense = 0u; sense < kSenses.size(); ++sense)
                        for (const Mode mode : { Mode::Today, Mode::Flip })
                        {
                            const Reach reach = pairReach(staticData, pair, role, kSenses[sense], mode);
                            REQUIRE(reach.started);
                            REQUIRE(reach.segmentLastsATick);
                            SwingGrids& swing = grids[uint32_t(crease)][uint32_t(role)][sense];
                            swing.connects[uint32_t(mode)][slopeStep][flatStep] = reach.connects;
                            if (mode == Mode::Flip)
                                swing.flipLaunches[slopeStep][flatStep] = reach.connects && reach.leavesTargetGround;
                        }
            }

    for (const Crease crease : { Crease::Concave, Crease::Convex })
        for (const Role role : { Role::SlopeAttacker, Role::FlatAttacker })
            for (std::size_t sense = 0u; sense < kSenses.size(); ++sense)
                for (const Mode mode : { Mode::Today, Mode::Flip })
                {
                    const SwingGrids& swing = grids[uint32_t(crease)][uint32_t(role)][sense];
                    uint32_t count = 0u;
                    const std::string windows = windowsOf(swing.connects[uint32_t(mode)], count);
                    std::string launches;
                    if (mode == Mode::Flip)
                    {
                        uint32_t launching = 0u;
                        const std::string launchWindows = windowsOf(swing.flipLaunches, launching);
                        launches = " launching=" + std::to_string(launching) + " launchWindows=" + launchWindows;
                    }
                    std::printf("[GoldenTrace.creaseReach] crease=%s role=%s sense=%s mode=%s connecting=%u/%u windows(slope:flat)=%s%s\n",
                        creaseName(crease), roleName(role), kSenses[sense].name, modeName(mode), count,
                        kDistanceSteps * kDistanceSteps, windows.c_str(), launches.c_str());
                }

    for (const Crease crease : { Crease::Concave, Crease::Convex })
    {
        std::array<std::array<uint32_t, kDistanceSteps>, kDistanceSteps> both{};
        std::array<std::array<uint32_t, kDistanceSteps>, kDistanceSteps> launching{};
        for (uint32_t slopeStep = 0u; slopeStep < kDistanceSteps; ++slopeStep)
            for (uint32_t flatStep = 0u; flatStep < kDistanceSteps; ++flatStep)
                for (const auto& role : grids[uint32_t(crease)])
                    for (const SwingGrids& swing : role)
                    {
                        const bool inBoth = swing.connects[0][slopeStep][flatStep] && swing.connects[1][slopeStep][flatStep];
                        both[slopeStep][flatStep] += inBoth ? 1u : 0u;
                        launching[slopeStep][flatStep] += inBoth && swing.flipLaunches[slopeStep][flatStep] ? 1u : 0u;
                    }
        for (uint32_t level = 4u; level >= 1u; --level)
        {
            uint32_t bestLaunching = 0u;
            bool any = false;
            for (uint32_t slopeStep = 0u; slopeStep < kDistanceSteps; ++slopeStep)
                for (uint32_t flatStep = 0u; flatStep < kDistanceSteps; ++flatStep)
                    if (both[slopeStep][flatStep] == level)
                    {
                        any = true;
                        bestLaunching = std::max(bestLaunching, launching[slopeStep][flatStep]);
                    }
            if (!any)
                continue;
            Grid at{};
            for (uint32_t slopeStep = 0u; slopeStep < kDistanceSteps; ++slopeStep)
                for (uint32_t flatStep = 0u; flatStep < kDistanceSteps; ++flatStep)
                    at[slopeStep][flatStep] = both[slopeStep][flatStep] == level && launching[slopeStep][flatStep] == bestLaunching;
            uint32_t count = 0u;
            const std::string windows = windowsOf(at, count);
            std::printf("[GoldenTrace.creaseBest] crease=%s swingsConnectingInBothModes=%u/4 mostOfThemLaunchingAfterTheFlip=%u placements=%u "
                        "windows(slope:flat)=%s\n",
                creaseName(crease), level, bestLaunching, count, windows.c_str());
        }
    }

    const std::size_t scenarioFirst = kSweepPlacements;
    const auto scenarioPair = [&](const glm::vec3& slopeFace, const glm::vec3& flatFace, std::size_t first)
    {
        const Rest& slope = rests[first];
        const Rest& flat = rests[first + 1u];
        return Pair{ slopeFace + slope.capsuleOffset, flatFace + flat.capsuleOffset, slopeFace, flatFace, slope.ground, flat.ground,
            slope.movementFace, flat.movementFace };
    };
    const std::array<Pair, 2u> scenarioPairs = { scenarioPair(kLowerF, kLowerG, scenarioFirst),
        scenarioPair(kUpperF, kUpperG, scenarioFirst + 2u) };
    const std::array<std::array<glm::vec3, 2u>, 2u> scenarioAims = { { { kLowerFToG, kLowerGToF }, { kUpperFToG, kUpperGToF } } };
    std::array<std::array<uint32_t, 2u>, 2u> connecting{};
    std::array<uint32_t, 2u> flipLaunching{};
    std::string detail;
    for (const Crease crease : { Crease::Concave, Crease::Convex })
        for (const Role role : { Role::SlopeAttacker, Role::FlatAttacker })
            for (const Sense& sense : kSenses)
                for (const Mode mode : { Mode::Today, Mode::Flip })
                {
                    const Reach reach = pairReach(staticData, scenarioPairs[uint32_t(crease)], role, sense, mode,
                        scenarioAims[uint32_t(crease)][uint32_t(role)]);
                    REQUIRE(reach.started);
                    REQUIRE(reach.segmentLastsATick);
                    connecting[uint32_t(crease)][uint32_t(mode)] += reach.connects ? 1u : 0u;
                    const bool launches = mode == Mode::Flip && reach.connects && reach.leavesTargetGround;
                    flipLaunching[uint32_t(crease)] += launches ? 1u : 0u;
                    char buffer[160];
                    std::snprintf(buffer, sizeof(buffer), " %s.%s.%s.%s=%d(axisM %.4f%s)", creaseName(crease), roleName(role), sense.name,
                        modeName(mode), reach.connects ? 1 : 0, double(reach.alongAxisM), launches ? " launches" : "");
                    detail += buffer;
                }
    std::printf("[GoldenTrace.creaseScenario] today=%u/8 flip=%u/8 concave(today %u flip %u launching %u) "
                "convex(today %u flip %u launching %u)%s\n",
        connecting[0][0] + connecting[1][0], connecting[0][1] + connecting[1][1], connecting[0][0], connecting[0][1], flipLaunching[0],
        connecting[1][0], connecting[1][1], flipLaunching[1], detail.c_str());

    CHECK(connecting[uint32_t(Crease::Concave)][uint32_t(Mode::Today)] == kScenarioConcaveToday);
    CHECK(connecting[uint32_t(Crease::Concave)][uint32_t(Mode::Flip)] == kScenarioConcaveFlip);
    CHECK(connecting[uint32_t(Crease::Convex)][uint32_t(Mode::Today)] == kScenarioConvexToday);
    CHECK(connecting[uint32_t(Crease::Convex)][uint32_t(Mode::Flip)] == kScenarioConvexFlip);
    CHECK(flipLaunching[uint32_t(Crease::Convex)] >= 1u);
}

} // namespace creaseReachTests

#endif
