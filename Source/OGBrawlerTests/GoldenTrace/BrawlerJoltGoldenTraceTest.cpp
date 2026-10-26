// SPDX-License-Identifier: BUSL-1.1
// docs/BrawlerJoltGoldenTraceTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceFormat.h"
#include "GoldenTraceScenario.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltRuntime.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltTickWorldHost.h"
#include "OGSimulationJolt/JoltWorld.h"

#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/SystemsExecutor.h"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/BrawlerRingoutScoreSystem.h"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#if __has_include("BrawlerJoltGolden.inl") && __has_include("BrawlerJoltGoldenEnvelope.inl")
#include "BrawlerJoltGolden.inl"
#include "BrawlerJoltGoldenEnvelope.inl"

namespace
{
inline constexpr bool kGoldenRecorded = true;

goldenTrace::GoldenView recordedGolden()
{
    static_assert(goldenTrace::everyFieldHasABound(goldenTraceData::kGoldenFields),
        "BrawlerJoltGolden.inl: every recorded field has a bound class (a 'p' field's last name segment is position, linearVelocity "
        "or velocity, and no field is 'd')");
    return goldenTrace::GoldenView{ &goldenTraceData::kGoldenHeader, goldenTraceData::kGoldenFields,
        std::size(goldenTraceData::kGoldenFields), goldenTraceData::kGoldenTicks, std::size(goldenTraceData::kGoldenTicks),
        goldenTraceData::kGoldenValues, std::size(goldenTraceData::kGoldenValues) };
}

goldenTrace::EnvelopeView recordedEnvelope()
{
    static_assert(sizeof(goldenTraceEnvelopeData::kEnvelopeTagOrder) - 1u == goldenTrace::kTagCount,
        "BrawlerJoltGoldenEnvelope.inl stores one envelope column per golden tag");
    return goldenTrace::EnvelopeView{ &goldenTraceEnvelopeData::kEnvelopeHeader, goldenTraceEnvelopeData::kEnvelopeNudges,
        std::size(goldenTraceEnvelopeData::kEnvelopeNudges), goldenTraceEnvelopeData::kEnvelopeTicks,
        std::size(goldenTraceEnvelopeData::kEnvelopeTicks), goldenTraceEnvelopeData::kEnvelopeValues,
        std::size(goldenTraceEnvelopeData::kEnvelopeValues) };
}
} // namespace
#else
namespace
{
inline constexpr bool kGoldenRecorded = false;
goldenTrace::GoldenView recordedGolden() { return {}; }
goldenTrace::EnvelopeView recordedEnvelope() { return {}; }
} // namespace
#endif

namespace
{

using namespace goldenTraceScenario;

static_assert(goldenTrace::kLinearVelocityBoundCmPerS >= 2.0 * -Frame::toLegacyCm(kGravity) * double(kDtSeconds),
    "goldenTrace::kLinearVelocityBoundCmPerS admits a one-tick velocity spike of 2*g*dt");

using Storage = SimulationObjectStorage<SimulatableBrawler>;
using Executor = SimulationIntegrationExecutor<simulatableBrawler::StaticData, JoltPhysicsBodyAdapter, JoltSpatialQueryAdapter,
    SimulatableBrawler>;
using HitDetection = brawlerHitDetection::System<JoltPhysicsBodyAdapter, JoltSpatialQueryAdapter>;
using SystemsExec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>, simulatableBrawler::StaticData, HitDetection,
    brawlerHitRouting::System, brawlerRingout::ScoreSystem>;

static_assert(brawlerHitDetection::firesBefore<SystemsExec, HitDetection, brawlerHitRouting::System>,
    "The golden harness fires the hit detector before hit routing, as the game's composition does (BrawlerCompositionUImpl)");

inline constexpr uint32_t kMappedCategories = (1u << collisionCategory::body) | (1u << collisionCategory::guard)
    | (1u << collisionCategory::queryRouting) | (1u << collisionCategory::projectile) | (1u << collisionCategory::world)
    | (1u << collisionCategory::character);

inline constexpr char kGenerator[] = "BrawlerJoltGoldenTrace.Record";

void logJoltProblems(const char* line)
{
    if (std::strncmp(line, "JoltRuntime: ", 13) != 0)
        std::fprintf(stderr, "%s\n", line);
}

struct RuntimeLease
{
    RuntimeLease() : runtime(JoltRuntime::acquire(&logJoltProblems)) {}
    ~RuntimeLease() { JoltRuntime::release(); }
    RuntimeLease(const RuntimeLease&) = delete;
    RuntimeLease& operator=(const RuntimeLease&) = delete;

    JoltRuntime& runtime;
};

glm::vec3 nudged(glm::vec3 point, unsigned int characterId, const std::optional<goldenTrace::Nudge>& nudge)
{
    if (nudge.has_value() && nudge->characterId == characterId)
    {
        float& component = point[static_cast<glm::length_t>(nudge->component)];
        component = std::nextafter(component, nudge->direction > 0 ? std::numeric_limits<float>::infinity()
                                                                   : -std::numeric_limits<float>::infinity());
    }
    return point;
}

struct Scene
{
    explicit Scene(std::optional<goldenTrace::Nudge> nudge, float gravityScale)
        : staticData(scenarioStaticData(gravityScale))
        , slotTemplate(joltTickWorldHost::slotTemplateOf<SimulatableBrawler>())
        , queryableStatics(arenaStatics())
        , world(lease.runtime,
              joltTickWorldHost::stepWorldConfigOf(slotTemplate, queryableStatics, physicsOnlyStatics, 0u,
                  Frame::withHeight(glm::vec3(0.f), staticData.m_movementStaticData.gravity)),
              &logJoltProblems)
        , staticBodies(joltTickWorldHost::buildStatics(world, queryableStatics, physicsOnlyStatics, &logJoltProblems))
        , bodies(world)
        , query(world, &logJoltProblems, JoltSpatialQueryConfig{ kMappedCategories })
        , systems(std::piecewise_construct, HitDetection(bodies, query), brawlerHitRouting::System{}, brawlerRingout::ScoreSystem{})
        , executor(storage, staticData, bodies, query)
    {
        query.excludeStaticBodiesFromQueries(staticBodies.physicsOnly);
        staticData.m_ringoutStaticData.spawnPoints[kSlotA] = kSpawnA;
        staticData.m_ringoutStaticData.spawnPoints[kSlotB] = kRespawnB;
        registerCharacter(kCharacterA, kSlotA, nudged(kSpawnA, kCharacterA, nudge));
        registerCharacter(kCharacterB, kSlotB, nudged(kSpawnB, kCharacterB, nudge));
    }

    void registerCharacter(unsigned int id, uint32_t slot, const glm::vec3& spawn)
    {
        SimulatableBrawler brawler(staticData);
        JoltPhysicsFactory factory(bodies, slotTemplate, slot, id, JoltPhysicsFactoryOptions{},
            [this](BodyId body, uint32_t shapeIndex, std::optional<BodyId> rootBodyId)
            {
                return query.registerShape(body, shapeIndex, rootBodyId);
            });
        const BodyId parentBodyId = factory.parentBodyId();
        brawler.editPhysicsComposite().forEach([&](auto& decl)
        {
            using D = std::decay_t<decltype(decl)>;
            const auto& subStaticData = D::staticDataOf(staticData);
            auto created = factory.createPhysicalObject(D::descriptor(), D::name);
            decl.bindings.ownBodyId = created.bodyId;
            decl.bindings.parentBodyId = parentBodyId;
            decl.bindings.attachmentOffset = D::attachmentOffset(subStaticData);
            decl.bindings.shapeIds = std::move(created.shapeIds);
            for (const auto& volume : D::queryVolumes(subStaticData))
                decl.bindings.queryVolumeIds.push_back(query.registerVolume(volume, parentBodyId));
        });

        const BodyId capsuleBodyId = brawler.getPhysicsComposite().get<brawlerMovementSimulation::PhysicsDeclaration>().bindings.ownBodyId;
        OG_CHECK(capsuleBodyId == parentBodyId, "golden harness: the movement declaration's body must be the slot's root capsule");
        brawler.setCharacterBindings({ capsuleBodyId });
        auto& movementIc = brawler.editAllState().editState().edit<brawlerMovementSimulation::InitialConditions>();
        movementIc.teleportPending = 1u;
        movementIc.teleportPos = spawn;
        brawler.editAllState().editState().edit<brawlerRingout::InitialConditions>().spawnSlot = spawnSlots.acquire(id);
        characterIdByRootBodyId[capsuleBodyId.value] = id;

        storage.add<SimulatableBrawler>(id, std::move(brawler));
        systems.notifyCharacterRegistered(id, storage, staticData, /*isAuthority*/ true);
        occupancy.occupied.set(slot);
        world.applyBodySlotOccupancy(occupancy);
    }

    void step(uint32_t tick)
    {
        const SimulationTimeStep timeStep(tick, /*isResimulating*/ false, StepKind::Normal, kDtSeconds);
        Executor::ResolvedInputsType inputs;
        auto& byId = std::get<std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        byId.emplace(kCharacterA, inputAt(kCharacterA, tick));
        byId.emplace(kCharacterB, inputAt(kCharacterB, tick));
        systems.firePreIntegrate(timeStep, storage, staticData, /*isAuthority*/ true);
        noteDetections(tick);
        executor.integrateAll(timeStep, inputs);
        systems.firePostIntegrate(timeStep, storage, staticData, /*isAuthority*/ true);
        world.step(kDtSeconds);
        executor.captureBodyStatesAll();
    }

    struct DetectionLog
    {
        uint32_t meleeHits = 0;
        uint32_t lastMeleeHitTick = 0;
        uint32_t lastMeleeTarget = 0;
        uint32_t guardBlocks = 0;
        uint32_t lastGuardBlockTick = 0;
        uint32_t projectileHits = 0;
        uint32_t projectileBlocks = 0;
        uint32_t lastProjectileTick = 0;
        uint32_t lastProjectileTarget = 0;
    };

    void noteDetections(uint32_t tick)
    {
        for (const unsigned int id : { kCharacterA, kCharacterB })
        {
            const auto& derived = storage.get<SimulatableBrawler>(id).getAllState().getDerivedState();
            DetectionLog& log = detections[id];
            const auto& radialDerived = derived.get<dAttackRadialSimulation::DerivedState>();
            for (const auto& hit : radialDerived.getHitsThisTick())
            {
                ++log.meleeHits;
                log.lastMeleeHitTick = tick;
                log.lastMeleeTarget = static_cast<uint32_t>(hit.targetId);
            }
            if (radialDerived.getGuardBlockedThisTick())
            {
                ++log.guardBlocks;
                log.lastGuardBlockTick = tick;
            }
            for (const auto& detected : derived.get<brawlerProjectileSimulation::DerivedState>().detectedThisTick)
            {
                const bool hit = detected.outcome == brawlerProjectileSimulation::SlotOutcome::Hit;
                const bool blocked = detected.outcome == brawlerProjectileSimulation::SlotOutcome::BlockedByGuard;
                if (!hit && !blocked)
                    continue;
                log.projectileHits += hit ? 1u : 0u;
                log.projectileBlocks += blocked ? 1u : 0u;
                log.lastProjectileTick = tick;
                log.lastProjectileTarget = characterIdOfRoot(detected.struckRootBodyId);
            }
        }
    }

    uint32_t characterIdOfRoot(BodyId rootBodyId) const
    {
        const auto found = characterIdByRootBodyId.find(rootBodyId.value);
        return found == characterIdByRootBodyId.end() ? 0u : found->second;
    }

    void recordCharacter(goldenTrace::Recorder& recorder, const std::string& prefix, unsigned int id) const
    {
        const SimulatableBrawler& brawler = storage.get<SimulatableBrawler>(id);
        const auto& state = brawler.getAllState().getState();
        const auto& derived = brawler.getAllState().getDerivedState();

        uint32_t declarationIndex = 0;
        brawler.getPhysicsComposite().forEach([&](const auto& decl)
        {
            using D = std::decay_t<decltype(decl)>;
            const PhysicsBodyState body = static_cast<PhysicsBodyState>(D::bodyStateOf(state.template get<typename D::StateType>()));
            const std::string field = prefix + "body" + std::to_string(declarationIndex++) + "." + D::name;
            recorder.position(field + ".position", body.position);
            recorder.rotation(field + ".rotation", body.rotation);
            recorder.position(field + ".linearVelocity", body.linearVelocity);
            recorder.axial(field + ".angularVelocity", body.angularVelocity);
        });

        const auto& movement = state.get<brawlerMovementSimulation::State>();
        recorder.position(prefix + "movement.velocity", movement.velocity);
        recorder.planar(prefix + "movement.committedStepDir", movement.committedStepDir);
        recorder.exact(prefix + "movement.flags", movement.flags);

        const auto& radialIc = state.get<dAttackRadialSimulation::InitialConditions>();
        const auto& radial = state.get<dAttackRadialSimulation::State>();
        recorder.scalar(prefix + "radial.initialAimAngle", radialIc.initialAimAngle);
        recorder.axial(prefix + "radial.initialAimRotationAxis", radialIc.initialAimRotationAxis);
        recorder.exact(prefix + "radial.activeAttackSequence", radialIc.activeAttackSequence);
        recorder.exact(prefix + "radial.currenSequenceId", radial.currenSequenceId);
        for (std::size_t index = 0; index < radial.hitTargets.size(); ++index)
            recorder.exact(prefix + "radial.hitTargets" + std::to_string(index), static_cast<uint32_t>(radial.hitTargets[index]));

        const auto& machine = state.get<dAttackMachineSimulation::State>();
        recorder.exact(prefix + "machine.currentState", static_cast<uint32_t>(machine.m_currentState));
        recorder.exact(prefix + "machine.activeAttackSequence", machine.m_activeAttackSequence);

        const auto& projectiles = state.get<brawlerProjectileSimulation::State>();
        for (std::size_t slot = 0; slot < projectiles.slots.size(); ++slot)
        {
            const std::string field = prefix + "projectile.slot" + std::to_string(slot);
            recorder.exact(field + ".spawnTick", projectiles.slots[slot].spawnTick);
            recorder.exact(field + ".endTick", projectiles.slots[slot].endTick);
            recorder.exact(field + ".endReason", projectiles.slots[slot].endReason);
        }

        const auto& ringout = state.get<brawlerRingout::State>();
        recorder.exact(prefix + "ringout.flags", ringout.flags);
        recorder.exact(prefix + "ringout.respawnAtTick", ringout.respawnAtTick);
        recorder.exact(prefix + "ringout.score", systems.get<brawlerRingout::ScoreSystem>().scoreOf(id));

        const DetectionLog& log = detections.at(id);
        recorder.exact(prefix + "hit.meleeHits", log.meleeHits);
        recorder.exact(prefix + "hit.lastMeleeHitTick", log.lastMeleeHitTick);
        recorder.exact(prefix + "hit.lastMeleeTarget", log.lastMeleeTarget);
        recorder.exact(prefix + "hit.guardBlocks", log.guardBlocks);
        recorder.exact(prefix + "hit.lastGuardBlockTick", log.lastGuardBlockTick);
        recorder.exact(prefix + "hit.projectileHits", log.projectileHits);
        recorder.exact(prefix + "hit.projectileBlocks", log.projectileBlocks);
        recorder.exact(prefix + "hit.lastProjectileTick", log.lastProjectileTick);
        recorder.exact(prefix + "hit.lastProjectileTarget", log.lastProjectileTarget);
        const auto& inbound = derived.get<brawlerInboundHit::DerivedState>();
        recorder.exact(prefix + "hit.inboundHit", inbound.wasHitThisTick ? 1u : 0u);
        recorder.exact(prefix + "hit.inboundGuardBlocked", inbound.wasGuardBlockedThisTick ? 1u : 0u);
        recorder.planar(prefix + "hit.inboundDirection", inbound.hitDirectionXY);
    }

    void record(goldenTrace::Recorder& recorder, uint32_t tick) const
    {
        recorder.beginSample(tick);
        recordCharacter(recorder, "A.", kCharacterA);
        recordCharacter(recorder, "B.", kCharacterB);
        recorder.endSample();
    }

    RuntimeLease lease;
    JoltWorldAccessScope accessScope;
    simulatableBrawler::StaticData staticData;
    std::vector<SlotBodyTemplate> slotTemplate;
    StaticWorldDescription queryableStatics;
    StaticWorldDescription physicsOnlyStatics;
    JoltWorld world;
    JoltTickStaticBodies staticBodies;
    JoltPhysicsBodyAdapter bodies;
    JoltSpatialQueryAdapter query;
    Storage storage;
    SystemsExec systems;
    Executor executor;
    BodySlotOccupancy occupancy;
    brawlerRingout::SpawnSlotAllocator spawnSlots;
    std::unordered_map<uint32_t, uint32_t> characterIdByRootBodyId;
    std::unordered_map<unsigned int, DetectionLog> detections{ { kCharacterA, DetectionLog{} }, { kCharacterB, DetectionLog{} } };
};

goldenTrace::Trace runScenario(std::optional<goldenTrace::Nudge> nudge = std::nullopt, float gravityScale = 1.f)
{
    goldenTrace::Trace trace;
    goldenTrace::Recorder recorder(trace);
    Scene scene(nudge, gravityScale);
    scene.record(recorder, 0u);
    for (uint32_t tick = 1u; tick <= kScenarioTicks; ++tick)
    {
        scene.step(tick);
        if (isSampleTick(tick))
            scene.record(recorder, tick);
    }
    return trace;
}

const std::vector<goldenTrace::Nudge>& envelopeNudges()
{
    static const std::vector<goldenTrace::Nudge> nudges{
        { kCharacterA, 0u, +1 }, { kCharacterA, 0u, -1 }, { kCharacterA, 1u, +1 }, { kCharacterA, 1u, -1 },
        { kCharacterB, 0u, +1 }, { kCharacterB, 0u, -1 }, { kCharacterB, 1u, +1 }, { kCharacterB, 1u, -1 } };
    return nudges;
}

goldenTrace::GoldenView viewOf(const goldenTrace::Trace& trace, std::vector<goldenTrace::FieldRecord>& fields,
    goldenTrace::GoldenHeader& header)
{
    fields.clear();
    for (const goldenTrace::FieldSpec& field : trace.fields)
        fields.push_back(goldenTrace::FieldRecord{ field.name.c_str(), goldenTrace::tagLetter(field.tag), field.components });
    header = goldenTrace::GoldenHeader{ goldenTrace::kFormatVersion, static_cast<uint32_t>(kAxisFrame), kUnitsPerMetre, kDtSeconds,
        kScenarioTicks, "live", kGenerator };
    return goldenTrace::GoldenView{ &header, fields.data(), fields.size(), trace.ticks.data(), trace.ticks.size(), trace.values.data(),
        trace.values.size() };
}

struct TraceReader
{
    const goldenTrace::Trace& trace;

    double at(std::size_t sample, const std::string& name, uint32_t component = 0u) const
    {
        const std::optional<std::size_t> field = trace.fieldIndex(name);
        OG_CHECK(field.has_value(), "golden harness: the coverage count names a field the trace does not record");
        return trace.sample(sample)[trace.fields[*field].offset + component];
    }
};

struct Coverage
{
    uint32_t walkTicks = 0;
    uint32_t rampTicks = 0;
    uint32_t plateauTicks = 0;
    uint32_t stepDescentTicks = 0;
    uint32_t rightSwings = 0;
    uint32_t leftSwings = 0;
    uint32_t overheadSwings = 0;
    uint32_t hadoukens = 0;
    uint32_t guardBlocksOfRightSwing = 0;
    uint32_t guardBlocksOfLeftSwing = 0;
    uint32_t projectilesBlocked = 0;
    uint32_t projectilesHit = 0;
    uint32_t detectedMeleeHits = 0;
    uint32_t detectedGuardBlocks = 0;
    uint32_t detectedProjectileHits = 0;
    uint32_t detectedProjectileBlocks = 0;
    uint32_t meleeHitFlinches = 0;
    uint32_t knockbackTicks = 0;
    uint32_t ledgeFallTicks = 0;
    uint32_t deaths = 0;
    uint32_t respawns = 0;
};

inline constexpr uint32_t kAttacking = static_cast<uint32_t>(DAttackState::Attacking);
inline constexpr uint32_t kGuardFlinch = static_cast<uint32_t>(DAttackState::GuardFlinch);
inline constexpr uint32_t kHitFlinch = static_cast<uint32_t>(DAttackState::HitFlinch);
inline constexpr uint32_t kSupported = static_cast<uint32_t>(brawlerMovementSimulation::SupportState::Supported);
inline constexpr double kWalkSpeedFloorCmPerS = 50.0;
inline constexpr double kKnockbackSpeedFloorCmPerS = 1000.0;
inline constexpr double kHeightSlackCm = 1.0;

uint32_t supportOf(double flags)
{
    return (static_cast<uint32_t>(flags) & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift;
}

Coverage coverageOf(const goldenTrace::Trace& trace)
{
    const TraceReader read{ trace };
    Coverage coverage;
    const std::string capsuleA = "A.body5.CharacterCapsule.position";
    const std::string capsuleB = "B.body5.CharacterCapsule.position";
    std::optional<double> restHeightA;
    for (std::size_t sample = 0; sample < trace.ticks.size(); ++sample)
    {
        if (trace.ticks[sample] == kWalkFirstTick - 1u)
            restHeightA = read.at(sample, capsuleA, 2u);
    }
    OG_CHECK(restHeightA.has_value(), "golden harness: the coverage count needs A's resting height before the walk");
    const double plateauHeightA = *restHeightA + Frame::toLegacyCm(kStepHeight);
    const double rampFoot = Frame::toLegacyCm(kRampFoot);
    const double rampTop = Frame::toLegacyCm(kRampTop);
    const double stepEdge = Frame::toLegacyCm(kStepEdge);
    const double floorStart = Frame::toLegacyCm(kFloorStart);

    for (std::size_t sample = 1; sample < trace.ticks.size(); ++sample)
    {
        const double ax = read.at(sample, capsuleA, 0u);
        const double az = read.at(sample, capsuleA, 2u);
        const double avx = read.at(sample, "A.movement.velocity", 0u);
        const double avy = read.at(sample, "A.movement.velocity", 1u);
        const bool aSupported = supportOf(read.at(sample, "A.movement.flags")) == kSupported;
        const uint32_t aState = static_cast<uint32_t>(read.at(sample, "A.machine.currentState"));
        const uint32_t aPrevious = static_cast<uint32_t>(read.at(sample - 1u, "A.machine.currentState"));
        const uint32_t aSequence = static_cast<uint32_t>(read.at(sample, "A.machine.activeAttackSequence"));
        const uint32_t aPreviousSequence = static_cast<uint32_t>(read.at(sample - 1u, "A.machine.activeAttackSequence"));

        if (aSupported && aState != kAttacking && std::hypot(avx, avy) > kWalkSpeedFloorCmPerS)
            ++coverage.walkTicks;
        if (aSupported && ax > rampFoot && ax < rampTop && az > *restHeightA + kHeightSlackCm
            && az < plateauHeightA - kHeightSlackCm)
            ++coverage.rampTicks;
        if (aSupported && ax > rampTop && ax < stepEdge && std::fabs(az - plateauHeightA) < kHeightSlackCm)
            ++coverage.plateauTicks;
        if (ax > stepEdge && az > *restHeightA + kHeightSlackCm && az < plateauHeightA - kHeightSlackCm)
            ++coverage.stepDescentTicks;

        if (aState == kAttacking && aPrevious != kAttacking)
        {
            coverage.rightSwings += aSequence == dAttackDirection::kRightSequenceId ? 1u : 0u;
            coverage.leftSwings += aSequence == dAttackDirection::kLeftSequenceId ? 1u : 0u;
            coverage.overheadSwings += aSequence == dAttackDirection::kForwardSequenceId ? 1u : 0u;
        }
        if (aState == kGuardFlinch && aPrevious != kGuardFlinch)
        {
            coverage.guardBlocksOfRightSwing += aPreviousSequence == dAttackDirection::kRightSequenceId ? 1u : 0u;
            coverage.guardBlocksOfLeftSwing += aPreviousSequence == dAttackDirection::kLeftSequenceId ? 1u : 0u;
        }

        for (const char* who : { "A.", "B." })
        {
            const std::string prefix = who;
            const uint32_t state = static_cast<uint32_t>(read.at(sample, prefix + "machine.currentState"));
            const uint32_t previous = static_cast<uint32_t>(read.at(sample - 1u, prefix + "machine.currentState"));
            const uint32_t sequence = static_cast<uint32_t>(read.at(sample, prefix + "machine.activeAttackSequence"));
            if (state == kAttacking && sequence == kHadoukenSequenceSentinel
                && !(previous == kAttacking
                    && static_cast<uint32_t>(read.at(sample - 1u, prefix + "machine.activeAttackSequence")) == kHadoukenSequenceSentinel))
                ++coverage.hadoukens;
            if (state == kHitFlinch && previous != kHitFlinch)
                ++coverage.meleeHitFlinches;
            for (uint32_t slot = 0; slot < brawlerProjectileSimulation::kMaxProjectilePoolSize; ++slot)
            {
                const std::string field = prefix + "projectile.slot" + std::to_string(slot) + ".endReason";
                const uint32_t reason = static_cast<uint32_t>(read.at(sample, field));
                const uint32_t previousReason = static_cast<uint32_t>(read.at(sample - 1u, field));
                if (reason != previousReason)
                {
                    coverage.projectilesBlocked += reason == static_cast<uint32_t>(brawlerProjectileSimulation::SlotOutcome::BlockedByGuard) ? 1u : 0u;
                    coverage.projectilesHit += reason == static_cast<uint32_t>(brawlerProjectileSimulation::SlotOutcome::Hit) ? 1u : 0u;
                }
            }
        }

        const double bx = read.at(sample, capsuleB, 0u);
        const double bvz = read.at(sample, "B.body5.CharacterCapsule.linearVelocity", 2u);
        const double bvx = read.at(sample, "B.movement.velocity", 0u);
        const double bvy = read.at(sample, "B.movement.velocity", 1u);
        if (std::hypot(bvx, bvy) > kKnockbackSpeedFloorCmPerS)
            ++coverage.knockbackTicks;
        if (bx < floorStart && bvz < 0.0)
            ++coverage.ledgeFallTicks;
        const bool dead = (static_cast<uint32_t>(read.at(sample, "B.ringout.flags")) & brawlerRingout::kFlagDead) != 0u;
        const bool wasDead = (static_cast<uint32_t>(read.at(sample - 1u, "B.ringout.flags")) & brawlerRingout::kFlagDead) != 0u;
        coverage.deaths += dead && !wasDead ? 1u : 0u;
        coverage.respawns += !dead && wasDead ? 1u : 0u;
    }
    const std::size_t last = trace.ticks.size() - 1u;
    for (const char* who : { "A.", "B." })
    {
        const std::string prefix = who;
        coverage.detectedMeleeHits += static_cast<uint32_t>(read.at(last, prefix + "hit.meleeHits"));
        coverage.detectedGuardBlocks += static_cast<uint32_t>(read.at(last, prefix + "hit.guardBlocks"));
        coverage.detectedProjectileHits += static_cast<uint32_t>(read.at(last, prefix + "hit.projectileHits"));
        coverage.detectedProjectileBlocks += static_cast<uint32_t>(read.at(last, prefix + "hit.projectileBlocks"));
    }
    return coverage;
}

void printCoverage(const char* label, const Coverage& coverage)
{
    std::printf("[GoldenTrace.coverage] %s walkTicks=%u rampTicks=%u plateauTicks=%u stepDescentTicks=%u rightSwings=%u leftSwings=%u "
                "overheadSwings=%u hadoukens=%u guardBlocksOfRightSwing=%u guardBlocksOfLeftSwing=%u projectilesBlocked=%u "
                "projectilesHit=%u hitFlinches=%u knockbackTicks=%u ledgeFallTicks=%u deaths=%u respawns=%u detectedMeleeHits=%u "
                "detectedGuardBlocks=%u detectedProjectileHits=%u detectedProjectileBlocks=%u\n",
        label, coverage.walkTicks, coverage.rampTicks, coverage.plateauTicks, coverage.stepDescentTicks, coverage.rightSwings,
        coverage.leftSwings, coverage.overheadSwings, coverage.hadoukens, coverage.guardBlocksOfRightSwing,
        coverage.guardBlocksOfLeftSwing, coverage.projectilesBlocked, coverage.projectilesHit, coverage.meleeHitFlinches,
        coverage.knockbackTicks, coverage.ledgeFallTicks, coverage.deaths, coverage.respawns, coverage.detectedMeleeHits,
        coverage.detectedGuardBlocks, coverage.detectedProjectileHits, coverage.detectedProjectileBlocks);
}

void printMetrics(const char* label, const std::array<double, goldenTrace::kTagCount>& metrics)
{
    std::printf("[GoldenTrace.%s]", label);
    for (std::size_t tag = 0; tag < goldenTrace::kTagCount; ++tag)
        std::printf(" %c=%.9g", goldenTrace::kTagLetters[tag], metrics[tag]);
    std::printf("\n");
}

void printBounds(const goldenTrace::CompareResult& result)
{
    for (std::size_t index = 0; index < goldenTrace::kBoundClassCount; ++index)
    {
        const goldenTrace::ClassMaximum& largest = result.classMaximum[index];
        const double bound = goldenTrace::kBounds[index];
        const double ratio = largest.metric == 0.0 ? 0.0 : largest.metric / bound;
        const std::string worst = largest.field.empty() ? std::string("none")
                                                        : "tick " + std::to_string(largest.tick) + " field '" + largest.field + "'";
        std::printf("[GoldenTrace.bound] class=%s max=%.9g bound=%.9g maxOverBound=%.6g worst=%s\n", goldenTrace::kBoundClassNames[index],
            largest.metric, bound, ratio, worst.c_str());
    }
}

void printSignedZero(const goldenTrace::CompareResult& result)
{
    std::printf("[GoldenTrace.signedZero]");
    for (std::size_t tag = 0; tag < goldenTrace::kTagCount; ++tag)
        if (static_cast<goldenTrace::Tag>(tag) != goldenTrace::Tag::Exact)
            std::printf(" %c=%zu", goldenTrace::kTagLetters[tag], result.signedZeroOnly[tag]);
    std::printf(" first=%s\n", result.firstSignedZero.empty() ? "none" : result.firstSignedZero.c_str());
}

bool writeBytes(const std::string& path, const std::string& bytes)
{
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(file);
}

std::optional<std::string> environmentValue(const char* name)
{
#if defined(_MSC_VER)
    char* value = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&value, &length, name) != 0 || value == nullptr)
        return std::nullopt;
    std::string text(value);
    std::free(value);
    return text;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

std::optional<goldenTrace::Mode> modeFromEnvironment()
{
    const std::string mode = environmentValue("OG_GOLDEN_MODE").value_or("");
    if (mode.empty() || mode == "exact")
        return goldenTrace::Mode::Exact;
    if (mode == "bounds")
        return goldenTrace::Mode::Bounds;
    return std::nullopt;
}

} // namespace

TEST_CASE("BrawlerJoltGoldenTrace.ScriptCoversEveryTickOnce", "[GoldenTrace]")
{
    uint32_t coveredTicks = 0;
    for (const unsigned int id : { kCharacterA, kCharacterB })
        for (uint32_t tick = 1u; tick <= kScenarioTicks; ++tick)
            coveredTicks += scriptRowAt(id, tick).has_value() ? 1u : 0u;
    REQUIRE(coveredTicks == 2u * kScenarioTicks);
}

TEST_CASE("BrawlerJoltGoldenTrace.MatchesTheRecordedTrace", "[GoldenTrace]")
{
    REQUIRE(kGoldenRecorded);
    const std::optional<goldenTrace::Mode> mode = modeFromEnvironment();
    {
        INFO("OG_GOLDEN_MODE is exact (the default) or bounds");
        REQUIRE(mode.has_value());
    }

    const goldenTrace::GoldenView golden = recordedGolden();
    const goldenTrace::EnvelopeView envelope = recordedEnvelope();
    REQUIRE(golden.header->formatVersion == goldenTrace::kFormatVersion);
    REQUIRE(envelope.header->formatVersion == goldenTrace::kFormatVersion);
    REQUIRE(std::bit_cast<uint32_t>(golden.header->dtSeconds) == std::bit_cast<uint32_t>(kDtSeconds));
    REQUIRE(golden.header->scenarioTicks == kScenarioTicks);
    if (*mode == goldenTrace::Mode::Exact)
    {
        INFO("exact mode compares a golden recorded at the same knobs; run OG_GOLDEN_MODE=bounds across a flip");
        REQUIRE(golden.header->axisFrame == static_cast<uint32_t>(kAxisFrame));
        REQUIRE(std::bit_cast<uint32_t>(golden.header->unitsPerMetre) == std::bit_cast<uint32_t>(kUnitsPerMetre));
    }

    const goldenTrace::Trace live = runScenario();
    const goldenTrace::CompareResult result = *mode == goldenTrace::Mode::Exact ? goldenTrace::compareExact(golden, live)
                                                                                : goldenTrace::compareBounds(golden, envelope, live);
    std::printf("[GoldenTrace.compare] mode=%s golden=(axisFrame %u, unitsPerMetre %g, %s) live=(axisFrame %u, unitsPerMetre %g) "
                "samples=%zu fields=%zu values=%zu mismatchingFields=%zu samplesPassedBeforeFirstMismatch=%u\n",
        *mode == goldenTrace::Mode::Exact ? "exact" : "bounds", golden.header->axisFrame, double(golden.header->unitsPerMetre),
        golden.header->sourceCommit, static_cast<uint32_t>(kAxisFrame), double(kUnitsPerMetre), result.comparedSamples,
        live.fields.size(), live.values.size(), result.mismatchingValues, result.ticksPassedBeforeFirstMismatch);
    printMetrics("maxMetric", result.maxMetric);
    if (*mode == goldenTrace::Mode::Exact)
        printSignedZero(result);
    if (*mode == goldenTrace::Mode::Bounds)
    {
        printBounds(result);
        printMetrics("envelopeRatio", result.envelopeRatio);
    }
    if (!result.firstMismatch.empty())
        std::printf("[GoldenTrace.firstMismatch] %s\n", result.firstMismatch.c_str());

    CHECK(result.schemaProblem == "");
    CHECK(result.firstMismatch == "");
    REQUIRE(result.passed());
}

TEST_CASE("BrawlerJoltGoldenTrace.ScenarioCoversEveryScriptedMechanic", "[GoldenTrace]")
{
    const Coverage coverage = coverageOf(runScenario());
    printCoverage("live", coverage);
    CHECK(coverage.walkTicks > 0u);
    CHECK(coverage.rampTicks > 0u);
    CHECK(coverage.plateauTicks > 0u);
    CHECK(coverage.stepDescentTicks > 0u);
    CHECK(coverage.rightSwings >= 2u);
    CHECK(coverage.leftSwings >= 1u);
    CHECK(coverage.overheadSwings >= 2u);
    CHECK(coverage.hadoukens >= 2u);
    CHECK(coverage.guardBlocksOfRightSwing >= 1u);
    CHECK(coverage.guardBlocksOfLeftSwing >= 1u);
    CHECK(coverage.projectilesBlocked >= 1u);
    CHECK(coverage.projectilesHit >= 1u);
    CHECK(coverage.meleeHitFlinches >= 2u);
    CHECK(coverage.knockbackTicks > 0u);
    CHECK(coverage.ledgeFallTicks > 0u);
    CHECK(coverage.deaths >= 1u);
    CHECK(coverage.respawns >= 1u);
    CHECK(coverage.detectedMeleeHits >= 2u);
    CHECK(coverage.detectedGuardBlocks >= 2u);
    CHECK(coverage.detectedProjectileHits >= 1u);
    CHECK(coverage.detectedProjectileBlocks >= 1u);
}

TEST_CASE("BrawlerJoltGoldenTrace.Record", "[.GoldenRecord]")
{
    const std::optional<std::string> outDirectory = environmentValue("OG_GOLDEN_OUT");
    REQUIRE(outDirectory.has_value());
    const std::string source = environmentValue("OG_GOLDEN_SOURCE").value_or("unknown");

    const goldenTrace::Trace base = runScenario();
    const goldenTrace::Trace repeat = runScenario();
    std::vector<goldenTrace::FieldRecord> fields;
    goldenTrace::GoldenHeader header{};
    const goldenTrace::CompareResult rerun = goldenTrace::compareExact(viewOf(base, fields, header), repeat);
    INFO(rerun.firstMismatch);
    INFO(rerun.firstSignedZero);
    REQUIRE(rerun.passed());
    REQUIRE(rerun.firstSignedZero.empty());

    std::vector<goldenTrace::Trace> nudgedRuns;
    for (const goldenTrace::Nudge& nudge : envelopeNudges())
        nudgedRuns.push_back(runScenario(nudge));
    const std::vector<goldenTrace::EnvelopeRow> envelope = goldenTrace::envelopeOf(base, nudgedRuns);

    const std::string golden = goldenTrace::writeGoldenInl(base, kDtSeconds, kScenarioTicks, source, kGenerator);
    const std::string envelopeText = goldenTrace::writeEnvelopeInl(base.ticks, envelope, envelopeNudges(), source);
    const std::string directory = *outDirectory + "/";
    REQUIRE(writeBytes(directory + "BrawlerJoltGolden.inl", golden));
    REQUIRE(writeBytes(directory + "BrawlerJoltGoldenEnvelope.inl", envelopeText));

    std::array<double, goldenTrace::kTagCount> envelopeMax{};
    for (const goldenTrace::EnvelopeRow& row : envelope)
        for (std::size_t tag = 0; tag < goldenTrace::kTagCount; ++tag)
            envelopeMax[tag] = std::max(envelopeMax[tag], row[tag]);
    std::printf("[GoldenTrace.record] samples=%zu fields=%zu values=%zu goldenBytes=%zu envelopeBytes=%zu runs=%zu source=%s\n",
        base.ticks.size(), base.fields.size(), base.values.size(), golden.size(), envelopeText.size(), nudgedRuns.size(), source.c_str());
    printMetrics("envelopeMax", envelopeMax);
    for (std::size_t sample = 0; sample < base.ticks.size(); ++sample)
    {
        if (envelope[sample][static_cast<std::size_t>(goldenTrace::Tag::Exact)] > 0.0)
        {
            std::printf("[GoldenTrace.envelopeFirstDiscreteDivergence] tick=%u differingDiscreteValues=%.0f\n", base.ticks[sample],
                envelope[sample][static_cast<std::size_t>(goldenTrace::Tag::Exact)]);
            break;
        }
    }
    printCoverage("recorded", coverageOf(base));
}

#endif
