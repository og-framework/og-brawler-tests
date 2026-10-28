#pragma once
// SPDX-License-Identifier: BUSL-1.1
// docs/GoldenTraceScene-rationale.md

#include "catch_amalgamated.hpp"

#include "GoldenTraceFormat.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltRuntime.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltTickWorldHost.h"
#include "OGSimulationJolt/JoltWorld.h"

#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/SystemsExecutor.h"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/BrawlerRingoutScoreSystem.h"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace goldenTraceScene
{

struct Character
{
    unsigned int id = 0u;
    uint32_t slot = 0u;
    const char* prefix = "";
    glm::vec3 spawn = glm::vec3(0.f);
};

struct SpawnPoint
{
    uint32_t slot = 0u;
    glm::vec3 point = glm::vec3(0.f);
};

struct Teleport
{
    unsigned int characterId = 0u;
    uint32_t tick = 0u;
    glm::vec3 to = glm::vec3(0.f);
};

template <typename ScenarioT>
concept Scenario = requires(unsigned int id, uint32_t tick, float gravityScale) {
    { ScenarioT::kGenerator } -> std::convertible_to<const char*>;
    { ScenarioT::kDtSeconds } -> std::convertible_to<float>;
    { ScenarioT::kScenarioTicks } -> std::convertible_to<uint32_t>;
    { ScenarioT::isSampleTick(tick) } -> std::same_as<bool>;
    { ScenarioT::statics() } -> std::same_as<StaticWorldDescription>;
    { ScenarioT::staticData(gravityScale) } -> std::same_as<simulatableBrawler::StaticData>;
    { ScenarioT::inputAt(id, tick) } -> std::same_as<simulatableBrawler::PlayerInput>;
    { ScenarioT::characters() } -> std::same_as<std::span<const Character>>;
    { ScenarioT::spawnTable() } -> std::same_as<std::span<const SpawnPoint>>;
    { ScenarioT::teleports() } -> std::same_as<std::span<const Teleport>>;
};

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

inline void logJoltProblems(const char* line)
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

inline glm::vec3 nudged(glm::vec3 point, unsigned int characterId, const std::optional<goldenTrace::Nudge>& nudge)
{
    if (nudge.has_value() && nudge->characterId == characterId)
    {
        float& component = point[static_cast<glm::length_t>(nudge->component)];
        component = std::nextafter(component, nudge->direction > 0 ? std::numeric_limits<float>::infinity()
                                                                   : -std::numeric_limits<float>::infinity());
    }
    return point;
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

template <Scenario ScenarioT>
struct Scene
{
    explicit Scene(std::optional<goldenTrace::Nudge> nudge, float gravityScale)
        : staticData(ScenarioT::staticData(gravityScale))
        , slotTemplate(joltTickWorldHost::slotTemplateOf<SimulatableBrawler>())
        , queryableStatics(ScenarioT::statics())
        , world(lease.runtime,
              joltTickWorldHost::stepWorldConfigOf(slotTemplate, queryableStatics, physicsOnlyStatics, 0u,
                  simFrame::withHeight(glm::vec3(0.f), staticData.m_movementStaticData.gravity)),
              &logJoltProblems)
        , staticBodies(joltTickWorldHost::buildStatics(world, queryableStatics, physicsOnlyStatics, &logJoltProblems))
        , bodies(world)
        , query(world, &logJoltProblems, JoltSpatialQueryConfig{ kMappedCategories })
        , systems(std::piecewise_construct, HitDetection(bodies, query), brawlerHitRouting::System{}, brawlerRingout::ScoreSystem{})
        , executor(storage, staticData, bodies, query)
    {
        query.excludeStaticBodiesFromQueries(staticBodies.physicsOnly);
        for (const SpawnPoint& spawnPoint : ScenarioT::spawnTable())
            staticData.m_ringoutStaticData.spawnPoints[spawnPoint.slot] = spawnPoint.point;
        for (const Character& character : ScenarioT::characters())
        {
            detections.emplace(character.id, DetectionLog{});
            registerCharacter(character.id, character.slot, nudged(character.spawn, character.id, nudge));
        }
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

    void applyTeleports(uint32_t tick)
    {
        for (const Teleport& teleport : ScenarioT::teleports())
        {
            if (teleport.tick != tick)
                continue;
            auto& movementIc = storage.get<SimulatableBrawler>(teleport.characterId)
                                   .editAllState()
                                   .editState()
                                   .edit<brawlerMovementSimulation::InitialConditions>();
            movementIc.teleportPending = 1u;
            movementIc.teleportPos = teleport.to;
        }
    }

    void step(uint32_t tick)
    {
        applyTeleports(tick);
        const SimulationTimeStep timeStep(tick, /*isResimulating*/ false, StepKind::Normal, ScenarioT::kDtSeconds);
        Executor::ResolvedInputsType inputs;
        auto& byId = std::get<std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        for (const Character& character : ScenarioT::characters())
            byId.emplace(character.id, ScenarioT::inputAt(character.id, tick));
        systems.firePreIntegrate(timeStep, storage, staticData, /*isAuthority*/ true);
        noteDetections(tick);
        executor.integrateAll(timeStep, inputs);
        systems.firePostIntegrate(timeStep, storage, staticData, /*isAuthority*/ true);
        world.step(ScenarioT::kDtSeconds);
        executor.captureBodyStatesAll();
    }

    void noteDetections(uint32_t tick)
    {
        for (const Character& character : ScenarioT::characters())
        {
            const auto& derived = storage.get<SimulatableBrawler>(character.id).getAllState().getDerivedState();
            DetectionLog& log = detections[character.id];
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
        recorder.exact(prefix + "ringout.score", systems.template get<brawlerRingout::ScoreSystem>().scoreOf(id));

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
        for (const Character& character : ScenarioT::characters())
            recordCharacter(recorder, character.prefix, character.id);
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
    std::unordered_map<unsigned int, DetectionLog> detections;
};

template <Scenario ScenarioT>
goldenTrace::Trace runScenario(std::optional<goldenTrace::Nudge> nudge = std::nullopt, float gravityScale = 1.f)
{
    goldenTrace::Trace trace;
    goldenTrace::Recorder recorder(trace);
    Scene<ScenarioT> scene(nudge, gravityScale);
    scene.record(recorder, 0u);
    for (uint32_t tick = 1u; tick <= ScenarioT::kScenarioTicks; ++tick)
    {
        scene.step(tick);
        if (ScenarioT::isSampleTick(tick))
            scene.record(recorder, tick);
    }
    return trace;
}

template <Scenario ScenarioT>
std::vector<goldenTrace::Nudge> envelopeNudges()
{
    std::vector<goldenTrace::Nudge> nudges;
    for (const Character& character : ScenarioT::characters())
        for (const uint32_t component : { 0u, 1u })
            for (const int direction : { +1, -1 })
                nudges.push_back(goldenTrace::Nudge{ character.id, component, direction });
    return nudges;
}

template <Scenario ScenarioT>
goldenTrace::GoldenView viewOf(const goldenTrace::Trace& trace, std::vector<goldenTrace::FieldRecord>& fields,
    goldenTrace::GoldenHeader& header)
{
    fields.clear();
    for (const goldenTrace::FieldSpec& field : trace.fields)
        fields.push_back(goldenTrace::FieldRecord{ field.name.c_str(), goldenTrace::tagLetter(field.tag), field.components });
    header = goldenTrace::GoldenHeader{ goldenTrace::kFormatVersion, goldenTrace::kSimulationAxisFrame,
        goldenTrace::kSimulationUnitsPerMetre, ScenarioT::kDtSeconds,
        ScenarioT::kScenarioTicks, "live", ScenarioT::kGenerator };
    return goldenTrace::GoldenView{ &header, fields.data(), fields.size(), trace.ticks.data(), trace.ticks.size(), trace.values.data(),
        trace.values.size() };
}

inline void printMetrics(const char* label, const std::array<double, goldenTrace::kTagCount>& metrics)
{
    std::printf("[GoldenTrace.%s]", label);
    for (std::size_t tag = 0; tag < goldenTrace::kTagCount; ++tag)
        std::printf(" %c=%.9g", goldenTrace::kTagLetters[tag], metrics[tag]);
    std::printf("\n");
}

inline void printBounds(const goldenTrace::CompareResult& result)
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

inline void printSignedZero(const goldenTrace::CompareResult& result)
{
    std::printf("[GoldenTrace.signedZero]");
    for (std::size_t tag = 0; tag < goldenTrace::kTagCount; ++tag)
        if (static_cast<goldenTrace::Tag>(tag) != goldenTrace::Tag::Exact)
            std::printf(" %c=%zu", goldenTrace::kTagLetters[tag], result.signedZeroOnly[tag]);
    std::printf(" first=%s\n", result.firstSignedZero.empty() ? "none" : result.firstSignedZero.c_str());
}

inline bool writeBytes(const std::string& path, const std::string& bytes)
{
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(file);
}

inline std::optional<std::string> environmentValue(const char* name)
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

inline std::optional<goldenTrace::Mode> modeFromEnvironment()
{
    const std::string mode = environmentValue("OG_GOLDEN_MODE").value_or("");
    if (mode.empty() || mode == "exact")
        return goldenTrace::Mode::Exact;
    if (mode == "bounds")
        return goldenTrace::Mode::Bounds;
    return std::nullopt;
}

template <Scenario ScenarioT>
void requireMatchesTheRecordedTrace(bool recorded, const goldenTrace::GoldenView& golden, const goldenTrace::EnvelopeView& envelope)
{
    REQUIRE(recorded);
    const std::optional<goldenTrace::Mode> mode = modeFromEnvironment();
    {
        INFO("OG_GOLDEN_MODE is exact (the default) or bounds");
        REQUIRE(mode.has_value());
    }

    REQUIRE(golden.header->formatVersion == goldenTrace::kFormatVersion);
    REQUIRE(envelope.header->formatVersion == goldenTrace::kFormatVersion);
    REQUIRE(std::bit_cast<uint32_t>(golden.header->dtSeconds) == std::bit_cast<uint32_t>(ScenarioT::kDtSeconds));
    REQUIRE(golden.header->scenarioTicks == ScenarioT::kScenarioTicks);
    if (*mode == goldenTrace::Mode::Exact)
    {
        INFO("exact mode compares a golden recorded in the same frame and unit; run OG_GOLDEN_MODE=bounds across a flip");
        REQUIRE(golden.header->axisFrame == goldenTrace::kSimulationAxisFrame);
        REQUIRE(std::bit_cast<uint32_t>(golden.header->unitsPerMetre)
            == std::bit_cast<uint32_t>(goldenTrace::kSimulationUnitsPerMetre));
    }

    const goldenTrace::Trace live = runScenario<ScenarioT>();
    const goldenTrace::CompareResult result = *mode == goldenTrace::Mode::Exact ? goldenTrace::compareExact(golden, live)
                                                                                : goldenTrace::compareBounds(golden, envelope, live);
    std::printf("[GoldenTrace.compare] mode=%s golden=(axisFrame %u, unitsPerMetre %g, %s) live=(axisFrame %u, unitsPerMetre %g) "
                "samples=%zu fields=%zu values=%zu mismatchingFields=%zu samplesPassedBeforeFirstMismatch=%u\n",
        *mode == goldenTrace::Mode::Exact ? "exact" : "bounds", golden.header->axisFrame, double(golden.header->unitsPerMetre),
        golden.header->sourceCommit, goldenTrace::kSimulationAxisFrame, double(goldenTrace::kSimulationUnitsPerMetre),
        result.comparedSamples,
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

template <Scenario ScenarioT>
goldenTrace::Trace recordGoldenAndEnvelope(const std::string& goldenFileName, const std::string& envelopeFileName)
{
    const std::optional<std::string> outDirectory = environmentValue("OG_GOLDEN_OUT");
    REQUIRE(outDirectory.has_value());
    const std::string source = environmentValue("OG_GOLDEN_SOURCE").value_or("unknown");

    const goldenTrace::Trace base = runScenario<ScenarioT>();
    const goldenTrace::Trace repeat = runScenario<ScenarioT>();
    std::vector<goldenTrace::FieldRecord> fields;
    goldenTrace::GoldenHeader header{};
    const goldenTrace::CompareResult rerun = goldenTrace::compareExact(viewOf<ScenarioT>(base, fields, header), repeat);
    INFO(rerun.firstMismatch);
    INFO(rerun.firstSignedZero);
    REQUIRE(rerun.passed());
    REQUIRE(rerun.firstSignedZero.empty());

    const std::vector<goldenTrace::Nudge> nudges = envelopeNudges<ScenarioT>();
    std::vector<goldenTrace::Trace> nudgedRuns;
    for (const goldenTrace::Nudge& nudge : nudges)
        nudgedRuns.push_back(runScenario<ScenarioT>(nudge));
    const std::vector<goldenTrace::EnvelopeRow> envelope = goldenTrace::envelopeOf(base, nudgedRuns);

    const std::string golden = goldenTrace::writeGoldenInl(base, ScenarioT::kDtSeconds, ScenarioT::kScenarioTicks, source,
        ScenarioT::kGenerator);
    const std::string envelopeText = goldenTrace::writeEnvelopeInl(base.ticks, envelope, nudges, source);
    const std::string directory = *outDirectory + "/";
    REQUIRE(writeBytes(directory + goldenFileName, golden));
    REQUIRE(writeBytes(directory + envelopeFileName, envelopeText));

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
    return base;
}

} // namespace goldenTraceScene
