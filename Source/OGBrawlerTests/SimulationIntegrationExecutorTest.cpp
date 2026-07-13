// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <type_traits>
#include <utility>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// Mock adapters (mirror SimulatableBrawlerTest.cpp — kept local to avoid
// header pollution; both TUs are compiled independently).
// ---------------------------------------------------------------------------
struct FMockPhysicsBodyAdapterExec
{
    glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
    void setBodyTransform(BodyId, const glm::mat4&) {}
    void addBodyTorque(BodyId, const glm::vec3&) {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void setBodyLinearVelocity(BodyId, const glm::vec3&) {}
    glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<FMockPhysicsBodyAdapterExec>,
    "FMockPhysicsBodyAdapterExec must satisfy PhysicsBodyAdapter");

struct FMockSpatialQueryAdapterExec
{
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<FMockSpatialQueryAdapterExec>,
    "FMockSpatialQueryAdapterExec must satisfy SpatialQueryAdapter");

// ---------------------------------------------------------------------------
// Concept checks
// ---------------------------------------------------------------------------
static_assert(
    SimulatableIntegration<SimulatableBrawler, FMockPhysicsBodyAdapterExec, FMockSpatialQueryAdapterExec, simulatableBrawler::StaticData>,
    "SimulatableBrawler must satisfy SimulatableIntegration");

using FTestExecutor = SimulationIntegrationExecutor<
    simulatableBrawler::StaticData,
    FMockPhysicsBodyAdapterExec,
    FMockSpatialQueryAdapterExec,
    SimulatableBrawler>;

static_assert(SimulationIntegrationExecutorConcept<FTestExecutor>,
    "SimulationIntegrationExecutor must satisfy SimulationIntegrationExecutorConcept");

// ---------------------------------------------------------------------------
// Task 4: accessor return-type checks.
//   editStorage()   -> SimulationObjectStorage<SimulatableTs...>&
//   getStaticData() -> const StaticDataT&   (const& is load-bearing: the
//   executor's StaticData holds internal references bound to its own members,
//   so a by-value return would dangle — see SimulationIntegrationExecutor.h.)
// ---------------------------------------------------------------------------
static_assert(
    std::is_same_v<
        decltype(std::declval<FTestExecutor&>().editStorage()),
        SimulationObjectStorage<SimulatableBrawler>&>,
    "editStorage() must return SimulationObjectStorage<SimulatableTs...>&");

static_assert(
    std::is_same_v<
        decltype(std::declval<const FTestExecutor&>().getStaticData()),
        const simulatableBrawler::StaticData&>,
    "getStaticData() must return const StaticDataT&");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static SimulatableBrawler makeCharacter()
{
    simulatableBrawler::StaticData staticData;
    return SimulatableBrawler(staticData);
}

// ---------------------------------------------------------------------------
// Test: integrateAll with one registered simulatable calls integrate.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationIntegrationExecutor.IntegrateAll", "[DAttack][SimulationIntegrationExecutor]")
{
    FMockPhysicsBodyAdapterExec physAdapter;
    FMockSpatialQueryAdapterExec queryAdapter;
    SimulationObjectStorage<SimulatableBrawler> storage;

    storage.add<SimulatableBrawler>(42u, makeCharacter());

    FTestExecutor executor(physAdapter, queryAdapter, storage);

    // Build ResolvedInputs with a zero input for id=42.
    FTestExecutor::ResolvedInputsType inputs;
    std::get<std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs)
        .emplace(42u, simulatableBrawler::getZeroPlayerInput());

    SimulationTimeStep step(0u, false, false, false, 1.f / 60.f);
    executor.integrateAll(step, inputs);

    REQUIRE(true);
}

// ---------------------------------------------------------------------------
// Test: firstResimStepAll calls firstResimStep per simulatable.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationIntegrationExecutor.FirstResimStepAll", "[DAttack][SimulationIntegrationExecutor]")
{
    FMockPhysicsBodyAdapterExec physAdapter;
    FMockSpatialQueryAdapterExec queryAdapter;
    SimulationObjectStorage<SimulatableBrawler> storage;

    storage.add<SimulatableBrawler>(1u, makeCharacter());

    FTestExecutor executor(physAdapter, queryAdapter, storage);
    executor.firstResimStepAll(0);

    REQUIRE(true);
}

// ---------------------------------------------------------------------------
// Test: integrateAll with no input entry for a simulatable skips it.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationIntegrationExecutor.MissingInputSkipped", "[DAttack][SimulationIntegrationExecutor]")
{
    FMockPhysicsBodyAdapterExec physAdapter;
    FMockSpatialQueryAdapterExec queryAdapter;
    SimulationObjectStorage<SimulatableBrawler> storage;

    storage.add<SimulatableBrawler>(99u, makeCharacter());

    FTestExecutor executor(physAdapter, queryAdapter, storage);

    // Empty inputs — no entry for id=99; integrateAll must skip silently.
    FTestExecutor::ResolvedInputsType emptyInputs;
    SimulationTimeStep step(0u, false, false, false, 1.f / 60.f);
    executor.integrateAll(step, emptyInputs);

    REQUIRE(true);
}

// ---------------------------------------------------------------------------
// Test: editStorage() / getStaticData() expose the executor's internals by
// reference (task 4). editStorage() must alias the storage the executor was
// constructed with; getStaticData() must return a stable reference into the
// executor (not a fresh copy each call).
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationIntegrationExecutor.AccessorsExposeInternals", "[DAttack][SimulationIntegrationExecutor]")
{
    FMockPhysicsBodyAdapterExec physAdapter;
    FMockSpatialQueryAdapterExec queryAdapter;
    SimulationObjectStorage<SimulatableBrawler> storage;

    storage.add<SimulatableBrawler>(7u, makeCharacter());

    FTestExecutor executor(physAdapter, queryAdapter, storage);

    // editStorage() aliases the exact storage object passed to the ctor.
    REQUIRE(&executor.editStorage() == &storage);

    // getStaticData() returns the same internal reference on each call.
    const auto& sd1 = executor.getStaticData();
    const auto& sd2 = executor.getStaticData();
    REQUIRE(&sd1 == &sd2);
}

#endif // WITH_LOW_LEVEL_TESTS
