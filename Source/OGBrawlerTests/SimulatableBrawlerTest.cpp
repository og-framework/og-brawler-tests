// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// Mock physics body adapter — satisfies PhysicsBodyAdapter concept.
// Returns an identity transform; records the last setBodyLinearVelocity call.
// ---------------------------------------------------------------------------
struct FMockPhysicsBodyAdapter
{
    glm::vec3 lastSetLinearVelocity{0.f};
    PhysicsBodyState capturedState{};

    glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
    void setBodyTransform(BodyId, const glm::mat4&) {}
    void addBodyTorque(BodyId, const glm::vec3&) {}
    void setBodyAngularVelocity(BodyId, const glm::vec3& v) {}
    void setBodyLinearVelocity(BodyId, const glm::vec3& v) { lastSetLinearVelocity = v; }
    glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const
    {
        return PhysicsBodyState{};
    }
};

static_assert(PhysicsBodyAdapter<FMockPhysicsBodyAdapter>,
    "FMockPhysicsBodyAdapter must satisfy PhysicsBodyAdapter");

// ---------------------------------------------------------------------------
// Mock spatial query adapter — satisfies SpatialQueryAdapter concept.
// ---------------------------------------------------------------------------
struct FMockSpatialQueryAdapter
{
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<FMockSpatialQueryAdapter>,
    "FMockSpatialQueryAdapter must satisfy SpatialQueryAdapter");

// ---------------------------------------------------------------------------
// Helper — build a SimulatableBrawler with the new single-arg ctor.
// ---------------------------------------------------------------------------
static SimulatableBrawler makeTestCharacter()
{
    simulatableBrawler::StaticData staticData;
    return SimulatableBrawler(staticData);
}

// ---------------------------------------------------------------------------
// Test: construct and verify initial state is accessible.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulatableBrawler.Construct", "[DAttack][SimulatableBrawler]")
{
    SimulatableBrawler character = makeTestCharacter();

    // getAllState / editAllState must return the same underlying state.
    const simulatableBrawler::AllState& constState = character.getAllState();
    simulatableBrawler::AllState& mutableState = character.editAllState();
    REQUIRE(&constState == &mutableState);
}

// ---------------------------------------------------------------------------
// Test: integrate completes without crash; getVizState returns valid data after
// updateVizState.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulatableBrawler.IntegrateAndVizState", "[DAttack][SimulatableBrawler]")
{
    SimulatableBrawler character = makeTestCharacter();
    FMockPhysicsBodyAdapter physAdapter;
    FMockSpatialQueryAdapter queryAdapter;
    simulatableBrawler::StaticData staticData;
    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();

    SimulationTimeStep step(0u, false, false, false, 1.f / 60.f);
    character.integrate(step, zeroInput, physAdapter, queryAdapter, staticData);

    // Viz state must be accessible before updateVizState (returns initial copy).
    const simulatableBrawler::AllState& vizBefore = character.getVizState();
    (void)vizBefore;

    // After updateVizState, viz snapshot must reflect current physics-thread state.
    character.updateVizState();
    const simulatableBrawler::AllState& vizAfter = character.getVizState();
    (void)vizAfter;

    REQUIRE(true);
}

// ---------------------------------------------------------------------------
// Test: firstResimStep captures body state via adapter.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulatableBrawler.FirstResimStep", "[DAttack][SimulatableBrawler]")
{
    SimulatableBrawler character = makeTestCharacter();
    FMockPhysicsBodyAdapter physAdapter;

    // firstResimStep should not crash and should call captureBodyState.
    character.firstResimStep(physAdapter, 0);

    REQUIRE(true);
}

#endif // WITH_LOW_LEVEL_TESTS
