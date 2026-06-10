// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// Mock physics adapter — tracks per-body transform and linear velocity.
// BodyId.value is used as the index into the bodies vector.
// ---------------------------------------------------------------------------
namespace projectiletests
{

struct MockPhysicsAdapter
{
    struct BodyRecord
    {
        glm::mat4 transform{ 1.f };
        glm::vec3 linearVelocity{ 0.f };
    };

    std::vector<BodyRecord> bodies;

    explicit MockPhysicsAdapter(std::size_t bodyCount)
        : bodies(bodyCount)
    {}

    glm::mat4 getBodyTransform(BodyId id) const            { return bodies[id.value].transform; }
    void setBodyTransform(BodyId id, const glm::mat4& t)   { bodies[id.value].transform = t; }
    void setBodyLinearVelocity(BodyId id, const glm::vec3& v) { bodies[id.value].linearVelocity = v; }
    void addBodyTorque(BodyId, const glm::vec3&)           {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const           { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const        { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

// ---------------------------------------------------------------------------
// Mock spatial query adapter — returns a configurable hit report.
// Set nextReport before calling integrate to inject hits.
// ---------------------------------------------------------------------------
struct MockSpatialQueryAdapter
{
    SpatialQueryReport nextReport{};

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return nextReport; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr float kDt = 1.f / 60.f;

// Parent body id is always kProjectilePoolSize — a distinct id not used by any slot body.
static constexpr uint32_t kParentBodyIdValue = brawlerProjectileSimulation::kProjectilePoolSize;

static brawlerProjectileSimulation::StaticData makeStaticData(
    float speed             = 500.f,
    float maxLifetime       = 3.f,
    float colliderRadius    = 30.f,
    float spawnForwardOffset = 50.f,
    float spawnZOffset      = 0.f)
{
    return brawlerProjectileSimulation::StaticData{
        speed, maxLifetime, colliderRadius, spawnForwardOffset, spawnZOffset
    };
}

// Build N bindings: slot i has ownBodyId=i, parentBodyId=kParentBodyIdValue.
static std::array<brawlerProjectileSimulation::RuntimeBindings,
                  brawlerProjectileSimulation::kProjectilePoolSize>
makeBindings()
{
    std::array<brawlerProjectileSimulation::RuntimeBindings,
               brawlerProjectileSimulation::kProjectilePoolSize> b{};
    for (uint32_t i = 0; i < brawlerProjectileSimulation::kProjectilePoolSize; ++i)
    {
        b[i].ownBodyId    = BodyId{ i };
        b[i].parentBodyId = BodyId{ kParentBodyIdValue };
        b[i].queryVolumeIds = { QueryVolumeId{ i } };
    }
    return b;
}

// Run one integrate tick.
// ic and state are modified in-place (deps holds references into the composite).
static void callIntegrate(
    brawlerProjectileSimulation::InitialConditions& ic,
    brawlerProjectileSimulation::State&             state,
    brawlerProjectileSimulation::DerivedState&      derived,
    MockPhysicsAdapter&                             physics,
    MockSpatialQueryAdapter&                        query,
    const brawlerProjectileSimulation::StaticData&  sd)
{
    using namespace brawlerProjectileSimulation;

    // Build a composite holding references to the caller's IC and State.
    SimulationComposite<InitialConditions, State> composite(ic, state);
    auto deps = makeDependencies<Dependencies>(composite);

    PlayerInput pi{};
    IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{ kDt, physics, query };
    AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> allInput{ pi, utils };

    auto bindings = makeBindings();
    integrate(kDt, allInput, sd, deps, bindings, derived);

    // Sync back: composite owns copies (the composite was constructed by value).
    ic    = composite.get<InitialConditions>();
    state = composite.get<State>();
}

} // namespace projectiletests

// ---------------------------------------------------------------------------
// Test: spawn request into empty pool — slot 0 becomes alive, body positioned
// and velocity set, request cleared.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.SpawnIntoEmptyPool", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData();
    State state;
    InitialConditions ic;
    ic.spawnRequestPending = 1;
    ic.spawnPos = glm::vec3(10.f, 20.f, 30.f);
    ic.velocity = glm::vec3(500.f, 0.f, 0.f);

    MockPhysicsAdapter physics{ kProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    callIntegrate(ic, state, derived, physics, query, sd);

    REQUIRE(ic.spawnRequestPending == 0u);

    REQUIRE(state.slots[0].isAlive == 1u);
    REQUIRE(state.slots[0].lifetime == Catch::Approx(0.f));
    REQUIRE(state.slots[0].hitObjectIndex == -1);

    // Body transform must reflect spawnPos.
    const glm::vec3 bodyPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(bodyPos.x == Catch::Approx(10.f));
    REQUIRE(bodyPos.y == Catch::Approx(20.f));
    REQUIRE(bodyPos.z == Catch::Approx(30.f));

    REQUIRE(physics.bodies[0].linearVelocity.x == Catch::Approx(500.f));
    REQUIRE(physics.bodies[0].linearVelocity.y == Catch::Approx(0.f));
    REQUIRE(physics.bodies[0].linearVelocity.z == Catch::Approx(0.f));

    // Other slots remain dead.
    REQUIRE(state.slots[1].isAlive == 0u);
    REQUIRE(state.slots[2].isAlive == 0u);
}

// ---------------------------------------------------------------------------
// Test: spawn request when all N slots are alive — request dropped, pool
// unchanged.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.SpawnDroppedWhenPoolFull", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData();
    State state;
    for (uint32_t i = 0; i < kProjectilePoolSize; ++i)
    {
        state.slots[i].isAlive  = 1;
        state.slots[i].lifetime = 0.5f;
    }

    InitialConditions ic;
    ic.spawnRequestPending = 1;
    ic.spawnPos = glm::vec3(1.f, 2.f, 3.f);
    ic.velocity = glm::vec3(100.f, 0.f, 0.f);

    MockPhysicsAdapter physics{ kProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    callIntegrate(ic, state, derived, physics, query, sd);

    // Request must be cleared even when dropped.
    REQUIRE(ic.spawnRequestPending == 0u);

    // All slots must remain alive — nothing was overwritten.
    for (uint32_t i = 0; i < kProjectilePoolSize; ++i)
        REQUIRE(state.slots[i].isAlive == 1u);
}

// ---------------------------------------------------------------------------
// Test: slot whose lifetime exceeds maxLifetime flips isAlive to 0 and body
// is parked below the z-plane.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.LifetimeExpiryKillsSlot", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    const float maxLifetime = 1.f;
    auto sd = makeStaticData(500.f, maxLifetime);

    State state;
    state.slots[0].isAlive  = 1;
    // Advance lifetime so one more kDt tick pushes it past maxLifetime.
    state.slots[0].lifetime = maxLifetime - kDt * 0.5f;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    callIntegrate(ic, state, derived, physics, query, sd);

    REQUIRE(state.slots[0].isAlive == 0u);

    const glm::vec3 parkPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(parkPos.z < -1000.f);
    REQUIRE(physics.bodies[0].linearVelocity == glm::vec3(0.f));

    // Lifetime expiry does not generate a derived hit.
    REQUIRE(derived.hits.empty());
}

// ---------------------------------------------------------------------------
// Test: overlap query reports a hit on a non-parent body — slot dies, body
// parked, hitObjectIndex recorded, derived.hits populated.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.HitOnNonParentKillsSlot", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);

    State state;
    state.slots[0].isAlive  = 1;
    state.slots[0].lifetime = 0.1f;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    // A hit whose bodyId is NOT the parent.
    SpatialQueryHit hit{};
    hit.objectIndex    = 42;
    hit.objectPosition = glm::vec3(5.f, 5.f, 0.f);
    hit.bodyId         = BodyId{ 99 };
    query.nextReport.hits.push_back(hit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd);

    REQUIRE(state.slots[0].isAlive == 0u);
    REQUIRE(state.slots[0].hitObjectIndex == 42);

    const glm::vec3 parkPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(parkPos.z < -1000.f);
    REQUIRE(physics.bodies[0].linearVelocity == glm::vec3(0.f));

    REQUIRE(derived.hits.size() == 1u);
    REQUIRE(derived.hits[0].hitObjectIndex == 42);
}

// ---------------------------------------------------------------------------
// Test: overlap query reports a hit whose bodyId IS the parent — slot stays
// alive (self-hit ignored).
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.HitOnParentBodyIgnored", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);

    State state;
    state.slots[0].isAlive  = 1;
    state.slots[0].lifetime = 0.1f;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    SpatialQueryHit parentHit{};
    parentHit.objectIndex    = 7;
    parentHit.objectPosition = glm::vec3(0.f);
    parentHit.bodyId         = BodyId{ kParentBodyIdValue };
    query.nextReport.hits.push_back(parentHit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd);

    REQUIRE(state.slots[0].isAlive == 1u);
    REQUIRE(derived.hits.empty());
}

#endif // WITH_LOW_LEVEL_TESTS
