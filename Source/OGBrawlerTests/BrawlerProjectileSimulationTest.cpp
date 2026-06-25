// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGBrawler/CollisionCategoryConstants.h"

// ---------------------------------------------------------------------------
// Closed-form projectile (Task 13).
//
// The projectile slot stores ONLY launch parameters
// { spawnTick, spawnPos, spawnDir, endTick, endReason, hitObjectIndex }; the
// per-tick world position is DERIVED from
//   pos(t) = spawnPos + spawnDir * projectileSpeed * dt * (currentTick - spawnTick)
// and snapped onto the physics body each tick. "Alive" is the predicate
//   slot.isAlive(currentTick) == (spawnTick != 0 && (endTick == 0 || currentTick < endTick)).
// These tests therefore drive state through spawnTick / endTick and pass an
// explicit currentTick into integrate via IntegrationUtils.
// ---------------------------------------------------------------------------

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

// Parent body id is always kMaxProjectilePoolSize — a distinct id not used by any slot body.
static constexpr uint32_t kParentBodyIdValue = brawlerProjectileSimulation::kMaxProjectilePoolSize;

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
                  brawlerProjectileSimulation::kMaxProjectilePoolSize>
makeBindings()
{
    std::array<brawlerProjectileSimulation::RuntimeBindings,
               brawlerProjectileSimulation::kMaxProjectilePoolSize> b{};
    for (uint32_t i = 0; i < brawlerProjectileSimulation::kMaxProjectilePoolSize; ++i)
    {
        b[i].ownBodyId    = BodyId{ i };
        b[i].parentBodyId = BodyId{ kParentBodyIdValue };
        b[i].queryVolumeIds = { QueryVolumeId{ i } };
    }
    return b;
}

// Run one integrate tick at the given simulation tick.
// ic and state are modified in-place (deps holds references into the composite).
static void callIntegrate(
    brawlerProjectileSimulation::InitialConditions& ic,
    brawlerProjectileSimulation::State&             state,
    brawlerProjectileSimulation::DerivedState&      derived,
    MockPhysicsAdapter&                             physics,
    MockSpatialQueryAdapter&                        query,
    const brawlerProjectileSimulation::StaticData&  sd,
    uint32_t                                        currentTick)
{
    using namespace brawlerProjectileSimulation;

    // Build a composite holding references to the caller's IC and State.
    SimulationComposite<InitialConditions, State> composite(ic, state);
    auto deps = makeDependencies<Dependencies>(composite);

    PlayerInput pi{};
    IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{ kDt, currentTick, physics, query };
    AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> allInput{ pi, utils };

    auto bindings = makeBindings();
    integrate(kDt, allInput, sd, deps, bindings, derived);

    // Sync back: composite owns copies (the composite was constructed by value).
    ic    = composite.get<InitialConditions>();
    state = composite.get<State>();
}

} // namespace projectiletests

// ---------------------------------------------------------------------------
// Test: spawn request into empty pool — slot 0 takes the launch parameters,
// body positioned at spawnPos with derived velocity, request cleared.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.SpawnIntoEmptyPool", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(/*speed*/500.f);
    State state;
    InitialConditions ic;
    ic.spawnRequestPending = 1;
    ic.spawnPos = glm::vec3(10.f, 20.f, 30.f);
    ic.spawnDir = glm::vec3(1.f, 0.f, 0.f);   // unit; velocity derived = dir * speed

    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    const uint32_t spawnTick = 100u;
    callIntegrate(ic, state, derived, physics, query, sd, spawnTick);

    REQUIRE(ic.spawnRequestPending == 0u);

    // Launch parameters captured; slot alive on the spawn tick.
    REQUIRE(state.slots[0].spawnTick == spawnTick);
    REQUIRE(state.slots[0].endTick == 0u);
    REQUIRE(state.slots[0].endReason == 0u);
    REQUIRE(state.slots[0].hitObjectIndex == -1);
    REQUIRE(state.slots[0].isAlive(spawnTick));

    // Body transform must reflect spawnPos (elapsed == 0 ⇒ derivedPos == spawnPos).
    const glm::vec3 bodyPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(bodyPos.x == Catch::Approx(10.f));
    REQUIRE(bodyPos.y == Catch::Approx(20.f));
    REQUIRE(bodyPos.z == Catch::Approx(30.f));

    // Linear velocity is derived: spawnDir * projectileSpeed.
    REQUIRE(physics.bodies[0].linearVelocity.x == Catch::Approx(500.f));
    REQUIRE(physics.bodies[0].linearVelocity.y == Catch::Approx(0.f));
    REQUIRE(physics.bodies[0].linearVelocity.z == Catch::Approx(0.f));

    // Other slots remain free / not alive.
    REQUIRE(state.slots[1].spawnTick == 0u);
    REQUIRE_FALSE(state.slots[1].isAlive(spawnTick));
    REQUIRE_FALSE(state.slots[2].isAlive(spawnTick));
}

// ---------------------------------------------------------------------------
// Test: closed-form determinism — after spawn, the body position at a later
// tick equals the analytic pos(t) without any per-tick re-sync.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.ClosedFormDerivedPosition", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(/*speed*/500.f, /*maxLifetime*/3.f);
    State state;
    InitialConditions ic;
    DerivedState derived;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    // Spawn at tick 100, origin, +X direction.
    ic.spawnRequestPending = 1;
    ic.spawnPos = glm::vec3(0.f, 0.f, 0.f);
    ic.spawnDir = glm::vec3(1.f, 0.f, 0.f);
    const uint32_t spawnTick = 100u;
    callIntegrate(ic, state, derived, physics, query, sd, spawnTick);
    REQUIRE(state.slots[0].isAlive(spawnTick));

    // 30 ticks later (no new request): elapsed = 30 * (1/60) = 0.5 s.
    // derivedPos = 0 + (1,0,0) * 500 * 0.5 = (250, 0, 0).
    const uint32_t laterTick = spawnTick + 30u;
    callIntegrate(ic, state, derived, physics, query, sd, laterTick);

    REQUIRE(state.slots[0].isAlive(laterTick));
    const glm::vec3 bodyPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(bodyPos.x == Catch::Approx(250.f));
    REQUIRE(bodyPos.y == Catch::Approx(0.f));
    REQUIRE(bodyPos.z == Catch::Approx(0.f));
}

// ---------------------------------------------------------------------------
// Test: spawn request when all N slots are alive — request dropped, the
// existing slots' launch parameters (and crucially their spawnTick — R-T5) are
// not overwritten.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.SpawnDroppedWhenPoolFull", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData();
    const uint32_t currentTick = 200u;

    State state;
    // Fill the runtime-configured number of slots so the pool is full.
    for (uint32_t i = 0; i < sd.projectilePoolSize; ++i)
    {
        state.slots[i].spawnTick = 50u + i;   // distinct, nonzero, < currentTick
        state.slots[i].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
        state.slots[i].endTick   = 0u;        // alive
    }

    InitialConditions ic;
    ic.spawnRequestPending = 1;
    ic.spawnPos = glm::vec3(1.f, 2.f, 3.f);
    ic.spawnDir = glm::vec3(0.f, 1.f, 0.f);

    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    // Request must be cleared even when dropped.
    REQUIRE(ic.spawnRequestPending == 0u);

    // All slots must remain alive with their original spawnTick (R-T5: never revised).
    for (uint32_t i = 0; i < sd.projectilePoolSize; ++i)
    {
        REQUIRE(state.slots[i].spawnTick == 50u + i);
        REQUIRE(state.slots[i].isAlive(currentTick));
    }
}

// ---------------------------------------------------------------------------
// Test: the runtime StaticData::projectilePoolSize field (R-P1) gates the active
// pool — with projectilePoolSize=1 only slot 0 is usable, so a second spawn drops
// even though the compile-time capacity (kMaxProjectilePoolSize) is larger.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.RuntimePoolSizeGatesActiveSlots", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    static_assert(kMaxProjectilePoolSize >= 2,
        "this test needs capacity > 1 to prove the runtime field, not capacity, is the gate");

    auto sd = makeStaticData();
    sd.projectilePoolSize = 1;   // runtime cap below compile-time capacity

    State state;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    // First spawn fills the single active slot.
    InitialConditions ic;
    ic.spawnRequestPending = 1;
    ic.spawnPos = glm::vec3(1.f, 0.f, 0.f);
    ic.spawnDir = glm::vec3(1.f, 0.f, 0.f);
    const uint32_t firstTick = 100u;
    callIntegrate(ic, state, derived, physics, query, sd, firstTick);
    REQUIRE(state.slots[0].isAlive(firstTick));

    // Second spawn must drop — the only active slot is taken; slot 1 stays free
    // despite physical capacity for it.
    ic.spawnRequestPending = 1;
    const uint32_t secondTick = firstTick + 1u;
    callIntegrate(ic, state, derived, physics, query, sd, secondTick);
    REQUIRE(ic.spawnRequestPending == 0u);
    REQUIRE_FALSE(state.slots[1].isAlive(secondTick));
    REQUIRE(state.slots[1].spawnTick == 0u);
}

// ---------------------------------------------------------------------------
// Test: a slot whose elapsed lifetime reaches maxLifetime ends with
// endReason == 1 (lifetimeExpired) and the body is parked below the z-plane.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.LifetimeExpiryKillsSlot", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    const float maxLifetime = 1.f;
    auto sd = makeStaticData(500.f, maxLifetime);

    // Spawned at tick 100; after 60 ticks elapsed = 60 * (1/60) = 1.0 s >= maxLifetime.
    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 60u;

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;
    DerivedState derived;

    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    REQUIRE(state.slots[0].endReason == 1u);
    REQUIRE(state.slots[0].endTick == currentTick);
    REQUIRE_FALSE(state.slots[0].isAlive(currentTick));

    const glm::vec3 parkPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(parkPos.z < -1000.f);
    REQUIRE(physics.bodies[0].linearVelocity == glm::vec3(0.f));

    // Lifetime expiry does not generate a derived hit.
    REQUIRE(derived.hits.empty());
}

// ---------------------------------------------------------------------------
// Test: overlap query reports a hit on a non-parent body — slot ends with
// endReason == 2 (hit), hitObjectIndex recorded, body parked, derived.hits
// populated.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.HitOnNonParentKillsSlot", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);

    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 6u;   // elapsed 0.1 s, well within lifetime

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    // A hit whose bodyId is NOT the parent. Categorised as a plain body hit (T29 —
    // the mock now populates objectCategories so the guard/body classifier routes it
    // to derived.hits, not derived.blocks).
    SpatialQueryHit hit{};
    hit.objectIndex    = 42;
    hit.objectPosition = glm::vec3(5.f, 5.f, 0.f);
    hit.bodyId         = BodyId{ 99 };
    hit.objectCategories = CollisionCategories::single(collisionCategory::body);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    REQUIRE(state.slots[0].endReason == 2u);
    REQUIRE(state.slots[0].endTick == currentTick);
    REQUIRE(state.slots[0].hitObjectIndex == 42);
    REQUIRE_FALSE(state.slots[0].isAlive(currentTick));

    const glm::vec3 parkPos = glm::vec3(physics.bodies[0].transform[3]);
    REQUIRE(parkPos.z < -1000.f);
    REQUIRE(physics.bodies[0].linearVelocity == glm::vec3(0.f));

    // Body hit routes to damage hits, not blocks.
    REQUIRE(derived.hits.size() == 1u);
    REQUIRE(derived.hits[0].objectIndex == 42);
    REQUIRE(derived.hits[0].tickStamp == currentTick);
    REQUIRE(derived.blocks.empty());
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

    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 6u;

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    SpatialQueryHit parentHit{};
    parentHit.objectIndex    = 7;
    parentHit.objectPosition = glm::vec3(0.f);
    parentHit.bodyId         = BodyId{ kParentBodyIdValue };
    query.nextReport.hits.push_back(parentHit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    REQUIRE(state.slots[0].isAlive(currentTick));
    REQUIRE(state.slots[0].endReason == 0u);
    REQUIRE(state.slots[0].endTick == 0u);
    REQUIRE(derived.hits.empty());
}

// ---------------------------------------------------------------------------
// T29 — Test: projectile hits a GUARD body whose forward axis faces the incoming
// projectile (within the front block cone). Expect a BLOCK: derivedState.blocks
// gets the entry, derivedState.hits stays empty, slot killed.
//
// Geometry: projectile spawnDir = +X (travelling +X). Incoming direction into the
// guard = -spawnDir = -X. Guard forward axis (first column of its body transform) is
// set to -X, so the guard faces the projectile. angle(guardForward, incoming) =
// angle(-X, -X) = 0 <= guardMiddleSectionHalfAngle (default 0.25 rad) → block
// (perfectly centred guard alignment is well inside the narrow middle section).
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.BlockOnGuardFront", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);   // guardMiddleSectionHalfAngle defaults to 0.25 rad

    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 6u;

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);   // +X travel
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    // Need an extra body for the guard (slot bodies 0..N-1, parent = N).
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 2 };
    MockSpatialQueryAdapter query;

    // Guard body lives at index kMaxProjectilePoolSize + 1 (distinct from parent).
    const uint32_t guardBodyId = kMaxProjectilePoolSize + 1;
    glm::mat4 guardXform(1.f);
    guardXform[0] = glm::vec4(-1.f, 0.f, 0.f, 0.f);   // forward axis = -X (faces +X projectile)
    physics.bodies[guardBodyId].transform = guardXform;

    SpatialQueryHit hit{};
    hit.objectIndex      = 7;
    hit.objectPosition   = glm::vec3(5.f, 0.f, 0.f);
    hit.bodyId           = BodyId{ guardBodyId };
    hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    // Slot is killed regardless of block/damage.
    REQUIRE(state.slots[0].endReason == 2u);
    REQUIRE(state.slots[0].endTick == currentTick);
    REQUIRE_FALSE(state.slots[0].isAlive(currentTick));

    // Routed to blocks, NOT hits.
    REQUIRE(derived.blocks.size() == 1u);
    REQUIRE(derived.blocks[0].objectIndex == 7);
    REQUIRE(derived.hits.empty());
}

// ---------------------------------------------------------------------------
// T29 — Test: projectile hits a GUARD body whose forward axis faces AWAY from the
// incoming projectile (outside the front block cone). Expect a DAMAGE hit: the guard
// didn't cover this angle, so it routes to derivedState.hits, not blocks.
//
// Geometry: projectile spawnDir = +X. Incoming = -X. Guard forward axis = +X (guard
// facing away). angle(+X, -X) = pi > guardMiddleSectionHalfAngle (0.25 rad) — and also
// outside the outer pi/2 cone — → damage hit.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.DamageOnGuardBack", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);

    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 6u;

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 2 };
    MockSpatialQueryAdapter query;

    const uint32_t guardBodyId = kMaxProjectilePoolSize + 1;
    glm::mat4 guardXform(1.f);
    guardXform[0] = glm::vec4(1.f, 0.f, 0.f, 0.f);   // forward axis = +X (faces away from -X incoming)
    physics.bodies[guardBodyId].transform = guardXform;

    SpatialQueryHit hit{};
    hit.objectIndex      = 9;
    hit.objectPosition   = glm::vec3(5.f, 0.f, 0.f);
    hit.bodyId           = BodyId{ guardBodyId };
    hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    REQUIRE(state.slots[0].endReason == 2u);
    REQUIRE(state.slots[0].endTick == currentTick);
    REQUIRE_FALSE(state.slots[0].isAlive(currentTick));

    // Outside the cone → damage hit, not a block.
    REQUIRE(derived.hits.size() == 1u);
    REQUIRE(derived.hits[0].objectIndex == 9);
    REQUIRE(derived.blocks.empty());
}

// ---------------------------------------------------------------------------
// T31 — Test: projectile hits a GUARD body whose forward axis is ~pi/4 off the
// incoming projectile direction. This is INSIDE the radial's old outer cone
// (pi/4 < pi/2) but OUTSIDE the new MIDDLE section (pi/4 > 0.25 rad). With the
// projectile block threshold narrowed to the middle section, the guard no longer
// covers this angle → DAMAGE hit, not a block.
//
// Geometry: guard forward axis = +X. Projectile spawnDir = normalize(-1,0,-1), so
// incoming = -spawnDir = normalize(+1,0,+1). angle(+X, incoming) = acos(1/sqrt2) =
// pi/4 ≈ 0.785 rad. 0.785 > 0.25 (middle) → blocked == false → routes to hits.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.DamageOnGuardSide", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);   // guardMiddleSectionHalfAngle defaults to 0.25 rad

    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 6u;

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    // incoming = -spawnDir = normalize(+1,0,+1) → pi/4 off the guard forward (+X).
    state.slots[0].spawnDir  = glm::normalize(glm::vec3(-1.f, 0.f, -1.f));
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 2 };
    MockSpatialQueryAdapter query;

    const uint32_t guardBodyId = kMaxProjectilePoolSize + 1;
    glm::mat4 guardXform(1.f);
    guardXform[0] = glm::vec4(1.f, 0.f, 0.f, 0.f);   // forward axis = +X (toward shooter side)
    physics.bodies[guardBodyId].transform = guardXform;

    SpatialQueryHit hit{};
    hit.objectIndex      = 11;
    hit.objectPosition   = glm::vec3(5.f, 0.f, 0.f);
    hit.bodyId           = BodyId{ guardBodyId };
    hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    REQUIRE(state.slots[0].endReason == 2u);
    REQUIRE(state.slots[0].endTick == currentTick);
    REQUIRE_FALSE(state.slots[0].isAlive(currentTick));

    // Inside the outer cone but outside the middle section → damage hit, not a block.
    REQUIRE(derived.hits.size() == 1u);
    REQUIRE(derived.hits[0].objectIndex == 11);
    REQUIRE(derived.blocks.empty());
}

// ---------------------------------------------------------------------------
// T30 — Test: a blocked projectile's indicator is placed on the target's inner
// circle at the edge FACING the shooter (ray-vs-circle intersection), NOT at the
// character root.
//
// Geometry: projectile spawned at (-500,0,50) travelling +X; target character
// (guard body) at origin; innerCircleRadius = 100. The launch ray crosses the
// circle first at x = -100 (the near side). Expect blocks[0].position.x ≈ -100,
// definitively NOT the body centre (x ≈ 0).
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.BlockIndicatorAtInnerCircle", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 3.f);   // guardMiddleSectionHalfAngle defaults to 0.25 rad
    sd.innerCircleRadius = 100.f;

    const uint32_t spawnTick   = 100u;
    const uint32_t currentTick = spawnTick + 6u;

    State state;
    state.slots[0].spawnTick = spawnTick;
    state.slots[0].spawnPos  = glm::vec3(-500.f, 0.f, 50.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);   // +X travel
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 2 };
    MockSpatialQueryAdapter query;

    // Guard body at origin, forward axis -X (faces the incoming +X projectile → block).
    const uint32_t guardBodyId = kMaxProjectilePoolSize + 1;
    glm::mat4 guardXform(1.f);
    guardXform[0] = glm::vec4(-1.f, 0.f, 0.f, 0.f);   // forward axis = -X
    guardXform[3] = glm::vec4(0.f, 0.f, 0.f, 1.f);    // character root at origin
    physics.bodies[guardBodyId].transform = guardXform;

    SpatialQueryHit hit{};
    hit.objectIndex      = 7;
    hit.objectPosition   = glm::vec3(0.f, 0.f, 0.f);   // body centre — the WRONG place for the marker
    hit.bodyId           = BodyId{ guardBodyId };
    hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, currentTick);

    REQUIRE(derived.blocks.size() == 1u);
    // Entry point on the inner circle facing the shooter: x = -100, NOT the body centre 0.
    REQUIRE(derived.blocks[0].position.x == Catch::Approx(-100.f));
    REQUIRE(derived.blocks[0].position.x != Catch::Approx(0.f));
    REQUIRE(derived.blocks[0].objectIndex == 7);
    REQUIRE(derived.hits.empty());
}

// ---------------------------------------------------------------------------
// T30 — Test: a damage-hit indicator persists for exactly indicatorPersistTicks
// ticks, then integrate prunes it.
//
// Hit fires at tick 100 with indicatorPersistTicks = 5. Ticks 100–104 → size 1;
// tick 105 → pruned (size 0). The slot is dead after the hit tick, so no entry is
// re-added; only the persist/prune behaviour is exercised.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.HitIndicatorPersistsForPersistTicks", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 100.f);   // big lifetime so it never expires here
    sd.indicatorPersistTicks = 5u;

    const uint32_t hitTick = 100u;

    State state;
    state.slots[0].spawnTick = hitTick;       // alive on the hit tick
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    SpatialQueryHit hit{};
    hit.objectIndex      = 42;
    hit.objectPosition   = glm::vec3(5.f, 0.f, 0.f);
    hit.bodyId           = BodyId{ 99 };
    hit.objectCategories = CollisionCategories::single(collisionCategory::body);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;

    // Hit lands at tick 100 → entry present.
    callIntegrate(ic, state, derived, physics, query, sd, hitTick);
    REQUIRE(derived.hits.size() == 1u);
    REQUIRE(derived.hits[0].tickStamp == hitTick);

    // Ticks 101–104: still within the window (104 - 100 = 4 < 5) → persists.
    for (uint32_t t = hitTick + 1u; t <= hitTick + 4u; ++t)
    {
        callIntegrate(ic, state, derived, physics, query, sd, t);
        REQUIRE(derived.hits.size() == 1u);
    }

    // Tick 105: 105 - 100 = 5 >= 5 → pruned.
    callIntegrate(ic, state, derived, physics, query, sd, hitTick + 5u);
    REQUIRE(derived.hits.empty());
}

// ---------------------------------------------------------------------------
// T30 — Test: a block indicator persists for exactly indicatorPersistTicks ticks,
// then integrate prunes it. Same shape as the hit-persistence test but the hit
// lands on a guard facing the shooter (block).
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.BlockIndicatorPersistsForPersistTicks", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 100.f);
    sd.indicatorPersistTicks = 5u;
    sd.innerCircleRadius = 100.f;

    const uint32_t hitTick = 100u;

    State state;
    state.slots[0].spawnTick = hitTick;
    state.slots[0].spawnPos  = glm::vec3(-500.f, 0.f, 0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 2 };
    MockSpatialQueryAdapter query;

    const uint32_t guardBodyId = kMaxProjectilePoolSize + 1;
    glm::mat4 guardXform(1.f);
    guardXform[0] = glm::vec4(-1.f, 0.f, 0.f, 0.f);   // faces the shooter → block
    physics.bodies[guardBodyId].transform = guardXform;

    SpatialQueryHit hit{};
    hit.objectIndex      = 7;
    hit.objectPosition   = glm::vec3(0.f, 0.f, 0.f);
    hit.bodyId           = BodyId{ guardBodyId };
    hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
    query.nextReport.hits.push_back(hit);

    DerivedState derived;

    callIntegrate(ic, state, derived, physics, query, sd, hitTick);
    REQUIRE(derived.blocks.size() == 1u);
    REQUIRE(derived.blocks[0].tickStamp == hitTick);

    for (uint32_t t = hitTick + 1u; t <= hitTick + 4u; ++t)
    {
        callIntegrate(ic, state, derived, physics, query, sd, t);
        REQUIRE(derived.blocks.size() == 1u);
    }

    callIntegrate(ic, state, derived, physics, query, sd, hitTick + 5u);
    REQUIRE(derived.blocks.empty());
}

// ---------------------------------------------------------------------------
// T30 — Test: two slots that hit different targets within the persist window
// both keep their indicator entries; the entries coexist in derived.hits.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.MultipleIndicatorsAccumulate", "[BrawlerProjectile]")
{
    using namespace projectiletests;
    using namespace brawlerProjectileSimulation;

    auto sd = makeStaticData(500.f, 100.f);   // default indicatorPersistTicks (20) — wide window

    const uint32_t firstTick = 100u;

    State state;
    // Slot 0 is alive and hits on the first tick. Slot 1 stays free for now.
    state.slots[0].spawnTick = firstTick;
    state.slots[0].spawnPos  = glm::vec3(0.f);
    state.slots[0].spawnDir  = glm::vec3(1.f, 0.f, 0.f);
    state.slots[0].endTick   = 0u;

    InitialConditions ic;
    MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 1 };
    MockSpatialQueryAdapter query;

    SpatialQueryHit hitA{};
    hitA.objectIndex      = 1;
    hitA.objectPosition   = glm::vec3(5.f, 0.f, 0.f);
    hitA.bodyId           = BodyId{ 90 };
    hitA.objectCategories = CollisionCategories::single(collisionCategory::body);
    query.nextReport.hits.clear();
    query.nextReport.hits.push_back(hitA);

    DerivedState derived;
    callIntegrate(ic, state, derived, physics, query, sd, firstTick);
    REQUIRE(derived.hits.size() == 1u);

    // Now bring slot 1 alive (spawnTick set just before its hit tick so it is not
    // "alive" on the first tick) and inject a DIFFERENT target.
    state.slots[1].spawnTick = firstTick + 1u;
    state.slots[1].spawnPos  = glm::vec3(0.f, 10.f, 0.f);
    state.slots[1].spawnDir  = glm::vec3(0.f, 1.f, 0.f);
    state.slots[1].endTick   = 0u;

    SpatialQueryHit hitB{};
    hitB.objectIndex      = 2;
    hitB.objectPosition   = glm::vec3(0.f, 15.f, 0.f);
    hitB.bodyId           = BodyId{ 91 };
    hitB.objectCategories = CollisionCategories::single(collisionCategory::body);
    query.nextReport.hits.clear();
    query.nextReport.hits.push_back(hitB);

    callIntegrate(ic, state, derived, physics, query, sd, firstTick + 1u);

    // Slot 0's entry (tick 100) is still inside the 20-tick window; slot 1's entry
    // (tick 101) was just added → both coexist.
    REQUIRE(derived.hits.size() == 2u);
    bool sawTarget1 = false, sawTarget2 = false;
    for (const auto& h : derived.hits)
    {
        if (h.objectIndex == 1) sawTarget1 = true;
        if (h.objectIndex == 2) sawTarget2 = true;
    }
    REQUIRE(sawTarget1);
    REQUIRE(sawTarget2);
}

// ---------------------------------------------------------------------------
// Test: wire footprint of the closed-form representation. The transient
// bodyState is excluded from the wire, so each slot is 37 B and the SIM_VECTOR
// State capacity is 4 + kMaxProjectilePoolSize * 37 B. T14 sizes the state sync
// buffer off this number.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerProjectile.WireFootprint", "[BrawlerProjectile]")
{
    using namespace brawlerProjectileSimulation;

    // Per-slot: 4 (spawnTick) + 12 (spawnPos) + 12 (spawnDir)
    //         + 4 (endTick) + 1 (endReason) + 4 (hitObjectIndex) = 37.
    constexpr std::uint32_t slotSize  = syncSize<ProjectileSlot>();
    constexpr std::uint32_t stateSize = syncSize<State>();

    static_assert(slotSize == 37u, "closed-form ProjectileSlot wire size drifted");
    static_assert(stateSize == sizeof(std::uint32_t) + kMaxProjectilePoolSize * 37u,
                  "State SIM_VECTOR capacity drifted");

    REQUIRE(slotSize == 37u);
    REQUIRE(stateSize == 4u + kMaxProjectilePoolSize * 37u);   // 115 B at capacity 3

    WARN("brawlerProjectileSimulation wire footprint: slot=" << slotSize
         << " B, State(SIM_VECTOR cap " << kMaxProjectilePoolSize << ")=" << stateSize << " B");
}

#endif // WITH_LOW_LEVEL_TESTS
