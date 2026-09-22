// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================
// THE HIT-ROUTING SYSTEM IS THE SINGLE RESOLVER  [movement-sim task 27]
//
// SUBJECT: `brawlerHitRouting::System::postIntegrate` -- specifically the part task 27 added. The
// routing pass already turned a radial overlap or a projectile impact into
// `wasHitThisTick` on the struck character's off-wire slice; it now turns it into the WHOLE
// reaction: kind, speed, direction and the lockout dwell.
//
// ⭐⭐ WHY HERE AND NOWHERE ELSE, and it is an ownership argument rather than a convenience.
// The dwell for a knockback is `max(lockoutDuration, knockbackSpeed / launchDecel)`. The machine
// cannot compute it -- `launchDecel` lives in `brawlerMovementSimulation::StaticData`, which the
// machine's `IntegrationUtils` does not receive, and task 16 turns such literals into ONE-TIME
// cvar reads, so a typed constant beside `kHitFlinchDuration` would be right on the day it shipped
// and silently wrong the day a cvar moved. The movement sim cannot compute it either: it never
// sees the attack table. The routing system is the ONE actor that already holds both, because
// `postIntegrate` receives the composite `simulatableBrawler::StaticData`. Resolving it here means
// NEITHER sub-simulation learns the other's constants.
//
// ⛔ THESE CASES DRIVE THE SYSTEM, NOT A COPY OF ITS ARITHMETIC. `resolveHitReaction` is a
// private static; every expectation below is built from the SHIPPED `HitReactionSpec` rows and the
// SHIPPED `launchDecel`, read off the same `simulatableBrawler::StaticData` the system is handed,
// so a retune moves both sides together and a case never asserts its own copy of a constant.
//
// TAGS: `[SimulatableBrawler]` is deliberate and it is NOT decoration -- it is the tag `[@og]`
// already whitelists (`OgTagAliases.cpp`), so these cases move the suite count because they are
// new cases and not because the filter was widened to admit them.
// ============================================================================

#include <memory>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/StorageView.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

#include "glm/geometric.hpp"

namespace hitRoutingTests
{

constexpr float kDt = 1.f / 60.f;

// Two registered characters, the routing system, and the two handles a case needs to pose a hit:
// the attacker's radial DerivedState (where `attackHits[]` lives) and the target's inbound slice.
//
// ⚠ THE STORAGE IS THE PRODUCTION ONE. `brawlerHitRouting::System` takes a
// `StorageView<SimulatableBrawler>` and sorts its own snapshot by ascending id (D4), so a rig that
// handed it a hand-built vector would be testing a different walk order from the one that ships.
struct FRoutingRig
{
    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    brawlerHitRouting::System system;

    FRoutingRig()
    {
        storage.add<SimulatableBrawler>(0u, SimulatableBrawler(staticData));
        storage.add<SimulatableBrawler>(1u, SimulatableBrawler(staticData));
        brawler(0u).setCharacterBindings({ BodyId{ 10u } });
        brawler(1u).setCharacterBindings({ BodyId{ 11u } });
        system.onCharacterRegistered(0u, view(), staticData);
        system.onCharacterRegistered(1u, view(), staticData);
    }

    // The production projection, not a hand-built view: `StorageView`'s constructor is private
    // and `projectTo` is the only mint, which is exactly the property that makes this rig hand the
    // system the same object the executor does.
    StorageView<SimulatableBrawler> view()
    { return storage.projectTo<SimulatableList<SimulatableBrawler>>(); }

    SimulatableBrawler& brawler(unsigned int id) { return storage.get<SimulatableBrawler>(id); }

    BodyId rootBodyId(unsigned int id) { return brawler(id).getCharacterBindings().capsuleBodyId; }

    void setPosition(unsigned int id, glm::vec3 p)
    {
        brawler(id).editAllState().editState()
            .edit<brawlerMovementSimulation::State>().bodyState.position = p;
    }

    // The attacker's wire sequence id -- the thing the resolver indexes the reaction table with.
    void setSequence(unsigned int id, unsigned int sequenceId)
    {
        brawler(id).editAllState().editState()
            .edit<dAttackRadialSimulation::State>().currenSequenceId = sequenceId;
    }

    // The attacker's wire aim, as `setRadialSimulationInitialConditions` writes it: an angle from
    // +X about an axis that is +Z or -Z. This is the ONLY aim on the wire, and it is the resolver's
    // fallback when the two body positions coincide exactly.
    void setAim(unsigned int id, float angleRadians, float axisZ)
    {
        auto& ic = brawler(id).editAllState().editState()
            .edit<dAttackRadialSimulation::InitialConditions>();
        ic.initialAimAngle = angleRadians;
        ic.initialAimRotationAxis = glm::vec3(0.f, 0.f, axisZ);
    }

    // Raise a radial DAMAGING hit from `attackerId` onto `targetId`, the way
    // `dAttackRadialSimulation::collisionCheck` does.
    //
    // ⭐ [movement-sim task 83] BOTH CONTAINERS, and the default tangent is the ZERO VECTOR.
    // `collisionCheck` now records every accepted hit twice -- in `attackHits`, the per-SWING
    // dedup ledger, and in `hitsThisTick`, the one-tick signal routing consumes -- so a rig
    // that posed only one of them would be posing a state the simulation cannot produce.
    // The zero tangent is not laziness either: it is the DEGENERATE case, and it is what
    // keeps the two pre-task-83 direction cases below meaningful. They assert the
    // away-from-attacker rule, which is now the FALLBACK rather than the primary rule, and
    // a zero tangent is exactly the input that selects it.
    void raiseRadialHit(unsigned int attackerId, unsigned int targetId,
                        glm::vec3 swingTangent = glm::vec3(0.f))
    {
        dAttackRadialSimulation::DAttackHit hit{};
        hit.position = glm::vec3(0.f);
        hit.hitRootBodyId = rootBodyId(targetId);
        hit.swingTangent = swingTangent;
        auto& derived = brawler(attackerId).editAllState().editDerivedState()
            .edit<dAttackRadialSimulation::DerivedState>();
        derived.editAttackHits().push_back(hit);
        derived.editHitsThisTick().push_back(hit);
    }

    void clearRadialHits(unsigned int attackerId)
    {
        auto& derived = brawler(attackerId).editAllState().editDerivedState()
            .edit<dAttackRadialSimulation::DerivedState>();
        derived.editAttackHits().clear();
        derived.editHitsThisTick().clear();
    }

    // End a projectile slot on `tick` with endReason 2 (hit), the way the projectile sim does.
    void raiseProjectileHit(unsigned int attackerId, unsigned int targetId,
                            std::uint32_t tick, glm::vec3 spawnDir)
    {
        auto& slot = brawler(attackerId).editAllState().editState()
            .edit<brawlerProjectileSimulation::State>().slots[0];
        slot.spawnTick = 1u;
        slot.spawnDir  = spawnDir;
        slot.endTick   = tick;
        slot.endReason = 2;
        slot.hitRootBodyId = rootBodyId(targetId);
    }

    void route(std::uint32_t tick)
    {
        system.postIntegrate(SimulationTimeStep(tick, false, false, false, kDt),
                             view(), staticData);
    }

    const brawlerInboundHit::DerivedState& inbound(unsigned int id)
    {
        return brawler(id).getAllState().getDerivedState()
            .get<brawlerInboundHit::DerivedState>();
    }

    float launchDecel() const { return staticData.m_movementStaticData.launchDecel; }
};


// ===========================================================================
// [movement-sim task 83] THE END-TO-END HARNESS -- A REAL SWING, THE REAL ROUTING PASS, AND
// THE TARGET'S OWN MACHINE AND MOVEMENT SUB-SIMULATIONS.
//
// ⭐⭐ WHY IT HAD TO BE BUILT, and it is the lesson this task exists to record. The rig above
// POSES a hit: it writes one entry into the attacker's radial DerivedState and routes one
// tick. Task 27's LLTs did the same, and the defect the user found at PIE is a property of
// how long that entry LIVES -- which a rig that supplies it by hand cannot have an opinion
// about. The rig's FIDELITY, not the assertion, bounded what the suite could catch. So this
// harness supplies nothing: the attacker's own collisionCheck registers the hit, the
// attacker's own integrate decides when the swing ends, and the routing pass is asked the
// same question on every one of those ticks.
//
// ⚠ AND HERE IS WHAT *THIS* RIG CANNOT SEE, said out loud for the same reason:
//   * ONE ATTACKER AND ONE TARGET. Nothing here exercises the <= 4 distinct targets cap, the
//     deterministic attacker walk order (D4), or two attackers hitting one target on the
//     same tick.
//   * NO ROLLBACK. Every tick is a fresh forward tick. The replay-dedup hazard (design
//     section 4) -- attackHits is derived, is not restored on a resim, and a replay anchored
//     before the hit tick registers nothing -- is INVISIBLE here and stays carried, not fixed.
//   * THE WEAPON NEVER ROTATES. The physics mock returns the identity for every body, so the
//     swing's segment never advances. That is what makes the hit tick a fixture choice
//     instead of a geometry, and it means this harness cannot say anything about a hit that
//     lands as the swing passes from one segment to the next.
//   * THE ATTACKER'S POSITION IS FROZEN and the target's overlap position is scripted, so a
//     direction re-resolved from MOVING positions -- the third side effect of the defect --
//     shows up here only as the hit count, never as a curve.
// ===========================================================================

namespace endToEnd
{

constexpr std::uint32_t kAttackerRoot    = 20u;
constexpr std::uint32_t kTargetRoot      = 21u;
constexpr std::uint32_t kRadialVolume    = 7u;
constexpr std::uint32_t kMovementVolume  = 91u;

// [movement-sim task 87] THE PROJECTILE POOL'S ids, one body and one query volume per slot.
// Distinct from every id above for the same reason the two gates in the adapter exist: ten
// sub-simulation instances reach the adapter per tick and a shared id would hand one sim's
// report to another. ⚠ INERT unless a case fires a Hadouken -- the projectile sim touches the
// adapter only for slots that are ALIVE, and a slot is alive only after a spawn request.
constexpr std::uint32_t kProjectileBody   = 31u;   // slots 0..2 -> 31, 32, 33
constexpr std::uint32_t kProjectileVolume = 61u;   // slots 0..2 -> 61, 62, 63
constexpr std::uint32_t kProjectileSlots  =
    static_cast<std::uint32_t>(brawlerProjectileSimulation::kMaxProjectilePoolSize);

// The capsule half height the production movement StaticData is authored against. Seating the
// target at halfHeight + rideHeight is the steady state: the vertical channel is then
// identically zero and the whole of the target's velocity is the knockback.
constexpr float kCapsuleHalfHeight = 96.f;

// The attacker's weapon sits at the origin for the whole run (see the mock), so a target 150 cm
// out along +X is inside the production annulus (inner 90, outer 300) and exactly on the swing
// plane. 0.30 s into the swing is 18 ticks; the design's arithmetic is quoted against it.
constexpr float        kTargetDistance = 150.f;
constexpr std::uint32_t kHitTick       = 18u;

// ⛔⛔ [movement-sim task 86] WHY THE RIG SCHEDULES THE OVERLAP AT ALL, and it is a real
// limitation of this fixture rather than a convenience.
//
// The radial's own damaging gate reads the WEAPON'S DIRECTION off the body transform
// (`collisionCheck` -> `getAttackSegment(currentDirection)`), NOT `attackTimer`. This rig's
// `SwingPhysicsAdapter::getBodyTransform` returns the IDENTITY on every tick, so the weapon
// never rotates and that gate is constant for a whole swing -- it cannot express a wind-up.
// `weaponOverlapsTarget` is the stand-in: it opens the query `windUpTicks` after a swing
// STARTS, which is where the weapon would have reached the target.
//
// ⚠ `kHitTick` IS THAT WIND-UP, and it is the rig's ONE authored number about the swing:
// sequences 0, 1 and 4 all open their Damaging keyframe at 0.30 s, which is 18 ticks at
// 60 Hz. ⛔ It is the offset from a swing's START, not an absolute tick -- a case that starts
// a SECOND swing gets the same 18 ticks of wind-up measured from that swing's own first tick.
// ⛔ Do NOT read it as a measured quantity: the fixture cannot measure it, which is exactly
// why it is spelled once, here, next to the reason.

struct SwingPhysicsAdapter
{
    glm::mat4 getBodyTransform(BodyId) const              { return glm::mat4(1.f); }
    void setBodyTransform(BodyId, const glm::mat4&)       {}
    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<SwingPhysicsAdapter>);

// ⛔ TWO GATES, NOT ONE. Ten sub-simulation instances reach this adapter per tick (five each on
// two characters). The overlap report must reach the ATTACKER's radial volume and nothing else
// -- the target's guard volume would otherwise see the same report -- and the ground sweep must
// reach the TARGET's movement volume and nothing else.
struct SwingQueryAdapter
{
    SpatialQueryReport radialReport;
    bool               weaponOverlapsTarget = false;
    SweepHit           ground;

    // ⭐⭐ [movement-sim task 87] THE PROJECTILE'S OVERLAP IS A DISTANCE TEST, NOT A SCHEDULE,
    // and that is the one way this arm is BETTER instrumented than the melee one above.
    // `kHitTick` has to be authored because the weapon never rotates here; the projectile has
    // no such problem -- it publishes its own closed-form position to the query layer on every
    // tick it is alive (`setVolumeParentTransform`, immediately before it asks for the overlap),
    // so the fixture can answer the overlap from the REAL separation and the hit tick comes out
    // MEASURED. `projectileContactRadius` is the rig's one authored number about it: the
    // projectile's own collider radius plus the target capsule's, i.e. the separation at which
    // the two shapes touch. Both are read off the shipped StaticData, never spelled here.
    SpatialQueryReport projectileReport;
    glm::vec2          targetXY{ 0.f };
    float              projectileContactRadius = 0.f;
    std::array<glm::vec3, kProjectileSlots> projectilePos{};

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        const bool attackerRadial =
            std::find(volumeIds.begin(), volumeIds.end(), QueryVolumeId{ kRadialVolume })
                != volumeIds.end();
        if (attackerRadial)
            return weaponOverlapsTarget ? radialReport : SpatialQueryReport{};

        // [movement-sim task 87] THE THIRD GATE. One volume per pool slot, answered from that
        // slot's last published position -- so a slot that is not alive (and therefore never
        // published one) cannot be answered at all, and every case written before this task
        // sees the same empty report it always did.
        for (std::uint32_t slot = 0u; slot < kProjectileSlots; ++slot)
        {
            if (std::find(volumeIds.begin(), volumeIds.end(),
                          QueryVolumeId{ kProjectileVolume + slot }) == volumeIds.end())
                continue;
            const glm::vec3& p = projectilePos[slot];
            return (glm::length(glm::vec2(p.x, p.y) - targetXY) <= projectileContactRadius)
                 ? projectileReport : SpatialQueryReport{};
        }
        return SpatialQueryReport{};
    }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4&, const glm::vec3&) const
    {
        return volumeId == QueryVolumeId{ kMovementVolume } ? ground : SweepHit{};
    }

    // [movement-sim task 87] WHERE THE PROJECTILE'S POSITION COMES FROM. The projectile sim
    // derives pos(t) from its closed form and publishes it here for every alive slot right
    // before querying that slot's overlap, so recording it is enough to answer the overlap from
    // the real separation. Nothing else in this fixture writes `projectilePos`.
    void setVolumeParentTransform(QueryVolumeId volumeId, const glm::mat4& transform)
    {
        for (std::uint32_t slot = 0u; slot < kProjectileSlots; ++slot)
            if (volumeId == QueryVolumeId{ kProjectileVolume + slot })
                projectilePos[slot] = glm::vec3(transform[3]);
    }
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<SwingQueryAdapter>);

// One tick's observation, recorded AFTER the routing pass -- which is where a reader of the
// slice stands, because routing owns the reset/set lifecycle.
struct TickSample
{
    std::uint32_t tick            = 0u;
    unsigned int  radialSequence  = InvalidAttackSequenceId;
    std::size_t   attackHits      = 0;   // the per-SWING ledger
    std::size_t   hitsThisTick    = 0;   // the per-TICK signal
    bool          wasHitThisTick  = false;
    DAttackState  targetMachine   = DAttackState::Idle;
    // [movement-sim task 86] The ATTACKER's machine state, so a case can see the swing end,
    // the drop to Idle and the fresh swing as three separate observations rather than
    // inferring them from the radial sequence id alone.
    DAttackState  attackerMachine = DAttackState::Idle;
    glm::vec2     hitDirectionXY{ 0.f };
    glm::vec3     targetVelocity{ 0.f };
    glm::vec3     targetPosition{ 0.f };
    // [movement-sim task 87] The attacker's projectile slot 0, so the shot is observable as it
    // travels and ends rather than only through the hit it routes. Identically zero on every
    // case that fires none.
    glm::vec3     projectilePos{ 0.f };
    std::uint32_t projectileSpawnTick = 0u;
    std::uint32_t projectileEndTick   = 0u;
    unsigned int  projectileEndReason = 0u;
};

struct FEndToEndRig
{
    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    brawlerHitRouting::System system;
    SwingPhysicsAdapter phys;
    SwingQueryAdapter   query;

    glm::vec2 attackerStick{ 0.f, -1.f };   // (0,-1) against aim (1,0,0) -> the right swing

    // [movement-sim task 86] THE FOLLOW-UP PRESS. -1 == off, which is every case written
    // before this task, so none of them changes the input it drives.
    int       followUpHoldFromTick = -1;

    // [movement-sim task 87] THE HADOUKEN. -1 == off; a tick number makes the attacker's
    // machine see `triggeredActionId == kHadoukenActionId` on exactly that tick, which is what
    // the shipped input layer's motion matcher delivers when a motion completes. On that tick
    // the attack BUTTONS are left alone, so the projectile is the only thing that fires.
    // ⛔ IT CANNOT BE TICK 0. `ProjectileSlot::isAlive` reads `spawnTick != 0` as "this slot
    // never spawned", so a projectile spawned on tick 0 is born free and never queries an
    // overlap. That is a property of the shipped slot encoding, not of this fixture.
    int       hadoukenOnTick = -1;
    glm::vec2 followUpStick{ 0.f, -1.f };   // -> kRightSequenceId (0), a left/right swing
    std::vector<TickSample> samples;

    FEndToEndRig()
    {
        // ⛔ THE PASSENGER ARM, AND IT IS A RIG CHOICE WITH A REASON. `drivesBody` ships TRUE;
        // under it step 5 sets kFlagHasCommand, which un-gates step 6', which then reads a
        // push-out that exists only because this rig has no solver. `engineStep()` below IS
        // this rig's engine, and it integrates the command exactly, so the passenger arm is
        // the honest configuration. Nothing this task changes lives in step 5 or 6'.
        staticData.m_movementStaticData.drivesBody = false;

        storage.add<SimulatableBrawler>(0u, SimulatableBrawler(staticData));
        storage.add<SimulatableBrawler>(1u, SimulatableBrawler(staticData));
        brawler(0u).setCharacterBindings({ BodyId{ kAttackerRoot } });
        brawler(1u).setCharacterBindings({ BodyId{ kTargetRoot } });

        brawler(0u).editPhysicsComposite()
            .edit<dAttackRadialSimulation::PhysicsDeclaration>()
            .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };

        auto& targetMovement = brawler(1u).editPhysicsComposite()
            .edit<brawlerMovementSimulation::PhysicsDeclaration>().bindings;
        targetMovement.ownBodyId      = BodyId{ kTargetRoot };
        targetMovement.parentBodyId   = BodyId{ kTargetRoot };
        targetMovement.queryVolumeIds = { QueryVolumeId{ kMovementVolume } };

        system.onCharacterRegistered(0u, view(), staticData);
        system.onCharacterRegistered(1u, view(), staticData);

        // FLAT GROUND AT EXACTLY RIDE HEIGHT, scripted rather than swept: an infinite flat
        // floor gives a clearance that does not depend on where the capsule slid to, so a
        // constant hit is not a simplification here, it is the geometry.
        const float probeLength = staticData.m_movementStaticData.rideHeight
                                + staticData.m_movementStaticData.snapDistance;
        query.ground.blocked     = true;
        query.ground.fraction    = staticData.m_movementStaticData.rideHeight / probeLength;
        query.ground.normal      = glm::vec3(0.f, 0.f, 1.f);
        query.ground.impactPoint =
            glm::vec3(0.f, 0.f, -staticData.m_movementStaticData.rideHeight);

        const float seatZ = kCapsuleHalfHeight + staticData.m_movementStaticData.rideHeight;
        setPosition(0u, glm::vec3(0.f, 0.f, seatZ));
        setPosition(1u, glm::vec3(kTargetDistance, 0.f, seatZ));

        SpatialQueryHit hit{};
        hit.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
        hit.bodyId           = BodyId{ kTargetRoot };
        hit.rootBodyId       = BodyId{ kTargetRoot };
        hit.objectCategories = CollisionCategories::single(collisionCategory::body);
        query.radialReport.hits.push_back(hit);

        // [movement-sim task 87] THE PROJECTILE POOL, bound for ALL THREE slots rather than
        // just the one a Hadouken lands in: the pool picks the lowest FREE slot itself, and a
        // fixture that bound only slot 0 would be quietly deciding that for it.
        // ⚠ Binding is INERT on its own -- these structs are only read for an alive slot.
        auto bindSlot = [](auto& declaration, std::uint32_t slot)
        {
            declaration.bindings.ownBodyId      = BodyId{ kProjectileBody + slot };
            declaration.bindings.parentBodyId   = BodyId{ kAttackerRoot };
            declaration.bindings.queryVolumeIds = { QueryVolumeId{ kProjectileVolume + slot } };
        };
        bindSlot(brawler(0u).editPhysicsComposite()
                     .edit<brawlerProjectileSimulation::PhysicsDeclaration<0>>(), 0u);
        bindSlot(brawler(0u).editPhysicsComposite()
                     .edit<brawlerProjectileSimulation::PhysicsDeclaration<1>>(), 1u);
        bindSlot(brawler(0u).editPhysicsComposite()
                     .edit<brawlerProjectileSimulation::PhysicsDeclaration<2>>(), 2u);

        // The separation at which the projectile's collider touches the target's capsule, both
        // read off the SHIPPED static data. This is the fixture's whole model of the projectile
        // overlap the engine would report, and it is the only authored number in the arm.
        query.projectileContactRadius = staticData.m_projectileStaticData.colliderRadius
                                      + staticData.m_movementStaticData.capsuleRadius;
        SpatialQueryHit projectileHit{};
        projectileHit.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
        projectileHit.bodyId           = BodyId{ kTargetRoot };
        projectileHit.rootBodyId       = BodyId{ kTargetRoot };
        projectileHit.objectCategories = CollisionCategories::single(collisionCategory::body);
        query.projectileReport.hits.push_back(projectileHit);
    }

    // [movement-sim task 87] MOVE THE TARGET, keeping the three places that have to agree in
    // step: its body position, the radial report's hit position (which the radial's own annulus
    // test reads) and the projectile report's. ⛔ The annulus is EXCLUSIVE at the inner radius
    // (`hitDistance > getInnerRadius()`), so a target seated exactly on it cannot be hit by a
    // swing at all -- which is what bounds how point-blank a FOLLOW-UP-able range can be.
    void setTargetDistance(float distance)
    {
        const float seatZ = kCapsuleHalfHeight + staticData.m_movementStaticData.rideHeight;
        setPosition(1u, glm::vec3(distance, 0.f, seatZ));
        query.radialReport.hits[0].objectPosition     = glm::vec3(distance, 0.f, 0.f);
        query.projectileReport.hits[0].objectPosition = glm::vec3(distance, 0.f, 0.f);
    }

    StorageView<SimulatableBrawler> view()
    { return storage.projectTo<SimulatableList<SimulatableBrawler>>(); }

    SimulatableBrawler& brawler(unsigned int id) { return storage.get<SimulatableBrawler>(id); }

    void setPosition(unsigned int id, glm::vec3 p)
    {
        brawler(id).editAllState().editState()
            .edit<brawlerMovementSimulation::State>().bodyState.position = p;
    }

    float knockbackSpeed() const
    { return staticData.m_hitReactions[dAttackDirection::kRightSequenceId].knockbackSpeed; }

    float launchDecel() const { return staticData.m_movementStaticData.launchDecel; }

    // v^2 / (2a) -- ruling #13's closed form, 500 cm at the shipped pair -- plus the exact
    // discretisation term of an N+1 tick linear ramp, v0*dt/2. 516.667 cm.
    float closedFormTravel() const
    {
        return knockbackSpeed() * knockbackSpeed() / (2.f * launchDecel())
             + knockbackSpeed() * kDt * 0.5f;
    }

    void run(int tickCount)
    {
        const glm::vec3 aim(1.f, 0.f, 0.f);
        // [movement-sim task 86] The swing the overlap schedule hangs off. -1 == no swing is
        // active, so the weapon reaches nothing. For every case that starts exactly one swing on
        // tick 0 this evaluates to `tick >= 18` on every tick -- what this rig did before.
        int          swingStartTick = -1;
        unsigned int lastSequence   = InvalidAttackSequenceId;

        for (int t = 0; t < tickCount; ++t)
        {
            const std::uint32_t tick = static_cast<std::uint32_t>(t);
            query.weaponOverlapsTarget =
                swingStartTick >= 0 && t >= swingStartTick + int(kHitTick);

            // The attack is pressed on tick 0 and RELEASED afterwards: held, it would queue
            // the chained sequence at 0.3 s and a second swing would confuse the measurement.
            //
            // ⭐ [movement-sim task 86] THE ONE EXCEPTION IS THE FOLLOW-UP PRESS, and it is
            // HELD rather than pulsed because held is what the shipped input path delivers.
            // `UOGBrawlerInputCollectionComponent::m_leftAttack` is a LATCHED LEVEL bool (set
            // from Enhanced Input's Triggered, cleared on Completed), `makeSimPlayerInput`
            // forwards it verbatim, and the machine's Idle entry is a LEVEL test --
            // `if (playerInput.attackLeft || playerInput.attackRight)`. A player who holds the
            // button through their own recovery therefore swings on the FIRST Idle tick, and
            // this rig must not be kinder to them than the game is.
            const bool followUp = followUpHoldFromTick >= 0 && t >= followUpHoldFromTick;
            // ⭐ [movement-sim task 87] THE HADOUKEN TICK PRESSES NOTHING. The machine's Idle
            // case handles the matched motion AHEAD of the attack buttons and `break`s, so a
            // press on that tick would be discarded anyway -- but leaving it out is what makes
            // the projectile the only thing the arm fires, and it keeps `(t == 0)` meaning
            // "the melee press" for every case written before this one.
            const bool firing   = hadoukenOnTick >= 0 && t == hadoukenOnTick;
            const bool pressing = !firing && ((t == 0 && hadoukenOnTick < 0) || followUp);
            const glm::vec2 stick = (t == 0) ? attackerStick
                                  : (followUp ? followUpStick : glm::vec2(0.f));
            const glm::vec3 moveWorld(stick.x, stick.y, 0.f);

            // [movement-sim task 87] The projectile's contact test needs the target's CURRENT
            // position and the adapter cannot see simulation state, so the rig hands it over
            // once per tick, BEFORE the integrate that will query it.
            {
                const glm::vec3 targetPos = brawler(1u).getAllState().getState()
                    .get<brawlerMovementSimulation::State>().bodyState.position;
                query.targetXY = glm::vec2(targetPos.x, targetPos.y);
            }

            const simulatableBrawler::PlayerInput attackerInput(
                dAttackRadialSimulation::PlayerInput(aim, pressing, false),
                // [movement-sim task 87] The 6th field is `triggeredActionId`, which the input
                // layer's motion matcher sets on the ONE tick a motion completes. Every case
                // before this task leaves `hadoukenOnTick` at -1 and therefore passes the 0 the
                // 5-arg form defaulted it to.
                dAttackMachineSimulation::PlayerInput{ aim, pressing, false, stick,
                                                       pressing ? moveWorld : glm::vec3(0.f),
                                                       firing ? inputSequence::kHadoukenActionId
                                                              : 0u },
                dAttackGuardSimulation::PlayerInput(aim),
                brawlerProjectileSimulation::PlayerInput{ aim },
                brawlerMovementSimulation::PlayerInput{},
                // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
                // composite. No field, no wire cost: the input composite is still 77 B and
                // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
                // every sub-sim name an InputType it OWNS.
                brawlerRingout::PlayerInput{});

            const simulatableBrawler::PlayerInput targetInput(
                dAttackRadialSimulation::PlayerInput(aim, false, false),
                dAttackMachineSimulation::PlayerInput{ aim, false, false,
                                                       glm::vec2(0.f), glm::vec3(0.f) },
                dAttackGuardSimulation::PlayerInput(aim),
                brawlerProjectileSimulation::PlayerInput{ aim },
                brawlerMovementSimulation::PlayerInput{},
                // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
                // composite. No field, no wire cost: the input composite is still 77 B and
                // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
                // every sub-sim name an InputType it OWNS.
                brawlerRingout::PlayerInput{});

            const SimulationTimeStep step(tick, false, false, false, kDt);
            brawler(0u).integrate(step, attackerInput, phys, query, staticData);
            brawler(1u).integrate(step, targetInput, phys, query, staticData);
            system.postIntegrate(step, view(), staticData);

            const auto& radialDerived = brawler(0u).getAllState().getDerivedState()
                .get<dAttackRadialSimulation::DerivedState>();
            const auto& targetState = brawler(1u).getAllState().getState();

            TickSample sample{};
            sample.tick           = tick;
            sample.radialSequence = brawler(0u).getAllState().getState()
                .get<dAttackRadialSimulation::State>().currenSequenceId;
            sample.attackHits     = radialDerived.getAttackHits().size();
            sample.hitsThisTick   = radialDerived.getHitsThisTick().size();
            sample.wasHitThisTick = brawler(1u).getAllState().getDerivedState()
                .get<brawlerInboundHit::DerivedState>().wasHitThisTick;
            sample.hitDirectionXY = brawler(1u).getAllState().getDerivedState()
                .get<brawlerInboundHit::DerivedState>().hitDirectionXY;
            sample.targetMachine  = targetState.get<dAttackMachineSimulation::State>().m_currentState;
            sample.attackerMachine = brawler(0u).getAllState().getState()
                .get<dAttackMachineSimulation::State>().m_currentState;
            sample.targetVelocity = targetState.get<brawlerMovementSimulation::State>().velocity;
            // [movement-sim task 87] Slot 0 of the ATTACKER's pool -- the slot a single
            // Hadouken always lands in, the pool picking the lowest free one.
            {
                const auto& slot = brawler(0u).getAllState().getState()
                    .get<brawlerProjectileSimulation::State>().slots[0];
                sample.projectilePos        = slot.bodyState.position;
                sample.projectileSpawnTick  = slot.spawnTick;
                sample.projectileEndTick    = slot.endTick;
                sample.projectileEndReason  = slot.endReason;
            }

            // THE ENGINE STEP. Nothing inside integrate advances a body; the generic
            // captureBodyStatesAll pass does, and an engine-free rig has to stand in for it.
            auto& ms = brawler(1u).editAllState().editState()
                .edit<brawlerMovementSimulation::State>();
            ms.bodyState.position += ms.velocity * kDt;
            sample.targetPosition = ms.bodyState.position;

            // [movement-sim task 86] A FRESH SWING re-hangs the overlap schedule, and a swing
            // that ENDED closes it. The radial writes currenSequenceId in the same integrate the
            // machine started the swing in, so the open fires on the swing's FIRST tick --
            // including tick 0's, which is why the gate above starts closed.
            if (!isRealAttackSequence(sample.radialSequence))
                swingStartTick = -1;
            else if (sample.radialSequence != lastSequence)
                swingStartTick = t;
            lastSequence = sample.radialSequence;

            samples.push_back(sample);
        }
    }

    std::size_t hitTickCount() const
    {
        std::size_t n = 0;
        for (const TickSample& s : samples) if (s.wasHitThisTick) ++n;
        return n;
    }

    std::string trace(std::size_t from, std::size_t to) const
    {
        std::string text;
        for (std::size_t i = from; i < to && i < samples.size(); ++i)
        {
            const TickSample& s = samples[i];
            text += std::to_string(s.tick) + ":led" + std::to_string(s.attackHits)
                  + "/tick" + std::to_string(s.hitsThisTick)
                  + "/hit" + (s.wasHitThisTick ? "1" : "0") + " ";
        }
        return text;
    }
};

} // namespace endToEnd

} // namespace hitRoutingTests

// ---------------------------------------------------------------------------
// ⭐⭐ THE TABLE IS AUTHORITATIVE, ROW BY ROW. The user's experiment, 2026-09-12: "left and
// right hits knock back while the overhead/forward hit stuns", and the projectile stuns. Each row
// is driven through the real system with the attacker's WIRE sequence id as the only thing that
// changes -- which is what makes this a statement about the table rather than about a branch.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("HitRouting.PerAttackReactionTableIsAuthoritative",
          "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;

    FRoutingRig rig;

    // PREMISE: the table covers the sequence list, which is the constructor's own assertion
    // restated as a measurement -- an out-of-range row is an out-of-range read in the resolver.
    REQUIRE(rig.staticData.m_hitReactions.size() == rig.staticData.m_attackSequences.size());
    REQUIRE(rig.staticData.m_hitReactions.size() == 5u);

    struct Row { unsigned int sequenceId; HitReactionKind kind; const char* what; };
    const Row rows[] = {
        { dAttackDirection::kRightSequenceId, HitReactionKind::Knockback, "right"        },
        { dAttackDirection::kLeftSequenceId,  HitReactionKind::Knockback, "left"         },
        { 2u,                                 HitReactionKind::Knockback, "left->left"   },
        { 3u,                                 HitReactionKind::Knockback, "right->right" },
        { dAttackDirection::kForwardSequenceId, HitReactionKind::Stun,    "forward"      },
    };

    for (const Row& row : rows)
    {
        rig.clearRadialHits(0u);
        rig.setPosition(0u, glm::vec3(0.f, 0.f, 0.f));
        rig.setPosition(1u, glm::vec3(100.f, 0.f, 0.f));
        rig.setSequence(0u, row.sequenceId);
        rig.raiseRadialHit(0u, 1u);
        rig.route(7u);

        const HitReactionSpec& spec = rig.staticData.m_hitReactions[row.sequenceId];
        const brawlerInboundHit::DerivedState& slice = rig.inbound(1u);
        INFO(row.what << " (seq " << row.sequenceId << "): kind=" << int(slice.reactionKind)
             << " speed=" << slice.knockbackSpeed << " dwell=" << slice.flinchDuration
             << " dir=(" << slice.hitDirectionXY.x << ", " << slice.hitDirectionXY.y << ")");

        REQUIRE(slice.wasHitThisTick);
        REQUIRE(slice.reactionKind == row.kind);
        REQUIRE(slice.reactionKind == spec.kind);

        if (row.kind == HitReactionKind::Knockback)
        {
            REQUIRE(slice.knockbackSpeed == Catch::Approx(spec.knockbackSpeed).margin(1e-4f));
            // ⭐⭐ THE DWELL IS DERIVED HERE, FROM THE SAME STATIC DATA. This is the row the
            // whole ownership argument exists for: `knockbackSpeed / launchDecel`, floored by the
            // authored lockout, computed by the actor that sees both numbers.
            REQUIRE(slice.flinchDuration
                    == Catch::Approx(glm::max(spec.lockoutDuration,
                                              spec.knockbackSpeed / rig.launchDecel()))
                           .margin(1e-6f));
            REQUIRE(slice.flinchDuration == Catch::Approx(0.5f).margin(1e-6f));
            REQUIRE(glm::length(slice.hitDirectionXY) == Catch::Approx(1.f).margin(1e-4f));
        }
        else
        {
            // A stun carries NO speed and NO direction -- the movement sim would otherwise have a
            // live direction sitting under a reaction that must not move the body.
            REQUIRE(slice.knockbackSpeed == 0.f);
            REQUIRE(slice.hitDirectionXY == glm::vec2(0.f));
            REQUIRE(slice.flinchDuration == Catch::Approx(spec.lockoutDuration).margin(1e-6f));
            // ⛔ [movement-sim task 86] THE ABSOLUTE PIN, and it is deliberately NOT a
            // computed expression. The stun is a HAND-AUTHORED tuning knob; this line is
            // what forces a human to edit a test on purpose whenever that number moves.
            // Raised 0.3f -> 0.65f in task 86 so a forward/overhead hit outlasts the
            // attacker's own recovery (see impl/design_stun_followup_window.md).
            REQUIRE(slice.flinchDuration == Catch::Approx(0.65f).margin(1e-6f));
        }

        // THE ATTACKER IS NOT HIT BY ITS OWN SWING (D5, pointer identity).
        REQUIRE_FALSE(rig.inbound(0u).wasHitThisTick);
    }

    // ⛔ AND THE RESET CLEARS EVERY NEW FIELD, not only the two bools. A tick with no hit must
    // leave nothing of the last one behind: a stale direction under a stale kind is the trap the
    // architect's review named (section 2.6), and it is harmless only for exactly as long as every
    // reader remembers to gate on `wasHitThisTick`.
    rig.clearRadialHits(0u);
    rig.route(8u);
    const brawlerInboundHit::DerivedState& cleared = rig.inbound(1u);
    REQUIRE_FALSE(cleared.wasHitThisTick);
    REQUIRE_FALSE(cleared.wasProjectileBlockedThisTick);
    REQUIRE(cleared.reactionKind == HitReactionKind::Stun);
    REQUIRE(cleared.knockbackSpeed == 0.f);
    REQUIRE(cleared.flinchDuration == 0.f);
    REQUIRE(cleared.hitDirectionXY == glm::vec2(0.f));
}

// ---------------------------------------------------------------------------
// The direction is computed ONCE, from two positions that are both ON THE WIRE, so both peers
// resolve the same shove. The coincident-position fallback is the attacker's wire aim.
// [movement-sim task 27]
//
// ⚠ [movement-sim task 83] WHAT THIS CASE MEASURES HAS MOVED ONE LEVEL DOWN, and the case is
// left standing rather than rewritten because the arithmetic it pins did not change. The
// PRIMARY direction is now the swing tangent (see KnockbackDirectionIsTheSwingTangentWithFallback
// below); away-from-attacker is what the resolver falls back to when the tangent's XY
// projection is degenerate, and the rig raises hits with a ZERO tangent, so every arm here is
// now an arm of that fallback -- including its own coincident-position and degenerate-aim
// fallbacks, which are unchanged and still the last two lines of defence against a NaN
// velocity.
// ---------------------------------------------------------------------------
TEST_CASE("HitRouting.KnockbackDirectionFromPositionsWithAimFallback",
          "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;

    // 1. AWAY FROM THE ATTACKER, in XY, normalised. The Z separation is deliberately large and
    //    must not appear anywhere in the answer: the knockback is XY-only by ruling.
    {
        FRoutingRig rig;
        rig.setPosition(0u, glm::vec3(10.f, 20.f, 0.f));
        rig.setPosition(1u, glm::vec3(40.f, 60.f, 500.f));
        rig.setSequence(0u, dAttackDirection::kRightSequenceId);
        rig.raiseRadialHit(0u, 1u);
        rig.route(3u);

        const glm::vec2 expected = glm::normalize(glm::vec2(30.f, 40.f));
        const glm::vec2 actual = rig.inbound(1u).hitDirectionXY;
        INFO("expected (" << expected.x << ", " << expected.y << ")  actual ("
             << actual.x << ", " << actual.y << ")");
        REQUIRE(actual.x == Catch::Approx(expected.x).margin(1e-5f));
        REQUIRE(actual.y == Catch::Approx(expected.y).margin(1e-5f));
        REQUIRE(glm::length(actual) == Catch::Approx(1.f).margin(1e-5f));
    }

    // 2. THE COINCIDENT FALLBACK. Two capsules at the same XY is not hypothetical -- it is what a
    //    spawn overlap or a solver mid-separation frame looks like -- and `normalize` of a zero
    //    vector is a NaN that would ride into `velocityUV` and stay there forever.
    for (const float axisZ : { 1.f, -1.f })
    {
        FRoutingRig rig;
        rig.setPosition(0u, glm::vec3(5.f, 5.f, 0.f));
        rig.setPosition(1u, glm::vec3(5.f, 5.f, 90.f));   // identical XY, different height
        rig.setSequence(0u, dAttackDirection::kRightSequenceId);
        rig.setAim(0u, glm::radians(30.f), axisZ);
        rig.raiseRadialHit(0u, 1u);
        rig.route(4u);

        const glm::vec2 actual = rig.inbound(1u).hitDirectionXY;
        const glm::vec2 expected(glm::cos(glm::radians(30.f)),
                                 axisZ * glm::sin(glm::radians(30.f)));
        INFO("aim 30 deg about z=" << axisZ << ": expected (" << expected.x << ", "
             << expected.y << ")  actual (" << actual.x << ", " << actual.y << ")");
        REQUIRE(glm::length(actual) == Catch::Approx(1.f).margin(1e-5f));
        REQUIRE(actual.x == Catch::Approx(expected.x).margin(1e-4f));
        REQUIRE(actual.y == Catch::Approx(expected.y).margin(1e-4f));
        // ...and it is not a NaN, which a bare normalize would have produced. `x == x` is false
        // for a NaN and this is the cheapest honest statement of it.
        REQUIRE(actual.x == actual.x);
        REQUIRE(actual.y == actual.y);
    }

    // 3. A DEGENERATE AIM still resolves. A character that has never swung carries
    //    `initialAimRotationAxis == (0,0,0)`, and `glm::rotate` normalises its axis -- so the
    //    unguarded spelling is a NaN matrix. +X is the resolver's last resort.
    {
        FRoutingRig rig;
        rig.setPosition(0u, glm::vec3(0.f));
        rig.setPosition(1u, glm::vec3(0.f));
        rig.setSequence(0u, dAttackDirection::kRightSequenceId);
        rig.raiseRadialHit(0u, 1u);
        rig.route(5u);

        const glm::vec2 actual = rig.inbound(1u).hitDirectionXY;
        INFO("degenerate aim -> (" << actual.x << ", " << actual.y << ")");
        REQUIRE(actual.x == actual.x);
        REQUIRE(actual.y == actual.y);
        REQUIRE(glm::length(actual) == Catch::Approx(1.f).margin(1e-5f));
    }
}

// ---------------------------------------------------------------------------
// The projectile STUNS -- user ruling 2026-09-12 -- and the `spawnDir` rule stays testable.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("HitRouting.ProjectileHitStunsInPlace", "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;

    // 1. AS AUTHORED: a stun, 0.65 s (task 86; 0.3 s before it), no speed, no direction --
    //    even though the two characters are metres apart and a direction was therefore
    //    computable.
    {
        FRoutingRig rig;
        REQUIRE(rig.staticData.m_projectileHitReaction.kind == HitReactionKind::Stun);
        rig.setPosition(0u, glm::vec3(0.f));
        rig.setPosition(1u, glm::vec3(300.f, 0.f, 0.f));
        rig.raiseProjectileHit(0u, 1u, 12u, glm::vec3(0.f, 1.f, 0.f));
        rig.route(12u);

        const brawlerInboundHit::DerivedState& slice = rig.inbound(1u);
        INFO("projectile: kind=" << int(slice.reactionKind) << " speed=" << slice.knockbackSpeed
             << " dwell=" << slice.flinchDuration);
        REQUIRE(slice.wasHitThisTick);
        REQUIRE(slice.reactionKind == HitReactionKind::Stun);
        REQUIRE(slice.knockbackSpeed == 0.f);
        REQUIRE(slice.hitDirectionXY == glm::vec2(0.f));
        REQUIRE(slice.flinchDuration
                == Catch::Approx(rig.staticData.m_projectileHitReaction.lockoutDuration)
                       .margin(1e-6f));
        // ⛔⛔ [movement-sim task 87] THE PROJECTILE'S ABSOLUTE PIN, and the line above is
        // exactly why it is needed. Every projectile assertion in this file is RELATIONAL
        // against `m_projectileHitReaction.lockoutDuration`, so all of them FOLLOW that literal
        // wherever it goes -- task 86's reviewer set it back to 0.3f on its own and the whole
        // suite stayed green. A relational assertion cannot see the number it reads move.
        // ⛔ This is the line a human has to retype, and it is deliberately NOT a computed
        // expression: the stun is a HAND-AUTHORED tuning knob (user ruling 2026-09-21) and must
        // never be derived from getDuration(), swingTickCount() or a keyframe time. The
        // BEHAVIOURAL claim the 0.65 exists to make -- that a point-blank projectile hit
        // outlasts the shooter's own recovery -- is measured in
        // HitRouting.ProjectilePointBlankFollowUpWindow below; this line only stops the number
        // moving unnoticed. ⚠ The melee half has had the same pin since task 86
        // (HitRouting.PerAttackReactionTableIsAuthoritative and HitRouting.StunHitFiresOnce).
        REQUIRE(rig.staticData.m_projectileHitReaction.lockoutDuration
                == Catch::Approx(0.65f).margin(1e-6f));

        // The endTick guard: the slot keeps endReason 2 until it recycles, so a LATER tick must
        // not re-flinch the target. This pre-dates task 27 and must survive it.
        rig.route(13u);
        REQUIRE_FALSE(rig.inbound(1u).wasHitThisTick);
    }

    // 2. WITH THE SPEC FLIPPED TO KNOCKBACK IN THE FIXTURE, the direction is the projectile's
    //    `spawnDir` and NOT the position difference -- a projectile shoves you the way it was
    //    travelling, not the way its shooter happens to be standing. The rule is authored-off
    //    today; this arm is what keeps it from rotting unseen until somebody turns it on.
    {
        FRoutingRig rig;
        rig.staticData.m_projectileHitReaction =
            HitReactionSpec{ HitReactionKind::Knockback, 1200.f, 0.f };
        rig.setPosition(0u, glm::vec3(0.f));
        rig.setPosition(1u, glm::vec3(300.f, 0.f, 0.f));   // +X apart...
        rig.raiseProjectileHit(0u, 1u, 12u, glm::vec3(0.f, 1.f, 0.f));  // ...but fired along +Y
        rig.route(12u);

        const brawlerInboundHit::DerivedState& slice = rig.inbound(1u);
        INFO("flipped spec: dir=(" << slice.hitDirectionXY.x << ", " << slice.hitDirectionXY.y
             << ") dwell=" << slice.flinchDuration);
        REQUIRE(slice.reactionKind == HitReactionKind::Knockback);
        REQUIRE(slice.knockbackSpeed == Catch::Approx(1200.f).margin(1e-4f));
        REQUIRE(slice.hitDirectionXY.x == Catch::Approx(0.f).margin(1e-5f));
        REQUIRE(slice.hitDirectionXY.y == Catch::Approx(1.f).margin(1e-5f));
        // ...and the dwell is derived from the flipped speed, not from the shipped 2000.
        REQUIRE(slice.flinchDuration
                == Catch::Approx(1200.f / rig.launchDecel()).margin(1e-6f));
    }
}


// ===========================================================================
// ⭐⭐ [movement-sim task 83] A RADIAL HIT FIRES ONCE. THIS IS THE USER'S BUG.
//
// The per-SWING dedup container was used as the per-TICK signal. `attackHits` accumulates
// across a whole swing and is cleared only in `deactivate()`; routing branch 2 iterated it on
// every post-integrate, so ONE hit re-fired on EVERY remaining tick of the swing.
//
// RED BEFORE THE FIX: `wasHitThisTick` true on every tick from the hit until the swing ended,
// not on one.
// ===========================================================================
TEST_CASE("HitRouting.RadialHitFiresOnceAcrossTheSwing", "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;
    using namespace hitRoutingTests::endToEnd;

    FEndToEndRig rig;
    rig.run(60);

    // --- PREMISES. Without these a "fires once" reading could be a swing that never
    //     connected, or one that ended early and took the signal with it. ---
    std::size_t ledgerPeak = 0;
    std::size_t swingTicks = 0;
    for (const TickSample& s : rig.samples)
    {
        ledgerPeak = std::max(ledgerPeak, s.attackHits);
        if (isRealAttackSequence(s.radialSequence)) ++swingTicks;
    }
    INFO("trace " << rig.trace(kHitTick - 1u, kHitTick + 30u));
    INFO("ledger peak " << ledgerPeak << ", swing ran for " << swingTicks << " ticks");

    // The swing connected, and its dedup ledger held exactly ONE entry -- unchanged by this
    // task, and the thing that was being misread as a per-tick signal.
    REQUIRE(ledgerPeak == 1u);
    // ...and it held it for many ticks after the hit. That span IS the defect's window: it is
    // how many times routing used to fire.
    REQUIRE(swingTicks > 20u);

    std::size_t ledgerLiveTicks = 0;
    for (const TickSample& s : rig.samples) if (s.attackHits == 1u) ++ledgerLiveTicks;
    INFO("the ledger was non-empty on " << ledgerLiveTicks << " ticks");
    REQUIRE(ledgerLiveTicks > 20u);

    // --- THE OBSERVABLE. One hit, one firing. ---
    INFO("wasHitThisTick was true on " << rig.hitTickCount() << " tick(s)");
    REQUIRE(rig.hitTickCount() == 1u);
    REQUIRE(rig.samples[kHitTick].wasHitThisTick);
    REQUIRE(rig.samples[kHitTick].hitsThisTick == 1u);
    // The tick after, the signal is gone even though the ledger still holds the entry. That
    // asymmetry is the whole fix, stated as an assertion.
    REQUIRE(rig.samples[kHitTick + 1u].attackHits == 1u);
    REQUIRE(rig.samples[kHitTick + 1u].hitsThisTick == 0u);
    REQUIRE_FALSE(rig.samples[kHitTick + 1u].wasHitThisTick);
}

// ===========================================================================
// ⭐⭐ [movement-sim task 83] THE USER'S TWELVE METRES, MEASURED.
//
// "the knockback is much further than 5 meters, it's closer to 12 meters." One cause, three
// symptoms: the velocity is RE-ASSIGNED at full launch speed on every tick of the swing (so
// it never decays while the swing lasts), the lockout timer restarts every tick, and the
// direction is re-resolved every tick.
//
// RED BEFORE THE FIX: roughly 13 m of travel with HitFlinch exiting around 1.2 s.
// AFTER: the discrete closed form, 516.667 cm, and a 0.5 s lockout from the hit.
//
// ⚠ THE EXPECTATIONS ARE DERIVED FROM THE SHIPPED CONSTANTS, never restated: the travel from
// knockbackSpeed and launchDecel, the dwell from the resolver's own max(lockout, v/a).
// ===========================================================================
TEST_CASE("HitRouting.KnockbackTravelThroughRoutingIsTheClosedForm",
          "[SimulatableBrawler][HitRouting][BrawlerMovement]")
{
    using namespace hitRoutingTests;
    using namespace hitRoutingTests::endToEnd;

    FEndToEndRig rig;
    rig.run(140);

    // --- PREMISES: the shipped reaction for this sequence really is a knockback, and the
    //     closed form really is the 5 m the user was promised. ---
    const HitReactionSpec& spec =
        rig.staticData.m_hitReactions[dAttackDirection::kRightSequenceId];
    REQUIRE(spec.kind == HitReactionKind::Knockback);
    INFO("knockbackSpeed " << rig.knockbackSpeed() << ", launchDecel " << rig.launchDecel()
         << " -> closed form " << (rig.knockbackSpeed() * rig.knockbackSpeed()
                                   / (2.f * rig.launchDecel())) << " cm");
    REQUIRE(rig.knockbackSpeed() * rig.knockbackSpeed() / (2.f * rig.launchDecel())
            == Catch::Approx(500.f).margin(1e-3f));

    // --- WHAT HAPPENED, read off the trace. ---
    const glm::vec3 start = rig.samples[kHitTick].targetPosition;
    glm::vec3 end = start;
    int enteredFlinchAt = -1;
    int leftFlinchAt    = -1;
    for (const TickSample& s : rig.samples)
    {
        end = s.targetPosition;
        if (s.tick <= kHitTick) continue;
        if (enteredFlinchAt < 0 && s.targetMachine == DAttackState::HitFlinch)
            enteredFlinchAt = int(s.tick);
        if (enteredFlinchAt >= 0 && leftFlinchAt < 0
            && s.targetMachine != DAttackState::HitFlinch)
            leftFlinchAt = int(s.tick);
    }
    const glm::vec2 displacement(end.x - start.x, end.y - start.y);
    const float travel = glm::length(displacement);

    INFO("travel " << travel << " cm (" << (travel / 100.f) << " m); direction ("
         << (travel > 0.f ? displacement.x / travel : 0.f) << ", "
         << (travel > 0.f ? displacement.y / travel : 0.f) << ")");
    INFO("HitFlinch entered at tick " << enteredFlinchAt << " (" << (enteredFlinchAt * kDt)
         << " s), left at tick " << leftFlinchAt << " (" << (leftFlinchAt * kDt) << " s)");
    INFO("routing fired on " << rig.hitTickCount() << " tick(s)");

    REQUIRE(enteredFlinchAt > 0);
    REQUIRE(leftFlinchAt > enteredFlinchAt);

    // 1. THE TRAVEL IS THE DISCRETE CLOSED FORM. 516.667 cm, and the 1 cm margin is tight
    //    enough that 13 m cannot hide inside it.
    //
    // ⭐ THE MARGIN IS DOING ONE PIECE OF REAL WORK AND IT IS WORTH NAMING, because it is a
    //    hand-off this rig can see and task 27's could not. The measured value is 517.222 cm,
    //    0.556 cm over the closed form. The LOCKOUT ends one tick BEFORE the slide does: the
    //    machine leaves HitFlinch on the first tick where its timer exceeds the 0.5 s dwell,
    //    which is thirty ticks after it entered, while the launchDecel ramp needs thirty-ONE
    //    (2000 / (4000/60) is 30 exactly in real arithmetic but 4000*(1/60) is 66.666664 in
    //    float, so a thirty-first tick is required to reach zero -- the same float note task 27
    //    pinned as zeroAtTick == 32). On that last tick the character is no longer Launched, so
    //    the ordinary braking model carries the remaining 66.667 cm/s down instead of
    //    launchDecel, and it travels 0.556 cm doing it. 0.1 % of the throw, and REPORTED rather
    //    than tuned away: whether the lockout should outlast the slide is the user's knob.
    //    ⛔ Task 27's rig could not see this at all -- it holds HitFlinch by hand for as long as
    //    the case ticks, so the hand-off never happens there.
    REQUIRE(travel == Catch::Approx(rig.closedFormTravel()).margin(1.f));

    // 2. THE LOCKOUT IS 0.5 s FROM THE HIT, not 0.5 s from the end of the swing. The dwell is
    //    the resolver's own max(lockoutDuration, knockbackSpeed / launchDecel), read from the
    //    same static data the resolver was handed. The machine leaves HitFlinch on the first
    //    tick where its timer EXCEEDS the dwell, and it entered one tick after the hit, so
    //    the honest bound is the dwell plus two ticks of quantisation.
    const float dwell = glm::max(spec.lockoutDuration, rig.knockbackSpeed() / rig.launchDecel());
    const float flinchSpan = float(leftFlinchAt - int(kHitTick)) * kDt;
    INFO("dwell required " << dwell << " s; measured from the hit tick " << flinchSpan << " s");
    REQUIRE(dwell == Catch::Approx(0.5f).margin(1e-6f));
    REQUIRE(flinchSpan >= dwell);
    REQUIRE(flinchSpan <= dwell + 3.f * kDt);

    // 3. AND IT FIRED ONCE -- the cause, restated where the symptom is measured, so a future
    //    change that fixes the distance by some other means cannot leave the cause standing.
    REQUIRE(rig.hitTickCount() == 1u);
}

// ===========================================================================
// [movement-sim task 83] THE SAME CAUSE, ON THE STUN SIDE. A forward hit (sequence 4) is
// authored as a stun with no knockback (0.3 s when task 83 was written, 0.65 s since task 86);
// re-firing it every tick made the stun last for the rest of the swing plus that dwell.
//
// RED BEFORE THE FIX.
//
// ⭐ [movement-sim task 86] AND IT NOW CARRIES THE FOLLOW-UP CLAIM TOO -- block 2 below. The
// authored 0.65 s exists so that a forward/overhead hit outlasts the ATTACKER's own recovery,
// and that is a statement about ticks in the real simulation, not about a literal. It is
// measured here, on this rig, rather than asserted in a document.
// ===========================================================================
TEST_CASE("HitRouting.StunHitFiresOnce", "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;
    using namespace hitRoutingTests::endToEnd;

    FEndToEndRig rig;
    // A ZERO stick classifies FORWARD, which is sequence 4 -- the stun row of the table.
    rig.attackerStick = glm::vec2(0.f);
    rig.run(120);

    const HitReactionSpec& spec =
        rig.staticData.m_hitReactions[dAttackDirection::kForwardSequenceId];
    REQUIRE(spec.kind == HitReactionKind::Stun);

    int enteredFlinchAt = -1;
    int leftFlinchAt    = -1;
    for (const TickSample& s : rig.samples)
    {
        if (s.tick <= kHitTick) continue;
        if (enteredFlinchAt < 0 && s.targetMachine == DAttackState::HitFlinch)
            enteredFlinchAt = int(s.tick);
        if (enteredFlinchAt >= 0 && leftFlinchAt < 0
            && s.targetMachine != DAttackState::HitFlinch)
            leftFlinchAt = int(s.tick);
    }

    INFO("trace " << rig.trace(kHitTick - 1u, kHitTick + 24u));
    INFO("routing fired on " << rig.hitTickCount() << " tick(s); HitFlinch " << enteredFlinchAt
         << " -> " << leftFlinchAt);

    // The premise: the forward swing connected. Sequence 4 rotates about +Y and its plane
    // contains the world X axis, so the same target is on its swing plane too.
    REQUIRE(enteredFlinchAt > 0);
    REQUIRE(leftFlinchAt > enteredFlinchAt);

    // THE OBSERVABLE: one firing, and a dwell of 0.65 s measured FROM THE HIT TICK.
    REQUIRE(rig.hitTickCount() == 1u);
    const float stunSpan = float(leftFlinchAt - int(kHitTick)) * kDt;
    INFO("stun span from the hit tick " << stunSpan << " s against an authored "
         << spec.lockoutDuration << " s");
    // ⛔ [movement-sim task 86] THE SECOND ABSOLUTE PIN. Same reason as the one in
    // HitRouting.ReactionTableIsAuthoritative: the stun duration is hand-authored, so the
    // only acceptable coupling to it is a line a human has to retype. ⛔ Do NOT replace this
    // with anything derived from getDuration(), swingTickCount() or a keyframe time.
    REQUIRE(spec.lockoutDuration == Catch::Approx(0.65f).margin(1e-6f));
    REQUIRE(stunSpan >= spec.lockoutDuration);
    REQUIRE(stunSpan <= spec.lockoutDuration + 3.f * kDt);

    // A stun carries no speed and no direction, and the body does not move.
    for (const TickSample& s : rig.samples)
    {
        INFO("tick " << s.tick);
        REQUIRE(glm::length(glm::vec2(s.targetVelocity.x, s.targetVelocity.y))
                == Catch::Approx(0.f).margin(1e-4f));
    }

    // =======================================================================
    // ⭐⭐ [movement-sim task 86] THE FOLLOW-UP WINDOW, MEASURED IN TICKS.
    //
    // THE CLAIM the authored 0.65 s exists to make: *a forward/overhead hit leaves the
    // attacker time to land a left/right before the defender recovers.*
    //
    // ⛔ A forward/overhead CANNOT CHAIN. The queue gate admits attackLeft only while the
    // active sequence is 0 or 2 and attackRight only while it is 1 or 3; sequence 4 matches
    // NEITHER. So the follow-up costs the whole recovery: finish sequence 4, drop to Idle,
    // start a FRESH sequence 0/1, and wait out ITS wind-up. That is what this block drives,
    // and every tick below is READ OFF THE SIMULATION -- nothing here recomputes it.
    //
    // ⛔⛔ AND THAT IS THE POINT. The stun duration is HAND-AUTHORED (user ruling
    // 2026-09-21) and must never be derived from getDuration(), swingTickCount() or a
    // keyframe time. The coupling is a TRIPWIRE, not a formula: retune any sequence and this
    // block goes RED and tells a human to re-choose the stun by hand. It never auto-updates.
    //
    // ⭐⭐ THE MEASURED TIMELINE, read off THIS rig's own INFO lines (not derived):
    //     tick 18  the hit; the defender enters HitFlinch on 19
    //     tick 33  the attacker is back in Idle  (sequence 4 ran to completion)
    //     tick 34  the held button starts a FRESH sequence 0
    //     tick 52  that swing's first damaging tick -- the follow-up CONNECTS
    //   ⇒ the window the stun must cover is 52 - 18 = 34 ticks = 0.567 s.
    //   The authored 0.65 s ends the flinch on tick 58, so the margin is SIX ticks.
    //
    // ⚠ 0.55 s DOES NOT SUFFICE, and it was witnessed failing: it ends the flinch on
    //   tick 52 exactly, i.e. on the very tick the follow-up lands, and the REQUIRE below
    //   reads Idle instead of HitFlinch. One tick short. ⛔ That is why the number is
    //   0.65 and not the 0.55 an earlier draft of the design proposed.
    //
    // ⛔ sequence 4's getDuration() is 0.52 s, NOT the 0.42 s of its last authored
    //   keyframe: DAttackRadialSequence APPENDS a zero-velocity point at
    //   +timeToReachZeroVelocity. An earlier hand derivation used 0.42 and came out six
    //   ticks early (tick 46 / 0.467 s). The simulation is the authority here, not the
    //   keyframe table.
    // =======================================================================
    {
        FEndToEndRig followUpRig;
        // A ZERO stick on tick 0 classifies FORWARD -- sequence 4, the stun row, as above.
        followUpRig.attackerStick = glm::vec2(0.f);
        // ...then the attacker HOLDS attack from tick 20, which is AFTER the hit at tick 18 and
        // while sequence 4 is still swinging. The hold cannot queue anything (the gate rejects
        // sequence 4), so the machine decides entirely on its own when the fresh swing starts.
        followUpRig.followUpHoldFromTick = 20;
        followUpRig.followUpStick        = glm::vec2(0.f, -1.f);   // -> kRightSequenceId
        followUpRig.run(120);

        int           idleAt        = -1;   // attacker back in Idle after sequence 4
        int           followUpAt    = -1;   // attacker Attacking again, from that Idle
        unsigned int  followUpSeq   = InvalidAttackSequenceId;
        int           followUpHitAt = -1;   // the follow-up's first DAMAGING tick -- it connects
        for (const TickSample& s : followUpRig.samples)
        {
            if (s.tick <= kHitTick) continue;
            if (idleAt < 0 && s.attackerMachine == DAttackState::Idle)
                idleAt = int(s.tick);
            if (idleAt >= 0 && followUpAt < 0 && s.attackerMachine == DAttackState::Attacking)
            {
                followUpAt  = int(s.tick);
                followUpSeq = s.radialSequence;
            }
            if (followUpAt >= 0 && followUpHitAt < 0 && s.wasHitThisTick)
                followUpHitAt = int(s.tick);
        }

        INFO("follow-up trace " << followUpRig.trace(kHitTick - 1u, kHitTick + 40u));
        INFO("sequence 4 getDuration() "
             << followUpRig.staticData.m_attackSequences[dAttackDirection::kForwardSequenceId]
                    .getDuration()
             << " s (the LAST keyframe is 0.42 s; the constructor APPENDS a zero-velocity "
                "point at +timeToReachZeroVelocity)");
        INFO("hit at tick " << kHitTick << "; attacker Idle at " << idleAt
             << "; fresh swing at " << followUpAt << " (sequence " << followUpSeq
             << "); its first damaging tick " << followUpHitAt
             << "; the stun measured above ended at tick " << leftFlinchAt);

        // THE PATH: sequence 4 ran to completion, the machine dropped to Idle, and the held
        // button started a FRESH swing there -- not a chain out of sequence 4.
        REQUIRE(idleAt > int(kHitTick));
        REQUIRE(followUpAt > idleAt);
        REQUIRE(isRealAttackSequence(followUpSeq));
        REQUIRE(followUpSeq != dAttackDirection::kForwardSequenceId);
        REQUIRE(followUpSeq == dAttackDirection::kRightSequenceId);

        // ...and the fresh swing CONNECTED, so `followUpHitAt` is its first damaging tick as
        // the simulation produced it, not as this file computed it.
        REQUIRE(followUpHitAt > followUpAt);

        // ⭐⭐ THE ASSERTION THE WHOLE TASK EXISTS FOR: on the tick the follow-up lands, the
        // defender is STILL STUNNED. `targetMachine` is sampled after that tick's integrate,
        // which consumed the PREVIOUS tick's inbound slice, so this reads the dwell of the
        // FIRST hit and cannot be contaminated by the second.
        REQUIRE(followUpRig.samples[std::size_t(followUpHitAt)].targetMachine == DAttackState::HitFlinch);

        // ...and the same statement as a NUMBER, so the margin is visible when it shrinks.
        // `leftFlinchAt` is the tick the stun ends, measured on the rig above -- which has no
        // follow-up, so nothing re-stuns the defender there and the span is the pure dwell.
        INFO("slack = " << (leftFlinchAt - followUpHitAt) << " tick(s)");
        REQUIRE(followUpHitAt < leftFlinchAt);
    }
}

// ===========================================================================
// ⭐⭐ [movement-sim task 87] THE PROJECTILE HALF OF TASK 86'S CLAIM, MEASURED.
//
// THE CLAIM the authored 0.65 s exists to make, for the projectile: *a point-blank Hadouken
// leaves the shooter time to land a left/right before the target recovers.* Task 86 pinned
// that claim for the MELEE stun only (HitRouting.StunHitFiresOnce), and its reviewer showed
// what that left open: every projectile assertion in this file is RELATIONAL against
// `m_projectileHitReaction.lockoutDuration`, so setting that literal back to 0.3f passed the
// ENTIRE suite. This case is the behavioural half of the fix (finding N-4); the one-line
// absolute pin is in HitRouting.ProjectileHitStunsInPlace above.
//
// ⛔⛔ THE STUN STAYS HAND-AUTHORED (user ruling 2026-09-21). Nothing here computes a
// required lockout from getDuration(), swingTickCount() or a keyframe time. Like task 86's,
// this is a TRIPWIRE: retune the Hadouken commitment, the swing wind-up or the projectile
// speed and it goes RED and tells a human to re-choose the number by hand.
//
// ⭐⭐ THE MEASURED TIMELINE, read off this case's own INFO lines (60 Hz, fire on tick 1):
//     tick  1  the Hadouken fires; the pool spawns slot 0 at x = 60 cm (spawnForwardOffset)
//     tick  2  the shot connects -- its FIRST live tick -- and the target flinches on 3
//     tick 19  the shooter is back in Idle (the flat 0.3 s Hadouken commitment, 18 ticks)
//     tick 20  the held button starts a FRESH sequence 0
//     tick 38  that swing's first damaging tick -- the follow-up CONNECTS
//   ⇒ the window the stun must cover is 38 - 2 = 36 ticks = 0.600 s.
//   The authored 0.65 s ends the flinch on tick 42, so the margin is FOUR ticks.
//
// ⛔ THAT MAKES THE POINT-BLANK PROJECTILE THE BINDING CASE, not the melee one, and it is
//   the reverse of what impl/design_stun_followup_window.md section 2 derived by hand
//   (0.504 s, ~8 ticks of slack). The recovery half of that derivation is CONFIRMED here to
//   the tick -- Idle at fire+18, fresh swing at fire+19, damaging at fire+37. What it got
//   wrong is the FLIGHT: it took the travel to be 90/800 = 0.113 s from the shooter's centre,
//   but the shot is born 60 cm ahead (spawnForwardOffset) and its 40 cm collider touches the
//   target's 42 cm capsule at 82 cm of separation -- so at point-blank the two are ALREADY
//   overlapping when the slot spawns and the hit lands on the first tick it is queried.
//   ⭐ The closer the target, the LONGER the stun must be, and this is the shortest flight
//   the geometry admits.
//
// ⚠ POINT-BLANK IS BOUNDED BY THE MELEE ANNULUS, NOT BY THE SHOT. The radial's gate is
//   `hitDistance > getAttackCircle().getInnerRadius()`, STRICTLY, so a target seated on the
//   inner radius can be shot but never followed up, and the claim would not be expressible.
//   One centimetre outside it is therefore the worst case that can be stated at all.
//
// ⚠ WHAT THIS ARM STILL CANNOT SEE, beyond the four limits listed at the harness: the
//   overlap is a 2-D circle test against the target's CURRENT position, which is exact for
//   this geometry (the shot flies at z = 50, the target capsule spans z 10..202) and would
//   not be for a target on a ledge; and the shooter never moves, so nothing here says what
//   happens if the target walks out of the annulus while the shot is in the air.
// ===========================================================================
TEST_CASE("HitRouting.ProjectilePointBlankFollowUpWindow", "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;
    using namespace hitRoutingTests::endToEnd;

    // The Hadouken cannot be fired on tick 0: a slot's `spawnTick == 0` IS its free marker.
    constexpr int kFireTick = 1;

    // --- RIG A: the shot alone. Nothing follows it up, so the flinch it ends on is the PURE
    //     dwell of the projectile's own stun -- the number rig B's slack is measured against.
    FEndToEndRig shotRig;
    const float pointBlank = shotRig.staticData.m_attackCircle.getInnerRadius() + 1.f;
    shotRig.setTargetDistance(pointBlank);
    shotRig.hadoukenOnTick = kFireTick;
    shotRig.run(120);

    int projectileHitAt   = -1;
    int enteredFlinchAt   = -1;
    int leftFlinchAt      = -1;
    for (const TickSample& s : shotRig.samples)
    {
        if (projectileHitAt < 0 && s.wasHitThisTick)       projectileHitAt = int(s.tick);
        if (projectileHitAt < 0) continue;
        if (enteredFlinchAt < 0 && s.targetMachine == DAttackState::HitFlinch)
            enteredFlinchAt = int(s.tick);
        if (enteredFlinchAt >= 0 && leftFlinchAt < 0
            && s.targetMachine != DAttackState::HitFlinch)
            leftFlinchAt = int(s.tick);
    }

    INFO("shot: fired on tick " << kFireTick << ", spawnTick "
         << shotRig.samples[std::size_t(kFireTick)].projectileSpawnTick
         << ", spawn x " << shotRig.samples[std::size_t(kFireTick)].projectilePos.x
         << " cm, target at " << pointBlank << " cm, contact radius "
         << shotRig.query.projectileContactRadius << " cm");
    INFO("shot: routed hit on tick " << projectileHitAt << "; HitFlinch " << enteredFlinchAt
         << " -> " << leftFlinchAt << "; routing fired on " << shotRig.hitTickCount()
         << " tick(s)");

    // PREMISES: the shot really was a projectile hit (endReason 2, the only reason routing
    // branch 3 admits), it landed once, and it stunned.
    REQUIRE(projectileHitAt > kFireTick);
    REQUIRE(shotRig.hitTickCount() == 1u);
    REQUIRE(shotRig.samples[std::size_t(projectileHitAt)].projectileEndReason == 2u);
    REQUIRE(shotRig.samples[std::size_t(projectileHitAt)].projectileEndTick
            == std::uint32_t(projectileHitAt));
    REQUIRE(enteredFlinchAt == projectileHitAt + 1);
    REQUIRE(leftFlinchAt > enteredFlinchAt);
    // A stun carries no speed: the body does not move, at any tick of the run.
    for (const TickSample& s : shotRig.samples)
    {
        INFO("tick " << s.tick);
        REQUIRE(glm::length(glm::vec2(s.targetVelocity.x, s.targetVelocity.y))
                == Catch::Approx(0.f).margin(1e-4f));
    }

    // THE DWELL, measured from the hit tick, against the authored spec -- and then against the
    // literal, which is the pin review finding N-4 asked for in behavioural form.
    const HitReactionSpec& spec = shotRig.staticData.m_projectileHitReaction;
    const float stunSpan = float(leftFlinchAt - projectileHitAt) * kDt;
    INFO("stun span from the hit tick " << stunSpan << " s against an authored "
         << spec.lockoutDuration << " s");
    REQUIRE(spec.kind == HitReactionKind::Stun);
    REQUIRE(stunSpan >= spec.lockoutDuration);
    REQUIRE(stunSpan <= spec.lockoutDuration + 3.f * kDt);

    // --- RIG B: the same shot, then the button HELD through the shooter's own recovery.
    FEndToEndRig followUpRig;
    followUpRig.setTargetDistance(pointBlank);
    followUpRig.hadoukenOnTick       = kFireTick;
    followUpRig.followUpHoldFromTick = kFireTick + 5;   // after the hit, during the commitment
    followUpRig.followUpStick        = glm::vec2(0.f, -1.f);   // -> kRightSequenceId
    followUpRig.run(120);

    int          shotHitAt      = -1;
    int          idleAt         = -1;
    int          followUpAt     = -1;
    unsigned int followUpSeq    = InvalidAttackSequenceId;
    int          followUpHitAt  = -1;
    for (const TickSample& s : followUpRig.samples)
    {
        if (shotHitAt < 0 && s.wasHitThisTick)        shotHitAt = int(s.tick);
        if (shotHitAt < 0) continue;
        if (idleAt < 0 && s.attackerMachine == DAttackState::Idle)
            idleAt = int(s.tick);
        if (idleAt >= 0 && followUpAt < 0 && s.attackerMachine == DAttackState::Attacking)
        {
            followUpAt  = int(s.tick);
            followUpSeq = s.radialSequence;
        }
        if (followUpAt >= 0 && followUpHitAt < 0 && s.wasHitThisTick)
            followUpHitAt = int(s.tick);
    }

    INFO("follow-up trace " << followUpRig.trace(std::size_t(kFireTick), 45u));
    INFO("shot hit at tick " << shotHitAt << "; shooter Idle at " << idleAt
         << "; fresh swing at " << followUpAt << " (sequence " << followUpSeq
         << "); its first damaging tick " << followUpHitAt
         << "; the stun measured above ended at tick " << leftFlinchAt);

    // THE PATH: the two rigs saw the same shot, the shooter ran its Hadouken commitment out,
    // dropped to Idle, and the held button started a FRESH left/right swing there.
    REQUIRE(shotHitAt == projectileHitAt);
    REQUIRE(idleAt > shotHitAt);
    REQUIRE(followUpAt == idleAt + 1);
    REQUIRE(isRealAttackSequence(followUpSeq));
    REQUIRE(followUpSeq == dAttackDirection::kRightSequenceId);
    REQUIRE(followUpHitAt > followUpAt);

    // ⭐⭐ THE ASSERTION THIS CASE EXISTS FOR: on the tick the follow-up lands, the defender is
    // STILL STUNNED by the PROJECTILE. `targetMachine` is sampled after that tick's integrate,
    // which consumed the PREVIOUS tick's inbound slice, so this reads the first hit's dwell and
    // cannot be contaminated by the second.
    REQUIRE(followUpRig.samples[std::size_t(followUpHitAt)].targetMachine
            == DAttackState::HitFlinch);

    // ...and the same statement as a NUMBER, so the margin is visible when it shrinks.
    // `leftFlinchAt` comes from rig A, which has no follow-up, so it is the pure dwell.
    INFO("slack = " << (leftFlinchAt - followUpHitAt) << " tick(s)");
    REQUIRE(followUpHitAt < leftFlinchAt);
}

// ===========================================================================
// ⭐ [movement-sim task 83] THE DIRECTION IS THE SWING TANGENT -- the user's first point.
//
// RED BEFORE THE FIX: the routed direction was away from the attacker, (1,0) for a target
// straight out along +X. The tangent for the right swing at that hit is cross(+Z, +X) = +Y.
// The two are ORTHOGONAL here, deliberately: no margin can confuse them.
// ===========================================================================
TEST_CASE("HitRouting.KnockbackDirectionIsTheSwingTangentWithFallback",
          "[SimulatableBrawler][HitRouting]")
{
    using namespace hitRoutingTests;
    using namespace hitRoutingTests::endToEnd;

    // 1. THE RIGHT SWING throws along +Y, and the target is straight out along +X, so the
    //    away-from-attacker answer this replaces is exactly +X.
    {
        FEndToEndRig rig;
        rig.run(24);
        const TickSample& hit = rig.samples[kHitTick];
        INFO("right swing routed direction (" << hit.hitDirectionXY.x << ", "
             << hit.hitDirectionXY.y << ")  [away-from-attacker would be (1, 0)]");
        REQUIRE(hit.wasHitThisTick);
        REQUIRE(hit.hitDirectionXY.x == Catch::Approx(0.f).margin(1e-4f));
        REQUIRE(hit.hitDirectionXY.y == Catch::Approx(1.f).margin(1e-4f));
        REQUIRE(glm::length(hit.hitDirectionXY) == Catch::Approx(1.f).margin(1e-4f));
    }

    // 2. THE LEFT SWING MIRRORS IT. Same geometry, opposite authored angular velocity, so the
    //    same tangent with the other sign -- which is the property the user will read as "it
    //    throws the way the weapon was going" rather than as a lookup table.
    {
        FEndToEndRig rig;
        rig.attackerStick = glm::vec2(0.f, 1.f);   // (0,+1) against aim (1,0,0) -> left swing
        rig.run(24);
        const TickSample& hit = rig.samples[kHitTick];
        INFO("left swing routed direction (" << hit.hitDirectionXY.x << ", "
             << hit.hitDirectionXY.y << ")");
        REQUIRE(hit.wasHitThisTick);
        REQUIRE(hit.hitDirectionXY.x == Catch::Approx(0.f).margin(1e-4f));
        REQUIRE(hit.hitDirectionXY.y == Catch::Approx(-1.f).margin(1e-4f));
    }

    // 3. THE DEGENERATE FALLBACK, on the posed rig because the geometry that produces it is
    //    not reachable through the shipped table: a swing whose axis is horizontal has a
    //    VERTICAL tangent, and the only such sequence stuns rather than knocking back. The
    //    resolver must still answer, and it must answer with the shipped away-from-attacker
    //    rule instead of a NaN -- which a bare normalize of a zero XY would have produced.
    {
        FRoutingRig rig;
        rig.setPosition(0u, glm::vec3(0.f, 0.f, 0.f));
        rig.setPosition(1u, glm::vec3(300.f, 400.f, 0.f));
        rig.setSequence(0u, dAttackDirection::kRightSequenceId);
        rig.raiseRadialHit(0u, 1u, /*swingTangent*/ glm::vec3(0.f, 0.f, 1.f));
        rig.route(9u);

        const glm::vec2 actual = rig.inbound(1u).hitDirectionXY;
        const glm::vec2 expected = glm::normalize(glm::vec2(300.f, 400.f));
        INFO("vertical tangent -> (" << actual.x << ", " << actual.y << "); expected the "
             << "away-from-attacker (" << expected.x << ", " << expected.y << ")");
        REQUIRE(actual.x == Catch::Approx(expected.x).margin(1e-5f));
        REQUIRE(actual.y == Catch::Approx(expected.y).margin(1e-5f));
        REQUIRE(actual.x == actual.x);
        REQUIRE(actual.y == actual.y);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
