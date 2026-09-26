// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================
// BEHAVIOUR OF MELEE DETECTION AS A SYSTEM, AND OF THE REDUCTION'S POSITION IN THE TICK
// [og-netcode-v2-field-defects task 9, second review: impl/review_defect_9_behaviour.md]
// [og-netcode-v2-field-defects task 20: detection + routing moved to preIntegrate(T+1)]
//
//   1. A replay anchored at the END of tick T recomputes every inbound signal T produced. Task 9
//      left the guard-block bit on the off-wire inbound slice, written by a post-integrate pass
//      of T, so a replay starting at T+1 read the frontier's stale slice (review F7, reproduced).
//      The same hole was older for wasHitThisTick (since T3) and wasProjectileBlockedThisTick
//      (T15). Task 20 moved detection and routing to preIntegrate(T+1): the signal is produced and
//      consumed inside T+1, from the restored end-of-T state, on the live path and the replay alike.
//      Three cases, one per signal. Each was RED on the pre-task-20 tree (impl_notes_defect_20.md).
//   2. A brawler that is in storage but has not been integrated yet reaches the detector with the
//      radial's DEFAULT State: currenSequenceId 0, InitialConditions Invalid. G-01 gates on the
//      pair first and returns (task 9 Rework (1)). Task 20 adds the pre-integrate reach: the
//      detector now runs BEFORE a new character's first integrate on every tick it is registered.
//   3. G-04's timer recovery keeps the sign of the authored angular velocity on every tick.
//
// THE RIG'S TICK BOUNDARY. `step(t)` is integrateAll(t), firePostIntegrate(t), then the NEXT
// step's firePreIntegrate(t+1) -- production order, cut after the reduction over t so a sample
// taken after step(t) sees what the machine reads on t+1. A replay anchored at the end of T
// restores the wire State, then calls reduce(T+1) (the replay's own first preIntegrate) and
// step(T+1). On the pre-task-20 tree reduce() is an empty hook and the rig runs exactly what
// that tree ran.
// The weapon contact is SCHEDULED (`weaponOverlapsTarget`) and stands for the poses integrate(t)
// left; the reduction over t reads it after integrate(t), which is what preIntegrate(t+1) sees.
// TAGS: `[SimulatableBrawler]` is in the `[@og]` whitelist; `[HitDetection]` is not.
// ============================================================================

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace hitDetectionBehaviourTests
{

constexpr float kDt = 1.f / 60.f;

constexpr std::uint32_t kAttackerRoot     = 30u;
constexpr std::uint32_t kTargetRoot       = 31u;
constexpr std::uint32_t kTargetGuardBody  = 32u;
constexpr std::uint32_t kTargetGuardShape = 33u;
constexpr std::uint32_t kRadialVolume     = 9u;
// [og-netcode-v2-field-defects task 17] The attacker's projectile slot 0, for the projectile cases.
constexpr std::uint32_t kProjectileVolume = 60u;
constexpr std::uint32_t kProjectileBody   = 70u;
constexpr float         kTargetDistance   = 150.f;
constexpr int           kContactSwingTick = 24;

// Same geometry as the order-swap rig: 0.5 rad off the attacker line, seq 0/2 side = blocks.
inline glm::vec3 blockingAim()   { return glm::vec3(-std::cos(0.5f), -std::sin(0.5f), 0.f); }
// A guard facing away from the attacker: never blocks (BrawlerHitDetectionSystemTest.cpp's name).
inline glm::vec3 turnedAwayAim() { return glm::vec3(1.f, 0.f, 0.f); }
// A shot the blocking guard faces head-on: the projectile's block test reads -spawnDir.
inline glm::vec3 shotIntoTheGuard() { return -blockingAim(); }

struct Physics
{
    glm::mat4 guardTransform{ 1.f };
    glm::mat4 getBodyTransform(BodyId id) const
    { return id == BodyId{ kTargetGuardBody } ? guardTransform : glm::mat4(1.f); }
    void setBodyTransform(BodyId id, const glm::mat4& m)
    { if (id == BodyId{ kTargetGuardBody }) guardTransform = m; }
    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
    bool isBodyResolvable(BodyId) const                   { return true; }
};
static_assert(PhysicsBodyAdapter<Physics>);

struct Query
{
    bool weaponOverlapsTarget     = false;
    bool guardShapeEnabled        = false;
    // [task 17] The attacker's projectile slot 0 touches the target: its guard (if enabled)
    // FIRST, then its body, the order a guard in front of the body presents.
    bool projectileOverlapsTarget = false;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        SpatialQueryReport report;
        const bool projectile = std::find(volumeIds.begin(), volumeIds.end(),
                                          QueryVolumeId{ kProjectileVolume }) != volumeIds.end();
        if (projectile && projectileOverlapsTarget)
        {
            SpatialQueryHit body{};
            body.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
            body.bodyId           = BodyId{ kTargetRoot };
            body.rootBodyId       = BodyId{ kTargetRoot };
            body.objectCategories = CollisionCategories::single(collisionCategory::body);
            if (guardShapeEnabled)
            {
                SpatialQueryHit guard = body;
                guard.bodyId           = BodyId{ kTargetGuardBody };
                guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
                report.hits.push_back(guard);
            }
            report.hits.push_back(body);
            return report;
        }
        const bool radial = std::find(volumeIds.begin(), volumeIds.end(),
                                      QueryVolumeId{ kRadialVolume }) != volumeIds.end();
        if (!radial || !weaponOverlapsTarget)
            return report;
        SpatialQueryHit body{};
        body.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
        body.bodyId           = BodyId{ kTargetRoot };
        body.rootBodyId       = BodyId{ kTargetRoot };
        body.objectCategories = CollisionCategories::single(collisionCategory::body);
        report.hits.push_back(body);
        if (guardShapeEnabled)
        {
            SpatialQueryHit guard = body;
            guard.bodyId           = BodyId{ kTargetGuardBody };
            guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
            report.hits.push_back(guard);
        }
        return report;
    }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId id)  { if (id == ShapeId{ kTargetGuardShape }) guardShapeEnabled = true; }
    void disableShape(ShapeId id) { if (id == ShapeId{ kTargetGuardShape }) guardShapeEnabled = false; }
};
static_assert(SpatialQueryAdapter<Query>);

using Detection = brawlerHitDetection::System<Physics, Query>;

struct Rig
{
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
        simulatableBrawler::StaticData, Detection, brawlerHitRouting::System>;
    // The production integration executor: integrateAll SKIPS an id with no resolved input,
    // which is what the resim NoSlot branch produces (collectResimInputAll emits nothing).
    using Integration = SimulationIntegrationExecutor<simulatableBrawler::StaticData, Physics, Query,
        SimulatableBrawler>;

    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    Physics phys;
    Query   query;
    Exec        exec{ std::piecewise_construct, Detection(phys, query), brawlerHitRouting::System{} };
    Integration integration{ storage, staticData, phys, query };

    Rig()
    {
        staticData.m_movementStaticData.drivesBody = false;
        storage.add<SimulatableBrawler>(0u, SimulatableBrawler(staticData));
        storage.add<SimulatableBrawler>(1u, SimulatableBrawler(staticData));
        brawler(0u).setCharacterBindings({ BodyId{ kAttackerRoot } });
        brawler(1u).setCharacterBindings({ BodyId{ kTargetRoot } });
        brawler(0u).editPhysicsComposite().edit<dAttackRadialSimulation::PhysicsDeclaration>()
            .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };
        auto& guard = brawler(1u).editPhysicsComposite()
            .edit<dAttackGuardSimulation::PhysicsDeclaration>().bindings;
        guard.ownBodyId    = BodyId{ kTargetGuardBody };
        guard.parentBodyId = BodyId{ kTargetRoot };
        guard.shapeIds     = { ShapeId{ kTargetGuardShape } };
        auto& shot = brawler(0u).editPhysicsComposite()
            .edit<brawlerProjectileSimulation::PhysicsDeclaration<0>>().bindings;
        shot.ownBodyId      = BodyId{ kProjectileBody };
        shot.parentBodyId   = BodyId{ kAttackerRoot };
        shot.queryVolumeIds = { QueryVolumeId{ kProjectileVolume } };
        exec.notifyCharacterRegistered(0u, storage, staticData, true);
        exec.notifyCharacterRegistered(1u, storage, staticData, true);
        reduce(0u, false);   // tick 0's own preIntegrate
    }

    SimulatableBrawler& brawler(unsigned int id) { return storage.get<SimulatableBrawler>(id); }

    static simulatableBrawler::PlayerInput makeInput(const glm::vec3& aim, bool attackLeft,
                                                     const glm::vec2& stick, std::uint8_t flags)
    {
        const glm::vec3 moveWorld(stick.x, stick.y, 0.f);
        return simulatableBrawler::PlayerInput(
            dAttackRadialSimulation::PlayerInput(aim, attackLeft, false),
            dAttackMachineSimulation::PlayerInput{ aim, attackLeft, false, stick, moveWorld },
            dAttackGuardSimulation::PlayerInput(aim),
            brawlerProjectileSimulation::PlayerInput{ aim },
            brawlerMovementSimulation::PlayerInput{ flags },
            brawlerRingout::PlayerInput{});
    }

    // The attacker presses seq 0 at pressTick (pressTick < 0: never).
    static simulatableBrawler::PlayerInput attackerInput(int t, int pressTick)
    {
        const bool press = t == pressTick;
        return makeInput(glm::vec3(1.f, 0.f, 0.f), press,
                         press ? glm::vec2(0.f, -1.f) : glm::vec2(0.f), 0u);
    }
    // The target idles holding a guard: blocking (default) or turned away from the attacker.
    static simulatableBrawler::PlayerInput targetInput(bool guardFacesAttacker = true)
    {
        return guardFacesAttacker
            ? makeInput(blockingAim(), false, glm::vec2(0.f), brawlerMovementSimulation::kInputFlagHoldGuard)
            : makeInput(turnedAwayAim(), false, glm::vec2(0.f), 0u);
    }

    // The reduction's hook for tick `t`: the systems executor's preIntegrate for step t.
    // `kind` is the prediction clock's step kind (a graduated Skip: `t` is two past the last
    // integrated tick).
    void reduce(std::uint32_t t, bool resim, StepKind kind = StepKind::Normal)
    {
        const SimulationTimeStep s(t, resim, kind, kDt);
        exec.firePreIntegrate(s, storage, staticData, /*isAuthority*/ !resim);
    }

    // One tick in production order, cut after the next step's preIntegrate (see the banner).
    void step(std::uint32_t t, bool resim, const ResolvedInputs<SimulatableBrawler>& inputs,
              const std::function<void()>& afterIntegrate = {})
    {
        const SimulationTimeStep s(t, resim, false, false, kDt);
        integration.integrateAll(s, inputs);
        if (afterIntegrate)
            afterIntegrate();
        exec.firePostIntegrate(s, storage, staticData, /*isAuthority*/ !resim);
        reduce(t + 1u, resim);
    }

    ResolvedInputs<SimulatableBrawler> inputsFor(int t, int pressTick, bool guardFacesAttacker = true)
    {
        ResolvedInputs<SimulatableBrawler> inputs;
        auto& map = std::get<0>(inputs);
        map.emplace(0u, attackerInput(t, pressTick));
        map.emplace(1u, targetInput(guardFacesAttacker));
        return inputs;
    }

    // [task 17] Put the attacker's projectile slot 0 in flight, as the pool's spawn writes it.
    void launchShot(std::uint32_t spawnTick, const glm::vec3& spawnDir)
    {
        auto& slot = brawler(0u).editAllState().editState()
            .edit<brawlerProjectileSimulation::State>().slots[0];
        slot.spawnTick = spawnTick;
        slot.spawnPos  = glm::vec3(-400.f, 0.f, 50.f);
        slot.spawnDir  = spawnDir;
        slot.endTick   = 0u;
        slot.endReason = 0u;
    }

    DAttackState machine(unsigned int id)
    {
        return brawler(id).getAllState().getState()
            .get<dAttackMachineSimulation::State>().m_currentState;
    }
    DAttackState attackerMachine() { return machine(0u); }

    const brawlerInboundHit::DerivedState& inbound(unsigned int id)
    {
        return brawler(id).getAllState().getDerivedState().get<brawlerInboundHit::DerivedState>();
    }

    // The wire State of both characters at the end of a tick: what prepareResimAll restores.
    struct Snapshot { simulatableBrawler::State a; simulatableBrawler::State b; };
    Snapshot snapshot()
    {
        return { brawler(0u).getAllState().getState(), brawler(1u).getAllState().getState() };
    }
    void restore(const Snapshot& s)
    {
        brawler(0u).editAllState().editState() = s.a;
        brawler(1u).editAllState().editState() = s.b;
    }
};

inline const char* name(DAttackState s) { return dAttackMachineSimulation::dAttackStateName(s); }

} // namespace hitDetectionBehaviourTests

using namespace hitDetectionBehaviourTests;

// ---------------------------------------------------------------------------
// 1a. Replay anchored at the END of the guard-block tick T (review F7, the task-20 RED).
// Live: the swing is blocked on T, the attacker recoils (GuardFlinch) on T+1. The replay restores
// the wire State at the end of T and re-runs T+1. The anchor on T stands for a correction whose
// FIRST disagreeing tick is T for some OTHER wire field (review F10: a block-only divergence now
// first disagrees at T+1, so F7 needs another field at T -- e.g. the target's guard aim arriving
// late). This rig restores the anchor directly; which field chose it does not enter the replay.
// PRE-TASK-20 (RED): the bit lived on the attacker's off-wire slice, written by post-integrate
// routing of T; the replay never re-ran it and read the frontier's `false` -> Attacking on T+1.
// TASK 20: the replay's preIntegrate(T+1) re-detects the block from the restored end-of-T state.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Behaviour.ReplayAnchoredAtTheEndOfTheBlockTickRecoils",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;
    const int frontier = T + 4;
    Rig rig;

    Rig::Snapshot endOfT;
    DAttackState liveAtT1 = DAttackState::Idle;
    for (int t = 0; t <= frontier; ++t)
    {
        rig.query.weaponOverlapsTarget = t >= T;
        rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press));
        if (t == T)
        {
            endOfT = rig.snapshot();
            REQUIRE(rig.brawler(0u).getAllState().getDerivedState()
                .get<dAttackRadialSimulation::DerivedState>().getGuardBlockedThisTick());
        }
        if (t == T + 1)
            liveAtT1 = rig.attackerMachine();
    }
    REQUIRE(liveAtT1 == DAttackState::GuardFlinch);
    REQUIRE_FALSE(rig.inbound(0u).wasGuardBlockedThisTick);   // the frontier's slice

    // The replay. The world is rewound to the end of T, where the blade touches the guard.
    rig.restore(endOfT);
    rig.query.weaponOverlapsTarget = true;
    rig.reduce(static_cast<std::uint32_t>(T + 1), true);
    rig.step(static_cast<std::uint32_t>(T + 1), true, rig.inputsFor(T + 1, press));
    const DAttackState replayAtT1 = rig.attackerMachine();

    INFO("live T+1=" << name(liveAtT1) << " replay T+1=" << name(replayAtT1));
    CHECK(replayAtT1 == DAttackState::GuardFlinch);
    CHECK(replayAtT1 == liveAtT1);
}

// 1a control. The recoil on the replayed T+1 comes from the replay's OWN preIntegrate(T+1) and
// from nothing else: skip that one call and the replay reads the frontier's stale slice and keeps
// swinging -- the pre-task-20 outcome. (Until task 20 this slot held the positive control
// `...RecoilsWhenTheBitIsCarried`, which set the slice by hand after the restore. With the
// reduction inside T+1 that model is the production path itself, so the case now pins the
// negative: it shows WHICH call carries the fix.)
TEST_CASE("HitDetection.Behaviour.ReplayWithoutItsOwnPreIntegrateLosesTheRecoil",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;
    Rig rig;
    Rig::Snapshot endOfT;
    for (int t = 0; t <= T + 4; ++t)
    {
        rig.query.weaponOverlapsTarget = t >= T;
        rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press));
        if (t == T)
            endOfT = rig.snapshot();
    }
    rig.restore(endOfT);
    rig.query.weaponOverlapsTarget = true;
    // No rig.reduce(T+1): integrate T+1 straight after the restore.
    rig.step(static_cast<std::uint32_t>(T + 1), true, rig.inputsFor(T + 1, press));
    INFO("replay T+1 without its preIntegrate = " << name(rig.attackerMachine()));
    CHECK(rig.attackerMachine() == DAttackState::Attacking);
}

// ---------------------------------------------------------------------------
// 1b. Replay anchored at the END of the BODY-hit tick T. The target's guard faces away, so the
// swing hits the body on T; live, the target enters HitFlinch on T+1.
// PRE-TASK-20 (RED): wasHitThisTick was written by post-integrate routing of T and never re-run
// by the replay -> the target stays Idle on the replayed T+1 (the older hole, since T3).
//
// ⚠ THE LEDGER. Detection skips a target already in the attacker's per-SWING ledger. Until
// og-netcode-v2-field-defects task 27 that ledger was DerivedState, which a restore does not roll
// back, so the replay re-detected the hit only if the swing had ended before the correction
// landed; section "frontier mid-swing" pinned the replay's target as Idle. Task 27 moved the
// ledger onto the wire (`State::hitTargets`, appended by the radial's integrate the tick AFTER
// the detection), so the restore brings back end-of-T's EMPTY ledger and the replayed T+1
// re-detects. That section was the task-27 positive control: RED (`Idle`) on the tree before
// it, GREEN (`HitFlinch`, == live) after.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Behaviour.ReplayAnchoredAtTheEndOfTheBodyHitTickFlinchesTheTarget",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;

    auto runLiveTo = [&](Rig& rig, int frontier, Rig::Snapshot& endOfT, DAttackState& liveTargetAtT1)
    {
        for (int t = 0; t <= frontier; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= T;
            rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press, false));
            if (t == T)
                endOfT = rig.snapshot();
            if (t == T + 1)
                liveTargetAtT1 = rig.machine(1u);
        }
    };
    auto replayT1 = [&](Rig& rig, const Rig::Snapshot& endOfT)
    {
        rig.restore(endOfT);
        rig.query.weaponOverlapsTarget = true;
        rig.reduce(static_cast<std::uint32_t>(T + 1), true);
        rig.step(static_cast<std::uint32_t>(T + 1), true, rig.inputsFor(T + 1, press, false));
        return rig.machine(1u);
    };

    SECTION("frontier after the swing ended")
    {
        Rig rig;
        Rig::Snapshot endOfT;
        DAttackState live = DAttackState::Idle;
        runLiveTo(rig, T + 30, endOfT, live);
        REQUIRE(live == DAttackState::HitFlinch);
        const auto& ledger = rig.brawler(0u).getAllState().getState()
            .get<dAttackRadialSimulation::State>().hitTargets;
        REQUIRE(ledger[0] == SimCharacterId::None);   // premise: the swing's deactivate cleared it
        const DAttackState replay = replayT1(rig, endOfT);
        INFO("live target T+1=" << name(live) << " replay target T+1=" << name(replay));
        CHECK(replay == DAttackState::HitFlinch);
        CHECK(replay == live);
    }

    SECTION("frontier mid-swing: the restored synced ledger lets the replay re-detect (task 27)")
    {
        Rig rig;
        Rig::Snapshot endOfT;
        DAttackState live = DAttackState::Idle;
        runLiveTo(rig, T + 4, endOfT, live);
        REQUIRE(live == DAttackState::HitFlinch);
        const auto& ledger = rig.brawler(0u).getAllState().getState()
            .get<dAttackRadialSimulation::State>().hitTargets;
        REQUIRE(ledger[0] == SimCharacterId{ 1u });   // premise: the frontier's ledger holds the target
        REQUIRE(endOfT.a.get<dAttackRadialSimulation::State>().hitTargets[0] == SimCharacterId::None);
        const DAttackState replay = replayT1(rig, endOfT);
        INFO("live target T+1=" << name(live) << " replay target T+1=" << name(replay));
        CHECK(replay == DAttackState::HitFlinch);
        CHECK(replay == live);
    }
}

// ---------------------------------------------------------------------------
// 1c. Replay anchored at the END of tick T, the last tick the attacker's projectile is in flight on
// the wire, when it is BLOCKED by the guard on T+1. Live, the shooter recoils (GuardFlinch, from any
// state: machine G-04) on T+1.
// PRE-TASK-20 (RED): routing branch 4 matched `slot.endTick == currentTick` in post-integrate of the
// block tick; a replay starting after it never re-ran that pass, and the slice held the frontier's
// `false`.
// [og-netcode-v2-field-defects task 17] Re-homed onto the detector. Until task 17 this case wrote the
// ENDED slot by hand after integrate(T) (endTick T, endReason 4) and routing matched that endTick
// against the tick integrated last. The shot is now DETECTED: brawlerHitDetection's projectile
// pass checks it at its tick-(T+1) closed-form position in preIntegrate(T+1), routes the block in
// the same pass, and integrate(T+1) recoils the shooter and ends the slot. The replay restores the
// end of T, where the slot is still in flight, and re-detects. The assertions are unchanged except
// the slot premise, which now reads the slot in flight at the anchor and ended with 4 on T+1.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Behaviour.ReplayAnchoredAtTheEndOfTheProjectileBlockTickRecoils",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int frontier = T + 4;
    Rig rig;

    auto launch = [&rig, T]() { rig.launchShot(static_cast<std::uint32_t>(T - 3), shotIntoTheGuard()); };

    Rig::Snapshot endOfT;
    DAttackState liveAtT1 = DAttackState::Idle;
    unsigned int liveEndReasonAtT1 = 0u;
    for (int t = 0; t <= frontier; ++t)
    {
        // Contact from T+1 on: step(T)'s reduction IS preIntegrate(T+1).
        rig.query.projectileOverlapsTarget = t >= T;
        rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, -1),
                 t == T - 3 ? std::function<void()>(launch) : std::function<void()>{});
        if (t == T)
            endOfT = rig.snapshot();
        if (t == T + 1)
        {
            liveAtT1 = rig.attackerMachine();
            liveEndReasonAtT1 = rig.brawler(0u).getAllState().getState()
                .get<brawlerProjectileSimulation::State>().slots[0].endReason;
        }
    }
    REQUIRE(liveAtT1 == DAttackState::GuardFlinch);
    REQUIRE_FALSE(rig.inbound(0u).wasProjectileBlockedThisTick);   // the frontier's slice
    REQUIRE(endOfT.a.get<brawlerProjectileSimulation::State>().slots[0].endTick == 0u);   // in flight at T
    REQUIRE(liveEndReasonAtT1 == 4u);

    rig.restore(endOfT);
    rig.query.projectileOverlapsTarget = true;
    rig.reduce(static_cast<std::uint32_t>(T + 1), true);
    rig.step(static_cast<std::uint32_t>(T + 1), true, rig.inputsFor(T + 1, -1));
    const DAttackState replayAtT1 = rig.attackerMachine();

    INFO("live shooter T+1=" << name(liveAtT1) << " replay shooter T+1=" << name(replayAtT1));
    CHECK(replayAtT1 == DAttackState::GuardFlinch);
    CHECK(replayAtT1 == liveAtT1);
}

// ---------------------------------------------------------------------------
// 1d. A projectile that reaches the target across a client graduated SKIP step (task 20 review F1).
// ClientPredictionClock's Skip advances the frontier by two: the step tick is L+2, and L+1 is only
// backfilled (SimulationReconciliation backfillSkippedTick), never integrated. The shot must be
// routed exactly once, and the next Normal step (L+3) must not route it again.
// Task 20 as first shipped matched `endTick == step.getTick() - 1` and dropped both signals below
// (RED, impl_notes_defect_20.md "Rework (1)"); Rework (1) fixed it with a per-step-kind offset.
// [og-netcode-v2-field-defects task 17] Re-homed onto the detector, which removed that offset. The
// shot is in flight at L and in contact from the Skip step on: preIntegrate(L+2) checks it at its
// tick-(L+2) closed-form position (the position integrate(L+2) snaps it to; integrate never visits
// L+1 either), routes the outcome in the same pass, and integrate(L+2) reacts and ends the slot.
// No tick arithmetic anywhere. The assertions are unchanged.
// ---------------------------------------------------------------------------
namespace hitDetectionBehaviourTests
{
struct SkipStepOutcome
{
    bool         hitRoutedOnSkip      = false;   // target's wasHitThisTick after pre(L+2)
    bool         blockRoutedOnSkip    = false;   // shooter's wasProjectileBlockedThisTick after pre(L+2)
    DAttackState shooterAfterSkip     = DAttackState::Idle;
    DAttackState targetAfterSkip      = DAttackState::Idle;
    bool         reRoutedOnNextNormal = false;   // either bit set again by pre(L+3)
};

// `blocked`: the target holds its guard facing the shot (a block, branch 4); otherwise it turns
// its guard away and the shot hits its body (branch 3).
inline SkipStepOutcome runProjectileContactOnASkipStep(bool blocked)
{
    constexpr std::uint32_t L = 40u;
    Rig rig;
    const glm::vec3 shotDir = blocked ? shotIntoTheGuard() : glm::vec3(1.f, 0.f, 0.f);
    auto launch = [&rig, shotDir]() { rig.launchShot(L - 3u, shotDir); };
    for (std::uint32_t t = 0u; t < L; ++t)          // ends with the Normal pre(L)
        rig.step(t, false, rig.inputsFor(static_cast<int>(t), -1, blocked),
                 t == L - 3u ? std::function<void()>(launch) : std::function<void()>{});

    const SimulationTimeStep sL(L, false, StepKind::Normal, kDt);
    rig.integration.integrateAll(sL, rig.inputsFor(static_cast<int>(L), -1, blocked));
    rig.exec.firePostIntegrate(sL, rig.storage, rig.staticData, /*isAuthority*/ false);
    REQUIRE(rig.brawler(0u).getAllState().getState()
        .get<brawlerProjectileSimulation::State>().slots[0].endTick == 0u);   // in flight at L

    SkipStepOutcome out;
    rig.query.projectileOverlapsTarget = true;
    rig.reduce(L + 2u, false, StepKind::Skip);      // the Skip step's preIntegrate
    out.hitRoutedOnSkip   = rig.inbound(1u).wasHitThisTick;
    out.blockRoutedOnSkip = rig.inbound(0u).wasProjectileBlockedThisTick;
    const SimulationTimeStep sSkip(L + 2u, false, StepKind::Skip, kDt);
    rig.integration.integrateAll(sSkip, rig.inputsFor(static_cast<int>(L + 2u), -1, blocked));
    out.shooterAfterSkip = rig.attackerMachine();
    out.targetAfterSkip  = rig.machine(1u);
    rig.exec.firePostIntegrate(sSkip, rig.storage, rig.staticData, /*isAuthority*/ false);

    rig.reduce(L + 3u, false);                      // the next Normal step
    out.reRoutedOnNextNormal = rig.inbound(1u).wasHitThisTick
                            || rig.inbound(0u).wasProjectileBlockedThisTick;
    return out;
}
} // namespace hitDetectionBehaviourTests

TEST_CASE("HitDetection.Behaviour.ProjectileHitOnTheTickBeforeASkipStepFlinchesTheTarget",
          "[SimulatableBrawler][HitDetection]")
{
    const SkipStepOutcome o = runProjectileContactOnASkipStep(false);   // branch 3
    INFO("slice after pre(L+2 skip): target wasHit=" << o.hitRoutedOnSkip
         << "; target after integrate(L+2)=" << name(o.targetAfterSkip)
         << "; re-routed on pre(L+3)=" << o.reRoutedOnNextNormal);
    CHECK(o.hitRoutedOnSkip);
    CHECK(o.targetAfterSkip == DAttackState::HitFlinch);
    CHECK_FALSE(o.blockRoutedOnSkip);
    CHECK_FALSE(o.reRoutedOnNextNormal);
}

TEST_CASE("HitDetection.Behaviour.ProjectileBlockOnTheTickBeforeASkipStepRecoils",
          "[SimulatableBrawler][HitDetection]")
{
    const SkipStepOutcome o = runProjectileContactOnASkipStep(true);    // branch 4
    INFO("slice after pre(L+2 skip): shooter blocked=" << o.blockRoutedOnSkip
         << "; shooter after integrate(L+2)=" << name(o.shooterAfterSkip)
         << "; re-routed on pre(L+3)=" << o.reRoutedOnNextNormal);
    CHECK(o.blockRoutedOnSkip);
    CHECK(o.shooterAfterSkip == DAttackState::GuardFlinch);
    CHECK_FALSE(o.hitRoutedOnSkip);
    CHECK_FALSE(o.reRoutedOnNextNormal);
}

// ---------------------------------------------------------------------------
// 2. A brawler in storage that has NOT been integrated reaches the detector (review F8; fixed by
//    task 9 Rework (1)).
// Reachable in production: a client registers a pawn (cache created, NO slot -- SimulationNetSync
// registerSimulatable, client overload); if the next physics frame opens with a resim,
// getAppliedCaptureTickRef answers NoSlot for every replayed tick, collectResimInputAll emits no
// input (SimulationInputResolution.h:1480-1484), integrateAll skips the id
// (SimulationIntegrationExecutor.h:77), and the systems executor walks the whole storage.
// Also reachable through the accepted game-thread registration tear.
// The detector then sees the radial's DEFAULT pair: State currenSequenceId 0, InitialConditions
// Invalid. G-01 gates on the pair FIRST and returns; the only assertion (attackTimer > 0) sits
// behind the gate.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Behaviour.NeverIntegratedBrawlerIsSkippedByTheDetector",
          "[SimulatableBrawler][HitDetection]")
{
    Rig rig;
    rig.storage.add<SimulatableBrawler>(2u, SimulatableBrawler(rig.staticData));
    // Give the newcomer a bound radial volume and let the weapon volume report contact, so a
    // detector that ran past its gate would have something to find.
    rig.brawler(2u).editPhysicsComposite().edit<dAttackRadialSimulation::PhysicsDeclaration>()
        .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };
    rig.exec.notifyCharacterRegistered(2u, rig.storage, rig.staticData, false);
    rig.query.weaponOverlapsTarget = true;

    // One resim step whose resolved inputs lack id 2 (the NoSlot row): integrateAll skips it,
    // both systems passes do not.
    rig.step(1u, true, rig.inputsFor(1, -1));

    const auto& state  = rig.brawler(2u).getAllState().getState();
    const auto& radial = state.get<dAttackRadialSimulation::State>();
    const auto& ic     = state.get<dAttackRadialSimulation::InitialConditions>();
    INFO("never-integrated: radial.currenSequenceId=" << radial.currenSequenceId
         << " radial.attackTimer=" << radial.attackTimer
         << " ic.activeAttackSequence=" << ic.activeAttackSequence);
    // The id really was not integrated: the default pair is still there.
    REQUIRE(radial.currenSequenceId == 0u);
    REQUIRE(ic.activeAttackSequence == InvalidAttackSequenceId);

    // No detection, no assert.
    const auto& derived = rig.brawler(2u).getAllState().getDerivedState();
    const auto& radialDerived = derived.get<dAttackRadialSimulation::DerivedState>();
    CHECK(radialDerived.getHitsThisTick().empty());
    CHECK(radial.hitTargets[0] == SimCharacterId::None);
    CHECK_FALSE(radialDerived.getGuardBlockedThisTick());
    CHECK_FALSE(derived.get<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick);

    // One integrate normalises the pair (IC Invalid + State 0 -> deactivate branch -> Invalid).
    ResolvedInputs<SimulatableBrawler> withId2 = rig.inputsFor(2, -1);
    std::get<0>(withId2).emplace(2u, Rig::attackerInput(2, -1));
    rig.step(2u, false, withId2);
    CHECK(rig.brawler(2u).getAllState().getState()
        .get<dAttackRadialSimulation::State>().currenSequenceId == InvalidAttackSequenceId);
}

// [task 20] The pre-integrate reach. Since detection runs in preIntegrate, EVERY registered
// character meets the detector before its first integrate -- on a live tick and on a resim
// replay alike, not only through the NoSlot / tear paths above. Both arms: the default pair
// produces no detection and trips no check.
// ⚠ This cannot be RED against the task-9-Rework-(1) gate, which already returns on the default
// pair (the section above pinned it post-integrate). It is RED only against the pre-Rework gate
// (assert-then-return) or a gate that trusts `currenSequenceId != Invalid` alone.
TEST_CASE("HitDetection.Behaviour.FreshCharacterMeetsTheDetectorBeforeItsFirstIntegrate",
          "[SimulatableBrawler][HitDetection]")
{
    for (const bool resim : { false, true })
    {
        Rig rig;
        rig.storage.add<SimulatableBrawler>(2u, SimulatableBrawler(rig.staticData));
        rig.brawler(2u).editPhysicsComposite().edit<dAttackRadialSimulation::PhysicsDeclaration>()
            .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };
        rig.exec.notifyCharacterRegistered(2u, rig.storage, rig.staticData, !resim);
        rig.query.weaponOverlapsTarget = true;

        // Its first tick's preIntegrate, before any integrate has touched it.
        rig.reduce(5u, resim);

        INFO((resim ? "resim" : "live") << " first preIntegrate");
        const auto& radial = rig.brawler(2u).getAllState().getState()
            .get<dAttackRadialSimulation::State>();
        REQUIRE(radial.currenSequenceId == 0u);
        const auto& radialDerived = rig.brawler(2u).getAllState().getDerivedState()
            .get<dAttackRadialSimulation::DerivedState>();
        CHECK(radialDerived.getHitsThisTick().empty());
        CHECK(radial.hitTargets[0] == SimCharacterId::None);
        CHECK_FALSE(radialDerived.getGuardBlockedThisTick());
        CHECK_FALSE(rig.inbound(2u).wasHitThisTick);
    }
}

// [task 20] The detector OWNS the per-tick signals it writes (guards doc G-13): each pass clears
// hitsThisTick, guardBlockedThisTick and guardHits for its character before its gate, so a
// signal lives from the reduction over T (preIntegrate(T+1)) to the next one.
//   * guardHits (the visualizer's blue sphere) must survive integrate(T+1): the snapshot is taken
//     after T+1's physics step. The swing's deactivate on T+1 used to clear it, which after the
//     move would have erased it before any snapshot saw it.
//   * An attacker the replay does NOT integrate (the NoSlot row) keeps last pass's hitsThisTick;
//     without the detector's own clear the next pass would carry that entry beside its own.
//     [og-netcode-v2-field-defects task 27, user ruling 2026-09-26] Since the per-swing ledger is
//     the synced State::hitTargets, recorded only by the attacker's OWN integrate, a mid-swing
//     attacker the replay skips never records the target, so the next pass RE-DETECTS it and
//     routes it again (the derived ledger used to suppress that). Accepted and pinned as the
//     measured behaviour: one registration per skipped pass, never two (the detector's clear).
TEST_CASE("HitDetection.Behaviour.TheDetectorOwnsThePerTickSignalsItWrites",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;

    SECTION("guardHits lives from the reduction over T to the next reduction")
    {
        Rig rig;
        for (int t = 0; t <= T; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= T;
            rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press));
        }
        auto guardHits = [&rig]()
        {
            return rig.brawler(0u).getAllState().getDerivedState()
                .get<dAttackRadialSimulation::DerivedState>().getGuardHits().size();
        };
        REQUIRE(guardHits() == 1u);
        std::size_t afterIntegrateT1 = 99u;
        rig.step(static_cast<std::uint32_t>(T + 1), false, rig.inputsFor(T + 1, press),
                 [&]() { afterIntegrateT1 = guardHits(); });
        INFO("guardHits after integrate(T+1)=" << afterIntegrateT1
             << " after the reduction over T+1=" << guardHits()
             << " attacker T+1=" << name(rig.attackerMachine()));
        REQUIRE(rig.attackerMachine() == DAttackState::GuardFlinch);   // deactivate ran on T+1
        CHECK(afterIntegrateT1 == 1u);   // what the post-physics visualization snapshot reads
        CHECK(guardHits() == 0u);        // one tick wide, not sticky
    }

    SECTION("an attacker left un-integrated re-detects once per pass and never doubles the entry")
    {
        Rig rig;
        for (int t = 0; t <= T; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= T;
            rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press, false));
        }
        REQUIRE(rig.inbound(1u).wasHitThisTick);   // the hit of T, routed for T+1
        // A resim step whose inputs lack the attacker: integrateAll skips it, so the radial's own
        // top-of-integrate clear does not run for it.
        ResolvedInputs<SimulatableBrawler> targetOnly;
        std::get<0>(targetOnly).emplace(1u, Rig::targetInput(false));
        rig.step(static_cast<std::uint32_t>(T + 1), true, targetOnly);
        const auto& attackerRadial = rig.brawler(0u).getAllState().getState()
            .get<dAttackRadialSimulation::State>();
        const std::size_t entries = rig.brawler(0u).getAllState().getDerivedState()
            .get<dAttackRadialSimulation::DerivedState>().getHitsThisTick().size();
        INFO("target slice after the reduction over T+1: wasHit=" << rig.inbound(1u).wasHitThisTick
             << " attacker hitsThisTick=" << entries);
        // The skipped integrate recorded nothing: the synced ledger is frozen empty.
        CHECK(attackerRadial.hitTargets[0] == SimCharacterId::None);
        // Re-detected and routed again (task 27 ruling) ...
        CHECK(rig.inbound(1u).wasHitThisTick);
        // ... but as ONE entry: the detector's own clear (G-13) dropped last pass's.
        CHECK(entries == 1u);
    }
}

// Second arm of task 9 Rework (1): the gate did not turn the detector into a no-op. A live
// mid-swing attacker passes the gate with attackTimer > 0 (the remaining assertion's condition)
// and still detects -- a guard block on the contact tick, and a body hit when the target turns
// its guard away.
TEST_CASE("HitDetection.Behaviour.MidSwingBrawlerStillDetectsBehindTheGate",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;

    SECTION("guard held: the swing is blocked on the contact tick")
    {
        Rig rig;
        for (int t = 0; t <= T; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= T;
            rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press));
        }
        const auto& radial = rig.brawler(0u).getAllState().getState()
            .get<dAttackRadialSimulation::State>();
        const auto& ic = rig.brawler(0u).getAllState().getState()
            .get<dAttackRadialSimulation::InitialConditions>();
        INFO("contact tick: curSeq=" << radial.currenSequenceId << " ic=" << ic.activeAttackSequence
             << " attackTimer=" << radial.attackTimer);
        REQUIRE(radial.currenSequenceId != InvalidAttackSequenceId);
        REQUIRE(radial.currenSequenceId == ic.activeAttackSequence);
        CHECK(radial.attackTimer > 0.f);
        CHECK(rig.brawler(0u).getAllState().getDerivedState()
            .get<dAttackRadialSimulation::DerivedState>().getGuardBlockedThisTick());
    }

    SECTION("guard turned away: the swing hits the body on the contact tick")
    {
        Rig rig;
        for (int t = 0; t <= T; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= T;
            rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press, false));
        }
        const auto& radial = rig.brawler(0u).getAllState().getState()
            .get<dAttackRadialSimulation::State>();
        CHECK(radial.attackTimer > 0.f);
        const auto& radialDerived = rig.brawler(0u).getAllState().getDerivedState()
            .get<dAttackRadialSimulation::DerivedState>();
        CHECK_FALSE(radialDerived.getGuardBlockedThisTick());
        REQUIRE(radialDerived.getHitsThisTick().size() == 1u);
        CHECK(radialDerived.getHitsThisTick()[0].hitRootBodyId == BodyId{ kTargetRoot });
    }
}

// ---------------------------------------------------------------------------
// 3. G-04: the detector samples the authored angular velocity at attackTimer - dt, after the
// radial's `attackTimer += dt`; collisionCheck sampled it at the pre-advance timer. Only the
// SIGN is consumed (swingTangentAt: `authoredAngularVelocity < 0.f`). Survey: every tick of every
// shipped sequence, float accumulation exactly as integrate does it, at the shipped Chaos step
// (DefaultEngine.ini AsyncFixedTimeStepSize=0.016667) and three neighbours.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Behaviour.SwingTimerRecoveryKeepsTheAngularVelocitySign",
          "[SimulatableBrawler][HitDetection]")
{
    simulatableBrawler::StaticData staticData;
    const auto& sequences = staticData.m_attackSimulationStaticData.getAttackSequences();
    REQUIRE_FALSE(sequences.empty());

    int ticks = 0;
    int signFlips = 0;
    int valueDiffs = 0;
    for (const float dt : { 0.016667f, 1.f / 60.f, 1.f / 30.f, 1.f / 120.f })
    {
        for (const auto& sequence : sequences)
        {
            float timer = 0.f;
            while (timer < sequence.getDuration())
            {
                const float before = timer;             // old: read before the advance
                timer = timer + dt;                     // radial integrate
                const float recovered = timer - dt;     // new: G-04
                ++ticks;
                if (before != recovered)
                    ++valueDiffs;
                if ((sequence.getAngularVelocity(before) < 0.f)
                    != (sequence.getAngularVelocity(recovered) < 0.f))
                    ++signFlips;
            }
        }
    }
    INFO("ticks surveyed=" << ticks << " timer values that differ=" << valueDiffs
         << " sign flips=" << signFlips);
    CHECK(ticks > 0);
    CHECK(signFlips == 0);
}

#endif // WITH_LOW_LEVEL_TESTS
