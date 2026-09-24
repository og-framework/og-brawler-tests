// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================
// BEHAVIOUR DIFFERENCES FROM MOVING MELEE DETECTION INTO A SYSTEM
// [og-netcode-v2-field-defects task 9, second review: impl/review_defect_9_behaviour.md]
//
// These cases pin what the move CHANGED, not what it fixed (the fix is pinned in
// BrawlerHitDetectionSystemTest.cpp). Each one asserts the CURRENT behaviour and names the old
// one, so a later fix flips it RED on purpose and the reader knows why. Case 1 (review F7) is
// task 20's to flip; case 2 (review F8) was fixed by task 9 Rework (1) and now pins the fix.
//
//   1. A replay anchored at the END of the block tick T no longer recoils on T+1. The old
//      radial State's hasHitGuard rode the wire and was restored with the end-of-T state; the
//      inbound-slice bit that replaces it is derived and is not.
//   2. A brawler that is in storage but has not been integrated yet (the resim NoSlot branch
//      emits no input for it, so integrateAll skips it) reaches the detector with the radial's
//      DEFAULT State: currenSequenceId 0, InitialConditions Invalid. Before task 9 Rework (1)
//      that skipped G-01's Invalid return and failed its OG_CHECK (checkf); now G-01 gates on
//      the pair first and returns. The old detector ran only inside integrate, which
//      normalises the pair first. A second case pins that a mid-swing attacker still detects.
//   3. G-04's timer recovery (attackTimer - dt instead of the pre-advance timer) keeps the
//      sign of the authored angular velocity on every tick of every shipped sequence.
//
// The rig is a trimmed copy of BrawlerHitDetectionSystemTest.cpp's FOrderSwapRig, in its own
// namespace so a unity build cannot merge the two.
// TAGS: `[SimulatableBrawler]` is in the `[@og]` whitelist; `[HitDetection]` is not.
// ============================================================================

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/BrawlerInboundHit.h"
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
constexpr float         kTargetDistance   = 150.f;
constexpr int           kContactSwingTick = 24;

// Same geometry as the order-swap rig: 0.5 rad off the attacker line, seq 0/2 side = blocks.
inline glm::vec3 blockingAim() { return glm::vec3(-std::cos(0.5f), -std::sin(0.5f), 0.f); }

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
    bool weaponOverlapsTarget = false;
    bool guardShapeEnabled    = false;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        SpatialQueryReport report;
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
        exec.notifyCharacterRegistered(0u, storage, staticData, true);
        exec.notifyCharacterRegistered(1u, storage, staticData, true);
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

    // The attacker presses seq 0 at pressTick; the target idles holding a blocking guard.
    static simulatableBrawler::PlayerInput attackerInput(int t, int pressTick)
    {
        const bool press = t == pressTick;
        return makeInput(glm::vec3(1.f, 0.f, 0.f), press,
                         press ? glm::vec2(0.f, -1.f) : glm::vec2(0.f), 0u);
    }
    static simulatableBrawler::PlayerInput targetInput()
    {
        return makeInput(blockingAim(), false, glm::vec2(0.f),
                         brawlerMovementSimulation::kInputFlagHoldGuard);
    }

    // One step through the PRODUCTION integration executor and the production systems executor.
    void step(std::uint32_t t, bool resim, const ResolvedInputs<SimulatableBrawler>& inputs)
    {
        const SimulationTimeStep s(t, resim, false, false, kDt);
        integration.integrateAll(s, inputs);
        exec.firePostIntegrate(s, storage, staticData, /*isAuthority*/ !resim);
    }

    ResolvedInputs<SimulatableBrawler> inputsFor(int t, int pressTick)
    {
        ResolvedInputs<SimulatableBrawler> inputs;
        auto& map = std::get<0>(inputs);
        map.emplace(0u, attackerInput(t, pressTick));
        map.emplace(1u, targetInput());
        return inputs;
    }

    DAttackState attackerMachine()
    {
        return brawler(0u).getAllState().getState()
            .get<dAttackMachineSimulation::State>().m_currentState;
    }
};

} // namespace hitDetectionBehaviourTests

using namespace hitDetectionBehaviourTests;

// ---------------------------------------------------------------------------
// 1. Replay anchored at the END of the block tick T.
// BEFORE: the correction state for T carried radial State hasHitGuard = true (written by
// collisionCheck on T, SIM_MEMBER(S, hasHitGuard) at git HEAD DAttackRadialSimulation.h:873),
// prepareResimAll restored it with the rest of State, and the replayed T+1 machine read it
// (HEAD DAttackMachineSimulation.h:726) -> GuardFlinch on T+1, as live.
// AFTER: the signal is the attacker's inbound slice (DerivedState, never restored). The replay
// starts from T+1, detection for T never re-runs, and the slice holds whatever the LAST LIVE
// routing wrote (the frontier tick's, here false). The replayed T+1 stays Attacking.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Behaviour.ReplayAnchoredAtTheEndOfTheBlockTickLosesTheRecoil",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;
    const int frontier = T + 4;
    Rig rig;

    simulatableBrawler::State attackerEndOfT;
    simulatableBrawler::State targetEndOfT;
    DAttackState liveAtT1 = DAttackState::Idle;
    for (int t = 0; t <= frontier; ++t)
    {
        rig.query.weaponOverlapsTarget = t >= T;
        rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press));
        if (t == T)
        {
            attackerEndOfT = rig.brawler(0u).getAllState().getState();
            targetEndOfT   = rig.brawler(1u).getAllState().getState();
            REQUIRE(rig.brawler(0u).getAllState().getDerivedState()
                .get<dAttackRadialSimulation::DerivedState>().getGuardBlockedThisTick());
        }
        if (t == T + 1)
            liveAtT1 = rig.attackerMachine();
    }
    REQUIRE(liveAtT1 == DAttackState::GuardFlinch);
    const bool frontierSlice = rig.brawler(0u).getAllState().getDerivedState()
        .get<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick;
    REQUIRE_FALSE(frontierSlice);

    // The replay: wire State restored to the end of T (what prepareResimAll writes), then T+1,
    // T+2 re-integrated as resim steps. The weapon still overlaps the guard.
    rig.brawler(0u).editAllState().editState() = attackerEndOfT;
    rig.brawler(1u).editAllState().editState() = targetEndOfT;
    rig.step(static_cast<std::uint32_t>(T + 1), true, rig.inputsFor(T + 1, press));
    const DAttackState replayAtT1 = rig.attackerMachine();
    rig.step(static_cast<std::uint32_t>(T + 2), true, rig.inputsFor(T + 2, press));
    const DAttackState replayAtT2 = rig.attackerMachine();

    INFO("live T+1=" << dAttackMachineSimulation::dAttackStateName(liveAtT1)
         << " replay T+1=" << dAttackMachineSimulation::dAttackStateName(replayAtT1)
         << " replay T+2=" << dAttackMachineSimulation::dAttackStateName(replayAtT2));
    // CURRENT behaviour: the replay diverges from live (and from the authority) on T+1.
    CHECK(replayAtT1 == DAttackState::Attacking);
    CHECK(replayAtT1 != liveAtT1);
    // With contact still present the replayed swing re-detects the block on T+1 and recoils
    // one tick LATE; with the blade already past the guard it would not recoil at all.
    CHECK(replayAtT2 == DAttackState::GuardFlinch);
}

// Control for case 1: the ONLY missing ingredient is the bit. Setting the slice after the
// restore -- a model of what the restored wire hasHitGuard used to supply -- gives the live
// outcome back on T+1.
TEST_CASE("HitDetection.Behaviour.ReplayAnchoredAtTheEndOfTheBlockTickRecoilsWhenTheBitIsCarried",
          "[SimulatableBrawler][HitDetection]")
{
    const int T = 40;
    const int press = T - kContactSwingTick;
    Rig rig;
    simulatableBrawler::State attackerEndOfT;
    simulatableBrawler::State targetEndOfT;
    for (int t = 0; t <= T + 4; ++t)
    {
        rig.query.weaponOverlapsTarget = t >= T;
        rig.step(static_cast<std::uint32_t>(t), false, rig.inputsFor(t, press));
        if (t == T)
        {
            attackerEndOfT = rig.brawler(0u).getAllState().getState();
            targetEndOfT   = rig.brawler(1u).getAllState().getState();
        }
    }
    rig.brawler(0u).editAllState().editState() = attackerEndOfT;
    rig.brawler(1u).editAllState().editState() = targetEndOfT;
    rig.brawler(0u).editAllState().editDerivedState()
        .edit<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick = true;
    rig.step(static_cast<std::uint32_t>(T + 1), true, rig.inputsFor(T + 1, press));
    CHECK(rig.attackerMachine() == DAttackState::GuardFlinch);
}

// ---------------------------------------------------------------------------
// 2. A brawler in storage that has NOT been integrated reaches the detector (review F8; fixed by
//    task 9 Rework (1)).
// Reachable in production: a client registers a pawn (cache created, NO slot -- SimulationNetSync
// registerSimulatable, client overload); if the next physics frame opens with a resim,
// getAppliedCaptureTickRef answers NoSlot for every replayed tick, collectResimInputAll emits no
// input (SimulationInputResolution.h:1480-1484), integrateAll skips the id
// (SimulationIntegrationExecutor.h:77), and firePostIntegrate walks the whole storage.
// Also reachable through the accepted game-thread registration tear (storage.add lands between
// integrateAll and firePostIntegrate).
// The detector then sees the radial's DEFAULT pair: State currenSequenceId 0, InitialConditions
// Invalid. Before the rework G-01 asserted `currenSequenceId == IC.activeAttackSequence` ahead of
// its equality return -> checkf (a crash in a UE Development build). After: G-01 gates on the
// pair FIRST and returns; the only assertion left (attackTimer > 0) sits behind the gate.
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
    // firePostIntegrate does not.
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
    CHECK(radialDerived.getAttackHits().empty());
    CHECK_FALSE(radialDerived.getGuardBlockedThisTick());
    CHECK_FALSE(derived.get<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick);

    // One integrate normalises the pair (IC Invalid + State 0 -> deactivate branch -> Invalid).
    ResolvedInputs<SimulatableBrawler> withId2 = rig.inputsFor(2, -1);
    std::get<0>(withId2).emplace(2u, Rig::attackerInput(2, -1));
    rig.step(2u, false, withId2);
    CHECK(rig.brawler(2u).getAllState().getState()
        .get<dAttackRadialSimulation::State>().currenSequenceId == InvalidAttackSequenceId);
}

// Second arm of the rework: the gate did not turn the detector into a no-op. A live mid-swing
// attacker passes the gate with attackTimer > 0 (the remaining assertion's condition) and still
// detects -- here a guard block on the contact tick, and a body hit when the target turns its guard away
// (the aim BrawlerHitDetectionSystemTest.cpp calls turnedAwayAim).
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
            ResolvedInputs<SimulatableBrawler> inputs;
            auto& map = std::get<0>(inputs);
            map.emplace(0u, Rig::attackerInput(t, press));
            map.emplace(1u, Rig::makeInput(glm::vec3(1.f, 0.f, 0.f), false, glm::vec2(0.f), 0u));
            rig.step(static_cast<std::uint32_t>(t), false, inputs);
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
