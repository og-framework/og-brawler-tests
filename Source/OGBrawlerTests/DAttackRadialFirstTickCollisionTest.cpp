// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

// ===========================================================================
// [movement-sim task 34] AN ATTACK HELD ON A CHARACTER'S VERY FIRST SIMULATED TICK
// MUST STILL REGISTER HITS.
//
// ⚠ THIS IS A REAL BUG FIX, NOT HARDENING. Every arm marked RED BEFORE below failed on
// the pre-fix header, with the measured pre-fix value quoted in the arm.
//
// THE DEFECT. `dAttackRadialSimulation::DerivedState()` seeded `attackHits` and
// `guardHits` with FOUR default-constructed entries — `attackHits(4)` is a RESIZE, not
// a reserve — and `collisionCheck`'s first line is
//     if (derivedState.editAttackHits().size() >= 4) return;
// a genuine "max 4 distinct targets per swing" cap (the container accumulates across the
// whole swing, deduped by rootBodyId, and is cleared only in `deactivate`). A freshly
// constructed DerivedState therefore arrived at that cap ALREADY SATISFIED, and
// collisionCheck became a silent no-op.
//
// THE REACHING PATH, WALKED END TO END BY THE RIG BELOW (not asserted from a fresh
// DerivedState in isolation — the value of this task is that the path is real):
//   1. SimulatableBrawler::integrate runs dAttackMachineSimulation::integrate3 BEFORE
//      dAttackRadialSimulation::integrate, in the same tick.
//   2. From Idle with attackLeft the machine picks a sequence (0 here, see the fixture
//      note) and propagates it into the radial InitialConditions.
//   3. Radial's `deactivate` branch is skipped — it needs
//      `ic.activeAttackSequence == InvalidAttackSequenceId`, and the machine has just
//      written a VALID one. `setInitialConditions` is skipped too, because
//      `ic.activeAttackSequence == 0 == state.currenSequenceId`, the default. Neither
//      function clears the hit containers anyway; `deactivate` is the only clear site.
//   4. collisionCheck early-returns on the seeded size for the WHOLE swing, until
//      `deactivate` finally fires at attackTimer >= duration.
//
// ⭐ The reaching path is WIDER than the backlog's derivation says. Step 3's
// `deactivate` skip does NOT depend on the two ids being equal — it depends only on the
// machine having written a valid sequence id, which it does for every first attack. The
// `== 0` coincidence only decides whether `setInitialConditions` runs, and that function
// does not clear either container. `FirstTickSwingRegistersHitsForwardSequence` is the
// arm that says so: it drives sequence 4 instead, so the ids DIFFER, setInitialConditions
// DOES run — and the swing was still blind before the fix.
//
// THE FIX. `DerivedState()` now RESERVES 4 in each container instead of resizing to 4,
// so `size()` means what the guard reads it as: "hits recorded during the current
// swing". The guard itself is untouched — it is a real cap, not an accident.
// ===========================================================================

namespace dattackradialfirstticktests
{

static constexpr float kDt = 1.f / 60.f;

// The struck character's actor-level id. Deliberately NOT 0: a default-constructed
// DAttackHit carries BodyId{0}, so a target at 0 could not be told apart from a phantom.
static constexpr std::uint32_t kTargetRootBody = 42u;

// The radial sub-simulation's query volume, so the scripted overlap report reaches the
// radial sub-sim and NOT the guard/projectile/movement sub-sims that also run this tick.
static constexpr std::uint32_t kRadialVolume = 7u;

// ---------------------------------------------------------------------------
// Mocks — deliberately local to this TU, matching the convention of the sibling radial
// test files (task 32's and task 33's each carry their own pair).
//
// getBodyTransform returns the IDENTITY for every body and setBodyTransform is dropped.
// That is a fixture choice, not laziness: the attacker's weapon then sits at the origin
// with no rotation for the whole run, so `currentDirection` is defaultForward (1,0,0) on
// every tick and the segment lookup cannot drift into becoming a second, uncontrolled
// discriminator. `addBodyTorque` is a no-op for the same reason — nothing spins.
// ---------------------------------------------------------------------------

struct MockPhysicsAdapter
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

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

struct MockSpatialQueryAdapter
{
    SpatialQueryReport report;

    // ⛔ GATED ON THE VOLUME LIST. Several sub-simulations reach the query adapter during
    // one SimulatableBrawler::integrate; an ungated mock would hand this report to all of
    // them. Only the radial sub-sim's volume gets it.
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        const bool isRadial = std::find(volumeIds.begin(), volumeIds.end(),
                                        QueryVolumeId{ kRadialVolume }) != volumeIds.end();
        return isRadial ? report : SpatialQueryReport{};
    }

    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

// What one run of the rig observes. Both hit COUNTS are reported: the total exposes the
// four phantom entries directly, while `attackHitsOnTarget` is the gameplay observable
// and is immune to them — a swing that never fires reads 0 on target whether the
// container holds four phantoms or nothing at all.
struct FirstTickOutcome
{
    DAttackState  machineState        = DAttackState::Idle;
    unsigned int  machineSequence     = InvalidAttackSequenceId;
    unsigned int  radialSequenceId    = InvalidAttackSequenceId;
    float         radialAttackTimer   = 0.f;
    std::size_t   attackHitsTotal     = 0;
    std::size_t   attackHitsOnTarget  = 0;
    std::size_t   guardHitsTotal      = 0;
};

// ===========================================================================
// THE RIG — a brand-new character, attacking from its very first integrate call.
//
// `SimulatableBrawler` is constructed here and integrated `tickCount` times with the
// attack input HELD. Nothing pre-warms it: no prior tick, no manual clear, no direct
// DerivedState surgery. That is the whole point — the state under test is the state a
// character is actually in on its first tick.
//
// FIXTURE NOTES (production StaticData is used verbatim — sequences, circle, all of it):
//   * aim (1,0,0) makes setRadialSimulationInitialConditions write initialAimAngle 0
//     about +Z, i.e. an IDENTITY initial rotation, so the sequence's own rotation axis
//     reaches collisionCheck unrotated.
//   * The move stick selects the sequence through dAttackDirection::classify: move
//     (0,-1,0) against aim (1,0,0) is a 90-degree SIDE case with a negative signed angle
//     → sequence 0, which is the value `State::currenSequenceId` also defaults to. A zero
//     stick classifies FORWARD → sequence 4 instead; both are exercised below.
//   * The target sits at (targetX, 0, 0). The production attack circle is inner 90 /
//     outer 300 / thickness 70, and the weapon is at the origin, so targetX 150 is inside
//     the annulus and 0 cm off the swing plane — comfortably inside the 35 cm half
//     thickness. targetX 500 is outside the outer radius; that is the negative control.
//   * Production sequence 0 runs from -pi/2 to +3pi/8 over 0.6 s. The weapon never
//     rotates in this rig (see the mock note), so the current direction stays at
//     defaultForward, which lands in the segment starting at -pi/8: state Damaging.
//     Both the swing's segment and the hit's segment resolve there, so the
//     `currentAttackSegment.index != hitAttackSegment.index` gate is satisfied and the
//     only surviving discriminators are the annulus and the half-thickness.
// ===========================================================================

static FirstTickOutcome runFirstTicks(int tickCount, float targetX, const glm::vec2& moveStick)
{
    simulatableBrawler::StaticData staticData;
    SimulatableBrawler character(staticData);
    character.setCharacterBindings({ BodyId{ 1u } });

    // Registration normally stamps these; the only field that matters here is the query
    // volume, which routes the scripted overlap report to the radial sub-sim alone.
    character.editPhysicsComposite()
        .edit<dAttackRadialSimulation::PhysicsDeclaration>()
        .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };

    MockPhysicsAdapter      physAdapter;
    MockSpatialQueryAdapter queryAdapter;

    SpatialQueryHit hit{};
    hit.objectPosition   = glm::vec3(targetX, 0.f, 0.f);
    hit.bodyId           = BodyId{ kTargetRootBody };
    hit.rootBodyId       = BodyId{ kTargetRootBody };
    hit.objectCategories = CollisionCategories::single(collisionCategory::body);
    queryAdapter.report.hits.push_back(hit);

    const glm::vec3 aim(1.f, 0.f, 0.f);
    const glm::vec3 moveWorld(moveStick.x, moveStick.y, 0.f);

    const simulatableBrawler::PlayerInput input(
        dAttackRadialSimulation::PlayerInput(aim, /*attackLeft*/ true, /*attackRight*/ false),
        dAttackMachineSimulation::PlayerInput{ aim, true, false, moveStick, moveWorld },
        dAttackGuardSimulation::PlayerInput(aim),
        brawlerProjectileSimulation::PlayerInput{ aim },
        brawlerMovementSimulation::PlayerInput{});

    for (int tick = 0; tick < tickCount; ++tick)
    {
        const SimulationTimeStep step(static_cast<std::uint32_t>(tick), false, false, false, kDt);
        character.integrate(step, input, physAdapter, queryAdapter, staticData);
    }

    const auto& allState = character.getAllState();
    const auto& radialDerived =
        allState.getDerivedState().get<dAttackRadialSimulation::DerivedState>();
    const auto& machineState = allState.getState().get<dAttackMachineSimulation::State>();
    const auto& radialState  = allState.getState().get<dAttackRadialSimulation::State>();

    FirstTickOutcome out{};
    out.machineState      = machineState.m_currentState;
    out.machineSequence   = machineState.m_activeAttackSequence;
    out.radialSequenceId  = radialState.currenSequenceId;
    out.radialAttackTimer = radialState.attackTimer;
    out.attackHitsTotal   = radialDerived.getAttackHits().size();
    out.guardHitsTotal    = radialDerived.getGuardHits().size();
    out.attackHitsOnTarget = static_cast<std::size_t>(
        std::count_if(radialDerived.getAttackHits().begin(),
                      radialDerived.getAttackHits().end(),
                      [](const dAttackRadialSimulation::DAttackHit& h)
                      { return h.hitRootBodyId == BodyId{ kTargetRootBody }; }));
    return out;
}

} // namespace dattackradialfirstticktests

// ===========================================================================
// ⭐ 1. THE ACCEPTANCE CRITERION — ONE TICK, FROM NOTHING, AND THE SWING CONNECTS.
//
// RED BEFORE THE FIX. Measured on the pre-fix header, 2026-09-04:
//     attackHitsOnTarget = 0   (expected 1)
//     attackHitsTotal    = 4   (the four phantom entries, untouched)
//     guardHitsTotal     = 4   (ditto)
// The premise assertions above the observable are what make this a walk of the reaching
// path rather than a restatement of it: they say the machine really did transition out of
// Idle on this tick, really did hand the radial sub-sim a valid sequence, and that the
// radial sub-sim really is mid-swing rather than idle or deactivated.
// ===========================================================================

TEST_CASE("DAttackRadial.FirstTickSwingRegistersHits", "[DAttack][RadialFirstTick]")
{
    using namespace dattackradialfirstticktests;

    const FirstTickOutcome out = runFirstTicks(/*tickCount*/ 1, /*targetX*/ 150.f,
                                               /*moveStick*/ glm::vec2(0.f, -1.f));

    // --- the reaching path's premises, pinned so a future change that stops the path
    //     reaching cannot leave this case passing for the wrong reason ---
    INFO("machine seq = " << out.machineSequence << ", radial curSeq = " << out.radialSequenceId);
    REQUIRE(out.machineState == DAttackState::Attacking);
    REQUIRE(out.machineSequence == 0u);
    // Equal to State::currenSequenceId's default, which is what makes `integrate` skip
    // setInitialConditions on this tick as well as `deactivate`.
    REQUIRE(out.radialSequenceId == 0u);
    REQUIRE(out.radialAttackTimer == Catch::Approx(kDt).margin(1e-6f));

    // --- the observable ---
    INFO("attackHitsOnTarget = " << out.attackHitsOnTarget
         << " (pre-fix: 0 — collisionCheck early-returned on four phantom entries)");
    REQUIRE(out.attackHitsOnTarget == 1u);
    REQUIRE(out.attackHitsTotal == 1u);      // pre-fix: 4
    REQUIRE(out.guardHitsTotal == 0u);       // pre-fix: 4
}

// ===========================================================================
// ⭐ 2. THE SAME DEFECT WITH THE IDS NOT EQUAL — the path is wider than documented.
//
// A zero move stick classifies FORWARD, so the machine writes sequence 4 while
// `State::currenSequenceId` is still its default 0. The ids DIFFER, so `integrate` takes
// the `setInitialConditions` branch this time — and that function does not clear either
// container, so the swing was blind here too.
//
// RED BEFORE THE FIX: attackHitsOnTarget = 0, attackHitsTotal = 4, guardHitsTotal = 4.
//
// Sequence 4 is the vertical one: rotation axis (0,1,0), points from -pi to +1.5pi/8 over
// 0.42 s. Its plane contains the world X axis, so a target on that axis 150 cm out is
// still 0 cm off the swing plane and inside the same 90..300 annulus.
// ===========================================================================

TEST_CASE("DAttackRadial.FirstTickSwingRegistersHitsForwardSequence", "[DAttack][RadialFirstTick]")
{
    using namespace dattackradialfirstticktests;

    const FirstTickOutcome out = runFirstTicks(/*tickCount*/ 1, /*targetX*/ 150.f,
                                               /*moveStick*/ glm::vec2(0.f, 0.f));

    INFO("machine seq = " << out.machineSequence << ", radial curSeq = " << out.radialSequenceId);
    REQUIRE(out.machineState == DAttackState::Attacking);
    REQUIRE(out.machineSequence == 4u);
    // setInitialConditions DID run this time — it is what wrote this value.
    REQUIRE(out.radialSequenceId == 4u);

    INFO("attackHitsOnTarget = " << out.attackHitsOnTarget << " (pre-fix: 0)");
    REQUIRE(out.attackHitsOnTarget == 1u);
    REQUIRE(out.attackHitsTotal == 1u);      // pre-fix: 4
    REQUIRE(out.guardHitsTotal == 0u);       // pre-fix: 4
}

// ===========================================================================
// 3. THE WHOLE FIRST SWING, NOT JUST ITS FIRST TICK — and the negative control.
//
// The failure mode the backlog describes is "silently registers NO hits FOR THAT WHOLE
// SWING", because nothing clears the containers until `deactivate` fires at
// attackTimer >= duration. Ten ticks (0.167 s, well inside sequence 0's 0.6 s) is the
// span; the count stays at exactly one because collisionCheck dedupes by rootBodyId.
//
// The out-of-range arm is the control that stops the in-range arm from being vacuous: it
// proves the rig can still report ZERO on-target hits when the geometry says it should,
// so a 1 above is the swing connecting and not the rig counting anything it is handed.
// Both arms read 0 on target before the fix — which is why the control is also stated in
// terms of the TOTAL, where the two differ (0 post-fix vs 4 pre-fix).
// ===========================================================================

TEST_CASE("DAttackRadial.FirstSwingKeepsRegisteringForItsWholeDuration", "[DAttack][RadialFirstTick]")
{
    using namespace dattackradialfirstticktests;

    SECTION("target inside the annulus — one hit, deduped across ten ticks")
    {
        const FirstTickOutcome out = runFirstTicks(10, 150.f, glm::vec2(0.f, -1.f));
        INFO("attackHitsOnTarget = " << out.attackHitsOnTarget << " (pre-fix: 0 for all ten ticks)");
        REQUIRE(out.attackHitsOnTarget == 1u);
        REQUIRE(out.attackHitsTotal == 1u);
    }

    SECTION("NEGATIVE CONTROL — target 500 cm out, past the 300 cm outer radius")
    {
        const FirstTickOutcome out = runFirstTicks(10, 500.f, glm::vec2(0.f, -1.f));
        REQUIRE(out.attackHitsOnTarget == 0u);
        // Post-fix the container is genuinely EMPTY, not "four phantoms and no real hit".
        REQUIRE(out.attackHitsTotal == 0u);   // pre-fix: 4
    }
}

// ===========================================================================
// 4. ⛔ THE FENCE — a fresh DerivedState RESERVES, IT DOES NOT RESIZE.
//
// This is the one case stated directly on the type rather than through the simulation,
// and it exists to stop the defect being reintroduced by a change that looks harmless.
// `size() == 0` is the property `collisionCheck`'s cap depends on; `capacity() >= 4` is
// the property the original `attackHits(4)` was reaching for, and the one
// SimulatableBrawlerTest's "the slice ctor really ran" case now anchors on.
//
// RED BEFORE THE FIX on the size assertions (both read 4); GREEN on the capacity ones,
// deliberately — a resize reserves too, so those arms do not discriminate and are here
// only to keep a future "fix" from deleting the reserve along with the resize.
// ===========================================================================

TEST_CASE("DAttackRadial.FreshDerivedStateIsEmptyButReserved", "[DAttack][RadialFirstTick]")
{
    const dAttackRadialSimulation::DerivedState fresh;

    REQUIRE(fresh.getAttackHits().empty());     // pre-fix: size 4
    REQUIRE(fresh.getGuardHits().empty());      // pre-fix: size 4
    REQUIRE(fresh.getAttackHits().capacity() >= 4u);
    REQUIRE(fresh.getGuardHits().capacity() >= 4u);

    // The copy ctor must not smuggle the phantoms back in either.
    const dAttackRadialSimulation::DerivedState copied(fresh);
    REQUIRE(copied.getAttackHits().empty());
    REQUIRE(copied.getGuardHits().empty());
}

#endif // WITH_LOW_LEVEL_TESTS
