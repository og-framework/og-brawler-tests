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
#include <string>
#include <utility>
#include <vector>

#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"

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
        brawlerMovementSimulation::PlayerInput{},
        // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
        // composite. No field, no wire cost: the input composite is still 77 B and
        // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
        // every sub-sim name an InputType it OWNS.
        brawlerRingout::PlayerInput{});

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


// ===========================================================================
// [movement-sim task 83] THE SWING-TANGENT RUNNER.
//
// A second runner beside runFirstTicks above, deliberately NOT a widening of it: task 34's
// four cases are a closed record of what that fix did, and their fixture (a target on the
// +X axis, a fixed three-value signature) is part of that record. This one varies the
// target POSITION, because the tangent is a function of where on the swing the hit landed
// and a target on the axis of symmetry would let a wrong cross product look right.
//
// Same mocks, same fixture reasoning, so the same fixture notes apply -- see runFirstTicks.
// One addition: the admissible hit directions are bounded by the SEGMENT GATE. The weapon
// never rotates here, so currentDirection stays at defaultForward (angle 0) and the swing
// sits in production sequence 0's segment spanning -pi/8 .. +pi/8. A hit outside that
// wedge resolves to a different segment index and is dropped -- correctly, but for a
// reason that has nothing to do with the tangent. Every target below is inside it.
struct SwingOutcome
{
    DAttackState  machineState       = DAttackState::Idle;
    unsigned int  radialSequenceId   = InvalidAttackSequenceId;
    bool          registered         = false;
    std::size_t   attackHitsTotal    = 0;
    std::uint32_t hitTick            = 0u;
    float         attackTimerAtHit   = 0.f;
    float         authoredOmegaAtHit = 0.f;
    glm::vec3     swingTangent{ 0.f };
    glm::vec3     hitDirectionOnPlane{ 0.f };
    glm::vec3     rotationAxis{ 0.f };

    // The whole swing, tick by tick, for the chained-sequence question: which sequence the
    // radial sub-sim was running and how many entries its per-swing ledger held.
    std::vector<std::pair<unsigned int, std::size_t>> trace;
};

// `hitFromTick` is NOT a convenience. The sign of the tangent is read from the sequence's
// AUTHORED angular velocity at state.attackTimer, and every shipped sequence starts FROM
// REST -- getAngularVelocity(0) is exactly 0 for all five. A hit registered on tick 0
// therefore reads a sign of zero, and both swings come out with the same tangent. That is
// an artefact of the never-rotating weapon in this fixture and not something production
// can produce: there, setInitialConditions places the weapon at the sequence's initial
// angle, which lies in a WindUp segment, so collisionCheck early-returns for the whole
// wind-up. Holding the overlap back until the timer is genuinely inside the Damaging span
// makes the sign a measurement instead of a coin toss.
// See the implementation note for task 83 for the narrow production case this uncovered:
// the Damaging gate reads the BODY's direction while the sign reads the TABLE's clock,
// and nothing forces those two to agree.
static SwingOutcome runSwing(int tickCount, glm::vec3 targetPosition,
                             const glm::vec2& moveStick, std::uint32_t hitFromTick = 18u)
{
    simulatableBrawler::StaticData staticData;
    SimulatableBrawler character(staticData);
    character.setCharacterBindings({ BodyId{ 1u } });
    character.editPhysicsComposite()
        .edit<dAttackRadialSimulation::PhysicsDeclaration>()
        .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };

    MockPhysicsAdapter      physAdapter;
    MockSpatialQueryAdapter queryAdapter;

    SpatialQueryHit hit{};
    hit.objectPosition   = targetPosition;
    hit.bodyId           = BodyId{ kTargetRootBody };
    hit.rootBodyId       = BodyId{ kTargetRootBody };
    hit.objectCategories = CollisionCategories::single(collisionCategory::body);
    const SpatialQueryReport liveReport{ { hit } };

    const glm::vec3 aim(1.f, 0.f, 0.f);
    const glm::vec3 moveWorld(moveStick.x, moveStick.y, 0.f);

    const simulatableBrawler::PlayerInput input(
        dAttackRadialSimulation::PlayerInput(aim, /*attackLeft*/ true, /*attackRight*/ false),
        dAttackMachineSimulation::PlayerInput{ aim, true, false, moveStick, moveWorld },
        dAttackGuardSimulation::PlayerInput(aim),
        brawlerProjectileSimulation::PlayerInput{ aim },
        brawlerMovementSimulation::PlayerInput{},
        // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
        // composite. No field, no wire cost: the input composite is still 77 B and
        // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
        // every sub-sim name an InputType it OWNS.
        brawlerRingout::PlayerInput{});

    SwingOutcome out{};

    for (int tick = 0; tick < tickCount; ++tick)
    {
        queryAdapter.report = (static_cast<std::uint32_t>(tick) >= hitFromTick)
            ? liveReport : SpatialQueryReport{};

        // collisionCheck reads state.attackTimer BEFORE integrate advances it, so the timer
        // the tangent's angular velocity was sampled at is the one standing here.
        const float timerBefore = character.getAllState().getState()
            .get<dAttackRadialSimulation::State>().attackTimer;

        const SimulationTimeStep step(static_cast<std::uint32_t>(tick), false, false, false, kDt);
        character.integrate(step, input, physAdapter, queryAdapter, staticData);

        const auto& radialDerived = character.getAllState().getDerivedState()
            .get<dAttackRadialSimulation::DerivedState>();
        const auto& radialState = character.getAllState().getState()
            .get<dAttackRadialSimulation::State>();

        out.trace.emplace_back(radialState.currenSequenceId, radialDerived.getAttackHits().size());

        if (!out.registered && !radialDerived.getAttackHits().empty())
        {
            const dAttackRadialSimulation::DAttackHit& registered =
                radialDerived.getAttackHits().front();
            out.registered       = true;
            out.hitTick          = static_cast<std::uint32_t>(tick);
            out.attackTimerAtHit = timerBefore;
            out.swingTangent     = registered.swingTangent;

            const unsigned int sequenceId = character.getAllState().getState()
                .get<dAttackRadialSimulation::InitialConditions>().activeAttackSequence;
            const DAttackRadialSequence& sequence = staticData.m_attackSequences[sequenceId];
            // initialAimAngle is 0 about +Z for aim (1,0,0), so the initial rotation is the
            // identity and the sequence's own axis IS the world axis. Stated rather than
            // assumed: the projection below only reduces to this under that fixture.
            out.rotationAxis        = sequence.getRotationAxis();
            out.authoredOmegaAtHit  = sequence.getAngularVelocity(timerBefore);
            out.hitDirectionOnPlane = targetPosition
                - glm::dot(targetPosition, out.rotationAxis) * out.rotationAxis;
        }
    }

    const auto& allState = character.getAllState();
    out.machineState     = allState.getState()
        .get<dAttackMachineSimulation::State>().m_currentState;
    out.radialSequenceId = allState.getState()
        .get<dAttackRadialSimulation::State>().currenSequenceId;
    out.attackHitsTotal  = allState.getDerivedState()
        .get<dAttackRadialSimulation::DerivedState>().getAttackHits().size();
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


// ===========================================================================
// [movement-sim task 83] THE SWING TANGENT -- the direction the weapon is TRAVELLING.
//
// The user, after PIE of task 27: "the knockback should happen in the direction that is
// orthogonal to the weapon at the moment of the hit". Task 27 shipped the direction the
// design asked for -- away from the attacker, capsule to capsule -- so this is a gap in
// the entry, not in that implementation.
//
// The tangent is built at the push site as cross(axis, rHat) signed by the sequence's
// AUTHORED angular velocity. The three cases below split that sentence into the three
// things that can independently be wrong: the PLANE it lies in, the SIGN it carries, and
// the derivative the sign is read from.
// ===========================================================================

TEST_CASE("DAttackRadial.SwingTangentIsOrthogonalToTheWeapon", "[DAttack][SwingTangent]")
{
    using namespace dattackradialfirstticktests;

    // 15 degrees off the weapon line: inside the +/- 22.5 degree segment wedge, and far
    // enough off the axis of symmetry that a tangent built from the wrong operand order or
    // from the unprojected hit vector would not coincidentally land on the right answer.
    const float theta = glm::radians(15.f);
    const glm::vec3 target(150.f * glm::cos(theta), 150.f * glm::sin(theta), 0.f);

    const SwingOutcome out = runSwing(/*tickCount*/ 24, target, /*moveStick*/ glm::vec2(0.f, -1.f));

    REQUIRE(out.registered);
    REQUIRE(out.attackHitsTotal == 1u);

    INFO("tangent = (" << out.swingTangent.x << ", " << out.swingTangent.y << ", "
         << out.swingTangent.z << "); axis = (" << out.rotationAxis.x << ", "
         << out.rotationAxis.y << ", " << out.rotationAxis.z << "); omega = "
         << out.authoredOmegaAtHit << " at t = " << out.attackTimerAtHit);

    // 1. IT IS A UNIT VECTOR, and it is not a NaN. Hit routing normalises its XY part and
    //    assigns the result straight into a velocity, so a NaN here never leaves the body.
    REQUIRE(glm::length(out.swingTangent) == Catch::Approx(1.f).margin(1e-5f));
    REQUIRE(out.swingTangent.x == out.swingTangent.x);
    REQUIRE(out.swingTangent.y == out.swingTangent.y);
    REQUIRE(out.swingTangent.z == out.swingTangent.z);

    // 2. IT LIES IN THE SWING PLANE: orthogonal to the rotation axis AND to the radial
    //    vector from the attacker to the hit. Those two statements together ARE "tangent to
    //    the arc", and neither alone is: a vector orthogonal to the radius only is any
    //    vector in the plane through the hit, and one orthogonal to the axis only is any
    //    vector in the swing plane.
    REQUIRE(glm::dot(out.swingTangent, out.rotationAxis) == Catch::Approx(0.f).margin(1e-5f));
    REQUIRE(glm::dot(out.swingTangent, glm::normalize(out.hitDirectionOnPlane))
            == Catch::Approx(0.f).margin(1e-5f));

    // 3. AND IT IS THE RIGHT ONE OF THE TWO. Orthogonality admits both signs; the authored
    //    angular velocity picks between them.
    const glm::vec3 expected =
        glm::cross(out.rotationAxis, glm::normalize(out.hitDirectionOnPlane))
        * (out.authoredOmegaAtHit < 0.f ? -1.f : 1.f);
    REQUIRE(out.swingTangent.x == Catch::Approx(expected.x).margin(1e-5f));
    REQUIRE(out.swingTangent.y == Catch::Approx(expected.y).margin(1e-5f));
    REQUIRE(out.swingTangent.z == Catch::Approx(expected.z).margin(1e-5f));
}

// ---------------------------------------------------------------------------
// The SIGN, and it is checked against the angle's own motion rather than against a
// restatement of the rule. Sequence 0 and sequence 1 are authored as mirror images, so a
// sign convention that were inverted would show up here as two throws in the SAME
// direction rather than as two wrong-but-opposite ones.
// ---------------------------------------------------------------------------
TEST_CASE("DAttackRadial.SwingTangentFollowsTheSwingDirection", "[DAttack][SwingTangent]")
{
    using namespace dattackradialfirstticktests;

    // A target on the weapon line. rHat is then exactly +X and cross(+Z, +X) is exactly +Y,
    // so the two arms differ in one component and in nothing else.
    const glm::vec3 target(150.f, 0.f, 0.f);

    // The stick against the aim decides which sequence the machine picks -- (0,-1) is the
    // right swing (id 0), (0,+1) the left (id 1).
    const SwingOutcome right = runSwing(24, target, glm::vec2(0.f, -1.f));
    const SwingOutcome left  = runSwing(24, target, glm::vec2(0.f,  1.f));

    REQUIRE(right.registered);
    REQUIRE(left.registered);

    simulatableBrawler::StaticData staticData;
    const DAttackRadialSequence& rightSeq =
        staticData.m_attackSequences[dAttackDirection::kRightSequenceId];
    const DAttackRadialSequence& leftSeq =
        staticData.m_attackSequences[dAttackDirection::kLeftSequenceId];

    INFO("right tangent (" << right.swingTangent.x << ", " << right.swingTangent.y
         << ", " << right.swingTangent.z << ") omega " << right.authoredOmegaAtHit
         << " | left tangent (" << left.swingTangent.x << ", " << left.swingTangent.y
         << ", " << left.swingTangent.z << ") omega " << left.authoredOmegaAtHit);

    // 1. THE PREMISE, MEASURED ON THE TABLE: the two sequences really do sweep opposite
    //    ways at the moment each hit landed. Read off getAngle, not off getAngularVelocity,
    //    so the sign the tangent used is checked against the ANGLE and not against the
    //    function that is supposed to differentiate it.
    const float rightSweep = rightSeq.getAngle(right.attackTimerAtHit + kDt)
                           - rightSeq.getAngle(right.attackTimerAtHit);
    const float leftSweep  = leftSeq.getAngle(left.attackTimerAtHit + kDt)
                           - leftSeq.getAngle(left.attackTimerAtHit);
    INFO("sweep over one tick: right " << rightSweep << ", left " << leftSweep);
    REQUIRE(rightSweep > 0.f);
    REQUIRE(leftSweep  < 0.f);

    // ...and the authored velocity the tangent actually READ is non-zero, so its sign is a
    //     measurement. Every sequence starts from rest, and a zero sign is not a direction.
    REQUIRE(right.authoredOmegaAtHit > 0.f);
    REQUIRE(left.authoredOmegaAtHit  < 0.f);

    // 2. AND THE TANGENTS FOLLOW THEM. cross(axis, rHat) is the direction of increasing
    //    angle, so a positive sweep keeps it and a negative one flips it.
    const glm::vec3 increasing =
        glm::cross(glm::vec3(0.f, 0.f, 1.f), glm::vec3(1.f, 0.f, 0.f));
    REQUIRE(increasing.y == Catch::Approx(1.f).margin(1e-6f));   // the fixture's own claim

    REQUIRE(right.swingTangent.y == Catch::Approx( 1.f).margin(1e-5f));
    REQUIRE(left.swingTangent.y  == Catch::Approx(-1.f).margin(1e-5f));
    REQUIRE(right.swingTangent.x == Catch::Approx(0.f).margin(1e-5f));
    REQUIRE(left.swingTangent.x  == Catch::Approx(0.f).margin(1e-5f));

    // 3. MIRRORED, which is the user-visible property: a left hit and a right hit throw
    //    opposite ways, and neither throws the target into the weapon.
    REQUIRE(glm::dot(right.swingTangent, left.swingTangent)
            == Catch::Approx(-1.f).margin(1e-5f));
}

// ---------------------------------------------------------------------------
// getAngularVelocity IS the derivative of getAngle, and it is the AUTHORED one. The sign
// the tangent reads comes from here; if this walked the segments differently from getAngle
// it would read a neighbouring segment's velocity near a point boundary and flip a throw.
// ---------------------------------------------------------------------------
TEST_CASE("DAttackRadial.GetAngularVelocityIsTheDerivativeOfGetAngle", "[DAttack][SwingTangent]")
{
    simulatableBrawler::StaticData staticData;

    for (std::size_t sequenceId = 0; sequenceId < staticData.m_attackSequences.size(); ++sequenceId)
    {
        const DAttackRadialSequence& sequence = staticData.m_attackSequences[sequenceId];
        const float duration = sequence.getDuration();

        // A CENTRED difference, which is exact for a quadratic: the angle inside a segment
        // is o0 + w0*dt + a*dt*dt/2, so (o(t+h) - o(t-h)) / 2h equals w(t) to rounding.
        // Samples astride an authored point are skipped -- the acceleration STEPS there and
        // no finite difference can be expected to agree. The skip test is the piecewise
        // constant acceleration itself, so the case never has to restate the authored times.
        const float h = 1e-4f;
        const float band = 2e-3f;

        for (int i = 1; i < 200; ++i)
        {
            const float t = duration * static_cast<float>(i) / 200.f;
            if (sequence.getAngularAcceleration(t - band)
                != sequence.getAngularAcceleration(t + band))
                continue;

            const float finite =
                (sequence.getAngle(t + h) - sequence.getAngle(t - h)) / (2.f * h);
            const float authored = sequence.getAngularVelocity(t);
            INFO("sequence " << sequenceId << " t=" << t << " finite=" << finite
                 << " authored=" << authored);
            REQUIRE(authored == Catch::Approx(finite).margin(1e-2f));
        }

        // OUTSIDE the authored span it clamps the way getAngle does -- the front velocity
        // before the first point, the back velocity after the last.
        REQUIRE(sequence.getAngularVelocity(-1.f)
                == Catch::Approx(sequence.getInitialVelocity()).margin(1e-6f));
        // The constructor appends a point that brings the velocity to zero at getDuration(),
        // so the after-the-end value is that zero rather than the last authored sweep.
        INFO("sequence " << sequenceId << " velocity past the end = "
             << sequence.getAngularVelocity(duration + 1.f));
        REQUIRE(sequence.getAngularVelocity(duration + 1.f) == Catch::Approx(0.f).margin(1e-3f));
    }
}

// ===========================================================================
// [movement-sim task 83] THE CHAINED-SEQUENCE QUESTION -- WRITTEN TO DECIDE IT, NOT TO FIX
// IT. The architect's hazard (design section 4): the second swing of a chain enters through
// setInitialConditions, which does not clear attackHits, so it might be unable to re-hit a
// target the first swing already hit. If that reproduces it is ROUTED TO THE LEAD, because
// it is a pre-existing behaviour with its own design question (should a chain re-hit?) and
// not part of this task.
//
// The attack input is HELD, which is what queues sequence 2 once the first swing passes
// 0.3 s; the radial sub-sim deactivates at getDuration() and the machine starts the queued
// swing on the following tick. The trace records the sequence and the ledger size on every
// tick, so a failure here says WHICH of the two swings registered rather than only that a
// count was wrong.
// ===========================================================================

TEST_CASE("DAttackRadial.ChainedSwingCanHitTheSameTargetAgain", "[DAttack][SwingTangent]")
{
    using namespace dattackradialfirstticktests;

    const SwingOutcome out = runSwing(/*tickCount*/ 90, glm::vec3(150.f, 0.f, 0.f),
                                      /*moveStick*/ glm::vec2(0.f, -1.f));

    // The first swing connected -- the premise, and it is task 34's result restated here
    // only so a failure below cannot be a swing that never hit anything.
    REQUIRE(out.registered);

    bool sawSecondSequence = false;
    unsigned int secondSequenceId = InvalidAttackSequenceId;
    std::size_t hitsDuringSecondSequence = 0;
    for (const auto& sample : out.trace)
    {
        if (isRealAttackSequence(sample.first) && sample.first != 0u)
        {
            sawSecondSequence = true;
            secondSequenceId = sample.first;
            hitsDuringSecondSequence = std::max(hitsDuringSecondSequence, sample.second);
        }
    }

    std::string traceText;
    for (std::size_t i = 0; i < out.trace.size(); ++i)
    {
        traceText += std::to_string(i) + ":seq" + std::to_string(out.trace[i].first)
                   + "/hits" + std::to_string(out.trace[i].second) + " ";
    }
    INFO("trace " << traceText);
    INFO("second sequence = " << secondSequenceId
         << ", ledger during it = " << hitsDuringSecondSequence);

    REQUIRE(sawSecondSequence);
    REQUIRE(secondSequenceId == 2u);
    // THE QUESTION: the chained swing registers the same target again.
    REQUIRE(hitsDuringSecondSequence >= 1u);
}

#endif // WITH_LOW_LEVEL_TESTS
