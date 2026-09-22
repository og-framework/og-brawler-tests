// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/DAttackMachineSimulation.h"
// [Task 36] InvalidAttackSequenceId / kHadoukenSequenceSentinel relocated here; this TU
// references both constants directly (transitively visible, but made explicit).
#include "OGBrawler/DAttackSequenceId.h"
// [Task 35] CharacterBindings relocated here; the fake-bindings construction sites below
// (brace-init via setCharacterBindings) construct it directly.
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// Reuse the mock adapters from SimulatableBrawlerTest.cpp via local redeclaration.
// These are structurally identical — kept in this TU to avoid a shared header.
// ---------------------------------------------------------------------------
namespace integrate3tests
{

struct MockPhysicsAdapter
{
    glm::vec3 lastSetLinearVelocity{0.f};
    PhysicsBodyState capturedState{};

    // [Task 33] The machine sim's Hadouken trigger now resolves the parent capsule position
    // on-demand via getBodyTransform(characterBindings.capsuleBodyId). By default every body
    // resolves to identity (origin), preserving the pre-T33 behaviour where parentPosition was
    // the origin. A test can opt in to a non-origin capsule by setting capsuleBodyId +
    // capsuleTransform; only that exact id then returns the special transform.
    BodyId capsuleBodyId{0};
    glm::mat4 capsuleTransform{1.f};

    glm::mat4 getBodyTransform(BodyId id) const
    {
        if (capsuleBodyId.value != 0 && id == capsuleBodyId)
            return capsuleTransform;
        return glm::mat4(1.f);
    }
    void setBodyTransform(BodyId, const glm::mat4&) {}
    void addBodyTorque(BodyId, const glm::vec3&) {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void setBodyLinearVelocity(BodyId, const glm::vec3& v) { lastSetLinearVelocity = v; }
    // Task 3b force seam. No-op: this task adds the capability only; task 12's
    // movement-sim tests are the ones that record these calls.
    void addBodyAcceleration(BodyId, const glm::vec3&) {}
    void addBodyVelocityChange(BodyId, const glm::vec3&) {}
    glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

struct MockSpatialQueryAdapter
{
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }
    // Task 7 sweep seam. No-op: this task adds the capability only; task 12's
    // movement-sim tests are the ones that script sweeps and record these calls.
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

// Helper: build a default character + run one integrate tick with the given input.
// Returns the dAttackMachineSimulation::State after integration.
static dAttackMachineSimulation::State integrateOnce(
    const glm::vec3& aimDirection,
    const glm::vec3& moveDirectionWorld,
    const glm::vec2& moveStick,
    bool attackLeft,
    bool attackRight)
{
    simulatableBrawler::StaticData staticData;
    SimulatableBrawler character(staticData);
    // [Task 33] Stamp fake CharacterBindings (registration normally does this). The default
    // mock resolves any body to identity, so the resolved parentPosition is the origin —
    // identical to the pre-T33 behaviour where integrate read the same identity transform.
    character.setCharacterBindings({ BodyId{1} });

    MockPhysicsAdapter physAdapter;
    MockSpatialQueryAdapter queryAdapter;

    simulatableBrawler::PlayerInput input(
        dAttackRadialSimulation::PlayerInput(aimDirection, attackLeft, attackRight),
        dAttackMachineSimulation::PlayerInput{aimDirection, attackLeft, attackRight, moveStick, moveDirectionWorld},
        dAttackGuardSimulation::PlayerInput(aimDirection),
        brawlerProjectileSimulation::PlayerInput{aimDirection},
        brawlerMovementSimulation::PlayerInput{},
        // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
        // composite. No field, no wire cost: the input composite is still 77 B and
        // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
        // every sub-sim name an InputType it OWNS.
        brawlerRingout::PlayerInput{});

    SimulationTimeStep step(0u, false, false, false, 1.f / 60.f);
    character.integrate(step, input, physAdapter, queryAdapter, staticData);

    return character.getAllState().getState().get<dAttackMachineSimulation::State>();
}

} // namespace integrate3tests

// ---------------------------------------------------------------------------
// Per-segment attack-selection tests
// ---------------------------------------------------------------------------

TEST_CASE("DAttack.Integrate3.ForwardStrikeWhenMoveAlignsWithAim", "[DAttack][AimRelativeMove]")
{
    using namespace integrate3tests;
    // Aim east, move east: signed angle ≈ 0 → within ±π/6 window → seq 4 (forward strike).
    const glm::vec3 aim  = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 move = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    auto state = integrateOnce(aim, move, glm::vec2(0.f, -1.f), true, false);

    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == 4u);
}

TEST_CASE("DAttack.Integrate3.LeftStrikeWhenMove90DegCCWOfAim", "[DAttack][AimRelativeMove]")
{
    using namespace integrate3tests;
    // Aim east (+x), move north (+y): cross(aim_xy, move_xy).z = 1 > 0, angle = π/2 > π/6 → seq 1.
    const glm::vec3 aim  = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 move = glm::normalize(glm::vec3(0.f, 1.f, 0.f));
    auto state = integrateOnce(aim, move, glm::vec2(0.f, -1.f), true, false);

    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == 1u);
}

TEST_CASE("DAttack.Integrate3.RightStrikeWhenMove90DegCWOfAim", "[DAttack][AimRelativeMove]")
{
    using namespace integrate3tests;
    // Aim east (+x), move south (-y): cross(aim_xy, move_xy).z = -1 < 0, angle = π/2 > π/6 → seq 0.
    const glm::vec3 aim  = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 move = glm::normalize(glm::vec3(0.f, -1.f, 0.f));
    auto state = integrateOnce(aim, move, glm::vec2(0.f, -1.f), true, false);

    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == 0u);
}

TEST_CASE("DAttack.Integrate3.ForwardStrikeWhenStickNeutral", "[DAttack][AimRelativeMove]")
{
    using namespace integrate3tests;
    // Zero stick: length(moveDirection) < epsilon branch → seq 4 regardless of moveDirectionWorld.
    const glm::vec3 aim  = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 move = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    auto state = integrateOnce(aim, move, glm::vec2(0.f, 0.f), true, false);

    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == 4u);
}

TEST_CASE("DAttack.Integrate3.AimWithDownwardZAndAlignedMoveXYIsForwardStrike", "[DAttack][AimRelativeMove]")
{
    using namespace integrate3tests;
    // Regression pin: with mouse-aim the aim vector points from the character capsule
    // (offset above z=0) down to the mouse projection on the z=0 plane, so the 3D
    // aimDirection carries a substantial negative z component. If integrate3 computes
    // the angle from the 3D aim against the XY move (the pre-fix code did), the
    // downward z inflates the angle past the ±π/6 threshold even when the XY directions
    // are perfectly aligned. The sign comes from a 2D cross that flickers ±1 with FP
    // noise, so the user observes random left/right/forward strikes per attack press.
    //
    // The fix: angle is computed from the XY-projected aim, matching the sign's reference.
    // With aim_xy ≈ move_xy, dotXY = 1, angle = 0, signedAngle = 0 → seq 4 (forward).
    const glm::vec3 aim  = glm::normalize(glm::vec3(0.722f, 0.f, -0.692f)); // mouse 100u E, capsule 96 up
    const glm::vec3 move = glm::normalize(glm::vec3(1.f, 0.f, 0.f));        // XY-aligned with aim
    auto state = integrateOnce(aim, move, glm::vec2(0.f, -1.f), true, false);

    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == 4u);
}

TEST_CASE("DAttack.Integrate3.NoTransitionWithoutAttackInput", "[DAttack][AimRelativeMove]")
{
    using namespace integrate3tests;
    // No attack buttons pressed: state must remain Idle.
    const glm::vec3 aim  = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 move = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    auto state = integrateOnce(aim, move, glm::vec2(0.f, -1.f), false, false);

    REQUIRE(state.m_currentState == DAttackState::Idle);
    REQUIRE(state.m_activeAttackSequence == InvalidAttackSequenceId);
}

// ---------------------------------------------------------------------------
// Mode-agnostic-sim invariant lock
// ---------------------------------------------------------------------------

TEST_CASE("DAttack.Integrate3.IsModeAgnostic", "[DAttack][AimRelativeMove]")
{
    // Rot-prevention guardrail: integrate3 must produce the same output state for an
    // identical (aimDirection, moveDirectionWorld) pair regardless of which input scheme
    // produced those world vectors. The test constructs the world vectors directly —
    // it does NOT call any UE-side input collection — so the only way this can fail
    // is if someone sneaks a mode flag into PlayerInput or into integrate3 itself.
    //
    // The two "schemes" are simulated by passing the same world vectors twice with
    // a hypothetical second reference frame that would also produce them (here we
    // just re-use the same vectors, since the invariant is that the sim is blind to
    // their origin). The test is explicit about the contract being checked.
    using namespace integrate3tests;

    const glm::vec3 aimDirection       = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 moveDirectionWorld = glm::normalize(glm::vec3(0.f, 1.f, 0.f)); // 90° CCW → seq 1

    // Call integrate3 via two separate SimulatableBrawler instances, each receiving
    // the same world vectors. Both must produce identical attack sequences.
    const glm::vec2 stickA = glm::vec2(0.f, -1.f); // "aim-relative" stick
    const glm::vec2 stickB = glm::vec2(0.f, -1.f); // "camera-relative" stick (same world result)

    auto stateA = integrateOnce(aimDirection, moveDirectionWorld, stickA, true, false);
    auto stateB = integrateOnce(aimDirection, moveDirectionWorld, stickB, true, false);

    REQUIRE(stateA.m_currentState        == stateB.m_currentState);
    REQUIRE(stateA.m_activeAttackSequence == stateB.m_activeAttackSequence);
    // Confirm the expected attack sequence so a future mode-aware change is conspicuous.
    REQUIRE(stateA.m_activeAttackSequence == 1u);
}

// ---------------------------------------------------------------------------
// Known angle-segmentation artifact: anti-parallel singularity
// ---------------------------------------------------------------------------

TEST_CASE("DAttack.Integrate3.AntiParallelMoveIsForwardStrike_KnownArtifact", "[DAttack][AimRelativeMove]")
{
    // NOTE: this test pins *current* behavior, not desired behavior.
    // When moveDirectionWorld ≈ -aimDirection, glm::cross(aimXY, moveXY).z = 0,
    // so signedAngle = sign(0) * π = 0, which falls into the forward-strike window
    // (|signedAngle| < π/6). The result is seq 4 (forward strike) even though the
    // player pushed the stick directly away from the aim direction.
    // Future fix: detect the anti-parallel case inside integrate3 and treat it as
    // a no-direction or back-strike. That fix is mode-agnostic and will deliberately
    // update this test.
    using namespace integrate3tests;

    const glm::vec3 aim  = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 move = glm::normalize(glm::vec3(-1.f, 0.f, 0.f)); // exactly anti-parallel
    auto state = integrateOnce(aim, move, glm::vec2(0.f, -1.f), true, false);

    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == 4u); // forward strike — known artifact
}

// ---------------------------------------------------------------------------
// [Task 25] Hadouken commitment duration
// ---------------------------------------------------------------------------

TEST_CASE("DAttack.Integrate3.HadoukenCommitmentHoldsAttackingState", "[DAttack][Hadouken]")
{
    using namespace integrate3tests;

    // Regression for the PIE-found "Hadouken + immediate normal swing" bug (T25).
    // Without a commitment window the machine fires the Hadouken on tick T, drops back to
    // Idle on tick T+1 (the radial early-returns on the sentinel, leaving currenSequenceId ==
    // InvalidAttackSequenceId), and a still-held attack button chains a normal swing on T+2.
    // The kHadoukenCommitmentSeconds gate must keep the machine in the Hadouken-Attacking
    // state (sentinel active) for the full commitment window before the normal exit fires.

    simulatableBrawler::StaticData staticData;
    SimulatableBrawler character(staticData);
    character.setCharacterBindings({ BodyId{1} }); // [Task 33] fake bindings; mock → origin
    MockPhysicsAdapter physAdapter;
    MockSpatialQueryAdapter queryAdapter;

    const glm::vec3 aim = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const float dt = 1.f / 60.f;

    // Drive one composite tick with attackLeft held and the given matcher result, returning
    // the machine sub-state afterwards. Same construction path as integrateOnce(), but reuses
    // the SAME character across ticks so m_timeInCurrentState accumulates.
    auto runTick = [&](unsigned int tick, uint32_t triggeredActionId, bool attackLeft)
    {
        simulatableBrawler::PlayerInput input(
            dAttackRadialSimulation::PlayerInput(aim, attackLeft, false),
            dAttackMachineSimulation::PlayerInput{aim, attackLeft, false, glm::vec2(0.f, 0.f), aim, triggeredActionId},
            dAttackGuardSimulation::PlayerInput(aim),
            brawlerProjectileSimulation::PlayerInput{aim},
            brawlerMovementSimulation::PlayerInput{},
            // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
            // composite. No field, no wire cost: the input composite is still 77 B and
            // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
            // every sub-sim name an InputType it OWNS.
            brawlerRingout::PlayerInput{});

        SimulationTimeStep step(tick, false, false, false, dt);
        character.integrate(step, input, physAdapter, queryAdapter, staticData);
        return character.getAllState().getState().get<dAttackMachineSimulation::State>();
    };

    // Tick 0 — matcher reports the completed Hadouken (rising edge) with attackLeft held.
    auto state = runTick(0u, inputSequence::kHadoukenActionId, /*attackLeft*/ true);
    REQUIRE(state.m_currentState == DAttackState::Attacking);
    REQUIRE(state.m_activeAttackSequence == kHadoukenSequenceSentinel);

    // Ticks 1..15 — attackLeft STILL held but no new rising edge (triggeredActionId 0).
    // 15 * dt = 0.25 s < kHadoukenCommitmentSeconds (0.3 s): the machine must stay committed
    // to the Hadouken-Attacking state and must NOT exit to Idle or chain a normal swing.
    for (unsigned int tick = 1; tick <= 15; ++tick)
    {
        state = runTick(tick, 0u, /*attackLeft*/ true);
        INFO("commitment tick " << tick << " (t=" << tick * dt << "s)");
        REQUIRE(state.m_currentState == DAttackState::Attacking);
        REQUIRE(state.m_activeAttackSequence == kHadoukenSequenceSentinel);
    }

    // Past the commitment window the normal exit gate is allowed to fire. Keep attackLeft held
    // and run further ticks; the machine must leave the Hadouken-Attacking state (sentinel
    // cleared) — landing in Idle or chaining a normal swing is both acceptable.
    bool leftSentinel = false;
    for (unsigned int tick = 16; tick <= 40 && !leftSentinel; ++tick)
    {
        state = runTick(tick, 0u, /*attackLeft*/ true);
        if (state.m_activeAttackSequence != kHadoukenSequenceSentinel)
        {
            leftSentinel = true;
        }
    }
    REQUIRE(leftSentinel);
}

// ---------------------------------------------------------------------------
// [Task 33] CharacterBindings sources the Hadouken spawn position
// ---------------------------------------------------------------------------

TEST_CASE("DAttack.Integrate3.MachineHadoukenUsesCharacterBindings", "[DAttack][Hadouken]")
{
    using namespace integrate3tests;

    // Proves the T33 plumbing: the Hadouken projectile spawn position is derived from the
    // parent capsule transform resolved on-demand via CharacterBindings.capsuleBodyId — NOT
    // from a pre-resolved IntegrationUtils value. Point the mock's capsule body at a non-origin
    // transform and confirm the spawn position picks it up: spawnPos == capsulePos +
    // aimXY*spawnForwardOffset + (0,0,spawnZOffset).

    simulatableBrawler::StaticData staticData;
    SimulatableBrawler character(staticData);

    const BodyId capsuleId{7};
    character.setCharacterBindings({ capsuleId });

    MockPhysicsAdapter physAdapter;
    physAdapter.capsuleBodyId   = capsuleId;
    physAdapter.capsuleTransform = glm::mat4(1.f);
    const glm::vec3 capsulePos(10.f, 20.f, 30.f);
    physAdapter.capsuleTransform[3] = glm::vec4(capsulePos, 1.f);

    MockSpatialQueryAdapter queryAdapter;

    const glm::vec3 aim = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    simulatableBrawler::PlayerInput input(
        dAttackRadialSimulation::PlayerInput(aim, false, false),
        // triggeredActionId = kHadoukenActionId fires the machine's Hadouken trigger block.
        dAttackMachineSimulation::PlayerInput{aim, false, false, glm::vec2(0.f), aim,
                                              inputSequence::kHadoukenActionId},
        dAttackGuardSimulation::PlayerInput(aim),
        brawlerProjectileSimulation::PlayerInput{aim},
        brawlerMovementSimulation::PlayerInput{},
        // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
        // composite. No field, no wire cost: the input composite is still 77 B and
        // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
        // every sub-sim name an InputType it OWNS.
        brawlerRingout::PlayerInput{});

    // Drive at a non-zero tick so the spawned slot's spawnTick (== currentTick) is non-zero
    // (spawnTick 0 reads as a free slot). The projectile sub-sim runs after the machine in the
    // composite and CONSUMES the spawn request, so we assert against the spawned slot, not the IC.
    SimulationTimeStep step(5u, false, false, false, 1.f / 60.f);
    character.integrate(step, input, physAdapter, queryAdapter, staticData);

    const auto& projState =
        character.getAllState().getState().get<brawlerProjectileSimulation::State>();
    REQUIRE(projState.slots[0].spawnTick == 5u); // spawned this tick

    // spawnPos == capsulePos + aimXY*spawnForwardOffset + (0,0,spawnZOffset), with capsulePos
    // sourced from getBodyTransform(characterBindings.capsuleBodyId) — the T33 plumbing under test.
    const glm::vec3 expectedSpawn =
        capsulePos
        + glm::vec3(1.f, 0.f, 0.f) * staticData.m_projectileStaticData.spawnForwardOffset
        + glm::vec3(0.f, 0.f, staticData.m_projectileStaticData.spawnZOffset);
    REQUIRE(projState.slots[0].spawnPos.x == Catch::Approx(expectedSpawn.x));
    REQUIRE(projState.slots[0].spawnPos.y == Catch::Approx(expectedSpawn.y));
    REQUIRE(projState.slots[0].spawnPos.z == Catch::Approx(expectedSpawn.z));
}

// ---------------------------------------------------------------------------
// [hit-resolution T2] Inbound-hit signal drives HitFlinch
// ---------------------------------------------------------------------------

TEST_CASE("DAttack.Integrate3.InboundHitTransitionsToHitFlinch", "[DAttack][HitFlinch]")
{
    using namespace integrate3tests;

    // T2 acceptance: with the real inbound-hit plain param threaded through the composite,
    // flipping wasHitThisTick=true before a tick must transition the machine into HitFlinch,
    // veto the attack input on that tick, and cancel any active/queued sequences (the T1
    // cancellation behaviour). In production the manager routing pass (T3) owns the flag; here
    // the test plays that role by writing the DerivedState slice directly on the composite —
    // exactly the field SimulatableBrawler::integrate now forwards to integrate3 by-ref.

    simulatableBrawler::StaticData staticData;
    SimulatableBrawler character(staticData);
    character.setCharacterBindings({ BodyId{1} }); // fake bindings; mock resolves to origin
    MockPhysicsAdapter physAdapter;
    MockSpatialQueryAdapter queryAdapter;

    const glm::vec3 aim = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
    const float dt = 1.f / 60.f;

    // Drive one composite tick. inboundHit is written onto the composite DerivedState slice
    // BEFORE integrate (playing T3's routing-pass role); attackLeft is held to prove the veto.
    auto runTick = [&](unsigned int tick, bool inboundHit, bool attackLeft)
    {
        // ⚠ [movement-sim task 27] THE DWELL NOW ARRIVES WITH THE HIT. The machine used to
        // dwell on the file-scope `kHitFlinchDuration`; it dwells on `State::m_flinchDuration`,
        // copied from this slice by the veto, because the lockout is per attack. A test that
        // plays the routing pass's role must therefore resolve the WHOLE reaction, not just the
        // bool — and the duration is read from the shipped projectile spec (a Stun at 0.3 s,
        // today's value) rather than re-typed, so this case still measures the authored number.
        auto& slice = character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>();
        slice.wasHitThisTick = inboundHit;
        slice.reactionKind   = staticData.m_projectileHitReaction.kind;
        slice.flinchDuration = staticData.m_projectileHitReaction.lockoutDuration;
        simulatableBrawler::PlayerInput input(
            dAttackRadialSimulation::PlayerInput(aim, attackLeft, false),
            dAttackMachineSimulation::PlayerInput{aim, attackLeft, false, glm::vec2(0.f, 0.f), aim},
            dAttackGuardSimulation::PlayerInput(aim),
            brawlerProjectileSimulation::PlayerInput{aim},
            brawlerMovementSimulation::PlayerInput{},
            // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
            // composite. No field, no wire cost: the input composite is still 77 B and
            // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
            // every sub-sim name an InputType it OWNS.
            brawlerRingout::PlayerInput{});
        SimulationTimeStep step(tick, false, false, false, dt);
        character.integrate(step, input, physAdapter, queryAdapter, staticData);
        return character.getAllState().getState().get<dAttackMachineSimulation::State>();
    };

    // Tick 0 — inbound hit true WHILE attackLeft held. The veto must win: HitFlinch (not
    // Attacking), and the attack sequence must be cancelled (not started).
    auto state = runTick(0u, /*inboundHit*/ true, /*attackLeft*/ true);
    REQUIRE(state.m_currentState == DAttackState::HitFlinch);
    REQUIRE(state.m_activeAttackSequence == InvalidAttackSequenceId);
    REQUIRE(state.m_queuedAttackSequence == InvalidAttackSequenceId);

    // Ticks 1..15 (< the resolved 0.3 s dwell) — flag cleared, attackLeft still held. The machine
    // must dwell in HitFlinch and ignore the attack input the whole time (input gating).
    for (unsigned int tick = 1; tick <= 15; ++tick)
    {
        state = runTick(tick, /*inboundHit*/ false, /*attackLeft*/ true);
        INFO("hitflinch dwell tick " << tick << " (t=" << tick * dt << "s)");
        REQUIRE(state.m_currentState == DAttackState::HitFlinch);
        REQUIRE(state.m_activeAttackSequence == InvalidAttackSequenceId);
    }

    // Past the resolved dwell the machine exits HitFlinch. With attackLeft released it lands in
    // Idle. (~18 ticks ≈ 0.3 s at 60 Hz; run a few past to catch the exit.)
    bool leftFlinch = false;
    for (unsigned int tick = 16; tick <= 40 && !leftFlinch; ++tick)
    {
        state = runTick(tick, /*inboundHit*/ false, /*attackLeft*/ false);
        if (state.m_currentState != DAttackState::HitFlinch)
            leftFlinch = true;
    }
    REQUIRE(leftFlinch);
    REQUIRE(state.m_currentState == DAttackState::Idle);
}

// ===========================================================================
// THE HIT-REACTION LOCKOUT IS THE MACHINE'S  [movement-sim task 27]
//
// USER RULING 2026-09-12, asked whether a brawler can attack while being knocked back:
// "No, and it should also block guarding."
//
// ⭐⭐ WHY THIS LIVES IN THE MACHINE AND NOT BEHIND A MOVEMENT FLAG, and it is the
// architecture that decides it rather than taste: `ExecutionOrder` is machine -> guard -> radial
// -> projectile -> movement LAST, and movement declares `ExternalDeps<const machine::State&>`, so
// the machine CANNOT read movement without `findFirstViolation` rejecting it. A lockout that gates
// attacking AND guarding AND the body has to live where all three can see it. `HitFlinch` already
// is that state: the veto sits ahead of the switch so attack input is refused, and
// `DAttackGuardSimulation` disables every guard shape whenever `m_currentState != Idle`.
//
// ⛔ WHAT TASK 27 CHANGES HERE, and the two things that would otherwise be silently wrong:
//   * the dwell is `State::m_flinchDuration`, resolved PER HIT by `brawlerHitRouting::System`
//     from the attacking sequence's `HitReactionSpec` -- never the old universal 0.3 s, and never
//     a typed constant derived from movement constants the machine cannot see (task 16 turns those
//     into cvar reads, so a typed constant would desynchronise the day one changed);
//   * the `!= HitFlinch` condition is GONE from the hit veto, so every hit RE-ENTERS and REPLACES.
//     Kept, that condition ends the lockout `flinchDuration` after hit ONE while the body is still
//     sliding from hit TWO -- which is the attacking-mid-flight window the lockout exists to close.
// ===========================================================================

namespace integrate3tests
{
// One character, driven a tick at a time, with the routing pass's role played by hand: a test
// writes the WHOLE resolved reaction onto the inbound slice exactly as
// `brawlerHitRouting::System::resolveHitReaction` does, because the machine now copies two of its
// fields into wire `State` and dwells on one of them.
struct FMachineRig
{
    simulatableBrawler::StaticData staticData;
    SimulatableBrawler            character;
    MockPhysicsAdapter            physAdapter;
    MockSpatialQueryAdapter       queryAdapter;

    static constexpr float kDt = 1.f / 60.f;

    FMachineRig() : character(staticData)
    {
        character.setCharacterBindings({ BodyId{1} });
    }

    const dAttackMachineSimulation::State& machine() const
    {
        return character.getAllState().getState().get<dAttackMachineSimulation::State>();
    }

    // The resolver's own arithmetic, spelled from the SHIPPED spec rather than re-typed: a
    // knockback's dwell is `max(lockoutDuration, knockbackSpeed / launchDecel)` -- the slide time
    // is a FLOOR, by the 2026-09-12 ruling, and an authored lockout may exceed it but never
    // undercut it.
    float resolvedDwell(const HitReactionSpec& spec) const
    {
        return spec.kind == HitReactionKind::Knockback
            ? glm::max(spec.lockoutDuration,
                       spec.knockbackSpeed / staticData.m_movementStaticData.launchDecel)
            : spec.lockoutDuration;
    }

    void deliver(const HitReactionSpec& spec, glm::vec2 directionXY)
    {
        const bool knockback = spec.kind == HitReactionKind::Knockback;
        auto& slice = character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>();
        slice.wasHitThisTick = true;
        slice.reactionKind   = spec.kind;
        slice.knockbackSpeed = knockback ? spec.knockbackSpeed : 0.f;
        slice.hitDirectionXY = knockback ? directionXY : glm::vec2(0.f);
        slice.flinchDuration = resolvedDwell(spec);
    }

    // [movement-sim task 84] THE AIM / MOVE / ACTION SEAM. These four defaults are EXACTLY what
    // `tick` hardcoded before this task -- aim east, neutral stick, move-world equal to aim, no
    // matcher action -- so every case written against the old rig is unchanged. Task 84's
    // end-tick cases set them, because the sequence a press selects is
    // `dAttackDirection::classify(aim, moveWorld, stick)` and that is the only way to enter
    // sequences 0 and 1 rather than the neutral-stick forward swing 4.
    glm::vec3     aim{ 1.f, 0.f, 0.f };
    glm::vec3     moveWorld{ 1.f, 0.f, 0.f };
    glm::vec2     moveStick{ 0.f, 0.f };
    std::uint32_t triggeredActionId = 0u;

    // The radial sub-state, read by task 84's dual-tap case to establish -- from observable state
    // rather than from a log line -- that the machine's `attackTimer < 0.1` branch really ran.
    const dAttackRadialSimulation::State& radial() const
    {
        return character.getAllState().getState().get<dAttackRadialSimulation::State>();
    }

    void tick(unsigned int t, bool attackLeft, bool attackRight = false)
    {
        const glm::vec3 aimDir = aim;
        simulatableBrawler::PlayerInput input(
            dAttackRadialSimulation::PlayerInput(aimDir, attackLeft, attackRight),
            dAttackMachineSimulation::PlayerInput{aimDir, attackLeft, attackRight,
                                                 moveStick, moveWorld, triggeredActionId},
            dAttackGuardSimulation::PlayerInput(aimDir),
            brawlerProjectileSimulation::PlayerInput{aimDir},
            brawlerMovementSimulation::PlayerInput{},
            // [ringout task 2, 2026-09-13] Ring-out's ZERO-BYTE PlayerInput, appended to the
            // composite. No field, no wire cost: the input composite is still 77 B and
            // ringWireBytes(1u) is still 86 B. Required only because ValidDependencies makes
            // every sub-sim name an InputType it OWNS.
            brawlerRingout::PlayerInput{});
        character.integrate(SimulationTimeStep(t, false, false, false, kDt),
                            input, physAdapter, queryAdapter, staticData);
        // The routing pass owns the one-shot: it clears the whole slice at the top of every tick.
        character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>() = brawlerInboundHit::DerivedState{};
    }
};
}

// ---------------------------------------------------------------------------
// ⭐⭐ THE LOCKOUT OUTLIVES THE SLIDE. A knockback slides for `knockbackSpeed / launchDecel`
// = 0.5 s; the pre-task-27 dwell was 0.3 s, so the character was free to attack and to guard for
// the last 0.2 s of its own flight. RED on the old dwell, by construction: at tick 24 (0.4 s) the
// old machine has been back in `Idle` for six ticks with `attackLeft` held, so it is swinging.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.Integrate3.LockoutOutlivesSlide", "[DAttack][HitFlinch]")
{
    using namespace integrate3tests;

    FMachineRig rig;
    const HitReactionSpec& knockback =
        rig.staticData.m_hitReactions[dAttackDirection::kRightSequenceId];

    // PREMISE: the shipped right-hand swing really is a knockback, and its dwell really is the
    // slide time rather than the authored floor of zero.
    REQUIRE(knockback.kind == HitReactionKind::Knockback);
    const float slideSeconds =
        knockback.knockbackSpeed / rig.staticData.m_movementStaticData.launchDecel;
    INFO("knockbackSpeed=" << knockback.knockbackSpeed << " launchDecel="
         << rig.staticData.m_movementStaticData.launchDecel << " -> slide " << slideSeconds
         << " s, authored lockout " << knockback.lockoutDuration
         << " s, resolved dwell " << rig.resolvedDwell(knockback) << " s");
    REQUIRE(slideSeconds == Catch::Approx(0.5f).margin(1e-6f));
    REQUIRE(rig.resolvedDwell(knockback) == Catch::Approx(slideSeconds).margin(1e-6f));
    // ...and it is STRICTLY longer than the value the machine used to dwell for.
    REQUIRE(rig.resolvedDwell(knockback) > 0.3f);

    rig.deliver(knockback, glm::vec2(1.f, 0.f));
    rig.tick(0u, /*attackLeft*/ true);
    REQUIRE(rig.machine().m_currentState == DAttackState::HitFlinch);
    REQUIRE(rig.machine().m_hitReaction == HitReactionKind::Knockback);
    REQUIRE(rig.machine().m_flinchDuration == Catch::Approx(slideSeconds).margin(1e-6f));

    // THROUGH THE LAST SLIDING TICK, with the attack button held the whole way. 30 ticks is
    // exactly the slide; the loop stops one short of the boundary so the exit below is the
    // measurement rather than an off-by-one.
    for (unsigned int t = 1u; t <= 29u; ++t)
    {
        rig.tick(t, /*attackLeft*/ true);
        INFO("lockout tick " << t << " (t=" << (t * FMachineRig::kDt) << " s)");
        // 1. THE ATTACK IS REFUSED.
        REQUIRE(rig.machine().m_currentState == DAttackState::HitFlinch);
        REQUIRE(rig.machine().m_activeAttackSequence == InvalidAttackSequenceId);
        // 2. AND SO IS THE GUARD -- this is `DAttackGuardSimulation`'s own shape gate, quoted:
        //    it disables every guard shape whenever the machine is not Idle.
        REQUIRE(rig.machine().m_currentState != DAttackState::Idle);
    }

    // ⛔ THE ROW THAT IS RED ON THE OLD DWELL. At 0.4 s the pre-task-27 machine had been back in
    // Idle since tick 19 and, with attackLeft still held, had started a swing.
    REQUIRE(rig.machine().m_timeInCurrentState > 0.3f);
    REQUIRE(rig.machine().m_timeInCurrentState < rig.machine().m_flinchDuration);

    // ...and it does come out, on the far side, so the lockout is a window and not a trap.
    bool left = false;
    for (unsigned int t = 30u; t <= 60u && !left; ++t)
    {
        rig.tick(t, /*attackLeft*/ false);
        left = rig.machine().m_currentState != DAttackState::HitFlinch;
        if (left)
            INFO("left HitFlinch at tick " << t);
    }
    REQUIRE(left);
    REQUIRE(rig.machine().m_currentState == DAttackState::Idle);
}

// ---------------------------------------------------------------------------
// ⛔ EVERY HIT RE-ENTERS AND REPLACES. RED on the `!= HitFlinch` condition the veto carried
// until task 27: with it, the second hit leaves the timer where the first left it, the lockout
// ends 0.5 s after hit ONE, and the reaction the movement sim reads is still hit one's.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.Integrate3.RehitRestartsLockoutAndReplaces", "[DAttack][HitFlinch]")
{
    using namespace integrate3tests;

    FMachineRig rig;
    const HitReactionSpec& knockback =
        rig.staticData.m_hitReactions[dAttackDirection::kRightSequenceId];
    const HitReactionSpec& stun =
        rig.staticData.m_hitReactions[dAttackDirection::kForwardSequenceId];
    REQUIRE(knockback.kind == HitReactionKind::Knockback);
    REQUIRE(stun.kind == HitReactionKind::Stun);

    rig.deliver(knockback, glm::vec2(1.f, 0.f));
    rig.tick(0u, false);
    for (unsigned int t = 1u; t <= 10u; ++t)
        rig.tick(t, false);

    const float elapsed = rig.machine().m_timeInCurrentState;
    INFO("first lockout has run " << elapsed << " s of " << rig.machine().m_flinchDuration
         << " s when the second hit lands");
    REQUIRE(elapsed > 0.15f);
    REQUIRE(rig.machine().m_currentState == DAttackState::HitFlinch);

    // THE SECOND HIT, a STUN, while still flinching from the knockback.
    rig.deliver(stun, glm::vec2(0.f));
    rig.tick(11u, false);

    // 1. THE TIMER RESTARTED, to EXACTLY zero. `integrate3` advances
    //    `m_timeInCurrentState += deltaTime` at the TOP of the body and the veto zeroes it
    //    afterwards, so a re-entry tick ends at 0 -- the same shape a first entry has, and not
    //    one step. Under the old `!= HitFlinch` guard this reads `elapsed + dt`, which is what
    //    the INFO line names so a break says which side it landed on.
    INFO("after the re-hit: timer=" << rig.machine().m_timeInCurrentState
         << " (the old `!= HitFlinch` guard would have left it at "
         << (elapsed + FMachineRig::kDt) << ")");
    REQUIRE(rig.machine().m_timeInCurrentState == 0.f);
    REQUIRE(rig.machine().m_timeInCurrentState < elapsed);

    // 2. AND THE REACTION WAS REPLACED, kind and dwell together. This is what the movement sim
    //    reads every tick, so a stale kind here is a character still sliding under a stun.
    REQUIRE(rig.machine().m_hitReaction == HitReactionKind::Stun);
    REQUIRE(rig.machine().m_flinchDuration
            == Catch::Approx(stun.lockoutDuration).margin(1e-6f));
    // [movement-sim task 86] ...and it is NOT the knockback's dwell. This line used to
    //    read `m_flinchDuration < knockbackSpeed / launchDecel`, which held only because
    //    the authored stun happened to be 0.3 s and the knockback's resolved dwell is
    //    0.5 s. Task 86 raised the stun to 0.65 s DELIBERATELY -- a stun is now the
    //    LONGER reaction -- so a directional `<` would encode a design assumption this
    //    task inverted. What the case actually needs to prove is that the replacement
    //    installed the STUN's own dwell rather than accidentally reusing the knockback's,
    //    so the assertion is DIFFERENCE, not direction, and it survives either ordering.
    INFO("stun dwell " << rig.machine().m_flinchDuration << " s against the knockback's "
         << rig.resolvedDwell(knockback) << " s");
    REQUIRE(rig.machine().m_flinchDuration
            != Catch::Approx(rig.resolvedDwell(knockback)).margin(1e-3f));

    // 3. ...and a knockback landing on a STUN replaces the other way, discarding the stun's
    //    remaining time -- the fourth row of the cross-kind table in `design_hit_reactions.md`.
    rig.deliver(knockback, glm::vec2(0.f, 1.f));
    rig.tick(12u, false);
    REQUIRE(rig.machine().m_hitReaction == HitReactionKind::Knockback);
    REQUIRE(rig.machine().m_flinchDuration
            == Catch::Approx(rig.resolvedDwell(knockback)).margin(1e-6f));
    REQUIRE(rig.machine().m_timeInCurrentState == 0.f);
}

// ---------------------------------------------------------------------------
// ⭐⭐ A KNOCKBACK LOCKOUT MAY OUTLAST ITS SLIDE; IT MAY NEVER UNDERCUT IT. The authored
// `lockoutDuration` is a FLOOR-CANDIDATE, not the answer: `max(lockoutDuration, slide)`. Authoring
// 0.8 s buys a grounded beat after the body has stopped; authoring 0.1 s buys nothing, because
// ending the lockout at 0.1 s would hand the attack button back to a character still travelling at
// 1600 cm/s. Both arms are driven on the MACHINE, which is what actually consumes the dwell.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.Integrate3.KnockbackLockoutCanOutlastSlideButNeverUndercutIt",
          "[DAttack][HitFlinch]")
{
    using namespace integrate3tests;

    FMachineRig rig;
    const float launchDecel = rig.staticData.m_movementStaticData.launchDecel;
    const float speed =
        rig.staticData.m_hitReactions[dAttackDirection::kRightSequenceId].knockbackSpeed;
    const float slideSeconds = speed / launchDecel;

    // ARM 1 -- AUTHORED LONGER THAN THE SLIDE. The lockout is the authored number.
    {
        const HitReactionSpec grounded{ HitReactionKind::Knockback, speed, 0.8f };
        REQUIRE(rig.resolvedDwell(grounded) == Catch::Approx(0.8f).margin(1e-6f));
        REQUIRE(rig.resolvedDwell(grounded) > slideSeconds);

        rig.deliver(grounded, glm::vec2(1.f, 0.f));
        rig.tick(0u, true);
        REQUIRE(rig.machine().m_flinchDuration == Catch::Approx(0.8f).margin(1e-6f));

        // At 0.6 s the body has been at rest for 0.1 s and the character is STILL locked out --
        // the grounded beat. 36 ticks is 0.6 s.
        for (unsigned int t = 1u; t <= 36u; ++t)
            rig.tick(t, true);
        INFO("grounded beat: timer=" << rig.machine().m_timeInCurrentState
             << " s, slide ended at " << slideSeconds << " s");
        REQUIRE(rig.machine().m_timeInCurrentState > slideSeconds);
        REQUIRE(rig.machine().m_currentState == DAttackState::HitFlinch);
    }

    // ARM 2 -- AUTHORED SHORTER THAN THE SLIDE. The slide wins; the authored number is ignored.
    {
        FMachineRig brief;
        const HitReactionSpec tooShort{ HitReactionKind::Knockback, speed, 0.1f };
        INFO("authored 0.1 s against a " << slideSeconds << " s slide -> resolved "
             << brief.resolvedDwell(tooShort) << " s");
        REQUIRE(brief.resolvedDwell(tooShort) == Catch::Approx(slideSeconds).margin(1e-6f));

        brief.deliver(tooShort, glm::vec2(1.f, 0.f));
        brief.tick(0u, true);
        REQUIRE(brief.machine().m_flinchDuration == Catch::Approx(slideSeconds).margin(1e-6f));

        // At 0.2 s -- twice the authored lockout -- the character is still locked out, because the
        // body is still moving. 12 ticks is 0.2 s.
        for (unsigned int t = 1u; t <= 12u; ++t)
            brief.tick(t, true);
        REQUIRE(brief.machine().m_timeInCurrentState > 0.1f);
        REQUIRE(brief.machine().m_currentState == DAttackState::HitFlinch);
        REQUIRE(brief.machine().m_activeAttackSequence == InvalidAttackSequenceId);
    }

    // ARM 3 -- A STUN IGNORES THE SLIDE TERM ENTIRELY: no knockback speed, so nothing to floor it
    // against, and the authored lockout is the whole answer.
    {
        const HitReactionSpec stun{ HitReactionKind::Stun, 0.f, 0.3f };
        REQUIRE(rig.resolvedDwell(stun) == Catch::Approx(0.3f).margin(1e-6f));
        REQUIRE(rig.resolvedDwell(stun) < slideSeconds);
    }
}

// ===========================================================================
// THE ATTACK END TICK  [movement-sim task 84, 2026-09-20]
//
// SUBJECT: `dAttackMachineSimulation::State::m_attackEndTick` -- the absolute sim tick on which
// the machine will next be `Idle`, written ONLY where `integrate3` produces a radial EDGE.
// These three cases drive the WHOLE `SimulatableBrawler`, because the claim is about agreement
// between the machine's prediction and the RADIAL's own float accumulation: `swingTickCount`
// replicates `attackTimer = attackTimer + dt` from `0.f` against `t < getDuration()` STEP FOR
// STEP, and nothing short of running both can show that it does.
// ===========================================================================

namespace integrate3tests
{

// What one swing looked like, observed from OUTSIDE the machine.
struct SwingTrace
{
    unsigned int  entryTick     = 0u;
    DAttackState  entryState    = DAttackState::Idle;
    unsigned int  entrySeq      = InvalidAttackSequenceId;
    std::uint32_t entryEndTick  = 0u;
    // Set when `m_activeAttackSequence` changes while the machine is STILL `Attacking`, i.e. the
    // `Attacking -> Attacking` chain.
    bool          chained       = false;
    unsigned int  chainTick     = 0u;
    unsigned int  chainSeq      = InvalidAttackSequenceId;
    std::uint32_t chainEndTick  = 0u;
    bool          reachedIdle   = false;
    unsigned int  firstIdleTick = 0u;
};

// Press on `t0`, keep the buttons down for `holdTicks` ticks in total (so `holdTicks == 1` means
// the press tick alone), then release and drive until the machine reads `Idle` again.
inline SwingTrace driveOneSwing(FMachineRig& rig, unsigned int t0, bool L, bool R,
                                unsigned int holdTicks, std::uint32_t action,
                                unsigned int maxTicks = 300u)
{
    SwingTrace tr;
    rig.triggeredActionId = action;
    rig.tick(t0, L, R);
    rig.triggeredActionId = 0u;

    tr.entryTick    = t0;
    tr.entryState   = rig.machine().m_currentState;
    tr.entrySeq     = rig.machine().m_activeAttackSequence;
    tr.entryEndTick = rig.machine().m_attackEndTick;

    unsigned int previousSeq = tr.entrySeq;
    for (unsigned int t = t0 + 1u; t < t0 + maxTicks; ++t)
    {
        const bool hold = (t - t0) < holdTicks;
        rig.tick(t, hold && L, hold && R);

        const dAttackMachineSimulation::State& m = rig.machine();
        if (m.m_currentState == DAttackState::Idle)
        {
            tr.reachedIdle   = true;
            tr.firstIdleTick = t;
            break;
        }
        if (m.m_activeAttackSequence != previousSeq)
        {
            tr.chained      = true;
            tr.chainTick    = t;
            tr.chainSeq     = m.m_activeAttackSequence;
            tr.chainEndTick = m.m_attackEndTick;
            previousSeq     = m.m_activeAttackSequence;
        }
    }
    return tr;
}

} // namespace integrate3tests

// ---------------------------------------------------------------------------
// ⭐⭐ THE EXACTNESS PROOF. For every authored sequence, and for the Hadouken, the tick the
// machine PREDICTED on the entry tick is the tick it actually returns to `Idle` on -- not one
// either side. It holds only because `swingTickCount` performs the radial's own float additions
// in the radial's own order: `ceil(duration / dt)` is NOT this number, because `36 x (1/60)`
// accumulated in float lands on a side of `0.6f` no reader can name and the helper must not
// guess. Sequences 2 and 3 are reachable only through the chain (the classifier never selects
// them from `Idle`), so their entry tick is the CHAIN tick and that is where the prediction is
// read.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.Integrate3.AttackEndTickMatchesTheFirstIdleTick", "[DAttack][AttackSlide]")
{
    using namespace integrate3tests;

    // `classify(aim, moveWorld, stick)`: neutral stick is the forward swing 4; moving 90 deg
    // counter-clockwise of the aim is 1, clockwise is 0. A press with the stick neutral cannot
    // reach 0 or 1, and no press at all can reach 2 or 3.
    struct Route
    {
        const char*   name;
        glm::vec3     moveWorld;
        glm::vec2     moveStick;
        bool          L;
        bool          R;
        unsigned int  holdTicks;      // long enough to queue the chain, where one is wanted
        std::uint32_t action;
        unsigned int  expectSeq;
        bool          expectChain;
        unsigned int  expectChainSeq;
    };

    const Route routes[] = {
        { "seq 4 (forward, neutral stick)",
          glm::vec3(1.f, 0.f, 0.f), glm::vec2(0.f, 0.f),  true, false,  1u, 0u, 4u, false, 0u },
        { "seq 0 (move clockwise of aim)",
          glm::vec3(0.f, -1.f, 0.f), glm::vec2(0.f, -1.f), true, false,  1u, 0u, 0u, false, 0u },
        { "seq 1 (move counter-clockwise of aim)",
          glm::vec3(0.f, 1.f, 0.f), glm::vec2(0.f, 1.f), false,  true,  1u, 0u, 1u, false, 0u },
        { "seq 2 (chained from 0, attackLeft held past 0.3 s)",
          glm::vec3(0.f, -1.f, 0.f), glm::vec2(0.f, -1.f), true, false, 24u, 0u, 0u,  true, 2u },
        { "seq 3 (chained from 1, attackRight held past 0.3 s)",
          glm::vec3(0.f, 1.f, 0.f), glm::vec2(0.f, 1.f), false,  true, 24u, 0u, 1u,  true, 3u },
    };

    for (const Route& route : routes)
    {
        FMachineRig rig;
        rig.moveWorld = route.moveWorld;
        rig.moveStick = route.moveStick;

        const SwingTrace tr = driveOneSwing(rig, 1u, route.L, route.R, route.holdTicks, route.action);

        INFO(route.name << ": entryTick=" << tr.entryTick << " entrySeq=" << tr.entrySeq
             << " entryEndTick=" << tr.entryEndTick << " chained=" << (tr.chained ? 1 : 0)
             << " chainTick=" << tr.chainTick << " chainSeq=" << tr.chainSeq
             << " chainEndTick=" << tr.chainEndTick << " firstIdleTick=" << tr.firstIdleTick);

        // PREMISE: the route really selected the sequence it claims to.
        REQUIRE(tr.entryState == DAttackState::Attacking);
        REQUIRE(tr.entrySeq == route.expectSeq);
        REQUIRE(tr.reachedIdle);

        // 1. THE PREDICTION EXISTS AND IS IN THE FUTURE. `k >= 1` for any positive duration, so
        //    the end tick is never the entry tick and never zero.
        REQUIRE(tr.entryEndTick > tr.entryTick);

        if (route.expectChain)
        {
            REQUIRE(tr.chained);
            REQUIRE(tr.chainSeq == route.expectChainSeq);
            // 2a. THE FIRST SWING'S PREDICTION IS THE CHAIN TICK ITSELF -- the chain fires on the
            //     exact tick the machine would otherwise have gone Idle, which is what makes
            //     "the first tick the machine will be Idle" the right definition of the field.
            REQUIRE(tr.entryEndTick == tr.chainTick);
            // 2b. ...and the rewritten prediction is the real end.
            REQUIRE(tr.chainEndTick > tr.chainTick);
            REQUIRE(tr.firstIdleTick == tr.chainEndTick);
        }
        else
        {
            REQUIRE_FALSE(tr.chained);
            // 2. THE WHOLE POINT: predicted == actual, to the tick.
            REQUIRE(tr.firstIdleTick == tr.entryEndTick);
        }
    }

    // THE HADOUKEN -- no radial at all. The machine gates ITSELF on
    // `m_timeInCurrentState < kHadoukenCommitmentSeconds`, accumulating `+= dt` from 0 with the
    // same `<` predicate, and exits on the FIRST tick the sum reaches the window. So its end tick
    // is `tick + swingTickCount(kHadoukenCommitmentSeconds, dt)` with NO `+ 1`: there is no radial
    // whose `Invalid` the machine sees a tick late.
    {
        FMachineRig rig;
        const SwingTrace tr =
            driveOneSwing(rig, 1u, /*L*/ false, /*R*/ false, /*holdTicks*/ 1u,
                          inputSequence::kHadoukenActionId);

        INFO("hadouken: entrySeq=" << tr.entrySeq << " entryEndTick=" << tr.entryEndTick
             << " firstIdleTick=" << tr.firstIdleTick);
        REQUIRE(tr.entryState == DAttackState::Attacking);
        REQUIRE(tr.entrySeq == kHadoukenSequenceSentinel);
        REQUIRE(tr.reachedIdle);
        REQUIRE(tr.entryEndTick > tr.entryTick);
        REQUIRE(tr.firstIdleTick == tr.entryEndTick);
    }
}

// ---------------------------------------------------------------------------
// ⭐ THE CHAIN REWRITES THE PREDICTION, and it must: the second swing has its own duration, and
// the movement slide re-reads the field every tick. `left -> left` is the authored 0 -> 2 chain.
// The first swing's end tick is consumed as the chain tick; the second's is written on it.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.Integrate3.ChainedSequenceRewritesTheEndTick", "[DAttack][AttackSlide]")
{
    using namespace integrate3tests;

    FMachineRig rig;
    rig.moveWorld = glm::vec3(0.f, -1.f, 0.f);
    rig.moveStick = glm::vec2(0.f, -1.f);

    const SwingTrace tr = driveOneSwing(rig, 1u, /*L*/ true, /*R*/ false, /*holdTicks*/ 24u, 0u);

    INFO("chain: entrySeq=" << tr.entrySeq << " entryEndTick=" << tr.entryEndTick
         << " chainTick=" << tr.chainTick << " chainSeq=" << tr.chainSeq
         << " chainEndTick=" << tr.chainEndTick << " firstIdleTick=" << tr.firstIdleTick);

    REQUIRE(tr.entrySeq == 0u);
    REQUIRE(tr.chained);
    REQUIRE(tr.chainSeq == 2u);

    // 1. IT WAS REWRITTEN -- a chain that left the field alone would leave a slide that had
    //    already stopped, or one that stopped in the middle of the second swing.
    REQUIRE(tr.chainEndTick != tr.entryEndTick);
    REQUIRE(tr.chainEndTick > tr.chainTick);

    // 2. THE TWO SEQUENCES REALLY HAVE DIFFERENT DURATIONS, so row 1 discriminates. Read from the
    //    SHIPPED table rather than restated: 0.6 s and 0.4 s at the time of writing.
    const float duration0 = rig.staticData.m_attackSequences[0].getDuration();
    const float duration2 = rig.staticData.m_attackSequences[2].getDuration();
    INFO("authored durations: seq0=" << duration0 << " s, seq2=" << duration2 << " s");
    REQUIRE(duration0 != duration2);

    // 3. ...and test 7's assertion holds for the SECOND swing.
    REQUIRE(tr.reachedIdle);
    REQUIRE(tr.firstIdleTick == tr.chainEndTick);
}

// ---------------------------------------------------------------------------
// ⛔⛔ THE DUAL-TAP RESTART BRANCH WRITES NO END TICK, AND THIS CASE IS WHY THAT IS DELIBERATE.
// While both attack buttons are down inside the first 0.1 s of a swing, `integrate3` re-enters
// `setRadialSimulationInitialConditions` on EVERY tick. With `m_activeAttackSequence` already 4
// the radial's edge predicate (`currenSequenceId != activeAttackSequence`) is FALSE and the swing
// does NOT restart -- a pre-existing no-op, routed as its own Backlog item and deliberately not
// fixed here. Writing `tick + swingTickCount(...) + 1` on that branch would push the predicted end
// LATER on every one of those held ticks while the radial ended on the original schedule: an
// attack that never ends and a slide that never stops.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.Integrate3.DualTapRestartLeavesTheEndTickAlone", "[DAttack][AttackSlide]")
{
    using namespace integrate3tests;

    FMachineRig rig;                      // neutral stick -> classify selects the forward swing 4

    rig.tick(1u, /*L*/ true, /*R*/ true);
    REQUIRE(rig.machine().m_currentState == DAttackState::Attacking);
    REQUIRE(rig.machine().m_activeAttackSequence == 4u);
    const std::uint32_t endTickAtEntry = rig.machine().m_attackEndTick;
    REQUIRE(endTickAtEntry > 1u);

    // Hold BOTH through the 0.1 s window. The machine runs BEFORE the radial, so the timer the
    // branch tested on tick `t` is the one this loop read after tick `t - 1`.
    unsigned int dualTapTicks = 0u;
    for (unsigned int t = 2u; t <= 7u; ++t)
    {
        const float timerSeenByTheMachine = rig.radial().attackTimer;
        rig.tick(t, /*L*/ true, /*R*/ true);

        // PREMISE, from observable state rather than from the log line: the branch's whole
        // condition held on this tick, so the branch RAN.
        if (timerSeenByTheMachine < 0.1f && rig.machine().m_currentState == DAttackState::Attacking)
        {
            ++dualTapTicks;
            INFO("dual-tap tick " << t << " (timer seen by the machine " << timerSeenByTheMachine
                 << " s): endTick=" << rig.machine().m_attackEndTick);
            // ⛔ UNCHANGED. Every one of these ticks would have pushed it later if the branch
            //    wrote.
            REQUIRE(rig.machine().m_attackEndTick == endTickAtEntry);
            REQUIRE(rig.machine().m_activeAttackSequence == 4u);
        }
    }
    INFO("dual-tap branch ran on " << dualTapTicks << " tick(s)");
    REQUIRE(dualTapTicks >= 5u);          // ANTI-VACUITY: the window really was exercised

    // ...AND THE MACHINE STILL IDLES ON THE TICK IT PREDICTED AT ENTRY. Buttons released from
    // here so the `timer > 0.3` queue cannot form a chain and change the subject.
    bool reachedIdle = false;
    unsigned int firstIdleTick = 0u;
    for (unsigned int t = 8u; t <= 200u; ++t)
    {
        rig.tick(t, false, false);
        if (rig.machine().m_currentState == DAttackState::Idle)
        {
            reachedIdle = true;
            firstIdleTick = t;
            break;
        }
    }
    INFO("endTickAtEntry=" << endTickAtEntry << " firstIdleTick=" << firstIdleTick);
    REQUIRE(reachedIdle);
    REQUIRE(firstIdleTick == endTickAtEntry);
}

#endif // WITH_LOW_LEVEL_TESTS
