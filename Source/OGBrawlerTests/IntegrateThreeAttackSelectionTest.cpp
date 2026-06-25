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
    glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

struct MockSpatialQueryAdapter
{
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }
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
        brawlerProjectileSimulation::PlayerInput{aimDirection});

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
            brawlerProjectileSimulation::PlayerInput{aim});

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
        brawlerProjectileSimulation::PlayerInput{aim});

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

#endif // WITH_LOW_LEVEL_TESTS
