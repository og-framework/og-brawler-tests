// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/SimulationComposite.h"
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
    // Task 3b force seam. No-op: this task adds the capability only; task 12's
    // movement-sim tests are the ones that record these calls.
    void addBodyAcceleration(BodyId, const glm::vec3&) {}
    void addBodyVelocityChange(BodyId, const glm::vec3&) {}
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
    // Task 7 sweep seam. No-op: this task adds the capability only; task 12's
    // movement-sim tests are the ones that script sweeps and record these calls.
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
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
// ⭐⭐ [ringout task 2, 2026-09-13] THE CALL SITE ACTUALLY RUNS.
//
// THIS CASE EXISTS BECAUSE EVERYTHING ELSE THIS TASK ADDED WOULD HAVE PASSED WITHOUT IT.
// Wiring `brawlerRingout` into the composite is four type-level edits — State, DerivedState,
// PlayerInput, ExecutionOrder — and every one of them compiles, serializes, fences and
// reports green whether or not `SimulatableBrawler::integrate` ever calls
// `brawlerRingout::integrate`. The wire footprint would still read 335 B. The budget fences
// would still re-price. The whole 474-case suite would still pass. And the sub-simulation
// would never run once, in PIE or anywhere else, until somebody noticed nobody ever died.
// An inert sub-simulation that looks green is worse than a missing one, so the call gets a
// test of its own rather than a comment.
//
// The proof is deliberately the SHORTEST one that can only pass if the call is present: seed
// the movement slice's solved position below the authored kill plane, integrate ONE tick
// through the real `SimulatableBrawler::integrate`, and read the dead bit back out of the
// composite. Nothing else in this function writes `brawlerRingout::State`.
//
// ⚠ It is NOT a test of the ring-out LAW — that lives in BrawlerRingoutSimulationTest.cpp's
// eight cases against the sub-sim directly, and duplicating them here would mean two places
// to update when the law changes. This case pins the WIRING, and the decoy arm below is what
// keeps it from passing on a default-constructed state that happens to look dead.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulatableBrawler.RingoutIntegrateIsReachedFromTheComposite",
          "[DAttack][SimulatableBrawler][BrawlerRingout]")
{
    FMockPhysicsBodyAdapter physAdapter;
    FMockSpatialQueryAdapter queryAdapter;
    const simulatableBrawler::StaticData staticData;
    const simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();

    // Authored, not assumed: the arms below are positioned relative to the SHIPPED kill
    // plane, so retuning it in brawlerRingout::StaticData moves this test with it instead of
    // silently making one arm vacuous.
    const float killPlaneZ = staticData.m_ringoutStaticData.killPlaneZ;

    auto integrateOneTickAtZ = [&](float z)
    {
        SimulatableBrawler character = makeTestCharacter();

        // Ring-out reads the position the movement sub-sim LEFT BEHIND, so seeding the
        // movement State slice is exactly the input the law consumes. `teleportPending` is 0
        // on a default-constructed InitialConditions, which the death arm also requires.
        character.editAllState().editState()
            .edit<brawlerMovementSimulation::State>().bodyState.position.z = z;

        const SimulationTimeStep step(0u, false, false, false, 1.f / 60.f);
        character.integrate(step, zeroInput, physAdapter, queryAdapter, staticData);
        return character.getAllState().getState().get<brawlerRingout::State>();
    };

    SECTION("below the kill plane, the composite's integrate kills the character")
    {
        const brawlerRingout::State dead = integrateOneTickAtZ(killPlaneZ - 100.f);

        // THE WHOLE POINT: this bit is only ever written by brawlerRingout::integrate, and
        // the only thing that can call it on this object is the block in
        // SimulatableBrawler::integrate. If that block is deleted, reordered out of the
        // function, or guarded off, this assertion is what fails.
        REQUIRE(brawlerRingout::isDead(dead));
        REQUIRE(dead.respawnAtTick
                == 0u + staticData.m_ringoutStaticData.respawnDelayTicks);
    }

    SECTION("above the kill plane it does not — the decoy that keeps the arm above honest")
    {
        // Without this arm, an implementation that set the dead bit unconditionally — or a
        // default-constructed State that already had it — would satisfy the section above.
        const brawlerRingout::State alive = integrateOneTickAtZ(killPlaneZ + 100.f);
        REQUIRE_FALSE(brawlerRingout::isDead(alive));
    }
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

// ---------------------------------------------------------------------------
// ⭐ THE WIRE FOOTPRINT of the simulatableBrawler::State composite.
//
// WHY THIS IS AN *ABSOLUTE* ASSERTION AND NOT A DELTA. The composite is written
// into FSimulationStateSyncBuffer, whose capacity is a FIXED
// `kBufferBytes = 384` (UE-sim/SyncedSimulationStateBuffer.h:213). Overflowing it
// is **a runtime out-of-bounds check, not a compile error** — the buffer's
// writeToBuffer logs "FSimulationStateSyncBuffer write OOB: ... (raise
// kBufferBytes)" (same file, :243) and the correction is dropped. A delta-only
// assertion cannot see that boundary: every individual step can be small and
// legal while the running total walks off the end. So this case pins the
// absolute number and, separately, the headroom against the capacity.
//
// [movement-sim task 5, 2026-09-02] ESTABLISHED here. Task 1 stood the movement
// slice up carrying a placeholder 52 B PhysicsBodyState and deliberately asserted
// NOTHING, because that shape was known to be temporary; it only recorded the
// figure. Task 5's LinearBodyState swap makes the shape final, so it is pinned.
// Supporting evidence, not the assertion: task 1 recorded the composite at 352 B,
// task 5 measured 324 B, a **-28 B** delta — exactly the 52 -> 24 B narrowing of
// the one movement slice, with every other slice untouched.
//
// [movement-sim task 29, 2026-09-04] 324 -> 268 B, a **-56 B** delta, and it is the
// WHOLE of the guard sub-simulation's wire presence: 4 B `attackTimer` (a phantom —
// never written by the sim, kept alive only by a test that used it as a mutation
// probe) plus 52 B `bodyState` (derivable every tick from the capsule pose and the
// aim input, so it carried no information). dAttackGuardSimulation::State now
// serializes NOTHING, joining its already-empty InitialConditions; the slice stays
// in the composite at 0 B. Every other slice is untouched — asserted directly two
// assertions below, so a break says which one moved.
// ---------------------------------------------------------------------------

// Serialized size of a SimulationComposite. The synced buffer writes a composite
// as a flat, byte-packed concatenation of its slices — no alignment padding — so
// the wire size is exactly the sum of the per-slice syncSize<>. This mirrors
// relayedInputRing::detail::CompositeSerializedSize; restated here so this file
// needs no input-codec include to price a STATE.
template <typename T> struct FCompositeWireSize;
template <typename... Ts> struct FCompositeWireSize<SimulationComposite<Ts...>>
{
    static constexpr std::uint32_t value = compositeSyncSize<Ts...>();
};

// FSimulationStateSyncBuffer::kBufferBytes, hand-mirrored.
//
// ⚠ It cannot be included: that constant lives in UE code and this target is
// engine-free BY CONFIGURATION (OGBrawlerTests.Target.cs sets
// bCompileAgainstEngine / bCompileAgainstCoreUObject / bCompileAgainstApplicationCore
// all false), which is the property that lets the whole brawler suite run without
// the editor. A copy is therefore the only way to fence the capacity from here —
// and a copy that is checked every run beats a capacity nobody checks at all.
// If UE-sim/SyncedSimulationStateBuffer.h:213 changes, change this line.
static constexpr std::uint32_t kStateSyncBufferBytes = 384u;

TEST_CASE("DAttack.SimulatableBrawler.WireFootprint", "[DAttack][SimulatableBrawler]")
{
    constexpr std::uint32_t kComposite =
        FCompositeWireSize<simulatableBrawler::State>::value;

    // What the buffer actually holds: the correction codec's
    // [tick u32][appliedCaptureTick u32] header, then the whole composite.
    constexpr std::uint32_t kBufferUsed =
        correctionStateBuffer::kHeaderBytes + kComposite;

    INFO("composite=" << kComposite << " B, bufferUsed=" << kBufferUsed
         << " B, capacity=" << kStateSyncBufferBytes
         << " B, headroom=" << (kStateSyncBufferBytes - kBufferUsed) << " B");

    // 1. THE ABSOLUTE SIZE. A diff that moves this is a wire change, and a wire
    //    change is a deliberate, versioned act (correctionStateBuffer::kWireFormatVersion).
    //    [ringout task 2, 2026-09-13] 326 -> 335 B, +9 B, and the slice that moved is
    //    RING-OUT's — a NEW sub-simulation appended to the composite, not a neighbour that
    //    grew. `brawlerRingout::InitialConditions` (4 B: `spawnSlot`) and
    //    `brawlerRingout::State` (5 B: `flags` + `respawnAtTick`) both ride the wire so a
    //    death and its respawn REPLAY IDENTICALLY under resim. The two ring-out slices are
    //    pinned individually three assertions below, so a future break says which of them
    //    moved. `brawlerRingout::DerivedState` (`diedThisTick`) is deliberately OFF the wire
    //    and costs nothing here — see the derived-composite section further down.
    //
    //    ⭐ `correctionStateBuffer::kWireFormatVersion` IS BUMPED 2 -> 3 FOR THIS CHANGE, AND
    //    THE APPEND ARGUMENT IS WHY THAT IS NOT OBVIOUS. Both slices are APPENDED at the end
    //    of the composite declaration in SimulatableBrawlerTypes.h, so every pre-existing
    //    field keeps the byte offset it already had — the exact condition movement-sim tasks
    //    11, 27 and 50 each cited when they DECLINED the bump. ⛔ That precedent does not
    //    carry here, because those tasks grew sub-simulations BOTH builds compiled in, and
    //    this one appends a sub-simulation an older archived build does not have AT ALL. The
    //    layout is silent about that: the reverse pairing (new client, old server) walks 335
    //    bytes out of a 326-byte payload and restores the dead bit and the respawn tick from
    //    whatever the fixed-capacity buffer last held. Mixed archives are ordinary practice
    //    in this project (three independently-cooked targets; see PLAYTEST_PORTABLE_README.md),
    //    so the version byte is the only thing between that and a loud refusal. The full
    //    reasoning lives at the constant in CorrectionStateBufferCodec.h; the cost side — a
    //    stale peer now gets a build-mismatch error instead of a silent ring-out-shaped hole —
    //    is written out in impl_notes_ringout_2.md §5/§5a.
    //
    //    [movement-sim task 84, 2026-09-20] 335 -> 339 B, +4 B, and the slice that moved is the
    //    MACHINE's -- an EXISTING slice growing by one field, not a new sub-simulation. The field
    //    is `dAttackMachineSimulation::State::m_attackEndTick`: the absolute tick on which the
    //    machine will next be `Idle`, written only where the machine produces a radial edge and
    //    read by the movement sub-simulation's attack-slide branch. It rides the wire because a
    //    remote proxy that enters a swing BY ADOPTION never simulated the edge and could not have
    //    computed the end -- the same argument ring-out records for `respawnAtTick` above.
    //    ⛔ `correctionStateBuffer::kWireFormatVersion` is NOT bumped: this is the append that
    //    grows an EXISTING slice, which the message below names as the one case that does not need
    //    it (task 27's precedent, and expressly NOT ring-out's, which added a whole sub-sim).
    //    The machine slice is re-quoted at 25 B in section 2 below.
    //
    //    [og-netcode-v2-field-defects task 9, 2026-09-23] 339 -> 338 B, -1 B, and the slice that
    //    moved is the RADIAL's: `dAttackRadialSimulation::State::hasHitGuard` LEFT THE WIRE. Hit
    //    detection moved out of the attacker's integrate into `brawlerHitDetection::System`, and the
    //    guard block it used to write into the radial State is now the radial DerivedState's per-tick
    //    `guardBlockedThisTick`, routed onto the inbound slice -- derived, recomputed on every replayed
    //    tick, never restored. ⛔ `correctionStateBuffer::kWireFormatVersion` IS BUMPED 3 -> 4, and
    //    this is the plain case the message below exists for: the field was the THIRD of four in the
    //    SECOND composite slice, so `bodyState` and every later slice moved one byte. A peer on 3
    //    reading a 4 payload decodes all of them one byte out of place. The radial slice is pinned
    //    in section 2 below so the next change here says which slice moved.
    //
    //    [og-netcode-v2-field-defects task 17, 2026-09-24] 338 -> 326 B, -12 B, and the slice that
    //    moved is the PROJECTILE's: `brawlerProjectileSimulation::ProjectileSlot::hitRootBodyId`
    //    (a 4 B BodyId) LEFT THE WIRE, once per pool slot, and the pool's SIM_VECTOR is priced at
    //    its capacity of 3 (115 -> 103 B). Projectile detection moved out of the shooter's
    //    integrate into `brawlerHitDetection::System`; the struck character now travels in the
    //    projectile DerivedState's per-pass `detectedThisTick`, read by routing in the same pass.
    //    `endReason` stays: the shooter's integrate still ends the slot with it, and it is what the
    //    correction gate compares when two peers disagree about a shot's outcome.
    //    ⛔ `correctionStateBuffer::kWireFormatVersion` IS BUMPED 4 -> 5: the field was the LAST of
    //    each slot, but the slots are an array inside the composite's third slice, so slot 1, slot
    //    2 and every later slice moved. Measured: this static_assert compiled at 326 on the tree
    //    after the removal (it fired at 338). The projectile slice is pinned in section 2 below.
    static_assert(FCompositeWireSize<simulatableBrawler::State>::value == 326u,
        "The simulatableBrawler::State wire footprint moved. That is a WIRE FORMAT "
        "CHANGE: re-measure it, re-price RoundVsPacketBudgetTest.cpp, and bump "
        "correctionStateBuffer::kWireFormatVersion if the layout moved OR if a whole "
        "sub-simulation entered or left the composite - an append that only grows an "
        "EXISTING slice is the one case that does not need the bump.");
    REQUIRE(kComposite == 326u);

    // 2. WHICH SLICE, so a break says what moved rather than only that something did.
    //
    // [movement-sim task 50, 2026-09-06] The movement State slice 49 -> 61 B, composite
    // 309 -> 321 B, +12 B. `brawlerMovementSimulation::State::positionCmd` moved from
    // off-wire scratch ONTO the wire (ruling #27 A): step 6' subtracts it from the
    // post-solve capture to recover the solver's push-out, and a correction restores by
    // WHOLE-STRUCT assignment over a DEFAULT-CONSTRUCTED state, so an off-wire operand was
    // zeroed on every adopted correction and step 6' silently skipped the first replayed
    // tick. Two other members left the type in the same edit and cost nothing here: the
    // second, off-wire velocity copy was exactly `velocity` (already serialized), and the
    // off-wire command marker was a twin of `flags`' bit 3.
    REQUIRE(syncSize<brawlerMovementSimulation::State>() == 61u);
    REQUIRE(syncSize<brawlerMovementSimulation::InitialConditions>() == 16u);

    // [movement-sim task 27, 2026-09-12] THE MACHINE SLICE 16 -> 21 B, composite 321 -> 326 B,
    // +5 B, and the INPUT wire DOES NOT MOVE (ZeroInputIsTheFold below re-quotes 77 B unchanged;
    // RoundVsPacketBudgetTest.cpp re-quotes ringWireBytes(1u) == 86u and the 82 B entry stride).
    // `m_hitReaction` (1 B) and `m_flinchDuration` (4 B) are APPENDED to
    // dAttackMachineSimulation::State: the movement sub-sim needs the reaction KIND on every tick
    // of a flinch (it decides freeze-vs-slide) and the machine needs the DWELL on every tick (the
    // dwell is per attack now, resolved by brawlerHitRouting from the attack's HitReactionSpec),
    // so both must survive a correction. An append leaves every preceding field's offset intact,
    // which is why correctionStateBuffer::kWireFormatVersion is NOT bumped here — the same
    // reasoning tasks 11 and 50 recorded for the movement slice.
    // ⚠ [ringout task 2, rework, 2026-09-13] STILL TRUE FOR TASK 27, BUT DO NOT GENERALISE IT.
    // Ring-out DID bump the version (2 -> 3) on an append, because it appended a whole
    // SUB-SIMULATION rather than growing an existing slice — see the block above and the rule at
    // the constant in CorrectionStateBufferCodec.h. Task 27 grew a sub-sim both builds compile in,
    // which is the case that correctly declines.
    // ⭐ [movement-sim task 84, 2026-09-20] THE MACHINE SLICE 21 -> 25 B, composite 335 -> 339 B,
    // +4 B, and the INPUT wire DOES NOT MOVE (ZeroInputIsTheFold below re-quotes 77 B unchanged;
    // RoundVsPacketBudgetTest.cpp re-quotes ringWireBytes(1u) == 86u and the 82 B entry stride).
    // `m_attackEndTick` (4 B) is APPENDED to dAttackMachineSimulation::State: the movement sub-sim
    // needs the tick the swing ENDS on every tick of the slide, and a proxy that adopts a
    // mid-swing correction never simulated the edge that produced it. Same append argument as
    // task 27 immediately above, and the same declined version bump for the same reason.
    REQUIRE(syncSize<dAttackMachineSimulation::State>() == 25u);

    // [og-netcode-v2-field-defects task 9, 2026-09-23] THE RADIAL SLICE, first pinned here, one
    // byte smaller than it was: `hasHitGuard` left it (see section 1). 61 -> 60 B, measured on the
    // tree after the removal (a probe pin's failure expansion read `60 == 1`), not derived.
    REQUIRE(syncSize<dAttackRadialSimulation::State>() == 60u);

    // [og-netcode-v2-field-defects task 17, 2026-09-24] THE PROJECTILE SLICE, first pinned here:
    // 4 (SIM_VECTOR count) + 3 slots x 33 B = 103 B, down from 115 B (3 x 37) when each slot
    // still carried hitRootBodyId. The per-slot footprint is pinned beside the type, in
    // BrawlerProjectileSimulationTest.cpp "BrawlerProjectile.WireFootprint".
    REQUIRE(syncSize<brawlerProjectileSimulation::State>() == 103u);

    // [movement-sim task 29] The guard slice, both halves, at zero. This is the term
    // the -56 B came out of, and pinning it HERE — beside the total — is what makes a
    // future re-serialization of the guard read as "the guard grew" rather than as an
    // unattributed 56 B on the composite.
    REQUIRE(syncSize<dAttackGuardSimulation::State>() == 0u);
    REQUIRE(syncSize<dAttackGuardSimulation::InitialConditions>() == 0u);

    // [ringout task 2, 2026-09-13] THE RING-OUT SLICES, 4 + 5 = the whole +9 B. Pinned here
    // BESIDE the total, which is the point of this section: these two numbers and the 335
    // above are the same arithmetic decomposed, so a future diff that moves the composite
    // says whether ring-out grew or a neighbour did. The identical pair is asserted a second
    // time at the DECLARATION in BrawlerRingoutSimulation.h — deliberately, because that is
    // where somebody adding a field is looking, and this file is where somebody investigating
    // a packet is looking.
    REQUIRE(syncSize<brawlerRingout::InitialConditions>() == 4u);
    REQUIRE(syncSize<brawlerRingout::State>() == 5u);

    // [ringout task 2] AND THE OFF-WIRE HALF, PINNED AS A CONCEPT AND NOT AS A ZERO BYTE
    // COUNT. `diedThisTick` is the death EDGE the score system reads (task 4) and it must
    // never become serializable: an edge that can arrive on a correction is an edge that can
    // be awarded twice.
    // ⛔ `syncSize<brawlerRingout::DerivedState>() == 0u` IS THE WRONG SPELLING AND WAS TRIED
    // FIRST — it does not compile at all ("use of undefined type
    // SerializableFields<brawlerRingout::DerivedState>"), because `syncSize` sums a
    // specialization that an off-wire type does not have. A zero there would have been
    // indistinguishable from the guard slices' real, declared 0 B anyway. `Serializable`
    // reads FALSE for "has no specialization" and TRUE for "declared empty", which is the
    // distinction that matters here.
    STATIC_REQUIRE_FALSE(Serializable<brawlerRingout::DerivedState>);
    STATIC_REQUIRE(Serializable<brawlerRingout::State>);
    STATIC_REQUIRE(Serializable<brawlerRingout::InitialConditions>);

    // 3. THE HEADROOM FENCE — the one that would have caught a silent runtime OOB.
    //    Also a compile-time fence, which is the whole point: it converts a failure
    //    mode that is otherwise a dropped correction plus a log line at runtime into
    //    a build break for whoever grows the composite.
    //
    //    [ringout task 2, 2026-09-13] MEASURED AFTER THIS TASK: bufferUsed = 8 + 335 = 343 of
    //    384, so the headroom is **41 B**, down from 50. This fence is an INEQUALITY and so
    //    it does not need re-quoting when the composite grows — which is exactly why the
    //    number is written here in prose: the assertion cannot tell you how close it is, and
    //    41 B is four more slices the size of ring-out's. The WARN at the end of this case
    //    now prints the live headroom on every run for the same reason. Raising
    //    `kBufferBytes` is wire-cheap (NetSerialize watermark-trims to usedBytes), so the
    //    next task to run out should raise it rather than economise on state.
    //
    //    [movement-sim task 84, 2026-09-20] RE-MEASURED AFTER THIS TASK: bufferUsed = 8 + 339 =
    //    347 of 384, so the headroom is **37 B**, down from 41. Four bytes, one field, on an
    //    existing slice. The paragraph above still holds and is not re-argued here.
    //
    //    [og-netcode-v2-field-defects task 9, 2026-09-23] RE-MEASURED: bufferUsed = 8 + 338 = 346 of
    //    384, headroom **38 B**, up from 37 -- the radial's `hasHitGuard` left the wire.
    static_assert(correctionStateBuffer::kHeaderBytes
                      + FCompositeWireSize<simulatableBrawler::State>::value
                  <= kStateSyncBufferBytes,
        "The State composite no longer fits FSimulationStateSyncBuffer::kBufferBytes. "
        "Raise kBufferBytes (UE-sim/SyncedSimulationStateBuffer.h) to the next 64-byte "
        "multiple above the new footprint, and update the copy of it in this file. "
        "Raising the capacity is wire-cheap: NetSerialize watermark-trims to usedBytes.");
    REQUIRE(kBufferUsed <= kStateSyncBufferBytes);

    WARN("simulatableBrawler::State wire footprint: composite=" << kComposite
         << " B (268 B before movement-sim task 11 grew the movement slice to 309;"
         << " +12 B more at task 50 for positionCmd; +5 B more at task 27 for the machine's"
         << " m_hitReaction and m_flinchDuration; +9 B more at ringout task 2 for the"
         << " brawlerRingout InitialConditions and State slices; +4 B more at movement-sim"
         << " task 84 for the machine's m_attackEndTick; -1 B at og-netcode-v2-field-defects"
         << " task 9 for the radial's hasHitGuard), buffer used="
         << kBufferUsed << "/" << kStateSyncBufferBytes << " B, headroom="
         << (kStateSyncBufferBytes - kBufferUsed) << " B");
}


// ===========================================================================
// THE ZERO INPUT IS A FOLD  [movement-sim task 22]
//
// simulatableBrawler::getZeroPlayerInput() no longer hand-builds one argument per
// sub-input. Each sub-simulation's PlayerInput owns a `static PlayerInput zero()`
// beside the type itself, SimulationComposite::zero() folds them, and the function
// is one line. Two properties are pinned here, and they are the two that could
// break silently:
//
//  1. BYTE IDENTITY. kZeroInputWireBefore below is the serialized zero input as it
//     stood BEFORE any production line of task 22 was written. PROVENANCE: on the
//     shipped tree (tasks 1/2/4/5/10a/10 applied, nothing of task 22), a temporary
//     Catch2 case serialized getZeroPlayerInput() through
//     writeCompositeInputToSyncedBuffer into a poison-filled 76-byte buffer and
//     WARN'd the hex. Those 76 bytes are transcribed below; the probe case was then
//     deleted and this permanent case took its place. No production file was edited
//     between the capture and the transcription.
//
//  2. ANTI-VACUITY - the tag survived. getZeroPlayerInput() must stay a DIFFERENT
//     VALUE from simulatableBrawler::PlayerInput{}: radial/machine/guard carry a
//     (0,0,1) forward aim, a value-initialised input carries (0,0,0). That gap is
//     both a normalize() guard and the tag SimulationInputResolutionTest and
//     SimulationNetSyncTest discriminate on - make default construction the zero and
//     every one of those anti-vacuity pairs keeps passing while testing nothing.
//     DO NOT change a default member initialiser to close this gap.
// ===========================================================================

namespace
{
    // Byte-addressable stand-in for the input sync buffer - the same shape
    // writeCompositeInputToSyncedBuffer reaches for, and the same trick the wire
    // tests in both suites already use.
    struct FZeroInputProbeBuffer
    {
        std::vector<std::uint8_t> bytes;

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        { T v; std::memcpy(&v, bytes.data() + off, sizeof(T)); return v; }
    };

    constexpr std::uint32_t kZeroInputWireBytes = 77u;

    // The captured "before" bytes, grouped by composite slice in wire order:
    // radial(14) -> machine(38) -> guard(12) -> projectile(12) -> movement(1).
    //
    // [movement-sim task 11, 2026-09-06] 76 -> 77 B. The movement sub-sim's PlayerInput
    // gained `holdGuard` as a `bool` member, so the slice that used to serialize nothing now
    // contributes exactly one byte. The first 76 bytes are UNCHANGED and still carry task
    // 22's original provenance; the appended 0x00 is not a re-capture, it is
    // `holdGuard == false`, which is that type's own zero() and is derivable by inspection.
    // Everything the block comment above says about the capture remains true of the prefix
    // it describes.
    //
    // [movement-sim task 51, 2026-09-06] UNCHANGED, and the fact that it is unchanged is the
    // hazard. That `bool` member became `uint8_t flags`, with holdGuard as bit 0: the same
    // 1 B, and the same 0x00 when neutral, so THIS ARRAY DID NOT MOVE and neither did
    // `kZeroInputWireBytes`. A byte-comparison fixture that passes because both sides changed
    // together proves nothing, so section 5 of the case below flips bit 0 and requires the
    // wire to move at exactly index 76 -- that, not this comment, is what keeps the trailing
    // byte a holdGuard byte rather than merely a zero byte.
    //
    // The three 0x0000803F runs are the (0,0,1) forward aims; the projectile aim is
    // (0,0,0), which is what the pre-fold call site passed it.
    constexpr std::uint8_t kZeroInputWireBefore[kZeroInputWireBytes] = {
        // radial: aimDirection (0,0,1), attackLeft, attackRight
        0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x80u,0x3Fu, 0x00u,0x00u,
        // machine: aimDirection (0,0,1), attackLeft, attackRight,
        //          moveDirection (0,0), moveDirectionWorld (0,0,0), triggeredActionId 0
        0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x80u,0x3Fu, 0x00u,0x00u,
        0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u,
        0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u,
        0x00u,0x00u,0x00u,0x00u,
        // guard: aimDirection (0,0,1)
        0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x80u,0x3Fu,
        // projectile: aimDirection (0,0,0)
        0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u,
        // movement: flags 0x00 -- ALL BITS CLEAR, and bit 0 is holdGuard (task 51)
        0x00u,
    };

    std::vector<std::uint8_t> serializeInput(const simulatableBrawler::PlayerInput& input)
    {
        FZeroInputProbeBuffer buf;
        // Poison, not zero: a slice that is never written would otherwise read back
        // as a legitimate all-zero value and the comparison would pass on a hole.
        buf.bytes.assign(kZeroInputWireBytes, 0xCDu);
        const std::uint32_t written = writeCompositeInputToSyncedBuffer(input, buf, 0u);
        REQUIRE(written == kZeroInputWireBytes);
        return buf.bytes;
    }

    // Index of the first differing byte, or -1. Reported instead of 76 separate
    // REQUIREs so a break names the offset without inflating the assertion count.
    int firstDifference(const std::vector<std::uint8_t>& actual, const std::uint8_t* expected)
    {
        for (std::uint32_t i = 0; i < kZeroInputWireBytes; ++i)
            if (actual[i] != expected[i]) return static_cast<int>(i);
        return -1;
    }

    // FIELD-EXHAUSTIVE equality over the whole input composite - every field of every
    // sub-input. Mirrors the helper in SimulationInputResolutionTest.cpp (where the
    // anti-vacuity pairing lives) deliberately, including its reason for not memcmp'ing:
    // padding bytes are not part of the value.
    bool sameInput(const simulatableBrawler::PlayerInput& a,
                   const simulatableBrawler::PlayerInput& b)
    {
        const auto& ra = a.get<dAttackRadialSimulation::PlayerInput>();
        const auto& rb = b.get<dAttackRadialSimulation::PlayerInput>();
        const auto& ma = a.get<dAttackMachineSimulation::PlayerInput>();
        const auto& mb = b.get<dAttackMachineSimulation::PlayerInput>();
        const auto& ga = a.get<dAttackGuardSimulation::PlayerInput>();
        const auto& gb = b.get<dAttackGuardSimulation::PlayerInput>();
        const auto& pa = a.get<brawlerProjectileSimulation::PlayerInput>();
        const auto& pb = b.get<brawlerProjectileSimulation::PlayerInput>();
        // [movement-sim task 51] The movement slice was missing from this "every field of
        // every sub-input" list since task 11 added it. Adding it does not change the
        // REQUIRE_FALSE below -- movement's zero() IS its default, so this term is equal on
        // both sides -- but it makes the comment above true, and it means a future bit that
        // stopped folding correctly would show up here rather than only on the wire.
        const auto& va = a.get<brawlerMovementSimulation::PlayerInput>();
        const auto& vb = b.get<brawlerMovementSimulation::PlayerInput>();

        return ra.aimDirection == rb.aimDirection
            && ra.attackLeft == rb.attackLeft
            && ra.attackRight == rb.attackRight
            && ma.aimDirection == mb.aimDirection
            && ma.attackLeft == mb.attackLeft
            && ma.attackRight == mb.attackRight
            && ma.moveDirection == mb.moveDirection
            && ma.moveDirectionWorld == mb.moveDirectionWorld
            && ma.triggeredActionId == mb.triggeredActionId
            && ga.aimDirection == gb.aimDirection
            && pa.aimDirection == pb.aimDirection
            && va.flags == vb.flags;
    }
}

TEST_CASE("DAttack.SimulatableBrawler.ZeroInputIsTheFold", "[DAttack][SimulatableBrawler]")
{
    // The array above is sized for exactly this footprint; a composite change that
    // moved it would otherwise compare against a stale length.
    static_assert(compositeSyncSize<dAttackRadialSimulation::PlayerInput,
                                    dAttackMachineSimulation::PlayerInput,
                                    dAttackGuardSimulation::PlayerInput,
                                    brawlerProjectileSimulation::PlayerInput,
                                    brawlerMovementSimulation::PlayerInput>()
                  == kZeroInputWireBytes,
        "The PlayerInput composite's wire footprint moved. Re-capture the zero input's "
        "bytes before touching kZeroInputWireBefore -- and treat it as a WIRE CHANGE.");

    const auto zero = simulatableBrawler::getZeroPlayerInput();

    // 1. BYTE IDENTITY - the fold reproduces the pre-task value exactly.
    const std::vector<std::uint8_t> after = serializeInput(zero);
    const int diffAt = firstDifference(after, kZeroInputWireBefore);
    INFO("first differing byte index (-1 = identical): " << diffAt);
    REQUIRE(diffAt == -1);

    // 2. THE FOLD IS ELEMENT-WISE. Each slice is that type's OWN zero(), so the
    //    composite carries no second definition of the neutral value.
    REQUIRE(zero.get<dAttackRadialSimulation::PlayerInput>().aimDirection
            == dAttackRadialSimulation::PlayerInput::zero().aimDirection);
    REQUIRE(zero.get<dAttackMachineSimulation::PlayerInput>().aimDirection
            == dAttackMachineSimulation::PlayerInput::zero().aimDirection);
    REQUIRE(zero.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
            == dAttackMachineSimulation::PlayerInput::zero().triggeredActionId);
    REQUIRE(zero.get<dAttackGuardSimulation::PlayerInput>().aimDirection
            == dAttackGuardSimulation::PlayerInput::zero().aimDirection);
    REQUIRE(zero.get<brawlerProjectileSimulation::PlayerInput>().aimDirection
            == brawlerProjectileSimulation::PlayerInput::zero().aimDirection);
    // [movement-sim task 51] The movement slice folds the same way; its neutral value is
    // `flags == 0`, i.e. every bit -- holdGuard today, wall-grab/jump/dash/ski-tuck later --
    // clear. That is why the fold survives each future bit without a re-capture here.
    REQUIRE(zero.get<brawlerMovementSimulation::PlayerInput>().flags
            == brawlerMovementSimulation::PlayerInput::zero().flags);

    // 3. ANTI-VACUITY - THE TAG SURVIVED. This is the assertion that would fail if
    //    anyone "simplified" the design by making default construction the zero.
    REQUIRE_FALSE(sameInput(simulatableBrawler::getZeroPlayerInput(),
                            simulatableBrawler::PlayerInput{}));

    // ...and WHICH slices carry the tag, so a break says what collapsed rather than
    // only that something did.
    REQUIRE(zero.get<dAttackRadialSimulation::PlayerInput>().aimDirection
            == glm::vec3(0.f, 0.f, 1.f));
    REQUIRE(zero.get<dAttackMachineSimulation::PlayerInput>().aimDirection
            == glm::vec3(0.f, 0.f, 1.f));
    REQUIRE(zero.get<dAttackGuardSimulation::PlayerInput>().aimDirection
            == glm::vec3(0.f, 0.f, 1.f));
    REQUIRE(simulatableBrawler::PlayerInput{}
                .get<dAttackRadialSimulation::PlayerInput>().aimDirection
            == glm::vec3(0.f, 0.f, 0.f));

    // 4. And the difference is visible ON THE WIRE too, not only field-wise - a
    //    value-initialised input does NOT serialize to the captured bytes.
    REQUIRE(firstDifference(serializeInput(simulatableBrawler::PlayerInput{}),
                            kZeroInputWireBefore) != -1);

    // 5. * [movement-sim task 51] THE TRAILING BYTE STILL DISCRIMINATES, and this is the
    //    assertion the re-layout owes. `brawlerMovementSimulation::PlayerInput` went from
    //    a `bool` member to `uint8_t flags`, with holdGuard as bit 0. Both are 1 B and both
    //    serialize 0x00 when neutral, so sections 1-4 would ALL have passed unchanged even if
    //    the byte had quietly stopped carrying holdGuard -- their discrimination comes from
    //    the (0,0,1) aim tags in the PREFIX, not from anything in the movement slice.
    //
    //    So: set bit 0 and require the wire to move AT EXACTLY index 76, to that exact value.
    //    That is what makes the last byte a holdGuard byte instead of merely a zero byte, and
    //    it is a live negative control -- deleting `SIM_MEMBER(MovementInput, flags)`, or
    //    re-pointing bit 0 at some other signal, turns this red.
    static constexpr std::uint32_t kMovementFlagsByte = kZeroInputWireBytes - 1u;   // 76
    REQUIRE(after[kMovementFlagsByte] == 0u);

    auto guarded = simulatableBrawler::getZeroPlayerInput();
    guarded.edit<brawlerMovementSimulation::PlayerInput>().flags =
        brawlerMovementSimulation::kInputFlagHoldGuard;
    const std::vector<std::uint8_t> guardedBytes = serializeInput(guarded);

    INFO("holdGuard-set input: first differing byte = "
         << firstDifference(guardedBytes, kZeroInputWireBefore)
         << ", byte[76] = " << static_cast<int>(guardedBytes[kMovementFlagsByte]));
    REQUIRE(firstDifference(guardedBytes, kZeroInputWireBefore)
            == static_cast<int>(kMovementFlagsByte));
    REQUIRE(guardedBytes[kMovementFlagsByte]
            == brawlerMovementSimulation::kInputFlagHoldGuard);
    REQUIRE(brawlerMovementSimulation::kInputFlagHoldGuard == 1u);
}

// ---------------------------------------------------------------------------
// [movement-sim task 23] simulatableBrawler::DerivedState is a
// SimulationDerivedComposite, and that is what makes D1 mechanical.
//
// D1 says derived state never crosses the wire. Until task 23 that was a COMMENT
// on the slice; a SerializableFields specialization added to any derived slice
// would have compiled, silently costing wire bytes and checksum coverage. The
// composite alias carries `requires (!(Serializable<Ts> || ...))`, so the same
// mistake is now a build break -- and the assertions below are over the REAL five
// production slices, not over mocks (SimulationDerivedCompositeTest.cpp in
// og-simulation-tests owns the mock-level accept/reject matrix).
//
// `BrawlerDerivedNameable` is a concept template rather than an inline
// `requires { ... }` because MSVC 14.38 mis-evaluates the inline form over a
// concrete instantiation -- the workaround task 22 recorded for this suite.
// ---------------------------------------------------------------------------

template <typename... Ts>
concept BrawlerDerivedNameable = requires { typename SimulationDerivedComposite<Ts...>; };

TEST_CASE("DAttack.SimulatableBrawler.DerivedStateIsOffWire", "[DAttack][SimulatableBrawler]")
{
    // 1. EVERY ONE OF THE FIVE SLICES is off-wire. These are the load-bearing
    //    absence assertions: Serializable<T> means "T has a SerializableFields
    //    specialization", and each of these types deliberately has none.
    STATIC_REQUIRE_FALSE(Serializable<dAttackRadialSimulation::DerivedState>);
    STATIC_REQUIRE_FALSE(Serializable<dAttackGuardSimulation::DerivedState>);
    STATIC_REQUIRE_FALSE(Serializable<brawlerProjectileSimulation::DerivedState>);
    STATIC_REQUIRE_FALSE(Serializable<brawlerMovementSimulation::DerivedState>);
    STATIC_REQUIRE_FALSE(Serializable<brawlerInboundHit::DerivedState>);

    // 2. ANTI-VACUITY. `Serializable` is not simply false for everything in sight:
    //    the State SLICES in the same TU satisfy it.
    STATIC_REQUIRE(Serializable<dAttackRadialSimulation::State>);
    STATIC_REQUIRE(Serializable<brawlerMovementSimulation::State>);

    // 2b. ⚠ AND HERE IS WHAT DOES *NOT* DISCRIMINATE, recorded so nobody adds it
    //     back believing it proves something. A SimulationComposite is never itself
    //     Serializable -- composites are written through writeCompositeToSyncedBuffer,
    //     which constrains the ELEMENTS, not the composite type. So
    //     `!Serializable<DerivedState>` holds for the on-wire State composite too,
    //     and asserting it would be a fence that cannot fail. The next two lines
    //     assert exactly that non-discrimination, which is a real fact about the
    //     serialization design, and is why the fence in section 3 exists instead.
    STATIC_REQUIRE_FALSE(Serializable<simulatableBrawler::DerivedState>);
    STATIC_REQUIRE_FALSE(Serializable<simulatableBrawler::State>);

    // 3. THE FENCE, over the production slice list. Naming the real five is fine;
    //    appending a slice that IS on the wire does not compile. This is the whole
    //    deliverable of task 23 -- if this pair ever reads TRUE/TRUE, the alias has
    //    been swapped back for an unconstrained one and D1 is a comment again.
    //    [ringout task 2, 2026-09-13] `brawlerRingout::DerivedState` joins the list. It is the
    //    OFF-WIRE half of this task's composite growth and the reason the concept above is
    //    worth having: `diedThisTick` is a single-tick EDGE, and an edge that could arrive on
    //    a correction is an edge the score system could award twice. Its absence from any
    //    `SerializableFields` specialization is what this line pins.
    STATIC_REQUIRE(BrawlerDerivedNameable<dAttackRadialSimulation::DerivedState,
                                          dAttackGuardSimulation::DerivedState,
                                          brawlerProjectileSimulation::DerivedState,
                                          brawlerMovementSimulation::DerivedState,
                                          brawlerInboundHit::DerivedState,
                                          brawlerRingout::DerivedState>);
    STATIC_REQUIRE_FALSE(BrawlerDerivedNameable<dAttackRadialSimulation::DerivedState,
                                                dAttackGuardSimulation::DerivedState,
                                                brawlerProjectileSimulation::DerivedState,
                                                brawlerMovementSimulation::DerivedState,
                                                brawlerInboundHit::DerivedState,
                                                brawlerRingout::DerivedState,
                                                dAttackRadialSimulation::State>);
    //    [ringout task 2] AND THE POISON ARM AIMED AT THIS TASK'S OWN SLICE, so the positive
    //    arm above is not the only thing standing between ring-out and the wire. Ring-out's
    //    `State` IS serializable; naming it here must therefore read FALSE exactly as the
    //    radial `State` does. Without this line, a future edit that gave
    //    `brawlerRingout::DerivedState` a `SerializableFields` specialization would flip the
    //    positive arm to a compile error only by luck of which list it was added to.
    STATIC_REQUIRE_FALSE(BrawlerDerivedNameable<dAttackRadialSimulation::DerivedState,
                                                dAttackGuardSimulation::DerivedState,
                                                brawlerProjectileSimulation::DerivedState,
                                                brawlerMovementSimulation::DerivedState,
                                                brawlerInboundHit::DerivedState,
                                                brawlerRingout::State>);

    // 4. THE ELEMENT LIST IS PINNED. Adding or dropping a derived slice is a
    //    deliberate edit to SimulatableBrawlerTypes.h, not something a refactor
    //    does on the way past.
    //    [ringout task 2, 2026-09-13] Sixth element: `brawlerRingout::DerivedState`, APPENDED.
    //    DerivedState never rides the wire, so unlike the State composite this order carries
    //    no byte offsets and appending is a convention here rather than a requirement.
    STATIC_REQUIRE(std::is_same_v<
        simulatableBrawler::DerivedState,
        SimulationDerivedComposite<dAttackRadialSimulation::DerivedState,
                                   dAttackGuardSimulation::DerivedState,
                                   brawlerProjectileSimulation::DerivedState,
                                   brawlerMovementSimulation::DerivedState,
                                   brawlerInboundHit::DerivedState,
                                   brawlerRingout::DerivedState>>);

    // 5. THE COPY STILL CARRIES THE SCRATCH. AllState is copied wholesale every
    //    render step by SimmableUpdateComponent's updateVizState(), and the viz
    //    reads the radial and projectile slices out of that copy. A naked class
    //    gave memberwise copy for free; this pins that the tuple does the same,
    //    through get<>/edit<> rather than through member names.
    simulatableBrawler::AllState allState;
    allState.editDerivedState().edit<brawlerInboundHit::DerivedState>().wasHitThisTick = true;
    allState.editDerivedState()
        .edit<brawlerInboundHit::DerivedState>().wasProjectileBlockedThisTick = true;
    allState.editDerivedState()
        .edit<brawlerProjectileSimulation::DerivedState>().hits.push_back({});

    const simulatableBrawler::AllState vizCopy = allState;
    REQUIRE(vizCopy.getDerivedState().get<brawlerInboundHit::DerivedState>().wasHitThisTick);
    REQUIRE(vizCopy.getDerivedState()
                .get<brawlerInboundHit::DerivedState>().wasProjectileBlockedThisTick);
    REQUIRE(vizCopy.getDerivedState()
                .get<brawlerProjectileSimulation::DerivedState>().hits.size() == 1u);

    // ...and the copy is a COPY: mutating the original does not reach into it.
    allState.editDerivedState().edit<brawlerInboundHit::DerivedState>().wasHitThisTick = false;
    REQUIRE(vizCopy.getDerivedState().get<brawlerInboundHit::DerivedState>().wasHitThisTick);

    // 6. THE DEFAULTED CONSTRUCTION STILL RUNS EACH SLICE'S OWN CONSTRUCTOR.
    //    The radial slice reserves 4 entries in each of its two hit vectors in its
    //    default ctor; a tuple that value-initialised past it would show 0.
    //
    //    [movement-sim task 34] RE-ANCHORED FROM size() TO capacity(), and the intent
    //    stated above is unchanged — capacity() is a strictly better observable for it.
    //    The ctor used to say `attackHits(4)`, which is a RESIZE, so size() happened to
    //    read 4 and was used as the proxy for "the slice ctor ran". Those four
    //    default-constructed entries were a live bug: the hit detector (dAttackRadialSimulation's
    //    collisionCheck then; brawlerHitDetection::detectRadialHits since og-netcode-v2-field-
    //    defects task 9) early-returns at `attackHits.size() >= 4`, so a fresh character
    //    swinging on its first tick registered no hits at all (task 34). The ctor now
    //    RESERVES, which is what the comment above always claimed it did. A tuple that
    //    value-initialised past the slice ctor still shows 0 here — capacity 0 — so this
    //    case still fails for exactly the reason it was written to catch.
    const simulatableBrawler::DerivedState fresh;
    REQUIRE(fresh.get<dAttackRadialSimulation::DerivedState>().getAttackHits().capacity() >= 4u);
    REQUIRE(fresh.get<dAttackRadialSimulation::DerivedState>().getGuardHits().capacity() >= 4u);
    REQUIRE_FALSE(fresh.get<brawlerInboundHit::DerivedState>().wasHitThisTick);
}


// ===========================================================================
// ⭐⭐ [movement-sim task 50] STEP 6' MUST SURVIVE A CORRECTION.
//
// THE DEFECT THESE TWO CASES PIN. Step 6' recovers the solver's push-out by
// subtracting the pose we HANDED the engine last tick from the pose the capture
// handed back. Before task 50 both operands of that subtraction were OFF-WIRE
// scratch -- a remembered pose and a remembered velocity, gated by an off-wire
// `bool` -- and a correction destroys off-wire scratch on BOTH of its paths:
//
//   * ADOPTED (`isSimilarTo` said no). `SimulationReconciliation::
//     injectCorrectionState` DEFAULT-CONSTRUCTS `T::StateType` and then
//     `readInto`s the wire fields over it, `StateCorrectionCache::
//     tryInsertingCorrectState` stores that whole struct
//     (`if (!predictionWasCorrect) m_stateBuffer[i] = std::move(state)`), and
//     `prepareResimAll` assigns it back WHOLE (`editState() = cache.getState(idx)`).
//     Everything off-wire is therefore zeroed. -> case 1.
//   * AGREED. The slot is NOT overwritten, so the restored scratch is the
//     client's own and is VALID -- and `SimulatableBrawler::firstResimStep`
//     cleared the command flag anyway, throwing a good push-out away. -> case 2.
//
// Either way step 6' was a no-op for the first replayed tick, the into-wall
// velocity component was not killed, and the replay ended the tick a full
// `acceleration * dt` above the authority -- 34.133 cm/s against
// `kDefaultSimilarityEpsilon = 0.0001`. The authority's NEXT correction then
// disagrees, anchors, restores, and the replay skips again: one resim per tick
// for the duration of the contact.
//
// ⚠ THE FIXTURE FLIP, AND WHY IT IS KEPT NOW THAT PRODUCTION AGREES WITH IT.
// `if (state.flags & kFlagHasCommand)` gates the whole of step 6', and that bit is
// set by step 5 only when `drivesBody` is true. When these cases were written the
// member defaulted to `false`, so both would have passed while testing nothing --
// hence the explicit flip on each case's OWN LOCAL `StaticData` instance, a fixture
// choice exactly as a mock adapter is.
// ⭐ [movement-sim task 15] PRODUCTION NOW SHIPS `drivesBody = true`, paired with
// `PhysicsSetup::body.simulatePhysics = true`, so these flips have become redundant
// with the default rather than contrary to it. They are KEPT DELIBERATELY: a case
// that states its own precondition does not silently change meaning the next time
// somebody moves a default -- which is precisely what task 15 did to the sibling rig
// in `BrawlerMovementSimulationTest.cpp`, and it cost six red cases to notice.
// ⛔ The `REQUIRE(pushOut > kPushOutEps)` premise in each case is what makes that
// non-negotiable: delete the fixture flip and the case fails loudly instead of
// passing vacuously.
//
// ⛔ THE ONE AUTHORED RULE THIS DOES NOT TOUCH. Nothing here relaxes an
// `isSimilarTo` comparison to agree with the behaviour under test. The assertion
// is the SHIPPED production comparison at the SHIPPED epsilon; only the
// simulation was changed.
// ===========================================================================

namespace
{
namespace movementResim
{
    constexpr float kDt = 1.f / 60.f;

    // The wall plane, in world +X. The capsule's origin may not pass it.
    constexpr float kWallX = 0.f;

    // T -- the tick the correction lands on. Ticks 2..6 are all contact ticks, so
    // the "stick into it for >= 3 ticks" precondition is met with margin and the
    // contact is a steady state rather than a transient.
    constexpr std::uint32_t kCorrectionTick = 6u;

    // `acceleration * dt`. THIS IS THE DEFECT'S MAGNITUDE: in sustained contact the
    // authority enters step 6' at exactly this speed (killed to 0, then re-accelerated
    // from 0 by the ContinuousAccelBrake model), so a replay that skips step 6'
    // re-accelerates from it instead and ends the tick at 2x.
    //
    // ⭐⭐ NOW ACTUALLY DERIVED — [movement-sim task 16, defect N-3 routed here from task 50's
    // review]. It used to be a `constexpr float` initialised from a hand-typed `2048.f / 60.f`,
    // under a comment saying "Derived from the production StaticData in
    // SimulatableBrawlerTypes.h, not typed in as a magic number" — precisely what it was not: the
    // 2048 was HAND-TYPED and had to be re-typed by hand on every retune, and the
    // comment asserted a derivation the code did not perform. That is the same shape task 9's
    // defect-pattern section wrote up — a name or comment claiming provenance the code lacks —
    // and it is fail-loud only in the sense that the failure would have been a confusing red
    // assertion in an unrelated task, days later, blamed on that task's change.
    //
    // ⚠ WHAT THIS DOES AND DOES NOT PIN. The property under test is the RATIO — a replay that
    // skips step 6' ends the tick at 2x the authority's speed — not the absolute 34.13 cm/s.
    // Reading the acceleration off the production object is therefore the CORRECT coupling:
    // retuning `acceleration` must move both sides of the comparison together, and a case that
    // went red on a pure retune would be a false alarm. It is R-P1 applied to a test: the
    // literal is spelled once, in SimulatableBrawlerTypes.h, and read from there.
    //
    // A function rather than a constant because `simulatableBrawler::StaticData` is
    // non-copyable and non-movable, so it cannot be a namespace-scope value initialised from a
    // temporary; the function-local static gives it one construction for the whole file.
    inline float accelPerTick()
    {
        static const simulatableBrawler::StaticData kProductionStaticData;
        return kProductionStaticData.m_movementStaticData.acceleration * kDt;
    }

    // Byte-addressable stand-in for FSimulationStateSyncBuffer -- the same shape
    // ZeroInputIsTheFold's probe buffer uses, and the shape
    // correctionStateBuffer's Buffer concept asks for.
    struct FCorrectionProbeBuffer
    {
        std::vector<std::uint8_t> bytes;

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        { T v; std::memcpy(&v, bytes.data() + off, sizeof(T)); return v; }
    };

    // ⭐ THE ADOPTION, THROUGH THE REAL CODEC AND NOTHING ELSE. This is
    // `injectCorrectionState` transcribed: a DEFAULT-CONSTRUCTED StateType, then
    // `readInto` over it. Using the shipped codec rather than copying fields by
    // hand is what makes these cases evidence that `positionCmd` RIDES THE WIRE
    // -- a hand-written field copy would pass whatever the serializer did.
    simulatableBrawler::State adoptOverTheWire(const simulatableBrawler::State& authority,
                                               std::uint32_t tick)
    {
        FCorrectionProbeBuffer buffer;
        // Poison, not zero: a field the codec never writes would otherwise read
        // back as a legitimate zero and the round trip would pass on a hole.
        buffer.bytes.assign(kStateSyncBufferBytes, 0xCDu);
        correctionStateBuffer::write(buffer, authority, tick, kNoInputCaptureTick);

        simulatableBrawler::State adopted;   // DEFAULT-CONSTRUCTED, as production does
        std::uint32_t appliedCaptureTick = 0u;
        const std::uint32_t readTick =
            correctionStateBuffer::readInto(buffer, adopted, appliedCaptureTick);
        REQUIRE(readTick == tick);
        REQUIRE(appliedCaptureTick == kNoInputCaptureTick);
        return adopted;
    }

    // One peer: a character, its adapters and its own StaticData instance.
    // ⚠ `simulatableBrawler::StaticData` is non-copyable and non-movable (its
    // sub-StaticData members hold references into its own siblings), so it must be
    // a named member constructed in place -- which is also why this struct is
    // neither copied nor returned by value anywhere below.
    struct FPeer
    {
        simulatableBrawler::StaticData staticData;
        SimulatableBrawler            character;
        FMockPhysicsBodyAdapter       phys;
        FMockSpatialQueryAdapter      query;

        FPeer() : character(staticData)
        {
            // THE FIXTURE FLIP -- see the block comment above. The constructor
            // ignores its StaticData argument entirely (SimulatableBrawler.cpp),
            // so setting this afterwards is equivalent to setting it before.
            staticData.m_movementStaticData.drivesBody = true;
            character.setCharacterBindings({ BodyId{1u} });
            // `queryVolumeIds` is left EMPTY on purpose: step 2 then skips the
            // sweep, the surface reads Airborne and the vertical term is pure
            // authored gravity. That term is identical on both peers and
            // orthogonal to the +X wall, so it can neither mask nor manufacture
            // the divergence this case measures.
        }

        brawlerMovementSimulation::State& movement()
        {
            return character.editAllState().editState()
                .edit<brawlerMovementSimulation::State>();
        }
        const brawlerMovementSimulation::State& movement() const
        {
            return character.getAllState().getState()
                .get<brawlerMovementSimulation::State>();
        }

        glm::vec3 lastPushOut() const
        {
            return character.getAllState().getDerivedState()
                .get<brawlerMovementSimulation::DerivedState>().lastPushOut;
        }

        // ⭐ THE ENGINE STEP AND THE POST-SOLVE CAPTURE, in the production order.
        // Nothing in SimulatableBrawler advances the body -- the generic
        // `captureBodyStatesAll` pass does, from whatever the solver produced, and
        // an engine-free rig must stand in for it. The solver here is analytic and
        // written from first principles: integrate the command, then push the body
        // back out of the wall along -X. It is deliberately NOT derived from step
        // 6's own arithmetic, which is what keeps the case non-circular.
        void engineStepAndCapture()
        {
            brawlerMovementSimulation::State& s = movement();
            glm::vec3 solved = s.bodyState.position + s.velocity * kDt;
            if (solved.x > kWallX)
                solved.x = kWallX;
            s.bodyState.position = solved;
        }

        void tick(std::uint32_t t, const simulatableBrawler::PlayerInput& input)
        {
            engineStepAndCapture();
            character.integrate(SimulationTimeStep(t, false, false, false, kDt),
                                input, phys, query, staticData);
        }
    };

    simulatableBrawler::PlayerInput stickIntoTheWall()
    {
        simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
        // The stick arrives as a world XY direction whose LENGTH is the deflection
        // (BrawlerInputPackaging.h), so a unit +X vector is "full deflection, into
        // the wall".
        input.edit<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld =
            glm::vec3(1.f, 0.f, 0.f);
        return input;
    }

    // Seat a peer exactly ON the wall plane so contact is established on tick 2
    // and every tick after it.
    void seatAgainstTheWall(FPeer& peer)
    {
        peer.movement().bodyState.position = glm::vec3(kWallX, 0.f, 0.f);
    }

    // The premises every case shares: the latent path is LIVE in this fixture, the
    // contact is real, and the authority is in the steady state the arithmetic
    // above describes. These stay GREEN in the pre-fix (RED) run -- they are what
    // makes the failing assertion a statement about the replay rather than about
    // the rig.
    void requireSustainedContactSteadyState(const FPeer& authority)
    {
        INFO("authority velocity=(" << authority.movement().velocity.x << ", "
             << authority.movement().velocity.y << ", "
             << authority.movement().velocity.z << ") pushOut.x="
             << authority.lastPushOut().x);

        // 1. STEP 6' RAN, and it found a real push-out. If the fixture flip above
        //    is ever removed this is the assertion that fails.
        REQUIRE(glm::length(authority.lastPushOut())
                > brawlerMovementSimulation::kPushOutEps);

        // 2. THE CONTACT CLAMP HELD: the into-wall component was killed to zero and
        //    the model re-accelerated from zero, so the authority ends the tick at
        //    exactly one tick of acceleration.
        REQUIRE(authority.movement().velocity.x
                == Catch::Approx(accelPerTick()).margin(1e-3f));

        // 3. AND THE BODY IS ON THE WALL, not through it.
        REQUIRE(authority.movement().bodyState.position.x
                == Catch::Approx(kWallX).margin(1e-4f));
    }
}
}

// ---------------------------------------------------------------------------
// CASE 1 -- THE ADOPTED CORRECTION. The disagreeing landing overwrites the slot
// with a default-constructed-then-readInto authority state, so every off-wire
// field is zeroed. The replay of T+1 must still reproduce the authority's T+1.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulatableBrawler.ReplayAfterAdoptionReproducesContactClamp",
          "[DAttack][SimulatableBrawler][MovementResim]")
{
    using namespace movementResim;

    const simulatableBrawler::PlayerInput stick = stickIntoTheWall();

    FPeer authority;
    FPeer client;
    seatAgainstTheWall(authority);
    seatAgainstTheWall(client);

    // ---- ticks 1..T: both peers press into the wall and agree throughout.
    for (std::uint32_t t = 1u; t <= kCorrectionTick; ++t)
    {
        authority.tick(t, stick);
        client.tick(t, stick);
    }
    requireSustainedContactSteadyState(authority);

    const simulatableBrawler::State authoritySlotT = authority.character.getAllState().getState();

    // ---- FORCE A DISAGREEING LANDING AT T.
    // The client mispredicted its position. The field is chosen for the one
    // property that matters: it is in SerializableFields, so `isSimilarTo` -- the
    // shipped resim trigger -- can see it. (Same technique, same reason, as
    // SimulationNetSyncTest.cpp's `divergeState`.)
    client.movement().bodyState.position.x = kWallX - 5.f;

    // THE PREMISE OF THE WHOLE CASE: this landing really does disagree, so
    // `tryInsertingCorrectState`'s `if (!predictionWasCorrect)` branch is the one
    // production takes and the slot really is overwritten.
    REQUIRE_FALSE(client.character.getAllState().getState().isSimilarTo(authoritySlotT));

    // ---- ADOPT. injectCorrectionState (default-construct + readInto over the
    // real codec) followed by prepareResimAll's whole-struct assignment.
    client.character.editAllState().editState() =
        adoptOverTheWire(authoritySlotT, kCorrectionTick);

    // The adoption restored every WIRE field exactly -- including, after task 50,
    // the one step 6' needs. Asserted before the replay so a failure here reads as
    // "the wire lost it" rather than as "the replay diverged".
    REQUIRE(client.character.getAllState().getState().isSimilarTo(authoritySlotT));

    // ⭐ AND SPECIFICALLY `positionCmd`, BY VALUE. `isSimilarTo` alone would not
    // prove this: before task 50 the field was off-wire, so the fold could not see
    // it and the line above passed while the field was silently zeroed. These three
    // are the direct statement that the codec CARRIED it:
    //   - it survived the round trip,
    //   - it is not the default-constructed value the buffer would leave behind,
    //   - and the command bit survived with it, because step 6's gate is that bit.
    const brawlerMovementSimulation::State& authorityAtT =
        authoritySlotT.get<brawlerMovementSimulation::State>();
    REQUIRE(client.movement().positionCmd == authorityAtT.positionCmd);
    REQUIRE_FALSE(client.movement().positionCmd == glm::vec3(0.f));
    REQUIRE((client.movement().flags & brawlerMovementSimulation::kFlagHasCommand) != 0u);

    client.character.firstResimStep(client.phys, 0);

    // ---- REPLAY T+1 on the client while the authority simulates it live.
    authority.tick(kCorrectionTick + 1u, stick);
    client.tick(kCorrectionTick + 1u, stick);

    INFO("authority v.x=" << authority.movement().velocity.x
         << "  client v.x=" << client.movement().velocity.x
         << "  divergence=" << (client.movement().velocity.x - authority.movement().velocity.x)
         << "  (a*dt = " << accelPerTick() << ")");

    // ⭐ THE ASSERTION. The shipped comparison, at the shipped epsilon.
    REQUIRE(client.character.getAllState().getState()
                .isSimilarTo(authority.character.getAllState().getState()));

    // ...and SPELLED OUT, so a break names the term rather than only the verdict.
    REQUIRE(client.movement().velocity.x
            == Catch::Approx(authority.movement().velocity.x).margin(1e-4f));
    REQUIRE(authority.movement().velocity.x == Catch::Approx(accelPerTick()).margin(1e-3f));
}

// ---------------------------------------------------------------------------
// CASE 2 -- THE AGREEING ANCHOR. The landing agreed, so the cache KEPT the
// client's own prediction and the scratch restored by prepareResimAll is valid.
// `firstResimStep` must not throw it away.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulatableBrawler.AgreeingAnchorKeepsPushOut",
          "[DAttack][SimulatableBrawler][MovementResim]")
{
    using namespace movementResim;

    const simulatableBrawler::PlayerInput stick = stickIntoTheWall();

    FPeer authority;
    FPeer client;
    seatAgainstTheWall(authority);
    seatAgainstTheWall(client);

    for (std::uint32_t t = 1u; t <= kCorrectionTick; ++t)
    {
        authority.tick(t, stick);
        client.tick(t, stick);
    }
    requireSustainedContactSteadyState(authority);

    const simulatableBrawler::State authoritySlotT = authority.character.getAllState().getState();

    // THE PREMISE: the prediction was CORRECT, so `tryInsertingCorrectState` takes
    // the other branch -- `m_stateBuffer[i]` is NOT overwritten and the slot keeps
    // the client's own state, off-wire scratch included.
    REQUIRE(client.character.getAllState().getState().isSimilarTo(authoritySlotT));

    // prepareResimAll still restores that slot WHOLE. Modelled as the copy-out /
    // assign-back it is, so the rig makes the same round trip production does.
    const simulatableBrawler::State cachedPrediction = client.character.getAllState().getState();
    client.character.editAllState().editState() = cachedPrediction;

    // ⭐ THE SUBJECT OF THIS CASE. Before task 50 this line cleared the command
    // marker unconditionally, which discarded a VALID push-out and made step 6' a
    // no-op for one replayed tick. Architecture 3.5 always described
    // `firstResimStep` as a no-op for this sub-simulation; task 50 makes that true
    // again.
    client.character.firstResimStep(client.phys, 0);

    authority.tick(kCorrectionTick + 1u, stick);
    client.tick(kCorrectionTick + 1u, stick);

    INFO("authority v.x=" << authority.movement().velocity.x
         << "  client v.x=" << client.movement().velocity.x
         << "  divergence=" << (client.movement().velocity.x - authority.movement().velocity.x)
         << "  (a*dt = " << accelPerTick() << ")");

    REQUIRE(client.character.getAllState().getState()
                .isSimilarTo(authority.character.getAllState().getState()));
    REQUIRE(client.lastPushOut() == authority.lastPushOut());
    REQUIRE(glm::length(client.lastPushOut()) > brawlerMovementSimulation::kPushOutEps);
}

// ---------------------------------------------------------------------------
// ⭐⭐ CASE 3 -- THE HIT SIGNAL IS OFF-WIRE, AND AN ANCHOR ON THE HIT TICK THEREFORE COSTS
// ONE EXTRA CORRECTION. PINNED, NOT FIXED. [movement-sim task 27]
//
// THE MECHANISM, and it PRE-DATES task 27. `wasHitThisTick` and the reaction beside it are written
// by `brawlerHitRouting::System::postIntegrate` at tick T for consumption at T+1, and they live on
// the DerivedState composite, which is off the wire by decision D1. A resim anchored at T restores
// the WIRE state and replays T+1 reading whatever the FRONTIER's post-integrate last left in that
// slice -- nothing re-fires the routing pass for the restored tick, and the radial derived hits it
// would need are gone. So a hit that landed at T+1 is MISSED on the first replayed tick.
//
// ⛔ WHY THIS IS BOUNDED AND NOT A STORM, which is the whole reason it is pinned rather than
// fixed: the CONSEQUENCE of a hit is entirely on the wire. `m_currentState`, `m_timeInCurrentState`,
// `m_hitReaction`, `m_flinchDuration` and the movement `velocity` all ride a correction, so the
// authority's T+1 correction disagrees ONCE, is adopted, and every tick after it is right. Task 27
// makes the one disagreeing tick more visible (a 5 m slide instead of a 0.3 s freeze) without
// making it longer.
//
// ⭐ WHAT WOULD FIX IT, recorded so the decision is not re-derived: one more byte of machine
// `State` carrying the hit signal itself (50 B of headroom after this task). It is cheap, and it is
// deliberately NOT taken here -- the question is whether the one-cycle pop is VISIBLE in PIE, and
// no agent can run PIE. The 27b PIE criterion asks for exactly that observation.
// ---------------------------------------------------------------------------
//
// ⭐ [og-netcode-v2-field-defects task 20] THE MECHANISM ABOVE IS NARROWED, NOT GONE. Routing now
// runs in preIntegrate of the CONSUMING tick, so a replay anchored at T re-routes T's hit on its
// first step -- when detection can re-find it. It cannot while the attacker's per-swing
// `attackHits` ledger (derived, never restored) still holds the target, i.e. whenever the
// correction lands mid-swing. That remaining case is exactly what this rig models (no routing
// runs here; the slice is delivered by hand) and is pinned end to end in
// BrawlerHitDetectionBehaviourTest.cpp, `...BodyHitTickFlinchesTheTarget`, section "frontier
// mid-swing"; Backlog task 21 closes it.
TEST_CASE("DAttack.SimulatableBrawler.ReplayAnchoredOnHitTickConvergesInOneCorrection",
          "[DAttack][SimulatableBrawler][MovementResim]")
{
    using namespace movementResim;

    const simulatableBrawler::PlayerInput idle = simulatableBrawler::getZeroPlayerInput();

    FPeer authority;
    FPeer client;

    // SEATED CLEAR OF THE RIG'S WALL. `FPeer::engineStepAndCapture` clamps the body at
    // `kWallX == 0` -- it is the task-50 contact fixture -- and a 5 m slide launched from the
    // origin would be pressed into that wall on every tick, which step 6' would then correctly
    // kill. This case is about a CORRECTION, not about contact, so the two peers start 10 m short
    // of the wall and the whole slide happens in open space.
    authority.movement().bodyState.position = glm::vec3(-1000.f, 0.f, 0.f);
    client.movement().bodyState.position    = glm::vec3(-1000.f, 0.f, 0.f);

    // The reaction the authority's routing pass would have resolved for the shipped right-hand
    // swing: a knockback at the authored speed, dwelling for its own slide time. Read from the
    // authority's OWN StaticData so the row moves with a retune.
    const HitReactionSpec& spec =
        authority.staticData.m_hitReactions[dAttackDirection::kRightSequenceId];
    const float dwell = spec.knockbackSpeed / authority.staticData.m_movementStaticData.launchDecel;
    REQUIRE(spec.kind == HitReactionKind::Knockback);

    const auto deliverHit = [&](FPeer& peer)
    {
        auto& slice = peer.character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>();
        slice.wasHitThisTick = true;
        slice.reactionKind   = HitReactionKind::Knockback;
        slice.knockbackSpeed = spec.knockbackSpeed;
        slice.hitDirectionXY = glm::vec2(1.f, 0.f);
        slice.flinchDuration = dwell;
    };
    const auto clearSlice = [&](FPeer& peer)
    {
        peer.character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>() = brawlerInboundHit::DerivedState{};
    };

    // ---- ticks 1..T: no hits anywhere; both peers agree.
    for (std::uint32_t t = 1u; t <= kCorrectionTick; ++t)
    {
        authority.tick(t, idle);
        client.tick(t, idle);
        clearSlice(authority);
        clearSlice(client);
    }
    const simulatableBrawler::State authoritySlotT = authority.character.getAllState().getState();
    REQUIRE(client.character.getAllState().getState().isSimilarTo(authoritySlotT));

    // ---- ANCHOR AT T. The landing AGREED, so the cache keeps the client's own prediction and
    //      `prepareResimAll` restores it whole -- modelled as the copy-out / assign-back it is.
    const simulatableBrawler::State cachedPrediction = client.character.getAllState().getState();
    client.character.editAllState().editState() = cachedPrediction;
    client.character.firstResimStep(client.phys, 0);

    // ---- T+1: THE AUTHORITY IS HIT. The client replays the same tick with NO hit signal, because
    //      nothing re-fires the routing pass for a replayed tick. That asymmetry IS the hazard.
    deliverHit(authority);
    authority.tick(kCorrectionTick + 1u, idle);
    clearSlice(authority);
    client.tick(kCorrectionTick + 1u, idle);

    const auto& authorityMachine = authority.character.getAllState().getState()
        .get<dAttackMachineSimulation::State>();
    const auto& clientMachine = client.character.getAllState().getState()
        .get<dAttackMachineSimulation::State>();
    INFO("T+1 authority: state=" << int(authorityMachine.m_currentState)
         << " reaction=" << int(authorityMachine.m_hitReaction)
         << " v.x=" << authority.movement().velocity.x
         << " | client: state=" << int(clientMachine.m_currentState)
         << " v.x=" << client.movement().velocity.x);

    // 1. THE DISAGREEMENT IS REAL, and it is the one the shipped comparison sees -- so production
    //    really does correct here rather than sliding on silently.
    REQUIRE(authorityMachine.m_currentState == DAttackState::HitFlinch);
    REQUIRE(authorityMachine.m_hitReaction == HitReactionKind::Knockback);
    REQUIRE(authority.movement().velocity.x
            == Catch::Approx(spec.knockbackSpeed).margin(1e-2f));
    REQUIRE(clientMachine.m_currentState == DAttackState::Idle);
    REQUIRE_FALSE(client.character.getAllState().getState()
                      .isSimilarTo(authority.character.getAllState().getState()));

    // 2. ...AND IT IS ADOPTED THROUGH THE REAL CODEC. Everything the hit did is on the wire, so
    //    the adoption alone -- with no replayed hit signal anywhere -- restores the whole slide.
    const simulatableBrawler::State authoritySlotT1 =
        authority.character.getAllState().getState();
    client.character.editAllState().editState() =
        adoptOverTheWire(authoritySlotT1, kCorrectionTick + 1u);
    client.character.firstResimStep(client.phys, 0);
    REQUIRE(client.character.getAllState().getState().isSimilarTo(authoritySlotT1));
    REQUIRE(client.character.getAllState().getState()
                .get<dAttackMachineSimulation::State>().m_hitReaction
            == HitReactionKind::Knockback);
    REQUIRE(client.character.getAllState().getState()
                .get<dAttackMachineSimulation::State>().m_flinchDuration
            == Catch::Approx(dwell).margin(1e-6f));
    REQUIRE(client.movement().velocity.x == Catch::Approx(spec.knockbackSpeed).margin(1e-2f));

    // 3. ONE CORRECTION, NOT A STORM. From the adopted tick on, the client reproduces the
    //    authority tick for tick with NO further hit signal on either side -- the decay runs off
    //    the wire state alone. Ten ticks is a fifth of the slide; a per-tick resim would fail on
    //    the first of them.
    for (std::uint32_t t = kCorrectionTick + 2u; t <= kCorrectionTick + 11u; ++t)
    {
        authority.tick(t, idle);
        client.tick(t, idle);
        INFO("replayed tick " << t << ": authority v.x=" << authority.movement().velocity.x
             << " client v.x=" << client.movement().velocity.x);
        REQUIRE(client.character.getAllState().getState()
                    .isSimilarTo(authority.character.getAllState().getState()));
        REQUIRE(client.movement().velocity.x
                == Catch::Approx(authority.movement().velocity.x).margin(1e-4f));
    }

    // ...and the slide really was running through all of that, so row 3 is not a comparison of two
    // stationary characters.
    REQUIRE(authority.movement().velocity.x > 0.f);
    REQUIRE(authority.movement().velocity.x < spec.knockbackSpeed);
}

#endif // WITH_LOW_LEVEL_TESTS
