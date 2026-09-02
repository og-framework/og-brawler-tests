// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
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
// this measures 324 B, a **-28 B** delta — exactly the 52 -> 24 B narrowing of the
// one movement slice, with every other slice untouched.
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
    static_assert(FCompositeWireSize<simulatableBrawler::State>::value == 324u,
        "The simulatableBrawler::State wire footprint moved. That is a WIRE FORMAT "
        "CHANGE: re-measure it, bump correctionStateBuffer::kWireFormatVersion if the "
        "layout (not just the size) changed, and re-price RoundVsPacketBudgetTest.cpp.");
    REQUIRE(kComposite == 324u);

    // 2. WHICH SLICE, so a break says what moved rather than only that something did.
    REQUIRE(syncSize<brawlerMovementSimulation::State>() == 24u);
    REQUIRE(syncSize<brawlerMovementSimulation::InitialConditions>() == 0u);

    // 3. THE HEADROOM FENCE — the one that would have caught a silent runtime OOB.
    //    Also a compile-time fence, which is the whole point: it converts a failure
    //    mode that is otherwise a dropped correction plus a log line at runtime into
    //    a build break for whoever grows the composite.
    static_assert(correctionStateBuffer::kHeaderBytes
                      + FCompositeWireSize<simulatableBrawler::State>::value
                  <= kStateSyncBufferBytes,
        "The State composite no longer fits FSimulationStateSyncBuffer::kBufferBytes. "
        "Raise kBufferBytes (UE-sim/SyncedSimulationStateBuffer.h) to the next 64-byte "
        "multiple above the new footprint, and update the copy of it in this file. "
        "Raising the capacity is wire-cheap: NetSerialize watermark-trims to usedBytes.");
    REQUIRE(kBufferUsed <= kStateSyncBufferBytes);

    WARN("simulatableBrawler::State wire footprint: composite=" << kComposite
         << " B (was 352 B before the LinearBodyState swap; -28 B), buffer used="
         << kBufferUsed << "/" << kStateSyncBufferBytes << " B");
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

    constexpr std::uint32_t kZeroInputWireBytes = 76u;

    // The captured "before" bytes, grouped by composite slice in wire order:
    // radial(14) -> machine(38) -> guard(12) -> projectile(12) -> movement(0).
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
        // movement: no serialized fields
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
            && pa.aimDirection == pb.aimDirection;
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
    STATIC_REQUIRE(BrawlerDerivedNameable<dAttackRadialSimulation::DerivedState,
                                          dAttackGuardSimulation::DerivedState,
                                          brawlerProjectileSimulation::DerivedState,
                                          brawlerMovementSimulation::DerivedState,
                                          brawlerInboundHit::DerivedState>);
    STATIC_REQUIRE_FALSE(BrawlerDerivedNameable<dAttackRadialSimulation::DerivedState,
                                                dAttackGuardSimulation::DerivedState,
                                                brawlerProjectileSimulation::DerivedState,
                                                brawlerMovementSimulation::DerivedState,
                                                brawlerInboundHit::DerivedState,
                                                dAttackRadialSimulation::State>);

    // 4. THE ELEMENT LIST IS PINNED. Adding or dropping a derived slice is a
    //    deliberate edit to SimulatableBrawlerTypes.h, not something a refactor
    //    does on the way past.
    STATIC_REQUIRE(std::is_same_v<
        simulatableBrawler::DerivedState,
        SimulationDerivedComposite<dAttackRadialSimulation::DerivedState,
                                   dAttackGuardSimulation::DerivedState,
                                   brawlerProjectileSimulation::DerivedState,
                                   brawlerMovementSimulation::DerivedState,
                                   brawlerInboundHit::DerivedState>>);

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
    const simulatableBrawler::DerivedState fresh;
    REQUIRE(fresh.get<dAttackRadialSimulation::DerivedState>().getAttackHits().size() == 4u);
    REQUIRE(fresh.get<dAttackRadialSimulation::DerivedState>().getGuardHits().size() == 4u);
    REQUIRE_FALSE(fresh.get<brawlerInboundHit::DerivedState>().wasHitThisTick);
}

#endif // WITH_LOW_LEVEL_TESTS
