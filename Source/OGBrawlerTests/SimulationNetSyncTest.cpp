// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
// [T7] The pure D5.4 source rule, whose REMOTE branch this task re-points off the
// correction-input channel and onto the relay store's last-known.
#include "OGBrawler/BrawlerVisualizationInputSource.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// SimulationNetSync.h declares SimulatableOwnerTraits primary; must come before
// the specialization below.
#include "OGSimulation/SimulationNetSync.h"
// [T39] The state-rotation width lives in TimeConfig; the StateRotation cases at
// the end of this file assert against the shipped compiled default rather than
// re-typing it.
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

// [T15] The input provider's second parameter — the character's own raw capture
// history, handed in by collectInputAll. Aliased so the many provider lambdas in
// this file stay readable. No case below reads it; the cases that exercise the
// matcher's use of it live in MotionMatcherSourceTest.cpp, which drives the real
// engine-free core directly.
using BrawlerLocalInputCache = LocalInputCache<simulatableBrawler::PlayerInput>;

// [T39] `sendCorrectionAll` gained the state-rotation width K, deliberately
// without a default (see the note at its definition). Every case in this file
// that predates T39 is about WHAT the send publishes, not about cadence, so it
// passes the every-frame value: K >= N degenerates to "write every writer", which
// is byte-for-byte the pre-T39 behaviour those cases were written against.
//
// The rotation itself is pinned in two places, on purpose: as a unit — coverage,
// wrap, the K >= N degeneracy and the clamp — in
// og-simulation-tests/Network/CorrectionRotationTest.cpp, and as an INTEGRATION
// fact (that sendCorrectionAll actually honours it, over real authority writers)
// by the StateRotation cases at the end of this file.
constexpr int32 kEveryFrameRotationK = correctionRotation::kMaxK;

// ---------------------------------------------------------------------------
// Minimal mock synced buffer types for owner concept satisfaction
// ---------------------------------------------------------------------------

struct MockStateSyncBuffer
{
    // [og-netcode-v2-input-relay T4] The correction-state buffer now carries the
    // per-tick applied-capture-tick REFERENCE beside the state (the join key
    // between the state channel and the relayed-input channel), so this mock
    // RECORDS what the authority published and PLAYS IT BACK on read — the same
    // object can then stand in for the wire in a server-writes / client-reads
    // case. The real buffer keeps both values in its byte payload
    // (OGSimulation/CorrectionStateBufferCodec.h); the state composite itself is
    // still not modelled here, because no case in this file asserts on it.
    uint32 lastTick = 0;
    uint32 lastAppliedCaptureTick = kNoInputCaptureTick;
    int    writeCount = 0;

    void write(const simulatableBrawler::State& /*state*/, uint32 tick, uint32 appliedCaptureTick)
    {
        lastTick = tick;
        lastAppliedCaptureTick = appliedCaptureTick;
        ++writeCount;
    }

    // Ref-less publish — mirrors the real buffer, which writes the sentinel
    // rather than leaving a previous publish's reference in place.
    void write(const simulatableBrawler::State& state, uint32 tick)
    {
        write(state, tick, kNoInputCaptureTick);
    }

    uint32_t readInto(simulatableBrawler::State& /*outState*/) const { return lastTick; }
    uint32   getAppliedCaptureTick() const { return lastAppliedCaptureTick; }

    template <typename T>
    T readFromBuffer(uint32 /*byteIt*/) const { return T{}; }

    template <typename T>
    void writeToBuffer(uint32 /*byteIt*/, T /*val*/) {}
};

// [og-netcode-v2-input-relay T8] This mock now stands in for ONE role, not two.
// It used to model both `FSimulationInputSyncBuffer` members on the component: the
// CLIENT->SERVER buffer and the replicated SERVER->CLIENT correction-input buffer.
// The latter is retired, so every `inputBuf` observation the T4/T6/T17 cases made
// through it is gone with the channel (see the retirement inventory at
// SimulationNetSync::sendCorrectionAll). What survives is the client->server role,
// which the concept still requires — hence the mock, and hence the `write`/
// `readInto` pair that keeps satisfying CompositeSyncedBufferConcept.
struct MockInputSyncBuffer
{
    uint32 lastTick = 0;
    int    writeCount = 0;
    simulatableBrawler::PlayerInput lastInput{};

    void write(const simulatableBrawler::PlayerInput& input, uint32 tick)
    {
        lastTick = tick;
        lastInput = input;
        ++writeCount;
    }
    // Plays the payload back, matching what MockStateSyncBuffer does. [T6 fixed
    // this: it used to discard `outInput` and answer only the tick, which silently
    // made every round-trip hand back a VALUE-INITIALISED input. Kept faithful even
    // though T8 removed the case that first depended on it — a lying double is a
    // trap regardless of who is currently standing on it.]
    uint32_t readInto(simulatableBrawler::PlayerInput& outInput) const
    {
        outInput = lastInput;
        return lastTick;
    }

    template <typename T>
    T readFromBuffer(uint32 /*byteIt*/) const { return T{}; }

    template <typename T>
    void writeToBuffer(uint32 /*byteIt*/, T /*val*/) {}
};

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay T5] MockRelayedInputRing — the wire payload.
//
// A std::vector-backed byte buffer exposing the codec's BUFFER CONCEPT, i.e. the
// same four methods FRelayedInputRing exposes over its TArray<uint8>. Tests
// therefore drive the REAL RelayedInputRingCodec and the REAL
// populateRemoteInputCache against it; only the UE replication half (NetSerialize
// + OnRep) is stood in for, and OnRep is stood in for by calling the bound
// callback directly, which is literally all OnRep_RelayedInputRing does.
// ---------------------------------------------------------------------------

struct MockRelayedInputRing
{
    std::vector<std::uint8_t> bytes;

    std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

    void bundleAddZeroedBytes(std::int32_t count)
    { bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u); }

    template <typename T>
    void writeToBuffer(std::uint32_t off, const T& value)
    { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

    template <typename T>
    T readFromBuffer(std::uint32_t off) const
    {
        T value;
        std::memcpy(&value, bytes.data() + off, sizeof(T));
        return value;
    }
};

// ---------------------------------------------------------------------------
// MockPredictionOwner — satisfies PredictionSyncedBufferOwnerConcept
// ---------------------------------------------------------------------------

struct MockPredictionOwner
{
    using SyncedCorrectionBufferType  = MockStateSyncBuffer;
    using SyncedRemoteInputBufferType = MockInputSyncBuffer;
    using RelayedInputRingType        = MockRelayedInputRing;

    std::function<void(const MockStateSyncBuffer&)>  onCorrectionStateReceived;
    // [T8] `onCorrectionInputReceived` and its set/clear pair are gone with the
    // channel. `outgoingInputBuffer` stays — it is the CLIENT->SERVER buffer.
    MockInputSyncBuffer outgoingInputBuffer;

    // [T5] The relay ring + its arrival hook. `replicateRelayRing()` stands in for
    // OnRep_RelayedInputRing, which does exactly this and nothing else.
    MockRelayedInputRing relayedInputRing;
    std::function<void(const MockRelayedInputRing&)> onRelayedInputReceived;

    void setOnRelayedInputReceivedCallback(
        std::function<void(const MockRelayedInputRing&)> fn)
    { onRelayedInputReceived = std::move(fn); }

    void clearOnRelayedInputReceivedCallback()
    { onRelayedInputReceived = nullptr; }

    const MockRelayedInputRing& getRelayedInputRing() const { return relayedInputRing; }
    MockRelayedInputRing&       getRelayedInputRing()       { return relayedInputRing; }

    void replicateRelayRing()
    {
        if (onRelayedInputReceived)
            onRelayedInputReceived(relayedInputRing);
    }

    void setOnCorrectionStateReceivedCallback(
        std::function<void(const MockStateSyncBuffer&)> fn)
    { onCorrectionStateReceived = std::move(fn); }

    void clearOnCorrectionStateReceivedCallback()
    { onCorrectionStateReceived = nullptr; }

    MockInputSyncBuffer* getClientToServerInputSyncedBuffer()
    { return &outgoingInputBuffer; }

    // Stage 1 (Task 9): redundancy-bundle send path. The real owner builds an
    // FInputRedundancyBundle from the queue and fires an unreliable RPC; the mock
    // does not build a bundle (the wire type is UE-side and not under test here)
    // but it DOES record what the queue offered it.
    //
    // [T9 part 3] That recording is what makes the "the wire carries the
    // UNDELAYED capture" assertion possible. It is the only observation point for
    // the outbound half of the provider branch.
    struct SentSlot
    {
        uint32 tick;
        simulatableBrawler::PlayerInput input;
    };
    std::vector<SentSlot> sentSlots;

    void sendLocalInputToAuthority(
        const PendingInputQueue<simulatableBrawler::PlayerInput>& queue,
        uint32 currentTick,
        uint32 redundancyDepth)
    {
        queue.forEachRecent(currentTick, static_cast<size_t>(redundancyDepth),
            [this](uint32 tick, const simulatableBrawler::PlayerInput& input) {
                sentSlots.push_back(SentSlot{ tick, input });
            });
    }
};

static_assert(PredictionSyncedBufferOwnerConcept<MockPredictionOwner,
                                                  simulatableBrawler::State,
                                                  simulatableBrawler::PlayerInput>,
    "MockPredictionOwner must satisfy PredictionSyncedBufferOwnerConcept");

// ---------------------------------------------------------------------------
// MockAuthorityOwner — satisfies AuthoritySyncedBufferOwnerConcept
// ---------------------------------------------------------------------------

// [T8] The authority owner's OUTBOUND surface is now state-only. `inputBuf`,
// `getSyncedCorrectionInputBuffer` and the `SyncedRemoteInputBufferType` typedef
// the concept used to demand for them are gone — the correction-input channel is
// retired, so there is nothing for an authority owner to publish an input into.
struct MockAuthorityOwner
{
    MockStateSyncBuffer stateBuf;
    // Stage 1 (Task 9): per-slot (capture_tick, input) inbound callback.
    std::function<void(uint32, const simulatableBrawler::PlayerInput&)> onRemoteMoveReceived;

    MockStateSyncBuffer& getSyncedCorrectionStateBuffer() { return stateBuf; }

    void setOnRemoteMoveReceivedCallback(
        std::function<void(uint32, const simulatableBrawler::PlayerInput&)> fn)
    { onRemoteMoveReceived = std::move(fn); }

    void clearOnRemoteMoveReceivedCallback()
    { onRemoteMoveReceived = nullptr; }
};

static_assert(
    AuthoritySyncedBufferOwnerConcept<MockAuthorityOwner,
        simulatableBrawler::State,
        simulatableBrawler::PlayerInput>,
    "MockAuthorityOwner must satisfy AuthoritySyncedBufferOwnerConcept");

// ---------------------------------------------------------------------------
// SimulatableOwnerTraits specialization for SimulatableBrawler, using mock
// owners for isolated unit tests in the DAttack module.
// NOTE: This specialization is only defined within this translation unit.
// The real specialization (USimmableUpdateComponent) lives in SimmableUpdateComponent.h.
// ---------------------------------------------------------------------------
template <>
struct SimulatableOwnerTraits<SimulatableBrawler>
{
    using PredictionOwnerType = MockPredictionOwner;
    using AuthorityOwnerType  = MockAuthorityOwner;
};

// ---------------------------------------------------------------------------
// sizeof static_assert — AuthorityWriter and LocalInputSender must be pointer-sized.
// ---------------------------------------------------------------------------
static_assert(sizeof(AuthorityWriter<SimulatableBrawler>) == sizeof(void*),
    "AuthorityWriter<SimulatableBrawler> must be exactly sizeof(void*)");
static_assert(sizeof(LocalInputSender<SimulatableBrawler>) == sizeof(void*),
    "LocalInputSender<SimulatableBrawler> must be exactly sizeof(void*)");
static_assert(std::is_trivially_copyable_v<AuthorityWriter<SimulatableBrawler>>,
    "AuthorityWriter<SimulatableBrawler> must be trivially copyable");
static_assert(std::is_trivially_copyable_v<LocalInputSender<SimulatableBrawler>>,
    "LocalInputSender<SimulatableBrawler> must be trivially copyable");

// ---------------------------------------------------------------------------
// Concept check
// ---------------------------------------------------------------------------
static_assert(
    SimulationNetSyncConcept<
        SimulationNetSync<SimulatableBrawler>,
        SimulatableBrawler>,
    "SimulationNetSync<SimulatableBrawler> must satisfy SimulationNetSyncConcept");

// ---------------------------------------------------------------------------
// Helper — build a SimulatableBrawler with the new single-arg ctor.
// ---------------------------------------------------------------------------
static SimulatableBrawler makeNetSyncTestCharacter()
{
    simulatableBrawler::StaticData staticData;
    return SimulatableBrawler(staticData);
}

// ---------------------------------------------------------------------------
// Test: register and unregister a client simulatable without crash.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationNetSync.RegisterUnregisterClient", "[DAttack][SimulationNetSync]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(42u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(42u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwner;

    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    auto inputProvider = [zeroInput](const SimulationTimeStep&, const BrawlerLocalInputCache&) { return zeroInput; };

    netSync.registerPredictionOwner<SimulatableBrawler>(42u, predictionOwner, std::move(inputProvider), inputResolution);

    // [T8] The correction-INPUT binding assertions that sat beside these are gone
    // with the channel; the state binding is what registration/unregistration still
    // has to get right for a LOCAL character. (A remote character additionally binds
    // the relay ring — covered by the [InputRelay] registration cases.)
    REQUIRE(predictionOwner.onCorrectionStateReceived != nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(42u, &predictionOwner, inputResolution, nullptr);

    REQUIRE(predictionOwner.onCorrectionStateReceived == nullptr);
}

// ---------------------------------------------------------------------------
// Test: register server simulatable and verify authority owner callback is set.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationNetSync.RegisterServer", "[DAttack][SimulationNetSync]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(1u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(1u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(1u, predictionOwner, nullptr, inputResolution);
    netSync.registerAuthorityOwner<SimulatableBrawler>(1u, authorityOwner, inputResolution);

    REQUIRE(authorityOwner.onRemoteMoveReceived != nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(1u, &predictionOwner, inputResolution, &authorityOwner);

    REQUIRE(authorityOwner.onRemoteMoveReceived == nullptr);
}

// ---------------------------------------------------------------------------
// Regression: sendLocalInputToAuthorityAll must be a no-op when the prediction
// owner was registered without an input provider (server side, and client-side
// remote characters). Previously m_localInputSenders was populated unconditionally
// while m_pendingInputQueues was gated on inputProvider, so the iteration would
// throw std::out_of_range on the server and crash onPostSimulationGameThread.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationNetSync.SendLocalInputNoProviderIsNoOp", "[DAttack][SimulationNetSync]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(1u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(1u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(1u, predictionOwner, nullptr, inputResolution);
    netSync.registerAuthorityOwner<SimulatableBrawler>(1u, authorityOwner, inputResolution);

    // Must not throw. (Stage 1, Task 9: now takes currentTick + redundancyDepth.)
    netSync.sendLocalInputToAuthorityAll(0u, 5u);

    REQUIRE(true);

    netSync.unregisterSimulatable<SimulatableBrawler>(1u, &predictionOwner, inputResolution, &authorityOwner);
}


// ===========================================================================
// [T9 parts 3+4] CLIENT LAYER-1 INPUT DELAY — end-to-end through the REAL
// SimulationInputResolution::collectInputAll provider branch.
// (og-netcode-v2-arch-latency; D5.2 client half.)
//
// WHY HERE and not in og-simulation-tests. The container and the offset rule are
// unit-tested there (Network/LocalInputCacheTest.cpp, MPL-2.0). What that
// suite cannot reach is the PRODUCTION BRANCH: collectInputAll is variadic over a
// simulatable pack and needs SimulatableOwnerTraits bound to concrete owners,
// which only a suite linking a real simulatable can supply. These cases drive the
// shipped code path, not a re-derivation of it.
//
// WHAT THEY PIN. The provider branch now produces TWO different values from one
// capture, and the split is the entire task:
//   * the integrator + correction cache get the capture from `tick - delay`;
//   * the outbound RPC queue gets the ORIGINAL capture at the CURRENT tick.
// If the outbound half ever started carrying the delayed value, the server would
// apply its own ServerInputDelayQueue delay ON TOP and the two ends would diverge
// by exactly `delay` ticks — a silent, permanent mispredict that no existing test
// would have caught. `OutboundQueueCarriesUndelayedCapture` is that guard.
//
// WHAT THEY DO NOT COVER. The game-thread -> physics-thread publication of the
// delay (OnRep_ConnectionTier -> the std::atomic) is UE-side and unlinkable here;
// these cases set the delay directly. As with the server-side integration suite,
// a green run here fixes the OFFSET, not the thread timing.
// ===========================================================================

namespace
{
    // Tag each tick's capture so the resolved input identifies which tick it was
    // captured at. aimDirection is a public field on the radial sub-input and is
    // carried through the composite untouched by collectInputAll.
    simulatableBrawler::PlayerInput taggedCapture(float tickTag)
    {
        simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
        input.edit<dAttackRadialSimulation::PlayerInput>().aimDirection.x = tickTag;
        return input;
    }

    float captureTagOf(const simulatableBrawler::PlayerInput& input)
    {
        return input.get<dAttackRadialSimulation::PlayerInput>().aimDirection.x;
    }

    // The tag the game's zero input carries — getZeroPlayerInput builds a
    // (0,0,1) aim, so an untagged neutral reads back as 0.f. Named rather than
    // spelled 0.f at the call sites so the assertions say what they mean.
    constexpr float kNeutralTag = 0.f;

    // THE tag alone CANNOT tell the injected zero input from a value-initialised
    // PlayerInput — both carry aimDirection.x == 0. That is precisely the blind
    // spot the T9 part-2 notes recorded (a probe that "passed" because the case
    // could not see the defect it existed to catch), so the neutral assertions
    // check the WHOLE aim vector: getZeroPlayerInput builds (0,0,1), a
    // value-initialised input would carry (0,0,0) into normalisation.
    bool isGameZeroInput(const simulatableBrawler::PlayerInput& input)
    {
        const glm::vec3 aim = input.get<dAttackRadialSimulation::PlayerInput>().aimDirection;
        const glm::vec3 expected =
            simulatableBrawler::getZeroPlayerInput()
                .get<dAttackRadialSimulation::PlayerInput>().aimDirection;
        return aim == expected;
    }
}

// THE divergence guard. See the block comment above.
TEST_CASE("DAttack.SimulationNetSync.OutboundQueueCarriesUndelayedCapture",
          "[DAttack][SimulationNetSync][ClientInputDelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(8u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(8u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwner;

    inputResolution.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(8u, predictionOwner,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        }, inputResolution);

    constexpr int32 kDelay = 3;
    inputResolution.setClientEffectiveInputDelayTicks(kDelay);

    constexpr unsigned int kLastTick = 6u;
    for (unsigned int tick = 1u; tick <= kLastTick; ++tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = inputResolution.collectInputAll(step);
        reconciliation.allocateFrontierSlotsAll(step); // [item 94] opens the frontier pair
        reconciliation.postPredictionAll(step);
    }

    // Depth 5 so the whole run is offered to the send path in one call.
    netSync.sendLocalInputToAuthorityAll(kLastTick, 5u);

    REQUIRE_FALSE(predictionOwner.sentSlots.empty());
    for (const auto& slot : predictionOwner.sentSlots)
    {
        // Slot tick T must carry the capture taken AT T. If the delayed value had
        // been enqueued instead, this would read T - kDelay and the server would
        // delay it again.
        REQUIRE(captureTagOf(slot.input) == Catch::Approx(static_cast<float>(slot.tick)));
        REQUIRE_FALSE(captureTagOf(slot.input)
            == Catch::Approx(static_cast<float>(slot.tick) - kDelay));
    }
}

namespace
{
    // Drive one authority tick through the real branch and hand back the input it
    // resolved, so each case asserts on the SAME tick it inspects the record for.
    // [item 87] Re-targeted onto the resolution peer — collectInputAll moved
    // off SimulationNetSync at the promotion (named prepareSimulationStep
    // between item 90 and item 94). [item 94] No allocateFrontierSlotsAll
    // call here — the authority role never opens a frontier pair.
    simulatableBrawler::PlayerInput authorityTick(
        SimulationInputResolution<SimulatableBrawler>& inputResolution, unsigned int id, unsigned int tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = inputResolution.collectInputAll(step);
        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        REQUIRE(map.find(id) != map.end());
        return map.at(id);
    }
}


// ===========================================================================
// [og-netcode-v2-input-relay T4] THE STATE CHANNEL CARRIES THE JOIN KEY —
// through the REAL sendCorrectionAll (server half) and the REAL
// injectCorrectionState -> correction-cache slot (client half).
//
// WHY THIS MATTERS. After T3 the relayed input and the correction state are two
// independently-cadenced channels. The state message is the ONLY thing that can
// say "the state I am correcting you to at tick T was produced by the input
// captured at tick X" — without it a resim cannot correlate a correction with
// the relayed input that produced it, and would have to guess from a delay that
// is only the INTENDED schedule (RelayDelaySpectrumDesign.md §5.3: where an
// actual reference exists, it always wins).
//
// The wire framing itself (layout, sentinel round-trip, the version fence at 2)
// is pinned in the core suite against the engine-agnostic codec
// (og-simulation-tests, WireFormat/CorrectionStateBufferCodecTest.cpp), and the
// per-slot storage semantics in CorrectionCache/AppliedCaptureTickSlotTest.cpp.
// These cases pin the two PRODUCTION call sites that connect them, which need
// SimulatableOwnerTraits bound to concrete owners and therefore live here — the
// same constraint the T2 block above documents.
//
// SCOPE: state side only. The old correction-INPUT write is deliberately left in
// place (expand/contract, fable finding B3) and is asserted to still happen, so
// a reviewer can see the dual-write survived intact until T8 removes it.
// ===========================================================================

TEST_CASE("DAttack.SimulationNetSync.CorrectionStateCarriesAppliedCaptureTick",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(30u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(30u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(30u, predictionOwner, nullptr, inputResolution);
    netSync.registerAuthorityOwner<SimulatableBrawler>(30u, authorityOwner, inputResolution);

    // Before any authority tick there is no applied input at all — the send must
    // publish the sentinel rather than an invented reference.
    netSync.sendCorrectionAll(SimulationTimeStep(10u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastTick == 10u);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);

    // A real capture, applied on a DIFFERENT authority tick than it was taken —
    // so a send that published its own tick, or the delay, would be caught.
    authorityOwner.onRemoteMoveReceived(51u, taggedCapture(51.f));
    authorityTick(inputResolution, 30u, 11u);

    netSync.sendCorrectionAll(SimulationTimeStep(11u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastTick == 11u);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == 51u);
    REQUIRE_FALSE(authorityOwner.stateBuf.lastAppliedCaptureTick == 11u);

    // UNDERRUN on the next tick: the authority substituted an input, so the state
    // must say so explicitly instead of repeating the now-stale 51 — a stale
    // reference would send the client looking up a relayed input the authority
    // never applied.
    authorityTick(inputResolution, 30u, 12u);
    netSync.sendCorrectionAll(SimulationTimeStep(12u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);
    REQUIRE_FALSE(authorityOwner.stateBuf.lastAppliedCaptureTick == 51u);

    // ...and it recovers on the next real arrival — per-tick classification, not
    // a latch.
    authorityOwner.onRemoteMoveReceived(52u, taggedCapture(52.f));
    authorityTick(inputResolution, 30u, 13u);
    netSync.sendCorrectionAll(SimulationTimeStep(13u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == 52u);

    // [T8] The EXPAND/CONTRACT FENCE assertion (B3) stood here: `inputBuf.lastTick
    // == 13u`, pinning that T4 left the old correction-INPUT write firing on every
    // send. T8 IS the contract half, so the fence has been DISCHARGED rather than
    // broken — there is no longer a second write for a fence to protect. Everything
    // above (the ref's per-tick classification, which is what this case is about)
    // is unchanged and still asserted.

    netSync.unregisterSimulatable<SimulatableBrawler>(30u, &predictionOwner, inputResolution, &authorityOwner);
}

TEST_CASE("DAttack.SimulationNetSync.CorrectionStateRefIsPerCharacter",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    // Two authority-owned characters whose queues run at different rates — the
    // reference is a property of the CHARACTER's applied input, so one being in
    // underrun must not contaminate the other.
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(31u, makeNetSyncTestCharacter());
    storage.add<SimulatableBrawler>(32u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(31u);
    reconciliation.createCacheFor<SimulatableBrawler>(32u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwnerA;
    MockPredictionOwner predictionOwnerB;
    MockAuthorityOwner  authorityOwnerA;
    MockAuthorityOwner  authorityOwnerB;

    netSync.registerPredictionOwner<SimulatableBrawler>(31u, predictionOwnerA, nullptr, inputResolution);
    netSync.registerAuthorityOwner<SimulatableBrawler>(31u, authorityOwnerA, inputResolution);
    netSync.registerPredictionOwner<SimulatableBrawler>(32u, predictionOwnerB, nullptr, inputResolution);
    netSync.registerAuthorityOwner<SimulatableBrawler>(32u, authorityOwnerB, inputResolution);

    // Only A has input available.
    authorityOwnerA.onRemoteMoveReceived(80u, taggedCapture(80.f));

    SimulationTimeStep step(20u, false, StepKind::Normal);
    // [item 94] No allocateFrontierSlotsAll call — both ids are authority-
    // owned (no cache), and the authority role never opens a frontier pair.
    inputResolution.collectInputAll(step);
    netSync.sendCorrectionAll(step, kEveryFrameRotationK);

    REQUIRE(authorityOwnerA.stateBuf.lastAppliedCaptureTick == 80u);
    REQUIRE(authorityOwnerB.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);
    REQUIRE(authorityOwnerA.stateBuf.lastTick == authorityOwnerB.stateBuf.lastTick);

    netSync.unregisterSimulatable<SimulatableBrawler>(31u, &predictionOwnerA, inputResolution, &authorityOwnerA);
    netSync.unregisterSimulatable<SimulatableBrawler>(32u, &predictionOwnerB, inputResolution, &authorityOwnerB);
}

// THE CLIENT HALF (design D3): the received reference is stashed in the
// CORRECTION-CACHE SLOT, parallel to the state it corrects — one per character
// PER TICK, because T6 reads it back for every resim tick.
TEST_CASE("DAttack.SimulationNetSync.InjectCorrectionStateStashesRefPerTick",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(33u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(33u);

    SimulationInputResolution<SimulatableBrawler> inputResolution(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation, inputResolution);
    MockPredictionOwner predictionOwner;

    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    netSync.registerPredictionOwner<SimulatableBrawler>(
        33u, predictionOwner, [zeroInput](const SimulationTimeStep&, const BrawlerLocalInputCache&) { return zeroInput; }, inputResolution);
    REQUIRE(predictionOwner.onCorrectionStateReceived != nullptr);

    // Predict three ticks so the cache has slots for them.
    //
    // [og-netcode-v2-input-relay item 84, re-pointed item 94] Completes the
    // frontier pair (reconciliation.allocateFrontierSlotsAll then
    // reconciliation.postPredictionAll) after each collectInputAll, same as
    // production does within one manager tick — otherwise the second
    // iteration's allocation trips the frontier-pair detector (the first
    // iteration's allocation was never completed).
    for (uint32 tick = 40u; tick <= 42u; ++tick)
    {
        const SimulationTimeStep step(tick, false, StepKind::Normal);
        inputResolution.collectInputAll(step);
        reconciliation.allocateFrontierSlotsAll(step); // [item 94] opens the frontier pair
        reconciliation.postPredictionAll(step);
    }

    // Corrections arrive for all three, each naming a different capture — and one
    // of them naming none (the authority substituted an input at that tick).
    const simulatableBrawler::State correctedState{};
    MockStateSyncBuffer wire;

    wire.write(correctedState, 40u, 31u);
    predictionOwner.onCorrectionStateReceived(wire);

    wire.write(correctedState, 41u, kNoInputCaptureTick);
    predictionOwner.onCorrectionStateReceived(wire);

    wire.write(correctedState, 42u, 33u);
    predictionOwner.onCorrectionStateReceived(wire);

    // Each tick answers with ITS OWN reference. A single-scalar stash would give
    // 33 for all three; this is the assertion that fails under it.
    REQUIRE(reconciliation.getAppliedCaptureTick<SimulatableBrawler>(33u, 40u) == 31u);
    REQUIRE(reconciliation.getAppliedCaptureTick<SimulatableBrawler>(33u, 41u) == kNoInputCaptureTick);
    REQUIRE(reconciliation.getAppliedCaptureTick<SimulatableBrawler>(33u, 42u) == 33u);

    // A tick outside the cache window cannot answer at all — deliberately
    // distinguishable from "the authority named no capture" (the sentinel).
    REQUIRE_FALSE(reconciliation.getAppliedCaptureTick<SimulatableBrawler>(33u, 4000u).has_value());
    // ...and an id with no cache (the authority's own role) answers the same way
    // rather than throwing.
    REQUIRE_FALSE(reconciliation.getAppliedCaptureTick<SimulatableBrawler>(999u, 40u).has_value());

    netSync.unregisterSimulatable<SimulatableBrawler>(33u, &predictionOwner, inputResolution);
}

namespace
{
    // Write one (captureTick, dA, input) entry into a mock ring through the REAL
    // codec, at the shipped depth of 1.
    void relayWrite(MockRelayedInputRing& ring,
                    uint32 captureTick,
                    std::uint8_t dA,
                    float tag,
                    std::int32_t depth = 1)
    {
        relayedInputRing::writeLatest<simulatableBrawler::PlayerInput>(
            ring, captureTick, dA, taggedCapture(tag), depth);
    }
}

namespace
{
    // storage + reconciliation + netsync, with the composition root's neutral
    // injection already done — so every delay line and relay store this rig later
    // creates is seeded with the GAME's zero rather than a value-initialised input.
    struct ResimRig
    {
        SimulationObjectStorage<SimulatableBrawler>   storage;
        SimulationReconciliation<SimulatableBrawler>  reconciliation{ storage };
        SimulationInputResolution<SimulatableBrawler>         inputResolution{ storage, reconciliation };
        SimulationNetSync<SimulatableBrawler>         netSync{ storage, reconciliation, inputResolution };

        ResimRig()
        {
            inputResolution.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
        }
    };

    // A locally-controlled character: provider PRESENT, so it gets a delay line
    // and no relay store. Its provider tags each capture with its capture tick,
    // which is what makes "which capture did this resolve to" readable.
    void addLocalCharacter(ResimRig& rig, unsigned int id, MockPredictionOwner& owner)
    {
        rig.storage.add<SimulatableBrawler>(id, makeNetSyncTestCharacter());
        rig.reconciliation.createCacheFor<SimulatableBrawler>(id);
        rig.netSync.registerPredictionOwner<SimulatableBrawler>(id, owner,
            [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
                return taggedCapture(static_cast<float>(step.getTick()));
            }, rig.inputResolution);
    }

    // A remote proxy: provider ABSENT, so it gets a relay store and no delay line.
    void addRemoteCharacter(ResimRig& rig, unsigned int id, MockPredictionOwner& owner)
    {
        rig.storage.add<SimulatableBrawler>(id, makeNetSyncTestCharacter());
        rig.reconciliation.createCacheFor<SimulatableBrawler>(id);
        rig.netSync.registerPredictionOwner<SimulatableBrawler>(id, owner, nullptr, rig.inputResolution);
    }

    // One prediction tick through the REAL collectInputAll + allocateFrontierSlotsAll
    // (item 94) — which is what creates the cache slot a correction can later
    // land in, and what fills the delay line.
    void predictTick(ResimRig& rig, unsigned int tick)
    {
        const SimulationTimeStep step(tick, false, StepKind::Normal);
        rig.inputResolution.collectInputAll(step);
        rig.reconciliation.allocateFrontierSlotsAll(step); // [item 94] opens the frontier pair
        rig.reconciliation.postPredictionAll(step);
    }

    // An authoritative correction for `tick` carrying `ref`, delivered through the
    // real OnRep-bound callback into the real injectCorrectionState.
    void landCorrection(MockPredictionOwner& owner, unsigned int tick, uint32 ref)
    {
        REQUIRE(owner.onCorrectionStateReceived != nullptr);
        MockStateSyncBuffer wire;
        wire.write(simulatableBrawler::State{}, tick, ref);
        owner.onCorrectionStateReceived(wire);
    }

    // [og-netcode-v2-input-relay T8] `landCorrectionInput` — the helper that drove
    // the OLD server->client correction-INPUT channel through the real OnRep-bound
    // callback — is gone with that channel. Its only user was the AC (c)
    // equivalence case, retired below for the same reason.

    // The resolved resim input for one character, or nullopt when the resolution
    // emitted NO entry at all — which is the NoSlot row, and is not the same thing
    // as resolving to a neutral.
    // [item 87] Re-targeted onto the resolution peer — collectResimInputAll
    // moved off SimulationNetSync at the promotion.
    std::optional<simulatableBrawler::PlayerInput> resimInputFor(
        SimulationInputResolution<SimulatableBrawler>& inputResolution, unsigned int tick, unsigned int id)
    {
        auto inputs = inputResolution.collectResimInputAll(tick);
        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        const auto it = map.find(id);
        return it == map.end()
            ? std::nullopt
            : std::optional<simulatableBrawler::PlayerInput>(it->second);
    }
}


// ===========================================================================
// [og-netcode-v2-input-relay T19] THE RELAY PROBES — WIRING, not arithmetic.
//
// The probe TYPES are unit-tested in og-simulation-tests
// (Network/RelayReadProbeTest.cpp, MPL-2.0): the four outcomes, the capture-tick
// gap, the rung-0 exclusion, the percentile, the window mechanics. Every one of
// those cases feeds the probes BY HAND.
//
// WHAT THAT CANNOT SHOW, and what these two cases exist for: that the SHIPPED code
// paths actually feed them. A probe wired to nothing passes every unit test it has
// and reports all-zero counters forever — and because this whole task is
// instrumentation, a silently-unwired probe is not a cosmetic defect, it is the
// task failing while looking like it succeeded. T9 would then read those zeros as
// evidence.
//
// Both call sites need SimulatableOwnerTraits bound to concrete owners
// (collectInputAll / collectResimInputAll are variadic over a simulatable pack),
// and the arrival path needs the real ingest behind a real callback — i.e. this
// suite, exactly as the [ProxyScheduledRead] and [ResimResolution] blocks above.
// ===========================================================================

TEST_CASE("DAttack.SimulationNetSync.RelayProbeMeasuresArrivalCadenceInCaptureTicks",
          "[DAttack][SimulationNetSync][RelayProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 91u, remote);

    const RelayArrivalProbe& arrival = rig.netSync.getDiagnostics().relayArrivalProbe();

    // THE BIND-TIME POPULATE IS NOT AN ARRIVAL. registerPredictionOwner has already
    // run it (against an as-yet-unwritten ring), and it is deliberately not fed to
    // this probe: no replication event occurred, and on the AUTHORITY that same
    // call runs on a role with no relay traffic at all.
    REQUIRE(arrival.sampleCount() == 0u);
    REQUIRE(arrival.noAdvance() == 0u);

    // Three real OnReps, delivered BACK TO BACK — so the local spacing is 1 — each
    // carrying a newest capture tick five ahead of the last. Only one of those two
    // numbers can be the reported gap, and it must be the capture-tick one:
    // `depth` is denominated in capture ticks, so a local-tick p99 could not
    // legitimately be compared against it.
    for (uint32 capture = 100u; capture <= 110u; capture += 5u)
    {
        relayWrite(remote.relayedInputRing, capture, 3u, static_cast<float>(capture));
        remote.replicateRelayRing();
    }

    REQUIRE(arrival.sampleCount() == 2u);       // the first arrival seeds, not samples
    REQUIRE(arrival.maxGap() == 5u);
    REQUIRE_FALSE(arrival.maxGap() == 1u);      // the local-tick answer
    REQUIRE(arrival.noAdvance() == 0u);

    // A RE-REPLICATION CARRYING NOTHING NEW is not a cadence sample. Counting it as
    // a gap of 0 would drag the percentile down and understate the depth the rule
    // demands.
    remote.replicateRelayRing();
    REQUIRE(arrival.sampleCount() == 2u);
    REQUIRE(arrival.noAdvance() == 1u);
    REQUIRE(arrival.maxGap() == 5u);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(91u, &remote, rig.inputResolution);
}


// ===========================================================================
// ⭐ [og-netcode-v2-input-relay T34 loss-counter fix] THE LOSS COUNTER'S WIRING.
//
// WHY HERE and not in og-simulation-tests. The arithmetic is unit-tested there
// ([Network][RelayProbe], MPL-2.0) on hand-fed numbers. What that suite structurally
// CANNOT show is the thing that was actually broken: whether the SHIPPED callback
// hands the probe a real delivered-count or a constant. `lostCaptureTicksX1000`
// read ~120 per mille on `runs/t34_run1_2char_2026-08-09_1938` — a run whose flush
// was working perfectly — because the probe was charging `gap - 1` while one arrival
// carried a whole burst. A unit test of the formula passes either way; only the real
// ring -> real codec -> real ingest -> real probe chain can fail on the wiring.
//
// This is the same argument the T20 miss-class wiring block below makes, and it is
// the second time on this initiative that an instrument's CALL SITE, not its
// arithmetic, was the defect.
// ===========================================================================

TEST_CASE("DAttack.SimulationNetSync.RelayArrivalLossCounterChargesTheBurstNotTheWatermark",
          "[DAttack][SimulationNetSync][RelayProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 92u, remote);

    const RelayArrivalProbe& arrival = rig.netSync.getDiagnostics().relayArrivalProbe();

    // Seed the watermark at capture 300 with a single-entry ring — the shape the
    // retired replace-latest write path produced on every replication.
    relayWrite(remote.relayedInputRing, 300u, 3u, 300.f);
    remote.replicateRelayRing();
    REQUIRE(arrival.sampleCount() == 0u);       // the first arrival seeds, not samples

    // ⭐ A FLUSH ROUND. Captures 301, 302 and 303 are published in ONE replication,
    // which is exactly what `ASimulationInputRelay::PreReplication` does. The
    // watermark advances by 3 and NOTHING WAS LOST.
    for (uint32 capture = 301u; capture <= 303u; ++capture)
        relayWrite(remote.relayedInputRing, capture, 3u, static_cast<float>(capture), /*depth=*/3);
    remote.replicateRelayRing();

    REQUIRE(arrival.sampleCount() == 1u);

    RelayArrivalWindowSummary peek;
    arrival.peekSummary(peek);

    // ⛔ THE ASSERTION THE OLD INSTRUMENT FAILED. It reported `gap - 1` = 2 lost
    // here, i.e. 666 per mille, on a round that delivered every capture tick the
    // sender produced.
    REQUIRE(peek.expectedCaptureTicks == 3u);
    REQUIRE(peek.deliveredCaptureTicks == 3u);
    REQUIRE(peek.lostCaptureTicks == 0u);
    REQUIRE(peek.lostCaptureTicksX1000 == 0u);

    // ⭐ AND A ROUND THAT REALLY DID LOSE ONE, through the same chain. Capture 304
    // never arrives; the round carries 305 and 306, and the ring ALSO still carries
    // 303 (a flush republishes what has not yet been evicted). 303 is already
    // resident in the store, so it is re-delivery and not new coverage — which is
    // why the count comes from `newCaptureTicksIngested` and not `entriesIngested`.
    relayWrite(remote.relayedInputRing, 305u, 3u, 305.f, /*depth=*/3);
    relayWrite(remote.relayedInputRing, 306u, 3u, 306.f, /*depth=*/3);
    remote.replicateRelayRing();

    REQUIRE(arrival.sampleCount() == 2u);
    arrival.peekSummary(peek);

    REQUIRE(peek.expectedCaptureTicks == 6u);       // 3 + 3
    REQUIRE(peek.deliveredCaptureTicks == 5u);      // 301,302,303 + 305,306 — NOT 303 twice
    REQUIRE(peek.lostCaptureTicks == 1u);           // capture 304, and only that
    REQUIRE(peek.lostCaptureTicksX1000 == 166u);

    // The self-check the `[RelayProbe.Arrival]` line carries: every expected tick is
    // either delivered or lost.
    REQUIRE(peek.lostCaptureTicks + peek.deliveredCaptureTicks
            == peek.expectedCaptureTicks);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(92u, &remote, rig.inputResolution);
}


// ===========================================================================
// [og-netcode-v2-input-relay T24] THE CORRECTION-VERDICT PROBE — WIRING, not
// arithmetic.
//
// The probe TYPE is unit-tested in og-simulation-tests
// (Network/CorrectionVerdictProbeTest.cpp, MPL-2.0) and the cache's verdict
// REPORT in CorrectionCache/CorrectionVerdictReportTest.cpp: the class split, the
// combined-count window, the rounding, the discard-is-not-a-verdict rule. All of
// those feed the probe by hand.
//
// WHAT THAT CANNOT SHOW, and what this block exists for — three things, each of
// which fails silently:
//
//   1. THAT THE SHIPPED CORRECTION CALLBACK FEEDS THE PROBE AT ALL. A probe wired
//      to nothing passes every unit test it has and reports all-zero counters
//      forever. This whole task is instrumentation, so an unwired probe is not a
//      cosmetic defect — it is the task failing while looking like it succeeded,
//      and T23 would then read the zeros as evidence that corrections are rare.
//      (They ARE rare. That is exactly why an all-zero bug is invisible here.)
//   2. THAT THE CLASS COMES FROM PROVIDER-PRESENCE, the same test the rest of
//      SimulationNetSync forks on. Getting this backwards — or deriving it from a
//      second, independent notion of "remote" — would put the two populations in
//      each other's counters, and the summary would look perfectly healthy while
//      describing the wrong characters. Only a suite with SimulatableOwnerTraits
//      bound to concrete owners can register one of each and tell them apart.
//   3. THAT THE VERDICT BIT SURVIVES THE WHOLE JOURNEY — cache -> reconciliation
//      -> netsync -> probe. A wiring that always passed `true` would count every
//      correction as an agreement, report a 0-per-mille disagreement rate on every
//      window, and read as the best possible result.
// ===========================================================================

namespace
{
    // Force this character's PREDICTED state to differ from the value-initialised
    // state a correction carries, so the next correction DISAGREES.
    //
    // `attackTimer` is used because it is in dAttackGuardSimulation::State's
    // SerializableFields, and isSimilarTo folds over exactly those — `testTick`,
    // the other obvious candidate, is NOT serialized and is therefore invisible to
    // the comparison. Must run BEFORE the predictTick that commits the slot.
    void divergeState(ResimRig& rig, unsigned int id, float attackTimer)
    {
        rig.storage.get<SimulatableBrawler>(id)
            .editAllState().editState()
            .edit<dAttackGuardSimulation::State>().attackTimer = attackTimer;
    }
}

TEST_CASE("DAttack.SimulationNetSync.CorrectionVerdictProbeClassifiesByProviderPresence",
          "[DAttack][SimulationNetSync][DivergenceProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner local;
    MockPredictionOwner remote;
    addLocalCharacter(rig, 95u, local);      // provider PRESENT
    addRemoteCharacter(rig, 96u, remote);    // provider ABSENT

    const CorrectionVerdictProbe& probe = rig.netSync.getDiagnostics().correctionVerdictProbe();

    // Registration alone feeds nothing — the probe counts CORRECTIONS, and none
    // has arrived. (The bind-time relay populate is not a correction either.)
    REQUIRE(probe.sampleCount() == 0u);

    for (unsigned int tick = 1u; tick <= 4u; ++tick)
        predictTick(rig, tick);

    // Both characters' slots hold the value-initialised state, so an authoritative
    // correction carrying the same AGREES.
    //
    // ASYMMETRIC ON PURPOSE — TWO local corrections against ONE remote. A
    // symmetric one-each drive would count identically under a SWAPPED class test,
    // and this case would then certify the exact defect it exists to catch. The
    // asymmetry is what makes the swap observable here rather than only in the
    // divergence case below.
    landCorrection(local,  2u, 1u);
    landCorrection(local,  3u, 2u);
    landCorrection(remote, 2u, 1u);

    // THE CLASS SPLIT, AT THE SHIPPED CALL SITE. Filed under the class each
    // character's provider-presence implies — not both under one, and not swapped.
    REQUIRE(probe.correctionsFor(PredictedCharacterClass::LocallyPredicted) == 2u);
    REQUIRE(probe.correctionsFor(PredictedCharacterClass::RemoteProxy)      == 1u);
    REQUIRE(probe.sampleCount() == 3u);

    // ...and both agreed, so the disagreement counters are still empty. This is
    // the assertion a wiring that hard-coded `false` would fail.
    REQUIRE(probe.disagreementsFor(PredictedCharacterClass::LocallyPredicted) == 0u);
    REQUIRE(probe.disagreementsFor(PredictedCharacterClass::RemoteProxy)      == 0u);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(95u, &local, rig.inputResolution);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(96u, &remote, rig.inputResolution);
}

TEST_CASE("DAttack.SimulationNetSync.CorrectionVerdictProbeCarriesTheRealDivergenceVerdict",
          "[DAttack][SimulationNetSync][DivergenceProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner local;
    MockPredictionOwner remote;
    addLocalCharacter(rig, 97u, local);
    addRemoteCharacter(rig, 98u, remote);

    const CorrectionVerdictProbe& probe = rig.netSync.getDiagnostics().correctionVerdictProbe();

    predictTick(rig, 1u);

    // THE REMOTE PROXY MISPREDICTS and the local character does not. Both slots
    // for tick 2 are committed by the same predictTick, but only the remote one
    // carries a state the authority will contradict.
    divergeState(rig, 98u, 25.f);
    predictTick(rig, 2u);

    landCorrection(local,  2u, 1u);
    landCorrection(remote, 2u, 1u);

    // THE VERDICT SURVIVED cache -> reconciliation -> netsync -> probe, and it
    // landed in the RIGHT class. A wiring that passed a constant, or that swapped
    // the classes, produces a different pair here.
    REQUIRE(probe.correctionsFor(PredictedCharacterClass::RemoteProxy)        == 1u);
    REQUIRE(probe.disagreementsFor(PredictedCharacterClass::RemoteProxy)      == 1u);
    REQUIRE(probe.correctionsFor(PredictedCharacterClass::LocallyPredicted)   == 1u);
    REQUIRE(probe.disagreementsFor(PredictedCharacterClass::LocallyPredicted) == 0u);

    // AND THE DISAGREEMENT IS THE SAME EVENT THE SIMULATION ACTED ON. The cache
    // overwrites the slot only when isSimilarTo said no, so restoring tick 2 must
    // now hand back the AUTHORITY's attackTimer (0), not the 25 this client
    // predicted. If the reported verdict could ever drift from the overwrite
    // decision, this is the assertion that would notice — the telemetry would
    // otherwise be free to describe a divergence the simulation never acted on.
    rig.reconciliation.prepareResimAll(2u);
    REQUIRE(rig.storage.get<SimulatableBrawler>(98u)
                .getAllState().getState()
                .get<dAttackGuardSimulation::State>().attackTimer == 0.f);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(97u, &local, rig.inputResolution);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(98u, &remote, rig.inputResolution);
}

TEST_CASE("DAttack.SimulationNetSync.CorrectionVerdictProbeIgnoresDiscardedCorrections",
          "[DAttack][SimulationNetSync][DivergenceProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 99u, remote);

    const CorrectionVerdictProbe& probe = rig.netSync.getDiagnostics().correctionVerdictProbe();

    for (unsigned int tick = 1u; tick <= 3u; ++tick)
        predictTick(rig, tick);

    // A correction for a tick with no slot. This is ROUTINE — the cache's
    // isAnomalousMiss block names three benign causes — and no comparison happens,
    // so it must not enter the denominator. Counting discards as agreements would
    // drive the measured rate toward zero for reasons that have nothing to do with
    // prediction quality, i.e. it would manufacture the exact result T23 is
    // looking for.
    landCorrection(remote, 9000u, 8993u);
    REQUIRE(probe.sampleCount() == 0u);
    REQUIRE(probe.correctionsFor(PredictedCharacterClass::RemoteProxy) == 0u);

    // A correction that DOES land is counted, so the zero above is a gate and not
    // a dead call site.
    landCorrection(remote, 3u, 1u);
    REQUIRE(probe.sampleCount() == 1u);
    REQUIRE(probe.correctionsFor(PredictedCharacterClass::RemoteProxy) == 1u);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(99u, &remote, rig.inputResolution);
}


// ===========================================================================
// [og-netcode-v2-input-relay / item 42] THE FRONTIER-LANDING SPLIT (I2) — THE
// INTEGRATION HALF.
//
// The three-way classification, the class split, the per-mille arithmetic and the
// window drive are swept as a unit in
// og-simulation-tests/Reconciliation/ResimGateProbeTest.cpp (MPL-2.0), where no
// cache, owner or simulatable is needed and every input is supplied by hand.
//
// WHAT THAT SUITE STRUCTURALLY CANNOT SHOW — three things, each of which fails
// silently and each of which would leave item 42 reporting confident zeros:
//
//   1. THAT THE SHIPPED CORRECTION CALLBACK FEEDS THE LANDING PROBE AT ALL. A
//      probe wired to nothing passes every unit test it has and reports all-zero
//      counters forever. And zero is not obviously wrong here: `landedAtFrontier`
//      is SUPPOSED to be small (59 triggers against 4,552 corrections), so an
//      unwired probe reads exactly like the mechanism working.
//   2. THAT THE FRONTIER COMES FROM THE LIVE CACHE, not from a value captured at
//      bind time or carried stale. The whole discriminator is `tick ==
//      predictionTick` at insert time; a frontier read that lags by one tick moves
//      every AtFrontier event into Behind and turns the finding's central claim
//      into its opposite while every counter still adds up.
//   3. THAT A DISCARD REACHES THE LANDING PROBE. Its sibling
//      CorrectionVerdictProbe deliberately returns early on `!verdict.landed`, and
//      the landing probe deliberately sits on the other side of that return. If
//      the two ever get reordered, the `discarded` bucket empties, and
//      `atFrontierPerMille` RISES — the healthy-looking direction — on precisely
//      the clients where item 41's aboveNewest anomaly is worst.
// ===========================================================================

TEST_CASE("DAttack.SimulationNetSync.LandingProbeSplitsByFrontierAndByClass",
          "[DAttack][SimulationNetSync][ResimLandingWiring]")
{
    ResimRig rig;
    MockPredictionOwner local;
    MockPredictionOwner remote;
    addLocalCharacter(rig, 195u, local);     // provider PRESENT
    addRemoteCharacter(rig, 196u, remote);   // provider ABSENT

    const CorrectionLandingProbe& probe = rig.netSync.getDiagnostics().correctionLandingProbe();
    const CorrectionVerdictProbe& verdicts = rig.netSync.getDiagnostics().correctionVerdictProbe();

    // Registration alone feeds nothing — this probe counts CORRECTIONS.
    REQUIRE(probe.sampleCount() == 0u);

    for (unsigned int tick = 1u; tick <= 5u; ++tick)
        predictTick(rig, tick);
    // Frontier is now 5 for both characters.

    // ASYMMETRIC IN BOTH DIMENSIONS — different totals per class and a different
    // bucket mix inside each, so that a SWAPPED class test, a POOLED counter and a
    // MERGED bucket each land on a distinguishable set of numbers. A one-of-each
    // drive would read identically under a swap and would certify the exact defect
    // this case exists to catch (the T24 lesson, applied at authoring time).
    landCorrection(local,  2u, 1u);      // behind
    landCorrection(local,  3u, 2u);      // behind
    landCorrection(local,  4u, 3u);      // behind
    landCorrection(local,  5u, 4u);      // AT THE FRONTIER — the gate opener
    landCorrection(remote, 3u, 2u);      // behind
    landCorrection(remote, 9000u, 8999u);  // no slot -> discarded
    landCorrection(remote, 9001u, 9000u);  // no slot -> discarded

    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::Behind)     == 3u);
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::AtFrontier) == 1u);
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::Discarded)  == 0u);
    REQUIRE(probe.countFor(PredictedCharacterClass::RemoteProxy,
                           CorrectionLandingSite::Behind)     == 1u);
    REQUIRE(probe.countFor(PredictedCharacterClass::RemoteProxy,
                           CorrectionLandingSite::AtFrontier) == 0u);
    REQUIRE(probe.countFor(PredictedCharacterClass::RemoteProxy,
                           CorrectionLandingSite::Discarded)  == 2u);

    // ⭐ THE TWO PROBES ON THIS ONE CALLBACK HAVE DIFFERENT SAMPLE SETS, AND THAT
    // DIFFERENCE IS THE POINT. Seven corrections arrived; five of them landed. The
    // verdict probe counts only the five (a discard produced no comparison, so
    // counting it would put a denominator under a verdict that never happened); the
    // landing probe counts all seven (a discard IS an observation of where the
    // correction stream sits relative to the cache window — it is item 41's
    // aboveNewest population). If a future edit moves the landing probe above the
    // `!verdict.landed` return, these two numbers become equal and this assertion
    // is what notices.
    REQUIRE(probe.sampleCount()    == 7u);
    REQUIRE(verdicts.sampleCount() == 5u);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(195u, &local, rig.inputResolution);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(196u, &remote, rig.inputResolution);
}

TEST_CASE("DAttack.SimulationNetSync.LandingProbeReadsTheFrontierLiveNotCaptured",
          "[DAttack][SimulationNetSync][ResimLandingWiring]")
{
    ResimRig rig;
    MockPredictionOwner local;
    addLocalCharacter(rig, 197u, local);

    const CorrectionLandingProbe& probe = rig.netSync.getDiagnostics().correctionLandingProbe();

    for (unsigned int tick = 1u; tick <= 3u; ++tick)
        predictTick(rig, tick);

    // Tick 3 IS the frontier right now.
    landCorrection(local, 3u, 2u);
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::AtFrontier) == 1u);

    // The frontier advances — this is `pushPredictionTick`, the same call that
    // copies the inherited resim bit forward and is therefore the exact mechanism
    // the split measures.
    predictTick(rig, 4u);

    // THE SAME TICK 3, LANDING AGAIN, IS NOW BEHIND. This is the assertion that a
    // captured-at-bind or cached frontier cannot satisfy: it would re-file this as
    // AtFrontier (stale value 3) and the next one as Behind, i.e. exactly inverted.
    landCorrection(local, 3u, 2u);
    landCorrection(local, 4u, 3u);

    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::Behind)     == 1u);
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::AtFrontier) == 2u);
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::Discarded)  == 0u);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(197u, &local, rig.inputResolution);
}

TEST_CASE("DAttack.SimulationNetSync.LandingProbeIsPurelyObservational",
          "[DAttack][SimulationNetSync][ResimLandingWiring]")
{
    // BEHAVIOUR-NEUTRALITY, ASSERTED RATHER THAN CLAIMED. Item 42 changes no
    // behaviour, and the one thing on this path that COULD have changed it is the
    // hoisted provider-presence lookup and the extra `findInputCache` read now
    // sitting between `injectCorrectionState` and the verdict probe. So: land a
    // correction that DISAGREES and check the cache still adopted authority state,
    // exactly as the T24 case one block up does — the simulation must act on the
    // correction identically with the landing probe in the path.
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 198u, remote);

    predictTick(rig, 1u);
    divergeState(rig, 198u, 25.f);
    predictTick(rig, 2u);

    landCorrection(remote, 2u, 1u);

    // It was classified...
    REQUIRE(rig.netSync.getDiagnostics().correctionLandingProbe().countFor(
                PredictedCharacterClass::RemoteProxy,
                CorrectionLandingSite::AtFrontier) == 1u);
    // ...the verdict still travelled...
    REQUIRE(rig.netSync.getDiagnostics().correctionVerdictProbe().disagreementsFor(
                PredictedCharacterClass::RemoteProxy) == 1u);
    // ...and the cache still overwrote the slot, so a restore hands back the
    // AUTHORITY's attackTimer (0) and not the 25 this client predicted.
    rig.reconciliation.prepareResimAll(2u);
    REQUIRE(rig.storage.get<SimulatableBrawler>(198u)
                .getAllState().getState()
                .get<dAttackGuardSimulation::State>().attackTimer == 0.f);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(198u, &remote, rig.inputResolution);
}


// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay / T39] STATE ROTATION — THE INTEGRATION HALF.
//
// The selection arithmetic (coverage, wrap, the K >= N degeneracy, the clamp) is
// swept exhaustively as a unit in
// og-simulation-tests/Network/CorrectionRotationTest.cpp, where no mock character
// is needed. What THAT suite structurally cannot show is that sendCorrectionAll
// actually calls it — a green kernel behind an unwired caller ships every-frame
// state and passes everything. These cases close that gap by counting real
// writes into real authority owners' correction buffers.
// ---------------------------------------------------------------------------

namespace
{
    // Owns the pieces a multi-character authority send needs, so a case can say
    // "three characters, K=1, four ticks" in a few lines. Owners are held by
    // pointer-stable storage because netSync keeps references to them.
    struct RotationRig
    {
        SimulationObjectStorage<SimulatableBrawler>  storage;
        SimulationReconciliation<SimulatableBrawler> reconciliation{ storage };
        SimulationInputResolution<SimulatableBrawler>        inputResolution{ storage, reconciliation };
        SimulationNetSync<SimulatableBrawler>        netSync{ storage, reconciliation, inputResolution };

        std::vector<std::unique_ptr<MockPredictionOwner>> predictionOwners;
        std::vector<std::unique_ptr<MockAuthorityOwner>>  authorityOwners;
        std::vector<unsigned int>                        ids;

        void addCharacters(unsigned int count)
        {
            for (unsigned int i = 0; i < count; ++i)
            {
                const unsigned int id = 100u + i;
                storage.add<SimulatableBrawler>(id, makeNetSyncTestCharacter());
                reconciliation.createCacheFor<SimulatableBrawler>(id);

                predictionOwners.push_back(std::make_unique<MockPredictionOwner>());
                authorityOwners.push_back(std::make_unique<MockAuthorityOwner>());

                netSync.registerPredictionOwner<SimulatableBrawler>(
                    id, *predictionOwners.back(), nullptr, inputResolution);
                netSync.registerAuthorityOwner<SimulatableBrawler>(
                    id, *authorityOwners.back(), inputResolution);

                ids.push_back(id);
            }
        }

        void send(uint32 tick, int32 k)
        {
            netSync.sendCorrectionAll(SimulationTimeStep(tick, false, StepKind::Normal), k);
        }

        // Total writes observed across every authority owner.
        int totalWrites() const
        {
            int total = 0;
            for (const auto& owner : authorityOwners)
                total += owner->stateBuf.writeCount;
            return total;
        }

        // The smallest per-character write count — the starvation detector. A
        // rotation that covers "most" characters is not a cadence, it is a leak.
        int minWrites() const
        {
            int smallest = -1;
            for (const auto& owner : authorityOwners)
            {
                const int w = owner->stateBuf.writeCount;
                if (smallest < 0 || w < smallest)
                    smallest = w;
            }
            return smallest;
        }
    };
}

TEST_CASE("DAttack.SimulationNetSync.StateRotationWritesExactlyKPerTick",
          "[DAttack][SimulationNetSync][StateRotation]")
{
    // THE HEADLINE FACT, measured through the real send path: with N characters
    // registered and K passed in, exactly K correction buffers are written per
    // call. This is what turns the state channel's per-tick byte cost into the
    // constant `K * stateBytes` the round-vs-packet LLT budgets against — a send
    // that sometimes wrote N would blow that budget silently.
    RotationRig rig;
    rig.addCharacters(4u);

    rig.send(/*tick=*/1u, /*k=*/1);
    REQUIRE(rig.totalWrites() == 1);

    rig.send(2u, 1);
    REQUIRE(rig.totalWrites() == 2);

    // ...and it is K, not "one", so a hardcoded single-writer implementation is
    // caught rather than passing the case above.
    RotationRig two;
    two.addCharacters(4u);
    two.send(1u, 2);
    REQUIRE(two.totalWrites() == 2);
    two.send(2u, 2);
    REQUIRE(two.totalWrites() == 4);
}

TEST_CASE("DAttack.SimulationNetSync.StateRotationStarvesNobody",
          "[DAttack][SimulationNetSync][StateRotation]")
{
    // ⭐ THE PROPERTY THE CADENCE CLAIM RESTS ON. "20 Hz per character at six
    // characters" is only honest if no character can be skipped indefinitely, so
    // this asserts the bound directly on the real send path: after ceil(N/K)
    // sends every single owner has been written at least once, and after a full
    // second every owner has been written the SAME number of times.
    RotationRig rig;
    rig.addCharacters(6u);

    constexpr int32 kK = 2;               // the shipped default
    constexpr uint32 kCoverageBound = 3u; // ceil(6/2)

    for (uint32 tick = 1u; tick <= kCoverageBound; ++tick)
        rig.send(tick, kK);

    REQUIRE(rig.minWrites() >= 1);
    REQUIRE(rig.totalWrites() == static_cast<int>(kCoverageBound) * kK);

    // One second at 60 Hz: 60 * K / N = 20 writes per character, exactly and
    // uniformly. Uniformity matters as much as the rate — a rotation that gave
    // one character 30 and another 10 would average correctly and feel wrong.
    RotationRig second;
    second.addCharacters(6u);
    for (uint32 tick = 1u; tick <= 60u; ++tick)
        second.send(tick, kK);

    for (const auto& owner : second.authorityOwners)
    {
        REQUIRE(owner->stateBuf.writeCount == 20);
    }
}

TEST_CASE("DAttack.SimulationNetSync.StateRotationAtTwoCharactersIsEveryFrame",
          "[DAttack][SimulationNetSync][StateRotation]")
{
    // THE BASELINE-PRESERVING PROPERTY: at two characters K=2 is K >= N, so every
    // character is written every tick — bit-for-bit the pre-T39 cadence, which is
    // what kept the archived two-character measurement baselines comparable across
    // the input-first-replication split. It was T39's reason for a compiled default
    // of 2, and it is a real property of the send path, so it keeps its own case.
    //
    // ⚠ [T34] IT IS NO LONGER THE SHIPPED CONFIGURATION. The compiled default is 1
    // for the pre-diet window (at K=2 with un-dieted states the second state batch
    // enters Iris's huge-object window at four characters — T38 §16.2); item 40
    // restores 2. So this case now drives K=2 EXPLICITLY and asserts the shipped
    // cadence separately, rather than assuming the two are the same number. The
    // archived-baseline comparison is correspondingly halved at two characters —
    // designed, and stated here because a run that expected 60 Hz would read 30 Hz
    // as a regression.
    RotationRig rig;
    rig.addCharacters(2u);

    constexpr int32 kEveryFrameAtTwo = 2;
    for (uint32 tick = 1u; tick <= 10u; ++tick)
        rig.send(tick, kEveryFrameAtTwo);

    for (const auto& owner : rig.authorityOwners)
    {
        REQUIRE(owner->stateBuf.writeCount == 10);
        REQUIRE(owner->stateBuf.lastTick == 10u);
    }

    // THE SHIPPED CADENCE at two characters, driven through the same rig: K=1
    // alternates, so each character is written on half the ticks. This is the row
    // that has to move when item 40 restores K=2.
    const int32 shippedK = TimeConfig{}.correctionRotationK;
    REQUIRE(shippedK == 1);

    RotationRig shipped;
    shipped.addCharacters(2u);
    for (uint32 tick = 1u; tick <= 10u; ++tick)
        shipped.send(tick, shippedK);

    for (const auto& owner : shipped.authorityOwners)
    {
        REQUIRE(owner->stateBuf.writeCount == 5);
    }
    REQUIRE(shipped.totalWrites() == 10);
}

TEST_CASE("DAttack.SimulationNetSync.StateRotationClampsAtTheSendPath",
          "[DAttack][SimulationNetSync][StateRotation]")
{
    // The shared clamp is called inside the selection predicate as well as at the
    // setter, so a K that reached the send path unclamped — through a future
    // caller that skipped SimulationManager::setCorrectionRotationK — still
    // degrades to "write one" rather than to "write none". A silently empty
    // correction channel is a permanent desync with no log line.
    RotationRig rig;
    rig.addCharacters(3u);

    rig.send(/*tick=*/1u, /*k=*/0);
    REQUIRE(rig.totalWrites() == 1);

    rig.send(2u, -7);
    REQUIRE(rig.totalWrites() == 2);
}

TEST_CASE("DAttack.SimulationNetSync.StateRotationIsANoOpWithNoAuthorityWriters",
          "[DAttack][SimulationNetSync][StateRotation]")
{
    // sendCorrectionAll runs unconditionally every tick, including on a pure
    // client where the authority-writer map is empty for every type, and on the
    // server before the first character registers. The rotation must not divide
    // by zero on that path — the predicate answers false and the cursor still
    // advances, because the cursor is type-independent.
    RotationRig rig;   // no characters added

    rig.send(1u, 2);
    rig.send(2u, 2);
    REQUIRE(rig.totalWrites() == 0);

    // Registering after the empty sends must not leave the newcomer stranded on
    // the wrong side of an advanced cursor — coverage is a property of positions,
    // so a late registration is picked up within one round.
    rig.addCharacters(2u);
    rig.send(3u, 2);
    REQUIRE(rig.minWrites() >= 1);
}


#endif // WITH_LOW_LEVEL_TESTS
