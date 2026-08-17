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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    auto inputProvider = [zeroInput](const SimulationTimeStep&, const BrawlerLocalInputCache&) { return zeroInput; };

    netSync.registerPredictionOwner<SimulatableBrawler>(42u, predictionOwner, std::move(inputProvider));

    // [T8] The correction-INPUT binding assertions that sat beside these are gone
    // with the channel; the state binding is what registration/unregistration still
    // has to get right for a LOCAL character. (A remote character additionally binds
    // the relay ring — covered by the [InputRelay] registration cases.)
    REQUIRE(predictionOwner.onCorrectionStateReceived != nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(42u, &predictionOwner, nullptr);

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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(1u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(1u, authorityOwner);

    REQUIRE(authorityOwner.onRemoteMoveReceived != nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(1u, &predictionOwner, &authorityOwner);

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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(1u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(1u, authorityOwner);

    // Must not throw. (Stage 1, Task 9: now takes currentTick + redundancyDepth.)
    netSync.sendLocalInputToAuthorityAll(0u, 5u);

    REQUIRE(true);

    netSync.unregisterSimulatable<SimulatableBrawler>(1u, &predictionOwner, &authorityOwner);
}

// ---------------------------------------------------------------------------
// Test: collectInputAll advances the reconciliation cache ring-buffer slot.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationNetSync.CacheSlotAdvances", "[DAttack][SimulationNetSync]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(10u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(10u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    int collectCallCount = 0;
    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    auto inputProvider = [zeroInput, &collectCallCount](const SimulationTimeStep&,
                                                        const BrawlerLocalInputCache&) {
        ++collectCallCount;
        return zeroInput;
    };

    netSync.registerPredictionOwner<SimulatableBrawler>(10u, predictionOwner, inputProvider);

    for (unsigned int tick = 1; tick <= 3; ++tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
        reconciliation.postPredictionAll(step);
    }

    REQUIRE(collectCallCount == 3);
}

// ===========================================================================
// [T9 parts 3+4] CLIENT LAYER-1 INPUT DELAY — end-to-end through the REAL
// SimulationNetSync::collectInputAll provider branch.
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

TEST_CASE("DAttack.SimulationNetSync.ClientDelayShiftsIntegratedInput",
          "[DAttack][SimulationNetSync][ClientInputDelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(7u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(7u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(7u, predictionOwner,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        });

    constexpr int32 kDelay = 2;
    netSync.setClientEffectiveInputDelayTicks(kDelay);
    REQUIRE(netSync.getClientEffectiveInputDelayTicks() == kDelay);

    for (unsigned int tick = 1u; tick <= 6u; ++tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
        reconciliation.postPredictionAll(step);

        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        REQUIRE(map.find(7u) != map.end());
        const float integratedTag = captureTagOf(map.at(7u));

        if (tick <= static_cast<unsigned int>(kDelay))
        {
            // PART 4: `tick - delay` is a tick that was never captured (this run
            // starts at tick 1, and tick 0 is the reserved pre-sim tick), so the
            // integrator sees the injected ZERO input rather than an
            // uninitialised slot or the live capture.
            REQUIRE(integratedTag == Catch::Approx(kNeutralTag));
            REQUIRE_FALSE(integratedTag == Catch::Approx(static_cast<float>(tick)));
            // ...and specifically the GAME's zero input, not a value-initialised
            // one. See isGameZeroInput.
            REQUIRE(isGameZeroInput(map.at(7u)));
        }
        else
        {
            // PART 3: exactly `kDelay` ticks behind — never the live capture.
            REQUIRE(integratedTag == Catch::Approx(static_cast<float>(tick) - kDelay));
        }
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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(8u, predictionOwner,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        });

    constexpr int32 kDelay = 3;
    netSync.setClientEffectiveInputDelayTicks(kDelay);

    constexpr unsigned int kLastTick = 6u;
    for (unsigned int tick = 1u; tick <= kLastTick; ++tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
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

TEST_CASE("DAttack.SimulationNetSync.ZeroClientDelayIsUnchangedBehaviour",
          "[DAttack][SimulationNetSync][ClientInputDelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(9u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(9u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    // Deliberately NO setClientEffectiveInputDelayTicks call and NO
    // setNeutralInput call — this is the shape every pre-T9 caller has, and it
    // must behave exactly as it did before.
    netSync.registerPredictionOwner<SimulatableBrawler>(9u, predictionOwner,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        });

    REQUIRE(netSync.getClientEffectiveInputDelayTicks() == 0);

    for (unsigned int tick = 1u; tick <= 4u; ++tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
        reconciliation.postPredictionAll(step);

        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        REQUIRE(captureTagOf(map.at(9u)) == Catch::Approx(static_cast<float>(tick)));
    }

    // THE case that makes this suite able to see a zero-delay regression at all.
    //
    // A Stall step does NOT push into the delay line (it pushes nothing anywhere
    // — no prediction tick, no pending-queue entry). So if the zero-delay path
    // were "read at(currentTick)" instead of "return the live capture", every
    // NORMAL tick above would still pass — the capture was just pushed — and only
    // a Stall would expose it, by reading an absent slot and integrating the
    // neutral input where the live capture belongs.
    //
    // Without this section, `resolveDelayedInput`'s `effectiveDelay <= 0`
    // early-out would look like a pure optimisation that could be "simplified"
    // away with a green suite.
    SimulationTimeStep stallStep(99u, false, StepKind::Stall);
    auto stalled = netSync.collectInputAll(stallStep);
    const auto& stalledMap = std::get<
        std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(stalled);

    REQUIRE(captureTagOf(stalledMap.at(9u)) == Catch::Approx(99.f));
    REQUIRE_FALSE(isGameZeroInput(stalledMap.at(9u)));
}

TEST_CASE("DAttack.SimulationNetSync.ClientDelayChangeShiftsTheOffset",
          "[DAttack][SimulationNetSync][ClientInputDelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(11u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(11u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(11u, predictionOwner,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        });

    auto integratedTagAt = [&](unsigned int tick) {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
        reconciliation.postPredictionAll(step);
        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        return captureTagOf(map.at(11u));
    };

    // Tier 0-ish: no delay.
    netSync.setClientEffectiveInputDelayTicks(0);
    for (unsigned int tick = 1u; tick <= 8u; ++tick)
    {
        REQUIRE(integratedTagAt(tick) == Catch::Approx(static_cast<float>(tick)));
    }

    // Tier escalates (this is what OnRep_ConnectionTier publishes). The very next
    // tick must already read from the delayed slot — the captures for the
    // intervening ticks are resident, so no neutral window is re-entered.
    netSync.setClientEffectiveInputDelayTicks(4);
    for (unsigned int tick = 9u; tick <= 12u; ++tick)
    {
        REQUIRE(integratedTagAt(tick) == Catch::Approx(static_cast<float>(tick) - 4.f));
    }

    // De-escalation snaps back to the live capture.
    netSync.setClientEffectiveInputDelayTicks(0);
    REQUIRE(integratedTagAt(13u) == Catch::Approx(13.f));
}

TEST_CASE("DAttack.SimulationNetSync.ResyncWipeClearsTheLocalInputCache",
          "[DAttack][SimulationNetSync][ClientInputDelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(12u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(12u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(12u, predictionOwner,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        });

    constexpr int32 kDelay = 2;
    netSync.setClientEffectiveInputDelayTicks(kDelay);

    for (unsigned int tick = 1u; tick <= 6u; ++tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
        reconciliation.postPredictionAll(step);
    }

    // A hard resync jumps the prediction clock. Captures keyed to the OLD clock
    // describe ticks that no longer mean what they meant, so they must not be
    // read at the new ones.
    //
    // THE RESYNC TARGET IS DELIBERATELY *INSIDE* THE RESIDENT RANGE, and this is
    // the difference between a test that works and one that only looks like it
    // does. A forward jump to some far tick (say 100) is unobservable: `at()`
    // validates the STORED tick against the requested one, so a stale capture at
    // tick 3 can never answer a query for tick 98 whether or not the line was
    // cleared. The bug is only reachable when the new clock asks for a tick
    // number that a pre-resync capture genuinely occupies — i.e. a backward or
    // short resync. Resyncing to tick 5 makes `5 - kDelay == 3` land exactly on
    // the surviving pre-resync capture 3.
    //
    // (An earlier version of this case resynced to tick 100 and PASSED with
    // wipeAllForResync's delay-line clear deleted. It proved nothing.)
    // Both wipes, in SimulationManager's resync-callback order. The reconciliation
    // wipe is not incidental: without it the correction cache rejects the
    // backward prediction tick outright ("Setting bad prediction tick"), which is
    // what production's paired call exists to prevent.
    reconciliation.wipeAllForResync(5u);
    netSync.wipeAllForResync(5u);

    SimulationTimeStep postResync(5u, false, StepKind::Normal);
    auto inputs = netSync.collectInputAll(postResync);
    const auto& map = std::get<
        std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);

    // Without the clear, this reads the pre-resync capture from tick 3 — an
    // input produced against a clock epoch that no longer exists. With it, the
    // line re-enters the same well-defined neutral window as session start.
    REQUIRE(captureTagOf(map.at(12u)) == Catch::Approx(kNeutralTag));
    REQUIRE_FALSE(captureTagOf(map.at(12u)) == Catch::Approx(3.f));
    REQUIRE(isGameZeroInput(map.at(12u)));
}

// ===========================================================================
// [og-netcode-v2-input-relay T2] THE APPLIED CAPTURE TICK + UNDERRUN SENTINEL —
// end-to-end through the REAL SimulationNetSync::collectInputAll REMOTE branch.
//
// WHY HERE and not in og-simulation-tests. Same reason as the client-delay block
// above: collectInputAll is variadic over a simulatable pack and needs
// SimulatableOwnerTraits bound to concrete owners, which only a suite linking a
// real simulatable can supply. The container-level half of this task — the proof
// that a bare dequeue CANNOT classify an underrun — is unit-tested in the core
// suite (og-simulation-tests, PCTimeManagement/RemoteMoveQueueDedupTest.cpp,
// [PCTM][RemoteMoveQueue][InputRelay], MPL-2.0). These cases drive the shipped
// branch that consumes that property.
//
// WHAT THEY PIN. The authority now records, per id, the CAPTURE TICK behind the
// input it applied — the join key the whole input relay is built on (T4
// replicates it, T6 resolves remote inputs by it). Two values leave the branch:
//   * a real applied input      -> the original capture tick (move.tick);
//   * a queue underrun          -> kNoInputCaptureTick, because the applied input
//                                  was a SUBSTITUTE that no client ever captured.
//
// THE DISCRIMINATING CASE is RemoteBranchRealCaptureTickZeroIsNotTheSentinel.
// `dequeueMove()` on an empty queue returns a value-initialised Move{} with tick
// 0, so an implementation that inferred the underrun from the RETURNED TICK
// would classify a genuine tick-0 capture — an ordinary session-start input — as
// "no real input", and T6 would then resolve a live input to game-zero. The first
// two cases below would pass under that broken implementation; only the third
// fails. It is the reason the production gate is a pre-dequeue empty() check.
// ===========================================================================

namespace
{
    // Drive one authority tick through the real branch and hand back the input it
    // resolved, so each case asserts on the SAME tick it inspects the record for.
    simulatableBrawler::PlayerInput authorityTick(
        SimulationNetSync<SimulatableBrawler>& netSync, unsigned int id, unsigned int tick)
    {
        SimulationTimeStep step(tick, false, StepKind::Normal);
        auto inputs = netSync.collectInputAll(step);
        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        REQUIRE(map.find(id) != map.end());
        return map.at(id);
    }
}

TEST_CASE("DAttack.SimulationNetSync.RemoteBranchRecordsAppliedCaptureTick",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(20u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(20u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    // Server shape: prediction owner WITHOUT an input provider (so collectInputAll
    // takes the remote branch) plus an authority owner (so the queue exists).
    netSync.registerPredictionOwner<SimulatableBrawler>(20u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(20u, authorityOwner);

    // Before the authority has ticked this id at all, there is no applied input —
    // which is precisely what the sentinel means. Seeding 0 here would claim
    // capture tick 0 was applied.
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(20u) == kNoInputCaptureTick);

    // Two inbound captures, at ticks deliberately unequal to the authority tick
    // they are applied on — the record must carry the CAPTURE tick, not the
    // application tick.
    authorityOwner.onRemoteMoveReceived(51u, taggedCapture(51.f));
    authorityOwner.onRemoteMoveReceived(52u, taggedCapture(52.f));

    const auto appliedAt1 = authorityTick(netSync, 20u, 1u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(20u) == 51u);
    REQUIRE(captureTagOf(appliedAt1) == Catch::Approx(51.f));
    // Not the authority tick, and not the sentinel.
    REQUIRE_FALSE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(20u) == 1u);
    REQUIRE_FALSE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(20u) == kNoInputCaptureTick);

    const auto appliedAt2 = authorityTick(netSync, 20u, 2u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(20u) == 52u);
    REQUIRE(captureTagOf(appliedAt2) == Catch::Approx(52.f));

    // The record is populated at registerAuthorityOwner, so unregistration must
    // clear it — after it the accessor answers the sentinel rather than throwing.
    // ([T8] this used to read "shares m_lastUsedInputs' key set, so unregistration
    // must clear both"; that twin is retired and this track stands alone.)
    netSync.unregisterSimulatable<SimulatableBrawler>(20u, &predictionOwner, &authorityOwner);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(20u) == kNoInputCaptureTick);
}

TEST_CASE("DAttack.SimulationNetSync.RemoteBranchUnderrunRecordsSentinel",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(21u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(21u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(21u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(21u, authorityOwner);

    // One capture available, then nothing — the authority keeps ticking either way
    // and substitutes an input when the queue runs dry.
    authorityOwner.onRemoteMoveReceived(70u, taggedCapture(70.f));

    authorityTick(netSync, 21u, 1u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(21u) == 70u);

    // UNDERRUN. The applied input is a substitute; no client capture stands behind
    // it, so the record must say so explicitly rather than go stale on 70.
    authorityTick(netSync, 21u, 2u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(21u) == kNoInputCaptureTick);
    REQUIRE_FALSE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(21u) == 70u);

    // A second consecutive underrun holds the sentinel.
    authorityTick(netSync, 21u, 3u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(21u) == kNoInputCaptureTick);

    // ...and the track RECOVERS: a later arrival is recorded normally, so the
    // sentinel is a per-tick classification and not a latch.
    authorityOwner.onRemoteMoveReceived(71u, taggedCapture(71.f));
    authorityTick(netSync, 21u, 4u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(21u) == 71u);

    netSync.unregisterSimulatable<SimulatableBrawler>(21u, &predictionOwner, &authorityOwner);
}

// THE case that makes this suite able to see the defect at all — see the block
// comment above.
TEST_CASE("DAttack.SimulationNetSync.RemoteBranchRealCaptureTickZeroIsNotTheSentinel",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(22u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(22u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(22u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(22u, authorityOwner);

    // A GENUINE capture tick 0 — an ordinary session-start input, not an underrun.
    authorityOwner.onRemoteMoveReceived(0u, taggedCapture(123.f));

    const auto applied = authorityTick(netSync, 22u, 1u);

    // The input really was the client's, so the record must be the real tick 0.
    REQUIRE(captureTagOf(applied) == Catch::Approx(123.f));
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(22u) == 0u);
    // An implementation that inferred the underrun from the dequeued tick would
    // read 0 here and store the sentinel, silently discarding a live input's join
    // key. This is the assertion that fails under that implementation.
    REQUIRE_FALSE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(22u) == kNoInputCaptureTick);

    // The very next tick DOES underrun, and is classified differently — proving
    // the two situations are distinguished by the queue state and not by the tick
    // value, which is identical in both.
    authorityTick(netSync, 22u, 2u);
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(22u) == kNoInputCaptureTick);

    netSync.unregisterSimulatable<SimulatableBrawler>(22u, &predictionOwner, &authorityOwner);
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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    netSync.registerPredictionOwner<SimulatableBrawler>(30u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(30u, authorityOwner);

    // Before any authority tick there is no applied input at all — the send must
    // publish the sentinel rather than an invented reference.
    netSync.sendCorrectionAll(SimulationTimeStep(10u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastTick == 10u);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);

    // A real capture, applied on a DIFFERENT authority tick than it was taken —
    // so a send that published its own tick, or the delay, would be caught.
    authorityOwner.onRemoteMoveReceived(51u, taggedCapture(51.f));
    authorityTick(netSync, 30u, 11u);

    netSync.sendCorrectionAll(SimulationTimeStep(11u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastTick == 11u);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == 51u);
    REQUIRE_FALSE(authorityOwner.stateBuf.lastAppliedCaptureTick == 11u);

    // UNDERRUN on the next tick: the authority substituted an input, so the state
    // must say so explicitly instead of repeating the now-stale 51 — a stale
    // reference would send the client looking up a relayed input the authority
    // never applied.
    authorityTick(netSync, 30u, 12u);
    netSync.sendCorrectionAll(SimulationTimeStep(12u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);
    REQUIRE_FALSE(authorityOwner.stateBuf.lastAppliedCaptureTick == 51u);

    // ...and it recovers on the next real arrival — per-tick classification, not
    // a latch.
    authorityOwner.onRemoteMoveReceived(52u, taggedCapture(52.f));
    authorityTick(netSync, 30u, 13u);
    netSync.sendCorrectionAll(SimulationTimeStep(13u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == 52u);

    // [T8] The EXPAND/CONTRACT FENCE assertion (B3) stood here: `inputBuf.lastTick
    // == 13u`, pinning that T4 left the old correction-INPUT write firing on every
    // send. T8 IS the contract half, so the fence has been DISCHARGED rather than
    // broken — there is no longer a second write for a fence to protect. Everything
    // above (the ref's per-tick classification, which is what this case is about)
    // is unchanged and still asserted.

    netSync.unregisterSimulatable<SimulatableBrawler>(30u, &predictionOwner, &authorityOwner);
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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwnerA;
    MockPredictionOwner predictionOwnerB;
    MockAuthorityOwner  authorityOwnerA;
    MockAuthorityOwner  authorityOwnerB;

    netSync.registerPredictionOwner<SimulatableBrawler>(31u, predictionOwnerA, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(31u, authorityOwnerA);
    netSync.registerPredictionOwner<SimulatableBrawler>(32u, predictionOwnerB, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(32u, authorityOwnerB);

    // Only A has input available.
    authorityOwnerA.onRemoteMoveReceived(80u, taggedCapture(80.f));

    SimulationTimeStep step(20u, false, StepKind::Normal);
    netSync.collectInputAll(step);
    netSync.sendCorrectionAll(step, kEveryFrameRotationK);

    REQUIRE(authorityOwnerA.stateBuf.lastAppliedCaptureTick == 80u);
    REQUIRE(authorityOwnerB.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);
    REQUIRE(authorityOwnerA.stateBuf.lastTick == authorityOwnerB.stateBuf.lastTick);

    netSync.unregisterSimulatable<SimulatableBrawler>(31u, &predictionOwnerA, &authorityOwnerA);
    netSync.unregisterSimulatable<SimulatableBrawler>(32u, &predictionOwnerB, &authorityOwnerB);
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

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;

    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    netSync.registerPredictionOwner<SimulatableBrawler>(
        33u, predictionOwner, [zeroInput](const SimulationTimeStep&, const BrawlerLocalInputCache&) { return zeroInput; });
    REQUIRE(predictionOwner.onCorrectionStateReceived != nullptr);

    // Predict three ticks so the cache has slots for them.
    for (uint32 tick = 40u; tick <= 42u; ++tick)
        netSync.collectInputAll(SimulationTimeStep(tick, false, StepKind::Normal));

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

    netSync.unregisterSimulatable<SimulatableBrawler>(33u, &predictionOwner);
}

// ===========================================================================
// [T5] THE CLIENT RELAYED-INPUT STORE, through the REAL SimulationNetSync wiring.
//
// WHY HERE and not in og-simulation-tests. The container, the derivation and the
// wire fence are unit-tested there (Network/RemoteInputCacheTest.cpp, MPL-2.0).
// What that suite cannot reach is the WIRING: which ids get a store, when the
// callback is bound, what the bind-time populate recovers, whether the resync wipe
// leaves the stores alone, and whether the injected neutral that reaches them is
// the GAME's zero rather than a value-initialised PlayerInput. Those need
// SimulatableOwnerTraits bound to concrete owners and a real InputType, i.e. this
// suite.
// ===========================================================================

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

TEST_CASE("DAttack.SimulationNetSync.RemoteInputCacheOnlyForRemoteCharacters",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    // PROVIDER-PRESENCE is the local-vs-remote test, and this case is the reason:
    // under COUCH CO-OP one client legitimately owns SEVERAL locally-controlled
    // characters, so any test shaped like "the client's character" would give the
    // siblings a relay store they must not have. Two local + one remote here.
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(50u, makeNetSyncTestCharacter());
    storage.add<SimulatableBrawler>(51u, makeNetSyncTestCharacter());
    storage.add<SimulatableBrawler>(52u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(50u);
    reconciliation.createCacheFor<SimulatableBrawler>(51u);
    reconciliation.createCacheFor<SimulatableBrawler>(52u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner localA, localB, remote;

    const simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    auto provider = [zeroInput](const SimulationTimeStep&, const BrawlerLocalInputCache&) { return zeroInput; };

    netSync.registerPredictionOwner<SimulatableBrawler>(50u, localA, provider);
    netSync.registerPredictionOwner<SimulatableBrawler>(51u, localB, provider);
    netSync.registerPredictionOwner<SimulatableBrawler>(52u, remote, nullptr);

    REQUIRE(netSync.findRemoteInputCache<SimulatableBrawler>(50u) == nullptr);
    REQUIRE(netSync.findRemoteInputCache<SimulatableBrawler>(51u) == nullptr);
    REQUIRE(netSync.findRemoteInputCache<SimulatableBrawler>(52u) != nullptr);

    // ...and the relay callback follows the same split: a locally-controlled
    // character's inputs are never relayed back to the client that produced them.
    REQUIRE(localA.onRelayedInputReceived == nullptr);
    REQUIRE(localB.onRelayedInputReceived == nullptr);
    REQUIRE(remote.onRelayedInputReceived != nullptr);

    // An unknown id answers nullptr rather than throwing — the accessor is
    // nullable by design, because the eventual callers see ids of both classes.
    REQUIRE(netSync.findRemoteInputCache<SimulatableBrawler>(9999u) == nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(52u, &remote);
    REQUIRE(remote.onRelayedInputReceived == nullptr);
    REQUIRE(netSync.findRemoteInputCache<SimulatableBrawler>(52u) == nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(50u, &localA);
    netSync.unregisterSimulatable<SimulatableBrawler>(51u, &localB);
}

TEST_CASE("DAttack.SimulationNetSync.RelayRingArrivalReachesTheStore",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(53u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(53u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner remote;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(53u, remote, nullptr);

    auto* store = netSync.findRemoteInputCache<SimulatableBrawler>(53u);
    REQUIRE(store != nullptr);
    REQUIRE_FALSE(store->findLatest().valid);

    // The server writes the ring; replication delivers it (OnRep_RelayedInputRing
    // does exactly what replicateRelayRing does here and nothing else).
    relayWrite(remote.relayedInputRing, 70u, 4u, 70.f);
    remote.replicateRelayRing();

    std::uint8_t dA = 0u;
    simulatableBrawler::PlayerInput input;
    REQUIRE(store->find(70u, dA, input));
    REQUIRE(dA == 4u);
    REQUIRE(captureTagOf(input) == Catch::Approx(70.f));

    // A capture tick that never arrived is a MISS, not a silently-invented neutral
    // — that is what keeps T7's ladder able to fall back deliberately.
    REQUIRE_FALSE(store->find(71u, dA, input));

    // `dLatest` / `lastKnown`, as views on the derivation.
    REQUIRE(store->findLatest().valid);
    REQUIRE(store->findLatest().captureTick == 70u);
    REQUIRE(store->findLatest().dA == 4u);
    REQUIRE(captureTagOf(store->fallback()) == Catch::Approx(70.f));

    netSync.unregisterSimulatable<SimulatableBrawler>(53u, &remote);
}

TEST_CASE("DAttack.SimulationNetSync.RemoteInputCachePopulatesAtBindWithoutALatch",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    // THE BIND-ORDER HOLE: the ring replicated (and its OnRep fired into a null
    // callback) BEFORE registration ran, and then the sender went quiet. Without
    // the bind-time populate the store would stay empty indefinitely.
    //
    // And note what is NOT needed to close it: no latch, no replay flag. Tier and
    // floor (T10/T11) needed those because they are change-notification-only
    // scalars. A replicated ring is a PERSISTENT property — re-reading it yields
    // the full current state, so THE PROPERTY IS ITS OWN LATCH.
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(54u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(54u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner remote;

    // Arrival BEFORE registration: the callback is still null, so this OnRep is a
    // no-op — exactly the production sequence.
    relayWrite(remote.relayedInputRing, 80u, 6u, 80.f);
    remote.replicateRelayRing();
    REQUIRE(remote.onRelayedInputReceived == nullptr);

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(54u, remote, nullptr);

    // Recovered at bind, with no further replication.
    auto* store = netSync.findRemoteInputCache<SimulatableBrawler>(54u);
    REQUIRE(store != nullptr);
    REQUIRE(store->findLatest().valid);
    REQUIRE(store->findLatest().captureTick == 80u);
    REQUIRE(store->findLatest().dA == 6u);

    netSync.unregisterSimulatable<SimulatableBrawler>(54u, &remote);
}

TEST_CASE("DAttack.SimulationNetSync.RemoteInputCacheFallsBackToTheGameZeroNotAValueInitialisedInput",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(55u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(55u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner remote;

    // Injection BEFORE registration (the composition-root order) — and the
    // order-independent path is covered by the re-injection below.
    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(55u, remote, nullptr);

    auto* store = netSync.findRemoteInputCache<SimulatableBrawler>(55u);
    REQUIRE(store != nullptr);
    REQUIRE_FALSE(store->findLatest().valid);

    // T7's rung 0. The value must be the GAME's zero — isGameZeroInput compares the
    // WHOLE aim vector, because a value-initialised PlayerInput also reads 0 on the
    // x component alone and would make this assertion vacuous.
    REQUIRE(isGameZeroInput(store->fallback()));
    REQUIRE_FALSE(isGameZeroInput(simulatableBrawler::PlayerInput{}));

    netSync.unregisterSimulatable<SimulatableBrawler>(55u, &remote);

    // ORDER-INDEPENDENCE: a store created BEFORE the injection is corrected by it,
    // the same guarantee setNeutralInput already gave the delay lines.
    SimulationNetSync<SimulatableBrawler> lateInjected(storage, reconciliation);
    MockPredictionOwner lateRemote;
    lateInjected.registerPredictionOwner<SimulatableBrawler>(55u, lateRemote, nullptr);
    REQUIRE_FALSE(isGameZeroInput(
        lateInjected.findRemoteInputCache<SimulatableBrawler>(55u)->fallback()));

    lateInjected.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    REQUIRE(isGameZeroInput(
        lateInjected.findRemoteInputCache<SimulatableBrawler>(55u)->fallback()));

    lateInjected.unregisterSimulatable<SimulatableBrawler>(55u, &lateRemote);
}

TEST_CASE("DAttack.SimulationNetSync.RemoteInputCacheSURVIVESAHardResyncWipe",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    // THE ruling this whole type exists for. wipeAllForResync sweeps the LOCAL
    // delay lines because their keys are the pre-resync prediction clock's ticks.
    // A relayed entry's key is the SENDER's capture tick — a server-domain identity
    // another machine produced — so a resync of OUR clock does not invalidate it,
    // and wiping it would blind every remote proxy for a window after every resync.
    //
    // This case drives the REAL wipeAllForResync with both kinds of character
    // registered, so a future edit that mirrors the delay-line loop onto the stores
    // fails here rather than in a playtest.
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(56u, makeNetSyncTestCharacter());   // local
    storage.add<SimulatableBrawler>(57u, makeNetSyncTestCharacter());   // remote

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(56u);
    reconciliation.createCacheFor<SimulatableBrawler>(57u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner local, remote;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(56u, local,
        [](const SimulationTimeStep& step, const BrawlerLocalInputCache&) {
            return taggedCapture(static_cast<float>(step.getTick()));
        });
    netSync.registerPredictionOwner<SimulatableBrawler>(57u, remote, nullptr);

    // Fill the local delay line with real captures...
    constexpr int32 kDelay = 2;
    netSync.setClientEffectiveInputDelayTicks(kDelay);
    for (unsigned int tick = 1u; tick <= 6u; ++tick)
        netSync.collectInputAll(SimulationTimeStep(tick, false, StepKind::Normal));

    // ...and the relay store with real relayed entries.
    relayWrite(remote.relayedInputRing, 90u, 3u, 90.f);
    remote.replicateRelayRing();

    auto* store = netSync.findRemoteInputCache<SimulatableBrawler>(57u);
    REQUIRE(store != nullptr);
    REQUIRE(store->has(90u));

    netSync.wipeAllForResync(500u);

    // THE ASSERTION: the relayed entry is untouched, stamp and value intact.
    std::uint8_t dA = 0u;
    simulatableBrawler::PlayerInput input;
    REQUIRE(store->find(90u, dA, input));
    REQUIRE(dA == 3u);
    REQUIRE(captureTagOf(input) == Catch::Approx(90.f));
    REQUIRE(store->findLatest().captureTick == 90u);

    // ...while the LOCAL delay line WAS wiped, which is what makes the assertion
    // above a contrast rather than a coincidence: the very next collect at a tick
    // whose capture existed before the wipe now reads the injected neutral.
    const auto inputs = netSync.collectInputAll(SimulationTimeStep(7u, false, StepKind::Normal));
    const auto& map = std::get<
        std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
    REQUIRE(isGameZeroInput(map.at(56u)));

    netSync.unregisterSimulatable<SimulatableBrawler>(56u, &local);
    netSync.unregisterSimulatable<SimulatableBrawler>(57u, &remote);
}

TEST_CASE("DAttack.SimulationNetSync.RelayRingVersionMismatchIsDroppedWholesale",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    // T5 is the FIRST reader of the ring's version byte. On a layout change an old
    // peer would otherwise read arbitrary bytes as capture ticks and insert them as
    // STORE KEYS — silent corruption with no crash. Driven here through the real
    // OnRep path so the fence is proven where it actually runs.
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(58u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(58u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner remote;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(58u, remote, nullptr);

    relayWrite(remote.relayedInputRing, 95u, 2u, 95.f);
    remote.relayedInputRing.bytes[relayedInputRing::kVersionOffset] =
        static_cast<std::uint8_t>(relayedInputRing::kWireFormatVersion + 1u);
    remote.replicateRelayRing();

    auto* store = netSync.findRemoteInputCache<SimulatableBrawler>(58u);
    REQUIRE(store != nullptr);
    REQUIRE_FALSE(store->has(95u));
    REQUIRE_FALSE(store->findLatest().valid);
    // ...and the proxy therefore still resolves the game zero rather than garbage.
    REQUIRE(isGameZeroInput(store->fallback()));

    netSync.unregisterSimulatable<SimulatableBrawler>(58u, &remote);
}

// ===========================================================================
// [og-netcode-v2-input-relay T17] THE AUTHORITY'S SUBSTITUTE IS THE GAME'S ZERO
// INPUT — through the REAL collectInputAll remote branch and the REAL
// sendCorrectionAll.
//
// THE DEFECT THESE PIN. `RemoteMoveQueue::dequeueMove()` returns a
// value-initialised `Move{}` on an empty queue, so the authority used to
// integrate — and replicate as "the input I applied" — a `PlayerInput{}`, whose
// (0,0,0) forward vectors are the exact value LocalInputCache.h documents as
// the one that "would be carried into normalisation and break". The game's real
// zero ((0,0,1) forwards, `getZeroPlayerInput`) was already injected on BOTH
// roles and simply unused on this path.
//
// NOT A LOSS-ONLY PATH, which is why this is worth its own cases: the remote
// branch runs from registerAuthorityOwner onward, so EVERY tick between
// server-side registration and the client's first input arriving is an underrun —
// every join window, in ordinary play, with no induced loss.
//
// [og-netcode-v2-input-relay T8] T17 NAMED THREE POISON SITES; ONE SURVIVES,
// BECAUSE THE OTHER TWO WERE THE CHANNEL.
//   1. the INTEGRATED value (collectInputAll's return, fed to integrateAll) —
//      ALIVE, and still pinned below. This is the site that decides what the
//      authority actually simulates.
//   2. `m_lastUsedInputs` on the same tick — GONE. That map existed only to be
//      replicated on the correction-INPUT channel; T8 retired both.
//   3. the REGISTRATION SEED of `m_lastUsedInputs` (AM-1) — GONE for the same
//      reason. Its whole argument was "it reaches peers before the first applied
//      input exists", which is a statement about a channel that no longer exists.
// So the case that pinned (3), `AuthorityRegistrationSeedsTheGameZeroInput`, is
// retired here, and the (2) assertions inside the surviving cases are removed.
// NOTHING ABOUT T17's ACTUAL FIX IS WEAKENED: the value the authority integrates
// on an underrun is still the injected game zero, still asserted with the
// anti-vacuity pairing, and still recovers on the next real arrival.
//
// ANTI-VACUITY. Every "it is the game zero" assertion is paired with "and it is
// NOT a value-initialised input". Those two are only different values because
// getZeroPlayerInput builds non-zero forward vectors; if they were ever made
// equal the assertions would still pass while testing nothing, so the pairing is
// what keeps these cases honest. `isGameZeroInput` (above) compares the whole aim
// vector for the same reason the tag alone was not enough for T9.
// ===========================================================================

namespace
{
    // FIELD-EXHAUSTIVE equality over the whole input composite. Every field of
    // every sub-input is compared — these are plain aggregates, so this is the
    // complete value, not a sample of it. Written rather than memcmp'd because
    // padding bytes are not part of the value and would make a passing case
    // depend on how the compiler laid the aggregates out.
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

    // A capture with EVERY field distinct from both the game zero and a
    // value-initialised input, so "passed through untouched" is a statement about
    // the whole composite rather than about the one tagged field.
    simulatableBrawler::PlayerInput richCapture()
    {
        simulatableBrawler::PlayerInput input = simulatableBrawler::getZeroPlayerInput();
        auto& radial = input.edit<dAttackRadialSimulation::PlayerInput>();
        radial.aimDirection = glm::vec3(0.6f, 0.f, 0.8f);
        radial.attackLeft   = true;
        radial.attackRight  = false;
        auto& machine = input.edit<dAttackMachineSimulation::PlayerInput>();
        machine.aimDirection       = glm::vec3(0.f, 0.8f, 0.6f);
        machine.attackLeft         = false;
        machine.attackRight        = true;
        machine.moveDirection      = glm::vec2(0.3f, -0.7f);
        machine.moveDirectionWorld = glm::vec3(0.3f, 0.f, -0.7f);
        machine.triggeredActionId  = 9u;
        input.edit<dAttackGuardSimulation::PlayerInput>().aimDirection = glm::vec3(1.f, 0.f, 0.f);
        input.edit<brawlerProjectileSimulation::PlayerInput>().aimDirection =
            glm::vec3(0.f, 1.f, 0.f);
        return input;
    }

    // THE ANTI-VACUITY GUARD ITSELF, asserted at the top of every case below: the
    // game zero and a value-initialised input must be DIFFERENT VALUES, or every
    // assertion in this block is trivially satisfiable.
    void requireGameZeroIsNotAValueInitialisedInput()
    {
        REQUIRE_FALSE(sameInput(simulatableBrawler::getZeroPlayerInput(),
                                simulatableBrawler::PlayerInput{}));
        REQUIRE_FALSE(isGameZeroInput(simulatableBrawler::PlayerInput{}));
    }
}

TEST_CASE("DAttack.SimulationNetSync.AuthorityUnderrunIntegratesTheGameZeroInput",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(60u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(60u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    // NEUTRAL FIRST — the authority path reads it once, at registration (see the
    // ordering assumption at registerAuthorityOwner).
    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(60u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(60u, authorityOwner);

    // SITE 1 — the integrated value. Nothing has ever arrived, so this tick
    // underruns: it is the join window, not a loss burst.
    const auto substituted = authorityTick(netSync, 60u, 1u);
    REQUIRE(sameInput(substituted, netSync.getNeutralInput<SimulatableBrawler>()));
    REQUIRE(isGameZeroInput(substituted));
    REQUIRE_FALSE(sameInput(substituted, simulatableBrawler::PlayerInput{}));

    // T2 REGRESSION GUARD: the substitution does not change the CLASSIFICATION.
    // No client capture stands behind the game zero either, so the ref stays the
    // sentinel — T6 resolves it, and it must not start looking like a real key.
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(60u) == kNoInputCaptureTick);

    // [T8] SITE 2's assertions stood here — the same value observed leaving the
    // server on the correction-INPUT channel (`authorityOwner.inputBuf.lastInput`).
    // Channel retired, observation point retired. What the authority publishes for
    // this tick is now the STATE plus the sentinel ref asserted immediately above.

    // A second consecutive underrun is substituted the same way — the fix is
    // per-tick, not a one-shot at registration.
    const auto substitutedAgain = authorityTick(netSync, 60u, 2u);
    REQUIRE(isGameZeroInput(substitutedAgain));
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(60u) == kNoInputCaptureTick);

    // ...and it RECOVERS: a real arrival is applied unchanged, so the substitute
    // is not latched over the client's own input.
    authorityOwner.onRemoteMoveReceived(70u, taggedCapture(70.f));
    const auto real = authorityTick(netSync, 60u, 3u);
    REQUIRE(captureTagOf(real) == Catch::Approx(70.f));
    REQUIRE_FALSE(isGameZeroInput(real));
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(60u) == 70u);

    netSync.unregisterSimulatable<SimulatableBrawler>(60u, &predictionOwner, &authorityOwner);
}

// [og-netcode-v2-input-relay T8] `AuthorityRegistrationSeedsTheGameZeroInput`
// (T17's SITE 3 / AM-1) STOOD HERE AND IS RETIRED WITH ITS SUBJECT.
//
// It registered an authority owner, called NO collectInputAll at all, ran
// sendCorrectionAll once, and asserted that what the server published was the
// registration SEED of `m_lastUsedInputs` — the game zero rather than a
// value-initialised input. Every clause of that sentence names something T8
// deleted: the seed, the map, and the publish. There is nothing left to observe,
// and no rewrite of the case could observe it, because a registered-but-unticked
// authority now publishes exactly one thing — the state, with the sentinel ref —
// and that assertion already exists (see `CorrectionStateRefIsPerCharacter` and
// the sentinel arm of the T4 ref case).
//
// The AM-1 DEFECT ITSELF cannot recur: it was "a value-initialised input reaches
// peers before any input is applied", and no input value reaches peers at all any
// more. What remains of AM-1's intent — that the composition root must inject the
// neutral on the authority role — is pinned by
// `AuthorityNeutralInputInjectionIsObservable` below, which still asserts the
// warning fires and still demonstrates the degradation it warns about.

// The other half of the fix: the NON-underrun arm must be untouched. A "fix" that
// substituted unconditionally, or that leaked the neutral into any field of a real
// input, passes both cases above and fails this one.
TEST_CASE("DAttack.SimulationNetSync.RemoteBranchPassesADequeuedInputThroughUntouched",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(62u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(62u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    netSync.registerPredictionOwner<SimulatableBrawler>(62u, predictionOwner, nullptr);
    netSync.registerAuthorityOwner<SimulatableBrawler>(62u, authorityOwner);

    const simulatableBrawler::PlayerInput sent = richCapture();
    // Every field distinct from both candidates for a substitution, so an equality
    // against `sent` cannot be satisfied by either of them.
    REQUIRE_FALSE(sameInput(sent, simulatableBrawler::getZeroPlayerInput()));
    REQUIRE_FALSE(sameInput(sent, simulatableBrawler::PlayerInput{}));

    authorityOwner.onRemoteMoveReceived(44u, sent);
    const auto applied = authorityTick(netSync, 62u, 1u);

    // The INTEGRATED value: every field, unchanged.
    REQUIRE(sameInput(applied, sent));
    REQUIRE_FALSE(sameInput(applied, netSync.getNeutralInput<SimulatableBrawler>()));
    // ...and the real join key, not the sentinel.
    REQUIRE(netSync.getLastUsedCaptureTick<SimulatableBrawler>(62u) == 44u);

    // [T8] A second arm asserted the same value through the retired
    // correction-INPUT channel (`inputBuf.lastInput == sent`). What the send path
    // still owes this case is that the join key survives to the wire, so assert
    // THAT instead — it is the surviving publication of "which input I applied".
    netSync.sendCorrectionAll(SimulationTimeStep(1u, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == 44u);
    REQUIRE_FALSE(authorityOwner.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);

    netSync.unregisterSimulatable<SimulatableBrawler>(62u, &predictionOwner, &authorityOwner);
}

// THE ROLE GUARD. The value the three cases above depend on is injected by the
// composition root, and before T17 the AUTHORITY branch of that injection was
// load-bearing only for a listen-server host — a comment was all that protected
// it. Now a dedicated server reads it on every underrun tick, so a regression that
// dropped the authority-side call would silently reinstate `PlayerInput{}` for the
// whole of every join window. These two SECTIONs pin the difference and the
// warning that makes it visible.
TEST_CASE("DAttack.SimulationNetSync.AuthorityNeutralInputInjectionIsObservable",
          "[DAttack][SimulationNetSync][InputRelay]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(63u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(63u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    MockAuthorityOwner  authorityOwner;

    std::vector<std::string> log;
    netSync.setLogger([&log](const char* msg) { log.emplace_back(msg); });

    const auto warnings = [&log]() {
        int n = 0;
        for (const auto& line : log)
            if (line.find("[Warning][NeutralInput]") != std::string::npos)
                ++n;
        return n;
    };

    SECTION("injection missing on the authority role is degraded AND warned")
    {
        // Exactly the regression: the composition root's authority-branch call
        // deleted, everything else identical.
        REQUIRE_FALSE(netSync.hasNeutralInput<SimulatableBrawler>());
        netSync.registerPredictionOwner<SimulatableBrawler>(63u, predictionOwner, nullptr);
        netSync.registerAuthorityOwner<SimulatableBrawler>(63u, authorityOwner);

        REQUIRE(warnings() == 1);

        // ...and the damage the warning describes is real, which is what makes the
        // warning worth having: the substitute the authority INTEGRATES degrades to
        // the value-initialised input. [T8] The seed half of this assertion (the
        // same degradation observed on the retired correction-input channel) is
        // gone; the integrated value is the site that still exists and it is the
        // one that matters — it is what the server simulates.
        const auto substituted = authorityTick(netSync, 63u, 1u);
        REQUIRE_FALSE(isGameZeroInput(substituted));
        REQUIRE(sameInput(substituted, simulatableBrawler::PlayerInput{}));
    }

    SECTION("injection present is silent and carries the game zero")
    {
        netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
        REQUIRE(netSync.hasNeutralInput<SimulatableBrawler>());
        netSync.registerPredictionOwner<SimulatableBrawler>(63u, predictionOwner, nullptr);
        netSync.registerAuthorityOwner<SimulatableBrawler>(63u, authorityOwner);

        // No warning — so the assertion above is a real discriminator and not a
        // line this code emits unconditionally.
        REQUIRE(warnings() == 0);

        // [T8] ...and the same tick, with the injection present, integrates the
        // game zero. Paired with the section above this is still a two-sided guard:
        // same code path, same tick, opposite injection state, opposite outcome.
        const auto substituted = authorityTick(netSync, 63u, 1u);
        REQUIRE(isGameZeroInput(substituted));
        REQUIRE_FALSE(sameInput(substituted, simulatableBrawler::PlayerInput{}));
    }

    netSync.unregisterSimulatable<SimulatableBrawler>(63u, &predictionOwner, &authorityOwner);
}

// ===========================================================================
// [T15] THE PROVIDER RECEIVES THE DELAY LINE, AND IT NEVER CONTAINS THE
// CURRENT TICK.
//
// This is the ordering that used to be an unwritten cross-file contract: the
// provider must not observe the tick it is being asked to produce. Before T15
// nothing at either site said so — it held only because collectInputAll's
// provider call happened to precede its localInputCache.push. Now the line arrives as
// a parameter and the two statements are adjacent in one function, so this case
// pins the property the code makes visible.
//
// It matters because the provider runs the motion matcher, whose rising-edge
// detection compares the live sample against "the previous tick". If the current
// tick were already resident, that comparison would read the current tick
// against itself and no edge would ever be reported.
//
// STALL COVERAGE: a Stall tick skips BOTH the delay-line push and the correction
// cache's input write, so the two history sources stay symmetric across one —
// a Stall re-entry at the same tick sees identical history either way.
// ===========================================================================
TEST_CASE("DAttack.SimulationNetSync.ProviderSeesHistoryStrictlyBeforeTheCurrentTick",
          "[DAttack][SimulationNetSync][MotionMatcherSource]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(70u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(70u);

    SimulationNetSync<SimulatableBrawler> netSync(storage, reconciliation);
    MockPredictionOwner predictionOwner;
    netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());

    // What the provider SAW, per tick, recorded from inside the real call.
    std::vector<uint32> observedTicks;
    std::vector<bool>   sawCurrentTick;
    std::vector<bool>   sawPreviousTick;
    std::vector<float>  previousTickTag;

    netSync.registerPredictionOwner<SimulatableBrawler>(70u, predictionOwner,
        [&](const SimulationTimeStep& step, const BrawlerLocalInputCache& line) {
            const auto tick = static_cast<int32>(step.getTick());
            observedTicks.push_back(step.getTick());
            sawCurrentTick.push_back(line.has(tick));
            sawPreviousTick.push_back(line.has(tick - 1));
            previousTickTag.push_back(
                line.has(tick - 1) ? captureTagOf(line.at(tick - 1)) : -1.f);
            return taggedCapture(static_cast<float>(step.getTick()));
        });

    constexpr unsigned int kFirst = 1u;
    constexpr unsigned int kLast  = 5u;
    for (unsigned int tick = kFirst; tick <= kLast; ++tick)
    {
        netSync.collectInputAll(SimulationTimeStep(tick, false, StepKind::Normal));
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false, StepKind::Normal));
    }

    REQUIRE(observedTicks.size() == (kLast - kFirst + 1u));

    for (size_t i = 0; i < observedTicks.size(); ++i)
    {
        CAPTURE(observedTicks[i]);
        // NEVER the current tick — the push is the statement after this call.
        CHECK(sawCurrentTick[i] == false);
        // ALWAYS the previous tick, once there has been one, and it carries the
        // RAW capture from that tick (not an already-delayed value).
        if (i == 0u)
        {
            CHECK(sawPreviousTick[i] == false);
        }
        else
        {
            CHECK(sawPreviousTick[i] == true);
            CHECK(previousTickTag[i] == Catch::Approx(static_cast<float>(observedTicks[i] - 1u)));
        }
    }

    // The same holds under a nonzero input delay: the line is CAPTURE-tick keyed,
    // so the delay changes which tick collectInputAll reads for the applied value
    // and nothing about what the provider can see.
    observedTicks.clear();
    sawCurrentTick.clear();
    sawPreviousTick.clear();
    previousTickTag.clear();

    netSync.setClientEffectiveInputDelayTicks(4);
    for (unsigned int tick = kLast + 1u; tick <= kLast + 4u; ++tick)
    {
        netSync.collectInputAll(SimulationTimeStep(tick, false, StepKind::Normal));
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false, StepKind::Normal));
    }

    REQUIRE(observedTicks.size() == 4u);
    for (size_t i = 0; i < observedTicks.size(); ++i)
    {
        CAPTURE(observedTicks[i]);
        CHECK(sawCurrentTick[i] == false);
        CHECK(sawPreviousTick[i] == true);
        CHECK(previousTickTag[i] == Catch::Approx(static_cast<float>(observedTicks[i] - 1u)));
    }
}

// ===========================================================================
// [T6] RESIM / CORRECTION INPUT RESOLUTION — THE RESOLUTION TABLE.
//
// collectResimInputAll RELOCATED from SimulationReconciliation to
// SimulationNetSync and stopped reading the correction cache's input VALUE. It
// now dispatches on TICK CLASS x CHARACTER CLASS:
//
//                    | LOCAL (provider present)   | REMOTE (proxy)
//   -----------------+----------------------------+---------------------------
//   Ref (corrected)  | localInputCache.at(ref)    | store.find(ref) -> input
//   Sentinel         | injected game zero         | injected game zero
//   ...store miss    | line miss -> its neutral   | store.fallback() (SELF-HEAL)
//   NoRef (frontier) | localInputCache.at(t - d)  | the scheduled read
//   NoSlot           | no entry at all
//
// EVERY CASE BELOW DRIVES THE REAL PATH end to end: the real collectInputAll to
// create the cache slots, the real OnRep-bound correction callback and
// injectCorrectionState to land the refs, the real RelayedInputRingCodec +
// populateRemoteInputCache to fill the stores, and the real
// collectResimInputAll to resolve. Nothing here mirrors production logic except
// the ONE deliberate transcription in the equivalence case, which is labelled as
// such because comparing against it is the entire point.
// ===========================================================================

namespace
{
    // storage + reconciliation + netsync, with the composition root's neutral
    // injection already done — so every delay line and relay store this rig later
    // creates is seeded with the GAME's zero rather than a value-initialised input.
    struct ResimRig
    {
        SimulationObjectStorage<SimulatableBrawler>   storage;
        SimulationReconciliation<SimulatableBrawler>  reconciliation{ storage };
        SimulationNetSync<SimulatableBrawler>         netSync{ storage, reconciliation };

        ResimRig()
        {
            netSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
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
            });
    }

    // A remote proxy: provider ABSENT, so it gets a relay store and no delay line.
    void addRemoteCharacter(ResimRig& rig, unsigned int id, MockPredictionOwner& owner)
    {
        rig.storage.add<SimulatableBrawler>(id, makeNetSyncTestCharacter());
        rig.reconciliation.createCacheFor<SimulatableBrawler>(id);
        rig.netSync.registerPredictionOwner<SimulatableBrawler>(id, owner, nullptr);
    }

    // One prediction tick through the REAL collectInputAll — which is what creates
    // the cache slot a correction can later land in, and what fills the delay line.
    void predictTick(ResimRig& rig, unsigned int tick)
    {
        const SimulationTimeStep step(tick, false, StepKind::Normal);
        rig.netSync.collectInputAll(step);
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
    std::optional<simulatableBrawler::PlayerInput> resimInputFor(
        SimulationNetSync<SimulatableBrawler>& netSync, unsigned int tick, unsigned int id)
    {
        auto inputs = netSync.collectResimInputAll(tick);
        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        const auto it = map.find(id);
        return it == map.end()
            ? std::nullopt
            : std::optional<simulatableBrawler::PlayerInput>(it->second);
    }
}

// --- CELL: corrected tick x LOCAL -> ownLocalInputCache.at(ref) -------------
// AC (a): the ref hit returns the input keyed by that CAPTURE IDENTITY, not by
// the resim tick and not by the current delay offset.
TEST_CASE("DAttack.SimulationNetSync.ResimCorrectedLocalReadsTheLocalInputCacheAtTheRef",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    ResimRig rig;
    MockPredictionOwner owner;
    addLocalCharacter(rig, 10u, owner);

    rig.netSync.setClientEffectiveInputDelayTicks(2);
    for (unsigned int tick = 1u; tick <= 8u; ++tick)
        predictTick(rig, tick);

    // The authority says: at tick 7 I applied the capture from tick 5.
    landCorrection(owner, 7u, 5u);

    const auto resolved = resimInputFor(rig.netSync, 7u, 10u);
    REQUIRE(resolved.has_value());
    REQUIRE(captureTagOf(*resolved) == Catch::Approx(5.f));
    // Not the resim tick's own capture — that is what a ref-blind implementation
    // returns.
    REQUIRE_FALSE(captureTagOf(*resolved) == Catch::Approx(7.f));

    // THE DISCRIMINATOR AGAINST A DELAY-DERIVED IMPLEMENTATION. The ref here does
    // NOT agree with `tick - currentEffectiveDelay` (which would say 4). The ref
    // is what the authority DID; the delay is only what we would have guessed.
    landCorrection(owner, 6u, 2u);
    const auto disagreeing = resimInputFor(rig.netSync, 6u, 10u);
    REQUIRE(disagreeing.has_value());
    REQUIRE(captureTagOf(*disagreeing) == Catch::Approx(2.f));
    REQUIRE_FALSE(captureTagOf(*disagreeing) == Catch::Approx(4.f));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(10u, &owner);
}

// --- CELL: corrected tick x REMOTE -> remoteInputCache.find(ref), dA IGNORED -
// AC (a) for the remote column, and half of AC (d): the entry's SCHEDULE STAMP
// is deliberately absurd, so an implementation that consulted it would reject a
// ref the authority already ruled on.
TEST_CASE("DAttack.SimulationNetSync.ResimCorrectedRemoteReadsTheStoreAtTheRefIgnoringTheStamp",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 20u, remote);

    for (unsigned int tick = 1u; tick <= 8u; ++tick)
        predictTick(rig, tick);

    // Two relayed entries arrive, depth-1 replace-latest: the ring holds only the
    // newest, but the STORE accumulates both. The first carries a stamp that
    // promises application at tick 45 — nowhere near the tick we resolve.
    relayWrite(remote.relayedInputRing, 5u, 40u, 5.f);
    remote.replicateRelayRing();
    relayWrite(remote.relayedInputRing, 6u, 3u, 6.f);
    remote.replicateRelayRing();

    landCorrection(remote, 7u, 5u);

    const auto resolved = resimInputFor(rig.netSync, 7u, 20u);
    REQUIRE(resolved.has_value());
    REQUIRE(captureTagOf(*resolved) == Catch::Approx(5.f));
    // Neither the newest entry nor a schedule-derived one.
    REQUIRE_FALSE(captureTagOf(*resolved) == Catch::Approx(6.f));
    REQUIRE_FALSE(isGameZeroInput(*resolved));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(20u, &remote);
}

// --- CELL (d): the LATE-RELEASE case, spelled out -------------------------
// RelayDelaySpectrumDesign.md §5.3 / T26: the server can release a capture LATE,
// so the intended schedule and the actual application disagree. The ref records
// what actually happened and MUST win.
TEST_CASE("DAttack.SimulationNetSync.ResimRefBeatsTheScheduleStampOnALateRelease",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 21u, remote);

    for (unsigned int tick = 1u; tick <= 14u; ++tick)
        predictTick(rig, tick);

    // Capture 6, stamped for application at 6+1 = 7.
    relayWrite(remote.relayedInputRing, 6u, 1u, 6.f);
    remote.replicateRelayRing();
    // ...and a newer capture, so a schedule-following implementation has something
    // else to reach for at tick 12.
    relayWrite(remote.relayedInputRing, 11u, 1u, 11.f);
    remote.replicateRelayRing();

    // But the authority released capture 6 LATE and applied it at tick 12.
    landCorrection(remote, 12u, 6u);

    const auto resolved = resimInputFor(rig.netSync, 12u, 21u);
    REQUIRE(resolved.has_value());
    REQUIRE(captureTagOf(*resolved) == Catch::Approx(6.f));
    // THE assertion: the scheduled read at tick 12 would probe 12-1 = 11 and hit.
    // A stamp-consulting resolution therefore returns 11 here and is wrong.
    REQUIRE_FALSE(captureTagOf(*resolved) == Catch::Approx(11.f));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(21u, &remote);
}

// --- CELLS: the SENTINEL ROW, both character classes -----------------------
// The authority substituted an input. T17 made that substitute the INJECTED GAME
// ZERO, so resolving to game zero here reproduces the authority exactly.
TEST_CASE("DAttack.SimulationNetSync.ResimSentinelResolvesGameZeroOnBothRows",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    ResimRig rig;
    MockPredictionOwner local;
    MockPredictionOwner remote;
    addLocalCharacter(rig, 30u, local);
    addRemoteCharacter(rig, 31u, remote);

    for (unsigned int tick = 1u; tick <= 6u; ++tick)
        predictTick(rig, tick);

    // Both characters are given data that a MIS-classified sentinel would return
    // instead: the local one has a capture at tick 5 in its line, and the remote
    // one has a relayed entry the scheduled read would find at tick 5.
    relayWrite(remote.relayedInputRing, 4u, 1u, 4.f);
    remote.replicateRelayRing();

    landCorrection(local,  5u, kNoInputCaptureTick);
    landCorrection(remote, 5u, kNoInputCaptureTick);

    const auto localResolved  = resimInputFor(rig.netSync, 5u, 30u);
    const auto remoteResolved = resimInputFor(rig.netSync, 5u, 31u);
    REQUIRE(localResolved.has_value());
    REQUIRE(remoteResolved.has_value());

    for (const auto* resolved : { &localResolved, &remoteResolved })
    {
        REQUIRE(isGameZeroInput(**resolved));
        REQUIRE(sameInput(**resolved, rig.netSync.getNeutralInput<SimulatableBrawler>()));
        // NEVER a value-initialised input — the exact poison T17 removed from the
        // authority path, which this row exists to reproduce.
        REQUIRE_FALSE(sameInput(**resolved, simulatableBrawler::PlayerInput{}));
    }

    // The discriminators, named: a NoRef mis-classification returns the tick-5
    // capture on the local row and the tick-4 relayed entry on the remote one.
    REQUIRE_FALSE(captureTagOf(*localResolved)  == Catch::Approx(5.f));
    REQUIRE_FALSE(captureTagOf(*remoteResolved) == Catch::Approx(4.f));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(30u, &local);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(31u, &remote);
}

// --- CELL: store miss x LOCAL -> the delay line's own injected neutral ------
TEST_CASE("DAttack.SimulationNetSync.ResimLocalRefMissResolvesTheInjectedNeutral",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    ResimRig rig;
    MockPredictionOwner local;
    addLocalCharacter(rig, 32u, local);

    for (unsigned int tick = 1u; tick <= 8u; ++tick)
        predictTick(rig, tick);

    // A ref naming a capture this client never took (post-resync is the only way
    // to reach this in production; here it is induced directly).
    landCorrection(local, 6u, 900u);

    const auto resolved = resimInputFor(rig.netSync, 6u, 32u);
    REQUIRE(resolved.has_value());
    REQUIRE(isGameZeroInput(*resolved));
    REQUIRE_FALSE(sameInput(*resolved, simulatableBrawler::PlayerInput{}));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(32u, &local);
}

// --- CELL: store miss x REMOTE -> lastKnown (THE SELF-HEAL) ----------------
// AC (b) rides along: a miss degrades THIS TICK's replay input and nothing else.
// The injected state, the stored ref and the store itself must come out of the
// resolution byte-for-byte unchanged.
TEST_CASE("DAttack.SimulationNetSync.ResimRemoteRefMissFallsBackToLastKnownWithoutCorruptingState",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 33u, remote);

    for (unsigned int tick = 1u; tick <= 10u; ++tick)
        predictTick(rig, tick);

    relayWrite(remote.relayedInputRing, 5u, 1u, 5.f);
    remote.replicateRelayRing();

    // The authority applied capture 7 — which was never relayed (an out-of-order
    // -older input is deliberately never relayed; §5.3a calls that a relay HOLE).
    landCorrection(remote, 8u, 7u);

    auto* store = rig.netSync.findRemoteInputCache<SimulatableBrawler>(33u);
    REQUIRE(store != nullptr);
    REQUIRE_FALSE(store->has(7u));

    const std::size_t residentBefore = store->residentCount();
    const uint32      latestBefore   = store->findLatest().captureTick;
    const auto        refBefore      =
        rig.reconciliation.getAppliedCaptureTick<SimulatableBrawler>(33u, 8u);

    const auto resolved = resimInputFor(rig.netSync, 8u, 33u);
    REQUIRE(resolved.has_value());
    // SELF-HEAL: last-known, not a neutral and not a dropped character.
    REQUIRE(captureTagOf(*resolved) == Catch::Approx(5.f));
    REQUIRE_FALSE(isGameZeroInput(*resolved));

    // AC (b): nothing the resolution touched was mutated by it. The state is a
    // complete anchor — the miss costs one tick's replay input, never the anchor.
    REQUIRE(store->residentCount() == residentBefore);
    REQUIRE(store->findLatest().captureTick == latestBefore);
    REQUIRE(rig.reconciliation.getAppliedCaptureTick<SimulatableBrawler>(33u, 8u) == refBefore);
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(33u, 8u).kind
            == AppliedCaptureRefKind::Ref);

    // ...and it is a pure read: resolving again answers identically.
    const auto again = resimInputFor(rig.netSync, 8u, 33u);
    REQUIRE(again.has_value());
    REQUIRE(sameInput(*again, *resolved));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(33u, &remote);
}

// --- CELL: frontier x LOCAL -> localInputCache.at(t - currentEffectiveDelay)
// Plus the NoSlot row, and a DEMONSTRATION of the accepted D2 offset-mixture edge
// (documented at the resolution site, deliberately not solved).
TEST_CASE("DAttack.SimulationNetSync.ResimFrontierLocalReDerivesFromTheCurrentDelay",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    ResimRig rig;
    MockPredictionOwner local;
    addLocalCharacter(rig, 34u, local);

    rig.netSync.setClientEffectiveInputDelayTicks(3);
    for (unsigned int tick = 1u; tick <= 10u; ++tick)
        predictTick(rig, tick);

    // No correction has landed anywhere, so every tick is NoRef.
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(34u, 9u).kind
            == AppliedCaptureRefKind::NoRef);

    const auto resolved = resimInputFor(rig.netSync, 9u, 34u);
    REQUIRE(resolved.has_value());
    REQUIRE(captureTagOf(*resolved) == Catch::Approx(6.f));      // 9 - 3

    // THE NoSlot ROW: a tick this character was never predicted at produces NO
    // ENTRY, rather than a neutral. integrateAll skips ids absent from the map,
    // and prepareResimAll likewise refuses to restore state for such a tick — so
    // an entry here would integrate an un-restored state.
    REQUIRE_FALSE(resimInputFor(rig.netSync, 4000u, 34u).has_value());

    // THE ACCEPTED D2 EDGE, demonstrated rather than asserted away: re-deriving
    // uses the CURRENT delay, so the same frontier tick resolves to a DIFFERENT
    // capture after a delay change than the one originally integrated there. This
    // is the offset-mixture exposure the task records at the code site and
    // explicitly does not fix; the every-frame state anchor supersedes it.
    rig.netSync.setClientEffectiveInputDelayTicks(1);
    const auto afterDelayChange = resimInputFor(rig.netSync, 9u, 34u);
    REQUIRE(afterDelayChange.has_value());
    REQUIRE(captureTagOf(*afterDelayChange) == Catch::Approx(8.f));   // 9 - 1
    REQUIRE_FALSE(captureTagOf(*afterDelayChange) == Catch::Approx(6.f));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(34u, &local);
}

// --- CELL: frontier x REMOTE -> the T7 scheduled read ----------------------
// All three rungs of the ladder, each with the value the OTHER rungs would have
// produced named so the case cannot pass by coincidence.
TEST_CASE("DAttack.SimulationNetSync.ResimFrontierRemoteRunsTheScheduledRead",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 35u, remote);

    for (unsigned int tick = 1u; tick <= 20u; ++tick)
        predictTick(rig, tick);

    // RUNG 0 — nothing has ever arrived: terminal fallback, and specifically the
    // INJECTED game zero rather than a value-initialised input.
    const auto empty = resimInputFor(rig.netSync, 9u, 35u);
    REQUIRE(empty.has_value());
    REQUIRE(isGameZeroInput(*empty));
    REQUIRE_FALSE(sameInput(*empty, simulatableBrawler::PlayerInput{}));

    // RUNG 1+2 — a scheduled HIT: dLatest = 3, so tick 9 probes capture 6, whose
    // own stamp is also 3. The regime has not shifted; take the entry.
    relayWrite(remote.relayedInputRing, 6u, 3u, 6.f);
    remote.replicateRelayRing();
    const auto scheduled = resimInputFor(rig.netSync, 9u, 35u);
    REQUIRE(scheduled.has_value());
    REQUIRE(captureTagOf(*scheduled) == Catch::Approx(6.f));

    // VERIFY-FAIL — the delay regime shifted. A newer entry stamped 5 makes
    // dLatest 5, so tick 11 probes capture 6 and HITS, but capture 6 carries the
    // OLD stamp of 3. Mismatch => the schedule is in transition => fall back to
    // last-known rather than guess.
    relayWrite(remote.relayedInputRing, 9u, 5u, 9.f);
    remote.replicateRelayRing();
    const auto verifyFail = resimInputFor(rig.netSync, 11u, 35u);
    REQUIRE(verifyFail.has_value());
    REQUIRE(captureTagOf(*verifyFail) == Catch::Approx(9.f));       // last-known
    // A verify-less implementation returns the probed entry instead.
    REQUIRE_FALSE(captureTagOf(*verifyFail) == Catch::Approx(6.f));

    // RUNG 3 — the probe MISSES outright: tick 20 probes capture 15, unoccupied.
    const auto probeMiss = resimInputFor(rig.netSync, 20u, 35u);
    REQUIRE(probeMiss.has_value());
    REQUIRE(captureTagOf(*probeMiss) == Catch::Approx(9.f));        // last-known
    REQUIRE_FALSE(isGameZeroInput(*probeMiss));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(35u, &remote);
}

// --- AC (e): the relay store SURVIVES a hard resync ------------------------
// T5's ruling, observed from the resolution layer: remote proxies keep resolving
// through the wipe window while local characters go neutral exactly as they did
// pre-T6. Strictly better than the old behaviour, which blinded both.
TEST_CASE("DAttack.SimulationNetSync.ResimRemoteStillResolvesAfterAHardResyncWipe",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    ResimRig rig;
    MockPredictionOwner local;
    MockPredictionOwner remote;
    addLocalCharacter(rig, 40u, local);
    addRemoteCharacter(rig, 41u, remote);

    rig.netSync.setClientEffectiveInputDelayTicks(2);
    for (unsigned int tick = 1u; tick <= 8u; ++tick)
        predictTick(rig, tick);

    relayWrite(remote.relayedInputRing, 5u, 1u, 5.f);
    remote.replicateRelayRing();

    // HARD RESYNC — both wipes, in the order the clock callback fires them.
    constexpr unsigned int kNewTick = 100u;
    rig.netSync.wipeAllForResync(kNewTick);
    rig.reconciliation.wipeAllForResync(kNewTick);

    // The relay store is untouched by the wipe: its keys are the SENDER's capture
    // ticks, which our clock jumping does not invalidate.
    auto* store = rig.netSync.findRemoteInputCache<SimulatableBrawler>(41u);
    REQUIRE(store != nullptr);
    REQUIRE(store->has(5u));

    for (unsigned int tick = kNewTick; tick <= kNewTick + 4u; ++tick)
        predictTick(rig, tick);

    // A correction under the NEW numbering, naming a capture from BEFORE the
    // resync — legitimate, because a capture tick is the sender's identity.
    landCorrection(remote, kNewTick + 3u, 5u);
    const auto remoteResolved = resimInputFor(rig.netSync, kNewTick + 3u, 41u);
    REQUIRE(remoteResolved.has_value());
    REQUIRE(captureTagOf(*remoteResolved) == Catch::Approx(5.f));

    // The LOCAL character's line WAS wiped, so the same ref misses and resolves to
    // the injected neutral — unchanged pre-T6 behaviour through the wipe window.
    landCorrection(local, kNewTick + 3u, 5u);
    const auto localResolved = resimInputFor(rig.netSync, kNewTick + 3u, 40u);
    REQUIRE(localResolved.has_value());
    REQUIRE(isGameZeroInput(*localResolved));
    REQUIRE_FALSE(sameInput(*localResolved, simulatableBrawler::PlayerInput{}));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(40u, &local);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(41u, &remote);
}

// ===========================================================================
// [og-netcode-v2-input-relay T8] AC (c) — "ResimResolutionIsByteIdenticalToThe
// OldAppliedTickLookup" — STOOD HERE AND IS RETIRED WITH ITS COUNTERPARTY.
//
// WHAT IT WAS. A MIGRATION PROOF. It transcribed the pre-T6 resim body
// (`cache.getInput(cache.getCacheIndex(simTick))`) as `legacyResimInputAll`,
// drove BOTH server channels for six consecutive ticks — the state (carrying T4's
// ref) and the old correction-INPUT — and asserted that T6's capture-tick
// resolution reproduced the old applied-tick lookup whole-map, per tick, with
// absolute per-tick values pinned against mutual-agreement.
//
// WHY IT CANNOT SURVIVE T8, and why that is not a coverage loss. The comparison
// needs the legacy counterparty to be REACHABLE, and the counterparty is
// "whatever the server wrote into the cache's input column". T8 deletes the only
// writer of that column from the server side (sendCorrectionAll's input write ->
// insertCorrectionInput). Post-T8 the column holds the CLIENT's own prediction,
// so `legacyResimInputAll` would no longer be reading the pre-T6 implementation
// at all — it would be reading a different thing that happens to compile, and the
// case would either fail (for the remote row, whose column now holds the
// prediction-time value rather than the authority's) or, if hand-fed, would be
// asserting agreement with a fabrication. A migration test cannot outlive the
// thing it migrates from.
//
// WHAT STILL GUARDS RESOLUTION CORRECTNESS — the other TEN cases of this
// [ResimResolution] tag, all present, all unchanged, none of which ever touched
// the retired channel:
//   corrected rows      ResimCorrectedLocalReadsTheLocalInputCacheAtTheRef
//                       ResimCorrectedRemoteReadsTheStoreAtTheRefIgnoringTheStamp
//   precedence          ResimRefBeatsTheScheduleStampOnALateRelease
//   sentinel rows       ResimSentinelResolvesGameZeroOnBothRows
//                       ResimSentinelMatchesWhatTheAuthorityIntegrated  (AM-2 —
//                       drives a REAL server underrun through the REAL
//                       sendCorrectionAll into the REAL OnRep, and is untouched
//                       because it rides the STATE channel and its sentinel ref)
//   miss / self-heal    ResimLocalRefMissResolvesTheInjectedNeutral
//                       ResimRemoteRefMissFallsBackToLastKnownWithoutCorrupting-
//                       State
//   frontier rows       ResimFrontierLocalReDerivesFromTheCurrentDelay
//                       ResimFrontierRemoteRunsTheScheduledRead  (which also
//                       carries the NoSlot row and the D2 offset-mixture demo)
//   resync survival     ResimRemoteStillResolvesAfterAHardResyncWipe
// Every property above is asserted DIRECTLY — absolute per-tick values against
// the resolution table — rather than by comparison against a second
// implementation. That is precisely why they carry forward and this one does not.
// ===========================================================================

// ===========================================================================
// AM-2 — THE T17 INTERLOCK, END TO END THROUGH THE REAL AUTHORITY PATH.
//
// Not a client-side unit assertion about a constant: this drives a real server
// underrun, replicates the sentinel ref through the real sendCorrectionAll into
// the real OnRep callback, and asserts the client's resolution equals the value
// the SERVER ACTUALLY INTEGRATED. If T17's authority-side substitution is ever
// reverted or drifts away from what this row resolves, this case fails — which is
// the whole reason it survives T17 rather than being retired with it.
// ===========================================================================
TEST_CASE("DAttack.SimulationNetSync.ResimSentinelMatchesWhatTheAuthorityIntegrated",
          "[DAttack][SimulationNetSync][ResimResolution]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(80u, makeNetSyncTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(80u);

    // TWO netsyncs over one storage: the SERVER half (authority owner, remote
    // branch) and the CLIENT half (a proxy for the same character). One rig cannot
    // play both roles — collectInputAll's authority branch never pushes the
    // prediction ticks a correction needs a slot in.
    SimulationNetSync<SimulatableBrawler> serverNetSync(storage, reconciliation);
    SimulationNetSync<SimulatableBrawler> clientNetSync(storage, reconciliation);
    MockAuthorityOwner  authorityOwner;
    MockPredictionOwner predictionOwner;

    // NEUTRAL FIRST on the server — the authority path reads it once, at
    // registration (the ordering assumption stated at registerAuthorityOwner).
    serverNetSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    clientNetSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    serverNetSync.registerAuthorityOwner<SimulatableBrawler>(80u, authorityOwner);
    clientNetSync.registerPredictionOwner<SimulatableBrawler>(80u, predictionOwner, nullptr);

    constexpr unsigned int kTick = 12u;
    for (unsigned int tick = 10u; tick <= kTick; ++tick)
        clientNetSync.collectInputAll(SimulationTimeStep(tick, false, StepKind::Normal));

    // 1) THE SERVER UNDERRUNS. Nothing has ever arrived for this character, so
    //    this is an ordinary join window, not induced loss.
    const auto serverIntegrated = authorityTick(serverNetSync, 80u, kTick);
    REQUIRE(isGameZeroInput(serverIntegrated));
    REQUIRE_FALSE(sameInput(serverIntegrated, simulatableBrawler::PlayerInput{}));

    // 2) THE SENTINEL REF REPLICATES, through the real send path.
    serverNetSync.sendCorrectionAll(SimulationTimeStep(kTick, false, StepKind::Normal), kEveryFrameRotationK);
    REQUIRE(authorityOwner.stateBuf.lastTick == kTick);
    REQUIRE(authorityOwner.stateBuf.lastAppliedCaptureTick == kNoInputCaptureTick);

    // 3) ...and lands on the client. The buffer the SERVER wrote is handed to the
    //    client's OnRep callback verbatim — no value is retyped across the wire.
    REQUIRE(predictionOwner.onCorrectionStateReceived != nullptr);
    predictionOwner.onCorrectionStateReceived(authorityOwner.stateBuf);
    REQUIRE(reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(80u, kTick).kind
            == AppliedCaptureRefKind::Sentinel);

    // 4) THE CLIENT RESOLVES GAME ZERO — and, the assertion that makes this an
    //    interlock rather than two independent unit tests, the SAME VALUE the
    //    authority integrated.
    const auto clientResolved = resimInputFor(clientNetSync, kTick, 80u);
    REQUIRE(clientResolved.has_value());
    REQUIRE(isGameZeroInput(*clientResolved));
    REQUIRE(sameInput(*clientResolved, serverIntegrated));
    REQUIRE_FALSE(sameInput(*clientResolved, simulatableBrawler::PlayerInput{}));

    serverNetSync.unregisterSimulatable<SimulatableBrawler>(80u, nullptr, &authorityOwner);
    clientNetSync.unregisterSimulatable<SimulatableBrawler>(80u, &predictionOwner);
}

// ===========================================================================
// [og-netcode-v2-input-relay T7] REMOTE-PROXY PREDICTION — THE UNIFIED
// SCHEDULED READ, through the REAL collectInputAll proxy branch.
//
// WHAT MOVED. The branch used to answer `getLastCorrectionInput().value_or(
// InputType{})`: hold the server's last reported input, and — when no correction
// had ever landed — a VALUE-INITIALISED PlayerInput, the (0,0,0)-forward poison
// T17 removed from the authority path. It now runs
// `resolveScheduledRelayedInput(store, N)`, the SAME free function T6's resim
// frontier row runs, whose terminal value is the INJECTED game zero.
//
// WHY THESE CASES LIVE HERE. The ladder itself is unit-tested against a bare
// store in og-simulation-tests, and its resim caller in the [ResimResolution]
// block above. What neither reaches is THIS branch: collectInputAll is variadic
// over a simulatable pack and needs SimulatableOwnerTraits bound to concrete
// owners, so only a suite linking a real simulatable can drive the shipped path.
//
// THE PROPERTY THE WHOLE BLOCK IS ABOUT — ONE CODE PATH, NO REGIME FLAG. There
// is no branch anywhere below that selects "degenerate" or "scheduled" mode. The
// regime is decided per input, per receiver, by whether the probe finds data:
// FloorZeroDegenerates... and ScheduledRegimeConsumes... exercise the two ends of
// the spectrum through the identical production code, and
// StampMismatch...FallsBackToLastKnown shows the transition between them healing
// itself with no intervention.
// ===========================================================================

namespace
{
    // The resolved PREDICTION input for one character at `tick`, through the REAL
    // collectInputAll. There is no NoSlot row here (unlike resimInputFor above):
    // the proxy branch emplaces for every id it sees, so a missing key is a
    // failure rather than a meaningful answer.
    simulatableBrawler::PlayerInput proxyInputFor(
        SimulationNetSync<SimulatableBrawler>& netSync, unsigned int tick, unsigned int id)
    {
        const auto inputs = netSync.collectInputAll(
            SimulationTimeStep(tick, false, StepKind::Normal));
        const auto& map = std::get<
            std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
        const auto it = map.find(id);
        REQUIRE(it != map.end());
        return it->second;
    }

    // THE ORACLE for the degenerate-equivalence case: a LAST-KNOWN-ONLY proxy
    // implementation — "hold the newest thing that arrived, else the injected
    // zero" — which is what the increment-1 design specified and what the pre-T7
    // branch approximated by holding the newest CORRECTION instead.
    //
    // Written against `findLatest()` + `getNeutralInput()` rather than by calling
    // `fallback()`: `fallback()` is a component OF the ladder under test, so
    // comparing the ladder to it would be comparing the implementation to itself.
    simulatableBrawler::PlayerInput lastKnownOnly(
        const RemoteInputCache<simulatableBrawler::PlayerInput>& store)
    {
        const auto latest = store.findLatest();
        return latest.valid ? latest.input : store.getNeutralInput();
    }
}

// --- RUNG 0: the D4 idle window, and the (0,0,0) poison this task removes ---
TEST_CASE("DAttack.SimulationNetSync.ProxyEmptyStoreIntegratesTheGameZeroNotAValueInitialisedInput",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    // ANTI-VACUITY FIRST, the shape T17 established: if the game zero and a
    // value-initialised input were ever made equal, every assertion below would be
    // trivially satisfiable and this case would stop protecting anything.
    requireGameZeroIsNotAValueInitialisedInput();

    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 60u, remote);

    // A character present in STORAGE but never REGISTERED. collectInputAll
    // iterates storage, so the proxy branch really does see ids like this one
    // (T5's accessor is nullable for exactly this reason), and it must answer the
    // same injected neutral rather than throwing or inventing InputType{}.
    rig.storage.add<SimulatableBrawler>(61u, makeNetSyncTestCharacter());
    rig.reconciliation.createCacheFor<SimulatableBrawler>(61u);

    // NOTE what the pre-T7 branch did here: `getLastCorrectionInput` returned
    // nullopt (no correction has ever landed in a session this young), so
    // `.value_or(InputType{})` handed the integrator a (0,0,0)-forward input on
    // EVERY tick of this window — not a loss-only path, an ordinary join.
    const auto coldStore = proxyInputFor(rig.netSync, 7u, 60u);
    REQUIRE(isGameZeroInput(coldStore));
    REQUIRE(sameInput(coldStore, rig.netSync.getNeutralInput<SimulatableBrawler>()));
    REQUIRE_FALSE(sameInput(coldStore, simulatableBrawler::PlayerInput{}));

    const auto noStore = proxyInputFor(rig.netSync, 8u, 61u);
    REQUIRE(isGameZeroInput(noStore));
    REQUIRE(sameInput(noStore, rig.netSync.getNeutralInput<SimulatableBrawler>()));
    REQUIRE_FALSE(sameInput(noStore, simulatableBrawler::PlayerInput{}));

    // SENTINEL AWARENESS (T6's rule, observed from this branch). The relay path
    // only ever carries real capture ticks, but the store refuses the sentinel as
    // a key outright — so even a feeder that tried to relay one leaves the store
    // empty, and the proxy resolves the injected zero rather than a successful
    // lookup of a value-initialised slot.
    relayWrite(remote.relayedInputRing, kNoInputCaptureTick, 0u, 123.f);
    remote.replicateRelayRing();
    auto* store = rig.netSync.findRemoteInputCache<SimulatableBrawler>(60u);
    REQUIRE(store != nullptr);
    REQUIRE_FALSE(store->findLatest().valid);

    const auto afterSentinel = proxyInputFor(rig.netSync, 9u, 60u);
    REQUIRE(isGameZeroInput(afterSentinel));
    REQUIRE_FALSE(sameInput(afterSentinel, simulatableBrawler::PlayerInput{}));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(60u, &remote);
}

// --- THE SCHEDULED REGIME: the proxy consumes the server's schedule ---------
TEST_CASE("DAttack.SimulationNetSync.ProxyScheduledRegimeConsumesTheServersScheduleTickForTick",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 62u, remote);

    // A HIGH-FLOOR wire: the server holds five ticks of delay for this sender and
    // stamps every relayed entry with it, so `captureTick + 5` is the tick the
    // authority intends to apply it at.
    constexpr std::uint8_t kSchedule = 5u;
    for (uint32 capture = 20u; capture <= 25u; ++capture)
    {
        relayWrite(remote.relayedInputRing, capture, kSchedule, static_cast<float>(capture));
        remote.replicateRelayRing();
    }

    // TICK FOR TICK. dLatest is 5 and every probed entry carries the same stamp,
    // so tick N resolves capture N-5 — the input the authority will apply at N.
    std::vector<float> resolvedTags;
    for (unsigned int tick = 25u; tick <= 30u; ++tick)
    {
        const auto resolved = proxyInputFor(rig.netSync, tick, 62u);
        // ...and none of them is the injected neutral, so "advancing" below is not
        // an artefact of falling back to a different empty answer.
        REQUIRE_FALSE(isGameZeroInput(resolved));
        resolvedTags.push_back(captureTagOf(resolved));
    }

    // (Named local, not a braced temporary inside the macro — REQUIRE's argument
    // is preprocessed, and braces do not protect commas the way parentheses do.)
    const std::vector<float> expectedTags{ 20.f, 21.f, 22.f, 23.f, 24.f, 25.f };
    REQUIRE(resolvedTags == expectedTags);

    // THE DISCRIMINATOR, spelled out rather than left implicit: a last-known-only
    // implementation answers the SAME value at all six ticks, because the store
    // stopped changing before the loop began. Only a schedule-consuming read
    // advances with the tick.
    auto* store = rig.netSync.findRemoteInputCache<SimulatableBrawler>(62u);
    REQUIRE(store != nullptr);
    REQUIRE(captureTagOf(lastKnownOnly(*store)) == Catch::Approx(25.f));
    REQUIRE(resolvedTags.front() != resolvedTags.back());

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(62u, &remote);
}

// --- THE VERIFY STEP: a delay-regime change falls back, then self-heals -----
TEST_CASE("DAttack.SimulationNetSync.ProxyStampMismatchInADelayTransitionFallsBackToLastKnown",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 63u, remote);

    for (uint32 capture = 20u; capture <= 24u; ++capture)
    {
        relayWrite(remote.relayedInputRing, capture, 5u, static_cast<float>(capture));
        remote.replicateRelayRing();
    }

    // THE REGIME SHIFTS: the sender's tier (or the floor) moved, so the server
    // stamps the next entry with 8. The authority applies ONE current delay to
    // every parked entry, so the freshest stamp is the best estimate of it — and
    // the older entries' stamps now describe a schedule nobody is using.
    relayWrite(remote.relayedInputRing, 25u, 8u, 25.f);
    remote.replicateRelayRing();

    // dLatest is 8, so tick 30 probes capture 22 — which IS resident, and would be
    // accepted by a verify-less ladder. Its stamp is the OLD 5. Mismatch means
    // "the schedule shifted under me", so fall back rather than guess.
    const auto transitioning = proxyInputFor(rig.netSync, 30u, 63u);
    REQUIRE(captureTagOf(transitioning) == Catch::Approx(25.f));            // last-known
    REQUIRE_FALSE(captureTagOf(transitioning) == Catch::Approx(22.f));      // verify-less

    // ...AND IT HEALS ITSELF, with no flag and no intervention: once the new
    // regime has produced entries of its own, the probe hits again on stamps that
    // agree. Tick 36 probes capture 28, stamped 8 like dLatest.
    for (uint32 capture = 26u; capture <= 30u; ++capture)
    {
        relayWrite(remote.relayedInputRing, capture, 8u, static_cast<float>(capture));
        remote.replicateRelayRing();
    }

    const auto healed = proxyInputFor(rig.netSync, 36u, 63u);
    REQUIRE(captureTagOf(healed) == Catch::Approx(28.f));
    REQUIRE_FALSE(captureTagOf(healed) == Catch::Approx(30.f));             // not last-known

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(63u, &remote);
}

// ===========================================================================
// THE HEADLINE AC — DEGENERATE EQUIVALENCE AT FLOOR 0, AND ITS EXACT SCOPE.
//
// The claim is STATISTICAL, not absolute (review A4), and this case is pinned to
// the conditions under which it IS absolute: an EMPTY or BEHIND store, i.e. the
// receiver's frontier running ahead of the newest capture that has reached it —
// which is the ordinary floor-0 situation, because with no scheduled delay
// nothing was ever arranged to make the sender's capture for tick N arrive in
// time for tick N.
//
// The sibling case below pins the OTHER half deliberately: a schedule-satisfying
// hit at floor 0 legitimately DIFFERS from last-known, and is a non-regression.
// Asserting absolute equivalence at floor 0 would be wrong, and would fail for
// the right reasons.
// ===========================================================================
TEST_CASE("DAttack.SimulationNetSync.ProxyAtFloorZeroDegeneratesToLastKnownWhenTheStoreIsEmptyOrBehind",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 64u, remote);

    auto* store = rig.netSync.findRemoteInputCache<SimulatableBrawler>(64u);
    REQUIRE(store != nullptr);

    // FLOOR 0 => every entry is stamped dA = 0, and the captures that reach us lag
    // our own prediction tick by the wire. Ticks 100..109; the sender's captures
    // arrive around 90..94, so the probe (N - 0 = N) is always unoccupied.
    struct Arrival { unsigned int atTick; uint32 capture; };
    const std::vector<Arrival> arrivals{
        { 102u, 90u }, { 104u, 92u }, { 107u, 94u },
    };

    std::vector<float> resolvedTags;
    std::vector<float> oracleTags;

    for (unsigned int tick = 100u; tick <= 109u; ++tick)
    {
        for (const Arrival& a : arrivals)
        {
            if (a.atTick != tick)
                continue;
            relayWrite(remote.relayedInputRing, a.capture, 0u, static_cast<float>(a.capture));
            remote.replicateRelayRing();
        }

        // The oracle is evaluated on the SAME store state the branch is about to
        // read, immediately before the read.
        const auto expected = lastKnownOnly(*store);
        const auto resolved = proxyInputFor(rig.netSync, tick, 64u);

        // WHOLE-COMPOSITE equality, not the tag alone.
        REQUIRE(sameInput(resolved, expected));

        // The pre-arrival ticks are the D4 window, and their agreement must be on
        // the INJECTED zero: the tag alone reads 0 for a value-initialised input
        // too (the trap kNeutralTag exists for), so both sides are checked on the
        // whole aim vector here.
        if (tick < arrivals.front().atTick)
        {
            REQUIRE(isGameZeroInput(resolved));
            REQUIRE_FALSE(sameInput(resolved, simulatableBrawler::PlayerInput{}));
        }
        else
        {
            REQUIRE_FALSE(isGameZeroInput(resolved));
        }

        resolvedTags.push_back(captureTagOf(resolved));
        oracleTags.push_back(captureTagOf(expected));
    }

    // ABSOLUTE PIN, so a mutual-agreement implementation (both sides constant, or
    // both off by one arrival) cannot ride through. Ticks 100-101 precede the
    // first arrival and are the D4 window; the rest hold the newest capture.
    const std::vector<float> expectedTags{
        kNeutralTag, kNeutralTag, 90.f, 90.f, 92.f, 92.f, 92.f, 94.f, 94.f, 94.f };
    REQUIRE(resolvedTags == expectedTags);
    REQUIRE(oracleTags == resolvedTags);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(64u, &remote);
}

TEST_CASE("DAttack.SimulationNetSync.ProxyAtFloorZeroLegitimatelyDiffersOnAScheduleSatisfyingHit",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    // THE OTHER HALF OF THE HEADLINE AC, and the reason it is worded
    // statistically. A mixed pair — a fast sender against a receiver whose lead
    // has wobbled DOWN inside the dead band — can have the sender's capture for
    // our tick N already resident. The probe then HITS at floor 0, and the answer
    // is the server's ACTUAL scheduled input rather than a stale hold.
    //
    // THAT IS A NON-REGRESSION, NOT A BUG: with dA = 0 the authority applies
    // capture N at tick N, so the hit reproduces the authority exactly while
    // last-known would run a tick early. This case exists so that a future reader
    // who finds floor-0 output differing from the old design does not "fix" it.
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 65u, remote);

    for (uint32 capture = 40u; capture <= 42u; ++capture)
    {
        relayWrite(remote.relayedInputRing, capture, 0u, static_cast<float>(capture));
        remote.replicateRelayRing();
    }

    auto* store = rig.netSync.findRemoteInputCache<SimulatableBrawler>(65u);
    REQUIRE(store != nullptr);

    const auto resolved = proxyInputFor(rig.netSync, 41u, 65u);
    REQUIRE(captureTagOf(resolved) == Catch::Approx(41.f));                  // the schedule
    REQUIRE(captureTagOf(lastKnownOnly(*store)) == Catch::Approx(42.f));     // the old design
    REQUIRE_FALSE(sameInput(resolved, lastKnownOnly(*store)));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(65u, &remote);
}

// --- THE SHARED-LADDER INTERLOCK -------------------------------------------
// Prediction and resim MUST resolve an uncorrected remote tick identically. If
// they ever diverge, a resim disagrees with the prediction it is replaying and
// manufactures divergence out of nothing — which is why T6 extracted the ladder
// as a free function and why T7 calls it rather than inlining a second copy.
// Two arms, a HIT and a FALLBACK, so the agreement cannot be an artefact of both
// branches landing on the same empty answer.
TEST_CASE("DAttack.SimulationNetSync.ProxyPredictionAndResimResolveTheSameTickIdentically",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 66u, remote);

    for (unsigned int tick = 1u; tick <= 11u; ++tick)
        predictTick(rig, tick);

    // ARM 1 — a scheduled HIT. dLatest = 3, so tick 12 probes capture 9.
    relayWrite(remote.relayedInputRing, 9u, 3u, 9.f);
    remote.replicateRelayRing();

    const auto predicted = proxyInputFor(rig.netSync, 12u, 66u);
    const auto replayed  = resimInputFor(rig.netSync, 12u, 66u);
    REQUIRE(replayed.has_value());
    REQUIRE(sameInput(predicted, *replayed));
    REQUIRE(captureTagOf(predicted) == Catch::Approx(9.f));
    REQUIRE_FALSE(isGameZeroInput(predicted));

    // ARM 2 — the probe MISSES and both fall back to last-known. Tick 13 probes
    // capture 10, which never arrived.
    const auto predictedMiss = proxyInputFor(rig.netSync, 13u, 66u);
    const auto replayedMiss  = resimInputFor(rig.netSync, 13u, 66u);
    REQUIRE(replayedMiss.has_value());
    REQUIRE(sameInput(predictedMiss, *replayedMiss));
    REQUIRE(captureTagOf(predictedMiss) == Catch::Approx(9.f));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(66u, &remote);
}

// ===========================================================================
// THE VIZ RE-POINT — getLastRelayedInput, and the rule that consumes it.
//
// Both named remote-proxy viz consumers move off the correction-input channel in
// this task: BrawlerVisualizationInputSource's REMOTE branch, and the
// SimmableUpdateComponent site that feeds it (which used to pass
// SimulationReconciliation::getLatestInput). The UE call site itself is not
// linkable here; what IS linkable — and what actually carries the semantics — is
// the netsync accessor that replaces the source, and the pure rule that consumes
// it. This case drives both, composed exactly as production composes them.
// ===========================================================================
TEST_CASE("DAttack.SimulationNetSync.ProxyVisualizationInputComesFromTheRemoteInputCacheLastKnown",
          "[DAttack][SimulationNetSync][ProxyScheduledRead]")
{
    requireGameZeroIsNotAValueInitialisedInput();

    ResimRig rig;
    MockPredictionOwner local, remote;
    addLocalCharacter(rig, 70u, local);
    addRemoteCharacter(rig, 71u, remote);

    // NOTHING IS INVENTED WHEN THERE IS NOTHING TO SHOW. The accessor answers
    // nullopt — not the injected game zero — for a LOCAL character (no store), an
    // UNKNOWN id, and a REMOTE character whose store is still cold. That nullopt
    // is what preserves the pre-existing "skip the input-carrying viz this frame"
    // behaviour; a fallback()-based accessor would silently start drawing an aim
    // indicator at the neutral pose instead.
    REQUIRE_FALSE(rig.netSync.getLastRelayedInput<SimulatableBrawler>(70u).has_value());
    REQUIRE_FALSE(rig.netSync.getLastRelayedInput<SimulatableBrawler>(9999u).has_value());
    REQUIRE_FALSE(rig.netSync.getLastRelayedInput<SimulatableBrawler>(71u).has_value());

    // THE LISTEN-SERVER HOST'S FALLBACK, verified rather than assumed: the
    // authority allocates no relay stores at all, so the accessor answers nullopt
    // there for exactly the structural reason getLatestInput did when the
    // correction caches were what was missing. The host's own pawn renders from
    // the LIVE sampler either way (asserted below).
    SimulationNetSync<SimulatableBrawler> serverNetSync(rig.storage, rig.reconciliation);
    MockAuthorityOwner authorityOwner;
    serverNetSync.setNeutralInput<SimulatableBrawler>(simulatableBrawler::getZeroPlayerInput());
    serverNetSync.registerAuthorityOwner<SimulatableBrawler>(71u, authorityOwner);
    REQUIRE_FALSE(serverNetSync.getLastRelayedInput<SimulatableBrawler>(71u).has_value());
    serverNetSync.unregisterSimulatable<SimulatableBrawler>(71u, nullptr, &authorityOwner);

    // COLD SOURCE -> the rule returns nullopt -> the call site's has_value() gate
    // skips the viz for that frame. Unchanged pre-existing behaviour.
    bool sampled = false;
    auto sampler = [&sampled]() { sampled = true; return richCapture(); };
    REQUIRE_FALSE(simulatableBrawler::selectVisualizationInput(
        false, sampler, rig.netSync.getLastRelayedInput<SimulatableBrawler>(71u)).has_value());
    REQUIRE_FALSE(sampled);     // the remote path must not observe a live read

    // TWO ARRIVALS, so "last-known" is pinned as the NEWEST entry and not merely
    // as "an entry". The stamps also make a scheduled read at tick 33 answer
    // capture 30 — a DIFFERENT value — which is the assertion that separates this
    // accessor from the per-tick ladder. The viz wants "what is this player
    // doing", not "which input does tick N run on".
    relayWrite(remote.relayedInputRing, 30u, 3u, 30.f);
    remote.replicateRelayRing();
    relayWrite(remote.relayedInputRing, 31u, 3u, 31.f);
    remote.replicateRelayRing();

    const auto lastKnown = rig.netSync.getLastRelayedInput<SimulatableBrawler>(71u);
    REQUIRE(lastKnown.has_value());
    REQUIRE(captureTagOf(*lastKnown) == Catch::Approx(31.f));
    REQUIRE(captureTagOf(proxyInputFor(rig.netSync, 33u, 71u)) == Catch::Approx(30.f));

    // ...and the rule hands the remote path that value VERBATIM.
    const auto selectedRemote =
        simulatableBrawler::selectVisualizationInput(
            false, sampler, rig.netSync.getLastRelayedInput<SimulatableBrawler>(71u));
    REQUIRE(selectedRemote.has_value());
    REQUIRE(sameInput(*selectedRemote, *lastKnown));
    REQUIRE_FALSE(sampled);

    // The LOCAL branch is untouched by the re-source: it samples live and never
    // looks at the remote source at all (which is nullopt for id 70 anyway).
    const auto selectedLocal =
        simulatableBrawler::selectVisualizationInput(
            true, sampler, rig.netSync.getLastRelayedInput<SimulatableBrawler>(70u));
    REQUIRE(sampled);
    REQUIRE(selectedLocal.has_value());
    REQUIRE(sameInput(*selectedLocal, richCapture()));

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(70u, &local);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(71u, &remote);
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

TEST_CASE("DAttack.SimulationNetSync.RelayProbeCountsPredictionAndResimReadsSeparately",
          "[DAttack][SimulationNetSync][RelayProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 90u, remote);

    const RelayReadProbe& probe = rig.netSync.getDiagnostics().relayReadProbe();

    // COLD STORE — rung 0 on every tick of the join window. It is COUNTED (the
    // window is worth seeing) but it opens NO stale run: rung 0 is "no data has
    // ever arrived", not "data arrived and stopped scheduling", and counting it
    // into the run is what would bias K upward.
    for (unsigned int tick = 1u; tick <= 3u; ++tick)
        predictTick(rig, tick);

    REQUIRE(probe.predictionCounters().noProbe == 3u);
    REQUIRE(probe.predictionCounters().total() == 3u);
    REQUIRE(probe.resimCounters().total() == 0u);
    REQUIRE(probe.currentFallbackRun(90u) == 0u);
    REQUIRE(probe.windowMaxFallbackRun() == 0u);

    // A SCHEDULED STREAM arrives: the server holds this wire at dA=2 and relays
    // captures 1..6 stamped with it.
    for (uint32 capture = 1u; capture <= 6u; ++capture)
    {
        relayWrite(remote.relayedInputRing, capture, 2u, static_cast<float>(capture));
        remote.replicateRelayRing();
    }

    // tick 4 probes capture 2, which is resident and consistently stamped.
    predictTick(rig, 4u);
    REQUIRE(probe.predictionCounters().hit == 1u);
    REQUIRE(probe.predictionCounters().total() == 4u);
    REQUIRE(probe.currentFallbackRun(90u) == 0u);

    // THE SECOND CALL SITE. A resim of a tick with no correction (NoRef) on a
    // remote character runs the IDENTICAL ladder — and lands in its OWN counter
    // block. Summing the two would erase a real frontier signal, so the assertion
    // below is that the prediction total does NOT move.
    const std::uint32_t predictionBefore = probe.predictionCounters().total();
    const auto resimResolved = resimInputFor(rig.netSync, 4u, 90u);
    REQUIRE(resimResolved.has_value());
    REQUIRE(probe.resimCounters().total() == 1u);
    REQUIRE(probe.resimCounters().hit == 1u);
    REQUIRE(probe.predictionCounters().total() == predictionBefore);

    // ...and the resim read does not touch the stale run either: a run is a
    // property of the MONOTONIC prediction stream, and resim revisits ticks out of
    // order.
    REQUIRE(probe.currentFallbackRun(90u) == 0u);

    // STARVATION: tick 20 probes capture 18, which never arrived and does not
    // alias a resident slot. THIS is what opens a stale run.
    predictTick(rig, 20u);
    REQUIRE(probe.predictionCounters().miss == 1u);
    REQUIRE(probe.currentFallbackRun(90u) == 1u);
    REQUIRE(probe.windowMaxFallbackRun() == 1u);

    // Per-id telemetry state is dropped on unregister — otherwise it accumulates
    // one entry per character that has ever existed in the session.
    REQUIRE(probe.trackedOwnerCount() == 1u);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(90u, &remote);
    REQUIRE(probe.trackedOwnerCount() == 0u);
}

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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(91u, &remote);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(92u, &remote);
}

// ===========================================================================
// [og-netcode-v2-input-relay T20] THE MISS CLASSES — WIRING, not arithmetic.
//
// Same argument as the block above, applied to T20's addition. The classification
// itself is unit-tested in og-simulation-tests ([RelayMissClass]); what those cases
// cannot show is that the SHIPPED prediction call site passes the whole report to
// the probe rather than just the outcome. If it passed only the outcome, every
// class counter would read zero for a whole session, `miss` would still look
// perfectly healthy, and the depth decision would be taken against three zeros.
//
// This case also reproduces the exact production geometry the classification exists
// to name: a DEPTH-1 ring whose intermediate capture tick was clobbered before it
// could replicate. The store's span covers the hole; the ring's never could.
// ===========================================================================

TEST_CASE("DAttack.SimulationNetSync.RelayProbeClassifiesMissesAtTheShippedCallSite",
          "[DAttack][SimulationNetSync][RelayProbeWiring]")
{
    ResimRig rig;
    MockPredictionOwner remote;
    addRemoteCharacter(rig, 92u, remote);

    const RelayReadProbe& probe = rig.netSync.getDiagnostics().relayReadProbe();

    // THE CLOBBER, at the shipped depth of 1. Capture 1 replicates; then captures 2
    // and 3 are both written before the next replication, so the ring carries only 3
    // and capture 2 is never transmitted to this client at all.
    relayWrite(remote.relayedInputRing, 1u, 2u, 1.f);
    remote.replicateRelayRing();
    relayWrite(remote.relayedInputRing, 2u, 2u, 2.f);
    relayWrite(remote.relayedInputRing, 3u, 2u, 3.f);
    remote.replicateRelayRing();

    // Tick 4 probes capture 2 — the hole. It lies BETWEEN the store's oldest (1) and
    // newest (3) residents, so it is the coverage class: the sender produced it and
    // the replace-latest ring lost it. This is the one class raising the depth would
    // move, and the whole depth decision turns on this counter being non-zero.
    predictTick(rig, 4u);
    REQUIRE(probe.predictionCounters().miss == 1u);
    REQUIRE(probe.predictionCounters().missInSpan == 1u);
    REQUIRE(probe.predictionCounters().missAboveNewest == 0u);

    // Tick 20 probes capture 18 — NEWER than anything that has arrived. No ring
    // depth delivers a capture the sender has not produced, so this must NOT be
    // counted as a coverage hole; conflating the two is precisely what would send
    // the next task after the wrong remedy.
    predictTick(rig, 20u);
    REQUIRE(probe.predictionCounters().miss == 2u);
    REQUIRE(probe.predictionCounters().missInSpan == 1u);
    REQUIRE(probe.predictionCounters().missAboveNewest == 1u);
    REQUIRE(probe.predictionCounters().missClassTotal()
            == probe.predictionCounters().miss);

    // THE SIGNED DELTA IS FED TOO, and it separates the same two reads continuously:
    // the in-span miss asked one tick BEHIND the newest arrival, the above-newest
    // miss asked fifteen ticks AHEAD of it.
    RelayDeltaSummary delta;
    probe.predictionCounters().delta.fillSummary(delta);
    REQUIRE(delta.samples == 2u);
    REQUIRE(delta.minDelta == -1);
    REQUIRE(delta.maxDelta == 15);

    // The resim call site keeps its own, still empty — the T19 separation survives
    // the widened payload.
    REQUIRE(probe.resimCounters().missClassTotal() == 0u);

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(92u, &remote);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(95u, &local);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(96u, &remote);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(97u, &local);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(98u, &remote);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(99u, &remote);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(195u, &local);
    rig.netSync.unregisterSimulatable<SimulatableBrawler>(196u, &remote);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(197u, &local);
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

    rig.netSync.unregisterSimulatable<SimulatableBrawler>(198u, &remote);
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
        SimulationNetSync<SimulatableBrawler>        netSync{ storage, reconciliation };

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
                    id, *predictionOwners.back(), nullptr);
                netSync.registerAuthorityOwner<SimulatableBrawler>(
                    id, *authorityOwners.back());

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
