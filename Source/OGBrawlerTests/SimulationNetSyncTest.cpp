// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// SimulationNetSync.h declares SimulatableOwnerTraits primary; must come before
// the specialization below.
#include "OGSimulation/SimulationNetSync.h"

// ---------------------------------------------------------------------------
// Minimal mock synced buffer types for owner concept satisfaction
// ---------------------------------------------------------------------------

struct MockStateSyncBuffer
{
    void write(const simulatableBrawler::State& /*state*/, uint32 /*tick*/) {}
    uint32_t readInto(simulatableBrawler::State& /*outState*/) const { return 0; }

    template <typename T>
    T readFromBuffer(uint32 /*byteIt*/) const { return T{}; }

    template <typename T>
    void writeToBuffer(uint32 /*byteIt*/, T /*val*/) {}
};

struct MockInputSyncBuffer
{
    void write(const simulatableBrawler::PlayerInput& /*input*/, uint32 /*tick*/) {}
    uint32_t readInto(simulatableBrawler::PlayerInput& /*outInput*/) const { return 0; }

    template <typename T>
    T readFromBuffer(uint32 /*byteIt*/) const { return T{}; }

    template <typename T>
    void writeToBuffer(uint32 /*byteIt*/, T /*val*/) {}
};

// ---------------------------------------------------------------------------
// MockPredictionOwner — satisfies PredictionSyncedBufferOwnerConcept
// ---------------------------------------------------------------------------

struct MockPredictionOwner
{
    using SyncedCorrectionBufferType  = MockStateSyncBuffer;
    using SyncedRemoteInputBufferType = MockInputSyncBuffer;

    std::function<void(const MockStateSyncBuffer&)>  onCorrectionStateReceived;
    std::function<void(const MockInputSyncBuffer&)>  onCorrectionInputReceived;
    MockInputSyncBuffer outgoingInputBuffer;

    void setOnCorrectionStateReceivedCallback(
        std::function<void(const MockStateSyncBuffer&)> fn)
    { onCorrectionStateReceived = std::move(fn); }

    void clearOnCorrectionStateReceivedCallback()
    { onCorrectionStateReceived = nullptr; }

    void setOnCorrectionInputReceivedCallback(
        std::function<void(const MockInputSyncBuffer&)> fn)
    { onCorrectionInputReceived = std::move(fn); }

    void clearOnCorrectionInputReceivedCallback()
    { onCorrectionInputReceived = nullptr; }

    MockInputSyncBuffer* getClientToServerInputSyncedBuffer()
    { return &outgoingInputBuffer; }

    // Stage 1 (Task 9): redundancy-bundle send path. The real owner builds an
    // FInputRedundancyBundle from the queue and fires an unreliable RPC; the mock
    // is a no-op (the bundle wire type is UE-side and not under test here).
    void sendLocalInputToAuthority(
        const PendingInputQueue<simulatableBrawler::PlayerInput>& /*queue*/,
        uint32 /*currentTick*/,
        uint32 /*redundancyDepth*/) {}
};

static_assert(PredictionSyncedBufferOwnerConcept<MockPredictionOwner,
                                                  simulatableBrawler::State,
                                                  simulatableBrawler::PlayerInput>,
    "MockPredictionOwner must satisfy PredictionSyncedBufferOwnerConcept");

// ---------------------------------------------------------------------------
// MockAuthorityOwner — satisfies AuthoritySyncedBufferOwnerConcept
// ---------------------------------------------------------------------------

struct MockAuthorityOwner
{
    using SyncedRemoteInputBufferType = MockInputSyncBuffer;

    MockStateSyncBuffer stateBuf;
    MockInputSyncBuffer inputBuf;
    // Stage 1 (Task 9): per-slot (capture_tick, input) inbound callback.
    std::function<void(uint32, const simulatableBrawler::PlayerInput&)> onRemoteMoveReceived;

    MockStateSyncBuffer& getSyncedCorrectionStateBuffer() { return stateBuf; }
    MockInputSyncBuffer& getSyncedCorrectionInputBuffer() { return inputBuf; }

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
    auto inputProvider = [zeroInput](const SimulationTimeStep&) { return zeroInput; };

    netSync.registerPredictionOwner<SimulatableBrawler>(42u, predictionOwner, std::move(inputProvider));

    REQUIRE(predictionOwner.onCorrectionStateReceived != nullptr);
    REQUIRE(predictionOwner.onCorrectionInputReceived != nullptr);

    netSync.unregisterSimulatable<SimulatableBrawler>(42u, &predictionOwner, nullptr);

    REQUIRE(predictionOwner.onCorrectionStateReceived == nullptr);
    REQUIRE(predictionOwner.onCorrectionInputReceived == nullptr);
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
    auto inputProvider = [zeroInput, &collectCallCount](const SimulationTimeStep&) {
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

#endif // WITH_LOW_LEVEL_TESTS
