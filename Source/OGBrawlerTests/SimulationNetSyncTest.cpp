// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <vector>

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

// ===========================================================================
// [T9 parts 3+4] CLIENT LAYER-1 INPUT DELAY — end-to-end through the REAL
// SimulationNetSync::collectInputAll provider branch.
// (og-netcode-v2-arch-latency; D5.2 client half.)
//
// WHY HERE and not in og-simulation-tests. The container and the offset rule are
// unit-tested there (Network/ClientInputDelayLineTest.cpp, MPL-2.0). What that
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
        [](const SimulationTimeStep& step) {
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
        [](const SimulationTimeStep& step) {
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
        [](const SimulationTimeStep& step) {
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
        [](const SimulationTimeStep& step) {
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

TEST_CASE("DAttack.SimulationNetSync.ResyncWipeClearsTheClientDelayLine",
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
        [](const SimulationTimeStep& step) {
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

#endif // WITH_LOW_LEVEL_TESTS
