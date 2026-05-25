// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// Compile-time concept checks
// ---------------------------------------------------------------------------

static_assert(SimulatableState<SimulatableBrawler>,
    "SimulatableBrawler must satisfy SimulatableState");

static_assert(
    SimulationReconciliationConcept<
        SimulationReconciliation<SimulatableBrawler>,
        SimulatableBrawler>,
    "SimulationReconciliation<SimulatableBrawler> must satisfy SimulationReconciliationConcept");

// ---------------------------------------------------------------------------
// Helper — build a SimulatableBrawler with the new single-arg ctor.
// ---------------------------------------------------------------------------
static SimulatableBrawler makeReconciliationTestCharacter()
{
    simulatableBrawler::StaticData staticData;
    return SimulatableBrawler(staticData);
}

// ---------------------------------------------------------------------------
// Test: createCacheFor / removeCacheFor complete without crash.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.CacheLifecycle", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);

    reconciliation.createCacheFor<SimulatableBrawler>(42u);
    reconciliation.removeCacheFor<SimulatableBrawler>(42u);

    REQUIRE(true);
}

// ---------------------------------------------------------------------------
// Test: postPredictionAll + collectResimInputAll advances cache ring-buffer slot.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.CacheSlotAdvances", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(10u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(10u);

    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();

    for (uint32 tick = 1u; tick <= 3u; ++tick)
    {
        SimulationTimeStep step(tick, false);
        reconciliation.pushPredictionTick<SimulatableBrawler>(10u, tick);
        reconciliation.pushPredictionInput<SimulatableBrawler>(10u, zeroInput);
        reconciliation.postPredictionAll(step);
    }

    // collectResimInputAll should return an input for tick 2 now in the ring.
    auto inputs = reconciliation.collectResimInputAll(2u);
    const auto& map = std::get<std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
    REQUIRE(map.find(10u) != map.end());
}

// ---------------------------------------------------------------------------
// Test: checkDivergenceAll returns 0 when no correction has been injected.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.NoDivergenceWithoutCorrection", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(1u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(1u);

    const unsigned int result = reconciliation.checkDivergenceAll();
    REQUIRE(result == 0u);
}

// ---------------------------------------------------------------------------
// Test: backfillSkippedTick writes the skipped tick's slot correctly.
// Regression test for the bug where only pushPredictionState was called (missing
// pushPredictionTick + pushPredictionInput), causing the cache index to land in
// the current tick's slot instead of the skipped one.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.SkipBackfillIndexed", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(5u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(5u);

    simulatableBrawler::PlayerInput zeroInput = simulatableBrawler::getZeroPlayerInput();
    auto& character = storage.get<SimulatableBrawler>(5u);

    // Tick 1 — normal step.
    reconciliation.pushPredictionTick<SimulatableBrawler>(5u, 1u);
    reconciliation.pushPredictionInput<SimulatableBrawler>(5u, zeroInput);
    reconciliation.postPredictionAll(SimulationTimeStep(1u, false));

    // Tick 3 — skip (tick 2 is the skipped slot).
    const uint32 skippedTick = 2u;
    reconciliation.backfillSkippedTick<SimulatableBrawler>(
        5u, skippedTick, character.getAllState().getState());

    // Tick 3 current slot.
    reconciliation.pushPredictionTick<SimulatableBrawler>(5u, 3u);
    reconciliation.pushPredictionInput<SimulatableBrawler>(5u, zeroInput);
    reconciliation.postPredictionAll(SimulationTimeStep(3u, false, StepKind::Skip));

    // collectResimInputAll must find an entry for the skipped tick.
    auto inputs = reconciliation.collectResimInputAll(skippedTick);
    const auto& map = std::get<std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs);
    REQUIRE(map.find(5u) != map.end());

    // The current tick (3) must also be findable.
    auto inputs3 = reconciliation.collectResimInputAll(3u);
    const auto& map3 = std::get<std::unordered_map<unsigned int, simulatableBrawler::PlayerInput>>(inputs3);
    REQUIRE(map3.find(5u) != map3.end());
}

#endif // WITH_LOW_LEVEL_TESTS
