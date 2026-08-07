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
// Test: postPredictionAll advances the cache ring-buffer slot.
//
// [og-netcode-v2-input-relay T6] This case used to observe the ring through
// `collectResimInputAll`, which has RELOCATED to SimulationNetSync (the resim
// input now comes from netsync-owned delay lines / relay stores, not from the
// cache). Instantiating SimulationNetSync here would require a second,
// conflicting SimulatableOwnerTraits<SimulatableBrawler> specialization in this
// binary, so the case observes the ring through the narrow query reconciliation
// KEPT instead — `getAppliedCaptureTickRef`, whose NoSlot kind is exactly the
// "this tick is outside the cache window" condition the old assertion inferred
// from a missing map entry. The resolution behaviour itself is covered in
// SimulationNetSyncTest.cpp's [ResimResolution] block.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.CacheSlotAdvances", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(10u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(10u);

    // [og-netcode-v2-input-relay T16] The per-tick
    // `pushPredictionInput<SimulatableBrawler>(10u, zeroInput)` is gone with the
    // cache's input column, and the `zeroInput` local with it — nothing here read
    // either. What this case asserts is SLOT EXISTENCE, and the slot is allocated
    // by pushPredictionTick alone. Assertion count unchanged.
    for (uint32 tick = 1u; tick <= 3u; ++tick)
    {
        SimulationTimeStep step(tick, false);
        reconciliation.pushPredictionTick<SimulatableBrawler>(10u, tick);
        reconciliation.postPredictionAll(step);
    }

    // Tick 2 is in the ring: it has a slot, and — no correction having landed —
    // that slot carries no reference, which is the frontier/hole classification.
    REQUIRE(reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(10u, 2u).kind
            == AppliedCaptureRefKind::NoRef);

    // A tick that was never predicted has no slot at all. The two are deliberately
    // different kinds: NoSlot means "not part of this resim", NoRef means "part of
    // it, but re-derive the input".
    REQUIRE(reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(10u, 4000u).kind
            == AppliedCaptureRefKind::NoSlot);
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
// pushPredictionTick), causing the cache index to land in the current tick's slot
// instead of the skipped one.
//
// [og-netcode-v2-input-relay T16] The original bug description named
// "pushPredictionTick + pushPredictionInput"; the input half is gone with the
// cache's input column, and the MISSING TICK PUSH was always the whole of the
// defect — it is what allocates the slot this case checks for. Same regression,
// same assertions.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.SkipBackfillIndexed", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(5u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(5u);

    auto& character = storage.get<SimulatableBrawler>(5u);

    // Tick 1 — normal step.
    reconciliation.pushPredictionTick<SimulatableBrawler>(5u, 1u);
    reconciliation.postPredictionAll(SimulationTimeStep(1u, false));

    // Tick 3 — skip (tick 2 is the skipped slot).
    const uint32 skippedTick = 2u;
    reconciliation.backfillSkippedTick<SimulatableBrawler>(
        5u, skippedTick, character.getAllState().getState());

    // Tick 3 current slot.
    reconciliation.pushPredictionTick<SimulatableBrawler>(5u, 3u);
    reconciliation.postPredictionAll(SimulationTimeStep(3u, false, StepKind::Skip));

    // The skipped tick must have its OWN slot — the bug this case guards is the
    // backfill landing in the current tick's slot instead, which would leave the
    // skipped tick unrepresented in the ring (NoSlot) and therefore un-resimulated.
    REQUIRE(reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(5u, skippedTick).kind
            != AppliedCaptureRefKind::NoSlot);

    // ...and the current tick (3) must still have its own, distinct one.
    REQUIRE(reconciliation.getAppliedCaptureTickRef<SimulatableBrawler>(5u, 3u).kind
            != AppliedCaptureRefKind::NoSlot);
}

#endif // WITH_LOW_LEVEL_TESTS
