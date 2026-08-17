// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/ResimGateProbe.h"
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
//
// [item 45] The signature gained the DEPTH POLICY (0 == no policy, the shipped
// value) and a defaulted out-pointer for the skip count. 0 is passed here for the
// same reason production passes it under the legacy trigger policy: this case is
// about the no-correction floor, not about depth.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.NoDivergenceWithoutCorrection", "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(1u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(1u);

    const unsigned int result = reconciliation.checkDivergenceAll(/*maxAnchorDepthTicks=*/0u);
    REQUIRE(result == 0u);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 45] THE DEPTH POLICY AND THE CONSUME SWEEP, at
// the level that actually owns them.
//
// WHY HERE AND NOT IN og-simulation-tests. `checkDivergenceAll` /
// `prepareResimAll` / `consumeResimAnchorsAll` sweep a `SimulationObjectStorage`
// of real simulatables, and this suite is the one that has one. The POLICY
// PREDICATES are swept exhaustively in og-simulation-tests
// (`ResimGatePolicyTest.cpp`) and the GATE SEMANTICS in `ResimGateSemanticsTest.cpp`;
// what is only observable from here is that the per-character sweep obeys them.
//
// The correction is injected through a MINIMAL BUFFER MOCK rather than a real wire
// buffer: `injectCorrectionState` needs exactly `readInto(state)` (returning the
// tick) and `getAppliedCaptureTick()`. Leaving the state default-constructed is
// deliberate and harmless — the shipped `FrontierExact` policy anchors on POSITION
// and never reads the verdict, so this case does not depend on the comparison's
// outcome.
// ---------------------------------------------------------------------------
namespace
{
    struct MockCorrectionBuffer
    {
        uint32 tick = 0u;

        template <typename S>
        uint32 readInto(S&) const { return tick; }

        uint32 getAppliedCaptureTick() const { return kNoInputCaptureTick; }
    };
}

TEST_CASE("DAttack.SimulationReconciliation.DeepAnchorsAreSkippedAndCountedNotClamped",
          "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(7u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(7u);

    // Predict to 100, land a frontier-exact correction (the shipped policy's
    // trigger), then keep predicting so the anchor falls behind the frontier.
    for (uint32 tick = 90u; tick <= 100u; ++tick)
    {
        reconciliation.pushPredictionTick<SimulatableBrawler>(7u, tick);
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
    }

    MockCorrectionBuffer buffer{ 100u };
    reconciliation.injectCorrectionState<SimulatableBrawler>(7u, buffer);

    // Depth 5: inside any policy, so the anchor is returned and nothing is skipped.
    for (uint32 tick = 101u; tick <= 105u; ++tick)
    {
        reconciliation.pushPredictionTick<SimulatableBrawler>(7u, tick);
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
    }

    unsigned int deepSkips = 99u;
    REQUIRE(reconciliation.checkDivergenceAll(12u, &deepSkips) == 100u);
    REQUIRE(deepSkips == 0u);

    // Now run the frontier out to depth 20 — beyond a 12-tick window.
    for (uint32 tick = 106u; tick <= 120u; ++tick)
    {
        reconciliation.pushPredictionTick<SimulatableBrawler>(7u, tick);
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
    }

    // SKIPPED AND COUNTED: the fold returns "no resim" and the exclusion is
    // reported, rather than the anchor being clamped up to `frontier - 12` (which
    // would restore an uncorrected mid-window slot and replay the identical
    // prediction — a no-op costing a full Chaos rewind).
    deepSkips = 0u;
    REQUIRE(reconciliation.checkDivergenceAll(12u, &deepSkips) == 0u);
    REQUIRE(deepSkips == 1u);

    // ⭐ AND THE ANCHOR IS NOT CONSUMED BY BEING SKIPPED — recovery is a newer
    // correction or the HardResync failsafe, so the same call with the policy OFF
    // (the shipped configuration) still finds it. This is the assertion that
    // separates "skipped" from "dropped".
    deepSkips = 99u;
    REQUIRE(reconciliation.checkDivergenceAll(0u, &deepSkips) == 100u);
    REQUIRE(deepSkips == 0u);
}

TEST_CASE("DAttack.SimulationReconciliation.ConsumeResimAnchorsAllClosesTheGateOncePerResim",
          "[DAttack][SimulationReconciliation]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(8u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(8u);

    for (uint32 tick = 50u; tick <= 60u; ++tick)
    {
        reconciliation.pushPredictionTick<SimulatableBrawler>(8u, tick);
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
    }

    MockCorrectionBuffer buffer{ 60u };
    reconciliation.injectCorrectionState<SimulatableBrawler>(8u, buffer);

    reconciliation.pushPredictionTick<SimulatableBrawler>(8u, 61u);
    reconciliation.postPredictionAll(SimulationTimeStep(61u, false));

    REQUIRE(reconciliation.checkDivergenceAll(0u) == 60u);

    // The production order: prepare (captures the anchor per character) -> replay ->
    // `[Resim.Finish]` (consumes). The replay itself writes no gate state, which is
    // what makes the storm structurally impossible, so the consume is the ONLY thing
    // that can close the gate.
    //
    // ⚠ [item 47 CORRECTED THE SPAN] this replayed tick 60 as well, and production
    // never does: `prepareResimulation` sets the clock cursor to `simTick` and
    // restores live state FROM slot 60, then every `onGameSimulationResimulation`
    // calls `advanceResimulation()` BEFORE integrating — so the first replayed tick
    // is 61 and the ANCHOR SLOT IS THE RESTORE SOURCE, never a write target. Under
    // item 45 the extra tick was harmless; under item 47 it would exercise a
    // PROTECTION production cannot perform (slot 60 is corrected by definition) and
    // inflate this case's protection counts. Nothing else about the case changes —
    // the gate assertions below are untouched and still pass.
    reconciliation.prepareResimAll(60u);
    reconciliation.postResimulationAll(SimulationTimeStep(61u, true));   // [item 55] return now a ResimSweepDiagnostics; unused here as before

    REQUIRE(reconciliation.checkDivergenceAll(0u) == 60u);   // still open before the edge

    REQUIRE(reconciliation.consumeResimAnchorsAll() == 0u);  // nothing survived
    REQUIRE(reconciliation.checkDivergenceAll(0u) == 0u);

    // TERMINATION at the sweep level: it STAYS closed as the frontier advances, with
    // the corrected slot still flagged. This is the same property
    // `ACompletedResimClosesTheGateAndItStaysClosed` pins on a bare cache, asserted
    // once through the class production actually calls.
    for (uint32 tick = 62u; tick <= 80u; ++tick)
    {
        reconciliation.pushPredictionTick<SimulatableBrawler>(8u, tick);
        reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
        REQUIRE(reconciliation.checkDivergenceAll(0u) == 0u);
    }
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 57 / RN-6] `consumeResimAnchorsAll`'s return is
// now fed to `ResimGateProbe::noteSurvivingAnchors` at the sole production call
// site (`SimulationManager.h`'s `[Resim.Finish]` block, beside `noteFinish`), and
// surfaced as a field on the `[ResimProbe.Gate]` line. This case proves the
// counter actually MOVES rather than asserting one reading in isolation ("a
// counter that cannot be shown to move is the initiative's signature defect" —
// item 57): the CONSUMED case (nothing survives — the same zero the two
// pre-existing assertions above pin, at the sweep level) and the SURVIVING case
// (a correction lands AFTER `prepareResimAll` captured the anchor but BEFORE
// `consumeResimAnchorsAll` runs — the mid-replay landing the header describes),
// fed into ONE probe so the field is shown reading 0 and then differing.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.SurvivingAnchorsFeedTheGateProbe",
          "[DAttack][SimulationReconciliation]")
{
    ResimGateProbe probe;
    ResimGateWindowSummary window;

    // --- THE CONSUMED CASE: nothing survives. Mirrors
    // ConsumeResimAnchorsAllClosesTheGateOncePerResim's scenario above, on its own
    // character id so the two cases cannot interact through shared cache state. ---
    {
        SimulationObjectStorage<SimulatableBrawler> storage;
        storage.add<SimulatableBrawler>(20u, makeReconciliationTestCharacter());

        SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
        reconciliation.createCacheFor<SimulatableBrawler>(20u);

        for (uint32 tick = 50u; tick <= 60u; ++tick)
        {
            reconciliation.pushPredictionTick<SimulatableBrawler>(20u, tick);
            reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
        }

        MockCorrectionBuffer buffer{ 60u };
        reconciliation.injectCorrectionState<SimulatableBrawler>(20u, buffer);

        reconciliation.pushPredictionTick<SimulatableBrawler>(20u, 61u);
        reconciliation.postPredictionAll(SimulationTimeStep(61u, false));

        REQUIRE(reconciliation.checkDivergenceAll(0u) == 60u);

        reconciliation.prepareResimAll(60u);
        reconciliation.postResimulationAll(SimulationTimeStep(61u, true));

        const unsigned int survivingAnchors = reconciliation.consumeResimAnchorsAll();
        REQUIRE(survivingAnchors == 0u);   // nothing survived — same as the case above

        probe.noteSurvivingAnchors(survivingAnchors);
        probe.fillSummary(window);
        REQUIRE(window.survivingAnchors == 0u);
    }

    // --- THE SURVIVING CASE: a correction lands mid-replay. Under the shipped
    // `FrontierExact` default (no `setResimTriggerPolicy` call — matches
    // production's compiled default), the second correction must land exactly on
    // the (now-advanced) frontier to set a NEW anchor, exactly like the first one
    // did — `resimGate::shouldSetPendingAnchor`'s `FrontierExact` branch is just
    // `landedAtFrontier`. ---
    {
        SimulationObjectStorage<SimulatableBrawler> storage;
        storage.add<SimulatableBrawler>(21u, makeReconciliationTestCharacter());

        SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
        reconciliation.createCacheFor<SimulatableBrawler>(21u);

        for (uint32 tick = 50u; tick <= 60u; ++tick)
        {
            reconciliation.pushPredictionTick<SimulatableBrawler>(21u, tick);
            reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
        }

        MockCorrectionBuffer buffer{ 60u };
        reconciliation.injectCorrectionState<SimulatableBrawler>(21u, buffer);

        reconciliation.pushPredictionTick<SimulatableBrawler>(21u, 61u);
        reconciliation.postPredictionAll(SimulationTimeStep(61u, false));

        REQUIRE(reconciliation.checkDivergenceAll(0u) == 60u);

        // Prepare captures the anchor this resim believes it is closing: 60.
        reconciliation.prepareResimAll(60u);

        // MID-REPLAY LANDING: a second correction lands exactly on the frontier
        // (61) — landedAtFrontier is true under `FrontierExact` — while the
        // (simulated) replay is still in flight, i.e. strictly between the
        // prepare above and the consume below. `raisePendingResimAnchorTo`'s
        // CAS-max moves the pending anchor from 60 to 61.
        MockCorrectionBuffer midReplayBuffer{ 61u };
        reconciliation.injectCorrectionState<SimulatableBrawler>(21u, midReplayBuffer);

        reconciliation.postResimulationAll(SimulationTimeStep(61u, true));

        // The game thread keeps predicting while the replay above was (notionally)
        // still in flight, so the frontier moves on to 62 — otherwise the raised
        // anchor (61) would equal the frontier (61) and `needsResimulation()` would
        // read false (anchor == frontier is "nothing to rewind to", not "gate
        // open"), masking the very survival this case exists to show.
        reconciliation.pushPredictionTick<SimulatableBrawler>(21u, 62u);
        reconciliation.postPredictionAll(SimulationTimeStep(62u, false));

        // The captured expected value (60) no longer matches the live pending
        // anchor (61): the CAS fails and the anchor SURVIVES.
        const unsigned int survivingAnchors = reconciliation.consumeResimAnchorsAll();
        REQUIRE(survivingAnchors == 1u);
        REQUIRE(reconciliation.checkDivergenceAll(0u) == 61u);   // gate still open, a DIFFERENT anchor

        probe.noteSurvivingAnchors(survivingAnchors);
        probe.fillSummary(window);
        REQUIRE(window.survivingAnchors == 1u);   // 0 (consumed case) + 1 (this case) — THE COUNTER MOVED
    }
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 47] THE MIN-FOLD, AT THE LEVEL THAT PERFORMS
// IT — a replay must not clobber a corrected slot, and the sweep must report
// which population it protected.
//
// WHY HERE AND NOT IN og-simulation-tests. The per-slot RULE and its two-clause
// classifier are swept on bare caches in `ResimGateSemanticsTest.cpp` /
// `ResimGatePolicyTest.cpp`. What is only observable from THIS suite is the thing
// that produces the defect's subtlest population: `checkDivergenceAll` folds the
// per-character anchors with `std::min` because a Chaos rewind is global, so
// `postResimulationAll` replays a NEWER-anchored character forward THROUGH its own
// corrected slots. That fold needs a `SimulationObjectStorage` with two real
// simulatables, and this is the suite that has one.
//
// The shipped `FrontierExact` policy is used throughout — no policy call — so the
// construction is one a default build actually reaches: every correction below is
// injected exactly ON that character's frontier, which is what the legacy policy
// anchors on.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationReconciliation.AReplaySweepProtectsCorrectedSlotsAndReportsThePopulation",
          "[DAttack][SimulationReconciliation][ReplayProtect]")
{
    SimulationObjectStorage<SimulatableBrawler> storage;
    storage.add<SimulatableBrawler>(20u, makeReconciliationTestCharacter());
    storage.add<SimulatableBrawler>(21u, makeReconciliationTestCharacter());

    SimulationReconciliation<SimulatableBrawler> reconciliation(storage);
    reconciliation.createCacheFor<SimulatableBrawler>(20u);
    reconciliation.createCacheFor<SimulatableBrawler>(21u);

    const auto predictBoth = [&](uint32 fromTick, uint32 toTick) {
        for (uint32 tick = fromTick; tick <= toTick; ++tick)
        {
            reconciliation.pushPredictionTick<SimulatableBrawler>(20u, tick);
            reconciliation.pushPredictionTick<SimulatableBrawler>(21u, tick);
            reconciliation.postPredictionAll(SimulationTimeStep(tick, false));
        }
    };
    const auto correct = [&](unsigned int id, uint32 tick) {
        MockCorrectionBuffer buffer{ tick };
        reconciliation.injectCorrectionState<SimulatableBrawler>(id, buffer);
    };

    predictBoth(50u, 60u);
    correct(20u, 60u);           // character A anchors 60 (frontier-exact)
    predictBoth(61u, 62u);
    correct(21u, 62u);           // character B anchors 62...
    predictBoth(63u, 65u);
    correct(21u, 65u);           // ...and raises to 65 (CAS-max); 62 stays corrected
    predictBoth(66u, 70u);

    // THE FOLD: the rewind is global, so it restores at the OLDEST tick anybody
    // still needs — A's 60, not B's 65.
    REQUIRE(reconciliation.checkDivergenceAll(/*maxAnchorDepthTicks=*/0u) == 60u);

    reconciliation.prepareResimAll(60u);

    // The replay: 61..70 (the anchor slot is the restore SOURCE, never replayed).
    // [item 55] Each tick's sweep now returns one ResimSweepDiagnostics rather
    // than a return value plus two out-pointers; accumulate its three fields.
    unsigned int discards = 0u;
    unsigned int fresh    = 0u;
    unsigned int stale    = 0u;
    for (uint32 tick = 61u; tick <= 70u; ++tick)
    {
        const auto tickDiagnostics =
            reconciliation.postResimulationAll(SimulationTimeStep(tick, true));
        discards += tickDiagnostics.discards;
        fresh    += tickDiagnostics.freshProtections;
        stale    += tickDiagnostics.staleProtections;
    }

    // ⭐ B's slot AT its own captured anchor (65) is FRESH by the tick clause — it
    // is information this resim was supposed to act on and did not. B's older
    // correction at 62, inside `(sharedMin, ownAnchor)`, is STALE — and protected
    // all the same, because the replay derives it from B's OLD PREDICTION at 60,
    // not from newer authority.
    REQUIRE(fresh == 1u);
    REQUIRE(stale == 1u);
    // A contributed neither: its only corrected slot is 60, the restore source,
    // which sits below the span. And nothing left the 60-slot window.
    REQUIRE(discards == 0u);

    // ⭐ P1 — CONSUME-ALL, AS SHIPPED, and item 47 does not change it: each cache
    // CASes against ITS OWN captured anchor, so B's anchor is consumed by a resim
    // that restored at A's tick and never applied B's correction. One resim; B is
    // uncorrected by it and self-heals on its next rotation landing
    // (<= ceil(N/K) ticks). What item 47 changes is that B's authority state is
    // still THERE when that happens, instead of having been replayed over.
    //
    // P2 (capture-the-restore-tick — B's CAS fails, its anchor survives, and a
    // CONVERGENT cascade of <= N resims each restores from that character's own
    // now-protected slot) is only MEANINGFUL with this item's protect-all: pre-47,
    // round 1 clobbered the very slots the follow-ups would restore from, so every
    // follow-up was hollow. It is a cost/quality call priced by item 46's
    // per-resim cost data, NOT decided here. Asserted rather than implied:
    REQUIRE(reconciliation.consumeResimAnchorsAll() == 0u);   // both consumed
    REQUIRE(reconciliation.checkDivergenceAll(0u) == 0u);     // one resim, gate shut
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
