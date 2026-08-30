// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins WindowResidency, NoSlotCause and classifyNoSlot -- what one poll's own sweep of the
// resident correction window learns about the RING ITSELF, and how a NoSlot answer at one
// tick is classified once that residency is known.
//
// WHAT THIS SUITE IS REALLY GUARDING is the ORDER the two guard causes are checked in.
// Both residency bounds are meaningless without a cache, and equally meaningless when
// nothing in the window is resident -- comparing an unset 0 against a real tick would
// silently misclassify almost every tick, which is exactly what a "tidied" reordering
// would produce without ever failing to compile.
//
// A SECOND thing this suite pins: rebuildAppliedCaptureInversion accumulates residency
// inside the SAME sweep it already runs over the window, so there is no second read of
// either diagnostic seam -- the accumulation is bounds derived from the join's own ticks.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>

#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryresidencytests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::classifyNoSlot;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::kAppliedPollWindowTicks;
using brawlerInputHistoryVisualization::LaneAdmission;
using brawlerInputHistoryVisualization::kNoSlotCauseCount;
using brawlerInputHistoryVisualization::NoSlotCause;
using brawlerInputHistoryVisualization::pollWindowEndingAt;
using brawlerInputHistoryVisualization::PollWindow;
using brawlerInputHistoryVisualization::rebuildAppliedCaptureInversion;
using brawlerInputHistoryVisualization::RowProvenanceSummary;
using brawlerInputHistoryVisualization::TickLanePollCounts;
using brawlerInputHistoryVisualization::WindowResidency;

// The reader double -- the same shape BrawlerInputHistoryPollTest.cpp's own MockSlotReader
// answers with: unset ticks default to NoSlot, exactly like the authority role does.
class MockSlotReader
{
public:
	void setRef(uint32_t simTick, AppliedCaptureRef ref) { m_refs[simTick] = ref; }
	void setHasCorrectionCache(bool hasCache) { m_hasCorrectionCache = hasCache; }

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		const auto it = m_refs.find(simTick);
		return (it == m_refs.end()) ? AppliedCaptureRef{} : it->second;
	}

	std::optional<SlotStateProvenance> slotProvenance(uint32_t) const { return std::nullopt; }

	bool hasCorrectionCache() const { return m_hasCorrectionCache; }

private:
	std::map<uint32_t, AppliedCaptureRef> m_refs;
	bool                                  m_hasCorrectionCache = true;
};

// A resident tick: the slot exists, nothing corrected it -- the ordinary client answer.
static void markResident(MockSlotReader& reader, uint32_t tick)
{
	reader.setRef(tick, AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
}

// One whole lane poll, for the poll-level cases below: the idle gate is handed no input
// and is switched OFF, so every case records whatever residency this same sweep learns.
static TickLanePollCounts poll(const MockSlotReader& reader, uint32_t liveTick,
                               InputHistoryTickLanes& lanes)
{
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(
		reader, liveTick, DAttackState::Idle, std::nullopt, false, std::nullopt, std::nullopt,
		std::nullopt, inversion, lanes);
}

} // namespace inputhistoryresidencytests

// ---------------------------------------------------------------------------
// classifyNoSlot is TOTAL and reaches every named cause, swept against the enumerator's
// OWN count rather than a literal -- a sweep that reaches four of five must fail here.
// ---------------------------------------------------------------------------
TEST_CASE("Residency.ClassifyNoSlotIsTotalAndReachesEveryCause", "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	WindowResidency healthy;
	healthy.hasCache       = true;
	healthy.anyResident    = true;
	healthy.oldestResident = 100u;
	healthy.newestResident = 150u;

	std::set<NoSlotCause> reached;

	for (const bool hasCache : { false, true })
	{
		for (const bool anyResident : { false, true })
		{
			WindowResidency residency = healthy;
			residency.hasCache        = hasCache;
			residency.anyResident     = anyResident;

			for (const uint32_t t : { healthy.oldestResident - 1u, healthy.oldestResident,
			                          healthy.newestResident, healthy.newestResident + 1u })
			{
				reached.insert(classifyNoSlot(t, residency));
			}
		}
	}

	CHECK(reached.size() == kNoSlotCauseCount);
	CHECK(kNoSlotCauseCount == 5u);

	// Each named cell, individually -- not just "some cause was returned".
	CHECK(classifyNoSlot(150u, WindowResidency{ false, true, 100u, 150u }) == NoSlotCause::NoCache);
	CHECK(classifyNoSlot(0u, WindowResidency{ true, false, 100u, 150u }) == NoSlotCause::Unclassifiable);
	CHECK(classifyNoSlot(99u, healthy) == NoSlotCause::Evicted);
	CHECK(classifyNoSlot(100u, healthy) == NoSlotCause::MissingInsideWindow);
	CHECK(classifyNoSlot(150u, healthy) == NoSlotCause::MissingInsideWindow);
	CHECK(classifyNoSlot(151u, healthy) == NoSlotCause::NotYetRun);
}

// ---------------------------------------------------------------------------
// The two guards run FIRST. A reordering that checks the bounds before the guards would
// pass every other test in this file and only fail here.
// ---------------------------------------------------------------------------
TEST_CASE("Residency.TheTwoGuardsAreCheckedBeforeAnyBoundComparison", "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	// A healthy residency, but no cache: even one tick past the frontier must still read
	// NoCache, never NotYetRun -- the bound comparison must not run first.
	WindowResidency noCache;
	noCache.hasCache       = false;
	noCache.anyResident    = true;
	noCache.oldestResident = 40u;
	noCache.newestResident = 99u;
	CHECK(classifyNoSlot(100u, noCache) == NoSlotCause::NoCache);

	// A cache exists but nothing in the window is resident: oldest/newest are unset (0),
	// and t = 0 must still read Unclassifiable, never a comparison against that 0.
	WindowResidency unclassifiable;
	unclassifiable.hasCache    = true;
	unclassifiable.anyResident = false;
	CHECK(classifyNoSlot(0u, unclassifiable) == NoSlotCause::Unclassifiable);
}

// ---------------------------------------------------------------------------
// rebuildAppliedCaptureInversion accumulates residency inside the same sweep: a healthy
// 60-tick window of real answers reports its own exact bounds.
// ---------------------------------------------------------------------------
TEST_CASE("Residency.AHealthyWindowReportsItsOwnExactBounds", "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	constexpr uint32_t kNewestTick = 1000u;

	MockSlotReader reader;
	for (uint32_t tick = kNewestTick - 59u; tick <= kNewestTick; ++tick)
		markResident(reader, tick);

	const PollWindow window = pollWindowEndingAt(kNewestTick, kAppliedPollWindowTicks);

	AppliedCaptureInversion inversion;
	WindowResidency         residency;
	residency.hasCache = true;
	const uint32_t filed = rebuildAppliedCaptureInversion(reader, window, inversion, residency);

	CHECK(filed == 60u);
	CHECK(residency.anyResident);
	CHECK(residency.oldestResident == kNewestTick - 59u);
	CHECK(residency.newestResident == kNewestTick);

	// The convenience overload existing callers already use is a pure delegation: same
	// window, same reader, same filed count -- one sweep either way.
	AppliedCaptureInversion unrelatedInversion;
	const uint32_t          filedViaOverload =
		rebuildAppliedCaptureInversion(reader, window, unrelatedInversion);
	CHECK(filedViaOverload == filed);
}

// ---------------------------------------------------------------------------
// The push-race and frontier shapes from a real ring: exactly one NoSlot tick at either
// edge of an otherwise-resident window.
// ---------------------------------------------------------------------------
TEST_CASE("Residency.APushRaceEvictionAndAFrontierNotYetRunClassifyAsExpected",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	constexpr uint32_t kP = 1000u;

	SECTION("the oldest window tick was evicted; the rest of the window is resident")
	{
		MockSlotReader reader;
		for (uint32_t tick = kP - 59u; tick <= kP - 1u; ++tick)
			markResident(reader, tick);
		// tick (kP - 60u) is left unset -> NoSlot, the evicted answer.

		AppliedCaptureInversion inversion;
		WindowResidency         residency;
		residency.hasCache = true;
		rebuildAppliedCaptureInversion(
			reader, pollWindowEndingAt(kP - 1u, kAppliedPollWindowTicks), inversion, residency);

		CHECK(residency.oldestResident == kP - 59u);
		CHECK(residency.newestResident == kP - 1u);
		CHECK(classifyNoSlot(kP - 60u, residency) == NoSlotCause::Evicted);
	}

	SECTION("the newest window tick has not run yet; the rest of the window is resident")
	{
		MockSlotReader reader;
		for (uint32_t tick = kP - 59u; tick <= kP - 1u; ++tick)
			markResident(reader, tick);
		// tick kP is left unset -> NoSlot, the frontier's genuine Pending.

		AppliedCaptureInversion inversion;
		WindowResidency         residency;
		residency.hasCache = true;
		rebuildAppliedCaptureInversion(
			reader, pollWindowEndingAt(kP, kAppliedPollWindowTicks), inversion, residency);

		CHECK(residency.oldestResident == kP - 59u);
		CHECK(residency.newestResident == kP - 1u);
		CHECK(classifyNoSlot(kP, residency) == NoSlotCause::NotYetRun);
	}

	SECTION("a Skip step's two-push backfill evicts two ticks at once, both Evicted")
	{
		MockSlotReader reader;
		for (uint32_t tick = kP - 58u; tick <= kP - 1u; ++tick)
			markResident(reader, tick);
		// ticks (kP - 60u) and (kP - 59u) are left unset -> both NoSlot.

		AppliedCaptureInversion inversion;
		WindowResidency         residency;
		residency.hasCache = true;
		rebuildAppliedCaptureInversion(
			reader, pollWindowEndingAt(kP - 1u, kAppliedPollWindowTicks), inversion, residency);

		CHECK(residency.oldestResident == kP - 58u);
		CHECK(residency.newestResident == kP - 1u);
		CHECK(classifyNoSlot(kP - 60u, residency) == NoSlotCause::Evicted);
		CHECK(classifyNoSlot(kP - 59u, residency) == NoSlotCause::Evicted);
	}
}

// ---------------------------------------------------------------------------
// Nothing in the window is resident at all: every tick reads Unclassifiable, regardless
// of where it sits, because the SECOND guard never lets a bound comparison run.
// ---------------------------------------------------------------------------
TEST_CASE("Residency.AnAllNoSlotWindowWithACacheIsUnclassifiableForEveryTick",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	const MockSlotReader reader; // every tick unset -> NoSlot
	const PollWindow     window = pollWindowEndingAt(500u, kAppliedPollWindowTicks);

	AppliedCaptureInversion inversion;
	WindowResidency         residency;
	residency.hasCache = true;
	rebuildAppliedCaptureInversion(reader, window, inversion, residency);

	CHECK_FALSE(residency.anyResident);

	CHECK(classifyNoSlot(0u, residency) == NoSlotCause::Unclassifiable);
	CHECK(classifyNoSlot(window.oldestTick, residency) == NoSlotCause::Unclassifiable);
	CHECK(classifyNoSlot(window.newestTick, residency) == NoSlotCause::Unclassifiable);
	CHECK(classifyNoSlot(window.newestTick + 500u, residency) == NoSlotCause::Unclassifiable);
}

// ---------------------------------------------------------------------------
// A wipe leaves exactly ONE resident tick, which collapses oldestResident and
// newestResident onto the same point. Every OTHER window tick then falls on one side of
// that single point, never inside a range -- so a wipe classifies as Evicted below the
// new frontier and NotYetRun above it, never MissingInsideWindow. That cause needs a
// WIDE resident window (oldestResident < newestResident); a single point cannot produce
// it, because the only tick a single-point range could name is the resident one itself,
// which by definition never reaches classifyNoSlot as a NoSlot answer.
// ---------------------------------------------------------------------------
TEST_CASE("Residency.AWipeCollapsesResidencyToASinglePointAndSplitsTheWindowAtIt",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	constexpr uint32_t kFrontierAfterWipe = 1000u;
	constexpr uint32_t kNewestTick        = kFrontierAfterWipe + 29u;

	MockSlotReader reader;
	markResident(reader, kFrontierAfterWipe); // every other tick in the window stays unset

	AppliedCaptureInversion inversion;
	WindowResidency         residency;
	residency.hasCache = true;
	rebuildAppliedCaptureInversion(
		reader, pollWindowEndingAt(kNewestTick, kAppliedPollWindowTicks), inversion, residency);

	CHECK(residency.anyResident);
	CHECK(residency.oldestResident == kFrontierAfterWipe);
	CHECK(residency.newestResident == kFrontierAfterWipe);

	CHECK(classifyNoSlot(kFrontierAfterWipe - 30u, residency) == NoSlotCause::Evicted);
	CHECK(classifyNoSlot(kFrontierAfterWipe - 1u, residency) == NoSlotCause::Evicted);
	CHECK(classifyNoSlot(kFrontierAfterWipe + 1u, residency) == NoSlotCause::NotYetRun);
	CHECK(classifyNoSlot(kNewestTick, residency) == NoSlotCause::NotYetRun);
}

// ---------------------------------------------------------------------------
// THE POLL-LEVEL CASES -- the same three shapes above, but through pollInputHistoryLanes
// so the WRITE rule (not just the classification) is pinned: which causes file nothing,
// which one still writes Pending, and what that does to a cell a healthy poll already
// recorded.
// ---------------------------------------------------------------------------

// ⭐ DERIVED, not copied from the Backlog bullet or the design doc (both were wrong about
// the wipe shape -- see impl_notes_phase5_30.md §4). T = 1000 sits 30 ticks in from this
// poll's own window floor (970) and 29 from its ceiling (1029), so neither split below is
// a degenerate edge case:
//   * 970..999 (30 ticks, t < T)  -> Evicted           -> file nothing, cell UNCHANGED
//   * 1000     (T itself)         -> resident, not NoSlot at all -- re-observed, IgnoredDuplicate
//   * 1001..1029 (29 ticks, t > T) -> NotYetRun         -> WRITES Pending, repainting a cell
//                                                          a moment ago RanUnconfirmed
// 30 + 1 + 29 == 60, the whole window. MissingInsideWindow is unreachable, exactly as the
// earlier poll-level work proved: a single resident point can only split a window in two,
// never leave a hole inside it.
// ⚠ No axis break is filed: the poll tick never moved, and a wipe is not derivable from
// the residency edges at all, so all 29 land in the epoch they were already in.
TEST_CASE("Residency.APollLevelWipeIsEvictedBelowTAndNotYetRunAboveIt",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	constexpr uint32_t kT          = 1000u;
	constexpr uint32_t kNewestTick = kT + 29u;
	constexpr uint32_t kOldestTick = kNewestTick - 59u; // == kT - 30u

	InputHistoryTickLanes lanes;

	// A healthy poll first: every tick in the window is resident, so every cell fills.
	{
		MockSlotReader healthy;
		for (uint32_t tick = kOldestTick; tick <= kNewestTick; ++tick)
			markResident(healthy, tick);

		const TickLanePollCounts counts = poll(healthy, kNewestTick, lanes);
		CHECK(counts.provenanceCellsRecorded == 60u);
	}

	for (uint32_t tick = kOldestTick; tick <= kNewestTick; ++tick)
	{
		REQUIRE(lanes.provenanceAt(tick) != nullptr);
		CHECK(*lanes.provenanceAt(tick) == RowProvenanceSummary::RanUnconfirmed);
	}

	// The wipe: only T survives.
	MockSlotReader wiped;
	markResident(wiped, kT);

	const TickLanePollCounts wipeCounts = poll(wiped, kNewestTick, lanes);

	CHECK(wipeCounts.provenanceCellsEvicted == 30u);
	CHECK(wipeCounts.provenanceCellsMissingInWindow == 0u);

	// No break, so every NotYetRun tick repaints the cell it already had: 29 Updated,
	// nothing Recorded, and T itself re-observed as the one Unchanged.
	// ⚠ THE THREE MUST STILL SUM WITH Evicted TO THE WHOLE WINDOW, OR A CELL IS HIDDEN.
	CHECK(wipeCounts.axisBreaksBackward == 0u);
	CHECK(wipeCounts.provenanceCellsUpdated == 29u);
	CHECK(wipeCounts.provenanceCellsRecorded == 0u);
	CHECK(wipeCounts.provenanceCellsUnchanged == 1u);

	for (uint32_t tick = kOldestTick; tick < kT; ++tick)
	{
		// Evicted: files nothing, so the healthy poll's observation is UNCHANGED.
		REQUIRE(lanes.provenanceAt(tick) != nullptr);
		CHECK(*lanes.provenanceAt(tick) == RowProvenanceSummary::RanUnconfirmed);
	}

	REQUIRE(lanes.provenanceAt(kT) != nullptr);
	CHECK(*lanes.provenanceAt(kT) == RowProvenanceSummary::RanUnconfirmed);

	for (uint32_t tick = kT + 1u; tick <= kNewestTick; ++tick)
	{
		// NotYetRun: WRITES Pending, repainting what a moment ago was RanUnconfirmed --
		// the second-order consequence the corrected AC calls out explicitly.
		REQUIRE(lanes.provenanceAt(tick) != nullptr);
		CHECK(*lanes.provenanceAt(tick) == RowProvenanceSummary::Pending);
	}

	// The live tick keeps the identity mapping, so its Pending repaints the very cell the
	// healthy poll wrote -- no epoch was opened and no second cell exists to hold it.
	REQUIRE(lanes.gate().laneTickOf(kNewestTick).has_value());
	const uint32_t liveLaneTick = *lanes.gate().laneTickOf(kNewestTick);
	CHECK(liveLaneTick == kNewestTick);
	CHECK(lanes.gate().axisEventCount() == 0u);

	// A later poll with the ring refilled repaints normally.
	MockSlotReader refilled;
	for (uint32_t tick = kOldestTick; tick <= kNewestTick; ++tick)
		markResident(refilled, tick);
	poll(refilled, kNewestTick, lanes);

	REQUIRE(lanes.provenanceAt(kT + 1u) != nullptr);
	CHECK(*lanes.provenanceAt(kT + 1u) == RowProvenanceSummary::RanUnconfirmed);
}

TEST_CASE("Residency.NoCorrectionCacheWritesNoProvenanceCellOverTenPolls",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	// The authority role: hasCorrectionCache() answers false for every tick, so every
	// NoSlot join classifies NoCache regardless of what the window's own bounds would say.
	MockSlotReader reader;
	reader.setHasCorrectionCache(false);

	InputHistoryTickLanes lanes;

	constexpr uint32_t kFirstTick = 500u;
	uint32_t           lastTick   = kFirstTick;

	for (uint32_t i = 0u; i < 10u; ++i)
	{
		lastTick = kFirstTick + i;
		const TickLanePollCounts counts = poll(reader, lastTick, lanes);

		CHECK(counts.provenanceNoCachePolls == 1u);
		CHECK(counts.provenanceCellsRecorded == 0u);
		CHECK(counts.provenanceCellsUpdated == 0u);
	}

	const PollWindow finalWindow = pollWindowEndingAt(lastTick, kAppliedPollWindowTicks);
	for (uint32_t tick = finalWindow.oldestTick; tick <= finalWindow.newestTick; ++tick)
		CHECK(lanes.provenanceAt(tick) == nullptr);

	REQUIRE(lanes.residencyReading().has_value());
	CHECK_FALSE(lanes.residencyReading()->residency.hasCache);
}

TEST_CASE("Residency.TheReadingKeepsAdvancingThroughAPausedIdleSpanWhileNoCellWrites",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryresidencytests;

	constexpr uint32_t kFirstTick = 1000u;
	const uint32_t     kWindow    = static_cast<uint32_t>(kAppliedPollWindowTicks);

	MockSlotReader reader;
	// Pre-seed enough history that the sliding 60-tick window is ALWAYS fully resident,
	// mirroring a real ring's no-holes property -- otherwise oldestResident would stick to
	// this seed's own start rather than genuinely tracking the window's moving edge.
	for (uint32_t tick = kFirstTick - kWindow; tick < kFirstTick; ++tick)
		markResident(reader, tick);

	InputHistoryTickLanes   lanes;
	AppliedCaptureInversion inversion;

	CaptureRowFields moving;
	moving.direction  = DirectionBucket::Forward;
	moving.buttonMask = 0u;

	CaptureRowFields idle;
	idle.direction  = DirectionBucket::Neutral;
	idle.buttonMask = 0u;

	uint32_t tick = kFirstTick;

	// 20 moving ticks: recorded normally, the ring resident throughout.
	for (uint32_t i = 0u; i < 20u; ++i, ++tick)
	{
		markResident(reader, tick);
		brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, tick, DAttackState::Idle,
			moving, true, std::nullopt, std::nullopt, std::nullopt, inversion, lanes);
	}

	// 30 idle ticks: the ring keeps advancing (residency keeps being derived, above the
	// gate), but the display's own idle predicate elides the lane writes once it engages.
	std::size_t recordedWhenPaused  = 0u;
	bool        pausedSnapshotTaken = false;

	for (uint32_t i = 0u; i < 30u; ++i, ++tick)
	{
		markResident(reader, tick);
		const TickLanePollCounts counts =
			brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, tick,
				DAttackState::Idle, idle, true, std::nullopt, std::nullopt, std::nullopt,
				inversion, lanes);

		if (!pausedSnapshotTaken && counts.admission == LaneAdmission::Elided)
		{
			recordedWhenPaused  = lanes.provenance().storedCellCount();
			pausedSnapshotTaken = true;
		}
	}

	REQUIRE(pausedSnapshotTaken);
	const uint32_t lastTick = tick - 1u;

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->simTick == lastTick);
	CHECK(lanes.residencyReading()->residency.oldestResident == lastTick - kWindow + 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == lastTick);

	// Once the pause engaged, no further provenance cell was written, even though the
	// residency reading above kept advancing all the way to the very last polled tick --
	// a frozen reading would fail the two CHECKs above instead of this one.
	CHECK(lanes.provenance().storedCellCount() == recordedWhenPaused);
}

#endif // WITH_LOW_LEVEL_TESTS
