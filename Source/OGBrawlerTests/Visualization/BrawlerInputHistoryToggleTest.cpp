// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins what happens to the input-history rows when the display's console toggle
// (`OGBrawler.InputHistoryDisplay`) is switched OFF and back ON mid-session.
//
// The toggle itself is UE code and unreachable from here -- Source/OGBrawlerTests links
// { Core, OGSimulation, OGBrawler } and not OGBrawlerUnreal. What IS reachable, and what
// the safety claim actually rests on, is the ring: switching off stops the poll and
// touches no state, so an off/on cycle is exactly a POLL GAP. This suite pins the two
// outcomes a gap can have and proves neither can overstate a hold.
//
//   * Gap SHORTER than the source cache: nothing was lost, because the cache still holds
//     every tick the paused poll missed. The resumed sweep recovers all of them.
//   * Gap LONGER than the source cache: ticks were evicted unseen. The resumed sweep must
//     OPEN A ROW rather than extend the pre-off one -- and the input is deliberately held
//     IDENTICAL across the gap here, so the only thing standing between the display and a
//     row that claims ticks nobody observed is the contiguity test.
//
// The second case is the one an eyeball passes and a session fails: with identical input
// every field matches across the gap, so a fold keyed on fields alone would silently
// splice it shut and report one long hold that never happened.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <set>

#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/BrawlerMotionMatching.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/Network/LocalInputCache.h"

namespace inputhistorytoggletests
{

using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryRow;
using brawlerInputHistoryVisualization::InputHistoryRowRing;

constexpr float kDeadzone = 0.15f;

// The production packer, so every capture here has the shape collectInputAll records.
static simulatableBrawler::PlayerInput makeCapture(glm::vec3 aimDirection,
                                                   glm::vec3 moveDirectionWorld)
{
	simulatableBrawler::ContinuousInputFields fields;
	fields.aimDirection       = aimDirection;
	fields.moveStick          = glm::vec2(moveDirectionWorld.x, moveDirectionWorld.y);
	fields.moveDirectionWorld = moveDirectionWorld;

	return simulatableBrawler::makeSimPlayerInput(fields, false, false, inputSequence::kNoMatch);
}

static simulatableBrawler::PlayerInput forwardCapture()
{
	return makeCapture(glm::vec3(1.f, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f));
}

static simulatableBrawler::PlayerInput backCapture()
{
	return makeCapture(glm::vec3(1.f, 0.f, 0.f), glm::vec3(-1.f, 0.f, 0.f));
}

// ---------------------------------------------------------------------------
// One session with the display switched off for a contiguous stretch of ticks.
// Captures are pushed every tick regardless -- the simulation does not stop when a
// diagnostic does, which is exactly why the source cache can cover a short gap.
// ---------------------------------------------------------------------------
struct ToggleSession
{
	InputHistoryRowRing ring;
	std::set<uint32_t>  observedTicks;   // ticks some sweep actually presented
	uint32_t            capturedTicks = 0u;
};

static ToggleSession runToggledSession(uint32_t tickCount,
                                       uint32_t offFirstTick,
                                       uint32_t offLastTick,
                                       uint32_t runLengthTicks)
{
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	const simulatableBrawler::DelayLineMotionHistory history(cache);

	ToggleSession session;

	for (uint32_t tick = 0u; tick < tickCount; ++tick)
	{
		const bool forward = ((tick / runLengthTicks) % 2u) == 0u;
		cache.push(static_cast<std::int32_t>(tick), forward ? forwardCapture() : backCapture());
		++session.capturedTicks;

		// The toggle's whole effect at the poll site: the sweep simply does not run.
		if (tick >= offFirstTick && tick <= offLastTick)
			continue;

		const brawlerInputHistoryVisualization::PollWindow window =
			brawlerInputHistoryVisualization::pollWindowEndingAt(
				tick, brawlerInputHistoryVisualization::kCapturePollWindowTicks);
		for (uint32_t seen = window.oldestTick; seen <= window.newestTick; ++seen)
		{
			if (history.at(seen) != nullptr)
				session.observedTicks.insert(seen);
		}

		brawlerInputHistoryVisualization::pollInputHistory(
			history, tick, kDeadzone, session.ring);
	}

	return session;
}

// The rows' total tick span and how many row boundaries are discontinuous.
struct RingCoverage
{
	uint32_t totalTicks = 0u;
	uint32_t gaps       = 0u;
};

static RingCoverage coverageOf(const InputHistoryRowRing& ring)
{
	RingCoverage coverage;

	for (std::size_t index = 0u; index < ring.size(); ++index)
	{
		const InputHistoryRow& row = ring.at(index);
		coverage.totalTicks += row.tickCount;

		if (index > 0u && row.firstCaptureTick != ring.at(index - 1u).lastCaptureTick() + 1u)
			++coverage.gaps;
	}

	return coverage;
}

// THE CRITERION ITSELF, counted rather than inferred: how many ticks inside some row's
// [firstCaptureTick, lastCaptureTick] span were never presented to the ring at all. A row
// that swallowed an off window would put every unobserved tick of that window in here.
static uint32_t unobservedTicksClaimedByRows(const InputHistoryRowRing& ring,
                                             const std::set<uint32_t>&  observedTicks)
{
	uint32_t claimed = 0u;

	for (std::size_t index = 0u; index < ring.size(); ++index)
	{
		const InputHistoryRow& row = ring.at(index);
		for (uint32_t tick = row.firstCaptureTick; tick <= row.lastCaptureTick(); ++tick)
		{
			if (observedTicks.find(tick) == observedTicks.end())
				++claimed;
		}
	}

	return claimed;
}

// ===========================================================================

TEST_CASE("Toggle.AnOffWindowShorterThanTheSourceCacheLosesNothingAndInflatesNothing",
          "[CharacterViz][InputHistoryViz]")
{
	// Off for 40 of 120 ticks. The source cache holds 64, so every missed tick is still
	// resident when the poll resumes and the resumed sweep recovers all of them.
	const ToggleSession session = runToggledSession(/*tickCount*/ 120u,
	                                                /*offFirstTick*/ 50u,
	                                                /*offLastTick*/ 89u,
	                                                /*runLengthTicks*/ 1000u);

	REQUIRE(brawlerInputHistoryVisualization::kCapturePollWindowTicks == 64u);
	CHECK(session.capturedTicks == 120u);

	const RingCoverage coverage = coverageOf(session.ring);

	// One unbroken row: the input never changed and no tick was lost, so the display is
	// entitled to say the direction was held for all 120 ticks -- it observed all 120.
	REQUIRE(session.ring.size() == 1u);
	CHECK(session.ring.newest().firstCaptureTick == 0u);
	CHECK(session.ring.newest().lastCaptureTick() == 119u);
	CHECK(session.ring.newest().tickCount == 120u);
	CHECK(coverage.totalTicks == 120u);
	CHECK(coverage.gaps == 0u);

	// Resuming re-presents ticks already folded, and re-presenting must stay a no-op:
	// a resumed poll that recounted its overlap would inflate the row past 120.
	CHECK(unobservedTicksClaimedByRows(session.ring, session.observedTicks) == 0u);
}

TEST_CASE("Toggle.AnOffWindowLongerThanTheSourceCacheOpensARowRatherThanSpanningTheGap",
          "[CharacterViz][InputHistoryViz]")
{
	// Off for 100 of 200 ticks, with the direction held CONSTANT across the whole session.
	// Every field matches across the gap, so only the contiguity test can stop the fold.
	const ToggleSession session = runToggledSession(/*tickCount*/ 200u,
	                                                /*offFirstTick*/ 50u,
	                                                /*offLastTick*/ 149u,
	                                                /*runLengthTicks*/ 1000u);

	const RingCoverage coverage = coverageOf(session.ring);

	// Two rows, not one: the pre-off row still ends where the poll stopped, and the
	// resumed poll opened its own at the oldest tick the cache could still answer for.
	REQUIRE(session.ring.size() == 2u);
	CHECK(session.ring.oldest().firstCaptureTick == 0u);
	CHECK(session.ring.oldest().lastCaptureTick() == 49u);
	CHECK(session.ring.oldest().tickCount == 50u);
	CHECK(session.ring.newest().firstCaptureTick == 87u);
	CHECK(session.ring.newest().lastCaptureTick() == 199u);
	CHECK(session.ring.newest().tickCount == 113u);

	// The discontinuity is DRAWN rather than smoothed over.
	CHECK(coverage.gaps == 1u);

	// Ticks 50..86 were evicted unseen. The display accounts for 163 of 200 -- it
	// UNDERSTATES what it never saw, which is the only honest direction to be wrong in.
	CHECK(coverage.totalTicks == 163u);
	CHECK(coverage.totalTicks < session.capturedTicks);
	CHECK(session.observedTicks.size() == 163u);

	// THE ACCEPTANCE CRITERION, counted: no row claims a single unobserved tick. Had the
	// resume extended the pre-off row instead, this would read 37 rather than 0.
	CHECK(unobservedTicksClaimedByRows(session.ring, session.observedTicks) == 0u);
}

TEST_CASE("Toggle.RepeatedOffOnCyclesLeaveTheRingBoundedOrderedAndExact",
          "[CharacterViz][InputHistoryViz]")
{
	// A hundred ten-tick stretches, alternating on and off, over a session long enough to
	// fold forty rows: a toggle someone flicks repeatedly must accumulate nothing.
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	const simulatableBrawler::DelayLineMotionHistory history(cache);

	InputHistoryRowRing ring;
	uint32_t            stretches = 0u;

	for (uint32_t tick = 0u; tick < 1000u; ++tick)
	{
		cache.push(static_cast<std::int32_t>(tick),
			((tick / 25u) % 2u == 0u) ? forwardCapture() : backCapture());

		if (tick % 10u == 0u)
			++stretches;

		// Ten ticks on, ten ticks off, over and over. Each gap is well inside the cache.
		if (((tick / 10u) % 2u) != 0u)
			continue;

		brawlerInputHistoryVisualization::pollInputHistory(history, tick, kDeadzone, ring);
	}

	CHECK(stretches == 100u);

	// Counted, then asserted once: rows strictly ascending and non-overlapping.
	uint32_t misordered = 0u;
	for (std::size_t index = 1u; index < ring.size(); ++index)
	{
		if (ring.at(index).firstCaptureTick <= ring.at(index - 1u).lastCaptureTick())
			++misordered;
	}

	const RingCoverage coverage = coverageOf(ring);

	CHECK(misordered == 0u);
	CHECK(ring.size() <= InputHistoryRowRing::capacity());

	// The direction flips every 25 ticks over 1000 ticks, so forty rows and not one more:
	// a cycle that opened a spurious row on each resume would land near a hundred instead.
	REQUIRE(ring.size() == 40u);
	CHECK(coverage.gaps == 0u);
	CHECK(ring.oldest().firstCaptureTick == 0u);

	// The session ENDS mid-off-stretch, so the last ten ticks are simply not folded yet --
	// pending, never lost, and the next resumed sweep still has them resident.
	CHECK(coverage.totalTicks == 990u);
	CHECK(ring.newest().lastCaptureTick() == 989u);
}

TEST_CASE("Toggle.SwitchingOffNeitherErasesAResidentRowNorReopensIt",
          "[CharacterViz][InputHistoryViz]")
{
	// A row folded before the display went off must still be there, unchanged, when it
	// comes back -- and the resumed sweep must not adopt it either.
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	const simulatableBrawler::DelayLineMotionHistory history(cache);

	InputHistoryRowRing ring;

	for (uint32_t tick = 0u; tick < 5u; ++tick)
		cache.push(static_cast<std::int32_t>(tick), forwardCapture());

	brawlerInputHistoryVisualization::pollInputHistory(history, 4u, kDeadzone, ring);
	REQUIRE(ring.size() == 1u);

	// OFF: 300 ticks pass with no poll at all. The capture cache scrolls right past every
	// tick the resident row covers.
	for (uint32_t tick = 5u; tick < 305u; ++tick)
		cache.push(static_cast<std::int32_t>(tick), backCapture());

	// ON again, sweeping a window that overlaps none of the pre-off row's ticks.
	brawlerInputHistoryVisualization::pollInputHistory(history, 304u, kDeadzone, ring);

	// The pre-off row survived the gap intact -- same span, same fields. Nothing in the
	// off window destroys, clears or re-keys the ring, so there is nothing to corrupt.
	REQUIRE(ring.size() == 2u);
	CHECK(ring.oldest().firstCaptureTick == 0u);
	CHECK(ring.oldest().tickCount == 5u);
	CHECK(ring.oldest().direction == DirectionBucket::Forward);

	// And the row the resumed poll opened is a fresh one covering only what it observed.
	CHECK(ring.newest().firstCaptureTick == 241u);
	CHECK(ring.newest().lastCaptureTick() == 304u);
	CHECK(ring.newest().direction == DirectionBucket::Back);
}

} // namespace inputhistorytoggletests

#endif // WITH_LOW_LEVEL_TESTS
