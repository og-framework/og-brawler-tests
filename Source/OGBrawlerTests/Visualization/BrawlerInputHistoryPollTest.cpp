// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins BrawlerInputHistoryVisualizationPoll.h -- the sweep that feeds the display's rows from the
// caches the client already keeps.
//
// WHAT THIS SUITE IS REALLY GUARDING is the claim that a render-rate poll of a
// 64-tick source ring is LOSSLESS. That claim has two halves and neither is visible
// from a single sweep: the window must span the whole source cache, and re-sweeping
// it must change nothing. A poll that got either half wrong would still look correct
// in a one-shot test and would be silently wrong in a session.
//
// The second thing it guards is the FIELD MAPPING. A capture carries two movement
// vectors, and the matcher classifies exactly one of them; drawing the other would
// put a glyph on screen for a sector the matcher never tested.
//
// The UE owner of the rings is not reachable from here -- Source/OGBrawlerTests links
// { Core, OGSimulation, OGBrawler } and not OGBrawlerUnreal -- which is precisely why
// the poll lives in og-brawler and takes its two sources as template parameters.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>

#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/BrawlerMotionMatching.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/Network/LocalInputCache.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistorypolltests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryPollCounts;
using brawlerInputHistoryVisualization::InputHistoryRow;
using brawlerInputHistoryVisualization::InputHistoryRowRing;
using brawlerInputHistoryVisualization::PollWindow;
using brawlerInputHistoryVisualization::RowProvenanceSummary;

constexpr float kDeadzone = 0.15f;

// The production packer, so every capture in this file has the shape collectInputAll
// records rather than a hand-built one that could drift away from it.
static simulatableBrawler::PlayerInput makeCapture(glm::vec3 aimDirection,
                                                   glm::vec2 moveStick,
                                                   glm::vec3 moveDirectionWorld,
                                                   bool      leftAttack   = false,
                                                   bool      rightAttack  = false,
                                                   uint32_t  triggeredActionId = inputSequence::kNoMatch)
{
	simulatableBrawler::ContinuousInputFields fields;
	fields.aimDirection       = aimDirection;
	fields.moveStick          = moveStick;
	fields.moveDirectionWorld = moveDirectionWorld;

	return simulatableBrawler::makeSimPlayerInput(fields, leftAttack, rightAttack, triggeredActionId);
}

// Aim along +X, so a world move along +X is Forward and along +Y is right-of-aim.
static simulatableBrawler::PlayerInput forwardCapture()
{
	return makeCapture(glm::vec3(1.f, 0.f, 0.f), glm::vec2(1.f, 0.f), glm::vec3(1.f, 0.f, 0.f));
}

static simulatableBrawler::PlayerInput backCapture()
{
	return makeCapture(glm::vec3(1.f, 0.f, 0.f), glm::vec2(-1.f, 0.f), glm::vec3(-1.f, 0.f, 0.f));
}

// ---------------------------------------------------------------------------
// The SlotReader double. Its DEFAULT answer is the authority's real answer: the
// server allocates no correction cache, so findCorrectionCache is null for every id
// and both seams answer NoSlot / nullopt.
// ---------------------------------------------------------------------------
class MockSlotReader
{
public:
	void setRef(uint32_t simTick, AppliedCaptureRef ref) { m_refs[simTick] = ref; }

	void setProvenance(uint32_t simTick, SlotStateProvenance provenance)
	{
		m_provenances[simTick] = provenance;
	}

	void setHasCorrectionCache(bool hasCache) { m_hasCorrectionCache = hasCache; }

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		const auto it = m_refs.find(simTick);
		return (it == m_refs.end()) ? AppliedCaptureRef{} : it->second;
	}

	std::optional<SlotStateProvenance> slotProvenance(uint32_t simTick) const
	{
		const auto it = m_provenances.find(simTick);
		return (it == m_provenances.end()) ? std::optional<SlotStateProvenance>{}
		                                   : std::optional<SlotStateProvenance>(it->second);
	}

	bool hasCorrectionCache() const { return m_hasCorrectionCache; }

private:
	std::map<uint32_t, AppliedCaptureRef>   m_refs;
	std::map<uint32_t, SlotStateProvenance> m_provenances;
	bool                                    m_hasCorrectionCache = true;
};

// The rows' total tick span and how many boundaries are discontinuous. Counted, then
// asserted once, so no CHECK ever runs inside a loop.
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

// A whole session: one capture pushed per sim tick into a REAL LocalInputCache, with a
// poll every `pollPeriodTicks` ticks and one final poll at the end. The direction flips
// every `runLengthTicks` ticks so the rows stay countable.
struct SessionResult
{
	InputHistoryRowRing ring;
	RingCoverage        coverage;
};

static SessionResult runSession(uint32_t tickCount, uint32_t pollPeriodTicks, uint32_t runLengthTicks)
{
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	const simulatableBrawler::DelayLineMotionHistory history(cache);

	SessionResult result;

	for (uint32_t tick = 0u; tick < tickCount; ++tick)
	{
		const bool forward = ((tick / runLengthTicks) % 2u) == 0u;
		cache.push(static_cast<std::int32_t>(tick), forward ? forwardCapture() : backCapture());

		if ((tick + 1u) % pollPeriodTicks == 0u)
		{
			brawlerInputHistoryVisualization::pollInputHistory(
				history, tick, kDeadzone, result.ring);
		}
	}

	brawlerInputHistoryVisualization::pollInputHistory(
		history, tickCount - 1u, kDeadzone, result.ring);

	result.coverage = coverageOf(result.ring);
	return result;
}

// ===========================================================================

TEST_CASE("Poll.TheSweepWindowSpansTheSourceCacheAndClampsAtTickZero",
          "[CharacterViz][InputHistoryViz]")
{
	SECTION("each width is pinned against the constant it mirrors, never against a literal")
	{
		// A source cache or a correction window that is resized has to move this with it.
		CHECK(brawlerInputHistoryVisualization::kCapturePollWindowTicks
		      == kLocalInputCacheCapacityTicks);
		CHECK(brawlerInputHistoryVisualization::kAppliedPollWindowTicks
		      == brawlerInputHistoryVisualization::kAppliedCaptureInversionCapacity);
	}

	SECTION("a mid-session window is exactly the cache's capacity, inclusive at both ends")
	{
		const PollWindow window = brawlerInputHistoryVisualization::pollWindowEndingAt(100u, 64u);

		CHECK(window.newestTick == 100u);
		CHECK(window.oldestTick == 37u);
		CHECK(window.tickCount() == 64u);
	}

	SECTION("an early session clamps at tick 0 rather than wrapping the subtraction")
	{
		// Unclamped, 5 - 63 wraps to the top of the tick space and the sweep either asks
		// for four billion ticks or asks for none. Both are silent.
		const PollWindow window = brawlerInputHistoryVisualization::pollWindowEndingAt(5u, 64u);

		CHECK(window.oldestTick == 0u);
		CHECK(window.newestTick == 5u);
		CHECK(window.tickCount() == 6u);
	}

	SECTION("tick 0 and a zero width both still name one real tick")
	{
		const PollWindow atZero = brawlerInputHistoryVisualization::pollWindowEndingAt(0u, 64u);
		const PollWindow noWidth = brawlerInputHistoryVisualization::pollWindowEndingAt(42u, 0u);

		CHECK(atZero.oldestTick == 0u);
		CHECK(atZero.tickCount() == 1u);
		CHECK(noWidth.oldestTick == 42u);
		CHECK(noWidth.tickCount() == 1u);
	}
}

TEST_CASE("Poll.TheGlyphClassifiesTheSameStickFieldTheMatcherDoes",
          "[CharacterViz][InputHistoryViz]")
{
	// THE TWO MOVEMENT VECTORS ARE MADE TO DISAGREE ON PURPOSE. In play they usually
	// point the same way, so a poll reading the wrong one passes every other test here.
	// inputSequence::matchSequence classifies a history entry on moveDirectionWorld.
	const simulatableBrawler::PlayerInput capture = makeCapture(
		/*aimDirection*/       glm::vec3(1.f, 0.f, 0.f),
		/*moveStick*/          glm::vec2(0.f, 1.f),
		/*moveDirectionWorld*/ glm::vec3(-1.f, 0.f, 0.f));

	const auto& machineInput = capture.get<dAttackMachineSimulation::PlayerInput>();
	const CaptureRowFields fields =
		brawlerInputHistoryVisualization::captureRowFieldsOf(machineInput, kDeadzone);

	// moveDirectionWorld is opposite the aim, so the row must read Back.
	CHECK(fields.direction == DirectionBucket::Back);

	// And the stick field, if it had been read, would have said something else entirely.
	CHECK(brawlerInputHistoryVisualization::directionBucketOf(
	          machineInput.moveDirection, glm::vec3(1.f, 0.f, 0.f), kDeadzone)
	      != DirectionBucket::Back);

	// Cross-checked against the matcher's own expression rather than against a literal.
	const std::optional<float> matcherAngle = inputSequence::aimRelativeAngle(
		glm::vec2(machineInput.moveDirectionWorld.x, machineInput.moveDirectionWorld.y),
		glm::vec3(machineInput.aimDirection.x, machineInput.aimDirection.y, 0.f),
		kDeadzone);

	REQUIRE(matcherAngle.has_value());
	CHECK(brawlerInputHistoryVisualization::nearestNamedDirection(*matcherAngle) == fields.direction);
}

TEST_CASE("Poll.TheRowFieldsAreExactlyTheTwoThePanelDraws",
          "[CharacterViz][InputHistoryViz]")
{
	SECTION("the button mask is the matcher's own mask, over all four held states")
	{
		uint32_t agreeing = 0u;

		for (int combination = 0; combination < 4; ++combination)
		{
			const bool leftAttack  = (combination & 0b01) != 0;
			const bool rightAttack = (combination & 0b10) != 0;

			const simulatableBrawler::PlayerInput capture = makeCapture(
				glm::vec3(1.f, 0.f, 0.f), glm::vec2(1.f, 0.f), glm::vec3(1.f, 0.f, 0.f),
				leftAttack, rightAttack);

			const CaptureRowFields fields = brawlerInputHistoryVisualization::captureRowFieldsOf(
				capture.get<dAttackMachineSimulation::PlayerInput>(), kDeadzone);

			if (fields.buttonMask == simulatableBrawler::motionButtonMask(leftAttack, rightAttack))
				++agreeing;
		}

		// Counted, so a mask that agreed only on the empty case cannot pass.
		CHECK(agreeing == 4u);
	}

	SECTION("a recorded motion match changes no row field, because the panel draws none")
	{
		// ⛔ ROW IDENTITY MUST EQUAL WHAT IS DRAWN. A match firing mid-hold would otherwise
		// split a row for a reason no one reading the panel could see.
		const simulatableBrawler::PlayerInput plain   = forwardCapture();
		const simulatableBrawler::PlayerInput matched = makeCapture(
			glm::vec3(1.f, 0.f, 0.f), glm::vec2(1.f, 0.f), glm::vec3(1.f, 0.f, 0.f),
			false, false, inputSequence::kHadoukenActionId);

		// The two captures really do differ; it is the ROW that ignores the difference.
		REQUIRE(matched.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
		        != plain.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId);

		const CaptureRowFields held = brawlerInputHistoryVisualization::captureRowFieldsOf(
			plain.get<dAttackMachineSimulation::PlayerInput>(), kDeadzone);
		const CaptureRowFields firing = brawlerInputHistoryVisualization::captureRowFieldsOf(
			matched.get<dAttackMachineSimulation::PlayerInput>(), kDeadzone);

		CHECK(held.direction == firing.direction);
		CHECK(held.buttonMask == firing.buttonMask);

		// And a hold spanning the match stays ONE row with its tick count intact.
		LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
		cache.push(0, plain);
		cache.push(1, matched);
		cache.push(2, plain);

		const simulatableBrawler::DelayLineMotionHistory history(cache);

		InputHistoryRowRing ring;
		brawlerInputHistoryVisualization::pollInputHistory(history, 2u, kDeadzone, ring);

		REQUIRE(ring.size() == 1u);
		CHECK(ring.newest().tickCount == 3u);
	}
}

TEST_CASE("Poll.ASweepRunsThroughTheProductionCacheAndItsProductionAdapter",
          "[CharacterViz][InputHistoryViz]")
{
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	for (uint32_t tick = 10u; tick < 15u; ++tick)
		cache.push(static_cast<std::int32_t>(tick), forwardCapture());

	// DelayLineMotionHistory is the SAME has()-gated adapter the motion matcher reads
	// through, so the display and the matcher cannot see different history.
	const simulatableBrawler::DelayLineMotionHistory history(cache);

	InputHistoryRowRing    ring;
	InputHistoryPollCounts counts;
	brawlerInputHistoryVisualization::mergeResidentCaptures(
		history, brawlerInputHistoryVisualization::pollWindowEndingAt(14u, 64u),
		kDeadzone, ring, counts);

	CHECK(counts.capturesPresented == 5u);
	CHECK(counts.capturesFolded == 5u);
	REQUIRE(ring.size() == 1u);
	CHECK(ring.newest().firstCaptureTick == 10u);
	CHECK(ring.newest().tickCount == 5u);
	CHECK(ring.newest().direction == DirectionBucket::Forward);
}

TEST_CASE("Poll.ARepeatedSweepOfTheSameWindowFoldsNothing", "[CharacterViz][InputHistoryViz]")
{
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	for (uint32_t tick = 0u; tick < 20u; ++tick)
		cache.push(static_cast<std::int32_t>(tick), forwardCapture());

	const simulatableBrawler::DelayLineMotionHistory history(cache);
	const PollWindow window = brawlerInputHistoryVisualization::pollWindowEndingAt(19u, 64u);

	InputHistoryRowRing    ring;
	InputHistoryPollCounts first;
	brawlerInputHistoryVisualization::mergeResidentCaptures(history, window, kDeadzone, ring, first);

	InputHistoryPollCounts second;
	brawlerInputHistoryVisualization::mergeResidentCaptures(history, window, kDeadzone, ring, second);

	// THIS GAP IS THE WHOLE LICENCE FOR POLLING AT RENDER RATE. The second sweep sees
	// exactly as much and folds none of it, so a 144 Hz poll of a 60 Hz source is free.
	CHECK(first.capturesPresented == 20u);
	CHECK(first.capturesFolded == 20u);
	CHECK(second.capturesPresented == 20u);
	CHECK(second.capturesFolded == 0u);

	REQUIRE(ring.size() == 1u);
	CHECK(ring.newest().tickCount == 20u);
	CHECK(ring.newest().lastCaptureTick() == 19u);
}

TEST_CASE("Poll.EveryCaptureSurvivesAPollPeriodShorterThanTheCacheWindow",
          "[CharacterViz][InputHistoryViz]")
{
	SECTION("polling every 63 ticks over a 64-tick cache loses nothing across 200 ticks")
	{
		const SessionResult session = runSession(/*tickCount*/ 200u,
		                                         /*pollPeriodTicks*/ 63u,
		                                         /*runLengthTicks*/ 25u);

		CHECK(session.coverage.totalTicks == 200u);
		CHECK(session.coverage.gaps == 0u);
		REQUIRE(session.ring.size() == 8u);
		CHECK(session.ring.oldest().firstCaptureTick == 0u);
		CHECK(session.ring.newest().lastCaptureTick() == 199u);
	}

	SECTION("polling every 80 ticks loses exactly the ticks the cache evicted, and shows it")
	{
		// 80 ticks pass between sweeps but only 64 are resident, so each of the two full
		// periods drops 16: sweeps see 16..79, 96..159 and 136..199, i.e. 168 of 200.
		const SessionResult session = runSession(/*tickCount*/ 200u,
		                                         /*pollPeriodTicks*/ 80u,
		                                         /*runLengthTicks*/ 25u);

		CHECK(session.coverage.totalTicks == 168u);

		// LOSS IS VISIBLE, NOT SILENT: a hole opens a new row rather than being folded
		// into its neighbour, so the display never overstates a hold it did not observe.
		CHECK(session.coverage.gaps == 1u);
		REQUIRE(session.ring.size() == 9u);
		CHECK(session.ring.oldest().firstCaptureTick == 16u);
		CHECK(session.ring.newest().lastCaptureTick() == 199u);
	}
}

TEST_CASE("Poll.AnAbsentTickOpensANewRowRatherThanExtendingOne", "[CharacterViz][InputHistoryViz]")
{
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	cache.push(0, forwardCapture());
	cache.push(1, forwardCapture());
	// Tick 2 is never captured -- a Stall tick pushes nothing, and has() reports it absent.
	cache.push(3, forwardCapture());
	cache.push(4, forwardCapture());

	const simulatableBrawler::DelayLineMotionHistory history(cache);

	InputHistoryRowRing    ring;
	InputHistoryPollCounts counts;
	brawlerInputHistoryVisualization::mergeResidentCaptures(
		history, brawlerInputHistoryVisualization::pollWindowEndingAt(4u, 64u),
		kDeadzone, ring, counts);

	CHECK(counts.capturesPresented == 4u);
	REQUIRE(ring.size() == 2u);
	CHECK(ring.oldest().tickCount == 2u);
	CHECK(ring.newest().firstCaptureTick == 3u);
	CHECK(ring.newest().tickCount == 2u);
}

TEST_CASE("Poll.TheJoinIsRebuiltEachSweepAndNeverAccumulates",
          "[CharacterViz][InputHistoryViz]")
{
	MockSlotReader corrected;
	corrected.setRef(2u, AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	corrected.setProvenance(2u, SlotStateProvenance::AuthorityAdopted);

	AppliedCaptureInversion inversion;

	const uint32_t firstFiled = brawlerInputHistoryVisualization::rebuildAppliedCaptureInversion(
		corrected,
		brawlerInputHistoryVisualization::pollWindowEndingAt(
			4u, brawlerInputHistoryVisualization::kAppliedPollWindowTicks),
		inversion);

	// The join window is clamped at tick 0 this early, so it names ticks 0..4 and no more.
	CHECK(firstFiled == 5u);
	CHECK(inversion.size() == 5u);

	// The join still answers for the corrected tick; only its DESTINATION has changed.
	REQUIRE(inversion.find(2u) != nullptr);
	CHECK(inversion.find(2u)->summary == RowProvenanceSummary::Corrected);

	// REBUILT, not accumulated: the second sweep's entries replace the first's outright,
	// which is why the inversion cannot grow past one resident window.
	const MockSlotReader aged{};
	const uint32_t       secondFiled = brawlerInputHistoryVisualization::rebuildAppliedCaptureInversion(
		aged,
		brawlerInputHistoryVisualization::pollWindowEndingAt(
			4u, brawlerInputHistoryVisualization::kAppliedPollWindowTicks),
		inversion);

	CHECK(secondFiled == firstFiled);
	CHECK(inversion.size() == 5u);

	// A sweep far downstream must name ITS OWN window and nothing else. This is what an
	// accumulating map fails: its stale entries crowd a fixed-capacity window out, so the
	// newest ticks -- the only ones anyone is looking at -- are the ones refused.
	const uint32_t laterFiled = brawlerInputHistoryVisualization::rebuildAppliedCaptureInversion(
		aged,
		brawlerInputHistoryVisualization::pollWindowEndingAt(
			200u, brawlerInputHistoryVisualization::kAppliedPollWindowTicks),
		inversion);

	CHECK(laterFiled == brawlerInputHistoryVisualization::kAppliedPollWindowTicks);
	CHECK(inversion.size() == brawlerInputHistoryVisualization::kAppliedPollWindowTicks);
	CHECK(inversion.find(0u) == nullptr);
}

TEST_CASE("Poll.OnTheAuthorityTheRingStillFillsAndEveryTickJoinsAsPending",
          "[CharacterViz][InputHistoryViz]")
{
	// The authority allocates no correction cache at all, so both seams answer NoSlot and
	// nullopt for every id. A listen-server HOST polls its own character on that manager,
	// so a sweep that assumed a cache would blank the host's display rather than the client's.
	LocalInputCache<simulatableBrawler::PlayerInput> cache(simulatableBrawler::getZeroPlayerInput());
	for (uint32_t tick = 0u; tick < 6u; ++tick)
		cache.push(static_cast<std::int32_t>(tick), forwardCapture());

	const simulatableBrawler::DelayLineMotionHistory history(cache);
	const MockSlotReader                             authority{};

	InputHistoryRowRing          ring;
	const InputHistoryPollCounts counts =
		brawlerInputHistoryVisualization::pollInputHistory(history, 5u, kDeadzone, ring);

	CHECK(counts.capturesFolded == 6u);
	REQUIRE(ring.size() == 1u);
	CHECK(ring.newest().tickCount == 6u);

	// And the join, asked separately, reads Pending for every one of those ticks.
	AppliedCaptureInversion inversion;
	brawlerInputHistoryVisualization::rebuildAppliedCaptureInversion(
		authority,
		brawlerInputHistoryVisualization::pollWindowEndingAt(
			5u, brawlerInputHistoryVisualization::kAppliedPollWindowTicks),
		inversion);

	std::size_t pending = 0u;
	for (std::size_t index = 0u; index < inversion.size(); ++index)
	{
		if (inversion.at(index).summary == RowProvenanceSummary::Pending)
			++pending;
	}

	CHECK(inversion.size() == 6u);
	CHECK(pending == inversion.size());
}

} // namespace inputhistorypolltests

#endif // WITH_LOW_LEVEL_TESTS
