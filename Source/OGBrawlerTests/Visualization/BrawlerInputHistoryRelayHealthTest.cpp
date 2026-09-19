// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the RELAY-HEALTH bar: per tick, did the scheduled relayed read find the capture it
// was looking for or fall back, and did that capture ever arrive.
//
// WHAT THIS SUITE IS REALLY GUARDING is the difference between "not yet" and "never".
// Both look like a fallback with no arrival, and only a horizon separates them: past
// `rollbackWindow + the store's own 64-tick capacity` an arrival could neither be replayed
// into the tick nor even be stored, so at that point -- and not one tick before it --
// "never" becomes a claim that cannot be withdrawn. A bar that said "never" early would
// accuse a relay that was merely slow.
//
// The second thing it guards is that the READ HALF of a cell is written once. The
// observation ring is last-write-wins, so a resim re-answers ticks this bar has already
// recorded; the answer it exists to show is the PREDICTION's, and re-reading a tick must
// never repaint a miss green.
//
// The third is that the bar's arrival never moved the meter. The 2-bar and 3-bar geometry
// is pinned to literals, and switching the stack's subject between a local and a remote
// character changes CELL CONTENT ONLY.

#include "catch_amalgamated.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryrelayhealth
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::FrameMeterBarCells;
using brawlerInputHistoryVisualization::FrameMeterBarKind;
using brawlerInputHistoryVisualization::FrameMeterBarSelection;
using brawlerInputHistoryVisualization::FrameMeterGeometry;
using brawlerInputHistoryVisualization::FrameMeterLayout;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneCellColor;
using brawlerInputHistoryVisualization::LaneCellFill;
using brawlerInputHistoryVisualization::LaneRunList;
using brawlerInputHistoryVisualization::PollWindow;
using brawlerInputHistoryVisualization::RelayHealthCell;
using brawlerInputHistoryVisualization::RelayHealthReadout;
using brawlerInputHistoryVisualization::RelayMissLabel;
using brawlerInputHistoryVisualization::RelayReadFacts;
using brawlerInputHistoryVisualization::RelayReadVerdict;
using brawlerInputHistoryVisualization::TickLanePollCounts;
using brawlerInputHistoryVisualization::buildRelayHealthReadout;
using brawlerInputHistoryVisualization::collectLaneRuns;
using brawlerInputHistoryVisualization::delayVerdictStyleOf;
using brawlerInputHistoryVisualization::frameMeterCellCount;
using brawlerInputHistoryVisualization::frameMeterGeometryFor;
using brawlerInputHistoryVisualization::frameMeterHeight;
using brawlerInputHistoryVisualization::kFrameMeterBarKindCount;
using brawlerInputHistoryVisualization::kInputDelayVerdictCount;
using brawlerInputHistoryVisualization::kLanePaletteMinCrossGap;
using brawlerInputHistoryVisualization::kLanePaletteMinPairGap;
using brawlerInputHistoryVisualization::kLaneElisionColor;
using brawlerInputHistoryVisualization::kMachineStateCellCount;
using brawlerInputHistoryVisualization::kRelayMissLabelCount;
using brawlerInputHistoryVisualization::kRelayReadVerdictCount;
using brawlerInputHistoryVisualization::kRowProvenanceSummaryCount;
using brawlerInputHistoryVisualization::kUnnamedLaneColor;
using brawlerInputHistoryVisualization::laneColorGap;
using brawlerInputHistoryVisualization::machineCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::provenanceCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::pollWindowEndingAt;
using brawlerInputHistoryVisualization::readRelayHealthBar;
using brawlerInputHistoryVisualization::relayHealthCellOf;
using brawlerInputHistoryVisualization::relayMissLabelLetter;
using brawlerInputHistoryVisualization::relayMissLabelOf;
using brawlerInputHistoryVisualization::relayNeverHorizonTicks;
using brawlerInputHistoryVisualization::relayReadVerdictStyleOf;
using brawlerInputHistoryVisualization::relayReadVerdictStyleOfOrdinal;
using brawlerInputHistoryVisualization::relayVerdictIsFallback;
using brawlerInputHistoryVisualization::InputDelayVerdict;


namespace
{

// The observation ring's SHAPE, driven by hand. The production ring is
// `RelayedReadObservationRing`; what this stands in for is the physics thread having
// already written it.
class ScriptedObservations
{
public:
	void note(const RelayedReadObservation& observation)
	{
		m_observations.push_back(observation);
	}

	// A read that found what it asked for.
	void hit(uint32_t simTick, uint32_t probeTick)
	{
		RelayedReadObservation observation;
		observation.simTick               = simTick;
		observation.probeTick             = probeTick;
		observation.probeTickFormed       = true;
		observation.appliedCaptureTick    = probeTick;
		observation.hasAppliedCaptureTick = true;
		observation.outcome               = ScheduledRelayedReadOutcome::Hit;
		note(observation);
	}

	// A read that asked for `probeTick`, did not get it, and fell back.
	void missed(uint32_t simTick, uint32_t probeTick, ScheduledRelayedReadMissClass missClass)
	{
		RelayedReadObservation observation;
		observation.simTick               = simTick;
		observation.probeTick             = probeTick;
		observation.probeTickFormed       = true;
		observation.appliedCaptureTick    = probeTick - 1u;
		observation.hasAppliedCaptureTick = true;
		observation.outcome               = ScheduledRelayedReadOutcome::Miss;
		observation.missClass             = missClass;
		note(observation);
	}

	// The join window: a read that formed no probe tick at all.
	void noProbe(uint32_t simTick)
	{
		RelayedReadObservation observation;
		observation.simTick = simTick;
		note(observation);
	}

	std::size_t size() const { return m_observations.size(); }

	const RelayedReadObservation* at(std::size_t index) const { return &m_observations[index]; }

	bool hasNewestSimTick() const { return !m_observations.empty(); }

	uint32_t newestSimTick() const
	{
		uint32_t newest = 0u;
		for (const RelayedReadObservation& observation : m_observations)
			newest = (observation.simTick > newest) ? observation.simTick : newest;

		return newest;
	}

private:
	std::vector<RelayedReadObservation> m_observations;
};

// The arrival ring's SHAPE. The production ring is `RelayedInputArrivalRing`; what this
// stands in for is the arrival door having already written it on the game thread.
class ScriptedArrivals
{
public:
	void arrived(uint32_t captureTick, uint32_t atSimTick)
	{
		m_arrivals.push_back(RelayedInputArrival{ captureTick, atSimTick });
	}

	const RelayedInputArrival* findArrival(uint32_t captureTick) const
	{
		for (const RelayedInputArrival& arrival : m_arrivals)
		{
			if (arrival.captureTick == captureTick)
				return &arrival;
		}
		return nullptr;
	}

private:
	std::vector<RelayedInputArrival> m_arrivals;
};

// The lane poll needs a slot reader; nothing in this suite reads provenance, so the
// simplest honest one answers "no cache" and files nothing.
struct NoAuthorityReader
{
	AppliedCaptureRef appliedCaptureRef(uint32_t) const { return AppliedCaptureRef{}; }
	std::optional<SlotStateProvenance> slotProvenance(uint32_t) const { return std::nullopt; }
	bool hasCorrectionCache() const { return false; }
};

CaptureRowFields movingInput()
{
	CaptureRowFields fields;
	fields.direction  = DirectionBucket::Forward;
	fields.buttonMask = 0u;
	return fields;
}

// One whole lane poll of a REMOTE proxy, through the production entry point.
TickLanePollCounts pollRemoteTick(uint32_t simTick, const ScriptedObservations& observations,
                                  const ScriptedArrivals& arrivals,
                                  uint32_t rollbackWindowTicks, InputHistoryTickLanes& lanes)
{
	NoAuthorityReader       reader;
	AppliedCaptureInversion inversion;

	return brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, movingInput(), false, std::nullopt, std::nullopt, std::nullopt,
		observations, arrivals, rollbackWindowTicks, inversion, lanes);
}

// The whole 8-field geometry, compared as BIT PATTERNS. A tolerance would pass exactly the
// sub-pixel drift an identity pin exists to refuse.
bool identicalBits(float left, float right)
{
	uint32_t leftBits  = 0u;
	uint32_t rightBits = 0u;
	std::memcpy(&leftBits, &left, sizeof(uint32_t));
	std::memcpy(&rightBits, &right, sizeof(uint32_t));
	return leftBits == rightBits;
}

bool identicalGeometry(const FrameMeterGeometry& left, const FrameMeterGeometry& right)
{
	return identicalBits(left.originX, right.originX)
	    && identicalBits(left.originY, right.originY)
	    && identicalBits(left.cellStride, right.cellStride)
	    && identicalBits(left.cellWidth, right.cellWidth)
	    && identicalBits(left.barHeight, right.barHeight)
	    && identicalBits(left.barGap, right.barGap)
	    && left.cellCount == right.cellCount
	    && left.barCount == right.barCount;
}

bool nearlyEqual(float left, float right)
{
	const float difference = (left > right) ? (left - right) : (right - left);
	return difference < 0.0005f;
}

// The facts a remote read leaves, with only the fields a row of the table cares about.
RelayReadFacts fallbackFacts(uint32_t simTick, uint32_t frontierSimTick,
                             uint32_t rollbackWindowTicks)
{
	RelayReadFacts facts;
	facts.simTick             = simTick;
	facts.frontierSimTick     = frontierSimTick;
	facts.rollbackWindowTicks = rollbackWindowTicks;
	facts.scheduledRead       = true;
	facts.missLabel           = RelayMissLabel::Loss;
	return facts;
}

} // namespace

// ===========================================================================
// 1. THE VERDICT TABLE -- EVERY ROW, AND BOTH EDGES.
// ===========================================================================

TEST_CASE("RelayHealth.EveryRowOfTheVerdictTableIsPinnedIncludingItsTwoEdges",
          "[CharacterViz][InputHistoryViz]")
{
	const uint32_t rollbackWindow = 12u;
	const uint32_t horizon        = relayNeverHorizonTicks(rollbackWindow);

	// ⛔ THE HORIZON IS THE STORE'S OWN CAPACITY PLUS THE WINDOW, not a literal: past it
	//   an arrival could neither be replayed nor even stored.
	CHECK(horizon == rollbackWindow + static_cast<uint32_t>(kRemoteInputCacheCapacityTicks));
	CHECK(horizon == 76u);

	// A hit is a hit whatever else is true of the tick.
	{
		RelayReadFacts facts = fallbackFacts(100u, 300u, rollbackWindow);
		facts.hit = true;
		const RelayHealthCell cell = relayHealthCellOf(facts);
		CHECK(cell.verdict == RelayReadVerdict::Hit);
		CHECK(cell.missLabel == RelayMissLabel::None);
		CHECK(cell.latenessTicks == 0u);
	}

	// No scheduled read -- rung 0, the join window's underflow guard, and the replay's ref
	// rung. ⛔ A REAL STATE, NOT A HOLE: nothing was asked for, so nothing was missing.
	{
		RelayReadFacts facts = fallbackFacts(100u, 300u, rollbackWindow);
		facts.scheduledRead = false;
		const RelayHealthCell cell = relayHealthCellOf(facts);
		CHECK(cell.verdict == RelayReadVerdict::Neutral);
		CHECK(cell.missLabel == RelayMissLabel::None);
	}

	// Fell back, nothing arrived, and the frontier is still inside the horizon.
	{
		const RelayHealthCell cell =
			relayHealthCellOf(fallbackFacts(100u, 100u + horizon - 1u, rollbackWindow));
		CHECK(cell.verdict == RelayReadVerdict::FallbackPending);
		CHECK(cell.missLabel == RelayMissLabel::Loss);
	}

	// Exactly at the horizon it is still Pending; one tick past it the claim becomes final.
	// ⭐ THE EDGE, BOTH SIDES.
	{
		CHECK(relayHealthCellOf(fallbackFacts(100u, 100u + horizon, rollbackWindow)).verdict
			== RelayReadVerdict::FallbackPending);
		CHECK(relayHealthCellOf(fallbackFacts(100u, 100u + horizon + 1u, rollbackWindow)).verdict
			== RelayReadVerdict::FallbackNeverArrived);
	}

	// Arrived inside the rollback window: a resim could still have run the tick on it.
	{
		RelayReadFacts facts = fallbackFacts(100u, 300u, rollbackWindow);
		facts.arrived          = true;
		facts.arrivedAtSimTick = 103u;
		const RelayHealthCell cell = relayHealthCellOf(facts);
		CHECK(cell.verdict == RelayReadVerdict::FallbackArrivedReplayable);
		CHECK(cell.latenessTicks == 3u);
	}

	// At exactly the window it is still replayable; one tick past it the capture can never
	// be applied to that tick at all.
	// ⭐ THE SECOND EDGE.
	{
		RelayReadFacts facts = fallbackFacts(100u, 300u, rollbackWindow);
		facts.arrived          = true;
		facts.arrivedAtSimTick = 100u + rollbackWindow;
		CHECK(relayHealthCellOf(facts).verdict == RelayReadVerdict::FallbackArrivedReplayable);
		CHECK(relayHealthCellOf(facts).latenessTicks == 12u);

		facts.arrivedAtSimTick = 100u + rollbackWindow + 1u;
		CHECK(relayHealthCellOf(facts).verdict == RelayReadVerdict::FallbackArrivedTooLate);
		CHECK(relayHealthCellOf(facts).latenessTicks == 13u);
	}

	// Arrived 40 ticks late, which the design names as the second arrival probe.
	{
		RelayReadFacts facts = fallbackFacts(100u, 300u, rollbackWindow);
		facts.arrived          = true;
		facts.arrivedAtSimTick = 140u;
		CHECK(relayHealthCellOf(facts).verdict == RelayReadVerdict::FallbackArrivedTooLate);
		CHECK(relayHealthCellOf(facts).latenessTicks == 40u);
	}

	// ⛔ AN ARRIVAL AT OR BEFORE THE TICK IS A RACE THE READ LOST, never a negative
	//   lateness wrapped into a vast unsigned.
	{
		RelayReadFacts facts = fallbackFacts(100u, 300u, rollbackWindow);
		facts.arrived          = true;
		facts.arrivedAtSimTick = 98u;
		CHECK(relayHealthCellOf(facts).latenessTicks == 0u);
		CHECK(relayHealthCellOf(facts).verdict == RelayReadVerdict::FallbackArrivedReplayable);
	}

	// Lateness saturates rather than wrapping through its own byte.
	{
		RelayReadFacts facts = fallbackFacts(100u, 900u, rollbackWindow);
		facts.arrived          = true;
		facts.arrivedAtSimTick = 900u;
		CHECK(relayHealthCellOf(facts).latenessTicks == 255u);
	}

	// An arrival outranks the horizon: a capture that got here is never "never".
	{
		RelayReadFacts facts = fallbackFacts(100u, 100u + horizon + 50u, rollbackWindow);
		facts.arrived          = true;
		facts.arrivedAtSimTick = 130u;
		CHECK(relayHealthCellOf(facts).verdict == RelayReadVerdict::FallbackArrivedTooLate);
	}
}

TEST_CASE("RelayHealth.OnlyAFallbackNamesACauseAndEachCauseHasItsOwnLetter",
          "[CharacterViz][InputHistoryViz]")
{
	using MissClass = ScheduledRelayedReadMissClass;
	using Outcome   = ScheduledRelayedReadOutcome;

	CHECK(relayMissLabelOf(Outcome::Miss, MissClass::InSpan) == RelayMissLabel::Loss);
	CHECK(relayMissLabelOf(Outcome::Miss, MissClass::AboveNewest) == RelayMissLabel::Starved);
	CHECK(relayMissLabelOf(Outcome::Miss, MissClass::BelowOldest) == RelayMissLabel::Evicted);
	CHECK(relayMissLabelOf(Outcome::VerifyFail, MissClass::NotAMiss) == RelayMissLabel::Verify);

	// ⛔ A READ THAT ASKED FOR NOTHING NAMES NO CAUSE: the underflow guard reports a Miss
	//   but formed no probe tick, and a hit missed nothing at all.
	CHECK(relayMissLabelOf(Outcome::Miss, MissClass::NoProbeTick) == RelayMissLabel::None);
	CHECK(relayMissLabelOf(Outcome::Hit, MissClass::NotAMiss) == RelayMissLabel::None);
	CHECK(relayMissLabelOf(Outcome::NoProbe, MissClass::NotAMiss) == RelayMissLabel::None);

	// A verify fail is a regime shift and the class field is meaningless on it.
	// ⭐ THE VERIFY ARM IS DECIDED BEFORE THE MISS CLASS IS EVEN LOOKED AT.
	CHECK(relayMissLabelOf(Outcome::VerifyFail, MissClass::InSpan) == RelayMissLabel::Verify);

	// One character each, and four distinct ones.
	CHECK(relayMissLabelLetter(RelayMissLabel::None) == '\0');
	CHECK(relayMissLabelLetter(RelayMissLabel::Loss) == 'L');
	CHECK(relayMissLabelLetter(RelayMissLabel::Starved) == 'S');
	CHECK(relayMissLabelLetter(RelayMissLabel::Evicted) == 'O');
	CHECK(relayMissLabelLetter(RelayMissLabel::Verify) == 'V');

	uint32_t distinctLetters = 0u;
	for (uint8_t left = 1u; left < kRelayMissLabelCount; ++left)
	{
		for (uint8_t right = static_cast<uint8_t>(left + 1u); right < kRelayMissLabelCount;
		     ++right)
		{
			if (relayMissLabelLetter(static_cast<RelayMissLabel>(left))
				!= relayMissLabelLetter(static_cast<RelayMissLabel>(right)))
			{
				++distinctLetters;
			}
		}
	}
	CHECK(distinctLetters == 6u);

	// One PAST the count pins the count itself, not merely the table.
	CHECK(relayMissLabelLetter(static_cast<RelayMissLabel>(kRelayMissLabelCount)) == '\0');
}

// ===========================================================================
// 2. THE PALETTE.
// ===========================================================================

TEST_CASE("RelayHealth.EveryVerdictHasItsOwnColourAndOnlyNoVerdictIsAHole",
          "[CharacterViz][InputHistoryViz]")
{
	uint32_t holes = 0u;
	for (uint8_t ordinal = 0u; ordinal < kRelayReadVerdictCount; ++ordinal)
	{
		if (relayReadVerdictStyleOfOrdinal(ordinal).fill == LaneCellFill::Hole)
			++holes;
	}

	// Seven real states, pairwise. A collapse of two would leave the bar readable and wrong.
	uint32_t distinctPairs = 0u;
	for (uint8_t left = 1u; left < kRelayReadVerdictCount; ++left)
	{
		for (uint8_t right = static_cast<uint8_t>(left + 1u); right < kRelayReadVerdictCount;
		     ++right)
		{
			if (laneColorGap(relayReadVerdictStyleOfOrdinal(left).color,
			                 relayReadVerdictStyleOfOrdinal(right).color)
				> kLanePaletteMinPairGap)
			{
				++distinctPairs;
			}
		}
	}

	CHECK(holes == 1u);
	CHECK(relayReadVerdictStyleOf(RelayReadVerdict::NoVerdict).fill == LaneCellFill::Hole);
	CHECK(distinctPairs == 21u);
	CHECK(relayReadVerdictStyleOfOrdinal(kRelayReadVerdictCount).fill == LaneCellFill::Unnamed);
}

TEST_CASE("RelayHealth.ThePaletteClearsTheCrossFloorAgainstTheThreeLanesAboveIt",
          "[CharacterViz][InputHistoryViz]")
{
	// Seven verdicts against nine provenance colours, four machine states and the two
	// out-of-palette markers: fifteen comparisons each, all clearing the cross floor.
	uint32_t separated = 0u;
	for (uint8_t verdict = 1u; verdict < kRelayReadVerdictCount; ++verdict)
	{
		const LaneCellColor color = relayReadVerdictStyleOfOrdinal(verdict).color;

		for (uint8_t provenance = 0u; provenance < kRowProvenanceSummaryCount; ++provenance)
		{
			if (laneColorGap(color, provenanceCellStyleOfOrdinal(provenance).color)
				> kLanePaletteMinCrossGap)
			{
				++separated;
			}
		}
		for (uint8_t machine = 1u; machine < kMachineStateCellCount; ++machine)
		{
			if (laneColorGap(color, machineCellStyleOfOrdinal(machine).color)
				> kLanePaletteMinCrossGap)
			{
				++separated;
			}
		}
		if (laneColorGap(color, kUnnamedLaneColor) > kLanePaletteMinCrossGap)
			++separated;
		if (laneColorGap(color, kLaneElisionColor) > kLanePaletteMinCrossGap)
			++separated;
	}

	CHECK(separated == 7u * 15u);
}

TEST_CASE("RelayHealth.HitIsTheDelayBarsAgreeGreenChannelForChannelAndOnPurpose",
          "[CharacterViz][InputHistoryViz]")
{
	// "I found the input I was looking for" is the same claim on both bars, and two greens
	// a shade apart would be read as two different claims. Pinned as an IDENTITY, so a
	// retune of either green that did not move the other fails here rather than drifting.
	// ⭐ THE ONE DELIBERATE COLLISION BETWEEN TWO OF THIS METER'S PALETTES.
	const LaneCellColor hit   = relayReadVerdictStyleOf(RelayReadVerdict::Hit).color;
	const LaneCellColor agree = delayVerdictStyleOf(InputDelayVerdict::Agree).color;

	CHECK(identicalBits(hit.r, agree.r));
	CHECK(identicalBits(hit.g, agree.g));
	CHECK(identicalBits(hit.b, agree.b));

	// And it is the ONLY one: every other relay colour clears the IN-PALETTE floor against
	// every delay colour, so no second pair can quietly become a near-collision. The two
	// bars are drawn one above the other, which is why the floor is asked of them at all.
	uint32_t separated = 0u;
	for (uint8_t verdict = 1u; verdict < kRelayReadVerdictCount; ++verdict)
	{
		if (static_cast<RelayReadVerdict>(verdict) == RelayReadVerdict::Hit)
			continue;

		for (uint8_t delay = 1u; delay < kInputDelayVerdictCount; ++delay)
		{
			if (laneColorGap(relayReadVerdictStyleOfOrdinal(verdict).color,
			                 brawlerInputHistoryVisualization::delayVerdictStyleOfOrdinal(delay)
			                     .color)
				> kLanePaletteMinPairGap)
			{
				++separated;
			}
		}
	}
	CHECK(separated == 6u * 6u);
}

// ===========================================================================
// 3. THE GEOMETRY -- WHAT THE FOURTH BAR COST, AND WHAT IT DID NOT.
// ===========================================================================

TEST_CASE("RelayHealth.AFourBarStackIsFourBarHeightsAndThreeGaps",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout layout;

	FrameMeterGeometry geometry;
	geometry.barHeight = layout.barHeight;
	geometry.barGap    = layout.barGap;

	geometry.barCount = 4u;
	CHECK(nearlyEqual(frameMeterHeight(geometry), 4.f * 14.f + 3.f * 3.f));
	CHECK(nearlyEqual(frameMeterHeight(geometry), 65.f));

	// The formula never mentions a kind, which is what makes the fourth bar a SLOT.
	// ⭐ THE TWO EXISTING BAR COUNTS ARE UNTOUCHED.
	geometry.barCount = 3u;
	CHECK(nearlyEqual(frameMeterHeight(geometry), 48.f));
	geometry.barCount = 2u;
	CHECK(nearlyEqual(frameMeterHeight(geometry), 31.f));

	CHECK(kFrameMeterBarKindCount == 4u);
}

TEST_CASE("RelayHealth.TheTwoBarAndThreeBarGeometryPinsAreUnmovedByTheFourthBar",
          "[CharacterViz][InputHistoryViz]")
{
	// ⛔ LITERALS, NOT A RECOMPUTATION. A pin that derived its own expectation from the
	//   same function would agree with any change that function made.
	const FrameMeterLayout layout;

	// 1920x1080, 120 cells: stride 8 (affordable 14.4 > 8), width 7, bar 960 wide,
	// originX (1920 - 960)/2 = 480, originY 1080 - 97.2 - height.
	const FrameMeterGeometry two =
		frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, 2u);
	CHECK(nearlyEqual(two.cellStride, 8.f));
	CHECK(nearlyEqual(two.cellWidth, 7.f));
	CHECK(nearlyEqual(two.originX, 480.f));
	CHECK(nearlyEqual(two.originY, 1080.f - 97.2f - 31.f));
	CHECK(nearlyEqual(two.originY, 951.8f));
	CHECK(two.barCount == 2u);

	const FrameMeterGeometry three =
		frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, 3u);
	CHECK(nearlyEqual(three.cellStride, 8.f));
	CHECK(nearlyEqual(three.cellWidth, 7.f));
	CHECK(nearlyEqual(three.originX, 480.f));
	CHECK(nearlyEqual(three.originY, 1080.f - 97.2f - 48.f));
	CHECK(nearlyEqual(three.originY, 934.8f));

	// ⭐ AND THE DEFAULT OVERLOAD STILL FORWARDS TWO, bit for bit -- the compatibility pin.
	CHECK(identicalGeometry(frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u), two));

	// 1280x720, the resolution the pair only just fits at.
	const FrameMeterGeometry three720 =
		frameMeterGeometryFor(layout, 1280.f, 720.f, 120u, 3u);
	CHECK(nearlyEqual(three720.originY, 720.f - 64.8f - 48.f));
	CHECK(nearlyEqual(three720.originY, 607.2f));
}

TEST_CASE("RelayHealth.SwitchingTheStackBetweenALocalAndARemoteChangesCellContentOnly",
          "[CharacterViz][InputHistoryViz]")
{
	// A bar that disappeared for a local subject would change the stack's bar count, and
	// with it the anchored stack's top edge and the primary's lift, so the whole display
	// would jump every time the fight moved.
	// ⭐ THE BAR STAYS PRESENT EITHER WAY.
	const FrameMeterLayout layout;

	InputHistoryTickLanes remoteLanes;
	ScriptedObservations  observations;
	ScriptedArrivals      arrivals;

	for (uint32_t tick = 500u; tick < 506u; ++tick)
	{
		observations.hit(tick, tick - 3u);
		pollRemoteTick(tick, observations, arrivals, 12u, remoteLanes);
	}

	// A locally controlled character is never polled for relay health at all: its lanes
	// hold no cell, and the reader's locality argument is what fills the bar.
	InputHistoryTickLanes localLanes;

	const PollWindow window = pollWindowEndingAt(505u, 6u);

	FrameMeterBarCells remoteBar;
	FrameMeterBarCells localBar;
	readRelayHealthBar(remoteLanes, window, /*isLocallyControlled=*/false, remoteBar);
	readRelayHealthBar(localLanes, window, /*isLocallyControlled=*/true, localBar);

	// Same number of cells, all filled on both -- the SHAPE is identical.
	CHECK(remoteBar.count == localBar.count);

	uint32_t remoteFilled = 0u;
	uint32_t localFilled  = 0u;
	uint32_t differingCells = 0u;
	for (uint32_t offset = 0u; offset < remoteBar.count; ++offset)
	{
		if (remoteBar.cells[offset].filled)
			++remoteFilled;
		if (localBar.cells[offset].filled)
			++localFilled;
		if (remoteBar.cells[offset].value != localBar.cells[offset].value)
			++differingCells;

		if (localBar.cells[offset].filled)
		{
			CHECK(localBar.cells[offset].value
				== static_cast<uint8_t>(RelayReadVerdict::LocalNoRelay));
		}
	}

	CHECK(remoteFilled == 6u);
	CHECK(localFilled == 6u);
	CHECK(differingCells == 6u);

	// ⛔ AND THE GEOMETRY IS BIT-IDENTICAL EITHER WAY: `frameMeterGeometryFor` is asked the
	//   same bar count on both, because the bar is present on both.
	const FrameMeterGeometry remoteGeometry = frameMeterGeometryFor(layout, 1920.f, 1080.f,
		frameMeterCellCount(window), 4u);
	const FrameMeterGeometry localGeometry = frameMeterGeometryFor(layout, 1920.f, 1080.f,
		frameMeterCellCount(window), 4u);
	CHECK(identicalGeometry(remoteGeometry, localGeometry));

	// The readout swaps the same way: facts, not a row of zeroes.
	const RelayHealthReadout localReadout =
		buildRelayHealthReadout(localLanes, window, /*isLocallyControlled=*/true);
	CHECK(localReadout.present);
	CHECK(localReadout.local);
	CHECK(localReadout.hits == 0u);

	const RelayHealthReadout remoteReadout =
		buildRelayHealthReadout(remoteLanes, window, /*isLocallyControlled=*/false);
	CHECK(remoteReadout.present);
	CHECK_FALSE(remoteReadout.local);
	CHECK(remoteReadout.hits == 6u);
}

// ===========================================================================
// 4. THE TWO WRITE POLICIES.
// ===========================================================================

TEST_CASE("RelayHealth.APendingCellIsRewrittenWhenItsCaptureArrivesAndIsNeverDuplicated",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	ScriptedObservations  observations;
	ScriptedArrivals      arrivals;

	observations.missed(600u, /*probeTick=*/597u, ScheduledRelayedReadMissClass::InSpan);
	const TickLanePollCounts first = pollRemoteTick(600u, observations, arrivals, 12u, lanes);

	CHECK(first.relayCellsRecorded == 1u);
	CHECK(first.relayCellsUpdated == 0u);

	const uint32_t laneTick = *lanes.gate().laneTickOf(600u);
	REQUIRE(lanes.relayHealthCellAt(laneTick) != nullptr);
	CHECK(lanes.relayHealthCellAt(laneTick)->verdict == RelayReadVerdict::FallbackPending);
	CHECK(lanes.relayHealthCellAt(laneTick)->missLabel == RelayMissLabel::Loss);

	const std::size_t storedAfterFirst = lanes.relayHealth().storedCellCount();

	// The capture lands four ticks later, inside the rollback window.
	observations.hit(601u, /*probeTick=*/598u);
	arrivals.arrived(/*captureTick=*/597u, /*atSimTick=*/604u);
	const TickLanePollCounts second = pollRemoteTick(601u, observations, arrivals, 12u, lanes);

	// `UpdatedCell` is the write result the arrival half exists to produce, and the lane is
	// addressed by tick so it cannot duplicate.
	// ⭐ ONE CELL UPDATED, NOT ADDED.
	CHECK(second.relayCellsUpdated == 1u);
	CHECK(lanes.relayHealthCellAt(laneTick)->verdict
		== RelayReadVerdict::FallbackArrivedReplayable);
	CHECK(lanes.relayHealthCellAt(laneTick)->latenessTicks == 4u);

	// ⛔ THE READ HALF IS UNCHANGED BY THE ARRIVAL: the cause belongs to the read that fell
	//   back, and an arrival knows nothing about why the read missed.
	CHECK(lanes.relayHealthCellAt(laneTick)->missLabel == RelayMissLabel::Loss);

	// The lane grew by exactly the new tick's cell, never by a second copy of tick 600.
	CHECK(lanes.relayHealth().storedCellCount() == storedAfterFirst + 1u);
}

TEST_CASE("RelayHealth.ARereadOfATickCannotRepaintItsMissAsAHit",
          "[CharacterViz][InputHistoryViz]")
{
	// A resim re-answers ticks this bar has already drawn. The answer the bar exists to
	// show is what the PREDICTION ran on: a re-read that repainted a miss green would erase
	// exactly the signal it was built for.
	// ⭐ THE OBSERVATION RING IS LAST-WRITE-WINS.
	InputHistoryTickLanes lanes;
	ScriptedObservations  observations;
	ScriptedArrivals      arrivals;

	observations.missed(700u, /*probeTick=*/697u, ScheduledRelayedReadMissClass::AboveNewest);
	pollRemoteTick(700u, observations, arrivals, 12u, lanes);

	const uint32_t laneTick = *lanes.gate().laneTickOf(700u);
	REQUIRE(lanes.relayHealthCellAt(laneTick) != nullptr);
	CHECK(lanes.relayHealthCellAt(laneTick)->verdict == RelayReadVerdict::FallbackPending);

	// The replay re-answers tick 700 as a Hit -- the ring's own last-write-wins rule.
	ScriptedObservations replayed;
	replayed.hit(700u, /*probeTick=*/697u);
	replayed.hit(701u, /*probeTick=*/698u);
	const TickLanePollCounts counts = pollRemoteTick(701u, replayed, arrivals, 12u, lanes);

	CHECK(lanes.relayHealthCellAt(laneTick)->verdict == RelayReadVerdict::FallbackPending);
	CHECK(lanes.relayHealthCellAt(laneTick)->missLabel == RelayMissLabel::Starved);
	CHECK(counts.relayCellsIgnored == 1u);

	// ⛔ AND THE RULE IS NOT "IGNORE EVERYTHING": tick 701 had no cell yet and is recorded.
	CHECK(counts.relayCellsRecorded == 1u);
	CHECK(lanes.relayHealthCellAt(*lanes.gate().laneTickOf(701u))->verdict
		== RelayReadVerdict::Hit);

	// A cell that is not a fallback has no arrival half left to learn, which is what the
	// predicate above says in one place.
	CHECK(relayVerdictIsFallback(RelayReadVerdict::FallbackPending));
	CHECK(relayVerdictIsFallback(RelayReadVerdict::FallbackArrivedReplayable));
	CHECK(relayVerdictIsFallback(RelayReadVerdict::FallbackArrivedTooLate));
	CHECK(relayVerdictIsFallback(RelayReadVerdict::FallbackNeverArrived));
	CHECK_FALSE(relayVerdictIsFallback(RelayReadVerdict::Hit));
	CHECK_FALSE(relayVerdictIsFallback(RelayReadVerdict::Neutral));
	CHECK_FALSE(relayVerdictIsFallback(RelayReadVerdict::LocalNoRelay));
	CHECK_FALSE(relayVerdictIsFallback(RelayReadVerdict::NoVerdict));
}

// ===========================================================================
// 5. THE RUN LETTERS AND THE READOUT.
// ===========================================================================

TEST_CASE("RelayHealth.ACausesRunEndsWhereTheCauseChangesEvenWhenTheColourDoesNot",
          "[CharacterViz][InputHistoryViz]")
{
	// Two neighbouring cells of one colour whose fallbacks had different causes are two
	// runs, or the letter on the second would speak for the first as well.
	// ⭐ A RUN IS WHAT ONE LABEL CAN TRUTHFULLY BE PRINTED ON.
	InputHistoryTickLanes lanes;
	ScriptedObservations  observations;
	ScriptedArrivals      arrivals;

	const ScheduledRelayedReadMissClass script[] = {
		ScheduledRelayedReadMissClass::InSpan,
		ScheduledRelayedReadMissClass::InSpan,
		ScheduledRelayedReadMissClass::AboveNewest,
		ScheduledRelayedReadMissClass::AboveNewest,
		ScheduledRelayedReadMissClass::AboveNewest,
	};

	for (uint32_t index = 0u; index < 5u; ++index)
	{
		const uint32_t tick = 800u + index;
		observations.missed(tick, tick - 3u, script[index]);
		pollRemoteTick(tick, observations, arrivals, 12u, lanes);
	}

	const PollWindow   window = pollWindowEndingAt(804u, 5u);
	FrameMeterBarCells bar;
	readRelayHealthBar(lanes, window, /*isLocallyControlled=*/false, bar);

	LaneRunList runs;
	collectLaneRuns(bar, runs);

	REQUIRE(runs.count == 2u);
	CHECK(runs.runs[0].length == 2u);
	CHECK(relayMissLabelLetter(static_cast<RelayMissLabel>(runs.runs[0].label)) == 'L');
	CHECK(runs.runs[1].length == 3u);
	CHECK(relayMissLabelLetter(static_cast<RelayMissLabel>(runs.runs[1].label)) == 'S');

	// Every cell in the two runs carries the same colour, so the split is the LABEL's.
	CHECK(runs.runs[0].value == runs.runs[1].value);

	// ⛔ A RUN WITH NO CAUSE GETS NO LETTER -- a hit names nothing.
	InputHistoryTickLanes hitLanes;
	ScriptedObservations  hits;
	for (uint32_t index = 0u; index < 3u; ++index)
	{
		hits.hit(900u + index, 897u + index);
		pollRemoteTick(900u + index, hits, arrivals, 12u, hitLanes);
	}

	FrameMeterBarCells hitBar;
	readRelayHealthBar(hitLanes, pollWindowEndingAt(902u, 3u), false, hitBar);
	LaneRunList hitRuns;
	collectLaneRuns(hitBar, hitRuns);
	REQUIRE(hitRuns.count == 1u);
	CHECK(relayMissLabelLetter(static_cast<RelayMissLabel>(hitRuns.runs[0].label)) == '\0');
}

TEST_CASE("RelayHealth.TheReadoutTalliesTheWindowAndItsLatenessIsTheMiddleOne",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	ScriptedObservations  observations;
	ScriptedArrivals      arrivals;

	// Five arrivals at 1, 2, 4, 9 and 30 ticks late; the middle one is 4.
	const uint32_t lateness[] = { 1u, 2u, 4u, 9u, 30u };
	for (uint32_t index = 0u; index < 5u; ++index)
	{
		const uint32_t tick = 1000u + index;
		observations.missed(tick, tick - 3u, ScheduledRelayedReadMissClass::InSpan);
		arrivals.arrived(tick - 3u, tick + lateness[index]);
	}

	// Two more ticks: one hit, one fallback that never arrives and is past the horizon.
	observations.hit(1005u, 1002u);
	observations.missed(1006u, 1003u, ScheduledRelayedReadMissClass::AboveNewest);

	// A frontier far past the horizon makes the unarrived cell final rather than pending.
	observations.hit(1200u, 1197u);

	for (uint32_t tick = 1000u; tick <= 1006u; ++tick)
		pollRemoteTick(tick, observations, arrivals, 12u, lanes);
	pollRemoteTick(1200u, observations, arrivals, 12u, lanes);

	const PollWindow window = pollWindowEndingAt(
		lanes.newestAxisTick(), brawlerInputHistoryVisualization::kTickLaneCapacity);

	const RelayHealthReadout readout =
		buildRelayHealthReadout(lanes, window, /*isLocallyControlled=*/false);

	CHECK(readout.present);
	CHECK_FALSE(readout.local);
	CHECK(readout.hits == 2u);
	// 1, 2, 4 and 9 ticks late all sit inside the 12-tick rollback window; only 30 does not.
	CHECK(readout.arrivedReplayable == 4u);
	CHECK(readout.arrivedTooLate == 1u);
	CHECK(readout.neverArrived == 1u);
	CHECK(readout.fallbackPending == 0u);
	CHECK(readout.loss == 5u);
	CHECK(readout.starved == 1u);

	// Four, not the mean of 9.2 -- a value no cell in the window actually has.
	// ⭐ THE MEDIAN IS THE MIDDLE OF THE FIVE ARRIVED CELLS.
	CHECK(readout.medianLatenessKnown);
	CHECK(readout.medianLatenessTicks == 4u);
}

TEST_CASE("RelayHealth.AnEmptyWindowDrawsNothingRatherThanARowOfZeroes",
          "[CharacterViz][InputHistoryViz]")
{
	// ⛔ ABSENT IS NOT ZERO: a tally of 0/0/0 states a measurement nobody made.
	InputHistoryTickLanes lanes;
	const RelayHealthReadout readout =
		buildRelayHealthReadout(lanes, pollWindowEndingAt(0u, 10u), /*isLocallyControlled=*/false);

	CHECK_FALSE(readout.present);
	CHECK_FALSE(readout.local);
	CHECK_FALSE(readout.medianLatenessKnown);
}

} // namespace inputhistoryrelayhealth

#endif // WITH_LOW_LEVEL_TESTS
