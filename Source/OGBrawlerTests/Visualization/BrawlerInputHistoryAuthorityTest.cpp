// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the AUTHORITY MARKER -- the vertical rule that says where the server actually is,
// and the prediction offset printed beneath it.
//
// WHAT THIS SUITE IS REALLY GUARDING is that the marker never names a tick the authority
// is not at. The client's lane axis is compacted: an idle span collapses to one cell, so
// the authority tick can land on time that has no cell of its own -- and the tempting
// repairs are both lies. Clamping to the nearest recorded tick points at a tick the server
// is demonstrably not on; hiding the marker reads as "no data" when the truth is "inside
// that collapsed span". The cases below construct both situations and pin the third answer:
// the span's own cell, which already stands for exactly that stretch of time.
//
// The second thing it guards is that this rule and the frozen horizon cannot be confused.
// They mean opposite things -- one says the cache can no longer answer, the other says the
// server is here -- and a reader who swapped them would draw the opposite conclusion about
// a desync. Their styles are swept field by field so no single edit can collapse them.
//
// The gate itself is NOT re-tested here: BrawlerInputHistoryPause owns the predicate, the
// hysteresis and the ledger. What is tested is where a marker lands on what the gate left.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryauthoritytests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::AuthorityMarkerAnchor;
using brawlerInputHistoryVisualization::AuthorityMarkerKind;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::FrameMeterAuthorityMarker;
using brawlerInputHistoryVisualization::FrameMeterGeometry;
using brawlerInputHistoryVisualization::FrameMeterLayout;
using brawlerInputHistoryVisualization::FrameMeterMarkerShape;
using brawlerInputHistoryVisualization::FrameMeterMarkerStyle;
using brawlerInputHistoryVisualization::FrameMeterSimTickPlacement;
using brawlerInputHistoryVisualization::InputDelayVerdict;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneAdmission;
using brawlerInputHistoryVisualization::LaneAxisEvent;
using brawlerInputHistoryVisualization::PollWindow;
using brawlerInputHistoryVisualization::PredictionOffsetReading;

using brawlerInputHistoryVisualization::authorityMarkerStyleOf;
using brawlerInputHistoryVisualization::authorityMarkerX;
using brawlerInputHistoryVisualization::authorityTickOf;
using brawlerInputHistoryVisualization::delayVerdictStyleOfOrdinal;
using brawlerInputHistoryVisualization::frameMeterAuthorityLabelTopY;
using brawlerInputHistoryVisualization::frameMeterAuthorityMarkerOf;
using brawlerInputHistoryVisualization::frameMeterCellCount;
using brawlerInputHistoryVisualization::frameMeterColumnOfLaneTick;
using brawlerInputHistoryVisualization::frameMeterCellX;
using brawlerInputHistoryVisualization::frameMeterDelayReadoutTopY;
using brawlerInputHistoryVisualization::frameMeterElisionLabelTopY;
using brawlerInputHistoryVisualization::frameMeterGeometryFor;
using brawlerInputHistoryVisualization::frameMeterHeight;
using brawlerInputHistoryVisualization::frameMeterWidth;
using brawlerInputHistoryVisualization::laneColorGap;
using brawlerInputHistoryVisualization::machineCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::placeFrameMeterSimTick;
using brawlerInputHistoryVisualization::provenanceCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::retainedLaneWindow;

using brawlerInputHistoryVisualization::kFrameMeterAuthorityOffBarStyle;
using brawlerInputHistoryVisualization::kFrameMeterAuthorityStyle;
using brawlerInputHistoryVisualization::kFrameMeterHorizonStyle;
using brawlerInputHistoryVisualization::kInputDelayVerdictCount;
using brawlerInputHistoryVisualization::kLaneElisionColor;
using brawlerInputHistoryVisualization::kLaneResyncColor;
using brawlerInputHistoryVisualization::kLaneElisionLedgerCapacity;
using brawlerInputHistoryVisualization::kLanePaletteMinCrossGap;
using brawlerInputHistoryVisualization::kLanePaletteMinPairGap;
using brawlerInputHistoryVisualization::kLanePauseEngageTicks;
using brawlerInputHistoryVisualization::kMachineStateCellCount;
using brawlerInputHistoryVisualization::kRowProvenanceSummaryCount;
using brawlerInputHistoryVisualization::kUnnamedLaneColor;

// ---------------------------------------------------------------------------
// Helpers. The provenance reader answers nothing: what is under test is where a marker
// lands on the lane AXIS, and the axis is the gate's, not the cells'.
// ---------------------------------------------------------------------------

class SilentReader
{
public:
	void setHasCorrectionCache(bool hasCache) { m_hasCorrectionCache = hasCache; }

	AppliedCaptureRef appliedCaptureRef(uint32_t) const { return AppliedCaptureRef{}; }
	std::optional<SlotStateProvenance> slotProvenance(uint32_t) const { return std::nullopt; }
	bool hasCorrectionCache() const { return m_hasCorrectionCache; }

private:
	bool m_hasCorrectionCache = true;
};

CaptureRowFields neutralInput()
{
	CaptureRowFields fields;
	fields.direction  = DirectionBucket::Neutral;
	fields.buttonMask = 0u;
	return fields;
}

CaptureRowFields movingInput()
{
	CaptureRowFields fields;
	fields.direction  = DirectionBucket::Forward;
	fields.buttonMask = 0u;
	return fields;
}

// `offsetTicks` is the estimator's, or nullopt for a role that does not predict. The
// poll -- not the draw -- is where it enters, which is the property this suite guards.
void pollTick(uint32_t simTick, bool active, InputHistoryTickLanes& lanes,
              std::optional<uint32_t> offsetTicks = std::nullopt)
{
	const SilentReader      reader;
	AppliedCaptureInversion inversion;

	brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, active ? movingInput() : neutralInput(), true, offsetTicks,
		std::nullopt, std::nullopt, inversion, lanes);
}

// `activeTicks` moving ticks, then `idleTicks` neutral ones, starting at `firstTick`.
uint32_t pollRun(uint32_t firstTick, uint32_t activeTicks, uint32_t idleTicks,
                 InputHistoryTickLanes& lanes,
                 std::optional<uint32_t> offsetTicks = std::nullopt)
{
	uint32_t tick = firstTick;
	for (uint32_t step = 0u; step < activeTicks; ++step, ++tick)
		pollTick(tick, true, lanes, offsetTicks);
	for (uint32_t step = 0u; step < idleTicks; ++step, ++tick)
		pollTick(tick, false, lanes, offsetTicks);

	return tick;
}

FrameMeterGeometry geometryFor(uint32_t cellCount)
{
	const FrameMeterLayout layout;
	return frameMeterGeometryFor(layout, 1280.f, 720.f, cellCount);
}

// The marker as the DRAW receives it. ⛔ THE READING IS FILED ON THE LANES FIRST, because
//   that is the only way it reaches a draw that holds them as const.
FrameMeterAuthorityMarker markerFor(InputHistoryTickLanes& lanes, uint32_t retainedTicks,
                                    uint32_t predictionTick, uint32_t offsetTicks)
{
	lanes.noteAuthorityReading(offsetTicks, predictionTick);

	const PollWindow window = retainedLaneWindow(lanes, retainedTicks);
	return frameMeterAuthorityMarkerOf(lanes, window);
}

bool nearlyEqual(float left, float right)
{
	const float delta = (left > right) ? (left - right) : (right - left);
	return delta < 0.01f;
}

// Both palettes, the six delay verdicts, the three out-of-palette cell colours, and nothing
// else -- these are every colour the authority rule can find itself drawn on top of.
std::vector<brawlerInputHistoryVisualization::LaneCellColor> everyCellColor()
{
	std::vector<brawlerInputHistoryVisualization::LaneCellColor> colors;

	for (uint8_t ordinal = 0u; ordinal < kRowProvenanceSummaryCount; ++ordinal)
		colors.push_back(provenanceCellStyleOfOrdinal(ordinal).color);
	for (uint8_t ordinal = 1u; ordinal < kMachineStateCellCount; ++ordinal)
		colors.push_back(machineCellStyleOfOrdinal(ordinal).color);
	// Ordinal 0 is NoVerdict, the hole -- it carries no colour to sweep.
	for (uint8_t ordinal = 1u; ordinal < kInputDelayVerdictCount; ++ordinal)
		colors.push_back(delayVerdictStyleOfOrdinal(ordinal).color);

	colors.push_back(kUnnamedLaneColor);
	colors.push_back(kLaneElisionColor);
	colors.push_back(kLaneResyncColor);
	return colors;
}

// ---------------------------------------------------------------------------
// THE OFFSET, AND THE TICK IT NAMES
// ---------------------------------------------------------------------------

TEST_CASE("Authority.TheAuthorityTickIsThePredictionTickLessTheOffsetAndNothingElse",
          "[CharacterViz][InputHistoryViz]")
{
	// The estimator forms its target as authorityTick + offset, so this subtraction is the
	// inverse of the production arithmetic rather than a second opinion about it.
	CHECK(authorityTickOf(PredictionOffsetReading{ 600u, 4u }) == 596u);
	CHECK(authorityTickOf(PredictionOffsetReading{ 600u, 0u }) == 600u);
	CHECK(authorityTickOf(PredictionOffsetReading{ 600u, 600u }) == 0u);

	// Early session: the offset floor is live before the first tick has been simulated, and
	// an unsigned wrap here would put the marker at the top of the tick space.
	CHECK(authorityTickOf(PredictionOffsetReading{ 3u, 10u }) == 0u);
	CHECK(authorityTickOf(PredictionOffsetReading{ 0u, 4u }) == 0u);
}

TEST_CASE("Authority.TheMarkerSitsOnTheCellForPredictionMinusTheOffset",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	pollRun(0u, 60u, 0u, lanes);

	// Nothing was elided, so the lane axis and the sim axis still agree tick for tick.
	REQUIRE(lanes.hasAxis());
	REQUIRE(lanes.newestAxisTick() == 59u);

	const FrameMeterAuthorityMarker marker = markerFor(lanes, 120u, 59u, 4u);

	CHECK(marker.kind == AuthorityMarkerKind::OnCell);
	CHECK(marker.anchor == AuthorityMarkerAnchor::Column);
	CHECK(marker.authorityTick == 55u);
	CHECK(marker.offsetTicks == 4u);
	CHECK(marker.barOffset == 55u);

	// The property a reader actually uses: counting columns from the newest cell back to
	// the rule gives the offset. Anything else makes the printed number and the picture
	// disagree, which is worse than showing neither.
	const PollWindow window    = retainedLaneWindow(lanes, 120u);
	const uint32_t   cellCount = frameMeterCellCount(window);
	CHECK(cellCount == 60u);
	CHECK(cellCount - 1u - marker.barOffset == marker.offsetTicks);
}

TEST_CASE("Authority.TheMarkerFollowsTheOffsetWhenItMoves",
          "[CharacterViz][InputHistoryViz]")
{
	// The offset is derived from RTT and jitter and floored by config, so it changes under
	// the display. A marker cached across frames would go on pointing at a stale tick.
	InputHistoryTickLanes lanes;
	pollRun(0u, 60u, 0u, lanes);

	uint32_t previousOffset = 0u;
	for (uint32_t offsetTicks = 4u; offsetTicks <= 12u; ++offsetTicks)
	{
		const FrameMeterAuthorityMarker marker = markerFor(lanes, 120u, 59u, offsetTicks);

		CHECK(marker.anchor == AuthorityMarkerAnchor::Column);
		CHECK(marker.barOffset == 59u - offsetTicks);
		CHECK(marker.offsetTicks == offsetTicks);

		if (offsetTicks != 4u)
			CHECK(marker.barOffset < previousOffset);

		previousOffset = marker.barOffset;
	}
}

// ---------------------------------------------------------------------------
// THE ELIDED TARGET -- THE CASE THE COMPACTED AXIS MADE REAL
// ---------------------------------------------------------------------------

TEST_CASE("Authority.AnAuthorityTickInsideAClosedElidedSpanLandsOnThatSpansOwnCell",
          "[CharacterViz][InputHistoryViz]")
{
	// Ten moving ticks, then a long stand-still, then movement again: the standing still is
	// collapsed and the ticks inside it have no cell of their own.
	InputHistoryTickLanes lanes;
	const uint32_t        afterIdle = pollRun(0u, 10u, 35u, lanes);
	pollRun(afterIdle, 15u, 0u, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);

	// The first fifteen idle ticks are still recorded -- the hysteresis has to run out
	// first -- so the span opens well after the player stopped.
	REQUIRE(span.simTick == 10u + kLanePauseEngageTicks);
	REQUIRE(span.skippedTicks == 45u - (10u + kLanePauseEngageTicks));
	REQUIRE_FALSE(lanes.gate().paused());

	const uint32_t insideTheSpan = span.simTick + span.skippedTicks / 2u;
	REQUIRE_FALSE(lanes.gate().laneTickOf(insideTheSpan).has_value());

	const uint32_t predictionTick = 59u;
	const FrameMeterAuthorityMarker marker =
		markerFor(lanes, 120u, predictionTick, predictionTick - insideTheSpan);

	CHECK(marker.authorityTick == insideTheSpan);
	CHECK(marker.kind == AuthorityMarkerKind::OnElidedSpan);
	CHECK(marker.anchor == AuthorityMarkerAnchor::Column);

	// ON the span's marker cell: that cell already stands for exactly this stretch of time.
	CHECK(marker.barOffset == span.laneTick);

	// And demonstrably NOT clamped. These two are the recorded ticks either side of the
	// span, which is where a nearest-neighbour repair would have put the rule.
	const std::optional<uint32_t> lastBefore = lanes.gate().laneTickOf(span.simTick - 1u);
	const std::optional<uint32_t> firstAfter =
		lanes.gate().laneTickOf(span.simTick + span.skippedTicks);
	REQUIRE(lastBefore.has_value());
	REQUIRE(firstAfter.has_value());
	CHECK(marker.barOffset != *lastBefore);
	CHECK(marker.barOffset != *firstAfter);
}

TEST_CASE("Authority.EveryTickInsideAClosedSpanResolvesToTheSameCellAndNoneIsHidden",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	const uint32_t        afterIdle = pollRun(0u, 10u, 35u, lanes);
	pollRun(afterIdle, 15u, 0u, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);

	// Sweeping the whole span rather than one tick inside it: a marker that disappeared
	// anywhere in here would read as "no data" when the truth is "inside that span".
	uint32_t placed = 0u;
	for (uint32_t simTick = span.simTick; simTick < span.simTick + span.skippedTicks; ++simTick)
	{
		const FrameMeterAuthorityMarker marker = markerFor(lanes, 120u, 59u, 59u - simTick);

		CHECK(marker.kind == AuthorityMarkerKind::OnElidedSpan);
		CHECK(marker.barOffset == span.laneTick);
		if (marker.anchor == AuthorityMarkerAnchor::Column)
			++placed;
	}

	CHECK(placed == span.skippedTicks);
}

TEST_CASE("Authority.WhileTheIdleSpanIsStillOpenTheMarkerFlagsTheRightEdgeInstead",
          "[CharacterViz][InputHistoryViz]")
{
	// Standing still is the steady state this display spends most of its time in, and the
	// span being collapsed right now has no ledger entry and no cell to land on yet.
	InputHistoryTickLanes lanes;
	const uint32_t        next = pollRun(0u, 10u, 31u, lanes);

	REQUIRE(lanes.gate().paused());
	REQUIRE(lanes.gate().axisEventCount() == 0u);

	const uint32_t predictionTick = next - 1u;
	const FrameMeterAuthorityMarker marker = markerFor(lanes, 120u, predictionTick, 4u);

	CHECK(marker.kind == AuthorityMarkerKind::InsideOpenSpan);
	CHECK(marker.anchor == AuthorityMarkerAnchor::RightEdge);
	CHECK(marker.offsetTicks == 4u);

	// It is not silently parked on the newest recorded cell, which is what a reader would
	// otherwise take for "authority is right here".
	const PollWindow         window   = retainedLaneWindow(lanes, 120u);
	const FrameMeterGeometry geometry = geometryFor(frameMeterCellCount(window));
	CHECK(nearlyEqual(authorityMarkerX(geometry, marker),
		geometry.originX + frameMeterWidth(geometry)));
	CHECK_FALSE(nearlyEqual(authorityMarkerX(geometry, marker),
		frameMeterCellX(geometry, geometry.cellCount - 1u)));
}

// ---------------------------------------------------------------------------
// THE TARGET OUTSIDE THE RETAINED WINDOW
// ---------------------------------------------------------------------------

TEST_CASE("Authority.AnAuthorityTickOlderThanTheWindowFlagsTheLeftEdgeAndSaysSo",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	pollRun(0u, 60u, 0u, lanes);

	// A thirty-tick window on sixty recorded ticks: an offset of forty reaches past its
	// oldest cell, which a long stall or a small retention setting both produce.
	const FrameMeterAuthorityMarker marker = markerFor(lanes, 30u, 59u, 40u);

	CHECK(marker.kind == AuthorityMarkerKind::OnCell);
	CHECK(marker.anchor == AuthorityMarkerAnchor::LeftEdge);
	CHECK(marker.authorityTick == 19u);
	CHECK(marker.offsetTicks == 40u);

	// The same window with the target inside it stays a column, so this is the window and
	// not the lanes making the difference.
	const FrameMeterAuthorityMarker inside = markerFor(lanes, 30u, 59u, 4u);
	CHECK(inside.anchor == AuthorityMarkerAnchor::Column);
	CHECK(inside.barOffset == 25u);
}

TEST_CASE("Authority.AnOffBarMarkerCannotDrawTheSameRuleAsOneAtTheOldestColumn",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	pollRun(0u, 60u, 0u, lanes);

	const FrameMeterAuthorityMarker offBar  = markerFor(lanes, 30u, 59u, 40u);
	const FrameMeterAuthorityMarker atZero  = markerFor(lanes, 30u, 59u, 29u);
	REQUIRE(offBar.anchor == AuthorityMarkerAnchor::LeftEdge);
	REQUIRE(atZero.anchor == AuthorityMarkerAnchor::Column);
	REQUIRE(atZero.barOffset == 0u);

	// ⛔ They share an x by construction, so the LOOK is the only thing keeping them apart.
	const FrameMeterGeometry geometry = geometryFor(30u);
	REQUIRE(nearlyEqual(authorityMarkerX(geometry, offBar),
		authorityMarkerX(geometry, atZero)));

	const FrameMeterMarkerStyle off = authorityMarkerStyleOf(offBar.anchor);
	const FrameMeterMarkerStyle on  = authorityMarkerStyleOf(atZero.anchor);
	CHECK(off.alpha != on.alpha);
	CHECK(off.thickness != on.thickness);
}

TEST_CASE("Authority.ATickTheElisionLedgerNoLongerReachesFlagsTheLeftEdgeToo",
          "[CharacterViz][InputHistoryViz]")
{
	// One more span than the ledger holds. The gate is driven directly here: what is under
	// test is placement against a ledger that has dropped an entry, not the poll.
	InputHistoryTickLanes lanes;
	uint32_t              tick = 0u;

	auto step = [&lanes, &tick](bool inactive)
	{
		if (lanes.editGate().admit(tick, inactive, true) == LaneAdmission::Recorded)
			lanes.noteAxisTick(*lanes.gate().laneTickOf(tick));
		++tick;
	};

	// One idle stretch per cycle, each one tick longer than the hysteresis needs, and one
	// more cycle than the ledger can hold -- so the oldest span is filed and then dropped.
	for (std::size_t cycle = 0u; cycle <= kLaneElisionLedgerCapacity + 1u; ++cycle)
	{
		step(false);
		for (uint32_t idle = 0u; idle < kLanePauseEngageTicks + 16u; ++idle)
			step(true);
	}
	step(false);

	// The ledger is full and the oldest span has been dropped, so its ticks can no longer
	// be placed on either axis.
	REQUIRE(lanes.gate().axisEventCount() == kLaneElisionLedgerCapacity);
	const uint32_t droppedSpanTick = lanes.gate().axisEventAt(0u).simTick - 4u;
	REQUIRE_FALSE(lanes.gate().laneTickOf(droppedSpanTick).has_value());

	const uint32_t predictionTick = tick - 1u;
	const FrameMeterAuthorityMarker marker =
		markerFor(lanes, 120u, predictionTick, predictionTick - droppedSpanTick);

	CHECK(marker.kind == AuthorityMarkerKind::TooOldToPlace);
	CHECK(marker.anchor == AuthorityMarkerAnchor::LeftEdge);
	CHECK(marker.offsetTicks == predictionTick - droppedSpanTick);
}

// ---------------------------------------------------------------------------
// THE ROLE WITH NO PREDICTION AT ALL
// ---------------------------------------------------------------------------

TEST_CASE("Authority.ARoleThatDoesNotPredictShowsNoOffsetAndNoRule",
          "[CharacterViz][InputHistoryViz]")
{
	// A dedicated server, a listen-server host and a standalone session all run the meter
	// and none of them has an estimator. An offset invented for them would be a fiction.
	InputHistoryTickLanes lanes;
	pollRun(0u, 60u, 0u, lanes);

	lanes.noteAuthorityReading(std::nullopt, 59u);

	const PollWindow window = retainedLaneWindow(lanes, 120u);
	const FrameMeterAuthorityMarker marker = frameMeterAuthorityMarkerOf(lanes, window);

	CHECK(marker.kind == AuthorityMarkerKind::NoEstimate);
	CHECK(marker.anchor == AuthorityMarkerAnchor::None);
	CHECK(marker.offsetTicks == 0u);
	CHECK(marker.authorityTick == 0u);
}

TEST_CASE("Authority.TheOffsetShownIsTheOneItWasHandedInEveryPlacement",
          "[CharacterViz][InputHistoryViz]")
{
	// The number under the rule is the estimator's, unmodified, whichever of the placements
	// the target turned out to need -- otherwise the value and the picture could disagree.
	InputHistoryTickLanes lanes;
	const uint32_t        afterIdle = pollRun(0u, 10u, 35u, lanes);
	pollRun(afterIdle, 15u, 0u, lanes);

	struct Case
	{
		uint32_t              retainedTicks;
		uint32_t              offsetTicks;
		AuthorityMarkerKind   kind;
		AuthorityMarkerAnchor anchor;
	};

	// One offset per placement the compacted axis can produce, so no arm can quietly lose
	// the number on its way to the label.
	const Case cases[] = {
		{ 120u,   4u, AuthorityMarkerKind::OnCell,       AuthorityMarkerAnchor::Column },
		{ 120u,  25u, AuthorityMarkerKind::OnElidedSpan, AuthorityMarkerAnchor::Column },
		{  10u, 200u, AuthorityMarkerKind::OnCell,       AuthorityMarkerAnchor::LeftEdge },
	};

	for (const Case& one : cases)
	{
		const FrameMeterAuthorityMarker marker =
			markerFor(lanes, one.retainedTicks, 59u, one.offsetTicks);

		CHECK(marker.kind == one.kind);
		CHECK(marker.anchor == one.anchor);
		CHECK(marker.offsetTicks == one.offsetTicks);
		CHECK(marker.authorityTick
			== authorityTickOf(PredictionOffsetReading{ 59u, one.offsetTicks }));
	}
}

// ---------------------------------------------------------------------------
// ONE SNAPSHOT, NOT TWO
//
// The rule used to be placed from a clock reading taken at DRAW time, against an axis
// built from a reading taken at POLL time. The prediction tick is written on the physics
// thread, so a step landing between the two reads put the rule one column away from the
// number under it -- a marker that flickered between two cells while its label held.
// ⛔ THE READING NOW RIDES THE POLL, so a later tick has nowhere to enter.
// ---------------------------------------------------------------------------

TEST_CASE("Authority.TheMarkerComesFromThePollsOwnReadingAndNoLaterTickCanMoveIt",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;

	// Sixty recorded ticks, each polled with the estimator's offset, so the last poll's
	// own tick is what the axis and the reading both stand on.
	pollRun(0u, 60u, 0u, lanes, 4u);

	REQUIRE(lanes.newestAxisTick() == 59u);
	REQUIRE(lanes.authorityReading().has_value());
	CHECK(lanes.authorityReading()->predictionTick == 59u);
	CHECK(lanes.authorityReading()->offsetTicks == 4u);

	const PollWindow window    = retainedLaneWindow(lanes, 120u);
	const uint32_t   cellCount = frameMeterCellCount(window);

	const FrameMeterAuthorityMarker drawn = frameMeterAuthorityMarkerOf(lanes, window);
	REQUIRE(drawn.anchor == AuthorityMarkerAnchor::Column);
	CHECK(drawn.authorityTick == 55u);

	// Drawing again -- a second frame, or the same frame after a physics step -- reaches
	// the same reading, because the draw takes none of its own.
	CHECK(frameMeterAuthorityMarkerOf(lanes, window).barOffset == drawn.barOffset);

	// The column a draw-time re-read WOULD have chosen once one more step had landed: it
	// exists, it is the neighbour, and it is not where the rule is. Without this contrast
	// the case could not tell a one-snapshot placement from a two-read one.
	uint32_t reReadColumn = 0u;
	REQUIRE(frameMeterColumnOfLaneTick(56u, window, cellCount, reReadColumn));
	CHECK(reReadColumn == drawn.barOffset + 1u);

	// ...while the printed offset does not move at all, which is why the disagreement
	// showed up as a jittering column under a constant number.
	CHECK(drawn.offsetTicks == 4u);
}

TEST_CASE("Authority.AnElidedPollStillMovesTheReadingRatherThanFreezingTheRule",
          "[CharacterViz][InputHistoryViz]")
{
	// While the gate is paused the poll ends early and writes no cell, but authority keeps
	// advancing. The reading is therefore taken BEFORE that early exit: one frozen at the
	// last recorded tick would leave the rule on a column the server had long since left.
	InputHistoryTickLanes lanes;
	const uint32_t        afterIdle = pollRun(0u, 20u, 30u, lanes, 4u);
	const uint32_t        lastPolled = afterIdle - 1u;

	REQUIRE(lanes.gate().paused());
	REQUIRE(lanes.authorityReading().has_value());
	CHECK(lanes.authorityReading()->predictionTick == lastPolled);
	CHECK(lanes.newestAxisTick() < lastPolled);

	const PollWindow window = retainedLaneWindow(lanes, 120u);
	const FrameMeterAuthorityMarker marker = frameMeterAuthorityMarkerOf(lanes, window);

	CHECK(marker.kind == AuthorityMarkerKind::InsideOpenSpan);
	CHECK(marker.anchor == AuthorityMarkerAnchor::RightEdge);
	CHECK(marker.authorityTick == lastPolled - 4u);
}

// ---------------------------------------------------------------------------
// THE HOIST -- frameMeterAuthorityMarkerOf IS NOW A THIN CALLER OF placeFrameMeterSimTick.
//
// Every kind but NoEstimate is a placement the shared helper can also be asked for
// directly; the two must then agree field for field, or the hoist changed something.
// NoEstimate has no equivalent call: it is the early-out for a role with no reading at
// all, so there is no sim tick to hand the helper in the first place.
// ---------------------------------------------------------------------------

TEST_CASE("Authority.TheMarkerIsAThinCallerOfThePlacementHelperAcrossEveryKind",
          "[CharacterViz][InputHistoryViz]")
{
	// NoEstimate -- no reading, so no placement call applies.
	{
		InputHistoryTickLanes lanes;
		pollRun(0u, 60u, 0u, lanes);
		lanes.noteAuthorityReading(std::nullopt, 59u);

		const PollWindow                window = retainedLaneWindow(lanes, 120u);
		const FrameMeterAuthorityMarker marker = frameMeterAuthorityMarkerOf(lanes, window);

		CHECK(marker.kind == AuthorityMarkerKind::NoEstimate);
		CHECK(marker.anchor == AuthorityMarkerAnchor::None);
	}

	// OnCell.
	{
		InputHistoryTickLanes lanes;
		pollRun(0u, 60u, 0u, lanes);
		const FrameMeterAuthorityMarker marker = markerFor(lanes, 120u, 59u, 4u);

		const PollWindow                 window    = retainedLaneWindow(lanes, 120u);
		const FrameMeterSimTickPlacement placement =
			placeFrameMeterSimTick(lanes, window, marker.authorityTick);

		REQUIRE(marker.kind == AuthorityMarkerKind::OnCell);
		CHECK(marker.kind == placement.kind);
		CHECK(marker.anchor == placement.anchor);
		CHECK(marker.barOffset == placement.barOffset);
	}

	// OnElidedSpan.
	{
		InputHistoryTickLanes lanes;
		const uint32_t        afterIdle = pollRun(0u, 10u, 35u, lanes);
		pollRun(afterIdle, 15u, 0u, lanes);

		const LaneAxisEvent& span           = lanes.gate().axisEventAt(0u);
		const uint32_t       insideTheSpan  = span.simTick + span.skippedTicks / 2u;
		const uint32_t     predictionTick = 59u;
		const FrameMeterAuthorityMarker marker =
			markerFor(lanes, 120u, predictionTick, predictionTick - insideTheSpan);

		const PollWindow                 window    = retainedLaneWindow(lanes, 120u);
		const FrameMeterSimTickPlacement placement =
			placeFrameMeterSimTick(lanes, window, marker.authorityTick);

		REQUIRE(marker.kind == AuthorityMarkerKind::OnElidedSpan);
		CHECK(marker.kind == placement.kind);
		CHECK(marker.anchor == placement.anchor);
		CHECK(marker.barOffset == placement.barOffset);
	}

	// InsideOpenSpan.
	{
		InputHistoryTickLanes lanes;
		const uint32_t        next           = pollRun(0u, 10u, 31u, lanes);
		const uint32_t        predictionTick = next - 1u;
		const FrameMeterAuthorityMarker marker = markerFor(lanes, 120u, predictionTick, 4u);

		const PollWindow                 window    = retainedLaneWindow(lanes, 120u);
		const FrameMeterSimTickPlacement placement =
			placeFrameMeterSimTick(lanes, window, marker.authorityTick);

		REQUIRE(marker.kind == AuthorityMarkerKind::InsideOpenSpan);
		CHECK(marker.kind == placement.kind);
		CHECK(marker.anchor == placement.anchor);
		CHECK(marker.barOffset == placement.barOffset);
	}

	// TooOldToPlace.
	{
		InputHistoryTickLanes lanes;
		uint32_t              tick = 0u;

		auto step = [&lanes, &tick](bool inactive)
		{
			if (lanes.editGate().admit(tick, inactive, true) == LaneAdmission::Recorded)
				lanes.noteAxisTick(*lanes.gate().laneTickOf(tick));
			++tick;
		};

		for (std::size_t cycle = 0u; cycle <= kLaneElisionLedgerCapacity + 1u; ++cycle)
		{
			step(false);
			for (uint32_t idle = 0u; idle < kLanePauseEngageTicks + 16u; ++idle)
				step(true);
		}
		step(false);

		const uint32_t droppedSpanTick = lanes.gate().axisEventAt(0u).simTick - 4u;
		const uint32_t predictionTick  = tick - 1u;
		const FrameMeterAuthorityMarker marker =
			markerFor(lanes, 120u, predictionTick, predictionTick - droppedSpanTick);

		const PollWindow                 window    = retainedLaneWindow(lanes, 120u);
		const FrameMeterSimTickPlacement placement =
			placeFrameMeterSimTick(lanes, window, marker.authorityTick);

		REQUIRE(marker.kind == AuthorityMarkerKind::TooOldToPlace);
		CHECK(marker.kind == placement.kind);
		CHECK(marker.anchor == placement.anchor);
		CHECK(marker.barOffset == placement.barOffset);
	}
}

// ---------------------------------------------------------------------------
// THE TWO VERTICAL MARKERS
// ---------------------------------------------------------------------------

TEST_CASE("Authority.TheAuthorityRuleAndTheFrozenHorizonCannotRenderIdentically",
          "[CharacterViz][InputHistoryViz]")
{
	// One says the correction cache can no longer answer for anything left of here; the
	// other says the server is here. Confusing them inverts a desync diagnosis, so all four
	// fields differ and no single edit can collapse the pair.
	const FrameMeterMarkerStyle authority[] = {
		kFrameMeterAuthorityStyle, kFrameMeterAuthorityOffBarStyle };

	for (const FrameMeterMarkerStyle& style : authority)
	{
		CHECK(laneColorGap(style.color, kFrameMeterHorizonStyle.color) >= kLanePaletteMinCrossGap);
		CHECK(style.alpha != kFrameMeterHorizonStyle.alpha);
		CHECK(style.thickness != kFrameMeterHorizonStyle.thickness);
		CHECK(style.shape != kFrameMeterHorizonStyle.shape);
	}

	// The horizon carries no number of its own, which is the difference a reader sees
	// before any of the other three register.
	CHECK(kFrameMeterHorizonStyle.shape == FrameMeterMarkerShape::PlainRule);
	CHECK(kFrameMeterAuthorityStyle.shape == FrameMeterMarkerShape::LabelledRule);
	CHECK(kFrameMeterAuthorityOffBarStyle.shape == FrameMeterMarkerShape::LabelledRule);
}

TEST_CASE("Authority.TheAuthorityRuleClearsThePaletteFloorAgainstEveryCellItCovers",
          "[CharacterViz][InputHistoryViz]")
{
	// The rule is drawn over cells, so a colour that merely differs from the horizon is not
	// enough: it has to stay visible against every colour either bar can put beneath it.
	uint32_t checked = 0u;
	for (const brawlerInputHistoryVisualization::LaneCellColor& cell : everyCellColor())
	{
		CHECK(laneColorGap(kFrameMeterAuthorityStyle.color, cell) >= kLanePaletteMinPairGap);
		++checked;
	}

	// Nine provenance colours, four machine states, six delay verdicts, the unnamed
	// sentinel and the two axis-event markers. A palette that grew without this sweep
	// growing with it would pass vacuously.
	CHECK(checked == static_cast<uint32_t>(kRowProvenanceSummaryCount)
		+ static_cast<uint32_t>(kMachineStateCellCount) - 1u
		+ static_cast<uint32_t>(kInputDelayVerdictCount) - 1u + 3u);
}

TEST_CASE("Authority.TheOffsetValueAndAnElisionCountNeverShareALine",
          "[CharacterViz][InputHistoryViz]")
{
	// Both are bare numbers printed beside the bars, and they mean entirely different
	// things, so they are put on opposite sides of the meter rather than near each other.
	const FrameMeterLayout   layout;
	const FrameMeterGeometry geometry = geometryFor(120u);
	const float              labelHeight = 12.f;

	const float elisionTop   = frameMeterElisionLabelTopY(geometry, layout, labelHeight);
	const float authorityTop = frameMeterAuthorityLabelTopY(geometry, layout);

	CHECK(elisionTop + labelHeight <= geometry.originY);
	CHECK(authorityTop >= geometry.originY + frameMeterHeight(geometry));
	CHECK(authorityTop > elisionTop);
}

} // namespace inputhistoryauthoritytests

#endif // WITH_LOW_LEVEL_TESTS
