// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins BrawlerInputHistoryVisualizationBars.h -- the two stacked frame-meter bars, their
// geometry, their two palettes, and the run detection that puts a number on a run's last cell.
//
// WHAT THIS SUITE IS REALLY GUARDING is that a reader can trust a vertical slice. The whole
// diagnostic value of two stacked bars is that a column is ONE capture tick in both, so the
// state machine's behaviour can be read against the ticks that were resimulated. That holds
// only because one geometry and one window feed both bars; two derivations that agree today
// would drift the moment either is touched, and nothing on screen would say so.
//
// The second thing it guards is that a HOLE STAYS A HOLE. The machine lane has gaps by design
// and a coloured cell in a netcode diagnostic reads as evidence, so a gap that acquired a
// colour -- or that quietly joined the run either side of it -- would be a fabricated claim.
//
// The lanes themselves are NOT re-tested here: BrawlerInputHistoryLaneTest.cpp owns the
// storage, the two re-poll rules and the clamp. What is tested is what a renderer READS.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryframemetertests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::AuthorityMarkerAnchor;
using brawlerInputHistoryVisualization::AuthorityMarkerKind;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::FrameMeterAxisEvent;
using brawlerInputHistoryVisualization::FrameMeterAxisEventList;
using brawlerInputHistoryVisualization::FrameMeterBarCells;
using brawlerInputHistoryVisualization::FrameMeterGeometry;
using brawlerInputHistoryVisualization::FrameMeterHorizon;
using brawlerInputHistoryVisualization::FrameMeterHorizonKind;
using brawlerInputHistoryVisualization::FrameMeterLayout;
using brawlerInputHistoryVisualization::FrameMeterSimTickPlacement;
using brawlerInputHistoryVisualization::InputDelayCell;
using brawlerInputHistoryVisualization::InputDelayDecomposition;
using brawlerInputHistoryVisualization::InputDelayReadout;
using brawlerInputHistoryVisualization::InputDelayVerdict;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneAdmission;
using brawlerInputHistoryVisualization::LaneAxisEvent;
using brawlerInputHistoryVisualization::LaneAxisEventKind;
using brawlerInputHistoryVisualization::LaneCellColor;
using brawlerInputHistoryVisualization::LaneCellFill;
using brawlerInputHistoryVisualization::LaneCellStyle;
using brawlerInputHistoryVisualization::LaneRun;
using brawlerInputHistoryVisualization::LaneRunList;
using brawlerInputHistoryVisualization::MachineStateCell;
using brawlerInputHistoryVisualization::PollWindow;
using brawlerInputHistoryVisualization::ProvenanceResidencyReadout;
using brawlerInputHistoryVisualization::ResidencyReading;
using brawlerInputHistoryVisualization::RowProvenanceSummary;
using brawlerInputHistoryVisualization::WindowResidency;

using brawlerInputHistoryVisualization::authorityMarkerX;
using brawlerInputHistoryVisualization::buildInputDelayReadout;
using brawlerInputHistoryVisualization::buildProvenanceResidencyReadout;
using brawlerInputHistoryVisualization::collectFrameMeterAxisEvents;
using brawlerInputHistoryVisualization::collectLaneRuns;
using brawlerInputHistoryVisualization::delayVerdictStyleOf;
using brawlerInputHistoryVisualization::delayVerdictStyleOfOrdinal;
using brawlerInputHistoryVisualization::frameMeterAuthorityLabelTopY;
using brawlerInputHistoryVisualization::frameMeterAuthorityMarkerOf;
using brawlerInputHistoryVisualization::frameMeterBarDrawsRunLabels;
using brawlerInputHistoryVisualization::frameMeterBarTopY;
using brawlerInputHistoryVisualization::frameMeterCellCount;
using brawlerInputHistoryVisualization::frameMeterCellX;
using brawlerInputHistoryVisualization::frameMeterDelayReadoutTopY;
using brawlerInputHistoryVisualization::frameMeterGeometryFor;
using brawlerInputHistoryVisualization::frameMeterHeight;
using brawlerInputHistoryVisualization::frameMeterHorizonOf;
using brawlerInputHistoryVisualization::frameMeterReadoutLineTopY;
using brawlerInputHistoryVisualization::frameMeterWidth;
using brawlerInputHistoryVisualization::laneColorGap;
using brawlerInputHistoryVisualization::laneLabelPrefersDarkInk;
using brawlerInputHistoryVisualization::machineCellStyleOf;
using brawlerInputHistoryVisualization::machineCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::placeFrameMeterSimTick;
using brawlerInputHistoryVisualization::provenanceCellStyleOf;
using brawlerInputHistoryVisualization::provenanceCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::readDelayBar;
using brawlerInputHistoryVisualization::readMachineStateBar;
using brawlerInputHistoryVisualization::readProvenanceBar;
using brawlerInputHistoryVisualization::retainedLaneWindow;
using brawlerInputHistoryVisualization::runLabelCenterX;
using brawlerInputHistoryVisualization::runLabelFits;

using brawlerInputHistoryVisualization::FrameMeterBarKind;
using brawlerInputHistoryVisualization::FrameMeterBarSelection;

using brawlerInputHistoryVisualization::frameMeterBarSlotOf;
using brawlerInputHistoryVisualization::frameMeterEnabledBarCount;

using brawlerInputHistoryVisualization::kAppliedPollWindowTicks;
using brawlerInputHistoryVisualization::kFrameMeterBarCount;
using brawlerInputHistoryVisualization::kFrameMeterBarKindCount;
using brawlerInputHistoryVisualization::kFrameMeterHorizonKindCount;
using brawlerInputHistoryVisualization::kInputDelayVerdictCount;
using brawlerInputHistoryVisualization::kLaneElisionColor;
using brawlerInputHistoryVisualization::kLaneElisionLedgerCapacity;
using brawlerInputHistoryVisualization::kLaneResyncColor;
using brawlerInputHistoryVisualization::kLanePauseEngageTicks;
using brawlerInputHistoryVisualization::kLanePaletteMinCrossGap;
using brawlerInputHistoryVisualization::kLanePaletteMinPairGap;
using brawlerInputHistoryVisualization::kMachineStateCellCount;
using brawlerInputHistoryVisualization::kRowProvenanceSummaryCount;
using brawlerInputHistoryVisualization::kTickLaneCapacity;
using brawlerInputHistoryVisualization::kUnnamedLaneColor;

// ---------------------------------------------------------------------------
// The horizon fixtures below drive the GATE the same way BrawlerInputHistoryAuthorityTest
// does (a reader that never names a resident tick, so the elision structure is real), and
// then inject the RESIDENCY the fixture wants directly -- the horizon reads it as one more
// filed reading, exactly like the authority marker reads its own. This tests the placement
// against a real gate without re-deriving a second correction-cache double.
// ---------------------------------------------------------------------------

class SilentReader
{
public:
	AppliedCaptureRef appliedCaptureRef(uint32_t) const { return AppliedCaptureRef{}; }
	std::optional<SlotStateProvenance> slotProvenance(uint32_t) const { return std::nullopt; }
	bool hasCorrectionCache() const { return true; }
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

void pollTick(uint32_t simTick, bool active, InputHistoryTickLanes& lanes)
{
	const SilentReader      reader;
	AppliedCaptureInversion inversion;

	brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, active ? movingInput() : neutralInput(), true, std::nullopt,
		std::nullopt, std::nullopt, inversion, lanes);
}

// `activeTicks` moving ticks, then `idleTicks` neutral ones, starting at `firstTick`.
uint32_t pollRun(uint32_t firstTick, uint32_t activeTicks, uint32_t idleTicks,
                 InputHistoryTickLanes& lanes)
{
	uint32_t tick = firstTick;
	for (uint32_t step = 0u; step < activeTicks; ++step, ++tick)
		pollTick(tick, true, lanes);
	for (uint32_t step = 0u; step < idleTicks; ++step, ++tick)
		pollTick(tick, false, lanes);

	return tick;
}

// ---------------------------------------------------------------------------
// Helpers. Everything below builds cells by hand rather than through a poll: what is
// under test is how a RENDERER reads a window, not how the window came to be filled.
// ---------------------------------------------------------------------------

FrameMeterBarCells barOfRuns(const uint8_t* values, const uint32_t* lengths, uint32_t runCount)
{
	FrameMeterBarCells bar;
	for (uint32_t run = 0u; run < runCount; ++run)
	{
		for (uint32_t step = 0u; step < lengths[run]; ++step)
		{
			bar.cells[bar.count].filled = true;
			bar.cells[bar.count].value  = values[run];
			++bar.count;
		}
	}
	return bar;
}

FrameMeterGeometry geometryFor(float viewportWidth, float viewportHeight, uint32_t cellCount)
{
	const FrameMeterLayout layout;
	return frameMeterGeometryFor(layout, viewportWidth, viewportHeight, cellCount);
}

bool nearlyEqual(float left, float right)
{
	const float delta = (left > right) ? (left - right) : (right - left);
	return delta < 0.01f;
}

// ---------------------------------------------------------------------------
// THE TWO PALETTES
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.EveryProvenanceSummaryKeepsItsOwnColourAcrossTheWholeLadder",
          "[CharacterViz][InputHistoryViz]")
{
	// These nine are the input panel's own colours, re-homed rather than re-invented, so a
	// tenth enumerator added without a table entry must fail here rather than render grey.
	uint32_t named = 0u;
	for (uint8_t ordinal = 0u; ordinal < kRowProvenanceSummaryCount; ++ordinal)
	{
		if (provenanceCellStyleOfOrdinal(ordinal).fill == LaneCellFill::State)
			++named;
	}

	uint32_t distinctPairs = 0u;
	for (uint8_t left = 0u; left < kRowProvenanceSummaryCount; ++left)
	{
		for (uint8_t right = static_cast<uint8_t>(left + 1u); right < kRowProvenanceSummaryCount;
		     ++right)
		{
			if (laneColorGap(provenanceCellStyleOfOrdinal(left).color,
			                 provenanceCellStyleOfOrdinal(right).color)
				> kLanePaletteMinPairGap)
			{
				++distinctPairs;
			}
		}
	}

	CHECK(named == 9u);
	CHECK(distinctPairs == 36u);

	// One PAST the count pins the count itself, not merely the table.
	CHECK(provenanceCellStyleOfOrdinal(kRowProvenanceSummaryCount).fill == LaneCellFill::Unnamed);

	// The pair the whole display exists to separate: a correction against a resimulation.
	CHECK(laneColorGap(provenanceCellStyleOf(RowProvenanceSummary::Corrected).color,
	                   provenanceCellStyleOf(RowProvenanceSummary::Resimulated).color)
		> 1.5f);
}

TEST_CASE("FrameMeter.EveryMachineStateHasItsOwnColourAndOnlyTheUnsampledCellIsAHole",
          "[CharacterViz][InputHistoryViz]")
{
	uint32_t holes = 0u;
	for (uint8_t ordinal = 0u; ordinal < kMachineStateCellCount; ++ordinal)
	{
		if (machineCellStyleOfOrdinal(ordinal).fill == LaneCellFill::Hole)
			++holes;
	}

	// The four real states, pairwise. A collapse of two would leave the bar readable and wrong.
	uint32_t distinctPairs = 0u;
	for (uint8_t left = 1u; left < kMachineStateCellCount; ++left)
	{
		for (uint8_t right = static_cast<uint8_t>(left + 1u); right < kMachineStateCellCount;
		     ++right)
		{
			if (laneColorGap(machineCellStyleOfOrdinal(left).color,
			                 machineCellStyleOfOrdinal(right).color)
				> kLanePaletteMinPairGap)
			{
				++distinctPairs;
			}
		}
	}

	CHECK(holes == 1u);
	CHECK(machineCellStyleOf(MachineStateCell::NotSampled).fill == LaneCellFill::Hole);
	CHECK(distinctPairs == 6u);
	CHECK(machineCellStyleOfOrdinal(kMachineStateCellCount).fill == LaneCellFill::Unnamed);

	// A fifth attack state must move this, so the table cannot silently stop covering its enum.
	CHECK(static_cast<unsigned>(kMachineStateCellCount) == kDAttackStateCount + 1u);
}

TEST_CASE("FrameMeter.TheTwoPalettesDoNotCollideAndTheSentinelSitsOutsideBoth",
          "[CharacterViz][InputHistoryViz]")
{
	float    closestCross = 3.f;
	uint32_t separated    = 0u;
	for (uint8_t machine = 1u; machine < kMachineStateCellCount; ++machine)
	{
		for (uint8_t provenance = 0u; provenance < kRowProvenanceSummaryCount; ++provenance)
		{
			const float gap = laneColorGap(machineCellStyleOfOrdinal(machine).color,
			                               provenanceCellStyleOfOrdinal(provenance).color);
			if (gap > kLanePaletteMinCrossGap)
				++separated;

			if (gap < closestCross)
				closestCross = gap;
		}
	}

	// The provenance palette's own closest pair. The cross-palette floor must beat it, or
	// the machine bar would read as one more spelling of the bar above it.
	const float closestInsideProvenance =
		laneColorGap(provenanceCellStyleOf(RowProvenanceSummary::Unknown).color,
		             provenanceCellStyleOf(RowProvenanceSummary::NoStateWritten).color);

	uint32_t sentinelClear = 0u;
	for (uint8_t ordinal = 0u; ordinal < kRowProvenanceSummaryCount; ++ordinal)
	{
		if (laneColorGap(kUnnamedLaneColor, provenanceCellStyleOfOrdinal(ordinal).color)
			> kLanePaletteMinCrossGap)
		{
			++sentinelClear;
		}
	}
	for (uint8_t ordinal = 1u; ordinal < kMachineStateCellCount; ++ordinal)
	{
		if (laneColorGap(kUnnamedLaneColor, machineCellStyleOfOrdinal(ordinal).color)
			> kLanePaletteMinCrossGap)
		{
			++sentinelClear;
		}
	}

	CHECK(separated == 36u);
	CHECK(closestCross > closestInsideProvenance);
	CHECK(sentinelClear == 13u);
}

TEST_CASE("FrameMeter.EveryDelayVerdictHasItsOwnColourAndOnlyNoVerdictIsAHole",
          "[CharacterViz][InputHistoryViz]")
{
	uint32_t holes = 0u;
	for (uint8_t ordinal = 0u; ordinal < kInputDelayVerdictCount; ++ordinal)
	{
		if (delayVerdictStyleOfOrdinal(ordinal).fill == LaneCellFill::Hole)
			++holes;
	}

	// Six verdicts, pairwise. A collapse of two would leave the bar readable and wrong.
	uint32_t distinctPairs = 0u;
	for (uint8_t left = 1u; left < kInputDelayVerdictCount; ++left)
	{
		for (uint8_t right = static_cast<uint8_t>(left + 1u); right < kInputDelayVerdictCount;
		     ++right)
		{
			if (laneColorGap(delayVerdictStyleOfOrdinal(left).color,
			                 delayVerdictStyleOfOrdinal(right).color)
				> kLanePaletteMinPairGap)
			{
				++distinctPairs;
			}
		}
	}

	CHECK(holes == 1u);
	CHECK(delayVerdictStyleOf(InputDelayVerdict::NoVerdict).fill == LaneCellFill::Hole);
	CHECK(distinctPairs == 15u);
	CHECK(delayVerdictStyleOfOrdinal(kInputDelayVerdictCount).fill == LaneCellFill::Unnamed);
}

TEST_CASE("FrameMeter.TheDelayVerdictPaletteDoesNotCollideWithEitherLaneAboveIt",
          "[CharacterViz][InputHistoryViz]")
{
	// Six verdicts against nine provenance colours, four machine states and the two
	// out-of-palette sentinels: fifteen comparisons each, all clearing the cross floor.
	uint32_t separated = 0u;
	for (uint8_t verdict = 1u; verdict < kInputDelayVerdictCount; ++verdict)
	{
		const LaneCellColor color = delayVerdictStyleOfOrdinal(verdict).color;

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

	CHECK(separated == 6u * 15u);
}

TEST_CASE("FrameMeter.ServerEarlierHasTheLargestIsolationOfTheSixVerdictColours",
          "[CharacterViz][InputHistoryViz]")
{
	// ⭐ A RELATION, NOT A LITERAL: re-tuning any colour must not silently invert which one
	// reads as the loudest divergence on the bar.
	auto isolationOf = [](uint8_t ordinal) -> float
	{
		float minGap = 4.f;
		for (uint8_t other = 1u; other < kInputDelayVerdictCount; ++other)
		{
			if (other == ordinal)
				continue;

			const float gap = laneColorGap(delayVerdictStyleOfOrdinal(ordinal).color,
			                               delayVerdictStyleOfOrdinal(other).color);
			if (gap < minGap)
				minGap = gap;
		}
		return minGap;
	};

	const uint8_t serverEarlier          = static_cast<uint8_t>(InputDelayVerdict::ServerEarlier);
	const float   serverEarlierIsolation = isolationOf(serverEarlier);

	uint32_t notExceeded = 0u;
	for (uint8_t ordinal = 1u; ordinal < kInputDelayVerdictCount; ++ordinal)
	{
		if (ordinal == serverEarlier)
			continue;

		if (isolationOf(ordinal) < serverEarlierIsolation)
			++notExceeded;
	}

	CHECK(notExceeded == kInputDelayVerdictCount - 2u);
}

// ---------------------------------------------------------------------------
// GEOMETRY -- THE ALIGNMENT AND THE PLACEMENT
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.AVerticalSliceIsOneCaptureTickInBothBars",
          "[CharacterViz][InputHistoryViz]")
{
	// Both lanes written at exactly the SAME three ticks, then read as two bars. If either
	// reader derived its own window the two fill patterns would slide apart by a tick and
	// this count would drop -- which is precisely what a reader of the screen cannot see.
	InputHistoryTickLanes lanes;
	const uint32_t        seeded[] = { 940u, 970u, 999u };
	for (uint32_t index = 0u; index < 3u; ++index)
	{
		lanes.editProvenance().record(seeded[index], RowProvenanceSummary::Corrected);
		lanes.editMachineState().recordIfAbsent(seeded[index], MachineStateCell::HitFlinch);
	}
	lanes.noteAxisTick(1000u);

	const PollWindow window = retainedLaneWindow(lanes, 120u);

	FrameMeterBarCells provenanceBar;
	FrameMeterBarCells machineBar;
	readProvenanceBar(lanes, window, provenanceBar);
	readMachineStateBar(lanes, window, machineBar);

	uint32_t alignedColumns = 0u;
	for (uint32_t offset = 0u; offset < provenanceBar.count; ++offset)
	{
		if (provenanceBar.cells[offset].filled == machineBar.cells[offset].filled)
			++alignedColumns;
	}

	const FrameMeterGeometry geometry = geometryFor(1920.f, 1080.f, frameMeterCellCount(window));
	const FrameMeterLayout   layout;

	// The meter WITHOUT the delay bar: index constants become kind lookups on a
	// selection with InputDelay off, reproducing today's two-bar shape exactly.
	FrameMeterBarSelection noDelay;
	noDelay.inputDelay = false;
	const uint32_t provenanceSlot = *frameMeterBarSlotOf(noDelay, FrameMeterBarKind::Provenance);
	const uint32_t characterStateSlot =
		*frameMeterBarSlotOf(noDelay, FrameMeterBarKind::CharacterState);

	CHECK(alignedColumns == 120u);
	CHECK(nearlyEqual(frameMeterBarTopY(geometry, characterStateSlot)
	                      - frameMeterBarTopY(geometry, provenanceSlot),
		geometry.barHeight + geometry.barGap));
	CHECK(frameMeterBarTopY(geometry, provenanceSlot)
		< frameMeterBarTopY(geometry, characterStateSlot));
	CHECK(nearlyEqual(frameMeterHeight(geometry), 2.f * layout.barHeight + layout.barGap));
}

TEST_CASE("FrameMeter.TheFourArgumentGeometryStaysTodaysShapeAndTheFifthAddsATallerBar",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout layout;

	// ⛔ THE FOUR-ARGUMENT FORM MUST RETURN TODAY'S GEOMETRY BYTE-FOR-BYTE.
	const FrameMeterGeometry two = geometryFor(1920.f, 1080.f, 120u);
	CHECK(two.barCount == kFrameMeterBarCount);
	CHECK(two.barCount == 2u);

	const FrameMeterGeometry three = frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, 3u);
	CHECK(three.barCount == 3u);
	CHECK(nearlyEqual(frameMeterHeight(three), 3.f * layout.barHeight + 2.f * layout.barGap));

	// The third row, by kind lookup rather than the deleted index constant -- with all
	// three bars on, CharacterState is slot 2 in the new declaration order.
	const FrameMeterBarSelection allThree;
	const uint32_t thirdSlot = *frameMeterBarSlotOf(allThree, FrameMeterBarKind::CharacterState);
	CHECK(nearlyEqual(frameMeterBarTopY(three, thirdSlot),
		three.originY + 2.f * (layout.barHeight + layout.barGap)));

	// Only the height, and through it originY, move -- everything else about the columns
	// is unchanged by the extra bar.
	CHECK(nearlyEqual(two.originX, three.originX));
	CHECK(nearlyEqual(two.cellStride, three.cellStride));
	CHECK(nearlyEqual(two.cellWidth, three.cellWidth));
	CHECK(two.cellCount == three.cellCount);
	CHECK(nearlyEqual(two.originY - three.originY, 17.f));

	// Still centred and inside the frame at all four viewports, at barCount 3.
	const float    widths[]  = { 1280.f, 1920.f, 2560.f, 3840.f };
	const float    heights[] = { 720.f, 1080.f, 1440.f, 2160.f };
	uint32_t       centred        = 0u;
	uint32_t       insideViewport = 0u;
	for (uint32_t index = 0u; index < 4u; ++index)
	{
		const FrameMeterGeometry g =
			frameMeterGeometryFor(layout, widths[index], heights[index], 120u, 3u);

		const float leftMargin  = g.originX;
		const float rightMargin = widths[index] - (g.originX + frameMeterWidth(g));
		if (nearlyEqual(leftMargin, rightMargin))
			++centred;

		if (leftMargin >= 0.f && rightMargin >= 0.f
			&& g.originY + frameMeterHeight(g) < heights[index])
		{
			++insideViewport;
		}
	}
	CHECK(centred == 4u);
	CHECK(insideViewport == 4u);
}

TEST_CASE("FrameMeter.OnlyTheDelayBarSkipsRunLabels",
          "[CharacterViz][InputHistoryViz]")
{
	CHECK(frameMeterBarDrawsRunLabels(FrameMeterBarKind::Provenance));
	CHECK(frameMeterBarDrawsRunLabels(FrameMeterBarKind::CharacterState));
	CHECK_FALSE(frameMeterBarDrawsRunLabels(FrameMeterBarKind::InputDelay));
}

// ---------------------------------------------------------------------------
// COMPACTION -- ALL EIGHT SELECTION SUBSETS, EACH SLOT NAMED EXPLICITLY.
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.AllThreeBarsOnGivesProvenanceZeroInputDelayOneCharacterStateTwo",
          "[CharacterViz][InputHistoryViz]")
{
	// The user's actual request, asserted directly so it cannot regress silently
	// underneath the general sweep below.
	const FrameMeterBarSelection allOn;

	CHECK(frameMeterBarSlotOf(allOn, FrameMeterBarKind::Provenance) == 0u);
	CHECK(frameMeterBarSlotOf(allOn, FrameMeterBarKind::InputDelay) == 1u);
	CHECK(frameMeterBarSlotOf(allOn, FrameMeterBarKind::CharacterState) == 2u);
	CHECK(frameMeterEnabledBarCount(allOn) == 3u);
}

TEST_CASE("FrameMeter.EverySelectionSubsetCompactsToContiguousSlotsAndTheSweepCannotShrink",
          "[CharacterViz][InputHistoryViz]")
{
	struct Expectation
	{
		bool                     provenance;
		bool                     inputDelay;
		bool                     characterState;
		uint32_t                 count;
		std::optional<uint32_t>  provenanceSlot;
		std::optional<uint32_t>  inputDelaySlot;
		std::optional<uint32_t>  characterStateSlot;
	};

	// All eight subsets of three flags, each spot named against the Backlog's own words.
	const Expectation table[] = {
		{ false, false, false, 0u, std::nullopt, std::nullopt, std::nullopt },
		{ true,  false, false, 1u, 0u,           std::nullopt, std::nullopt },
		{ false, true,  false, 1u, std::nullopt, 0u,           std::nullopt },
		{ true,  true,  false, 2u, 0u,           1u,           std::nullopt },
		{ false, false, true,  1u, std::nullopt, std::nullopt, 0u           },
		{ true,  false, true,  2u, 0u,           std::nullopt, 1u           },
		{ false, true,  true,  2u, std::nullopt, 0u,           1u           },
		{ true,  true,  true,  3u, 0u,           1u,           2u           },
	};

	const uint32_t subsetCount = static_cast<uint32_t>(sizeof(table) / sizeof(table[0]));

	uint32_t agreed = 0u;
	for (const Expectation& row : table)
	{
		FrameMeterBarSelection selection;
		selection.provenance     = row.provenance;
		selection.inputDelay     = row.inputDelay;
		selection.characterState = row.characterState;

		const bool matches =
			frameMeterEnabledBarCount(selection) == row.count
			&& frameMeterBarSlotOf(selection, FrameMeterBarKind::Provenance) == row.provenanceSlot
			&& frameMeterBarSlotOf(selection, FrameMeterBarKind::InputDelay) == row.inputDelaySlot
			&& frameMeterBarSlotOf(selection, FrameMeterBarKind::CharacterState)
			       == row.characterStateSlot;

		if (matches)
			++agreed;
	}

	// The count itself is checked against the ENUM's own size, so a fourth bar kind
	// added without extending this table's coverage fails here rather than passing quietly.
	CHECK(subsetCount == (1u << kFrameMeterBarKindCount));
	CHECK(agreed == subsetCount);
}

// ---------------------------------------------------------------------------
// THE ZERO-BAR AND ONE-BAR GUARDS -- compaction makes both reachable.
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.ZeroBarsGivesZeroHeightAndDrawsNothingRatherThanOffScreen",
          "[CharacterViz][InputHistoryViz]")
{
	// Master on, all three bars off. Unsigned barCount - 1u underflowed here before this
	// task; the guard must make it a real, in-frame, zero-height geometry instead.
	const FrameMeterGeometry zero =
		frameMeterGeometryFor(FrameMeterLayout{}, 1920.f, 1080.f, 120u, 0u);

	CHECK(zero.barCount == 0u);
	CHECK(frameMeterHeight(zero) == 0.f);
	CHECK(zero.originY >= 0.f);
	CHECK(zero.originY <= 1080.f);
}

TEST_CASE("FrameMeter.OneBarHasNoGapTermAndStaysCentredInsideTheFrame",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout layout;
	const float            widths[]  = { 1280.f, 1920.f, 2560.f, 3840.f };
	const float            heights[] = { 720.f, 1080.f, 1440.f, 2160.f };

	uint32_t centred        = 0u;
	uint32_t insideViewport = 0u;
	for (uint32_t index = 0u; index < 4u; ++index)
	{
		const FrameMeterGeometry one =
			frameMeterGeometryFor(layout, widths[index], heights[index], 120u, 1u);

		CHECK(nearlyEqual(frameMeterHeight(one), layout.barHeight));

		const float leftMargin  = one.originX;
		const float rightMargin = widths[index] - (one.originX + frameMeterWidth(one));
		if (nearlyEqual(leftMargin, rightMargin))
			++centred;

		if (leftMargin >= 0.f && rightMargin >= 0.f
			&& one.originY + frameMeterHeight(one) < heights[index])
		{
			++insideViewport;
		}
	}

	CHECK(centred == 4u);
	CHECK(insideViewport == 4u);
}

TEST_CASE("FrameMeter.TheBarSitsBottomMiddleAtEveryViewportSize",
          "[CharacterViz][InputHistoryViz]")
{
	const float    widths[]  = { 1280.f, 1920.f, 2560.f, 3840.f };
	const float    heights[] = { 720.f, 1080.f, 1440.f, 2160.f };
	const uint32_t sizes     = 4u;

	const FrameMeterLayout layout;

	uint32_t centred = 0u;
	uint32_t bottomAnchored = 0u;
	uint32_t insideViewport = 0u;
	for (uint32_t index = 0u; index < sizes; ++index)
	{
		const FrameMeterGeometry geometry = geometryFor(widths[index], heights[index], 120u);

		const float leftMargin  = geometry.originX;
		const float rightMargin = widths[index] - (geometry.originX + frameMeterWidth(geometry));
		if (nearlyEqual(leftMargin, rightMargin))
			++centred;

		const float bottomEdge = geometry.originY + frameMeterHeight(geometry);
		if (nearlyEqual(heights[index] - bottomEdge, heights[index] * layout.bottomMarginFraction))
			++bottomAnchored;

		if (leftMargin >= 0.f && rightMargin >= 0.f && bottomEdge < heights[index])
			++insideViewport;
	}

	CHECK(centred == sizes);
	CHECK(bottomAnchored == sizes);
	CHECK(insideViewport == sizes);

	// Cells keep their readable size until the bar would overrun the frame, and only then
	// shrink -- so a wide retained window on a small viewport still fits rather than clipping.
	CHECK(geometryFor(1920.f, 1080.f, 120u).cellStride == layout.preferredCellStride);
	CHECK(geometryFor(640.f, 480.f, 120u).cellStride < layout.preferredCellStride);
}

TEST_CASE("FrameMeter.OffsetZeroIsTheOldestTickAndTheNewestDrawsRightmost",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	lanes.noteAxisTick(1000u);

	const PollWindow         window   = retainedLaneWindow(lanes, 120u);
	const FrameMeterGeometry geometry = geometryFor(1920.f, 1080.f, frameMeterCellCount(window));

	uint32_t ascendingColumns = 0u;
	for (uint32_t offset = 1u; offset < geometry.cellCount; ++offset)
	{
		if (frameMeterCellX(geometry, offset) > frameMeterCellX(geometry, offset - 1u))
			++ascendingColumns;
	}

	CHECK(window.oldestTick == 881u);
	CHECK(window.newestTick == 1000u);
	CHECK(ascendingColumns == 119u);
	CHECK(frameMeterCellX(geometry, geometry.cellCount - 1u) > frameMeterCellX(geometry, 0u));
}

// ---------------------------------------------------------------------------
// RUNS AND THEIR NUMBERS
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.RunsAreDetectedAtDrawTimeAndNumberedOnTheirLastCell",
          "[CharacterViz][InputHistoryViz]")
{
	// The reference meter's own four numbers, in its own order. Nothing upstream folds a
	// run; these come out of neighbouring per-tick cells and nothing else.
	const uint8_t  values[]  = { 5u, 3u, 7u, 8u };
	const uint32_t lengths[] = { 11u, 4u, 17u, 5u };

	LaneRunList runs;
	collectLaneRuns(barOfRuns(values, lengths, 4u), runs);

	uint32_t matchedLengths = 0u;
	for (uint32_t index = 0u; index < runs.count && index < 4u; ++index)
	{
		if (runs.runs[index].length == lengths[index]
			&& runs.runs[index].value == values[index])
		{
			++matchedLengths;
		}
	}

	CHECK(runs.count == 4u);
	CHECK(matchedLengths == 4u);

	// ⛔ THE LAST CELL, NOT THE FIRST: the number belongs at the run's right-hand end.
	CHECK(runs.runs[0].lastOffset == 10u);
	CHECK(runs.runs[1].lastOffset == 14u);
	CHECK(runs.runs[2].lastOffset == 31u);
	CHECK(runs.runs[3].lastOffset == 36u);

	// And the number's own anchor lands inside that last cell. A centre taken off
	// firstOffset would still print the right number, in the wrong place, silently.
	const FrameMeterGeometry geometry = geometryFor(1920.f, 1080.f, 120u);
	const float              anchor   = runLabelCenterX(geometry, runs.runs[2]);
	CHECK((anchor > frameMeterCellX(geometry, runs.runs[2].lastOffset)
		&& anchor < frameMeterCellX(geometry, runs.runs[2].lastOffset + 1u)));
}

TEST_CASE("FrameMeter.AHoleBreaksARunAndIsNeverPartOfOne",
          "[CharacterViz][InputHistoryViz]")
{
	// Three cells, a two-tick gap, three more of the SAME value. A run detector that
	// skipped over holes would report one run of six and claim a hold nobody observed.
	FrameMeterBarCells bar;
	const bool         filled[] = { true, true, true, false, false, true, true, true };
	for (uint32_t offset = 0u; offset < 8u; ++offset)
	{
		bar.cells[offset].filled = filled[offset];
		bar.cells[offset].value  = 4u;
		++bar.count;
	}

	LaneRunList runs;
	collectLaneRuns(bar, runs);

	uint32_t coveredCells = 0u;
	uint32_t holesCovered = 0u;
	for (uint32_t index = 0u; index < runs.count; ++index)
	{
		coveredCells += runs.runs[index].length;

		for (uint32_t offset = runs.runs[index].firstOffset;
		     offset <= runs.runs[index].lastOffset; ++offset)
		{
			if (!bar.cells[offset].filled)
				++holesCovered;
		}
	}

	CHECK(runs.count == 2u);
	CHECK(runs.runs[0].length == 3u);
	CHECK(runs.runs[1].firstOffset == 5u);
	CHECK(holesCovered == 0u);
	CHECK(coveredCells == 6u);
}

TEST_CASE("FrameMeter.ALabelIsSuppressedOnlyWhenItsOwnRunIsNarrowerThanTheNumber",
          "[CharacterViz][InputHistoryViz]")
{
	// Overlapping a neighbour is expected and the reference does it. What is not drawn is a
	// number wider than the run it describes, which no reader could attribute to anything.
	const FrameMeterGeometry roomy  = geometryFor(1920.f, 1080.f, 120u);
	const FrameMeterGeometry narrow = geometryFor(480.f, 320.f, 120u);

	const LaneRun single{ 40u, 40u, 1u, 2u };
	const LaneRun pair{ 40u, 41u, 2u, 2u };
	const LaneRun eleven{ 30u, 40u, 11u, 2u };

	CHECK(runLabelFits(roomy, single));
	CHECK_FALSE(runLabelFits(narrow, single));
	CHECK(runLabelFits(narrow, pair));
	CHECK(runLabelFits(narrow, eleven));
}

TEST_CASE("FrameMeter.TheInkFlipsSoARunLengthNeverVanishesIntoItsCell",
          "[CharacterViz][InputHistoryViz]")
{
	CHECK(laneLabelPrefersDarkInk(provenanceCellStyleOf(RowProvenanceSummary::RanUnconfirmed).color));
	CHECK(laneLabelPrefersDarkInk(machineCellStyleOf(MachineStateCell::GuardFlinch).color));
	CHECK_FALSE(laneLabelPrefersDarkInk(machineCellStyleOf(MachineStateCell::Idle).color));
	CHECK_FALSE(laneLabelPrefersDarkInk(provenanceCellStyleOf(RowProvenanceSummary::Pending).color));
}

// ---------------------------------------------------------------------------
// RETENTION, THE HORIZON, AND READING THE REAL LANES
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.TheRetainedTickCVarBoundsTheDrawnWindowAndNotTheAllocation",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	lanes.noteAxisTick(1000u);

	// 180 is the value the setting exists to make tryable, and it must need no rebuild and
	// move no allocation -- retention bounds the READ and nothing else.
	CHECK(frameMeterCellCount(retainedLaneWindow(lanes, 120u)) == 120u);
	CHECK(frameMeterCellCount(retainedLaneWindow(lanes, 180u)) == 180u);
	CHECK(frameMeterCellCount(retainedLaneWindow(lanes, 240u)) == 240u);
	CHECK(frameMeterCellCount(retainedLaneWindow(lanes, 300u)) == 240u);
	CHECK(brawlerInputHistoryVisualization::ProvenanceLane::capacity() == kTickLaneCapacity);
}

// ---------------------------------------------------------------------------
// REWRITTEN -- the old case pinned `cellCount - 60` LANE cells from the left, which is the
// defect this task fixes: the writable window is 60 SIM ticks, and once the idle gate has
// elided anything, that span is FEWER lane cells, so the old rule sat LEFT of the truth.
// This fixture reproduces the exact shape the old arithmetic got wrong: a closed idle span
// sits between two runs of recorded ticks, so the residency edge and its LANE column
// genuinely differ from `cellCount - 60`.
//
// OLD expectation (the defect): `offset == cellCount - kAppliedPollWindowTicks`.
// ⭐ NEW expectation: the column is the LANE column of `oldestResident` -- 84 here, not 60.
// ---------------------------------------------------------------------------
TEST_CASE("FrameMeter.TheFrozenHorizonSitsWhereTheCorrectionCacheWindowBegins",
          "[CharacterViz][InputHistoryViz]")
{
	// 100 recorded ticks, a 40-tick idle span (closed after the 15-tick hysteresis), 30 more
	// recorded -- so by the last poll the gate has already collapsed one span.
	InputHistoryTickLanes lanes;
	const uint32_t        afterIdle = pollRun(0u, 100u, 40u, lanes);
	pollRun(afterIdle, 30u, 0u, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);

	// The residency edge, T - 59, lands BEFORE the span (in the still-identity-mapped
	// region), which is exactly where the old lane-cell arithmetic diverges from the truth.
	const uint32_t liveSimTick    = afterIdle + 30u - 1u;
	const uint32_t oldestResident = liveSimTick - 59u;
	REQUIRE(oldestResident < span.simTick);

	WindowResidency residency;
	residency.hasCache       = true;
	residency.anyResident    = true;
	residency.oldestResident = oldestResident;
	residency.newestResident = liveSimTick;
	lanes.noteResidencyReading(residency, liveSimTick);

	const PollWindow         window    = retainedLaneWindow(lanes, 120u);
	const uint32_t           cellCount = frameMeterCellCount(window);
	const FrameMeterHorizon  horizon   = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::OnCell);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::Column);
	CHECK(horizon.edgeSimTick == oldestResident);

	// The LANE column of the residency edge, asserted numerically.
	CHECK(horizon.barOffset == 84u);

	// ⛔ AND asserted different from the defect's own arithmetic -- the whole point of the fix.
	CHECK(horizon.barOffset != cellCount - static_cast<uint32_t>(kAppliedPollWindowTicks));
}

// A healthy window with no elision at all is the ONE case where the old, defective
// arithmetic and the new residency-driven placement land on the same column -- said here so
// nobody mistakes agreement for the fix having done nothing.
TEST_CASE("FrameMeter.OnAHealthyWindowWithNoElisionTheHorizonAgreesWithTheOldArithmetic",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	pollRun(0u, 120u, 0u, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 0u);

	WindowResidency residency;
	residency.hasCache       = true;
	residency.anyResident    = true;
	residency.oldestResident = 60u;   // 119 - 59
	residency.newestResident = 119u;
	lanes.noteResidencyReading(residency, 119u);

	const PollWindow        window    = retainedLaneWindow(lanes, 120u);
	const uint32_t          cellCount = frameMeterCellCount(window);
	const FrameMeterHorizon horizon   = frameMeterHorizonOf(lanes, window);

	CHECK(cellCount == 120u);
	CHECK(horizon.kind == FrameMeterHorizonKind::OnCell);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::Column);
	CHECK(horizon.barOffset == 60u);
	CHECK(horizon.barOffset == cellCount - static_cast<uint32_t>(kAppliedPollWindowTicks));
}

// ---------------------------------------------------------------------------
// EVERY FrameMeterHorizonKind, ONE FIXTURE EACH.
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.NoResidencyReadingAtAllDrawsNothing", "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	const PollWindow      window{ 0u, 0u };

	const FrameMeterHorizon horizon = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::NoReading);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::None);
}

TEST_CASE("FrameMeter.NoCorrectionCacheFlagsTheRightEdgeAndTheReadoutSaysWhy",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;

	WindowResidency residency;
	residency.hasCache = false;
	lanes.noteResidencyReading(residency, 59u);

	const PollWindow        window  = retainedLaneWindow(lanes, 120u);
	const FrameMeterHorizon horizon = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::NoCache);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::RightEdge);
}

TEST_CASE("FrameMeter.NothingResidentInTheWindowFreezesTheWholeBarAtTheRightEdge",
          "[CharacterViz][InputHistoryViz]")
{
	// hasCache true, but nothing in the window is resident -- the first tick after
	// createCacheFor, or a reader that never names a slot at all.
	InputHistoryTickLanes lanes;

	WindowResidency residency;
	residency.hasCache    = true;
	residency.anyResident = false;
	lanes.noteResidencyReading(residency, 59u);

	const PollWindow        window  = retainedLaneWindow(lanes, 120u);
	const FrameMeterHorizon horizon = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::WholeBarFrozen);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::RightEdge);
}

TEST_CASE("FrameMeter.TheEdgeInsideAClosedSpanLandsOnThatSpansOwnMarkerCell",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	const uint32_t        afterIdle = pollRun(0u, 10u, 35u, lanes);
	pollRun(afterIdle, 15u, 0u, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);

	const uint32_t insideTheSpan = span.simTick + span.skippedTicks / 2u;
	REQUIRE_FALSE(lanes.gate().laneTickOf(insideTheSpan).has_value());

	WindowResidency residency;
	residency.hasCache       = true;
	residency.anyResident    = true;
	residency.oldestResident = insideTheSpan;
	residency.newestResident = 59u;
	lanes.noteResidencyReading(residency, 59u);

	const PollWindow        window  = retainedLaneWindow(lanes, 120u);
	const FrameMeterHorizon horizon = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::OnElidedSpan);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::Column);
	CHECK(horizon.barOffset == span.laneTick);
}

TEST_CASE("FrameMeter.TheEdgeInsideTheStillOpenSpanFlagsTheRightEdge",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;
	const uint32_t        lastTick = pollRun(0u, 10u, 20u, lanes) - 1u;

	REQUIRE(lanes.gate().paused());
	REQUIRE(lanes.gate().axisEventCount() == 0u);

	// Inside the still-open span (which began at tick 10 + kLanePauseEngageTicks).
	const uint32_t edgeInsideOpenSpan = 10u + kLanePauseEngageTicks + 2u;
	REQUIRE_FALSE(lanes.gate().laneTickOf(edgeInsideOpenSpan).has_value());

	WindowResidency residency;
	residency.hasCache       = true;
	residency.anyResident    = true;
	residency.oldestResident = edgeInsideOpenSpan;
	residency.newestResident = lastTick;
	lanes.noteResidencyReading(residency, lastTick);

	const PollWindow        window  = retainedLaneWindow(lanes, 120u);
	const FrameMeterHorizon horizon = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::InsideOpenSpan);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::RightEdge);
}

TEST_CASE("FrameMeter.TheEdgeOlderThanTheLedgerStillReachesFlagsWholeBarLive",
          "[CharacterViz][InputHistoryViz]")
{
	// The gate is driven directly -- what is under test is the horizon's placement against a
	// ledger that has already dropped an entry, not the poll.
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

	REQUIRE(lanes.gate().axisEventCount() == kLaneElisionLedgerCapacity);
	const uint32_t droppedSpanTick = lanes.gate().axisEventAt(0u).simTick - 4u;
	REQUIRE_FALSE(lanes.gate().laneTickOf(droppedSpanTick).has_value());

	WindowResidency residency;
	residency.hasCache       = true;
	residency.anyResident    = true;
	residency.oldestResident = droppedSpanTick;
	residency.newestResident = tick - 1u;
	lanes.noteResidencyReading(residency, tick - 1u);

	const PollWindow        window  = retainedLaneWindow(lanes, 120u);
	const FrameMeterHorizon horizon = frameMeterHorizonOf(lanes, window);

	CHECK(horizon.kind == FrameMeterHorizonKind::WholeBarLive);
	CHECK(horizon.anchor == AuthorityMarkerAnchor::None);
}

TEST_CASE("FrameMeter.EveryHorizonKindIsReachedAndTheSweepIsCountedAgainstItsEnum",
          "[CharacterViz][InputHistoryViz]")
{
	std::set<FrameMeterHorizonKind> reached;

	// NoReading.
	{
		InputHistoryTickLanes lanes;
		const PollWindow      window{ 0u, 0u };
		reached.insert(frameMeterHorizonOf(lanes, window).kind);
	}
	// NoCache.
	{
		InputHistoryTickLanes lanes;
		WindowResidency       residency;
		residency.hasCache = false;
		lanes.noteResidencyReading(residency, 59u);
		reached.insert(frameMeterHorizonOf(lanes, retainedLaneWindow(lanes, 120u)).kind);
	}
	// WholeBarFrozen.
	{
		InputHistoryTickLanes lanes;
		WindowResidency       residency;
		residency.hasCache    = true;
		residency.anyResident = false;
		lanes.noteResidencyReading(residency, 59u);
		reached.insert(frameMeterHorizonOf(lanes, retainedLaneWindow(lanes, 120u)).kind);
	}
	// OnCell.
	{
		InputHistoryTickLanes lanes;
		pollRun(0u, 120u, 0u, lanes);
		WindowResidency residency;
		residency.hasCache       = true;
		residency.anyResident    = true;
		residency.oldestResident = 60u;
		residency.newestResident = 119u;
		lanes.noteResidencyReading(residency, 119u);
		reached.insert(frameMeterHorizonOf(lanes, retainedLaneWindow(lanes, 120u)).kind);
	}
	// OnElidedSpan.
	{
		InputHistoryTickLanes lanes;
		const uint32_t        afterIdle = pollRun(0u, 10u, 35u, lanes);
		pollRun(afterIdle, 15u, 0u, lanes);
		const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);
		WindowResidency      residency;
		residency.hasCache       = true;
		residency.anyResident    = true;
		residency.oldestResident = span.simTick + span.skippedTicks / 2u;
		residency.newestResident = 59u;
		lanes.noteResidencyReading(residency, 59u);
		reached.insert(frameMeterHorizonOf(lanes, retainedLaneWindow(lanes, 120u)).kind);
	}
	// InsideOpenSpan.
	{
		InputHistoryTickLanes lanes;
		const uint32_t        lastTick = pollRun(0u, 10u, 20u, lanes) - 1u;
		WindowResidency       residency;
		residency.hasCache       = true;
		residency.anyResident    = true;
		residency.oldestResident = 10u + kLanePauseEngageTicks + 2u;
		residency.newestResident = lastTick;
		lanes.noteResidencyReading(residency, lastTick);
		reached.insert(frameMeterHorizonOf(lanes, retainedLaneWindow(lanes, 120u)).kind);
	}
	// WholeBarLive.
	{
		InputHistoryTickLanes lanes;
		uint32_t              tick = 0u;
		auto                  step = [&lanes, &tick](bool inactive)
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
		const uint32_t  droppedSpanTick = lanes.gate().axisEventAt(0u).simTick - 4u;
		WindowResidency residency;
		residency.hasCache       = true;
		residency.anyResident    = true;
		residency.oldestResident = droppedSpanTick;
		residency.newestResident = tick - 1u;
		lanes.noteResidencyReading(residency, tick - 1u);
		reached.insert(frameMeterHorizonOf(lanes, retainedLaneWindow(lanes, 120u)).kind);
	}

	CHECK(reached.size() == static_cast<std::size_t>(kFrameMeterHorizonKindCount));
}

// ---------------------------------------------------------------------------
// THE TWO MARKERS' SHARED X MAPPING -- the RightEdge coincidence, proved rather than assumed.
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.ARightEdgeHorizonAndARightEdgeAuthorityMarkerShareAnX",
          "[CharacterViz][InputHistoryViz]")
{
	// Standing still past the idle-engage threshold with the pause on is the steady state
	// where BOTH markers land InsideOpenSpan/RightEdge -- a coincidence the design accepts.
	InputHistoryTickLanes lanes;
	const uint32_t        lastTick = pollRun(0u, 10u, 20u, lanes) - 1u;

	lanes.noteAuthorityReading(4u, lastTick);

	WindowResidency residency;
	residency.hasCache       = true;
	residency.anyResident    = true;
	residency.oldestResident = 10u + kLanePauseEngageTicks + 2u;
	residency.newestResident = lastTick;
	lanes.noteResidencyReading(residency, lastTick);

	const PollWindow                window   = retainedLaneWindow(lanes, 120u);
	const FrameMeterGeometry        geometry = geometryFor(1280.f, 720.f, frameMeterCellCount(window));
	const FrameMeterHorizon         horizon  = frameMeterHorizonOf(lanes, window);
	const auto                      marker   = frameMeterAuthorityMarkerOf(lanes, window);

	REQUIRE(horizon.kind == FrameMeterHorizonKind::InsideOpenSpan);
	REQUIRE(horizon.anchor == AuthorityMarkerAnchor::RightEdge);
	REQUIRE(marker.anchor == AuthorityMarkerAnchor::RightEdge);

	CHECK(nearlyEqual(authorityMarkerX(geometry, horizon), authorityMarkerX(geometry, marker)));
}

// ---------------------------------------------------------------------------
// THE PROVENANCE RESIDENCY READOUT.
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.TheResidencyReadoutCountsAHealthyWindowAndAWipeAndANoCacheRole",
          "[CharacterViz][InputHistoryViz]")
{
	// Healthy: sixty resident ticks, oldest to newest.
	{
		InputHistoryTickLanes lanes;
		WindowResidency       residency;
		residency.hasCache       = true;
		residency.anyResident    = true;
		residency.oldestResident = 1175u;
		residency.newestResident = 1234u;
		lanes.noteResidencyReading(residency, 1234u);

		const ProvenanceResidencyReadout readout = buildProvenanceResidencyReadout(lanes);
		CHECK(readout.present);
		CHECK(readout.hasCache);
		CHECK(readout.anyResident);
		CHECK(readout.oldestResident == 1175u);
		CHECK(readout.newestResident == 1234u);
		CHECK(readout.residentCount == 60u);
	}

	// The wipe: only the frontier itself survives -- the edge equals the frontier.
	{
		InputHistoryTickLanes lanes;
		WindowResidency       residency;
		residency.hasCache       = true;
		residency.anyResident    = true;
		residency.oldestResident = 1234u;
		residency.newestResident = 1234u;
		lanes.noteResidencyReading(residency, 1234u);

		const ProvenanceResidencyReadout readout = buildProvenanceResidencyReadout(lanes);
		CHECK(readout.residentCount == 1u);
		CHECK(readout.oldestResident == readout.newestResident);
	}

	// No correction cache at all: the authority role.
	{
		InputHistoryTickLanes lanes;
		WindowResidency       residency;
		residency.hasCache    = false;
		residency.anyResident = false;
		lanes.noteResidencyReading(residency, 900u);

		const ProvenanceResidencyReadout readout = buildProvenanceResidencyReadout(lanes);
		CHECK_FALSE(readout.hasCache);
		CHECK(readout.residentCount == 0u);
	}

	// No reading at all -- the provenance bar is not being fed.
	InputHistoryTickLanes noReading;
	CHECK_FALSE(buildProvenanceResidencyReadout(noReading).present);
}

TEST_CASE("FrameMeter.BothBarsAreReadFromOneWindowAndAHoleIsNeverAState",
          "[CharacterViz][InputHistoryViz]")
{
	// The fill asymmetry, as a renderer sees it: provenance answers for every tick in the
	// window, machine state only for the two that were live when a poll ran.
	InputHistoryTickLanes lanes;
	for (uint32_t tick = 100u; tick <= 104u; ++tick)
		lanes.editProvenance().record(tick, RowProvenanceSummary::Confirmed);

	lanes.editMachineState().recordIfAbsent(101u, MachineStateCell::Attacking);
	lanes.editMachineState().recordIfAbsent(103u, MachineStateCell::HitFlinch);
	lanes.noteAxisTick(104u);

	const PollWindow window = retainedLaneWindow(lanes, 5u);

	FrameMeterBarCells provenanceBar;
	FrameMeterBarCells machineBar;
	readProvenanceBar(lanes, window, provenanceBar);
	readMachineStateBar(lanes, window, machineBar);

	uint32_t provenanceFilled = 0u;
	uint32_t machineFilled    = 0u;
	for (uint32_t offset = 0u; offset < provenanceBar.count; ++offset)
	{
		if (provenanceBar.cells[offset].filled)
			++provenanceFilled;
		if (machineBar.cells[offset].filled)
			++machineFilled;
	}

	CHECK(provenanceBar.count == machineBar.count);
	CHECK(provenanceFilled == 5u);
	CHECK(machineFilled == 2u);
	CHECK_FALSE(machineBar.cells[0].filled);
	CHECK(machineBar.cells[1].value == static_cast<uint8_t>(MachineStateCell::Attacking));
}

// ---------------------------------------------------------------------------
// THE DELAY BAR AND ITS READOUT
// ---------------------------------------------------------------------------

TEST_CASE("FrameMeter.ANoVerdictCellIsAHoleAndALagUnverifiedCellIsFilled",
          "[CharacterViz][InputHistoryViz]")
{
	// Both counted in one fixture, as the AC requires: an untouched tick (NoVerdict) and a
	// server half with no client half yet (LagUnverified) must not be drawn the same way.
	InputHistoryTickLanes lanes;

	InputDelayCell verified;
	verified.serverLagTicks = 4;
	lanes.editDelay().record(101u, verified);
	lanes.noteAxisTick(101u);

	const PollWindow window = retainedLaneWindow(lanes, 5u);

	FrameMeterBarCells bar;
	readDelayBar(lanes, window, bar);

	uint32_t holes  = 0u;
	uint32_t filled = 0u;
	for (uint32_t offset = 0u; offset < bar.count; ++offset)
	{
		if (bar.cells[offset].filled)
			++filled;
		else
			++holes;
	}

	CHECK(bar.count == frameMeterCellCount(window));
	CHECK(holes == 4u);
	CHECK(filled == 1u);
	CHECK(bar.cells[4].value == static_cast<uint8_t>(InputDelayVerdict::LagUnverified));
}

TEST_CASE("FrameMeter.TheDelayReadoutCountsDivergenceAndNamesTheNewestVerdict",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;

	InputDelayDecomposition decomposition;
	decomposition.effectiveTicks = 3;
	decomposition.formulaTicks   = 3;
	decomposition.publishedTicks = 3;
	lanes.noteDelayReading(decomposition, 500u);

	// lag(0) < D(3) - 1 -> ServerEarlier.
	InputDelayCell earlier;
	earlier.clientDelayTicks = 3;
	earlier.serverLagTicks   = 0;
	lanes.editDelay().record(496u, earlier);

	// lag(9) > D(3) -> ServerLater.
	InputDelayCell later;
	later.clientDelayTicks = 3;
	later.serverLagTicks   = 9;
	lanes.editDelay().record(497u, later);

	// lag == D -> Agree, and the NEWEST filled cell in the window.
	InputDelayCell agree;
	agree.clientDelayTicks = 3;
	agree.serverLagTicks   = 3;
	lanes.editDelay().record(498u, agree);

	lanes.noteAxisTick(500u);

	const PollWindow        window  = retainedLaneWindow(lanes, 10u);
	const InputDelayReadout readout = buildInputDelayReadout(lanes, window);

	CHECK(readout.present);
	CHECK(readout.divergedInWindow == 2u);
	REQUIRE(readout.newestVerdict.has_value());
	CHECK(*readout.newestVerdict == InputDelayVerdict::Agree);
	CHECK_FALSE(readout.formulaMismatch);
	CHECK_FALSE(readout.publishMismatch);

	InputHistoryTickLanes noReading;
	noReading.noteAxisTick(10u);
	const PollWindow noReadingWindow = retainedLaneWindow(noReading, 5u);
	CHECK_FALSE(buildInputDelayReadout(noReading, noReadingWindow).present);
}

TEST_CASE("FrameMeter.TheDelayReadoutSitsBelowTheOffsetLabelAsASecondLine",
          "[CharacterViz][InputHistoryViz]")
{
	// The offset label and an elision count already must not share a line; the readout
	// adds a SECOND line below the offset for the same reason -- three numbers, three rows.
	const FrameMeterLayout   layout;
	const FrameMeterGeometry geometry    = geometryFor(1920.f, 1080.f, 120u);
	const float              labelHeight = 12.f;

	const float offsetTop  = frameMeterAuthorityLabelTopY(geometry, layout);
	const float readoutTop = frameMeterDelayReadoutTopY(geometry, layout, labelHeight);

	CHECK(readoutTop > offsetTop);
	CHECK(nearlyEqual(readoutTop, offsetTop + labelHeight + layout.backdropPadding));
}


// ---------------------------------------------------------------------------
// THE AXIS EVENTS -- ONE WALK, TWO KINDS
//
// A resync marker and an elision marker occupy the same one empty lane tick and are drawn
// by the same mechanism, so they are collected by the same walk. What separates them is
// the CLAIM each makes, and colour is how this meter states a claim.
// ---------------------------------------------------------------------------

// One admit through the gate, with the axis tick filed exactly as the poll files it.
// How many of one collection's marks are elisions. Derived from the ONE walk the header
// still offers, so this fixture cannot disagree with it about which markers a window reaches.
uint32_t elisionMarkCount(const FrameMeterAxisEventList& events)
{
	uint32_t count = 0u;
	for (uint32_t index = 0u; index < events.count; ++index)
	{
		if (events.marks[index].kind == LaneAxisEventKind::Elision)
			++count;
	}
	return count;
}

void stepLane(uint32_t simTick, bool inactive, std::optional<uint32_t> axisBreakFromSimTick,
              InputHistoryTickLanes& lanes)
{
	if (lanes.editGate().admit(simTick, inactive, true, axisBreakFromSimTick)
		== LaneAdmission::Recorded)
	{
		lanes.noteAxisTick(*lanes.gate().laneTickOf(simTick));
	}
}

TEST_CASE("FrameMeter.TheResyncMarkerClearsTheCrossFloorAgainstEveryColourTheBarsCanDraw",
          "[CharacterViz][InputHistoryViz]")
{
	// The marker is a cut drawn THROUGH every bar at one column, so a resync that read as a
	// state would turn a break in the axis into a claim about a tick -- and one that read as
	// the elision beside it would say time was removed when time was re-run.
	uint32_t checked = 0u;

	for (uint8_t ordinal = 0u; ordinal < kRowProvenanceSummaryCount; ++ordinal)
	{
		CHECK(laneColorGap(kLaneResyncColor, provenanceCellStyleOfOrdinal(ordinal).color)
			>= kLanePaletteMinCrossGap);
		++checked;
	}
	// Ordinal 0 is the hole in each of the two lower palettes -- no colour to sweep.
	for (uint8_t ordinal = 1u; ordinal < kMachineStateCellCount; ++ordinal)
	{
		CHECK(laneColorGap(kLaneResyncColor, machineCellStyleOfOrdinal(ordinal).color)
			>= kLanePaletteMinCrossGap);
		++checked;
	}
	for (uint8_t ordinal = 1u; ordinal < kInputDelayVerdictCount; ++ordinal)
	{
		CHECK(laneColorGap(kLaneResyncColor, delayVerdictStyleOfOrdinal(ordinal).color)
			>= kLanePaletteMinCrossGap);
		++checked;
	}

	const LaneCellColor outOfPalette[] = { kUnnamedLaneColor, kLaneElisionColor };
	for (const LaneCellColor& marker : outOfPalette)
	{
		CHECK(laneColorGap(kLaneResyncColor, marker) >= kLanePaletteMinCrossGap);
		++checked;
	}

	// Nine provenance colours, four machine states, six delay verdicts, the unnamed
	// sentinel and the elision band. A palette that grew without this sweep growing with
	// it would pass vacuously.
	CHECK(checked == static_cast<uint32_t>(kRowProvenanceSummaryCount)
		+ static_cast<uint32_t>(kMachineStateCellCount) - 1u
		+ static_cast<uint32_t>(kInputDelayVerdictCount) - 1u + 2u);
}

TEST_CASE("FrameMeter.TheStormsResyncMarksCarryTheSignedDriftAtEveryMarkerColumn",
          "[CharacterViz][InputHistoryViz]")
{
	// THE STORM LAYOUT, STATED IN FULL -- every number below falls out of these four. The
	// clock assigns newTick over oldTick and the two sit 22 apart, so the signed delta is
	// newTick - oldTick. Each cycle then polls the new epoch's first tick and the ticks up
	// to oldTick again: oldTick - newTick + 1 = 23 RECORDED lane ticks, plus the ONE the
	// marker cell costs, is 24 lane ticks per cycle.
	const uint32_t kNewTick  = 6197u;
	const uint32_t kOldTick  = 6219u;
	const uint32_t kCycles   = 128u;
	const uint32_t kBaseline = 50u;

	InputHistoryTickLanes lanes;

	for (uint32_t tick = 1u; tick <= kBaseline; ++tick)
		stepLane(tick, false, std::nullopt, lanes);
	REQUIRE(lanes.newestAxisTick() == kBaseline);

	uint32_t lastSimTick = kBaseline;
	for (uint32_t cycle = 0u; cycle < kCycles; ++cycle)
	{
		stepLane(kNewTick, false, std::optional<uint32_t>(lastSimTick), lanes);
		for (uint32_t tick = kNewTick + 1u; tick <= kOldTick; ++tick)
			stepLane(tick, false, std::nullopt, lanes);
		lastSimTick = kOldTick;
	}

	// Read off the axis's OWN bookkeeping rather than counted by hand.
	const uint32_t laneTicksPerCycle = kOldTick - kNewTick + 2u;
	CHECK(laneTicksPerCycle == 24u);
	CHECK(lanes.newestAxisTick() - kBaseline == kCycles * laneTicksPerCycle);
	REQUIRE(lanes.gate().axisEventCount() == kLaneElisionLedgerCapacity);

	const PollWindow window = retainedLaneWindow(lanes, 120u);
	REQUIRE(window.tickCount() == 120u);

	FrameMeterAxisEventList events;
	collectFrameMeterAxisEvents(lanes, window, events);

	// The newest lane tick is kBaseline + 128 * 24 = 3122 and the window reaches back 119 of
	// them, to 3003; the markers sit at 51 + 24k, so 3003 IS one of them, and five of the
	// storm's 128 fall inside the bar, a whole cycle apart.
	REQUIRE(events.count == 5u);
	CHECK(events.marks[0].offset == 0u);

	for (uint32_t index = 0u; index < events.count; ++index)
	{
		CHECK(events.marks[index].kind == LaneAxisEventKind::Resync);
		CHECK(events.marks[index].deltaTicks == -static_cast<int32_t>(kOldTick - kNewTick));
		CHECK(events.marks[index].skippedTicks == 0u);

		if (index > 0u)
		{
			CHECK(events.marks[index].offset - events.marks[index - 1u].offset
				== laneTicksPerCycle);
		}
	}

	// And not one of them is an elision, which is exactly the blindness this second kind of
	// marker exists to end: the collection this one replaced saw nothing here at all.
	CHECK(elisionMarkCount(events) == 0u);
}

TEST_CASE("FrameMeter.OverAPauseFixtureTheAxisEventsAreExactlyTheElisionsMarkForMark",
          "[CharacterViz][InputHistoryViz]")
{
	// The generalisation must have changed NOTHING for an elision-only ledger, so the two
	// collections are pinned side by side -- and each is pinned against the gate that made
	// the cut, so agreeing with each other is not enough to pass.
	InputHistoryTickLanes lanes;

	uint32_t next = pollRun(0u, 10u, 40u, lanes);
	next          = pollRun(next, 10u, 60u, lanes);
	pollRun(next, 10u, 0u, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 2u);

	const PollWindow window = retainedLaneWindow(lanes, 120u);

	FrameMeterAxisEventList events;
	collectFrameMeterAxisEvents(lanes, window, events);

	REQUIRE(events.count == 2u);
	REQUIRE(elisionMarkCount(events) == events.count);

	for (uint32_t index = 0u; index < events.count; ++index)
	{
		const LaneAxisEvent& filed = lanes.gate().axisEventAt(index);

		CHECK(events.marks[index].offset == filed.laneTick - window.oldestTick);
		CHECK(events.marks[index].skippedTicks == filed.skippedTicks);
		CHECK(events.marks[index].kind == LaneAxisEventKind::Elision);

		// An elision's label is its count; the signed delta belongs to the other kind and
		// stays at zero, so one marker can never print both labels.
		CHECK(events.marks[index].deltaTicks == 0);
	}
}

TEST_CASE("FrameMeter.AForwardResyncsDeadRangeLandsOnItsMarkerCellRatherThanOffTheBar",
          "[CharacterViz][InputHistoryViz]")
{
	// A forward resync jumps the axis over ticks nothing ever simulated. The axis has no
	// cells for them, exactly as it has none for an elided span, so the marker cell answers
	// for them. ⛔ NEVER THE OFF-THE-LEFT-EDGE ANSWER, which would report authority as
	//   older than a bar that is holding its column.
	const uint32_t kLastOldEpochTick = 100u;
	const uint32_t kNewEpochFirst    = 131u;

	InputHistoryTickLanes lanes;

	for (uint32_t tick = 1u; tick <= kLastOldEpochTick; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	stepLane(kNewEpochFirst, false, std::optional<uint32_t>(kLastOldEpochTick), lanes);
	for (uint32_t tick = kNewEpochFirst + 1u; tick <= kNewEpochFirst + 9u; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& resync = lanes.gate().axisEventAt(0u);
	REQUIRE(resync.kind == LaneAxisEventKind::Resync);

	const PollWindow window = retainedLaneWindow(lanes, 120u);

	// Both ends of the dead range and a tick in the middle of it.
	const uint32_t deadTicks[] = { kLastOldEpochTick + 1u, 115u, kNewEpochFirst - 1u };
	for (const uint32_t simTick : deadTicks)
	{
		REQUIRE_FALSE(lanes.gate().laneTickOf(simTick).has_value());

		const FrameMeterSimTickPlacement placement =
			placeFrameMeterSimTick(lanes, window, simTick);

		CHECK(placement.kind == AuthorityMarkerKind::OnElidedSpan);
		CHECK(placement.anchor == AuthorityMarkerAnchor::Column);
		CHECK(placement.barOffset == resync.laneTick - window.oldestTick);
	}

	// The ticks either side still own their own cells, so the dead range is exactly the
	// never-simulated one and not a tick wider at either end.
	CHECK(placeFrameMeterSimTick(lanes, window, kLastOldEpochTick).kind
		== AuthorityMarkerKind::OnCell);
	CHECK(placeFrameMeterSimTick(lanes, window, kNewEpochFirst).kind
		== AuthorityMarkerKind::OnCell);
}

TEST_CASE("FrameMeter.AResyncIsNeverReadAsASpanWhenTheLedgerHoldsBothKinds",
          "[CharacterViz][InputHistoryViz]")
{
	// A BACKWARD resync's dead range is empty -- its ticks were run twice, not never -- so
	// the branch that answers for a forward jump has to decline here, with a real elision
	// filed either side of it for the walk to step past.
	InputHistoryTickLanes lanes;

	for (uint32_t tick = 0u; tick <= 9u; ++tick)
		stepLane(tick, false, std::nullopt, lanes);
	for (uint32_t tick = 10u; tick <= 29u; ++tick)
		stepLane(tick, true, std::nullopt, lanes);
	for (uint32_t tick = 30u; tick <= 40u; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	stepLane(35u, false, std::optional<uint32_t>(40u), lanes);
	for (uint32_t tick = 36u; tick <= 45u; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	for (uint32_t tick = 46u; tick <= 65u; ++tick)
		stepLane(tick, true, std::nullopt, lanes);
	for (uint32_t tick = 66u; tick <= 70u; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 3u);
	CHECK(lanes.gate().axisEventAt(0u).kind == LaneAxisEventKind::Elision);
	CHECK(lanes.gate().axisEventAt(1u).kind == LaneAxisEventKind::Resync);
	CHECK(lanes.gate().axisEventAt(2u).kind == LaneAxisEventKind::Elision);

	const LaneAxisEvent& resync = lanes.gate().axisEventAt(1u);

	// The structural reason it can never be mis-read: its dead range begins PAST where it
	// ends, so the range is empty and the walk falls through to the older entries.
	CHECK(resync.fromSimTick + 1u > resync.simTick);

	const PollWindow window = retainedLaneWindow(lanes, 120u);

	FrameMeterAxisEventList events;
	collectFrameMeterAxisEvents(lanes, window, events);
	REQUIRE(events.count == 3u);
	CHECK(events.marks[0].skippedTicks == 5u);
	CHECK(events.marks[0].deltaTicks == 0);
	CHECK(events.marks[1].skippedTicks == 0u);
	CHECK(events.marks[1].deltaTicks == -5);
	CHECK(events.marks[2].skippedTicks == 5u);
	CHECK(events.marks[2].deltaTicks == 0);

	// Two of the three are elisions, named by their own kind rather than by a second walk.
	REQUIRE(elisionMarkCount(events) == 2u);
	CHECK(events.marks[0].kind == LaneAxisEventKind::Elision);
	CHECK(events.marks[2].kind == LaneAxisEventKind::Elision);

	// EXHAUSTIVE over every tick the fixture ever admitted: exactly the two elided spans --
	// five ticks each -- have no cell of their own, and the resync's re-run ticks all land
	// on a column. A dead range wrongly opened for the backward break would move this.
	uint32_t onSpan = 0u;
	uint32_t onCell = 0u;
	for (uint32_t simTick = 0u; simTick <= 70u; ++simTick)
	{
		const AuthorityMarkerKind kind = placeFrameMeterSimTick(lanes, window, simTick).kind;
		if (kind == AuthorityMarkerKind::OnElidedSpan)
			++onSpan;
		if (kind == AuthorityMarkerKind::OnCell)
			++onCell;
	}

	CHECK(onSpan == 10u);
	CHECK(onCell == 61u);

	for (uint32_t simTick = resync.simTick; simTick <= resync.fromSimTick; ++simTick)
	{
		CHECK(placeFrameMeterSimTick(lanes, window, simTick).kind
			== AuthorityMarkerKind::OnCell);
	}
}

TEST_CASE("FrameMeter.AResyncEndingAPausedSpanDrawsTwoAdjacentCellsOfDifferentKinds",
          "[CharacterViz][InputHistoryViz]")
{
	// The one arrangement that puts BOTH kinds of marker side by side: the player stands
	// still long enough for the gate to pause, and the clock hard-resyncs before anything
	// moves again. The break closes the open span first, so the elision claims one lane
	// tick and the resync claims the very next.
	// ⛔ THE COLOUR AND THE LABEL ARE CHOSEN PER MARK. A renderer that decided either
	//   once for the whole ledger would paint these two adjacent cells the same.
	const uint32_t kActiveTicks = 30u;
	const uint32_t kIdleUntil   = 60u;
	const uint32_t kNewTick     = 34u;

	InputHistoryTickLanes lanes;

	for (uint32_t tick = 1u; tick <= kActiveTicks; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	for (uint32_t tick = kActiveTicks + 1u; tick <= kIdleUntil; ++tick)
		stepLane(tick, true, std::nullopt, lanes);

	REQUIRE(lanes.gate().paused());

	// The break arrives from the tick the previous epoch was last polled at, which is what
	// the poll hands the gate -- never a tick chosen for this fixture.
	stepLane(kNewTick, false, std::optional<uint32_t>(kIdleUntil), lanes);

	for (uint32_t tick = kNewTick + 1u; tick <= kNewTick + 10u; ++tick)
		stepLane(tick, false, std::nullopt, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 2u);

	const LaneAxisEvent& elision = lanes.gate().axisEventAt(0u);
	const LaneAxisEvent& resync  = lanes.gate().axisEventAt(1u);

	CHECK(elision.kind == LaneAxisEventKind::Elision);
	CHECK(resync.kind == LaneAxisEventKind::Resync);

	// Adjacency is the gate's own bookkeeping, read off the ledger rather than assumed:
	// closing the span costs one lane tick and the break takes the one after it.
	CHECK(resync.laneTick == elision.laneTick + 1u);

	const PollWindow window = retainedLaneWindow(lanes, 120u);

	FrameMeterAxisEventList events;
	collectFrameMeterAxisEvents(lanes, window, events);

	REQUIRE(events.count == 2u);
	CHECK(elisionMarkCount(events) == 1u);

	// TWO CELLS AT ADJACENT COLUMNS, and the renderer is told they are of different kinds.
	CHECK(events.marks[1].offset == events.marks[0].offset + 1u);
	CHECK(events.marks[0].kind == LaneAxisEventKind::Elision);
	CHECK(events.marks[1].kind == LaneAxisEventKind::Resync);
	CHECK(events.marks[0].kind != events.marks[1].kind);

	// TWO COLOURS: the kinds above select these two constants, and they are further apart
	// than the palette floor any two drawn colours must clear.
	CHECK(laneColorGap(kLaneElisionColor, kLaneResyncColor) >= kLanePaletteMinCrossGap);

	// And two LABELS: each mark carries the number its own kind prints and zero for the
	// other, so one marker can never print both.
	CHECK(events.marks[0].skippedTicks
		== kIdleUntil + 1u - (kActiveTicks + 1u + kLanePauseEngageTicks));
	CHECK(events.marks[0].deltaTicks == 0);
	CHECK(events.marks[1].skippedTicks == 0u);
	CHECK(events.marks[1].deltaTicks
		== static_cast<int32_t>(kNewTick) - static_cast<int32_t>(kIdleUntil));

	// Neither cell belongs to a lane: both are holes, so §7.3's rule covers them and the
	// runs either side keep their own lengths.
	CHECK(lanes.provenanceAt(elision.laneTick) == nullptr);
	CHECK(lanes.provenanceAt(resync.laneTick) == nullptr);
	CHECK(lanes.machineCellAt(elision.laneTick) == MachineStateCell::NotSampled);
	CHECK(lanes.machineCellAt(resync.laneTick) == MachineStateCell::NotSampled);
}

} // namespace inputhistoryframemetertests

#endif // WITH_LOW_LEVEL_TESTS
