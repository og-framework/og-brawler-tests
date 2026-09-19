// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the SECOND frame-meter stack: who it draws, where it sits, and where its
// input-delay bar's client half comes from.
//
// WHAT THIS SUITE IS REALLY GUARDING is that adding a stack did not move the one that
// was already there. The frame meter is the instrument a netcode investigation is read
// off, and an instrument that shifted when it grew a second channel would silently
// invalidate every screenshot taken before the change. So the first three cases here are
// IDENTITY pins on bit patterns rather than tolerances: the anchored geometry, the lift
// of zero, and a locally controlled character's delay cells.
//
// The second thing it guards is that the second stack FITS. There is no room under the
// bottom-anchored meter -- its own label band and readouts already spend the bottom
// margin -- so the nearest stack takes that anchor and the primary is lifted off it. That
// arithmetic is pinned at 1080p AND at 720p, because a PIE window is usually the smaller
// of the two and a stack that clips off the top of one is a stack nobody can read.
//
// The third is that a REMOTE character's delay bar compares two things that are
// comparable. A remote proxy has no delay line of this client's to read, so its client
// half is the relayed-resolution decision -- and a client half taken from the wrong
// source would produce a confidently wrong verdict rather than no verdict.

#include "catch_amalgamated.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPanel.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistorynexteststack
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::FrameMeterBarKind;
using brawlerInputHistoryVisualization::FrameMeterBarSelection;
using brawlerInputHistoryVisualization::FrameMeterGeometry;
using brawlerInputHistoryVisualization::FrameMeterLayout;
using brawlerInputHistoryVisualization::InputDelayCell;
using brawlerInputHistoryVisualization::InputDelayDecomposition;
using brawlerInputHistoryVisualization::InputDelayVerdict;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::NearestCharacterCandidate;
using brawlerInputHistoryVisualization::NearestCharacterCandidateList;
using brawlerInputHistoryVisualization::PollWindow;

using brawlerInputHistoryVisualization::delayVerdictOf;
using brawlerInputHistoryVisualization::frameMeterBarSlotOf;
using brawlerInputHistoryVisualization::frameMeterElisionLabelTopY;
using brawlerInputHistoryVisualization::frameMeterEnabledBarCount;
using brawlerInputHistoryVisualization::frameMeterGeometryFor;
using brawlerInputHistoryVisualization::frameMeterHeight;
using brawlerInputHistoryVisualization::frameMeterLiftedBy;
using brawlerInputHistoryVisualization::frameMeterPrimaryLift;
using brawlerInputHistoryVisualization::frameMeterReadoutLineTopY;
using brawlerInputHistoryVisualization::frameMeterStackBottomY;
using brawlerInputHistoryVisualization::frameMeterStackHeight;
using brawlerInputHistoryVisualization::frameMeterStackTopY;
using brawlerInputHistoryVisualization::nearestCharacterDistanceCm;
using brawlerInputHistoryVisualization::nearestCharacterIdTo;
using brawlerInputHistoryVisualization::nearestPlanarDistanceSquared;

using brawlerInputHistoryVisualization::kFrameMeterStackGap;
using brawlerInputHistoryVisualization::kNearestCandidateCapacity;
using brawlerInputHistoryVisualization::kNearestHysteresisCm;
using brawlerInputHistoryVisualization::kTickLaneCapacity;

// ---------------------------------------------------------------------------
// HELPERS
// ---------------------------------------------------------------------------

// Bit-for-bit equality on a float, so an identity pin is an IDENTITY pin. A tolerance
// here would pass a geometry that had moved by a fraction of a pixel, which over a
// stride of eight is exactly the drift these cases exist to refuse.
// ⛔ COMPARED AS BIT PATTERNS, NEVER AS FLOATS.
bool identicalBits(float left, float right)
{
	std::uint32_t leftBits  = 0u;
	std::uint32_t rightBits = 0u;
	std::memcpy(&leftBits, &left, sizeof(leftBits));
	std::memcpy(&rightBits, &right, sizeof(rightBits));
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
	const float delta = (left > right) ? (left - right) : (right - left);
	return delta < 0.01f;
}

NearestCharacterCandidateList listOf(const std::vector<NearestCharacterCandidate>& entries)
{
	NearestCharacterCandidateList candidates;
	for (const NearestCharacterCandidate& entry : entries)
		candidates.add(entry);
	return candidates;
}

// A reader that names the AUTHORITY's applied capture for a scripted set of sim ticks --
// the server half of every delay cell. Every other tick has no slot at all.
class ScriptedAuthorityReader
{
public:
	// `appliedAt[simTick] = captureTick` -- the tick the authority ran, and what it ran.
	void nameCapture(uint32_t simTick, uint32_t captureTick)
	{
		m_applied.push_back({ simTick, captureTick, false, std::nullopt });
	}

	// The same, plus what the correction cache says became of that slot's state.
	void nameCapture(uint32_t simTick, uint32_t captureTick, SlotStateProvenance provenance)
	{
		m_applied.push_back({ simTick, captureTick, false, provenance });
	}

	// The authority substituted at `simTick`, naming no capture at all.
	void nameSentinel(uint32_t simTick)
	{
		m_applied.push_back({ simTick, 0u, true, std::nullopt });
	}

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		for (const Entry& entry : m_applied)
		{
			if (entry.simTick != simTick)
				continue;

			AppliedCaptureRef ref;
			ref.kind = entry.sentinel ? AppliedCaptureRefKind::Sentinel
			                          : AppliedCaptureRefKind::Ref;
			ref.captureTick = entry.captureTick;
			return ref;
		}

		return AppliedCaptureRef{};
	}

	std::optional<SlotStateProvenance> slotProvenance(uint32_t simTick) const
	{
		for (const Entry& entry : m_applied)
		{
			if (entry.simTick == simTick)
				return entry.provenance;
		}
		return std::nullopt;
	}

	bool hasCorrectionCache() const { return true; }

private:
	struct Entry
	{
		uint32_t                           simTick     = 0u;
		uint32_t                           captureTick = 0u;
		bool                               sentinel    = false;
		std::optional<SlotStateProvenance> provenance;
	};

	std::vector<Entry> m_applied;
};

// The relayed-read ring's SHAPE, driven by hand. The production ring is
// `RelayedReadObservationRing` and the two are pinned against each other by
// `TheRingIsExactlyOneDisplayWindowWide` below; what this stands in for is the physics
// thread having already written it.
class ScriptedObservations
{
public:
	void served(uint32_t simTick, uint32_t appliedCaptureTick)
	{
		RelayedReadObservation observation;
		observation.simTick               = simTick;
		observation.appliedCaptureTick    = appliedCaptureTick;
		observation.hasAppliedCaptureTick = true;
		observation.outcome               = ScheduledRelayedReadOutcome::Hit;
		m_observations.push_back(observation);
	}

	// This client resolved the injected neutral: no capture of the sender's stands
	// behind the tick at all.
	void servedNeutral(uint32_t simTick)
	{
		RelayedReadObservation observation;
		observation.simTick = simTick;
		m_observations.push_back(observation);
	}

	std::size_t size() const { return m_observations.size(); }

	const RelayedReadObservation* at(std::size_t index) const
	{
		return &m_observations[index];
	}

	// The production ring tracks this as it is written; a stand-in derives it from what it
	// was handed, which is the same number for the same script.
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

// The arrival ring's SHAPE, driven by hand. The production ring is
// `RelayedInputArrivalRing`; what this stands in for is the arrival door having already
// written it on the game thread.
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

// Nothing has arrived, which is what every delay-bar fixture here is silent about.
const ScriptedArrivals kNoArrivals;

CaptureRowFields movingInput()
{
	CaptureRowFields fields;
	fields.direction  = DirectionBucket::Forward;
	fields.buttonMask = 0u;
	return fields;
}

// One lane poll of a LOCALLY CONTROLLED character: the delay line's own effective delay
// supplies the client half, exactly as it always has.
void pollLocalTick(uint32_t simTick, int32_t effectiveDelay,
                   const ScriptedAuthorityReader& reader, InputHistoryTickLanes& lanes)
{
	AppliedCaptureInversion inversion;

	InputDelayDecomposition delay;
	delay.effectiveTicks = effectiveDelay;

	brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, movingInput(), false, std::nullopt, delay, std::nullopt,
		inversion, lanes);
}

// One lane poll of a REMOTE proxy: no tier decomposition to read, and the client half
// comes from the relayed reads this client served for it.
void pollRemoteTick(uint32_t simTick, const ScriptedAuthorityReader& reader,
                    const ScriptedObservations& observations, InputHistoryTickLanes& lanes)
{
	AppliedCaptureInversion inversion;

	brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, movingInput(), false, std::nullopt, std::nullopt, std::nullopt,
		observations, kNoArrivals, 0u, inversion, lanes);
}

InputDelayVerdict verdictAtCaptureTick(const InputHistoryTickLanes& lanes, uint32_t captureTick)
{
	const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(captureTick);
	if (!laneTick.has_value())
		return InputDelayVerdict::NoVerdict;

	const InputDelayCell* cell = lanes.delay().find(*laneTick);
	return (cell == nullptr) ? InputDelayVerdict::NoVerdict : delayVerdictOf(*cell);
}

// ===========================================================================
// 1. THE THREE IDENTITY PINS -- what a second stack must not have moved.
// ===========================================================================

TEST_CASE("NearestStack.TheAnchoredGeometryIsBitIdenticalToTheOneStackDisplayAtEveryViewport",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout layout;

	// The whole range a PIE window is ever opened at, plus the two the budget is argued
	// at. ⛔ THE COMPARISON IS ON BIT PATTERNS: a tolerance would pass a drift.
	const float widths[]  = { 960.f, 1280.f, 1600.f, 1920.f, 2560.f, 3840.f };
	const float heights[] = { 540.f,  720.f,  900.f, 1080.f, 1440.f, 2160.f };

	std::size_t compared          = 0u;
	std::size_t fourArgIdentical  = 0u;
	std::size_t liftZeroIdentical = 0u;

	for (uint32_t index = 0u; index < 6u; ++index)
	{
		for (uint32_t cellCount = 1u; cellCount <= 240u; cellCount += 17u)
		{
			++compared;

			// The four-argument form still forwards to the two-bar shape.
			const FrameMeterGeometry fourArg =
				frameMeterGeometryFor(layout, widths[index], heights[index], cellCount);
			const FrameMeterGeometry fiveArg = frameMeterGeometryFor(
				layout, widths[index], heights[index], cellCount, 2u);

			if (identicalGeometry(fourArg, fiveArg))
				++fourArgIdentical;

			// With no second stack the lift is zero, and a lift of zero must be the
			// IDENTITY -- `OGBrawler.InputHistoryNearest 0` is pixel-for-pixel today.
			// ⭐ THE ESCAPE HATCH'S OWN PIN.
			for (uint32_t barCount = 1u; barCount <= 3u; ++barCount)
			{
				const FrameMeterGeometry anchored = frameMeterGeometryFor(
					layout, widths[index], heights[index], cellCount, barCount);

				if (identicalGeometry(anchored, frameMeterLiftedBy(anchored, 0.f)))
					++liftZeroIdentical;
			}
		}
	}

	CHECK(compared == 90u);
	CHECK(fourArgIdentical == compared);
	CHECK(liftZeroIdentical == compared * 3u);
}

TEST_CASE("NearestStack.TheLiftMovesTheOriginAndNothingElseAboutTheColumns",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout   layout;
	const FrameMeterGeometry anchored = frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, 3u);
	const FrameMeterGeometry lifted   = frameMeterLiftedBy(anchored, 124.f);

	// ⛔ ONE STRIDE ACROSS BOTH STACKS: a column must mean the same width of screen on
	//   either of them, or a reader comparing the two is comparing different windows.
	CHECK(identicalBits(anchored.originX, lifted.originX));
	CHECK(identicalBits(anchored.cellStride, lifted.cellStride));
	CHECK(identicalBits(anchored.cellWidth, lifted.cellWidth));
	CHECK(identicalBits(anchored.barHeight, lifted.barHeight));
	CHECK(identicalBits(anchored.barGap, lifted.barGap));
	CHECK(anchored.cellCount == lifted.cellCount);
	CHECK(anchored.barCount == lifted.barCount);

	CHECK(nearlyEqual(anchored.originY - lifted.originY, 124.f));
}

TEST_CASE("NearestStack.ALocalCharactersDelayCellsAreTheOnesItAlwaysHad",
          "[CharacterViz][InputHistoryViz]")
{
	// The authority applied capture `tick - 3` at each of six consecutive ticks, and the
	// client's own effective delay is 3: every cell agrees, which is the shipped
	// steady-state reading this pin is written against.
	ScriptedAuthorityReader reader;
	for (uint32_t simTick = 100u; simTick < 106u; ++simTick)
		reader.nameCapture(simTick, simTick - 3u);

	InputHistoryTickLanes lanes;
	for (uint32_t simTick = 100u; simTick < 106u; ++simTick)
		pollLocalTick(simTick, 3, reader, lanes);

	std::size_t agreed    = 0u;
	std::size_t clientSet = 0u;
	std::size_t serverSet = 0u;

	for (uint32_t captureTick = 97u; captureTick < 103u; ++captureTick)
	{
		const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(captureTick);
		REQUIRE(laneTick.has_value());

		const InputDelayCell* cell = lanes.delay().find(*laneTick);
		REQUIRE(cell != nullptr);

		if (cell->clientDelayTicks.has_value() && *cell->clientDelayTicks == 3)
			++clientSet;
		if (cell->serverLagTicks.has_value() && *cell->serverLagTicks == 3)
			++serverSet;
		if (delayVerdictOf(*cell) == InputDelayVerdict::Agree)
			++agreed;

		// ⛔ A LOCAL POLL FILES NO SENTINEL: the relay ring's own arm cannot reach it.
		CHECK_FALSE(cell->serverNamedNoCapture);
	}

	CHECK(clientSet == 6u);
	CHECK(serverSet == 6u);
	CHECK(agreed == 6u);
}

// ===========================================================================
// 2. THE SCREEN BUDGET -- the arithmetic, at both resolutions.
// ===========================================================================

TEST_CASE("NearestStack.TheTwoStacksFitAt1080pAndAt720pWithNothingClipped",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout layout;

	// The small font's line height at the shipped scale. The draw site measures the real
	// number every frame; this is the value the screen budget was argued at, and the
	// containment below is asserted for a range around it precisely because a font is
	// allowed to disagree with a design document.
	const float labelHeight = 14.f;

	// \u26a0 THE TWO STACKS DRAW DIFFERENT NUMBERS OF BARS. The relay-health bar is on the
	// stack following someone else's character and on no other, so the nearest draws four
	// and the primary three. Each still spends its own readout lines: the nearest the relay
	// line and residency, the primary its tier line, residency and the clock it keeps.
	const uint32_t primaryBarCount     = 3u;
	const uint32_t nearestBarCount     = 4u;
	const uint32_t nearestReadoutLines = 2u;
	const uint32_t primaryReadoutLines = 3u;

	// nearest bars 4*14 + 3*3 = 65; band above 3 + 14; offset label 3 + 14; two readouts
	// 2*(14+3) = 34.
	CHECK(nearlyEqual(
		frameMeterStackHeight(layout, nearestBarCount, nearestReadoutLines, labelHeight),
		3.f + 14.f + 65.f + 3.f + 14.f + 34.f));
	CHECK(nearlyEqual(
		frameMeterStackHeight(layout, nearestBarCount, nearestReadoutLines, labelHeight),
		133.f));

	// The primary: bars 3*14 + 2*3 = 48, and three readouts 3*(14+3) = 51.
	CHECK(nearlyEqual(
		frameMeterStackHeight(layout, primaryBarCount, primaryReadoutLines, labelHeight),
		3.f + 14.f + 48.f + 3.f + 14.f + 51.f));
	CHECK(nearlyEqual(
		frameMeterStackHeight(layout, primaryBarCount, primaryReadoutLines, labelHeight),
		133.f));

	// \u26d4 THE LIFT IS THE LIFTED STACK'S OWN HEIGHT **PLUS THE BAR BANDS' DIFFERENCE**:
	//   133 + (65 - 48) + 8 = 158. The anchored stack's bars are drawn ABOVE an origin
	//   pinned to the bottom margin, so its fourth bar raises its TOP edge by 17 without
	//   moving its bottom, and a lift blind to that leaves the two overlapping by 17 px.
	CHECK(nearlyEqual(frameMeterPrimaryLift(layout, primaryBarCount, primaryReadoutLines,
		labelHeight, nearestBarCount), 158.f));

	// Equal bar counts collapse the correction to nothing, which is the shape this
	// function shipped with. \u2b50 THE OLD ANSWER IS A SPECIAL CASE OF THE NEW ONE.
	CHECK(nearlyEqual(frameMeterPrimaryLift(layout, primaryBarCount, primaryReadoutLines,
		labelHeight, primaryBarCount), 141.f));
	CHECK(nearlyEqual(kFrameMeterStackGap, 8.f));

	const float widths[]  = { 1920.f, 1280.f };
	const float heights[] = { 1080.f,  720.f };

	// The two anchors, spelled out: 1080 - 0.09*1080 - 65 and 720 - 0.09*720 - 65.
	const float expectedAnchorY[] = { 917.8f, 590.2f };

	const float lift = frameMeterPrimaryLift(layout, primaryBarCount, primaryReadoutLines,
		labelHeight, nearestBarCount);

	std::size_t anchorsAgreed   = 0u;
	std::size_t primariesOnTop  = 0u;
	std::size_t nothingClipped  = 0u;

	for (uint32_t index = 0u; index < 2u; ++index)
	{
		const FrameMeterGeometry nearest =
			frameMeterGeometryFor(layout, widths[index], heights[index], 120u, nearestBarCount);

		if (nearlyEqual(nearest.originY, expectedAnchorY[index]))
			++anchorsAgreed;

		// \u26d4 THE PRIMARY ASKS FOR ITS OWN GEOMETRY, never the anchored stack's lifted: the
		//   two no longer share a bar count, so a lifted copy would carry the wrong band.
		const FrameMeterGeometry primary = frameMeterLiftedBy(
			frameMeterGeometryFor(layout, widths[index], heights[index], 120u, primaryBarCount),
			lift);

		// The lifted stack's LOWEST pixel clears the anchored stack's HIGHEST by exactly
		// the gap -- so the two never touch and never drift apart.
		const float primaryBottom =
			frameMeterStackBottomY(primary, layout, primaryReadoutLines, labelHeight);
		const float nearestTop = frameMeterStackTopY(nearest, layout, labelHeight);

		if (nearlyEqual(nearestTop - primaryBottom, kFrameMeterStackGap))
			++primariesOnTop;

		// \u26d4 EVERYTHING INSIDE [0, H): bars, BOTH label bands and every readout. The 720p
		//   column is the one that matters -- a PIE window is usually the smaller.
		const float nearestBottom =
			frameMeterStackBottomY(nearest, layout, nearestReadoutLines, labelHeight);
		const float primaryTop = frameMeterStackTopY(primary, layout, labelHeight);

		if (primaryTop >= 0.f && nearestBottom < heights[index]
			&& primary.originX >= 0.f && nearest.originX >= 0.f)
		{
			++nothingClipped;
		}
	}

	CHECK(anchorsAgreed == 2u);
	CHECK(primariesOnTop == 2u);
	CHECK(nothingClipped == 2u);

	// 1080p: the anchor is at 917.8, the primary's own origin is 982.8 - 48 - 158 = 776.8
	// and its band starts 17 above that at 759.8; the nearest's last readout ends at
	// 900.8 + 133 = 1033.8, inside 1080.
	const FrameMeterGeometry nearest1080 =
		frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, nearestBarCount);
	CHECK(nearlyEqual(frameMeterStackTopY(frameMeterLiftedBy(
		frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, primaryBarCount), 158.f),
		layout, labelHeight), 759.8f));
	CHECK(nearlyEqual(frameMeterStackBottomY(nearest1080, layout, nearestReadoutLines,
		labelHeight), 1033.8f));

	// 720p: 655.2 - 48 - 158 - 17 = 432.2 at the top, 573.2 + 133 = 706.2 at the bottom --
	// 13.8 px of the 64.8 px bottom margin still unspent, and nothing above the viewport.
	// \u2b50 THE BOTTOM EDGE DID NOT MOVE when the fourth bar landed: the anchored origin is
	// measured UP from the bottom margin, so a taller bar band spends the top, not the
	// bottom, and the readout-line count is what the bottom is made of.
	const FrameMeterGeometry nearest720 =
		frameMeterGeometryFor(layout, 1280.f, 720.f, 120u, nearestBarCount);
	CHECK(nearlyEqual(frameMeterStackTopY(frameMeterLiftedBy(
		frameMeterGeometryFor(layout, 1280.f, 720.f, 120u, primaryBarCount), 158.f),
		layout, labelHeight), 432.2f));
	CHECK(nearlyEqual(frameMeterStackBottomY(nearest720, layout, nearestReadoutLines,
		labelHeight), 706.2f));

	// \u26a0 WHERE 720p ACTUALLY RUNS OUT, measured rather than assumed. The bottom edge is
	// `664.2 + 3 * labelHeight` -- unchanged by the fourth bar -- so the nearest stack's
	// last readout leaves the viewport once the small font reports a line height of 19 px
	// or more: probes 10..18 fit and 19 and 20 do not. The top never binds: it is now
	// `516.2 - 6 * labelHeight`, still 396 px clear at 20. The shipped font reports 14,
	// which leaves 5 px of margin on that bound.
	std::size_t heightsThatFit = 0u;
	std::size_t topsInside     = 0u;
	for (float probe = 10.f; probe <= 20.f; probe += 1.f)
	{
		const FrameMeterGeometry anchored =
			frameMeterGeometryFor(layout, 1280.f, 720.f, 120u, nearestBarCount);
		const FrameMeterGeometry lifted = frameMeterLiftedBy(
			frameMeterGeometryFor(layout, 1280.f, 720.f, 120u, primaryBarCount),
			frameMeterPrimaryLift(layout, primaryBarCount, primaryReadoutLines, probe,
				nearestBarCount));

		if (frameMeterStackTopY(lifted, layout, probe) >= 0.f)
			++topsInside;

		if (frameMeterStackTopY(lifted, layout, probe) >= 0.f
			&& frameMeterStackBottomY(anchored, layout, nearestReadoutLines, probe) < 720.f)
		{
			++heightsThatFit;
		}
	}
	CHECK(topsInside == 11u);
	CHECK(heightsThatFit == 9u);
}

TEST_CASE("NearestStack.TheStackHeightIsTheBandsThePlacementHelpersActuallyUse",
          "[CharacterViz][InputHistoryViz]")
{
	const FrameMeterLayout   layout;
	const float              labelHeight = 14.f;
	const FrameMeterGeometry geometry =
		frameMeterGeometryFor(layout, 1920.f, 1080.f, 120u, 3u);

	// The derived height must equal the distance between the two helpers that place the
	// real bands -- otherwise the lift is an independent guess that can drift from the
	// draw. ⛔ NOT RESTATED, DERIVED.
	for (uint32_t readoutLines = 0u; readoutLines <= 3u; ++readoutLines)
	{
		const float top = frameMeterElisionLabelTopY(geometry, layout, labelHeight);
		const float bottom = (readoutLines == 0u)
			? geometry.originY + frameMeterHeight(geometry) + layout.backdropPadding + labelHeight
			: frameMeterReadoutLineTopY(geometry, layout, labelHeight, readoutLines - 1u)
			      + labelHeight;

		CHECK(nearlyEqual(bottom - top,
			frameMeterStackHeight(layout, geometry.barCount, readoutLines, labelHeight)));
	}
}

// ===========================================================================
// 3. WHO THE SECOND STACK DRAWS.
// ===========================================================================

TEST_CASE("NearestStack.NearestPicksTheSmallestPlanarDistanceIgnoringTheLocalIdItself",
          "[CharacterViz][InputHistoryViz]")
{
	// The local at the origin; one rival at 300 cm, one at 900, one directly overhead in
	// Z -- which this selection cannot see at all, and must not, because a brawler above
	// another is not the one it is fighting.
	const NearestCharacterCandidateList candidates = listOf({
		NearestCharacterCandidate{ 10u,   0.f,   0.f },
		NearestCharacterCandidate{ 20u, 900.f,   0.f },
		NearestCharacterCandidate{ 30u,   0.f, 300.f },
	});

	CHECK(nearestCharacterIdTo(candidates, 10u, std::nullopt) == std::optional<unsigned int>(30u));

	// ⛔ THE LOCAL IS NOT ITS OWN NEIGHBOUR: at distance zero it would win every time.
	CHECK(nearestCharacterIdTo(candidates, 10u, std::nullopt) != std::optional<unsigned int>(10u));

	// Asked from a different local, the answer is that one's own neighbourhood: from 20's
	// position at (900, 0) the origin is 900 cm away and 30 is sqrt(900^2 + 300^2) = 948.7.
	CHECK(nearestCharacterIdTo(candidates, 20u, std::nullopt) == std::optional<unsigned int>(10u));

	CHECK(nearlyEqual(*nearestCharacterDistanceCm(candidates, 10u, 30u), 300.f));
	CHECK(nearlyEqual(nearestPlanarDistanceSquared(candidates.candidates[0],
		candidates.candidates[1]), 810000.f));
}

TEST_CASE("NearestStack.NearestHoldsThePreviousChoiceUntilARivalIsAClear50cmCloser",
          "[CharacterViz][InputHistoryViz]")
{
	// The held choice sits at 400 cm. A rival that closes to 360 is 40 cm nearer -- inside
	// the margin -- and must NOT take the stack; at 349 it is 51 cm nearer and must.
	const NearestCharacterCandidateList inside = listOf({
		NearestCharacterCandidate{ 10u,   0.f, 0.f },
		NearestCharacterCandidate{ 20u, 400.f, 0.f },
		NearestCharacterCandidate{ 30u, 360.f, 0.f },
	});

	CHECK(nearestCharacterIdTo(inside, 10u, std::optional<unsigned int>(20u))
		== std::optional<unsigned int>(20u));

	// ⭐ WITHOUT A PREVIOUS CHOICE THE SAME POSITIONS GIVE THE NEARER ONE. The hysteresis
	// is what is being tested, so the fixture must be one where it CHANGES the answer.
	CHECK(nearestCharacterIdTo(inside, 10u, std::nullopt) == std::optional<unsigned int>(30u));

	const NearestCharacterCandidateList outside = listOf({
		NearestCharacterCandidate{ 10u,   0.f, 0.f },
		NearestCharacterCandidate{ 20u, 400.f, 0.f },
		NearestCharacterCandidate{ 30u, 349.f, 0.f },
	});

	CHECK(nearestCharacterIdTo(outside, 10u, std::optional<unsigned int>(20u))
		== std::optional<unsigned int>(30u));

	CHECK(nearlyEqual(kNearestHysteresisCm, 50.f));

	// A previous choice that has left the world is not held: it is no longer a candidate,
	// so the margin has nothing to protect.
	const NearestCharacterCandidateList departed = listOf({
		NearestCharacterCandidate{ 10u,   0.f, 0.f },
		NearestCharacterCandidate{ 30u, 900.f, 0.f },
	});
	CHECK(nearestCharacterIdTo(departed, 10u, std::optional<unsigned int>(20u))
		== std::optional<unsigned int>(30u));
}

TEST_CASE("NearestStack.OneCharacterHasNoNearestAndNeitherDoesAnUnknownLocal",
          "[CharacterViz][InputHistoryViz]")
{
	const NearestCharacterCandidateList alone =
		listOf({ NearestCharacterCandidate{ 10u, 0.f, 0.f } });

	CHECK_FALSE(nearestCharacterIdTo(alone, 10u, std::nullopt).has_value());

	// Held choices do not survive being the only character either -- there is nobody for
	// the margin to hold against.
	CHECK_FALSE(nearestCharacterIdTo(alone, 10u, std::optional<unsigned int>(20u)).has_value());

	// An empty world, and a local this list has never heard of: both answer nothing
	// rather than picking somebody at an unknown distance.
	CHECK_FALSE(nearestCharacterIdTo(NearestCharacterCandidateList{}, 10u, std::nullopt)
		.has_value());
	CHECK_FALSE(nearestCharacterIdTo(alone, 99u, std::nullopt).has_value());
	CHECK_FALSE(nearestCharacterDistanceCm(alone, 10u, 99u).has_value());
}

TEST_CASE("NearestStack.EqualDistancesResolveByIdSoTheGatherOrderCannotChangeTheAnswer",
          "[CharacterViz][InputHistoryViz]")
{
	// ⛔ THE ORDER IS TOTAL. Two rivals at the same range must give one answer whichever
	//   order the storage happened to walk them in, or the stack flickers between them.
	const NearestCharacterCandidateList ascending = listOf({
		NearestCharacterCandidate{ 10u,    0.f, 0.f },
		NearestCharacterCandidate{ 20u,  500.f, 0.f },
		NearestCharacterCandidate{ 30u, -500.f, 0.f },
	});

	const NearestCharacterCandidateList descending = listOf({
		NearestCharacterCandidate{ 10u,    0.f, 0.f },
		NearestCharacterCandidate{ 30u, -500.f, 0.f },
		NearestCharacterCandidate{ 20u,  500.f, 0.f },
	});

	CHECK(nearestCharacterIdTo(ascending, 10u, std::nullopt)
		== nearestCharacterIdTo(descending, 10u, std::nullopt));
	CHECK(nearestCharacterIdTo(ascending, 10u, std::nullopt)
		== std::optional<unsigned int>(20u));
}

TEST_CASE("NearestStack.TheCandidateListRefusesRatherThanOverwritesWhenItIsFull",
          "[CharacterViz][InputHistoryViz]")
{
	NearestCharacterCandidateList candidates;
	for (unsigned int index = 0u; index < kNearestCandidateCapacity + 4u; ++index)
		candidates.add(NearestCharacterCandidate{ index, static_cast<float>(index) * 10.f, 0.f });

	CHECK(candidates.count == kNearestCandidateCapacity);

	// ⛔ THE FIRST ONES ADDED SURVIVE: overwriting would drop whichever brawler the wrap
	//   happened to land on, which is a different bug on every frame.
	CHECK(candidates.candidates[0].id == 0u);
	CHECK(candidates.candidates[kNearestCandidateCapacity - 1u].id
		== static_cast<unsigned int>(kNearestCandidateCapacity) - 1u);
}

// ===========================================================================
// 4. THE REMOTE DELAY BAR -- the four verdicts, from the relayed-read ring.
// ===========================================================================

TEST_CASE("NearestStack.TheRingIsExactlyOneDisplayWindowWide",
          "[CharacterViz][InputHistoryViz]")
{
	// The resolver may not name the display's constant and the display may not name the
	// resolver's, so the two are equal only by intent -- which is what this pins.
	// ⛔ A RING SHORTER THAN THE WINDOW silently blanks the oldest cells of the bar.
	CHECK(kRelayedReadObservationCapacityTicks == static_cast<std::size_t>(kTickLaneCapacity));
	CHECK(RelayedReadObservationRing::size() == kRelayedReadObservationCapacityTicks);
}

TEST_CASE("NearestStack.ARemoteAgreesWhenThisClientPredictedTheCaptureTheServerApplied",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	// Steady state: the relay's schedule stamp and the server's own hold are both 3, so
	// each capture is applied at the same tick on both ends.
	for (uint32_t simTick = 100u; simTick < 106u; ++simTick)
	{
		reader.nameCapture(simTick, simTick - 3u);
		observations.served(simTick, simTick - 3u);
	}

	InputHistoryTickLanes lanes;
	for (uint32_t simTick = 100u; simTick < 106u; ++simTick)
		pollRemoteTick(simTick, reader, observations, lanes);

	std::size_t agreed = 0u;
	for (uint32_t captureTick = 97u; captureTick < 103u; ++captureTick)
	{
		if (verdictAtCaptureTick(lanes, captureTick) == InputDelayVerdict::Agree)
			++agreed;
	}

	CHECK(agreed == 6u);
}

TEST_CASE("NearestStack.APressTheClientNeverAppliedLeavesItsOwnCellUnverifiedAndNotAgreed",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	// The authority applied capture 97 at tick 100 -- a hold of 3. This client never
	// received 97 in time and predicted tick 100 on the older capture 90 instead, so its
	// own hold on capture 90 reads 10.
	//
	// Cell 97 has an authority answer and no client one; cell 90 the reverse, plus
	// whatever the authority said about 90 earlier.
	// ⛔ THE TWO HALVES LAND ON DIFFERENT CELLS, and that is the mechanism.
	reader.nameCapture(100u, 97u);
	reader.nameCapture(99u, 90u);
	observations.served(100u, 90u);
	observations.served(99u, 90u);

	InputHistoryTickLanes lanes;
	pollRemoteTick(99u, reader, observations, lanes);
	pollRemoteTick(100u, reader, observations, lanes);

	// Cell 97: the server named a capture this client never applied, so there is a lag
	// and nothing to compare it against.
	CHECK(verdictAtCaptureTick(lanes, 97u) == InputDelayVerdict::LagUnverified);

	// Cell 90: the server applied it at 99 (a hold of 9) and this client FIRST applied it
	// at 99 too -- the earliest of the two ticks it was served on, not the latest.
	// ⭐ FIRST APPLICATION WINS, which is what makes the two halves comparable at all.
	const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(90u);
	REQUIRE(laneTick.has_value());
	const InputDelayCell* cell = lanes.delay().find(*laneTick);
	REQUIRE(cell != nullptr);
	REQUIRE(cell->clientDelayTicks.has_value());
	CHECK(*cell->clientDelayTicks == 9);
	CHECK(verdictAtCaptureTick(lanes, 90u) == InputDelayVerdict::Agree);
}

TEST_CASE("NearestStack.AServerThatHeldACaptureLongerThanThisClientDidReadsAsServerLater",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	// This client applied capture 95 at tick 98 -- a hold of 3 -- while the authority did
	// not get to it until tick 103, a hold of 8. ⛔ THE SERVER IS LATER ON THIS CAPTURE,
	//   which is the one direction a queue delay alone can produce.
	reader.nameCapture(103u, 95u);
	observations.served(98u, 95u);

	InputHistoryTickLanes lanes;
	pollRemoteTick(103u, reader, observations, lanes);

	CHECK(verdictAtCaptureTick(lanes, 95u) == InputDelayVerdict::ServerLater);
}

TEST_CASE("NearestStack.AClientThatResolvedNeutralLeavesTheHalfAbsentRatherThanClaimingZero",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	// Nothing had ever arrived for this proxy, so the read served the injected zero and
	// stands behind no capture at all.
	reader.nameCapture(100u, 97u);
	observations.servedNeutral(100u);

	InputHistoryTickLanes lanes;
	pollRemoteTick(100u, reader, observations, lanes);

	// ⛔ ABSENT, NOT ZERO. A client half of 0 would read as "this client applied the
	//   capture the instant it was made", which is a claim, and a false one.
	CHECK(verdictAtCaptureTick(lanes, 97u) == InputDelayVerdict::LagUnverified);
}

TEST_CASE("NearestStack.AServerSubstitutionStillReadsAsNoCaptureNamedOnARemote",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	// The authority underran and substituted its neutral at tick 100, while this client
	// predicted that tick on capture 97.
	reader.nameSentinel(100u);
	observations.served(100u, 97u);

	InputHistoryTickLanes lanes;
	pollRemoteTick(100u, reader, observations, lanes);

	// The sentinel arm speaks for the tick it was asked about, so the cell is tick 100's.
	CHECK(verdictAtCaptureTick(lanes, 100u) == InputDelayVerdict::NoCaptureNamed);
}

TEST_CASE("NearestStack.AReplayedTickOverwritesWhatThePredictionPassRecordedForIt",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	// The prediction pass ran tick 100 on the fallback 90; the replay then re-ran it on
	// the authority's own ref, 97. The RING is last-write-wins per sim tick, so by the
	// time the display reads it only the replay's answer is there for tick 100.
	reader.nameCapture(100u, 97u);
	observations.served(100u, 97u);

	InputHistoryTickLanes lanes;
	pollRemoteTick(100u, reader, observations, lanes);

	// ⭐ AFTER A REPLAY THE CELL AGREES BY CONSTRUCTION when the ref was resident, which
	// is exactly why a NON-agreement that survives a replay is a relay-store miss.
	CHECK(verdictAtCaptureTick(lanes, 97u) == InputDelayVerdict::Agree);
}

TEST_CASE("NearestStack.TheRemoteClientHalfOnlyEverFallsAndTheReadIsIdempotent",
          "[CharacterViz][InputHistoryViz]")
{
	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	reader.nameCapture(100u, 90u);
	observations.served(108u, 90u);
	observations.served(104u, 90u);
	observations.served(100u, 90u);

	InputHistoryTickLanes lanes;
	pollRemoteTick(100u, reader, observations, lanes);

	const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(90u);
	REQUIRE(laneTick.has_value());

	const InputDelayCell* first = lanes.delay().find(*laneTick);
	REQUIRE(first != nullptr);
	REQUIRE(first->clientDelayTicks.has_value());

	// The ring is addressed by sim tick and therefore walked out of order; taking the last
	// one seen would make the reading depend on where the ring happened to wrap.
	// ⛔ THE MINIMUM, NEVER THE LAST ONE WALKED.
	CHECK(*first->clientDelayTicks == 10);

	// Re-presenting the same observations changes nothing -- the same idempotence the
	// capture sweep rests on.
	const InputDelayCell before = *first;
	pollRemoteTick(101u, reader, observations, lanes);
	pollRemoteTick(102u, reader, observations, lanes);

	const InputDelayCell* after = lanes.delay().find(*laneTick);
	REQUIRE(after != nullptr);
	CHECK(*after == before);
}

// ===========================================================================
// 5. THE REMOTE PROVENANCE BAR -- the investigation's own signature.
// ===========================================================================

TEST_CASE("NearestStack.ARemoteProxysAdoptionRunDrawsOneCellPerTickInTheAdoptionColour",
          "[CharacterViz][InputHistoryViz]")
{
	using brawlerInputHistoryVisualization::FrameMeterBarCells;
	using brawlerInputHistoryVisualization::LaneRunList;
	using brawlerInputHistoryVisualization::RowProvenanceSummary;
	using brawlerInputHistoryVisualization::collectLaneRuns;
	using brawlerInputHistoryVisualization::provenanceCellStyleOf;
	using brawlerInputHistoryVisualization::readProvenanceBar;
	using brawlerInputHistoryVisualization::retainedLaneWindow;

	// A swing the proxy entered by ADOPTION: for twelve consecutive ticks the authority
	// disagreed with this client's prediction and its state was copied in. That run is
	// what the remote weapon-swing investigation predicted and could not see, because
	// nothing polled the lanes for a character this client does not control.
	const uint32_t swingTicks = 12u;

	ScriptedAuthorityReader reader;
	ScriptedObservations    observations;

	for (uint32_t simTick = 200u; simTick < 200u + swingTicks; ++simTick)
	{
		reader.nameCapture(simTick, simTick - 2u, SlotStateProvenance::AuthorityAdopted);
		observations.served(simTick, simTick - 2u);
	}

	InputHistoryTickLanes lanes;
	for (uint32_t simTick = 200u; simTick < 200u + swingTicks; ++simTick)
		pollRemoteTick(simTick, reader, observations, lanes);

	const PollWindow window = retainedLaneWindow(lanes, 120u);

	FrameMeterBarCells bar;
	readProvenanceBar(lanes, window, bar);

	LaneRunList runs;
	collectLaneRuns(bar, runs);

	// ⭐ THE RUN LENGTH IS THE SWING LENGTH. One cell per tick is the whole design of the
	// meter, so a twelve-tick adoption reads as a twelve-cell run carrying its own number.
	std::size_t adoptionRuns = 0u;
	uint32_t    longestRun   = 0u;
	for (uint32_t index = 0u; index < runs.count; ++index)
	{
		if (runs.runs[index].value != static_cast<uint8_t>(RowProvenanceSummary::Corrected))
			continue;

		++adoptionRuns;
		if (runs.runs[index].length > longestRun)
			longestRun = runs.runs[index].length;
	}

	CHECK(adoptionRuns == 1u);
	CHECK(longestRun == swingTicks);
}

TEST_CASE("NearestStack.TheAdoptionSummaryAndItsColourAreTheOnesTheSeamActuallyNames",
          "[CharacterViz][InputHistoryViz]")
{
	using brawlerInputHistoryVisualization::LaneCellStyle;
	using brawlerInputHistoryVisualization::RowProvenanceSummary;
	using brawlerInputHistoryVisualization::provenanceCellStyleOf;
	using brawlerInputHistoryVisualization::summaryOfSlotProvenance;

	// The seam's enumerator is `AuthorityAdopted`; the DISPLAY's summary for it is
	// `Corrected`, and the colour that summary draws is ORANGE. There is no
	// `AuthorityAdopted` cell value, and no red one: the only red-adjacent entry in this
	// palette is the magenta reserved for a provenance lie, which must never appear.
	// ⚠ THE VOCABULARY, PINNED BECAUSE IT IS EASY TO MISQUOTE.
	CHECK(summaryOfSlotProvenance(SlotStateProvenance::AuthorityAdopted)
		== RowProvenanceSummary::Corrected);

	const LaneCellStyle adopted = provenanceCellStyleOf(RowProvenanceSummary::Corrected);
	CHECK(nearlyEqual(adopted.color.r, 1.00f));
	CHECK(nearlyEqual(adopted.color.g, 0.55f));
	CHECK(nearlyEqual(adopted.color.b, 0.05f));

	// And the one it must never be confused with: agreement is green, so a bar that has
	// gone the adoption colour cannot be read as a bar that is agreeing.
	const LaneCellStyle confirmed = provenanceCellStyleOf(RowProvenanceSummary::Confirmed);
	CHECK(nearlyEqual(confirmed.color.r, 0.15f));
	CHECK(nearlyEqual(confirmed.color.g, 0.80f));
	CHECK(nearlyEqual(confirmed.color.b, 0.30f));
}

// ===========================================================================
// 6. THE ROW PANEL'S ABSENT TWIN.
// ===========================================================================

TEST_CASE("NearestStack.TheAbsentRemotePanelIsMarkedBesideTheRealOneAndNeverUnderIt",
          "[CharacterViz][InputHistoryViz]")
{
	using brawlerInputHistoryVisualization::PanelLayout;
	using brawlerInputHistoryVisualization::kPanelPlaceholderGap;
	using brawlerInputHistoryVisualization::panelPlaceholderTopY;
	using brawlerInputHistoryVisualization::panelPlaceholderX;
	using brawlerInputHistoryVisualization::panelRowTopY;
	using brawlerInputHistoryVisualization::placedPanelLayout;

	// ⛔ BESIDE, NOT BELOW: a second panel under this one does not fit at 720p at all.
	for (float viewportHeight : { 1080.f, 720.f })
	{
		const PanelLayout layout =
			placedPanelLayout(PanelLayout{}, 1.f, 24u, viewportHeight);

		CHECK(nearlyEqual(panelPlaceholderX(layout),
			layout.originX + layout.rowWidth + kPanelPlaceholderGap));
		CHECK(panelPlaceholderX(layout) > layout.originX + layout.rowWidth);

		// On the panel's own top row, so the two read as one display.
		CHECK(nearlyEqual(panelPlaceholderTopY(layout),
			panelRowTopY(layout, 0u) + layout.textOffsetY));
		CHECK(panelPlaceholderTopY(layout) >= 0.f);
	}

	CHECK(nearlyEqual(kPanelPlaceholderGap, 8.f));
}

TEST_CASE("NearestStack.ALocalPollAndARemotePollCannotBothSupplyOneCellsClientHalf",
          "[CharacterViz][InputHistoryViz]")
{
	// The two sources are selected by TYPE, so this is a compile-time property rather
	// than a runtime one; what a case can still pin is that the null source is a real
	// type, that it carries no observations, and that the poll that takes it is the same
	// function the local path has always called.
	CHECK(std::is_empty_v<brawlerInputHistoryVisualization::NoRemoteDelayObservations>);
	CHECK_FALSE(std::is_same_v<brawlerInputHistoryVisualization::NoRemoteDelayObservations,
		ScriptedObservations>);
}

} // namespace inputhistorynexteststack

#endif // WITH_LOW_LEVEL_TESTS
