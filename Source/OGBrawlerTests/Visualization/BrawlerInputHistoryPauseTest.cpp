// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the IDLE GATE -- the one predicate that stops both frame-meter lanes while nothing
// is happening, the sim-tick to lane-tick mapping it owns, and the marker a collapsed span
// leaves behind.
//
// WHAT THIS SUITE IS REALLY GUARDING is that the two bars stay one clock. Their whole
// diagnostic value is that a vertical slice is ONE capture tick in both, and a pause is the
// first thing in this display able to break that: two lanes each deciding for themselves
// when to stop would drift apart by a tick and go on looking perfectly aligned. So the
// decision is made once, and the cases below construct a sequence where two independent
// evaluations demonstrably land in different places.
//
// The second thing it guards is that eliding never invents adjacency. A bar that removed
// three hundred idle ticks and drew the runs either side of them as neighbours would be
// telling a reader something false, so a span collapses to a MARKED cell carrying the count
// of what it removed -- and the run lengths either side stay their own.
//
// The lanes' storage and the bars' geometry are NOT re-tested here: BrawlerInputHistoryLane
// and BrawlerInputHistoryFrameMeter own those. What is tested is what the gate decides.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistorypausetests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::FrameMeterBarCells;
using brawlerInputHistoryVisualization::FrameMeterAxisEventList;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneAdmission;
using brawlerInputHistoryVisualization::LaneAxisEvent;
using brawlerInputHistoryVisualization::LaneAxisEventKind;
using brawlerInputHistoryVisualization::LaneCellColor;
using brawlerInputHistoryVisualization::LaneCellStyle;
using brawlerInputHistoryVisualization::LaneIdleGate;
using brawlerInputHistoryVisualization::LaneRunList;
using brawlerInputHistoryVisualization::MachineStateCell;
using brawlerInputHistoryVisualization::PollWindow;
using brawlerInputHistoryVisualization::RowProvenanceSummary;
using brawlerInputHistoryVisualization::TickLanePollCounts;

using brawlerInputHistoryVisualization::collectFrameMeterAxisEvents;
using brawlerInputHistoryVisualization::collectLaneRuns;
using brawlerInputHistoryVisualization::laneColorGap;
using brawlerInputHistoryVisualization::laneTickIsInactive;
using brawlerInputHistoryVisualization::machineCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::provenanceCellStyleOfOrdinal;
using brawlerInputHistoryVisualization::readMachineStateBar;
using brawlerInputHistoryVisualization::readProvenanceBar;
using brawlerInputHistoryVisualization::retainedLaneWindow;

using brawlerInputHistoryVisualization::kDirectionBucketCount;
using brawlerInputHistoryVisualization::kLaneElisionColor;
using brawlerInputHistoryVisualization::kLanePaletteMinCrossGap;
using brawlerInputHistoryVisualization::kLanePauseEngageTicks;
using brawlerInputHistoryVisualization::kMachineStateCellCount;
using brawlerInputHistoryVisualization::kRowProvenanceSummaryCount;

// ---------------------------------------------------------------------------
// Helpers. The reader answers for every tick it is asked about, so the provenance lane
// fills wherever the gate lets it -- which is what makes an elision visible as absence.
//
// ⭐ DEFAULTS TO A RESIDENT NoRef, NOT NoSlot: classifyNoSlot's Unclassifiable cause files
// nothing for a window with no resident tick at all, and this reader's whole point is that
// every asked tick DOES answer. A NoSlot default would make it silent instead, which is
// what "answers nothing" already means for AuthorityTest's SilentReader.
// ---------------------------------------------------------------------------

class EveryTickReader
{
public:
	void setRef(uint32_t simTick, AppliedCaptureRef ref) { m_refs[simTick] = ref; }
	void setHasCorrectionCache(bool hasCache) { m_hasCorrectionCache = hasCache; }

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		const auto it = m_refs.find(simTick);
		return (it == m_refs.end())
		           ? AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick }
		           : it->second;
	}

	std::optional<SlotStateProvenance> slotProvenance(uint32_t) const { return std::nullopt; }

	bool hasCorrectionCache() const { return m_hasCorrectionCache; }

private:
	std::map<uint32_t, AppliedCaptureRef> m_refs;
	bool                                  m_hasCorrectionCache = true;
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

// One poll, with the scratch inversion the production store also keeps on its stack.
TickLanePollCounts poll(const EveryTickReader&          reader,
                        uint32_t                        simTick,
                        DAttackState                    machineState,
                        std::optional<CaptureRowFields> liveInput,
                        bool                            pauseWhileIdle,
                        InputHistoryTickLanes&          lanes)
{
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(
		reader, simTick, machineState, liveInput, pauseWhileIdle, std::nullopt, std::nullopt,
		std::nullopt, inversion, lanes);
}

// ---------------------------------------------------------------------------
// THE PREDICATE
// ---------------------------------------------------------------------------

TEST_CASE("Pause.InactiveMeansTheDisplaysOwnNeutralAndNothingElse",
          "[CharacterViz][InputHistoryViz]")
{
	// The whole point of reusing the panel's bucket and the existing button mask: the two
	// displays cannot end up with two ideas of what "no input" is.
	CHECK(laneTickIsInactive(DirectionBucket::Neutral, 0u, DAttackState::Idle));

	// Each conjunct falsified alone. A stick off centre, either attack button, or a machine
	// doing anything at all is a tick worth a cell.
	CHECK_FALSE(laneTickIsInactive(DirectionBucket::Forward, 0u, DAttackState::Idle));
	CHECK_FALSE(laneTickIsInactive(DirectionBucket::Neutral, 1u, DAttackState::Idle));
	CHECK_FALSE(laneTickIsInactive(DirectionBucket::Neutral, 2u, DAttackState::Idle));
	CHECK_FALSE(laneTickIsInactive(DirectionBucket::Neutral, 0u, DAttackState::Attacking));

	// Exhaustive rather than spot-checked: over every bucket, every mask and every state
	// exactly ONE combination qualifies. A predicate that widened anywhere moves this.
	std::size_t inactiveCombinations = 0u;
	for (unsigned bucket = 0u; bucket < static_cast<unsigned>(kDirectionBucketCount); ++bucket)
	{
		for (uint8_t mask = 0u; mask < 4u; ++mask)
		{
			for (unsigned state = 0u; state < static_cast<unsigned>(kDAttackStateCount); ++state)
			{
				if (laneTickIsInactive(static_cast<DirectionBucket>(bucket), mask,
					    static_cast<DAttackState>(state)))
				{
					++inactiveCombinations;
				}
			}
		}
	}

	CHECK(inactiveCombinations == 1u);
}

// ---------------------------------------------------------------------------
// HYSTERESIS
// ---------------------------------------------------------------------------

TEST_CASE("Pause.EngagingWaitsForTheHysteresisRunAndResumingDoesNot",
          "[CharacterViz][InputHistoryViz]")
{
	// Slow in, instant out. A stick resting on the deadzone chatters for a tick or three,
	// and a pause that engaged on that would stripe the bar with tiny markers; a pause that
	// were slow to RESUME would miss the first frames of the thing worth looking at.
	LaneIdleGate gate;

	std::size_t recordedBeforeTheTrigger = 0u;
	for (uint32_t tick = 0u; tick < kLanePauseEngageTicks; ++tick)
	{
		if (gate.admit(tick, true, true) == LaneAdmission::Recorded)
			++recordedBeforeTheTrigger;
	}

	CHECK(recordedBeforeTheTrigger == kLanePauseEngageTicks);
	CHECK_FALSE(gate.paused());

	// The tick after the run is the first one elided.
	CHECK(gate.admit(kLanePauseEngageTicks, true, true) == LaneAdmission::Elided);
	CHECK(gate.paused());

	std::size_t elidedWhileIdle = 0u;
	for (uint32_t tick = kLanePauseEngageTicks + 1u; tick < 200u; ++tick)
	{
		if (gate.admit(tick, true, true) == LaneAdmission::Elided)
			++elidedWhileIdle;
	}

	CHECK(elidedWhileIdle == 200u - kLanePauseEngageTicks - 1u);

	// ⛔ THE VERY FIRST ACTIVE TICK IS RECORDED, with no run of its own to serve first.
	CHECK(gate.admit(200u, false, true) == LaneAdmission::Recorded);
	CHECK_FALSE(gate.paused());

	// And the counter really did reset, so the next pause pays the full run again.
	CHECK(gate.admit(201u, true, true) == LaneAdmission::Recorded);
	CHECK(gate.consecutiveInactiveTicks() == 1u);
}

TEST_CASE("Pause.ARePresentedTickIsOneDecisionAndNotTwo",
          "[CharacterViz][InputHistoryViz]")
{
	// The poll runs at render rate, so the same simulation tick arrives again on every
	// frame drawn without the simulation advancing. Counting those would engage the pause
	// after a fraction of the ticks the hysteresis names, on a fast machine only.
	LaneIdleGate gate;

	CHECK(gate.admit(500u, true, true) == LaneAdmission::Recorded);

	std::size_t repeats = 0u;
	for (int frame = 0; frame < 40; ++frame)
	{
		if (gate.admit(500u, true, true) == LaneAdmission::Recorded)
			++repeats;
	}

	CHECK(repeats == 40u);
	CHECK(gate.consecutiveInactiveTicks() == 1u);
	CHECK_FALSE(gate.paused());
}

// ---------------------------------------------------------------------------
// THE SHARED DECISION -- the case this whole suite exists for
// ---------------------------------------------------------------------------

TEST_CASE("Pause.OneGateDecidesForBothLanesSoAColumnCannotMeanTwoTicks",
          "[CharacterViz][InputHistoryViz]")
{
	// THE SEQUENCE IS BUILT TO SEPARATE THE CONJUNCTS. Thirty ticks of a neutral stick
	// while the machine attacks, thirty of a moving stick while it is idle, then forty of
	// both at once. Only the third block satisfies the real predicate, so a lane pausing on
	// either half ALONE would stop somewhere the other lane kept going.
	const uint32_t kFirstTick     = 100u;
	const uint32_t kAttackingEnd  = 130u;   // [100, 130): neutral stick, machine Attacking
	const uint32_t kMovingEnd     = 160u;   // [130, 160): moving stick, machine Idle
	const uint32_t kLastTick      = 200u;   // [160, 200): both -- the only inactive block

	EveryTickReader       reader;
	InputHistoryTickLanes lanes;

	// The two rival readings, run on the SAME gate class so the comparison is real code
	// against real code rather than against a hand-computed expectation.
	LaneIdleGate inputOnly;
	LaneIdleGate machineOnly;

	uint32_t firstElidedShared     = 0u;
	uint32_t firstElidedInputOnly  = 0u;
	uint32_t firstElidedMachineOnly = 0u;

	for (uint32_t tick = kFirstTick; tick < kLastTick; ++tick)
	{
		const bool         stickNeutral = (tick < kAttackingEnd) || (tick >= kMovingEnd);
		const DAttackState machineState =
			(tick < kAttackingEnd) ? DAttackState::Attacking : DAttackState::Idle;

		const CaptureRowFields input = stickNeutral ? neutralInput() : movingInput();

		if (poll(reader, tick, machineState, input, true, lanes).admission
			== LaneAdmission::Elided && firstElidedShared == 0u)
		{
			firstElidedShared = tick;
		}

		if (inputOnly.admit(tick, stickNeutral, true) == LaneAdmission::Elided
			&& firstElidedInputOnly == 0u)
		{
			firstElidedInputOnly = tick;
		}

		if (machineOnly.admit(tick, machineState == DAttackState::Idle, true)
			== LaneAdmission::Elided && firstElidedMachineOnly == 0u)
		{
			firstElidedMachineOnly = tick;
		}
	}

	// ⭐ THE SEQUENCE DISCRIMINATES. All three stop in different places, so a test that
	// passes under the shared reading could not also pass under either half alone.
	CHECK(firstElidedShared == kMovingEnd + kLanePauseEngageTicks);
	CHECK(firstElidedInputOnly == kFirstTick + kLanePauseEngageTicks);
	CHECK(firstElidedMachineOnly == kAttackingEnd + kLanePauseEngageTicks);

	// And the divergence is not merely in WHEN they stopped: the two rival gates place the
	// very last tick at different lane ticks, which is a column meaning two different
	// capture ticks in the two bars -- the failure this display cannot survive.
	const std::optional<uint32_t> inputOnlyLast   = inputOnly.laneTickOf(kMovingEnd);
	const std::optional<uint32_t> machineOnlyLast = machineOnly.laneTickOf(kMovingEnd);
	REQUIRE(inputOnlyLast.has_value());
	CHECK_FALSE(machineOnlyLast.has_value());

	// ⛔ THE REAL RUN HAS ONE MAPPING. Every recorded tick carries a cell in BOTH lanes at
	// the SAME lane tick, and every elided tick carries one in neither.
	std::size_t recordedInBoth   = 0u;
	std::size_t elidedInNeither  = 0u;
	for (uint32_t tick = kFirstTick; tick < kLastTick; ++tick)
	{
		const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(tick);

		if (!laneTick.has_value())
		{
			++elidedInNeither;
			continue;
		}

		if (lanes.provenanceAt(*laneTick) != nullptr
			&& lanes.machineCellAt(*laneTick) != MachineStateCell::NotSampled)
		{
			++recordedInBoth;
		}
	}

	CHECK(recordedInBoth == firstElidedShared - kFirstTick);
	CHECK(elidedInNeither == kLastTick - firstElidedShared);
}

// ---------------------------------------------------------------------------
// THE MARKER
// ---------------------------------------------------------------------------

TEST_CASE("Pause.ACollapsedSpanIsMarkedAndCarriesTheTicksItRemoved",
          "[CharacterViz][InputHistoryViz]")
{
	// Without the count a reader would see two runs side by side and have no way to know
	// whether four ticks or four hundred sat between them. The gap in the bar is one cell
	// wide either way, so the number cannot be read off the picture -- it has to be carried.
	EveryTickReader       reader;
	InputHistoryTickLanes lanes;

	const uint32_t kIdleFrom = 20u;
	const uint32_t kResumeAt = 400u;

	for (uint32_t tick = 0u; tick < kIdleFrom; ++tick)
		poll(reader, tick, DAttackState::Attacking, movingInput(), true, lanes);

	for (uint32_t tick = kIdleFrom; tick < kResumeAt; ++tick)
		poll(reader, tick, DAttackState::Idle, neutralInput(), true, lanes);

	poll(reader, kResumeAt, DAttackState::Attacking, movingInput(), true, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);

	const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);

	// The span begins where the hysteresis ran out, not where the stick went neutral.
	CHECK(span.simTick == kIdleFrom + kLanePauseEngageTicks);
	CHECK(span.skippedTicks == kResumeAt - span.simTick);

	// The resumed tick lands immediately after it, so the marker sits BETWEEN the runs.
	// ⛔ A COLLAPSED SPAN COSTS ONE LANE TICK, whatever it swallowed.
	const uint32_t lastRecordedLaneTick = *lanes.gate().laneTickOf(span.simTick - 1u);
	CHECK(span.laneTick == lastRecordedLaneTick + 1u);
	CHECK(*lanes.gate().laneTickOf(kResumeAt) == span.laneTick + 1u);

	// A cell in neither lane, so nothing is claimed about the ticks that went.
	CHECK(lanes.provenanceAt(span.laneTick) == nullptr);
	CHECK(lanes.machineCellAt(span.laneTick) == MachineStateCell::NotSampled);

	// And the renderer is handed the same count, at the offset it must draw it on.
	const PollWindow        window = retainedLaneWindow(lanes, 120u);
	FrameMeterAxisEventList events;
	collectFrameMeterAxisEvents(lanes, window, events);

	REQUIRE(events.count == 1u);
	CHECK(events.marks[0].kind == LaneAxisEventKind::Elision);
	CHECK(events.marks[0].skippedTicks == span.skippedTicks);
	CHECK(events.marks[0].offset == span.laneTick - window.oldestTick);
}

TEST_CASE("Pause.RunLengthsEitherSideOfAnElisionAreStillTheirOwn",
          "[CharacterViz][InputHistoryViz]")
{
	// The marker cell is a hole, and a hole ends the run it interrupts and starts none of
	// its own. So the numbers on the bar keep meaning "this many consecutive ticks" rather
	// than quietly absorbing the ones that were removed.
	EveryTickReader       reader;
	InputHistoryTickLanes lanes;

	const uint32_t kFirstRun = 12u;
	const uint32_t kIdleFrom = kFirstRun;
	const uint32_t kResumeAt = 300u;
	const uint32_t kLastTick = kResumeAt + 9u;

	for (uint32_t tick = 0u; tick < kFirstRun; ++tick)
		poll(reader, tick, DAttackState::Attacking, movingInput(), true, lanes);

	for (uint32_t tick = kIdleFrom; tick < kResumeAt; ++tick)
		poll(reader, tick, DAttackState::Idle, neutralInput(), true, lanes);

	for (uint32_t tick = kResumeAt; tick <= kLastTick; ++tick)
		poll(reader, tick, DAttackState::HitFlinch, movingInput(), true, lanes);

	const PollWindow   window = retainedLaneWindow(lanes, 120u);
	FrameMeterBarCells bar;
	readMachineStateBar(lanes, window, bar);

	LaneRunList runs;
	collectLaneRuns(bar, runs);

	// Attacking, then the recorded head of the idle run, then HitFlinch. The hysteresis
	// ticks before the pause engaged are real observations and keep their own run.
	REQUIRE(runs.count == 3u);
	CHECK(runs.runs[0].value == static_cast<uint8_t>(MachineStateCell::Attacking));
	CHECK(runs.runs[0].length == kFirstRun);
	CHECK(runs.runs[1].value == static_cast<uint8_t>(MachineStateCell::Idle));
	CHECK(runs.runs[1].length == kLanePauseEngageTicks);
	CHECK(runs.runs[2].value == static_cast<uint8_t>(MachineStateCell::HitFlinch));
	CHECK(runs.runs[2].length == kLastTick - kResumeAt + 1u);

	// ⛔ AND NO RUN SPANS THE MARKER: the two neighbours are one cell apart, not adjacent.
	CHECK(runs.runs[2].firstOffset == runs.runs[1].lastOffset + 2u);
}

// ---------------------------------------------------------------------------
// THE ESCAPE HATCH, AND WHAT THE PAUSE BUYS
// ---------------------------------------------------------------------------

TEST_CASE("Pause.TurningItOffRestoresFullFidelityRecording",
          "[CharacterViz][InputHistoryViz]")
{
	// This is the documented answer to eliding a rollback that happens while idle: the same
	// idle sequence, recorded whole, with the lane axis back to being the simulation's own.
	EveryTickReader       reader;
	InputHistoryTickLanes lanes;

	const uint32_t kTicks = 200u;

	std::size_t elided = 0u;
	for (uint32_t tick = 0u; tick < kTicks; ++tick)
	{
		if (poll(reader, tick, DAttackState::Idle, neutralInput(), false, lanes).admission
			== LaneAdmission::Elided)
		{
			++elided;
		}
	}

	CHECK(elided == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
	CHECK_FALSE(lanes.gate().paused());

	// Nothing elided means nothing remapped: a lane tick is still the capture tick.
	std::size_t identityMapped = 0u;
	for (uint32_t tick = 0u; tick < kTicks; ++tick)
	{
		const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(tick);
		if (laneTick.has_value() && *laneTick == tick)
			++identityMapped;
	}

	CHECK(identityMapped == kTicks);
	REQUIRE(lanes.hasAxis());
	CHECK(lanes.newestAxisTick() == kTicks - 1u);
}

TEST_CASE("Pause.ALongIdleNoLongerEvictsTheActivityEitherSideOfIt",
          "[CharacterViz][InputHistoryViz]")
{
	// The request, stated as a test. A thousand idle ticks is four times the lane's whole
	// capacity, so without the pause they scroll every trace of the first burst out of the
	// window; with it they cost one cell and both bursts are still on screen together.
	const uint32_t kBurst    = 20u;
	const uint32_t kIdleFrom = kBurst;
	const uint32_t kResumeAt = kBurst + 1000u;
	const uint32_t kLastTick = kResumeAt + kBurst;

	EveryTickReader reader;

	InputHistoryTickLanes paused;
	InputHistoryTickLanes unpaused;

	for (uint32_t tick = 0u; tick < kBurst; ++tick)
	{
		poll(reader, tick, DAttackState::Attacking, movingInput(), true, paused);
		poll(reader, tick, DAttackState::Attacking, movingInput(), false, unpaused);
	}

	for (uint32_t tick = kIdleFrom; tick < kResumeAt; ++tick)
	{
		poll(reader, tick, DAttackState::Idle, neutralInput(), true, paused);
		poll(reader, tick, DAttackState::Idle, neutralInput(), false, unpaused);
	}

	for (uint32_t tick = kResumeAt; tick < kLastTick; ++tick)
	{
		poll(reader, tick, DAttackState::HitFlinch, movingInput(), true, paused);
		poll(reader, tick, DAttackState::HitFlinch, movingInput(), false, unpaused);
	}

	FrameMeterBarCells bar;
	LaneRunList        runs;

	readMachineStateBar(paused, retainedLaneWindow(paused, 120u), bar);
	collectLaneRuns(bar, runs);

	// Attacking, the recorded head of the idle run, HitFlinch -- the whole session in one
	// 120-tick window, which is exactly what the pause was asked for.
	CHECK(runs.count == 3u);
	CHECK(runs.runs[0].value == static_cast<uint8_t>(MachineStateCell::Attacking));
	CHECK(runs.runs[0].length == kBurst);

	readMachineStateBar(unpaused, retainedLaneWindow(unpaused, 120u), bar);
	collectLaneRuns(bar, runs);

	// ⛔ AND THE SAME SESSION WITHOUT IT: the first burst is a thousand ticks behind the
	// window, and all that is left is the wall of Idle the pause exists to remove.
	CHECK(runs.count == 2u);
	CHECK(runs.runs[0].value == static_cast<uint8_t>(MachineStateCell::Idle));
	CHECK(runs.runs[0].length == 100u);
}

// ---------------------------------------------------------------------------
// THE JOIN ACROSS AN ELISION
// ---------------------------------------------------------------------------

TEST_CASE("Pause.ALateCorrectionStillRepaintsExactlyOneCellAcrossAnElision",
          "[CharacterViz][InputHistoryViz]")
{
	// The property the per-tick lanes exist for must survive the remapping. A correction
	// landing on a tick recorded BEFORE the pause has to find that tick's own lane cell and
	// no other, and a capture inside the elided span has to find none at all.
	EveryTickReader       reader;
	InputHistoryTickLanes lanes;

	const uint32_t kActive   = 5u;
	const uint32_t kResumeAt = 30u;
	const uint32_t kMarked   = 3u;      // an active tick, still inside the resident window

	for (uint32_t tick = 0u; tick < kActive; ++tick)
		poll(reader, tick, DAttackState::Attacking, movingInput(), true, lanes);

	for (uint32_t tick = kActive; tick < kResumeAt; ++tick)
		poll(reader, tick, DAttackState::Idle, neutralInput(), true, lanes);

	poll(reader, kResumeAt, DAttackState::Attacking, movingInput(), true, lanes);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& span = lanes.gate().axisEventAt(0u);

	const uint32_t markedLaneTick = *lanes.gate().laneTickOf(kMarked);
	REQUIRE(lanes.provenanceAt(markedLaneTick) != nullptr);
	const RowProvenanceSummary before = *lanes.provenanceAt(markedLaneTick);

	// The authority now names that capture, which is the arm a real correction arrives on.
	reader.setRef(kMarked, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, kMarked });
	const TickLanePollCounts counts =
		poll(reader, kResumeAt + 1u, DAttackState::Attacking, movingInput(), true, lanes);

	REQUIRE(lanes.provenanceAt(markedLaneTick) != nullptr);
	CHECK(*lanes.provenanceAt(markedLaneTick) != before);
	CHECK(counts.provenanceCellsUpdated == 1u);

	// The sweep spans the whole session, so every skipped tick reached the gate.
	// ⛔ AND THE ELIDED TICKS ARE REFUSED, not mapped to a neighbour's cell.
	CHECK(counts.provenanceCellsElided == span.skippedTicks);

	std::size_t elidedTicksWithoutALaneTick = 0u;
	for (uint32_t tick = span.simTick; tick < kResumeAt; ++tick)
	{
		if (!lanes.gate().laneTickOf(tick).has_value())
			++elidedTicksWithoutALaneTick;
	}

	CHECK(elidedTicksWithoutALaneTick == span.skippedTicks);
}

// ---------------------------------------------------------------------------
// THE MARKER'S COLOUR
// ---------------------------------------------------------------------------

TEST_CASE("Pause.TheElisionMarkerBelongsToNeitherPalette",
          "[CharacterViz][InputHistoryViz]")
{
	// Missing time is not a state, and a marker that read as one would put a fifteenth
	// value into a fourteen-value display. Swept one PAST each count, so a palette that
	// stopped covering its own enum is caught by the same walk.
	float closestToProvenance = 3.f;
	for (unsigned ordinal = 0u; ordinal <= static_cast<unsigned>(kRowProvenanceSummaryCount);
	     ++ordinal)
	{
		const LaneCellStyle style = provenanceCellStyleOfOrdinal(static_cast<uint8_t>(ordinal));
		const float         gap   = laneColorGap(kLaneElisionColor, style.color);

		if (gap < closestToProvenance)
			closestToProvenance = gap;
	}

	float closestToMachine = 3.f;
	for (unsigned ordinal = 0u; ordinal <= static_cast<unsigned>(kMachineStateCellCount);
	     ++ordinal)
	{
		const LaneCellStyle style = machineCellStyleOfOrdinal(static_cast<uint8_t>(ordinal));

		// The hole carries no colour at all, so comparing against it would compare against
		// black and pass for free.
		if (style.fill == brawlerInputHistoryVisualization::LaneCellFill::Hole)
			continue;

		const float gap = laneColorGap(kLaneElisionColor, style.color);
		if (gap < closestToMachine)
			closestToMachine = gap;
	}

	CHECK(closestToProvenance >= kLanePaletteMinCrossGap);
	CHECK(closestToMachine >= kLanePaletteMinCrossGap);
}

} // namespace inputhistorypausetests

#endif // WITH_LOW_LEVEL_TESTS
