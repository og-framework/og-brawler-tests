// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the GATE'S SECOND DISCONTINUITY -- a hard resync, alongside the idle elision
// PauseTest.cpp already covers. A resync opens a new EPOCH: re-run sim ticks map to
// FRESH lane ticks instead of the ones their first run already used, which is the
// structural fix for a re-run tick silently overwriting or hiding behind its own history.
//
// What this suite is really guarding: `laneTickOf` for a re-run tick must answer with
// the NEW epoch's lane tick, never the old one, and the old epoch's own cell must stay
// exactly as it was. A gate that forgot either half would still look plausible -- the
// storm would scroll, but the wrong cell would repaint, or history would vanish.
//
// The second half of the file drives the REAL poll through a mock reader and pins the
// other side of the same story: WHO decides a break has happened. The poll owns that,
// from two readings it already holds -- the gate's own last polled tick, which sees a
// backward jump and nothing else, and the clock's own resync count, which sees either
// direction and names the exact tick the clock left. The residency edges are asked for
// neither: they cannot tell a wipe from a push that landed mid-sweep.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>

#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryaxistests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::ClockDriftReading;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneAdmission;
using brawlerInputHistoryVisualization::LaneAxisEvent;
using brawlerInputHistoryVisualization::LaneAxisEventKind;
using brawlerInputHistoryVisualization::LaneIdleGate;
using brawlerInputHistoryVisualization::MachineStateCell;
using brawlerInputHistoryVisualization::RowProvenanceSummary;
using brawlerInputHistoryVisualization::TickLanePollCounts;

using brawlerInputHistoryVisualization::kAppliedPollWindowTicks;
using brawlerInputHistoryVisualization::kLaneAxisEventKindCount;
using brawlerInputHistoryVisualization::kLaneElisionLedgerCapacity;
using brawlerInputHistoryVisualization::kLanePauseEngageTicks;

// The poll window's own width, in sim ticks: every residency edge below is a consequence
// of it, so no case restates the number.
static const uint32_t kWindowTicks = static_cast<uint32_t>(kAppliedPollWindowTicks);

// ---------------------------------------------------------------------------
// The SlotReader double, driven by a RESIDENT SET rather than by per-tick refs: what
// these cases are about is the ring's own two edges moving, and a wipe is precisely the
// ring losing every slot at once. An unset tick answers NoSlot, as the real seam does.
// ---------------------------------------------------------------------------
class MockSlotReader
{
public:
	void clearResidents() { m_residents.clear(); }
	void markResident(uint32_t simTick) { m_residents.insert(simTick); }

	void markResidentRange(uint32_t firstTick, uint32_t lastTick)
	{
		for (uint32_t tick = firstTick; tick <= lastTick; ++tick)
			m_residents.insert(tick);
	}

	void setProvenance(uint32_t simTick, std::optional<SlotStateProvenance> provenance)
	{
		m_provenances[simTick] = provenance;
	}

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		return (m_residents.find(simTick) == m_residents.end())
		           ? AppliedCaptureRef{}
		           : AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick };
	}

	std::optional<SlotStateProvenance> slotProvenance(uint32_t simTick) const
	{
		const auto it = m_provenances.find(simTick);
		return (it == m_provenances.end()) ? std::nullopt : it->second;
	}

	bool hasCorrectionCache() const { return true; }

private:
	std::set<uint32_t>                                    m_residents;
	std::map<uint32_t, std::optional<SlotStateProvenance>> m_provenances;
};

// One whole lane poll through the REAL entry point. The idle gate is handed no input and
// switched OFF, so the only thing that can end a poll early is the break these cases are
// about. ⛔ NO CASE BELOW CALLS admit() ITSELF -- the detection under test is the poll's.
static TickLanePollCounts poll(const MockSlotReader&  reader,
                               uint32_t               liveSimTick,
                               DAttackState           machineState,
                               InputHistoryTickLanes& lanes)
{
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(
		reader, liveSimTick, machineState, std::nullopt, false, std::nullopt, std::nullopt,
		std::nullopt, inversion, lanes);
}

// The same entry point with the idle gate LIVE: a neutral capture and the pause
// switched on, so a re-presented tick is answered by the gate's own early return.
static TickLanePollCounts pollIdle(const MockSlotReader&  reader,
                                   uint32_t               liveSimTick,
                                   InputHistoryTickLanes& lanes)
{
	CaptureRowFields idle;
	idle.direction  = DirectionBucket::Neutral;
	idle.buttonMask = 0u;

	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(
		reader, liveSimTick, DAttackState::Idle, idle, true, std::nullopt, std::nullopt,
		std::nullopt, inversion, lanes);
}

// The same entry point again, with a CLOCK READING. A forward resync leaves nothing
// else behind: the poll's tick jumps AHEAD, so every arm watching for a tick going
// backwards stays silent, and the residency edges cannot be asked.
static TickLanePollCounts pollWithClock(const MockSlotReader&  reader,
                                        uint32_t               liveSimTick,
                                        ClockDriftReading      clock,
                                        InputHistoryTickLanes& lanes)
{
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(
		reader, liveSimTick, DAttackState::Idle, std::nullopt, false, std::nullopt,
		std::nullopt, std::optional<ClockDriftReading>(clock), inversion, lanes);
}

// One reading of the clock's event seam. The drift half is left alone -- the clock
// suite pins that, and no case here reads it.
// ⛔ THE COUNT IS A TOTAL: a case MOVES it between two polls, never sets a delta.
static ClockDriftReading seamReading(uint32_t simTick, uint32_t hardResyncCount,
                                     uint32_t fromTick, uint32_t toTick)
{
	ClockDriftReading reading;
	reading.predictionTick         = simTick;
	reading.hardResyncCount        = hardResyncCount;
	reading.lastHardResyncFromTick = fromTick;
	reading.lastHardResyncToTick   = toTick;
	return reading;
}

// ---------------------------------------------------------------------------
// A BACKWARD BREAK -- THE STORM'S OWN SHAPE
// ---------------------------------------------------------------------------

TEST_CASE("Axis.ABackwardBreakOpensANewEpochAndNeverOverwritesTheFirstEpochsCell",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;

	// 100 recorded ticks, identity-mapped (sim 1..100 -> lane 1..100): nothing elided or
	// resynced yet, so every sim tick is its own lane tick.
	for (uint32_t tick = 1u; tick <= 100u; ++tick)
	{
		REQUIRE(lanes.editGate().admit(tick, false, true) == LaneAdmission::Recorded);
		const std::optional<uint32_t> laneTick = lanes.gate().laneTickOf(tick);
		REQUIRE(laneTick.has_value());
		lanes.editProvenance().record(*laneTick, RowProvenanceSummary::Confirmed);
	}

	// The first epoch's own cell for tick 90, pinned before the break touches anything.
	const RowProvenanceSummary* beforeBreak = lanes.provenanceAt(90u);
	REQUIRE(beforeBreak != nullptr);
	CHECK(*beforeBreak == RowProvenanceSummary::Confirmed);

	// The backward break: the previous epoch was last polled at 100, the new one starts at 78.
	REQUIRE(lanes.editGate().admit(78u, false, true, std::optional<uint32_t>(100u))
	        == LaneAdmission::Recorded);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& event = lanes.gate().axisEventAt(0u);
	CHECK(event.kind == LaneAxisEventKind::Resync);
	CHECK(event.simTick == 78u);
	CHECK(event.fromSimTick == 100u);
	CHECK(event.skippedTicks == 0u);
	CHECK(event.laneTick == 101u);

	REQUIRE(lanes.gate().laneTickOf(78u).has_value());
	CHECK(*lanes.gate().laneTickOf(78u) == 102u);

	// Re-run ticks 79..90 in the new epoch.
	for (uint32_t tick = 79u; tick <= 90u; ++tick)
		REQUIRE(lanes.editGate().admit(tick, false, true) == LaneAdmission::Recorded);

	// Sim tick 90 now names the NEW epoch's lane tick, not its old one.
	REQUIRE(lanes.gate().laneTickOf(90u).has_value());
	CHECK(*lanes.gate().laneTickOf(90u) == 114u);

	// The re-run's own cell, written at the new lane tick with a DIFFERENT value.
	lanes.editProvenance().record(114u, RowProvenanceSummary::Resimulated);

	// ⛔ THE FIRST EPOCH'S CELL AT LANE TICK 90 IS UNTOUCHED -- a second epoch never
	// overwrites the first; it only ever gets a fresh cell of its own.
	const RowProvenanceSummary* afterBreak = lanes.provenanceAt(90u);
	REQUIRE(afterBreak != nullptr);
	CHECK(*afterBreak == RowProvenanceSummary::Confirmed);
}

TEST_CASE("Axis.AForwardBreakLeavesTheNeverSimulatedRangeUnreachableByLaneTickOf",
          "[CharacterViz][InputHistoryViz]")
{
	LaneIdleGate gate;

	for (uint32_t tick = 1u; tick <= 100u; ++tick)
		REQUIRE(gate.admit(tick, false, true) == LaneAdmission::Recorded);

	// A forward jump: the previous epoch's last tick was 100, the new one starts at 131.
	REQUIRE(gate.admit(131u, false, true, std::optional<uint32_t>(100u))
	        == LaneAdmission::Recorded);

	REQUIRE(gate.axisEventCount() == 1u);
	const LaneAxisEvent& event = gate.axisEventAt(0u);
	CHECK(event.kind == LaneAxisEventKind::Resync);
	CHECK(event.simTick == 131u);
	CHECK(event.fromSimTick == 100u);

	// 100..130 were never simulated by either epoch: neither answers for them.
	CHECK_FALSE(gate.laneTickOf(115u).has_value());

	// The new epoch's own first tick has a fresh lane tick, one past the marker.
	REQUIRE(gate.laneTickOf(131u).has_value());
	CHECK(*gate.laneTickOf(131u) == event.laneTick + 1u);

	// Tick 100 itself was the OLD epoch's own last tick, recorded before the break --
	// it keeps the identity mapping the old epoch always had.
	REQUIRE(gate.laneTickOf(100u).has_value());
	CHECK(*gate.laneTickOf(100u) == 100u);
}

TEST_CASE("Axis.ABreakWhilePausedClosesTheOpenSpanFirstThenFilesTheResyncAtTheNextLaneTick",
          "[CharacterViz][InputHistoryViz]")
{
	LaneIdleGate gate;

	// 74 active ticks, then inactive from 75 -- the span engages at 90 (the 16th
	// consecutive inactive tick, kLanePauseEngageTicks == 15).
	for (uint32_t tick = 1u; tick <= 74u; ++tick)
		REQUIRE(gate.admit(tick, false, true) == LaneAdmission::Recorded);

	for (uint32_t tick = 75u; tick <= 89u; ++tick)
		REQUIRE(gate.admit(tick, true, true) == LaneAdmission::Recorded);

	REQUIRE(gate.admit(90u, true, true) == LaneAdmission::Elided);
	REQUIRE(gate.paused());
	REQUIRE(gate.axisEventCount() == 0u);

	// The poll keeps polling while paused; the last tick it saw before the break is 100.
	for (uint32_t tick = 91u; tick <= 100u; ++tick)
		REQUIRE(gate.admit(tick, true, true) == LaneAdmission::Elided);

	REQUIRE(gate.admit(78u, false, true, std::optional<uint32_t>(100u))
	        == LaneAdmission::Recorded);

	// An elision cannot span an epoch: the still-open span closes first, THEN the resync
	// files -- both kinds present, at consecutive lane ticks.
	REQUIRE(gate.axisEventCount() == 2u);

	const LaneAxisEvent& elision = gate.axisEventAt(0u);
	CHECK(elision.kind == LaneAxisEventKind::Elision);
	CHECK(elision.simTick == 90u);
	CHECK(elision.skippedTicks == 11u);

	const LaneAxisEvent& resync = gate.axisEventAt(1u);
	CHECK(resync.kind == LaneAxisEventKind::Resync);
	CHECK(resync.simTick == 78u);
	CHECK(resync.fromSimTick == 100u);
	CHECK(resync.laneTick == elision.laneTick + 1u);
}

// ---------------------------------------------------------------------------
// THE STORM SHAPE -- 128 REPEATS, THE LEDGER'S OWN CAPACITY REACHED
// ---------------------------------------------------------------------------

TEST_CASE("Axis.TheStormOfBackwardBreaksConsumesTwentyThreeLaneTicksPerBreakAndTheLedgerDropsTheOldest",
          "[CharacterViz][InputHistoryViz]")
{
	InputHistoryTickLanes lanes;

	auto step = [&lanes](uint32_t tick, std::optional<uint32_t> axisBreakFromSimTick)
	{
		const LaneAdmission admission =
			lanes.editGate().admit(tick, false, true, axisBreakFromSimTick);
		if (admission == LaneAdmission::Recorded)
			lanes.noteAxisTick(*lanes.gate().laneTickOf(tick));
	};

	// 50 ticks of pre-storm baseline, still identity-mapped.
	for (uint32_t tick = 1u; tick <= 50u; ++tick)
		step(tick, std::nullopt);
	REQUIRE(lanes.newestAxisTick() == 50u);

	// 128 repeats of a backward resync to the SAME frozen tick (1000) -- the constant-
	// target storm shape -- each preceded by 22 sim ticks of the epoch it interrupts.
	uint32_t lastSimTick = 50u;
	for (uint32_t cycle = 0u; cycle < 128u; ++cycle)
	{
		step(1000u, std::optional<uint32_t>(lastSimTick));
		for (uint32_t offset = 1u; offset <= 21u; ++offset)
			step(1000u + offset, std::nullopt);
		lastSimTick = 1000u + 21u;
	}

	// 1 marker + 22 recorded ticks per break, never hand-picked: the axis advanced by
	// exactly 128 * 23 lane ticks over the whole storm.
	CHECK(lanes.newestAxisTick() - 50u == 128u * 23u);

	// 128 markers into a 120-entry ledger: the oldest 8 are dropped.
	REQUIRE(lanes.gate().axisEventCount() == kLaneElisionLedgerCapacity);

	// A tick from BEFORE the storm, once explainable only by the now-dropped first
	// marker, answers nullopt -- never the guessed identity mapping.
	CHECK_FALSE(lanes.gate().laneTickOf(25u).has_value());
}

// ---------------------------------------------------------------------------
// EVERY LaneAxisEventKind, SWEPT AGAINST ITS OWN COUNT
// ---------------------------------------------------------------------------

TEST_CASE("Axis.BothAxisEventKindsAreReachedAndTheSweepIsCountedAgainstItsEnum",
          "[CharacterViz][InputHistoryViz]")
{
	std::size_t reached = 0u;

	// Elision.
	{
		LaneIdleGate gate;
		for (uint32_t tick = 0u; tick < 10u; ++tick)
			gate.admit(tick, false, true);
		for (uint32_t tick = 10u; tick < 10u + kLanePauseEngageTicks + 5u; ++tick)
			gate.admit(tick, true, true);
		gate.admit(10u + kLanePauseEngageTicks + 5u, false, true);

		REQUIRE(gate.axisEventCount() == 1u);
		if (gate.axisEventAt(0u).kind == LaneAxisEventKind::Elision)
			++reached;
	}

	// Resync.
	{
		LaneIdleGate gate;
		for (uint32_t tick = 1u; tick <= 10u; ++tick)
			gate.admit(tick, false, true);
		gate.admit(3u, false, true, std::optional<uint32_t>(10u));

		REQUIRE(gate.axisEventCount() == 1u);
		if (gate.axisEventAt(0u).kind == LaneAxisEventKind::Resync)
			++reached;
	}

	CHECK(reached == static_cast<std::size_t>(kLaneAxisEventKindCount));
}

// ---------------------------------------------------------------------------
// THE POLL'S OWN DETECTION -- the one reading, and the shapes that must not fire.
//
// The test is the poll tick going BACKWARD, which only the resync assignment can do.
// The residency edges are not consulted: the window clamps the newer one at the poll
// tick, so on a ring the physics thread is still pushing they move by different amounts
// with nothing wrong at all. Every healthy shape below answers zero.
// ---------------------------------------------------------------------------

TEST_CASE("Axis.ThePollSeesTheBackwardBreakAndTheSecondEpochLeavesTheFirstsCellsAlone",
          "[CharacterViz][InputHistoryViz]")
{
	// The storm's own numbers, from the session log: the clock sat at 6219 and a hard
	// resync assigned it back to 6197, wiping every cache on the way.
	constexpr uint32_t kOldTick     = 6219u;
	constexpr uint32_t kNewTick     = 6197u;
	constexpr uint32_t kCertified   = 6210u;

	MockSlotReader reader;
	reader.markResidentRange(kOldTick - kWindowTicks + 1u, kOldTick);
	// One tick the authority has already certified, so the first epoch's cell for it
	// carries a value the wiped second epoch cannot reproduce.
	reader.setProvenance(kCertified, SlotStateProvenance::AuthorityAgreedKeptPrediction);

	InputHistoryTickLanes lanes;

	const TickLanePollCounts healthy = poll(reader, kOldTick, DAttackState::Idle, lanes);
	CHECK(healthy.axisBreaksBackward == 0u);
	REQUIRE(lanes.gate().axisEventCount() == 0u);

	// The first epoch's own cells, read through the gate and pinned before the break.
	REQUIRE(lanes.gate().laneTickOf(kCertified).has_value());
	const uint32_t firstEpochCertifiedLaneTick = *lanes.gate().laneTickOf(kCertified);
	REQUIRE(lanes.provenanceAt(firstEpochCertifiedLaneTick) != nullptr);
	CHECK(*lanes.provenanceAt(firstEpochCertifiedLaneTick) == RowProvenanceSummary::Confirmed);

	REQUIRE(lanes.gate().laneTickOf(kOldTick).has_value());
	const uint32_t firstEpochLiveLaneTick = *lanes.gate().laneTickOf(kOldTick);
	CHECK(lanes.machineCellAt(firstEpochLiveLaneTick) == MachineStateCell::Idle);

	// The resync: the prediction tick was ASSIGNED backward and every cache wiped, so the
	// very next poll sees a ring holding its own new tick and nothing else.
	reader.clearResidents();
	reader.markResident(kNewTick);
	reader.setProvenance(kCertified, std::nullopt);

	const TickLanePollCounts broken = poll(reader, kNewTick, DAttackState::Attacking, lanes);

	CHECK(broken.axisBreaksBackward == 1u);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const uint32_t markerLaneTick = lanes.gate().axisEventAt(0u).laneTick;
	CHECK(lanes.gate().axisEventAt(0u).kind == LaneAxisEventKind::Resync);
	CHECK(lanes.gate().axisEventAt(0u).simTick == kNewTick);
	CHECK(lanes.gate().axisEventAt(0u).fromSimTick == kOldTick);
	CHECK(lanes.gate().axisEventAt(0u).skippedTicks == 0u);

	// The ring refills, one tick per poll, all the way back to the tick it fell from.
	uint32_t breaksWhileRefilling = 0u;
	for (uint32_t tick = kNewTick + 1u; tick <= kOldTick; ++tick)
	{
		reader.markResident(tick);
		const TickLanePollCounts refill = poll(reader, tick, DAttackState::Attacking, lanes);
		breaksWhileRefilling += refill.axisBreaksBackward;
	}

	// Every refill poll steps the tick forward, so no second break is filed.
	CHECK(breaksWhileRefilling == 0u);
	REQUIRE(lanes.gate().axisEventCount() == 1u);

	// The re-run of the same tick number lands one whole epoch past the marker cell.
	REQUIRE(lanes.gate().laneTickOf(kOldTick).has_value());
	const uint32_t secondEpochLiveLaneTick = *lanes.gate().laneTickOf(kOldTick);
	CHECK(secondEpochLiveLaneTick == markerLaneTick + 1u + (kOldTick - kNewTick));
	CHECK(lanes.machineCellAt(secondEpochLiveLaneTick) == MachineStateCell::Attacking);

	// ⛔ THE FIRST EPOCH'S CELLS ARE UNTOUCHED -- a second epoch only ever gets fresh ones.
	CHECK(lanes.machineCellAt(firstEpochLiveLaneTick) == MachineStateCell::Idle);

	REQUIRE(lanes.gate().laneTickOf(kCertified).has_value());
	const uint32_t secondEpochCertifiedLaneTick = *lanes.gate().laneTickOf(kCertified);
	CHECK(secondEpochCertifiedLaneTick != firstEpochCertifiedLaneTick);
	REQUIRE(lanes.provenanceAt(secondEpochCertifiedLaneTick) != nullptr);
	CHECK(*lanes.provenanceAt(secondEpochCertifiedLaneTick)
	      == RowProvenanceSummary::RanUnconfirmed);
	REQUIRE(lanes.provenanceAt(firstEpochCertifiedLaneTick) != nullptr);
	CHECK(*lanes.provenanceAt(firstEpochCertifiedLaneTick) == RowProvenanceSummary::Confirmed);
}

// ---------------------------------------------------------------------------
// THE SHAPES THAT MUST NOT FIRE -- one case each, because a detector that fired on any
// of them would file a spurious epoch on every session's first second.
// ---------------------------------------------------------------------------

TEST_CASE("Axis.ASkipMovesBothResidencyEdgesByTwoAndFilesNoBreak",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kFirstPollTick = 6219u;

	MockSlotReader reader;
	reader.markResidentRange(kFirstPollTick - kWindowTicks + 1u, kFirstPollTick);

	InputHistoryTickLanes lanes;
	poll(reader, kFirstPollTick, DAttackState::Idle, lanes);

	// A Skip advances the clock by two in one step and BOTH ticks get slots, so the
	// frontier and the old edge move by the same two.
	reader.markResidentRange(kFirstPollTick + 1u, kFirstPollTick + 2u);
	const TickLanePollCounts skipped =
		poll(reader, kFirstPollTick + 2u, DAttackState::Idle, lanes);

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.oldestResident
	      == kFirstPollTick + 2u - kWindowTicks + 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == kFirstPollTick + 2u);

	CHECK(skipped.axisBreaksBackward == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
}

TEST_CASE("Axis.StartupHoldsTheOldEdgeAtTickZeroWhileTheFrontierClimbsAndFilesNoBreak",
          "[CharacterViz][InputHistoryViz]")
{
	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	// The tick-0 phantom: for a whole window's worth of ticks the window's own oldest end
	// is clamped at 0, so the old edge cannot move while the frontier climbs past it.
	uint32_t breaksSeen = 0u;
	for (uint32_t tick = 0u; tick < kWindowTicks; ++tick)
	{
		reader.markResident(tick);
		const TickLanePollCounts counts = poll(reader, tick, DAttackState::Idle, lanes);
		breaksSeen += counts.axisBreaksBackward;
	}

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.oldestResident == 0u);
	CHECK(lanes.residencyReading()->residency.newestResident == kWindowTicks - 1u);

	// And the first poll that actually scrolls the window: both edges move by one.
	reader.markResident(kWindowTicks);
	const TickLanePollCounts scrolled = poll(reader, kWindowTicks, DAttackState::Idle, lanes);
	breaksSeen += scrolled.axisBreaksBackward;

	CHECK(lanes.residencyReading()->residency.oldestResident == 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == kWindowTicks);

	CHECK(breaksSeen == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
}

TEST_CASE("Axis.ALateCorrectionCacheGivesNoPreviousEdgeToCompareAndFilesNoBreak",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kFirstPollTick = 500u;

	// Nothing resident at all yet: the cache for this id has not been created.
	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	const TickLanePollCounts before = poll(reader, kFirstPollTick, DAttackState::Idle, lanes);
	CHECK(before.axisBreaksBackward == 0u);
	REQUIRE(lanes.residencyReading().has_value());
	REQUIRE_FALSE(lanes.residencyReading()->residency.anyResident);

	// The cache lands late and answers for exactly one tick, its own first.
	reader.markResident(kFirstPollTick + 1u);
	const TickLanePollCounts created =
		poll(reader, kFirstPollTick + 1u, DAttackState::Idle, lanes);

	CHECK(lanes.residencyReading()->residency.oldestResident == kFirstPollTick + 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == kFirstPollTick + 1u);

	CHECK(created.axisBreaksBackward == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
}

TEST_CASE("Axis.APollThatFindsNothingResidentFilesNoBreakEvenAfterAResidentOne",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kFirstPollTick = 6219u;

	MockSlotReader reader;
	reader.markResidentRange(kFirstPollTick - kWindowTicks + 1u, kFirstPollTick);

	InputHistoryTickLanes lanes;
	poll(reader, kFirstPollTick, DAttackState::Idle, lanes);

	// Nothing resident this poll: both bounds are unset, which classifyNoSlot's own
	// ordering rule already refuses to read as a fact about the ring.
	// ⛔ AN EMPTY WINDOW IS NOT EVIDENCE OF A WIPE.
	reader.clearResidents();
	const TickLanePollCounts empty =
		poll(reader, kFirstPollTick + 1u, DAttackState::Idle, lanes);

	REQUIRE(lanes.residencyReading().has_value());
	REQUIRE_FALSE(lanes.residencyReading()->residency.anyResident);

	CHECK(empty.axisBreaksBackward == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
}

TEST_CASE("Axis.TwoPollsOfTheSameTickFileNoBreakAndReturnTheSameAdmission",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kPollTick = 6219u;

	MockSlotReader reader;
	reader.markResidentRange(kPollTick - kWindowTicks + 1u, kPollTick);

	InputHistoryTickLanes lanes;

	// The poll runs at render rate, so it re-presents the same tick on the next frame
	// drawn: identical edges, identical tick, and neither test may read that as a break.
	const TickLanePollCounts first  = poll(reader, kPollTick, DAttackState::Idle, lanes);
	const TickLanePollCounts second = poll(reader, kPollTick, DAttackState::Idle, lanes);

	CHECK(second.axisBreaksBackward == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
	CHECK(second.admission == first.admission);
	CHECK(second.admission == LaneAdmission::Recorded);
}

// ---------------------------------------------------------------------------
// THE RING RUNS AHEAD OF THE POLL -- the production shape, and the one the fixtures
// above never presented.
//
// `liveSimTick` is the game thread's read of a counter the physics thread owns, and the
// sweep runs later in the same call. A step landing in that gap moves the ring's old
// edge while the frontier the WINDOW can see stays pinned at the tick being polled, so
// the two residency edges move by different amounts with nothing wrong at all.
// ⛔ EVERY CASE BELOW MARKS A RESIDENT TICK GREATER THAN THE TICK IT POLLS.
// ---------------------------------------------------------------------------

TEST_CASE("Axis.ARingThatRanAheadOfAnUnchangedPollTickFilesNoBreak",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kPollTick = 6219u;

	MockSlotReader reader;
	reader.markResidentRange(kPollTick - kWindowTicks + 1u, kPollTick);

	InputHistoryTickLanes lanes;
	const TickLanePollCounts first = poll(reader, kPollTick, DAttackState::Idle, lanes);

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.oldestResident == kPollTick - kWindowTicks + 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == kPollTick);

	// One physics step lands between the read of the tick and the sweep: the ring recycles
	// its oldest slot and its frontier goes one PAST the tick this poll is re-presenting.
	reader.clearResidents();
	reader.markResidentRange(kPollTick - kWindowTicks + 2u, kPollTick + 1u);

	const TickLanePollCounts second = poll(reader, kPollTick, DAttackState::Idle, lanes);

	CHECK(second.axisBreaksBackward == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);

	// The old edge moved by one and the clamped frontier could not -- and that difference
	// is the whole of what the ring did, not evidence of anything.
	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.oldestResident == kPollTick - kWindowTicks + 2u);
	CHECK(lanes.residencyReading()->residency.newestResident == kPollTick);

	CHECK(second.admission == first.admission);
	CHECK(second.admission == LaneAdmission::Recorded);
}

TEST_CASE("Axis.ARingAheadByASkipWithThePollAdvancingOneFilesNoBreak",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kFirstPollTick = 6219u;

	// Ring and poll in LOCKSTEP first, which is where every session starts and where the
	// two cases either side of this one begin. A Skip then pushes the cache twice while
	// the ring is already one ahead, so the old edge moves by two and the clamped
	// frontier by one -- different amounts, with nothing wrong at all.
	MockSlotReader reader;
	reader.markResidentRange(kFirstPollTick - kWindowTicks + 1u, kFirstPollTick);

	InputHistoryTickLanes lanes;
	poll(reader, kFirstPollTick, DAttackState::Idle, lanes);

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.oldestResident == kFirstPollTick - kWindowTicks + 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == kFirstPollTick);

	reader.clearResidents();
	reader.markResidentRange(kFirstPollTick - kWindowTicks + 3u, kFirstPollTick + 2u);

	const TickLanePollCounts skipped =
		poll(reader, kFirstPollTick + 1u, DAttackState::Idle, lanes);

	CHECK(skipped.axisBreaksBackward == 0u);
	CHECK(skipped.axisBreaksFromSeam == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);

	// The old edge moved by TWO and the clamped frontier by one.
	CHECK(lanes.residencyReading()->residency.oldestResident == kFirstPollTick - kWindowTicks + 3u);
	CHECK(lanes.residencyReading()->residency.newestResident == kFirstPollTick + 1u);
}

TEST_CASE("Axis.ARingThatRanAheadUnderAPausedIdleTickLeavesTheOpenSpanOpen",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kFirstTick = 6100u;

	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	// Ring and poll in lockstep until the pause engages, so the only thing under test
	// afterwards is what the advancing ring does to a span that is already open.
	uint32_t tick = kFirstTick;
	for (uint32_t idle = 0u; idle <= kLanePauseEngageTicks; ++idle, ++tick)
	{
		reader.clearResidents();
		reader.markResidentRange(tick - kWindowTicks + 1u, tick);
		pollIdle(reader, tick, lanes);
	}

	const uint32_t pausedTick = tick - 1u;

	REQUIRE(lanes.gate().paused());
	REQUIRE(lanes.gate().consecutiveInactiveTicks() == kLanePauseEngageTicks + 1u);
	REQUIRE(lanes.gate().axisEventCount() == 0u);

	// The next frame re-presents the SAME idle tick while a step lands under it.
	reader.clearResidents();
	reader.markResidentRange(pausedTick - kWindowTicks + 2u, pausedTick + 1u);

	const TickLanePollCounts repolled = pollIdle(reader, pausedTick, lanes);

	CHECK(repolled.axisBreaksBackward == 0u);
	CHECK(repolled.admission == LaneAdmission::Elided);

	// A break here would close the open span and zero the hysteresis, so the pause could
	// not re-engage for another whole run of inactive ticks.
	CHECK(lanes.gate().axisEventCount() == 0u);
	CHECK(lanes.gate().paused());
	CHECK(lanes.gate().consecutiveInactiveTicks() == kLanePauseEngageTicks + 1u);
}

TEST_CASE("Axis.TheSweepClampsTheNewestResidentAtTheTickBeingPolled",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kPollTick = 6219u;

	// The ring reaches two ticks past the one the game thread read. The sweep walks the
	// WINDOW, so it must stop at the poll tick rather than follow the ring to its frontier.
	MockSlotReader reader;
	reader.markResidentRange(kPollTick - kWindowTicks + 1u, kPollTick + 2u);

	InputHistoryTickLanes lanes;
	const TickLanePollCounts counts = poll(reader, kPollTick, DAttackState::Idle, lanes);

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.anyResident);
	CHECK(lanes.residencyReading()->residency.oldestResident == kPollTick - kWindowTicks + 1u);
	CHECK(lanes.residencyReading()->residency.newestResident == kPollTick);

	CHECK(counts.axisBreaksBackward == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
}

// ---------------------------------------------------------------------------
// THE CLOCK'S OWN SEAM -- the only race-free witness, and the only forward one.
//
// A counter that changed since the last reading is not derived from anything: it is
// the clock saying it corrected itself, whichever way the tick moved. It also names
// the exact tick the clock left, which nothing on this side of the seam can.
// ---------------------------------------------------------------------------

TEST_CASE("Axis.AForwardResyncSeenOnlyThroughTheSeamFilesTheExactFromAndToPair",
          "[CharacterViz][InputHistoryViz]")
{
	// A FORWARD hard resync: the clock is assigned ahead, so the poll tick jumps ahead
	// too and the backward arm has nothing to see. The wiped ring shows only as a
	// one-tick residency span -- a state, honestly drawn, and not what files the break.
	constexpr uint32_t kFromTick = 6197u;
	constexpr uint32_t kJump     = 31u;
	constexpr uint32_t kToTick   = kFromTick + kJump;

	MockSlotReader reader;
	reader.markResidentRange(kFromTick - kWindowTicks + 1u, kFromTick);

	InputHistoryTickLanes lanes;

	const TickLanePollCounts healthy =
		pollWithClock(reader, kFromTick, seamReading(kFromTick, 0u, 0u, 0u), lanes);
	REQUIRE(healthy.axisBreaksFromSeam == 0u);
	REQUIRE(lanes.gate().axisEventCount() == 0u);

	reader.clearResidents();
	reader.markResident(kToTick);

	const TickLanePollCounts broken = pollWithClock(
		reader, kToTick, seamReading(kToTick, 1u, kFromTick, kToTick), lanes);

	CHECK(broken.axisBreaksFromSeam == 1u);
	CHECK(broken.axisBreaksBackward == 0u);
	// The backward arm being silent on a FORWARD jump is what it is for, not a dispute.
	CHECK(broken.axisBreakDisagreements == 0u);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	CHECK(lanes.gate().axisEventAt(0u).kind == LaneAxisEventKind::Resync);
	CHECK(lanes.gate().axisEventAt(0u).fromSimTick == kFromTick);
	CHECK(lanes.gate().axisEventAt(0u).simTick == kToTick);

	REQUIRE(lanes.residencyReading().has_value());
	CHECK(lanes.residencyReading()->residency.oldestResident == kToTick);
	CHECK(lanes.residencyReading()->residency.newestResident == kToTick);

	// The ticks the client never ran have no cell, and the epoch before them still does.
	CHECK_FALSE(lanes.gate().laneTickOf(kFromTick + 1u).has_value());
	CHECK_FALSE(lanes.gate().laneTickOf(kToTick - 1u).has_value());
	CHECK(lanes.gate().laneTickOf(kFromTick).has_value());
}

TEST_CASE("Axis.TheStormThroughBothArmsFilesOneBreakPerResyncAndNeverDisagrees",
          "[CharacterViz][InputHistoryViz]")
{
	// The user's own session: 128 x `oldTick=6219 -> newTick=6197`. BOTH arms see each
	// one -- the poll tick goes backwards and the clock's count moves -- and the two must
	// produce ONE break between them, never two and never none.
	constexpr uint32_t kOldTick = 6219u;
	constexpr uint32_t kNewTick = 6197u;
	constexpr uint32_t kEpochs  = 128u;
	constexpr int64_t  kLabel   =
		static_cast<int64_t>(kNewTick) - static_cast<int64_t>(kOldTick);

	MockSlotReader reader;
	reader.markResidentRange(kNewTick - kWindowTicks + 1u, kOldTick);

	InputHistoryTickLanes lanes;

	uint32_t resyncCount   = 0u;
	uint32_t fromTheSeam   = 0u;
	uint32_t fromTheArm    = 0u;
	uint32_t disagreements = 0u;

	pollWithClock(reader, kOldTick, seamReading(kOldTick, resyncCount, 0u, 0u), lanes);

	for (uint32_t epoch = 0u; epoch < kEpochs; ++epoch)
	{
		++resyncCount;
		const TickLanePollCounts broken = pollWithClock(
			reader, kNewTick, seamReading(kNewTick, resyncCount, kOldTick, kNewTick), lanes);

		fromTheSeam   += broken.axisBreaksFromSeam;
		fromTheArm    += broken.axisBreaksBackward;
		disagreements += broken.axisBreakDisagreements;

		// The epoch it interrupts: the client climbs back to the tick it fell from, and
		// the count does not move again until the next resync fires.
		for (uint32_t tick = kNewTick + 1u; tick <= kOldTick; ++tick)
		{
			const TickLanePollCounts refill = pollWithClock(
				reader, tick, seamReading(tick, resyncCount, kOldTick, kNewTick), lanes);

			fromTheSeam   += refill.axisBreaksFromSeam;
			fromTheArm    += refill.axisBreaksBackward;
			disagreements += refill.axisBreakDisagreements;
		}
	}

	CHECK(fromTheSeam == kEpochs);
	CHECK(fromTheArm == kEpochs);
	CHECK(disagreements == 0u);

	// ONE break FILED per resync. The storm outruns the ledger, which drops its oldest:
	// that costs reach backwards and never fabricates a column.
	REQUIRE(kEpochs > kLaneElisionLedgerCapacity);
	REQUIRE(lanes.gate().axisEventCount() == kLaneElisionLedgerCapacity);

	std::size_t labelled = 0u;
	for (std::size_t index = 0u; index < lanes.gate().axisEventCount(); ++index)
	{
		const LaneAxisEvent& event = lanes.gate().axisEventAt(index);
		if (event.kind == LaneAxisEventKind::Resync
		    && static_cast<int64_t>(event.simTick)
		           - static_cast<int64_t>(event.fromSimTick) == kLabel)
		{
			++labelled;
		}
	}

	CHECK(labelled == kLaneElisionLedgerCapacity);
	CHECK(kLabel == -22);
}

TEST_CASE("Axis.TheSeamNamesTheExactOldTickWhereTheBackwardArmCouldOnlyBoundIt",
          "[CharacterViz][InputHistoryViz]")
{
	// A poll is a RENDER frame and the clock runs at sim rate, so the clock can leave a
	// tick this display never polled. The backward arm can name only the last tick it
	// DID see, which is a bound on the old tick; the seam names the old tick itself.
	constexpr uint32_t kLastPolled = 6200u;
	constexpr uint32_t kOldTick    = 6219u;
	constexpr uint32_t kNewTick    = 6197u;

	MockSlotReader reader;
	reader.markResidentRange(kNewTick - kWindowTicks + 1u, kOldTick);

	InputHistoryTickLanes lanes;
	pollWithClock(reader, kLastPolled, seamReading(kLastPolled, 0u, 0u, 0u), lanes);

	REQUIRE(lanes.gate().lastPolledSimTick().has_value());
	REQUIRE(*lanes.gate().lastPolledSimTick() == kLastPolled);

	// Nineteen sim ticks pass between two drawn frames, and then the resync fires.
	const TickLanePollCounts broken = pollWithClock(
		reader, kNewTick, seamReading(kNewTick, 1u, kOldTick, kNewTick), lanes);

	CHECK(broken.axisBreaksFromSeam == 1u);
	CHECK(broken.axisBreaksBackward == 1u);
	CHECK(broken.axisBreakDisagreements == 0u);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	const LaneAxisEvent& event = lanes.gate().axisEventAt(0u);

	CHECK(event.fromSimTick == kOldTick);
	CHECK(event.fromSimTick != kLastPolled);

	const int64_t label =
		static_cast<int64_t>(event.simTick) - static_cast<int64_t>(event.fromSimTick);
	const int64_t bound =
		static_cast<int64_t>(event.simTick) - static_cast<int64_t>(kLastPolled);

	// ⭐ THE LOG'S OWN NUMBER, AND THE ONE THE OLD BOUND WOULD HAVE DRAWN INSTEAD.
	CHECK(label == -22);
	CHECK(bound == -3);
	CHECK(label != bound);
}

TEST_CASE("Axis.EitherArmFiringAloneIsCountedAsADisagreementAndStillFilesOneBreak",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kOldTick   = 6219u;
	constexpr uint32_t kNewTick   = 6197u;
	constexpr uint32_t kBoundTick = 6210u;

	MockSlotReader reader;
	reader.markResidentRange(kNewTick - kWindowTicks + 1u, kOldTick);

	// The seam says a BACKWARD resync fired and the poll's tick did not move. One of the
	// two is wrong about time, and the display reports that rather than choosing.
	{
		InputHistoryTickLanes lanes;
		pollWithClock(reader, kOldTick, seamReading(kOldTick, 0u, 0u, 0u), lanes);

		const TickLanePollCounts broken = pollWithClock(
			reader, kOldTick, seamReading(kOldTick, 1u, kOldTick, kNewTick), lanes);

		CHECK(broken.axisBreaksFromSeam == 1u);
		CHECK(broken.axisBreaksBackward == 0u);
		CHECK(broken.axisBreakDisagreements == 1u);

		// ONE break, never none: a reported disagreement is not a refusal to draw.
		REQUIRE(lanes.gate().axisEventCount() == 1u);
		CHECK(lanes.gate().axisEventAt(0u).fromSimTick == kOldTick);
	}

	// The other way round: the poll's tick went backwards and the count did not move.
	{
		InputHistoryTickLanes lanes;
		pollWithClock(reader, kBoundTick, seamReading(kBoundTick, 4u, kOldTick, kNewTick),
			lanes);

		const TickLanePollCounts broken = pollWithClock(
			reader, kNewTick, seamReading(kNewTick, 4u, kOldTick, kNewTick), lanes);

		CHECK(broken.axisBreaksFromSeam == 0u);
		CHECK(broken.axisBreaksBackward == 1u);
		CHECK(broken.axisBreakDisagreements == 1u);

		// ONE break, never two -- and its label is the bound, because the seam's own pair
		// belongs to a resync this poll did not witness.
		REQUIRE(lanes.gate().axisEventCount() == 1u);
		CHECK(lanes.gate().axisEventAt(0u).fromSimTick == kBoundTick);
		CHECK(lanes.gate().axisEventAt(0u).fromSimTick != kOldTick);
	}
}

TEST_CASE("Axis.AFirstReadingAndAnAbsentOneFileNoSeamBreakAndNoDisagreement",
          "[CharacterViz][InputHistoryViz]")
{
	// A counter is a TOTAL. The first reading a display takes has nothing to difference
	// against, so a clock that resynced a hundred times before the display opened files
	// nothing -- and a role that hands in no clock at all cannot be cross-checked.
	constexpr uint32_t kTick = 6100u;

	MockSlotReader reader;
	reader.markResidentRange(kTick - kWindowTicks + 1u, kTick);

	InputHistoryTickLanes lanes;

	const TickLanePollCounts first = pollWithClock(
		reader, kTick, seamReading(kTick, 100u, kTick - 22u, kTick), lanes);

	CHECK(first.axisBreaksFromSeam == 0u);
	CHECK(first.axisBreakDisagreements == 0u);
	CHECK(first.rateMarksFiled == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);

	const TickLanePollCounts silent = poll(reader, kTick + 1u, DAttackState::Idle, lanes);
	CHECK(silent.axisBreaksFromSeam == 0u);
	CHECK(silent.axisBreakDisagreements == 0u);
	CHECK(silent.rateMarksFiled == 0u);

	// And a reading arriving after that gap is a first reading again: the display was
	// told nothing about the polls in between and may not claim them.
	const TickLanePollCounts resumed = pollWithClock(
		reader, kTick + 2u, seamReading(kTick + 2u, 101u, kTick - 22u, kTick), lanes);

	CHECK(resumed.axisBreaksFromSeam == 0u);
	CHECK(resumed.axisBreakDisagreements == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);
	CHECK(lanes.rateMarkCount() == 0u);
}

} // namespace inputhistoryaxistests

#endif // WITH_LOW_LEVEL_TESTS
