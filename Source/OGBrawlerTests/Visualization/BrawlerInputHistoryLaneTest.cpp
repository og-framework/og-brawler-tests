// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins BrawlerInputHistoryVisualizationLanes.h and the lane half of
// BrawlerInputHistoryVisualizationPoll.h -- the two PER-TICK bars of the frame meter.
//
// WHAT THIS SUITE IS REALLY GUARDING is the reason the lanes exist at all. The display
// used to hang lineage off a run-length row, and a worst-case-wins merge over such a row
// SATURATES: one resimulated tick inside a 99-tick hold made all 99 read as resimulated
// and never recovered. A per-tick cell is the fix, and the property that proves it is
// "a late correction repaints exactly one cell" -- which no one-shot poll can show.
//
// The second thing it guards is the fill ASYMMETRY. Provenance can be back-filled from
// the correction cache; machine state can only be sampled at the live tick. So the two
// lanes take DIFFERENT re-poll rules, and the machine lane must show holes rather than
// carry a neighbour forward -- a fabricated cell in a netcode diagnostic reads as evidence.
//
// The join itself is NOT re-tested here: BrawlerInputHistoryProvenanceTest.cpp owns the
// four arms and the ladder. What is tested is that the lane is fed BY that join, which is
// why every expected cell below is compared against captureSummaryOf rather than a literal.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistorylanetests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneWriteResult;
using brawlerInputHistoryVisualization::MachineStateCell;
using brawlerInputHistoryVisualization::ProvenanceLane;
using brawlerInputHistoryVisualization::RowProvenanceSummary;
using brawlerInputHistoryVisualization::TickLanePollCounts;

// ---------------------------------------------------------------------------
// The SlotReader double. Its DEFAULT is settable because the interesting cases are
// about what a SECOND poll changes, which needs a whole window answering one way first.
// ---------------------------------------------------------------------------
class MockSlotReader
{
public:
	void setDefaultRef(AppliedCaptureRef ref) { m_defaultRef = ref; }

	void setDefaultProvenance(std::optional<SlotStateProvenance> provenance)
	{
		m_defaultProvenance = provenance;
	}

	void setRef(uint32_t simTick, AppliedCaptureRef ref) { m_refs[simTick] = ref; }

	void setProvenance(uint32_t simTick, SlotStateProvenance provenance)
	{
		m_provenances[simTick] = provenance;
	}

	void setHasCorrectionCache(bool hasCache) { m_hasCorrectionCache = hasCache; }

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		const auto it = m_refs.find(simTick);
		return (it == m_refs.end()) ? m_defaultRef : it->second;
	}

	std::optional<SlotStateProvenance> slotProvenance(uint32_t simTick) const
	{
		const auto it = m_provenances.find(simTick);
		return (it == m_provenances.end()) ? m_defaultProvenance
		                                   : std::optional<SlotStateProvenance>(it->second);
	}

	bool hasCorrectionCache() const { return m_hasCorrectionCache; }

private:
	AppliedCaptureRef                       m_defaultRef{};
	std::optional<SlotStateProvenance>      m_defaultProvenance{};
	std::map<uint32_t, AppliedCaptureRef>   m_refs;
	std::map<uint32_t, SlotStateProvenance> m_provenances;
	bool                                    m_hasCorrectionCache = true;
};

// One poll, with the scratch inversion the production store also keeps on its stack.
//
// The idle gate is handed no input and is switched OFF, so every case below records
// whatever it is given. ⛔ THE PAUSE IS PINNED IN ITS OWN SUITE, NOT INCIDENTALLY HERE.
static TickLanePollCounts poll(const MockSlotReader&  reader,
                               uint32_t               liveTick,
                               DAttackState           machineState,
                               InputHistoryTickLanes& lanes)
{
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(
		reader, liveTick, machineState, std::nullopt, false, std::nullopt, std::nullopt,
		std::nullopt, inversion, lanes);
}

// ---------------------------------------------------------------------------
// THE CELL MODEL
// ---------------------------------------------------------------------------

TEST_CASE("Lane.ACellIsKeyedOnItsOwnTickAndARewriteTouchesNoNeighbour",
          "[CharacterViz][InputHistoryViz]")
{
	// The whole point of the per-tick shape: a lineage change is local to one tick. On a
	// run-length row the same change would ratchet the entire run and never come back.
	ProvenanceLane lane;

	CHECK(lane.record(10u, RowProvenanceSummary::Confirmed) == LaneWriteResult::RecordedCell);
	CHECK(lane.record(11u, RowProvenanceSummary::Confirmed) == LaneWriteResult::RecordedCell);
	CHECK(lane.record(12u, RowProvenanceSummary::Confirmed) == LaneWriteResult::RecordedCell);

	// A correction lands for the middle tick only.
	CHECK(lane.record(11u, RowProvenanceSummary::Resimulated) == LaneWriteResult::UpdatedCell);

	REQUIRE(lane.find(11u) != nullptr);
	CHECK(*lane.find(11u) == RowProvenanceSummary::Resimulated);

	// Counted, so no CHECK runs inside the loop: both neighbours must be untouched.
	std::size_t untouchedNeighbours = 0u;
	for (const uint32_t tick : { 10u, 12u })
	{
		if (lane.find(tick) != nullptr && *lane.find(tick) == RowProvenanceSummary::Confirmed)
			++untouchedNeighbours;
	}

	CHECK(untouchedNeighbours == 2u);
	CHECK(lane.storedCellCount() == 3u);
}

TEST_CASE("Lane.TheTwoRePollRulesDifferOnAnAlreadyRecordedTick",
          "[CharacterViz][InputHistoryViz]")
{
	// Provenance is REWRITABLE because a correction can arrive after the tick was first
	// written; machine state is not, because a later poll has no older sample to offer.
	ProvenanceLane rewritable;
	CHECK(rewritable.record(5u, RowProvenanceSummary::RanUnconfirmed)
		== LaneWriteResult::RecordedCell);

	// Same value re-presented: a no-op, and distinguishable from a real change.
	CHECK(rewritable.record(5u, RowProvenanceSummary::RanUnconfirmed)
		== LaneWriteResult::IgnoredDuplicate);
	CHECK(rewritable.record(5u, RowProvenanceSummary::Corrected) == LaneWriteResult::UpdatedCell);
	CHECK(*rewritable.find(5u) == RowProvenanceSummary::Corrected);

	brawlerInputHistoryVisualization::MachineStateLane firstWins;
	CHECK(firstWins.recordIfAbsent(5u, MachineStateCell::Attacking)
		== LaneWriteResult::RecordedCell);

	// A DIFFERENT value on an already-sampled tick is still refused: first sample wins.
	CHECK(firstWins.recordIfAbsent(5u, MachineStateCell::Idle)
		== LaneWriteResult::IgnoredDuplicate);
	CHECK(*firstWins.find(5u) == MachineStateCell::Attacking);
}

TEST_CASE("Lane.CapacityIsFixedAtTwoFortyAndOldTicksLeaveResidency",
          "[CharacterViz][InputHistoryViz]")
{
	// Fixed allocation is the acceptance criterion, and residency is what replaces the
	// eviction pass a resizable ring would need.
	CHECK(brawlerInputHistoryVisualization::kTickLaneCapacity == 240u);
	CHECK(ProvenanceLane::capacity() == 240u);

	ProvenanceLane lane;

	std::size_t recorded = 0u;
	for (uint32_t tick = 0u; tick < 240u; ++tick)
	{
		if (lane.record(tick, RowProvenanceSummary::Confirmed) == LaneWriteResult::RecordedCell)
			++recorded;
	}

	CHECK(recorded == 240u);
	CHECK(lane.storedCellCount() == 240u);
	REQUIRE(lane.find(0u) != nullptr);

	// Tick 240 shares tick 0's slot index. The oldest leaves; the count does not grow.
	CHECK(lane.record(240u, RowProvenanceSummary::Resimulated) == LaneWriteResult::RecordedCell);
	CHECK(lane.find(0u) == nullptr);
	REQUIRE(lane.find(240u) != nullptr);
	CHECK(lane.storedCellCount() == 240u);

	// A write for a tick a whole capacity behind the newest is refused outright, because
	// accepting it would land on a slot the display is currently drawing.
	CHECK(lane.record(0u, RowProvenanceSummary::ProvenanceLie) == LaneWriteResult::IgnoredStale);
	CHECK(lane.find(0u) == nullptr);
	CHECK(*lane.find(240u) == RowProvenanceSummary::Resimulated);
}

TEST_CASE("Lane.AGapLeavesHolesAndNothingIsCarriedForward",
          "[CharacterViz][InputHistoryViz]")
{
	// The ruling is to go with the gaps. A bar that invents plausible history is worse than
	// one with visible holes, because a fabricated cell would be read as evidence.
	InputHistoryTickLanes lanes;
	lanes.editMachineState().recordIfAbsent(10u, MachineStateCell::Attacking);
	lanes.editMachineState().recordIfAbsent(14u, MachineStateCell::Idle);
	lanes.noteAxisTick(14u);

	std::size_t holes = 0u;
	for (uint32_t tick = 11u; tick <= 13u; ++tick)
	{
		if (lanes.machineCellAt(tick) == MachineStateCell::NotSampled)
			++holes;
	}

	CHECK(holes == 3u);
	CHECK(lanes.machineCellAt(10u) == MachineStateCell::Attacking);
	CHECK(lanes.machineCellAt(14u) == MachineStateCell::Idle);

	// A tick past the newest sample is unsampled too, not the newest state extended.
	CHECK(lanes.machineCellAt(15u) == MachineStateCell::NotSampled);
}

TEST_CASE("MachineCell.EveryDAttackStateHasItsOwnCellAndNoneReadsAsUnsampled",
          "[CharacterViz][InputHistoryViz]")
{
	// A state that folded onto NotSampled would be indistinguishable from a hole, which is
	// the one confusion this lane exists to prevent.
	const unsigned stateCount = static_cast<unsigned>(kDAttackStateCount);

	CHECK(static_cast<unsigned>(brawlerInputHistoryVisualization::kMachineStateCellCount)
		== stateCount + 1u);
	CHECK(static_cast<unsigned>(DAttackState::HitFlinch) + 1u == stateCount);

	std::size_t sampled  = 0u;
	std::size_t distinct = 0u;
	for (unsigned left = 0u; left < stateCount; ++left)
	{
		const MachineStateCell leftCell =
			brawlerInputHistoryVisualization::machineStateCellOf(static_cast<DAttackState>(left));

		if (leftCell != MachineStateCell::NotSampled)
			++sampled;

		for (unsigned right = left + 1u; right < stateCount; ++right)
		{
			const MachineStateCell rightCell = brawlerInputHistoryVisualization::machineStateCellOf(
				static_cast<DAttackState>(right));

			if (leftCell != rightCell)
				++distinct;
		}
	}

	CHECK(sampled == stateCount);
	CHECK(distinct == 6u);     // all four-choose-two pairs, counted rather than spot-checked

	// Outside the enumeration -- what a torn cross-thread read looks like -- is a hole.
	CHECK(brawlerInputHistoryVisualization::machineStateCellOf(
		static_cast<DAttackState>(stateCount)) == MachineStateCell::NotSampled);
}

TEST_CASE("LaneRetention.TheRetainedTickCountIsClampedAndTheRingIsNot",
          "[CharacterViz][InputHistoryViz]")
{
	using brawlerInputHistoryVisualization::clampRetainedLaneTicks;

	CHECK(brawlerInputHistoryVisualization::kTickLaneDefaultRetainedTicks == 120u);

	// Outside [1, 240] pulls to the nearer end; a zero or negative setting would otherwise
	// make the display draw nothing at all with no way to tell why.
	CHECK(clampRetainedLaneTicks(0) == 1u);
	CHECK(clampRetainedLaneTicks(-7) == 1u);
	CHECK(clampRetainedLaneTicks(241) == 240u);
	CHECK(clampRetainedLaneTicks(1000000) == 240u);

	// Inside it, honoured exactly -- 180 is the value the setting exists to make reachable.
	CHECK(clampRetainedLaneTicks(1) == 1u);
	CHECK(clampRetainedLaneTicks(120) == 120u);
	CHECK(clampRetainedLaneTicks(180) == 180u);
	CHECK(clampRetainedLaneTicks(240) == 240u);

	// ⛔ AND THE ALLOCATION IS UNMOVED BY ANY OF IT: retention bounds the read, not the ring.
	CHECK(ProvenanceLane::capacity() == brawlerInputHistoryVisualization::kTickLaneCapacity);
}

// ---------------------------------------------------------------------------
// THE POLL
// ---------------------------------------------------------------------------

TEST_CASE("LanePoll.TheProvenanceLaneIsFedByTheReusedFourArmJoin",
          "[CharacterViz][InputHistoryViz]")
{
	// Every expectation here is captureSummaryOf's own answer, never a literal: if the lane
	// stopped going through the join, these would still be four values but the wrong four.
	//
	// ⭐ EDITED, same reason as ruling Q6: the NoSlot tick must sit NEWER than every resident
	// tick in the fixture, not older -- a NoSlot tick below the window's own oldest resident
	// tick classifies Evicted (files nothing), which is the wipe/hole shape Q6 already ruled
	// unreachable outside a wipe, not what this case means to exercise.
	MockSlotReader reader;
	reader.setRef(91u, AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setRef(92u, AppliedCaptureRef{ AppliedCaptureRefKind::Sentinel, kNoInputCaptureTick });
	reader.setRef(93u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 93u });
	reader.setRef(94u, AppliedCaptureRef{ AppliedCaptureRefKind::NoSlot, kNoInputCaptureTick });

	InputHistoryTickLanes lanes;
	poll(reader, 94u, DAttackState::Idle, lanes);

	const std::optional<SlotStateProvenance> noLineage{};

	REQUIRE(lanes.provenanceAt(94u) != nullptr);
	REQUIRE(lanes.provenanceAt(93u) != nullptr);
	CHECK(*lanes.provenanceAt(94u) == brawlerInputHistoryVisualization::captureSummaryOf(
		AppliedCaptureRefKind::NoSlot, noLineage));
	CHECK(*lanes.provenanceAt(91u) == brawlerInputHistoryVisualization::captureSummaryOf(
		AppliedCaptureRefKind::NoRef, noLineage));
	CHECK(*lanes.provenanceAt(92u) == brawlerInputHistoryVisualization::captureSummaryOf(
		AppliedCaptureRefKind::Sentinel, noLineage));
	CHECK(*lanes.provenanceAt(93u) == brawlerInputHistoryVisualization::captureSummaryOf(
		AppliedCaptureRefKind::Ref, noLineage));

	// Counted: all six unordered pairs distinct. That number is what fails if any two arms
	// collapse on the way to the lane, whichever two they are.
	const RowProvenanceSummary cells[] = { *lanes.provenanceAt(94u), *lanes.provenanceAt(91u),
	                                       *lanes.provenanceAt(92u), *lanes.provenanceAt(93u) };
	std::size_t distinctPairs = 0u;
	for (std::size_t left = 0u; left < 4u; ++left)
	{
		for (std::size_t right = left + 1u; right < 4u; ++right)
		{
			if (cells[left] != cells[right])
				++distinctPairs;
		}
	}

	CHECK(distinctPairs == 6u);
}

TEST_CASE("LanePoll.AReplayedLineageReachesTheLaneThroughTheUncorrectedArm",
          "[CharacterViz][InputHistoryViz]")
{
	// Protect-all-corrected means a replay never writes a slot carrying a correction, so
	// NoRef is the ONLY arm a Replayed lineage can reach a reader through. A lane fed by a
	// join that consulted lineage on Ref alone would make Resimulated structurally
	// unreachable while Corrected still worked -- correct-looking and wrong.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);
	reader.setProvenance(70u, SlotStateProvenance::Replayed);

	InputHistoryTickLanes lanes;
	poll(reader, 100u, DAttackState::Idle, lanes);

	REQUIRE(lanes.provenanceAt(70u) != nullptr);
	REQUIRE(lanes.provenanceAt(69u) != nullptr);
	CHECK(*lanes.provenanceAt(70u) == RowProvenanceSummary::Resimulated);
	CHECK(*lanes.provenanceAt(69u) == RowProvenanceSummary::RanUnconfirmed);
	CHECK(*lanes.provenanceAt(71u) == RowProvenanceSummary::RanUnconfirmed);
}

TEST_CASE("LanePoll.AnAuthorityNamedCaptureLandsOnItsCaptureTickNotItsAppliedTick",
          "[CharacterViz][InputHistoryViz]")
{
	// The inversion is what makes the lane a CAPTURE-tick axis rather than an applied-tick
	// one, and the two disagree exactly when a correction lands.
	MockSlotReader reader;
	reader.setRef(93u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 88u });
	reader.setProvenance(93u, SlotStateProvenance::AuthorityAdopted);

	InputHistoryTickLanes lanes;
	poll(reader, 93u, DAttackState::Idle, lanes);

	// Capture 88 carries the verdict even though applied tick 88's own observation was a
	// bare NoSlot: an authority-named join outranks an assumed same-tick one.
	REQUIRE(lanes.provenanceAt(88u) != nullptr);
	CHECK(*lanes.provenanceAt(88u) == RowProvenanceSummary::Corrected);

	// And nothing is filed under 93, because tick 93 ran somebody else's capture.
	CHECK(lanes.provenanceAt(93u) == nullptr);
}

TEST_CASE("LanePoll.ALateCorrectionRepaintsExactlyOneCell",
          "[CharacterViz][InputHistoryViz]")
{
	// On a row keyed by input identity this same correction would ratchet the whole run.
	// ⭐ THE HEADLINE PROPERTY, and the one the retired run-length row could not have.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);

	InputHistoryTickLanes    lanes;
	const TickLanePollCounts first = poll(reader, 100u, DAttackState::Idle, lanes);

	const uint32_t windowTicks =
		static_cast<uint32_t>(brawlerInputHistoryVisualization::kAppliedPollWindowTicks);

	CHECK(first.provenanceCellsRecorded == windowTicks);
	CHECK(first.provenanceCellsUpdated == 0u);

	// The correction arrives for ONE tick already recorded as an ordinary prediction.
	reader.setProvenance(70u, SlotStateProvenance::AuthorityAdopted);
	const TickLanePollCounts second = poll(reader, 100u, DAttackState::Idle, lanes);

	CHECK(second.provenanceCellsRecorded == 0u);
	CHECK(second.provenanceCellsUpdated == 1u);
	CHECK(second.provenanceCellsUnchanged == windowTicks - 1u);

	// Counted independently of the poll's own bookkeeping: exactly one cell in the whole
	// window now reads anything other than the prediction it started as.
	std::size_t changedCells = 0u;
	for (uint32_t tick = 101u - windowTicks; tick <= 100u; ++tick)
	{
		const RowProvenanceSummary* cell = lanes.provenanceAt(tick);
		if (cell != nullptr && *cell != RowProvenanceSummary::RanUnconfirmed)
			++changedCells;
	}

	CHECK(changedCells == 1u);
	REQUIRE(lanes.provenanceAt(70u) != nullptr);
	CHECK(*lanes.provenanceAt(70u) == RowProvenanceSummary::Corrected);
}

TEST_CASE("LanePoll.AnAlreadyObservedProvenanceSurvivesANoSlotReAskOfTheSameTick",
          "[CharacterViz][InputHistoryViz]")
{
	// The correction cache's own resident window and the poll's 60-tick window are each
	// anchored on a clock read at a slightly different moment, so a capture tick can be
	// evicted from the cache while still inside the poll's own window: the reader then
	// answers NoSlot for a tick it used to answer for. NoSlot cannot tell "pressed, not
	// yet run" from "evicted, previously observed" -- a plain WRITES-NEVER-MERGES lane
	// lets that re-ask overwrite a real observation with Pending, which is the user's
	// report: a cell reads sky blue and stays that way, 20/50/100 slots behind the
	// frontier, until it scrolls out.
	MockSlotReader reader;
	reader.setRef(50u, AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setProvenance(50u, SlotStateProvenance::AuthorityAgreedKeptPrediction);

	InputHistoryTickLanes lanes;
	poll(reader, 100u, DAttackState::Idle, lanes);

	REQUIRE(lanes.provenanceAt(50u) != nullptr);
	REQUIRE(*lanes.provenanceAt(50u) == RowProvenanceSummary::Confirmed);

	// Tick 50 is still inside the new [46,105] window, but its cache slot is now reported
	// evicted -- the same tick that answered NoRef above now answers NoSlot.
	reader.setRef(50u, AppliedCaptureRef{ AppliedCaptureRefKind::NoSlot, kNoInputCaptureTick });
	poll(reader, 105u, DAttackState::Idle, lanes);

	REQUIRE(lanes.provenanceAt(50u) != nullptr);
	CHECK(*lanes.provenanceAt(50u) == RowProvenanceSummary::Confirmed);
}

TEST_CASE("LanePoll.ABackwardHardResyncRepaintsAResidentCellBackToPending",
          "[CharacterViz][InputHistoryViz]")
{
	// The retired T29 guard used cell CONTENT as a proxy for "the slot once existed", and
	// this is the one case where that proxy is WRONG: a backward hard resync can leave a
	// real value resident for a tick that is genuinely pending again. classifyNoSlot
	// answers from the window's own bounds, not the cell's content, so it repaints.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);
	reader.setRef(70u, AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setProvenance(70u, SlotStateProvenance::AuthorityAgreedKeptPrediction);

	InputHistoryTickLanes lanes;
	poll(reader, 100u, DAttackState::Idle, lanes);

	REQUIRE(lanes.provenanceAt(70u) != nullptr);
	REQUIRE(*lanes.provenanceAt(70u) == RowProvenanceSummary::Confirmed);

	// The ring wipes back to a much older frontier (tick 60); tick 70 is now genuinely
	// pending again even though its cell still holds the earlier Confirmed observation.
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoSlot, kNoInputCaptureTick });
	reader.setRef(60u, AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setRef(70u, AppliedCaptureRef{ AppliedCaptureRefKind::NoSlot, kNoInputCaptureTick });
	poll(reader, 100u, DAttackState::Idle, lanes);

	REQUIRE(lanes.provenanceAt(70u) != nullptr);
	CHECK(*lanes.provenanceAt(70u) == RowProvenanceSummary::Pending);
}

TEST_CASE("LanePoll.AGenuineFrontierPendingIsStillProducedForATickNeverObservedBefore",
          "[CharacterViz][InputHistoryViz]")
{
	// ⭐ EDITED, ruling Q6: the NoSlot tick now sits at the window's NEWEST end, which is
	// the only shape a real ring can produce for a tick nobody has reported on yet. The
	// old shape (a hole at 104 while 105 stayed resident) is pinned by the sibling case
	// below instead, since a real ring can only produce it via a wipe.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);
	reader.setRef(105u, AppliedCaptureRef{ AppliedCaptureRefKind::NoSlot, kNoInputCaptureTick });

	InputHistoryTickLanes lanes;
	poll(reader, 105u, DAttackState::Idle, lanes);

	REQUIRE(lanes.provenanceAt(105u) != nullptr);
	CHECK(*lanes.provenanceAt(105u) == RowProvenanceSummary::Pending);
}

TEST_CASE("LanePoll.ANoSlotHoleInsideAnOtherwiseResidentWindowFilesNothing",
          "[CharacterViz][InputHistoryViz]")
{
	// The PRE-Q6-edit shape of the test above: a hole at 104 while 105 (a NEWER tick) is
	// resident. On a real ring this is unreachable except via a wipe, and classifies
	// MissingInsideWindow -- the classifier files nothing rather than guessing Pending.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);
	reader.setRef(104u, AppliedCaptureRef{ AppliedCaptureRefKind::NoSlot, kNoInputCaptureTick });

	InputHistoryTickLanes lanes;
	poll(reader, 105u, DAttackState::Idle, lanes);

	CHECK(lanes.provenanceAt(104u) == nullptr);
}

TEST_CASE("LanePoll.TheMachineLaneSamplesTheLiveTickOnceAndARePollIsANoOp",
          "[CharacterViz][InputHistoryViz]")
{
	// The poll runs at RENDER rate, so the same sim tick is re-presented constantly. The
	// machine lane's rule is first-sample-wins, which is the opposite of the other lane's.
	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	const TickLanePollCounts first = poll(reader, 100u, DAttackState::Attacking, lanes);
	CHECK(first.machineCellsRecorded == 1u);
	CHECK(first.machineCellsIgnored == 0u);
	CHECK(lanes.machineCellAt(100u) == MachineStateCell::Attacking);

	const TickLanePollCounts second = poll(reader, 100u, DAttackState::Idle, lanes);
	CHECK(second.machineCellsRecorded == 0u);
	CHECK(second.machineCellsIgnored == 1u);
	CHECK(lanes.machineCellAt(100u) == MachineStateCell::Attacking);

	// The next sim tick is a new sample, and the tick between polls stayed a hole.
	poll(reader, 102u, DAttackState::GuardFlinch, lanes);
	CHECK(lanes.machineCellAt(102u) == MachineStateCell::GuardFlinch);
	CHECK(lanes.machineCellAt(101u) == MachineStateCell::NotSampled);
}

TEST_CASE("LanePoll.BothLanesShareOneTickAxisAndOneRetainedWindow",
          "[CharacterViz][InputHistoryViz]")
{
	// A vertical slice through the two bars must be the same capture tick in both, which is
	// only true if one window is derived once and both lanes are read through it.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);

	InputHistoryTickLanes lanes;
	poll(reader, 200u, DAttackState::Attacking, lanes);
	poll(reader, 201u, DAttackState::Idle, lanes);

	REQUIRE(lanes.hasAxis());
	CHECK(lanes.newestAxisTick() == 201u);

	const brawlerInputHistoryVisualization::PollWindow window =
		brawlerInputHistoryVisualization::retainedLaneWindow(lanes, 8u);

	CHECK(window.newestTick == 201u);
	CHECK(window.tickCount() == 8u);

	// Both lanes answer for every tick of that one window -- provenance for all eight,
	// machine state for the two that were live when a poll ran and no others.
	std::size_t provenanceCells = 0u;
	std::size_t machineSamples  = 0u;
	for (uint32_t offset = 0u; offset < window.tickCount(); ++offset)
	{
		const uint32_t tick = window.oldestTick + offset;

		if (lanes.provenanceAt(tick) != nullptr)
			++provenanceCells;

		if (lanes.machineCellAt(tick) != MachineStateCell::NotSampled)
			++machineSamples;
	}

	CHECK(provenanceCells == 8u);
	CHECK(machineSamples == 2u);
}

TEST_CASE("Lane.AnOrdinaryRunOfAdvancingPollsFilesNoAxisBreakAtAll",
          "[CharacterViz][InputHistoryViz]")
{
	// The steady state every other case in this file runs in, asserted for the one thing
	// none of them says out loud: the ordinary poll files NOTHING on the axis ledger.
	// Every poll steps the tick forward, so the one test there is never fires.
	MockSlotReader reader;
	reader.setDefaultRef(AppliedCaptureRef{ AppliedCaptureRefKind::NoRef, kNoInputCaptureTick });
	reader.setDefaultProvenance(SlotStateProvenance::Predicted);

	InputHistoryTickLanes lanes;

	constexpr uint32_t kFirstTick = 2000u;
	constexpr uint32_t kPolls     = 30u;

	uint32_t breaksSeen = 0u;
	for (uint32_t tick = kFirstTick; tick < kFirstTick + kPolls; ++tick)
	{
		const TickLanePollCounts counts = poll(reader, tick, DAttackState::Idle, lanes);
		breaksSeen += counts.axisBreaksBackward;
	}

	CHECK(breaksSeen == 0u);
	CHECK(lanes.gate().axisEventCount() == 0u);

	// And the axis stayed the identity mapping it starts as: no epoch was ever opened.
	REQUIRE(lanes.gate().laneTickOf(kFirstTick + kPolls - 1u).has_value());
	CHECK(*lanes.gate().laneTickOf(kFirstTick + kPolls - 1u) == kFirstTick + kPolls - 1u);
}

} // namespace inputhistorylanetests

#endif // WITH_LOW_LEVEL_TESTS
