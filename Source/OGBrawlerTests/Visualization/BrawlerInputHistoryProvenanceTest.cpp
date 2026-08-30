// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the pressed-vs-ran join: the inversion of the correction cache's forward map
// and the fold that reduces a span of capture ticks to one summary.
//
// NO ROW STORES A SUMMARY. The input row is run-length compressed on INPUT identity and
// lineage varies per tick, so a stored per-row summary saturates to its worst tick and
// never recovers. These functions are the join, unchanged; their readers are these tests
// until a tick-keyed one exists.
//
// WHAT THIS SUITE IS REALLY GUARDING is a pair of collapses that a careless fold makes
// silently. AppliedCaptureRefKind has FOUR arms, and NoRef -- a slot that exists with no
// correction on it -- is the ORDINARY client answer; folding it into NoSlot would report
// the majority of ticks as "not yet run", and folding it into Sentinel would report them
// as unknown. Separately, SlotStateProvenance::ReplayedOverCorrection must reach the
// display rather than being folded away: zero occurrences is the assertion, not the
// omission, and a fold that cannot express the lie cannot report the regression.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

#include "OGBrawler/BrawlerInputHistoryVisualization.h"
#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryvizjointests
{

using brawlerInputHistoryVisualization::AppendResult;
using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::AppliedSlotObservation;
using brawlerInputHistoryVisualization::CaptureJoin;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryRow;
using brawlerInputHistoryVisualization::InputHistoryRowRing;
using brawlerInputHistoryVisualization::RowProvenanceSummary;
using brawlerInputHistoryVisualization::captureSummaryOf;
using brawlerInputHistoryVisualization::kAppliedCaptureInversionCapacity;
using brawlerInputHistoryVisualization::kRowProvenanceSummaryCount;
using brawlerInputHistoryVisualization::residentRowSummary;
using brawlerInputHistoryVisualization::rowProvenanceRank;
using brawlerInputHistoryVisualization::summaryOfSlotProvenance;
using brawlerInputHistoryVisualization::worseRowProvenance;

// The four arms, in declaration order, so a sweep cannot quietly skip one.
constexpr AppliedCaptureRefKind kRefKinds[] = {
	AppliedCaptureRefKind::NoSlot,
	AppliedCaptureRefKind::NoRef,
	AppliedCaptureRefKind::Sentinel,
	AppliedCaptureRefKind::Ref,
};

// No lineage at all -- what the authority answers, and what an absent slot answers.
constexpr std::optional<SlotStateProvenance> kNoLineage = std::nullopt;

static RowProvenanceSummary summaryOfKind(AppliedCaptureRefKind kind)
{
	return captureSummaryOf(kind, kNoLineage);
}

// One observation as the two seams answer it for a resident slot.
static AppliedSlotObservation observation(uint32_t              appliedTick,
                                          AppliedCaptureRefKind kind,
                                          uint32_t              captureTick = kNoInputCaptureTick,
                                          std::optional<SlotStateProvenance> provenance = std::nullopt)
{
	AppliedSlotObservation observed;
	observed.appliedTick = appliedTick;
	observed.ref         = AppliedCaptureRef{ kind, captureTick };
	observed.provenance  = provenance;
	return observed;
}

// A one-row ring spanning [firstTick, firstTick + tickCount), all one held state.
static InputHistoryRowRing ringWithOneRow(uint32_t firstTick, uint32_t tickCount)
{
	InputHistoryRowRing ring;
	for (uint32_t offset = 0u; offset < tickCount; ++offset)
	{
		ring.appendCapture(firstTick + offset, DirectionBucket::Forward, 0u);
	}
	return ring;
}

} // namespace inputhistoryvizjointests

// ---------------------------------------------------------------------------
// The four-arm join. One outcome per arm, and no two of them equal.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Join.EachAppliedCaptureRefKindArmFoldsToItsOwnOutcome",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	// Pressed, not yet run -- the normal state inside the input-delay window.
	CHECK(summaryOfKind(AppliedCaptureRefKind::NoSlot) == RowProvenanceSummary::Pending);

	// The slot exists, nothing corrected it. The ordinary client steady state.
	CHECK(summaryOfKind(AppliedCaptureRefKind::NoRef) == RowProvenanceSummary::RanUnconfirmed);

	// A correction landed naming no capture: documented ambiguous, never "not run".
	CHECK(summaryOfKind(AppliedCaptureRefKind::Sentinel) == RowProvenanceSummary::Unknown);
	CHECK(summaryOfKind(AppliedCaptureRefKind::Sentinel) != RowProvenanceSummary::Pending);

	// The authority named the capture but no lineage came back with it.
	CHECK(summaryOfKind(AppliedCaptureRefKind::Ref) == RowProvenanceSummary::LineageUnavailable);

	// ALL SIX PAIRS, counted: this is the assertion that fails if any two arms collapse.
	unsigned int distinctPairs = 0u;
	for (std::size_t left = 0u; left < 4u; ++left)
	{
		for (std::size_t right = left + 1u; right < 4u; ++right)
		{
			if (summaryOfKind(kRefKinds[left]) != summaryOfKind(kRefKinds[right]))
				++distinctPairs;
		}
	}

	CHECK(distinctPairs == 6u);
}

// ---------------------------------------------------------------------------
// The majority case, singled out: NoRef is neither of its neighbours.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Join.NoRefIsTreatedAsNeitherNoSlotNorSentinel",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	CHECK(summaryOfKind(AppliedCaptureRefKind::NoRef) != summaryOfKind(AppliedCaptureRefKind::NoSlot));
	CHECK(summaryOfKind(AppliedCaptureRefKind::NoRef) != summaryOfKind(AppliedCaptureRefKind::Sentinel));

	// It must not be reported as pending: the tick DID run, as a prediction.
	CHECK(summaryOfKind(AppliedCaptureRefKind::NoRef) != RowProvenanceSummary::Pending);
	CHECK(summaryOfKind(AppliedCaptureRefKind::NoRef) != RowProvenanceSummary::Unknown);

	// And the separation survives a lineage byte arriving with the observation.
	const std::optional<SlotStateProvenance> predicted = SlotStateProvenance::Predicted;
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoRef, predicted) == RowProvenanceSummary::RanUnconfirmed);
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoSlot, predicted) == RowProvenanceSummary::Pending);
	CHECK(captureSummaryOf(AppliedCaptureRefKind::Sentinel, predicted) == RowProvenanceSummary::Unknown);
}

// ---------------------------------------------------------------------------
// The whole lineage alphabet, swept: a seventh enumerator fails here.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Join.EverySlotStateProvenanceValueHasItsOwnSummary",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	CHECK(summaryOfSlotProvenance(SlotStateProvenance::Empty) == RowProvenanceSummary::NoStateWritten);
	CHECK(summaryOfSlotProvenance(SlotStateProvenance::Predicted) == RowProvenanceSummary::RanUnconfirmed);
	CHECK(summaryOfSlotProvenance(SlotStateProvenance::AuthorityAdopted) == RowProvenanceSummary::Corrected);
	CHECK(summaryOfSlotProvenance(SlotStateProvenance::AuthorityAgreedKeptPrediction) == RowProvenanceSummary::Confirmed);
	CHECK(summaryOfSlotProvenance(SlotStateProvenance::Replayed) == RowProvenanceSummary::Resimulated);
	CHECK(summaryOfSlotProvenance(SlotStateProvenance::ReplayedOverCorrection) == RowProvenanceSummary::ProvenanceLie);

	// Sweeping the enum's OWN count is what makes an unextended fold fail rather than
	// silently landing on the out-of-range default.
	unsigned int mapped   = 0u;
	unsigned int distinct = 0u;
	for (std::uint8_t left = 0u; left < kSlotStateProvenanceCount; ++left)
	{
		const RowProvenanceSummary summary =
			summaryOfSlotProvenance(static_cast<SlotStateProvenance>(left));

		if (summary != RowProvenanceSummary::Unknown)
			++mapped;

		for (std::uint8_t right = static_cast<std::uint8_t>(left + 1u); right < kSlotStateProvenanceCount; ++right)
		{
			if (summary != summaryOfSlotProvenance(static_cast<SlotStateProvenance>(right)))
				++distinct;
		}
	}

	CHECK(mapped == kSlotStateProvenanceCount);
	CHECK(distinct == 15u);
	CHECK(kSlotStateProvenanceCount == 6u);
}

// ---------------------------------------------------------------------------
// nullopt is absence, not a lineage value -- the seam answers it on the authority.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Join.AMissingLineageIsItsOwnFactAndNeverAValue",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	const RowProvenanceSummary refWithoutLineage = captureSummaryOf(AppliedCaptureRefKind::Ref, kNoLineage);
	CHECK(refWithoutLineage == RowProvenanceSummary::LineageUnavailable);

	// It is not silently read as the zero enumerator, nor as any other lineage value.
	unsigned int differsFromEveryLineage = 0u;
	for (std::uint8_t value = 0u; value < kSlotStateProvenanceCount; ++value)
	{
		const std::optional<SlotStateProvenance> present = static_cast<SlotStateProvenance>(value);
		if (captureSummaryOf(AppliedCaptureRefKind::Ref, present) != refWithoutLineage)
			++differsFromEveryLineage;
	}

	CHECK(differsFromEveryLineage == kSlotStateProvenanceCount);
	CHECK(refWithoutLineage != summaryOfSlotProvenance(SlotStateProvenance::Empty));

	// The uncorrected arm answers its own fact too, and it is not the Ref one.
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoRef, kNoLineage) == RowProvenanceSummary::RanUnconfirmed);
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoRef, kNoLineage) != refWithoutLineage);

	// An absent slot has no lineage to lose, so its arm answers pending either way.
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoSlot, kNoLineage) == RowProvenanceSummary::Pending);
}

// ---------------------------------------------------------------------------
// A replay can only ever be seen through the UNCORRECTED arm.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Join.AReplayedLineageSurfacesThroughTheUncorrectedArm",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	// Protect-all-corrected means a replay never writes a slot whose correction bit is
	// set, so NoRef is the only arm that can carry a replayed lineage at all.
	const std::optional<SlotStateProvenance> replayed = SlotStateProvenance::Replayed;
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoRef, replayed) == RowProvenanceSummary::Resimulated);
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoRef, replayed) != RowProvenanceSummary::RanUnconfirmed);

	// A resimulated tick outranks both authority verdicts, so a mixed row reads as resimulated.
	CHECK(rowProvenanceRank(RowProvenanceSummary::Resimulated) > rowProvenanceRank(RowProvenanceSummary::Corrected));
	CHECK(rowProvenanceRank(RowProvenanceSummary::Resimulated) > rowProvenanceRank(RowProvenanceSummary::Confirmed));
	CHECK(worseRowProvenance(RowProvenanceSummary::Confirmed, RowProvenanceSummary::Resimulated)
	      == RowProvenanceSummary::Resimulated);
}

// ---------------------------------------------------------------------------
// The provenance lie shouts. It is representable, and it wins every fold.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Join.TheProvenanceLieSurfacesRatherThanBeingFoldedAway",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	const std::optional<SlotStateProvenance> lie = SlotStateProvenance::ReplayedOverCorrection;
	CHECK(captureSummaryOf(AppliedCaptureRefKind::Ref, lie) == RowProvenanceSummary::ProvenanceLie);
	CHECK(captureSummaryOf(AppliedCaptureRefKind::NoRef, lie) == RowProvenanceSummary::ProvenanceLie);

	// Top of the ladder: nothing can outrank it, so no fold can hide it.
	unsigned int outranksEverythingElse = 0u;
	for (std::uint8_t value = 0u; value < kRowProvenanceSummaryCount; ++value)
	{
		const RowProvenanceSummary other = static_cast<RowProvenanceSummary>(value);
		if (worseRowProvenance(other, RowProvenanceSummary::ProvenanceLie) == RowProvenanceSummary::ProvenanceLie)
			++outranksEverythingElse;
	}

	CHECK(outranksEverythingElse == kRowProvenanceSummaryCount);

	// And one lying tick in a sixty-tick row is enough to make the row shout.
	InputHistoryRowRing     ring = ringWithOneRow(100u, 60u);
	AppliedCaptureInversion inversion;
	for (uint32_t tick = 100u; tick < 160u; ++tick)
	{
		inversion.observe(observation(tick, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
		                              SlotStateProvenance::AuthorityAgreedKeptPrediction));
	}
	inversion.observe(observation(137u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick, lie));

	CHECK(residentRowSummary(inversion, ring.newest()) == RowProvenanceSummary::ProvenanceLie);
}

// ---------------------------------------------------------------------------
// The inversion proper: the Ref arm re-files under the capture the authority named.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Inversion.AnAuthorityNamedCaptureIsRefiledUnderItsCaptureTick",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	AppliedCaptureInversion inversion;
	CHECK(inversion.empty());

	// The authority applied capture 97 at tick 100 -- a three-tick displacement.
	REQUIRE(inversion.observe(observation(100u, AppliedCaptureRefKind::Ref, 97u,
	                                      SlotStateProvenance::AuthorityAdopted)));
	CHECK(inversion.size() == 1u);

	const CaptureJoin* joined = inversion.find(97u);
	REQUIRE(joined != nullptr);
	CHECK(joined->captureTick == 97u);
	CHECK(joined->appliedTick == 100u);
	CHECK(joined->authorityNamed);
	CHECK(joined->summary == RowProvenanceSummary::Corrected);

	// Nothing was filed under the APPLIED tick: tick 100 ran somebody else's capture.
	CHECK(inversion.find(100u) == nullptr);
}

// ---------------------------------------------------------------------------
// Every other arm names no capture, so it speaks for the tick it was asked about.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Inversion.AnUnnamedObservationSpeaksForTheTickItWasAskedAbout",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	AppliedCaptureInversion inversion;
	REQUIRE(inversion.observe(observation(10u, AppliedCaptureRefKind::NoSlot)));
	REQUIRE(inversion.observe(observation(11u, AppliedCaptureRefKind::NoRef)));
	REQUIRE(inversion.observe(observation(12u, AppliedCaptureRefKind::Sentinel)));
	CHECK(inversion.size() == 3u);

	REQUIRE(inversion.find(10u) != nullptr);
	CHECK(inversion.find(10u)->summary == RowProvenanceSummary::Pending);
	CHECK_FALSE(inversion.find(10u)->authorityNamed);
	CHECK(inversion.find(11u)->summary == RowProvenanceSummary::RanUnconfirmed);
	CHECK(inversion.find(12u)->summary == RowProvenanceSummary::Unknown);

	// The sentinel capture tick is UINT32_MAX and must never become a key of its own.
	CHECK(inversion.find(kNoInputCaptureTick) == nullptr);
}

// ---------------------------------------------------------------------------
// Collision policy: the authority's own join beats the same-tick assumption.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Inversion.AnAuthorityNamedJoinOutranksAnAssumedSameTickOne",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	SECTION("the assumed entry is observed first")
	{
		AppliedCaptureInversion inversion;
		REQUIRE(inversion.observe(observation(97u, AppliedCaptureRefKind::NoRef)));
		REQUIRE(inversion.observe(observation(100u, AppliedCaptureRefKind::Ref, 97u,
		                                      SlotStateProvenance::AuthorityAdopted)));

		REQUIRE(inversion.size() == 1u);
		REQUIRE(inversion.find(97u) != nullptr);
		CHECK(inversion.find(97u)->appliedTick == 100u);
		CHECK(inversion.find(97u)->authorityNamed);
		CHECK(inversion.find(97u)->summary == RowProvenanceSummary::Corrected);
	}

	SECTION("the authority-named entry is observed first")
	{
		AppliedCaptureInversion inversion;
		REQUIRE(inversion.observe(observation(100u, AppliedCaptureRefKind::Ref, 97u,
		                                      SlotStateProvenance::AuthorityAdopted)));
		REQUIRE(inversion.observe(observation(97u, AppliedCaptureRefKind::NoRef)));

		REQUIRE(inversion.size() == 1u);
		CHECK(inversion.find(97u)->appliedTick == 100u);
		CHECK(inversion.find(97u)->summary == RowProvenanceSummary::Corrected);
	}

	SECTION("two unnamed observations of one tick fold worst-wins")
	{
		AppliedCaptureInversion inversion;
		REQUIRE(inversion.observe(observation(50u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
		                                      SlotStateProvenance::Predicted)));
		REQUIRE(inversion.observe(observation(50u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
		                                      SlotStateProvenance::Replayed)));

		REQUIRE(inversion.size() == 1u);
		CHECK(inversion.find(50u)->summary == RowProvenanceSummary::Resimulated);
	}
}

// ---------------------------------------------------------------------------
// The window is one correction cache's worth, and it says so when it is full.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.Inversion.TheWindowMatchesTheResidentCorrectionCache",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	// Against the cache's OWN constant, not a second literal that happens to agree.
	CHECK(kAppliedCaptureInversionCapacity == StateCorrectionCache<int, int>::StateBufferSize);
	CHECK(AppliedCaptureInversion::capacity() == kAppliedCaptureInversionCapacity);

	AppliedCaptureInversion inversion;
	unsigned int            accepted = 0u;
	for (uint32_t tick = 0u; tick < kAppliedCaptureInversionCapacity; ++tick)
	{
		if (inversion.observe(observation(tick, AppliedCaptureRefKind::NoRef)))
			++accepted;
	}

	CHECK(accepted == kAppliedCaptureInversionCapacity);
	CHECK(inversion.size() == kAppliedCaptureInversionCapacity);

	// One past the window is refused rather than overwriting a resident entry.
	CHECK_FALSE(inversion.observe(observation(9000u, AppliedCaptureRefKind::NoRef)));
	CHECK(inversion.size() == kAppliedCaptureInversionCapacity);

	inversion.clear();
	CHECK(inversion.empty());
}

// ---------------------------------------------------------------------------
// The ladder is the declaration order, and the fold is a max over it.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowFold.TheLadderIsTheDeclarationOrderAndTheFoldIsAMaxOverIt",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	CHECK(kRowProvenanceSummaryCount == 9u);
	CHECK(rowProvenanceRank(RowProvenanceSummary::Unknown) == 0u);
	CHECK(rowProvenanceRank(RowProvenanceSummary::ProvenanceLie) == kRowProvenanceSummaryCount - 1u);

	// Every ordered pair, counted once: commutative, idempotent, and never below either.
	unsigned int commutative = 0u;
	unsigned int idempotent  = 0u;
	unsigned int neverBelow  = 0u;
	for (std::uint8_t left = 0u; left < kRowProvenanceSummaryCount; ++left)
	{
		const RowProvenanceSummary a = static_cast<RowProvenanceSummary>(left);
		if (worseRowProvenance(a, a) == a)
			++idempotent;

		for (std::uint8_t right = 0u; right < kRowProvenanceSummaryCount; ++right)
		{
			const RowProvenanceSummary b = static_cast<RowProvenanceSummary>(right);
			if (worseRowProvenance(a, b) == worseRowProvenance(b, a))
				++commutative;

			if (rowProvenanceRank(worseRowProvenance(a, b)) >= rowProvenanceRank(a)
			 && rowProvenanceRank(worseRowProvenance(a, b)) >= rowProvenanceRank(b))
				++neverBelow;
		}
	}

	CHECK(idempotent == kRowProvenanceSummaryCount);
	CHECK(commutative == 81u);
	CHECK(neverBelow == 81u);

	// Unknown is the identity, which is what lets an unjoined row carry the default.
	CHECK(worseRowProvenance(RowProvenanceSummary::Unknown, RowProvenanceSummary::Pending)
	      == RowProvenanceSummary::Pending);
}

// ---------------------------------------------------------------------------
// A MIXED row reads as its worst tick, not its first, last or commonest.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowFold.WorstCaseWinsAcrossAMixedRow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	// One row over capture ticks 20..29.
	InputHistoryRowRing ring = ringWithOneRow(20u, 10u);
	REQUIRE(ring.size() == 1u);
	REQUIRE(ring.newest().tickCount == 10u);

	AppliedCaptureInversion inversion;
	for (uint32_t tick = 20u; tick < 30u; ++tick)
	{
		inversion.observe(observation(tick, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
		                              SlotStateProvenance::AuthorityAgreedKeptPrediction));
	}

	// Nine confirmed ticks and one corrected: the row must read corrected.
	inversion.observe(observation(26u, AppliedCaptureRefKind::Ref, 26u,
	                              SlotStateProvenance::AuthorityAdopted));
	CHECK(residentRowSummary(inversion, ring.newest()) == RowProvenanceSummary::Corrected);

	// Add one resimulated tick and the row moves up again, not back down.
	inversion.observe(observation(22u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
	                              SlotStateProvenance::Replayed));
	CHECK(residentRowSummary(inversion, ring.newest()) == RowProvenanceSummary::Resimulated);

	// A tick just outside the row's span contributes nothing to it.
	AppliedCaptureInversion outside;
	outside.observe(observation(19u, AppliedCaptureRefKind::Ref, 19u, SlotStateProvenance::ReplayedOverCorrection));
	outside.observe(observation(30u, AppliedCaptureRefKind::Ref, 30u, SlotStateProvenance::ReplayedOverCorrection));
	CHECK(residentRowSummary(outside, ring.newest()) == RowProvenanceSummary::Unknown);
}

// ---------------------------------------------------------------------------
// ⛔ A READER THAT CARRIES A SUMMARY FORWARD MUST MERGE MONOTONELY, because the resident
//   window SCROLLS: a plain assignment erases a fact whose slot has since aged out.
//
// This is the shape of the merge, not of the display. What must never carry it forward is
// an INPUT row, whose span is a run of identical presses and not a tick.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowFold.CarryingASummaryAcrossAScrollingWindowIsMonotone",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	const InputHistoryRowRing ring    = ringWithOneRow(40u, 5u);
	const InputHistoryRow&    span    = ring.newest();
	RowProvenanceSummary      carried = RowProvenanceSummary::Unknown;

	AppliedCaptureInversion corrected;
	corrected.observe(observation(42u, AppliedCaptureRefKind::Ref, 42u, SlotStateProvenance::AuthorityAdopted));

	carried = worseRowProvenance(carried, residentRowSummary(corrected, span));
	CHECK(carried == RowProvenanceSummary::Corrected);

	// Re-polling the same window changes nothing.
	carried = worseRowProvenance(carried, residentRowSummary(corrected, span));
	CHECK(carried == RowProvenanceSummary::Corrected);

	// Once the corrected slot ages out, a later poll must not downgrade what is carried
	// back to what the empty window alone would say.
	const AppliedCaptureInversion agedOut;
	carried = worseRowProvenance(carried, residentRowSummary(agedOut, span));
	CHECK(carried == RowProvenanceSummary::Corrected);

	// A weaker window is also a no-op; only a worse one moves it.
	AppliedCaptureInversion weaker;
	weaker.observe(observation(41u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
	                           SlotStateProvenance::Predicted));
	carried = worseRowProvenance(carried, residentRowSummary(weaker, span));
	CHECK(carried == RowProvenanceSummary::Corrected);

	AppliedCaptureInversion worse;
	worse.observe(observation(43u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
	                          SlotStateProvenance::Replayed));
	carried = worseRowProvenance(carried, residentRowSummary(worse, span));
	CHECK(carried == RowProvenanceSummary::Resimulated);
}

// ---------------------------------------------------------------------------
// Never joined stays Unknown -- which is NOT "pressed but not yet run".
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowFold.ARowWithNoResidentJoinStaysUnknown",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	InputHistoryRowRing           ring = ringWithOneRow(70u, 4u);
	const AppliedCaptureInversion empty;

	CHECK(residentRowSummary(empty, ring.newest()) == RowProvenanceSummary::Unknown);
	CHECK(residentRowSummary(empty, ring.newest()) != RowProvenanceSummary::Pending);

	// A row with no ticks cannot be joined and must not wrap its span to UINT32_MAX.
	const InputHistoryRow unopened;
	CHECK(unopened.tickCount == 0u);
	AppliedCaptureInversion everywhere;
	everywhere.observe(observation(0u, AppliedCaptureRefKind::Ref, 0u, SlotStateProvenance::ReplayedOverCorrection));
	CHECK(residentRowSummary(everywhere, unopened) == RowProvenanceSummary::Unknown);
}

// ---------------------------------------------------------------------------
// Every row in a multi-row ring is joined, oldest first, on its own span.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowFold.EachRowInTheRingIsJoinedOnItsOwnSpan",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizjointests;

	// Three rows: 0..2 forward, 3..5 back, 6..8 forward again.
	InputHistoryRowRing ring;
	for (uint32_t tick = 0u; tick < 9u; ++tick)
	{
		const DirectionBucket direction =
			(tick >= 3u && tick < 6u) ? DirectionBucket::Back : DirectionBucket::Forward;
		ring.appendCapture(tick, direction, 0u);
	}

	REQUIRE(ring.size() == 3u);

	AppliedCaptureInversion inversion;
	inversion.observe(observation(1u, AppliedCaptureRefKind::NoSlot));
	inversion.observe(observation(4u, AppliedCaptureRefKind::Ref, 4u, SlotStateProvenance::AuthorityAdopted));
	inversion.observe(observation(7u, AppliedCaptureRefKind::NoRef, kNoInputCaptureTick,
	                              SlotStateProvenance::Replayed));

	// One observation per row, and each row must pick up only the one inside its own span.
	CHECK(residentRowSummary(inversion, ring.at(0u)) == RowProvenanceSummary::Pending);
	CHECK(residentRowSummary(inversion, ring.at(1u)) == RowProvenanceSummary::Corrected);
	CHECK(residentRowSummary(inversion, ring.at(2u)) == RowProvenanceSummary::Resimulated);

	// Reading a row leaves it exactly as it was; the fold takes it by const reference.
	CHECK(ring.at(1u).firstCaptureTick == 3u);
	CHECK(ring.at(1u).tickCount == 3u);
	CHECK(ring.at(1u).direction == DirectionBucket::Back);
}

#endif // WITH_LOW_LEVEL_TESTS
