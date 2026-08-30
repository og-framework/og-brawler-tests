// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins BrawlerInputHistoryVisualizationDelay.h -- the split of the client's effective
// input delay into its arms, and the four-way verdict on what the session relay floor is
// doing to them.
//
// WHAT THIS SUITE IS REALLY GUARDING is that the decomposition decomposes PRODUCTION
// rather than plausibly re-deriving it. A readout that agrees with itself and disagrees
// with the sim is worse than no readout: it would exonerate exactly the divergence it
// was built to expose. So the sweep asserts the header's effective delay against a real
// ReplicatedTierConsumer fed the same tier -- real code against real code -- and not
// merely against the header's own base and floor.
//
// The floor helpers themselves are NOT re-tested here: RelayDelayFloorTest.cpp in the
// og-simulation suite owns the clamp, the hard cap, the max and the advisory table. What
// is tested is the classification built on top of them, and the claim that the unfloored
// base this header reports is the one production would have used.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

#include "OGBrawler/BrawlerInputHistoryVisualization.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationDelay.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistorydelaytests
{

using brawlerInputHistoryVisualization::InputDelayBaseArm;
using brawlerInputHistoryVisualization::InputDelayDecomposition;
using brawlerInputHistoryVisualization::InputDelayWinner;
using brawlerInputHistoryVisualization::RelayFloorClass;
using brawlerInputHistoryVisualization::decomposeInputDelay;

// The pre-floor rule, written out ONCE here as the sweep's independent oracle. A copy in
// the header would drift from production unnoticed, and noticing is what a test is for;
// RelayDelayFloorTest.cpp keeps `preFloorTierInputDelayTicks` for exactly that reason.
// ⭐ THE SECOND COPY BELONGS ON THIS SIDE OF THE FENCE, NEVER IN THE HEADER.
int32_t expectedBaseTicks(bool tierKnown, int32_t tierIndex, const TimeConfig& cfg)
{
	if (!tierKnown)
	{
		return cfg.rttTierInputDelays[kMaxConnectionTierIndex];
	}

	const int32_t tier = clampConnectionTierIndex(tierIndex);
	return (tier == 0 && cfg.lanZeroDelayOverride) ? 0 : cfg.rttTierInputDelays[tier];
}

struct ExpectedVerdict
{
	RelayFloorClass  floorClass = RelayFloorClass::Inert;
	InputDelayWinner winner     = InputDelayWinner::Fallback;
};

// The classification table, transcribed from the header's contract and nothing else.
ExpectedVerdict expectedVerdict(bool tierKnown, int32_t floorTicks, int32_t baseTicks)
{
	const InputDelayWinner baseWinner =
		tierKnown ? InputDelayWinner::Tier : InputDelayWinner::Fallback;

	if (floorTicks == 0)
	{
		return ExpectedVerdict{ RelayFloorClass::Inert, baseWinner };
	}
	if (floorTicks < baseTicks)
	{
		return ExpectedVerdict{ RelayFloorClass::ActiveNotBinding, baseWinner };
	}
	if (floorTicks == baseTicks)
	{
		return ExpectedVerdict{ RelayFloorClass::Tie, InputDelayWinner::Equal };
	}
	return ExpectedVerdict{ RelayFloorClass::Binding, InputDelayWinner::Floor };
}

// ---------------------------------------------------------------------------
// 1. THE SWEEP -- the decomposition against the real client-side formula.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.TheSweepAgreesWithARealReplicatedTierConsumerOnEveryTierFloorAndLanSetting",
          "[CharacterViz][InputHistoryViz]")
{
	std::size_t combinations    = 0u;
	std::size_t tiersSwept      = 0u;
	std::size_t baseAgreed      = 0u;
	std::size_t effectiveAgreed = 0u;
	std::size_t consumerAgreed  = 0u;
	std::size_t classAgreed     = 0u;
	std::size_t winnerAgreed    = 0u;
	std::size_t requestCarried  = 0u;
	std::size_t capAgreed       = 0u;
	std::size_t advisoryAgreed  = 0u;

	std::size_t inertSeen      = 0u;
	std::size_t notBindingSeen = 0u;
	std::size_t tieSeen        = 0u;
	std::size_t bindingSeen    = 0u;

	for (int knownPass = 0; knownPass < 2; ++knownPass)
	{
		const bool tierKnown = (knownPass == 1);

		for (int lanPass = 0; lanPass < 2; ++lanPass)
		{
			const bool lanOverride = (lanPass == 1);

			for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
			{
				// Counted in ONE pass only, so the assertion below is about the tier
				// ladder's width rather than about how many passes wrap it.
				if (knownPass == 0 && lanPass == 0)
				{
					++tiersSwept;
				}

				TimeConfig cfg;
				cfg.lanZeroDelayOverride = lanOverride;
				cfg.relayDelayFloorTicks = 0;

				const int32_t base = expectedBaseTicks(tierKnown, tier, cfg);
				const int32_t cap  = relayDelayFloorHardCapTicks(cfg);

				const int32_t requestedFloors[] = {
					0, 1, base - 1, base, base + 1, cap, cap + 1, -1
				};

				for (const int32_t requested : requestedFloors)
				{
					cfg.relayDelayFloorTicks = requested;

					const InputDelayDecomposition reading =
						decomposeInputDelay(tierKnown, tier, cfg, 0, 0);

					// The cross-check's other end: the SAME formula production runs on
					// the client, fed the same tier, reading the same config.
					ReplicatedTierConsumer consumer(cfg);
					if (tierKnown)
					{
						consumer.onReplicatedTierReceived(tier);
					}

					const ExpectedVerdict expected =
						expectedVerdict(tierKnown, reading.floorTicks, base);

					++combinations;

					if (reading.baseTicks == base)
						++baseAgreed;
					if (reading.effectiveTicks == applyRelayDelayFloor(reading.baseTicks, cfg))
						++effectiveAgreed;
					if (reading.effectiveTicks == consumer.effectiveInputDelayTicks())
						++consumerAgreed;
					if (reading.floorClass == expected.floorClass)
						++classAgreed;
					if (reading.winner == expected.winner)
						++winnerAgreed;
					if (reading.floorRequested == requested)
						++requestCarried;
					if (reading.floorHardCap == cap)
						++capAgreed;
					if (reading.advisory == classifyRelayDelayFloor(cfg))
						++advisoryAgreed;

					switch (reading.floorClass)
					{
					case RelayFloorClass::Inert:            ++inertSeen;      break;
					case RelayFloorClass::ActiveNotBinding: ++notBindingSeen; break;
					case RelayFloorClass::Tie:              ++tieSeen;        break;
					case RelayFloorClass::Binding:          ++bindingSeen;    break;
					}
				}
			}
		}
	}

	// The ladder's width comes from the config array, so a fifth tier joins the sweep
	// without an edit here -- and a sweep that quietly stopped covering it fails.
	CHECK(tiersSwept == kConnectionTierCount);
	CHECK(combinations == 2u * 2u * kConnectionTierCount * 8u);

	CHECK(baseAgreed == combinations);
	CHECK(effectiveAgreed == combinations);
	CHECK(consumerAgreed == combinations);
	CHECK(classAgreed == combinations);
	CHECK(winnerAgreed == combinations);
	CHECK(requestCarried == combinations);
	CHECK(capAgreed == combinations);
	CHECK(advisoryAgreed == combinations);

	// ⛔ A SWEEP THAT REACHES ONLY ONE CLASS PROVES ONLY THAT ONE CLASS.
	CHECK(inertSeen > 0u);
	CHECK(notBindingSeen > 0u);
	CHECK(tieSeen > 0u);
	CHECK(bindingSeen > 0u);
}

// ---------------------------------------------------------------------------
// 2. THE DEGENERATE MODE -- a floor of 0 decides nothing, anywhere.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.AZeroFloorIsInertForEveryTierAndBothLanSettings",
          "[CharacterViz][InputHistoryViz]")
{
	std::size_t checked      = 0u;
	std::size_t inert        = 0u;
	std::size_t armWon       = 0u;
	std::size_t floorSilent  = 0u;

	for (int knownPass = 0; knownPass < 2; ++knownPass)
	{
		const bool tierKnown = (knownPass == 1);

		for (int lanPass = 0; lanPass < 2; ++lanPass)
		{
			for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
			{
				TimeConfig cfg;
				cfg.lanZeroDelayOverride = (lanPass == 1);
				cfg.relayDelayFloorTicks = 0;

				const InputDelayDecomposition reading =
					decomposeInputDelay(tierKnown, tier, cfg, 0, 0);

				++checked;
				if (reading.floorClass == RelayFloorClass::Inert)
					++inert;
				if (reading.winner == (tierKnown ? InputDelayWinner::Tier
				                                 : InputDelayWinner::Fallback))
					++armWon;
				if (reading.effectiveTicks == reading.baseTicks)
					++floorSilent;
			}
		}
	}

	CHECK(checked == 2u * 2u * kConnectionTierCount);
	CHECK(inert == checked);
	CHECK(armWon == checked);
	// ⭐ The identity the probe recipe leans on, observed from the outside.
	CHECK(floorSilent == checked);
}

// ---------------------------------------------------------------------------
// 3. THE TIE -- the one verdict that names no arm.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.AFloorEqualToTheBaseIsATieAndNeitherArmWins",
          "[CharacterViz][InputHistoryViz]")
{
	std::size_t ties = 0u;
	std::size_t equals = 0u;

	for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
	{
		TimeConfig cfg;
		cfg.relayDelayFloorTicks = expectedBaseTicks(true, tier, cfg);

		const InputDelayDecomposition reading = decomposeInputDelay(true, tier, cfg, 0, 0);

		if (reading.floorClass == RelayFloorClass::Tie)
			++ties;
		if (reading.winner == InputDelayWinner::Equal)
			++equals;
	}

	CHECK(ties == kConnectionTierCount);
	CHECK(equals == kConnectionTierCount);

	// The fallback arm ties too -- the verdict is about the two NUMBERS, not the arm.
	TimeConfig cfg;
	cfg.relayDelayFloorTicks = cfg.rttTierInputDelays[kMaxConnectionTierIndex];

	const InputDelayDecomposition noTier = decomposeInputDelay(false, 0, cfg, 0, 0);
	CHECK(noTier.baseArm == InputDelayBaseArm::Fallback);
	CHECK(noTier.floorClass == RelayFloorClass::Tie);
	CHECK(noTier.winner == InputDelayWinner::Equal);
}

// ---------------------------------------------------------------------------
// 4. THE REQUEST AND THE CLAMP ARE TWO FACTS, and the readout keeps both.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.AnOutOfRangeFloorIsClampedAndTheRequestIsStillReported",
          "[CharacterViz][InputHistoryViz]")
{
	TimeConfig cfg;
	const int32_t cap = relayDelayFloorHardCapTicks(cfg);
	REQUIRE(cap > cfg.rttTierInputDelays[kMaxConnectionTierIndex]);

	cfg.relayDelayFloorTicks = -1;
	const InputDelayDecomposition negative = decomposeInputDelay(true, 1, cfg, 0, 0);
	CHECK(negative.floorRequested == -1);
	CHECK(negative.floorTicks == 0);
	CHECK(negative.floorClass == RelayFloorClass::Inert);
	CHECK(negative.winner == InputDelayWinner::Tier);
	CHECK(negative.floorHardCap == cap);

	cfg.relayDelayFloorTicks = cap + 1;
	const InputDelayDecomposition overCap = decomposeInputDelay(true, 1, cfg, 0, 0);
	CHECK(overCap.floorRequested == cap + 1);
	CHECK(overCap.floorTicks == cap);
	CHECK(overCap.effectiveTicks == cap);
	CHECK(overCap.floorClass == RelayFloorClass::Binding);
	CHECK(overCap.winner == InputDelayWinner::Floor);

	// An out-of-range tier is clamped the same way the wire value is on arrival.
	const InputDelayDecomposition wild = decomposeInputDelay(true, 99, cfg, 0, 0);
	CHECK(wild.tierIndex == kMaxConnectionTierIndex);
}

// ---------------------------------------------------------------------------
// 5. THE LAN OVERRIDE -- tier 0 only, and a floor still beats it.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.TheLanOverrideIsTierZeroOnlyAndANonzeroFloorStillDominatesIt",
          "[CharacterViz][InputHistoryViz]")
{
	std::size_t checked = 0u;
	std::size_t flagAgreed = 0u;

	for (int knownPass = 0; knownPass < 2; ++knownPass)
	{
		const bool tierKnown = (knownPass == 1);

		for (int lanPass = 0; lanPass < 2; ++lanPass)
		{
			const bool lanOverride = (lanPass == 1);

			for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
			{
				TimeConfig cfg;
				cfg.lanZeroDelayOverride = lanOverride;

				const InputDelayDecomposition reading =
					decomposeInputDelay(tierKnown, tier, cfg, 0, 0);

				++checked;
				if (reading.lanOverrideApplied == (tierKnown && lanOverride && tier == 0))
					++flagAgreed;
			}
		}
	}

	CHECK(checked == 2u * 2u * kConnectionTierCount);
	CHECK(flagAgreed == checked);

	// The precedence rule: the override zeroes the base, and any real floor then binds.
	TimeConfig cfg;
	cfg.lanZeroDelayOverride = true;
	cfg.relayDelayFloorTicks = 1;

	const InputDelayDecomposition lanTierZero = decomposeInputDelay(true, 0, cfg, 0, 0);
	CHECK(lanTierZero.lanOverrideApplied);
	CHECK(lanTierZero.baseTicks == 0);
	CHECK(lanTierZero.effectiveTicks == 1);
	CHECK(lanTierZero.floorClass == RelayFloorClass::Binding);
	CHECK(lanTierZero.winner == InputDelayWinner::Floor);
}

// ---------------------------------------------------------------------------
// 6. THE TWO CARRIED VALUES -- recorded, never repaired.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.ADisagreementBetweenTheFormulaAndThePublishedValueIsCarriedNotRepaired",
          "[CharacterViz][InputHistoryViz]")
{
	TimeConfig cfg;
	cfg.relayDelayFloorTicks = 3;

	const InputDelayDecomposition reading = decomposeInputDelay(true, 0, cfg, 7, 2);

	CHECK(reading.formulaTicks == 7);
	CHECK(reading.publishedTicks == 2);
	// ⛔ Neither carried value moved the readout's OWN answer, which is the whole point:
	//   a display that reconciled them would hide the divergence it exists to show.
	CHECK(reading.effectiveTicks == 3);
	CHECK(reading.floorClass == RelayFloorClass::Binding);
}

// ---------------------------------------------------------------------------
// 7. THE WORKED EXAMPLE -- the tree's own shipped numbers.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.AtTheShippedTableAndAFloorOfThreeTheFloorDecidesTheBottomTwoTiers",
          "[CharacterViz][InputHistoryViz]")
{
	TimeConfig cfg;

	// The worked example rests on these four numbers, so it states them rather than
	// assuming them; the compiled table moving is a reason to re-derive the example.
	REQUIRE(cfg.rttTierInputDelays[0] == 1);
	REQUIRE(cfg.rttTierInputDelays[1] == 2);
	REQUIRE(cfg.rttTierInputDelays[2] == 3);
	REQUIRE(cfg.rttTierInputDelays[kMaxConnectionTierIndex] == 4);
	REQUIRE_FALSE(cfg.lanZeroDelayOverride);

	// ⚠ The COMPILED default is 0; 3 is what the shipped `RelayDelayFloorTicks` key sets.
	cfg.relayDelayFloorTicks = 3;

	const InputDelayDecomposition tier0 = decomposeInputDelay(true, 0, cfg, 0, 0);
	CHECK(tier0.baseTicks == 1);
	CHECK(tier0.effectiveTicks == 3);
	CHECK(tier0.floorClass == RelayFloorClass::Binding);
	CHECK(tier0.winner == InputDelayWinner::Floor);

	const InputDelayDecomposition tier1 = decomposeInputDelay(true, 1, cfg, 0, 0);
	CHECK(tier1.baseTicks == 2);
	CHECK(tier1.floorClass == RelayFloorClass::Binding);
	CHECK(tier1.winner == InputDelayWinner::Floor);

	const InputDelayDecomposition tier2 = decomposeInputDelay(true, 2, cfg, 0, 0);
	CHECK(tier2.baseTicks == 3);
	CHECK(tier2.floorClass == RelayFloorClass::Tie);
	CHECK(tier2.winner == InputDelayWinner::Equal);

	const InputDelayDecomposition tier3 =
		decomposeInputDelay(true, kMaxConnectionTierIndex, cfg, 0, 0);
	CHECK(tier3.baseTicks == 4);
	CHECK(tier3.effectiveTicks == 4);
	CHECK(tier3.floorClass == RelayFloorClass::ActiveNotBinding);
	CHECK(tier3.winner == InputDelayWinner::Tier);

	const InputDelayDecomposition noTier = decomposeInputDelay(false, 0, cfg, 0, 0);
	CHECK(noTier.baseArm == InputDelayBaseArm::Fallback);
	CHECK(noTier.baseTicks == 4);
	CHECK(noTier.floorClass == RelayFloorClass::ActiveNotBinding);
	CHECK(noTier.winner == InputDelayWinner::Fallback);

	// Below max(rttTierInputDelays), so the advisory stays silent at this floor.
	CHECK(noTier.advisory == RelayDelayFloorAdvisory::None);
}

// ---------------------------------------------------------------------------
// 8. THE DELAY LANE -- storage, the poll's delay half, and the verdict.
//
// WHAT THIS SECTION GUARDS is the wiring between the pure decomposition above and the
// per-tick lane: that the verdict's ORDERED rules read the right cell, that the two
// halves really do have opposite write policies, that the pause elides this lane
// exactly as it elides the other two, and that the client key is a SIM-tick
// subtraction mapped once -- never a lane-tick one.
// ---------------------------------------------------------------------------

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputDelayCell;
using brawlerInputHistoryVisualization::InputDelayVerdict;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::TickLanePollCounts;
using brawlerInputHistoryVisualization::delayVerdictOf;
using brawlerInputHistoryVisualization::kInputDelayVerdictCount;

// The SlotReader double: every case below tells it exactly which applied ticks name a
// capture, and the rest answer as ordinary NoRef ticks the join folds away.
class MockSlotReader
{
public:
	void setRef(uint32_t simTick, AppliedCaptureRef ref) { m_refs[simTick] = ref; }
	void setHasCorrectionCache(bool hasCache) { m_hasCorrectionCache = hasCache; }

	AppliedCaptureRef appliedCaptureRef(uint32_t simTick) const
	{
		const auto it = m_refs.find(simTick);
		return (it == m_refs.end()) ? AppliedCaptureRef{} : it->second;
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
// `pauseWhileIdle` defaults off, so a case that does not care about the gate gets one
// less thing to reason about; the pause cases below turn it on explicitly.
TickLanePollCounts poll(const MockSlotReader&                   reader,
                        uint32_t                                simTick,
                        std::optional<InputDelayDecomposition>  delay,
                        InputHistoryTickLanes&                  lanes,
                        bool                                    active         = true,
                        bool                                    pauseWhileIdle = false)
{
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, active ? movingInput() : neutralInput(), pauseWhileIdle,
		std::nullopt, delay, std::nullopt, inversion, lanes);
}

InputDelayDecomposition decompositionWithEffective(int32_t effectiveTicks)
{
	InputDelayDecomposition d;
	d.effectiveTicks = effectiveTicks;
	return d;
}

// ---------------------------------------------------------------------------
// 8.1 THE VERDICT -- each rule pinned through the REAL poll, and all seven reachable.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.EachVerdictRuleIsPinnedThroughTheRealPollAndAllSevenAreReachable",
          "[CharacterViz][InputHistoryViz]")
{
	bool seen[kInputDelayVerdictCount] = {};
	const auto mark = [&](InputDelayVerdict v) { seen[static_cast<uint8_t>(v)] = true; };

	// Agree: the join {capture 200, applied 203} (lag 3) with client delay 3 at 200,
	// filed in ONE poll -- captureSim == 203 - 3 == 200, the join's own capture tick.
	{
		MockSlotReader        reader;
		InputHistoryTickLanes lanes;
		reader.setRef(203u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 200u });
		poll(reader, 203u, decompositionWithEffective(3), lanes);

		REQUIRE(lanes.delayCellAt(200u) != nullptr);
		const InputDelayVerdict v = delayVerdictOf(*lanes.delayCellAt(200u));
		CHECK(v == InputDelayVerdict::Agree);
		mark(v);
	}

	// LagShortByOne: the SAME lag (3), client delay 4 -> lag == D - 1.
	{
		MockSlotReader        reader;
		InputHistoryTickLanes lanes;
		reader.setRef(203u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 200u });
		poll(reader, 204u, decompositionWithEffective(4), lanes);

		REQUIRE(lanes.delayCellAt(200u) != nullptr);
		const InputDelayVerdict v = delayVerdictOf(*lanes.delayCellAt(200u));
		CHECK(v == InputDelayVerdict::LagShortByOne);
		mark(v);
	}

	// ServerLater: client delay 2 filed EARLY, the lag-3 join arrives on a LATER poll.
	{
		MockSlotReader        reader;
		InputHistoryTickLanes lanes;
		poll(reader, 202u, decompositionWithEffective(2), lanes);
		reader.setRef(203u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 200u });
		poll(reader, 203u, std::nullopt, lanes);

		REQUIRE(lanes.delayCellAt(200u) != nullptr);
		const InputDelayVerdict v = delayVerdictOf(*lanes.delayCellAt(200u));
		CHECK(v == InputDelayVerdict::ServerLater);
		mark(v);
	}

	// ServerEarlier: the lag-3 join arrives first, client delay 5 files LATER --
	// lag (3) < D - 1 (4), the server's held time reads SMALLER than the client's delay.
	{
		MockSlotReader        reader;
		InputHistoryTickLanes lanes;
		reader.setRef(203u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 200u });
		poll(reader, 203u, std::nullopt, lanes);
		poll(reader, 205u, decompositionWithEffective(5), lanes);

		REQUIRE(lanes.delayCellAt(200u) != nullptr);
		const InputDelayVerdict v = delayVerdictOf(*lanes.delayCellAt(200u));
		CHECK(v == InputDelayVerdict::ServerEarlier);
		mark(v);
	}

	// LagUnverified: a server half with no client half at all.
	{
		MockSlotReader        reader;
		InputHistoryTickLanes lanes;
		reader.setRef(210u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 200u });
		poll(reader, 210u, std::nullopt, lanes);

		REQUIRE(lanes.delayCellAt(200u) != nullptr);
		const InputDelayVerdict v = delayVerdictOf(*lanes.delayCellAt(200u));
		CHECK(v == InputDelayVerdict::LagUnverified);
		mark(v);
	}

	// NoCaptureNamed: a Sentinel join at 93 -- T3's rule keys it at the tick it speaks
	// for, so it lands at 93 and nowhere else.
	{
		MockSlotReader        reader;
		InputHistoryTickLanes lanes;
		reader.setRef(93u, AppliedCaptureRef{ AppliedCaptureRefKind::Sentinel, kNoInputCaptureTick });
		poll(reader, 93u, std::nullopt, lanes);

		REQUIRE(lanes.delayCellAt(93u) != nullptr);
		const InputDelayVerdict v = delayVerdictOf(*lanes.delayCellAt(93u));
		CHECK(v == InputDelayVerdict::NoCaptureNamed);
		mark(v);

		// An untouched tick carries no cell at all -- there is nothing to have a verdict.
		CHECK(lanes.delayCellAt(999u) == nullptr);
	}

	// NoVerdict: the rule's own degenerate case -- nothing claimed yet.
	{
		const InputDelayVerdict v = delayVerdictOf(InputDelayCell{});
		CHECK(v == InputDelayVerdict::NoVerdict);
		mark(v);
	}

	std::size_t reached = 0u;
	for (bool b : seen)
	{
		if (b)
			++reached;
	}
	CHECK(reached == kInputDelayVerdictCount);
}

// ---------------------------------------------------------------------------
// 8.2 THE TWO WRITE POLICIES -- first-sample-wins vs rewritable, on one cell.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.ARePresentedClientHalfKeepsItsFirstValueAndALaterJoinUpdatesTheServerHalfOnly",
          "[CharacterViz][InputHistoryViz]")
{
	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	// The SAME capture (tick 500) re-presented on 40 polls, each with a DIFFERENT
	// effectiveTicks -- so each poll's own liveSimTick differs too, and captureSim
	// (liveSimTick - effectiveTicks) lands on 500 every single time.
	uint32_t ignoredTotal  = 0u;
	uint32_t recordedTotal = 0u;
	for (int32_t effective = 1; effective <= 40; ++effective)
	{
		const TickLanePollCounts counts =
			poll(reader, 500u + static_cast<uint32_t>(effective), decompositionWithEffective(effective), lanes);
		ignoredTotal  += counts.delayClientIgnored;
		recordedTotal += counts.delayClientRecorded;
	}

	CHECK(recordedTotal == 1u);
	CHECK(ignoredTotal == 39u);
	REQUIRE(lanes.delayCellAt(500u) != nullptr);
	CHECK(lanes.delayCellAt(500u)->clientDelayTicks == 1);

	// A join naming that same capture arrives later and UPDATES the server half only.
	reader.setRef(503u, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 500u });
	const TickLanePollCounts later = poll(reader, 503u, std::nullopt, lanes);

	CHECK(later.delayServerUpdated == 1u);
	REQUIRE(lanes.delayCellAt(500u) != nullptr);
	CHECK(lanes.delayCellAt(500u)->serverLagTicks == 3);
	// ⭐ The client half is UNCHANGED by the server-side write.
	CHECK(lanes.delayCellAt(500u)->clientDelayTicks == 1);
}

// ---------------------------------------------------------------------------
// 8.3 THE PAUSE -- the delay lane is elided exactly like the other two, and a join
// naming a capture BEFORE the span still lands on its own (unshifted) tick.
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.PauseElidesBothHalvesOfTheDelayLaneAndAJoinBeforeTheSpanStillLandsCorrectly",
          "[CharacterViz][InputHistoryViz]")
{
	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	// 10 moving, 35 idle, then 15 more moving -- the T16 shape. First elided at tick 25
	// (kMovingEnd + kLanePauseEngageTicks == 10 + 15); the span closes at the first
	// moving tick after it, 45, having skipped 20 (ticks 25..44). Every tick is polled
	// EXACTLY ONCE, in order, so the three targeted calls below are ordinary steps of
	// the same sequence rather than a re-poll of an already-decided tick.
	uint32_t tick = 0u;
	for (; tick < 10u; ++tick)
		poll(reader, tick, std::nullopt, lanes, /*active*/ true, /*pauseWhileIdle*/ true);
	for (; tick < 45u; ++tick)
		poll(reader, tick, std::nullopt, lanes, /*active*/ false, /*pauseWhileIdle*/ true);
	for (; tick < 50u; ++tick)
		poll(reader, tick, std::nullopt, lanes, /*active*/ true, /*pauseWhileIdle*/ true);

	REQUIRE(lanes.gate().axisEventCount() == 1u);
	REQUIRE(lanes.gate().axisEventAt(0u).simTick == 25u);
	REQUIRE(lanes.gate().axisEventAt(0u).skippedTicks == 20u);

	// tick 50: a client sample keyed inside the (now closed) span is refused and counted.
	const TickLanePollCounts client =
		poll(reader, tick, decompositionWithEffective(10), lanes, /*active*/ true, /*pauseWhileIdle*/ true);
	CHECK(client.delayClientElided == 1u);
	++tick;

	// tick 51: a join naming a capture INSIDE the span is refused and counted.
	reader.setRef(tick, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 30u });
	const TickLanePollCounts insideSpan =
		poll(reader, tick, std::nullopt, lanes, /*active*/ true, /*pauseWhileIdle*/ true);
	CHECK(insideSpan.delayServerElided == 1u);
	CHECK(lanes.delayCellAt(30u) == nullptr);
	++tick;

	// tick 52: a join naming a capture BEFORE the span lands on the correct (unshifted)
	// lane tick -- the T16 "...AcrossAnElision" shape, on this lane.
	reader.setRef(tick, AppliedCaptureRef{ AppliedCaptureRefKind::Ref, 5u });
	const TickLanePollCounts beforeSpan =
		poll(reader, tick, std::nullopt, lanes, /*active*/ true, /*pauseWhileIdle*/ true);
	CHECK(beforeSpan.delayServerRecorded == 1u);
	REQUIRE(lanes.delayCellAt(5u) != nullptr);
	CHECK(lanes.delayCellAt(5u)->serverLagTicks == static_cast<int32_t>(tick) - 5);
	++tick;

	for (; tick < 60u; ++tick)
		poll(reader, tick, std::nullopt, lanes, /*active*/ true, /*pauseWhileIdle*/ true);
}

// ---------------------------------------------------------------------------
// 8.4 THE READING -- filed above the gate, so a paused meter still shows the last
// polled decomposition (the T18r "AnElidedPollStillMovesTheReading..." shape).
// ---------------------------------------------------------------------------

TEST_CASE("InputDelay.TheDelayReadingIsFiledAboveTheGateAndSurvivesAnElidedPoll",
          "[CharacterViz][InputHistoryViz]")
{
	MockSlotReader        reader;
	InputHistoryTickLanes lanes;

	uint32_t tick = 0u;
	for (; tick < 20u; ++tick)
		poll(reader, tick, decompositionWithEffective(static_cast<int32_t>(tick)), lanes,
			/*active*/ true, /*pauseWhileIdle*/ true);
	for (; tick < 50u; ++tick)
		poll(reader, tick, decompositionWithEffective(static_cast<int32_t>(tick)), lanes,
			/*active*/ false, /*pauseWhileIdle*/ true);

	// The gate is paused (tick 49 was elided), yet the reading carries the LAST
	// polled tick and ITS decomposition -- not the last RECORDED one.
	REQUIRE(lanes.gate().paused());
	REQUIRE(lanes.delayReading().has_value());
	CHECK(lanes.delayReading()->simTick == 49u);
	CHECK(lanes.delayReading()->decomposition.effectiveTicks == 49);
}

} // namespace inputhistorydelaytests

#endif // WITH_LOW_LEVEL_TESTS
