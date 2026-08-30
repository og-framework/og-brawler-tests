// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins the CLOCK READING and its readout -- the one thing the frame meter can say about
// skips and stalls, which are RATES on a continuous axis and cannot be drawn as events.
// Both ticks of a skip get cells and a stall repeats a frame, so nothing about a cell
// changes; the drift STATE that decides them is readable, and that is what is filed here.
//
// WHAT THIS SUITE IS REALLY GUARDING is the static run: how long the estimator's authority
// tick has stood still. That is the field the display was built for -- a client resyncing
// to the same frozen target 128 times looks, from every other reading, like a clock that
// cannot converge, and only this number says the target never moved.
//
// A poll is a render frame and the simulation runs at its own rate, so a poll-counted run
// would print a plausible number that means nothing. The units get their own case below.
// ⛔ THE RUN IS IN THE CLIENT'S SIM TICKS, NEVER IN POLLS.
//
// A hard resync ASSIGNS the prediction tick backwards, so during a storm the client's tick
// number is confined to a short loop, and a run measured as a difference of tick numbers
// stays small however long the authority has been frozen.
// ⭐ THE RUN ACCUMULATES SIMULATED TICKS AND IS NOT A DIFFERENCE OF TICK NUMBERS.

#include "catch_amalgamated.hpp"

#include <cstdint>
#include <optional>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPoll.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SlotStateProvenance.h"

namespace inputhistoryclocktests
{

using brawlerInputHistoryVisualization::AppliedCaptureInversion;
using brawlerInputHistoryVisualization::CaptureRowFields;
using brawlerInputHistoryVisualization::ClockDriftReading;
using brawlerInputHistoryVisualization::ClockDriftReadout;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryTickLanes;
using brawlerInputHistoryVisualization::LaneAdmission;

using brawlerInputHistoryVisualization::buildClockDriftReadout;

using DriftAction = ClientPredictionClock::DriftAction;

// The estimator's offset, from the storm's own log line: target 6197 = auth 6193 + 4.
static constexpr uint32_t kOffsetTicks = 4u;

// A reader that answers for nothing. Every reading below is filed ABOVE the gate, so what
// the correction ring holds is beside the point and a silent one keeps it that way.
class SilentReader
{
public:
	AppliedCaptureRef appliedCaptureRef(uint32_t) const { return AppliedCaptureRef{}; }
	std::optional<SlotStateProvenance> slotProvenance(uint32_t) const { return std::nullopt; }
	bool hasCorrectionCache() const { return false; }
};

static CaptureRowFields movingInput()
{
	CaptureRowFields fields;
	fields.direction  = DirectionBucket::Forward;
	fields.buttonMask = 0u;
	return fields;
}

static CaptureRowFields neutralInput()
{
	CaptureRowFields fields;
	fields.direction  = DirectionBucket::Neutral;
	fields.buttonMask = 0u;
	return fields;
}

// One reading, built the way the UE passthrough will: the two ticks are read as a pair and
// the drift falls out of them. ⛔ NO CASE BELOW HANDS IN A DRIFT OF ITS OWN.
// `simTick` is deliberately left unset -- noteClockDriftReading stamps it from the poll.
static ClockDriftReading readingOf(uint32_t predictionTick, uint32_t authorityTick,
                                   DriftAction pendingAction, uint32_t stallDebtTicks = 0u)
{
	ClockDriftReading reading;
	reading.predictionTick = predictionTick;
	reading.authorityTick  = authorityTick;
	reading.targetTick     = authorityTick + kOffsetTicks;
	reading.driftTicks =
		static_cast<int32_t>(reading.targetTick) - static_cast<int32_t>(predictionTick);
	reading.pendingAction  = pendingAction;
	reading.stallDebtTicks = stallDebtTicks;
	return reading;
}

// One whole lane poll through the REAL entry point. The poll's liveSimTick IS the client's
// prediction tick, exactly as the UE caller's vizSimulationStep.getTick() is -- both come
// from the one clock read. ⛔ NO CASE GIVES THE TWO DIFFERENT VALUES.
static LaneAdmission poll(uint32_t simTick, std::optional<ClockDriftReading> clock,
                          InputHistoryTickLanes& lanes, bool active = true,
                          bool pauseWhileIdle = false)
{
	const SilentReader      reader;
	AppliedCaptureInversion inversion;
	return brawlerInputHistoryVisualization::pollInputHistoryLanes(reader, simTick,
		DAttackState::Idle, active ? movingInput() : neutralInput(), pauseWhileIdle,
		std::nullopt, std::nullopt, clock, inversion, lanes).admission;
}

// ---------------------------------------------------------------------------
// THE STORM -- THE CASE THE FIELD EXISTS FOR
// ---------------------------------------------------------------------------

TEST_CASE("Clock.AFrozenAuthorityTickAccumulatesThroughAResyncStormThatNeverMovesTheAxis",
          "[CharacterViz][InputHistoryViz]")
{
	// The user's own session, replayed: 128 x "hard resync oldTick=6219 -> newTick=6197
	// drift=-22". Every fire carries the SAME pair, because the target never moved -- the
	// client converged 128 times on a frozen authority tick.
	//
	// ⭐ EVERY NUMBER BELOW IS DERIVED FROM THAT PAIR. The epoch's width is the two ticks'
	// difference and the run is that width times the epoch count; nothing is copied in.
	constexpr uint32_t kFrozenAuthority = 6193u;   // target 6197 = 6193 + 4
	constexpr uint32_t kResyncNewTick   = kFrozenAuthority + kOffsetTicks;
	constexpr uint32_t kResyncOldTick   = 6219u;   // where the clock stands when it fires
	constexpr uint32_t kEpochTicks      = kResyncOldTick - kResyncNewTick;
	constexpr uint32_t kEpochs          = 128u;

	InputHistoryTickLanes lanes;

	// Three healthy polls first, the authority moving on each: the run is only meaningful
	// against a tick that HAS moved, and this is where it last did.
	for (uint32_t step = 0u; step < 3u; ++step)
	{
		const uint32_t tick = kResyncNewTick - 3u + step;
		poll(tick, readingOf(tick, kFrozenAuthority - 3u + step, DriftAction::None), lanes);
	}
	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == 0u);

	// The freeze: from here the authority tick never changes again.
	poll(kResyncNewTick, readingOf(kResyncNewTick, kFrozenAuthority, DriftAction::None), lanes);
	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == 0u);

	// One epoch is the client running kResyncNewTick -> kResyncOldTick and being assigned
	// back. Counted rather than asserted per poll, so the case reports one number.
	uint32_t pendingResyncPolls = 0u;
	uint32_t highestTickReached = kResyncNewTick;

	for (uint32_t epoch = 0u; epoch < kEpochs; ++epoch)
	{
		for (uint32_t tick = kResyncNewTick + 1u; tick <= kResyncOldTick; ++tick)
		{
			const int32_t drift =
				static_cast<int32_t>(kResyncNewTick) - static_cast<int32_t>(tick);
			const DriftAction pending = (drift == -static_cast<int32_t>(kEpochTicks))
			                                ? DriftAction::HardResync
			                                : DriftAction::Stall;

			poll(tick, readingOf(tick, kFrozenAuthority, pending), lanes);

			if (buildClockDriftReadout(lanes).reading.pendingAction == DriftAction::HardResync)
				++pendingResyncPolls;
			if (tick > highestTickReached)
				highestTickReached = tick;
		}

		// The resync itself: the prediction tick is ASSIGNED backwards, so the next poll is
		// at kResyncNewTick again. The last epoch is left standing at the pending-resync
		// poll, which is the frame the readout has to be right about.
		if (epoch + 1u < kEpochs)
		{
			poll(kResyncNewTick,
				readingOf(kResyncNewTick, kFrozenAuthority, DriftAction::None), lanes);
		}
	}

	const ClockDriftReadout readout = buildClockDriftReadout(lanes);

	REQUIRE(readout.present);
	CHECK(readout.reading.simTick == kResyncOldTick);
	CHECK(readout.reading.predictionTick == kResyncOldTick);
	CHECK(readout.reading.targetTick == kResyncNewTick);
	CHECK(readout.reading.authorityTick == kFrozenAuthority);
	CHECK(readout.reading.driftTicks == -static_cast<int32_t>(kEpochTicks));
	CHECK(readout.reading.pendingAction == DriftAction::HardResync);
	CHECK(pendingResyncPolls == kEpochs);

	// The run, accumulated: kEpochs epochs each advancing the client by kEpochTicks.
	CHECK(readout.authorityStaticTicks == kEpochs * kEpochTicks);

	// And the reason it is an accumulator: the client's tick NUMBER never leaves the resync
	// loop, so a run measured as (this tick - the tick the run began on) would have read
	// kEpochTicks, a healthy-looking number for a minute-long outage.
	// ⭐ THE ACCUMULATED RUN AND THAT TICK-NUMBER SPAN ARE PINNED SIDE BY SIDE.
	CHECK(highestTickReached - kResyncNewTick == kEpochTicks);
	CHECK(readout.authorityStaticTicks > highestTickReached - kResyncNewTick);
}

// ---------------------------------------------------------------------------
// THE UNITS -- SIM TICKS, NOT POLLS
// ---------------------------------------------------------------------------

TEST_CASE("Clock.TheStaticRunCountsSimTicksSimulatedAndNotPollsTaken",
          "[CharacterViz][InputHistoryViz]")
{
	// A poll is a RENDER frame. At 120 Hz render over a 60 Hz sim two polls see the same
	// sim tick, so a run that counted polls would read double -- plausible, and wrong.
	constexpr uint32_t kFirstTick        = 900u;
	constexpr uint32_t kAuthorityTick    = 850u;
	constexpr uint32_t kPolls            = 1338u;
	constexpr uint32_t kPollsPerSimTick  = 2u;
	constexpr uint32_t kSimTicksAdvanced = (kPolls - 1u) / kPollsPerSimTick;

	InputHistoryTickLanes lanes;

	for (uint32_t index = 0u; index < kPolls; ++index)
	{
		const uint32_t tick = kFirstTick + index / kPollsPerSimTick;
		poll(tick, readingOf(tick, kAuthorityTick, DriftAction::None), lanes);
	}

	const ClockDriftReadout readout = buildClockDriftReadout(lanes);

	REQUIRE(readout.present);
	CHECK(readout.authorityStaticTicks == kSimTicksAdvanced);
	CHECK(readout.authorityStaticTicks != kPolls);
	CHECK(readout.authorityStaticTicks != kPolls - 1u);
	CHECK(readout.reading.simTick == kFirstTick + kSimTicksAdvanced);
}

TEST_CASE("Clock.AnAdvancingClockUnderAFrozenAuthorityRunsOneStaticTickPerSimTick",
          "[CharacterViz][InputHistoryViz]")
{
	// The healthy control for the storm above: with nothing resyncing, the accumulated run
	// and the plain difference of tick numbers agree, so the storm is the only place the
	// two part company.
	//
	// ⛔ A RUN OF N TICKS NEEDS N+1 POLLS -- the one that started it, and the one N later.
	constexpr uint32_t kFirstTick     = 4881u;
	constexpr uint32_t kAuthorityTick = 4800u;
	constexpr uint32_t kRunTicks      = 1338u;

	InputHistoryTickLanes lanes;

	for (uint32_t index = 0u; index <= kRunTicks; ++index)
	{
		const uint32_t tick = kFirstTick + index;
		poll(tick, readingOf(tick, kAuthorityTick, DriftAction::None), lanes);
	}

	const ClockDriftReadout readout = buildClockDriftReadout(lanes);

	REQUIRE(readout.present);
	CHECK(readout.authorityStaticTicks == kRunTicks);
	CHECK(readout.authorityStaticTicks == readout.reading.simTick - kFirstTick);
}

// ---------------------------------------------------------------------------
// THE RESET
// ---------------------------------------------------------------------------

TEST_CASE("Clock.AMovedAuthorityTickResetsTheRunOnThePollThatSeesItMove",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kFirstTick     = 300u;
	constexpr uint32_t kAuthorityTick = 250u;
	constexpr uint32_t kFrozenTicks   = 49u;

	InputHistoryTickLanes lanes;

	for (uint32_t index = 0u; index <= kFrozenTicks; ++index)
	{
		const uint32_t tick = kFirstTick + index;
		poll(tick, readingOf(tick, kAuthorityTick, DriftAction::None), lanes);
	}

	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == kFrozenTicks);

	// The relay delivers again: the run restarts ON THIS POLL, not on the one after it.
	const uint32_t movedTick = kFirstTick + kFrozenTicks + 1u;
	poll(movedTick, readingOf(movedTick, kAuthorityTick + 1u, DriftAction::None), lanes);
	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == 0u);
	CHECK(buildClockDriftReadout(lanes).reading.authorityTick == kAuthorityTick + 1u);

	poll(movedTick + 1u, readingOf(movedTick + 1u, kAuthorityTick + 1u, DriftAction::None), lanes);
	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == 1u);
}

// ---------------------------------------------------------------------------
// ABOVE THE GATE -- the T18r shape, for the fourth reading that needs it
// ---------------------------------------------------------------------------

TEST_CASE("Clock.TheReadingIsFiledAboveTheGateAndAnElidedPollStillMovesIt",
          "[CharacterViz][InputHistoryViz]")
{
	// While the gate is paused the poll ends early and writes no cell, but the clock keeps
	// running. A reading frozen at the last RECORDED tick would say the authority had been
	// static for exactly as long as the player had been idle, which is a different fact.
	constexpr uint32_t kAuthorityTick = 100u;
	constexpr uint32_t kActiveTicks   = 20u;
	constexpr uint32_t kIdleTicks     = 30u;

	InputHistoryTickLanes lanes;

	uint32_t tick = 0u;
	for (; tick < kActiveTicks; ++tick)
		poll(tick, readingOf(tick, kAuthorityTick, DriftAction::None), lanes, true, true);

	LaneAdmission lastAdmission = LaneAdmission::Recorded;
	for (; tick < kActiveTicks + kIdleTicks; ++tick)
		lastAdmission =
			poll(tick, readingOf(tick, kAuthorityTick, DriftAction::None), lanes, false, true);

	const uint32_t lastPolled = tick - 1u;

	REQUIRE(lanes.gate().paused());
	REQUIRE(lastAdmission == LaneAdmission::Elided);
	CHECK(lanes.newestAxisTick() < lastPolled);

	const ClockDriftReadout readout = buildClockDriftReadout(lanes);
	REQUIRE(readout.present);
	CHECK(readout.reading.simTick == lastPolled);
	CHECK(readout.reading.predictionTick == lastPolled);
	CHECK(readout.authorityStaticTicks == lastPolled);
}

// ---------------------------------------------------------------------------
// NO READING AT ALL
// ---------------------------------------------------------------------------

TEST_CASE("Clock.NoReadingDrawsNothingRatherThanAPlausibleZero",
          "[CharacterViz][InputHistoryViz]")
{
	constexpr uint32_t kAuthorityTick = 700u;
	constexpr uint32_t kRunTicks      = 39u;

	// Nothing polled yet: there is no clock to draw, and the run is not a claim about one.
	InputHistoryTickLanes lanes;
	CHECK_FALSE(buildClockDriftReadout(lanes).present);
	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == 0u);

	for (uint32_t index = 0u; index <= kRunTicks; ++index)
		poll(index, readingOf(index, kAuthorityTick, DriftAction::None), lanes);

	REQUIRE(buildClockDriftReadout(lanes).present);
	REQUIRE(buildClockDriftReadout(lanes).authorityStaticTicks == kRunTicks);

	// The authority role, which does not predict: nullopt draws nothing, and the run it
	// never contributed to is left exactly as it was.
	poll(kRunTicks + 1u, std::nullopt, lanes);
	CHECK_FALSE(buildClockDriftReadout(lanes).present);
	CHECK(lanes.authorityStaticSimTicks() == kRunTicks);

	// ⚠ A reading arriving after a gap starts a NEW run, at zero: the lanes cannot claim
	// the authority stood still across polls they were told nothing about.
	poll(kRunTicks + 2u, readingOf(kRunTicks + 2u, kAuthorityTick, DriftAction::None), lanes);
	CHECK(buildClockDriftReadout(lanes).authorityStaticTicks == 0u);
}

// ---------------------------------------------------------------------------
// THE FIELDS THEMSELVES
// ---------------------------------------------------------------------------

TEST_CASE("Clock.EveryPendingDriftActionAndTheStallDebtReachTheReadoutUnchanged",
          "[CharacterViz][InputHistoryViz]")
{
	// The readout restates the reading and derives nothing from it, so every enumerator the
	// clock can return must arrive intact -- including None, which the line prints as a dash
	// and which a readout treating absence as "nothing pending" would lose.
	const DriftAction kActions[] = { DriftAction::None, DriftAction::Skip, DriftAction::Stall,
		DriftAction::HardResync };

	InputHistoryTickLanes lanes;

	uint32_t carried = 0u;
	uint32_t tick    = 5000u;
	for (const DriftAction action : kActions)
	{
		const uint32_t debt = static_cast<uint32_t>(action) + 1u;
		poll(tick, readingOf(tick, 4990u, action, debt), lanes);

		const ClockDriftReadout readout = buildClockDriftReadout(lanes);
		if (readout.present && readout.reading.pendingAction == action
		    && readout.reading.stallDebtTicks == debt)
		{
			++carried;
		}
		++tick;
	}

	CHECK(carried == sizeof(kActions) / sizeof(kActions[0]));

	// The drift is the pair's difference and the readout does not recompute it: a target
	// above the prediction tick is POSITIVE, below it NEGATIVE, both carried signed.
	poll(6000u, readingOf(6000u, 6100u, DriftAction::Skip), lanes);
	CHECK(buildClockDriftReadout(lanes).reading.driftTicks
		== static_cast<int32_t>(6100u + kOffsetTicks) - 6000);

	poll(6200u, readingOf(6200u, 6100u, DriftAction::HardResync), lanes);
	CHECK(buildClockDriftReadout(lanes).reading.driftTicks
		== static_cast<int32_t>(6100u + kOffsetTicks) - 6200);
}

} // namespace inputhistoryclocktests

#endif // WITH_LOW_LEVEL_TESTS
