// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerMotionMatching.h"
#include "OGBrawler/InputSequence/GameMotions.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGSimulation/Network/LocalInputCache.h"

// ---------------------------------------------------------------------------
// og-netcode-v2-input-relay T15 — WHERE THE MOTION MATCHER GETS ITS HISTORY.
//
// These cases drive the REAL production entry points —
// simulatableBrawler::resolveTriggeredActionId / motionButtonEdge /
// DelayLineMotionHistory in OGBrawler/BrawlerMotionMatching.h — against the two
// competing history sources:
//
//   THE NEW SOURCE  LocalInputCache: raw captures, keyed by CAPTURE tick.
//   THE OLD SOURCE  the correction cache's input column: the APPLIED input,
//                   keyed by APPLICATION tick, so under an input delay `d`
//                   slot `t` holds capture(t - d).
//
// The old source is modelled by AppliedColumnHistory below — the delay-`d` shift
// in the abstract, so `d` can be swept. It is a MODEL of a retired mechanism,
// deliberately kept: the DEFECT it describes (see AC (a) and AC (b) below) is a
// property of application-tick-keyed history in general, not of the container
// that used to provide it, and these two cases are what stop the matcher being
// re-sourced back onto one. They do not touch the correction cache and never did.
//
// [og-netcode-v2-input-relay T16] THE REAL-CACHE HALF OF THIS FILE IS RETIRED.
// `CacheColumnHistory` — the pre-T15 accessor reproduced verbatim over a REAL
// StateCorrectionCache — and the two cases that drove it are gone, because T16
// deletes the cache's input column outright (`m_inputBuffer`, `getInput`,
// `getLatestInput`, `pushPredictionInput`). Those were MIGRATION PROOFS: "the new
// source is byte-identical to the old one at d = 0", and "the old cache source
// has a tick-0 phantom slot the delay line does not". With the counterparty
// deleted there is nothing left to be byte-identical TO and no phantom left to
// demonstrate — the comparison is not merely unsupported, it is meaningless. See
// the retirement blocks at their old positions below for the exact accounting.
//
// AM-1 context: this file exists because the code under test was extracted out
// of UOGBrawlerInputCollectionComponent::buildPlayerInput first. This module
// links { Core, OGSimulation, OGBrawler } and NOT OGBrawlerUnreal, so before the
// extraction the only reachable shape was a test-local re-implementation — which
// proves nothing. Everything asserted below is production code.
// ---------------------------------------------------------------------------

namespace motionsourcetests
{

constexpr float kDeadzone = 0.2f;

// Aim pointing along world +X. Every angle below is aim-RELATIVE, so this choice
// only fixes the frame the sticks are expressed in.
inline glm::vec3 aim() { return glm::vec3(1.f, 0.f, 0.f); }

// World-frame stick vector for an aim-relative angle, given aim = +X.
// InputSequence's convention: right = (fwd.y, -fwd.x) = (0, -1), and the angle is
// atan2(dot(right, s), dot(fwd, s)) — so s = (cos t, -sin t).
inline glm::vec2 stickForAngle(float theta)
{
	return glm::vec2(std::cos(theta), -std::sin(theta));
}

inline dAttackMachineSimulation::PlayerInput frame(glm::vec2 stick,
                                                   bool leftAttack = false,
                                                   bool rightAttack = false)
{
	dAttackMachineSimulation::PlayerInput in;
	in.aimDirection       = aim();
	in.moveDirection      = stick;
	in.moveDirectionWorld = glm::vec3(stick.x, stick.y, 0.f);
	in.attackLeft         = leftAttack;
	in.attackRight        = rightAttack;
	return in;
}

// The live continuous sample for the tick being matched.
//
// NOTE, and it is load-bearing for reading these cases: matchSequence
// (InputSequence.h) `(void)`-casts currentStick, currentReferenceForwardXY AND
// currentButtonsHeld. Only currentButtonsEDGE and currentTick are used. So the
// value here cannot influence any result below — the current frame is not part
// of the match at all, which is exactly why the seam cannot be bridged by
// leaning on the live sample and why the HISTORY source is the whole story.
inline simulatableBrawler::ContinuousInputFields liveFields()
{
	simulatableBrawler::ContinuousInputFields fields;
	fields.aimDirection       = aim();
	fields.moveStick          = stickForAngle(inputSequence::angle::Down);
	fields.moveDirectionWorld = glm::vec3(fields.moveStick.x, fields.moveStick.y, 0.f);
	return fields;
}

// A raw capture feed, keyed by CAPTURE tick. Signed keys so a below-zero tick is
// representable and distinguishable from "absent".
class CaptureFeed
{
public:
	void set(std::int32_t tick, const dAttackMachineSimulation::PlayerInput& in)
	{
		m_byTick[tick] = in;
	}

	const dAttackMachineSimulation::PlayerInput* find(std::int32_t tick) const
	{
		const auto it = m_byTick.find(tick);
		return it == m_byTick.end() ? nullptr : &it->second;
	}

private:
	std::map<std::int32_t, dAttackMachineSimulation::PlayerInput> m_byTick;
};

// THE NEW SOURCE, modelled: at(t) answers with capture t.
class RawCaptureHistory
{
public:
	explicit RawCaptureHistory(const CaptureFeed& feed) : m_feed(&feed) {}

	const dAttackMachineSimulation::PlayerInput* at(std::uint32_t tick) const
	{
		return m_feed->find(static_cast<std::int32_t>(tick));
	}

private:
	const CaptureFeed* m_feed;
};

// THE OLD SOURCE, modelled: at(t) answers with capture(t - d), because the
// correction cache's slot `t` holds the input APPLIED at t, and under an input
// delay of `d` the input applied at t is the capture taken at t - d.
class AppliedColumnHistory
{
public:
	AppliedColumnHistory(const CaptureFeed& feed, int delayTicks)
		: m_feed(&feed), m_delay(delayTicks) {}

	const dAttackMachineSimulation::PlayerInput* at(std::uint32_t tick) const
	{
		return m_feed->find(static_cast<std::int32_t>(tick) - m_delay);
	}

private:
	const CaptureFeed* m_feed;
	int                m_delay;
};

// [og-netcode-v2-input-relay T16] `BrawlerCache` (the
// StateCorrectionCache<State, PlayerInput> alias) and `CacheColumnHistory` — the
// pre-T15 accessor reproduced verbatim over a real cache — STOOD HERE AND ARE
// RETIRED. `CacheColumnHistory::at` was two lines, and both of them
// (`getCacheIndex` + `getInput`) named the input column T16 deletes; `getInput`
// no longer exists on the class at all, so the transcription cannot be written
// against the post-T16 tree in any form. The file's include of
// OGSimulation/CorrectionCache.h went with them — nothing here touches the cache
// any more.

// Packs a machine-level frame into the composite the delay line stores, via the
// production packer.
inline simulatableBrawler::PlayerInput composite(glm::vec2 stick, bool leftAttack, bool rightAttack)
{
	simulatableBrawler::ContinuousInputFields fields;
	fields.aimDirection       = aim();
	fields.moveStick          = stick;
	fields.moveDirectionWorld = glm::vec3(stick.x, stick.y, 0.f);
	return simulatableBrawler::makeSimPlayerInput(fields, leftAttack, rightAttack,
	                                              inputSequence::kNoMatch);
}

} // namespace motionsourcetests

// ---------------------------------------------------------------------------
// AC (a) / AM-6 — THE BLIND SPOT, over the WHOLE final-step window.
//
// A completed quarter-arc whose final step (Down) sits at capture T-1-k, swept
// across every position k the matcher's `windowAfterFinalStep` admits, at every
// interesting delay.
//
// EXPECTED, and the shape here is the point:
//   * the NEW source matches at EVERY k — the window is fully reachable;
//   * the OLD source matches iff k >= d, and misses for k < d.
//
// The `k >= d` boundary is not a weakening of the finding, it IS the finding.
// The displacement is UNIFORM: every history read moves by the same d, so
// inter-step gap matching (maxGapFrames) cancels out entirely and only the
// distance from the newest matched step to currentTick — windowAfterFinalStep —
// is corrupted. Concretely the old source cannot see captures T-d .. T-1 at all,
// so the player must reach the final direction d frames earlier than designed:
// 1 extra frame at tier 0, 4 at the worst shipped tier, 8 at the scenario-4
// floor. The count assertion at the end pins that exact boundary, so this case
// fails if the defect is ever mis-stated in EITHER direction (a wider claim, or
// a fix that silently narrows the reachable window).
// ---------------------------------------------------------------------------
TEST_CASE("MotionMatcherSource.BlindSpot.FinalStepWindowIsFullyReachable",
          "[InputSequence][MotionMatcherSource]")
{
	using namespace motionsourcetests;

	constexpr std::uint32_t kCurrentTick = 100u;
	// windowAfterFinalStep of every shipped definition in kGameMotions.
	constexpr int kWindowAfterFinalStep = 6;

	int discriminatingPairs = 0;

	for (int delay : { 1, 2, 4, 8 })
	{
		for (int k = 0; k <= kWindowAfterFinalStep; ++k)
		{
			CAPTURE(delay, k);

			CaptureFeed feed;
			// Below-deadzone captures across the whole reachable range, so BOTH
			// sources have full coverage and only the three arc frames can match
			// anything. Without this the old source could "fail" merely by
			// reading an absent tick.
			for (int t = static_cast<int>(kCurrentTick) - 45;
			     t < static_cast<int>(kCurrentTick); ++t)
			{
				feed.set(t, frame(glm::vec2(0.f, 0.f)));
			}

			const int finalStepTick = static_cast<int>(kCurrentTick) - 1 - k;
			feed.set(finalStepTick - 2, frame(stickForAngle(inputSequence::angle::Back)));
			feed.set(finalStepTick - 1, frame(stickForAngle(inputSequence::angle::DownBack)));
			feed.set(finalStepTick,     frame(stickForAngle(inputSequence::angle::Down)));

			const auto fields = liveFields();

			const std::uint32_t newSource = simulatableBrawler::resolveTriggeredActionId(
				RawCaptureHistory(feed), kCurrentTick, fields,
				/*leftAttack*/ true, /*rightAttack*/ false, kDeadzone, kGameMotions);
			REQUIRE(newSource == inputSequence::kHadoukenActionId);

			const std::uint32_t oldSource = simulatableBrawler::resolveTriggeredActionId(
				AppliedColumnHistory(feed, delay), kCurrentTick, fields,
				/*leftAttack*/ true, /*rightAttack*/ false, kDeadzone, kGameMotions);

			if (k < delay)
			{
				CHECK(oldSource == inputSequence::kNoMatch);
				++discriminatingPairs;
			}
			else
			{
				CHECK(oldSource == inputSequence::kHadoukenActionId);
			}
		}
	}

	// d=1 -> k in {0}; d=2 -> {0,1}; d=4 -> {0..3}; d=8 -> {0..6}. 1+2+4+7.
	// If this ever reads 0 the case has gone vacuous and asserts nothing.
	REQUIRE(discriminatingPairs == 14);
}

// ---------------------------------------------------------------------------
// AC (b) / AM-5 — A HELD BUTTON PRODUCES EXACTLY ONE EDGE TICK.
//
// A button first pressed at capture P and held from then on. Asserted twice per
// delay: at the edge-mask level (motionButtonEdge, the real function), and
// end-to-end through resolveTriggeredActionId using the shipped single-step
// back-shortcut motion with the stick parked at Back — that motion is
// satisfiable on every tick in the span, so the ONLY thing that can gate it is
// the rising edge, which makes "which ticks fired" a direct readout of "which
// ticks saw an edge".
//
// EXPECTED: the new source reports the edge on exactly one tick, P, at every
// delay. The old source holds the edge open for d+1 consecutive ticks
// (P .. P+d), because it reads "previously held" from capture(T-1-d). At d = 0
// the two coincide, which is the d = 0 arm the AC asks for.
//
// (The phantom window is latent rather than player-visible today: the machine
// sim's Hadouken trigger sits inside DAttackState::Idle and leaves Idle on the
// first fire, so the extra open ticks find it out of Idle. That is a property of
// the CONSUMER, not of this code, and it stops holding once the machine can
// re-enter Idle inside a 100-133 ms window — i.e. at floor 6-8.)
// ---------------------------------------------------------------------------
TEST_CASE("MotionMatcherSource.RisingEdge.HeldButtonEdgesExactlyOnce",
          "[InputSequence][MotionMatcherSource]")
{
	using namespace motionsourcetests;

	constexpr int kPressTick = 120;

	for (int delay : { 0, 1, 2, 4, 8 })
	{
		CAPTURE(delay);

		CaptureFeed feed;
		for (int t = kPressTick - 40; t <= kPressTick + 20; ++t)
		{
			feed.set(t, frame(stickForAngle(inputSequence::angle::Back),
			                  /*leftAttack*/ false,
			                  /*rightAttack*/ t >= kPressTick));
		}

		std::vector<int> newEdgeTicks, oldEdgeTicks, newFireTicks, oldFireTicks;

		for (int tick = kPressTick - 5; tick <= kPressTick + 15; ++tick)
		{
			const dAttackMachineSimulation::PlayerInput* live = feed.find(tick);
			REQUIRE(live != nullptr);

			const std::uint8_t held =
				simulatableBrawler::motionButtonMask(live->attackLeft, live->attackRight);
			const auto currentTick = static_cast<std::uint32_t>(tick);
			const auto fields = liveFields();

			if (simulatableBrawler::motionButtonEdge(RawCaptureHistory(feed), currentTick, held) != 0u)
				newEdgeTicks.push_back(tick);
			if (simulatableBrawler::motionButtonEdge(AppliedColumnHistory(feed, delay), currentTick, held) != 0u)
				oldEdgeTicks.push_back(tick);

			if (simulatableBrawler::resolveTriggeredActionId(
					RawCaptureHistory(feed), currentTick, fields,
					live->attackLeft, live->attackRight, kDeadzone, kGameMotions)
				== inputSequence::kHadoukenActionId)
			{
				newFireTicks.push_back(tick);
			}

			if (simulatableBrawler::resolveTriggeredActionId(
					AppliedColumnHistory(feed, delay), currentTick, fields,
					live->attackLeft, live->attackRight, kDeadzone, kGameMotions)
				== inputSequence::kHadoukenActionId)
			{
				oldFireTicks.push_back(tick);
			}
		}

		const std::vector<int> exactlyThePressTick{ kPressTick };
		CHECK(newEdgeTicks == exactlyThePressTick);
		CHECK(newFireTicks == exactlyThePressTick);

		// Verified negative: the old source keeps the edge open for d+1 ticks.
		std::vector<int> expectedOld;
		for (int i = 0; i <= delay; ++i)
			expectedOld.push_back(kPressTick + i);
		CHECK(oldEdgeTicks == expectedOld);
		CHECK(oldFireTicks == expectedOld);
	}
}

// ===========================================================================
// [og-netcode-v2-input-relay T16] AC (c) — "ZeroDelay.MatchesTheOldCacheSource
// ForEveryTickAboveZero" — STOOD HERE AND IS RETIRED WITH ITS COUNTERPARTY.
//
// WHAT IT WAS. A MIGRATION PROOF, of the same shape T8 retired in
// SimulationNetSyncTest.cpp. It wrote 46 scripted ticks into a REAL
// StateCorrectionCache (pushPredictionTick + pushPredictionInput) and the same
// captures into a REAL LocalInputCache, then swept ticks 131..145 asserting
// the shipped adapter and the verbatim pre-T15 cache accessor produced the
// IDENTICAL match result at every tick, with an anti-vacuity pin
// (matches == 1, noMatches > 0) so "identical" could not be satisfied by two
// always-kNoMatch functions.
//
// WHY IT CANNOT SURVIVE T16, and why that is not a coverage loss. The claim is
// "the new source equals the OLD CACHE SOURCE at d = 0". T16 deletes the old
// cache source: `pushPredictionInput` (how the case fed it) and `getInput` (how
// the case read it) are both gone from StateCorrectionCache, so neither half of
// the comparison can be written. Hand-feeding a stand-in would assert agreement
// with a fabrication rather than with the shipped predecessor — the thing the
// lead's ruling on T8 names explicitly. A migration test cannot outlive the thing
// it migrates from.
//
// COUNT: 17 assertions, 1 case (15 sweep REQUIREs + the 2 anti-vacuity pins).
//
// WHAT STILL GUARDS THE SAME PROPERTY, and the honest limit of it. AC (b)
// (RisingEdge.HeldButtonEdgesExactlyOnce, above) sweeps d in {0,1,2,4,8} and
// pins the new and old outcomes SEPARATELY at each d; at d = 0 its `expectedOld`
// degenerates to `{ kPressTick }` — the same list it independently requires of
// the new source — so the two are pinned to the same value there and any
// divergence fails the case. That is the degenerate-equivalence claim, over the
// ABSTRACT applied-column model (AppliedColumnHistory), which needs no cache to
// exist. It is NOT a whole-map byte-comparison and does not pretend to be; what
// is gone is exactly the claim about the retired CONTAINER, which is the claim
// the container's deletion made unaskable.
// ===========================================================================

// ---------------------------------------------------------------------------
// AC (c), second half — TICK 0: ABSENT IS ABSENT UNDER THE NEW SOURCE.
//
// [og-netcode-v2-input-relay T16] THIS CASE IS HALF RETIRED, DELIBERATELY.
//
// It used to make two claims side by side. The first was about the OLD cache
// source: StateCorrectionCache's constructor does m_tickBuffer.fill(0), so every
// unwritten slot claims tick 0, `getCacheIndex(0)` resolves to a real index for a
// slot that was never a prediction slot, and `getInput` on it then OG_CHECK-fails
// on an empty optional. That claim, and the 25 assertions that carried it (an
// 11-tick x 2 loop proving index 0 was never a write target, plus 3 direct
// getCacheIndex assertions), DIE WITH THE COLUMN — the phantom was a property of
// the input column's optional, and there is no longer an input read for it to
// trap. Nothing about it can be demonstrated against the post-T16 tree.
//
// The second claim is about the DELAY LINE and stands entirely on its own: its
// slots start at tick = -1, occupied = false, has() is the gate the adapter
// consults, and matching at a tick whose search range includes tick 0 stays
// well-defined. That half is KEPT VERBATIM below — 5 assertions, all of which
// only ever named `line` and `history`.
// ---------------------------------------------------------------------------
TEST_CASE("MotionMatcherSource.TickZero.DelayLineHasNoPhantomSlot",
          "[InputSequence][MotionMatcherSource]")
{
	using namespace motionsourcetests;

	constexpr uint32 kFirstTick = 100u;
	constexpr uint32 kLastTick  = 110u;

	LocalInputCache<simulatableBrawler::PlayerInput> line(
		simulatableBrawler::getZeroPlayerInput());

	for (uint32 tick = kFirstTick; tick <= kLastTick; ++tick)
	{
		line.push(static_cast<int32>(tick), composite(glm::vec2(0.f, 0.f), false, false));
	}

	// THE NEW SOURCE: absent is absent. Tick 0 is not special to it — it reads
	// exactly like any other never-pushed tick, which is the whole property.
	const simulatableBrawler::DelayLineMotionHistory history(line);
	REQUIRE(line.has(0) == false);
	REQUIRE(history.at(0u) == nullptr);
	REQUIRE(history.at(kFirstTick - 1u) == nullptr);
	REQUIRE(history.at(kFirstTick) != nullptr);

	// And it stays well-defined at a tick whose search range includes tick 0.
	const auto fields = liveFields();
	REQUIRE(simulatableBrawler::resolveTriggeredActionId(
		        simulatableBrawler::DelayLineMotionHistory(line), 1u, fields,
		        /*leftAttack*/ true, /*rightAttack*/ false, kDeadzone, kGameMotions)
	        == inputSequence::kNoMatch);
}

// ---------------------------------------------------------------------------
// AM-4 — THE ADAPTER IS has()-GATED, and that is observable.
//
// LocalInputCache::at() answers an absent tick with the NEUTRAL input and
// never returns null. matchSequence's contract is nullptr-for-out-of-window. The
// two are near-equivalent in outcome (the neutral's (0,0,1) aim makes
// aimRelativeAngle return nullopt, so the matcher skips the entry either way),
// but only the gate keeps "no capture" and "a neutral capture" different facts
// in the data — which is what AC (c)'s byte-identical claim rests on.
//
// Also pins the tick-domain round trip: matchSequence casts a below-zero search
// tick to uint32_t, and the adapter must recover it as absent rather than as a
// wildly out-of-range positive tick.
// ---------------------------------------------------------------------------
TEST_CASE("MotionMatcherSource.Adapter.AbsentTickIsNullNotNeutral",
          "[InputSequence][MotionMatcherSource]")
{
	using namespace motionsourcetests;

	LocalInputCache<simulatableBrawler::PlayerInput> line(
		simulatableBrawler::getZeroPlayerInput());

	const auto packed = composite(stickForAngle(inputSequence::angle::Back), true, false);
	line.push(50, packed);

	const simulatableBrawler::DelayLineMotionHistory history(line);

	// Present tick: the machine sub-input of the stored composite, by reference.
	const dAttackMachineSimulation::PlayerInput* present = history.at(50u);
	REQUIRE(present != nullptr);
	REQUIRE(present->attackLeft == true);
	REQUIRE(present == &line.at(50).get<dAttackMachineSimulation::PlayerInput>());

	// Absent tick: the LINE hands back the neutral (a real object, never null)...
	REQUIRE(line.has(49) == false);
	REQUIRE(line.at(49).get<dAttackMachineSimulation::PlayerInput>().attackLeft == false);
	// ...while the ADAPTER reports absence. This is the gate.
	REQUIRE(history.at(49u) == nullptr);

	// Below-zero search ticks arrive here as very large unsigned values.
	REQUIRE(history.at(static_cast<std::uint32_t>(-1)) == nullptr);
	REQUIRE(history.at(static_cast<std::uint32_t>(-30)) == nullptr);
}

// ---------------------------------------------------------------------------
// AM-8 — CAPACITY COUPLING PIN.
//
// The matcher's deepest reach is inputSequence::kHistoryWindowFrames, because
// matchSequence hard-floors its search there. The delay line must retain at
// least that many ticks or the window is silently truncated — a motion simply
// stops matching, with nothing logged anywhere.
//
// BrawlerMotionMatching.h carries the same relation as a static_assert; this
// case exists so a violation reports as a named failing test rather than as a
// compiler diagnostic in whichever translation unit happened to include the
// header first.
//
// THE MARGIN IS INDEPENDENT OF THE INPUT DELAY. The line is keyed by CAPTURE
// tick, so relayDelayFloorTicks (and the tier delay) change WHICH tick
// collectInputAll reads for the applied value, never which ticks are resident.
// This bound therefore holds unchanged at floor 0, at the scenario-4 floor of 8,
// and at the hard cap.
// ---------------------------------------------------------------------------
TEST_CASE("MotionMatcherSource.CapacityPin.DelayLineOutlivesTheMatcherWindow",
          "[InputSequence][MotionMatcherSource]")
{
	REQUIRE(simulatableBrawler::kMotionMatcherDeepestReachTicks
	        == static_cast<std::size_t>(inputSequence::kHistoryWindowFrames));

	REQUIRE(kLocalInputCacheCapacityTicks
	        >= simulatableBrawler::kMotionMatcherDeepestReachTicks
	           + simulatableBrawler::kMotionMatcherResidencyMarginTicks);

	// A default-constructed line really does retain that many distinct ticks —
	// the constant and the container agree.
	LocalInputCache<simulatableBrawler::PlayerInput> line(
		simulatableBrawler::getZeroPlayerInput());
	REQUIRE(line.capacity() == kLocalInputCacheCapacityTicks);

	constexpr int kNewestTick = 500;
	const int reach = static_cast<int>(simulatableBrawler::kMotionMatcherDeepestReachTicks);
	for (int tick = kNewestTick - reach; tick <= kNewestTick; ++tick)
		line.push(tick, simulatableBrawler::getZeroPlayerInput());

	const simulatableBrawler::DelayLineMotionHistory history(line);
	for (int tick = kNewestTick - reach; tick <= kNewestTick; ++tick)
	{
		CAPTURE(tick);
		REQUIRE(history.at(static_cast<std::uint32_t>(tick)) != nullptr);
	}
}

#endif // WITH_LOW_LEVEL_TESTS
