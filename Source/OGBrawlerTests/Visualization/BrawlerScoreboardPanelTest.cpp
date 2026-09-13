// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins brawlerScoreboardVisualization -- the ring-out scoreboard's PURE CORE: the row
// model, the ordering, the clamps, and the three-column layout the on-screen board is
// drawn from [ringout task 6].
//
// WHY THESE CLAIMS LIVE IN A CATCH2 SUITE AT ALL. The renderer is UE code and
// Source/OGBrawlerTests links { Core, OGSimulation, OGBrawler } and NOT OGBrawlerUnreal,
// so nothing written against a canvas is reachable from here. The claims that decide
// whether the board is USEFUL rather than merely correct are therefore made in pure code:
//
//   1. TWO PEERS SEE THE SAME PLAYER IN THE SAME ROW. The gather walks an unordered-map
//      sweep, so without the ordering fold the row a player occupies depends on who
//      joined first on that machine. This is the claim with real stakes.
//   2. THE BOARD IS FLUSH RIGHT AT EVERY SCALE. Right-flush is the one thing the input
//      pane's flush-LEFT mirror could not inherit, because x = 0 needs no measurement and
//      x = viewportWidth - width needs two.
//   3. A CONSOLE VALUE OUTSIDE ITS RANGE IS PULLED TO THE NEARER END -- including a
//      non-number, which would otherwise multiply the entire layout into NaN.
//
// THE SCALE-THEN-PLACE ORDER IS THE ONE THAT HIDES. Centring an unscaled height is exact
// at scale 1 and wrong at every other, so a suite that only ran the shipped default would
// never see it. Both the seeded mistake and the shipped answer are computed below and
// asserted to AGREE at 1 and DISAGREE everywhere else.
//
// ⚠ THE TAG SHAPE IS DELIBERATE. `[CharacterViz]` is the whitelisted term in
// `OgTagAliases.cpp`'s `[@og]` alias; `[BrawlerRingoutScoreboard]` is a sub-tag and is NOT
// itself whitelisted -- exactly as the 199 existing input-history cases ride in as
// `[CharacterViz][InputHistoryViz]`. Putting the whitelisted tag FIRST is what makes these
// cases visible to `[@og]` without an alias edit.

#include "catch_amalgamated.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationLanes.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPanel.h"
#include "OGBrawler/BrawlerScoreboardVisualization.h"

namespace ringoutscoreboardtests
{

using brawlerScoreboardVisualization::ScoreboardInk;
using brawlerScoreboardVisualization::ScoreboardLayout;
using brawlerScoreboardVisualization::ScoreboardRow;
using brawlerScoreboardVisualization::ScoreboardSwatchRect;
using brawlerScoreboardVisualization::clampScoreboardBackgroundAlpha;
using brawlerScoreboardVisualization::clampScoreboardScale;
using brawlerScoreboardVisualization::kScoreboardBackdropInk;
using brawlerScoreboardVisualization::kScoreboardDeadRowInk;
using brawlerScoreboardVisualization::kScoreboardDefaultBackgroundAlpha;
using brawlerScoreboardVisualization::kScoreboardDefaultScale;
using brawlerScoreboardVisualization::kScoreboardLiveRowInk;
using brawlerScoreboardVisualization::kScoreboardMaxBackgroundAlpha;
using brawlerScoreboardVisualization::kScoreboardMaxRows;
using brawlerScoreboardVisualization::kScoreboardMaxScale;
using brawlerScoreboardVisualization::kScoreboardMinBackgroundAlpha;
using brawlerScoreboardVisualization::kScoreboardMinScale;
using brawlerScoreboardVisualization::orderedScoreboardRows;
using brawlerScoreboardVisualization::placedScoreboardLayout;
using brawlerScoreboardVisualization::scaledScoreboardLayout;
using brawlerScoreboardVisualization::scoreboardCenteredOriginY;
using brawlerScoreboardVisualization::scoreboardDrawnRowCount;
using brawlerScoreboardVisualization::scoreboardHeight;
using brawlerScoreboardVisualization::scoreboardRightFlushOriginX;
using brawlerScoreboardVisualization::scoreboardRowDrawsCountdown;
using brawlerScoreboardVisualization::scoreboardRowInk;
using brawlerScoreboardVisualization::scoreboardRowPrecedes;
using brawlerScoreboardVisualization::scoreboardRowSwatch;
using brawlerScoreboardVisualization::scoreboardRowTopY;
using brawlerScoreboardVisualization::scoreboardSwatchRect;
using brawlerScoreboardVisualization::scoreboardWindowHeight;
using brawlerScoreboardVisualization::sortScoreboardRowsById;

static bool near(float left, float right, float tolerance = 1e-3f)
{
	return std::abs(left - right) < tolerance;
}

// The scales every sweep below runs, spanning both clamp ends and the shipped default.
// The same six the input-history panel suite probes, so a difference between the two
// boards is never a difference in where they were sampled.
constexpr float kProbedScales[] = { 0.25f, 0.5f, 1.f, 1.5f, 2.5f, 4.f };

constexpr std::size_t kProbedScaleCount = 6u;

// Four viewports, not one: a placement bug is invisible at whatever size you happened to
// test, and right-flush is the placement that reads the WIDTH -- so the ultrawide, which
// the panel suite added for the aspect ratio, is load-bearing here for a second reason.
struct ProbedViewport
{
	float width;
	float height;
};

constexpr ProbedViewport kProbedViewports[] = {
	ProbedViewport{ 1280.f, 720.f },
	ProbedViewport{ 1920.f, 1080.f },
	ProbedViewport{ 2560.f, 1440.f },
	ProbedViewport{ 3440.f, 1440.f },
};

constexpr std::size_t kProbedViewportCount = 4u;

// Row counts spanning an empty board, a one-player board, the advisory character cap
// (`kPreDietCharacterCap` = 4 -- a warning, not an enforced ceiling) and a full board at
// this header's own cap.
constexpr std::size_t kProbedRowCounts[] = { 0u, 1u, 2u, 4u, 8u };

constexpr std::size_t kProbedRowCountCount = 5u;

// Where the board's own vertical middle lands, given a placed layout and the rows it drew.
static float scoreboardCenterY(const ScoreboardLayout& layout, std::size_t drawnRowCount)
{
	return layout.originY + scoreboardHeight(layout, drawnRowCount) * 0.5f;
}

// ⚠ THE FIRST EIGHT ENTRIES OF THE SHIPPED BRAWLER PALETTE, TRANSCRIBED AS TEST DATA --
// and the honest statement of what that is worth, because a transcribed constant that
// silently drifts from its source is worse than no constant at all.
//
// ⛔ THIS COPY IS NOT THE AUTHORITY AND NOTHING BELOW TREATS IT AS ONE. The palette itself
// is `kBrawlerPalette` in `OGBrawlerUECharacter.cpp`, which this target does not link, so
// no assertion here can be a claim ABOUT it. Two other mechanisms carry that claim: a
// `static_assert` at the palette's own declaration pins its CARDINALITY against
// `kScoreboardMaxRows`, and review pins the entries. What these eight values are for is a
// REALISM ARM -- the properties asserted below (distinct in, distinct out; unchanged by the
// dead flag) are properties of the BOARD, and they are worth checking against colours that
// are actually close together rather than only against synthetic opposites. If the shipped
// palette changes, nothing here goes red and nothing here is wrong; it simply stops being
// the realistic sample it was chosen to be.
constexpr ScoreboardInk kTranscribedPaletteSample[] = {
	ScoreboardInk{ 0.90f, 0.20f, 0.20f }, // red
	ScoreboardInk{ 0.20f, 0.45f, 0.95f }, // blue
	ScoreboardInk{ 0.95f, 0.75f, 0.10f }, // amber
	ScoreboardInk{ 0.20f, 0.75f, 0.35f }, // green
	ScoreboardInk{ 0.70f, 0.30f, 0.85f }, // violet
	ScoreboardInk{ 0.15f, 0.80f, 0.80f }, // cyan
	ScoreboardInk{ 0.95f, 0.50f, 0.15f }, // orange
	ScoreboardInk{ 0.95f, 0.45f, 0.70f }, // pink
};

constexpr std::size_t kTranscribedPaletteCount = 8u;

// How far apart two swatches are, on the channel that separates them MOST. Chebyshev
// rather than a sum, because two colours that differ a lot in one channel are told apart
// easily even when the other two agree -- which is how the shipped palette is built.
static float swatchSeparation(const ScoreboardInk& left, const ScoreboardInk& right)
{
	const float dr = std::abs(left.r - right.r);
	const float dg = std::abs(left.g - right.g);
	const float db = std::abs(left.b - right.b);
	return std::max(dr, std::max(dg, db));
}

// The closest any two swatches in a set come to each other. This is the quantity column one
// exists to keep large: if it reaches zero, two rows are indistinguishable.
static float minimumSwatchSeparation(const std::vector<ScoreboardInk>& swatches)
{
	float smallest = std::numeric_limits<float>::max();
	for (std::size_t a = 0u; a < swatches.size(); ++a)
	{
		for (std::size_t b = a + 1u; b < swatches.size(); ++b)
		{
			const float separation = swatchSeparation(swatches[a], swatches[b]);
			if (separation < smallest)
				smallest = separation;
		}
	}
	return smallest;
}

// A roster in a DELIBERATELY WRONG order, with a payload that differs per id so that
// "the ids came out sorted" and "each row kept its own score" are separable claims.
// Ids 7, 2, 9, 4 -- neither ascending, nor descending, nor reverse of the vector order.
//
// ⭐ EACH ROW ALSO CARRIES ITS OWN TINT [ringout task 10], and a DIFFERENT one, so that
// "the swatch travelled with the id" is a third separable claim on the same fixture. Four
// identical tints here would have made the ordering fold look correct however it moved
// them.
static std::vector<ScoreboardRow> shuffledRoster()
{
	std::vector<ScoreboardRow> rows;
	rows.push_back(ScoreboardRow{ 7u, 300u, false, 0u, kTranscribedPaletteSample[0] });
	rows.push_back(ScoreboardRow{ 2u, 100u, true, 45u, kTranscribedPaletteSample[1] });
	rows.push_back(ScoreboardRow{ 9u, 400u, true, 7u, kTranscribedPaletteSample[2] });
	rows.push_back(ScoreboardRow{ 4u, 200u, false, 0u, kTranscribedPaletteSample[3] });
	return rows;
}

// Same-swatch test, spelled once: an EXACT comparison, because a fold that merely
// approximated a tint would still be a fold that rewrote it.
static bool sameSwatch(const ScoreboardInk& left, const ScoreboardInk& right)
{
	return left.r == right.r && left.g == right.g && left.b == right.b;
}

} // namespace ringoutscoreboardtests

// ---------------------------------------------------------------------------
// A row is five plain fields, and its defaults are part of the contract: a fighter the
// score system seeded but who has never scored draws a row of `0`, not a missing row.
//
// ⭐ FIVE, NOT FOUR, SINCE [ringout task 10] -- the fifth is the brawler's own tint, which
// is what column one draws now that the raw id has stopped being shown. The case name and
// this comment moved together with the field on purpose: a suite that still said "four"
// would be the first place a reader looked and the first place they were misled.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.ARowIsFivePlainFieldsAndZeroIsARealScore",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardRow fresh;

	CHECK(fresh.characterId == 0u);
	CHECK(fresh.score == 0u);
	CHECK(fresh.isDead == false);
	CHECK(fresh.ticksUntilRespawn == 0u);

	// ⛔ THE DEFAULT SWATCH IS WHITE, AND IT IS NOT AN ARBITRARY PLACEHOLDER.
	// `AOGBrawlerUECharacter::BrawlerColor` itself defaults to `FLinearColor::White`, so a
	// brawler that has not been possessed yet -- or whose colour has not replicated in yet
	// -- draws the swatch its MESH is actually wearing in that window. Board and world
	// agree even before the palette has spoken.
	CHECK(near(fresh.swatch.r, 1.f));
	CHECK(near(fresh.swatch.g, 1.f));
	CHECK(near(fresh.swatch.b, 1.f));

	// A fighter in play draws no countdown, whatever the stale tick field happens to hold.
	// ⛔ THE GATE IS THE LEVEL, NEVER THE COUNTDOWN BEING NON-ZERO. `respawnAtTick` is
	// left at its last value once the dead bit clears, so a living fighter carrying a
	// non-zero remainder is the NORMAL state one tick after a respawn, not a corrupt one.
	const ScoreboardRow aliveWithStaleCountdown{ 3u, 5u, false, 99u };
	CHECK(scoreboardRowDrawsCountdown(aliveWithStaleCountdown) == false);

	const ScoreboardRow out{ 3u, 5u, true, 0u };
	CHECK(scoreboardRowDrawsCountdown(out) == true);

	// The countdown is drawn on the tick it reaches zero too -- that tick is still a dead
	// tick, and a row that blanked one tick early would blink.
	static_assert(scoreboardRowDrawsCountdown(ScoreboardRow{ 0u, 0u, true, 0u }),
	              "a dead fighter draws column three even at a zero remainder");
	static_assert(!scoreboardRowDrawsCountdown(ScoreboardRow{ 0u, 0u, false, 9u }),
	              "a living fighter never draws column three");
}

// ---------------------------------------------------------------------------
// ⭐ THE ORDERING, WHICH IS THE CLAIM WITH REAL STAKES. The UE gather walks a
// `StorageView` sweep, whose order is unordered-map order -- unspecified, and varying with
// registration history. Without this fold, two peers watching the same match see the same
// scores in different rows.
//
// Sorted IDS and CARRIED PAYLOAD are asserted separately: a fold that sorted a vector of
// ids and then re-read the scores from the unsorted source would pass the first and fail
// the second, and it would look right on a board where everyone has the same score.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.ShuffledRowsComeOutSortedByIdAndEachRowKeepsItsOwnPayload",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const std::vector<ScoreboardRow> ordered = orderedScoreboardRows(shuffledRoster());

	REQUIRE(ordered.size() == 4u);

	CHECK(ordered[0].characterId == 2u);
	CHECK(ordered[1].characterId == 4u);
	CHECK(ordered[2].characterId == 7u);
	CHECK(ordered[3].characterId == 9u);

	// The payload travelled WITH the id, not with the slot it used to sit in.
	CHECK(ordered[0].score == 100u);
	CHECK(ordered[1].score == 200u);
	CHECK(ordered[2].score == 300u);
	CHECK(ordered[3].score == 400u);

	// ⭐ AND SO DID THE TINT. Id 2 was pushed SECOND carrying blue and comes out FIRST
	// still carrying blue; id 7 was pushed FIRST carrying red and comes out THIRD still
	// carrying red. A fold that sorted the ids and re-read the colours from the unsorted
	// source passes every line above this one and fails these four.
	CHECK(sameSwatch(ordered[0].swatch, kTranscribedPaletteSample[1]));  // id 2 -- blue
	CHECK(sameSwatch(ordered[1].swatch, kTranscribedPaletteSample[3]));  // id 4 -- green
	CHECK(sameSwatch(ordered[2].swatch, kTranscribedPaletteSample[0]));  // id 7 -- red
	CHECK(sameSwatch(ordered[3].swatch, kTranscribedPaletteSample[2]));  // id 9 -- amber

	CHECK(ordered[0].isDead == true);
	CHECK(ordered[0].ticksUntilRespawn == 45u);
	CHECK(ordered[1].isDead == false);
	CHECK(ordered[2].isDead == false);
	CHECK(ordered[3].isDead == true);
	CHECK(ordered[3].ticksUntilRespawn == 7u);

	// ⭐ EVERY ARRIVAL ORDER, NOT ONE. The gather's order is not merely unknown, it is
	// DIFFERENT ON DIFFERENT MACHINES, so the claim is about the whole permutation group:
	// all 24 orderings of these four rows must produce the same board.
	std::vector<ScoreboardRow> permuted = shuffledRoster();
	std::sort(permuted.begin(), permuted.end(),
	          [](const ScoreboardRow& a, const ScoreboardRow& b)
	          { return a.characterId < b.characterId; });

	std::size_t permutations       = 0u;
	std::size_t agreedWithTheBoard = 0u;

	do
	{
		++permutations;

		const std::vector<ScoreboardRow> board = orderedScoreboardRows(permuted);

		bool identical = board.size() == ordered.size();
		for (std::size_t slot = 0u; identical && slot < board.size(); ++slot)
		{
			identical = board[slot].characterId == ordered[slot].characterId
			            && board[slot].score == ordered[slot].score
			            && board[slot].isDead == ordered[slot].isDead
			            && board[slot].ticksUntilRespawn == ordered[slot].ticksUntilRespawn
			            // ⭐ THE SWATCH IS IN THE COMPARISON [ringout task 10]. Left out,
			            // every permutation would still have "agreed" while column one drew
			            // four tints in whatever order the gather happened to hand them in.
			            && sameSwatch(board[slot].swatch, ordered[slot].swatch);
		}

		if (identical)
			++agreedWithTheBoard;
	} while (std::next_permutation(permuted.begin(), permuted.end(),
	                               [](const ScoreboardRow& a, const ScoreboardRow& b)
	                               { return a.characterId < b.characterId; }));

	CHECK(permutations == 24u);
	CHECK(agreedWithTheBoard == 24u);

	// The relation itself, which is what the sort is parameterised on. Irreflexive and
	// asymmetric on distinct ids, and it reads NOTHING but the id -- a comparator that
	// tie-broke on the score would reorder the board every time somebody scored.
	const ScoreboardRow low{ 2u, 900u, true, 5u };
	const ScoreboardRow high{ 9u, 0u, false, 0u };
	CHECK(scoreboardRowPrecedes(low, high));
	CHECK_FALSE(scoreboardRowPrecedes(high, low));
	CHECK_FALSE(scoreboardRowPrecedes(low, low));

	// An empty roster and a one-row roster are not special cases in this code, and the
	// board on which nobody has joined yet is a real frame.
	CHECK(orderedScoreboardRows({}).empty());
	CHECK(orderedScoreboardRows({ ScoreboardRow{ 5u, 1u, false, 0u } }).size() == 1u);
}

// ---------------------------------------------------------------------------
// ⭐ WHY `stable_sort` AND NOT `sort`. `scoreboardRowPrecedes` orders on the id alone, so
// two rows carrying the same id are TIED, and `std::sort` leaves tied elements in an
// unspecified relative order -- reintroducing exactly the machine-varying nondeterminism
// the fold exists to remove. Registration makes ids unique, so this input is illegal; the
// point is that the ordering is deterministic anyway rather than deterministic-by-luck.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TwoRowsSharingAnIdStillOrderDeterministically",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	std::vector<ScoreboardRow> duplicated;
	duplicated.push_back(ScoreboardRow{ 5u, 111u, false, 0u });
	duplicated.push_back(ScoreboardRow{ 1u, 222u, false, 0u });
	duplicated.push_back(ScoreboardRow{ 5u, 333u, false, 0u });
	duplicated.push_back(ScoreboardRow{ 5u, 444u, false, 0u });

	const std::vector<ScoreboardRow> board = orderedScoreboardRows(duplicated);

	REQUIRE(board.size() == 4u);
	CHECK(board[0].characterId == 1u);
	CHECK(board[1].characterId == 5u);
	CHECK(board[2].characterId == 5u);
	CHECK(board[3].characterId == 5u);

	// The three tied rows kept their ARRIVAL order among themselves. That is the property
	// `std::sort` does not PROMISE and `std::stable_sort` does.
	// ⚠ AND ON THIS TOOLCHAIN `std::sort` HAPPENS TO GIVE IT TOO at this size, so these
	// three lines pass either way -- see
	// `Scoreboard.TheStableSortIsAGuardThisSuiteCanOnlyDiscriminateAboveItsOwnCap` for the
	// measurement, and for why task 6's red probe against it stayed green until that case
	// drove the shipped ordering past the insertion-sort threshold.
	CHECK(board[1].score == 111u);
	CHECK(board[2].score == 333u);
	CHECK(board[3].score == 444u);

	// And running it again on its own output is a fixed point, so the board does not
	// shuffle its tied rows from frame to frame.
	const std::vector<ScoreboardRow> again = orderedScoreboardRows(board);
	REQUIRE(again.size() == 4u);
	for (std::size_t slot = 0u; slot < 4u; ++slot)
	{
		CHECK(again[slot].characterId == board[slot].characterId);
		CHECK(again[slot].score == board[slot].score);
	}

	// The in-place form is the same function, and the board is built from it.
	std::vector<ScoreboardRow> inPlace = duplicated;
	sortScoreboardRowsById(inPlace);
	REQUIRE(inPlace.size() == 4u);
	for (std::size_t slot = 0u; slot < 4u; ++slot)
		CHECK(inPlace[slot].score == board[slot].score);
}

// ---------------------------------------------------------------------------
// Two console values, two ranges, and the same contract the input pane already offers:
// OUT OF RANGE IS PULLED TO THE NEARER END, never rejected. A rejected value would leave
// the console echoing a number that nothing uses.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.EveryConsoleValueIsClampedToItsStatedRangeAndNoneOfThemRejects",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	// The shipped defaults, which are the task's ruling and belong in a test rather than
	// in a comment somebody has to trust. ⛔ `OGBrawler.ScoreboardAlpha` DEFAULTS TO 0 --
	// a fully transparent backdrop is SKIPPED, not drawn invisibly.
	CHECK(kScoreboardDefaultScale == 1.0f);
	CHECK(kScoreboardDefaultBackgroundAlpha == 0.f);

	CHECK(kScoreboardMinScale == 0.25f);
	CHECK(kScoreboardMaxScale == 4.f);
	CHECK(kScoreboardMinBackgroundAlpha == 0.f);
	CHECK(kScoreboardMaxBackgroundAlpha == 1.f);

	// The span matches the input pane's exactly, so "scale 2" means the same thing on both
	// panels for a tuner who has one of each on screen.
	CHECK(kScoreboardMinScale == brawlerInputHistoryVisualization::kPanelMinScale);
	CHECK(kScoreboardMaxScale == brawlerInputHistoryVisualization::kPanelMaxScale);

	CHECK(clampScoreboardScale(-4.f) == kScoreboardMinScale);
	CHECK(clampScoreboardScale(0.f) == kScoreboardMinScale);
	CHECK(clampScoreboardScale(0.24f) == kScoreboardMinScale);
	CHECK(clampScoreboardScale(kScoreboardMinScale) == kScoreboardMinScale);
	CHECK(clampScoreboardScale(kScoreboardDefaultScale) == kScoreboardDefaultScale);
	CHECK(clampScoreboardScale(2.5f) == 2.5f);
	CHECK(clampScoreboardScale(kScoreboardMaxScale) == kScoreboardMaxScale);
	CHECK(clampScoreboardScale(4.01f) == kScoreboardMaxScale);
	CHECK(clampScoreboardScale(1000.f) == kScoreboardMaxScale);

	// ⭐ THE NON-NUMBER CASE. A console float can arrive as NaN, and an unclamped NaN
	// multiplies every geometric field in the layout into a non-number -- a board that
	// draws nowhere, from a value the console accepted without complaint. It must land on
	// the floor, and here it does.
	// ⚠ THIS CASE DOES NOT DISCRIMINATE THE NEGATED SPELLING FROM `if (x < min)` ON THIS
	// BUILD -- see `Scoreboard.TheNaNClampIsCorrectHereForAReasonTheToolchainSupplies`,
	// which pins WHY, and the header's own note at `clampScoreboardScale`.
	CHECK(clampScoreboardScale(std::numeric_limits<float>::quiet_NaN())
	      == kScoreboardMinScale);
	CHECK(clampScoreboardScale(-std::numeric_limits<float>::quiet_NaN())
	      == kScoreboardMinScale);
	CHECK(clampScoreboardScale(std::numeric_limits<float>::signaling_NaN())
	      == kScoreboardMinScale);

	// Infinities are the other two values a console float can take. +inf is above the max
	// and lands on it; -inf fails the first test and lands on the min.
	CHECK(clampScoreboardScale(std::numeric_limits<float>::infinity())
	      == kScoreboardMaxScale);
	CHECK(clampScoreboardScale(-std::numeric_limits<float>::infinity())
	      == kScoreboardMinScale);

	CHECK(clampScoreboardBackgroundAlpha(-1.f) == kScoreboardMinBackgroundAlpha);
	CHECK(clampScoreboardBackgroundAlpha(0.f) == kScoreboardMinBackgroundAlpha);
	CHECK(clampScoreboardBackgroundAlpha(kScoreboardDefaultBackgroundAlpha)
	      == kScoreboardDefaultBackgroundAlpha);
	CHECK(clampScoreboardBackgroundAlpha(0.5f) == 0.5f);
	CHECK(clampScoreboardBackgroundAlpha(1.f) == kScoreboardMaxBackgroundAlpha);
	CHECK(clampScoreboardBackgroundAlpha(7.5f) == kScoreboardMaxBackgroundAlpha);
	CHECK(clampScoreboardBackgroundAlpha(std::numeric_limits<float>::quiet_NaN())
	      == kScoreboardMinBackgroundAlpha);
	CHECK(clampScoreboardBackgroundAlpha(std::numeric_limits<float>::infinity())
	      == kScoreboardMaxBackgroundAlpha);

	// ⭐ BOTH CLAMPS ARE `constexpr`, AND THAT IS CHECKED AT COMPILE TIME RATHER THAN
	// PROMISED. The UE accessor calls these rather than re-implementing the range at the
	// console, which is the discipline `InputHistoryVisualizationUImpl.h` already states;
	// a static_assert is what stops a later edit quietly making them runtime-only.
	static_assert(clampScoreboardScale(-1.f) == kScoreboardMinScale, "below min");
	static_assert(clampScoreboardScale(9.f) == kScoreboardMaxScale, "above max");
	static_assert(clampScoreboardScale(1.f) == 1.f, "in range is identity");
	static_assert(clampScoreboardBackgroundAlpha(-1.f) == kScoreboardMinBackgroundAlpha,
	              "below min");
	static_assert(clampScoreboardBackgroundAlpha(9.f) == kScoreboardMaxBackgroundAlpha,
	              "above max");
}

// ---------------------------------------------------------------------------
// ⚠⚠ A FINDING, PINNED RATHER THAN ASSERTED AWAY. Task 6's red probe P7/P8 -- rewrite the
// clamps' first test as the reflex `if (x < min) return min;` -- STAYED GREEN. The reason
// is not that the test is vacuous; it is that THIS TARGET COMPILES `/fp:fast`, under which
// the compiler may assume no NaN exists and rewrite `x < c` as `!(x >= c)`. Measured on
// this tree on 2026-09-13 with a `volatile`-sourced NaN: `n < 0.25f` evaluates TRUE, which
// IEEE-754 says it must not. So on THIS build both spellings land NaN on the minimum and
// nothing can tell them apart.
//
// ⛔ THE NEGATION IS STILL THE RIGHT SPELLING -- it is a PORTABILITY guard. `/fp:fast` is a
// per-module setting, so a module compiled `/fp:precise`, or a non-MSVC toolchain, restores
// the difference immediately and silently.
//
// THIS CASE PINS THE TOOLCHAIN FACT ITSELF, so it goes RED the day that flag changes --
// which is exactly the day the next reader needs to be told that the negation has become
// load-bearing again, and that the probe will bite from then on.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheNaNClampIsCorrectHereForAReasonTheToolchainSupplies",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	// `volatile` so the value cannot be folded away before the comparison is emitted --
	// this is the same rig the 2026-09-13 measurement used.
	volatile float nanSource = std::numeric_limits<float>::quiet_NaN();
	const float    notANumber = nanSource;

	// It really is a NaN when it reaches the comparisons.
	CHECK(std::isnan(notANumber));

	// The IEEE-754 half, which holds on this build too: NaN is UNORDERED, so `>=` is false.
	CHECK_FALSE(notANumber >= kScoreboardMinScale);
	CHECK_FALSE(notANumber > kScoreboardMaxScale);

	// ⛔ THE `/fp:fast` HALF, AND THE WHOLE POINT OF THIS CASE. Under IEEE-754 `NaN < x` is
	// FALSE. Here it is TRUE, because the compiler rewrote it as the negation. The day this
	// line fails, the flag changed, the two clamp spellings stopped being equivalent, and
	// the header's note at `clampScoreboardScale` has become load-bearing rather than
	// precautionary.
	const bool lessThanIsRewritten = (notANumber < kScoreboardMinScale);
	CHECK(lessThanIsRewritten);

	// Which is precisely why the shipped clamps are correct here whichever way they are
	// read -- and why a red probe against the negation cannot bite on this build.
	CHECK(clampScoreboardScale(notANumber) == kScoreboardMinScale);
	CHECK(clampScoreboardBackgroundAlpha(notANumber) == kScoreboardMinBackgroundAlpha);

	// ...and the guarantee that actually matters downstream, said without reference to
	// either spelling: NOTHING THE CONSOLE CAN TYPE PUTS A NaN INTO THE LAYOUT.
	const ScoreboardLayout poisoned = placedScoreboardLayout(
		ScoreboardLayout{}, clampScoreboardScale(notANumber), 4u, 1280.f, 720.f);
	CHECK_FALSE(std::isnan(poisoned.originX));
	CHECK_FALSE(std::isnan(poisoned.originY));
	CHECK_FALSE(std::isnan(poisoned.rowWidth));
	CHECK_FALSE(std::isnan(poisoned.rowHeight));
	CHECK_FALSE(std::isnan(poisoned.textScale));
	CHECK(near(poisoned.rowWidth, 132.f * kScoreboardMinScale));

	// The control: an UNCLAMPED NaN does poison the layout, so the clamp above is the thing
	// doing the work and not an accident of the multiplication.
	const ScoreboardLayout unguarded =
		placedScoreboardLayout(ScoreboardLayout{}, notANumber, 4u, 1280.f, 720.f);
	CHECK(std::isnan(unguarded.rowWidth));
	CHECK(std::isnan(unguarded.textScale));
}

// ---------------------------------------------------------------------------
// ⚠⚠ THE SECOND PROBE THAT DID NOT BITE -- AND THE CASE THAT THE FINDING PRODUCED.
//
// Task 6's red probe P2 (`stable_sort` swapped for `sort`) STAYED GREEN against the
// thirteen cases that existed when it first ran, every one of them at N <= 8. Measured on
// this tree on 2026-09-13: MSVC's `std::sort` falls back to INSERTION SORT at or below 32
// elements, and insertion sort is stable, so the two algorithms produce byte-identical
// output at N = 4, 8, 16 and 32 and first diverge at N = 33. This board's cap is 8 rows, so
// AT EVERY SIZE THE BOARD CAN REACH, NOTHING DISTINGUISHES THEM.
//
// ⭐ This case was written in response, and it drives the SHIPPED ordering at N = 33 too --
// purely to pin the divergence point. On the re-run P2 bit there, one failed assertion and
// only that one. ⛔ SO READ THE COVERAGE PRECISELY: the guard is covered ABOVE the cap and
// remains undiscriminated AT it. It earns its place against a cap that rises past 32, a
// standard library that moves its threshold, or another toolchain -- all edits nowhere near
// this file. Claiming more than that in a comment would be claiming coverage that the
// eight-row board does not have.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheStableSortIsAGuardThisSuiteCanOnlyDiscriminateAboveItsOwnCap",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	// The board's own cap is well inside the insertion-sort window.
	CHECK(kScoreboardMaxRows <= 32u);

	// A full board of TIED rows -- every row the same id, so every pair is a tie and any
	// instability at all would show. Both algorithms, side by side.
	std::vector<ScoreboardRow> tiedFullBoard;
	for (std::size_t index = 0u; index < kScoreboardMaxRows; ++index)
	{
		tiedFullBoard.push_back(
			ScoreboardRow{ 1u, static_cast<uint32_t>(index), false, 0u });
	}

	std::vector<ScoreboardRow> viaStable = tiedFullBoard;
	std::stable_sort(viaStable.begin(), viaStable.end(), scoreboardRowPrecedes);

	std::vector<ScoreboardRow> viaPlain = tiedFullBoard;
	std::sort(viaPlain.begin(), viaPlain.end(), scoreboardRowPrecedes);

	std::size_t agreed = 0u;
	for (std::size_t index = 0u; index < kScoreboardMaxRows; ++index)
	{
		if (viaStable[index].score == viaPlain[index].score)
			++agreed;
	}

	// ⛔ THEY AGREE AT THIS SIZE. That is the finding: a probe swapping one for the other
	// cannot go red here, and the green it returns is expected rather than a gap.
	CHECK(agreed == kScoreboardMaxRows);

	// And the shipped call agrees with both, which is the only claim the board depends on.
	const std::vector<ScoreboardRow> board = orderedScoreboardRows(tiedFullBoard);
	std::size_t matchedShipped = 0u;
	for (std::size_t index = 0u; index < kScoreboardMaxRows; ++index)
	{
		if (board[index].score == viaStable[index].score)
			++matchedShipped;
	}
	CHECK(matchedShipped == kScoreboardMaxRows);

	// ⭐ THE DIVERGENCE POINT, PINNED. Above the insertion-sort threshold the two stop
	// agreeing, which is what makes "insertion sort is why the probe stayed green" a
	// measurement rather than an explanation. ⚠ This asserts a STANDARD-LIBRARY property,
	// deliberately: if it ever fails, the threshold moved and the note in the header
	// naming 32 needs rewriting.
	std::vector<ScoreboardRow> wide;
	for (std::size_t index = 0u; index < 33u; ++index)
	{
		wide.push_back(ScoreboardRow{ static_cast<unsigned int>(index % 3u),
		                              static_cast<uint32_t>(index), false, 0u });
	}

	std::vector<ScoreboardRow> wideStable = wide;
	std::stable_sort(wideStable.begin(), wideStable.end(), scoreboardRowPrecedes);

	std::vector<ScoreboardRow> widePlain = wide;
	std::sort(widePlain.begin(), widePlain.end(), scoreboardRowPrecedes);

	bool identicalAt33 = true;
	for (std::size_t index = 0u; index < 33u; ++index)
		identicalAt33 = identicalAt33 && wideStable[index].score == widePlain[index].score;

	CHECK_FALSE(identicalAt33);

	// ...and at 32, one element fewer, they still agree.
	std::vector<ScoreboardRow> narrow(wide.begin(), wide.begin() + 32);

	std::vector<ScoreboardRow> narrowStable = narrow;
	std::stable_sort(narrowStable.begin(), narrowStable.end(), scoreboardRowPrecedes);

	std::vector<ScoreboardRow> narrowPlain = narrow;
	std::sort(narrowPlain.begin(), narrowPlain.end(), scoreboardRowPrecedes);

	bool identicalAt32 = true;
	for (std::size_t index = 0u; index < 32u; ++index)
	{
		identicalAt32 =
			identicalAt32 && narrowStable[index].score == narrowPlain[index].score;
	}

	CHECK(identicalAt32);

	// ⛔ WHAT THE BOARD ACTUALLY DEPENDS ON, and it is true at every size: the shipped
	// ordering is IDEMPOTENT and agrees with `stable_sort` whatever the population.
	const std::vector<ScoreboardRow> wideBoard  = orderedScoreboardRows(wide);
	const std::vector<ScoreboardRow> wideAgain  = orderedScoreboardRows(wideBoard);
	std::size_t stayedPut = 0u;
	for (std::size_t index = 0u; index < 33u; ++index)
	{
		if (wideBoard[index].score == wideStable[index].score
		    && wideAgain[index].score == wideBoard[index].score)
		{
			++stayedPut;
		}
	}
	CHECK(stayedPut == 33u);
}

// ---------------------------------------------------------------------------
// ONE FACTOR, REACHING BOTH THE GEOMETRY AND THE GLYPHS. `GetSmallFont()` is FIXED-SIZE,
// so `textScale` is the only thing that makes the text grow with the rows -- and a
// `GetTextSize` call that omits it measures unscaled glyphs, which is how a right-aligned
// column drifts further out of true the larger the board gets.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.OneFactorScalesBothTheGeometryAndTheTextAndTheyCannotDriftApart",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	std::size_t swept            = 0u;
	std::size_t heightScaled     = 0u;
	std::size_t widthScaled      = 0u;
	std::size_t idScaled         = 0u;
	std::size_t scoreScaled      = 0u;
	std::size_t statusScaled     = 0u;
	std::size_t textOffsetScaled = 0u;
	std::size_t textScaled       = 0u;
	std::size_t rowCapUnscaled   = 0u;
	std::size_t originsUntouched = 0u;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const float            scale  = kProbedScales[index];
		const ScoreboardLayout scaled = scaledScoreboardLayout(base, scale);

		++swept;

		if (near(scaled.rowHeight, base.rowHeight * scale))
			++heightScaled;
		if (near(scaled.rowWidth, base.rowWidth * scale))
			++widthScaled;
		// ⭐ COLUMN ONE IS THREE FIELDS SINCE [ringout task 10] -- an offset, a width and
		// an inset -- and ALL THREE must take the same factor. A width that scaled while
		// the inset did not would change the swatch's proportions with the board's size.
		if (near(scaled.swatchX, base.swatchX * scale)
		    && near(scaled.swatchWidth, base.swatchWidth * scale)
		    && near(scaled.swatchInsetY, base.swatchInsetY * scale))
			++idScaled;
		if (near(scaled.scoreRightX, base.scoreRightX * scale))
			++scoreScaled;
		if (near(scaled.statusRightX, base.statusRightX * scale))
			++statusScaled;
		if (near(scaled.textOffsetY, base.textOffsetY * scale))
			++textOffsetScaled;

		// ⛔ THE ONE THAT HIDES. A layout whose geometry scaled and whose textScale did
		// not would give a bigger box holding the same tiny glyphs, and every right-edged
		// column would sit further from its number the larger the board got.
		if (near(scaled.textScale, base.textScale * scale))
			++textScaled;

		// ⚠ A COUNT IS NOT A LENGTH. Scaling `maxRows` would change how many fighters the
		// board can show when the user changes how big it is.
		if (scaled.maxRows == base.maxRows)
			++rowCapUnscaled;

		// ⛔ SCALING DECIDES SIZE ONLY. Where the board sits is placement, decided after.
		if (scaled.originX == base.originX && scaled.originY == base.originY)
			++originsUntouched;
	}

	CHECK(swept == kProbedScaleCount);
	CHECK(heightScaled == swept);
	CHECK(widthScaled == swept);
	CHECK(idScaled == swept);
	CHECK(scoreScaled == swept);
	CHECK(statusScaled == swept);
	CHECK(textOffsetScaled == swept);
	CHECK(textScaled == swept);
	CHECK(rowCapUnscaled == swept);
	CHECK(originsUntouched == swept);

	// The geometry and the text move by the SAME ratio, said as a ratio rather than as two
	// multiplications -- this is the assertion that fails if the two ever get separate
	// factors, even if both are still "scaled".
	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const ScoreboardLayout scaled =
			scaledScoreboardLayout(base, kProbedScales[index]);
		CHECK(near(scaled.rowHeight / base.rowHeight, scaled.textScale / base.textScale));
		CHECK(near(scaled.rowWidth / base.rowWidth, scaled.textScale / base.textScale));
	}
}

// ---------------------------------------------------------------------------
// At the shipped default, every number is still the one the board was authored from. A
// scale pass that quietly rounded or re-derived a field would show up here first.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.AtScaleOneEveryLayoutNumberIsStillTheOneTheBoardWasDrawnFrom",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	CHECK(near(base.rowHeight, 18.f));
	CHECK(near(base.rowWidth, 132.f));
	CHECK(near(base.swatchX, 6.f));
	CHECK(near(base.swatchWidth, 30.f));
	CHECK(near(base.swatchInsetY, 3.f));
	CHECK(near(base.scoreRightX, 78.f));
	CHECK(near(base.statusRightX, 126.f));
	CHECK(near(base.textOffsetY, 2.f));
	CHECK(near(base.textScale, 1.f));
	CHECK(base.maxRows == kScoreboardMaxRows);

	const ScoreboardLayout identity = scaledScoreboardLayout(base, kScoreboardDefaultScale);

	CHECK(near(identity.rowHeight, base.rowHeight));
	CHECK(near(identity.rowWidth, base.rowWidth));
	CHECK(near(identity.swatchX, base.swatchX));
	CHECK(near(identity.swatchWidth, base.swatchWidth));
	CHECK(near(identity.swatchInsetY, base.swatchInsetY));
	CHECK(near(identity.scoreRightX, base.scoreRightX));
	CHECK(near(identity.statusRightX, base.statusRightX));
	CHECK(near(identity.textOffsetY, base.textOffsetY));
	CHECK(near(identity.textScale, base.textScale));
	CHECK(identity.maxRows == base.maxRows);
}

// ---------------------------------------------------------------------------
// THE ROW, LEFT TO RIGHT -- EXACTLY THREE COLUMNS, and no column leaves the row at any
// scale. The columns are checked as a strict ordering rather than as three numbers,
// because the failure that matters is two of them crossing, not either one moving.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheRowIsThreeOrderedColumnsAndNoneOfThemLeavesIt",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	std::size_t swept      = 0u;
	std::size_t ordered    = 0u;
	std::size_t insideRow  = 0u;
	std::size_t rowsDescend = 0u;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const ScoreboardLayout layout = placedScoreboardLayout(
			base, kProbedScales[index], 4u, 1920.f, 1080.f);

		++swept;

		// ⭐ COLUMN ONE IS A RECTANGLE SINCE [ringout task 10], so its RIGHT edge is
		// `swatchX + swatchWidth` rather than the single x an id text needed. The ordering
		// claim is the stronger one for it: the swatch must not merely start left of the
		// score column, it must END left of it.
		if (layout.swatchX + layout.swatchWidth < layout.scoreRightX
		    && layout.scoreRightX < layout.statusRightX)
			++ordered;

		if (layout.swatchX > 0.f && layout.statusRightX < layout.rowWidth)
			++insideRow;

		// Strictly increasing in the slot, which IS top-down: slot 0 is the topmost row,
		// and on a canvas whose y grows DOWNWARD the topmost row is the smallest y.
		bool descending = near(scoreboardRowTopY(layout, 0u), layout.originY);
		for (std::size_t slot = 1u; slot < kScoreboardMaxRows; ++slot)
		{
			descending = descending
			             && scoreboardRowTopY(layout, slot)
			                    > scoreboardRowTopY(layout, slot - 1u)
			             && near(scoreboardRowTopY(layout, slot)
			                         - scoreboardRowTopY(layout, slot - 1u),
			                     layout.rowHeight);
		}
		if (descending)
			++rowsDescend;
	}

	CHECK(swept == kProbedScaleCount);
	CHECK(ordered == swept);
	CHECK(insideRow == swept);
	CHECK(rowsDescend == swept);

	// A worked row in absolute pixels at scale 2, so the three columns are pinned as
	// numbers and not only as an ordering.
	const ScoreboardLayout twice = scaledScoreboardLayout(base, 2.f);
	CHECK(near(twice.swatchX, 12.f));
	CHECK(near(twice.swatchWidth, 60.f));
	CHECK(near(twice.swatchInsetY, 6.f));
	CHECK(near(twice.scoreRightX, 156.f));
	CHECK(near(twice.statusRightX, 252.f));
	CHECK(near(twice.rowWidth, 264.f));
	CHECK(near(twice.rowHeight, 36.f));
	CHECK(near(twice.textOffsetY, 4.f));
	CHECK(near(twice.textScale, 2.f));
}

// ---------------------------------------------------------------------------
// ⭐ THE RIGHT-FLUSH CLAIM. The board's right edge sits exactly ON the viewport's right
// edge -- at every scale, at every viewport, and for every row count. This is the mirror
// of `kPanelLeftEdgeX = 0.f`, and it is the one placement the input pane could not hand
// over unchanged, because x = 0 needs no measurement and this needs the width.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheBoardIsFlushRightAtEveryScaleAndEveryViewport",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	std::size_t swept          = 0u;
	std::size_t rightEdgeFlush = 0u;
	std::size_t widthUnmoved   = 0u;

	for (std::size_t view = 0u; view < kProbedViewportCount; ++view)
	{
		const ProbedViewport viewport = kProbedViewports[view];

		for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
		{
			for (std::size_t count = 0u; count < kProbedRowCountCount; ++count)
			{
				const ScoreboardLayout layout =
					placedScoreboardLayout(base, kProbedScales[index],
					                       kProbedRowCounts[count],
					                       viewport.width, viewport.height);

				++swept;

				if (near(layout.originX + layout.rowWidth, viewport.width))
					++rightEdgeFlush;

				// ⛔ THE ROW COUNT MUST NOT REACH x. An originX that moved with the number
				// of players would slide the board sideways every time somebody joined.
				if (near(layout.rowWidth, base.rowWidth * kProbedScales[index]))
					++widthUnmoved;
			}
		}
	}

	CHECK(swept == kProbedViewportCount * kProbedScaleCount * kProbedRowCountCount);
	CHECK(rightEdgeFlush == swept);
	CHECK(widthUnmoved == swept);

	// THE SEEDED MISTAKE: flush the UNSCALED width. It agrees at scale 1 and disagrees at
	// every other, and the disagreement is exactly the board's overhang off the screen.
	std::size_t agreedAtOne     = 0u;
	std::size_t disagreedBeyond = 0u;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const float scale = kProbedScales[index];
		const float wrong = scoreboardRightFlushOriginX(1920.f, base.rowWidth);
		const float right =
			placedScoreboardLayout(base, scale, 4u, 1920.f, 1080.f).originX;

		if (scale == 1.f && near(wrong, right))
			++agreedAtOne;

		if (scale != 1.f && !near(wrong, right))
			++disagreedBeyond;
	}

	CHECK(agreedAtOne == 1u);
	CHECK(disagreedBeyond == kProbedScaleCount - 1u);

	// A worked case in absolute pixels: 132 px at scale 1.5 is 198, so the origin is
	// 1280 - 198 at 720p and the right edge is back on 1280.
	const ScoreboardLayout worked =
		placedScoreboardLayout(base, 1.5f, 4u, 1280.f, 720.f);
	CHECK(near(worked.rowWidth, 198.f));
	CHECK(near(worked.originX, 1082.f));
	CHECK(near(worked.originX + worked.rowWidth, 1280.f));

	// And it is the opposite edge from the input pane's, which is what makes the two read
	// as a pair rather than as two panels fighting for one corner.
	CHECK(worked.originX > brawlerInputHistoryVisualization::kPanelLeftEdgeX);
}

// ---------------------------------------------------------------------------
// ⭐⭐ SCALE THEN PLACE, AND THE HEIGHT CENTRED IS THE DRAWN ONE.
//
// Two separate claims, and both are invisible if only the shipped default is run:
//   * centring an UNSCALED height is exact at scale 1 and off-centre at every other;
//   * centring the RESERVED window height instead of the DRAWN height is exact only on a
//     full board, and draws a two-player board visibly high for the whole match.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheBoardIsCentredOnTheScaledDrawnHeightAtEveryViewport",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	std::size_t swept           = 0u;
	std::size_t centred         = 0u;
	std::size_t marginsEqual    = 0u;
	std::size_t heightWasScaled = 0u;

	for (std::size_t view = 0u; view < kProbedViewportCount; ++view)
	{
		const ProbedViewport viewport = kProbedViewports[view];

		for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
		{
			for (std::size_t count = 0u; count < kProbedRowCountCount; ++count)
			{
				const float       scale    = kProbedScales[index];
				const std::size_t rowCount = kProbedRowCounts[count];

				const ScoreboardLayout layout = placedScoreboardLayout(
					base, scale, rowCount, viewport.width, viewport.height);
				const std::size_t drawn = scoreboardDrawnRowCount(layout, rowCount);

				++swept;

				if (near(scoreboardCenterY(layout, drawn), viewport.height * 0.5f))
					++centred;

				// Equal air above and below is the same claim said a second way, and it is
				// the one that fails if the height used for centring is not the drawn one.
				const float above = layout.originY;
				const float below = viewport.height - layout.originY
				                    - scoreboardHeight(layout, drawn);
				if (near(above, below))
					++marginsEqual;

				if (near(scoreboardHeight(layout, drawn),
				         static_cast<float>(drawn) * base.rowHeight * scale))
				{
					++heightWasScaled;
				}
			}
		}
	}

	CHECK(swept == kProbedViewportCount * kProbedScaleCount * kProbedRowCountCount);
	CHECK(centred == swept);
	CHECK(marginsEqual == swept);
	CHECK(heightWasScaled == swept);

	// A worked case in absolute pixels: 4 rows at 1.5 is 108 px, centred on 720. The scale
	// is deliberately not 1, where scale-then-centre and centre-then-scale agree.
	const ScoreboardLayout shipped = placedScoreboardLayout(base, 1.5f, 4u, 1280.f, 720.f);
	CHECK(near(scoreboardHeight(shipped, 4u), 108.f));
	CHECK(near(shipped.originY, 306.f));

	// ⛔ SEEDED MISTAKE 1 -- centre the UNSCALED height. Agrees at 1, wrong everywhere else.
	std::size_t agreedAtOne     = 0u;
	std::size_t disagreedBeyond = 0u;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const float scale = kProbedScales[index];
		const float wrong = scoreboardCenteredOriginY(720.f, scoreboardHeight(base, 4u));
		const float right = placedScoreboardLayout(base, scale, 4u, 1280.f, 720.f).originY;

		if (scale == 1.f && near(wrong, right))
			++agreedAtOne;

		if (scale != 1.f && !near(wrong, right))
			++disagreedBeyond;
	}

	CHECK(agreedAtOne == 1u);
	CHECK(disagreedBeyond == kProbedScaleCount - 1u);

	// ⛔ SEEDED MISTAKE 2 -- centre the RESERVED window instead of the drawn rows. This is
	// what `placedPanelLayout` correctly does for the input pane, and it is wrong here: it
	// agrees only on a FULL board and puts a partial board high by half the empty height.
	std::size_t agreedOnAFullBoard = 0u;
	std::size_t highOnAPartialOne  = 0u;

	for (std::size_t count = 0u; count < kProbedRowCountCount; ++count)
	{
		const std::size_t      rowCount = kProbedRowCounts[count];
		const ScoreboardLayout layout =
			placedScoreboardLayout(base, 1.f, rowCount, 1280.f, 720.f);
		const std::size_t drawn = scoreboardDrawnRowCount(layout, rowCount);

		const float reservedCentring =
			scoreboardCenteredOriginY(720.f, scoreboardWindowHeight(layout));

		if (drawn == kScoreboardMaxRows && near(reservedCentring, layout.originY))
			++agreedOnAFullBoard;

		if (drawn < kScoreboardMaxRows)
		{
			// The reserved centring sits HIGHER (smaller y) by exactly half the height of
			// the rows that are not there.
			const float lift = layout.originY - reservedCentring;
			if (lift > 0.f
			    && near(lift,
			            static_cast<float>(kScoreboardMaxRows - drawn) * layout.rowHeight
			                * 0.5f))
			{
				++highOnAPartialOne;
			}
		}
	}

	CHECK(agreedOnAFullBoard == 1u);
	CHECK(highOnAPartialOne == kProbedRowCountCount - 1u);
}

// ---------------------------------------------------------------------------
// ⛔ THE ROW CAP IS A SCREEN BOUND, NOT A POPULATION BOUND, AND IT IS DELIBERATELY NOT 4.
// `kPreDietCharacterCap == 4` is an ADVISORY cap -- its fence warns and lets the session
// run -- and it is already mirrored three times; a fourth mirror here would be a DRAW cap
// that silently hides a fighter in any over-cap session, which is reachable TODAY and not
// only on the day item 40 lifts the cap. The number is set from what a 720p screen holds
// at the maximum scale instead, and this case asserts that arithmetic.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheRowCapIsAScreenBoundAndAFullBoardFitsThe720pViewport",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	CHECK(kScoreboardMaxRows == 8u);

	// Twice the advisory cap, so lifting `kPreDietCharacterCap` from 4 to 8 needs no edit
	// in the layout header at all.
	CHECK(kScoreboardMaxRows == 2u * 4u);

	// 8 rows at 18 px at the maximum scale of 4 is 576 px -- inside 720, which is the
	// shortest viewport this project draws to.
	const ScoreboardLayout biggest = scaledScoreboardLayout(base, kScoreboardMaxScale);
	CHECK(near(scoreboardWindowHeight(biggest), 576.f));
	CHECK(scoreboardWindowHeight(biggest) < 720.f);

	// And it still fits at every probed viewport, which is the claim the constant is for.
	std::size_t fits = 0u;
	for (std::size_t view = 0u; view < kProbedViewportCount; ++view)
	{
		if (scoreboardWindowHeight(biggest) < kProbedViewports[view].height)
			++fits;
	}
	CHECK(fits == kProbedViewportCount);

	// The cap is a CAP: more rosters than rows draws the cap's worth, and fewer draws what
	// there is rather than what was reserved.
	CHECK(scoreboardDrawnRowCount(base, 0u) == 0u);
	CHECK(scoreboardDrawnRowCount(base, 1u) == 1u);
	CHECK(scoreboardDrawnRowCount(base, 4u) == 4u);
	CHECK(scoreboardDrawnRowCount(base, kScoreboardMaxRows) == kScoreboardMaxRows);
	CHECK(scoreboardDrawnRowCount(base, kScoreboardMaxRows + 1u) == kScoreboardMaxRows);
	CHECK(scoreboardDrawnRowCount(base, 1000u) == kScoreboardMaxRows);

	// The two heights are DIFFERENT QUANTITIES and only agree on a full board: the drawn
	// one is what the board occupies, the reserved one is the worst-case footprint.
	CHECK(near(scoreboardHeight(base, 4u), 72.f));
	CHECK(near(scoreboardWindowHeight(base), 144.f));
	CHECK(near(scoreboardHeight(base, kScoreboardMaxRows), scoreboardWindowHeight(base)));

	// Reading rowHeight off the LAYOUT is what makes both of them scaled whenever the
	// layout is a scaled one.
	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const ScoreboardLayout scaled =
			scaledScoreboardLayout(base, kProbedScales[index]);
		CHECK(near(scoreboardWindowHeight(scaled),
		           scoreboardWindowHeight(base) * kProbedScales[index]));
	}
}

// ---------------------------------------------------------------------------
// ⚠ WHAT HAPPENS WHEN THE BOARD IS WIDER THAN THE SCREEN. It OVERHANGS to the left and
// keeps its right edge flush -- it does not un-flush. Pinned here rather than left to be
// found in play, and pinned together with the finding that the case is UNREACHABLE inside
// the clamped scale range at every viewport this project draws to.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheBoardOverhangsRatherThanUnflushingWhenItCannotFit",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	// ⭐ THE FINDING: at the widest the clamp allows, the board is 528 px, which is inside
	// the narrowest viewport here. So no clamped scale can push it off the left edge, and
	// the overhang branch below is reachable only from a viewport nothing ships.
	const ScoreboardLayout widest = scaledScoreboardLayout(base, kScoreboardMaxScale);
	CHECK(near(widest.rowWidth, 528.f));

	std::size_t swept       = 0u;
	std::size_t stayedOnScreen = 0u;

	for (std::size_t view = 0u; view < kProbedViewportCount; ++view)
	{
		for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
		{
			const ScoreboardLayout layout = placedScoreboardLayout(
				base, kProbedScales[index], 4u,
				kProbedViewports[view].width, kProbedViewports[view].height);

			++swept;
			if (layout.originX >= 0.f)
				++stayedOnScreen;
		}
	}

	CHECK(swept == kProbedViewportCount * kProbedScaleCount);
	CHECK(stayedOnScreen == swept);

	// And the documented behaviour when it IS narrower than the board: a negative origin,
	// and the right edge still exactly on the viewport's. ⛔ NOT A CLAMP TO ZERO -- pulling
	// it back would break the one property the panel is defined by, and would hide an
	// unusable scale behind a board that merely looked cramped.
	const ScoreboardLayout overhanging =
		placedScoreboardLayout(base, kScoreboardMaxScale, 4u, 400.f, 720.f);
	CHECK(overhanging.originX < 0.f);
	CHECK(near(overhanging.originX, -128.f));
	CHECK(near(overhanging.originX + overhanging.rowWidth, 400.f));
}

// ---------------------------------------------------------------------------
// ⭐ THE PLACEMENT WAS CHOSEN BY MEASUREMENT, SO IT IS CHECKED BY MEASUREMENT. The
// input-history pane is flush LEFT and the frame meter is horizontally CENTRED near the
// BOTTOM; right-flush was chosen because it collides with neither. This case states the
// margins as NUMBERS at the ruled defaults and REPORTS the settings at which each
// clearance is lost, rather than asserting the overlap away.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.ItClearsBothShippedSurfacesAtTheRuledDefaults",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	using brawlerInputHistoryVisualization::FrameMeterGeometry;
	using brawlerInputHistoryVisualization::FrameMeterLayout;
	using brawlerInputHistoryVisualization::PanelLayout;
	using brawlerInputHistoryVisualization::frameMeterGeometryFor;
	using brawlerInputHistoryVisualization::frameMeterWidth;
	using brawlerInputHistoryVisualization::kPanelDefaultScale;
	using brawlerInputHistoryVisualization::kPanelVisibleRows;
	using brawlerInputHistoryVisualization::kTickLaneDefaultRetainedTicks;
	using brawlerInputHistoryVisualization::placedPanelLayout;

	const ScoreboardLayout base;
	const FrameMeterLayout meterLayout;

	// The ruled defaults at 720p: scale 1, and the four players the advisory cap names.
	const ScoreboardLayout board =
		placedScoreboardLayout(base, kScoreboardDefaultScale, 4u, 1280.f, 720.f);
	const PanelLayout pane =
		placedPanelLayout(PanelLayout{}, kPanelDefaultScale, kPanelVisibleRows, 720.f);
	const FrameMeterGeometry meter =
		frameMeterGeometryFor(meterLayout, 1280.f, 720.f, kTickLaneDefaultRetainedTicks);

	const float boardLeft   = board.originX;
	const float boardBottom = board.originY + scoreboardHeight(board, 4u);
	const float paneRight   = pane.originX + pane.rowWidth;
	const float meterRight  = meter.originX + frameMeterWidth(meter)
	                          + meterLayout.backdropPadding;

	// The board, in absolute pixels.
	CHECK(near(boardLeft, 1148.f));
	CHECK(near(board.originY, 324.f));
	CHECK(near(boardBottom, 396.f));

	// ---- vs the input-history pane: OPPOSITE EDGES, 1064 px apart -------------------
	CHECK(near(paneRight, 84.f));
	CHECK(boardLeft > paneRight);
	CHECK(near(boardLeft - paneRight, 1064.f));

	// ---- vs the frame meter: clear in BOTH axes ------------------------------------
	CHECK(near(meter.originX, 160.f));
	CHECK(near(meterRight, 1123.f));
	CHECK(boardLeft > meterRight);
	CHECK(near(boardLeft - meterRight, 25.f));

	CHECK(boardBottom < meter.originY);
	CHECK(near(meter.originY, 624.2f, 0.05f));

	// Ultrawide only widens both clearances: the meter stays centred on a fixed-width band
	// while the board follows the right edge out.
	const ScoreboardLayout wideBoard =
		placedScoreboardLayout(base, kScoreboardDefaultScale, 4u, 3440.f, 1440.f);
	const FrameMeterGeometry wideMeter =
		frameMeterGeometryFor(meterLayout, 3440.f, 1440.f, kTickLaneDefaultRetainedTicks);
	CHECK(wideBoard.originX
	      > wideMeter.originX + frameMeterWidth(wideMeter) + meterLayout.backdropPadding);
	CHECK(wideBoard.originX - wideMeter.originX > boardLeft - meter.originX);

	// ---- ⚠ REPORTED, NOT ASSERTED AWAY: where each clearance goes ------------------
	//
	// 1. The x clearance against the meter is the tight one -- 25 px at 720p -- and it is
	//    gone by scale 1.25, because the board grows leftward from a fixed right edge.
	const ScoreboardLayout wider =
		placedScoreboardLayout(base, 1.25f, 4u, 1280.f, 720.f);
	CHECK(wider.originX < meterRight);

	// 2. ...but sharing a band of x is not a collision while they share no band of y, and
	//    at FOUR players they never do inside the clamp range: the bottom edge at the
	//    maximum scale is still well above the meter.
	const ScoreboardLayout biggestFour =
		placedScoreboardLayout(base, kScoreboardMaxScale, 4u, 1280.f, 720.f);
	CHECK(biggestFour.originY + scoreboardHeight(biggestFour, 4u) < meter.originY);

	// 3. ⛔ THE ONE COMBINATION THAT DOES COLLIDE, AND IT IS REACHABLE TODAY: a FULL
	//    eight-row board at the maximum scale overlaps the frame meter in both axes at
	//    720p. The three CHECKs below pin that corner.
	//
	//    ⛔ AN EARLIER REVISION OF THIS COMMENT CALLED IT "unreachable in a shipped
	//    session". THAT WAS FALSE. It is corrected in place rather than quietly dropped,
	//    because a claim in a test file is read as a verified fact and this one was an
	//    assumption. `kPreDietCharacterCap == 4` is ADVISORY, not enforced: its own fence
	//    in `SimulationManagerUImpl.cpp` (grep `PreDietCap`) says so in as many words --
	//    "WARNING, not Log, and not an assert: an over-cap session still RUNS - report, do
	//    not crash." It logs and proceeds, so a 5th character registers. Nothing else in
	//    `Source/OGBrawlerUnreal` rejects a join either (`ApproveLogin|PreLogin|MaxPlayers`
	//    `|GameSession` -> zero hits), and `ScoreSystem::onCharacterRegistered` seeds a
	//    roster entry for EVERY authority-registered character without consulting the spawn
	//    table. Eight rows can therefore exist.
	//
	//    WHAT IT ACTUALLY TAKES. ⚠ The corrected sentence is MEASURED, not swapped for a
	//    second assumption. A scratch probe outside the tree drove THESE SHIPPED FUNCTIONS
	//    over rows 0..8 x every 0.01 step of the 0.25..4 clamp range at 720p, against the
	//    two meter edges this case asserts above (1123 and 624.2). Result: only an
	//    EIGHT-row board collides at all, and only above scale ~3.67. FIVE, SIX AND SEVEN
	//    ROWS NEVER COLLIDE ANYWHERE IN THE CLAMP RANGE -- so the row counts just past the
	//    advisory cap still draw clear, and failing takes TWICE that cap AND the top of the
	//    scale range together.
	//    ⛔ READ THE COVERAGE PRECISELY: only the far corner (8 rows, scale 4) is ASSERTED
	//    below. The sweep is a reported measurement, not a test -- asserting it would need
	//    row counts this suite does not probe, and the corner is what the acceptance turns
	//    on.
	//
	//    ⚠ ACCEPTED, NOT DESIGNED AROUND -- and the reason is NOT unreachability. A
	//    session with more than four characters is ALREADY degraded by the netcode's own
	//    warning on the same registration path: "input-loss margins unsafe (T44/T38 §16);
	//    land item 40". That is unsafe for reasons that have nothing to do with ring-out.
	//    A viz overlap in that configuration is consistent with it rather than a new
	//    defect, and a clamp bought here would hide the overlap without making the session
	//    playable. If a hard player ceiling is ever enforced, re-read this paragraph.
	const ScoreboardLayout biggestFull = placedScoreboardLayout(
		base, kScoreboardMaxScale, kScoreboardMaxRows, 1280.f, 720.f);
	const float fullBottom =
		biggestFull.originY + scoreboardHeight(biggestFull, kScoreboardMaxRows);
	CHECK(near(fullBottom, 648.f));
	CHECK(fullBottom > meter.originY);
	CHECK(biggestFull.originX < meterRight);
}

// ---------------------------------------------------------------------------
// ⛔ THE INK IS KEYED ON A BOOLEAN, AND THAT IS WHAT KEEPS THIS PANEL OUTSIDE
// `palette_legend_lint.ps1`'s SCOPE. That lint governs the frame meter's three ENUM-keyed
// palettes, each of which owes a legend table in the rationale doc that stays in step with
// its switch arms. A board with one boolean has no enumerator names for such a table to
// list. If a future task gives it an enum-keyed colour, it owes the table and the lint arm
// in the same change.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheInkIsKeyedOnABooleanAndTheTwoRowStatesAreDistinguishable",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardInk live = scoreboardRowInk(false);
	const ScoreboardInk dead = scoreboardRowInk(true);

	CHECK(near(live.r, kScoreboardLiveRowInk.r));
	CHECK(near(live.g, kScoreboardLiveRowInk.g));
	CHECK(near(live.b, kScoreboardLiveRowInk.b));
	CHECK(near(dead.r, kScoreboardDeadRowInk.r));
	CHECK(near(dead.g, kScoreboardDeadRowInk.g));
	CHECK(near(dead.b, kScoreboardDeadRowInk.b));

	// ⛔ THE TWO STATES MUST NOT LOOK THE SAME. A palette whose two arms collided would be
	// a dead-row colour nobody could see, and the countdown column would be the only cue.
	const bool identicalInk =
		near(live.r, dead.r) && near(live.g, dead.g) && near(live.b, dead.b);
	CHECK_FALSE(identicalInk);

	// Every channel is a legal linear 0..1 value on all three inks.
	const ScoreboardInk inks[] = { kScoreboardLiveRowInk, kScoreboardDeadRowInk,
	                               kScoreboardBackdropInk };
	std::size_t inRange = 0u;
	for (std::size_t index = 0u; index < 3u; ++index)
	{
		const ScoreboardInk ink = inks[index];
		if (ink.r >= 0.f && ink.r <= 1.f && ink.g >= 0.f && ink.g <= 1.f && ink.b >= 0.f
		    && ink.b <= 1.f)
		{
			++inRange;
		}
	}
	CHECK(inRange == 3u);

	// The dead row is DIMMER than the live one -- readable, but obviously not in play.
	CHECK(dead.r + dead.g + dead.b < live.r + live.g + live.b);

	// ...and still bright enough to read: a dead row is where the countdown is, so an ink
	// that vanished into the backdrop would hide the one number that row exists to show.
	CHECK(dead.r + dead.g + dead.b
	      > kScoreboardBackdropInk.r + kScoreboardBackdropInk.g + kScoreboardBackdropInk.b
	            + 1.f);

	// `constexpr`, so the selector can be used in a constant expression and a later edit
	// cannot quietly make it runtime-only.
	static_assert(scoreboardRowInk(true).r == kScoreboardDeadRowInk.r,
	              "the dead arm is selected at compile time");
	static_assert(scoreboardRowInk(false).r == kScoreboardLiveRowInk.r,
	              "the live arm is selected at compile time");
}

// ---------------------------------------------------------------------------
// THE COUNTDOWN [ringout task 6b]. The only piece of real arithmetic in the whole UE
// layer, moved down into this header so that it could be probed at all.
//
// ⭐ WHY THIS CASE EXISTS AND THE REST OF 6b's LAYER HAS NONE. `gatherScoreboardRows`,
// `displayTick` and the three CVar accessors are `Source/OGBrawlerUnreal` code, and this
// suite links { Core, OGSimulation, OGBrawler } and not that module: a red probe against
// any of them stays green because nothing here can reach them. That is a real limit of
// this suite and it is stated rather than worked around. What COULD be pulled across the
// line was pulled across, and this is it.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheRespawnCountdownClampsAtZeroRatherThanWrapping",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace brawlerScoreboardVisualization;

	// The ordinary case: the respawn is ahead, so the countdown is the plain difference.
	CHECK(scoreboardTicksUntilRespawn(300u, 180u) == 120u);
	CHECK(scoreboardTicksUntilRespawn(181u, 180u) == 1u);

	// The boundary. The respawn tick has ARRIVED but the sub-simulation has not yet run
	// the step that clears `kFlagDead`, so a board drawn on this frame still asks.
	CHECK(scoreboardTicksUntilRespawn(180u, 180u) == 0u);

	// ⛔ THE CLAIM THIS CASE EXISTS FOR. The display tick MOVES BACKWARDS on a hard
	// resync, so `displayTick` can legitimately exceed a `respawnAtTick` that has not been
	// reached. Both are uint32_t; the unguarded difference is not a small negative, it is
	// a number near 2^32. Asserting the WRAPPED value as well as the clamped one is what
	// makes this a claim about the guard rather than about arithmetic in general.
	const uint32_t respawnAt   = 180u;
	const uint32_t afterResync = 200u;

	const uint32_t unguarded = respawnAt - afterResync;
	CHECK(unguarded == 4294967276u);            // 2^32 - 20: a ten-digit status column
	CHECK(unguarded > 4000000000u);

	CHECK(scoreboardTicksUntilRespawn(respawnAt, afterResync) == 0u);

	// The extreme of the same failure: a resync that rewinds across the whole range.
	CHECK(scoreboardTicksUntilRespawn(0u, 4294967295u) == 0u);

	// The largest honest countdown the type can carry is still returned intact -- the
	// clamp may not be implemented by capping the result at something convenient.
	CHECK(scoreboardTicksUntilRespawn(4294967295u, 0u) == 4294967295u);

	// ⛔ NOT GATED ON THE DEAD FLAG HERE. This answers an arithmetic question; whether the
	// answer means anything is `scoreboardRowDrawsCountdown`'s, asked on the flag. A row
	// that is alive draws no countdown however large this number is.
	ScoreboardRow living;
	living.isDead            = false;
	living.ticksUntilRespawn = scoreboardTicksUntilRespawn(300u, 180u);
	CHECK(living.ticksUntilRespawn == 120u);
	CHECK_FALSE(scoreboardRowDrawsCountdown(living));

	// `constexpr`, so the clamp holds in a constant expression too and a later edit cannot
	// quietly make it runtime-only.
	static_assert(scoreboardTicksUntilRespawn(180u, 200u) == 0u,
	              "the backwards-tick clamp is decided at compile time");
	static_assert(scoreboardTicksUntilRespawn(300u, 180u) == 120u,
	              "the ordinary countdown is decided at compile time");
}


// ---------------------------------------------------------------------------
// ⭐⭐ THE SWATCH-TO-CHARACTER BINDING [ringout task 10] -- THE CLAIM WITH THE REAL STAKES
// ON THIS TASK, and the one the whole column is worth nothing without.
//
// A scoreboard that shows colours instead of ids is USEFUL only if the colour on a row is
// the colour of the body that row is about. Every way of getting that wrong compiles, and
// most of them draw a board that looks entirely healthy: one colour repeated, the first
// character's colour on every row, or -- the one this suite can actually reach -- an
// ordering fold that sorts the ids and leaves the tints where they were.
//
// ⛔ WHAT THIS CASE CAN AND CANNOT SEE, STATED RATHER THAN IMPLIED. The gather that reads
// `AOGBrawlerUECharacter::GetBrawlerColor` is `Source/OGBrawlerUnreal` code and this suite
// links { Core, OGSimulation, OGBrawler } and not that module, so a mis-binding typed THERE
// is invisible from here -- finding F26, measured by task 6b. What was moved across the
// line, and what this case pins, is the part of the binding that CAN live in pure code: the
// mapping from a row to the colour it draws, and the guarantee that the ordering fold does
// not permute the two apart.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheSwatchIsBoundToItsOwnRowAndSurvivesTheOrderingFold",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	// ---------------------------------------------------------------------
	// 1. THE MAPPING ITSELF. `scoreboardRowSwatch` returns the ROW'S colour and has no
	//    other source to return. A selector that reached for a constant -- the live ink,
	//    say, or a fixed default -- would draw a board of identical swatches.
	// ---------------------------------------------------------------------
	std::size_t rowsCheckedAgainstTheirOwnTint = 0u;
	for (std::size_t index = 0u; index < kTranscribedPaletteCount; ++index)
	{
		ScoreboardRow row;
		row.characterId = static_cast<unsigned int>(100u + index);
		row.swatch      = kTranscribedPaletteSample[index];

		if (sameSwatch(scoreboardRowSwatch(row), kTranscribedPaletteSample[index]))
			++rowsCheckedAgainstTheirOwnTint;
	}
	CHECK(rowsCheckedAgainstTheirOwnTint == kTranscribedPaletteCount);

	// ⛔ AND IT IS NOT A CONSTANT DRESSED UP AS A FUNCTION. Two rows differing ONLY in the
	//    swatch must give two different answers -- the decoy that catches a selector which
	//    ignores its argument and happens to return a plausible colour.
	ScoreboardRow first;
	ScoreboardRow second;
	first.swatch  = kTranscribedPaletteSample[0];
	second.swatch = kTranscribedPaletteSample[1];
	CHECK_FALSE(sameSwatch(scoreboardRowSwatch(first), scoreboardRowSwatch(second)));

	// ⛔ AND IT IS NOT THE ROW INK. The two colour channels on this board are separate: the
	//    ink says STATUS and is keyed on a bool, the swatch says IDENTITY and is carried.
	//    A selector that returned `kScoreboardLiveRowInk` for a white-defaulted row would
	//    pass by coincidence, so this is asserted on a row whose tint is not white.
	CHECK_FALSE(sameSwatch(scoreboardRowSwatch(first), scoreboardRowInk(false)));
	CHECK_FALSE(sameSwatch(scoreboardRowSwatch(first), scoreboardRowInk(true)));

	// ---------------------------------------------------------------------
	// 2. ⭐⭐ THE DEAD ROW KEEPS ITS FULL TINT -- THE DELIBERATE DECISION, ASSERTED SO IT
	//    CANNOT BE UNDONE BY ACCIDENT. The acceptance criterion is that a dead or
	//    respawning brawler's row STAYS IDENTIFIABLE. It does, because column one does not
	//    move at all: the same row, dead or alive, draws the same three floats.
	// ---------------------------------------------------------------------
	std::size_t swatchesUnchangedByDeath = 0u;
	std::size_t inksChangedByDeath       = 0u;
	for (std::size_t index = 0u; index < kTranscribedPaletteCount; ++index)
	{
		ScoreboardRow alive;
		alive.characterId = static_cast<unsigned int>(index);
		alive.swatch      = kTranscribedPaletteSample[index];

		ScoreboardRow out     = alive;
		out.isDead            = true;
		out.ticksUntilRespawn = 90u;

		if (sameSwatch(scoreboardRowSwatch(alive), scoreboardRowSwatch(out)))
			++swatchesUnchangedByDeath;

		// ...and the STATUS did change, on the channel that is meant to carry it. Both
		// halves are needed: a board that dimmed nothing at all would also pass the line
		// above, and then being out would be invisible.
		if (!sameSwatch(scoreboardRowInk(alive.isDead), scoreboardRowInk(out.isDead)))
			++inksChangedByDeath;
	}
	CHECK(swatchesUnchangedByDeath == kTranscribedPaletteCount);
	CHECK(inksChangedByDeath == kTranscribedPaletteCount);

	// The third cue, for completeness: the countdown column appears. Status is stated
	// TWICE, which is the reason the identity channel is free to state nothing.
	ScoreboardRow counting;
	counting.swatch            = kTranscribedPaletteSample[4];
	counting.isDead            = true;
	counting.ticksUntilRespawn = 90u;
	CHECK(scoreboardRowDrawsCountdown(counting));
	CHECK(sameSwatch(scoreboardRowSwatch(counting), kTranscribedPaletteSample[4]));

	// ---------------------------------------------------------------------
	// 3. ⭐ DISTINCT IN, DISTINCT OUT -- at a FULL board, and while every fighter on it is
	//    dead at once. "Two brawlers never share a swatch" is a property of the palette,
	//    which this target cannot link; what it CAN pin is that the board never collapses
	//    two tints that arrived distinct. That is the half a mis-binding breaks.
	// ---------------------------------------------------------------------
	std::vector<ScoreboardRow> fullBoard;
	for (std::size_t index = 0u; index < kTranscribedPaletteCount; ++index)
	{
		ScoreboardRow row;
		// Descending ids, so the fold has real work to do on every row.
		row.characterId       = static_cast<unsigned int>(kTranscribedPaletteCount - index);
		row.swatch            = kTranscribedPaletteSample[index];
		row.isDead            = true;
		row.ticksUntilRespawn = static_cast<uint32_t>(10u * (index + 1u));
		fullBoard.push_back(row);
	}
	REQUIRE(fullBoard.size() == kScoreboardMaxRows);

	const std::vector<ScoreboardRow> drawn = orderedScoreboardRows(fullBoard);
	REQUIRE(drawn.size() == kScoreboardMaxRows);

	std::vector<ScoreboardInk> drawnSwatches;
	for (std::size_t slot = 0u; slot < drawn.size(); ++slot)
		drawnSwatches.push_back(scoreboardRowSwatch(drawn[slot]));

	const float shippedSeparation = minimumSwatchSeparation(drawnSwatches);

	// ⛔ STRICTLY POSITIVE IS THE CLAIM THAT MATTERS: no two of the eight rows on a full,
	//    entirely-dead board draw the same colour.
	CHECK(shippedSeparation > 0.f);

	// ...and not merely different, but different ENOUGH to read. Measured on the shipped
	// palette's first eight on 2026-09-13 the closest pair is 0.25 apart (amber/orange and
	// violet/pink both sit there), so 0.2 is a floor this sample clears with margin rather
	// than a number fitted to it.
	CHECK(shippedSeparation > 0.2f);

	// ⛔ THE DECOY THAT MAKES SECTION 2 A CLAIM RATHER THAN A PREFERENCE. The rejected
	//    design was "dim the swatch while the fighter is out". Any such rule is a
	//    contraction toward a common colour, and a contraction is a many-to-one map on the
	//    one axis that must stay one-to-one. Modelled here as the mildest version
	//    imaginable -- a half-blend toward the dead ink -- it costs exactly half the
	//    separation, on a board where ALL EIGHT fighters are down at once, which is
	//    precisely when a player is hunting for their own row.
	std::vector<ScoreboardInk> dimmedSwatches;
	for (std::size_t slot = 0u; slot < drawn.size(); ++slot)
	{
		const ScoreboardInk carried = scoreboardRowSwatch(drawn[slot]);
		dimmedSwatches.push_back(
			ScoreboardInk{ (carried.r + kScoreboardDeadRowInk.r) * 0.5f,
			               (carried.g + kScoreboardDeadRowInk.g) * 0.5f,
			               (carried.b + kScoreboardDeadRowInk.b) * 0.5f });
	}
	const float dimmedSeparation = minimumSwatchSeparation(dimmedSwatches);

	CHECK(dimmedSeparation < shippedSeparation);
	CHECK(near(dimmedSeparation, shippedSeparation * 0.5f));

	// ⭐ And the shipped rule preserves the separation EXACTLY, because it is the identity.
	std::vector<ScoreboardInk> asSupplied;
	for (std::size_t index = 0u; index < kTranscribedPaletteCount; ++index)
		asSupplied.push_back(kTranscribedPaletteSample[index]);
	CHECK(near(minimumSwatchSeparation(asSupplied), shippedSeparation));

	// ---------------------------------------------------------------------
	// 4. ⛔ THE ID DID NOT STOP BEING THE KEY. The task changed which column is DRAWN and
	//    nothing else: rows are still ordered by `characterId`, so two peers still show the
	//    same player in the same row whatever order their tints were assigned in.
	// ---------------------------------------------------------------------
	bool ascendingById = true;
	for (std::size_t slot = 1u; slot < drawn.size(); ++slot)
		ascendingById =
			ascendingById && drawn[slot - 1u].characterId < drawn[slot].characterId;
	CHECK(ascendingById);

	// ⛔ AND THE ORDER IS NOT THE TINT ORDER. The rows went in with ASCENDING tints and
	//    DESCENDING ids; if the board had been ordered by colour the two sequences would
	//    agree, and they must not.
	bool inPaletteOrder = true;
	for (std::size_t slot = 0u; slot < drawn.size(); ++slot)
		inPaletteOrder =
			inPaletteOrder && sameSwatch(drawn[slot].swatch, kTranscribedPaletteSample[slot]);
	CHECK_FALSE(inPaletteOrder);

	// The comparator reads the id and nothing else -- two rows that differ ONLY in colour
	// are tied, in both directions.
	const ScoreboardRow sameIdRed{ 5u, 0u, false, 0u, kTranscribedPaletteSample[0] };
	const ScoreboardRow sameIdBlue{ 5u, 0u, false, 0u, kTranscribedPaletteSample[1] };
	CHECK_FALSE(scoreboardRowPrecedes(sameIdRed, sameIdBlue));
	CHECK_FALSE(scoreboardRowPrecedes(sameIdBlue, sameIdRed));

	// `constexpr`, so the mapping holds in a constant expression and a later edit cannot
	// quietly make it runtime-only -- nor branch it on the dead flag, which is the edit
	// section 2 exists to forbid.
	static_assert(
		scoreboardRowSwatch(
			ScoreboardRow{ 1u, 0u, false, 0u, ScoreboardInk{ 0.25f, 0.5f, 0.75f } })
				.g
			== 0.5f,
		"the swatch is the row's own, decided at compile time");
	static_assert(
		scoreboardRowSwatch(
			ScoreboardRow{ 1u, 0u, true, 60u, ScoreboardInk{ 0.25f, 0.5f, 0.75f } })
				.g
			== 0.5f,
		"a dead row draws the same swatch a live one does");
}

// ---------------------------------------------------------------------------
// ⭐ COLUMN ONE'S GEOMETRY [ringout task 10]. The swatch is a BAND INSIDE ITS OWN ROW: it
// must not touch the row above or below, must not reach the score column, and must not
// leave the board -- at every scale and every viewport, not only at the one that was run
// while it was being written.
//
// ⛔ WHY THE INSET IS LOAD-BEARING AND NOT A COSMETIC MARGIN: two adjacent swatches with no
// gap between them read as ONE two-tone rectangle, which is exactly the confusion column
// one was added to remove.
// ---------------------------------------------------------------------------
TEST_CASE("Scoreboard.TheSwatchIsABandInsideItsOwnRowAtEveryScale",
          "[CharacterViz][BrawlerRingoutScoreboard]")
{
	using namespace ringoutscoreboardtests;

	const ScoreboardLayout base;

	// The base relations the scale can never alter, pinned at compile time: an inset that
	// grew past half the row height would invert the band, and a swatch that reached the
	// score column would sit under a number.
	static_assert(ScoreboardLayout{}.swatchInsetY * 2.f < ScoreboardLayout{}.rowHeight,
	              "the swatch inset must leave a positive band inside its row");
	static_assert(ScoreboardLayout{}.swatchX + ScoreboardLayout{}.swatchWidth
	                  < ScoreboardLayout{}.scoreRightX,
	              "the swatch must end left of the score column");

	std::size_t swept          = 0u;
	std::size_t positiveHeight = 0u;
	std::size_t insideOwnRow   = 0u;
	std::size_t clearOfTheText = 0u;
	std::size_t insideTheBoard = 0u;
	std::size_t sameProportion = 0u;

	// The proportion of a row the band occupies. Scale-INVARIANT, because both terms are
	// multiplied by the same factor -- which is the whole reason the height needs no clamp.
	const float baseProportion =
		(base.rowHeight - 2.f * base.swatchInsetY) / base.rowHeight;

	for (std::size_t scaleIndex = 0u; scaleIndex < kProbedScaleCount; ++scaleIndex)
	{
		for (std::size_t viewIndex = 0u; viewIndex < kProbedViewportCount; ++viewIndex)
		{
			const ScoreboardLayout layout = placedScoreboardLayout(
				base, kProbedScales[scaleIndex], kScoreboardMaxRows,
				kProbedViewports[viewIndex].width, kProbedViewports[viewIndex].height);

			for (std::size_t slot = 0u; slot < kScoreboardMaxRows; ++slot)
			{
				const ScoreboardSwatchRect rect = scoreboardSwatchRect(layout, slot);

				++swept;

				if (rect.height > 0.f && rect.width > 0.f)
					++positiveHeight;

				// Strictly inside its own row: a gap above AND a gap below, so no two
				// swatches ever meet.
				const float rowTop    = scoreboardRowTopY(layout, slot);
				const float rowBottom = rowTop + layout.rowHeight;
				if (rect.y > rowTop && (rect.y + rect.height) < rowBottom)
					++insideOwnRow;

				// Clear of column two's right edge, so the swatch can never sit under a
				// score.
				if ((rect.x + rect.width) < (layout.originX + layout.scoreRightX))
					++clearOfTheText;

				// Inside the board's own box, horizontally.
				if (rect.x > layout.originX
				    && (rect.x + rect.width) < layout.originX + layout.rowWidth)
					++insideTheBoard;

				if (near(rect.height / layout.rowHeight, baseProportion))
					++sameProportion;
			}
		}
	}

	CHECK(swept == kProbedScaleCount * kProbedViewportCount * kScoreboardMaxRows);
	CHECK(positiveHeight == swept);
	CHECK(insideOwnRow == swept);
	CHECK(clearOfTheText == swept);
	CHECK(insideTheBoard == swept);
	CHECK(sameProportion == swept);

	// ⛔ ADJACENT SWATCHES NEVER TOUCH, asserted directly rather than inferred from the two
	//    "inside its own row" checks -- that is the claim the inset exists for.
	const ScoreboardLayout placed =
		placedScoreboardLayout(base, 1.f, kScoreboardMaxRows, 1920.f, 1080.f);

	std::size_t gapsBetweenNeighbours = 0u;
	for (std::size_t slot = 1u; slot < kScoreboardMaxRows; ++slot)
	{
		const ScoreboardSwatchRect above = scoreboardSwatchRect(placed, slot - 1u);
		const ScoreboardSwatchRect below = scoreboardSwatchRect(placed, slot);
		if ((above.y + above.height) < below.y)
			++gapsBetweenNeighbours;
	}
	CHECK(gapsBetweenNeighbours == kScoreboardMaxRows - 1u);

	// A worked rectangle in absolute pixels, at the shipped default scale and at 2, so the
	// four numbers are pinned as numbers and not only as relations. Row 0 of an eight-row
	// board at 1920x1080: rowHeight 18, drawn height 144, originY (1080-144)/2 = 468.
	const ScoreboardSwatchRect firstRow = scoreboardSwatchRect(placed, 0u);
	CHECK(near(firstRow.x, placed.originX + 6.f));
	CHECK(near(firstRow.y, 468.f + 3.f));
	CHECK(near(firstRow.width, 30.f));
	CHECK(near(firstRow.height, 12.f));

	const ScoreboardLayout twice =
		placedScoreboardLayout(base, 2.f, kScoreboardMaxRows, 1920.f, 1080.f);
	const ScoreboardSwatchRect firstRowTwice = scoreboardSwatchRect(twice, 0u);
	CHECK(near(firstRowTwice.width, 60.f));
	CHECK(near(firstRowTwice.height, 24.f));

	// ⛔ THE X IS THE BOARD'S ORIGIN PLUS THE COLUMN OFFSET, AND THE ORIGIN IS THE
	//    RIGHT-FLUSH ONE. A swatch drawn at an absolute `swatchX` would sit at the far LEFT
	//    of the screen while the rest of the row sat at the far right -- a failure that is
	//    invisible on a board whose origin happens to be 0.
	CHECK(placed.originX > 0.f);
	CHECK(near(firstRow.x - placed.originX, placed.swatchX));

	// The second row is exactly one row height below the first, and so is its swatch.
	const ScoreboardSwatchRect secondRow = scoreboardSwatchRect(placed, 1u);
	CHECK(near(secondRow.y - firstRow.y, placed.rowHeight));
	CHECK(near(secondRow.x, firstRow.x));
	CHECK(near(secondRow.height, firstRow.height));
}

#endif // WITH_LOW_LEVEL_TESTS
