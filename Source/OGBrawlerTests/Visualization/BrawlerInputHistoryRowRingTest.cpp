// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins brawlerInputHistoryVisualization::InputHistoryRowRing -- the run-length fold
// that turns a stream of per-tick captures into the display's rows.
//
// WHAT THIS SUITE IS REALLY GUARDING is the two properties a naive "append and
// count" ring silently lacks. First, IDEMPOTENCE: the consumer polls a 64-slot
// source cache at render-frame rate, so it re-presents already-folded ticks on
// almost every frame, and a ring that counted them twice would inflate every number
// on the display without bound. Second, DEPTH: tickCount must be free to exceed the
// source cache's capacity, because the reference display's longest row reads 99 --
// longer than that cache can ever hold.

#include "catch_amalgamated.hpp"

#include <cstddef>
#include <cstdint>

#include "OGBrawler/BrawlerInputHistoryVisualization.h"
#include "OGSimulation/Network/LocalInputCache.h"

namespace inputhistoryvizringtests
{

using brawlerInputHistoryVisualization::AppendResult;
using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::InputHistoryRow;
using brawlerInputHistoryVisualization::InputHistoryRowRing;

// The mask motionButtonMask produces: bit 0 = attackLeft, bit 1 = attackRight.
constexpr uint8_t kNoButtons   = 0b00;
constexpr uint8_t kLeftAttack  = 0b01;
constexpr uint8_t kRightAttack = 0b10;

// One held state, so a test only has to name the field it is varying. These two fields
// ARE the row's whole identity, and the panel draws both of them.
struct Capture
{
	DirectionBucket direction  = DirectionBucket::Forward;
	uint8_t         buttonMask = kNoButtons;
};

static AppendResult append(InputHistoryRowRing& ring, uint32_t captureTick, const Capture& capture)
{
	return ring.appendCapture(captureTick, capture.direction, capture.buttonMask);
}

// Feed a contiguous run of one held state, starting at firstTick.
static void appendRun(InputHistoryRowRing& ring,
                      uint32_t             firstTick,
                      uint32_t             tickCount,
                      const Capture&       capture)
{
	for (uint32_t offset = 0u; offset < tickCount; ++offset)
	{
		append(ring, firstTick + offset, capture);
	}
}

} // namespace inputhistoryvizringtests

// ---------------------------------------------------------------------------
// The core fold: consecutive equal captures are ONE row, not N.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.ConsecutiveEqualCapturesFoldIntoOneRow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	InputHistoryRowRing ring;
	CHECK(ring.empty());
	CHECK(ring.size() == 0u);

	const Capture held{ DirectionBucket::ForwardRight, kLeftAttack };

	CHECK(append(ring, 10u, held) == AppendResult::OpenedRow);
	CHECK(append(ring, 11u, held) == AppendResult::ExtendedRow);
	CHECK(append(ring, 12u, held) == AppendResult::ExtendedRow);
	CHECK(append(ring, 13u, held) == AppendResult::ExtendedRow);
	CHECK(append(ring, 14u, held) == AppendResult::ExtendedRow);

	REQUIRE(ring.size() == 1u);

	const InputHistoryRow& row = ring.newest();
	CHECK(row.direction == DirectionBucket::ForwardRight);
	CHECK(row.buttonMask == kLeftAttack);
	CHECK(row.firstCaptureTick == 10u);
	CHECK(row.tickCount == 5u);
	CHECK(row.lastCaptureTick() == 14u);
}

// ---------------------------------------------------------------------------
// A row is longer than the cache it was folded from. This is the reference display's
// 99-tick row, and a tickCount clamped to the source ring cannot express it.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.TickCountExceedsTheSourceCacheCapacity",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	constexpr uint32_t kReferenceLongestRowTicks = 99u;

	// Named, not a literal 64: if the source cache is ever resized this stays a claim
	// about the cache rather than about a number that used to equal it.
	CHECK(kReferenceLongestRowTicks > kLocalInputCacheCapacityTicks);

	InputHistoryRowRing ring;
	const Capture       held{ DirectionBucket::Back, kNoButtons };

	appendRun(ring, 500u, kReferenceLongestRowTicks, held);

	REQUIRE(ring.size() == 1u);
	CHECK(ring.newest().tickCount == kReferenceLongestRowTicks);
	CHECK(ring.newest().tickCount > kLocalInputCacheCapacityTicks);
	CHECK(ring.newest().firstCaptureTick == 500u);
	CHECK(ring.newest().lastCaptureTick() == 598u);
}

// ---------------------------------------------------------------------------
// Idempotence: re-presenting a tick the ring has already folded changes NOTHING.
// This is the property render-rate polling of a 64-slot cache depends on.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.ReAppendingAnAlreadyFoldedTickIsANoOp",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	InputHistoryRowRing ring;
	const Capture       held{ DirectionBucket::Left, kRightAttack };

	appendRun(ring, 10u, 5u, held);
	REQUIRE(ring.size() == 1u);
	REQUIRE(ring.newest().tickCount == 5u);

	// The first, the middle and the last tick of the row.
	CHECK(append(ring, 10u, held) == AppendResult::IgnoredDuplicate);
	CHECK(append(ring, 12u, held) == AppendResult::IgnoredDuplicate);
	CHECK(append(ring, 14u, held) == AppendResult::IgnoredDuplicate);

	// The whole run again, which is exactly what a second poll of an unchanged source
	// cache presents. Without idempotence this alone would double the count.
	appendRun(ring, 10u, 5u, held);

	CHECK(ring.size() == 1u);
	CHECK(ring.newest().tickCount == 5u);
	CHECK(ring.newest().firstCaptureTick == 10u);
	CHECK(ring.newest().lastCaptureTick() == 14u);
	CHECK(ring.newest().direction == DirectionBucket::Left);
}

// ---------------------------------------------------------------------------
// A stale tick is REJECTED, and in particular is not pushed back into the older row
// that happens to cover it. Rejection is a different fact from idempotence.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.StaleTickIsRejectedAndNotFoldedIntoAnOlderRow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	InputHistoryRowRing ring;
	const Capture       older{ DirectionBucket::Forward, kNoButtons };
	const Capture       newer{ DirectionBucket::Back, kNoButtons };

	appendRun(ring, 10u, 3u, older);   // row 0: ticks 10..12
	appendRun(ring, 13u, 3u, newer);   // row 1: ticks 13..15
	REQUIRE(ring.size() == 2u);
	REQUIRE(ring.oldest().tickCount == 3u);

	// A tick belonging to the OLDER row, re-presented with that row's own fields. The
	// tempting wrong answer is to grow row 0; it is behind the newest row's span.
	CHECK(append(ring, 11u, older) == AppendResult::IgnoredStale);
	CHECK(append(ring, 9u, older) == AppendResult::IgnoredStale);

	// And with the NEWER row's fields, so the rejection is not field-sensitive.
	CHECK(append(ring, 11u, newer) == AppendResult::IgnoredStale);

	CHECK(ring.size() == 2u);
	CHECK(ring.oldest().firstCaptureTick == 10u);
	CHECK(ring.oldest().tickCount == 3u);
	CHECK(ring.newest().firstCaptureTick == 13u);
	CHECK(ring.newest().tickCount == 3u);
}

// ---------------------------------------------------------------------------
// EACH of the two folded fields breaks a run on its own. A ring that compared only
// the direction would pass a direction-only test and mis-draw every button press.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.EachFoldedFieldChangeOpensANewRow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	const Capture base{ DirectionBucket::Forward, kNoButtons };

	SECTION("direction")
	{
		InputHistoryRowRing ring;
		Capture             changed = base;
		changed.direction           = DirectionBucket::ForwardRight;

		CHECK(append(ring, 10u, base) == AppendResult::OpenedRow);
		CHECK(append(ring, 11u, changed) == AppendResult::OpenedRow);
		CHECK(ring.size() == 2u);
		CHECK(ring.newest().direction == DirectionBucket::ForwardRight);
		CHECK(ring.newest().tickCount == 1u);
	}

	SECTION("buttonMask")
	{
		InputHistoryRowRing ring;
		Capture             changed = base;
		changed.buttonMask          = kRightAttack;

		CHECK(append(ring, 10u, base) == AppendResult::OpenedRow);
		CHECK(append(ring, 11u, changed) == AppendResult::OpenedRow);
		CHECK(ring.size() == 2u);
		CHECK(ring.newest().buttonMask == kRightAttack);
		CHECK(ring.newest().tickCount == 1u);
	}
}

// ---------------------------------------------------------------------------
// A gap opens a row even when the input is identical: the skipped ticks were never
// observed, so folding across them would report a hold that never happened.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.NonContiguousTickOpensANewRow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	InputHistoryRowRing ring;
	const Capture       held{ DirectionBucket::Right, kNoButtons };

	CHECK(append(ring, 10u, held) == AppendResult::OpenedRow);
	CHECK(append(ring, 12u, held) == AppendResult::OpenedRow);

	REQUIRE(ring.size() == 2u);
	CHECK(ring.oldest().firstCaptureTick == 10u);
	CHECK(ring.oldest().tickCount == 1u);
	CHECK(ring.newest().firstCaptureTick == 12u);
	CHECK(ring.newest().tickCount == 1u);

	// The new row still extends normally from its own first tick.
	CHECK(append(ring, 13u, held) == AppendResult::ExtendedRow);
	CHECK(ring.newest().tickCount == 2u);
}

// ---------------------------------------------------------------------------
// Eviction drops the OLDEST row and leaves the newest untouched -- including the
// newest row's ability to keep growing across the eviction.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.RowRing.EvictionIsOldestFirstAndLeavesTheNewestIntact",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizringtests;

	InputHistoryRowRing ring;
	const Capture       held{ DirectionBucket::BackLeft, kLeftAttack };

	// Every tick is separated by a gap, so each append opens its own row.
	constexpr uint32_t kOverfillRows = 3u;
	const uint32_t     rowsToPush =
		static_cast<uint32_t>(InputHistoryRowRing::capacity()) + kOverfillRows;

	uint32_t openedRows = 0u;
	for (uint32_t row = 0u; row < rowsToPush; ++row)
	{
		openedRows += (append(ring, row * 2u, held) == AppendResult::OpenedRow) ? 1u : 0u;
	}

	// Counted rather than asserted per iteration: the claim is that overfilling never
	// silently turns an open into an extend, and one number says exactly that.
	CHECK(openedRows == rowsToPush);
	REQUIRE(ring.size() == InputHistoryRowRing::capacity());

	// The first kOverfillRows rows are gone; the survivors are still in order.
	CHECK(ring.oldest().firstCaptureTick == kOverfillRows * 2u);
	CHECK(ring.at(1u).firstCaptureTick == (kOverfillRows + 1u) * 2u);
	CHECK(ring.newest().firstCaptureTick == (rowsToPush - 1u) * 2u);
	CHECK(ring.newest().tickCount == 1u);
	CHECK(ring.newest().buttonMask == kLeftAttack);

	// The newest row survived every eviction intact enough to still be extended.
	CHECK(append(ring, (rowsToPush - 1u) * 2u + 1u, held) == AppendResult::ExtendedRow);
	CHECK(ring.newest().tickCount == 2u);
	CHECK(ring.size() == InputHistoryRowRing::capacity());
	CHECK(ring.oldest().firstCaptureTick == kOverfillRows * 2u);
}

#endif // WITH_LOW_LEVEL_TESTS
