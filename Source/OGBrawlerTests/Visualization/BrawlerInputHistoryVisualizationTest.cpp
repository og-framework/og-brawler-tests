// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins brawlerInputHistoryVisualization::directionBucketOf -- which of the nine
// aim-relative compass buckets one captured input falls in.
//
// WHAT THIS SUITE IS REALLY GUARDING is that the glyph is DERIVED from
// inputSequence rather than re-quantized beside it. The sector-width and midpoint
// cases below are the ones that would go green against a hand-rolled atan2 table
// for a while and then silently drift off the motion matcher's pi/8 MotionStep
// tolerance. The deadzone and aim-relativity cases pin the two inputs a
// re-implementation is most likely to hard-code.

#include "catch_amalgamated.hpp"

#include <cmath>
#include <utility>

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "OGBrawler/BrawlerInputHistoryVisualization.h"
#include "OGBrawler/InputSequence/InputSequence.h"

namespace inputhistoryviztests
{

using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::directionBucketOf;
using brawlerInputHistoryVisualization::nearestNamedDirection;

// g_moveStickDeadzone's shipped default. Named here rather than read from the
// atomic: the header takes the deadzone as an argument so tests need no global.
constexpr float kDeadzone = 0.15f;

constexpr glm::vec3 kAimPlusX(1.f, 0.f, 0.f);

// Just inside a sector edge -- half a sector, backed off by a margin far wider
// than any float rounding in atan2.
constexpr float kInsideEdge = inputSequence::pi / 8.f - 0.01f;

// Build a unit stick at a given AIM-RELATIVE angle, from aimRelativeAngle's OWN
// perpendicular so the helper cannot disagree with the function it feeds. In Unreal
// world axes that perpendicular is the character's LEFT, so the angle grows leftward.
static glm::vec2 stickAt(float aimRelativeRadians, glm::vec3 aim = kAimPlusX)
{
	const glm::vec2 fwd = glm::normalize(glm::vec2(aim.x, aim.y));
	const glm::vec2 right(fwd.y, -fwd.x);
	return std::cos(aimRelativeRadians) * fwd + std::sin(aimRelativeRadians) * right;
}

static DirectionBucket bucketAt(float aimRelativeRadians, glm::vec3 aim = kAimPlusX)
{
	return directionBucketOf(stickAt(aimRelativeRadians, aim), aim, kDeadzone);
}

} // namespace inputhistoryviztests

// ---------------------------------------------------------------------------
// All eight compass sectors, at their own named angle.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.DirectionBucket.EightNamedAnglesClassifyToTheirOwnBucket",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryviztests;
	namespace angle = inputSequence::angle;

	CHECK(bucketAt(angle::Forward)     == DirectionBucket::Forward);
	CHECK(bucketAt(angle::DownForward) == DirectionBucket::ForwardLeft);
	CHECK(bucketAt(angle::Down)        == DirectionBucket::Left);
	CHECK(bucketAt(angle::DownBack)    == DirectionBucket::BackLeft);
	CHECK(bucketAt(angle::Back)        == DirectionBucket::Back);
	CHECK(bucketAt(angle::UpBack)      == DirectionBucket::BackRight);
	CHECK(bucketAt(angle::Up)          == DirectionBucket::Right);
	CHECK(bucketAt(angle::UpForward)   == DirectionBucket::ForwardRight);
}

// ---------------------------------------------------------------------------
// Each sector is a full 45 degrees, not just its centre line -- the shape the
// motion matcher's pi/8 MotionStep tolerance already has.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.DirectionBucket.EachSectorIsFortyFiveDegreesWide",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryviztests;
	namespace angle = inputSequence::angle;

	const std::pair<float, DirectionBucket> sectors[] = {
		{ angle::Forward,     DirectionBucket::Forward      },
		{ angle::DownForward, DirectionBucket::ForwardLeft  },
		{ angle::Down,        DirectionBucket::Left         },
		{ angle::DownBack,    DirectionBucket::BackLeft     },
		{ angle::Back,        DirectionBucket::Back         },
		{ angle::UpBack,      DirectionBucket::BackRight    },
		{ angle::Up,          DirectionBucket::Right        },
		{ angle::UpForward,   DirectionBucket::ForwardRight },
	};

	for (const auto& sector : sectors)
	{
		CHECK(bucketAt(sector.first - kInsideEdge) == sector.second);
		CHECK(bucketAt(sector.first + kInsideEdge) == sector.second);
	}
}

// ---------------------------------------------------------------------------
// The sector edge sits on the pi/8 midpoint, and an exact midpoint is a real tie
// that declaration order breaks toward the earlier named angle.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.DirectionBucket.SectorEdgeIsThePiOverEightMidpoint",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryviztests;

	const float edge = inputSequence::pi / 8.f;

	CHECK(bucketAt(edge - 0.01f) == DirectionBucket::Forward);
	CHECK(bucketAt(edge + 0.01f) == DirectionBucket::ForwardLeft);

	// Fed the exact midpoint the two named angles are equidistant, so the earlier
	// table entry keeps it. Taken through the angle rather than a stick, so no
	// atan2 rounding stands between the input and the tie.
	CHECK(nearestNamedDirection(edge) == DirectionBucket::Forward);
	CHECK(nearestNamedDirection(3.f * inputSequence::pi / 8.f) == DirectionBucket::ForwardLeft);
}

// ---------------------------------------------------------------------------
// Below the deadzone there is no direction -- and the deadzone is the CALLER'S.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.DirectionBucket.SubDeadzoneStickIsNeutral",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryviztests;

	const glm::vec2 shortForwardStick(0.10f, 0.f);

	CHECK(directionBucketOf(shortForwardStick, kAimPlusX, kDeadzone) == DirectionBucket::Neutral);
	CHECK(directionBucketOf(glm::vec2(0.f, 0.f), kAimPlusX, kDeadzone) == DirectionBucket::Neutral);

	// The same stick against a deadzone it clears. The glyph follows whatever
	// number the caller gates on, which is what lets it track g_moveStickDeadzone.
	CHECK(directionBucketOf(shortForwardStick, kAimPlusX, 0.05f) == DirectionBucket::Forward);
}

// ---------------------------------------------------------------------------
// A reference forward with no XY component has no direction to be relative to.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.DirectionBucket.DegenerateReferenceForwardIsNeutral",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryviztests;

	const glm::vec2 fullStick(1.f, 0.f);

	CHECK(directionBucketOf(fullStick, glm::vec3(0.f, 0.f, 1.f), kDeadzone) == DirectionBucket::Neutral);
	CHECK(directionBucketOf(fullStick, glm::vec3(0.f, 0.f, 0.f), kDeadzone) == DirectionBucket::Neutral);
	CHECK(directionBucketOf(fullStick, glm::vec3(1e-9f, 1e-9f, 1.f), kDeadzone) == DirectionBucket::Neutral);
}

// ---------------------------------------------------------------------------
// The bucket is aim-relative. A world-frame quantizer would pass every case above
// and fail this one.
// ---------------------------------------------------------------------------
TEST_CASE("InputHistoryViz.DirectionBucket.BucketsAreAimRelativeNotWorldFrame",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryviztests;
	namespace angle = inputSequence::angle;

	// One relative angle, four unrelated aims, one answer.
	CHECK(bucketAt(angle::DownForward, glm::vec3( 1.f,  0.f, 0.f)) == DirectionBucket::ForwardLeft);
	CHECK(bucketAt(angle::DownForward, glm::vec3( 0.f,  1.f, 0.f)) == DirectionBucket::ForwardLeft);
	CHECK(bucketAt(angle::DownForward, glm::vec3(-1.f,  0.f, 0.f)) == DirectionBucket::ForwardLeft);
	CHECK(bucketAt(angle::DownForward, glm::vec3( 0.6f, -0.8f, 0.f)) == DirectionBucket::ForwardLeft);

	// One WORLD stick, two aims, two answers.
	const glm::vec2 worldPlusX(1.f, 0.f);
	CHECK(directionBucketOf(worldPlusX, glm::vec3(1.f, 0.f, 0.f), kDeadzone) == DirectionBucket::Forward);
	CHECK(directionBucketOf(worldPlusX, glm::vec3(0.f, 1.f, 0.f), kDeadzone) == DirectionBucket::Left);
}

#endif // WITH_LOW_LEVEL_TESTS
