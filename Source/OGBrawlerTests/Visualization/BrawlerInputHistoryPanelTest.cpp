// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins brawlerInputHistoryVisualization's PANEL layer -- the three-column layout the
// on-screen display is drawn from, and the vector arrow that is its direction glyph.
//
// WHY THESE CLAIMS LIVE IN A CATCH2 SUITE AT ALL. The renderer is UE code and
// Source/OGBrawlerTests links { Core, OGSimulation, OGBrawler } and NOT
// OGBrawlerUnreal, so nothing written against a canvas is reachable from here. The
// two claims that decide whether the panel is USEFUL rather than merely correct --
// that the newest row draws at the TOP, and that moving TOWARD THE AIM draws an arrow
// pointing UP -- are therefore made in pure code, where they can be tested.
//
// THE SCREEN-SPACE Y SIGN IS THE ONE THAT BITES. A canvas counts y DOWNWARD, so an
// upward arrow runs toward the SMALLER y while the rows below run toward the larger.
// Both conventions are asserted against each other below rather than one at a time,
// because a display with both signs flipped is self-consistent and upside down.

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "glm/vec2.hpp"

#include "OGBrawler/BrawlerInputHistoryVisualization.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationBars.h"
#include "OGBrawler/BrawlerInputHistoryVisualizationPanel.h"
#include "OGBrawler/BrawlerMotionMatching.h"
#include "OGBrawler/InputSequence/InputSequence.h"

namespace inputhistoryvizpaneltests
{

using brawlerInputHistoryVisualization::DirectionBucket;
using brawlerInputHistoryVisualization::DirectionGlyph;
using brawlerInputHistoryVisualization::InputHistoryRowRing;
using brawlerInputHistoryVisualization::PanelLayout;
using brawlerInputHistoryVisualization::buttonMaskGlyph;
using brawlerInputHistoryVisualization::directionArrowAxis;
using brawlerInputHistoryVisualization::directionGlyphOf;
using brawlerInputHistoryVisualization::clampPanelBackgroundAlpha;
using brawlerInputHistoryVisualization::clampPanelScale;
using brawlerInputHistoryVisualization::clampPanelVisibleRows;
using brawlerInputHistoryVisualization::kNamedDirections;
using brawlerInputHistoryVisualization::kPanelDefaultBackgroundAlpha;
using brawlerInputHistoryVisualization::kPanelDefaultScale;
using brawlerInputHistoryVisualization::kPanelLeftEdgeX;
using brawlerInputHistoryVisualization::kPanelMaxBackgroundAlpha;
using brawlerInputHistoryVisualization::kPanelMaxScale;
using brawlerInputHistoryVisualization::kPanelMaxVisibleRows;
using brawlerInputHistoryVisualization::kPanelMinBackgroundAlpha;
using brawlerInputHistoryVisualization::kPanelMinScale;
using brawlerInputHistoryVisualization::kPanelMinVisibleRows;
using brawlerInputHistoryVisualization::kPanelVisibleRows;
using brawlerInputHistoryVisualization::nearestNamedDirection;
using brawlerInputHistoryVisualization::panelCenteredOriginY;
using brawlerInputHistoryVisualization::panelDrawnRowCount;
using brawlerInputHistoryVisualization::panelRingIndexForSlot;
using brawlerInputHistoryVisualization::panelRowTopY;
using brawlerInputHistoryVisualization::panelWindowHeight;
using brawlerInputHistoryVisualization::placedPanelLayout;
using brawlerInputHistoryVisualization::scaledPanelLayout;

// The eight that have a direction. Neutral is deliberately absent: it is the one
// bucket every sweep below has to treat differently.
constexpr DirectionBucket kNamedBuckets[] = {
	DirectionBucket::Forward,  DirectionBucket::ForwardRight, DirectionBucket::Right,
	DirectionBucket::BackRight, DirectionBucket::Back,        DirectionBucket::BackLeft,
	DirectionBucket::Left,     DirectionBucket::ForwardLeft,
};

constexpr std::size_t kNamedBucketCount = 8u;

// sqrt(2)/2 -- a diagonal arrow's two components.
constexpr float kDiagonal = 0.70710678f;

// Every component expected below is 0, +/-1 or +/-sqrt(2)/2, so one tolerance covers all.
static bool axisIs(glm::vec2 axis, float x, float y)
{
	return std::abs(axis.x - x) < 1e-4f && std::abs(axis.y - y) < 1e-4f;
}

static float dot(const glm::vec2& left, const glm::vec2& right)
{
	return left.x * right.x + left.y * right.y;
}

static float length(const glm::vec2& vector)
{
	return std::sqrt(dot(vector, vector));
}

static glm::vec2 minus(const glm::vec2& left, const glm::vec2& right)
{
	return glm::vec2(left.x - right.x, left.y - right.y);
}

// `rowCount` single-tick rows. The direction alternates, so every capture is a new
// held state and therefore opens its own row rather than extending the last.
static InputHistoryRowRing ringOfRows(std::size_t rowCount, uint32_t firstTick = 1u)
{
	InputHistoryRowRing ring;

	for (std::size_t index = 0u; index < rowCount; ++index)
	{
		const DirectionBucket direction =
			(index % 2u == 0u) ? DirectionBucket::Forward : DirectionBucket::Back;

		ring.appendCapture(firstTick + static_cast<uint32_t>(index), direction, 0u);
	}

	return ring;
}

// The scales every sweep below runs, spanning both clamp ends and the shipped default.
constexpr float kProbedScales[] = { 0.25f, 0.5f, 1.f, 1.5f, 2.5f, 4.f };

constexpr std::size_t kProbedScaleCount = 6u;

// Four viewports, not one: a centring bug is invisible at whatever size you happened
// to test. The last is a 21:9 ultrawide, the shape the panel is furthest from square in.
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

static bool near(float left, float right, float tolerance = 1e-3f)
{
	return std::abs(left - right) < tolerance;
}

// Where the panel's own vertical middle lands, given a placed layout.
static float panelCenterY(const PanelLayout& layout)
{
	return layout.originY + panelWindowHeight(layout) * 0.5f;
}

} // namespace inputhistoryvizpaneltests

// ---------------------------------------------------------------------------
// Eight directions, eight axes, and Neutral is none of them. A table collapsing two
// buckets onto one axis would draw the same arrow for two different inputs.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.EveryNamedDirectionGetsItsOwnUnitAxisAndNeutralGetsNone",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	std::size_t unitLength    = 0u;
	std::size_t distinctPairs = 0u;

	for (std::size_t a = 0u; a < kNamedBucketCount; ++a)
	{
		const glm::vec2 axis = directionArrowAxis(kNamedBuckets[a]);
		if (std::abs(length(axis) - 1.f) < 1e-4f)
			++unitLength;

		for (std::size_t b = a + 1u; b < kNamedBucketCount; ++b)
		{
			if (length(minus(axis, directionArrowAxis(kNamedBuckets[b]))) > 1e-4f)
				++distinctPairs;
		}
	}

	// Counted, then asserted once: a CHECK inside those loops would multiply silently.
	CHECK(unitLength == kNamedBucketCount);
	CHECK(distinctPairs == 28u);

	const glm::vec2 neutral = directionArrowAxis(DirectionBucket::Neutral);
	CHECK(neutral.x == 0.f);
	CHECK(neutral.y == 0.f);
}

// ---------------------------------------------------------------------------
// ALL EIGHT INDIVIDUALLY -- THE FOUR AXES AND THE FOUR DIAGONALS. A screen mapping
// that is right on the axes and wrong on the diagonals passes any suite that only
// checks up/down/left/right, and that is the likeliest way to half-fix this.
// Each row starts at the angle the MATCHER tests and walks the production
// classification, so a bucket table drifting off inputSequence fails here too.
//
// +pi/2 DRAWS LEFT AND -pi/2 DRAWS RIGHT: aimRelativeAngle's perpendicular is the
// character's left in Unreal world axes, whatever numpad name the constant carries.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.EachOfTheEightAimRelativeAnglesDrawsItsOwnCompassArrow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;
	namespace angle = inputSequence::angle;

	const PanelLayout layout;

	// Aim-relative 0 is "moving where I am aiming", and that is what draws straight UP.
	const DirectionBucket forward = nearestNamedDirection(angle::Forward);
	CHECK(forward == DirectionBucket::Forward);
	CHECK(axisIs(directionArrowAxis(forward), 0.f, -1.f));

	const DirectionBucket forwardLeft = nearestNamedDirection(angle::DownForward);
	CHECK(forwardLeft == DirectionBucket::ForwardLeft);
	CHECK(axisIs(directionArrowAxis(forwardLeft), -kDiagonal, -kDiagonal));

	const DirectionBucket left = nearestNamedDirection(angle::Down);
	CHECK(left == DirectionBucket::Left);
	CHECK(axisIs(directionArrowAxis(left), -1.f, 0.f));

	const DirectionBucket backLeft = nearestNamedDirection(angle::DownBack);
	CHECK(backLeft == DirectionBucket::BackLeft);
	CHECK(axisIs(directionArrowAxis(backLeft), -kDiagonal, kDiagonal));

	const DirectionBucket back = nearestNamedDirection(angle::Back);
	CHECK(back == DirectionBucket::Back);
	CHECK(axisIs(directionArrowAxis(back), 0.f, 1.f));

	const DirectionBucket backRight = nearestNamedDirection(angle::UpBack);
	CHECK(backRight == DirectionBucket::BackRight);
	CHECK(axisIs(directionArrowAxis(backRight), kDiagonal, kDiagonal));

	const DirectionBucket right = nearestNamedDirection(angle::Up);
	CHECK(right == DirectionBucket::Right);
	CHECK(axisIs(directionArrowAxis(right), 1.f, 0.f));

	const DirectionBucket forwardRight = nearestNamedDirection(angle::UpForward);
	CHECK(forwardRight == DirectionBucket::ForwardRight);
	CHECK(axisIs(directionArrowAxis(forwardRight), kDiagonal, -kDiagonal));

	// The ninth bucket closes the compass: no angle, so no arrow.
	CHECK(axisIs(directionArrowAxis(DirectionBucket::Neutral), 0.f, 0.f));
	CHECK(directionGlyphOf(layout, panelRowTopY(layout, 0u), DirectionBucket::Neutral).isNeutralDot);
}

// ---------------------------------------------------------------------------
// The arrow is the MIRROR IMAGE of the angle the matcher tested -- mirrored because
// the matcher's positive side is the character's left. Swept over inputSequence's own
// constants, so an edit to either the compass table or the enum's order lands here.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.EveryArrowMirrorsTheAngleTheMatcherTestedAcrossTheAimAxis",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	std::size_t swept  = 0u;
	std::size_t agreed = 0u;

	for (const auto& named : kNamedDirections)
	{
		++swept;

		// Reflected in the vertical on a canvas whose y grows downward: x is the
		// NEGATED sine of the aim-relative angle and y its negated cosine.
		if (axisIs(directionArrowAxis(named.bucket),
		           -std::sin(named.angle), -std::cos(named.angle)))
		{
			++agreed;
		}
	}

	// Counted, then asserted once: a CHECK inside that loop would multiply silently.
	CHECK(swept == kNamedBucketCount);
	CHECK(agreed == swept);
}

// ---------------------------------------------------------------------------
// The whole point of drawing rather than typing a glyph: the arrow must point the way
// the player pressed, on a canvas whose y runs the opposite way from a compass rose.
// Both conventions are asserted against each other, because a display with both signs
// flipped is self-consistent and upside down.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.MovingTowardTheAimDrawsUpwardOnACanvasWhoseYGrowsDownward",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout layout;

	// The panel's OWN downward convention, read from the row stack rather than assumed:
	// slot 1 sits below slot 0, so larger y is lower on the screen.
	CHECK(panelRowTopY(layout, 1u) > panelRowTopY(layout, 0u));

	// Toward the aim is toward the smaller y, and straight up -- no sideways drift.
	CHECK(directionArrowAxis(DirectionBucket::Forward).y < 0.f);
	CHECK(directionArrowAxis(DirectionBucket::Forward).x == 0.f);
	CHECK(directionArrowAxis(DirectionBucket::Back).y > 0.f);

	// Right-of-aim draws to the right, and neither side leaves the horizontal.
	CHECK(directionArrowAxis(DirectionBucket::Right).x > 0.f);
	CHECK(directionArrowAxis(DirectionBucket::Right).y == 0.f);
	CHECK(directionArrowAxis(DirectionBucket::Left).x < 0.f);

	// A diagonal has to agree with BOTH of its components, which a single flipped sign
	// somewhere in the derivation would break.
	CHECK(directionArrowAxis(DirectionBucket::ForwardRight).x > 0.f);
	CHECK(directionArrowAxis(DirectionBucket::ForwardRight).y < 0.f);
	CHECK(directionArrowAxis(DirectionBucket::BackLeft).x < 0.f);
	CHECK(directionArrowAxis(DirectionBucket::BackLeft).y > 0.f);

	// And the drawn shaft carries the sign through: the TIP of a toward-the-aim arrow
	// is above its tail, in the same y the rows above are measured in.
	const DirectionGlyph toward =
		directionGlyphOf(layout, panelRowTopY(layout, 0u), DirectionBucket::Forward);
	CHECK(toward.shaft.to.y < toward.shaft.from.y);

	const DirectionGlyph away =
		directionGlyphOf(layout, panelRowTopY(layout, 0u), DirectionBucket::Back);
	CHECK(away.shaft.to.y > away.shaft.from.y);
}

// ---------------------------------------------------------------------------
// Neutral has no direction, so any arrow drawn for it points somewhere the player did
// not press. It is a dot, and it is the ONLY bucket that is.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.NeutralIsACentredDotAndEveryOtherBucketIsAnArrow",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout layout;
	const float       top = panelRowTopY(layout, 3u);

	const DirectionGlyph neutral = directionGlyphOf(layout, top, DirectionBucket::Neutral);
	CHECK(neutral.isNeutralDot);
	CHECK(neutral.dotSize == layout.neutralDotSize);

	// Centred on the same point the arrows rotate about, in both axes.
	CHECK(neutral.dotOrigin.x + neutral.dotSize * 0.5f
	      == layout.originX + layout.directionCenterX);
	CHECK(neutral.dotOrigin.y + neutral.dotSize * 0.5f == top + layout.rowHeight * 0.5f);

	std::size_t arrows = 0u;
	for (std::size_t index = 0u; index < kNamedBucketCount; ++index)
	{
		if (!directionGlyphOf(layout, top, kNamedBuckets[index]).isNeutralDot)
			++arrows;
	}

	CHECK(arrows == kNamedBucketCount);
}

// ---------------------------------------------------------------------------
// A shaft alone is a line, not an arrow. The barbs are what say which end is the tip,
// and they have to do it for all eight rotations, not just the axis-aligned four.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.TheBarbsTrailTheTipAndMirrorEachOtherInEveryRotation",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout layout;
	const float       top = panelRowTopY(layout, 0u);

	std::size_t rootedAtTheTip = 0u;
	std::size_t trailingBehind = 0u;
	std::size_t mirrored       = 0u;
	std::size_t shorterThanTheShaft = 0u;

	for (std::size_t index = 0u; index < kNamedBucketCount; ++index)
	{
		const DirectionGlyph glyph = directionGlyphOf(layout, top, kNamedBuckets[index]);
		const glm::vec2      axis  = directionArrowAxis(kNamedBuckets[index]);
		const glm::vec2      perpendicular(-axis.y, axis.x);

		const glm::vec2 tip   = glyph.shaft.to;
		const glm::vec2 left  = minus(glyph.leftBarb.to, tip);
		const glm::vec2 right = minus(glyph.rightBarb.to, tip);

		if (length(minus(glyph.leftBarb.from, tip)) < 1e-4f
			&& length(minus(glyph.rightBarb.from, tip)) < 1e-4f)
		{
			++rootedAtTheTip;
		}

		// Both barb ends lie BACK along the shaft, which is what makes the tip a tip.
		if (dot(left, axis) < 0.f && dot(right, axis) < 0.f)
			++trailingBehind;

		// Equal and opposite across the shaft, and genuinely off it -- a barb pair that
		// collapsed onto the shaft would satisfy "equal and opposite" at zero.
		const float acrossLeft  = dot(left, perpendicular);
		const float acrossRight = dot(right, perpendicular);
		if (std::abs(acrossLeft + acrossRight) < 1e-4f && std::abs(acrossLeft) > 1e-3f)
			++mirrored;

		if (length(left) < length(minus(glyph.shaft.to, glyph.shaft.from)))
			++shorterThanTheShaft;
	}

	CHECK(rootedAtTheTip == kNamedBucketCount);
	CHECK(trailingBehind == kNamedBucketCount);
	CHECK(mirrored == kNamedBucketCount);
	CHECK(shorterThanTheShaft == kNamedBucketCount);
}

// ---------------------------------------------------------------------------
// THREE COLUMNS, IN ORDER, INSIDE THE ROW. The panel was cut down from five and the
// failure to guard against is a leftover gap or a column hanging off the edge.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.TheRowIsThreeOrderedColumnsAndNoGlyphLeavesIt",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout layout;

	// Count ends before the arrow cell starts, and the arrow cell ends before buttons.
	CHECK(layout.tickCountRightX < layout.directionCenterX - layout.directionExtent);
	CHECK(layout.directionCenterX + layout.directionExtent < layout.buttonsX);
	CHECK(layout.buttonsX < layout.rowWidth);

	// A rotated arrow is as tall as it is wide, so the cell has to fit it vertically too.
	CHECK(2.f * layout.directionExtent < layout.rowHeight);

	const float top = panelRowTopY(layout, 2u);

	std::size_t insideTheRow = 0u;
	for (std::size_t index = 0u; index < kNamedBucketCount; ++index)
	{
		const DirectionGlyph glyph = directionGlyphOf(layout, top, kNamedBuckets[index]);
		const glm::vec2      points[] = {
			glyph.shaft.from, glyph.shaft.to, glyph.leftBarb.to, glyph.rightBarb.to,
		};

		std::size_t contained = 0u;
		for (const glm::vec2& point : points)
		{
			if (point.x >= layout.originX && point.x <= layout.originX + layout.rowWidth
				&& point.y >= top && point.y <= top + layout.rowHeight)
			{
				++contained;
			}
		}

		if (contained == 4u)
			++insideTheRow;
	}

	CHECK(insideTheRow == kNamedBucketCount);
}

TEST_CASE("Panel.TheNewestRowDrawsAtTheTopAndTheRowsDescendFromIt",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const InputHistoryRowRing ring = ringOfRows(8u, 100u);
	const PanelLayout         layout;
	const std::size_t         drawn = panelDrawnRowCount(layout, ring.size());

	CHECK(drawn == 8u);
	CHECK(panelRingIndexForSlot(ring.size(), 0u) == ring.size() - 1u);
	CHECK(ring.at(panelRingIndexForSlot(ring.size(), 0u)).firstCaptureTick
	      == ring.newest().firstCaptureTick);

	std::size_t slotsBelowTheLast  = 0u;
	std::size_t olderThanTheAbove  = 0u;
	for (std::size_t slot = 1u; slot < drawn; ++slot)
	{
		if (panelRowTopY(layout, slot) > panelRowTopY(layout, slot - 1u))
			++slotsBelowTheLast;

		const uint32_t above = ring.at(panelRingIndexForSlot(ring.size(), slot - 1u)).firstCaptureTick;
		const uint32_t below = ring.at(panelRingIndexForSlot(ring.size(), slot)).firstCaptureTick;
		if (below < above)
			++olderThanTheAbove;
	}

	// y grows downward on a canvas, so "newest at top" is exactly "y increases with slot".
	CHECK(slotsBelowTheLast == drawn - 1u);
	CHECK(olderThanTheAbove == drawn - 1u);
}

TEST_CASE("Panel.TheDrawnWindowKeepsTheNewestRowsAndClampsNoTickCount",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout         layout;
	const InputHistoryRowRing shallow = ringOfRows(5u);
	CHECK(panelDrawnRowCount(layout, shallow.size()) == 5u);

	const InputHistoryRowRing full = ringOfRows(InputHistoryRowRing::capacity());
	CHECK(full.size() == InputHistoryRowRing::capacity());
	CHECK(panelDrawnRowCount(layout, full.size()) == kPanelVisibleRows);

	// The bottom slot is the OLDEST drawn row, so the rows dropped are the oldest ones.
	CHECK(panelRingIndexForSlot(full.size(), kPanelVisibleRows - 1u)
	      == full.size() - kPanelVisibleRows);

	// The reference display's 99-tick row outlives the 64-tick source cache. The panel
	// must not be the thing that truncates it back.
	InputHistoryRowRing longHold;
	for (uint32_t tick = 0u; tick < 99u; ++tick)
	{
		longHold.appendCapture(tick, DirectionBucket::Right, 0u);
	}

	CHECK(longHold.size() == 1u);
	CHECK(longHold.at(panelRingIndexForSlot(longHold.size(), 0u)).tickCount == 99u);
}

TEST_CASE("Panel.TheButtonGlyphReadsTheExistingMotionButtonMaskConvention",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	// Built through motionButtonMask rather than from literals: the glyph and the mask
	// must agree about which bit is which, and a literal would only pin this file.
	CHECK(std::strcmp(buttonMaskGlyph(simulatableBrawler::motionButtonMask(false, false)), "--") == 0);
	CHECK(std::strcmp(buttonMaskGlyph(simulatableBrawler::motionButtonMask(true, false)), "L-") == 0);
	CHECK(std::strcmp(buttonMaskGlyph(simulatableBrawler::motionButtonMask(false, true)), "-R") == 0);
	CHECK(std::strcmp(buttonMaskGlyph(simulatableBrawler::motionButtonMask(true, true)), "LR") == 0);
}

// ---------------------------------------------------------------------------
// THE DEFECT THIS CASE EXISTS TO CATCH: a scale that moves the geometry and leaves the
// text behind. The font is fixed-size, so a geometry-only scale is a bigger box holding
// the same tiny glyphs -- which looks like a layout bug and reads as one.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.OneFactorScalesBothTheGeometryAndTheTextAndTheyCannotDriftApart",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout base;

	std::size_t scaledEveryNumber = 0u;
	std::size_t textAgreedWithRows = 0u;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const float       scale  = kProbedScales[index];
		const PanelLayout scaled = scaledPanelLayout(base, scale);

		const bool everyNumber =
			near(scaled.rowHeight,        base.rowHeight * scale)
			&& near(scaled.rowWidth,        base.rowWidth * scale)
			&& near(scaled.tickCountRightX, base.tickCountRightX * scale)
			&& near(scaled.directionCenterX, base.directionCenterX * scale)
			&& near(scaled.directionExtent, base.directionExtent * scale)
			&& near(scaled.buttonsX,        base.buttonsX * scale)
			&& near(scaled.arrowThickness,  base.arrowThickness * scale)
			&& near(scaled.neutralDotSize,  base.neutralDotSize * scale)
			&& near(scaled.textOffsetY,     base.textOffsetY * scale)
			&& near(scaled.textScale,       base.textScale * scale);

		if (everyNumber)
			++scaledEveryNumber;

		// The anti-drift claim itself: the text's factor IS the rows' factor, recovered
		// from the geometry rather than read off the same field twice.
		if (near(scaled.rowHeight / base.rowHeight, scaled.textScale))
			++textAgreedWithRows;
	}

	CHECK(scaledEveryNumber == kProbedScaleCount);
	CHECK(textAgreedWithRows == kProbedScaleCount);

	// And the defect stated positively, at a scale that is not the identity -- the shipped
	// default is 1.0, where a text factor left behind would be invisible.
	const PanelLayout shipped = scaledPanelLayout(base, 1.5f);
	CHECK(shipped.textScale == 1.5f);
	CHECK(shipped.textScale != base.textScale);
	CHECK(near(shipped.rowHeight, 27.f));
	CHECK(near(shipped.rowWidth, 126.f));

	// Placement runs the same scale, so a placed layout carries the same text factor.
	const PanelLayout placed = placedPanelLayout(base, 1.5f, kPanelVisibleRows, 720.f);
	CHECK(placed.textScale == shipped.textScale);
	CHECK(near(placed.rowHeight, shipped.rowHeight));
}

// ---------------------------------------------------------------------------
// The literals below are the panel's PRE-SCALE numbers, transcribed. If a scale factor
// leaked into the base layout, or a column was nudged while the scaling went in, this
// is what says so -- and it is what makes the change provably a scaling and not a
// re-layout wearing one.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.AtScaleOneEveryLayoutNumberIsStillTheOneThePanelWasDrawnFrom",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout base;

	CHECK(base.rowHeight == 18.f);
	CHECK(base.rowWidth == 84.f);
	CHECK(base.tickCountRightX == 34.f);
	CHECK(base.directionCenterX == 48.f);
	CHECK(base.directionExtent == 6.f);
	CHECK(base.buttonsX == 62.f);
	CHECK(base.arrowThickness == 1.6f);
	CHECK(base.neutralDotSize == 5.f);
	CHECK(base.textOffsetY == 2.f);
	CHECK(base.textScale == 1.f);
	CHECK(base.visibleRows == kPanelVisibleRows);
	CHECK(kPanelVisibleRows == 24u);

	// Scaling by one is the identity on every one of them, field for field.
	const PanelLayout identity = scaledPanelLayout(base, 1.f);

	CHECK(identity.rowHeight == base.rowHeight);
	CHECK(identity.rowWidth == base.rowWidth);
	CHECK(identity.tickCountRightX == base.tickCountRightX);
	CHECK(identity.directionCenterX == base.directionCenterX);
	CHECK(identity.directionExtent == base.directionExtent);
	CHECK(identity.buttonsX == base.buttonsX);
	CHECK(identity.arrowThickness == base.arrowThickness);
	CHECK(identity.neutralDotSize == base.neutralDotSize);
	CHECK(identity.textOffsetY == base.textOffsetY);
	CHECK(identity.textScale == base.textScale);
	CHECK(identity.visibleRows == base.visibleRows);

	// The derived geometry too, so this covers what a renderer actually asks for and
	// not merely the struct's fields. Rows and glyph are measured from the origin,
	// because the origin is the one thing this task moved on purpose.
	std::size_t rowsAtTheSameOffset = 0u;
	for (std::size_t slot = 0u; slot < kPanelVisibleRows; ++slot)
	{
		if (near(panelRowTopY(identity, slot) - identity.originY,
		         static_cast<float>(slot) * 18.f))
		{
			++rowsAtTheSameOffset;
		}
	}

	CHECK(rowsAtTheSameOffset == kPanelVisibleRows);

	std::size_t glyphsUnmoved = 0u;
	for (std::size_t index = 0u; index < kNamedBucketCount; ++index)
	{
		const DirectionGlyph before =
			directionGlyphOf(base, panelRowTopY(base, 2u), kNamedBuckets[index]);
		const DirectionGlyph after =
			directionGlyphOf(identity, panelRowTopY(identity, 2u), kNamedBuckets[index]);

		if (length(minus(before.shaft.from, after.shaft.from)) < 1e-4f
			&& length(minus(before.shaft.to, after.shaft.to)) < 1e-4f
			&& length(minus(before.leftBarb.to, after.leftBarb.to)) < 1e-4f
			&& length(minus(before.rightBarb.to, after.rightBarb.to)) < 1e-4f)
		{
			++glyphsUnmoved;
		}
	}

	CHECK(glyphsUnmoved == kNamedBucketCount);
}

// ---------------------------------------------------------------------------
// The arrow is drawn ABOUT a point, so a scale that moved its cell without moving that
// point would slide every arrow off centre -- worst on the diagonals, where it reads as
// a wrong direction rather than as a wrong position.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.TheArrowStaysCentredInItsCellAndTheBarbsScaleWithTheShaft",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout base;

	std::size_t centredOnTheCell   = 0u;
	std::size_t sameFractionOfCell = 0u;
	std::size_t barbsInProportion  = 0u;
	std::size_t neutralDotCentred  = 0u;
	std::size_t swept              = 0u;

	// The cell centre as a FRACTION of the row, at scale 1. Scale-invariance against
	// this is what "still centred" means once every number has been multiplied.
	const float baseFractionX = base.directionCenterX / base.rowWidth;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const PanelLayout layout = scaledPanelLayout(base, kProbedScales[index]);
		const float       top    = panelRowTopY(layout, 3u);
		const glm::vec2   center(layout.originX + layout.directionCenterX,
		                         top + layout.rowHeight * 0.5f);

		if (near(layout.directionCenterX / layout.rowWidth, baseFractionX)
			&& near((center.y - top) / layout.rowHeight, 0.5f))
		{
			++sameFractionOfCell;
		}

		const DirectionGlyph dot = directionGlyphOf(layout, top, DirectionBucket::Neutral);
		if (near(dot.dotOrigin.x + dot.dotSize * 0.5f, center.x)
			&& near(dot.dotOrigin.y + dot.dotSize * 0.5f, center.y))
		{
			++neutralDotCentred;
		}

		for (std::size_t bucket = 0u; bucket < kNamedBucketCount; ++bucket)
		{
			const DirectionGlyph glyph =
				directionGlyphOf(layout, top, kNamedBuckets[bucket]);

			++swept;

			// The shaft's midpoint IS the cell centre, which is what "about a point" means.
			const glm::vec2 midpoint((glyph.shaft.from.x + glyph.shaft.to.x) * 0.5f,
			                         (glyph.shaft.from.y + glyph.shaft.to.y) * 0.5f);

			if (near(midpoint.x, center.x) && near(midpoint.y, center.y))
				++centredOnTheCell;

			// A barb that kept its absolute length would vanish into a large arrow.
			const float barbLength = length(minus(glyph.leftBarb.to, glyph.shaft.to));
			if (near(barbLength / layout.directionExtent,
			         length(minus(glyph.rightBarb.to, glyph.shaft.to))
			             / layout.directionExtent,
			         1e-3f)
				&& barbLength > 0.f)
			{
				++barbsInProportion;
			}
		}
	}

	CHECK(swept == kProbedScaleCount * kNamedBucketCount);
	CHECK(centredOnTheCell == swept);
	CHECK(barbsInProportion == swept);
	CHECK(sameFractionOfCell == kProbedScaleCount);
	CHECK(neutralDotCentred == kProbedScaleCount);

	// Proportion pinned across two scales explicitly, so "in proportion" is a number
	// and not just an equality between two things that could both be wrong.
	const PanelLayout    one = scaledPanelLayout(base, 1.f);
	const PanelLayout    two = scaledPanelLayout(base, 2.f);
	const DirectionGlyph atOne =
		directionGlyphOf(one, panelRowTopY(one, 0u), DirectionBucket::ForwardRight);
	const DirectionGlyph atTwo =
		directionGlyphOf(two, panelRowTopY(two, 0u), DirectionBucket::ForwardRight);

	CHECK(near(length(minus(atTwo.shaft.to, atTwo.shaft.from)),
	           length(minus(atOne.shaft.to, atOne.shaft.from)) * 2.f));
	CHECK(near(length(minus(atTwo.leftBarb.to, atTwo.shaft.to)),
	           length(minus(atOne.leftBarb.to, atOne.shaft.to)) * 2.f));
}

// ---------------------------------------------------------------------------
// FLUSH LEFT AND VERTICALLY CENTRED -- and the centring computed AFTER the scale. The
// last clause is the whole case: centring a pre-scale height is exactly right at scale
// 1 and visibly wrong at every other, which is the bug that ships because only the
// default was ever looked at. The final block seeds that mistake and shows it caught.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.ThePanelIsFlushLeftAndCentredOnTheScaledHeightAtEveryViewport",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	const PanelLayout base;

	std::size_t flushLeft      = 0u;
	std::size_t centred        = 0u;
	std::size_t marginsEqual   = 0u;
	std::size_t heightWasScaled = 0u;
	std::size_t swept          = 0u;

	for (std::size_t view = 0u; view < kProbedViewportCount; ++view)
	{
		const ProbedViewport viewport = kProbedViewports[view];

		for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
		{
			const float       scale = kProbedScales[index];
			const PanelLayout layout =
				placedPanelLayout(base, scale, kPanelVisibleRows, viewport.height);

			++swept;

			if (layout.originX == kPanelLeftEdgeX && layout.originX == 0.f)
				++flushLeft;

			if (near(panelCenterY(layout), viewport.height * 0.5f))
				++centred;

			// Equal air above and below is the same claim said a second way, and it is
			// the one that fails if the height used for centring is not the drawn one.
			const float above = layout.originY;
			const float below = viewport.height - layout.originY - panelWindowHeight(layout);
			if (near(above, below))
				++marginsEqual;

			if (near(panelWindowHeight(layout),
			         static_cast<float>(kPanelVisibleRows) * base.rowHeight * scale))
			{
				++heightWasScaled;
			}
		}
	}

	CHECK(swept == kProbedViewportCount * kProbedScaleCount);
	CHECK(flushLeft == swept);
	CHECK(centred == swept);
	CHECK(marginsEqual == swept);
	CHECK(heightWasScaled == swept);

	// A worked case in absolute pixels: 24 rows at 1.5 is 648 px, centred on 720. The
	// scale is deliberately not 1, where scale-then-centre and centre-then-scale agree.
	const PanelLayout shipped =
		placedPanelLayout(base, 1.5f, kPanelVisibleRows, 720.f);
	CHECK(near(panelWindowHeight(shipped), 648.f));
	CHECK(near(shipped.originY, 36.f));
	CHECK(near(shipped.rowWidth, 126.f));

	// THE SEEDED MISTAKE: centre the UNSCALED height instead. It agrees at scale 1 and
	// disagrees everywhere else, so a suite that only ran the default would miss it.
	std::size_t agreedAtOne     = 0u;
	std::size_t disagreedBeyond = 0u;

	for (std::size_t index = 0u; index < kProbedScaleCount; ++index)
	{
		const float scale = kProbedScales[index];
		const float wrong = panelCenteredOriginY(720.f, panelWindowHeight(base));
		const float right =
			placedPanelLayout(base, scale, kPanelVisibleRows, 720.f).originY;

		if (scale == 1.f && near(wrong, right))
			++agreedAtOne;

		if (scale != 1.f && !near(wrong, right))
			++disagreedBeyond;
	}

	CHECK(agreedAtOne == 1u);
	CHECK(disagreedBeyond == kProbedScaleCount - 1u);
}

// ---------------------------------------------------------------------------
// Three console values, three ranges, and the same contract the lane window already
// offers: OUT OF RANGE IS PULLED TO THE NEARER END, never rejected and never resizing
// anything. A rejected value would leave the console echoing a number nothing uses.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.EveryConsoleValueIsClampedToItsStatedRangeAndNoneOfThemRejects",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	// The shipped defaults, which are the user's ruling and belong in a test rather
	// than in a comment somebody has to trust.
	CHECK(kPanelDefaultScale == 1.0f);
	CHECK(kPanelDefaultBackgroundAlpha == 0.f);
	CHECK(kPanelVisibleRows == 24u);

	CHECK(kPanelMinScale == 0.25f);
	CHECK(kPanelMaxScale == 4.f);
	CHECK(kPanelMinBackgroundAlpha == 0.f);
	CHECK(kPanelMaxBackgroundAlpha == 1.f);
	CHECK(kPanelMinVisibleRows == 1u);

	// The row window's upper end is the RING's capacity, not a number of its own: a
	// larger window would reserve height for rows that cannot exist.
	CHECK(kPanelMaxVisibleRows == InputHistoryRowRing::capacity());
	CHECK(kPanelMaxVisibleRows == 64u);

	CHECK(clampPanelScale(-4.f) == kPanelMinScale);
	CHECK(clampPanelScale(0.f) == kPanelMinScale);
	CHECK(clampPanelScale(kPanelMinScale) == kPanelMinScale);
	CHECK(clampPanelScale(kPanelDefaultScale) == kPanelDefaultScale);
	CHECK(clampPanelScale(kPanelMaxScale) == kPanelMaxScale);
	CHECK(clampPanelScale(1000.f) == kPanelMaxScale);

	// A console float can arrive as a non-number, and it would otherwise multiply every
	// number in the layout into one. It fails both comparisons, so it lands on the floor.
	CHECK(clampPanelScale(std::numeric_limits<float>::quiet_NaN()) == kPanelMinScale);

	CHECK(clampPanelBackgroundAlpha(-1.f) == kPanelMinBackgroundAlpha);
	CHECK(clampPanelBackgroundAlpha(0.f) == kPanelMinBackgroundAlpha);
	CHECK(clampPanelBackgroundAlpha(kPanelDefaultBackgroundAlpha)
	      == kPanelDefaultBackgroundAlpha);
	CHECK(clampPanelBackgroundAlpha(1.f) == kPanelMaxBackgroundAlpha);
	CHECK(clampPanelBackgroundAlpha(7.5f) == kPanelMaxBackgroundAlpha);
	CHECK(clampPanelBackgroundAlpha(std::numeric_limits<float>::quiet_NaN())
	      == kPanelMinBackgroundAlpha);

	// Taken as int64 so a negative console value clamps instead of wrapping enormous.
	CHECK(clampPanelVisibleRows(-9) == kPanelMinVisibleRows);
	CHECK(clampPanelVisibleRows(0) == kPanelMinVisibleRows);
	CHECK(clampPanelVisibleRows(1) == 1u);
	CHECK(clampPanelVisibleRows(24) == kPanelVisibleRows);
	CHECK(clampPanelVisibleRows(64) == kPanelMaxVisibleRows);
	CHECK(clampPanelVisibleRows(65) == kPanelMaxVisibleRows);
	CHECK(clampPanelVisibleRows(1000000) == kPanelMaxVisibleRows);
}

// ---------------------------------------------------------------------------
// The row window is a READ BOUND. Shrinking it must draw fewer rows and store exactly
// as many as before -- if it resized anything, a user tuning the display live would be
// throwing away history to look at less of it.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.TheRowWindowChangesWhatIsDrawnAndNeverWhatIsStored",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;

	InputHistoryRowRing ring = ringOfRows(InputHistoryRowRing::capacity());
	CHECK(ring.size() == 64u);

	const std::size_t windows[] = { 1u, 8u, 19u, 24u, 40u, 64u };

	std::size_t drewItsWindow   = 0u;
	std::size_t newestStayedTop = 0u;
	std::size_t storageUnmoved  = 0u;

	for (std::size_t index = 0u; index < 6u; ++index)
	{
		const PanelLayout layout = placedPanelLayout(PanelLayout{}, kPanelDefaultScale,
		                                             windows[index], 1080.f);

		if (panelDrawnRowCount(layout, ring.size()) == windows[index])
			++drewItsWindow;

		// Slot 0 is the newest whatever the window is -- the window trims the BOTTOM.
		if (ring.at(panelRingIndexForSlot(ring.size(), 0u)).firstCaptureTick
		    == ring.newest().firstCaptureTick)
		{
			++newestStayedTop;
		}

		if (ring.size() == 64u && InputHistoryRowRing::capacity() == 64u)
			++storageUnmoved;
	}

	CHECK(drewItsWindow == 6u);
	CHECK(newestStayedTop == 6u);
	CHECK(storageUnmoved == 6u);

	// A window larger than the ring holds draws what there is, not what it reserved.
	const InputHistoryRowRing shallow = ringOfRows(5u);
	const PanelLayout         wide =
		placedPanelLayout(PanelLayout{}, 1.f, kPanelMaxVisibleRows, 1080.f);
	CHECK(panelDrawnRowCount(wide, shallow.size()) == 5u);

	// ...and the reserved height still follows the window, which is what keeps the
	// panel's top edge from creeping upward as rows arrive.
	CHECK(near(panelWindowHeight(wide),
	           static_cast<float>(kPanelMaxVisibleRows) * 18.f));
}

// ---------------------------------------------------------------------------
// A tall centred panel and a bottom-centred frame meter can share a band of y that they
// never used to. At the ruled defaults they clear each other in BOTH axes; this case
// states the margins as NUMBERS at both aspect ratios, and pins the scales at which each
// clearance is lost -- reporting the overlap rather than asserting it away.
// ---------------------------------------------------------------------------
TEST_CASE("Panel.ThePanelClearsTheFrameMeterAtTheRuledDefaultsInBothAxes",
          "[CharacterViz][InputHistoryViz]")
{
	using namespace inputhistoryvizpaneltests;
	using brawlerInputHistoryVisualization::FrameMeterGeometry;
	using brawlerInputHistoryVisualization::FrameMeterLayout;
	using brawlerInputHistoryVisualization::frameMeterGeometryFor;
	using brawlerInputHistoryVisualization::kTickLaneDefaultRetainedTicks;
	using brawlerInputHistoryVisualization::kTickLaneMaxRetainedTicks;

	const FrameMeterLayout meterLayout;
	const PanelLayout      base;

	// At the default retained window and the ruled panel scale, at 720p.
	const PanelLayout panel =
		placedPanelLayout(base, kPanelDefaultScale, kPanelVisibleRows, 720.f);
	const FrameMeterGeometry meter = frameMeterGeometryFor(meterLayout, 1280.f, 720.f,
		kTickLaneDefaultRetainedTicks);

	const float panelRight = panel.originX + panel.rowWidth;
	const float meterLeft  = meter.originX - meterLayout.backdropPadding;

	CHECK(near(panelRight, 84.f));
	CHECK(near(meterLeft, 157.f));
	CHECK(panelRight < meterLeft);

	// And at this scale they no longer share a band of y either, so the separation does
	// not rest on x alone the way it did at the larger panel.
	CHECK(panel.originY < meter.originY);
	CHECK(panel.originY + panelWindowHeight(panel) < meter.originY);

	// Ultrawide moves the meter further right, so the clearance only grows.
	const FrameMeterGeometry wide = frameMeterGeometryFor(meterLayout, 3440.f, 1440.f,
		kTickLaneDefaultRetainedTicks);
	const PanelLayout widePanel =
		placedPanelLayout(base, kPanelDefaultScale, kPanelVisibleRows, 1440.f);

	CHECK(widePanel.originX + widePanel.rowWidth
	      < wide.originX - meterLayout.backdropPadding);
	CHECK(wide.originX > meter.originX);

	// And the settings at which each clearance is GONE, pinned here rather than left to be
	// discovered in play: a larger panel at 720p, and the widest lane window at 720p.
	const PanelLayout big = placedPanelLayout(base, 2.5f, kPanelVisibleRows, 720.f);
	CHECK(big.originX + big.rowWidth > meterLeft);
	CHECK(big.originY + panelWindowHeight(big) > meter.originY);

	// The vertical clearance is the first to go, well below the scale that closes x.
	const PanelLayout taller = placedPanelLayout(base, 1.5f, kPanelVisibleRows, 720.f);
	CHECK(taller.originY + panelWindowHeight(taller) > meter.originY);
	CHECK(taller.originX + taller.rowWidth < meterLeft);

	const FrameMeterGeometry widest = frameMeterGeometryFor(meterLayout, 1280.f, 720.f,
		kTickLaneMaxRetainedTicks);
	CHECK(panelRight > widest.originX - meterLayout.backdropPadding);

	// ---------------------------------------------------------------------------
	// barCount 3 -- the delay bar on. The meter grows 17 px taller and its top moves UP
	// by that much, so the panel must clear the taller meter too, at the same defaults.
	// ---------------------------------------------------------------------------
	const FrameMeterGeometry meterThree = frameMeterGeometryFor(
		meterLayout, 1280.f, 720.f, kTickLaneDefaultRetainedTicks, 3u);
	const float meterThreeLeft = meterThree.originX - meterLayout.backdropPadding;

	CHECK(panelRight < meterThreeLeft);
	CHECK(panel.originY + panelWindowHeight(panel) < meterThree.originY);
	CHECK(near(meter.originY - meterThree.originY, 17.f));

	// The taller meter loses vertical clearance FIRST: at scale 1.18 barCount 3 has
	// already swallowed it while barCount 2 -- unedited above -- still clears.
	const PanelLayout tallerThree = placedPanelLayout(base, 1.18f, kPanelVisibleRows, 720.f);
	CHECK(tallerThree.originY + panelWindowHeight(tallerThree) > meterThree.originY);
	CHECK(tallerThree.originY + panelWindowHeight(tallerThree) < meter.originY);
}

#endif // WITH_LOW_LEVEL_TESTS
