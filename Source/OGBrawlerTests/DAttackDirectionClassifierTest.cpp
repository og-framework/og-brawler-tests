// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// Pins dAttackDirection::classify — the ONE definition of "which way does an attack
// go", shared by the authoritative simulation and the indicator that draws it.
//
// ⛔ THE CASE THIS FILE EXISTS FOR is the anti-parallel singularity: backing straight
// away from aim makes cross(aimXY, moveXY).z collapse to ~0, its sign flip on FP
// noise, and the sequence flicker between the two sides every frame. It was a live
// bug (pressing S opposite the aim). A test that only checks the happy path would
// have passed throughout.

#include "catch_amalgamated.hpp"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/DAttackSequenceId.h"

#include <glm/gtc/matrix_transform.hpp>

namespace
{
	constexpr unsigned int kForward      = dAttackDirection::kForwardSequenceId;
	constexpr unsigned int kPositiveSide = dAttackDirection::kLeftSequenceId;
	constexpr unsigned int kNegativeSide = dAttackDirection::kRightSequenceId;

	// Rotate +X by `degrees` about Z — the plane the classifier projects onto.
	glm::vec3 dirAt(float degrees)
	{
		const float r = glm::radians(degrees);
		return glm::vec3(std::cos(r), std::sin(r), 0.f);
	}

	unsigned int classifyAt(float moveDegrees, const glm::vec3& aim = glm::vec3(1.f, 0.f, 0.f))
	{
		const glm::vec3 move = dirAt(moveDegrees);
		return dAttackDirection::classify(aim, move, glm::vec2(move.x, move.y));
	}
}

TEST_CASE("classify: no movement attacks forward", "[attackdirection]")
{
	const unsigned int d = dAttackDirection::classify(
		glm::vec3(1.f, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f), glm::vec2(0.f, 0.f));
	CHECK(d == kForward);
}

TEST_CASE("classify: movement aligned with aim attacks forward", "[attackdirection]")
{
	const unsigned int d = classifyAt(0.f);
	CHECK(d == kForward);
}

TEST_CASE("classify: movement outside the side cone picks a side", "[attackdirection]")
{
	CHECK(classifyAt(90.f)  == kPositiveSide);
	CHECK(classifyAt(-90.f) == kNegativeSide);
	CHECK(classifyAt(90.f)  != kForward);
	CHECK(classifyAt(-90.f) != kForward);
}

TEST_CASE("classify: the two sides are genuinely distinct", "[attackdirection]")
{
	// A test that could not tell the sides apart would pass even if classify
	// returned a constant, so assert the disagreement explicitly.
	CHECK(classifyAt(90.f) != classifyAt(-90.f));
}

// ---------------------------------------------------------------------------
// ⛔ THE REGRESSION THIS FILE EXISTS FOR.
// ---------------------------------------------------------------------------

TEST_CASE("classify: backing straight away from aim attacks FORWARD, not a side",
          "[attackdirection][regression]")
{
	const unsigned int d = classifyAt(180.f);
	CHECK(d == kForward);
}

TEST_CASE("classify: the back cone is STABLE — no side flicker near anti-parallel",
          "[attackdirection][regression]")
{
	// The old code flipped between the two sides across this range because the
	// cross product's sign was noise. Every sample inside the cone must be forward,
	// and — the part that actually pins the bug — they must all AGREE.
	for (float deg = 171.f; deg <= 189.f; deg += 0.5f)
	{
		const unsigned int d = classifyAt(deg);
		INFO("move angle = " << deg);
		CHECK(d == kForward);
	}
}

TEST_CASE("classify: the back cone has EDGES — just outside it, sides resume",
          "[attackdirection][regression]")
{
	// Without this the previous test would pass on a classify() that returned
	// forward unconditionally. 169 deg is outside the 10 deg cone on both sides.
	CHECK(classifyAt(169.f)  == kPositiveSide);
	CHECK(classifyAt(-169.f) == kNegativeSide);
}

TEST_CASE("classify: the back cone is independent of the aim direction",
          "[attackdirection][regression]")
{
	// The singularity is a property of the ANGLE BETWEEN the vectors, not of any
	// particular world axis, so it must hold whatever the player is aiming at.
	for (float aimDeg = 0.f; aimDeg < 360.f; aimDeg += 37.f)
	{
		const glm::vec3 aim  = dirAt(aimDeg);
		const glm::vec3 back = -aim;
		const unsigned int d = dAttackDirection::classify(aim, back, glm::vec2(back.x, back.y));
		INFO("aim angle = " << aimDeg);
		CHECK(d == kForward);
	}
}

TEST_CASE("classify: aim with downward z still classifies on the XY projection",
          "[attackdirection]")
{
	// Mouse-on-floor aim comes from a capsule above z=0, so aimDirection carries a
	// downward z. Projecting is what keeps that from inflating the angle.
	const glm::vec3 aimDown = glm::normalize(glm::vec3(1.f, 0.f, -0.6f));
	const glm::vec3 move    = glm::vec3(1.f, 0.f, 0.f);
	const unsigned int d = dAttackDirection::classify(aimDown, move, glm::vec2(move.x, move.y));
	CHECK(d == kForward);
}

TEST_CASE("classify: the two cone constants are separate and differently sized",
          "[attackdirection]")
{
	// They answer different questions and must not be silently coupled.
	CHECK(dAttackDirection::kBackpedalConeHalfAngle != dAttackDirection::kForwardConeHalfAngle);
	CHECK(dAttackDirection::kBackpedalConeHalfAngle > 0.f);
}

#endif // WITH_LOW_LEVEL_TESTS
