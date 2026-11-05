// SPDX-License-Identifier: BUSL-1.1
// docs/AimPlaneTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/InputMapping/AimPlane.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SurfaceFrame.h"

#include "glm/geometric.hpp"
#include "glm/vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>

#if __has_include("Math/UnrealMathUtility.h")
#include "CoreMinimal.h"
#define OG_AIM_PLANE_TEST_HAS_UNREAL_MATH 1
#else
#define OG_AIM_PLANE_TEST_HAS_UNREAL_MATH 0
#endif

namespace aimPlaneTest
{
constexpr double kPi = 3.14159265358979323846;

static_assert(simFrame::kForward == glm::vec3(0.f, 0.f, -1.f) && simFrame::kRight == glm::vec3(1.f, 0.f, 0.f)
		&& simFrame::kUp == glm::vec3(0.f, 1.f, 0.f),
	"AimPlaneTest: ueToSimAxes hard-codes og-simulation's Y-up frame (Unreal X -> -Z, Y -> X, Z -> Y)");

#if OG_AIM_PLANE_TEST_HAS_UNREAL_MATH
static_assert(dInput::aimPlane::kParallelTolerance == UE_KINDA_SMALL_NUMBER
		&& dInput::aimPlane::kZeroAimTolerance == UE_KINDA_SMALL_NUMBER,
	"AimPlaneTest: the helper's two rejects use the engine's KINDA_SMALL_NUMBER, as the input component did");
#endif

glm::vec3 ueToSimAxes(const glm::dvec3& v)
{
	return glm::vec3(static_cast<float>(v.y), static_cast<float>(v.z), static_cast<float>(-v.x));
}

std::uint32_t bitsOf(float value)
{
	std::uint32_t bits = 0u;
	std::memcpy(&bits, &value, sizeof(bits));
	return bits;
}

bool sameBits(const std::optional<glm::vec3>& a, const std::optional<glm::vec3>& b)
{
	if (a.has_value() != b.has_value())
		return false;
	if (!a.has_value())
		return true;
	return bitsOf(a->x) == bitsOf(b->x) && bitsOf(a->y) == bitsOf(b->y) && bitsOf(a->z) == bitsOf(b->z);
}

std::optional<glm::vec3> todaysMouseAim(const glm::dvec3& origin, const glm::dvec3& direction, const glm::dvec3& capsule)
{
#if OG_AIM_PLANE_TEST_HAS_UNREAL_MATH
	const FVector worldLocation(origin.x, origin.y, origin.z);
	const FVector worldDirection(direction.x, direction.y, direction.z);
	const FVector planeNormal = FVector(0.f, 0.f, 1.f);
	const FVector planePoint(capsule.x, capsule.y, capsule.z);
	const FVector planePointToLinePoint = worldLocation - planePoint;
	const float dotProduct = FVector::DotProduct(planeNormal, worldDirection);
	if (FMath::Abs(dotProduct) < UE_KINDA_SMALL_NUMBER)
		return std::nullopt;
	const float distance = FVector::DotProduct(planeNormal, planePointToLinePoint) / dotProduct;
	const FVector intersectionPoint = worldLocation - distance * worldDirection;
	const FVector aimVec = intersectionPoint - planePoint;
	if (aimVec.IsNearlyZero())
		return std::nullopt;
	return glm::normalize(ueToSimAxes(glm::dvec3(aimVec.X, aimVec.Y, aimVec.Z)));
#else
	const glm::dvec3 planePointToLinePoint = origin - capsule;
	const float dotProduct = static_cast<float>(0.0 * direction.x + 0.0 * direction.y + 1.0 * direction.z);
	if (std::fabs(dotProduct) < 1.e-4f)
		return std::nullopt;
	const float distance = static_cast<float>(
		(0.0 * planePointToLinePoint.x + 0.0 * planePointToLinePoint.y + 1.0 * planePointToLinePoint.z) / dotProduct);
	const glm::dvec3 intersectionPoint = origin - direction * static_cast<double>(distance);
	const glm::dvec3 aimVec = intersectionPoint - capsule;
	constexpr double tolerance = 1.e-4f;
	if (std::fabs(aimVec.x) <= tolerance && std::fabs(aimVec.y) <= tolerance && std::fabs(aimVec.z) <= tolerance)
		return std::nullopt;
	return glm::normalize(ueToSimAxes(aimVec));
#endif
}

glm::dvec3 upTilted(double degrees, bool towardMinusY)
{
	const double radians = degrees * kPi / 180.0;
	const double across = -std::sin(radians);
	return towardMinusY ? glm::dvec3(0.0, across, std::cos(radians)) : glm::dvec3(across, 0.0, std::cos(radians));
}

glm::dvec3 roundedToFloat(const glm::dvec3& v)
{
	return glm::dvec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

struct PlaneBasis
{
	glm::dvec3 u;
	glm::dvec3 v;
};

PlaneBasis basisOf(const glm::dvec3& n)
{
	const glm::dvec3 reference = std::fabs(n.y) < 0.9 ? glm::dvec3(0.0, 1.0, 0.0) : glm::dvec3(1.0, 0.0, 0.0);
	const glm::dvec3 v = glm::normalize(glm::cross(n, reference));
	return PlaneBasis{ glm::cross(v, n), v };
}

glm::dvec3 cameraAt(const glm::dvec3& capsule, double distanceCm, double yawDeg, double pitchDeg)
{
	const double yaw = yawDeg * kPi / 180.0;
	const double pitch = pitchDeg * kPi / 180.0;
	return capsule + distanceCm * glm::dvec3(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));
}

double angleRad(const glm::dvec3& a, const glm::dvec3& b)
{
	return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

glm::dvec3 widened(const glm::vec3& v)
{
	return glm::dvec3(v.x, v.y, v.z);
}

const glm::dvec3 kCapsules[] = { { 0.0, 0.0, 96.0 }, { 1234.5678, -987.654321, 96.25 }, { -3210.125, 4567.875, 1301.0625 },
	{ 731.3, 12.7, -455.9 } };
constexpr double kOffsetsCm[] = { -1500.0, -700.0, -250.0, 0.0, 250.0, 700.0, 1500.0 };
constexpr double kCameraDistancesCm[] = { 600.0, 2500.0 };
constexpr double kCameraPitchesDeg[] = { 10.0, 30.0, 45.0, 60.0, 80.0 };

template <typename F>
void forEachCameraAndTarget(const glm::dvec3& capsule, const PlaneBasis& basis, const glm::dvec3& lift, F&& visit)
{
	for (const double distanceCm : kCameraDistancesCm)
		for (const double pitchDeg : kCameraPitchesDeg)
			for (int yawStep = 0; yawStep < 24; ++yawStep)
			{
				const glm::dvec3 origin = cameraAt(capsule, distanceCm, yawStep * 15.0 + 7.5, pitchDeg);
				for (const double a : kOffsetsCm)
					for (const double b : kOffsetsCm)
					{
						if (a == 0.0 && b == 0.0)
							continue;
						const glm::dvec3 target = capsule + a * basis.u + b * basis.v + lift;
						visit(origin, glm::normalize(target - origin), target, distanceCm);
					}
			}
}

std::optional<glm::vec3> aimOn(const glm::dvec3& origin, const glm::dvec3& direction, const glm::dvec3& capsule, const glm::dvec3& n)
{
	return dInput::aimPlane::unitAim(origin, direction, capsule, n, &ueToSimAxes);
}

glm::dvec3 swingWorldHeading(const glm::vec3& upSim, const glm::vec3& aimSim)
{
	const glm::vec3 local = glm::normalize(simFrame::flatten(surfaceFrame::tiltFromSurface(upSim, aimSim)));
	return widened(surfaceFrame::tiltToSurface(upSim, local));
}
} // namespace aimPlaneTest

TEST_CASE("AimPlane.WorldUpIsTodaysMouseAimBitForBit", "[DAttack][AimPlane]")
{
	using namespace aimPlaneTest;
	const glm::dvec3 worldUp(0.0, 0.0, 1.0);
	const PlaneBasis basis{ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
	int compared = 0;
	int mismatched = 0;
	int rejected = 0;
	double worstOffPlane = 0.0;
	for (const glm::dvec3& capsule : kCapsules)
		for (const double liftCm : { -50.0, 0.0, 37.5 })
			forEachCameraAndTarget(capsule, basis, glm::dvec3(0.0, 0.0, liftCm),
				[&](const glm::dvec3& origin, const glm::dvec3& direction, const glm::dvec3&, double) {
					const std::optional<glm::vec3> today = todaysMouseAim(origin, direction, capsule);
					const std::optional<glm::vec3> helper = aimOn(origin, direction, capsule, worldUp);
					++compared;
					if (!sameBits(today, helper))
						++mismatched;
					if (!helper.has_value())
						++rejected;
					else
						worstOffPlane = std::max(worstOffPlane, std::fabs(static_cast<double>(helper->y)));
				});

	const glm::dvec3 capsule = kCapsules[1];
	const glm::dvec3 levelOrigin(capsule.x + 400.0, capsule.y - 300.0, capsule.z);
	const glm::dvec3 above = capsule + glm::dvec3(0.0, 0.0, 300.0);
	const glm::dvec3 rays[][2] = { { levelOrigin, glm::normalize(glm::dvec3(-1.0, 0.2, -0.5)) },
		{ levelOrigin, glm::normalize(glm::dvec3(0.3, 1.0, 0.25)) }, { levelOrigin, glm::dvec3(1.0, 0.0, 0.0) },
		{ levelOrigin, glm::normalize(glm::dvec3(0.0, 1.0, 9.0e-5)) }, { levelOrigin, glm::normalize(glm::dvec3(0.0, 1.0, -1.5e-4)) },
		{ above, glm::dvec3(0.0, 0.0, -1.0) }, { above, glm::normalize(glm::dvec3(0.25, 0.0, -300.0)) } };
	for (const auto& ray : rays)
	{
		const std::optional<glm::vec3> today = todaysMouseAim(ray[0], ray[1], capsule);
		const std::optional<glm::vec3> helper = aimOn(ray[0], ray[1], capsule, worldUp);
		++compared;
		if (!sameBits(today, helper))
			++mismatched;
		if (!helper.has_value())
			++rejected;
	}

	std::printf("[AimPlane] world up: compared %d mismatched %d rejected %d, worst |aim.y| %.9g\n", compared, mismatched,
		rejected, worstOffPlane);
	CHECK(compared == 4 * 3 * 2 * 5 * 24 * 48 + 7);
	CHECK(mismatched == 0);
	CHECK(rejected == 3);
}

TEST_CASE("AimPlane.ThirtyDegreeAimLiesInThePlaneAndPointsAtTheHit", "[DAttack][AimPlane]")
{
	using namespace aimPlaneTest;
	for (const bool towardMinusY : { false, true })
	{
		const glm::dvec3 n = roundedToFloat(upTilted(30.0, towardMinusY));
		const glm::dvec3 nSim = widened(ueToSimAxes(n));
		const PlaneBasis basis = basisOf(n);
		int checked = 0;
		int rejected = 0;
		double worstDot = 0.0;
		double worstAngle = 0.0;
		for (const glm::dvec3& capsule : kCapsules)
			forEachCameraAndTarget(capsule, basis, glm::dvec3(0.0),
				[&](const glm::dvec3& origin, const glm::dvec3& direction, const glm::dvec3& target, double) {
					const std::optional<glm::vec3> aim = aimOn(origin, direction, capsule, n);
					if (!aim.has_value())
					{
						++rejected;
						return;
					}
					++checked;
					const glm::dvec3 toHit = widened(ueToSimAxes(target - capsule));
					worstDot = std::max(worstDot, std::fabs(glm::dot(widened(*aim), nSim)));
					worstAngle = std::max(worstAngle, angleRad(widened(*aim), toHit));
				});
		std::printf("[AimPlane] 30 deg %s: checked %d rejected %d, worst |dot(aim, n)| %.9g, worst angle to the hit %.9g rad\n",
			towardMinusY ? "toward -Y" : "toward -X", checked, rejected, worstDot, worstAngle);
		CHECK(checked == 4 * 2 * 5 * 24 * 48);
		CHECK(rejected == 0);
		CHECK(worstDot <= 1e-6);
		CHECK(worstAngle <= 1e-6);
	}
}

TEST_CASE("AimPlane.TheSwingHeadingPassesUnderTheCursorOnTheSlope", "[DAttack][AimPlane]")
{
	using namespace aimPlaneTest;
	const glm::dvec3 n = roundedToFloat(upTilted(30.0, false));
	const glm::vec3 upSim = ueToSimAxes(n);
	const PlaneBasis basis = basisOf(n);
	const glm::dvec3 worldUp(0.0, 0.0, 1.0);
	double worstInPlaneLift = 0.0;
	double worstHeading = 0.0;
	double worstTodaysHeadingThirdPerson = 0.0;
	double worstTodaysHeadingInFront = 0.0;
	int todaysPlaneBehindTheCamera = 0;
	for (const glm::dvec3& capsule : kCapsules)
		for (const double distanceCm : kCameraDistancesCm)
			for (const double pitchDeg : kCameraPitchesDeg)
				for (int yawStep = 0; yawStep < 24; ++yawStep)
				{
					const glm::dvec3 origin = cameraAt(capsule, distanceCm, yawStep * 15.0 + 7.5, pitchDeg);
					for (const double a : kOffsetsCm)
						for (const double b : kOffsetsCm)
						{
							if (a == 0.0 && b == 0.0)
								continue;
							const glm::dvec3 target = capsule + a * basis.u + b * basis.v;
							const glm::dvec3 direction = glm::normalize(target - origin);
							const glm::dvec3 toHit = widened(ueToSimAxes(target - capsule));
							const std::optional<glm::vec3> aim = aimOn(origin, direction, capsule, n);
							REQUIRE(aim.has_value());
							const glm::vec3 local = surfaceFrame::tiltFromSurface(upSim, *aim);
							worstInPlaneLift = std::max(worstInPlaneLift, std::fabs(static_cast<double>(simFrame::height(local))));
							worstHeading = std::max(worstHeading, angleRad(swingWorldHeading(upSim, *aim), toHit));

							const std::optional<glm::dvec3> todaysOffset = dInput::aimPlane::hitOffset(origin, direction, capsule, worldUp);
							REQUIRE(todaysOffset.has_value());
							if (glm::dot(*todaysOffset + capsule - origin, direction) <= 0.0)
							{
								++todaysPlaneBehindTheCamera;
								continue;
							}
							const std::optional<glm::vec3> today = todaysMouseAim(origin, direction, capsule);
							REQUIRE(today.has_value());
							const double todaysGap = angleRad(swingWorldHeading(upSim, *today), toHit);
							worstTodaysHeadingInFront = std::max(worstTodaysHeadingInFront, todaysGap);
							if (distanceCm == 2500.0 && pitchDeg == 60.0)
								worstTodaysHeadingThirdPerson = std::max(worstTodaysHeadingThirdPerson, todaysGap);
						}
				}
	std::printf("[AimPlane] swing heading: worst local lift %.9g, worst heading to the cursor %.9g rad; today's horizontal "
				"plane: %d rays hit it behind the camera, worst heading %.6g deg in front, %.6g deg at 2500 cm / 60 deg\n",
		worstInPlaneLift, worstHeading, todaysPlaneBehindTheCamera, worstTodaysHeadingInFront * 180.0 / kPi,
		worstTodaysHeadingThirdPerson * 180.0 / kPi);
	CHECK(worstInPlaneLift <= 1e-6);
	CHECK(worstHeading <= 1e-5);
	CHECK(worstTodaysHeadingThirdPerson * 180.0 / kPi > 1.0);
}

TEST_CASE("AimPlane.ANearParallelRayIsRejected", "[DAttack][AimPlane]")
{
	using namespace aimPlaneTest;
	const glm::dvec3 capsule = kCapsules[1];
	for (const glm::dvec3& n : { glm::dvec3(0.0, 0.0, 1.0), roundedToFloat(upTilted(30.0, false)), roundedToFloat(upTilted(30.0, true)) })
	{
		const PlaneBasis basis = basisOf(n);
		const glm::dvec3 origin = capsule + 800.0 * n + 300.0 * basis.v;
		CHECK_FALSE(aimOn(origin, basis.u, capsule, n).has_value());
		CHECK_FALSE(aimOn(origin, -basis.v, capsule, n).has_value());
		CHECK_FALSE(aimOn(origin, glm::normalize(basis.u - 9.0e-5 * n), capsule, n).has_value());
		CHECK_FALSE(aimOn(origin, glm::normalize(basis.u + 9.0e-5 * n), capsule, n).has_value());
		CHECK(aimOn(origin, glm::normalize(basis.u - 1.5e-4 * n), capsule, n).has_value());
		CHECK(aimOn(origin, glm::normalize(basis.u + 1.5e-4 * n), capsule, n).has_value());
	}
}

TEST_CASE("AimPlane.AZeroAimIsRejected", "[DAttack][AimPlane]")
{
	using namespace aimPlaneTest;
	for (const glm::dvec3& capsule : kCapsules)
		for (const glm::dvec3& n : { glm::dvec3(0.0, 0.0, 1.0), roundedToFloat(upTilted(30.0, false)), roundedToFloat(upTilted(30.0, true)) })
		{
			const PlaneBasis basis = basisOf(n);
			const glm::dvec3 origin = capsule + 300.0 * n;
			CHECK_FALSE(aimOn(origin, -n, capsule, n).has_value());
			CHECK_FALSE(dInput::aimPlane::hitOffset(origin, -n, capsule, n).has_value());
			CHECK_FALSE(aimOn(origin, glm::normalize(capsule + 0.5e-4 * basis.u - origin), capsule, n).has_value());
			const glm::dvec3 nearby = capsule + 0.25 * basis.u;
			CHECK(aimOn(origin, glm::normalize(nearby - origin), capsule, n).has_value());
		}
}

TEST_CASE("AimPlane.TheTiltedPlaneUpIsEmptyOnFlatGround", "[DAttack][AimPlane]")
{
	using namespace aimPlaneTest;
	using dInput::aimPlane::tiltedPlaneUp;
	CHECK_FALSE(tiltedPlaneUp(simFrame::kUp).has_value());
	CHECK_FALSE(tiltedPlaneUp(glm::vec3(0.f, 0.99999994f, 0.f)).has_value());
	CHECK_FALSE(tiltedPlaneUp(glm::vec3(5.e-7f, 1.f, 0.f)).has_value());
	CHECK_FALSE(tiltedPlaneUp(glm::vec3(0.f)).has_value());
	CHECK_FALSE(tiltedPlaneUp(glm::vec3(0.f, -1.f, 0.f)).has_value());
	CHECK_FALSE(tiltedPlaneUp(glm::vec3(0.5f, -0.8660254f, 0.f)).has_value());
	CHECK_FALSE(tiltedPlaneUp(glm::vec3(1.f, 0.f, 0.f)).has_value());

	for (const glm::vec3& up : { glm::vec3(2.e-6f, 1.f, 0.f), ueToSimAxes(roundedToFloat(upTilted(30.0, false))),
			 ueToSimAxes(roundedToFloat(upTilted(30.0, true))), ueToSimAxes(roundedToFloat(upTilted(12.0, false))) })
	{
		const std::optional<glm::vec3> tilted = tiltedPlaneUp(up);
		REQUIRE(tilted.has_value());
		CHECK(bitsOf(tilted->x) == bitsOf(up.x));
		CHECK(bitsOf(tilted->y) == bitsOf(up.y));
		CHECK(bitsOf(tilted->z) == bitsOf(up.z));
	}
}

#endif // WITH_LOW_LEVEL_TESTS
