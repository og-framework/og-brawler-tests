// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraFramingTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>

#if __has_include("Math/UnrealMathUtility.h")
#include "CoreMinimal.h"
#define OG_CAMERA_FRAMING_TEST_HAS_UNREAL_MATH 1
#else
#define OG_CAMERA_FRAMING_TEST_HAS_UNREAL_MATH 0
#endif

namespace dAttackCameraFramingTest
{
using dAttackCameraBehaviour::FovAxis;
using dAttackCameraBehaviour::OrbitCameraSettings;

constexpr float kAspects[] = { 4.f / 3.f, 16.f / 9.f, 21.f / 9.f, 9.f / 16.f };
constexpr const char* kAspectNames[] = { "4:3", "16:9", "21:9", "9:16" };
constexpr float kTouchToleranceCm = 1.f;

std::uint32_t bitsOf(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

float todaysActorDistance(float halfScreenH, float halfScreenV, float aspect, float fovDegrees, float margin,
    float minDistance, float maxDistance)
{
    const float paddedHalfH = halfScreenH + margin;
    const float paddedHalfV = halfScreenV + margin;
#if OG_CAMERA_FRAMING_TEST_HAS_UNREAL_MATH
    const float tanHalfFov = FMath::Tan(FMath::DegreesToRadians(fovDegrees * 0.5f));
    const float safeTan = FMath::Max(tanHalfFov, 0.001f);
    const float requiredDistance = FMath::Max(paddedHalfH, paddedHalfV * aspect) / safeTan;
    return FMath::Clamp(requiredDistance, minDistance, maxDistance);
#else
    const float tanHalfFov = std::tan(fovDegrees * 0.5f * (3.1415926535897932f / 180.f));
    const float safeTan = (0.001f < tanHalfFov) ? tanHalfFov : 0.001f;
    const float vertical = paddedHalfV * aspect;
    const float requiredDistance = ((vertical < paddedHalfH) ? paddedHalfH : vertical) / safeTan;
    const float atMost = (requiredDistance < maxDistance) ? requiredDistance : maxDistance;
    return (minDistance < atMost) ? atMost : minDistance;
#endif
}

float todaysBoomLength(float pitchDeg, float targetPitchDeg, const OrbitCameraSettings& settings)
{
    constexpr float kTodaysAtTarget = 900.f;
    constexpr float kTodaysAtHorizon = 400.f;
    const float target = dAttackCameraBehaviour::clampPitchDeg(targetPitchDeg, settings);
    const float towardTarget = std::clamp(pitchDeg, 0.f, target) / target;
    return kTodaysAtTarget - (kTodaysAtTarget - kTodaysAtHorizon) * (1.f - towardTarget);
}

struct VisibleHalfExtents
{
    double horizontal;
    double vertical;
};

VisibleHalfExtents visibleOnTheShorterAxis(double distance, float fovDeg, float aspect)
{
    const double tanShorter = std::tan(static_cast<double>(fovDeg) * 0.5 * 3.14159265358979323846 / 180.0);
    const double shorter = distance * tanShorter;
    if (aspect >= 1.f)
        return VisibleHalfExtents{ shorter * aspect, shorter };
    return VisibleHalfExtents{ shorter, shorter / aspect };
}

enum class AspectRatioAxisConstraint
{
    MaintainXFov,
    MaintainYFov
};

constexpr AspectRatioAxisConstraint kCamerasAxisConstraint = AspectRatioAxisConstraint::MaintainXFov;
constexpr double kCameraComponentAspectRatio = 1.777778;

VisibleHalfExtents renderedHalfExtents(
    double distance, float fieldOfViewDeg, AspectRatioAxisConstraint constraint, float windowAspect)
{
    constexpr double kPi = 3.14159265358979323846;
    const double halfFov = std::max(0.001, static_cast<double>(fieldOfViewDeg)) * kPi / 360.0;
    const bool maintainX = constraint == AspectRatioAxisConstraint::MaintainXFov;
    const double xMultiplier = maintainX ? 1.0 : 1.0 / static_cast<double>(windowAspect);
    const double yMultiplier = maintainX ? static_cast<double>(windowAspect) : 1.0;
    const double matrixHalfFov = maintainX ? halfFov : std::atan(std::tan(halfFov) / kCameraComponentAspectRatio);
    const double tanMatrixHalfFov = std::tan(matrixHalfFov);
    return VisibleHalfExtents{ distance * tanMatrixHalfFov / xMultiplier, distance * tanMatrixHalfFov / yMultiplier };
}
} // namespace dAttackCameraFramingTest

using namespace dAttackCameraFramingTest;

TEST_CASE("DAttackCamera.IsoFramingOnTheHorizontalAxisIsTodaysActorFormula", "[DAttack][CameraFraming]")
{
    const float halfExtents[] = { 0.f, 37.5f, 412.25f, 1800.f, 9000.f };
    const float aspects[] = { 4.f / 3.f, 16.f / 9.f, 21.f / 9.f, 9.f / 16.f, 1.f, 1280.f / 1024.f, 2560.f / 1080.f };
    const float fovs[] = { 20.f, 11.33f, 45.f, 90.f, 0.05f };
    const float margins[] = { 800.f, 0.f, 250.5f };
    struct Limits
    {
        float minDist;
        float maxDist;
    };
    const Limits limits[] = { { 2000.f, 25000.f }, { 0.f, 1.0e9f } };
    int compared = 0;
    int mismatched = 0;
    for (const float halfH : halfExtents)
        for (const float halfV : halfExtents)
            for (const float aspect : aspects)
                for (const float fov : fovs)
                    for (const float margin : margins)
                        for (const Limits& limit : limits)
                        {
                            const float expected = todaysActorDistance(halfH, halfV, aspect, fov, margin, limit.minDist, limit.maxDist);
                            const float actual = dAttackCameraBehaviour::isoFramingDistance(halfH, halfV, aspect, fov,
                                FovAxis::Horizontal, margin, margin, limit.minDist, limit.maxDist);
                            ++compared;
                            if (bitsOf(actual) != bitsOf(expected))
                            {
                                ++mismatched;
                                UNSCOPED_INFO("halfH " << halfH << " halfV " << halfV << " aspect " << aspect << " fov "
                                    << fov << " margin " << margin << " min " << limit.minDist << " -> today "
                                    << expected << " new " << actual);
                            }
                        }
    INFO("compared " << compared << " mismatched " << mismatched);
    CHECK(compared == 5 * 5 * 7 * 5 * 3 * 2);
    CHECK(mismatched == 0);
}

TEST_CASE("DAttackCamera.HorizontalFovReturnsTheAuthoredBitsWhereNoConversionApplies", "[DAttack][CameraFraming]")
{
    const float fovs[] = { 20.f, 11.33f, 58.72f, 90.f };
    for (const float fov : fovs)
    {
        for (const float aspect : kAspects)
            CHECK(bitsOf(dAttackCameraBehaviour::horizontalFovDeg(fov, FovAxis::Horizontal, aspect)) == bitsOf(fov));
        CHECK(bitsOf(dAttackCameraBehaviour::horizontalFovDeg(fov, FovAxis::Shorter, 9.f / 16.f)) == bitsOf(fov));
        CHECK(bitsOf(dAttackCameraBehaviour::horizontalFovDeg(fov, FovAxis::Shorter, 1.f)) == bitsOf(fov));
    }
}

TEST_CASE("DAttackCamera.TheShorterAxisDefaultsLookLikeTodayOnSixteenByNine", "[DAttack][CameraFraming]")
{
    const float isoHorizontal = dAttackCameraBehaviour::horizontalFovDeg(11.33f, FovAxis::Shorter, 16.f / 9.f);
    const float thirdPersonHorizontal = dAttackCameraBehaviour::horizontalFovDeg(
        58.72f, dAttackCameraBehaviour::kFovAxis, 16.f / 9.f);
    INFO("iso " << isoHorizontal << " third person " << thirdPersonHorizontal);
    CHECK(isoHorizontal == Catch::Approx(20.f).margin(0.006f));
    CHECK(thirdPersonHorizontal == Catch::Approx(90.f).margin(0.006f));
    CHECK(dAttackCameraBehaviour::kFovAxis == FovAxis::Shorter);
    const OrbitCameraSettings settings;
    CHECK(dAttackCameraBehaviour::kFovDeg == 40.f);
    CHECK(settings.fovDeg == dAttackCameraBehaviour::kFovDeg);
    CHECK(settings.fovAxis == FovAxis::Shorter);
    CHECK(dAttackCameraBehaviour::fovAxisFromConsoleValue(0) == FovAxis::Horizontal);
    CHECK(dAttackCameraBehaviour::fovAxisFromConsoleValue(-1) == FovAxis::Horizontal);
    CHECK(dAttackCameraBehaviour::fovAxisFromConsoleValue(1) == FovAxis::Shorter);
    CHECK(dAttackCameraBehaviour::fovAxisFromConsoleValue(2) == FovAxis::Shorter);
}

TEST_CASE("DAttackCamera.IsoFramingOnTheShorterAxisTouchesTheBindingAxisAtEveryAspect", "[DAttack][CameraFraming]")
{
    struct Box
    {
        float halfH;
        float halfV;
    };
    const Box boxes[] = { { 1500.f, 200.f }, { 100.f, 1200.f }, { 600.f, 600.f }, { 0.f, 0.f }, { 2500.f, 1406.25f } };
    const float fovs[] = { 11.33f, 20.f, 40.f };
    constexpr float kMarginH = 400.f;
    constexpr float kMarginV = 300.f;
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
    {
        const float aspect = kAspects[a];
        for (const float fov : fovs)
        {
            for (const Box& box : boxes)
            {
                const float distance = dAttackCameraBehaviour::isoFramingDistance(box.halfH, box.halfV, aspect, fov,
                    FovAxis::Shorter, kMarginH, kMarginV, 0.f, 1.0e9f);
                const VisibleHalfExtents visible = visibleOnTheShorterAxis(distance, fov, aspect);
                const double paddedH = static_cast<double>(box.halfH) + kMarginH;
                const double paddedV = static_cast<double>(box.halfV) + kMarginV;
                const bool touchesH = std::fabs(visible.horizontal - paddedH) <= kTouchToleranceCm;
                const bool touchesV = std::fabs(visible.vertical - paddedV) <= kTouchToleranceCm;
                const bool fitsH = paddedH <= visible.horizontal + kTouchToleranceCm;
                const bool fitsV = paddedV <= visible.vertical + kTouchToleranceCm;
                const float fieldOfView = dAttackCameraBehaviour::horizontalFovDeg(fov, FovAxis::Shorter, aspect);
                const VisibleHalfExtents rendered = renderedHalfExtents(distance, fieldOfView, kCamerasAxisConstraint, aspect);
                INFO(kAspectNames[a] << " fov " << fov << " box (" << box.halfH << ", " << box.halfV << ") distance "
                    << distance << " visible (" << visible.horizontal << ", " << visible.vertical << ") padded ("
                    << paddedH << ", " << paddedV << ") rendered (" << rendered.horizontal << ", " << rendered.vertical
                    << ")");
                CHECK(((touchesH && fitsV) || (touchesV && fitsH)));
                CHECK(std::fabs(rendered.horizontal - visible.horizontal) <= kTouchToleranceCm);
                CHECK(std::fabs(rendered.vertical - visible.vertical) <= kTouchToleranceCm);
            }
        }
    }
}

TEST_CASE("DAttackCamera.UnderTheLocalPlayerDefaultOnlySixteenByNineRendersAsFramed", "[DAttack][CameraFraming]")
{
    const float fovs[] = { 11.33f, dAttackCameraBehaviour::kFovDeg };
    constexpr double kDistance = 10000.0;
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
    {
        const float aspect = kAspects[a];
        for (const float fov : fovs)
        {
            const float fieldOfView = dAttackCameraBehaviour::horizontalFovDeg(fov, FovAxis::Shorter, aspect);
            const VisibleHalfExtents visible = visibleOnTheShorterAxis(kDistance, fov, aspect);
            const VisibleHalfExtents asTheCameras = renderedHalfExtents(kDistance, fieldOfView, kCamerasAxisConstraint, aspect);
            const VisibleHalfExtents asTheDefault =
                renderedHalfExtents(kDistance, fieldOfView, AspectRatioAxisConstraint::MaintainYFov, aspect);
            const double expectedRatio = static_cast<double>(aspect) / kCameraComponentAspectRatio;
            INFO(kAspectNames[a] << " fov " << fov << " visible (" << visible.horizontal << ", " << visible.vertical
                << ") as the cameras (" << asTheCameras.horizontal << ", " << asTheCameras.vertical
                << ") MaintainYFov (" << asTheDefault.horizontal << ", " << asTheDefault.vertical << ")");
            CHECK(std::fabs(asTheCameras.horizontal - visible.horizontal) <= kTouchToleranceCm);
            CHECK(std::fabs(asTheCameras.vertical - visible.vertical) <= kTouchToleranceCm);
            CHECK(asTheDefault.horizontal / visible.horizontal == Catch::Approx(expectedRatio).epsilon(1.0e-5));
            CHECK(asTheDefault.vertical / visible.vertical == Catch::Approx(expectedRatio).epsilon(1.0e-5));
        }
    }
}

TEST_CASE("DAttackCamera.IsoFramingSeparatesTheHorizontalAndVerticalMargins", "[DAttack][CameraFraming]")
{
    const float wideOnly = dAttackCameraBehaviour::isoFramingDistance(
        1000.f, 0.f, 16.f / 9.f, 11.33f, FovAxis::Shorter, 400.f, 0.f, 0.f, 1.0e9f);
    const float wideMoreH = dAttackCameraBehaviour::isoFramingDistance(
        1000.f, 0.f, 16.f / 9.f, 11.33f, FovAxis::Shorter, 800.f, 0.f, 0.f, 1.0e9f);
    const float wideMoreV = dAttackCameraBehaviour::isoFramingDistance(
        1000.f, 0.f, 16.f / 9.f, 11.33f, FovAxis::Shorter, 400.f, 100.f, 0.f, 1.0e9f);
    INFO("wide " << wideOnly << " more H " << wideMoreH << " more V (not binding) " << wideMoreV);
    CHECK(wideMoreH > wideOnly);
    CHECK(wideMoreV == wideOnly);

    const float tallOnly = dAttackCameraBehaviour::isoFramingDistance(
        0.f, 1000.f, 16.f / 9.f, 11.33f, FovAxis::Shorter, 0.f, 300.f, 0.f, 1.0e9f);
    const float tallMoreV = dAttackCameraBehaviour::isoFramingDistance(
        0.f, 1000.f, 16.f / 9.f, 11.33f, FovAxis::Shorter, 0.f, 600.f, 0.f, 1.0e9f);
    INFO("tall " << tallOnly << " more V " << tallMoreV);
    CHECK(tallMoreV > tallOnly);
}

TEST_CASE("DAttackCamera.IsoFramingHonoursTheDistanceClamp", "[DAttack][CameraFraming]")
{
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
    {
        INFO(kAspectNames[a]);
        for (const FovAxis axis : { FovAxis::Horizontal, FovAxis::Shorter })
        {
            const float nearDistance = dAttackCameraBehaviour::isoFramingDistance(
                0.f, 0.f, kAspects[a], 11.33f, axis, 1.f, 1.f, 2000.f, 25000.f);
            const float farDistance = dAttackCameraBehaviour::isoFramingDistance(
                90000.f, 90000.f, kAspects[a], 11.33f, axis, 400.f, 300.f, 2000.f, 25000.f);
            const float inside = dAttackCameraBehaviour::isoFramingDistance(
                500.f, 300.f, kAspects[a], 11.33f, axis, 400.f, 300.f, 2000.f, 25000.f);
            const float unclamped = dAttackCameraBehaviour::isoFramingDistance(
                500.f, 300.f, kAspects[a], 11.33f, axis, 400.f, 300.f, 0.f, 1.0e9f);
            INFO("axis " << static_cast<int>(axis) << " near " << nearDistance << " far " << farDistance << " inside " << inside);
            CHECK(nearDistance == 2000.f);
            CHECK(farDistance == 25000.f);
            CHECK(inside == unclamped);
            CHECK(inside > 2000.f);
            CHECK(inside < 25000.f);
        }
    }
}

TEST_CASE("DAttackCamera.BoomLengthWithTodaysConstantsAsSettingsIsTodaysCurve", "[DAttack][CameraFraming]")
{
    OrbitCameraSettings todays;
    todays.boomLengthAtTargetPitch = 900.f;
    todays.boomLengthAtHorizon = 400.f;
    const float pitches[] = { 0.f, 1.f, 9.99f, 10.f, 17.1887f, 30.f, 45.f, 47.78f, 59.999f, 60.f, 74.4845f, 80.f, 85.f, -3.f };
    const float targets[] = { 10.f, 35.f, 45.f, 60.f, 80.f, 0.f, -30.f, 95.f };
    int compared = 0;
    int mismatched = 0;
    for (const float pitch : pitches)
        for (const float target : targets)
        {
            const float expected = todaysBoomLength(pitch, target, todays);
            const float actual = dAttackCameraBehaviour::boomLength(pitch, target, todays);
            ++compared;
            if (bitsOf(actual) != bitsOf(expected))
            {
                ++mismatched;
                UNSCOPED_INFO("pitch " << pitch << " target " << target << " today " << expected << " new " << actual);
            }
        }
    INFO("compared " << compared << " mismatched " << mismatched);
    CHECK(compared == 14 * 8);
    CHECK(mismatched == 0);

    const OrbitCameraSettings defaults;
    CHECK(defaults.boomLengthAtTargetPitch == 2500.f);
    CHECK(defaults.boomLengthAtHorizon == 550.f);
    CHECK(dAttackCameraBehaviour::boomLength(60.f, 60.f, defaults) == 2500.f);
    CHECK(dAttackCameraBehaviour::boomLength(0.f, 60.f, defaults) == 550.f);
}

#endif // WITH_LOW_LEVEL_TESTS
