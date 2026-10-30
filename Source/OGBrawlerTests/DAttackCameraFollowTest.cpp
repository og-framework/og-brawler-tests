// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraFollowTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>

#if __has_include("Math/UnrealMathUtility.h")
#include "CoreMinimal.h"
#define OG_CAMERA_FOLLOW_TEST_HAS_UNREAL_MATH 1
#else
#define OG_CAMERA_FOLLOW_TEST_HAS_UNREAL_MATH 0
#endif

namespace dAttackCameraFollowTest
{
constexpr float kActorStartCm = 100.f;
constexpr float kActorStopCm = 10.f;
constexpr float kActorLerpSpeed = 6.f;
constexpr double kReportedSpeedCmPerSecond = 570.0;
constexpr float kDeltaSeconds[] = { 1.f / 30.f, 1.f / 60.f, 1.f / 120.f, 1.f / 144.f };

std::uint64_t bitsOf(double value)
{
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

bool sameBits(const glm::dvec3& a, const glm::dvec3& b)
{
    return bitsOf(a.x) == bitsOf(b.x) && bitsOf(a.y) == bitsOf(b.y) && bitsOf(a.z) == bitsOf(b.z);
}

glm::dvec3 todaysInterpTo(const glm::dvec3& current, const glm::dvec3& target, float deltaSeconds, float interpSpeed)
{
#if OG_CAMERA_FOLLOW_TEST_HAS_UNREAL_MATH
    const FVector result = FMath::VInterpTo(
        FVector(current.x, current.y, current.z), FVector(target.x, target.y, target.z), deltaSeconds, interpSpeed);
    return glm::dvec3(result.X, result.Y, result.Z);
#else
    if (interpSpeed <= 0.f)
        return target;
    const glm::dvec3 dist = target - current;
    if (dist.x * dist.x + dist.y * dist.y + dist.z * dist.z < 1.e-4f)
        return target;
    const float scaled = deltaSeconds * interpSpeed;
    const float atMostOne = (scaled < 1.f) ? scaled : 1.f;
    const float fraction = (0.f < atMostOne) ? atMostOne : 0.f;
    return current + glm::dvec3(dist.x * fraction, dist.y * fraction, dist.z * fraction);
#endif
}

glm::dvec3 todaysActorStep(const glm::dvec3& current, const glm::dvec3& target, float deltaSeconds, float lerpSpeed, float deadzone)
{
#if OG_CAMERA_FOLLOW_TEST_HAS_UNREAL_MATH
    const FVector currentLocation(current.x, current.y, current.z);
    const FVector targetLocation(target.x, target.y, target.z);
    if (FVector::Dist(currentLocation, targetLocation) > deadzone)
    {
        const FVector moved = FMath::VInterpTo(currentLocation, targetLocation, deltaSeconds, lerpSpeed);
        return glm::dvec3(moved.X, moved.Y, moved.Z);
    }
    return current;
#else
    const double dx = target.x - current.x;
    const double dy = target.y - current.y;
    const double dz = target.z - current.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > deadzone)
        return todaysInterpTo(current, target, deltaSeconds, lerpSpeed);
    return current;
#endif
}

struct ConstantSpeedRun
{
    int ticks = 0;
    int firstStartTick = -1;
    int holdTicksAfterStart = 0;
    int holdTicksInTheLastSecond = 0;
    int nonMonotoneTicks = 0;
    int overshootTicks = 0;
    int offAxisTicks = 0;
    double largestError = 0.0;
};

ConstantSpeedRun followConstantSpeed(float deltaSeconds, float startCm, float stopCm)
{
    ConstantSpeedRun run;
    glm::dvec3 camera(0.0);
    bool tracking = false;
    const int ticks = static_cast<int>(std::lround(3.0 / static_cast<double>(deltaSeconds)));
    const int lastSecondStart = ticks - static_cast<int>(std::lround(1.0 / static_cast<double>(deltaSeconds)));
    for (int tick = 1; tick <= ticks; ++tick)
    {
        const glm::dvec3 target(kReportedSpeedCmPerSecond * static_cast<double>(deltaSeconds) * tick, 0.0, 0.0);
        const dAttackCameraBehaviour::FollowStep step = dAttackCameraBehaviour::followStep(
            camera, target, tracking, deltaSeconds, kActorLerpSpeed, startCm, stopCm);
        if (run.firstStartTick < 0 && step.tracking)
            run.firstStartTick = tick;
        if (run.firstStartTick >= 0)
        {
            if (!step.tracking || !(step.position.x > camera.x))
            {
                ++run.holdTicksAfterStart;
                if (tick > lastSecondStart)
                    ++run.holdTicksInTheLastSecond;
            }
            if (step.position.x < camera.x)
                ++run.nonMonotoneTicks;
            if (step.position.x > target.x)
                ++run.overshootTicks;
            if (step.position.y != 0.0 || step.position.z != 0.0)
                ++run.offAxisTicks;
        }
        run.largestError = std::fmax(run.largestError, target.x - step.position.x);
        camera = step.position;
        tracking = step.tracking;
        run.ticks = tick;
    }
    return run;
}
} // namespace dAttackCameraFollowTest

using namespace dAttackCameraFollowTest;

TEST_CASE("DAttackCamera.FollowInterpIsTheEngineVInterpTo", "[DAttack][CameraFollow]")
{
    const glm::dvec3 currents[] = { glm::dvec3(0.0), glm::dvec3(1250.5, -380.25, 2400.125), glm::dvec3(-1.0e6, 3.0e5, 7.5) };
    const glm::dvec3 offsets[] = { glm::dvec3(0.0), glm::dvec3(0.004, 0.0, 0.0), glm::dvec3(0.006, 0.007, 0.0),
        glm::dvec3(95.0, 0.0, 0.0), glm::dvec3(-33.3, 812.7, -0.1), glm::dvec3(2.0e4, -1.5e4, 9.0e3) };
    const float deltas[] = { 1.f / 30.f, 1.f / 60.f, 1.f / 144.f, 0.f, -0.01f, 0.5f };
    const float speeds[] = { 6.f, 1.f, 0.f, -2.f, 200.f };
    int compared = 0;
    int mismatched = 0;
    for (const glm::dvec3& current : currents)
        for (const glm::dvec3& offset : offsets)
            for (const float delta : deltas)
                for (const float speed : speeds)
                {
                    const glm::dvec3 target = current + offset;
                    const glm::dvec3 expected = todaysInterpTo(current, target, delta, speed);
                    const glm::dvec3 actual = dAttackCameraBehaviour::interpTo(current, target, delta, speed);
                    ++compared;
                    if (!sameBits(actual, expected))
                    {
                        ++mismatched;
                        UNSCOPED_INFO("offset (" << offset.x << ", " << offset.y << ", " << offset.z << ") dt " << delta
                            << " speed " << speed << " -> engine " << expected.x << " new " << actual.x);
                    }
                }
    INFO("compared " << compared << " mismatched " << mismatched << " engine " << OG_CAMERA_FOLLOW_TEST_HAS_UNREAL_MATH);
    CHECK(compared == 3 * 6 * 6 * 5);
    CHECK(mismatched == 0);
}

TEST_CASE("DAttackCamera.FollowWithTheStopAtTheStartIsTodaysActorRule", "[DAttack][CameraFollow]")
{
    const float stops[] = { kActorStartCm, 5000.f };
    for (const float stop : stops)
    {
        for (const float delta : kDeltaSeconds)
        {
            glm::dvec3 today(10.0, -20.0, 30.0);
            glm::dvec3 camera = today;
            bool tracking = false;
            int mismatched = 0;
            int moved = 0;
            int held = 0;
            for (int tick = 1; tick <= 900; ++tick)
            {
                const double seconds = static_cast<double>(delta) * tick;
                const double speed = (tick % 300 < 150) ? kReportedSpeedCmPerSecond : 120.0;
                glm::dvec3 target(10.0 + speed * seconds, -20.0 + 40.0 * std::sin(seconds * 3.0), 30.0 - 15.0 * seconds);
                if (tick % 200 == 0)
                    target.y += 400.0;
                const glm::dvec3 nextToday = todaysActorStep(today, target, delta, kActorLerpSpeed, kActorStartCm);
                const dAttackCameraBehaviour::FollowStep step = dAttackCameraBehaviour::followStep(
                    camera, target, tracking, delta, kActorLerpSpeed, kActorStartCm, stop);
                if (!sameBits(step.position, nextToday))
                    ++mismatched;
                if (sameBits(nextToday, today))
                    ++held;
                else
                    ++moved;
                today = nextToday;
                camera = step.position;
                tracking = step.tracking;
            }
            INFO("stop " << stop << " dt " << delta << " moved " << moved << " held " << held);
            CHECK(mismatched == 0);
            CHECK(moved > 0);
            CHECK(held > 0);
        }
    }
}

TEST_CASE("DAttackCamera.FollowTracksAConstantSpeedTargetWithoutHoldTicks", "[DAttack][CameraFollow]")
{
    for (const float delta : kDeltaSeconds)
    {
        const ConstantSpeedRun run = followConstantSpeed(delta, kActorStartCm, kActorStopCm);
        INFO("dt " << delta << " ticks " << run.ticks << " first start " << run.firstStartTick << " holds after start "
            << run.holdTicksAfterStart << " largest error " << run.largestError);
        REQUIRE(run.firstStartTick > 1);
        CHECK(run.holdTicksAfterStart == 0);
        CHECK(run.nonMonotoneTicks == 0);
        CHECK(run.overshootTicks == 0);
        CHECK(run.offAxisTicks == 0);
        CHECK(run.largestError < 2.0 * kActorStartCm);
    }
}

TEST_CASE("DAttackCamera.FollowWithOneDeadzoneStopsAndGoesOnAConstantSpeedTarget", "[DAttack][CameraFollow]")
{
    for (const float delta : kDeltaSeconds)
    {
        const ConstantSpeedRun run = followConstantSpeed(delta, kActorStartCm, kActorStartCm);
        INFO("dt " << delta << " ticks " << run.ticks << " first start " << run.firstStartTick << " holds after start "
            << run.holdTicksAfterStart << " in the last second " << run.holdTicksInTheLastSecond);
        REQUIRE(run.firstStartTick > 1);
        CHECK(run.holdTicksAfterStart > 0);
        CHECK(run.holdTicksInTheLastSecond > 0);
    }
}

TEST_CASE("DAttackCamera.FollowNeverStartsOnAJitteringStationaryTarget", "[DAttack][CameraFollow]")
{
    const glm::dvec3 rest(1250.0, -800.0, 3400.0);
    const double diagonal = 40.0 / std::sqrt(3.0);
    const glm::dvec3 jitters[] = { glm::dvec3(40.0, 0.0, 0.0), glm::dvec3(0.0, 40.0, 0.0), glm::dvec3(0.0, 0.0, 40.0),
        glm::dvec3(diagonal, -diagonal, diagonal) };
    for (const float delta : kDeltaSeconds)
    {
        for (const glm::dvec3& jitter : jitters)
        {
            glm::dvec3 camera = rest;
            bool tracking = false;
            int started = 0;
            for (int tick = 1; tick <= 600; ++tick)
            {
                const double side = (tick % 2 == 0) ? 1.0 : -1.0;
                const double scale = 0.25 + 0.75 * static_cast<double>((tick * 7) % 4) / 3.0;
                const glm::dvec3 target = rest + jitter * (side * scale);
                const dAttackCameraBehaviour::FollowStep step = dAttackCameraBehaviour::followStep(
                    camera, target, tracking, delta, kActorLerpSpeed, kActorStartCm, kActorStopCm);
                if (step.tracking || !sameBits(step.position, rest))
                    ++started;
                camera = step.position;
                tracking = step.tracking;
            }
            INFO("dt " << delta << " jitter (" << jitter.x << ", " << jitter.y << ", " << jitter.z << ")");
            CHECK(started == 0);
        }
    }
}

TEST_CASE("DAttackCamera.FollowChasesAJumpToTheStopThresholdThenHolds", "[DAttack][CameraFollow]")
{
    constexpr float kDelta = 1.f / 60.f;
    const glm::dvec3 start(0.0, 0.0, 0.0);
    const glm::dvec3 jumped(200.0, 0.0, 0.0);
    glm::dvec3 camera = start;
    bool tracking = false;
    int movingTicks = 0;
    int nonShrinkingTicks = 0;
    int tick = 0;
    double previousError = 200.0;
    for (; tick < 600; ++tick)
    {
        const dAttackCameraBehaviour::FollowStep step = dAttackCameraBehaviour::followStep(
            camera, jumped, tracking, kDelta, kActorLerpSpeed, kActorStartCm, kActorStopCm);
        camera = step.position;
        tracking = step.tracking;
        if (!tracking)
            break;
        ++movingTicks;
        const double error = jumped.x - camera.x;
        if (!(error < previousError) || error < 0.0)
            ++nonShrinkingTicks;
        previousError = error;
    }
    const double heldError = jumped.x - camera.x;
    INFO("moving ticks " << movingTicks << " held error " << heldError);
    CHECK(movingTicks == 29);
    CHECK(nonShrinkingTicks == 0);
    CHECK(heldError <= kActorStopCm);
    CHECK(heldError > kActorStopCm * 0.9 * 0.9);
    CHECK(camera.y == 0.0);
    CHECK(camera.z == 0.0);

    const glm::dvec3 held = camera;
    int movedWhileHeld = 0;
    for (int later = 0; later < 300; ++later)
    {
        const dAttackCameraBehaviour::FollowStep step = dAttackCameraBehaviour::followStep(
            camera, jumped, tracking, kDelta, kActorLerpSpeed, kActorStartCm, kActorStopCm);
        if (step.tracking || !sameBits(step.position, held))
            ++movedWhileHeld;
        camera = step.position;
        tracking = step.tracking;
    }
    CHECK(movedWhileHeld == 0);

    const glm::dvec3 nudged = jumped + glm::dvec3(0.0, 80.0, 0.0);
    const dAttackCameraBehaviour::FollowStep small = dAttackCameraBehaviour::followStep(
        camera, nudged, tracking, kDelta, kActorLerpSpeed, kActorStartCm, kActorStopCm);
    CHECK_FALSE(small.tracking);
    CHECK(sameBits(small.position, held));

    const glm::dvec3 pushed = jumped + glm::dvec3(0.0, 150.0, 0.0);
    const dAttackCameraBehaviour::FollowStep restarted = dAttackCameraBehaviour::followStep(
        camera, pushed, tracking, kDelta, kActorLerpSpeed, kActorStartCm, kActorStopCm);
    CHECK(restarted.tracking);
    CHECK(restarted.position.y > held.y);
}

#endif // WITH_LOW_LEVEL_TESTS
