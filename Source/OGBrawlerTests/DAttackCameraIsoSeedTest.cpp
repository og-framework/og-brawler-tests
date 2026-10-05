// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraIsoSeedTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"

#include "glm/vec2.hpp"

#include <cmath>
#include <vector>

#if __has_include("Math/Rotator.h")
#include "CoreMinimal.h"
#define OG_ISO_SEED_TEST_HAS_UNREAL_MATH 1
#else
#define OG_ISO_SEED_TEST_HAS_UNREAL_MATH 0
#endif

namespace dAttackCameraIsoSeedTest
{
using dAttackCameraBehaviour::OrbitCameraInput;
using dAttackCameraBehaviour::OrbitCameraSettings;
using dAttackCameraBehaviour::OrbitCameraState;

constexpr float kDts[] = { 1.f / 30.f, 1.f / 60.f, 1.f / 100.f, 1.f / 144.f, 1.f / 240.f };
constexpr float kIsoPitchDegrees = -60.f;
constexpr float kLegacyTargetPitchDegrees = 45.8366f;
constexpr float kFullBoomLength = 900.f;
constexpr float kLookSeconds = 5.25f;

struct LookRun
{
    std::vector<float> pitches;
    std::vector<float> lengths;
    OrbitCameraState final;
};

LookRun runHorizontalLook(float dt, OrbitCameraState state, float targetDeg, float seconds)
{
    const OrbitCameraSettings settings;
    const OrbitCameraInput input{ true, glm::vec2(0.f), glm::vec2(1.f, 0.f), targetDeg };
    LookRun run;
    const long frames = std::lround(seconds / dt);
    for (long frame = 0; frame < frames; ++frame)
    {
        state = dAttackCameraBehaviour::integrate(dt, input, settings, state);
        run.pitches.push_back(state.pitchDeg);
        run.lengths.push_back(dAttackCameraBehaviour::boomLength(state.pitchDeg, targetDeg, settings));
    }
    run.final = state;
    return run;
}

void requireHoldsTargetAtFullLength(const OrbitCameraState& seed, float targetDeg)
{
    for (const float dt : kDts)
    {
        const LookRun run = runHorizontalLook(dt, seed, targetDeg, kLookSeconds);
        float worstPitchError = 0.f;
        float shortestLength = kFullBoomLength;
        for (std::size_t frame = 0; frame < run.pitches.size(); ++frame)
        {
            worstPitchError = std::fmax(worstPitchError, std::fabs(run.pitches[frame] - targetDeg));
            shortestLength = std::fmin(shortestLength, run.lengths[frame]);
        }
        INFO("fps " << 1.f / dt << " target " << targetDeg << " worst pitch error " << worstPitchError
                    << " shortest length " << shortestLength << " yaw " << seed.yawDeg << " -> " << run.final.yawDeg);
        REQUIRE(worstPitchError == 0.f);
        REQUIRE(shortestLength == kFullBoomLength);
        REQUIRE(std::fabs(std::remainder(run.final.yawDeg - seed.yawDeg, 360.f)) > 1.f);
    }
}
} // namespace dAttackCameraIsoSeedTest

using namespace dAttackCameraIsoSeedTest;

TEST_CASE("DAttackCamera.IsoTargetPitchStaysInsideThePitchLimits", "[DAttack][CameraIsoSeed]")
{
    const OrbitCameraSettings settings;
    CHECK(dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(kIsoPitchDegrees, settings) == 60.f);
    CHECK(dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(-45.f, settings) == 45.f);

    const float inputs[] = { 30.f, 0.f, -0.5f, -1.f, -60.f, -89.f, -90.f, -120.f, -180.f, 400.f, -400.f };
    for (const float uePitch : inputs)
    {
        const float pitch = dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(uePitch, settings);
        INFO("ue pitch " << uePitch << " -> " << pitch << " deg below the horizon");
        CHECK(pitch >= dAttackCameraBehaviour::kPitchMinDeg);
        CHECK(pitch <= dAttackCameraBehaviour::kPitchMaxDeg);
    }
    CHECK(dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(30.f, settings) == dAttackCameraBehaviour::kPitchMinDeg);
    CHECK(dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(-90.f, settings) == dAttackCameraBehaviour::kPitchMaxDeg);

    OrbitCameraSettings narrow;
    narrow.pitchMinDeg = 30.f;
    narrow.pitchMaxDeg = 50.f;
    CHECK(dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(kIsoPitchDegrees, narrow) == 50.f);
    CHECK(dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(-10.f, narrow) == 30.f);
}

TEST_CASE("DAttackCamera.IsoSeedHoldsPitchAndFullLengthUnderHorizontalLook", "[DAttack][CameraIsoSeed]")
{
    const OrbitCameraSettings settings;
    const float target = dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(kIsoPitchDegrees, settings);
    const float isoYaws[] = { 0.f, 45.f, -137.5f, 180.f, 400.f };
    for (const float isoYaw : isoYaws)
    {
        INFO("iso yaw " << isoYaw);
        const OrbitCameraState seed = dAttackCameraBehaviour::seedFromUERotation(kIsoPitchDegrees, isoYaw, settings);
        CHECK(seed.pitchDeg == target);
        CHECK(seed.holdoffSeconds == 0.f);
        CHECK(std::fabs(std::remainder(seed.yawDeg - isoYaw, 360.f)) < 1e-4f);
        CHECK(std::fabs(seed.yawDeg) <= 180.f);
        CHECK(dAttackCameraBehaviour::boomLength(seed.pitchDeg, target, settings) == kFullBoomLength);
        requireHoldsTargetAtFullLength(seed, target);
    }
}

TEST_CASE("DAttackCamera.OffTargetSeedConvergesToTheIsoPitch", "[DAttack][CameraIsoSeed]")
{
    const OrbitCameraSettings settings;
    const float target = dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(kIsoPitchDegrees, settings);
    const float startPitches[] = { 17.1887f, 74.4845f };
    for (const float dt : kDts)
    {
        for (const float start : startPitches)
        {
            const LookRun run = runHorizontalLook(dt, OrbitCameraState{ 0.f, start, 0.f }, target, 4.f);
            float furthestPast = 0.f;
            for (const float pitch : run.pitches)
                furthestPast = std::fmax(furthestPast, (pitch - target) * (start < target ? 1.f : -1.f));
            INFO("fps " << 1.f / dt << " start " << start << " final " << run.final.pitchDeg << " target " << target);
            CHECK(run.final.pitchDeg == Catch::Approx(target).margin(0.05f));
            CHECK(std::fabs(run.final.pitchDeg - kLegacyTargetPitchDegrees) > 10.f);
            CHECK(furthestPast <= 0.f);
            CHECK(run.lengths.back() == Catch::Approx(kFullBoomLength).margin(5.f));
        }
    }
}

#if OG_ISO_SEED_TEST_HAS_UNREAL_MATH
TEST_CASE("DAttackCamera.UnrealIsoRotatorSeedReadsAsTheTargetPitch", "[DAttack][CameraIsoSeed]")
{
    const OrbitCameraSettings settings;
    const float target = dAttackCameraBehaviour::pitchDegFromUEPitchDegrees(kIsoPitchDegrees, settings);
    const double isoYaws[] = { 0.0, 45.0, -137.5, 180.0 };
    for (const double isoYaw : isoYaws)
    {
        const FRotator iso(kIsoPitchDegrees, isoYaw, 0.0);
        const OrbitCameraState seed = dAttackCameraBehaviour::seedFromUERotation(
            static_cast<float>(iso.Pitch), static_cast<float>(iso.Yaw), settings);
        const FRotator boom(-seed.pitchDeg, seed.yawDeg, 0.f);
        const FVector isoForward = iso.Vector();
        const FVector boomForward = boom.Vector();
        INFO("iso yaw " << isoYaw << " seed (" << seed.yawDeg << ", " << seed.pitchDeg << ") iso forward ("
                        << isoForward.X << ", " << isoForward.Y << ", " << isoForward.Z << ") boom forward ("
                        << boomForward.X << ", " << boomForward.Y << ", " << boomForward.Z << ")");
        CHECK(seed.pitchDeg == target);
        CHECK(boomForward.Equals(isoForward, 1e-5));
        CHECK(boom.Quaternion().Equals(iso.Quaternion(), 1e-5));
        CHECK(boomForward.Z == Catch::Approx(-std::sqrt(3.0) / 2.0).margin(1e-5));
        requireHoldsTargetAtFullLength(seed, target);
    }
}
#endif

#endif // WITH_LOW_LEVEL_TESTS
