// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraSlopeFrameTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace dAttackCameraSlopeFrameTest
{
using dAttackCameraBehaviour::OrbitCameraInput;
using dAttackCameraBehaviour::OrbitCameraSettings;
using dAttackCameraBehaviour::OrbitCameraState;
using dAttackCameraBehaviour::WorldView;

constexpr double kPi = 3.14159265358979323846;
constexpr float kDts[] = { 1.f / 30.f, 1.f / 60.f, 1.f / 144.f, 1.f / 240.f };
constexpr float kIsoTargetDeg = 60.f;
constexpr float kSlopeDeg = 30.f;

std::uint32_t bitsOf(float value)
{
    std::uint32_t bits = 0u;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

glm::vec3 upTiltedTowardMinusX(double degrees)
{
    const double radians = degrees * kPi / 180.0;
    return glm::vec3(static_cast<float>(-std::sin(radians)), 0.f, static_cast<float>(std::cos(radians)));
}

glm::vec3 upTiltedTowardMinusY(double degrees)
{
    const double radians = degrees * kPi / 180.0;
    return glm::vec3(0.f, static_cast<float>(-std::sin(radians)), static_cast<float>(std::cos(radians)));
}

OrbitCameraState stateAt(float yawDeg, float pitchDeg, const glm::vec3& frameUp)
{
    OrbitCameraState state;
    state.yawDeg = yawDeg;
    state.pitchDeg = pitchDeg;
    state.frameUp = frameUp;
    return state;
}

double angleDeg(const glm::vec3& a, const glm::vec3& b)
{
    return static_cast<double>(dAttackCameraBehaviour::angleBetweenDeg(a, b));
}

double yawGapDeg(double a, double b)
{
    return std::fabs(std::remainder(a - b, 360.0));
}

OrbitCameraState run(float dt, int frames, const OrbitCameraInput& input, const OrbitCameraSettings& settings,
    OrbitCameraState state)
{
    for (int frame = 0; frame < frames; ++frame)
        state = dAttackCameraBehaviour::integrate(dt, input, settings, state);
    return state;
}
} // namespace dAttackCameraSlopeFrameTest

using namespace dAttackCameraSlopeFrameTest;

TEST_CASE("DAttackCamera.SlopeFrameWorldUpIsTodaysView", "[DAttack][CameraSlopeFrame]")
{
    const OrbitCameraSettings defaults;
    const glm::vec3 upOneUlpShort(0.f, 0.f, std::nextafter(1.f, 0.f));
    long compared = 0;
    long mismatched = 0;
    for (const glm::vec3& up : { dAttackCameraBehaviour::kCameraUp, upOneUlpShort })
    {
        for (float yaw = -180.f; yaw <= 180.f; yaw += 7.5f)
        {
            for (float pitch = 1.f; pitch <= 89.f; pitch += 0.5f)
            {
                const WorldView view = dAttackCameraBehaviour::worldView(stateAt(yaw, pitch, up), defaults);
                ++compared;
                if (bitsOf(view.yawDeg) != bitsOf(yaw) || bitsOf(view.pitchDeg) != bitsOf(pitch))
                    ++mismatched;
            }
        }
    }
    INFO("compared " << compared << " mismatched " << mismatched);
    CHECK(compared == 2 * 49 * 177);
    CHECK(mismatched == 0);

    const glm::vec3 justTilted = upTiltedTowardMinusX(2.e-6 * 180.0 / kPi);
    REQUIRE_FALSE(dAttackCameraBehaviour::isCameraUpFlat(justTilted));
    double worstDeg = 0.0;
    for (float yaw = -180.f; yaw <= 180.f; yaw += 7.5f)
    {
        for (float pitch = 10.f; pitch <= 80.f; pitch += 0.5f)
        {
            const WorldView view = dAttackCameraBehaviour::worldView(stateAt(yaw, pitch, justTilted), defaults);
            worstDeg = std::max(worstDeg, yawGapDeg(view.yawDeg, yaw));
            worstDeg = std::max(worstDeg, std::fabs(static_cast<double>(view.pitchDeg) - pitch));
        }
    }
    INFO("worst gap just past the flat tolerance " << worstDeg << " deg");
    CHECK(worstDeg < 1.e-3);
}

TEST_CASE("DAttackCamera.SlopeFramePitchFollowsTheRiseAlongTheView", "[DAttack][CameraSlopeFrame]")
{
    const OrbitCameraSettings defaults;
    REQUIRE(defaults.slopePitchMinDeg == 35.f);
    REQUIRE(defaults.slopePitchMaxDeg == 75.f);
    OrbitCameraSettings unclamped = defaults;
    unclamped.slopePitchMinDeg = dAttackCameraBehaviour::kHardPitchMinDeg;
    unclamped.slopePitchMaxDeg = dAttackCameraBehaviour::kHardPitchMaxDeg;
    const OrbitCameraSettings* const bothLimits[] = { &defaults, &unclamped };

    struct Face
    {
        const char* name;
        glm::vec3 up;
        float upSlopeYawDeg;
        float contourYawDeg;
        float downSlopeYawDeg;
        float atDefaults[3];
        float atUnclamped[3];
    };
    const Face faces[] = {
        { "30 deg rising along +X", upTiltedTowardMinusX(kSlopeDeg), 0.f, 90.f, 180.f, { 35.f, 60.f, 75.f },
            { 30.f, 60.f, 89.f } },
        { "30 deg rising along +X, contour the other way", upTiltedTowardMinusX(kSlopeDeg), 0.f, -90.f, -180.f,
            { 35.f, 60.f, 75.f }, { 30.f, 60.f, 89.f } },
        { "30 deg rising along +Y", upTiltedTowardMinusY(kSlopeDeg), 90.f, 0.f, -90.f, { 35.f, 60.f, 75.f },
            { 30.f, 60.f, 89.f } },
        { "12 deg rising along +X", upTiltedTowardMinusX(12.0), 0.f, 90.f, 180.f, { 48.f, 60.f, 72.f },
            { 48.f, 60.f, 72.f } },
        { "G3 capture face normal (0, 0.866025448, 0.49999997)", glm::vec3(-0.49999997f, 0.f, 0.866025448f), 0.f, 90.f,
            180.f, { 35.f, 60.f, 75.f }, { 30.f, 60.f, 89.f } },
        { "G3 capture face normal (-1.49011612e-08, 0.866025388, 0.5)",
            glm::vec3(-0.5f, -1.49011612e-08f, 0.866025388f), 0.f, 90.f, 180.f, { 35.f, 60.f, 75.f },
            { 30.f, 60.f, 89.f } },
    };
    for (const Face& face : faces)
    {
        const float yaws[3] = { face.upSlopeYawDeg, face.contourYawDeg, face.downSlopeYawDeg };
        const char* const headings[3] = { "up-slope", "contour", "down-slope" };
        for (int heading = 0; heading < 3; ++heading)
        {
            const OrbitCameraState state = stateAt(yaws[heading], kIsoTargetDeg, face.up);
            const WorldView atDefaults = dAttackCameraBehaviour::worldView(state, defaults);
            const WorldView atUnclamped = dAttackCameraBehaviour::worldView(state, unclamped);
            INFO(face.name << " " << headings[heading] << ": pitch " << atDefaults.pitchDeg << " at the defaults, "
                << atUnclamped.pitchDeg << " unclamped");
            CHECK(atDefaults.pitchDeg == Catch::Approx(face.atDefaults[heading]).margin(1e-4f));
            CHECK(atUnclamped.pitchDeg == Catch::Approx(face.atUnclamped[heading]).margin(1e-4f));
            CHECK(bitsOf(atDefaults.yawDeg) == bitsOf(yaws[heading]));
            CHECK(bitsOf(atUnclamped.yawDeg) == bitsOf(yaws[heading]));
        }
    }

    long compared = 0;
    long yawMismatched = 0;
    for (const glm::vec3& up : { upTiltedTowardMinusX(kSlopeDeg), upTiltedTowardMinusY(kSlopeDeg),
             upTiltedTowardMinusX(12.0), upTiltedTowardMinusY(-25.0), upTiltedTowardMinusX(30.09) })
    {
        for (float yaw = -180.f; yaw <= 180.f; yaw += 2.5f)
        {
            for (float pitch = 1.f; pitch <= 89.f; pitch += 0.5f)
            {
                for (const OrbitCameraSettings* settings : bothLimits)
                {
                    const WorldView view = dAttackCameraBehaviour::worldView(stateAt(yaw, pitch, up), *settings);
                    ++compared;
                    if (bitsOf(view.yawDeg) != bitsOf(yaw))
                        ++yawMismatched;
                }
            }
        }
    }
    INFO("views compared " << compared << ", world yaw not the player's " << yawMismatched);
    CHECK(compared == 5 * 145 * 177 * 2);
    CHECK(yawMismatched == 0);
}

TEST_CASE("DAttackCamera.SlopeFrameYawIsThePlayersAndThePitchNeverJumps", "[DAttack][CameraSlopeFrame]")
{
    const glm::vec3 slope = upTiltedTowardMinusX(kSlopeDeg);
    const OrbitCameraSettings defaults;
    OrbitCameraSettings unclamped = defaults;
    unclamped.slopePitchMinDeg = dAttackCameraBehaviour::kHardPitchMinDeg;
    unclamped.slopePitchMaxDeg = dAttackCameraBehaviour::kHardPitchMaxDeg;
    constexpr float kYawStepDeg = 0.5f;
    constexpr int kSteps = 720;
    const double riseBoundDeg = kYawStepDeg * std::tan(kSlopeDeg * kPi / 180.0);
    const OrbitCameraSettings* const bothLimits[] = { &defaults, &unclamped };

    for (const OrbitCameraSettings* settings : bothLimits)
    {
        INFO("slope pitch limits " << settings->slopePitchMinDeg << " to " << settings->slopePitchMaxDeg);
        WorldView previous = dAttackCameraBehaviour::worldView(stateAt(-180.f, kIsoTargetDeg, slope), *settings);
        float previousYaw = -180.f;
        int yawStepMismatches = 0;
        double worstYawJumpDeg = 0.0;
        double worstPitchStepDeg = 0.0;
        float lowestPitch = previous.pitchDeg;
        float highestPitch = previous.pitchDeg;
        for (int step = 1; step <= kSteps; ++step)
        {
            const float yaw = -180.f + kYawStepDeg * static_cast<float>(step);
            const WorldView view = dAttackCameraBehaviour::worldView(stateAt(yaw, kIsoTargetDeg, slope), *settings);
            if (bitsOf(view.yawDeg - previous.yawDeg) != bitsOf(yaw - previousYaw))
                ++yawStepMismatches;
            worstYawJumpDeg = std::max(worstYawJumpDeg, yawGapDeg(view.yawDeg, previous.yawDeg));
            worstPitchStepDeg = std::max(worstPitchStepDeg, std::fabs(static_cast<double>(view.pitchDeg) - previous.pitchDeg));
            lowestPitch = std::min(lowestPitch, view.pitchDeg);
            highestPitch = std::max(highestPitch, view.pitchDeg);
            previous = view;
            previousYaw = yaw;
        }
        INFO("yaw steps not equal to the input step " << yawStepMismatches << ", largest yaw step " << worstYawJumpDeg
            << " deg, largest pitch step " << worstPitchStepDeg << " deg (bound " << riseBoundDeg << "), pitch "
            << lowestPitch << " to " << highestPitch);
        CHECK(yawStepMismatches == 0);
        CHECK(worstYawJumpDeg == Catch::Approx(kYawStepDeg).margin(1e-4));
        CHECK(worstPitchStepDeg <= riseBoundDeg + 1e-4);
        CHECK(worstPitchStepDeg > 0.5 * riseBoundDeg);
    }
}

TEST_CASE("DAttackCamera.SlopeFramePitchDoesNotPopWhenTheFrameLeavesWorldUp", "[DAttack][CameraSlopeFrame]")
{
    const OrbitCameraSettings defaults;
    constexpr int kEaseSteps = 100;
    for (const float pitch : { 20.f, 80.f })
    {
        for (const float yaw : { 0.f, 45.f, 90.f, 135.f, 180.f, -45.f, -90.f, -135.f })
        {
            float previous = dAttackCameraBehaviour::worldView(stateAt(yaw, pitch, dAttackCameraBehaviour::kCameraUp),
                defaults).pitchDeg;
            CHECK(bitsOf(previous) == bitsOf(pitch));
            double direction = 0.0;
            double worstStepDeg = 0.0;
            double worstReversalDeg = 0.0;
            for (int step = 1; step <= kEaseSteps; ++step)
            {
                const glm::vec3 up = upTiltedTowardMinusX(kSlopeDeg * static_cast<double>(step) / kEaseSteps);
                const float current = dAttackCameraBehaviour::worldView(stateAt(yaw, pitch, up), defaults).pitchDeg;
                const double change = static_cast<double>(current) - previous;
                worstStepDeg = std::max(worstStepDeg, std::fabs(change));
                if (direction == 0.0 && change != 0.0)
                    direction = change > 0.0 ? 1.0 : -1.0;
                else if (direction * change < 0.0)
                    worstReversalDeg = std::max(worstReversalDeg, std::fabs(change));
                previous = current;
            }
            INFO("pitch " << pitch << " yaw " << yaw << ": ends at " << previous << ", largest step " << worstStepDeg
                << " deg, largest reversal " << worstReversalDeg << " deg");
            CHECK(worstStepDeg <= 1.0);
            CHECK(worstReversalDeg <= 1e-5);
            if (yaw == 0.f)
                CHECK(previous == Catch::Approx(pitch == 20.f ? 20.f : 50.f).margin(1e-4f));
            if (yaw == 180.f)
                CHECK(previous == Catch::Approx(pitch == 20.f ? 50.f : 80.f).margin(1e-4f));
        }
    }
}

TEST_CASE("DAttackCamera.SlopeFramePitchLimitsAreOrdered", "[DAttack][CameraSlopeFrame]")
{
    const glm::vec3 slope = upTiltedTowardMinusX(kSlopeDeg);
    const OrbitCameraSettings defaults;
    const dAttackCameraBehaviour::PitchLimits atDefaults = dAttackCameraBehaviour::slopePitchLimits(defaults);
    CHECK(atDefaults.minDeg == 35.f);
    CHECK(atDefaults.maxDeg == 75.f);

    OrbitCameraSettings inverted = defaults;
    inverted.slopePitchMinDeg = 50.f;
    inverted.slopePitchMaxDeg = 40.f;
    const dAttackCameraBehaviour::PitchLimits ordered = dAttackCameraBehaviour::slopePitchLimits(inverted);
    CHECK(ordered.minDeg == 40.f);
    CHECK(ordered.maxDeg == 40.f);
    CHECK(dAttackCameraBehaviour::worldView(stateAt(0.f, kIsoTargetDeg, slope), inverted).pitchDeg == 40.f);
    CHECK(dAttackCameraBehaviour::worldView(stateAt(180.f, kIsoTargetDeg, slope), inverted).pitchDeg == kIsoTargetDeg);

    OrbitCameraSettings outside = defaults;
    outside.slopePitchMinDeg = -5.f;
    outside.slopePitchMaxDeg = 120.f;
    const dAttackCameraBehaviour::PitchLimits hard = dAttackCameraBehaviour::slopePitchLimits(outside);
    CHECK(hard.minDeg == dAttackCameraBehaviour::kHardPitchMinDeg);
    CHECK(hard.maxDeg == dAttackCameraBehaviour::kHardPitchMaxDeg);
    outside.slopePitchMinDeg = 0.5f;
    outside.slopePitchMaxDeg = 0.2f;
    const dAttackCameraBehaviour::PitchLimits low = dAttackCameraBehaviour::slopePitchLimits(outside);
    CHECK(low.minDeg == dAttackCameraBehaviour::kHardPitchMinDeg);
    CHECK(low.maxDeg == dAttackCameraBehaviour::kHardPitchMinDeg);

    OrbitCameraSettings tuned = defaults;
    tuned.slopePitchMinDeg = 45.f;
    tuned.slopePitchMaxDeg = 65.f;
    CHECK(dAttackCameraBehaviour::worldView(stateAt(0.f, kIsoTargetDeg, slope), tuned).pitchDeg == 45.f);
    CHECK(dAttackCameraBehaviour::worldView(stateAt(180.f, kIsoTargetDeg, slope), tuned).pitchDeg == 65.f);
}

TEST_CASE("DAttackCamera.SlopeFrameTargetFollowsTheGroundUpToThirtyPointOneDegrees", "[DAttack][CameraSlopeFrame]")
{
    CHECK(dAttackCameraBehaviour::kCameraFollowMaxSlopeDeg == 30.f);
    CHECK(dAttackCameraBehaviour::kCameraFollowSlopeEpsilonDeg == 0.1f);

    for (const double degrees : { 30.0, 30.0 - 1.32e-5, 30.0 + 6.4e-6, 30.09, 12.0, 1.e-3 })
    {
        INFO("slope " << degrees << " deg");
        const glm::vec3 n = upTiltedTowardMinusX(degrees);
        const glm::vec3 target = dAttackCameraBehaviour::cameraTargetUp(n, true);
        CHECK(target.x == Catch::Approx(n.x).margin(1e-6f));
        CHECK(target.y == 0.f);
        CHECK(target.z == Catch::Approx(n.z).margin(1e-6f));
        CHECK(glm::length(target) == Catch::Approx(1.f).margin(1e-6f));
        CHECK_FALSE(dAttackCameraBehaviour::isCameraUpFlat(target));
    }

    for (const double degrees : { 30.2, 30.11, 45.0, 60.0, 89.0 })
    {
        INFO("slope " << degrees << " deg");
        const glm::vec3 target = dAttackCameraBehaviour::cameraTargetUp(upTiltedTowardMinusY(degrees), true);
        CHECK(bitsOf(target.x) == bitsOf(0.f));
        CHECK(bitsOf(target.y) == bitsOf(0.f));
        CHECK(bitsOf(target.z) == bitsOf(1.f));
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (const glm::vec3& flat : { glm::vec3(0.f, 0.f, 1.f), glm::vec3(0.f, 0.f, std::nextafter(1.f, 0.f)),
             glm::vec3(5.e-7f, 0.f, 1.f), glm::vec3(0.f, 0.f, 0.f), glm::vec3(nan, 0.f, 1.f), glm::vec3(0.f, 0.f, -1.f) })
    {
        INFO("ground up (" << flat.x << ", " << flat.y << ", " << flat.z << ")");
        const glm::vec3 target = dAttackCameraBehaviour::cameraTargetUp(flat, true);
        CHECK(bitsOf(target.x) == bitsOf(0.f));
        CHECK(bitsOf(target.y) == bitsOf(0.f));
        CHECK(bitsOf(target.z) == bitsOf(1.f));
    }
}

TEST_CASE("DAttackCamera.SlopeFrameTargetIsWorldUpWhenUnsupported", "[DAttack][CameraSlopeFrame]")
{
    for (const double degrees : { 0.0, 12.0, 30.0, 45.0 })
    {
        INFO("slope " << degrees << " deg");
        const glm::vec3 target = dAttackCameraBehaviour::cameraTargetUp(upTiltedTowardMinusX(degrees), false);
        CHECK(bitsOf(target.x) == bitsOf(0.f));
        CHECK(bitsOf(target.y) == bitsOf(0.f));
        CHECK(bitsOf(target.z) == bitsOf(1.f));
    }
}

TEST_CASE("DAttackCamera.SlopeFrameLookModeMovesTheFrameUpOnlyWithTheYawDrivenPull", "[DAttack][CameraSlopeFrame]")
{
    const glm::vec3 slope = upTiltedTowardMinusX(kSlopeDeg);
    const OrbitCameraSettings defaults;

    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const OrbitCameraState start = stateAt(20.f, 30.f, dAttackCameraBehaviour::kCameraUp);

        OrbitCameraInput still;
        still.lookHeld = true;
        still.targetPitchDeg = kIsoTargetDeg;
        still.targetUp = slope;
        const OrbitCameraState idle = run(dt, 600, still, defaults, start);
        CHECK(bitsOf(idle.frameUp.x) == bitsOf(start.frameUp.x));
        CHECK(bitsOf(idle.frameUp.y) == bitsOf(start.frameUp.y));
        CHECK(bitsOf(idle.frameUp.z) == bitsOf(start.frameUp.z));
        CHECK(bitsOf(idle.pitchDeg) == bitsOf(start.pitchDeg));
        CHECK(bitsOf(idle.yawDeg) == bitsOf(start.yawDeg));

        OrbitCameraInput pitchOnly = still;
        pitchOnly.mouseUnits = glm::vec2(0.f, 0.1f);
        const OrbitCameraState tilted = run(dt, 20, pitchOnly, defaults, start);
        CHECK(bitsOf(tilted.frameUp.z) == bitsOf(start.frameUp.z));
        CHECK(tilted.pitchDeg != start.pitchDeg);

        OrbitCameraInput turning = still;
        turning.mouseUnits = glm::vec2(-0.2f, 0.f);
        const float yawPerFrameDeg = 0.2f * dAttackCameraBehaviour::kMouseDegreesPerUnit;
        constexpr int kTurnFrames = 120;
        OrbitCameraState state = start;
        double worstShareGap = 0.0;
        double worstGeodesicGap = 0.0;
        for (int frame = 1; frame <= kTurnFrames; ++frame)
        {
            state = dAttackCameraBehaviour::integrate(dt, turning, defaults, state);
            const double pitchShare = (kIsoTargetDeg - state.pitchDeg) / (kIsoTargetDeg - start.pitchDeg);
            const double upShare = angleDeg(state.frameUp, slope) / kSlopeDeg;
            worstShareGap = std::max(worstShareGap, std::fabs(upShare - pitchShare));
            worstGeodesicGap = std::max(worstGeodesicGap, std::fabs(
                angleDeg(state.frameUp, start.frameUp)
                + angleDeg(state.frameUp, slope) - kSlopeDeg));
        }
        const double expectedShare = std::exp(-kTurnFrames * yawPerFrameDeg / dAttackCameraBehaviour::kPullYawDegreesPerEFold);
        const double upShare = angleDeg(state.frameUp, slope) / kSlopeDeg;
        INFO("after " << kTurnFrames * yawPerFrameDeg << " deg of yaw: frame-up share " << upShare << ", expected "
            << expectedShare << ", worst share gap to the pitch " << worstShareGap << ", worst off-geodesic "
            << worstGeodesicGap << " deg");
        CHECK(upShare == Catch::Approx(expectedShare).margin(1e-3));
        CHECK(worstShareGap < 1e-4);
        CHECK(worstGeodesicGap < 1e-3);
        CHECK(glm::length(state.frameUp) == Catch::Approx(1.f).margin(1e-6f));

        OrbitCameraSettings dominance = defaults;
        dominance.dominanceRatio = 2.f;
        OrbitCameraInput diagonal = still;
        diagonal.mouseUnits = glm::vec2(-0.2f, 0.2f);
        const OrbitCameraState noWeight = run(dt, 20, diagonal, dominance, start);
        CHECK(bitsOf(noWeight.frameUp.z) == bitsOf(start.frameUp.z));

        OrbitCameraSettings holdoff = defaults;
        holdoff.verticalHoldoffSeconds = 0.6f;
        OrbitCameraInput stickUp = still;
        stickUp.stick = glm::vec2(0.f, 1.f);
        const OrbitCameraState afterTilt = run(dt, 1, stickUp, holdoff, start);
        REQUIRE(afterTilt.holdoffSeconds == 0.6f);
        const int holdFrames = static_cast<int>(0.5f / dt);
        const OrbitCameraState duringHoldoff = run(dt, holdFrames, turning, holdoff, afterTilt);
        CHECK(bitsOf(duringHoldoff.frameUp.z) == bitsOf(start.frameUp.z));
        const OrbitCameraState afterHoldoff = run(dt, holdFrames, turning, holdoff, duringHoldoff);
        CHECK(afterHoldoff.frameUp.z < start.frameUp.z);
    }
}

TEST_CASE("DAttackCamera.SlopeFrameLookModeOffReturnsToWorldUpAtTheIsoPitch", "[DAttack][CameraSlopeFrame]")
{
    const OrbitCameraSettings defaults;
    REQUIRE(defaults.returnDegPerSec == 90.f);
    const glm::vec3 slope = upTiltedTowardMinusX(kSlopeDeg);

    struct Start
    {
        float pitchDeg;
        glm::vec3 frameUp;
        float angleDeg;
    };
    const Start starts[] = {
        { 25.f, slope, 35.f },
        { 80.f, upTiltedTowardMinusY(-12.0), 20.f },
        { 55.f, slope, 30.f },
        { 60.f, dAttackCameraBehaviour::kCameraUp, 0.f },
    };

    for (const float dt : kDts)
    {
        for (const Start& s : starts)
        {
            INFO("fps " << 1.f / dt << " start pitch " << s.pitchDeg);
            const OrbitCameraState start = stateAt(33.f, s.pitchDeg, s.frameUp);
            OrbitCameraInput off;
            off.lookHeld = false;
            off.mouseUnits = glm::vec2(40.f, -30.f);
            off.stick = glm::vec2(1.f, 1.f);
            off.targetPitchDeg = kIsoTargetDeg;
            off.targetUp = upTiltedTowardMinusX(12.0);

            const float seconds = s.angleDeg / defaults.returnDegPerSec;
            OrbitCameraState state = start;
            int reachedFrame = (state.pitchDeg == kIsoTargetDeg && state.frameUp == dAttackCameraBehaviour::kCameraUp) ? 0 : -1;
            double worstRateGapDeg = 0.0;
            for (int frame = 1; frame <= 1000 && reachedFrame < 0; ++frame)
            {
                const OrbitCameraState next = dAttackCameraBehaviour::integrate(dt, off, defaults, state);
                const double moved = std::max(angleDeg(next.frameUp, state.frameUp),
                    std::fabs(static_cast<double>(next.pitchDeg) - state.pitchDeg));
                worstRateGapDeg = std::max(worstRateGapDeg, moved - defaults.returnDegPerSec * dt);
                CHECK(bitsOf(next.yawDeg) == bitsOf(start.yawDeg));
                state = next;
                if (state.pitchDeg == kIsoTargetDeg && state.frameUp == dAttackCameraBehaviour::kCameraUp)
                    reachedFrame = frame;
            }
            const int framesAtTheRate = static_cast<int>(std::ceil(seconds / dt - 1e-3f));
            INFO("reached after " << reachedFrame << " frames, " << framesAtTheRate << " at the rate ("
                << seconds << " s), worst step over the rate " << worstRateGapDeg << " deg");
            CHECK(reachedFrame >= framesAtTheRate);
            CHECK(reachedFrame <= framesAtTheRate + 1);
            CHECK(worstRateGapDeg < 1e-3);
            CHECK(bitsOf(state.pitchDeg) == bitsOf(kIsoTargetDeg));
            CHECK(bitsOf(state.frameUp.x) == bitsOf(0.f));
            CHECK(bitsOf(state.frameUp.y) == bitsOf(0.f));
            CHECK(bitsOf(state.frameUp.z) == bitsOf(1.f));
            CHECK(bitsOf(state.yawDeg) == bitsOf(start.yawDeg));

            const WorldView view = dAttackCameraBehaviour::worldView(state, defaults);
            CHECK(bitsOf(view.pitchDeg) == bitsOf(kIsoTargetDeg));
            CHECK(bitsOf(view.yawDeg) == bitsOf(start.yawDeg));

            const OrbitCameraState settled = run(dt, 60, off, defaults, state);
            CHECK(bitsOf(settled.pitchDeg) == bitsOf(kIsoTargetDeg));
            CHECK(bitsOf(settled.frameUp.z) == bitsOf(1.f));
        }

        OrbitCameraSettings frozen = defaults;
        frozen.returnDegPerSec = 0.f;
        OrbitCameraInput off;
        off.targetPitchDeg = kIsoTargetDeg;
        const OrbitCameraState start = stateAt(33.f, 25.f, slope);
        const OrbitCameraState held = run(dt, 120, off, frozen, start);
        CHECK(bitsOf(held.pitchDeg) == bitsOf(start.pitchDeg));
        CHECK(bitsOf(held.frameUp.x) == bitsOf(start.frameUp.x));
        CHECK(bitsOf(held.frameUp.z) == bitsOf(start.frameUp.z));
        frozen.returnDegPerSec = -40.f;
        const OrbitCameraState heldNegative = run(dt, 120, off, frozen, start);
        CHECK(bitsOf(heldNegative.pitchDeg) == bitsOf(start.pitchDeg));
        CHECK(bitsOf(heldNegative.frameUp.z) == bitsOf(start.frameUp.z));
    }
}

#endif // WITH_LOW_LEVEL_TESTS
