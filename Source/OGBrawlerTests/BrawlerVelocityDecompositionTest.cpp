// SPDX-License-Identifier: BUSL-1.1
// docs/BrawlerVelocityDecompositionTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/CompilerControl.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SurfaceFrame.h"

#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"

namespace brawlerVelocityDecompositionTests
{
namespace movement = brawlerMovementSimulation;

OGSIM_OPTIMIZE_OFF
movement::VelocityChannels legacyDecompose(const glm::vec3& velocity,
                                           const glm::vec3& u,
                                           const glm::vec3& v,
                                           const glm::vec3& up)
{
    const float     b      = glm::dot(velocity, v);
    const glm::vec3 planar = velocity - b * v;
    const float     s      = glm::dot(u, up);
    const float a = (glm::dot(planar, u) - glm::dot(planar, up) * s) / (1.f - s * s);
    const float c =  glm::dot(planar, up) - a * s;
    return movement::VelocityChannels{ glm::vec2(a, b), c };
}
OGSIM_OPTIMIZE_ON

struct Frame
{
    const char* name;
    glm::vec3   n;
    glm::vec3   u;
    glm::vec3   v;
};

inline Frame frameFor(const char* name, const glm::vec3& n)
{
    Frame frame{ name, n, glm::vec3(simFrame::kForward), glm::vec3(simFrame::kRight) };
    movement::buildTangentFrame(n, frame.u, frame.v);
    return frame;
}

inline glm::vec3 horizontal(float azimuthDegrees)
{
    const float a = glm::radians(azimuthDegrees);
    return simFrame::kForward * glm::cos(a) + simFrame::kLeft * glm::sin(a);
}

inline glm::vec3 tiltedAbout(float axisAzimuthDegrees, float tiltDegrees)
{
    const float t = glm::radians(tiltDegrees);
    const glm::vec3 towards = glm::cross(horizontal(axisAzimuthDegrees), movement::kWorldUp);
    return movement::kWorldUp * glm::cos(t) + towards * glm::sin(t);
}

inline glm::vec3 alongX(float tiltDegrees, float sign)
{
    const float t = glm::radians(tiltDegrees);
    return simFrame::kRight * (sign * glm::sin(t)) + movement::kWorldUp * glm::cos(t);
}

inline glm::vec3 alongZ(float tiltDegrees, float sign)
{
    const float t = glm::radians(tiltDegrees);
    return simFrame::kForward * (sign * glm::sin(t)) + movement::kWorldUp * glm::cos(t);
}

inline std::vector<Frame> generalFrames()
{
    std::vector<Frame> frames{
        frameFor("+X 30", alongX(30.f, 1.f)),
        frameFor("-X 30", alongX(30.f, -1.f)),
    };
    static const char* const kNames[] = {
        "20 about 30", "40 about 30", "20 about 45", "40 about 45",
        "20 about 60", "40 about 60", "20 about 120", "40 about 120",
    };
    int i = 0;
    for (const float azimuth : { 30.f, 45.f, 60.f, 120.f })
        for (const float tilt : { 20.f, 40.f })
            frames.push_back(frameFor(kNames[i++], tiltedAbout(azimuth, tilt)));
    return frames;
}

inline std::vector<Frame> fastPathFrames()
{
    return {
        frameFor("flat", movement::kWorldUp),
        frameFor("+Z 30", alongZ(30.f, 1.f)),
        frameFor("-Z 30", alongZ(30.f, -1.f)),
    };
}

struct Lcg
{
    std::uint32_t state;
    float next(float range)
    {
        state = state * 1664525u + 1013904223u;
        const float unit = static_cast<float>(state >> 8) * (1.f / 16777216.f);
        return (unit * 2.f - 1.f) * range;
    }
    glm::vec3 vector(float range)
    {
        const float x = next(range);
        const float y = next(range);
        const float z = next(range);
        return glm::vec3(x, y, z);
    }
};

inline glm::vec3 recompose(const Frame& frame, const movement::VelocityChannels& channels)
{
    return frame.u * channels.uv.x + frame.v * channels.uv.y + movement::kWorldUp * channels.up;
}

inline float largestComponentError(const glm::vec3& a, const glm::vec3& b)
{
    const glm::vec3 d = glm::abs(a - b);
    return glm::max(d.x, glm::max(d.y, d.z));
}

inline glm::vec3 pureFall() { return simFrame::withHeight(glm::vec3(0.f), -20.f); }

inline bool sameBits(float a, float b) { return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b); }

inline void requireRecomposes(const Frame& frame, const glm::vec3& velocity, const char* what)
{
    const movement::VelocityChannels channels = movement::decomposeVelocity(velocity, frame.u, frame.v, movement::kWorldUp);
    const glm::vec3 back = recompose(frame, channels);
    const float error = largestComponentError(back, velocity);
    INFO(frame.name << " " << what << " V=(" << velocity.x << ", " << velocity.y << ", " << velocity.z << ") channels=("
         << channels.uv.x << ", " << channels.uv.y << ", " << channels.up << ") recomposed=(" << back.x << ", " << back.y
         << ", " << back.z << ") error=" << error);
    CHECK(error <= 1.e-5f * glm::length(velocity));
}

} // namespace brawlerVelocityDecompositionTests

TEST_CASE("BrawlerVelocityDecomposition.TheFramesUnderTestTakeTheDualSolve", "[BrawlerMovement]")
{
    using namespace brawlerVelocityDecompositionTests;
    const movement::StaticData sd = simulatableBrawler::StaticData{}.m_movementStaticData;
    for (const Frame& frame : generalFrames())
    {
        INFO(frame.name << " n=(" << frame.n.x << ", " << frame.n.y << ", " << frame.n.z << ") dot(v, up)="
             << glm::dot(frame.v, movement::kWorldUp));
        CHECK(glm::abs(glm::length(frame.n) - 1.f) < 1.e-6f);
        CHECK(glm::dot(frame.n, movement::kWorldUp) >= sd.cosMaxSlope);
        CHECK(glm::abs(glm::dot(frame.v, movement::kWorldUp)) > movement::kDualBasisFastPathTolerance);
        CHECK(glm::dot(frame.u, glm::cross(frame.v, movement::kWorldUp)) > 0.f);
    }
    for (const Frame& frame : fastPathFrames())
    {
        INFO(frame.name << " dot(v, up)=" << glm::dot(frame.v, movement::kWorldUp));
        CHECK(glm::abs(glm::dot(frame.v, movement::kWorldUp)) <= movement::kDualBasisFastPathTolerance);
    }
}

TEST_CASE("BrawlerVelocityDecomposition.EveryFrameRecomposesTheVelocity", "[BrawlerMovement]")
{
    using namespace brawlerVelocityDecompositionTests;
    std::vector<Frame> frames = generalFrames();
    for (const Frame& frame : fastPathFrames())
        frames.push_back(frame);

    std::uint32_t cases = 0u;
    for (const Frame& frame : frames)
    {
        requireRecomposes(frame, pureFall(), "pure fall");
        requireRecomposes(frame, frame.u * 6.f, "walk along u");
        requireRecomposes(frame, frame.v * 6.f, "walk along v");
        cases += 3u;
        Lcg random{ 0x9e3779b9u };
        for (int i = 0; i < 50; ++i, ++cases)
            requireRecomposes(frame, random.vector(25.f), "pseudo-random");
    }
    std::printf("[VelocityDecomposition.recompose] frames=%zu cases=%u\n", frames.size(), cases);
}

TEST_CASE("BrawlerVelocityDecomposition.APureFallLeavesTheTangentChannelsEmpty", "[BrawlerMovement]")
{
    using namespace brawlerVelocityDecompositionTests;
    const glm::vec3 fall = pureFall();
    const float bound = 1.e-6f * glm::length(fall);

    for (const Frame& frame : generalFrames())
    {
        const movement::VelocityChannels channels = movement::decomposeVelocity(fall, frame.u, frame.v, movement::kWorldUp);
        INFO(frame.name << " channels=(" << channels.uv.x << ", " << channels.uv.y << ", " << channels.up << ")");
        CHECK(glm::abs(channels.uv.x) <= bound);
        CHECK(glm::abs(channels.uv.y) <= bound);
        CHECK(glm::abs(channels.up - simFrame::height(fall)) <= 1.e-5f * glm::length(fall));
    }

    SECTION("the pre-task solve, frozen as legacyDecompose, leaks the fall on +X 30")
    {
        const Frame frame = frameFor("+X 30", alongX(30.f, 1.f));
        const movement::VelocityChannels legacy = legacyDecompose(fall, frame.u, frame.v, movement::kWorldUp);
        const glm::vec3 back = recompose(frame, legacy);
        std::printf("[VelocityDecomposition.legacy] n=(%.6f, %.6f, %.6f) channels=(%.4f, %.4f, %.4f) recomposed=(%.4f, %.4f, %.4f)\n",
            double(frame.n.x), double(frame.n.y), double(frame.n.z), double(legacy.uv.x), double(legacy.uv.y), double(legacy.up),
            double(back.x), double(back.y), double(back.z));
        CHECK((glm::abs(legacy.uv.x) > bound || glm::abs(legacy.uv.y) > bound));
        CHECK(glm::abs(back.x - 5.f * glm::sqrt(3.f)) < 1.e-4f);
        CHECK(glm::abs(back.y - simFrame::height(fall)) < 1.e-4f);
        CHECK(glm::abs(back.z) < 1.e-4f);
    }
}

TEST_CASE("BrawlerVelocityDecomposition.TheFastPathIsTheLegacySolveBitForBit", "[BrawlerMovement]")
{
    using namespace brawlerVelocityDecompositionTests;
    std::uint32_t compared = 0u;
    std::uint32_t differing = 0u;
    for (const Frame& frame : fastPathFrames())
    {
        Lcg random{ 0x2545f491u };
        for (int i = 0; i < 10000; ++i)
        {
            const glm::vec3 velocity = random.vector(30.f);
            const movement::VelocityChannels now = movement::decomposeVelocity(velocity, frame.u, frame.v, movement::kWorldUp);
            const movement::VelocityChannels before = legacyDecompose(velocity, frame.u, frame.v, movement::kWorldUp);
            ++compared;
            if (sameBits(now.uv.x, before.uv.x) && sameBits(now.uv.y, before.uv.y) && sameBits(now.up, before.up))
                continue;
            if (differing++ == 0u)
                std::printf("[VelocityDecomposition.fastPath] first difference frame=%s i=%d now=(%.9g, %.9g, %.9g) legacy=(%.9g, %.9g, %.9g)\n",
                    frame.name, i, double(now.uv.x), double(now.uv.y), double(now.up),
                    double(before.uv.x), double(before.uv.y), double(before.up));
        }
    }
    std::printf("[VelocityDecomposition.fastPath] compared=%u bitIdentical=%u differing=%u\n", compared, compared - differing, differing);
    CHECK(compared == 30000u);
    CHECK(differing == 0u);
}

TEST_CASE("BrawlerVelocityDecomposition.PlannedSlopesAlongXRecomposeAFallALandingAndAKnockback", "[BrawlerMovement]")
{
    using namespace brawlerVelocityDecompositionTests;
    const float knockbackSpeed =
        simulatableBrawler::StaticData{}.m_hitReactions[dAttackDirection::kRightSequenceId].knockbackSpeed;
    REQUIRE(knockbackSpeed > 0.f);

    for (const float sign : { 1.f, -1.f })
    {
        const Frame frame = frameFor(sign > 0.f ? "+X 30" : "-X 30", alongX(30.f, sign));
        const glm::vec3 up = movement::kWorldUp;
        const float gapRateScale = 1.f / glm::dot(up, frame.n);

        const glm::vec3 fall = pureFall();
        const glm::vec3 landing = horizontal(70.f) * 5.f + simFrame::withHeight(glm::vec3(0.f), -12.f);
        const glm::vec3 upSlope = glm::normalize(surfaceFrame::tiltToSurface(frame.n, glm::normalize(simFrame::flatten(-frame.n))));
        const glm::vec3 across  = glm::normalize(surfaceFrame::tiltToSurface(frame.n, horizontal(0.f)));
        const glm::vec3 knockbackUp     = upSlope * knockbackSpeed;
        const glm::vec3 knockbackAcross = glm::normalize(upSlope + across) * knockbackSpeed;

        struct Case { const char* what; glm::vec3 velocity; };
        for (const Case& c : { Case{ "fall", fall }, Case{ "landing", landing }, Case{ "knockback up the face", knockbackUp },
                               Case{ "knockback across the face", knockbackAcross } })
        {
            const movement::VelocityChannels now = movement::decomposeVelocity(c.velocity, frame.u, frame.v, up);
            const movement::VelocityChannels legacy = legacyDecompose(c.velocity, frame.u, frame.v, up);
            const glm::vec3 back = recompose(frame, now);
            const glm::vec3 legacyBack = recompose(frame, legacy);
            const float gapRate = glm::dot(c.velocity, frame.n) * gapRateScale;
            std::printf("[VelocityDecomposition.planned] n=(%+.4f, %.4f, %.4f) %s V=(%.4f, %.4f, %.4f) channels=(%.4f, %.4f, %.4f) "
                        "error=%.3g legacyChannels=(%.4f, %.4f, %.4f) legacyError=%.4f\n",
                double(frame.n.x), double(frame.n.y), double(frame.n.z), c.what, double(c.velocity.x), double(c.velocity.y),
                double(c.velocity.z), double(now.uv.x), double(now.uv.y), double(now.up), double(largestComponentError(back, c.velocity)),
                double(legacy.uv.x), double(legacy.uv.y), double(legacy.up), double(largestComponentError(legacyBack, c.velocity)));
            INFO(frame.name << " " << c.what << " c=" << now.up << " gapRate=" << gapRate);
            CHECK(largestComponentError(back, c.velocity) <= 1.e-5f * glm::length(c.velocity));
            CHECK(glm::abs(now.up - gapRate) <= 1.e-5f * glm::length(c.velocity));
        }
    }
}

#endif
