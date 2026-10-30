// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraMatchSoloTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>

namespace dAttackCameraMatchSoloTest
{
using dAttackCameraBehaviour::FovAxis;
using dAttackCameraBehaviour::IsoFramingSettings;
using dAttackCameraBehaviour::OrbitCameraSettings;

constexpr float kAspects[] = { 4.f / 3.f, 16.f / 9.f, 21.f / 9.f, 9.f / 16.f };
constexpr const char* kAspectNames[] = { "4:3", "16:9", "21:9", "9:16" };
constexpr float kBoomsCm[] = { 800.f, 1000.f, 1200.f, 1400.f, 1600.f, 1800.f, 2000.f, 2500.f };
constexpr float kToleranceCm = 1.f;
constexpr double kPi = 3.14159265358979323846;

constexpr IsoFramingSettings kActorDefaults{ 11.33f, FovAxis::Shorter, 400.f, 300.f, 2000.f, 25000.f };

struct SoloFov
{
    float fovDeg;
    FovAxis fovAxis;
};

constexpr SoloFov kSoloFovs[] = {
    { dAttackCameraBehaviour::kFovDeg, FovAxis::Shorter },
    { 58.72f, FovAxis::Shorter },
    { 90.f, FovAxis::Horizontal },
    { 70.f, FovAxis::Horizontal },
};

std::uint32_t bitsOf(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

bool sameBits(const IsoFramingSettings& a, const IsoFramingSettings& b)
{
    return bitsOf(a.fovDeg) == bitsOf(b.fovDeg) && a.fovAxis == b.fovAxis && bitsOf(a.marginH) == bitsOf(b.marginH)
        && bitsOf(a.marginV) == bitsOf(b.marginV) && bitsOf(a.minDist) == bitsOf(b.minDist)
        && bitsOf(a.maxDist) == bitsOf(b.maxDist);
}

OrbitCameraSettings soloSettings(float boomCm, const SoloFov& fov)
{
    OrbitCameraSettings settings;
    settings.boomLengthAtTargetPitch = boomCm;
    settings.fovDeg = fov.fovDeg;
    settings.fovAxis = fov.fovAxis;
    return settings;
}

struct HalfTangents
{
    double horizontal;
    double vertical;
};

HalfTangents soloHalfTangents(const SoloFov& fov, float aspect)
{
    const double authored = std::tan(static_cast<double>(fov.fovDeg) * 0.5 * kPi / 180.0);
    const double a = static_cast<double>(aspect);
    if (fov.fovAxis == FovAxis::Shorter && a > 1.0)
        return HalfTangents{ authored * a, authored };
    return HalfTangents{ authored, authored / a };
}

struct VisibleHalfExtents
{
    double horizontal;
    double vertical;
};

VisibleHalfExtents renderedHalfExtents(double distance, float fieldOfViewDeg, float windowAspect)
{
    const double halfFov = std::max(0.001, static_cast<double>(fieldOfViewDeg)) * kPi / 360.0;
    const double halfWidth = distance * std::tan(halfFov);
    return VisibleHalfExtents{ halfWidth, halfWidth / static_cast<double>(windowAspect) };
}

struct Group
{
    float halfH;
    float halfV;
    const char* name;
};
} // namespace dAttackCameraMatchSoloTest

using namespace dAttackCameraMatchSoloTest;

TEST_CASE("DAttackCamera.MatchedMarginsAreTheSoloReachAtTheLiveAspect", "[DAttack][CameraMatchSolo]")
{
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
        for (const SoloFov& fov : kSoloFovs)
            for (const float boom : kBoomsCm)
            {
                const float aspect = kAspects[a];
                const HalfTangents tangents = soloHalfTangents(fov, aspect);
                const dAttackCameraBehaviour::IsoMargins margins =
                    dAttackCameraBehaviour::matchedIsoMargins(boom, fov.fovDeg, fov.fovAxis, aspect);
                const dAttackCameraBehaviour::SightReach reach =
                    dAttackCameraBehaviour::soloSightReach(soloSettings(boom, fov), aspect);
                INFO(kAspectNames[a] << " fov " << fov.fovDeg << " axis " << static_cast<int>(fov.fovAxis) << " boom "
                    << boom << " margins (" << margins.horizontal << ", " << margins.vertical << ") geometry ("
                    << boom * tangents.horizontal << ", " << boom * tangents.vertical << ")");
                CHECK(std::fabs(margins.horizontal - boom * tangents.horizontal) <= kToleranceCm);
                CHECK(std::fabs(margins.vertical - boom * tangents.vertical) <= kToleranceCm);
                CHECK(bitsOf(reach.halfWidth) == bitsOf(margins.horizontal));
                CHECK(bitsOf(reach.halfHeight) == bitsOf(margins.vertical));
            }
}

TEST_CASE("DAttackCamera.MatchSoloFramesOnePlayerFromTheSoloBoom", "[DAttack][CameraMatchSolo]")
{
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
        for (const SoloFov& fov : kSoloFovs)
            for (const float boom : kBoomsCm)
            {
                const float aspect = kAspects[a];
                const IsoFramingSettings framing =
                    dAttackCameraBehaviour::resolveIsoFraming(true, kActorDefaults, soloSettings(boom, fov), aspect);
                const float distance = dAttackCameraBehaviour::isoFramingDistance(0.f, 0.f, aspect, framing);
                INFO(kAspectNames[a] << " fov " << fov.fovDeg << " axis " << static_cast<int>(fov.fovAxis) << " boom "
                    << boom << " distance " << distance << " margins (" << framing.marginH << ", " << framing.marginV
                    << ")");
                CHECK(std::fabs(distance - boom) <= kToleranceCm);
                CHECK(bitsOf(framing.fovDeg) == bitsOf(fov.fovDeg));
                CHECK(framing.fovAxis == fov.fovAxis);
                CHECK(framing.minDist == 0.f);
                CHECK(bitsOf(framing.maxDist) == bitsOf(kActorDefaults.maxDist));
            }

    const IsoFramingSettings defaults =
        dAttackCameraBehaviour::resolveIsoFraming(true, kActorDefaults, OrbitCameraSettings{}, 16.f / 9.f);
    CHECK(std::fabs(dAttackCameraBehaviour::isoFramingDistance(0.f, 0.f, 16.f / 9.f, defaults)
              - dAttackCameraBehaviour::kBoomLengthAtTargetPitch)
        <= kToleranceCm);
}

TEST_CASE("DAttackCamera.MatchSoloFramesOnePlayerFromTheTunedDefaultBoom", "[DAttack][CameraMatchSolo]")
{
    const OrbitCameraSettings defaults;
    CHECK(defaults.boomLengthAtTargetPitch == 2500.f);
    CHECK(defaults.fovDeg == 40.f);
    CHECK(defaults.fovAxis == FovAxis::Shorter);
    CHECK(defaults.fovDegAtHorizon == 58.f);
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
    {
        const float aspect = kAspects[a];
        const IsoFramingSettings framing =
            dAttackCameraBehaviour::resolveIsoFraming(true, kActorDefaults, defaults, aspect);
        const float distance = dAttackCameraBehaviour::isoFramingDistance(0.f, 0.f, aspect, framing);
        INFO(kAspectNames[a] << " distance " << distance << " margins (" << framing.marginH << ", " << framing.marginV
            << ")");
        CHECK(std::fabs(distance - 2500.f) <= kToleranceCm);
        CHECK(bitsOf(framing.fovDeg) == bitsOf(40.f));
        CHECK(framing.fovAxis == FovAxis::Shorter);

        OrbitCameraSettings wideAtTheHorizon = defaults;
        wideAtTheHorizon.fovDegAtHorizon = 90.f;
        CHECK(sameBits(dAttackCameraBehaviour::resolveIsoFraming(true, kActorDefaults, wideAtTheHorizon, aspect), framing));

        const dAttackCameraBehaviour::CameraSight sight =
            dAttackCameraBehaviour::cameraSight(defaults, true, kActorDefaults, aspect, 0.f, 0.f);
        CHECK(bitsOf(sight.soloHorizontalFovAtHorizonDeg)
            == bitsOf(dAttackCameraBehaviour::horizontalFovDeg(58.f, FovAxis::Shorter, aspect)));
        CHECK(bitsOf(sight.soloHorizontalFovDeg)
            == bitsOf(dAttackCameraBehaviour::horizontalFovDeg(40.f, FovAxis::Shorter, aspect)));
        CHECK(std::fabs(sight.isoDistance - 2500.f) <= kToleranceCm);
    }
}

TEST_CASE("DAttackCamera.MatchSoloKeepsTheSoloReachBeyondTheOutermostPlayers", "[DAttack][CameraMatchSolo]")
{
    constexpr Group kGroups[] = {
        { 300.f, 0.f, "6 m along camera-right" },
        { 0.f, 300.f, "6 m along camera-up" },
        { 300.f, 100.f, "6 m right, 2 m up" },
        { 1500.f, 50.f, "30 m right, 1 m up" },
    };
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
        for (const SoloFov& fov : kSoloFovs)
            for (const float boom : kBoomsCm)
                for (const Group& group : kGroups)
                {
                    const float aspect = kAspects[a];
                    const HalfTangents tangents = soloHalfTangents(fov, aspect);
                    const double soloReachH = boom * tangents.horizontal;
                    const double soloReachV = boom * tangents.vertical;
                    const IsoFramingSettings framing =
                        dAttackCameraBehaviour::resolveIsoFraming(true, kActorDefaults, soloSettings(boom, fov), aspect);
                    const float distance =
                        dAttackCameraBehaviour::isoFramingDistance(group.halfH, group.halfV, aspect, framing);
                    const float fieldOfView =
                        dAttackCameraBehaviour::horizontalFovDeg(framing.fovDeg, framing.fovAxis, aspect);
                    const VisibleHalfExtents visible = renderedHalfExtents(distance, fieldOfView, aspect);
                    const double reachH = visible.horizontal - group.halfH;
                    const double reachV = visible.vertical - group.halfV;
                    INFO(kAspectNames[a] << " fov " << fov.fovDeg << " axis " << static_cast<int>(fov.fovAxis)
                        << " boom " << boom << " group " << group.name << " distance " << distance << " reach ("
                        << reachH << ", " << reachV << ") solo reach (" << soloReachH << ", " << soloReachV << ")");
                    const bool touchesH = std::fabs(reachH - soloReachH) <= kToleranceCm;
                    const bool touchesV = std::fabs(reachV - soloReachV) <= kToleranceCm;
                    CHECK(reachH >= soloReachH - kToleranceCm);
                    CHECK(reachV >= soloReachV - kToleranceCm);
                    CHECK((touchesH || touchesV));
                    if (group.halfV == 0.f)
                    {
                        CHECK(touchesH);
                        CHECK(std::fabs(reachH - framing.marginH) <= kToleranceCm);
                    }
                    if (group.halfH == 0.f)
                    {
                        CHECK(touchesV);
                        CHECK(std::fabs(reachV - framing.marginV) <= kToleranceCm);
                    }
                }
}

TEST_CASE("DAttackCamera.MatchSoloOffIsTheIsoCamerasOwnFraming", "[DAttack][CameraMatchSolo]")
{
    constexpr float kOffAspects[] = { 4.f / 3.f, 16.f / 9.f, 21.f / 9.f, 9.f / 16.f, 1.f, 3.f, 0.4f };
    constexpr IsoFramingSettings kIsoSettings[] = {
        kActorDefaults,
        { 20.f, FovAxis::Horizontal, 800.f, 800.f, 2000.f, 25000.f },
        { 40.f, FovAxis::Shorter, 0.f, 150.f, 0.f, 1000000.f },
        { 0.05f, FovAxis::Horizontal, 400.f, 300.f, 500.f, 3000.f },
    };
    constexpr float kHalfExtents[] = { 0.f, 1.f, 120.f, 900.f, 4000.f };
    int compared = 0;
    int mismatched = 0;
    for (const float aspect : kOffAspects)
        for (const IsoFramingSettings& iso : kIsoSettings)
            for (const SoloFov& fov : kSoloFovs)
                for (const float boom : kBoomsCm)
                {
                    const IsoFramingSettings resolved =
                        dAttackCameraBehaviour::resolveIsoFraming(false, iso, soloSettings(boom, fov), aspect);
                    if (!sameBits(resolved, iso))
                        ++mismatched;
                    if (bitsOf(dAttackCameraBehaviour::horizontalFovDeg(resolved.fovDeg, resolved.fovAxis, aspect))
                        != bitsOf(dAttackCameraBehaviour::horizontalFovDeg(iso.fovDeg, iso.fovAxis, aspect)))
                        ++mismatched;
                    for (const float halfH : kHalfExtents)
                        for (const float halfV : kHalfExtents)
                        {
                            const float viaSettings =
                                dAttackCameraBehaviour::isoFramingDistance(halfH, halfV, aspect, resolved);
                            const float existing = dAttackCameraBehaviour::isoFramingDistance(halfH, halfV, aspect,
                                iso.fovDeg, iso.fovAxis, iso.marginH, iso.marginV, iso.minDist, iso.maxDist);
                            ++compared;
                            if (bitsOf(viaSettings) != bitsOf(existing))
                                ++mismatched;
                        }
                }
    INFO("compared " << compared << " mismatched " << mismatched);
    CHECK(compared == 7 * 4 * 4 * 8 * 25);
    CHECK(mismatched == 0);
}

TEST_CASE("DAttackCamera.CameraSightReportsTheSoloReachAndTheIsoDistance", "[DAttack][CameraMatchSolo]")
{
    const OrbitCameraSettings defaults;
    for (std::size_t a = 0; a < std::size(kAspects); ++a)
    {
        const float aspect = kAspects[a];
        const SoloFov fov{ defaults.fovDeg, defaults.fovAxis };
        const HalfTangents tangents = soloHalfTangents(fov, aspect);
        const double horizontalDeg = 2.0 * std::atan(tangents.horizontal) * 180.0 / kPi;
        const double verticalDeg = 2.0 * std::atan(tangents.vertical) * 180.0 / kPi;

        const dAttackCameraBehaviour::CameraSight matched =
            dAttackCameraBehaviour::cameraSight(defaults, true, kActorDefaults, aspect, 300.f, 0.f);
        INFO(kAspectNames[a] << " fov (" << matched.soloHorizontalFovDeg << ", " << matched.soloVerticalFovDeg
            << ") geometry (" << horizontalDeg << ", " << verticalDeg << ") reach (" << matched.soloReach.halfWidth
            << ", " << matched.soloReach.halfHeight << ") iso " << matched.isoDistance);
        CHECK(matched.soloBoom == dAttackCameraBehaviour::kBoomLengthAtTargetPitch);
        CHECK(std::fabs(matched.soloHorizontalFovDeg - horizontalDeg) <= 0.001);
        CHECK(std::fabs(matched.soloVerticalFovDeg - verticalDeg) <= 0.001);
        CHECK(std::fabs(matched.soloReach.halfWidth - matched.soloBoom * tangents.horizontal) <= kToleranceCm);
        CHECK(std::fabs(matched.soloReach.halfHeight - matched.soloBoom * tangents.vertical) <= kToleranceCm);
        CHECK(matched.matchSolo);
        CHECK(bitsOf(matched.iso.marginH) == bitsOf(matched.soloReach.halfWidth));
        CHECK(bitsOf(matched.iso.marginV) == bitsOf(matched.soloReach.halfHeight));
        CHECK(std::fabs(matched.isoDistance * tangents.horizontal - (300.0 + matched.soloReach.halfWidth))
            <= kToleranceCm);

        const dAttackCameraBehaviour::CameraSight own =
            dAttackCameraBehaviour::cameraSight(defaults, false, kActorDefaults, aspect, 300.f, 0.f);
        CHECK_FALSE(own.matchSolo);
        CHECK(sameBits(own.iso, kActorDefaults));
        CHECK(bitsOf(own.isoDistance)
            == bitsOf(dAttackCameraBehaviour::isoFramingDistance(300.f, 0.f, aspect, kActorDefaults.fovDeg,
                kActorDefaults.fovAxis, kActorDefaults.marginH, kActorDefaults.marginV, kActorDefaults.minDist,
                kActorDefaults.maxDist)));
        CHECK(bitsOf(own.soloReach.halfWidth) == bitsOf(matched.soloReach.halfWidth));
    }

    CHECK(bitsOf(dAttackCameraBehaviour::cameraSight(defaults, true, kActorDefaults, 9.f / 16.f, 0.f, 0.f)
                     .soloHorizontalFovDeg)
        == bitsOf(defaults.fovDeg));
}

#endif // WITH_LOW_LEVEL_TESTS
