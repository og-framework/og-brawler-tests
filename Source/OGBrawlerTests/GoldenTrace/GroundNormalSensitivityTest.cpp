// SPDX-License-Identifier: BUSL-1.1
// docs/GroundNormalSensitivityTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"
#include "../GroundAlignedMachineRig.h"

#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationComparisonGlm.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGSimulation/SpatialQueryResult.h"

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat4x4.hpp"
#include "glm/trigonometric.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace groundNormalSensitivityTests
{

using namespace goldenTraceSlopeScenario;

enum class Source : uint32_t
{
    Face,
    Gjk,
};

enum class Zone : uint32_t
{
    Interior,
    Concave,
    Convex,
};

enum class SurfaceClass : uint32_t
{
    Face,
    Flat,
    Edge,
    Unsupported,
};

inline constexpr std::array<float, 4u> kDeltasM = { 1.e-6f, 1.e-5f, 5.e-5f, 1.e-4f };
inline constexpr std::size_t kBandDelta = 3u;
inline constexpr uint32_t kDirections = 6u;
inline constexpr float kGridStepM = 0.25f;
inline constexpr float kEdgeMarginM = 1.f;
inline constexpr int kGridAcrossSteps = int((kHalfWidth - kEdgeMarginM) / kGridStepM);
inline constexpr int kGridAlongSteps = int((kSlopeHalfExtents.z - kEdgeMarginM) / kGridStepM);
inline constexpr float kCreaseStepM = 0.01f;
inline constexpr int kCreaseSteps = 100;
inline constexpr float kAboveSurfaceM = 1.2f;
inline constexpr uint32_t kAims = 36u;
inline constexpr float kAimStepDeg = 10.f;
inline constexpr float kAimHeight = -0.35f;
inline constexpr float kPoisonTiltRad = 1.e-3f;
inline constexpr float kOnSurfaceDot = 1.f - 1.e-6f;
inline constexpr int kDirectCheckEveryCm = 10;
inline constexpr int kDirectCheckEveryGridStep = 8;
inline constexpr std::array<float, 4u> kAngleBinsRad = { 1.e-6f, 1.e-5f, 1.e-4f, 1.e-3f };
inline constexpr std::size_t kListedFlipsAtMost = 12u;
inline constexpr std::array<const char*, kDirections> kDirectionNames = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

static_assert(kDeltasM[kBandDelta] == kDefaultSimilarityEpsilon, "the crease band is measured at the largest delta, the position threshold");
static_assert(float(kGridAcrossSteps) * kGridStepM == kHalfWidth - kEdgeMarginM, "the interior grid reaches the 1 m margin across the face");
static_assert(float(kGridAlongSteps) * kGridStepM == kSlopeHalfExtents.z - kEdgeMarginM, "the interior grid reaches the 1 m margin along the fall line");

struct ProbeScenario
{
    static constexpr const char* kGenerator = "GroundNormalSensitivity.probe";
    static constexpr float kDtSeconds = goldenTraceSlopeScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = 1u;
    static constexpr glm::vec3 kParkedFacePoint{ 5.f, kLandingCentre.y + kLandingHalfExtents.y, kLandingCentre.z + 5.f };
    static constexpr std::array<goldenTraceScene::Character, 1u> kCharacterTable = {
        goldenTraceScene::Character{ 1u, 0u, "", raised(kParkedFacePoint, kTeleportHeight) } };
    static constexpr std::array<goldenTraceScene::SpawnPoint, 1u> kSpawnTable = {
        goldenTraceScene::SpawnPoint{ 0u, raised(kParkedFacePoint, kSpawnHeight) } };

    static bool isSampleTick(uint32_t) { return false; }
    static StaticWorldDescription statics() { return slopeStatics(); }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return slopeStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int, uint32_t) { return simulatableBrawler::getZeroPlayerInput(); }
    static std::span<const goldenTraceScene::Character> characters() { return kCharacterTable; }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable() { return kSpawnTable; }
    static std::span<const goldenTraceScene::Teleport> teleports() { return {}; }
};

static_assert(goldenTraceScene::Scenario<ProbeScenario>, "the probe scenario provides everything goldenTraceScene::Scene reads");

struct SampleQuery
{
    JoltSpatialQueryAdapter& inner;
    Source source = Source::Face;
    float tiltRadians = 0.f;
    SweepHit last;

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& transform, const glm::vec3& delta)
    {
        SweepHit hit = inner.sweep(volumeId, transform, delta);
        if (source == Source::Gjk)
            hit.surfaceNormal = glm::vec3(0.f);
        if (tiltRadians != 0.f)
        {
            const glm::mat4 tilt = glm::rotate(glm::mat4(1.f), tiltRadians, simFrame::kRight);
            hit.normal = glm::vec3(tilt * glm::vec4(hit.normal, 0.f));
            hit.surfaceNormal = glm::vec3(tilt * glm::vec4(hit.surfaceNormal, 0.f));
        }
        last = hit;
        return hit;
    }
};

struct Probe
{
    glm::vec3 position = glm::vec3(0.f);
    Zone zone = Zone::Interior;
    int creaseCm = 0;
    bool directCheck = false;
    int across = 0;
    int along = 0;
};

glm::vec3 creaseFacePoint(Zone zone, int signedCm)
{
    const float distance = kCreaseStepM * float(signedCm < 0 ? -signedCm : signedCm);
    if (zone == Zone::Concave)
    {
        if (signedCm < 0)
            return glm::vec3(0.f, kLowerCreaseHeight + distance * kUpTheFallLine.y, kLowerCreaseZ + distance * kUpTheFallLine.z);
        return glm::vec3(0.f, kLowerCreaseHeight, kLowerCreaseZ + distance);
    }
    if (signedCm < 0)
        return glm::vec3(0.f, kUpperCreaseHeight - distance * kUpTheFallLine.y, kUpperCreaseZ - distance * kUpTheFallLine.z);
    return glm::vec3(0.f, kUpperCreaseHeight, kUpperCreaseZ - distance);
}

std::vector<Probe> probes()
{
    std::vector<Probe> out;
    for (int across = -kGridAcrossSteps; across <= kGridAcrossSteps; ++across)
        for (int along = -kGridAlongSteps; along <= kGridAlongSteps; ++along)
        {
            const glm::vec3 facePoint = kFacePoint + (kGridStepM * float(across)) * simFrame::kRight
                + (kGridStepM * float(along)) * kUpTheFallLine;
            out.push_back(Probe{ raised(facePoint, kAboveSurfaceM), Zone::Interior, 0,
                across % kDirectCheckEveryGridStep == 0 && along % kDirectCheckEveryGridStep == 0, across, along });
        }
    for (const Zone zone : { Zone::Concave, Zone::Convex })
        for (int signedCm = -kCreaseSteps; signedCm <= kCreaseSteps; ++signedCm)
            out.push_back(Probe{ raised(creaseFacePoint(zone, signedCm), kAboveSurfaceM), zone, signedCm,
                signedCm % kDirectCheckEveryCm == 0 });
    return out;
}

std::array<glm::vec3, kAims> aims()
{
    std::array<glm::vec3, kAims> out{};
    for (uint32_t index = 0u; index < kAims; ++index)
    {
        const float heading = glm::radians(kAimStepDeg * float(index));
        out[index] = glm::normalize(std::cos(heading) * simFrame::kForward + std::sin(heading) * simFrame::kLeft + kAimHeight * simFrame::kUp);
    }
    return out;
}

struct AimIC
{
    dAttackRadialSimulation::InitialConditions radial;
    brawlerProjectileSimulation::InitialConditions projectile;
    bool started = false;
};

using AimICs = std::array<AimIC, kAims>;
using NormalKey = std::array<uint32_t, 3u>;

NormalKey keyOf(const glm::vec3& n) { return NormalKey{ std::bit_cast<uint32_t>(n.x), std::bit_cast<uint32_t>(n.y), std::bit_cast<uint32_t>(n.z) }; }

AimICs computeICs(const std::array<glm::vec3, kAims>& aimTable, const glm::vec3& capsule, const brawlerGround::GroundSample& ground)
{
    AimICs out{};
    for (uint32_t index = 0u; index < kAims; ++index)
    {
        const groundAlignedMachineRig::MachineTickResult swing = groundAlignedMachineRig::swingStart(aimTable[index], ground);
        const groundAlignedMachineRig::MachineTickResult launch = groundAlignedMachineRig::hadoukenLaunch(aimTable[index], capsule, ground);
        out[index] = AimIC{ swing.radialIC, launch.projectileIC,
            swing.machine.m_currentState == DAttackState::Attacking
                && swing.radialIC.activeAttackSequence != InvalidAttackSequenceId
                && launch.projectileIC.spawnRequestPending == 1u };
    }
    return out;
}

struct ICCache
{
    const std::array<glm::vec3, kAims>& aimTable;
    std::map<NormalKey, AimICs> byNormal;

    const AimICs& of(const brawlerGround::GroundSample& ground)
    {
        const NormalKey key = keyOf(ground.normal);
        auto found = byNormal.find(key);
        if (found == byNormal.end())
            found = byNormal.emplace(key, computeICs(aimTable, glm::vec3(0.f), ground)).first;
        return found->second;
    }
};

bool sameIC(const AimIC& a, const AimIC& b)
{
    return simFrameDetail::sameBits(a.radial.initialAimAngle, b.radial.initialAimAngle)
        && simFrameDetail::sameBits(a.radial.initialAimRotationAxis, b.radial.initialAimRotationAxis)
        && a.radial.activeAttackSequence == b.radial.activeAttackSequence
        && simFrameDetail::sameBits(a.projectile.spawnDir, b.projectile.spawnDir);
}

bool comparatorFlips(const AimIC& a, const AimIC& b)
{
    return !fieldwiseIsSimilarTo(a.radial, b.radial) || !leafIsSimilarToAsFieldwiseDoes(a.projectile.spawnDir, b.projectile.spawnDir);
}

bool directEpsilonFlips(const AimIC& a, const AimIC& b)
{
    return !isSimilarToField(a.radial.initialAimAngle, b.radial.initialAimAngle)
        || !isSimilarToField(a.radial.initialAimRotationAxis, b.radial.initialAimRotationAxis)
        || a.radial.activeAttackSequence != b.radial.activeAttackSequence
        || !isSimilarToField(a.projectile.spawnDir, b.projectile.spawnDir);
}

struct Tally
{
    uint32_t positions = 0u;
    uint32_t pairs = 0u;
    uint32_t normalDiffers = 0u;
    uint32_t icFlips = 0u;
    uint32_t icFlipsDirectEpsilon = 0u;
    uint32_t aimFlips = 0u;
    uint32_t noOpPairs = 0u;
    float maxNormalAngleRad = 0.f;

    void add(const Tally& other)
    {
        positions += other.positions;
        pairs += other.pairs;
        normalDiffers += other.normalDiffers;
        icFlips += other.icFlips;
        icFlipsDirectEpsilon += other.icFlipsDirectEpsilon;
        aimFlips += other.aimFlips;
        noOpPairs += other.noOpPairs;
        maxNormalAngleRad = std::max(maxNormalAngleRad, other.maxNormalAngleRad);
    }
};

struct ProbeSamples
{
    brawlerGround::GroundSample ground;
    SweepHit hit;
    std::array<std::array<bool, kDirections>, kDeltasM.size()> normalDiffers{};
};

struct FlipRecord
{
    std::size_t probe = 0u;
    std::size_t deltaIndex = 0u;
    uint32_t direction = 0u;
    float angleRad = 0.f;
};

struct SourceRun
{
    std::array<std::array<Tally, kDeltasM.size()>, 3u> tallies{};
    std::vector<FlipRecord> creaseFlips;
    std::vector<ProbeSamples> samples;
    uint32_t notStarted = 0u;
    uint32_t directMismatches = 0u;
    uint32_t directChecks = 0u;
    std::size_t distinctNormals = 0u;
};

glm::vec3 offsetAlong(uint32_t direction, float delta)
{
    glm::vec3 offset(0.f);
    offset[glm::length_t(direction / 2u)] = direction % 2u == 0u ? delta : -delta;
    return offset;
}

float angleBetween(const glm::vec3& a, const glm::vec3& b)
{
    return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

template <typename SceneT>
struct Sampler
{
    SceneT& scene;
    const brawlerMovementSimulation::State& state;
    const brawlerMovementSimulation::RuntimeBindings& bindings;

    brawlerGround::GroundSample at(SampleQuery& query, const glm::vec3& position) const
    {
        brawlerMovementSimulation::State moved = state;
        moved.bodyState.position = position;
        return brawlerMovementSimulation::sampleAttackGround<true>(moved, scene.staticData.m_movementStaticData, bindings, query);
    }
};

template <typename SamplerT>
SourceRun measure(const SamplerT& sampler, JoltSpatialQueryAdapter& adapter, Source source, const std::vector<Probe>& probeTable,
    const std::array<glm::vec3, kAims>& aimTable, float perturbedSampleTiltRad)
{
    SourceRun run;
    ICCache cache{ aimTable, {} };
    SampleQuery query{ adapter, source };
    for (std::size_t probeIndex = 0u; probeIndex < probeTable.size(); ++probeIndex)
    {
        const Probe& probe = probeTable[probeIndex];
        query.tiltRadians = 0.f;
        ProbeSamples samples;
        samples.ground = sampler.at(query, probe.position);
        samples.hit = query.last;
        const AimICs& base = cache.of(samples.ground);
        for (const AimIC& ic : base)
            run.notStarted += ic.started ? 0u : 1u;
        if (probe.directCheck)
        {
            const AimICs direct = computeICs(aimTable, probe.position, samples.ground);
            for (uint32_t index = 0u; index < kAims; ++index)
            {
                ++run.directChecks;
                run.directMismatches += sameIC(direct[index], base[index]) ? 0u : 1u;
            }
        }
        for (std::size_t deltaIndex = 0u; deltaIndex < kDeltasM.size(); ++deltaIndex)
        {
            Tally& tally = run.tallies[uint32_t(probe.zone)][deltaIndex];
            ++tally.positions;
            for (uint32_t direction = 0u; direction < kDirections; ++direction)
            {
                query.tiltRadians = perturbedSampleTiltRad;
                const glm::vec3 movedPosition = probe.position + offsetAlong(direction, kDeltasM[deltaIndex]);
                const brawlerGround::GroundSample moved = sampler.at(query, movedPosition);
                ++tally.pairs;
                tally.noOpPairs += simFrameDetail::sameBits(movedPosition, probe.position) ? 1u : 0u;
                const bool differs = !simFrameDetail::sameBits(moved.normal, samples.ground.normal);
                samples.normalDiffers[deltaIndex][direction] = differs;
                if (!differs)
                    continue;
                ++tally.normalDiffers;
                const float angle = angleBetween(moved.normal, samples.ground.normal);
                tally.maxNormalAngleRad = std::max(tally.maxNormalAngleRad, angle);
                if (probe.zone != Zone::Interior)
                    run.creaseFlips.push_back(FlipRecord{ probeIndex, deltaIndex, direction, angle });
                const AimICs& perturbed = cache.of(moved);
                bool flips = false;
                bool flipsDirect = false;
                for (uint32_t index = 0u; index < kAims; ++index)
                {
                    run.notStarted += perturbed[index].started ? 0u : 1u;
                    const bool aimFlips = comparatorFlips(base[index], perturbed[index]);
                    tally.aimFlips += aimFlips ? 1u : 0u;
                    flips = flips || aimFlips;
                    flipsDirect = flipsDirect || directEpsilonFlips(base[index], perturbed[index]);
                }
                tally.icFlips += flips ? 1u : 0u;
                tally.icFlipsDirectEpsilon += flipsDirect ? 1u : 0u;
            }
        }
        run.samples.push_back(samples);
    }
    run.distinctNormals = cache.byNormal.size();
    return run;
}

const char* sourceName(Source source) { return source == Source::Face ? "face" : "gjk"; }
const char* zoneName(Zone zone) { return zone == Zone::Interior ? "interior" : (zone == Zone::Concave ? "concave" : "convex"); }
const char* className(SurfaceClass surface)
{
    switch (surface)
    {
    case SurfaceClass::Face:
        return "face";
    case SurfaceClass::Flat:
        return "flat";
    case SurfaceClass::Edge:
        return "edge";
    case SurfaceClass::Unsupported:
        return "unsupported";
    }
    return "?";
}

SurfaceClass classOf(const ProbeSamples& samples, float cosMaxSlope)
{
    const glm::vec3 face = glm::dot(samples.hit.surfaceNormal, samples.hit.surfaceNormal) > 0.f ? samples.hit.surfaceNormal : samples.hit.normal;
    if (!samples.hit.blocked || glm::dot(face, simFrame::kUp) < cosMaxSlope)
        return SurfaceClass::Unsupported;
    if (glm::dot(samples.ground.normal, kFaceNormal) >= kOnSurfaceDot)
        return SurfaceClass::Face;
    if (glm::dot(samples.ground.normal, simFrame::kUp) >= kOnSurfaceDot)
        return SurfaceClass::Flat;
    return SurfaceClass::Edge;
}

void printRow(Source source, const char* region, float delta, const Tally& tally)
{
    std::printf("[GroundSensitivity] source=%s region=%s delta=%g positions=%u normalDiffers=%u icFlips=%u flipRate=%.6f\n",
        sourceName(source), region, double(delta), tally.positions, tally.normalDiffers, tally.icFlips,
        tally.pairs == 0u ? 0.0 : double(tally.icFlips) / double(tally.pairs));
    std::printf("[GroundSensitivity.detail] source=%s region=%s delta=%g pairs=%u noOpPairs=%u aimPairs=%u aimFlips=%u "
                "icFlipsDirectEpsilon=%u maxNormalAngleRad=%.3g\n",
        sourceName(source), region, double(delta), tally.pairs, tally.noOpPairs, tally.pairs * kAims, tally.aimFlips,
        tally.icFlipsDirectEpsilon, double(tally.maxNormalAngleRad));
}

std::string runsOf(const std::vector<int>& cms)
{
    std::string text;
    char buffer[48];
    std::size_t index = 0u;
    while (index < cms.size())
    {
        std::size_t last = index;
        while (last + 1u < cms.size() && cms[last + 1u] == cms[last] + 1)
            ++last;
        std::snprintf(buffer, sizeof(buffer), "%s%d..%d", text.empty() ? "" : ",", cms[index], cms[last]);
        text += buffer;
        index = last + 1u;
    }
    return text.empty() ? std::string("none") : text;
}

template <typename SceneT>
struct SceneFixture
{
    SceneT scene{ std::nullopt, 1.f };
    const SimulatableBrawler& brawler = scene.storage.template get<SimulatableBrawler>(ProbeScenario::kCharacterTable[0].id);
    const brawlerMovementSimulation::State& state =
        brawler.getAllState().getState().template get<brawlerMovementSimulation::State>();
    const brawlerMovementSimulation::RuntimeBindings& bindings =
        brawler.getPhysicsComposite().template get<brawlerMovementSimulation::PhysicsDeclaration>().bindings;
};

TEST_CASE("BrawlerJoltSlopeGroundNormalSensitivity.PrintsTheFlipRates", "[GoldenTrace]")
{
    SceneFixture<goldenTraceScene::Scene<ProbeScenario>> fixture;
    REQUIRE_FALSE(fixture.bindings.queryVolumeIds.empty());
    const Sampler<goldenTraceScene::Scene<ProbeScenario>> sampler{ fixture.scene, fixture.state, fixture.bindings };
    const std::vector<Probe> probeTable = probes();
    const std::array<glm::vec3, kAims> aimTable = aims();
    const float cosMaxSlope = fixture.scene.staticData.m_movementStaticData.cosMaxSlope;

    std::array<SourceRun, 2u> runs = { measure(sampler, fixture.scene.query, Source::Face, probeTable, aimTable, 0.f),
        measure(sampler, fixture.scene.query, Source::Gjk, probeTable, aimTable, 0.f) };

    for (const Source source : { Source::Face, Source::Gjk })
    {
        const SourceRun& run = runs[uint32_t(source)];
        for (std::size_t deltaIndex = 0u; deltaIndex < kDeltasM.size(); ++deltaIndex)
        {
            printRow(source, "interior", kDeltasM[deltaIndex], run.tallies[uint32_t(Zone::Interior)][deltaIndex]);
            Tally crease = run.tallies[uint32_t(Zone::Concave)][deltaIndex];
            crease.add(run.tallies[uint32_t(Zone::Convex)][deltaIndex]);
            printRow(source, "crease", kDeltasM[deltaIndex], crease);
        }
    }

    for (const Source source : { Source::Face, Source::Gjk })
        for (const Zone zone : { Zone::Concave, Zone::Convex })
            for (std::size_t deltaIndex = 0u; deltaIndex < kDeltasM.size(); ++deltaIndex)
            {
                const Tally& tally = runs[uint32_t(source)].tallies[uint32_t(zone)][deltaIndex];
                std::printf("[GroundSensitivity.crease] source=%s crease=%s delta=%g positions=%u normalDiffers=%u icFlips=%u flipRate=%.6f\n",
                    sourceName(source), zoneName(zone), double(kDeltasM[deltaIndex]), tally.positions, tally.normalDiffers, tally.icFlips,
                    tally.pairs == 0u ? 0.0 : double(tally.icFlips) / double(tally.pairs));
            }

    uint32_t interiorOffFace = 0u;
    uint32_t startPenetrating = 0u;
    for (const Source source : { Source::Face, Source::Gjk })
    {
        const SourceRun& run = runs[uint32_t(source)];
        uint32_t offFace = 0u;
        uint32_t penetrating = 0u;
        float maxAngleFromFace = 0.f;
        std::array<uint32_t, kAngleBinsRad.size()> angleAbove{};
        std::string offFaceList;
        std::array<std::array<std::vector<int>, 4u>, 2u> classCms{};
        std::array<std::vector<int>, 2u> edgeBandCms{};
        std::array<std::vector<int>, 2u> bandCms{};
        std::array<uint32_t, 2u> creasePenetrating{};
        for (std::size_t index = 0u; index < probeTable.size(); ++index)
        {
            const Probe& probe = probeTable[index];
            const ProbeSamples& samples = run.samples[index];
            const SurfaceClass surface = classOf(samples, cosMaxSlope);
            if (probe.zone == Zone::Interior)
            {
                offFace += surface == SurfaceClass::Face ? 0u : 1u;
                penetrating += samples.hit.startPenetrating ? 1u : 0u;
                const float angleFromFace = angleBetween(samples.ground.normal, kFaceNormal);
                maxAngleFromFace = std::max(maxAngleFromFace, angleFromFace);
                for (std::size_t bin = 0u; bin < kAngleBinsRad.size(); ++bin)
                    angleAbove[bin] += angleFromFace > kAngleBinsRad[bin] ? 1u : 0u;
                if (angleFromFace > kAngleBinsRad.back())
                {
                    char buffer[64];
                    std::snprintf(buffer, sizeof(buffer), " (%.2f,%.2f):%.3g", double(kGridStepM * float(probe.across)),
                        double(kGridStepM * float(probe.along)), double(angleFromFace));
                    offFaceList += buffer;
                }
                continue;
            }
            const uint32_t crease = probe.zone == Zone::Concave ? 0u : 1u;
            classCms[crease][uint32_t(surface)].push_back(probe.creaseCm);
            creasePenetrating[crease] += samples.hit.startPenetrating ? 1u : 0u;
            const SurfaceClass beneath = probe.creaseCm < 0 ? SurfaceClass::Face : SurfaceClass::Flat;
            if (surface != beneath)
                edgeBandCms[crease].push_back(probe.creaseCm);
            const auto& differs = samples.normalDiffers[kBandDelta];
            if (std::any_of(differs.begin(), differs.end(), [](bool value) { return value; }))
                bandCms[crease].push_back(probe.creaseCm);
        }
        if (source == Source::Face)
            interiorOffFace += offFace;
        startPenetrating += penetrating;
        std::printf("[GroundSensitivity.interiorSamples] source=%s positions=%u offFace=%u startPenetrating=%u maxAngleFromFaceRad=%.3g "
                    "angleFromFaceAbove(1e-6/1e-5/1e-4/1e-3 rad)=%u/%u/%u/%u distinctNormals=%zu notStarted=%u directChecks=%u "
                    "directMismatches=%u\n",
            sourceName(source), run.tallies[uint32_t(Zone::Interior)][0].positions, offFace, penetrating, double(maxAngleFromFace),
            angleAbove[0], angleAbove[1], angleAbove[2], angleAbove[3], run.distinctNormals, run.notStarted, run.directChecks,
            run.directMismatches);
        std::printf("[GroundSensitivity.interiorOffFace] source=%s positions(acrossM,upTheFallLineM):angleRad=%s\n", sourceName(source),
            offFaceList.empty() ? " none" : offFaceList.c_str());
        std::string flips;
        for (std::size_t index = 0u; index < run.creaseFlips.size() && index < kListedFlipsAtMost; ++index)
        {
            const FlipRecord& flip = run.creaseFlips[index];
            char buffer[96];
            std::snprintf(buffer, sizeof(buffer), " %s:%dcm:%s%g:%.3g", zoneName(probeTable[flip.probe].zone), probeTable[flip.probe].creaseCm,
                kDirectionNames[flip.direction], double(kDeltasM[flip.deltaIndex]), double(flip.angleRad));
            flips += buffer;
        }
        std::printf("[GroundSensitivity.creaseNormalDiffers] source=%s pairs=%zu first(crease:cm:direction delta:angleRad)=%s\n",
            sourceName(source), run.creaseFlips.size(), flips.empty() ? " none" : flips.c_str());
        for (const Zone zone : { Zone::Concave, Zone::Convex })
        {
            const uint32_t crease = zone == Zone::Concave ? 0u : 1u;
            std::string classes;
            for (const SurfaceClass surface : { SurfaceClass::Face, SurfaceClass::Flat, SurfaceClass::Edge, SurfaceClass::Unsupported })
                classes += std::string(" ") + className(surface) + "=" + runsOf(classCms[crease][uint32_t(surface)]);
            std::printf("[GroundSensitivity.creaseSurface] source=%s crease=%s cm(negative=slope side)%s startPenetrating=%u edgeBandCm=%s "
                        "edgeBandPositions=%zu\n",
                sourceName(source), zoneName(zone), classes.c_str(), creasePenetrating[crease], runsOf(edgeBandCms[crease]).c_str(),
                edgeBandCms[crease].size());
            const std::vector<int>& band = bandCms[crease];
            std::printf("[GroundSensitivity.creaseBand] source=%s crease=%s delta=%g positionsWithNormalDiffers=%zu spanCm=%s widthM=%.2f "
                        "runs=%s\n",
                sourceName(source), zoneName(zone), double(kDeltasM[kBandDelta]), band.size(),
                band.empty() ? "none" : (std::to_string(band.front()) + ".." + std::to_string(band.back())).c_str(),
                band.empty() ? 0.0 : double(kCreaseStepM) * double(band.back() - band.front()), runsOf(band).c_str());
        }
        CHECK(run.notStarted == 0u);
        CHECK(run.directChecks > 0u);
        CHECK(run.directMismatches == 0u);
    }

    std::vector<Probe> interiorProbes;
    std::copy_if(probeTable.begin(), probeTable.end(), std::back_inserter(interiorProbes),
        [](const Probe& probe) { return probe.zone == Zone::Interior; });
    const SourceRun control = measure(sampler, fixture.scene.query, Source::Face, interiorProbes, aimTable, kPoisonTiltRad);
    const Tally& controlTally = control.tallies[uint32_t(Zone::Interior)][0];
    std::printf("[GroundSensitivity.control] source=face region=interior delta=%g perturbedSampleTiltRad=%g pairs=%u normalDiffers=%u "
                "icFlips=%u\n",
        double(kDeltasM[0]), double(kPoisonTiltRad), controlTally.pairs, controlTally.normalDiffers, controlTally.icFlips);

    CHECK(interiorOffFace == 0u);
    CHECK(startPenetrating == 0u);
    CHECK(controlTally.pairs > 0u);
    CHECK(controlTally.icFlips == controlTally.pairs);
    for (std::size_t deltaIndex = 0u; deltaIndex < kDeltasM.size(); ++deltaIndex)
    {
        const Tally& tally = runs[uint32_t(Source::Face)].tallies[uint32_t(Zone::Interior)][deltaIndex];
        INFO("face interior delta " << kDeltasM[deltaIndex] << " pairs " << tally.pairs << " normalDiffers " << tally.normalDiffers);
        CHECK(tally.icFlips == 0u);
    }
}

} // namespace groundNormalSensitivityTests

#endif
