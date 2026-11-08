// SPDX-License-Identifier: BUSL-1.1
// docs/SupportEstimateTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceScenario.h"
#include "GoldenTraceScene.h"
#include "GoldenTraceSlopeScenario.h"
#include "QuarterCylinderPlatform.inl"

#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/trigonometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace supportEstimateTests
{

namespace movement = brawlerMovementSimulation;

inline constexpr float kMarchStepM = 0.005f;
inline constexpr float kMarchBeforeM = 0.3f;
inline constexpr float kMarchPastM = 0.6f;
inline constexpr float kCaseTwoBeforeM = 0.1f;
inline constexpr std::array<float, 2u> kCaseThreePastM = { 0.05f, 0.2f };
inline constexpr float kWallMaxHeight = 0.035f;
inline constexpr float kLipMinDeg = 58.f;
inline constexpr float kLipMaxDeg = 62.f;
inline constexpr float kSameNormalCos = 0.9999f;
inline constexpr float kSeamFaceTolerance = 1e-6f;
inline constexpr float kSeamHalfWidthM = 0.3f;
inline constexpr float kSeamStepM = 0.01f;
inline constexpr std::size_t kSeamLines = 18u;
inline constexpr float kSeamFirstS = 0.5f;
inline constexpr float kSeamSpacingS = 0.5f;
inline constexpr float kInteriorMarginM = 0.6f;
inline constexpr float kOnPlaneM = 1e-3f;
inline constexpr float kTopS = 10.02f;
inline constexpr float kLowS = -0.02f;
inline constexpr float kTipPlusW = 10.025f;
inline constexpr float kTipMinusW = -10.02f;
inline constexpr float kChamferSum = 14.175f;
inline constexpr float kHalfSqrt2 = 0.70710678f;
inline constexpr glm::vec2 kTopRim{ kTopS, 1.5f };
inline constexpr glm::vec2 kTopOutward{ 1.f, 0.f };
inline constexpr glm::vec2 kTipRim{ 2.5f, kTipPlusW };
inline constexpr glm::vec2 kTipOutward{ 0.f, 1.f };
inline constexpr glm::vec3 kFlatSlabCentre{ 0.f, -0.25f, 0.f };
inline constexpr glm::vec3 kFlatSlabHalfExtents{ 5.f, 0.25f, 5.f };
inline constexpr glm::vec3 kParking{ 80.f, 40.f, 80.f };
inline constexpr std::array<float, 3u> kCreaseLinesX = { -3.f, 0.f, 3.f };
inline constexpr float kCreaseMarchFromM = 1.f;
inline constexpr float kCreaseMarchToM = 0.6f;

enum class World
{
    Platform,
    FlatSlab,
    Slope,
};

glm::quat rotationOf(const quarterCylinderPlatform::ImportedHull& hull)
{
    return glm::quat(hull.rotationWxyz[0], hull.rotationWxyz[1], hull.rotationWxyz[2], hull.rotationWxyz[3]);
}

struct PlatformFrame
{
    glm::vec3 origin{ 0.f };
    glm::vec3 along{ 0.f };
    glm::vec3 lateral{ 0.f };
    glm::vec3 normal{ 0.f };
    float top = 0.f;

    glm::vec3 surface(const glm::vec2& sw) const { return origin + along * sw.x + lateral * sw.y + normal * top; }
};

PlatformFrame platformFrame()
{
    const quarterCylinderPlatform::ImportedHull& hull = quarterCylinderPlatform::kHulls[0];
    const glm::quat rotation = rotationOf(hull);
    PlatformFrame frame;
    frame.origin = glm::vec3(hull.translation[0], hull.translation[1], hull.translation[2]);
    frame.along = rotation * glm::vec3(0.f, 0.f, -1.f);
    frame.lateral = rotation * glm::vec3(1.f, 0.f, 0.f);
    frame.normal = rotation * glm::vec3(0.f, 1.f, 0.f);
    for (const glm::vec3& point : hull.points)
        frame.top = std::max(frame.top, point.y);
    return frame;
}

float insideMargin(const glm::vec2& sw)
{
    return std::min({ sw.x - kLowS, kTopS - sw.x, kTipPlusW - sw.y, sw.y - kTipMinusW,
        (kChamferSum - sw.x - std::abs(sw.y)) * kHalfSqrt2 });
}

StaticWorldDescription platformStatics()
{
    StaticWorldDescription description;
    uint64_t stableKey = 1u;
    for (const quarterCylinderPlatform::ImportedHull& hull : quarterCylinderPlatform::kHulls)
    {
        StaticShapeDescriptor descriptor = goldenTraceScenario::staticBox(glm::vec3(0.f), glm::quat(1.f, 0.f, 0.f, 0.f),
            glm::vec3(1.f), stableKey++);
        descriptor.shape = StaticConvexHull{ std::vector<glm::vec3>(hull.points.begin(), hull.points.end()) };
        descriptor.localToWorld = glm::translate(glm::mat4(1.f), glm::vec3(hull.translation[0], hull.translation[1], hull.translation[2]))
            * glm::mat4_cast(rotationOf(hull));
        description.shapes.push_back(std::move(descriptor));
    }
    return description;
}

struct EstimateScenario
{
    static constexpr const char* kGenerator = "SupportEstimate.probe";
    static constexpr float kDtSeconds = goldenTraceScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = 1u;
    static constexpr std::array<goldenTraceScene::Character, 1u> kCharacterTable = {
        goldenTraceScene::Character{ 1u, 0u, "", kParking } };
    static constexpr std::array<goldenTraceScene::SpawnPoint, 1u> kSpawnTable = { goldenTraceScene::SpawnPoint{ 0u, kParking } };

    static inline World world = World::Platform;

    static bool isSampleTick(uint32_t) { return false; }
    static StaticWorldDescription statics()
    {
        if (world == World::Platform)
            return platformStatics();
        if (world == World::Slope)
            return goldenTraceSlopeScenario::slopeStatics();
        StaticWorldDescription description;
        description.shapes.push_back(goldenTraceScenario::staticBox(kFlatSlabCentre, glm::quat(1.f, 0.f, 0.f, 0.f), kFlatSlabHalfExtents, 1u));
        return description;
    }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return goldenTraceScenario::scenarioStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int, uint32_t) { return simulatableBrawler::getZeroPlayerInput(); }
    static std::span<const goldenTraceScene::Character> characters() { return kCharacterTable; }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable() { return kSpawnTable; }
    static std::span<const goldenTraceScene::Teleport> teleports() { return {}; }
};

static_assert(goldenTraceScene::Scenario<EstimateScenario>, "the support-estimate scenario provides everything goldenTraceScene::Scene reads");

struct CountingQuery
{
    JoltSpatialQueryAdapter& inner;
    std::optional<QueryVolumeId> centre;
    uint32_t sweeps = 0u;
    uint32_t centreSweeps = 0u;
    SweepHit lastCentre{};

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& transform, const glm::vec3& delta)
    {
        ++sweeps;
        const SweepHit hit = inner.sweep(volumeId, transform, delta);
        if (centre.has_value() && volumeId == *centre)
        {
            ++centreSweeps;
            lastCentre = hit;
        }
        return hit;
    }
};

struct WorldSelection
{
    explicit WorldSelection(World world)
    {
        EstimateScenario::world = world;
    }
};

struct Sample
{
    glm::vec3 position{ 0.f };
    float x = 0.f;
    movement::GroundSupport ground{};
    bool edge = false;
    bool centreSwept = false;
    SweepHit centre{};
    uint32_t sweeps = 0u;
};

struct Fixture
{
    WorldSelection selection;
    goldenTraceScene::Scene<EstimateScenario> scene;
    const SimulatableBrawler& brawler;
    const movement::RuntimeBindings& bindings;
    const movement::StaticData& sd;
    CountingQuery query;

    explicit Fixture(World world)
        : selection(world)
        , scene(std::nullopt, 1.f)
        , brawler(scene.storage.template get<SimulatableBrawler>(EstimateScenario::kCharacterTable[0].id))
        , bindings(brawler.getPhysicsComposite().template get<movement::PhysicsDeclaration>().bindings)
        , sd(scene.staticData.m_movementStaticData)
        , query{ scene.query, std::nullopt }
    {
        if (bindings.queryVolumeIds.size() > 1u)
            query.centre = bindings.queryVolumeIds[1];
    }

    float probeRadius() const { return sd.capsuleRadius - movement::kProbeShrink; }

    glm::vec3 bodyOverPlane(const glm::vec3& foot, const glm::vec3& n, const glm::vec3& planePoint, float clearance) const
    {
        const float r = probeRadius();
        const float bottom = (r + glm::dot(n, planePoint) - n.x * foot.x - n.z * foot.z) / n.y - r + clearance;
        return glm::vec3(foot.x, bottom + sd.capsuleHalfHeight, foot.z);
    }

    Sample at(const glm::vec3& position, float x = 0.f)
    {
        movement::State state;
        state.bodyState.position = position;
        const uint32_t centreBefore = query.centreSweeps;
        const uint32_t sweepsBefore = query.sweeps;
        Sample sample;
        sample.position = position;
        sample.x = x;
        sample.ground = movement::estimateSupport(state, sd, bindings, query);
        sample.centreSwept = query.centreSweeps != centreBefore;
        sample.centre = sample.centreSwept ? query.lastCentre : SweepHit{};
        sample.sweeps = query.sweeps - sweepsBefore;
        const SweepHit& hit = sample.ground.probe.hit;
        sample.edge = hit.blocked && !movement::exactContactOf(hit).faceContact;
        return sample;
    }
};

bool sameBits(const glm::vec3& a, const glm::vec3& b)
{
    return std::bit_cast<uint32_t>(a.x) == std::bit_cast<uint32_t>(b.x) && std::bit_cast<uint32_t>(a.y) == std::bit_cast<uint32_t>(b.y)
        && std::bit_cast<uint32_t>(a.z) == std::bit_cast<uint32_t>(b.z);
}

float tiltDeg(const glm::vec3& n)
{
    return glm::degrees(std::atan2(glm::length(glm::vec2(n.x, n.z)), n.y));
}

const char* kindName(movement::SupportKind kind)
{
    switch (kind)
    {
    case movement::SupportKind::Face: return "Face";
    case movement::SupportKind::Lip: return "Lip";
    default: return "None";
    }
}

bool isWall(const glm::vec3& n) { return glm::dot(n, n) > 0.f && std::abs(n.y) < kWallMaxHeight; }
bool isLip(const glm::vec3& n) { return glm::dot(n, n) > 0.f && tiltDeg(n) >= kLipMinDeg && tiltDeg(n) <= kLipMaxDeg; }

struct Rim
{
    const char* name;
    World world;
    glm::vec3 rimPoint{ 0.f };
    glm::vec3 outward{ 0.f };
    glm::vec3 normal{ 0.f };
    bool expectEdgeBefore = false;
};

Rim platformRim(const char* name, const glm::vec2& rim, const glm::vec2& outward, bool expectEdgeBefore)
{
    const PlatformFrame frame = platformFrame();
    return Rim{ name, World::Platform, frame.surface(rim), glm::normalize(frame.along * outward.x + frame.lateral * outward.y), frame.normal,
        expectEdgeBefore };
}

Rim flatRim()
{
    return Rim{ "flatSlab", World::FlatSlab, glm::vec3(kFlatSlabCentre.x + kFlatSlabHalfExtents.x, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f),
        glm::vec3(0.f, 1.f, 0.f), false };
}

int marchSteps() { return int(std::lround((kMarchBeforeM + kMarchPastM) / kMarchStepM)); }

std::vector<Sample> marchOver(Fixture& fixture, const Rim& rim)
{
    const glm::vec3 atRim = fixture.bodyOverPlane(rim.rimPoint, rim.normal, rim.rimPoint, fixture.sd.rideHeight);
    std::vector<Sample> samples;
    for (int step = 0; step <= marchSteps(); ++step)
    {
        const float x = -kMarchBeforeM + kMarchStepM * float(step);
        const glm::vec3 foot = rim.rimPoint + rim.outward * x;
        samples.push_back(fixture.at(glm::vec3(foot.x, atRim.y, foot.z), x));
    }
    return samples;
}

const Sample& sampleAt(const std::vector<Sample>& samples, float x)
{
    return *std::min_element(samples.begin(), samples.end(),
        [x](const Sample& a, const Sample& b) { return std::abs(a.x - x) < std::abs(b.x - x); });
}

void printSample(const char* tag, const char* name, const Sample& s)
{
    const SweepHit& hit = s.ground.probe.hit;
    std::printf("[%s] rim=%s x=%.3f kind=%s eligible=%d blocked=%d walkable=%d edge=%d clr=%.4f gjkTiltDeg=%.3f exactTiltDeg=%.3f "
                "centre=%d centreBlocked=%d centreTiltDeg=%.3f centreExactTiltDeg=%.3f faceTiltDeg=%.3f\n",
        tag, name, double(s.x), kindName(s.ground.kind), s.ground.eligible ? 1 : 0, hit.blocked ? 1 : 0, s.ground.probe.walkable ? 1 : 0,
        s.edge ? 1 : 0, double(s.ground.probe.clearance), double(tiltDeg(hit.normal)), double(tiltDeg(hit.surfaceNormal)),
        s.centreSwept ? 1 : 0, s.centre.blocked ? 1 : 0, double(tiltDeg(s.centre.normal)), double(tiltDeg(s.centre.surfaceNormal)),
        double(tiltDeg(s.ground.faceNormal)));
}

} // namespace supportEstimateTests

TEST_CASE("SupportEstimate.RegistersTheGroundProbeAndTheCentreProbe", "[GoldenTrace]")
{
    using namespace supportEstimateTests;

    Fixture fixture(World::Platform);
    REQUIRE(fixture.bindings.queryVolumeIds.size() == 2u);
    const PlatformFrame frame = platformFrame();
    const glm::vec3 atRim = fixture.bodyOverPlane(frame.surface(kTopRim), frame.normal, frame.surface(kTopRim), fixture.sd.rideHeight);
    const Sample sample = fixture.at(atRim);
    REQUIRE(sample.ground.probe.hit.blocked);
    REQUIRE(sample.sweeps == (sample.edge ? 2u : 1u));
}

TEST_CASE("SupportEstimate.FaceInteriorIsTheExactFaceWithNoCentreSweep", "[GoldenTrace]")
{
    using namespace supportEstimateTests;

    const float clearances[] = { 0.05f, 0.1f, 0.3f };
    uint32_t samples = 0u;
    uint32_t centreSweeps = 0u;
    float worstGjkRad = 0.f;
    const auto expectFace = [&](Fixture& fixture, const glm::vec3& foot, const glm::vec3& n, const char* world)
    {
        for (const float clearance : clearances)
        {
            const Sample s = fixture.at(fixture.bodyOverPlane(foot, n, foot, clearance));
            const SweepHit& hit = s.ground.probe.hit;
            INFO(world << " foot=(" << foot.x << ", " << foot.y << ", " << foot.z << ") clearance " << clearance << " kind "
                 << kindName(s.ground.kind) << " gjkTilt " << tiltDeg(hit.normal) << " exactTilt " << tiltDeg(hit.surfaceNormal));
            REQUIRE(hit.blocked);
            REQUIRE(glm::dot(hit.surfaceNormal, n) > kSameNormalCos);
            REQUIRE(s.ground.kind == movement::SupportKind::Face);
            REQUIRE(s.ground.eligible);
            REQUIRE(sameBits(s.ground.faceNormal, hit.surfaceNormal));
            REQUIRE(sameBits(s.ground.frameNormal, hit.surfaceNormal));
            REQUIRE_FALSE(s.centreSwept);
            worstGjkRad = std::max(worstGjkRad, std::acos(std::min(1.f, glm::dot(hit.normal, hit.surfaceNormal))));
            ++samples;
        }
    };

    {
        Fixture fixture(World::Platform);
        const PlatformFrame frame = platformFrame();
        for (float s = 0.5f; s <= 9.5f; s += 1.f)
            for (float w = -9.5f; w <= 9.5f; w += 1.f)
            {
                const glm::vec2 sw(s, w);
                if (insideMargin(sw) < kInteriorMarginM || std::abs(w) < kInteriorMarginM)
                    continue;
                expectFace(fixture, frame.surface(sw), frame.normal, "platform");
            }
        centreSweeps += fixture.query.centreSweeps;
    }
    {
        Fixture fixture(World::FlatSlab);
        for (float x = -4.f; x <= 4.f; x += 1.f)
            for (float z = -4.f; z <= 4.f; z += 1.f)
                expectFace(fixture, glm::vec3(x, 0.f, z), glm::vec3(0.f, 1.f, 0.f), "flatSlab");
        centreSweeps += fixture.query.centreSweeps;
    }
    {
        Fixture fixture(World::Slope);
        for (float a = -10.f; a <= 10.f; a += 2.f)
            for (float b = -4.f; b <= 4.f; b += 2.f)
            {
                const glm::vec3 foot = goldenTraceSlopeScenario::kFacePoint + goldenTraceSlopeScenario::kUpTheFallLine * a + glm::vec3(b, 0.f, 0.f);
                expectFace(fixture, foot, goldenTraceSlopeScenario::kFaceNormal, "slope");
            }
        centreSweeps += fixture.query.centreSweeps;
    }
    std::printf("[SupportEstimate.interior] samples=%u centreSweeps=%u worstGjkRad=%.6f edgeContactRad=%.6f\n", samples, centreSweeps,
        double(worstGjkRad), double(std::acos(movement::kEdgeContactCos)));
    REQUIRE(centreSweeps == 0u);
    REQUIRE(samples > 300u);
}

TEST_CASE("SupportEstimate.RimIsFaceThenLipThenNoneWhereWalkableEnds", "[GoldenTrace]")
{
    using namespace supportEstimateTests;

    const Rim rims[] = { platformRim("platformTop", kTopRim, kTopOutward, true), flatRim() };
    for (const Rim& rim : rims)
    {
        Fixture fixture(rim.world);
        const std::vector<Sample> samples = marchOver(fixture, rim);
        INFO("rim " << rim.name);

        const Sample& interior = sampleAt(samples, -kMarchBeforeM);
        REQUIRE(interior.ground.kind == movement::SupportKind::Face);
        const glm::vec3 topExact = interior.ground.probe.hit.surfaceNormal;

        const Sample& before = sampleAt(samples, -kCaseTwoBeforeM);
        printSample("SupportEstimate.case2", rim.name, before);
        REQUIRE(before.ground.kind == movement::SupportKind::Face);
        REQUIRE(glm::dot(before.ground.faceNormal, rim.normal) > kSameNormalCos);
        REQUIRE(sameBits(before.ground.faceNormal, topExact));
        if (rim.expectEdgeBefore)
        {
            REQUIRE(before.edge);
            REQUIRE(before.centreSwept);
        }
        for (const float past : kCaseThreePastM)
        {
            const Sample& s = sampleAt(samples, past);
            printSample("SupportEstimate.case3", rim.name, s);
            REQUIRE(s.ground.kind == movement::SupportKind::Lip);
            REQUIRE(s.ground.eligible);
            REQUIRE(sameBits(s.ground.faceNormal, movement::kWorldUp));
            REQUIRE(sameBits(s.ground.frameNormal, movement::kWorldUp));
        }

        std::optional<std::size_t> firstNone;
        std::optional<std::size_t> firstUnwalkable;
        std::optional<std::size_t> lastFace;
        std::optional<std::size_t> firstLip;
        movement::SupportKind previous = samples.front().ground.kind;
        for (std::size_t i = 0u; i < samples.size(); ++i)
        {
            const Sample& s = samples[i];
            if (s.x > 0.f && !firstNone.has_value() && s.ground.kind == movement::SupportKind::None)
                firstNone = i;
            if (s.x > 0.f && !firstUnwalkable.has_value() && !s.ground.probe.walkable)
                firstUnwalkable = i;
            if (s.ground.kind == movement::SupportKind::Face)
                lastFace = i;
            if (!firstLip.has_value() && s.ground.kind == movement::SupportKind::Lip)
                firstLip = i;
            if (s.ground.kind != previous)
                printSample("SupportEstimate.transition", rim.name, s);
            previous = s.ground.kind;
        }
        REQUIRE(firstNone.has_value());
        REQUIRE(firstUnwalkable.has_value());
        REQUIRE(firstLip.has_value());
        REQUIRE(lastFace.has_value());
        const Sample& none = samples[*firstNone];
        std::printf("[SupportEstimate.rimSummary] rim=%s lastFaceX=%.3f firstLipX=%.3f firstNoneX=%.3f firstUnwalkableX=%.3f "
                    "noneBlocked=%d gjkTiltAtNoneDeg=%.3f gjkTiltBeforeNoneDeg=%.3f cosMaxSlopeDeg=%.3f\n",
            rim.name, double(samples[*lastFace].x), double(samples[*firstLip].x), double(none.x), double(samples[*firstUnwalkable].x),
            none.ground.probe.hit.blocked ? 1 : 0, double(tiltDeg(none.ground.probe.hit.normal)),
            double(tiltDeg(samples[*firstNone - 1u].ground.probe.hit.normal)), double(fixture.sd.maxSlopeAngleDeg));
        REQUIRE(*firstNone == *firstUnwalkable);
        REQUIRE(none.ground.probe.hit.blocked);
        REQUIRE(tiltDeg(none.ground.probe.hit.normal) > fixture.sd.maxSlopeAngleDeg);
        REQUIRE(*lastFace < *firstLip);
        for (std::size_t i = *firstLip; i < *firstNone; ++i)
            REQUIRE(samples[i].ground.kind == movement::SupportKind::Lip);
        for (std::size_t i = *firstNone; i < samples.size(); ++i)
            REQUIRE(samples[i].ground.kind == movement::SupportKind::None);
    }
}

TEST_CASE("SupportEstimate.WallAndLipFacesAreNeverTheFace", "[GoldenTrace]")
{
    using namespace supportEstimateTests;

    const Rim rims[] = { platformRim("platformTop", kTopRim, kTopOutward, true), platformRim("platformTip", kTipRim, kTipOutward, false),
        flatRim() };
    uint32_t wallSeen = 0u;
    uint32_t lipSeen = 0u;
    uint32_t checked = 0u;
    for (const Rim& rim : rims)
    {
        Fixture fixture(rim.world);
        for (const Sample& s : marchOver(fixture, rim))
        {
            const glm::vec3 reported[] = { s.ground.probe.hit.surfaceNormal, s.centreSwept ? s.centre.surfaceNormal : glm::vec3(0.f) };
            for (const glm::vec3& face : reported)
            {
                if (!isWall(face) && !isLip(face))
                    continue;
                wallSeen += isWall(face) ? 1u : 0u;
                lipSeen += isLip(face) ? 1u : 0u;
                INFO("rim " << rim.name << " x " << s.x << " reported face tilt " << tiltDeg(face) << " kind " << kindName(s.ground.kind)
                     << " face tilt " << tiltDeg(s.ground.faceNormal));
                REQUIRE_FALSE((s.ground.kind == movement::SupportKind::Face && glm::dot(s.ground.faceNormal, face) > kSameNormalCos));
                ++checked;
            }
            if (s.ground.kind == movement::SupportKind::Face)
                REQUIRE(glm::dot(s.ground.faceNormal, movement::kWorldUp) >= fixture.sd.cosMaxSlope);
        }
    }
    std::printf("[SupportEstimate.wallAndLip] checked=%u wallSeen=%u lipSeen=%u\n", checked, wallSeen, lipSeen);
    REQUIRE(wallSeen > 0u);
    REQUIRE(lipSeen > 0u);
}

TEST_CASE("SupportEstimate.SeamCrossingsStayOnTheFace", "[GoldenTrace]")
{
    using namespace supportEstimateTests;

    Fixture fixture(World::Platform);
    const PlatformFrame frame = platformFrame();
    uint32_t crossings = 0u;
    uint32_t samples = 0u;
    uint32_t edges = 0u;
    uint32_t lips = 0u;
    float worst = 0.f;
    const int steps = int(std::lround(2.f * kSeamHalfWidthM / kSeamStepM));
    for (std::size_t line = 0u; line < kSeamLines; ++line)
        for (const float direction : { 1.f, -1.f })
        {
            const float s = kSeamFirstS + kSeamSpacingS * float(line);
            const float offset = direction > 0.f ? 0.f : 0.5f * kSeamStepM;
            for (int step = 0; step <= steps; ++step)
            {
                const float w = direction * (-kSeamHalfWidthM + kSeamStepM * float(step) + offset);
                const glm::vec3 foot = frame.surface(glm::vec2(s, w));
                const Sample sample = fixture.at(fixture.bodyOverPlane(foot, frame.normal, foot, fixture.sd.rideHeight), w);
                INFO("seam line s " << s << " w " << w << " kind " << kindName(sample.ground.kind) << " edge " << sample.edge);
                REQUIRE(sample.ground.kind == movement::SupportKind::Face);
                const glm::vec3 d = sample.ground.faceNormal - frame.normal;
                const float largest = std::max({ std::abs(d.x), std::abs(d.y), std::abs(d.z) });
                worst = std::max(worst, largest);
                REQUIRE(largest <= kSeamFaceTolerance);
                edges += sample.edge ? 1u : 0u;
                lips += sample.ground.kind == movement::SupportKind::Lip ? 1u : 0u;
                ++samples;
            }
            ++crossings;
        }
    std::printf("[SupportEstimate.seam] crossings=%u samples=%u edges=%u centreSweeps=%u lips=%u worstComponent=%.3e\n", crossings, samples,
        edges, fixture.query.centreSweeps, lips, double(worst));
    REQUIRE(crossings == 36u);
    REQUIRE(lips == 0u);
}

TEST_CASE("SupportEstimate.ConcaveCreaseTakesTheRisingFaceFromItsFirstContact", "[GoldenTrace]")
{
    using namespace supportEstimateTests;
    using namespace goldenTraceSlopeScenario;

    Fixture fixture(World::Slope);
    const glm::vec3 flatNormal(0.f, 1.f, 0.f);
    const glm::vec3 flatPoint(0.f, kLowerCreaseHeight, kLowerCreaseZ);
    const int steps = int(std::lround((kCreaseMarchFromM + kCreaseMarchToM) / kMarchStepM));
    uint32_t risingSamples = 0u;
    for (const float lineX : kCreaseLinesX)
    {
        std::optional<int> firstRising;
        uint32_t flatFaces = 0u;
        uint32_t centreFaces = 0u;
        for (int step = 0; step <= steps; ++step)
        {
            const float z = kLowerCreaseZ + kCreaseMarchFromM - kMarchStepM * float(step);
            const glm::vec3 foot(lineX, 0.f, z);
            const glm::vec3 overFlat = fixture.bodyOverPlane(foot, flatNormal, flatPoint, fixture.sd.rideHeight);
            const glm::vec3 overSlope = fixture.bodyOverPlane(foot, kFaceNormal, kFacePoint, fixture.sd.rideHeight);
            const Sample s = fixture.at(overFlat.y > overSlope.y ? overFlat : overSlope, z - kLowerCreaseZ);
            const SweepHit& hit = s.ground.probe.hit;
            const bool onRising = hit.blocked && std::abs(glm::dot(hit.impactPoint - kFacePoint, kFaceNormal)) < kOnPlaneM
                && hit.impactPoint.z < kLowerCreaseZ - kOnPlaneM;
            if (onRising && !firstRising.has_value())
            {
                firstRising = step;
                printSample("SupportEstimate.creaseFirstRising", "lowerCrease", s);
            }
            INFO("line x " << lineX << " z " << z << " kind " << kindName(s.ground.kind) << " face tilt " << tiltDeg(s.ground.faceNormal)
                 << " onRising " << onRising);
            REQUIRE(s.ground.kind == movement::SupportKind::Face);
            if (firstRising.has_value())
            {
                REQUIRE(glm::dot(s.ground.faceNormal, kFaceNormal) > kSameNormalCos);
                ++risingSamples;
            }
            else
            {
                flatFaces += glm::dot(s.ground.faceNormal, flatNormal) > kSameNormalCos ? 1u : 0u;
                centreFaces += s.centreSwept ? 1u : 0u;
            }
        }
        REQUIRE(firstRising.has_value());
        std::printf("[SupportEstimate.crease] lineX=%.1f firstRisingDzM=%.3f flatFacesBefore=%u centreSweepsBefore=%u\n",
            double(lineX), double(kCreaseMarchFromM - kMarchStepM * float(*firstRising)), flatFaces, centreFaces);
    }
    REQUIRE(risingSamples > 0u);
}

#endif
