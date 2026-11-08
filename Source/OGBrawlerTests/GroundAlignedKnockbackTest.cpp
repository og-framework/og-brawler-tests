// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SurfaceFrame.h"

#include "BrawlerSimulationMocks.h"

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/trigonometric.hpp"

namespace groundAlignedKnockbackTests
{
namespace movement = brawlerMovementSimulation;
namespace machine  = dAttackMachineSimulation;

constexpr float         kDt = 1.f / 60.f;
constexpr BodyId        kBodyId{ 1u };
constexpr QueryVolumeId kVolumeId{ 92u };
constexpr QueryVolumeId kCentreVolumeId{ 93u };

inline glm::vec3 slopeNormal(float degrees)
{
    const float theta = glm::radians(degrees);
    return glm::vec3(0.f, glm::cos(theta), glm::sin(theta));
}

inline glm::vec3 inPlaneUpSlope(const glm::vec3& n)
{
    return surfaceFrame::tiltToSurface(n, simFrame::kForward);
}

inline glm::vec3 outOfPlane(const glm::vec3& inPlane, const glm::vec3& n, float degrees)
{
    const float a = glm::radians(degrees);
    return inPlane * glm::cos(a) + n * glm::sin(a);
}

struct PlaneGround
{
    glm::vec3 planeNormal = glm::vec3(0.f, 1.f, -0.f);
    float     planeOffset = 0.f;
    float     capsuleRadius     = 0.42f;
    float     capsuleHalfHeight = 0.96f;
    glm::vec3 exactFace         = glm::vec3(0.f);
    bool      omitSurfaceNormal = false;

    std::optional<glm::vec3> risingPlaneNormal;
    float     risingPlaneOffset = 0.f;

    std::optional<glm::vec3> centrePlaneNormal;
    glm::vec3 centreExactFace = glm::vec3(0.f);
    bool      centreMisses    = false;

    bool  lastBlocked   = false;
    float lastClearance = 0.f;
    int   centreSweeps  = 0;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& pose, const glm::vec3& delta)
    {
        if (volumeId == kCentreVolumeId)
            return sweepCentre(pose, delta);
        if (volumeId == kVolumeId && risingPlaneNormal.has_value())
            return sweepValley(pose, delta);

        SweepHit hit{};
        lastBlocked = false;
        if (volumeId != kVolumeId)
            return hit;

        const float length = glm::length(delta);
        if (length <= 0.f)
            return hit;
        const glm::vec3 dir = delta / length;

        const glm::vec3 centre(pose[3].x, pose[3].y, pose[3].z);
        const glm::vec3 bottom =
            centre - simFrame::withHeight(glm::vec3(0.f), capsuleHalfHeight - capsuleRadius);

        const float gap  = glm::dot(planeNormal, bottom) - planeOffset - capsuleRadius;
        const float rate = -glm::dot(planeNormal, dir);

        if (rate <= 0.f || gap / rate > length)
        {
            hit.fraction  = 1.f;
            lastClearance = length;
            return hit;
        }

        const float travel = gap / rate;
        hit.blocked       = true;
        hit.fraction      = travel > 0.f ? travel / length : 0.f;
        hit.normal        = planeNormal;
        hit.surfaceNormal = omitSurfaceNormal ? glm::vec3(0.f)
                          : (glm::dot(exactFace, exactFace) > 0.f ? exactFace : planeNormal);
        hit.impactPoint   = centre + dir * travel;
        lastBlocked       = true;
        lastClearance     = hit.fraction * length;
        return hit;
    }

    SweepHit sweepValley(const glm::mat4& pose, const glm::vec3& delta)
    {
        PlaneGround lower = *this;
        lower.risingPlaneNormal.reset();
        PlaneGround rising = lower;
        rising.planeNormal = *risingPlaneNormal;
        rising.planeOffset = risingPlaneOffset;
        const SweepHit a = lower.sweep(kVolumeId, pose, delta);
        const SweepHit c = rising.sweep(kVolumeId, pose, delta);
        const bool risingFirst = c.blocked && (!a.blocked || c.fraction < a.fraction);
        lastBlocked   = risingFirst ? rising.lastBlocked : lower.lastBlocked;
        lastClearance = risingFirst ? rising.lastClearance : lower.lastClearance;
        return risingFirst ? c : a;
    }

    SweepHit sweepCentre(const glm::mat4& pose, const glm::vec3& delta)
    {
        ++centreSweeps;
        SweepHit hit{};
        const float length = glm::length(delta);
        if (centreMisses || length <= 0.f)
            return hit;
        const glm::vec3 dir = delta / length;
        const glm::vec3 n = centrePlaneNormal.value_or(planeNormal);
        const float radius = movement::kCenterProbeRadius;

        const glm::vec3 sphere = glm::vec3(pose[3].x, pose[3].y, pose[3].z)
            - simFrame::withHeight(glm::vec3(0.f), capsuleHalfHeight - radius);
        const float gap  = glm::dot(n, sphere) - planeOffset - radius;
        const float rate = -glm::dot(n, dir);
        if (rate <= 0.f || gap / rate > length)
            return hit;

        const float travel = gap / rate;
        hit.blocked       = true;
        hit.fraction      = travel > 0.f ? travel / length : 0.f;
        hit.normal        = n;
        hit.surfaceNormal = glm::dot(centreExactFace, centreExactFace) > 0.f ? centreExactFace : n;
        hit.impactPoint   = sphere + dir * travel - n * radius;
        return hit;
    }

    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<PlaneGround>);

struct Rig
{
    simulatableBrawler::StaticData game;
    movement::StaticData           sd;

    movement::InitialConditions     ic;
    movement::State                 state;
    movement::DerivedState          derived;
    machine::State                  machineState;
    brawlerInboundHit::DerivedState inbound;

    brawlerTestMocks::MockPhysicsAdapter phys{ 8u };
    PlaneGround                          ground;
    movement::RuntimeBindings            bindings;

    explicit Rig(const glm::vec3& planeNormal)
        : sd(game.m_movementStaticData)
    {
        sd.drivesBody = false;
        bindings.ownBodyId      = kBodyId;
        bindings.parentBodyId   = kBodyId;
        bindings.queryVolumeIds = { kVolumeId, kCentreVolumeId };
        ground.planeNormal      = planeNormal;
    }

    movement::GroundSupport estimate()
    {
        return movement::estimateSupport(state, sd, bindings, ground);
    }

    float knockbackSpeed() const
    { return game.m_hitReactions[dAttackDirection::kRightSequenceId].knockbackSpeed; }

    float heightForClearance(float clearance) const
    {
        const float axisHalf = ground.capsuleHalfHeight - ground.capsuleRadius;
        return axisHalf + clearance
             + (ground.planeOffset + ground.capsuleRadius) / simFrame::height(ground.planeNormal);
    }

    void placeAtClearance(float clearance)
    {
        state.bodyState.position = simFrame::withHeight(glm::vec3(0.f), heightForClearance(clearance));
    }

    void enterKnockbackFlinch()
    {
        machineState.m_currentState       = DAttackState::HitFlinch;
        machineState.m_timeInCurrentState = 0.f;
        machineState.m_hitReaction        = HitReactionKind::Knockback;
        machineState.m_flinchDuration     = 5.f;
    }

    void hitAlong(const glm::vec2& directionXY)
    {
        inbound.wasHitThisTick = true;
        inbound.reactionKind   = HitReactionKind::Knockback;
        inbound.knockbackSpeed = knockbackSpeed();
        inbound.hitDirectionXY = directionXY;
        inbound.flinchDuration = 0.5f;
        enterKnockbackFlinch();
    }

    void placeOnTheHover()
    {
        placeAtClearance(sd.rideHeight);
        const float probeLength = sd.rideHeight + sd.snapDistance;
        for (int i = 0; i < 64; ++i)
        {
            glm::mat4 pose(1.f);
            pose[3] = glm::vec4(state.bodyState.position, 1.f);
            const SweepHit hit = ground.sweep(kVolumeId, pose, -movement::kWorldUp * probeLength);
            if (hit.blocked && hit.fraction * probeLength <= sd.rideHeight)
                return;
            const float h = simFrame::height(state.bodyState.position);
            state.bodyState.position = simFrame::withHeight(state.bodyState.position, std::nextafter(h, -1.f));
        }
    }

    void hitWorld(const glm::vec3& directionWorld)
    {
        const glm::vec2 xy = simFrame::planar(directionWorld);
        hitAlong(glm::dot(xy, xy) > 0.f ? glm::normalize(xy) : glm::vec2(1.f, 0.f));
        inbound.hitDirectionWorld = directionWorld;
    }

    float signedClearance() const
    {
        const glm::vec3 bottom = state.bodyState.position
            - simFrame::withHeight(glm::vec3(0.f), ground.capsuleHalfHeight - ground.capsuleRadius);
        const float gap = glm::dot(ground.planeNormal, bottom) - ground.planeOffset - ground.capsuleRadius;
        return gap / simFrame::height(ground.planeNormal);
    }

    void engineStepWithContact()
    {
        state.bodyState.position += state.velocity * kDt;
        const glm::vec3 bottom = state.bodyState.position
            - simFrame::withHeight(glm::vec3(0.f), ground.capsuleHalfHeight - ground.capsuleRadius);
        const float gap = glm::dot(ground.planeNormal, bottom) - ground.planeOffset - ground.capsuleRadius;
        if (gap < 0.f)
            state.bodyState.position -= ground.planeNormal * gap;
    }

    void tick(std::uint32_t t)
    {
        SimulationComposite<movement::InitialConditions, movement::State, machine::State>
            composite(ic, state, machineState);
        auto deps = makeDependencies<movement::Dependencies>(composite);
        const movement::PlayerInputView pi{ .flags = 0u, .moveDirectionWorld = glm::vec3(0.f) };
        movement::IntegrationUtils<brawlerTestMocks::MockPhysicsAdapter, PlaneGround>
            utils{ kDt, t, phys, ground };
        movement::AllInput<brawlerTestMocks::MockPhysicsAdapter, PlaneGround>
            allInput{ pi, utils };
        movement::integrate(kDt, allInput, sd, deps, bindings, derived, inbound);
        inbound = brawlerInboundHit::DerivedState{};
        ic    = composite.get<movement::InitialConditions>();
        state = composite.get<movement::State>();
    }

    void engineStep() { state.bodyState.position += state.velocity * kDt; }

    void engineStepInTheValley()
    {
        state.bodyState.position += state.velocity * kDt;
        const std::pair<glm::vec3, float> planes[] = { { ground.planeNormal, ground.planeOffset },
            { ground.risingPlaneNormal.value_or(ground.planeNormal), ground.risingPlaneOffset } };
        for (const auto& [normal, offset] : planes)
        {
            const glm::vec3 bottom = state.bodyState.position
                - simFrame::withHeight(glm::vec3(0.f), ground.capsuleHalfHeight - ground.capsuleRadius);
            const float gap = glm::dot(normal, bottom) - offset - ground.capsuleRadius;
            if (gap < 0.f)
                state.bodyState.position -= normal * gap;
        }
    }

    movement::SupportState support() const
    {
        return static_cast<movement::SupportState>(
            (state.flags & movement::kFlagSupportMask) >> movement::kFlagSupportShift);
    }
};

inline bool sameBits(const glm::vec3& a, const glm::vec3& b)
{
    return std::bit_cast<std::uint32_t>(a.x) == std::bit_cast<std::uint32_t>(b.x)
        && std::bit_cast<std::uint32_t>(a.y) == std::bit_cast<std::uint32_t>(b.y)
        && std::bit_cast<std::uint32_t>(a.z) == std::bit_cast<std::uint32_t>(b.z);
}

inline bool sameState(const movement::State& a, const movement::State& b)
{
    return sameBits(a.velocity, b.velocity)
        && sameBits(a.bodyState.position, b.bodyState.position)
        && sameBits(a.positionCmd, b.positionCmd)
        && a.flags == b.flags
        && a.stepStartTick == b.stepStartTick
        && std::bit_cast<std::uint32_t>(a.committedStepDir.x) == std::bit_cast<std::uint32_t>(b.committedStepDir.x)
        && std::bit_cast<std::uint32_t>(a.committedStepDir.y) == std::bit_cast<std::uint32_t>(b.committedStepDir.y);
}

inline const char* firstDifferingField(const movement::State& a, const movement::State& b)
{
    if (!sameBits(a.velocity, b.velocity))                     return "velocity";
    if (!sameBits(a.bodyState.position, b.bodyState.position)) return "bodyState.position";
    if (!sameBits(a.positionCmd, b.positionCmd))               return "positionCmd";
    if (a.flags != b.flags)                                    return "flags";
    if (a.stepStartTick != b.stepStartTick)                    return "stepStartTick";
    if (!sameState(a, b))                                      return "committedStepDir";
    return "none";
}

inline float angleBetween(const glm::vec3& a, const glm::vec3& b)
{
    return glm::atan(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

inline glm::vec3 swingFrameToWorld(const glm::vec3& attackerGround, float heading, const glm::vec3& swingFrameDirection)
{
    return surfaceFrame::tiltMatrix(attackerGround)
         * glm::mat3(glm::rotate(glm::mat4(1.f), heading, simFrame::kUp)) * swingFrameDirection;
}

inline void setPreviousSupport(Rig& rig, movement::SupportState previous)
{
    rig.state.flags = static_cast<std::uint8_t>((rig.state.flags & ~movement::kFlagSupportMask)
        | ((static_cast<std::uint8_t>(previous) << movement::kFlagSupportShift) & movement::kFlagSupportMask));
}

inline glm::vec3 inPlaneChannels(const glm::vec3& velocity, const glm::vec3& n)
{
    return velocity - movement::kWorldUp * (glm::dot(velocity, n) / glm::dot(movement::kWorldUp, n));
}

inline glm::vec3 inPlaneDirection(const glm::vec3& n, float degrees)
{
    const float a = glm::radians(degrees);
    return glm::normalize(surfaceFrame::tiltToSurface(n,
        simFrame::fromForwardRight(glm::vec2(glm::cos(a), glm::sin(a)), 0.f)));
}

} // namespace groundAlignedKnockbackTests

TEST_CASE("GroundAlignedKnockback.DetachedTargetDeceleratesWorldHorizontally", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    const glm::vec3 start = outOfPlane(inPlaneUpSlope(n), n, 5.f) * 20.f;

    Rig aligned(n);
    aligned.placeAtClearance(0.2f);
    aligned.state.velocity = start;
    aligned.enterKnockbackFlinch();

    const float gravityStep = aligned.sd.gravity * kDt;
    const float decelStep   = aligned.sd.launchDecel * kDt;

    int checkedTicks = 0;
    float worstVertical = 0.f;
    for (std::uint32_t t = 1u; t <= 30u; ++t)
    {
        const glm::vec3 before = aligned.state.velocity;
        aligned.tick(t);
        if (!aligned.ground.lastBlocked)
            break;
        const glm::vec3 after = aligned.state.velocity;

        INFO("tick " << t << " clearance " << aligned.ground.lastClearance
             << " v=(" << after.x << ", " << after.y << ", " << after.z << ")");
        REQUIRE(aligned.support() == movement::SupportState::Unsupported);

        const float verticalChange = simFrame::height(after) - simFrame::height(before);
        REQUIRE(verticalChange == Catch::Approx(gravityStep).margin(1e-5f));
        worstVertical = glm::max(worstVertical, glm::abs(verticalChange - gravityStep));
        REQUIRE(sameBits(aligned.derived.surfaceNormal, movement::kWorldUp));

        const glm::vec3 horizontalBefore = simFrame::flatten(before);
        const glm::vec3 horizontalAfter  = simFrame::flatten(after);
        REQUIRE(glm::length(horizontalAfter)
                == Catch::Approx(glm::length(horizontalBefore) - decelStep).margin(1e-4f));
        REQUIRE(glm::dot(glm::normalize(horizontalAfter), glm::normalize(horizontalBefore))
                == Catch::Approx(1.f).margin(1e-6f));

        aligned.engineStep();
        ++checkedTicks;
    }
    std::printf("[GroundAlignedKnockback.detachedBasis] ticks=%d worstAlignedVerticalErr=%.9g gravityStep=%.9g\n",
        checkedTicks, worstVertical, gravityStep);
    REQUIRE(checkedTicks >= 3);
}

static void inPlaneTangentSlidesAlongTheFace()
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    float worstAngle    = 0.f;
    float worstSpeedErr = 0.f;
    int   directions    = 0;
    for (int degrees = 0; degrees < 360; degrees += 15)
    {
        const glm::vec3 d = inPlaneDirection(n, static_cast<float>(degrees));
        REQUIRE(glm::abs(glm::dot(d, n)) < 1e-6f);

        Rig rig(n);
        rig.placeOnTheHover();
        rig.hitWorld(d);
        const float speed = rig.knockbackSpeed();

        rig.tick(1u);
        const glm::vec3 v = rig.state.velocity;
        const float angle = angleBetween(v, d);
        const float speedErr = glm::abs(glm::length(v) - speed);
        INFO("direction " << degrees << " deg v=(" << v.x << ", " << v.y << ", " << v.z << ") angle " << angle
             << " |v| " << glm::length(v));
        REQUIRE(rig.support() == movement::SupportState::Supported);
        REQUIRE(angle <= 1e-5f);
        REQUIRE(speedErr <= 1e-5f);
        worstAngle    = glm::max(worstAngle, angle);
        worstSpeedErr = glm::max(worstSpeedErr, speedErr);

        rig.engineStep();
        rig.tick(2u);
        REQUIRE(rig.support() == movement::SupportState::Supported);
        ++directions;
    }
    std::printf("[GroundAlignedKnockback.inPlane] law=v2 directions=%d worstAngleRad=%.9g worstSpeedErr=%.9g\n",
        directions, worstAngle, worstSpeedErr);
    REQUIRE(directions == 24);
}

TEST_CASE("GroundAlignedKnockback.InPlaneTangentSlidesAlongTheFaceUnderV2", "[BrawlerMovement]")
{
    inPlaneTangentSlidesAlongTheFace();
}

static void directionLeavingTheFaceLaunchesOnTheHitTick()
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    const float headings[] = { 180.f, 150.f, 210.f, 120.f, 240.f };
    float worstVerticalErr = 0.f;
    for (const float heading : headings)
    {
        const float a = glm::radians(heading);
        const glm::vec3 d = simFrame::fromForwardRight(glm::vec2(glm::cos(a), glm::sin(a)), 0.f);
        REQUIRE(glm::dot(d, n) > 0.2f);

        Rig rig(n);
        rig.placeOnTheHover();
        rig.hitWorld(d);
        const float speed       = rig.knockbackSpeed();
        const float gravityStep = rig.sd.gravity * kDt;

        rig.tick(1u);
        const glm::vec3 v = rig.state.velocity;
        const float expectedVertical = simFrame::height(d) * speed + gravityStep;
        const float verticalErr = simFrame::height(v) - expectedVertical;
        INFO("heading " << heading << " v=(" << v.x << ", " << v.y << ", " << v.z << ") vertical " << simFrame::height(v)
             << " expected " << expectedVertical << " lost " << -verticalErr << " m/s");
        CHECK(rig.support() == movement::SupportState::Unsupported);
        CHECK(simFrame::height(v) == Catch::Approx(expectedVertical).margin(1e-5f));
        CHECK(glm::length(simFrame::flatten(v)) == Catch::Approx(speed).margin(1e-5f));
        CHECK(sameBits(rig.derived.surfaceNormal, movement::kWorldUp));
        worstVerticalErr = glm::max(worstVerticalErr, glm::abs(verticalErr));
    }
    std::printf("[GroundAlignedKnockback.launch] law=v2 headings=5 worstVerticalErr=%.9g\n", worstVerticalErr);
}

TEST_CASE("GroundAlignedKnockback.DirectionLeavingTheFaceLaunchesOnTheHitTickUnderV2", "[BrawlerMovement]")
{
    directionLeavingTheFaceLaunchesOnTheHitTick();
}

static void flatTargetHitByASlopeTangentLaunches()
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 attackerGround = slopeNormal(30.f);
    const float headings[] = { 0.f, 0.7f, -1.f, 0.3f };
    for (const float heading : headings)
    {
        const glm::vec3 d = glm::normalize(swingFrameToWorld(attackerGround, heading, simFrame::kForward));
        REQUIRE(simFrame::height(d) > 0.1f);

        Rig rig(glm::vec3(0.f, 1.f, -0.f));
        rig.placeOnTheHover();
        rig.hitWorld(d);
        const float speed       = rig.knockbackSpeed();
        const float gravityStep = rig.sd.gravity * kDt;

        rig.tick(1u);
        const glm::vec3 v = rig.state.velocity;
        INFO("heading " << heading << " d=(" << d.x << ", " << d.y << ", " << d.z << ") v=(" << v.x << ", " << v.y
             << ", " << v.z << ")");
        REQUIRE(rig.support() == movement::SupportState::Unsupported);
        REQUIRE(simFrame::height(v) == Catch::Approx(simFrame::height(d) * speed + gravityStep).margin(1e-5f));
    }
}

TEST_CASE("GroundAlignedKnockback.FlatTargetHitByASlopeTangentLaunchesUnderV2", "[BrawlerMovement]")
{
    flatTargetHitByASlopeTangentLaunches();
}

static void overheadDownswingNeverDetaches()
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 grounds[] = { glm::vec3(0.f, 1.f, -0.f), slopeNormal(30.f) };
    const float pi = glm::pi<float>();
    int hits     = 0;
    int detaches = 0;
    float highestDirectionHeight = -1.f;
    for (const glm::vec3& targetGround : grounds)
        for (const glm::vec3& attackerGround : grounds)
            for (int h = 0; h < 8; ++h)
                for (int s = 0; s <= 14; ++s)
                {
                    const float theta   = 2.f * pi / 8.f + (-1.5f * pi / 8.f - 2.f * pi / 8.f) * (static_cast<float>(s) / 14.f);
                    const float heading = static_cast<float>(h) * (pi / 4.f);
                    const glm::vec3 tangent = simFrame::fromForwardRight(glm::vec2(glm::sin(theta), 0.f), -glm::cos(theta));
                    const glm::vec3 d = glm::normalize(swingFrameToWorld(attackerGround, heading, tangent));
                    highestDirectionHeight = glm::max(highestDirectionHeight, simFrame::height(d));

                    Rig rig(targetGround);
                    rig.placeOnTheHover();
                    rig.hitWorld(d);
                    rig.tick(1u);
                    INFO("theta " << theta << " heading " << heading << " target n.y " << targetGround.y
                         << " attacker n.y " << attackerGround.y);
                    CHECK(rig.support() == movement::SupportState::Supported);
                    if (rig.support() == movement::SupportState::Unsupported)
                        ++detaches;
                    ++hits;
                }
    std::printf("[GroundAlignedKnockback.overhead] law=v2 hits=%d detaches=%d highestDirectionHeight=%.6g\n",
        hits, detaches, highestDirectionHeight);
    REQUIRE(hits == 480);
    REQUIRE(detaches == 0);
}

TEST_CASE("GroundAlignedKnockback.OverheadDownswingNeverDetachesUnderV2", "[BrawlerMovement]")
{
    overheadDownswingNeverDetaches();
}

TEST_CASE("GroundAlignedKnockback.V2UnwalkableExactFaceOverAWalkableSweepIsLipOrTheCentresFace", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 up = movement::kWorldUp;
    const glm::vec3 sweeps[] = { glm::vec3(0.f, 1.f, -0.f), slopeNormal(20.f) };
    const glm::vec3 faces[]  = { slopeNormal(-60.f), simFrame::kForward };
    enum class Centre { SweepPlane, SameEdge, Miss };

    int lips        = 0;
    int centreFaces = 0;
    int heldTicks   = 0;
    for (const glm::vec3& sweep : sweeps)
        for (const glm::vec3& face : faces)
            for (const Centre centre : { Centre::SweepPlane, Centre::SameEdge, Centre::Miss })
            {
                Rig rig(sweep);
                rig.ground.exactFace = face;
                if (centre == Centre::SameEdge)
                    rig.ground.centreExactFace = face;
                rig.ground.centreMisses = centre == Centre::Miss;
                rig.placeOnTheHover();
                INFO("sweep n=(" << sweep.x << ", " << sweep.y << ", " << sweep.z << ") face=(" << face.x << ", " << face.y
                     << ", " << face.z << ") centre " << int(centre));
                REQUIRE(glm::dot(face, sweep) < movement::kEdgeContactCos);

                const movement::GroundSupport g = rig.estimate();
                REQUIRE(rig.ground.centreSweeps == 1);
                REQUIRE(g.eligible);
                if (centre == Centre::SweepPlane)
                {
                    REQUIRE(g.kind == movement::SupportKind::Face);
                    REQUIRE(sameBits(g.faceNormal, sweep));
                    REQUIRE(sameBits(g.frameNormal, sweep));
                    ++centreFaces;
                }
                else
                {
                    REQUIRE(g.kind == movement::SupportKind::Lip);
                    REQUIRE(sameBits(g.faceNormal, up));
                    REQUIRE(sameBits(g.frameNormal, up));
                    ++lips;
                }

                const glm::vec3 across = glm::normalize(simFrame::flatten(face));
                const glm::vec3 horizontals[] = { across, -across, glm::normalize(glm::cross(up, across)) };
                for (const glm::vec3& d : horizontals)
                {
                    Rig hit(sweep);
                    hit.ground.exactFace = face;
                    hit.ground.centreExactFace = rig.ground.centreExactFace;
                    hit.ground.centreMisses = rig.ground.centreMisses;
                    hit.placeOnTheHover();
                    hit.hitWorld(d);
                    hit.tick(1u);
                    const bool leaves = !surfaceFrame::isFlat(g.faceNormal) && glm::dot(d, g.faceNormal) > movement::kMinLaunchSine;
                    INFO("horizontal d=(" << d.x << ", " << d.y << ", " << d.z << ") leaves the estimate's face " << leaves);
                    REQUIRE(hit.support() == (leaves ? movement::SupportState::Unsupported : movement::SupportState::Supported));
                    REQUIRE(sameBits(hit.derived.surfaceNormal, leaves ? up : g.frameNormal));
                    REQUIRE(sameBits(hit.derived.contactNormal, sweep));
                    if (g.kind == movement::SupportKind::Lip || surfaceFrame::isFlat(sweep))
                    {
                        REQUIRE(simFrame::height(hit.state.velocity) == Catch::Approx(0.f).margin(0.05f));
                        ++heldTicks;
                    }
                }
            }
    std::printf("[GroundAlignedKnockback.v2UnwalkableFace] lips=%d centreFaces=%d heldTicks=%d\n", lips, centreFaces, heldTicks);
    REQUIRE(lips == 8);
    REQUIRE(centreFaces == 4);
    REQUIRE(heldTicks == 30);
}

TEST_CASE("GroundAlignedKnockback.V2WalkableExactFaceStillDecides", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 up = movement::kWorldUp;
    struct Pair { glm::vec3 sweep; glm::vec3 face; };
    const Pair pairs[] = {
        { slopeNormal(15.f), slopeNormal(30.f) },
        { slopeNormal(15.f), simFrame::kUp },
        { slopeNormal(29.f), slopeNormal(30.f) },
        { glm::vec3(0.f, 1.f, -0.f), slopeNormal(30.f) },
    };
    const float headings[] = { 180.f, 150.f, 210.f };
    int launches     = 0;
    int faceContacts = 0;
    for (const Pair& pair : pairs)
    {
        const bool edge = glm::dot(pair.face, pair.sweep) < movement::kEdgeContactCos;
        faceContacts += edge ? 0 : 1;

        Rig probe(pair.sweep);
        probe.ground.exactFace = pair.face;
        probe.ground.centrePlaneNormal = pair.face;
        probe.placeOnTheHover();
        const movement::GroundSupport g = probe.estimate();
        INFO("sweep n.y " << pair.sweep.y << " face=(" << pair.face.x << ", " << pair.face.y << ", " << pair.face.z << ") edge " << edge);
        REQUIRE(g.kind == movement::SupportKind::Face);
        REQUIRE(sameBits(g.faceNormal, pair.face));
        REQUIRE(sameBits(g.frameNormal, pair.face));
        REQUIRE(probe.ground.centreSweeps == (edge ? 1 : 0));

        if (surfaceFrame::isFlat(pair.face))
            continue;
        for (const float heading : headings)
        {
            const float a = glm::radians(heading);
            const glm::vec3 d = simFrame::fromForwardRight(glm::vec2(glm::cos(a), glm::sin(a)), 0.f);
            REQUIRE(glm::dot(d, pair.face) > movement::kMinLaunchSine);

            Rig rig(pair.sweep);
            rig.ground.exactFace = pair.face;
            rig.ground.centrePlaneNormal = pair.face;
            rig.placeOnTheHover();
            rig.hitWorld(d);
            rig.tick(1u);
            INFO("heading " << heading);
            REQUIRE(rig.support() == movement::SupportState::Unsupported);
            REQUIRE(sameBits(rig.derived.surfaceNormal, up));
            REQUIRE(simFrame::height(rig.state.velocity)
                    == Catch::Approx(simFrame::height(d) * rig.knockbackSpeed() + rig.sd.gravity * kDt).margin(1e-5f));
            ++launches;
        }
    }
    REQUIRE(faceContacts == 1);
    REQUIRE(launches == 9);
}

TEST_CASE("GroundAlignedKnockback.V2CommittedAirborneTargetReHitAboveItsHoverStaysUnsupported", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 planes[] = { glm::vec3(0.f, 1.f, -0.f), slopeNormal(30.f) };
    int heldAirborne = 0;
    int supportedControls = 0;
    for (const glm::vec3& n : planes)
        for (const float clearance : { 0.15f, 0.3f, 0.45f })
            for (const float degrees : { 0.f, 90.f, 180.f, 270.f })
                for (const float tilt : { -20.f, 0.f })
                {
                    const glm::vec3 d = glm::normalize(outOfPlane(inPlaneDirection(n, degrees), n, tilt));
                    for (const movement::SupportState previous : { movement::SupportState::Unsupported, movement::SupportState::Supported })
                    {
                        Rig rig(n);
                        rig.placeAtClearance(clearance);
                        setPreviousSupport(rig, previous);
                        rig.hitWorld(d);
                        INFO("n.y " << n.y << " clearance " << clearance << " heading " << degrees << " tilt " << tilt
                             << " previous " << static_cast<int>(previous));
                        REQUIRE(rig.estimate().kind == movement::SupportKind::Face);
                        rig.tick(1u);
                        if (previous == movement::SupportState::Unsupported)
                        {
                            REQUIRE(rig.support() == movement::SupportState::Unsupported);
                            REQUIRE(sameBits(rig.derived.surfaceNormal, movement::kWorldUp));
                            const glm::vec3 ballistic = d * rig.knockbackSpeed() + movement::kWorldUp * (rig.sd.gravity * kDt);
                            REQUIRE(glm::length(rig.state.velocity - ballistic) <= 1e-4f);
                            ++heldAirborne;
                        }
                        else
                        {
                            REQUIRE(rig.support() == movement::SupportState::Supported);
                            ++supportedControls;
                        }
                    }
                }
    std::printf("[GroundAlignedKnockback.v2ReHitAirborne] heldAirborne=%d supportedControls=%d\n", heldAirborne, supportedControls);
    REQUIRE(heldAirborne == 48);
    REQUIRE(supportedControls == 48);
}

TEST_CASE("GroundAlignedKnockback.V2HysteresisReadsTheRestoredFlagsNotDerivedState", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    int checked = 0;
    for (const movement::SupportState flagged : { movement::SupportState::Unsupported, movement::SupportState::Supported })
    {
        const movement::SupportState stale = flagged == movement::SupportState::Unsupported
            ? movement::SupportState::Supported
            : movement::SupportState::Unsupported;
        Rig rig(n);
        rig.placeAtClearance(0.3f);
        rig.enterKnockbackFlinch();
        rig.state.velocity = inPlaneUpSlope(n) * 5.f;
        setPreviousSupport(rig, flagged);
        rig.derived.lastSupportState = static_cast<std::uint8_t>(stale);
        rig.tick(1u);
        INFO("flags " << static_cast<int>(flagged) << " stale DerivedState " << static_cast<int>(stale));
        REQUIRE(rig.support() == flagged);
        ++checked;
    }
    REQUIRE(checked == 2);
}

TEST_CASE("GroundAlignedKnockback.V2LaunchedTargetReEntersTheBandUnsupportedUntilItsHover", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 planes[] = { glm::vec3(0.f, 1.f, -0.f), slopeNormal(30.f) };
    for (const glm::vec3& n : planes)
    {
        Rig rig(n);
        rig.placeAtClearance(0.8f);
        rig.enterKnockbackFlinch();
        rig.state.velocity = simFrame::withHeight(glm::vec3(0.f), -1.f);
        setPreviousSupport(rig, movement::SupportState::Unsupported);
        const float probeLength = rig.sd.rideHeight + rig.sd.snapDistance;

        int noneTicks = 0;
        int bandTicks = 0;
        int landingTick = 0;
        float landingClearance = 0.f;
        float lowestBandClearance = probeLength;
        for (std::uint32_t t = 1u; t <= 120u && landingTick == 0; ++t)
        {
            rig.tick(t);
            const float clearance = rig.ground.lastClearance;
            INFO("n.y " << n.y << " tick " << t << " blocked " << rig.ground.lastBlocked << " clearance " << clearance
                 << " vy " << simFrame::height(rig.state.velocity));
            if (!rig.ground.lastBlocked)
            {
                REQUIRE(rig.support() == movement::SupportState::Unsupported);
                ++noneTicks;
            }
            else if (clearance > rig.sd.rideHeight)
            {
                REQUIRE(rig.support() == movement::SupportState::Unsupported);
                REQUIRE(sameBits(rig.derived.surfaceNormal, movement::kWorldUp));
                lowestBandClearance = glm::min(lowestBandClearance, clearance);
                ++bandTicks;
            }
            else
            {
                REQUIRE(rig.support() == movement::SupportState::Supported);
                landingTick = static_cast<int>(t);
                landingClearance = clearance;
            }
            rig.engineStep();
        }
        std::printf("[GroundAlignedKnockback.v2ReEntry] n.y=%.4f noneTicks=%d bandTicks=%d lowestBandClearanceM=%.5f landingTick=%d "
                    "landingClearanceM=%.5f rideHeightM=%.4f\n",
            n.y, noneTicks, bandTicks, lowestBandClearance, landingTick, landingClearance, rig.sd.rideHeight);
        REQUIRE(noneTicks > 0);
        REQUIRE(bandTicks > 0);
        REQUIRE(landingTick > 0);
        REQUIRE(landingClearance <= rig.sd.rideHeight);
    }
}

TEST_CASE("GroundAlignedKnockback.V2WalkerIsSupportedOnEveryKindButNoneAtAnyVelocity", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    struct Ground { glm::vec3 n; bool lip; };
    const Ground grounds[] = { { glm::vec3(0.f, 1.f, -0.f), false }, { slopeNormal(30.f), false }, { glm::vec3(0.f, 1.f, -0.f), true } };
    int supported = 0;
    int none = 0;
    int lips = 0;
    for (const Ground& surface : grounds)
    {
        const glm::vec3& n = surface.n;
        const glm::vec3 inPlane = inPlaneUpSlope(n);
        const glm::vec3 velocities[] = {
            glm::vec3(0.f),
            simFrame::withHeight(glm::vec3(0.f), 20.f),
            simFrame::withHeight(glm::vec3(0.f), -20.f),
            outOfPlane(inPlane, n, 30.f) * 20.f,
            inPlane * 20.f,
        };
        for (const float clearance : { 0.05f, 0.1f, 0.2f, 0.45f, 0.8f })
            for (const glm::vec3& velocity : velocities)
                for (const movement::SupportState previous : { movement::SupportState::Unsupported, movement::SupportState::Supported })
                {
                    Rig rig(n);
                    if (surface.lip)
                    {
                        rig.ground.exactFace = slopeNormal(-60.f);
                        rig.ground.centreMisses = true;
                    }
                    rig.placeAtClearance(clearance);
                    rig.state.velocity = velocity;
                    setPreviousSupport(rig, previous);
                    const movement::SupportKind kind = rig.estimate().kind;
                    rig.tick(1u);
                    INFO("n.y " << n.y << " lip " << surface.lip << " clearance " << clearance << " v=(" << velocity.x << ", " << velocity.y
                         << ", " << velocity.z << ") previous " << static_cast<int>(previous) << " kind " << static_cast<int>(kind));
                    if (surface.lip && kind != movement::SupportKind::None)
                    {
                        REQUIRE(kind == movement::SupportKind::Lip);
                        ++lips;
                    }
                    if (kind == movement::SupportKind::None)
                    {
                        REQUIRE(rig.support() == movement::SupportState::Unsupported);
                        ++none;
                    }
                    else
                    {
                        REQUIRE(rig.support() == movement::SupportState::Supported);
                        ++supported;
                    }
                }
    }
    std::printf("[GroundAlignedKnockback.v2Walker] supported=%d none=%d lips=%d\n", supported, none, lips);
    REQUIRE(supported == 120);
    REQUIRE(none == 30);
    REQUIRE(lips == 40);
}

TEST_CASE("GroundAlignedKnockback.V2CommittedCrossingOntoARisingFaceSlidesAtVCosTheta", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 flat(0.f, 1.f, -0.f);
    const glm::vec3 n = slopeNormal(30.f);
    const float speed = 20.f;
    const float expected = speed * glm::cos(glm::radians(30.f));
    for (const bool decays : { false, true })
    {
        Rig rig(flat);
        const int lockoutTicks = static_cast<int>(glm::ceil(
            glm::max(rig.game.m_hitReactions[dAttackDirection::kRightSequenceId].lockoutDuration,
                     rig.knockbackSpeed() / rig.sd.launchDecel) / kDt - 1e-3f));
        if (!decays)
            rig.sd.launchDecel = 0.f;
        rig.placeOnTheHover();
        rig.state.bodyState.position.z = 1.2f;
        rig.ground.risingPlaneNormal = n;
        rig.state.velocity = simFrame::fromForwardRight(glm::vec2(1.f, 0.f), 0.f) * speed;
        REQUIRE(glm::dot(rig.state.velocity, n) == Catch::Approx(-0.5f * speed).margin(1e-5f));
        rig.enterKnockbackFlinch();
        setPreviousSupport(rig, movement::SupportState::Supported);
        const float decelStep = rig.sd.launchDecel * kDt;

        int unsupportedTicks = 0;
        int faceTick = 0;
        float faceTickSpeed = 0.f;
        float faceTickExpected = 0.f;
        float worstSlideErr = 0.f;
        float minClearance = rig.sd.rideHeight;
        for (int t = 1; t <= lockoutTicks; ++t)
        {
            const movement::GroundSupport g = rig.estimate();
            const glm::vec3 entering = rig.state.velocity;
            const float enteringIntoFace = glm::dot(entering, n);
            rig.tick(static_cast<std::uint32_t>(t));
            const bool onTheRisingFace = glm::dot(g.faceNormal, n) > 0.99999f;
            const float slide = glm::length(inPlaneChannels(rig.state.velocity, n));
            INFO("decays " << decays << " tick " << t << " rising face " << onTheRisingFace << " clearance " << rig.ground.lastClearance
                 << " slide " << slide << " v=(" << rig.state.velocity.x << ", " << rig.state.velocity.y << ", " << rig.state.velocity.z
                 << ")");
            unsupportedTicks += rig.support() == movement::SupportState::Unsupported ? 1 : 0;
            if (onTheRisingFace)
            {
                if (faceTick == 0)
                {
                    faceTick = t;
                    faceTickSpeed = slide;
                    faceTickExpected = glm::sqrt(glm::dot(entering, entering) - enteringIntoFace * enteringIntoFace) - decelStep;
                    REQUIRE(slide == Catch::Approx(faceTickExpected).margin(1e-3f));
                    if (!decays)
                        REQUIRE(faceTickExpected == Catch::Approx(expected).margin(1e-3f));
                }
                else if (!decays)
                    worstSlideErr = glm::max(worstSlideErr, glm::abs(slide - expected));
                minClearance = glm::min(minClearance, rig.ground.lastClearance);
            }
            rig.engineStepInTheValley();
        }
        std::printf("[GroundAlignedKnockback.v2Crossing] decays=%d lockoutTicks=%d faceTick=%d faceTickSlideMps=%.6f expectedMps=%.6f "
                    "worstSlideErrMps=%.3g unsupportedTicks=%d minClearanceOnTheFaceM=%.5f\n",
            decays ? 1 : 0, lockoutTicks, faceTick, faceTickSpeed, faceTickExpected, worstSlideErr, unsupportedTicks, minClearance);
        REQUIRE(faceTick > 1);
        REQUIRE(unsupportedTicks == 0);
        REQUIRE(worstSlideErr <= 1e-3f);
    }
}

TEST_CASE("GroundAlignedKnockback.V2HitTickIntoTheFaceGivesTheProjectedVelocity", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    int projected = 0;
    float worstErr = 0.f;
    std::vector<glm::vec3> directions;
    for (int degrees = 0; degrees < 360; degrees += 45)
        directions.push_back(glm::normalize(outOfPlane(inPlaneDirection(n, static_cast<float>(degrees)), n, -30.f)));
    directions.push_back(simFrame::fromForwardRight(glm::vec2(1.f, 0.f), 0.f));
    for (const glm::vec3& d : directions)
    {
        REQUIRE(glm::dot(d, n) < -movement::kMinLaunchSine);
        Rig rig(n);
        rig.placeOnTheHover();
        rig.hitWorld(d);
        const float speed = rig.knockbackSpeed();
        const glm::vec3 expected = (d - n * glm::dot(d, n)) * speed;
        rig.tick(1u);
        const float err = glm::length(rig.state.velocity - expected);
        INFO("d=(" << d.x << ", " << d.y << ", " << d.z << ") v=(" << rig.state.velocity.x << ", " << rig.state.velocity.y << ", "
             << rig.state.velocity.z << ") expected=(" << expected.x << ", " << expected.y << ", " << expected.z << ")");
        REQUIRE(rig.support() == movement::SupportState::Supported);
        REQUIRE(err <= 1e-4f * speed);
        REQUIRE(glm::length(expected) == Catch::Approx(speed * glm::sqrt(1.f - glm::dot(d, n) * glm::dot(d, n))).margin(1e-4f));
        worstErr = glm::max(worstErr, err);
        ++projected;
    }

    int inPlaneUnredirected = 0;
    for (int degrees = 0; degrees < 360; degrees += 15)
    {
        const glm::vec3 d = inPlaneDirection(n, static_cast<float>(degrees));
        Rig rig(n);
        rig.placeOnTheHover();
        rig.hitWorld(d);
        movement::State written = rig.state;
        written.velocity = d * rig.knockbackSpeed();
        INFO("in-plane direction " << degrees << " deg");
        REQUIRE(!movement::redirectsIntoFace(written, rig.sd, rig.estimate(), movement::SupportState::Supported, true, true));
        ++inPlaneUnredirected;
    }
    std::printf("[GroundAlignedKnockback.v2IntoFaceHit] projected=%d worstErrMps=%.3g inPlaneUnredirected=%d\n", projected, worstErr,
        inPlaneUnredirected);
    REQUIRE(projected == 9);
    REQUIRE(inPlaneUnredirected == 24);
}

TEST_CASE("GroundAlignedKnockback.V2HitTickIntoTheFaceJustAboveTheHoverIsRedirected", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    constexpr float kAboveTheHoverM = 1e-5f;
    const glm::vec3 n = slopeNormal(30.f);
    int projected = 0;
    float worstErr = 0.f;
    float largestExcess = 0.f;
    for (int degrees = 0; degrees < 360; degrees += 45)
    {
        const glm::vec3 d = glm::normalize(outOfPlane(inPlaneDirection(n, static_cast<float>(degrees)), n, -30.f));
        REQUIRE(glm::dot(d, n) < -movement::kMinLaunchSine);
        Rig rig(n);
        rig.placeAtClearance(rig.sd.rideHeight + kAboveTheHoverM);
        setPreviousSupport(rig, movement::SupportState::Supported);
        rig.derived.lastSupportState = static_cast<std::uint8_t>(movement::SupportState::Supported);
        const float clearance = rig.estimate().probe.clearance;
        REQUIRE(clearance > rig.sd.rideHeight);
        REQUIRE(clearance < rig.sd.rideHeight + 2.f * kAboveTheHoverM);
        rig.hitWorld(d);
        const float speed = rig.knockbackSpeed();
        movement::State written = rig.state;
        written.velocity = d * speed;
        INFO("heading " << degrees << " deg, clearance above rideHeight " << clearance - rig.sd.rideHeight << " m");
        CHECK(!movement::redirectsIntoFace(written, rig.sd, rig.estimate(), movement::SupportState::Supported, true, false));
        CHECK(movement::redirectsIntoFace(written, rig.sd, rig.estimate(), movement::SupportState::Supported, true, true));

        const glm::vec3 inFace = (d - n * glm::dot(d, n)) * speed;
        const float e = rig.sd.rideHeight - clearance;
        const float servo = glm::clamp(glm::max(rig.sd.hoverStiffness * e, -rig.sd.hoverPullDownAccel),
            -rig.sd.hoverMaxAccel, rig.sd.hoverMaxAccel);
        const glm::vec3 expected = inFace
            + movement::kWorldUp * glm::max((rig.sd.gravity + servo) * kDt, -rig.sd.terminalFallSpeed);
        rig.tick(1u);
        const float err = glm::length(rig.state.velocity - expected);
        INFO("v=(" << rig.state.velocity.x << ", " << rig.state.velocity.y << ", " << rig.state.velocity.z << ") expected=("
             << expected.x << ", " << expected.y << ", " << expected.z << ")");
        CHECK(rig.support() == movement::SupportState::Supported);
        CHECK(err <= 1e-4f * speed);
        worstErr = glm::max(worstErr, err);
        largestExcess = glm::max(largestExcess, clearance - rig.sd.rideHeight);
        ++projected;
    }
    std::printf("[GroundAlignedKnockback.v2IntoFaceHitAboveTheHover] projected=%d largestExcessM=%.3g worstErrMps=%.3g\n",
        projected, largestExcess, worstErr);
    REQUIRE(projected == 8);
}

TEST_CASE("GroundAlignedKnockback.V2LandingIsNotRedirected", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 planes[] = { glm::vec3(0.f, 1.f, -0.f), slopeNormal(30.f) };
    int landings = 0;
    for (const glm::vec3& n : planes)
        for (const float clearance : { 0.04f, 0.08f })
        {
            const glm::vec3 falling = simFrame::withHeight(glm::vec3(0.f), -3.f) + inPlaneUpSlope(n) * 4.f;
            REQUIRE(glm::dot(falling, n) < -movement::kMinLaunchSine * glm::length(falling));
            Rig landing(n);
            Rig sliding(n);
            for (Rig* rig : { &landing, &sliding })
            {
                rig->placeAtClearance(clearance);
                rig->enterKnockbackFlinch();
                rig->state.velocity = falling;
            }
            setPreviousSupport(landing, movement::SupportState::Unsupported);
            landing.derived.lastSupportState = static_cast<std::uint8_t>(movement::SupportState::Supported);
            setPreviousSupport(sliding, movement::SupportState::Supported);
            sliding.derived.lastSupportState = static_cast<std::uint8_t>(movement::SupportState::Unsupported);
            INFO("n.y " << n.y << " clearance " << clearance);
            REQUIRE(!movement::redirectsIntoFace(landing.state, landing.sd, landing.estimate(), movement::SupportState::Supported, true, false));
            REQUIRE(movement::redirectsIntoFace(sliding.state, sliding.sd, sliding.estimate(), movement::SupportState::Supported, true, false));
            landing.tick(1u);
            sliding.tick(1u);
            REQUIRE(landing.support() == movement::SupportState::Supported);
            REQUIRE(sliding.support() == movement::SupportState::Supported);
            REQUIRE(!sameBits(landing.state.velocity, sliding.state.velocity));
            REQUIRE(glm::dot(landing.state.velocity, n) < glm::dot(sliding.state.velocity, n));
            ++landings;
        }
    std::printf("[GroundAlignedKnockback.v2Landing] landings=%d\n", landings);
    REQUIRE(landings == 4);
}
#endif // WITH_LOW_LEVEL_TESTS
