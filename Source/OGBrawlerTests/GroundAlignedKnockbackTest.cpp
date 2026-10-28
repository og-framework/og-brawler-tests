// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
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

    bool  lastBlocked   = false;
    float lastClearance = 0.f;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& pose, const glm::vec3& delta)
    {
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
        hit.surfaceNormal = planeNormal;
        hit.impactPoint   = centre + dir * travel;
        lastBlocked       = true;
        lastClearance     = hit.fraction * length;
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

    explicit Rig(const glm::vec3& planeNormal) : sd(game.m_movementStaticData)
    {
        sd.drivesBody = false;
        bindings.ownBodyId      = kBodyId;
        bindings.parentBodyId   = kBodyId;
        bindings.queryVolumeIds = { kVolumeId };
        ground.planeNormal      = planeNormal;
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

    template <bool Aligned>
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
        movement::integrate<Aligned>(kDt, allInput, sd, deps, bindings, derived, inbound);
        inbound = brawlerInboundHit::DerivedState{};
        ic    = composite.get<movement::InitialConditions>();
        state = composite.get<movement::State>();
    }

    void engineStep() { state.bodyState.position += state.velocity * kDt; }

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

inline glm::vec3 inPlaneDirection(const glm::vec3& n, float degrees)
{
    const float a = glm::radians(degrees);
    return glm::normalize(surfaceFrame::tiltToSurface(n,
        simFrame::fromForwardRight(glm::vec2(glm::cos(a), glm::sin(a)), 0.f)));
}

} // namespace groundAlignedKnockbackTests

TEST_CASE("GroundAlignedKnockback.LiftTestIsSurfaceRelativeOnASlope", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n  = slopeNormal(30.f);
    const glm::vec3 up = movement::kWorldUp;
    const glm::vec3 t  = inPlaneUpSlope(n);
    REQUIRE(glm::abs(glm::dot(t, n)) < 1e-6f);
    REQUIRE(simFrame::height(t) > 0.49f);

    movement::State state;

    state.velocity = t * 20.f;
    INFO("in-plane up-slope v=(" << state.velocity.x << ", " << state.velocity.y << ", " << state.velocity.z
         << ") dot(v, n)=" << glm::dot(state.velocity, n) << " dot(v, up)=" << glm::dot(state.velocity, up));
    REQUIRE_FALSE(movement::detachesFromSupport<true>(state, up, n, true));
    REQUIRE(movement::detachesFromSupport<false>(state, up, n, true));

    state.velocity = outOfPlane(t, n, 5.f) * 20.f;
    REQUIRE(movement::detachesFromSupport<true>(state, up, n, true));
    REQUIRE_FALSE(movement::detachesFromSupport<true>(state, up, n, false));

    state.velocity = outOfPlane(t, n, 1.f) * 20.f;
    REQUIRE_FALSE(movement::detachesFromSupport<true>(state, up, n, true));
    state.velocity = outOfPlane(t, n, 3.f) * 20.f;
    REQUIRE(movement::detachesFromSupport<true>(state, up, n, true));

    int inPlaneDetaches = 0;
    int outOfPlaneHolds = 0;
    for (int i = 0; i < 360; ++i)
    {
        const glm::vec3 inPlane = surfaceFrame::tiltToSurface(n,
            simFrame::fromForwardRight(glm::vec2(glm::cos(glm::radians(float(i))), glm::sin(glm::radians(float(i)))), 0.f));
        state.velocity = inPlane * 20.f;
        if (movement::detachesFromSupport<true>(state, up, n, true))
            ++inPlaneDetaches;
        state.velocity = outOfPlane(inPlane, n, 5.f) * 20.f;
        if (!movement::detachesFromSupport<true>(state, up, n, true))
            ++outOfPlaneHolds;
    }
    INFO("in-plane detaches " << inPlaneDetaches << "/360, 5 deg out holds " << outOfPlaneHolds << "/360");
    REQUIRE(inPlaneDetaches == 0);
    REQUIRE(outOfPlaneHolds == 0);
}

TEST_CASE("GroundAlignedKnockback.FlatGroundPredicateKeepsTodaysAnswer", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 up = movement::kWorldUp;
    const glm::vec3 faces[] = {
        simFrame::kUp,
        glm::vec3(-0.f, 1.f, 0.f),
        glm::vec3(0.f, 1.f, -0.f),
        glm::vec3(-0.f, 1.f, -0.f),
        glm::vec3(5e-7f, 1.f, -5e-7f),
    };
    for (const glm::vec3& face : faces)
        REQUIRE(surfaceFrame::isFlat(face));

    std::uint32_t seed = 0x13u;
    const auto next = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return (static_cast<float>(seed >> 8) / 16777216.f) * 40.f - 20.f;
    };

    movement::State state;
    int checked = 0;
    for (int i = 0; i < 1000; ++i)
    {
        const float h = (i % 10 == 0) ? 0.f : ((i % 10 == 1) ? -0.f : next());
        state.velocity = glm::vec3(next(), h, next());
        for (const bool committed : { false, true })
        {
            const bool today = committed && glm::dot(state.velocity, up) > 0.f;
            for (const glm::vec3& face : faces)
            {
                REQUIRE(movement::detachesFromSupport<false>(state, up, face, committed) == today);
                REQUIRE(movement::detachesFromSupport<true>(state, up, face, committed) == today);
                ++checked;
            }
        }
    }
    REQUIRE(checked == 10000);
}

TEST_CASE("GroundAlignedKnockback.FlatGroundIntegrateKeepsTodaysBits", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    struct Seed { float clearance; glm::vec3 velocity; bool hitTick; };
    const Seed seeds[] = {
        { 0.1f,  glm::vec3(0.f), true },
        { 0.1f,  glm::vec3(3.f, 4.f, -2.f), false },
        { 0.2f,  glm::vec3(-6.f, -1.f, 5.f), false },
        { 0.35f, glm::vec3(0.f, 2.5f, -0.f), false },
    };

    int ticks = 0;
    for (const Seed& s : seeds)
    {
        Rig today(glm::vec3(0.f, 1.f, -0.f));
        Rig aligned(glm::vec3(0.f, 1.f, -0.f));
        for (Rig* rig : { &today, &aligned })
        {
            rig->placeAtClearance(s.clearance);
            rig->state.velocity = s.velocity;
            if (s.hitTick)
                rig->hitAlong(glm::normalize(glm::vec2(0.6f, -0.8f)));
            else
                rig->enterKnockbackFlinch();
        }
        for (std::uint32_t t = 1u; t <= 40u; ++t)
        {
            today.tick<false>(t);
            aligned.tick<true>(t);
            INFO("seed clearance " << s.clearance << " tick " << t);
            REQUIRE(sameState(today.state, aligned.state));
            REQUIRE(sameBits(today.derived.surfaceNormal, aligned.derived.surfaceNormal));
            REQUIRE(today.derived.lastSupportState == aligned.derived.lastSupportState);
            today.engineStep();
            aligned.engineStep();
            ++ticks;
        }
    }
    REQUIRE(ticks == 160);
}

TEST_CASE("GroundAlignedKnockback.DetachedTargetDeceleratesWorldHorizontally", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    const glm::vec3 start = outOfPlane(inPlaneUpSlope(n), n, 5.f) * 20.f;

    Rig aligned(n);
    aligned.placeAtClearance(0.2f);
    aligned.state.velocity = start;
    aligned.enterKnockbackFlinch();

    Rig today(n);
    today.placeAtClearance(0.2f);
    today.state.velocity = start;
    today.enterKnockbackFlinch();

    const float gravityStep = aligned.sd.gravity * kDt;
    const float decelStep   = aligned.sd.launchDecel * kDt;

    int checkedTicks = 0;
    float worstVertical = 0.f;
    float worstTodayVertical = 0.f;
    for (std::uint32_t t = 1u; t <= 30u; ++t)
    {
        const glm::vec3 before = aligned.state.velocity;
        aligned.tick<true>(t);
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

        const glm::vec3 todayBefore = today.state.velocity;
        today.tick<false>(t);
        const float todayChange = simFrame::height(today.state.velocity) - simFrame::height(todayBefore);
        worstTodayVertical = glm::max(worstTodayVertical, glm::abs(todayChange - gravityStep));

        aligned.engineStep();
        today.engineStep();
        ++checkedTicks;
    }
    std::printf("[GroundAlignedKnockback.detachedBasis] ticks=%d worstAlignedVerticalErr=%.9g "
                "worstTodayVerticalErr=%.9g gravityStep=%.9g\n",
        checkedTicks, worstVertical, worstTodayVertical, gravityStep);
    REQUIRE(checkedTicks >= 3);
    REQUIRE(worstTodayVertical > 0.1f);
}

TEST_CASE("GroundAlignedKnockback.InPlaneTangentSlidesAlongTheFace", "[BrawlerMovement]")
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

        rig.tick<true>(1u);
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
        rig.tick<true>(2u);
        REQUIRE(rig.support() == movement::SupportState::Supported);
        ++directions;
    }
    std::printf("[GroundAlignedKnockback.inPlane] directions=%d worstAngleRad=%.9g worstSpeedErr=%.9g\n",
        directions, worstAngle, worstSpeedErr);
    REQUIRE(directions == 24);
}

TEST_CASE("GroundAlignedKnockback.DirectionLeavingTheFaceLaunchesOnTheHitTick", "[BrawlerMovement]")
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

        rig.tick<true>(1u);
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
    std::printf("[GroundAlignedKnockback.launch] headings=5 worstVerticalErr=%.9g\n", worstVerticalErr);
}

TEST_CASE("GroundAlignedKnockback.FlatTargetHitByASlopeTangentLaunches", "[BrawlerMovement]")
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

        rig.tick<true>(1u);
        const glm::vec3 v = rig.state.velocity;
        INFO("heading " << heading << " d=(" << d.x << ", " << d.y << ", " << d.z << ") v=(" << v.x << ", " << v.y
             << ", " << v.z << ")");
        REQUIRE(rig.support() == movement::SupportState::Unsupported);
        REQUIRE(simFrame::height(v) == Catch::Approx(simFrame::height(d) * speed + gravityStep).margin(1e-5f));
    }
}

TEST_CASE("GroundAlignedKnockback.DirectionIntoTheFaceStaysSupported", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    const glm::vec3 d = simFrame::fromForwardRight(glm::vec2(1.f, 0.f), 0.f);
    REQUIRE(glm::dot(d, n) == Catch::Approx(-0.5f).margin(1e-6f));

    for (const bool contact : { false, true })
    {
        Rig rig(n);
        rig.sd.drivesBody = contact;
        rig.placeOnTheHover();
        rig.hitWorld(d);
        const float speed = rig.knockbackSpeed();
        const int lockoutTicks = static_cast<int>(glm::ceil(
            glm::max(rig.game.m_hitReactions[dAttackDirection::kRightSequenceId].lockoutDuration,
                     speed / rig.sd.launchDecel) / kDt - 1e-3f));

        const glm::vec3 start = rig.state.bodyState.position;
        rig.tick<true>(1u);
        INFO("contact " << contact);
        REQUIRE(rig.support() == movement::SupportState::Supported);
        const glm::vec3 hitTickVelocity = rig.state.velocity;

        float minClearance = rig.signedClearance();
        int   unsupportedTicks = 0;
        int   firstUnsupportedTick = 0;
        float peakVerticalSpeed = simFrame::height(hitTickVelocity);
        for (int t = 2; t <= lockoutTicks; ++t)
        {
            if (contact)
                rig.engineStepWithContact();
            else
                rig.engineStep();
            minClearance = glm::min(minClearance, rig.signedClearance());
            rig.tick<true>(static_cast<std::uint32_t>(t));
            if (rig.support() == movement::SupportState::Unsupported)
            {
                ++unsupportedTicks;
                if (firstUnsupportedTick == 0)
                    firstUnsupportedTick = t;
            }
            peakVerticalSpeed = glm::max(peakVerticalSpeed, simFrame::height(rig.state.velocity));
        }
        if (contact)
            rig.engineStepWithContact();
        else
            rig.engineStep();
        minClearance = glm::min(minClearance, rig.signedClearance());

        const glm::vec3 travel = rig.state.bodyState.position - start;
        std::printf("[GroundAlignedKnockback.intoFace] mode=%s lockoutTicks=%d hitTickSpeed=%.6g hitTickVertical=%.6g "
                    "travelM=%.6g alongDM=%.6g alongFaceM=%.6g minClearanceM=%.6g rideHeightM=%.6g "
                    "hitTickAlongFace=%.6g unsupportedTicks=%d firstUnsupportedTick=%d peakVerticalSpeed=%.6g "
                    "endSpeed=%.6g\n",
            contact ? "emulatedContact" : "servoOnly", lockoutTicks,
            glm::length(hitTickVelocity), simFrame::height(hitTickVelocity),
            glm::length(travel), glm::dot(travel, d),
            glm::length(travel - n * glm::dot(travel, n)),
            minClearance, rig.sd.rideHeight,
            glm::length(hitTickVelocity - n * glm::dot(hitTickVelocity, n)),
            unsupportedTicks, firstUnsupportedTick, peakVerticalSpeed, glm::length(rig.state.velocity));
    }
}

TEST_CASE("GroundAlignedKnockback.OverheadDownswingNeverDetaches", "[BrawlerMovement]")
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
                    rig.tick<true>(1u);
                    INFO("theta " << theta << " heading " << heading << " target n.y " << targetGround.y
                         << " attacker n.y " << attackerGround.y);
                    CHECK(rig.support() == movement::SupportState::Supported);
                    if (rig.support() == movement::SupportState::Unsupported)
                        ++detaches;
                    ++hits;
                }
    std::printf("[GroundAlignedKnockback.overhead] hits=%d detaches=%d highestDirectionHeight=%.6g\n",
        hits, detaches, highestDirectionHeight);
    REQUIRE(hits == 480);
    REQUIRE(detaches == 0);
}

TEST_CASE("GroundAlignedKnockback.FlatTargetWithAHorizontalDirectionKeepsTodaysBits", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    struct Seed { float clearance; glm::vec3 velocity; };
    const Seed seeds[] = {
        { 0.1f,  glm::vec3(0.f) },
        { 0.1f,  glm::vec3(2.f, -0.3f, 1.f) },
        { 0.12f, glm::vec3(-1.f, 0.2f, -3.f) },
        { 0.08f, glm::vec3(0.f, -0.f, 0.5f) },
        { 0.7f,  glm::vec3(0.f, -2.f, 0.f) },
    };
    const glm::vec3 planes[] = { glm::vec3(0.f, 1.f, -0.f), simFrame::kUp };
    const glm::vec3 directions[] = {
        simFrame::compose(glm::normalize(glm::vec2(0.6f, -0.8f)), 0.f),
        simFrame::compose(glm::vec2(1.f, 0.f), -0.f),
        simFrame::compose(glm::normalize(glm::vec2(-0.3f, 0.95f)), 0.f),
    };

    int ticks = 0;
    for (const glm::vec3& plane : planes)
        for (const Seed& s : seeds)
            for (const glm::vec3& d : directions)
            {
                Rig today(plane);
                Rig aligned(plane);
                for (Rig* rig : { &today, &aligned })
                {
                    rig->placeAtClearance(s.clearance);
                    rig->state.velocity = s.velocity;
                    rig->hitWorld(d);
                }
                for (std::uint32_t t = 1u; t <= 40u; ++t)
                {
                    today.tick<false>(t);
                    aligned.tick<true>(t);
                    INFO("seed clearance " << s.clearance << " vertical " << simFrame::height(s.velocity)
                         << " d=(" << d.x << ", " << d.y << ", " << d.z << ") tick " << t
                         << " first differing field " << firstDifferingField(today.state, aligned.state)
                         << " today v=(" << today.state.velocity.x << ", " << today.state.velocity.y << ", "
                         << today.state.velocity.z << ") aligned v=(" << aligned.state.velocity.x << ", "
                         << aligned.state.velocity.y << ", " << aligned.state.velocity.z << ")");
                    REQUIRE(sameState(today.state, aligned.state));
                    REQUIRE(sameBits(today.derived.surfaceNormal, aligned.derived.surfaceNormal));
                    REQUIRE(today.derived.lastSupportState == aligned.derived.lastSupportState);
                    today.engineStep();
                    aligned.engineStep();
                    ++ticks;
                }
            }
    REQUIRE(ticks == 1200);
}

TEST_CASE("GroundAlignedKnockback.KnobOffIgnoresTheWorldDirection", "[BrawlerMovement]")
{
    using namespace groundAlignedKnockbackTests;

    const glm::vec3 n = slopeNormal(30.f);
    const glm::vec3 planes[] = { n, glm::vec3(0.f, 1.f, -0.f) };
    const glm::vec3 directions[] = {
        inPlaneDirection(n, 0.f),
        inPlaneDirection(n, 100.f),
        simFrame::fromForwardRight(glm::vec2(-1.f, 0.f), 0.f),
        simFrame::fromForwardRight(glm::vec2(1.f, 0.f), 0.f),
        glm::normalize(simFrame::fromForwardRight(glm::vec2(0.3f, 0.9f), -0.4f)),
    };

    int ticks = 0;
    for (const glm::vec3& plane : planes)
        for (const glm::vec3& d : directions)
        {
            Rig withWorld(plane);
            Rig withoutWorld(plane);
            for (Rig* rig : { &withWorld, &withoutWorld })
            {
                rig->placeAtClearance(0.11f);
                rig->state.velocity = glm::vec3(1.f, -0.25f, 0.5f);
                rig->hitWorld(d);
            }
            withoutWorld.inbound.hitDirectionWorld = glm::vec3(0.f);
            for (std::uint32_t t = 1u; t <= 40u; ++t)
            {
                withWorld.tick<false>(t);
                withoutWorld.tick<false>(t);
                INFO("plane n.y " << plane.y << " d=(" << d.x << ", " << d.y << ", " << d.z << ") tick " << t
                     << " first differing field " << firstDifferingField(withWorld.state, withoutWorld.state));
                REQUIRE(sameState(withWorld.state, withoutWorld.state));
                REQUIRE(sameBits(withWorld.derived.surfaceNormal, withoutWorld.derived.surfaceNormal));
                REQUIRE(withWorld.derived.lastSupportState == withoutWorld.derived.lastSupportState);
                withWorld.engineStep();
                withoutWorld.engineStep();
                ++ticks;
            }
        }
    REQUIRE(ticks == 400);
}

#endif // WITH_LOW_LEVEL_TESTS
