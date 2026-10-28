// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/HitReaction.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include "catch_amalgamated.hpp"

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StorageView.h"
#include "OGSimulation/SurfaceFrame.h"

static_assert(std::is_aggregate_v<HitReactionSpec>);
static_assert(!Serializable<brawlerInboundHit::DerivedState>);
static_assert(static_cast<std::uint8_t>(HitDirectionSource::SwingTangent) == 0u);

namespace groundAlignedHitRoutingTests
{
    constexpr unsigned int kAttacker = 0u;
    constexpr unsigned int kTarget = 1u;

    const glm::vec3 kThirtyDegreeNormal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));

    struct Rig
    {
        simulatableBrawler::StaticData staticData;
        SimulationObjectStorage<SimulatableBrawler> storage;
        brawlerHitRouting::System system;

        Rig()
        {
            storage.add<SimulatableBrawler>(kAttacker, SimulatableBrawler(staticData));
            storage.add<SimulatableBrawler>(kTarget, SimulatableBrawler(staticData));
            brawler(kAttacker).setCharacterBindings({ BodyId{ 10u } });
            brawler(kTarget).setCharacterBindings({ BodyId{ 11u } });
            system.onCharacterRegistered(kAttacker, view(), staticData);
            system.onCharacterRegistered(kTarget, view(), staticData);
        }

        StorageView<SimulatableBrawler> view() { return storage.projectTo<SimulatableList<SimulatableBrawler>>(); }

        SimulatableBrawler& brawler(unsigned int id) { return storage.get<SimulatableBrawler>(id); }

        void setPositions(const glm::vec3& attacker, const glm::vec3& target)
        {
            brawler(kAttacker).editAllState().editState().edit<brawlerMovementSimulation::State>().bodyState.position = attacker;
            brawler(kTarget).editAllState().editState().edit<brawlerMovementSimulation::State>().bodyState.position = target;
        }

        void setAim(float angle, const glm::vec3& axis)
        {
            auto& ic = brawler(kAttacker).editAllState().editState().edit<dAttackRadialSimulation::InitialConditions>();
            ic.initialAimAngle = angle;
            ic.initialAimRotationAxis = axis;
        }

        void raiseMelee(unsigned int sequenceId, const glm::vec3& swingTangent)
        {
            brawler(kAttacker).editAllState().editState().edit<dAttackRadialSimulation::State>().currenSequenceId = sequenceId;
            dAttackRadialSimulation::DAttackHit hit{};
            hit.hitRootBodyId = BodyId{ 11u };
            hit.swingTangent = swingTangent;
            hit.targetId = static_cast<SimCharacterId>(kTarget);
            brawler(kAttacker).editAllState().editDerivedState().edit<dAttackRadialSimulation::DerivedState>()
                .editHitsThisTick().push_back(hit);
        }

        void raiseProjectile(const glm::vec3& spawnDir, bool targetFlinching)
        {
            if (targetFlinching)
                brawler(kTarget).editAllState().editState().edit<dAttackMachineSimulation::State>().m_currentState = DAttackState::HitFlinch;
            auto& slot = brawler(kAttacker).editAllState().editState().edit<brawlerProjectileSimulation::State>().slots[0];
            slot.spawnTick = 1u;
            slot.spawnDir = spawnDir;
            auto& detected = brawler(kAttacker).editAllState().editDerivedState()
                .edit<brawlerProjectileSimulation::DerivedState>().detectedThisTick[0];
            detected.outcome = brawlerProjectileSimulation::SlotOutcome::Hit;
            detected.struckRootBodyId = BodyId{ 11u };
        }

        const brawlerInboundHit::DerivedState& route()
        {
            system.preIntegrate(SimulationTimeStep(5u, false, false, false, 1.f / 60.f), view(), staticData);
            return brawler(kTarget).getAllState().getDerivedState().get<brawlerInboundHit::DerivedState>();
        }
    };

    glm::vec2 todayNormalisedXY(const glm::vec3& v, const glm::vec2& fallback)
    {
        const glm::vec2 xy = simFrame::planar(v);
        const float lengthSq = glm::dot(xy, xy);
        return lengthSq > 0.f ? xy * (1.f / glm::sqrt(lengthSq)) : fallback;
    }

    glm::vec2 todayAimXY(float angle, const glm::vec3& storedAxis)
    {
        constexpr glm::vec3 aimFallbackAxis = simFrame::withHeight(glm::vec3(0.f), -1.f);
        constexpr glm::vec4 forward4(simFrame::kForward, 0.f);
        const glm::vec3 axis = (glm::dot(storedAxis, storedAxis) > 0.f) ? storedAxis : aimFallbackAxis;
        const glm::vec3 aim = glm::vec3(glm::rotate(glm::mat4(1.f), angle, axis) * forward4);
        return todayNormalisedXY(aim, glm::vec2(1.f, 0.f));
    }

    glm::vec2 todayMeleeXY(const glm::vec3& tangent, const glm::vec3& attacker, const glm::vec3& target,
                           float angle, const glm::vec3& axis)
    {
        return todayNormalisedXY(tangent, todayNormalisedXY(target - attacker, todayAimXY(angle, axis)));
    }

    bool sameBits(const glm::vec2& a, const glm::vec2& b)
    {
        return std::bit_cast<std::uint32_t>(a.x) == std::bit_cast<std::uint32_t>(b.x)
            && std::bit_cast<std::uint32_t>(a.y) == std::bit_cast<std::uint32_t>(b.y);
    }

    bool sameBits(const glm::vec3& a, const glm::vec3& b)
    {
        return std::bit_cast<std::uint32_t>(a.x) == std::bit_cast<std::uint32_t>(b.x)
            && std::bit_cast<std::uint32_t>(a.y) == std::bit_cast<std::uint32_t>(b.y)
            && std::bit_cast<std::uint32_t>(a.z) == std::bit_cast<std::uint32_t>(b.z);
    }

    float maxAbsDifference(const glm::vec3& a, const glm::vec3& b)
    {
        const glm::vec3 d = glm::abs(a - b);
        return glm::max(d.x, glm::max(d.y, d.z));
    }

    glm::mat3 slopeAttackerFrame(float heading)
    {
        return surfaceFrame::tiltMatrix(kThirtyDegreeNormal) * glm::mat3(glm::rotate(glm::mat4(1.f), heading, simFrame::kUp));
    }

    HitReactionSpec authoredForwardUpRow()
    {
        return HitReactionSpec{ HitReactionKind::Knockback, 20.f, 0.f, HitDirectionSource::AuthoredInSwingFrame,
                                glm::normalize(simFrame::kForward + simFrame::kUp) };
    }
}

TEST_CASE("GroundAlignedHitRouting.ShippedRowsKeepTheSwingTangentSource", "[DAttack]")
{
    const simulatableBrawler::StaticData staticData;
    for (const HitReactionSpec& row : staticData.m_hitReactions)
    {
        CHECK(row.directionSource == HitDirectionSource::SwingTangent);
        CHECK(row.authoredDirection == glm::vec3(0.f));
    }
    const HitReactionSpec& overhead = staticData.m_hitReactions[dAttackDirection::kForwardSequenceId];
    CHECK(overhead.kind == HitReactionKind::Stun);
    CHECK(overhead.knockbackSpeed == 0.f);
    CHECK(overhead.lockoutDuration == 0.72f);
    CHECK(staticData.m_projectileHitReaction.directionSource == HitDirectionSource::SwingTangent);
    CHECK(staticData.m_projectileHitOnFlinchReaction.directionSource == HitDirectionSource::SwingTangent);
}

TEST_CASE("GroundAlignedHitRouting.MeleeKnockbackWritesTheNormalisedTangent", "[DAttack]")
{
    using namespace groundAlignedHitRoutingTests;
    const glm::vec3 attacker(0.f);
    const glm::vec3 target(1.f, 0.f, -1.f);
    const float aimAngle = 0.3f;
    const glm::vec3 aimAxis(0.f, 1.f, 0.f);

    const glm::vec3 tangents[] = {
        glm::vec3(0.6f, 0.f, -1.7f),
        slopeAttackerFrame(0.4f) * glm::vec3(-2.3f, 0.f, 0.f),
        slopeAttackerFrame(-1.9f) * glm::vec3(0.f, 0.f, 1.4f),
    };
    for (const glm::vec3& tangent : tangents)
    {
        Rig rig;
        rig.setPositions(attacker, target);
        rig.setAim(aimAngle, aimAxis);
        rig.raiseMelee(0u, tangent);
        const brawlerInboundHit::DerivedState& slice = rig.route();
        const glm::vec3 expected = tangent / std::sqrt(glm::dot(tangent, tangent));
        INFO("tangent " << tangent.x << ", " << tangent.y << ", " << tangent.z << ": |world - t/|t|| = "
            << maxAbsDifference(slice.hitDirectionWorld, expected));
        CHECK(slice.reactionKind == HitReactionKind::Knockback);
        CHECK(maxAbsDifference(slice.hitDirectionWorld, expected) <= 1.e-6f);
        CHECK(std::abs(glm::length(slice.hitDirectionWorld) - 1.f) <= 1.e-6f);
        CHECK(sameBits(slice.hitDirectionXY, todayMeleeXY(tangent, attacker, target, aimAngle, aimAxis)));
    }
}

TEST_CASE("GroundAlignedHitRouting.MeleeTangentWithNoGroundProjectionWritesTheHorizontalFallback", "[DAttack]")
{
    using namespace groundAlignedHitRoutingTests;
    const glm::vec3 attacker(0.f);
    const glm::vec3 target(-2.f, 0.5f, 0.7f);
    const float aimAngle = 0.3f;
    const glm::vec3 aimAxis(0.f, 1.f, 0.f);
    const glm::vec3 vertical = simFrame::withHeight(glm::vec3(0.f), 3.f);

    Rig rig;
    rig.setPositions(attacker, target);
    rig.setAim(aimAngle, aimAxis);
    rig.raiseMelee(2u, vertical);
    const brawlerInboundHit::DerivedState& slice = rig.route();
    const glm::vec2 xy = todayMeleeXY(vertical, attacker, target, aimAngle, aimAxis);
    CHECK(sameBits(slice.hitDirectionXY, xy));
    CHECK(sameBits(slice.hitDirectionWorld, simFrame::compose(xy, 0.f)));
    CHECK(simFrame::height(slice.hitDirectionWorld) == 0.f);
}

TEST_CASE("GroundAlignedHitRouting.StunWritesAZeroDirection", "[DAttack]")
{
    using namespace groundAlignedHitRoutingTests;
    {
        Rig rig;
        rig.setPositions(glm::vec3(0.f), glm::vec3(1.f, 0.f, -1.f));
        rig.setAim(0.3f, glm::vec3(0.f, 1.f, 0.f));
        rig.raiseMelee(dAttackDirection::kForwardSequenceId, slopeAttackerFrame(0.4f) * glm::vec3(0.f, -1.f, -0.5f));
        const brawlerInboundHit::DerivedState& slice = rig.route();
        CHECK(slice.wasHitThisTick);
        CHECK(slice.reactionKind == HitReactionKind::Stun);
        CHECK(sameBits(slice.hitDirectionWorld, glm::vec3(0.f)));
        CHECK(sameBits(slice.hitDirectionXY, glm::vec2(0.f)));
    }
    {
        Rig rig;
        rig.staticData.m_hitReactions[dAttackDirection::kForwardSequenceId].directionSource = HitDirectionSource::AuthoredInSwingFrame;
        rig.staticData.m_hitReactions[dAttackDirection::kForwardSequenceId].authoredDirection = glm::normalize(simFrame::kForward + simFrame::kUp);
        rig.setPositions(glm::vec3(0.f), glm::vec3(1.f, 0.f, -1.f));
        rig.raiseMelee(dAttackDirection::kForwardSequenceId, glm::vec3(0.6f, 0.f, -1.7f));
        const brawlerInboundHit::DerivedState& slice = rig.route();
        CHECK(slice.reactionKind == HitReactionKind::Stun);
        CHECK(sameBits(slice.hitDirectionWorld, glm::vec3(0.f)));
    }
    {
        Rig rig;
        rig.setPositions(glm::vec3(0.f), glm::vec3(1.f, 0.f, -1.f));
        rig.raiseProjectile(glm::normalize(glm::vec3(0.3f, 0.5f, -0.8f)), false);
        const brawlerInboundHit::DerivedState& slice = rig.route();
        CHECK(slice.wasHitThisTick);
        CHECK(slice.reactionKind == HitReactionKind::Stun);
        CHECK(sameBits(slice.hitDirectionWorld, glm::vec3(0.f)));
    }
}

TEST_CASE("GroundAlignedHitRouting.ProjectileOnFlinchKnockbackWritesTheSpawnDirection", "[DAttack]")
{
    using namespace groundAlignedHitRoutingTests;
    const glm::vec3 spawnDirs[] = {
        glm::normalize(glm::vec3(0.3f, 0.5f, -0.8f)),
        slopeAttackerFrame(2.2f) * simFrame::kForward,
        simFrame::withHeight(glm::vec3(0.f), 1.f),
    };
    for (const bool authoredOnTheProjectileRow : { false, true })
    {
        for (const glm::vec3& spawnDir : spawnDirs)
        {
            Rig rig;
            if (authoredOnTheProjectileRow)
            {
                rig.staticData.m_projectileHitOnFlinchReaction.directionSource = HitDirectionSource::AuthoredInSwingFrame;
                rig.staticData.m_projectileHitOnFlinchReaction.authoredDirection = glm::normalize(simFrame::kForward + simFrame::kUp);
            }
            rig.setPositions(glm::vec3(0.f), glm::vec3(1.f, 0.f, -1.f));
            rig.raiseProjectile(spawnDir, true);
            const brawlerInboundHit::DerivedState& slice = rig.route();
            INFO("spawnDir " << spawnDir.x << ", " << spawnDir.y << ", " << spawnDir.z << ", authored row " << authoredOnTheProjectileRow
                << ": |world - spawnDir| = " << maxAbsDifference(slice.hitDirectionWorld, spawnDir));
            CHECK(slice.reactionKind == HitReactionKind::Knockback);
            CHECK(maxAbsDifference(slice.hitDirectionWorld, spawnDir) <= 1.e-6f);
            CHECK(sameBits(slice.hitDirectionXY, todayNormalisedXY(spawnDir, glm::vec2(1.f, 0.f))));
        }
    }
    {
        Rig rig;
        rig.setPositions(glm::vec3(0.f), glm::vec3(1.f, 0.f, -1.f));
        rig.raiseProjectile(glm::vec3(0.f), true);
        const brawlerInboundHit::DerivedState& slice = rig.route();
        CHECK(sameBits(slice.hitDirectionXY, glm::vec2(1.f, 0.f)));
        CHECK(sameBits(slice.hitDirectionWorld, simFrame::compose(glm::vec2(1.f, 0.f), 0.f)));
    }
}

TEST_CASE("GroundAlignedHitRouting.AuthoredRowRotatesIntoTheAttackersGroundFrame", "[DAttack]")
{
    using namespace groundAlignedHitRoutingTests;
    const glm::vec3 n = kThirtyDegreeNormal;
    const glm::vec3 attacker(0.f);
    const glm::vec3 target(1.f, 0.f, -1.f);
    const glm::vec3 tangent(0.6f, 0.f, -1.7f);
    const float headings[] = { 0.f, 0.4f, -1.9f, 2.9f };
    for (const float heading : headings)
    {
        const glm::mat3 frame = slopeAttackerFrame(heading);
        const surfaceFrame::AngleAxis ic = surfaceFrame::toAngleAxis(frame);
        const glm::vec3 icUp = glm::mat3(glm::rotate(glm::mat4(1.f), ic.angle, ic.axis)) * simFrame::kUp;
        REQUIRE(maxAbsDifference(icUp, n) <= 1.e-6f);

        Rig rig;
        rig.staticData.m_hitReactions[0] = authoredForwardUpRow();
        rig.setPositions(attacker, target);
        rig.setAim(ic.angle, ic.axis);
        rig.raiseMelee(0u, tangent);
        const brawlerInboundHit::DerivedState& slice = rig.route();

        const glm::vec3 expected = glm::normalize(frame * glm::normalize(simFrame::kForward + simFrame::kUp));
        const glm::vec3 aimInPlane = frame * simFrame::kForward;
        INFO("heading " << heading << ": |world - R_ic dir| = " << maxAbsDifference(slice.hitDirectionWorld, expected)
            << ", world . n = " << glm::dot(slice.hitDirectionWorld, n)
            << ", world . aim = " << glm::dot(slice.hitDirectionWorld, aimInPlane));
        CHECK(slice.reactionKind == HitReactionKind::Knockback);
        CHECK(maxAbsDifference(slice.hitDirectionWorld, expected) <= 1.e-6f);
        CHECK(std::abs(glm::dot(slice.hitDirectionWorld, n) - std::sqrt(0.5f)) <= 1.e-6f);
        CHECK(std::abs(glm::dot(slice.hitDirectionWorld, aimInPlane) - std::sqrt(0.5f)) <= 1.e-6f);
        CHECK(sameBits(slice.hitDirectionXY, todayMeleeXY(tangent, attacker, target, ic.angle, ic.axis)));
    }
}

TEST_CASE("GroundAlignedHitRouting.DirectionXYIsTodaysOnEveryFallbackPath", "[DAttack]")
{
    using namespace groundAlignedHitRoutingTests;
    const float c = std::cos(glm::radians(30.f));
    const float s = std::sin(glm::radians(30.f));
    struct Case
    {
        const char* name;
        glm::vec3 tangent;
        glm::vec3 attacker;
        glm::vec3 target;
        float angle;
        glm::vec3 axis;
    };
    const Case cases[] = {
        { "positions",         glm::vec3(0.f),              glm::vec3(0.f),             glm::vec3(-2.f, 0.5f, 0.7f), 0.3f, glm::vec3(0.f, 1.f, 0.f) },
        { "aim -Y",            glm::vec3(0.f),              glm::vec3(1.f, 0.f, 1.f),   glm::vec3(1.f, 0.f, 1.f),    0.7f, glm::vec3(-0.f, -1.f, 0.f) },
        { "aim zero axis",     glm::vec3(0.f),              glm::vec3(1.f, 0.f, 1.f),   glm::vec3(1.f, 0.f, 1.f),    0.7f, glm::vec3(0.f) },
        { "aim tilted axis",   glm::vec3(0.f),              glm::vec3(1.f, 0.f, 1.f),   glm::vec3(1.f, 0.f, 1.f),    1.1f, glm::vec3(s, c, 0.f) },
        { "vertical tangent",  glm::vec3(0.f, 3.f, 0.f),    glm::vec3(1.f, 0.f, 1.f),   glm::vec3(1.f, 0.f, 1.f),    1.1f, glm::vec3(s, c, 0.f) },
    };
    for (const bool authored : { false, true })
    {
        for (const Case& k : cases)
        {
            Rig rig;
            if (authored)
                rig.staticData.m_hitReactions[1] = authoredForwardUpRow();
            rig.setPositions(k.attacker, k.target);
            rig.setAim(k.angle, k.axis);
            rig.raiseMelee(1u, k.tangent);
            const brawlerInboundHit::DerivedState& slice = rig.route();
            const glm::vec2 expected = todayMeleeXY(k.tangent, k.attacker, k.target, k.angle, k.axis);
            INFO(k.name << ", authored row " << authored << ": xy = " << slice.hitDirectionXY.x << ", " << slice.hitDirectionXY.y
                << ", today = " << expected.x << ", " << expected.y);
            CHECK(sameBits(slice.hitDirectionXY, expected));
            CHECK(std::abs(glm::length(slice.hitDirectionWorld) - 1.f) <= 1.e-6f);
        }
    }
}

#endif
