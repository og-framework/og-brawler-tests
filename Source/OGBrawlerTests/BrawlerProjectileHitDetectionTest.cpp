// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================
// PROJECTILE HIT DETECTION MUST NOT DEPEND ON WHICH CHARACTER INTEGRATES FIRST
// [og-netcode-v2-field-defects task 17]
//
// SUBJECT: the projectile's cross-character reads, which task 9 left inside the SHOOTER's
// integrate: the overlap of each live slot (it sees the target's guard shape only if the target's
// integrate has already enabled it this tick), the target's guard transform, and the other
// projectiles' bodies (the projectile-vs-projectile cancel). Task 17 moves all three into
// brawlerHitDetection::System's preIntegrate walk.
//
// THE RIG. Two production SimulatableBrawlers in the production storage and the production systems
// executor (detection, then routing). One tick is `pre(t)`, both integrates in the order the case
// chooses (either may be left out, which is what a resim NoSlot row does), then `post(t)`. A sample
// is taken after `post(t)`: the end of tick t.
//   * `ProjectilePhysics` records every body transform written and returns it on read, so the
//     target's guard transform and the projectile bodies are where the sub-simulations put them.
//     Every projectile body starts PARKED, as the pool leaves an unused slot.
//   * `ProjectileQuery` records each query volume's parent transform, tracks the target guard
//     shape's enable bit, and answers a projectile volume's overlap with:
//       - the TARGET (brawler 1), for a volume of the SHOOTER's (brawler 0's), while
//         `targetContact` is set: its guard (if enabled) FIRST, then its body.
//         A guard facing the shot stands in front of the body; the projectile takes the FIRST
//         non-parent entry, so the order is the rig's model of that.
//         [og-ground-aligned-attacks task 20b] The order no longer decides a block: the struck
//         character's guard entry decides wherever it is listed (BrawlerProjectileHitDetection.h
//         `guardEntryOf`). The cases at the end of this file run both orders on a scripted report.
//       - every OTHER projectile body within `projectileContact` of the volume that is not parked.
//         Body transforms are read as last written, so a body parked mid-pass disappears from
//         every later query: the physics-thread write model.
//   Contact with the target is SCHEDULED, as in every melee rig here. Projectile-vs-projectile
//   contact is a distance, because both positions are closed-form.
//
// These cases are written against APIs both the pre-task and the post-task tree compile
// (the slot's wire fields, the inbound slice, the machine), so the same file measures RED and GREEN.
//
// TAGS: `[SimulatableBrawler]` is in the `[@og]` whitelist; `[HitDetection]` is not.
// ============================================================================

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerProjectileHitDetection.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "BrawlerTestInputs.h"
#include "BrawlerSimulationMocks.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SimulationFrame.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"

namespace projectileDetectionTests
{

constexpr float kDt = 1.f / 60.f;

constexpr std::uint32_t kShooterRoot      = 20u;   // brawler 0
constexpr std::uint32_t kTargetRoot       = 21u;   // brawler 1
constexpr std::uint32_t kTargetGuardBody  = 22u;
constexpr std::uint32_t kTargetGuardShape = 23u;
constexpr std::uint32_t kSlots            =
    static_cast<std::uint32_t>(brawlerProjectileSimulation::kMaxProjectilePoolSize);
// Brawler b's slot k: query volume kVolume + 3b + k, body kBody + 3b + k.
constexpr std::uint32_t kVolume = 60u;
constexpr std::uint32_t kBody   = 70u;

inline BodyId        projectileBody(unsigned int brawler, std::uint32_t slot)   { return BodyId{ kBody + kSlots * brawler + slot }; }
inline QueryVolumeId projectileVolume(unsigned int brawler, std::uint32_t slot) { return QueryVolumeId{ kVolume + kSlots * brawler + slot }; }
inline BodyId        rootOf(unsigned int brawler) { return BodyId{ brawler == 0u ? kShooterRoot : kTargetRoot }; }

// A guard that faces the shooter exactly when the shot travels along -blockingAim().
inline glm::vec3 blockingAim()   { return glm::vec3(-std::sin(0.5f), 0.f, std::cos(0.5f)); }
inline glm::vec3 shotDirection() { return -blockingAim(); }
inline glm::vec3 turnedAwayAim() { return glm::vec3(0.f, 0.f, -1.f); }

inline bool isParked(const glm::mat4& m) { return simFrame::height(glm::vec3(m[3])) < -10.f; }

struct ProjectilePhysics
{
    std::map<std::uint32_t, glm::mat4> transforms;

    glm::mat4 getBodyTransform(BodyId id) const
    {
        const auto it = transforms.find(id.value);
        return it == transforms.end() ? glm::mat4(1.f) : it->second;
    }
    void setBodyTransform(BodyId id, const glm::mat4& m)  { transforms[id.value] = m; }
    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
    bool isBodyResolvable(BodyId) const                   { return true; }
};
static_assert(PhysicsBodyAdapter<ProjectilePhysics>);

struct ProjectileQuery
{
    const ProjectilePhysics* physics = nullptr;
    bool  targetContact     = false;
    bool  guardShapeEnabled = false;
    float projectileContact = 0.f;
    std::map<std::uint32_t, glm::mat4> volumeParents;
    std::vector<std::pair<std::uint32_t, glm::mat4>> parentWrites;   // every call, in order
    mutable int projectileOverlapCalls = 0;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        SpatialQueryReport report;
        if (volumeIds.size() != 1u)
            return report;
        const std::uint32_t volume = volumeIds[0].value;
        if (volume < kVolume || volume >= kVolume + 2u * kSlots)
            return report;
        ++projectileOverlapCalls;
        const std::uint32_t ownBody = kBody + (volume - kVolume);
        const auto parent = volumeParents.find(volume);
        const glm::vec3 at = parent == volumeParents.end() ? glm::vec3(0.f) : glm::vec3(parent->second[3]);

        // Only the SHOOTER's (brawler 0's) shots reach the target; brawler 1 IS the target.
        if (targetContact && volume < kVolume + kSlots)
        {
            SpatialQueryHit body{};
            body.objectPosition   = glm::vec3(0.f, 0.5f, -0.f);
            body.bodyId           = BodyId{ kTargetRoot };
            body.rootBodyId       = BodyId{ kTargetRoot };
            body.objectCategories = CollisionCategories::single(collisionCategory::body);
            if (guardShapeEnabled)
            {
                SpatialQueryHit guard = body;
                guard.bodyId           = BodyId{ kTargetGuardBody };
                guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
                report.hits.push_back(guard);
            }
            report.hits.push_back(body);
        }

        for (std::uint32_t b = kBody; b < kBody + 2u * kSlots; ++b)
        {
            if (b == ownBody)
                continue;
            const glm::mat4 m = physics->getBodyTransform(BodyId{ b });
            if (isParked(m))
                continue;
            const glm::vec3 p(m[3]);
            if (glm::length(p - at) > projectileContact)
                continue;
            SpatialQueryHit projectile{};
            projectile.objectPosition   = p;
            projectile.bodyId           = BodyId{ b };
            projectile.rootBodyId       = rootOf((b - kBody) / kSlots);
            projectile.objectCategories = CollisionCategories::single(collisionCategory::projectile);
            report.hits.push_back(projectile);
        }
        return report;
    }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId id, const glm::mat4& m)
    {
        volumeParents[id.value] = m;
        parentWrites.emplace_back(id.value, m);
    }
    void enableShape(ShapeId id)  { if (id == ShapeId{ kTargetGuardShape }) guardShapeEnabled = true; }
    void disableShape(ShapeId id) { if (id == ShapeId{ kTargetGuardShape }) guardShapeEnabled = false; }
};
static_assert(SpatialQueryAdapter<ProjectileQuery>);

using Detection = brawlerHitDetection::System<ProjectilePhysics, ProjectileQuery>;

enum class Order { TargetFirst, ShooterFirst };

struct Sample
{
    DAttackState  shooterMachine = DAttackState::Idle;
    DAttackState  targetMachine  = DAttackState::Idle;
    std::uint32_t endTick[2]     = { 0u, 0u };   // slot 0 of brawler 0 and of brawler 1
    unsigned int  endReason[2]   = { 0u, 0u };
    bool          targetHitAfterPre     = false;  // routed onto the target's slice by pre(t)
    // [og-attackstatetransition-cleanup task 4] `shooterBlockedAfterPre` is gone with
    // wasProjectileBlockedThisTick: a block routes nothing onto the shooter's slice any more.
    bool          shooterHitAfterPre    = false;
};

struct Rig
{
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
        simulatableBrawler::StaticData, Detection, brawlerHitRouting::System>;

    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    ProjectilePhysics phys;
    ProjectileQuery   query;
    Exec              exec{ std::piecewise_construct, Detection(phys, query), brawlerHitRouting::System{} };

    Order order = Order::TargetFirst;
    bool  stunTargetOnTickZero = false;
    glm::vec3 targetAim = blockingAim();
    bool  targetHoldsGuard = true;
    std::vector<Sample> samples;   // index == tick, for the ticks run() stepped

    Rig()
    {
        staticData.m_movementStaticData.drivesBody = false;
        query.physics = &phys;
        // The projectile's collider against a projectile body (PhysicsSetup::body, a 30 cm sphere).
        query.projectileContact = staticData.m_projectileStaticData.colliderRadius + 0.3f;

        storage.add<SimulatableBrawler>(0u, SimulatableBrawler(staticData));
        storage.add<SimulatableBrawler>(1u, SimulatableBrawler(staticData));
        brawler(0u).setCharacterBindings({ BodyId{ kShooterRoot } });
        brawler(1u).setCharacterBindings({ BodyId{ kTargetRoot } });

        auto& guard = brawler(1u).editPhysicsComposite()
            .edit<dAttackGuardSimulation::PhysicsDeclaration>().bindings;
        guard.ownBodyId    = BodyId{ kTargetGuardBody };
        guard.parentBodyId = BodyId{ kTargetRoot };
        guard.shapeIds     = { ShapeId{ kTargetGuardShape } };

        for (unsigned int b = 0u; b < 2u; ++b)
        {
            bindSlot(brawler(b).editPhysicsComposite()
                .edit<brawlerProjectileSimulation::PhysicsDeclaration<0>>(), b, 0u);
            bindSlot(brawler(b).editPhysicsComposite()
                .edit<brawlerProjectileSimulation::PhysicsDeclaration<1>>(), b, 1u);
            bindSlot(brawler(b).editPhysicsComposite()
                .edit<brawlerProjectileSimulation::PhysicsDeclaration<2>>(), b, 2u);
            for (std::uint32_t k = 0u; k < kSlots; ++k)
                park(projectileBody(b, k));
        }

        exec.notifyCharacterRegistered(0u, storage, staticData, true);
        exec.notifyCharacterRegistered(1u, storage, staticData, true);
    }

    template <typename Decl>
    static void bindSlot(Decl& declaration, unsigned int b, std::uint32_t slot)
    {
        declaration.bindings.ownBodyId      = projectileBody(b, slot);
        declaration.bindings.parentBodyId   = rootOf(b);
        declaration.bindings.queryVolumeIds = { projectileVolume(b, slot) };
    }

    void park(BodyId body)
    {
        glm::mat4 m(1.f);
        m[3] = glm::vec4(simFrame::withHeight(glm::vec3(0.f), brawlerProjectileSimulation::kParkHeight), 1.f);
        phys.setBodyTransform(body, m);
    }

    SimulatableBrawler& brawler(unsigned int id) { return storage.get<SimulatableBrawler>(id); }

    // Launch slot 0 of `b` as the pool's spawn does on `spawnTick`: the wire slot, and the body
    // snapped to the launch position.
    void launch(unsigned int b, std::uint32_t spawnTick, glm::vec3 spawnPos, glm::vec3 spawnDir)
    {
        auto& slot = brawler(b).editAllState().editState()
            .edit<brawlerProjectileSimulation::State>().slots[0];
        slot.spawnTick = spawnTick;
        slot.spawnPos  = spawnPos;
        slot.spawnDir  = spawnDir;
        slot.endTick   = 0u;
        slot.endReason = 0u;
        glm::mat4 m(1.f);
        m[3] = glm::vec4(spawnPos, 1.f);
        phys.setBodyTransform(projectileBody(b, 0u), m);
    }

    static simulatableBrawler::PlayerInput makeInput(const glm::vec3& aim, std::uint8_t flags)
    {
        return brawlerTestInputs::make({ .aimDirection = aim, .flags = flags });
    }

    // The stun the field capture opens with, delivered the way routing delivers it: tick 0's
    // pre-integrate pass has reset the slice, so it is seeded after that pass and before the
    // machine reads it.
    void seedStun()
    {
        const HitReactionSpec& stun = staticData.m_hitReactions[dAttackDirection::kForwardSequenceId];
        auto& slice = brawler(1u).editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>();
        slice.wasHitThisTick = true;
        slice.reactionKind   = stun.kind;
        slice.flinchDuration = stun.lockoutDuration;
    }

    const brawlerInboundHit::DerivedState& inbound(unsigned int id)
    {
        return brawler(id).getAllState().getDerivedState().get<brawlerInboundHit::DerivedState>();
    }

    // One step: pre(t), the integrates (in `order`, skipping any left out), post(t).
    void step(std::uint32_t t, bool integrateShooter = true, bool integrateTarget = true,
              StepKind kind = StepKind::Normal, bool resim = false)
    {
        const SimulationTimeStep s(t, resim, kind, kDt);
        exec.firePreIntegrate(s, storage, staticData, /*isAuthority*/ !resim);
        Sample sample{};
        sample.targetHitAfterPre      = inbound(1u).wasHitThisTick;
        sample.shooterHitAfterPre     = inbound(0u).wasHitThisTick;
        if (t == 0u && stunTargetOnTickZero)
            seedStun();

        const auto shooterInput = makeInput(glm::vec3(0.f, 0.f, -1.f), 0u);
        const auto targetInput  = makeInput(targetAim,
            targetHoldsGuard ? brawlerMovementSimulation::kInputFlagHoldGuard : std::uint8_t(0u));
        auto integrateShooterNow = [&]()
        { if (integrateShooter) brawler(0u).integrate(s, shooterInput, phys, query, staticData); };
        auto integrateTargetNow = [&]()
        { if (integrateTarget) brawler(1u).integrate(s, targetInput, phys, query, staticData); };
        if (order == Order::TargetFirst) { integrateTargetNow(); integrateShooterNow(); }
        else                             { integrateShooterNow(); integrateTargetNow(); }

        exec.firePostIntegrate(s, storage, staticData, /*isAuthority*/ !resim);

        const auto& shooterState = brawler(0u).getAllState().getState();
        const auto& targetState  = brawler(1u).getAllState().getState();
        sample.shooterMachine = shooterState.get<dAttackMachineSimulation::State>().m_currentState;
        sample.targetMachine  = targetState.get<dAttackMachineSimulation::State>().m_currentState;
        const auto& a = shooterState.get<brawlerProjectileSimulation::State>().slots[0];
        const auto& b = targetState.get<brawlerProjectileSimulation::State>().slots[0];
        sample.endTick[0] = a.endTick;  sample.endReason[0] = a.endReason;
        sample.endTick[1] = b.endTick;  sample.endReason[1] = b.endReason;
        if (samples.size() <= t)
            samples.resize(t + 1u);
        samples[t] = sample;
    }

    const Sample& at(std::uint32_t t) const { return samples.at(t); }

    int routedTargetHits() const
    {
        int n = 0;
        for (const Sample& s : samples) if (s.targetHitAfterPre) ++n;
        return n;
    }
};

inline const char* name(DAttackState s) { return dAttackMachineSimulation::dAttackStateName(s); }

// The target alone, stunned on tick 0 and holding its guard: the first tick its machine reads Idle.
inline std::uint32_t measureFirstIdleTick()
{
    Rig rig;
    rig.stunTargetOnTickZero = true;
    for (std::uint32_t t = 0u; t < 200u; ++t)
    {
        rig.step(t);
        if (t > 0u && rig.at(t).targetMachine == DAttackState::Idle)
            return t;
    }
    return 0u;
}

struct ArmResult
{
    unsigned int  endReason = 0u;
    std::uint32_t endTick   = 0u;
    DAttackState  targetAtC   = DAttackState::Idle;
    DAttackState  targetAtC1  = DAttackState::Idle;
    DAttackState  shooterAtC  = DAttackState::Idle;
    DAttackState  shooterAtC1 = DAttackState::Idle;
    int           routedHits  = 0;
    int           blocksEnded = 0;   // ticks on which the shooter's slot ended BLOCKED (endReason 4); was routedBlocks
    std::string   describe() const
    {
        return "endReason=" + std::to_string(endReason) + " endTick=" + std::to_string(endTick)
             + " target C/C+1=" + name(targetAtC) + "/" + name(targetAtC1)
             + " shooter C/C+1=" + name(shooterAtC) + "/" + name(shooterAtC1)
             + " routedHits=" + std::to_string(routedHits)
             + " blocksEnded=" + std::to_string(blocksEnded);
    }
};

// The shot is in flight from well before C; contact with the target is scheduled from C on.
inline ArmResult runOrderSwapArm(Order order, std::uint32_t C)
{
    Rig rig;
    rig.order = order;
    rig.stunTargetOnTickZero = true;
    const std::uint32_t spawn = C - 5u;
    for (std::uint32_t t = 0u; t <= C + 3u; ++t)
    {
        rig.query.targetContact = t >= C;
        rig.step(t);
        if (t == spawn)
            rig.launch(0u, spawn, glm::vec3(0.f, 0.5f, 4.f), shotDirection());
    }
    ArmResult r;
    r.endReason   = rig.at(C + 3u).endReason[0];
    r.endTick     = rig.at(C + 3u).endTick[0];
    r.targetAtC   = rig.at(C).targetMachine;
    r.targetAtC1  = rig.at(C + 1u).targetMachine;
    r.shooterAtC  = rig.at(C).shooterMachine;
    r.shooterAtC1 = rig.at(C + 1u).shooterMachine;
    for (std::uint32_t t = 0u; t < rig.samples.size(); ++t)
    {
        const Sample& s = rig.samples[t];
        if (s.targetHitAfterPre) ++r.routedHits;
        if (s.endTick[0] == t && s.endReason[0] == 4u) ++r.blocksEnded;
    }
    return r;
}

} // namespace projectileDetectionTests

using namespace projectileDetectionTests;

// ---------------------------------------------------------------------------
// ⭐ THE ORDER-SWAP LLT (task 17 AC 1). The projectile reaches the target on the target's FIRST
// Idle tick after a stun, the guard held and facing the shot. Both integrate orders must give the
// same slot outcome and the same routed reaction.
// PRE-TASK (RED, "today: 2 vs 4"): target-first, the target's integrate enables the guard before
// the shooter's integrate queries -> blocked (4); shooter-first, the guard is still off -> hit (2).
// TASK 17: detection runs in pre(C) against the state integrate(C-1) left, where the target is still
// stunned and its guard off: a HIT in both arms, reacted to in integrate(C). That is melee's
// convention for the same physical moment (the reaction reads the world as the previous tick's
// physics step left it).
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Projectile.StunExitTickOutcomeIsIndependentOfIntegrateOrder",
          "[SimulatableBrawler][HitDetection]")
{
    const std::uint32_t firstIdle = measureFirstIdleTick();
    INFO("target's first Idle tick after the stun = " << firstIdle);
    REQUIRE(firstIdle > 6u);

    const ArmResult targetFirst  = runOrderSwapArm(Order::TargetFirst,  firstIdle);
    const ArmResult shooterFirst = runOrderSwapArm(Order::ShooterFirst, firstIdle);
    INFO("arm 1 (target first):  " << targetFirst.describe());
    INFO("arm 2 (shooter first): " << shooterFirst.describe());

    CHECK(targetFirst.endReason   == shooterFirst.endReason);
    CHECK(targetFirst.endTick     == shooterFirst.endTick);
    CHECK(targetFirst.targetAtC   == shooterFirst.targetAtC);
    CHECK(targetFirst.targetAtC1  == shooterFirst.targetAtC1);
    CHECK(targetFirst.shooterAtC1 == shooterFirst.shooterAtC1);
    CHECK(targetFirst.routedHits  == shooterFirst.routedHits);
    // The outcome itself: a hit on C, reacted to in integrate(C).
    CHECK(targetFirst.endReason  == 2u);
    CHECK(targetFirst.endTick    == firstIdle);
    CHECK(targetFirst.targetAtC  == DAttackState::HitFlinch);
    CHECK(shooterFirst.targetAtC == DAttackState::HitFlinch);
    CHECK(targetFirst.routedHits == 1);
    CHECK(targetFirst.blocksEnded == 0);
}

// One tick later the guard is up in the state integrate(C) left, so both arms BLOCK, and the
// shooter recoils in the same step. [og-attackstatetransition-cleanup task 4] It no longer recoils
// (machine G-04 retired): the shooter stays Idle, and the block shows only as the slot's endReason 4. Not RED on the pre-task tree (the guard is up for both orders
// there too); it pins the block side of the same convention.
TEST_CASE("HitDetection.Projectile.TheTickAfterTheStunExitBlocksInBothOrders",
          "[SimulatableBrawler][HitDetection]")
{
    const std::uint32_t firstIdle = measureFirstIdleTick();
    REQUIRE(firstIdle > 6u);
    const std::uint32_t C = firstIdle + 1u;

    const ArmResult targetFirst  = runOrderSwapArm(Order::TargetFirst,  C);
    const ArmResult shooterFirst = runOrderSwapArm(Order::ShooterFirst, C);
    INFO("arm 1 (target first):  " << targetFirst.describe());
    INFO("arm 2 (shooter first): " << shooterFirst.describe());

    CHECK(targetFirst.endReason == 4u);
    CHECK(shooterFirst.endReason == 4u);
    CHECK(targetFirst.endTick == C);
    CHECK(shooterFirst.endTick == C);
    CHECK(targetFirst.blocksEnded == 1);
    CHECK(shooterFirst.blocksEnded == 1);
    CHECK(targetFirst.routedHits == 0);
    CHECK(shooterFirst.shooterAtC1 == targetFirst.shooterAtC1);
    CHECK(targetFirst.shooterAtC1 == DAttackState::Idle);
}

// ---------------------------------------------------------------------------
// Projectile-vs-projectile cancel (task 17 AC 2, brief Hazard 3). Two shots from DIFFERENT
// characters fly head-on; both must end with the cancel outcome (3) on the SAME tick, in both
// integrate orders, and neither may register a hit.
// PRE-TASK (measured): whichever shooter integrated first cancelled and PARKED its body; the second
// shooter's query then no longer saw it, so its shot flew on. Order-dependent.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Projectile.CancelEndsBothShotsOnTheSameTickInBothOrders",
          "[SimulatableBrawler][HitDetection]")
{
    struct CancelResult { Sample last; int routedTargetHits = 0; std::uint32_t firstContact = 0u; };
    // Shot A from brawler 0 along +X, shot B from brawler 1 along -X, launched together. The
    // first contact tick is where a slot's query at its closed-form position first reaches the
    // other shot's body at the position its last integrate snapped it to: the same separation for
    // either shooter, and in either tree.
    auto runCancel = [](Order order, bool shotAlsoReachesTheTarget)
    {
        Rig rig;
        rig.order = order;
        rig.targetHoldsGuard = false;
        rig.targetAim = turnedAwayAim();
        constexpr std::uint32_t spawn = 10u;
        constexpr float halfGap = 2.f;
        const float stepLength = rig.staticData.m_projectileStaticData.projectileSpeed * kDt;
        CancelResult result;
        for (std::uint32_t t = spawn + 1u; t < spawn + 60u && result.firstContact == 0u; ++t)
        {
            const float separation = 2.f * halfGap - stepLength * float(2u * (t - spawn) - 1u);
            if (separation <= rig.query.projectileContact)
                result.firstContact = t;
        }
        for (std::uint32_t t = 0u; t <= 60u; ++t)
        {
            rig.query.targetContact = shotAlsoReachesTheTarget && t >= result.firstContact;
            rig.step(t);
            if (t == spawn)
            {
                rig.launch(0u, spawn, glm::vec3(0.f, 0.5f, halfGap), glm::vec3(0.f, 0.f, -1.f));
                rig.launch(1u, spawn, glm::vec3(0.f, 0.5f, -halfGap), glm::vec3(0.f, 0.f, 1.f));
            }
        }
        result.last = rig.samples.back();
        result.routedTargetHits = rig.routedTargetHits();
        return result;
    };

    for (const bool alsoTarget : { false, true })
    {
        const CancelResult targetFirst  = runCancel(Order::TargetFirst,  alsoTarget);
        const CancelResult shooterFirst = runCancel(Order::ShooterFirst, alsoTarget);
        auto describe = [](const CancelResult& r)
        {
            const Sample& s = r.last;
            return "slot A end=" + std::to_string(s.endTick[0]) + "/" + std::to_string(s.endReason[0])
                 + " slot B end=" + std::to_string(s.endTick[1]) + "/" + std::to_string(s.endReason[1])
                 + " routed target hits=" + std::to_string(r.routedTargetHits);
        };
        INFO((alsoTarget ? "shot A's report also names the target, FIRST" : "head-on only")
             << "; first contact tick " << targetFirst.firstContact);
        INFO("target first:  " << describe(targetFirst));
        INFO("shooter first: " << describe(shooterFirst));
        REQUIRE(targetFirst.firstContact > 10u);
        for (const CancelResult* r : { &targetFirst, &shooterFirst })
        {
            const Sample& s = r->last;
            CHECK(s.endReason[0] == 3u);
            CHECK(s.endReason[1] == 3u);
            CHECK(s.endTick[0] == r->firstContact);
            CHECK(s.endTick[1] == r->firstContact);
            CHECK(r->routedTargetHits == 0);
        }
    }
}

// ---------------------------------------------------------------------------
// HardResync (task 17 AC; review_defect_20_c1.md §1.3). The prediction clock can teleport the
// frontier forward; SimulationManager builds the first post-jump step as Normal, so no system can
// see the jump. A shot whose contact tick is the pre-jump tick L must be routed EXACTLY ONCE.
// PRE-TASK (RED): the slot ended in integrate(L); routing matched `endTick == stepTick - 1` in
// pre(X) with X > L + 1, so it was never routed.
// TASK 17: detected and routed in pre(L), reacted to in integrate(L); the jump changes nothing.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Projectile.AShotEndingOnThePreJumpTickIsRoutedOnceAcrossAHardResync",
          "[SimulatableBrawler][HitDetection]")
{
    Rig rig;
    rig.targetHoldsGuard = false;
    rig.targetAim = turnedAwayAim();
    constexpr std::uint32_t L = 30u;
    constexpr std::uint32_t X = L + 6u;   // the jump
    for (std::uint32_t t = 0u; t <= L; ++t)
    {
        rig.query.targetContact = t >= L;
        rig.step(t);
        if (t == L - 4u)
            rig.launch(0u, L - 4u, glm::vec3(0.f, 0.5f, 4.f), glm::vec3(0.f, 0.f, -1.f));
    }
    rig.query.targetContact = false;
    for (std::uint32_t t = X; t <= X + 2u; ++t)
        rig.step(t);   // StepKind::Normal: what the manager hands systems after a HardResync

    bool flinched = false;
    for (const Sample& s : rig.samples) flinched = flinched || s.targetMachine == DAttackState::HitFlinch;
    INFO("slot end=" << rig.samples.back().endTick[0] << "/" << rig.samples.back().endReason[0]
         << " routed target hits=" << rig.routedTargetHits() << " target flinched=" << flinched);
    CHECK(rig.samples.back().endTick[0] == L);
    CHECK(rig.samples.back().endReason[0] == 2u);
    CHECK(rig.routedTargetHits() == 1);
    CHECK(flinched);
}

// ---------------------------------------------------------------------------
// G-14 pinned directly (the user's timing ruling, 2026-09-24: detection checks pos(t) on the
// step's OWN tick). The query adapter logs every parent transform the pass hands it; the shot's
// volume must be placed at closedFormPosition(slot, sd, elapsedSecondsAt(slot, dt, step tick)),
// on a Normal step and on a Skip step (tick + 2), and at the position the same step's integrate
// snaps the body to. No contact is scheduled, so the pass and integrate are the only writers.
// (review_defect_17.md N1: an off-by-one in the tick was caught only by the cancel geometry.)
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Projectile.ThePassQueriesTheShotAtItsClosedFormPositionOnTheStepsOwnTick",
          "[SimulatableBrawler][HitDetection]")
{
    Rig rig;
    rig.targetHoldsGuard = false;
    rig.targetAim = turnedAwayAim();
    constexpr std::uint32_t S = 10u;
    const auto& sd = rig.staticData.m_projectileStaticData;
    const std::uint32_t volume = projectileVolume(0u, 0u).value;

    for (std::uint32_t t = 0u; t <= S; ++t)
    {
        rig.step(t);
        if (t == S)
            rig.launch(0u, S, glm::vec3(0.f, 0.5f, 4.f), glm::vec3(0.f, 0.f, -1.f));
    }
    rig.step(S + 1u);
    rig.step(S + 2u);

    auto expectedAt = [&](std::uint32_t tick)
    {
        const auto& slot = rig.brawler(0u).getAllState().getState()
            .get<brawlerProjectileSimulation::State>().slots[0];
        return brawlerProjectileSimulation::closedFormPosition(slot, sd,
            brawlerProjectileSimulation::elapsedSecondsAt(slot, kDt, tick));
    };
    auto checkStep = [&](std::uint32_t tick, StepKind kind, const char* label)
    {
        rig.query.parentWrites.clear();
        rig.step(tick, true, true, kind);
        std::vector<glm::vec3> placed;
        for (const auto& [v, m] : rig.query.parentWrites)
            if (v == volume) placed.push_back(glm::vec3(m[3]));
        const glm::vec3 expected = expectedAt(tick);
        const glm::vec3 snapped(rig.phys.getBodyTransform(projectileBody(0u, 0u))[3]);
        INFO(label << " step, tick " << tick << ": expected x=" << expected.x
             << " placed=" << (placed.empty() ? -1.f : placed.front().x) << " (" << placed.size()
             << " writes) snapped x=" << snapped.x << " pos(tick-1) x=" << expectedAt(tick - 1u).x);
        REQUIRE(placed.size() == 1u);
        CHECK(placed.front() == expected);
        CHECK(snapped == expected);
        CHECK(expectedAt(tick - 1u) != expected);   // the pin can see a one-tick shift
        CHECK(rig.at(tick).endTick[0] == 0u);       // still in flight: nothing parked it
    };
    checkStep(S + 3u, StepKind::Normal, "Normal");
    checkStep(S + 5u, StepKind::Skip,   "Skip");
}

// ---------------------------------------------------------------------------
// Brief Hazard 1: a shooter that is NOT integrated on a step while its slot overlaps a target.
// Detection walks every character in storage, but only the shooter's own integrate ends its slot,
// and nothing carries "already routed" across a tick. So a live slot on a shooter that skips its
// integrate is detected again on the next pass.
// Production reaches "in storage, not integrated" only on a resim NoSlot row (a character with no
// correction-cache slot for the replayed tick, i.e. registered after it) and on the game-thread
// registration tear. A live slot on such a shooter is EITHER from a later tick than the step
// (detection skips it explicitly: guard G-07) OR was restored from a server correction
// (prepareResimAll writes the whole State, projectile slots included).
//   * section 1, a slot from a later tick: two un-integrated steps before the slot's spawn tick,
//     the target in contact throughout: EXACTLY ONE routed hit.
//   * section 2, a RESTORED slot, REACHABLE on a client replay (review_defect_17.md B1, accepted
//     by the user 2026-09-25 as a documented residual; the gate is Backlog task 23): a proxy
//     registers at R, a resim restores onto it a server shot spawned before R, and a later
//     shared-min resim anchors before R. On the NoSlot replay ticks before R the proxy is not
//     integrated, so its slot is not ended, and the same shot is routed on EVERY such tick.
//     Client-only and transient (the target's next correction heals it; the server never
//     resims). The projectile analogue of melee's F9 (task 9 behaviour review).
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.Projectile.AnUnintegratedShooterRoutesItsShotOnce",
          "[SimulatableBrawler][HitDetection]")
{
    SECTION("NoSlot steps precede the slot's spawn tick: one routed hit")
    {
        Rig rig;
        rig.targetHoldsGuard = false;
        rig.targetAim = turnedAwayAim();
        constexpr std::uint32_t S = 20u;   // the spawn tick the frontier state carries
        for (std::uint32_t t = 0u; t < S - 2u; ++t)
            rig.step(t);
        // The replay's un-restored frontier state: a shot launched on S, already in contact.
        rig.launch(0u, S, glm::vec3(0.f, 0.5f, 4.f), glm::vec3(0.f, 0.f, -1.f));
        rig.query.targetContact = true;
        rig.step(S - 2u, /*integrateShooter*/ false, true, StepKind::Normal, /*resim*/ true);
        rig.step(S - 1u, /*integrateShooter*/ false, true, StepKind::Normal, /*resim*/ true);
        for (std::uint32_t t = S; t <= S + 3u; ++t)
            rig.step(t, true, true, StepKind::Normal, /*resim*/ true);
        INFO("routed target hits=" << rig.routedTargetHits() << " slot end="
             << rig.samples.back().endTick[0] << "/" << rig.samples.back().endReason[0]);
        CHECK(rig.at(S - 2u).targetHitAfterPre == false);
        CHECK(rig.at(S - 1u).targetHitAfterPre == false);
        CHECK(rig.routedTargetHits() == 1);
        CHECK(rig.samples.back().endReason[0] == 2u);
    }

    SECTION("REACHABLE residual (task 23): a slot restored onto a NoSlot proxy is routed on every NoSlot replay tick")
    {
        Rig rig;
        rig.targetHoldsGuard = false;
        rig.targetAim = turnedAwayAim();
        constexpr std::uint32_t S = 10u;   // the server's spawn tick of the restored shot
        constexpr std::uint32_t A = 20u;   // the later shared-min resim's anchor, before R
        constexpr std::uint32_t R = 22u;   // the tick the proxy (brawler 0) registered on this client
        for (std::uint32_t t = 0u; t < A; ++t)
            rig.step(t, /*integrateShooter*/ false);   // not registered here yet
        // An earlier resim anchored at or after R restored the proxy from its correction; the
        // server's State carried a live shot spawned on S < R. `launch` writes that slot.
        rig.launch(0u, S, glm::vec3(0.f, 0.5f, 4.f), glm::vec3(0.f, 0.f, -1.f));
        rig.query.targetContact = true;
        rig.step(A,      /*integrateShooter*/ false, true, StepKind::Normal, /*resim*/ true);   // NoSlot
        rig.step(A + 1u, /*integrateShooter*/ false, true, StepKind::Normal, /*resim*/ true);   // NoSlot
        rig.step(R,      true, true, StepKind::Normal, /*resim*/ true);   // the proxy's first integrate
        rig.step(R + 1u, true, true, StepKind::Normal, /*resim*/ true);
        INFO("routed target hits=" << rig.routedTargetHits()
             << " -- one per NoSlot replay tick, then the proxy's first integrate (R) ends the slot."
                " Accepted residual; the gate is Backlog task 23");
        CHECK(rig.at(A).targetHitAfterPre);
        CHECK(rig.at(A + 1u).targetHitAfterPre);
        CHECK(rig.routedTargetHits() == 3);
        CHECK(rig.samples.back().endTick[0] == R);
    }
}


// ---------------------------------------------------------------------------
// THE TARGET'S GUARD DECIDES, WHICHEVER ENTRY IS LISTED FIRST [og-ground-aligned-attacks task 20b]
// A shot can reach the target's weapon sphere (`body`) and its concentric guard sphere on the same
// tick, and the Jolt adapter sorts the weapon entry first (declaration 0 before 1). Each case runs a
// scripted report through the single-shooter `detectProjectileHits`, in both entry orders.
// ---------------------------------------------------------------------------
namespace projectileGuardEntryTests
{
using brawlerProjectileSimulation::SlotOutcome;
using brawlerTestMocks::MockPhysicsAdapter;
using brawlerTestMocks::MockSpatialQueryAdapter;

constexpr std::uint32_t kPool           = static_cast<std::uint32_t>(brawlerProjectileSimulation::kMaxProjectilePoolSize);
constexpr std::uint32_t kShooterRootId  = kPool;
constexpr std::uint32_t kTargetRootId   = kPool + 1u;
constexpr std::uint32_t kTargetWeapon   = kPool + 2u;
constexpr std::uint32_t kTargetGuard    = kPool + 3u;
constexpr std::uint32_t kOtherRootId    = kPool + 4u;
constexpr std::uint32_t kOtherGuard     = kPool + 5u;
constexpr std::uint32_t kOtherShot      = kPool + 6u;
constexpr std::uint32_t kBodyCount      = kPool + 7u;
constexpr std::uint32_t kEntrySpawnTick = 100u;

enum class Entry { TargetBody, TargetGuard, OtherGuard, Projectile };

inline const char* entryName(Entry e)
{
    switch (e)
    {
    case Entry::TargetBody:  return "body";
    case Entry::TargetGuard: return "guard";
    case Entry::OtherGuard:  return "otherGuard";
    case Entry::Projectile:  return "projectile";
    }
    return "?";
}

inline SpatialQueryHit entryHit(Entry e)
{
    SpatialQueryHit hit{};
    hit.objectPosition = simFrame::withHeight(glm::vec3(0.f), 0.3f);
    switch (e)
    {
    case Entry::TargetBody:
        hit.bodyId = BodyId{ kTargetWeapon };  hit.rootBodyId = BodyId{ kTargetRootId };
        hit.objectCategories = CollisionCategories::single(collisionCategory::body);
        break;
    case Entry::TargetGuard:
        hit.bodyId = BodyId{ kTargetGuard };   hit.rootBodyId = BodyId{ kTargetRootId };
        hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
        break;
    case Entry::OtherGuard:
        hit.bodyId = BodyId{ kOtherGuard };    hit.rootBodyId = BodyId{ kOtherRootId };
        hit.objectCategories = CollisionCategories::single(collisionCategory::guard);
        break;
    case Entry::Projectile:
        hit.bodyId = BodyId{ kOtherShot };     hit.rootBodyId = BodyId{ kOtherRootId };
        hit.objectCategories = CollisionCategories::single(collisionCategory::projectile);
        break;
    }
    return hit;
}

// The shot travels along -kForward; a guard whose forward is +kForward faces it.
inline glm::vec3 entryShotDirection() { return -simFrame::kForward; }

inline glm::mat4 guardAt(float heading, const glm::vec3& root)
{
    glm::mat4 m = glm::rotate(glm::mat4(1.f), heading, simFrame::kUp);
    m[3] = glm::vec4(root, 1.f);
    return m;
}

inline const glm::vec3 kTargetGuardRoot(0.25f, 0.f, -0.5f);
inline const glm::vec3 kWeaponCentre(-0.75f, 0.3f, 1.5f);

struct EntryResult
{
    SlotOutcome   outcome = SlotOutcome::None;
    std::uint32_t struckRoot = 0u;
    glm::vec3     targetRootPosition{ 0.f };
};

// `targetGuardHeading`: 0 faces the shot, pi faces away. The other character's guard (when listed)
// always faces the shot, and the weapon body is turned away and placed elsewhere, so a predicate
// that read either of them instead of the target's guard would change the outcome.
inline EntryResult classify(const std::vector<Entry>& order, float targetGuardHeading)
{
    using namespace brawlerProjectileSimulation;
    static const simulatableBrawler::StaticData brawlerData;
    const StaticData& sd = brawlerData.m_projectileStaticData;

    State state;
    state.slots[0].spawnTick = kEntrySpawnTick;
    state.slots[0].spawnPos  = simFrame::withHeight(glm::vec3(0.f), 0.5f) + simFrame::kForward * 2.f;
    state.slots[0].spawnDir  = entryShotDirection();
    state.slots[0].endTick   = 0u;

    MockPhysicsAdapter physics{ kBodyCount };
    physics.bodies[kTargetGuard].transform  = guardAt(targetGuardHeading, kTargetGuardRoot);
    physics.bodies[kTargetWeapon].transform = guardAt(glm::pi<float>(), kWeaponCentre);
    physics.bodies[kOtherGuard].transform   = guardAt(0.f, glm::vec3(3.f, 0.f, 0.f));

    MockSpatialQueryAdapter query;
    for (const Entry e : order)
        query.nextReport.hits.push_back(entryHit(e));

    std::array<RuntimeBindings, kMaxProjectilePoolSize> bindings{};
    for (std::uint32_t i = 0; i < kMaxProjectilePoolSize; ++i)
    {
        bindings[i].ownBodyId      = BodyId{ i };
        bindings[i].parentBodyId   = BodyId{ kShooterRootId };
        bindings[i].queryVolumeIds = { QueryVolumeId{ i } };
    }
    DerivedState derived;
    brawlerHitDetection::detectProjectileHits(projectileDetectionTests::kDt, kEntrySpawnTick, sd, state, bindings, derived, physics, query);

    const SlotDetection& d = derived.detectedThisTick[0];
    return EntryResult{ d.outcome, d.struckRootBodyId.value, d.targetRootPosition };
}

inline std::string describe(const std::vector<Entry>& order, const EntryResult& r)
{
    std::string s = "[";
    for (std::size_t i = 0; i < order.size(); ++i)
        s += std::string(i ? ", " : "") + entryName(order[i]);
    return s + "] -> outcome " + std::to_string(static_cast<int>(r.outcome))
         + " struckRoot " + std::to_string(r.struckRoot);
}
} // namespace projectileGuardEntryTests

TEST_CASE("HitDetection.Projectile.AFacingGuardBlocksWhicheverEntryIsListedFirst", "[BrawlerProjectile]")
{
    using namespace projectileGuardEntryTests;
    for (const std::vector<Entry>& order : { std::vector<Entry>{ Entry::TargetBody, Entry::TargetGuard },
                                             std::vector<Entry>{ Entry::TargetGuard, Entry::TargetBody } })
    {
        const EntryResult r = classify(order, 0.f);
        std::printf("[ProjectileGuardEntry.facing] %s\n", describe(order, r).c_str());
        INFO(describe(order, r));
        CHECK(r.outcome == SlotOutcome::BlockedByGuard);
        CHECK(r.struckRoot == kTargetRootId);
        CHECK(r.targetRootPosition == kTargetGuardRoot);
    }
}

TEST_CASE("HitDetection.Projectile.AGuardFacingAwayIsAHitInBothOrders", "[BrawlerProjectile]")
{
    using namespace projectileGuardEntryTests;
    for (const std::vector<Entry>& order : { std::vector<Entry>{ Entry::TargetBody, Entry::TargetGuard },
                                             std::vector<Entry>{ Entry::TargetGuard, Entry::TargetBody } })
    {
        const EntryResult r = classify(order, glm::pi<float>());
        std::printf("[ProjectileGuardEntry.facingAway] %s\n", describe(order, r).c_str());
        INFO(describe(order, r));
        CHECK(r.outcome == SlotOutcome::Hit);
        CHECK(r.struckRoot == kTargetRootId);
        CHECK(r.targetRootPosition == kTargetGuardRoot);
    }
}

TEST_CASE("HitDetection.Projectile.NoGuardEntryOfTheStruckCharacterIsAHit", "[BrawlerProjectile]")
{
    using namespace projectileGuardEntryTests;
    for (const std::vector<Entry>& order : { std::vector<Entry>{ Entry::TargetBody },
                                             std::vector<Entry>{ Entry::TargetBody, Entry::OtherGuard } })
    {
        const EntryResult r = classify(order, 0.f);
        std::printf("[ProjectileGuardEntry.noGuard] %s\n", describe(order, r).c_str());
        INFO(describe(order, r));
        CHECK(r.outcome == SlotOutcome::Hit);
        CHECK(r.struckRoot == kTargetRootId);
        CHECK(r.targetRootPosition == glm::vec3(0.f));
    }
}

TEST_CASE("HitDetection.Projectile.AProjectileEntryFirstCancelsInBothOrders", "[BrawlerProjectile]")
{
    using namespace projectileGuardEntryTests;
    for (const std::vector<Entry>& order : { std::vector<Entry>{ Entry::Projectile, Entry::TargetBody, Entry::TargetGuard },
                                             std::vector<Entry>{ Entry::Projectile, Entry::TargetGuard, Entry::TargetBody } })
    {
        const EntryResult r = classify(order, 0.f);
        std::printf("[ProjectileGuardEntry.projectileFirst] %s\n", describe(order, r).c_str());
        INFO(describe(order, r));
        CHECK(r.outcome == SlotOutcome::CancelledByProjectile);
    }
}

// The `break`: one contact per slot, the first non-parent entry. A projectile listed AFTER the target
// does not cancel the slot.
TEST_CASE("HitDetection.Projectile.AProjectileListedAfterTheTargetDoesNotCancel", "[BrawlerProjectile]")
{
    using namespace projectileGuardEntryTests;
    const EntryResult hit = classify({ Entry::TargetBody, Entry::Projectile }, glm::pi<float>());
    const EntryResult block = classify({ Entry::TargetBody, Entry::TargetGuard, Entry::Projectile }, 0.f);
    std::printf("[ProjectileGuardEntry.projectileAfter] %s | %s\n",
        describe({ Entry::TargetBody, Entry::Projectile }, hit).c_str(),
        describe({ Entry::TargetBody, Entry::TargetGuard, Entry::Projectile }, block).c_str());
    CHECK(hit.outcome == SlotOutcome::Hit);
    CHECK(block.outcome == SlotOutcome::BlockedByGuard);
}

#endif // WITH_LOW_LEVEL_TESTS
