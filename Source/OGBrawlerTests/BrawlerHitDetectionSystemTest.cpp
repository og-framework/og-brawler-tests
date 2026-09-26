// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================
// MELEE HIT DETECTION MUST NOT DEPEND ON WHICH CHARACTER INTEGRATES FIRST
// [og-netcode-v2-field-defects task 9, D1]
//
// SUBJECT: the two cross-character reads a melee hit makes -- (a) whether the TARGET's guard
// shape is query-enabled, and (b) the TARGET's guard transform. Both are written in the
// target's OWN integrate, so a detector that runs inside the ATTACKER's integrate sees this
// tick's values on a peer that integrates the target first and last tick's values on a peer
// that integrates the attacker first. `integrateAll` walks an `unordered_map`, so which of
// the two a peer is depends on its registration history.
//
// THE FIELD SHAPE (capture_knockback_8.md, 7/7 phantoms): a target leaves a stun holding its
// guard; the attacker's swing makes contact on the target's FIRST Idle tick. The peer that
// integrates the target first sees the guard and the attacker recoils (GuardFlinch); the peer
// that integrates the attacker first sees only the body and the target is thrown (Knockback).
// Four PIE sessions could not run the discriminating experiment because the attacker always
// registered second. This file runs it: the SAME inputs, both integrate orders, same tick.
//
// ⭐ THE RIG, AND WHAT IT MODELS. Two production `SimulatableBrawler`s in the production
// storage, integrated in an order the case chooses, followed by the production systems
// executor. The fakes model exactly the two engine behaviours the defect is about and nothing
// else:
//   * `OrderSwapPhysics` RECORDS the target guard body's transform when the guard sub-sim
//     writes it and returns it on read. Every other body reads the identity (the weapon never
//     rotates, so its segment stays Damaging -- the sibling radial rigs' fixture choice).
//   * `OrderSwapQuery` tracks the target guard SHAPE's enable bit as the guard sub-sim toggles
//     it, and its overlap reports a guard hit only while that bit is set -- the Chaos query
//     adapter's `SetQueryEnabled` behaviour, reduced to one bit.
// ⚠ Contact is SCHEDULED (`weaponOverlapsTarget`), as in every radial rig here: the weapon
// never rotates, so the tick the blade reaches the target is a fixture choice, placed on the
// target's first Idle tick -- which is MEASURED by a dry run, never authored.
//
// [og-netcode-v2-field-defects task 20] THE RIG'S TICK BOUNDARY. Detection and routing run in
// preIntegrate of T+1, over the state tick T left. `tick(t)` is therefore integrate(t), the
// post-integrate phase of t, then the NEXT step's pre-integrate phase (t+1): production call
// order, cut after the reduction over t. A sample taken after tick(t) sees exactly what t+1's
// integrate reads -- the same observation point this file had when both systems were
// post-integrate, which is why the expectations below did not move.
//
// TAGS: `[SimulatableBrawler]` is in the `[@og]` whitelist (OgTagAliases.cpp); `[HitDetection]`
// is NOT, and a case carrying only it would pass on a direct call and never run under `[@og]`.
// ============================================================================

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/StorageView.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SimulationSerialization.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"

namespace hitDetectionTests
{

constexpr float kDt = 1.f / 60.f;

constexpr std::uint32_t kAttackerRoot     = 20u;
constexpr std::uint32_t kTargetRoot       = 21u;
constexpr std::uint32_t kTargetGuardBody  = 22u;
constexpr std::uint32_t kTargetGuardShape = 23u;
constexpr std::uint32_t kRadialVolume     = 7u;

// The attacker's weapon sits at the origin (identity transform); the target 150 cm out along
// +X is inside the production annulus (inner 90, outer 300) and on the swing plane.
constexpr float kTargetDistance = 150.f;

// The measured field timeline: contact 24 ticks into the attacker's seq-0 swing.
constexpr int kContactSwingTick = 24;

// A guard that BLOCKS seq 0. `wouldGuardBlock` with the attacker at the origin and the target
// at +X: the collision direction is -X, so a guard facing -X rotated by +0.5 rad about +Z sits
// 0.5 rad off the line (inside [shieldAngle 0.25, pi/2)) with cross(n, g).z = sin(0.5) > 0 --
// the seq 0/2 side. Written as an AIM because the guard sub-sim builds its transform from the
// target's aim input, which is the path a player's stick takes.
inline glm::vec3 blockingAim()   { return glm::vec3(-std::cos(0.5f), -std::sin(0.5f), 0.f); }
// A guard facing AWAY from the attacker: td = pi, outside the outer gate, never blocks.
inline glm::vec3 turnedAwayAim() { return glm::vec3(1.f, 0.f, 0.f); }

struct OrderSwapPhysics
{
    glm::mat4 guardTransform{ 1.f };

    glm::mat4 getBodyTransform(BodyId id) const
    { return id == BodyId{ kTargetGuardBody } ? guardTransform : glm::mat4(1.f); }
    void setBodyTransform(BodyId id, const glm::mat4& m)
    { if (id == BodyId{ kTargetGuardBody }) guardTransform = m; }
    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
    bool isBodyResolvable(BodyId) const                   { return true; }
};

static_assert(PhysicsBodyAdapter<OrderSwapPhysics>);

struct OrderSwapQuery
{
    bool weaponOverlapsTarget = false;
    bool guardShapeEnabled    = false;
    mutable int radialOverlapCalls = 0;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        const bool radial = std::find(volumeIds.begin(), volumeIds.end(),
                                      QueryVolumeId{ kRadialVolume }) != volumeIds.end();
        if (!radial)
            return SpatialQueryReport{};
        ++radialOverlapCalls;
        SpatialQueryReport report;
        if (!weaponOverlapsTarget)
            return report;

        SpatialQueryHit body{};
        body.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
        body.bodyId           = BodyId{ kTargetRoot };
        body.rootBodyId       = BodyId{ kTargetRoot };
        body.objectCategories = CollisionCategories::single(collisionCategory::body);
        report.hits.push_back(body);

        if (guardShapeEnabled)
        {
            SpatialQueryHit guard{};
            guard.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
            guard.bodyId           = BodyId{ kTargetGuardBody };
            guard.rootBodyId       = BodyId{ kTargetRoot };
            guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
            report.hits.push_back(guard);
        }
        return report;
    }

    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId id)  { if (id == ShapeId{ kTargetGuardShape }) guardShapeEnabled = true; }
    void disableShape(ShapeId id) { if (id == ShapeId{ kTargetGuardShape }) guardShapeEnabled = false; }
};

static_assert(SpatialQueryAdapter<OrderSwapQuery>);

enum class Order { TargetFirst, AttackerFirst };

// One tick, observed AFTER the systems ran -- where a reader of the routed slice stands.
struct Sample
{
    DAttackState    attackerMachine = DAttackState::Idle;
    DAttackState    targetMachine   = DAttackState::Idle;
    HitReactionKind targetReaction  = HitReactionKind::Stun;
    bool            targetWasHit    = false;
    std::size_t     ledgerTargets   = 0;
    std::size_t     guardHits       = 0;
};

struct Scenario
{
    bool      stunTargetOnTickZero = true;    // the field shape; false = target Idle throughout
    glm::vec3 targetAimBeforeContact = blockingAim();
    glm::vec3 targetAimFromContact   = blockingAim();
    bool      attackerSwings = true;
    int       extraTicksAfterContact = 3;
};

using Detection = brawlerHitDetection::System<OrderSwapPhysics, OrderSwapQuery>;

// The production systems phases for these two systems: the SAME executor type the manager
// instantiates, with detection handed THIS rig's fakes through the piecewise constructor, exactly
// as SimulationManagerUImpl hands it the Chaos adapters. Firing order is template order.
struct FOrderSwapRig
{
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
        simulatableBrawler::StaticData,
        Detection,
        brawlerHitRouting::System>;

    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    OrderSwapPhysics phys;
    OrderSwapQuery   query;
    Exec             exec{ std::piecewise_construct, Detection(phys, query), brawlerHitRouting::System{} };

    FOrderSwapRig()
    {
        staticData.m_movementStaticData.drivesBody = false;
        storage.add<SimulatableBrawler>(0u, SimulatableBrawler(staticData));
        storage.add<SimulatableBrawler>(1u, SimulatableBrawler(staticData));
        brawler(0u).setCharacterBindings({ BodyId{ kAttackerRoot } });
        brawler(1u).setCharacterBindings({ BodyId{ kTargetRoot } });

        brawler(0u).editPhysicsComposite()
            .edit<dAttackRadialSimulation::PhysicsDeclaration>()
            .bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };

        auto& guard = brawler(1u).editPhysicsComposite()
            .edit<dAttackGuardSimulation::PhysicsDeclaration>().bindings;
        guard.ownBodyId    = BodyId{ kTargetGuardBody };
        guard.parentBodyId = BodyId{ kTargetRoot };
        guard.shapeIds     = { ShapeId{ kTargetGuardShape } };

        exec.notifyCharacterRegistered(0u, storage, staticData, /*isAuthority*/ true);
        exec.notifyCharacterRegistered(1u, storage, staticData, /*isAuthority*/ true);
        firePreIntegrate(0u);   // tick 0's own pre-integrate phase
    }

    SimulatableBrawler& brawler(unsigned int id) { return storage.get<SimulatableBrawler>(id); }

    static simulatableBrawler::PlayerInput makeInput(const glm::vec3& aim, bool attackLeft,
                                                     const glm::vec2& stick, std::uint8_t flags)
    {
        const glm::vec3 moveWorld(stick.x, stick.y, 0.f);
        return simulatableBrawler::PlayerInput(
            dAttackRadialSimulation::PlayerInput(aim, attackLeft, false),
            dAttackMachineSimulation::PlayerInput{ aim, attackLeft, false, stick, moveWorld },
            dAttackGuardSimulation::PlayerInput(aim),
            brawlerProjectileSimulation::PlayerInput{ aim },
            brawlerMovementSimulation::PlayerInput{ flags },
            brawlerRingout::PlayerInput{});
    }

    void integrateBoth(std::uint32_t t, Order order,
                       const simulatableBrawler::PlayerInput& attackerInput,
                       const simulatableBrawler::PlayerInput& targetInput)
    {
        const SimulationTimeStep step(t, false, false, false, kDt);
        if (order == Order::TargetFirst)
        {
            brawler(1u).integrate(step, targetInput, phys, query, staticData);
            brawler(0u).integrate(step, attackerInput, phys, query, staticData);
        }
        else
        {
            brawler(0u).integrate(step, attackerInput, phys, query, staticData);
            brawler(1u).integrate(step, targetInput, phys, query, staticData);
        }
    }

    void firePostIntegrate(std::uint32_t t, bool resim = false)
    {
        exec.firePostIntegrate(SimulationTimeStep(t, resim, false, false, kDt),
                               storage, staticData, /*isAuthority*/ !resim);
    }

    void firePreIntegrate(std::uint32_t t, bool resim = false)
    {
        exec.firePreIntegrate(SimulationTimeStep(t, resim, false, false, kDt),
                              storage, staticData, /*isAuthority*/ !resim);
    }

    // One tick: both integrates in `order`, the post-integrate phase of t, then the
    // pre-integrate phase of t+1 -- where detection and routing reduce tick t (task 20).
    void tick(std::uint32_t t, Order order,
              const simulatableBrawler::PlayerInput& attackerInput,
              const simulatableBrawler::PlayerInput& targetInput)
    {
        integrateBoth(t, order, attackerInput, targetInput);
        firePostIntegrate(t);
        firePreIntegrate(t + 1u);
    }

    Sample sample()
    {
        Sample s{};
        const auto& attacker = brawler(0u).getAllState();
        const auto& target   = brawler(1u).getAllState();
        s.attackerMachine = attacker.getState().get<dAttackMachineSimulation::State>().m_currentState;
        s.targetMachine   = target.getState().get<dAttackMachineSimulation::State>().m_currentState;
        s.targetReaction  = target.getState().get<dAttackMachineSimulation::State>().m_hitReaction;
        s.targetWasHit    = target.getDerivedState().get<brawlerInboundHit::DerivedState>().wasHitThisTick;
        const auto& radial = attacker.getDerivedState().get<dAttackRadialSimulation::DerivedState>();
        const auto& ledger = attacker.getState().get<dAttackRadialSimulation::State>().hitTargets;
        s.ledgerTargets = static_cast<std::size_t>(std::count_if(ledger.begin(), ledger.end(),
            [](SimCharacterId id) { return id != SimCharacterId::None; }));
        s.guardHits  = radial.getGuardHits().size();
        return s;
    }
};

// The stun the field capture opens with, delivered the way routing delivers it: the target's
// inbound slice carries the forward swing's Stun row, and its machine consumes it on tick 0.
inline void seedStun(FOrderSwapRig& rig)
{
    const HitReactionSpec& stun = rig.staticData.m_hitReactions[dAttackDirection::kForwardSequenceId];
    auto& slice = rig.brawler(1u).editAllState().editDerivedState()
        .edit<brawlerInboundHit::DerivedState>();
    slice.wasHitThisTick = true;
    slice.reactionKind   = stun.kind;
    slice.flinchDuration = stun.lockoutDuration;
}

// DRY RUN: the target alone (the attacker idles), both orders are irrelevant. Returns the first
// tick on which the target's machine reads Idle after the seeded stun.
inline int measureFirstIdleTick()
{
    FOrderSwapRig rig;
    seedStun(rig);
    const auto idleAttacker = FOrderSwapRig::makeInput(glm::vec3(1.f, 0.f, 0.f), false, glm::vec2(0.f), 0u);
    const auto target = FOrderSwapRig::makeInput(blockingAim(), false, glm::vec2(0.f),
                                                 brawlerMovementSimulation::kInputFlagHoldGuard);
    for (int t = 0; t < 200; ++t)
    {
        rig.tick(static_cast<std::uint32_t>(t), Order::TargetFirst, idleAttacker, target);
        if (t > 0 && rig.sample().targetMachine == DAttackState::Idle)
            return t;
    }
    return -1;
}

struct Arm
{
    int                 contactTick = -1;
    std::vector<Sample> samples;       // index == tick
    const Sample& at(int t) const { return samples.at(static_cast<std::size_t>(t)); }
};

// Runs one arm. The attacker presses seq 0 on `contactTick - kContactSwingTick` for one tick;
// the blade reaches the target from `contactTick` on.
inline Arm runArm(Order order, const Scenario& scenario, int contactTick)
{
    FOrderSwapRig rig;
    if (scenario.stunTargetOnTickZero)
        seedStun(rig);

    Arm arm;
    arm.contactTick = contactTick;
    const int pressTick = contactTick - kContactSwingTick;
    const glm::vec3 attackerAim(1.f, 0.f, 0.f);
    for (int t = 0; t <= contactTick + scenario.extraTicksAfterContact; ++t)
    {
        const bool press = scenario.attackerSwings && t == pressTick;
        // (0,-1) against aim (1,0,0) classifies as the right swing: sequence 0.
        const auto attacker = FOrderSwapRig::makeInput(attackerAim, press,
            press ? glm::vec2(0.f, -1.f) : glm::vec2(0.f), 0u);
        const auto target = FOrderSwapRig::makeInput(
            t < contactTick ? scenario.targetAimBeforeContact : scenario.targetAimFromContact,
            false, glm::vec2(0.f), brawlerMovementSimulation::kInputFlagHoldGuard);

        rig.query.weaponOverlapsTarget = t >= contactTick;
        rig.tick(static_cast<std::uint32_t>(t), order, attacker, target);
        arm.samples.push_back(rig.sample());
    }
    return arm;
}

inline const char* stateName(DAttackState s) { return dAttackMachineSimulation::dAttackStateName(s); }

inline std::string describe(const Arm& arm)
{
    const int t = arm.contactTick;
    std::string text;
    for (int k = t - 1; k <= t + 1; ++k)
    {
        const Sample& s = arm.at(k);
        text += "T" + std::string(k < t ? "-1" : (k == t ? "" : "+1"))
              + ": attacker=" + stateName(s.attackerMachine)
              + " target=" + stateName(s.targetMachine)
              + " reaction=" + std::to_string(static_cast<unsigned>(s.targetReaction))
              + " targetHit=" + (s.targetWasHit ? "1" : "0")
              + " ledgerTargets=" + std::to_string(s.ledgerTargets)
              + " guardHits=" + std::to_string(s.guardHits) + " | ";
    }
    return text;
}

} // namespace hitDetectionTests

using namespace hitDetectionTests;

// ---------------------------------------------------------------------------
// ⭐ THE ORDER-SWAP LLT (task 9 AC 1) -- the field shape, both integrate orders.
// Contact lands on the target's first Idle tick T after a stun, with the guard held and
// facing the attacker. Required: the SAME outcome in both arms, and it is the guard's:
// the attacker recoils (GuardFlinch on T+1) and the target is not hit.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.StunExitTickOutcomeIsIndependentOfIntegrateOrder",
          "[SimulatableBrawler][HitDetection]")
{
    const int firstIdle = measureFirstIdleTick();
    INFO("target's first Idle tick after the stun = " << firstIdle);
    REQUIRE(firstIdle > kContactSwingTick);

    const Scenario scenario{};
    const Arm targetFirst   = runArm(Order::TargetFirst,   scenario, firstIdle);
    const Arm attackerFirst = runArm(Order::AttackerFirst, scenario, firstIdle);
    INFO("arm 1 (target first):   " << describe(targetFirst));
    INFO("arm 2 (attacker first): " << describe(attackerFirst));

    // Premises: the swing is live and the target is exactly on its first Idle tick at T.
    const int T = firstIdle;
    REQUIRE(targetFirst.at(T - 1).targetMachine == DAttackState::HitFlinch);
    REQUIRE(targetFirst.at(T).targetMachine == DAttackState::Idle);
    REQUIRE(attackerFirst.at(T).targetMachine == DAttackState::Idle);
    REQUIRE(targetFirst.at(T).attackerMachine == DAttackState::Attacking);
    REQUIRE(attackerFirst.at(T).attackerMachine == DAttackState::Attacking);

    // The discriminating observable: identical in both arms, and it is the block.
    CHECK(attackerFirst.at(T + 1).attackerMachine == targetFirst.at(T + 1).attackerMachine);
    CHECK(attackerFirst.at(T + 1).targetMachine   == targetFirst.at(T + 1).targetMachine);
    CHECK(attackerFirst.at(T).targetWasHit        == targetFirst.at(T).targetWasHit);
    CHECK(targetFirst.at(T + 1).attackerMachine   == DAttackState::GuardFlinch);
    CHECK(attackerFirst.at(T + 1).attackerMachine == DAttackState::GuardFlinch);
    CHECK_FALSE(targetFirst.at(T).targetWasHit);
    CHECK_FALSE(attackerFirst.at(T).targetWasHit);
    CHECK(targetFirst.at(T + 1).targetMachine   == DAttackState::Idle);
    CHECK(attackerFirst.at(T + 1).targetMachine == DAttackState::Idle);
}

// ---------------------------------------------------------------------------
// DetectionSeesThisTicksGuardTransform (task 9 AC 2) -- read (b) alone. The target is Idle
// throughout, so its guard shape is enabled on every tick and read (a) cannot differ; it turns
// its guard from facing away to facing the attacker ON the contact tick, inside its own
// integrate. Both arms must classify the hit against the SAME guard transform: blocked.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.DetectionSeesThisTicksGuardTransform",
          "[SimulatableBrawler][HitDetection]")
{
    Scenario scenario{};
    scenario.stunTargetOnTickZero   = false;
    scenario.targetAimBeforeContact = turnedAwayAim();
    scenario.targetAimFromContact   = blockingAim();
    const int T = 40;

    const Arm targetFirst   = runArm(Order::TargetFirst,   scenario, T);
    const Arm attackerFirst = runArm(Order::AttackerFirst, scenario, T);
    INFO("arm 1 (target first):   " << describe(targetFirst));
    INFO("arm 2 (attacker first): " << describe(attackerFirst));

    REQUIRE(targetFirst.at(T).targetMachine == DAttackState::Idle);
    REQUIRE(attackerFirst.at(T).targetMachine == DAttackState::Idle);
    REQUIRE(targetFirst.at(T).attackerMachine == DAttackState::Attacking);

    CHECK(attackerFirst.at(T + 1).attackerMachine == targetFirst.at(T + 1).attackerMachine);
    CHECK(attackerFirst.at(T).targetWasHit        == targetFirst.at(T).targetWasHit);
    CHECK(targetFirst.at(T + 1).attackerMachine   == DAttackState::GuardFlinch);
    CHECK(attackerFirst.at(T + 1).attackerMachine == DAttackState::GuardFlinch);
    CHECK_FALSE(attackerFirst.at(T).targetWasHit);
}

// ---------------------------------------------------------------------------
// GuardBlockRoutesThroughTheInboundSlice (task 9 AC 3). The block detected on T is the
// radial DerivedState's per-tick guardBlockedThisTick, copied by routing onto the attacker's
// inbound slice as wasGuardBlockedThisTick, and the machine recoils on T+1. Then the REPLAY:
// wire State restored to the end of T-1 (what an adopted correction does), T and T+1
// re-integrated. The bit is RECOMPUTED on the replay -- no wire field carries it any more.
// [task 20] It is recomputed by the replay's preIntegrate(T+1), over the replayed end of T.
// The anchor-AT-T replay (the one task 9 could not recover) is
// HitDetection.Behaviour.ReplayAnchoredAtTheEndOfTheBlockTickRecoils.
// ---------------------------------------------------------------------------
namespace hitDetectionTests
{
    template <typename S>
    concept HasHitGuardMember = requires(S s) { s.hasHitGuard; };
}

TEST_CASE("HitDetection.GuardBlockRoutesThroughTheInboundSlice",
          "[SimulatableBrawler][HitDetection]")
{
    // The field left the wire: not a member of the radial State, and the slice that carries
    // its replacement is off-wire (no SerializableFields specialization at all).
    STATIC_REQUIRE_FALSE(HasHitGuardMember<dAttackRadialSimulation::State>);
    STATIC_REQUIRE_FALSE(Serializable<brawlerInboundHit::DerivedState>);

    const int T = 40;
    FOrderSwapRig rig;
    const glm::vec3 attackerAim(1.f, 0.f, 0.f);
    auto attackerInput = [&](int t)
    {
        const bool press = t == T - kContactSwingTick;
        return FOrderSwapRig::makeInput(attackerAim, press,
            press ? glm::vec2(0.f, -1.f) : glm::vec2(0.f), 0u);
    };
    const auto targetInput = FOrderSwapRig::makeInput(blockingAim(), false, glm::vec2(0.f),
        brawlerMovementSimulation::kInputFlagHoldGuard);

    struct Observed
    {
        bool         radialBlockedAtT    = false;
        bool         sliceBlockedAtT     = false;
        bool         sliceBlockedAtT1    = false;
        DAttackState attackerAtT         = DAttackState::Idle;
        DAttackState attackerAtT1        = DAttackState::Idle;
        std::size_t  guardHitsAtT        = 0;
    };
    auto observeTick = [&](int t, Observed& o)
    {
        const auto& attacker = rig.brawler(0u).getAllState();
        const bool radial = attacker.getDerivedState()
            .get<dAttackRadialSimulation::DerivedState>().getGuardBlockedThisTick();
        const bool slice = attacker.getDerivedState()
            .get<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick;
        const DAttackState machine =
            attacker.getState().get<dAttackMachineSimulation::State>().m_currentState;
        if (t == T)
        {
            o.radialBlockedAtT = radial;
            o.sliceBlockedAtT  = slice;
            o.attackerAtT      = machine;
            o.guardHitsAtT     = attacker.getDerivedState()
                .get<dAttackRadialSimulation::DerivedState>().getGuardHits().size();
        }
        if (t == T + 1)
        {
            o.sliceBlockedAtT1 = slice;
            o.attackerAtT1     = machine;
        }
    };

    simulatableBrawler::State savedAttacker;
    simulatableBrawler::State savedTarget;
    Observed live;
    for (int t = 0; t <= T + 1; ++t)
    {
        rig.query.weaponOverlapsTarget = t >= T;
        rig.tick(static_cast<std::uint32_t>(t), Order::TargetFirst, attackerInput(t), targetInput);
        observeTick(t, live);
        if (t == T - 1)
        {
            savedAttacker = rig.brawler(0u).getAllState().getState();
            savedTarget   = rig.brawler(1u).getAllState().getState();
        }
    }

    INFO("live: radialBlocked@T=" << live.radialBlockedAtT << " slice@T=" << live.sliceBlockedAtT
         << " slice@T+1=" << live.sliceBlockedAtT1
         << " attacker@T=" << dAttackMachineSimulation::dAttackStateName(live.attackerAtT)
         << " attacker@T+1=" << dAttackMachineSimulation::dAttackStateName(live.attackerAtT1));
    REQUIRE(live.attackerAtT == DAttackState::Attacking);
    CHECK(live.radialBlockedAtT);
    CHECK(live.sliceBlockedAtT);
    CHECK(live.guardHitsAtT == 1u);
    CHECK(live.attackerAtT1 == DAttackState::GuardFlinch);
    // One tick wide: routing's whole-slice reset clears it on T+1 (no block is detected there).
    CHECK_FALSE(live.sliceBlockedAtT1);

    // THE REPLAY anchored at the end of T-1. The radial's per-swing ledgers are NOT restored
    // (they are derived); the swing's deactivate on T+1 already cleared them, as it would in
    // production. The world is rewound to the end of T-1, where the blade has not arrived, and
    // the replay opens with its own preIntegrate(T) over that state.
    rig.brawler(0u).editAllState().editState() = savedAttacker;
    rig.brawler(1u).editAllState().editState() = savedTarget;
    rig.query.weaponOverlapsTarget = false;
    rig.firePreIntegrate(static_cast<std::uint32_t>(T), /*resim*/ true);
    Observed replay;
    for (int t = T; t <= T + 1; ++t)
    {
        rig.query.weaponOverlapsTarget = true;
        const SimulationTimeStep step(static_cast<std::uint32_t>(t), true, false, false, kDt);
        rig.brawler(1u).integrate(step, targetInput, rig.phys, rig.query, rig.staticData);
        rig.brawler(0u).integrate(step, attackerInput(t), rig.phys, rig.query, rig.staticData);
        rig.firePostIntegrate(static_cast<std::uint32_t>(t), /*resim*/ true);
        rig.firePreIntegrate(static_cast<std::uint32_t>(t + 1), /*resim*/ true);
        observeTick(t, replay);
    }
    INFO("replay: radialBlocked@T=" << replay.radialBlockedAtT << " slice@T=" << replay.sliceBlockedAtT
         << " attacker@T+1=" << dAttackMachineSimulation::dAttackStateName(replay.attackerAtT1));
    CHECK(replay.radialBlockedAtT);
    CHECK(replay.sliceBlockedAtT);
    CHECK(replay.attackerAtT1 == DAttackState::GuardFlinch);
    CHECK(replay.attackerAtT1 == live.attackerAtT1);
}

// ---------------------------------------------------------------------------
// The executor runs detection with the adapters it was CONSTRUCTED with (task 9 AC: "a fixture
// proving postIntegrate runs with the same adapters the executor uses"), and integrate no
// longer queries at all. Measured on a Damaging tick: the radial overlap count moves only
// inside the systems executor, by exactly one (one attacker mid-swing), on this rig's own query.
// [task 20] ...and inside the NEXT step's firePreIntegrate, never inside firePostIntegrate.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.ExecutorFiresDetectionWithTheAdaptersItWasConstructedWith",
          "[SimulatableBrawler][HitDetection]")
{
    FOrderSwapRig rig;
    REQUIRE(&rig.exec.get<Detection>().queryAdapter()   == &rig.query);
    REQUIRE(&rig.exec.get<Detection>().physicsAdapter() == &rig.phys);

    const glm::vec3 attackerAim(1.f, 0.f, 0.f);
    const auto idleTarget = FOrderSwapRig::makeInput(turnedAwayAim(), false, glm::vec2(0.f), 0u);
    const auto press   = FOrderSwapRig::makeInput(attackerAim, true,  glm::vec2(0.f, -1.f), 0u);
    const auto release = FOrderSwapRig::makeInput(attackerAim, false, glm::vec2(0.f), 0u);

    rig.tick(0u, Order::AttackerFirst, press, idleTarget);
    REQUIRE(rig.sample().attackerMachine == DAttackState::Attacking);

    rig.query.weaponOverlapsTarget = true;
    const int beforeIntegrate = rig.query.radialOverlapCalls;
    rig.integrateBoth(1u, Order::AttackerFirst, release, idleTarget);
    const int afterIntegrate = rig.query.radialOverlapCalls;
    REQUIRE(rig.brawler(0u).getAllState().getDerivedState()
        .get<dAttackRadialSimulation::DerivedState>().getHitsThisTick().empty());
    rig.firePostIntegrate(1u);
    const int afterPost = rig.query.radialOverlapCalls;
    rig.firePreIntegrate(2u);
    const int afterPre = rig.query.radialOverlapCalls;

    INFO("overlap calls: integrate " << (afterIntegrate - beforeIntegrate)
         << ", post-integrate " << (afterPost - afterIntegrate)
         << ", next pre-integrate " << (afterPre - afterPost));
    CHECK(afterIntegrate - beforeIntegrate == 0);
    CHECK(afterPost - afterIntegrate == 0);
    CHECK(afterPre - afterPost == 1);
    CHECK(rig.brawler(0u).getAllState().getDerivedState()
        .get<dAttackRadialSimulation::DerivedState>().getHitsThisTick().size() == 1u);
    CHECK(rig.sample().targetWasHit);
}

// ---------------------------------------------------------------------------
// The ordering pin the manager's BrawlerSystemsExec static_asserts on (SimulationManagerUImpl.h).
// The UE layer is not compiled into this suite, so the TRAIT is proved here, both arms: the
// shipped order reads true, the reversed and the missing orders read false.
// ---------------------------------------------------------------------------
TEST_CASE("HitDetection.FiresBeforeTraitSeesTheExecutorOrder",
          "[SimulatableBrawler][HitDetection]")
{
    using L  = SimulatableList<SimulatableBrawler>;
    using SD = simulatableBrawler::StaticData;
    using Shipped  = SimulationSystemsExecutor<L, SD, Detection, brawlerHitRouting::System>;
    using Reversed = SimulationSystemsExecutor<L, SD, brawlerHitRouting::System, Detection>;
    using Missing  = SimulationSystemsExecutor<L, SD, brawlerHitRouting::System>;

    STATIC_REQUIRE(brawlerHitDetection::firesBefore<Shipped, Detection, brawlerHitRouting::System>);
    STATIC_REQUIRE_FALSE(brawlerHitDetection::firesBefore<Reversed, Detection, brawlerHitRouting::System>);
    STATIC_REQUIRE_FALSE(brawlerHitDetection::firesBefore<Missing, Detection, brawlerHitRouting::System>);
    STATIC_REQUIRE_FALSE(brawlerHitDetection::firesBefore<int, Detection, brawlerHitRouting::System>);
}

#endif // WITH_LOW_LEVEL_TESTS
