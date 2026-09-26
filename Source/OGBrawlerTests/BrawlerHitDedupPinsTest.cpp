// SPDX-License-Identifier: BUSL-1.1
// docs/BrawlerHitDedupPinsTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackGuardSimulation.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace hitDedupPinsTests
{

constexpr float kDt               = 1.f / 60.f;
constexpr int   kContactSwingTick = 24;
constexpr int   kContactTick      = 40;
constexpr int   kPressTick        = kContactTick - kContactSwingTick;
constexpr int   kLastTick         = kContactTick + 40;
constexpr float kTargetDistance   = 150.f;

struct Handles
{
    unsigned int  attackerId;
    unsigned int  targetId;
    std::uint32_t attackerRoot;
    std::uint32_t targetRoot;
    std::uint32_t targetGuardBody;
    std::uint32_t targetGuardShape;
    std::uint32_t radialVolume;
};

constexpr Handles kPeerA{ 1u, 2u, 30u, 31u, 32u, 33u, 9u };
constexpr Handles kPeerB{ 1u, 2u, 130u, 131u, 132u, 133u, 19u };

static_assert(kPeerA.attackerId == kPeerB.attackerId && kPeerA.targetId == kPeerB.targetId,
    "Task 26 ruling 2026-09-26: both peers register the SAME storage keys. From task 25 the key is "
    "the peer-stable SimCharacterId, so a different key is a different character and task 27's "
    "ledger bytes would differ legitimately.");
static_assert(kPeerA.attackerId != 0u && kPeerA.targetId != 0u && kPeerA.attackerId != kPeerA.targetId,
    "Task 25: SimCharacterId 0 means no character; the two characters need two distinct non-zero keys.");
static_assert(kPeerA.attackerRoot != kPeerB.attackerRoot && kPeerA.targetRoot != kPeerB.targetRoot
    && kPeerA.targetGuardBody != kPeerB.targetGuardBody && kPeerA.targetGuardShape != kPeerB.targetGuardShape
    && kPeerA.radialVolume != kPeerB.radialVolume,
    "Task 26 ruling 2026-09-26: every per-process engine handle differs between the peers; a handle "
    "the two peers share cannot expose that handle on the wire.");

inline glm::vec3 guardFacingTheAttackerAim() { return glm::vec3(-std::cos(0.5f), -std::sin(0.5f), 0.f); }
inline glm::vec3 guardTurnedAwayAim()        { return glm::vec3(1.f, 0.f, 0.f); }

struct Physics
{
    std::uint32_t guardBody = 0u;
    glm::mat4 guardTransform{ 1.f };
    glm::mat4 getBodyTransform(BodyId id) const
    { return id == BodyId{ guardBody } ? guardTransform : glm::mat4(1.f); }
    void setBodyTransform(BodyId id, const glm::mat4& m)
    { if (id == BodyId{ guardBody }) guardTransform = m; }
    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
    bool isBodyResolvable(BodyId) const                   { return true; }
};
static_assert(PhysicsBodyAdapter<Physics>);

struct Query
{
    Handles h{};
    bool weaponOverlapsTarget = false;
    bool guardShapeEnabled    = false;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        SpatialQueryReport report;
        const bool radial = std::find(volumeIds.begin(), volumeIds.end(),
                                      QueryVolumeId{ h.radialVolume }) != volumeIds.end();
        if (!radial || !weaponOverlapsTarget)
            return report;
        SpatialQueryHit body{};
        body.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
        body.bodyId           = BodyId{ h.targetRoot };
        body.rootBodyId       = BodyId{ h.targetRoot };
        body.objectCategories = CollisionCategories::single(collisionCategory::body);
        report.hits.push_back(body);
        if (guardShapeEnabled)
        {
            SpatialQueryHit guard = body;
            guard.bodyId           = BodyId{ h.targetGuardBody };
            guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
            report.hits.push_back(guard);
        }
        return report;
    }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId id)  { if (id == ShapeId{ h.targetGuardShape }) guardShapeEnabled = true; }
    void disableShape(ShapeId id) { if (id == ShapeId{ h.targetGuardShape }) guardShapeEnabled = false; }
};
static_assert(SpatialQueryAdapter<Query>);

using Detection = brawlerHitDetection::System<Physics, Query>;

struct Rig
{
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
        simulatableBrawler::StaticData, Detection, brawlerHitRouting::System>;
    using Integration = SimulationIntegrationExecutor<simulatableBrawler::StaticData, Physics, Query,
        SimulatableBrawler>;

    Handles h;
    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    Physics phys;
    Query   query;
    Exec        exec;
    Integration integration;

    Rig(const Handles& handles, bool registerTargetFirst)
        : h(handles)
        , exec(std::piecewise_construct, Detection(phys, query), brawlerHitRouting::System{})
        , integration(storage, staticData, phys, query)
    {
        phys.guardBody = h.targetGuardBody;
        query.h = h;
        staticData.m_movementStaticData.drivesBody = false;
        const unsigned int first  = registerTargetFirst ? h.targetId : h.attackerId;
        const unsigned int second = registerTargetFirst ? h.attackerId : h.targetId;
        storage.add<SimulatableBrawler>(first, SimulatableBrawler(staticData));
        storage.add<SimulatableBrawler>(second, SimulatableBrawler(staticData));
        attacker().setCharacterBindings({ BodyId{ h.attackerRoot } });
        target().setCharacterBindings({ BodyId{ h.targetRoot } });
        attacker().editPhysicsComposite().edit<dAttackRadialSimulation::PhysicsDeclaration>()
            .bindings.queryVolumeIds = { QueryVolumeId{ h.radialVolume } };
        auto& guard = target().editPhysicsComposite()
            .edit<dAttackGuardSimulation::PhysicsDeclaration>().bindings;
        guard.ownBodyId    = BodyId{ h.targetGuardBody };
        guard.parentBodyId = BodyId{ h.targetRoot };
        guard.shapeIds     = { ShapeId{ h.targetGuardShape } };
        exec.notifyCharacterRegistered(first, storage, staticData, true);
        exec.notifyCharacterRegistered(second, storage, staticData, true);
        reduce(0u);
    }

    SimulatableBrawler& attacker() { return storage.get<SimulatableBrawler>(h.attackerId); }
    SimulatableBrawler& target()   { return storage.get<SimulatableBrawler>(h.targetId); }

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
    int secondPressTick = -1;

    simulatableBrawler::PlayerInput attackerInput(int t) const
    {
        const bool press = t == kPressTick || t == secondPressTick;
        return makeInput(glm::vec3(1.f, 0.f, 0.f), press,
                         press ? glm::vec2(0.f, -1.f) : glm::vec2(0.f), 0u);
    }
    static simulatableBrawler::PlayerInput targetInput(bool guardFacesTheAttacker)
    {
        return guardFacesTheAttacker
            ? makeInput(guardFacingTheAttackerAim(), false, glm::vec2(0.f),
                        brawlerMovementSimulation::kInputFlagHoldGuard)
            : makeInput(guardTurnedAwayAim(), false, glm::vec2(0.f), 0u);
    }

    void reduce(std::uint32_t t)
    {
        const SimulationTimeStep s(t, false, StepKind::Normal, kDt);
        exec.firePreIntegrate(s, storage, staticData, /*isAuthority*/ true);
    }

    void step(int t, bool guardFacesTheAttacker)
    {
        ResolvedInputs<SimulatableBrawler> inputs;
        auto& map = std::get<0>(inputs);
        map.emplace(h.attackerId, attackerInput(t));
        map.emplace(h.targetId, targetInput(guardFacesTheAttacker));
        const SimulationTimeStep s(static_cast<std::uint32_t>(t), false, false, false, kDt);
        integration.integrateAll(s, inputs);
        exec.firePostIntegrate(s, storage, staticData, /*isAuthority*/ true);
        reduce(static_cast<std::uint32_t>(t) + 1u);
    }

    const dAttackRadialSimulation::DerivedState& attackerRadial()
    {
        return attacker().getAllState().getDerivedState().get<dAttackRadialSimulation::DerivedState>();
    }
    bool targetRoutedAHit()
    {
        return target().getAllState().getDerivedState().get<brawlerInboundHit::DerivedState>().wasHitThisTick;
    }
    static DAttackState machine(SimulatableBrawler& b)
    {
        return b.getAllState().getState().get<dAttackMachineSimulation::State>().m_currentState;
    }
};

inline const char* name(DAttackState s) { return dAttackMachineSimulation::dAttackStateName(s); }

struct SyncedBytes
{
    std::vector<std::uint8_t> bytes;

    template <typename T>
    void writeToBuffer(std::uint32_t off, const T& value)
    { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

    template <typename T>
    T readFromBuffer(std::uint32_t off) const
    { T v; std::memcpy(&v, bytes.data() + off, sizeof(T)); return v; }
};

inline std::vector<std::uint8_t> syncedBytesOf(SimulatableBrawler& b)
{
    SyncedBytes buffer;
    buffer.bytes.assign(4096u, 0xCDu);
    const std::uint32_t written = writeCompositeToSyncedBuffer(b.getAllState().getState(), buffer, 0u);
    buffer.bytes.resize(std::min<std::size_t>(written, buffer.bytes.size()));
    return buffer.bytes;
}

inline int firstDifference(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b)
{
    if (a.size() != b.size())
        return static_cast<int>(std::min(a.size(), b.size()));
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] != b[i])
            return static_cast<int>(i);
    return -1;
}

constexpr unsigned int  kMultiAttackerId   = 1u;
constexpr std::uint32_t kMultiAttackerRoot = 50u;
constexpr std::uint32_t kMultiRadialVolume = 59u;
constexpr int           kMultiTargetCount  = 4;
constexpr std::uint32_t kNoGuardBody       = 999u;

inline unsigned int  multiTargetId(int i)   { return 2u + static_cast<unsigned int>(i); }
inline std::uint32_t multiTargetRoot(int i) { return 51u + static_cast<std::uint32_t>(i); }

struct MultiQuery
{
    std::vector<int> overlapping;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds) const
    {
        SpatialQueryReport report;
        if (std::find(volumeIds.begin(), volumeIds.end(), QueryVolumeId{ kMultiRadialVolume }) == volumeIds.end())
            return report;
        for (const int i : overlapping)
        {
            SpatialQueryHit body{};
            body.objectPosition   = glm::vec3(kTargetDistance, 0.f, 0.f);
            body.bodyId           = BodyId{ multiTargetRoot(i) };
            body.rootBodyId       = BodyId{ multiTargetRoot(i) };
            body.objectCategories = CollisionCategories::single(collisionCategory::body);
            report.hits.push_back(body);
        }
        return report;
    }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};
static_assert(SpatialQueryAdapter<MultiQuery>);

struct MultiTargetRig
{
    using MultiDetection = brawlerHitDetection::System<Physics, MultiQuery>;
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
        simulatableBrawler::StaticData, MultiDetection, brawlerHitRouting::System>;
    using Integration = SimulationIntegrationExecutor<simulatableBrawler::StaticData, Physics, MultiQuery,
        SimulatableBrawler>;

    simulatableBrawler::StaticData staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    Physics    phys;
    MultiQuery query;
    Exec        exec;
    Integration integration;

    MultiTargetRig()
        : exec(std::piecewise_construct, MultiDetection(phys, query), brawlerHitRouting::System{})
        , integration(storage, staticData, phys, query)
    {
        phys.guardBody = kNoGuardBody;
        staticData.m_movementStaticData.drivesBody = false;
        storage.add<SimulatableBrawler>(kMultiAttackerId, SimulatableBrawler(staticData));
        attacker().setCharacterBindings({ BodyId{ kMultiAttackerRoot } });
        attacker().editPhysicsComposite().edit<dAttackRadialSimulation::PhysicsDeclaration>()
            .bindings.queryVolumeIds = { QueryVolumeId{ kMultiRadialVolume } };
        for (int i = 0; i < kMultiTargetCount; ++i)
        {
            storage.add<SimulatableBrawler>(multiTargetId(i), SimulatableBrawler(staticData));
            target(i).setCharacterBindings({ BodyId{ multiTargetRoot(i) } });
        }
        exec.notifyCharacterRegistered(kMultiAttackerId, storage, staticData, true);
        for (int i = 0; i < kMultiTargetCount; ++i)
            exec.notifyCharacterRegistered(multiTargetId(i), storage, staticData, true);
        const SimulationTimeStep first(0u, false, StepKind::Normal, kDt);
        exec.firePreIntegrate(first, storage, staticData, /*isAuthority*/ true);
    }

    SimulatableBrawler& attacker()    { return storage.get<SimulatableBrawler>(kMultiAttackerId); }
    SimulatableBrawler& target(int i) { return storage.get<SimulatableBrawler>(multiTargetId(i)); }

    void step(int t)
    {
        ResolvedInputs<SimulatableBrawler> inputs;
        auto& map = std::get<0>(inputs);
        const bool press = t == kPressTick;
        map.emplace(kMultiAttackerId, Rig::makeInput(glm::vec3(1.f, 0.f, 0.f), press,
                                                     press ? glm::vec2(0.f, -1.f) : glm::vec2(0.f), 0u));
        for (int i = 0; i < kMultiTargetCount; ++i)
            map.emplace(multiTargetId(i), Rig::makeInput(guardTurnedAwayAim(), false, glm::vec2(0.f), 0u));
        const SimulationTimeStep s(static_cast<std::uint32_t>(t), false, false, false, kDt);
        integration.integrateAll(s, inputs);
        exec.firePostIntegrate(s, storage, staticData, /*isAuthority*/ true);
        const SimulationTimeStep next(static_cast<std::uint32_t>(t) + 1u, false, StepKind::Normal, kDt);
        exec.firePreIntegrate(next, storage, staticData, /*isAuthority*/ true);
    }

    std::size_t hitsDetectedThisTick()
    {
        return attacker().getAllState().getDerivedState().get<dAttackRadialSimulation::DerivedState>()
            .getHitsThisTick().size();
    }
    bool targetRoutedAHit(int i)
    {
        return target(i).getAllState().getDerivedState().get<brawlerInboundHit::DerivedState>().wasHitThisTick;
    }
};

} // namespace hitDedupPinsTests

using namespace hitDedupPinsTests;

TEST_CASE("HitDedup.GuardBlockEndsTheSwingAndTheTargetIsNotHitLaterInIt",
          "[SimulatableBrawler][HitDedup]")
{
    SECTION("blocked on the contact tick, guard turned away from the next tick on")
    {
        Rig rig(kPeerA, false);
        bool blockedOnContact = false;
        DAttackState attackerAfterBlock = DAttackState::Idle;
        int ticksWithAHit = 0;
        int ticksTargetFlinched = 0;
        for (int t = 0; t <= kLastTick; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= kContactTick;
            rig.step(t, t <= kContactTick);
            if (t == kContactTick)
                blockedOnContact = rig.attackerRadial().getGuardBlockedThisTick();
            if (t == kContactTick + 1)
                attackerAfterBlock = Rig::machine(rig.attacker());
            if (t > kContactTick)
            {
                if (!rig.attackerRadial().getHitsThisTick().empty() || rig.targetRoutedAHit())
                    ++ticksWithAHit;
                if (Rig::machine(rig.target()) == DAttackState::HitFlinch)
                    ++ticksTargetFlinched;
            }
        }
        INFO("blocked on contact=" << blockedOnContact << " attacker on contact+1=" << name(attackerAfterBlock)
             << " ticks with a hit after the block=" << ticksWithAHit
             << " ticks the target spent in HitFlinch=" << ticksTargetFlinched);
        REQUIRE(blockedOnContact);
        CHECK(attackerAfterBlock == DAttackState::GuardFlinch);
        CHECK(ticksWithAHit == 0);
        CHECK(ticksTargetFlinched == 0);
    }

    SECTION("control: the same swing with contact from the tick after the block tick hits")
    {
        Rig rig(kPeerA, false);
        int firstHitTick = -1;
        DAttackState targetAfterFirstHit = DAttackState::Idle;
        for (int t = 0; t <= kLastTick; ++t)
        {
            rig.query.weaponOverlapsTarget = t >= kContactTick + 1;
            rig.step(t, false);
            if (firstHitTick < 0 && !rig.attackerRadial().getHitsThisTick().empty())
                firstHitTick = t;
            if (firstHitTick >= 0 && t == firstHitTick + 1)
                targetAfterFirstHit = Rig::machine(rig.target());
        }
        INFO("first hit detected over tick " << firstHitTick << ", target on the next tick="
             << name(targetAfterFirstHit));
        CHECK(firstHitTick == kContactTick + 1);
        CHECK(targetAfterFirstHit == DAttackState::HitFlinch);
    }
}

TEST_CASE("HitDedup.SyncedBytesMatchAcrossPeersWhoseEngineHandlesDiffer",
          "[SimulatableBrawler][HitDedup]")
{
    Rig peerA(kPeerA, false);
    Rig peerB(kPeerB, true);

    int attackerMismatchTicks = 0;
    int targetMismatchTicks = 0;
    int firstMismatchTick = -1;
    int firstMismatchOffset = -1;
    std::size_t attackerWireBytes = 0u;
    DAttackState targetOnContactA = DAttackState::Idle;
    DAttackState targetOnContactB = DAttackState::Idle;
    for (int t = 0; t <= kLastTick; ++t)
    {
        peerA.query.weaponOverlapsTarget = t >= kContactTick;
        peerB.query.weaponOverlapsTarget = t >= kContactTick;
        peerA.step(t, false);
        peerB.step(t, false);
        if (t == kContactTick + 1)
        {
            targetOnContactA = Rig::machine(peerA.target());
            targetOnContactB = Rig::machine(peerB.target());
        }

        const std::vector<std::uint8_t> attackerA = syncedBytesOf(peerA.attacker());
        const std::vector<std::uint8_t> attackerB = syncedBytesOf(peerB.attacker());
        attackerWireBytes = attackerA.size();
        const int attackerDiff = firstDifference(attackerA, attackerB);
        const int targetDiff = firstDifference(syncedBytesOf(peerA.target()), syncedBytesOf(peerB.target()));
        if (attackerDiff >= 0)
            ++attackerMismatchTicks;
        if (targetDiff >= 0)
            ++targetMismatchTicks;
        if (firstMismatchTick < 0 && (attackerDiff >= 0 || targetDiff >= 0))
        {
            firstMismatchTick = t;
            firstMismatchOffset = attackerDiff >= 0 ? attackerDiff : targetDiff;
        }
    }

    INFO("attacker synced composite " << attackerWireBytes << " B; ticks sampled " << (kLastTick + 1)
         << "; attacker mismatch ticks " << attackerMismatchTicks << ", target mismatch ticks "
         << targetMismatchTicks << "; first mismatch tick " << firstMismatchTick << " at byte "
         << firstMismatchOffset);
    INFO("A mismatch means a per-process engine handle reached a synced field. After task 27 that is "
         "a hitTargets entry written from a BodyId (or any handle other than the peer-stable "
         "SimCharacterId), which makes every correction of a mid-swing attacker compare unequal.");
    REQUIRE(attackerWireBytes > 0u);
    REQUIRE(attackerWireBytes < 4096u);
    REQUIRE(targetOnContactA == DAttackState::HitFlinch);
    REQUIRE(targetOnContactB == DAttackState::HitFlinch);
    REQUIRE(Rig::machine(peerA.attacker()) == DAttackState::Idle);
    CHECK(attackerMismatchTicks == 0);
    CHECK(targetMismatchTicks == 0);
}

TEST_CASE("HitDedup.AdoptingAnIdleAttackerMidSwingLetsTheNextSwingHit",
          "[SimulatableBrawler][HitDedup]")
{
    const int adoptTick   = kContactTick + 5;
    const int secondPress = adoptTick + 2;
    const int lastTick    = secondPress + 45;

    Rig rig(kPeerA, false);
    rig.secondPressTick = secondPress;
    simulatableBrawler::State idleAttacker;
    int hitsBeforeAdoption = 0;
    int hitsAfterAdoption = 0;
    int firstHitAfterAdoption = -1;
    DAttackState attackerAtAdoption = DAttackState::Idle;
    for (int t = 0; t <= lastTick; ++t)
    {
        rig.query.weaponOverlapsTarget = t >= kContactTick;
        rig.step(t, false);
        if (t == kPressTick - 1)
            idleAttacker = rig.attacker().getAllState().getState();
        const bool routed = rig.targetRoutedAHit();
        if (t <= adoptTick && routed)
            ++hitsBeforeAdoption;
        if (t > adoptTick && routed)
        {
            ++hitsAfterAdoption;
            if (firstHitAfterAdoption < 0)
                firstHitAfterAdoption = t;
        }
        if (t == adoptTick)
        {
            attackerAtAdoption = Rig::machine(rig.attacker());
            rig.attacker().editAllState().editState() = idleAttacker;
        }
    }
    INFO("hits before adoption=" << hitsBeforeAdoption << " attacker at adoption=" << name(attackerAtAdoption)
         << " hits after adoption=" << hitsAfterAdoption << " first over tick " << firstHitAfterAdoption);
    REQUIRE(hitsBeforeAdoption == 1);
    REQUIRE(attackerAtAdoption == DAttackState::Attacking);
    CHECK(hitsAfterAdoption == 1);
    CHECK(firstHitAfterAdoption == secondPress);
}

TEST_CASE("HitDedup.ASwingHitsAtMostThreeDistinctTargets",
          "[SimulatableBrawler][HitDedup]")
{
    auto run = [](const std::vector<int>& firstOverlapTick, std::vector<int>& hitTickOf,
                  std::size_t& maxDetectedInOnePass)
    {
        MultiTargetRig rig;
        hitTickOf.assign(kMultiTargetCount, -1);
        maxDetectedInOnePass = 0u;
        for (int t = 0; t <= kLastTick; ++t)
        {
            rig.query.overlapping.clear();
            for (int i = 0; i < kMultiTargetCount; ++i)
                if (t >= firstOverlapTick[i])
                    rig.query.overlapping.push_back(i);
            rig.step(t);
            maxDetectedInOnePass = std::max(maxDetectedInOnePass, rig.hitsDetectedThisTick());
            for (int i = 0; i < kMultiTargetCount; ++i)
                if (rig.targetRoutedAHit(i) && hitTickOf[i] < 0)
                    hitTickOf[i] = t;
        }
    };
    auto hitCount = [](const std::vector<int>& hitTickOf)
    { return std::count_if(hitTickOf.begin(), hitTickOf.end(), [](int t) { return t >= 0; }); };

    SECTION("four targets in one pass: three are hit, the fourth is not")
    {
        std::vector<int> hitTickOf;
        std::size_t maxInOnePass = 0u;
        run({ kContactTick, kContactTick, kContactTick, kContactTick }, hitTickOf, maxInOnePass);
        INFO("hit ticks " << hitTickOf[0] << " " << hitTickOf[1] << " " << hitTickOf[2] << " " << hitTickOf[3]
             << "; most hits detected in one pass " << maxInOnePass);
        CHECK(maxInOnePass == 3u);
        CHECK(hitCount(hitTickOf) == 3);
        CHECK(hitTickOf[0] == kContactTick);
        CHECK(hitTickOf[1] == kContactTick);
        CHECK(hitTickOf[2] == kContactTick);
        CHECK(hitTickOf[3] == -1);
    }

    SECTION("four targets on four ticks: three are hit, the fourth is not")
    {
        std::vector<int> hitTickOf;
        std::size_t maxInOnePass = 0u;
        run({ kContactTick, kContactTick + 1, kContactTick + 2, kContactTick + 3 }, hitTickOf, maxInOnePass);
        INFO("hit ticks " << hitTickOf[0] << " " << hitTickOf[1] << " " << hitTickOf[2] << " " << hitTickOf[3]);
        CHECK(maxInOnePass == 1u);
        CHECK(hitCount(hitTickOf) == 3);
        CHECK(hitTickOf[0] == kContactTick);
        CHECK(hitTickOf[1] == kContactTick + 1);
        CHECK(hitTickOf[2] == kContactTick + 2);
        CHECK(hitTickOf[3] == -1);
    }

    SECTION("control: the fourth target alone is hit on the tick the others would have used")
    {
        std::vector<int> hitTickOf;
        std::size_t maxInOnePass = 0u;
        run({ 1000, 1000, 1000, kContactTick + 3 }, hitTickOf, maxInOnePass);
        INFO("fourth target hit over tick " << hitTickOf[3]);
        CHECK(hitCount(hitTickOf) == 1);
        CHECK(hitTickOf[3] == kContactTick + 3);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
