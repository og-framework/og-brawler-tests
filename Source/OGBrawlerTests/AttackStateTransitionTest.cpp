// SPDX-License-Identifier: BUSL-1.1
// docs/AttackStateTransitionTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "BrawlerTestInputs.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerHitRoutingSystem.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/InputSequence/GameMotions.h"
#include "OGBrawler/OGBrawlerLog.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/StorageView.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <vector>

namespace attackTransitionTests
{

struct MockPhysicsAdapter
{
    glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
    void setBodyTransform(BodyId, const glm::mat4&) {}
    void addBodyTorque(BodyId, const glm::vec3&) {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void setBodyLinearVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&) {}
    void addBodyVelocityChange(BodyId, const glm::vec3&) {}
    glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

struct MockSpatialQueryAdapter
{
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

inline float degrees(float d)
{
    return d * inputSequence::pi / 180.f;
}

inline glm::vec3 stickAt(const glm::vec3& aim, float aimRelativeAngle)
{
    const glm::vec3 forward = glm::normalize(glm::vec3(aim.x, aim.y, 0.f));
    const glm::vec3 right(forward.y, -forward.x, 0.f);
    return std::cos(aimRelativeAngle) * forward + std::sin(aimRelativeAngle) * right;
}

struct TickRecord
{
    std::uint32_t tick = 0u;
    DAttackState  state = DAttackState::Idle;
    unsigned int  activeSequence = InvalidAttackSequenceId;
    unsigned int  queuedSequence = InvalidAttackSequenceId;
    std::uint32_t attackEndTick = 0u;
    bool          fired = false;
    bool          spawned = false;
};

struct FAttackRig
{
    static constexpr float kDt = 1.f / 60.f;

    simulatableBrawler::StaticData              staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    brawlerHitRouting::System                   routing;
    SimulatableBrawler&                         character;
    MockPhysicsAdapter                          physAdapter;
    MockSpatialQueryAdapter                     queryAdapter;

    float         dt = kDt;
    glm::vec3     aim{ 1.f, 0.f, 0.f };
    glm::vec3     moveWorld{ 0.f, 0.f, 0.f };
    glm::vec2     moveStick{ 0.f, 0.f };
    std::uint32_t nextTick = 1u;

    std::vector<std::string>              logLines;
    std::function<void(const char*)>      previousSink;
    std::vector<TickRecord>               history;
    std::set<std::uint32_t>               spawnTicksSeen;
    std::uint32_t                         slotReuses = 0u;
    bool                                  blockLiveShotOnNextTick = false;
    int                                   lastBlockedSlot = -1;
    brawlerProjectileSimulation::ProjectileSlot blockedSlotAtTheBlockTick;

    static SimulatableBrawler& addShooter(SimulationObjectStorage<SimulatableBrawler>& into,
                                          const simulatableBrawler::StaticData& data)
    {
        into.add<SimulatableBrawler>(0u, SimulatableBrawler(data));
        return into.get<SimulatableBrawler>(0u);
    }

    FAttackRig()
        : character(addShooter(storage, staticData))
        , previousSink(::ogblog::g_sink)
    {
        character.setCharacterBindings({ BodyId{ 1 } });
        routing.onCharacterRegistered(0u, view(), staticData);
        ::ogblog::setGlobal([this](const char* line) { logLines.emplace_back(line); });
    }

    ~FAttackRig()
    {
        ::ogblog::setGlobal(std::move(previousSink));
    }

    FAttackRig(const FAttackRig&) = delete;
    FAttackRig& operator=(const FAttackRig&) = delete;

    StorageView<SimulatableBrawler> view()
    {
        return storage.projectTo<SimulatableList<SimulatableBrawler>>();
    }

    const dAttackMachineSimulation::State& machine() const
    {
        return character.getAllState().getState().get<dAttackMachineSimulation::State>();
    }

    const dAttackRadialSimulation::State& radial() const
    {
        return character.getAllState().getState().get<dAttackRadialSimulation::State>();
    }

    const brawlerProjectileSimulation::State& projectiles() const
    {
        return character.getAllState().getState().get<brawlerProjectileSimulation::State>();
    }

    std::uint32_t commitmentTicks() const
    {
        return dAttackMachineSimulation::swingTickCount(kHadoukenCommitmentSeconds, dt);
    }

    void holdStick(float aimRelativeAngle)
    {
        moveWorld = stickAt(aim, aimRelativeAngle);
        moveStick = glm::vec2(moveWorld.x, moveWorld.y);
    }

    void holdStickOffBack(float offsetFromBack)
    {
        holdStick(inputSequence::angle::Back + offsetFromBack);
    }

    void releaseStick()
    {
        moveWorld = glm::vec3(0.f);
        moveStick = glm::vec2(0.f);
    }

    std::size_t logLinesContaining(const std::string& needle) const
    {
        std::size_t n = 0u;
        for (const std::string& line : logLines)
            n += line.find(needle) != std::string::npos ? 1u : 0u;
        return n;
    }

    TickRecord tick(bool attackLeft, bool attackRight, std::uint32_t triggeredActionId = 0u)
    {
        const std::uint32_t t = nextTick++;

        std::vector<std::uint32_t> slotSpawnTicksBefore;
        for (const brawlerProjectileSimulation::ProjectileSlot& slot : projectiles().slots)
            slotSpawnTicksBefore.push_back(slot.spawnTick);

        simulatableBrawler::PlayerInput input = brawlerTestInputs::make({
            .aimDirection       = aim,
            .attackLeft         = attackLeft,
            .attackRight        = attackRight,
            .moveStick          = moveStick,
            .moveDirectionWorld = moveWorld,
            .triggeredActionId  = triggeredActionId });
        const SimulationTimeStep step(t, false, false, false, dt);
        const bool blockThisTick = blockLiveShotOnNextTick;
        if (blockThisTick)
            poseBlockOfNewestLiveShot(step);
        character.integrate(step, input, physAdapter, queryAdapter, staticData);
        if (blockThisTick)
            blockedSlotAtTheBlockTick = projectiles().slots[lastBlockedSlot];
        character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>() = brawlerInboundHit::DerivedState{};
        character.editAllState().editDerivedState()
            .edit<brawlerProjectileSimulation::DerivedState>().detectedThisTick.fill(
                brawlerProjectileSimulation::SlotDetection{});

        TickRecord record;
        record.tick           = t;
        record.state          = machine().m_currentState;
        record.activeSequence = machine().m_activeAttackSequence;
        record.queuedSequence = machine().m_queuedAttackSequence;
        record.attackEndTick  = machine().m_attackEndTick;
        record.fired          = record.state == DAttackState::Attacking
                                && record.activeSequence == kHadoukenSequenceSentinel
                                && machine().m_timeInCurrentState == 0.f;

        const auto& slots = projectiles().slots;
        for (std::size_t i = 0; i < slots.size(); ++i)
        {
            if (slots[i].spawnTick == t)
            {
                record.spawned = true;
                spawnTicksSeen.insert(t);
                slotReuses += slotSpawnTicksBefore[i] != 0u ? 1u : 0u;
            }
        }

        history.push_back(record);
        return record;
    }

    void poseBlockOfNewestLiveShot(const SimulationTimeStep& step)
    {
        blockLiveShotOnNextTick = false;
        lastBlockedSlot = -1;
        const auto& slots = projectiles().slots;
        for (std::size_t i = 0; i < slots.size(); ++i)
            if (slots[i].isAlive(step.getTick()) && slots[i].spawnTick < step.getTick()
                && (lastBlockedSlot < 0 || slots[i].spawnTick > slots[lastBlockedSlot].spawnTick))
                lastBlockedSlot = static_cast<int>(i);
        REQUIRE(lastBlockedSlot >= 0);

        const brawlerProjectileSimulation::ProjectileSlot& slot = slots[lastBlockedSlot];
        brawlerProjectileSimulation::SlotDetection& detected = character.editAllState().editDerivedState()
            .edit<brawlerProjectileSimulation::DerivedState>().detectedThisTick[lastBlockedSlot];
        detected.outcome            = brawlerProjectileSimulation::SlotOutcome::BlockedByGuard;
        detected.struckRootBodyId   = BodyId{ 2 };
        detected.targetRootPosition = slot.spawnPos + slot.spawnDir * 300.f;
        detected.objectPosition     = detected.targetRootPosition - slot.spawnDir * 40.f;
        routing.preIntegrate(step, view(), staticData);
    }

    const brawlerProjectileSimulation::ProjectileSlot& blockedSlot() const
    {
        REQUIRE(lastBlockedSlot >= 0);
        return blockedSlotAtTheBlockTick;
    }

    bool everIn(DAttackState state) const
    {
        return std::any_of(history.begin(), history.end(),
                           [state](const TickRecord& r) { return r.state == state; });
    }

    std::vector<std::uint32_t> firedTicks() const
    {
        std::vector<std::uint32_t> ticks;
        for (const TickRecord& r : history)
            if (r.fired)
                ticks.push_back(r.tick);
        return ticks;
    }

    std::vector<std::uint32_t> spawnedTicks() const
    {
        std::vector<std::uint32_t> ticks;
        for (const TickRecord& r : history)
            if (r.spawned)
                ticks.push_back(r.tick);
        return ticks;
    }

    std::vector<std::uint32_t> realSequenceTicks() const
    {
        std::vector<std::uint32_t> ticks;
        for (const TickRecord& r : history)
            if (isRealAttackSequence(r.activeSequence) || isRealAttackSequence(r.queuedSequence))
                ticks.push_back(r.tick);
        return ticks;
    }

    void run(std::uint32_t ticks, bool attackLeft, bool attackRight)
    {
        for (std::uint32_t i = 0; i < ticks; ++i)
            tick(attackLeft, attackRight);
    }

    std::vector<unsigned int> swingSequences() const
    {
        std::vector<unsigned int> swings;
        const TickRecord* previous = nullptr;
        for (const TickRecord& r : history)
        {
            const bool swinging = r.state == DAttackState::Attacking && isRealAttackSequence(r.activeSequence);
            const bool wasSwinging = previous != nullptr
                && previous->state == DAttackState::Attacking
                && isRealAttackSequence(previous->activeSequence);
            if (swinging && (!wasSwinging
                             || previous->activeSequence != r.activeSequence
                             || previous->attackEndTick != r.attackEndTick))
                swings.push_back(r.activeSequence);
            previous = &r;
        }
        return swings;
    }

    std::vector<unsigned int> queuedSequences() const
    {
        std::vector<unsigned int> queued;
        for (const TickRecord& r : history)
            if (r.queuedSequence != InvalidAttackSequenceId
                && (queued.empty() || queued.back() != r.queuedSequence))
                queued.push_back(r.queuedSequence);
        return queued;
    }

    std::string trace() const
    {
        std::string text;
        for (const TickRecord& r : history)
        {
            text += std::to_string(r.tick) + ":" + dAttackMachineSimulation::dAttackStateName(r.state)
                  + "/" + (r.activeSequence == kHadoukenSequenceSentinel ? std::string("H")
                           : r.activeSequence == InvalidAttackSequenceId ? std::string("-")
                           : std::to_string(r.activeSequence))
                  + (r.queuedSequence != InvalidAttackSequenceId ? "q" + std::to_string(r.queuedSequence) : std::string())
                  + (r.spawned ? "*" : "") + " ";
        }
        return text;
    }
};

inline constexpr float kRightOfAim = inputSequence::pi / 2.f;
inline constexpr float kLeftOfAim  = -inputSequence::pi / 2.f;

inline constexpr unsigned int kAuthoredRightFollowUp = 2u;
inline constexpr unsigned int kAuthoredLeftFollowUp  = 3u;

inline unsigned int authoredFollowUpOf(unsigned int side)
{
    return side == dAttackDirection::kRightSequenceId ? kAuthoredRightFollowUp : kAuthoredLeftFollowUp;
}

inline float sideAngle(unsigned int side)
{
    return side == dAttackDirection::kRightSequenceId ? kRightOfAim : kLeftOfAim;
}

inline unsigned int classified(const FAttackRig& rig)
{
    return dAttackDirection::classify(rig.aim, rig.moveWorld, rig.moveStick);
}

inline constexpr std::uint32_t kSwingRunTicks = 200u;
inline constexpr float kFollowUpWindowSeconds = 0.3f;

inline std::uint32_t fireAndSettle(FAttackRig& rig)
{
    rig.holdStickOffBack(0.f);
    rig.tick(false, true, inputSequence::kHadoukenActionId);
    rig.releaseStick();
    const std::uint32_t idleTick = 1u + rig.commitmentTicks();
    while (rig.nextTick <= idleTick + 2u)
        rig.tick(false, false);
    return idleTick;
}

inline std::vector<std::uint32_t> everyPeriod(std::uint32_t first, std::uint32_t period, std::size_t count)
{
    std::vector<std::uint32_t> ticks;
    for (std::size_t i = 0; i < count; ++i)
        ticks.push_back(first + static_cast<std::uint32_t>(i) * period);
    return ticks;
}

} // namespace attackTransitionTests

TEST_CASE("DAttack.AttackTransitions.HeldBackAndAttack2RefiresProjectile", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    FAttackRig rig;
    rig.holdStickOffBack(0.f);

    const std::uint32_t period = rig.commitmentTicks();
    constexpr std::size_t kShots = 7u;
    REQUIRE(period > 1u);

    rig.tick(false, true, inputSequence::kHadoukenActionId);
    while (rig.nextTick <= 1u + (kShots - 1u) * period + period / 2u)
        rig.tick(false, true);

    const std::vector<std::uint32_t> expected = everyPeriod(1u, period, kShots);
    INFO("commitment period = " << period << " ticks; first real sequence tick(s): "
         << Catch::StringMaker<std::vector<std::uint32_t>>::convert(rig.realSequenceTicks()));
    CHECK(rig.realSequenceTicks().empty());
    CHECK(rig.firedTicks() == expected);
    CHECK(rig.spawnedTicks() == expected);

    for (const TickRecord& r : rig.history)
    {
        INFO("tick " << r.tick << " state=" << dAttackMachineSimulation::dAttackStateName(r.state) << " activeSeq=" << r.activeSequence
             << " endTick=" << r.attackEndTick);
        REQUIRE(r.state == DAttackState::Attacking);
        REQUIRE(r.activeSequence == kHadoukenSequenceSentinel);
        if (r.fired)
            REQUIRE(r.attackEndTick == r.tick + period);
    }
}

TEST_CASE("DAttack.AttackTransitions.NoShotDroppedAtDefaultPool", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    FAttackRig rig;
    rig.holdStickOffBack(0.f);

    const brawlerProjectileSimulation::StaticData& projectileSD = rig.staticData.m_projectileStaticData;
    const std::uint32_t period = rig.commitmentTicks();
    constexpr std::size_t kShots = 10u;
    INFO("pool=" << projectileSD.projectilePoolSize << " lifetime=" << projectileSD.maxLifetime
         << " s period=" << period << " ticks at dt=" << FAttackRig::kDt);

    rig.tick(false, true, inputSequence::kHadoukenActionId);
    while (rig.nextTick <= 1u + (kShots - 1u) * period + period / 2u)
        rig.tick(false, true);

    const std::vector<std::uint32_t> fired = rig.firedTicks();
    const std::vector<std::uint32_t> spawned(rig.spawnTicksSeen.begin(), rig.spawnTicksSeen.end());
    CHECK(fired.size() == kShots);
    CHECK(spawned == fired);
    CHECK(rig.logLinesContaining("[Projectile.poolFull]") == 0u);
    CHECK(rig.slotReuses > 0u);
}

TEST_CASE("DAttack.AttackTransitions.ReleaseStopsFire", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    SECTION("releasing attack 2 after a repeat shot")
    {
        FAttackRig rig;
        rig.holdStickOffBack(0.f);
        const std::uint32_t period = rig.commitmentTicks();

        rig.tick(false, true, inputSequence::kHadoukenActionId);
        while (rig.nextTick <= 1u + period)
            rig.tick(false, true);
        REQUIRE(rig.firedTicks() == everyPeriod(1u, period, 2u));

        const std::uint32_t lastShot = 1u + period;
        while (rig.nextTick < lastShot + period)
        {
            const TickRecord r = rig.tick(false, false);
            INFO("tick " << r.tick);
            CHECK(r.state == DAttackState::Attacking);
            CHECK(r.activeSequence == kHadoukenSequenceSentinel);
        }

        const TickRecord exit = rig.tick(false, false);
        CHECK(exit.tick == lastShot + period);
        CHECK(exit.state == DAttackState::Idle);

        while (rig.nextTick <= lastShot + 3u * period)
            rig.tick(false, false);
        CHECK(rig.spawnedTicks() == everyPeriod(1u, period, 2u));
    }

    SECTION("stick leaving the back cone with attack 2 still held")
    {
        FAttackRig rig;
        rig.holdStickOffBack(0.f);
        const std::uint32_t period = rig.commitmentTicks();

        rig.tick(false, true, inputSequence::kHadoukenActionId);
        rig.holdStickOffBack(degrees(30.f));
        while (rig.nextTick < 1u + period)
            rig.tick(false, true);

        const TickRecord exit = rig.tick(false, true);
        INFO("exit tick " << exit.tick << " state=" << dAttackMachineSimulation::dAttackStateName(exit.state)
             << " activeSeq=" << exit.activeSequence);
        CHECK(exit.tick == 1u + period);
        CHECK_FALSE(exit.spawned);
        CHECK(exit.activeSequence != kHadoukenSequenceSentinel);
        CHECK(exit.state == DAttackState::Idle);

        for (std::uint32_t i = 0; i < 3u * period; ++i)
            rig.tick(false, true);
        CHECK(rig.spawnedTicks() == std::vector<std::uint32_t>{ 1u });
    }
}

TEST_CASE("DAttack.AttackTransitions.HeldFromIdleFires", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    FAttackRig rig;
    rig.holdStickOffBack(0.f);

    const TickRecord r = rig.tick(false, true, 0u);
    INFO("state=" << dAttackMachineSimulation::dAttackStateName(r.state) << " activeSeq=" << r.activeSequence);
    CHECK(r.state == DAttackState::Attacking);
    CHECK(r.activeSequence == kHadoukenSequenceSentinel);
    CHECK(r.fired);
    CHECK(r.spawned);
    CHECK(r.attackEndTick == r.tick + rig.commitmentTicks());
}

TEST_CASE("DAttack.AttackTransitions.BackConeIsTheMatchersCone", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    const float inside  = degrees(15.f);
    const float outside = degrees(30.f);
    REQUIRE(inside > dAttackDirection::kBackpedalConeHalfAngle);
    REQUIRE(inside < kBackShortcutStep.angleTolerance);
    REQUIRE(outside > kBackShortcutStep.angleTolerance);
    REQUIRE(outside < inputSequence::pi - dAttackDirection::kForwardConeHalfAngle);

    for (const float sign : { 1.f, -1.f })
    {
        {
            FAttackRig rig;
            rig.holdStickOffBack(sign * inside);
            const unsigned int classified =
                dAttackDirection::classify(rig.aim, rig.moveWorld, rig.moveStick);
            const TickRecord r = rig.tick(false, true);
            INFO("15 deg off back, side " << sign << ": classifier says " << classified
                 << ", machine activeSeq=" << r.activeSequence);
            CHECK(classified != dAttackDirection::kForwardSequenceId);
            CHECK(r.activeSequence == kHadoukenSequenceSentinel);
            CHECK(r.spawned);
        }
        {
            FAttackRig rig;
            rig.holdStickOffBack(sign * outside);
            const unsigned int classified =
                dAttackDirection::classify(rig.aim, rig.moveWorld, rig.moveStick);
            const TickRecord r = rig.tick(false, true);
            INFO("30 deg off back, side " << sign << ": classifier says " << classified
                 << ", machine activeSeq=" << r.activeSequence);
            CHECK(r.state == DAttackState::Attacking);
            CHECK(r.activeSequence == classified);
            CHECK_FALSE(r.spawned);
        }
    }
}

TEST_CASE("DAttack.AttackTransitions.StickMatchesStepAtTheBoundaries", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;
    using inputSequence::stickMatchesStep;

    const glm::vec3 aim(1.f, 0.f, 0.f);
    const float tolerance = kBackShortcutStep.angleTolerance;
    const float margin = degrees(0.1f);
    constexpr float kDeadzone = 0.2f;

    for (const float sign : { 1.f, -1.f })
    {
        INFO("side " << sign);
        CHECK(stickMatchesStep(stickAt(aim, inputSequence::angle::Back + sign * (tolerance - margin)),
                               aim, kBackShortcutStep, kDeadzone));
        CHECK_FALSE(stickMatchesStep(stickAt(aim, inputSequence::angle::Back + sign * (tolerance + margin)),
                                     aim, kBackShortcutStep, kDeadzone));
    }

    const glm::vec3 deadBack = stickAt(aim, inputSequence::angle::Back);
    CHECK(stickMatchesStep(deadBack, aim, kBackShortcutStep, kDeadzone));
    CHECK_FALSE(stickMatchesStep(deadBack * 0.1f, aim, kBackShortcutStep, kDeadzone));
    CHECK(stickMatchesStep(deadBack * 0.1f, aim, kBackShortcutStep, 0.05f));
    CHECK_FALSE(stickMatchesStep(glm::vec3(0.f), aim, kBackShortcutStep, dAttackDirection::kMoveMagnitudeEpsilon));
    CHECK(stickMatchesStep(deadBack, glm::normalize(glm::vec3(1.f, 0.f, -1.f)), kBackShortcutStep, kDeadzone));
    CHECK(stickMatchesStep(deadBack + glm::vec3(0.f, 0.f, 5.f), aim, kBackShortcutStep, kDeadzone));
    CHECK_FALSE(stickMatchesStep(deadBack, glm::vec3(0.f, 0.f, 1.f), kBackShortcutStep, kDeadzone));
}

TEST_CASE("DAttack.AttackTransitions.StickMatchesStepIsTheMatchersPerFrameTest", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    REQUIRE(kGameMotions.size() == 3u);
    const inputSequence::MotionCommand& backShortcut = kGameMotions[2];
    REQUIRE(backShortcut.steps.size() == 1u);
    CHECK(backShortcut.steps[0].targetAngle == inputSequence::angle::Back);
    CHECK(backShortcut.steps[0].angleTolerance == inputSequence::pi / 8.f);
    CHECK(backShortcut.steps[0].maxGapFrames == 8u);
    CHECK(kBackShortcutStep.targetAngle == backShortcut.steps[0].targetAngle);
    CHECK(kBackShortcutStep.angleTolerance == backShortcut.steps[0].angleTolerance);
    CHECK(kBackShortcutStep.maxGapFrames == backShortcut.steps[0].maxGapFrames);

    const glm::vec3 aim = glm::normalize(glm::vec3(0.6f, -0.8f, -0.3f));
    constexpr float kDeadzone = 0.2f;
    constexpr std::uint32_t kTick = 40u;
    const std::vector<inputSequence::MotionCommand> onlyBackShortcut{ backShortcut };

    std::size_t matches = 0u;
    for (int d = 0; d < 360; ++d)
    {
        simulatableBrawler::PlayerInput entry{};
        entry.aimDirection       = aim;
        entry.moveDirectionWorld = stickAt(aim, degrees(static_cast<float>(d)));

        const bool predicate = inputSequence::stickMatchesStep(
            entry.moveDirectionWorld, entry.aimDirection, kBackShortcutStep, kDeadzone);
        const std::uint32_t matched = inputSequence::matchSequence(
            [&](std::uint32_t t) { return t == kTick - 1u ? &entry : nullptr; },
            kTick, glm::vec2(0.f), aim, 0b10, 0b10, kDeadzone, onlyBackShortcut);

        INFO("aim-relative angle " << d << " deg");
        CHECK(predicate == (matched == inputSequence::kHadoukenActionId));
        matches += predicate ? 1u : 0u;
    }
    CHECK(matches > 0u);
    CHECK(matches < 360u);
}

TEST_CASE("DAttack.AttackTransitions.LeftSwingFollowUpWithAttack1", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    FAttackRig rig;
    rig.holdStick(kLeftOfAim);
    REQUIRE(classified(rig) == dAttackDirection::kLeftSequenceId);

    rig.run(kSwingRunTicks, true, false);

    const std::vector<unsigned int> swings = rig.swingSequences();
    INFO("swings " << Catch::StringMaker<std::vector<unsigned int>>::convert(swings) << "\n" << rig.trace());
    REQUIRE(swings.size() >= 3u);
    CHECK(swings[0] == dAttackDirection::kLeftSequenceId);
    CHECK(swings[1] == kAuthoredLeftFollowUp);
    CHECK(swings[2] == kAuthoredLeftFollowUp);
    CHECK(rig.logLinesContaining("[Machine.transition] Attacking -> Attacking (queued seq=3)") >= 2u);
}

TEST_CASE("DAttack.AttackTransitions.RightSwingFollowUpWithAttack2", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    FAttackRig rig;
    rig.holdStick(kRightOfAim);
    REQUIRE(classified(rig) == dAttackDirection::kRightSequenceId);

    rig.run(kSwingRunTicks, false, true);

    const std::vector<unsigned int> swings = rig.swingSequences();
    INFO("swings " << Catch::StringMaker<std::vector<unsigned int>>::convert(swings) << "\n" << rig.trace());
    REQUIRE(swings.size() >= 3u);
    CHECK(swings[0] == dAttackDirection::kRightSequenceId);
    CHECK(swings[1] == kAuthoredRightFollowUp);
    CHECK(swings[2] == kAuthoredRightFollowUp);
    CHECK(rig.firedTicks().empty());
}

TEST_CASE("DAttack.AttackTransitions.PreviouslyWorkingFollowUpsStillChain", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    struct Row { unsigned int side; bool attackLeft; bool attackRight; };
    const Row rows[] = {
        { dAttackDirection::kRightSequenceId, true,  false },
        { dAttackDirection::kLeftSequenceId,  false, true  },
    };

    for (const Row& row : rows)
    {
        FAttackRig rig;
        rig.holdStick(sideAngle(row.side));
        REQUIRE(classified(rig) == row.side);

        rig.run(kSwingRunTicks, row.attackLeft, row.attackRight);

        const std::vector<unsigned int> swings = rig.swingSequences();
        INFO("side " << row.side << " L=" << row.attackLeft << " R=" << row.attackRight << " swings "
             << Catch::StringMaker<std::vector<unsigned int>>::convert(swings) << "\n" << rig.trace());
        REQUIRE(swings.size() >= 3u);
        CHECK(swings[0] == row.side);
        CHECK(swings[1] == authoredFollowUpOf(row.side));
        CHECK(swings[2] == authoredFollowUpOf(row.side));
    }
}

TEST_CASE("DAttack.AttackTransitions.OppositeOrForwardPressIsNotAFollowUp", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    struct Buttons { bool attackLeft; bool attackRight; };
    const Buttons buttonRows[] = { { true, false }, { false, true } };
    const unsigned int sides[] = { dAttackDirection::kRightSequenceId, dAttackDirection::kLeftSequenceId };

    for (const Buttons& buttons : buttonRows)
    {
        for (const unsigned int side : sides)
        {
            const unsigned int opposite = side == dAttackDirection::kRightSequenceId
                ? dAttackDirection::kLeftSequenceId : dAttackDirection::kRightSequenceId;

            struct Then { const char* name; bool neutral; float angle; unsigned int expectedNext; };
            const Then thens[] = {
                { "opposite side", false, sideAngle(opposite), opposite },
                { "forward",       false, 0.f,                 dAttackDirection::kForwardSequenceId },
                { "neutral stick", true,  0.f,                 dAttackDirection::kForwardSequenceId },
            };

            for (const Then& then : thens)
            {
                FAttackRig rig;
                rig.holdStick(sideAngle(side));
                const TickRecord start = rig.tick(buttons.attackLeft, buttons.attackRight);
                REQUIRE(start.activeSequence == side);

                if (then.neutral)
                    rig.releaseStick();
                else
                    rig.holdStick(then.angle);
                REQUIRE(classified(rig) == then.expectedNext);

                while (rig.nextTick <= start.attackEndTick + 2u)
                    rig.tick(buttons.attackLeft, buttons.attackRight);

                const std::vector<unsigned int> swings = rig.swingSequences();
                INFO("start side " << side << ", then " << then.name << ", L=" << buttons.attackLeft
                     << " R=" << buttons.attackRight << " swings "
                     << Catch::StringMaker<std::vector<unsigned int>>::convert(swings) << "\n" << rig.trace());
                CHECK(rig.queuedSequences().empty());
                CHECK(rig.history[start.attackEndTick - 1u].state == DAttackState::Idle);
                REQUIRE(swings.size() == 2u);
                CHECK(swings[0] == side);
                CHECK(swings[1] == then.expectedNext);
            }
        }
    }
}

TEST_CASE("DAttack.AttackTransitions.FirstSwingAfterProjectileIsNormal", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    struct Buttons { bool attackLeft; bool attackRight; };
    const Buttons buttonRows[] = { { true, false }, { false, true } };
    const unsigned int sides[] = { dAttackDirection::kRightSequenceId, dAttackDirection::kLeftSequenceId };

    for (const Buttons& buttons : buttonRows)
    {
        for (const unsigned int side : sides)
        {
            FAttackRig rig;
            const std::uint32_t idleTick = fireAndSettle(rig);
            REQUIRE(rig.history[idleTick - 1u].state == DAttackState::Idle);
            REQUIRE(rig.spawnedTicks() == std::vector<std::uint32_t>{ 1u });

            rig.holdStick(sideAngle(side));
            REQUIRE(classified(rig) == side);
            rig.run(kSwingRunTicks, buttons.attackLeft, buttons.attackRight);

            const std::vector<unsigned int> swings = rig.swingSequences();
            INFO("side " << side << " L=" << buttons.attackLeft << " R=" << buttons.attackRight << " swings "
                 << Catch::StringMaker<std::vector<unsigned int>>::convert(swings) << "\n" << rig.trace());
            REQUIRE(swings.size() >= 2u);
            CHECK(swings[0] == side);
            CHECK(swings[1] == authoredFollowUpOf(side));
        }
    }
}

TEST_CASE("DAttack.AttackTransitions.ProjectileInputMidSwingIsNotAFollowUp", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    FAttackRig rig;

    float leftOfBack = degrees(15.f);
    rig.holdStickOffBack(leftOfBack);
    if (classified(rig) != dAttackDirection::kLeftSequenceId)
        leftOfBack = -leftOfBack;
    rig.holdStickOffBack(leftOfBack);
    REQUIRE(classified(rig) == dAttackDirection::kLeftSequenceId);
    REQUIRE(dAttackMachineSimulation::wantsHeldProjectile(dAttackMachineSimulation::PlayerInputView{
        .aimDirection = rig.aim, .attackLeft = false, .attackRight = true,
        .moveDirection = rig.moveStick, .moveDirectionWorld = rig.moveWorld }));

    rig.holdStick(kLeftOfAim);
    const TickRecord start = rig.tick(true, false);
    REQUIRE(start.activeSequence == dAttackDirection::kLeftSequenceId);

    const std::uint32_t pastFollowUpWindow =
        1u + dAttackMachineSimulation::swingTickCount(kFollowUpWindowSeconds, rig.dt) + 6u;
    REQUIRE(pastFollowUpWindow + 4u < start.attackEndTick);
    while (rig.nextTick < pastFollowUpWindow)
        rig.tick(false, false);
    REQUIRE(rig.radial().attackTimer > 0.3f);

    rig.holdStickOffBack(leftOfBack);
    while (rig.nextTick <= start.attackEndTick + 3u)
        rig.tick(false, true);

    INFO("swing end tick " << start.attackEndTick << "\n" << rig.trace());
    CHECK(rig.queuedSequences().empty());
    CHECK(rig.swingSequences() == std::vector<unsigned int>{ dAttackDirection::kLeftSequenceId });
    CHECK(rig.history[start.attackEndTick - 1u].state == DAttackState::Idle);
    REQUIRE_FALSE(rig.firedTicks().empty());
    CHECK(rig.firedTicks().front() == start.attackEndTick + 1u);
    CHECK(rig.spawnedTicks().front() == start.attackEndTick + 1u);
}

TEST_CASE("DAttack.AttackTransitions.DualPressCannotHijackProjectile", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    SECTION("a dual press with a neutral stick during the commitment")
    {
        FAttackRig rig;
        const std::uint32_t period = rig.commitmentTicks();
        rig.holdStickOffBack(0.f);
        rig.tick(false, true, inputSequence::kHadoukenActionId);
        rig.releaseStick();

        rig.run(3u, false, false);
        rig.run(3u, true, true);
        while (rig.nextTick <= 1u + 3u * period)
            rig.tick(false, false);

        INFO(rig.trace());
        CHECK(rig.realSequenceTicks().empty());
        CHECK(rig.spawnedTicks() == std::vector<std::uint32_t>{ 1u });
        for (const TickRecord& r : rig.history)
        {
            if (r.tick < 1u + period)
            {
                INFO("tick " << r.tick);
                CHECK(r.state == DAttackState::Attacking);
                CHECK(r.activeSequence == kHadoukenSequenceSentinel);
            }
        }
        CHECK(rig.history[period].state == DAttackState::Idle);
    }

    SECTION("both buttons held with the stick back")
    {
        FAttackRig rig;
        const std::uint32_t period = rig.commitmentTicks();
        constexpr std::size_t kShots = 5u;
        rig.holdStickOffBack(0.f);
        rig.tick(true, true, inputSequence::kHadoukenActionId);
        while (rig.nextTick <= 1u + (kShots - 1u) * period + period / 2u)
            rig.tick(true, true);

        INFO(rig.trace());
        CHECK(rig.realSequenceTicks().empty());
        CHECK(rig.firedTicks() == everyPeriod(1u, period, kShots));
        CHECK(rig.spawnedTicks() == everyPeriod(1u, period, kShots));
        CHECK(rig.logLinesContaining("[Projectile.poolFull]") == 0u);
    }
}

TEST_CASE("DAttack.AttackTransitions.FollowUpPressIsTheClassifiedSideWithoutProjectileIntent", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;
    using dAttackMachineSimulation::PlayerInputView;
    using dAttackMachineSimulation::followUpPressSequence;
    using dAttackMachineSimulation::followUpSequenceFor;

    CHECK(dAttackMachineSimulation::kRightFollowUpSequenceId == kAuthoredRightFollowUp);
    CHECK(dAttackMachineSimulation::kLeftFollowUpSequenceId == kAuthoredLeftFollowUp);
    CHECK(followUpSequenceFor(kHadoukenSequenceSentinel, dAttackDirection::kLeftSequenceId) == InvalidAttackSequenceId);
    CHECK(followUpSequenceFor(kHadoukenSequenceSentinel, dAttackDirection::kRightSequenceId) == InvalidAttackSequenceId);

    const glm::vec3 aim = glm::normalize(glm::vec3(0.6f, -0.8f, -0.3f));
    auto view = [&](float aimRelativeAngle, bool attackLeft, bool attackRight, std::uint32_t action = 0u) {
        const glm::vec3 stick = stickAt(aim, aimRelativeAngle);
        return PlayerInputView{ .aimDirection = aim, .attackLeft = attackLeft, .attackRight = attackRight,
                                .moveDirection = glm::vec2(stick.x, stick.y), .moveDirectionWorld = stick,
                                .triggeredActionId = action };
    };

    std::size_t projectileRows = 0u;
    std::size_t sideRows = 0u;
    for (int d = 0; d < 360; ++d)
    {
        const float a = degrees(static_cast<float>(d));
        const PlayerInputView neither = view(a, false, false);
        const PlayerInputView attack1 = view(a, true, false);
        const PlayerInputView attack2 = view(a, false, true);
        const unsigned int side = dAttackDirection::classify(attack1.aimDirection, attack1.moveDirectionWorld,
                                                             attack1.moveDirection);
        INFO("aim-relative angle " << d << " deg, classifier " << side);

        CHECK(followUpPressSequence(neither) == InvalidAttackSequenceId);
        CHECK(followUpPressSequence(attack1) == side);
        CHECK(followUpPressSequence(view(a, true, false, inputSequence::kHadoukenActionId)) == InvalidAttackSequenceId);

        if (dAttackMachineSimulation::wantsHeldProjectile(attack2))
        {
            ++projectileRows;
            sideRows += side != dAttackDirection::kForwardSequenceId ? 1u : 0u;
            CHECK(followUpPressSequence(attack2) == InvalidAttackSequenceId);
            CHECK(followUpPressSequence(view(a, true, true)) == InvalidAttackSequenceId);
        }
        else
        {
            CHECK(followUpPressSequence(attack2) == side);
            CHECK(followUpPressSequence(view(a, true, true)) == side);
        }
    }
    CHECK(projectileRows > 0u);
    CHECK(sideRows > 0u);
}

TEST_CASE("DAttack.AttackTransitions.DualPressOnAGuardBlockTickQueuesNothing", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;

    for (const bool insideRestartWindow : { true, false })
    {
        FAttackRig rig;
        rig.holdStick(kRightOfAim);
        const TickRecord start = rig.tick(true, false);
        REQUIRE(start.activeSequence == dAttackDirection::kRightSequenceId);

        const std::uint32_t blockTick = insideRestartWindow ? 3u : 25u;
        while (rig.nextTick < blockTick)
            rig.tick(false, false);
        if (insideRestartWindow)
            REQUIRE(rig.radial().attackTimer < 0.1f);
        else
            REQUIRE(rig.radial().attackTimer > 0.3f);

        rig.character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick = true;
        const TickRecord blocked = rig.tick(true, true);

        INFO((insideRestartWindow ? "block inside the 0.1 s restart window" : "block past 0.3 s") << "\n" << rig.trace());
        CHECK(blocked.state == DAttackState::GuardFlinch);
        CHECK(blocked.activeSequence == InvalidAttackSequenceId);
        CHECK(blocked.queuedSequence == InvalidAttackSequenceId);

        rig.run(2u * rig.commitmentTicks(), false, false);
        REQUIRE(rig.history.back().state == DAttackState::Idle);
        CHECK(rig.history.back().queuedSequence == InvalidAttackSequenceId);

        rig.history.clear();
        rig.run(kSwingRunTicks, true, false);
        const std::vector<unsigned int> swings = rig.swingSequences();
        INFO("swings after the recoil " << Catch::StringMaker<std::vector<unsigned int>>::convert(swings));
        REQUIRE(swings.size() >= 2u);
        CHECK(swings[0] == dAttackDirection::kRightSequenceId);
        CHECK(swings[1] == kAuthoredRightFollowUp);
    }
}

TEST_CASE("DAttack.AttackTransitions.ProjectileBlockDoesNotFlinchShooter", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;
    constexpr std::uint32_t kFlightTicksBeforeTheBlock = 13u;

    SECTION("mid-commitment")
    {
        FAttackRig rig;
        const std::uint32_t period = rig.commitmentTicks();
        REQUIRE(period > kFlightTicksBeforeTheBlock + 1u);
        rig.holdStickOffBack(0.f);
        rig.tick(false, true, inputSequence::kHadoukenActionId);
        rig.releaseStick();
        while (rig.nextTick < 1u + kFlightTicksBeforeTheBlock)
            rig.tick(false, false);

        rig.blockLiveShotOnNextTick = true;
        const TickRecord blocked = rig.tick(false, false);
        INFO(rig.trace());
        CHECK(rig.blockedSlot().endReason == 4u);
        CHECK(rig.blockedSlot().endTick == blocked.tick);
        CHECK(blocked.state == DAttackState::Attacking);
        CHECK(blocked.activeSequence == kHadoukenSequenceSentinel);
        CHECK(blocked.attackEndTick == 1u + period);

        while (rig.nextTick <= 1u + period + 2u)
            rig.tick(false, false);
        CHECK_FALSE(rig.everIn(DAttackState::GuardFlinch));
        CHECK(rig.history[period - 1u].state == DAttackState::Attacking);
        CHECK(rig.history[period].state == DAttackState::Idle);
        CHECK(rig.logLinesContaining("(projectile blocked)") == 0u);
    }

    SECTION("in rapid fire")
    {
        FAttackRig rig;
        const std::uint32_t period = rig.commitmentTicks();
        constexpr std::size_t kShots = 5u;
        rig.holdStickOffBack(0.f);
        rig.tick(false, true, inputSequence::kHadoukenActionId);
        const std::uint32_t blockTick = 1u + period + kFlightTicksBeforeTheBlock;
        while (rig.nextTick <= 1u + (kShots - 1u) * period + period / 2u)
        {
            rig.blockLiveShotOnNextTick = rig.nextTick == blockTick;
            rig.tick(false, true);
        }

        INFO(rig.trace());
        CHECK(rig.blockedSlot().spawnTick == 1u + period);
        CHECK(rig.blockedSlot().endReason == 4u);
        CHECK(rig.blockedSlot().endTick == blockTick);
        CHECK_FALSE(rig.everIn(DAttackState::GuardFlinch));
        CHECK(rig.realSequenceTicks().empty());
        CHECK(rig.firedTicks() == everyPeriod(1u, period, kShots));
        CHECK(rig.spawnedTicks() == everyPeriod(1u, period, kShots));
    }

    SECTION("Idle, after a hit ended the commitment early")
    {
        FAttackRig rig;
        rig.holdStickOffBack(0.f);
        rig.tick(false, true, inputSequence::kHadoukenActionId);
        rig.releaseStick();
        rig.tick(false, false);

        brawlerInboundHit::DerivedState& slice =
            rig.character.editAllState().editDerivedState().edit<brawlerInboundHit::DerivedState>();
        slice.wasHitThisTick = true;
        slice.reactionKind   = HitReactionKind::Stun;
        slice.flinchDuration = 0.05f;
        REQUIRE(rig.tick(false, false).state == DAttackState::HitFlinch);
        while (rig.nextTick < 1u + kFlightTicksBeforeTheBlock)
            rig.tick(false, false);
        REQUIRE(rig.history.back().state == DAttackState::Idle);

        rig.blockLiveShotOnNextTick = true;
        const TickRecord blocked = rig.tick(false, false);
        const TickRecord after = rig.tick(false, false);
        INFO(rig.trace());
        CHECK(rig.blockedSlot().spawnTick == 1u);
        CHECK(rig.blockedSlot().endReason == 4u);
        CHECK(blocked.state == DAttackState::Idle);
        CHECK(after.state == DAttackState::Idle);
        CHECK_FALSE(rig.everIn(DAttackState::GuardFlinch));
    }
}

TEST_CASE("DAttack.AttackTransitions.CommitmentIs45Ticks", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;
    constexpr std::uint32_t kCommitmentTicks = 45u;

    for (const float dt : { 1.f / 60.f, 0.016667f })
    {
        INFO("dt=" << dt);
        {
            FAttackRig rig;
            rig.dt = dt;
            CHECK(rig.commitmentTicks() == kCommitmentTicks);
            rig.holdStickOffBack(0.f);
            const TickRecord shot = rig.tick(false, true, inputSequence::kHadoukenActionId);
            rig.releaseStick();
            while (rig.nextTick <= 1u + kCommitmentTicks + 2u)
                rig.tick(false, false);

            INFO(rig.trace());
            CHECK(shot.attackEndTick == shot.tick + kCommitmentTicks);
            std::uint32_t attackingTicks = 0u;
            for (const TickRecord& r : rig.history)
                attackingTicks += r.state == DAttackState::Attacking ? 1u : 0u;
            CHECK(attackingTicks == kCommitmentTicks);
            CHECK(rig.history[kCommitmentTicks].tick == shot.tick + kCommitmentTicks);
            CHECK(rig.history[kCommitmentTicks].state == DAttackState::Idle);
        }
        {
            FAttackRig rig;
            rig.dt = dt;
            rig.holdStickOffBack(0.f);
            rig.tick(false, true, inputSequence::kHadoukenActionId);
            while (rig.nextTick <= 1u + 3u * kCommitmentTicks)
                rig.tick(false, true);
            INFO(rig.trace());
            CHECK(rig.firedTicks() == everyPeriod(1u, kCommitmentTicks, 4u));
        }
    }
}

TEST_CASE("DAttack.AttackTransitions.MeleeGuardBlockRecoilIsUnchanged", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;
    constexpr std::uint32_t kMeleeGuardBlockRecoilTicks = 18u;

    for (const float dt : { 1.f / 60.f, 0.016667f })
    {
        FAttackRig rig;
        rig.dt = dt;
        rig.holdStick(kRightOfAim);
        REQUIRE(rig.tick(true, false).activeSequence == dAttackDirection::kRightSequenceId);
        rig.releaseStick();
        while (rig.nextTick < 10u)
            rig.tick(false, false);

        rig.character.editAllState().editDerivedState()
            .edit<brawlerInboundHit::DerivedState>().wasGuardBlockedThisTick = true;
        const TickRecord blocked = rig.tick(false, false);
        REQUIRE(blocked.state == DAttackState::GuardFlinch);
        while (rig.nextTick <= blocked.tick + 2u * kMeleeGuardBlockRecoilTicks
               && rig.history.back().state == DAttackState::GuardFlinch)
            rig.tick(false, false);

        INFO("dt=" << dt << "\n" << rig.trace());
        CHECK(rig.history.back().state == DAttackState::Idle);
        CHECK(rig.history.back().tick - blocked.tick == kMeleeGuardBlockRecoilTicks);
    }
}

TEST_CASE("DAttack.AttackTransitions.CommitmentLeavesThePunishWindow", "[DAttack][AttackTransitions]")
{
    using namespace attackTransitionTests;
    constexpr std::uint32_t kAssumedInputDelayTicks   = 6u;
    constexpr std::uint32_t kPunishReactionTicks      = 10u;
    constexpr std::uint32_t kMaxPunishableFlightTicks = 5u;

    const simulatableBrawler::StaticData staticData;
    auto ticksToTheAimSegment = [&staticData](unsigned int sequenceId, float dt) {
        const DAttackRadialSequence& sequence = staticData.m_attackSequences[sequenceId];
        const DAttackSegment aimSegment = sequence.getAttackSegment(0.f);
        INFO("sequence " << sequenceId << " aim segment " << aimSegment.index);
        REQUIRE(aimSegment.index + 1u < sequence.getAttackPointCount());
        REQUIRE(aimSegment.state == DAttackRadialSequenceState::Damaging);
        return dAttackMachineSimulation::swingTickCount(sequence.getAttackPointTime(aimSegment.index), dt);
    };

    for (const float dt : { 1.f / 60.f, 0.016667f })
    {
        const std::uint32_t rightK   = ticksToTheAimSegment(dAttackDirection::kRightSequenceId, dt);
        const std::uint32_t leftK    = ticksToTheAimSegment(dAttackDirection::kLeftSequenceId, dt);
        const std::uint32_t forwardK = ticksToTheAimSegment(dAttackDirection::kForwardSequenceId, dt);
        const std::uint32_t slowestK = std::max({ rightK, leftK, forwardK });
        const std::uint32_t required =
            slowestK + kAssumedInputDelayTicks + kPunishReactionTicks + kMaxPunishableFlightTicks;
        const std::uint32_t commitment = dAttackMachineSimulation::swingTickCount(kHadoukenCommitmentSeconds, dt);
        INFO("dt=" << dt << " side k=" << rightK << "/" << leftK << " forward k=" << forwardK
             << " required=" << required << " commitment=" << commitment);
        CHECK(commitment >= required);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
