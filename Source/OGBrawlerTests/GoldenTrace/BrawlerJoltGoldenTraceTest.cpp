// SPDX-License-Identifier: BUSL-1.1
// docs/BrawlerJoltGoldenTraceTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "GoldenTraceFormat.h"
#include "GoldenTraceScenario.h"
#include "GoldenTraceScene.h"

#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/DAttackDirectionClassifier.h"

#include <cmath>
#include <cstdio>
#include <iterator>
#include <optional>
#include <span>
#include <string>

#if __has_include("BrawlerJoltGolden.inl") && __has_include("BrawlerJoltGoldenEnvelope.inl")
#include "BrawlerJoltGolden.inl"
#include "BrawlerJoltGoldenEnvelope.inl"

namespace
{
inline constexpr bool kGoldenRecorded = true;

goldenTrace::GoldenView recordedGolden()
{
    static_assert(goldenTrace::everyFieldHasABound(goldenTraceData::kGoldenFields),
        "BrawlerJoltGolden.inl: every recorded field has a bound class (a 'p' field's last name segment is position, linearVelocity "
        "or velocity, and no field is 'd')");
    return goldenTrace::GoldenView{ &goldenTraceData::kGoldenHeader, goldenTraceData::kGoldenFields,
        std::size(goldenTraceData::kGoldenFields), goldenTraceData::kGoldenTicks, std::size(goldenTraceData::kGoldenTicks),
        goldenTraceData::kGoldenValues, std::size(goldenTraceData::kGoldenValues) };
}

goldenTrace::EnvelopeView recordedEnvelope()
{
    static_assert(sizeof(goldenTraceEnvelopeData::kEnvelopeTagOrder) - 1u == goldenTrace::kTagCount,
        "BrawlerJoltGoldenEnvelope.inl stores one envelope column per golden tag");
    return goldenTrace::EnvelopeView{ &goldenTraceEnvelopeData::kEnvelopeHeader, goldenTraceEnvelopeData::kEnvelopeNudges,
        std::size(goldenTraceEnvelopeData::kEnvelopeNudges), goldenTraceEnvelopeData::kEnvelopeTicks,
        std::size(goldenTraceEnvelopeData::kEnvelopeTicks), goldenTraceEnvelopeData::kEnvelopeValues,
        std::size(goldenTraceEnvelopeData::kEnvelopeValues) };
}
} // namespace
#else
namespace
{
inline constexpr bool kGoldenRecorded = false;
goldenTrace::GoldenView recordedGolden() { return {}; }
goldenTrace::EnvelopeView recordedEnvelope() { return {}; }
} // namespace
#endif

namespace
{

using namespace goldenTraceScenario;

static_assert(goldenTrace::kLinearVelocityBoundCmPerS >= 2.0 * -goldenTrace::toLegacyCm(kGravity) * double(kDtSeconds),
    "goldenTrace::kLinearVelocityBoundCmPerS admits a one-tick velocity spike of 2*g*dt");

struct FlatScenario
{
    static constexpr const char* kGenerator = "BrawlerJoltGoldenTrace.Record";
    static constexpr float kDtSeconds = goldenTraceScenario::kDtSeconds;
    static constexpr uint32_t kScenarioTicks = goldenTraceScenario::kScenarioTicks;

    static bool isSampleTick(uint32_t tick) { return goldenTraceScenario::isSampleTick(tick); }
    static StaticWorldDescription statics() { return arenaStatics(); }
    static simulatableBrawler::StaticData staticData(float gravityScale) { return scenarioStaticData(gravityScale); }
    static simulatableBrawler::PlayerInput inputAt(unsigned int characterId, uint32_t tick)
    {
        return goldenTraceScenario::inputAt(characterId, tick);
    }

    static constexpr goldenTraceScene::Character kCharacters[] = { { kCharacterA, kSlotA, "A.", kSpawnA },
        { kCharacterB, kSlotB, "B.", kSpawnB } };
    static constexpr goldenTraceScene::SpawnPoint kSpawnTable[] = { { kSlotA, kSpawnA }, { kSlotB, kRespawnB } };

    static std::span<const goldenTraceScene::Character> characters() { return kCharacters; }
    static std::span<const goldenTraceScene::SpawnPoint> spawnTable() { return kSpawnTable; }
    static std::span<const goldenTraceScene::Teleport> teleports() { return {}; }
};

static_assert(goldenTraceScene::Scenario<FlatScenario>, "the flat scenario provides everything goldenTraceScene::Scene reads");

struct TraceReader
{
    const goldenTrace::Trace& trace;

    double at(std::size_t sample, const std::string& name, uint32_t component = 0u) const
    {
        const std::optional<std::size_t> field = trace.fieldIndex(name);
        OG_CHECK(field.has_value(), "golden harness: the coverage count names a field the trace does not record");
        return trace.sample(sample)[trace.fields[*field].offset + component];
    }
};

struct Coverage
{
    uint32_t walkTicks = 0;
    uint32_t rampTicks = 0;
    uint32_t plateauTicks = 0;
    uint32_t stepDescentTicks = 0;
    uint32_t rightSwings = 0;
    uint32_t leftSwings = 0;
    uint32_t overheadSwings = 0;
    uint32_t hadoukens = 0;
    uint32_t guardBlocksOfRightSwing = 0;
    uint32_t guardBlocksOfLeftSwing = 0;
    uint32_t projectilesBlocked = 0;
    uint32_t projectilesHit = 0;
    uint32_t detectedMeleeHits = 0;
    uint32_t detectedGuardBlocks = 0;
    uint32_t detectedProjectileHits = 0;
    uint32_t detectedProjectileBlocks = 0;
    uint32_t meleeHitFlinches = 0;
    uint32_t knockbackTicks = 0;
    uint32_t ledgeFallTicks = 0;
    uint32_t deaths = 0;
    uint32_t respawns = 0;
};

inline constexpr uint32_t kAttacking = static_cast<uint32_t>(DAttackState::Attacking);
inline constexpr uint32_t kGuardFlinch = static_cast<uint32_t>(DAttackState::GuardFlinch);
inline constexpr uint32_t kHitFlinch = static_cast<uint32_t>(DAttackState::HitFlinch);
inline constexpr uint32_t kSupported = static_cast<uint32_t>(brawlerMovementSimulation::SupportState::Supported);
inline constexpr double kWalkSpeedFloorCmPerS = 50.0;
inline constexpr double kKnockbackSpeedFloorCmPerS = 1000.0;
inline constexpr double kHeightSlackCm = 1.0;

uint32_t supportOf(double flags)
{
    return (static_cast<uint32_t>(flags) & brawlerMovementSimulation::kFlagSupportMask) >> brawlerMovementSimulation::kFlagSupportShift;
}

Coverage coverageOf(const goldenTrace::Trace& trace)
{
    const TraceReader read{ trace };
    Coverage coverage;
    const std::string capsuleA = "A.body5.CharacterCapsule.position";
    const std::string capsuleB = "B.body5.CharacterCapsule.position";
    std::optional<double> restHeightA;
    for (std::size_t sample = 0; sample < trace.ticks.size(); ++sample)
    {
        if (trace.ticks[sample] == kWalkFirstTick - 1u)
            restHeightA = read.at(sample, capsuleA, 2u);
    }
    OG_CHECK(restHeightA.has_value(), "golden harness: the coverage count needs A's resting height before the walk");
    const double plateauHeightA = *restHeightA + goldenTrace::toLegacyCm(kStepHeight);
    const double rampFoot = goldenTrace::toLegacyCm(kRampFoot);
    const double rampTop = goldenTrace::toLegacyCm(kRampTop);
    const double stepEdge = goldenTrace::toLegacyCm(kStepEdge);
    const double floorStart = goldenTrace::toLegacyCm(kFloorStart);

    for (std::size_t sample = 1; sample < trace.ticks.size(); ++sample)
    {
        const double ax = read.at(sample, capsuleA, 0u);
        const double az = read.at(sample, capsuleA, 2u);
        const double avx = read.at(sample, "A.movement.velocity", 0u);
        const double avy = read.at(sample, "A.movement.velocity", 1u);
        const bool aSupported = supportOf(read.at(sample, "A.movement.flags")) == kSupported;
        const uint32_t aState = static_cast<uint32_t>(read.at(sample, "A.machine.currentState"));
        const uint32_t aPrevious = static_cast<uint32_t>(read.at(sample - 1u, "A.machine.currentState"));
        const uint32_t aSequence = static_cast<uint32_t>(read.at(sample, "A.machine.activeAttackSequence"));
        const uint32_t aPreviousSequence = static_cast<uint32_t>(read.at(sample - 1u, "A.machine.activeAttackSequence"));

        if (aSupported && aState != kAttacking && std::hypot(avx, avy) > kWalkSpeedFloorCmPerS)
            ++coverage.walkTicks;
        if (aSupported && ax > rampFoot && ax < rampTop && az > *restHeightA + kHeightSlackCm
            && az < plateauHeightA - kHeightSlackCm)
            ++coverage.rampTicks;
        if (aSupported && ax > rampTop && ax < stepEdge && std::fabs(az - plateauHeightA) < kHeightSlackCm)
            ++coverage.plateauTicks;
        if (ax > stepEdge && az > *restHeightA + kHeightSlackCm && az < plateauHeightA - kHeightSlackCm)
            ++coverage.stepDescentTicks;

        if (aState == kAttacking && aPrevious != kAttacking)
        {
            coverage.rightSwings += aSequence == dAttackDirection::kRightSequenceId ? 1u : 0u;
            coverage.leftSwings += aSequence == dAttackDirection::kLeftSequenceId ? 1u : 0u;
            coverage.overheadSwings += aSequence == dAttackDirection::kForwardSequenceId ? 1u : 0u;
        }
        if (aState == kGuardFlinch && aPrevious != kGuardFlinch)
        {
            coverage.guardBlocksOfRightSwing += aPreviousSequence == dAttackDirection::kRightSequenceId ? 1u : 0u;
            coverage.guardBlocksOfLeftSwing += aPreviousSequence == dAttackDirection::kLeftSequenceId ? 1u : 0u;
        }

        for (const char* who : { "A.", "B." })
        {
            const std::string prefix = who;
            const uint32_t state = static_cast<uint32_t>(read.at(sample, prefix + "machine.currentState"));
            const uint32_t previous = static_cast<uint32_t>(read.at(sample - 1u, prefix + "machine.currentState"));
            const uint32_t sequence = static_cast<uint32_t>(read.at(sample, prefix + "machine.activeAttackSequence"));
            if (state == kAttacking && sequence == kHadoukenSequenceSentinel
                && !(previous == kAttacking
                    && static_cast<uint32_t>(read.at(sample - 1u, prefix + "machine.activeAttackSequence")) == kHadoukenSequenceSentinel))
                ++coverage.hadoukens;
            if (state == kHitFlinch && previous != kHitFlinch)
                ++coverage.meleeHitFlinches;
            for (uint32_t slot = 0; slot < brawlerProjectileSimulation::kMaxProjectilePoolSize; ++slot)
            {
                const std::string field = prefix + "projectile.slot" + std::to_string(slot) + ".endReason";
                const uint32_t reason = static_cast<uint32_t>(read.at(sample, field));
                const uint32_t previousReason = static_cast<uint32_t>(read.at(sample - 1u, field));
                if (reason != previousReason)
                {
                    coverage.projectilesBlocked += reason == static_cast<uint32_t>(brawlerProjectileSimulation::SlotOutcome::BlockedByGuard) ? 1u : 0u;
                    coverage.projectilesHit += reason == static_cast<uint32_t>(brawlerProjectileSimulation::SlotOutcome::Hit) ? 1u : 0u;
                }
            }
        }

        const double bx = read.at(sample, capsuleB, 0u);
        const double bvz = read.at(sample, "B.body5.CharacterCapsule.linearVelocity", 2u);
        const double bvx = read.at(sample, "B.movement.velocity", 0u);
        const double bvy = read.at(sample, "B.movement.velocity", 1u);
        if (std::hypot(bvx, bvy) > kKnockbackSpeedFloorCmPerS)
            ++coverage.knockbackTicks;
        if (bx < floorStart && bvz < 0.0)
            ++coverage.ledgeFallTicks;
        const bool dead = (static_cast<uint32_t>(read.at(sample, "B.ringout.flags")) & brawlerRingout::kFlagDead) != 0u;
        const bool wasDead = (static_cast<uint32_t>(read.at(sample - 1u, "B.ringout.flags")) & brawlerRingout::kFlagDead) != 0u;
        coverage.deaths += dead && !wasDead ? 1u : 0u;
        coverage.respawns += !dead && wasDead ? 1u : 0u;
    }
    const std::size_t last = trace.ticks.size() - 1u;
    for (const char* who : { "A.", "B." })
    {
        const std::string prefix = who;
        coverage.detectedMeleeHits += static_cast<uint32_t>(read.at(last, prefix + "hit.meleeHits"));
        coverage.detectedGuardBlocks += static_cast<uint32_t>(read.at(last, prefix + "hit.guardBlocks"));
        coverage.detectedProjectileHits += static_cast<uint32_t>(read.at(last, prefix + "hit.projectileHits"));
        coverage.detectedProjectileBlocks += static_cast<uint32_t>(read.at(last, prefix + "hit.projectileBlocks"));
    }
    return coverage;
}

void printCoverage(const char* label, const Coverage& coverage)
{
    std::printf("[GoldenTrace.coverage] %s walkTicks=%u rampTicks=%u plateauTicks=%u stepDescentTicks=%u rightSwings=%u leftSwings=%u "
                "overheadSwings=%u hadoukens=%u guardBlocksOfRightSwing=%u guardBlocksOfLeftSwing=%u projectilesBlocked=%u "
                "projectilesHit=%u hitFlinches=%u knockbackTicks=%u ledgeFallTicks=%u deaths=%u respawns=%u detectedMeleeHits=%u "
                "detectedGuardBlocks=%u detectedProjectileHits=%u detectedProjectileBlocks=%u\n",
        label, coverage.walkTicks, coverage.rampTicks, coverage.plateauTicks, coverage.stepDescentTicks, coverage.rightSwings,
        coverage.leftSwings, coverage.overheadSwings, coverage.hadoukens, coverage.guardBlocksOfRightSwing,
        coverage.guardBlocksOfLeftSwing, coverage.projectilesBlocked, coverage.projectilesHit, coverage.meleeHitFlinches,
        coverage.knockbackTicks, coverage.ledgeFallTicks, coverage.deaths, coverage.respawns, coverage.detectedMeleeHits,
        coverage.detectedGuardBlocks, coverage.detectedProjectileHits, coverage.detectedProjectileBlocks);
}

} // namespace

TEST_CASE("BrawlerJoltGoldenTrace.ScriptCoversEveryTickOnce", "[GoldenTrace]")
{
    uint32_t coveredTicks = 0;
    for (const unsigned int id : { kCharacterA, kCharacterB })
        for (uint32_t tick = 1u; tick <= kScenarioTicks; ++tick)
            coveredTicks += scriptRowAt(id, tick).has_value() ? 1u : 0u;
    REQUIRE(coveredTicks == 2u * kScenarioTicks);
}

TEST_CASE("BrawlerJoltGoldenTrace.MatchesTheRecordedTrace", "[GoldenTrace]")
{
    goldenTraceScene::requireMatchesTheRecordedTrace<FlatScenario>(kGoldenRecorded, recordedGolden(), recordedEnvelope());
}

TEST_CASE("BrawlerJoltGoldenTrace.ScenarioCoversEveryScriptedMechanic", "[GoldenTrace]")
{
    const Coverage coverage = coverageOf(goldenTraceScene::runScenario<FlatScenario>());
    printCoverage("live", coverage);
    CHECK(coverage.walkTicks > 0u);
    CHECK(coverage.rampTicks > 0u);
    CHECK(coverage.plateauTicks > 0u);
    CHECK(coverage.stepDescentTicks > 0u);
    CHECK(coverage.rightSwings >= 2u);
    CHECK(coverage.leftSwings >= 1u);
    CHECK(coverage.overheadSwings >= 2u);
    CHECK(coverage.hadoukens >= 2u);
    CHECK(coverage.guardBlocksOfRightSwing >= 1u);
    CHECK(coverage.guardBlocksOfLeftSwing >= 1u);
    CHECK(coverage.projectilesBlocked >= 1u);
    CHECK(coverage.projectilesHit >= 1u);
    CHECK(coverage.meleeHitFlinches >= 2u);
    CHECK(coverage.knockbackTicks > 0u);
    CHECK(coverage.ledgeFallTicks > 0u);
    CHECK(coverage.deaths >= 1u);
    CHECK(coverage.respawns >= 1u);
    CHECK(coverage.detectedMeleeHits >= 2u);
    CHECK(coverage.detectedGuardBlocks >= 2u);
    CHECK(coverage.detectedProjectileHits >= 1u);
    CHECK(coverage.detectedProjectileBlocks >= 1u);
}

TEST_CASE("BrawlerJoltGoldenTrace.Record", "[.GoldenRecord]")
{
    const goldenTrace::Trace base
        = goldenTraceScene::recordGoldenAndEnvelope<FlatScenario>("BrawlerJoltGolden.inl", "BrawlerJoltGoldenEnvelope.inl");
    printCoverage("recorded", coverageOf(base));
}

#endif
