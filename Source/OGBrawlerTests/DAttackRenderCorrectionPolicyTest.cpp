// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "BrawlerSimulationMocks.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackGuardSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/PhysicsDeclaration.h"
#include "OGSimulation/RenderCorrectionSmoothing.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/SimulationFrame.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <type_traits>
#include <utility>
#include <vector>

namespace dAttackRenderCorrectionPolicyTest
{
    constexpr size_t kBodies = 8;
    using Snapshot = RenderSnapshotT<kBodies>;
    using Correction = RenderCorrectionT<kBodies>;
    using Capture = RenderCorrectionCapture<kBodies>;
    using Ring = SpscRing<Correction, 8>;
    using Smoother = RenderCorrectionSmoother<kBodies>;
    using brawlerTestMocks::MockPhysicsAdapter;
    using brawlerTestMocks::MockSpatialQueryAdapter;

    using MovementDeclaration = brawlerMovementSimulation::PhysicsDeclaration;
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(MovementDeclaration::bodyStateOf(
                      std::declval<const MovementDeclaration::StateType&>()))>,
                      LinearBodyState>,
        "The brawler's root declaration (the movement capsule) must carry a LinearBodyState: fillRenderSnapshot then writes "
        "RenderBody::hasRotation 0 for it, and RenderCorrectionSmoother's root-frame pass OG_CHECKs exactly that before it "
        "draws the weapon and the guard in the capsule's frame. A rotating root is not composed.");

    constexpr float kDt = 1.f / 60.f;
    constexpr unsigned int kId = 7u;
    constexpr uint8_t kRadial = 0u;
    constexpr uint8_t kGuard = 1u;
    constexpr uint8_t kCapsule = 5u;
    constexpr uint64_t kCarrierStep = 100u;
    constexpr SimTick kShownTick = 50u;
    constexpr int kFrames = 20;
    constexpr int kCorrectedSwingStep = 8;
    constexpr double kAlpha = 0.5;
    constexpr float kWalkSpeed = 1.f;
    constexpr glm::vec3 kAttachment = glm::vec3(0.f, 0.3f, -0.f);
    constexpr glm::vec3 kCapsuleStart = glm::vec3(2.f, 0.9f, -5.f);
    constexpr glm::vec3 kAimForward = glm::vec3(0.f, 0.f, -1.f);
    constexpr glm::vec3 kAimLeft = glm::vec3(1.f, 0.f, -0.f);
    constexpr uint8_t kSnapInRootFrame = static_cast<uint8_t>(RenderCorrectionMode::SnapInRootFrame);

    struct Pose
    {
        glm::vec3 position{ 0.f };
        glm::quat rotation{ 1.f, 0.f, 0.f, 0.f };
    };

    struct BrawlerPoses
    {
        glm::vec3 capsule{ 0.f };
        glm::vec3 capsuleVelocity{ 0.f };
        Pose radial;
        Pose guard;
    };

    struct Frame
    {
        Snapshot supplied;
        Snapshot rendered;
        RenderCorrectionFrame summary;
    };

    glm::quat rotationOf(const glm::mat4& transform)
    {
        return glm::quat_cast(glm::mat3(transform));
    }

    glm::quat radialRotationFromIntegrate(const dAttackRadialSimulation::StaticData& staticData, unsigned int icSequence,
        unsigned int stateSequence)
    {
        using namespace dAttackRadialSimulation;
        InitialConditions ic{};
        ic.initialAimAngle = 0.f;
        ic.initialAimRotationAxis = glm::vec3(-0.f, -1.f, 0.f);
        ic.activeAttackSequence = icSequence;
        State state{};
        state.currenSequenceId = stateSequence;
        SimulationComposite<InitialConditions, State> composite(ic, state);
        auto deps = makeDependencies<Dependencies>(composite);

        MockPhysicsAdapter physics{ 2u };
        PlayerInputView view{};
        IntegrationUtils<MockPhysicsAdapter> utils{ kDt, physics };
        AllInput<MockPhysicsAdapter> input{ view, utils };
        RuntimeBindings bindings{};
        bindings.ownBodyId = BodyId{ 0u };
        bindings.parentBodyId = BodyId{ 1u };
        DerivedState derived{};
        integrate(kDt, input, staticData, deps, bindings, derived);

        REQUIRE_FALSE(physics.setTransformCalls.empty());
        REQUIRE(physics.setTransformCalls.back().bodyId == BodyId{ 0u });
        return rotationOf(physics.setTransformCalls.back().transform);
    }

    glm::quat guardRotationFromIntegrate(const dAttackGuardSimulation::StaticData& staticData, const glm::vec3& aim)
    {
        using namespace dAttackGuardSimulation;
        dAttackMachineSimulation::State machine{};
        SimulationComposite<InitialConditions, State, dAttackMachineSimulation::State> composite(InitialConditions{}, State{},
            machine);
        auto deps = makeDependencies<Dependencies>(composite);

        MockPhysicsAdapter physics{ 2u };
        MockSpatialQueryAdapter query{};
        PlayerInputView view{};
        view.aimDirection = aim;
        IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{ kDt, physics, query };
        AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> input{ view, utils };
        RuntimeBindings bindings{};
        bindings.ownBodyId = BodyId{ 0u };
        bindings.parentBodyId = BodyId{ 1u };
        DerivedState derived{};
        integrate(kDt, input, staticData, deps, bindings, derived);
        return composite.get<State>().bodyState.rotation;
    }

    double bladeElevationDegrees(const glm::quat& rotation)
    {
        const glm::vec3 blade = rotation * glm::vec3(0.f, 0.f, -1.f);
        const double z = std::clamp(static_cast<double>(simFrame::height(blade)) / static_cast<double>(glm::length(blade)), -1.0, 1.0);
        return std::asin(z) * 180.0 / std::numbers::pi;
    }

    double angleBetweenDegrees(const glm::quat& first, const glm::quat& second)
    {
        const glm::dquat difference = glm::conjugate(glm::dquat(first)) * glm::dquat(second);
        return renderCorrectionDetail::rotationDegrees(renderCorrectionDetail::normalized(difference));
    }

    const RenderBody& bodyOf(const Snapshot& snapshot, uint8_t declarationIndex)
    {
        const RenderBody* const body = findRenderBody(snapshot, kId, declarationIndex);
        REQUIRE(body != nullptr);
        return *body;
    }

    glm::dvec3 offsetFromCapsule(const Snapshot& snapshot, uint8_t declarationIndex)
    {
        return glm::dvec3(bodyOf(snapshot, declarationIndex).position) - glm::dvec3(bodyOf(snapshot, kCapsule).position);
    }

    struct Rig
    {
        simulatableBrawler::StaticData staticData;
        SimulationObjectStorage<SimulatableBrawler> storage;
        Capture capture;
        Ring ring;
        Smoother smoother;

        Rig()
        {
            storage.add<SimulatableBrawler>(kId, SimulatableBrawler(staticData));
        }

        const DAttackRadialSequence& swingSequence() const
        {
            return staticData.m_attackSimulationStaticData.getAttackSequences()[0];
        }

        glm::quat idleRadialRotation() const
        {
            return radialRotationFromIntegrate(staticData.m_attackSimulationStaticData, InvalidAttackSequenceId, 0u);
        }

        glm::quat swingRadialRotation(int swingStep) const
        {
            const glm::quat start = radialRotationFromIntegrate(staticData.m_attackSimulationStaticData, 0u,
                InvalidAttackSequenceId);
            const float angle = swingSequence().getAngle(static_cast<float>(swingStep) * kDt) - swingSequence().getInitialAngle();
            return glm::angleAxis(angle, glm::vec3(-0.f, -1.f, 0.f)) * start;
        }

        glm::quat guardRotation(const glm::vec3& aim) const
        {
            return guardRotationFromIntegrate(staticData.m_guardSimulationStaticData, aim);
        }

        void write(const BrawlerPoses& poses)
        {
            auto& state = storage.get<SimulatableBrawler>(kId).editAllState().editState();
            PhysicsBodyState& radial = state.edit<dAttackRadialSimulation::State>().bodyState;
            radial.position = poses.radial.position;
            radial.rotation = poses.radial.rotation;
            PhysicsBodyState& guard = state.edit<dAttackGuardSimulation::State>().bodyState;
            guard.position = poses.guard.position;
            guard.rotation = poses.guard.rotation;
            LinearBodyState& capsule = state.edit<brawlerMovementSimulation::State>().bodyState;
            capsule.position = poses.capsule;
            capsule.linearVelocity = poses.capsuleVelocity;
        }

        Snapshot snapshotOf(const BrawlerPoses& poses, uint64_t physicsStep, SimTick tick)
        {
            write(poses);
            TickOutcome outcome;
            outcome.tick = tick;
            outcome.physicsStep = physicsStep;
            Snapshot snapshot;
            fillRenderSnapshot(snapshot, storage, outcome, 0.0);
            return snapshot;
        }

        template <typename StepPoses>
        std::vector<Frame> run(const BrawlerPoses& shown, const BrawlerPoses& correctedHead, StepPoses stepPoses, bool smoothEveryBody)
        {
            capture.noteShownSnapshot(snapshotOf(shown, kCarrierStep - 1u, kShownTick));
            write(correctedHead);
            Correction correction;
            REQUIRE(capture.onStepAfterReplay(1u, kShownTick, kCarrierStep, storage, correction) == RenderCorrectionCaptureOutcome::Captured);
            REQUIRE(ring.tryPush(correction));

            RenderCorrectionParams params;
            params.stepSeconds = kDt;

            std::vector<Frame> frames;
            Snapshot previous;
            for (int frameIndex = 0; frameIndex < kFrames; ++frameIndex)
            {
                const uint64_t step = kCarrierStep + static_cast<uint64_t>(frameIndex);
                const Snapshot next = snapshotOf(stepPoses(frameIndex), step, kShownTick + 1u + static_cast<SimTick>(frameIndex));
                Snapshot poses;
                RenderInterpolationFrame interpolation;
                interpolation.nextStep = step;
                if (frameIndex == 0)
                {
                    poses = next;
                    interpolation.alpha = 0.0;
                    interpolation.hasPrev = false;
                }
                else
                {
                    blendRenderPoses(previous, next, kAlpha, 1.0e6f, poses);
                    interpolation.alpha = kAlpha;
                    interpolation.hasPrev = true;
                    interpolation.prevStep = step - 1u;
                }
                previous = next;
                if (smoothEveryBody)
                {
                    for (uint16_t index = 0; index < poses.bodyCount; ++index)
                    {
                        poses.bodies[index].renderCorrection = static_cast<uint8_t>(RenderCorrectionMode::Smooth);
                    }
                }

                Frame frame;
                frame.supplied = poses;
                smoother.correctPoses(ring, interpolation, params, poses, frame.summary);
                frame.rendered = poses;
                frames.push_back(frame);
            }
            return frames;
        }
    };

    void checkBodyFlags(const Snapshot& snapshot)
    {
        CHECK(bodyOf(snapshot, kCapsule).isRoot == 1u);
        CHECK(bodyOf(snapshot, kCapsule).hasRotation == 0u);
        CHECK(bodyOf(snapshot, kCapsule).renderCorrection == static_cast<uint8_t>(RenderCorrectionMode::Smooth));
        CHECK(bodyOf(snapshot, kRadial).renderCorrection == kSnapInRootFrame);
        CHECK(bodyOf(snapshot, kGuard).renderCorrection == kSnapInRootFrame);
        for (uint8_t projectile = 2u; projectile < kCapsule; ++projectile)
        {
            CHECK(bodyOf(snapshot, projectile).renderCorrection == static_cast<uint8_t>(RenderCorrectionMode::Smooth));
        }
    }
} // namespace dAttackRenderCorrectionPolicyTest

TEST_CASE("DAttack.RenderCorrection.ASwingLearnedFromACorrectionDrawsTheBladeInItsSwingPlaneOnEveryFrame", "[DAttack]")
{
    using namespace dAttackRenderCorrectionPolicyTest;
    Rig rig;
    const glm::vec3 corrected = kCapsuleStart + glm::vec3(0.f, 0.f, -0.02f);
    const glm::quat guardForward = rig.guardRotation(kAimForward);

    BrawlerPoses shown;
    shown.capsule = kCapsuleStart;
    shown.radial = Pose{ kCapsuleStart + kAttachment, rig.idleRadialRotation() };
    shown.guard = Pose{ kCapsuleStart + kAttachment, guardForward };
    REQUIRE(bladeElevationDegrees(shown.radial.rotation) == Catch::Approx(90.0).margin(1e-3));

    const auto swingAt = [&](int swingStep) {
        BrawlerPoses poses;
        poses.capsule = corrected;
        poses.radial = Pose{ corrected + kAttachment, rig.swingRadialRotation(swingStep) };
        poses.guard = Pose{ corrected + kAttachment, guardForward };
        return poses;
    };

    const std::vector<Frame> frames = rig.run(shown, swingAt(kCorrectedSwingStep),
        [&](int frameIndex) { return swingAt(kCorrectedSwingStep + 1 + frameIndex); }, false);

    checkBodyFlags(frames.front().supplied);
    CHECK(frames.front().summary.policySnaps == 2u);
    CHECK(frames.front().summary.maxPolicySnapDeg > 80.0);
    double worstElevation = 0.0;
    for (size_t frameIndex = 0; frameIndex < frames.size(); ++frameIndex)
    {
        const double supplied = bladeElevationDegrees(bodyOf(frames[frameIndex].supplied, kRadial).rotation);
        const double rendered = bladeElevationDegrees(bodyOf(frames[frameIndex].rendered, kRadial).rotation);
        worstElevation = std::max(worstElevation, std::abs(rendered));
        INFO("frame " << frameIndex + 1 << " supplied elevation " << supplied << " rendered elevation " << rendered);
        CHECK(std::abs(supplied) <= 1.0);
        CHECK(std::abs(rendered) <= 1.0);
    }
    INFO("worst rendered elevation " << worstElevation);
    CHECK(worstElevation <= 1.0);
}

TEST_CASE("DAttack.RenderCorrection.TheGuardFacesTheCorrectedAimFromTheFirstCorrectedFrame", "[DAttack]")
{
    using namespace dAttackRenderCorrectionPolicyTest;
    Rig rig;
    const glm::quat idle = rig.idleRadialRotation();
    const glm::quat guardForward = rig.guardRotation(kAimForward);
    const glm::quat guardLeft = rig.guardRotation(kAimLeft);
    REQUIRE(angleBetweenDegrees(guardForward, guardLeft) == Catch::Approx(90.0).margin(1e-2));

    BrawlerPoses shown;
    shown.capsule = kCapsuleStart;
    shown.radial = Pose{ kCapsuleStart + kAttachment, idle };
    shown.guard = Pose{ kCapsuleStart + kAttachment, guardForward };
    BrawlerPoses turned = shown;
    turned.guard.rotation = guardLeft;

    const std::vector<Frame> frames = rig.run(shown, turned, [&](int) { return turned; }, false);

    checkBodyFlags(frames.front().supplied);
    CHECK(frames.front().summary.policySnaps == 1u);
    for (size_t frameIndex = 0; frameIndex < frames.size(); ++frameIndex)
    {
        const glm::quat supplied = bodyOf(frames[frameIndex].supplied, kGuard).rotation;
        const glm::quat rendered = bodyOf(frames[frameIndex].rendered, kGuard).rotation;
        INFO("frame " << frameIndex + 1 << " rendered guard is " << angleBetweenDegrees(supplied, rendered)
                      << " degrees off the supplied pose");
        CHECK(rendered.w == supplied.w);
        CHECK(rendered.x == supplied.x);
        CHECK(rendered.y == supplied.y);
        CHECK(rendered.z == supplied.z);
    }
}

TEST_CASE("DAttack.RenderCorrection.TheWeaponAndTheGuardKeepTheirOffsetFromTheSmoothedCapsule", "[DAttack]")
{
    using namespace dAttackRenderCorrectionPolicyTest;
    const glm::vec3 capsuleError = glm::vec3(0.f, 0.f, -0.4f);
    const glm::vec3 ownError = glm::vec3(0.f, 0.f, -(0.4f - kWalkSpeed * kDt));
    REQUIRE(simFrame::forwardRight(ownError).x == Catch::Approx(0.38333).margin(1e-5));

    const auto runOnce = [&](bool smoothEveryBody) {
        Rig rig;
        const glm::quat idle = rig.idleRadialRotation();
        const glm::quat guardForward = rig.guardRotation(kAimForward);
        const auto capsuleAt = [&](int tickIndex) {
            return kCapsuleStart + glm::vec3(0.f, 0.f, -(kWalkSpeed * kDt * static_cast<float>(tickIndex)));
        };
        const auto posesAt = [&](int tickIndex) {
            BrawlerPoses poses;
            poses.capsule = capsuleAt(tickIndex);
            poses.capsuleVelocity = glm::vec3(0.f, 0.f, -kWalkSpeed);
            poses.radial = Pose{ capsuleAt(tickIndex - 1) + kAttachment, idle };
            poses.guard = Pose{ capsuleAt(tickIndex - 1) + kAttachment, guardForward };
            return poses;
        };
        BrawlerPoses shown = posesAt(0);
        shown.capsule += capsuleError;
        shown.radial.position += ownError;
        shown.guard.position += ownError;
        return rig.run(shown, posesAt(0), [&](int frameIndex) { return posesAt(1 + frameIndex); }, smoothEveryBody);
    };

    const std::vector<Frame> policy = runOnce(false);
    const std::vector<Frame> smoothed = runOnce(true);
    REQUIRE(policy.size() == smoothed.size());

    checkBodyFlags(policy.front().supplied);
    CHECK(policy.front().summary.policySnaps == 2u);
    CHECK(policy.front().summary.smoothedBodies == 1u);
    CHECK(policy.front().summary.maxPolicySnap == Catch::Approx(simFrame::forwardRight(ownError).x).margin(1e-5));
    double worstWeapon = 0.0;
    double worstGuard = 0.0;
    double largestCapsuleOffset = 0.0;
    for (size_t frameIndex = 0; frameIndex < policy.size(); ++frameIndex)
    {
        const Frame& frame = policy[frameIndex];
        const double weapon = glm::length(offsetFromCapsule(frame.rendered, kRadial) - offsetFromCapsule(frame.supplied, kRadial));
        const double guard = glm::length(offsetFromCapsule(frame.rendered, kGuard) - offsetFromCapsule(frame.supplied, kGuard));
        const double capsuleOffset = glm::length(glm::dvec3(bodyOf(frame.rendered, kCapsule).position)
            - glm::dvec3(bodyOf(frame.supplied, kCapsule).position));
        worstWeapon = std::max(worstWeapon, weapon);
        worstGuard = std::max(worstGuard, guard);
        largestCapsuleOffset = std::max(largestCapsuleOffset, capsuleOffset);
        INFO("frame " << frameIndex + 1 << " capsule drawn " << capsuleOffset << " cm off its pose; weapon offset error "
                      << weapon << " cm, guard offset error " << guard << " cm");
        CHECK(weapon <= 1e-5);
        CHECK(guard <= 1e-5);

        const RenderBody& capsule = bodyOf(frame.rendered, kCapsule);
        const RenderBody& reference = bodyOf(smoothed[frameIndex].rendered, kCapsule);
        CHECK(std::memcmp(&capsule.position, &reference.position, sizeof(capsule.position)) == 0);
        CHECK(std::memcmp(&capsule.rotation, &reference.rotation, sizeof(capsule.rotation)) == 0);
    }
    INFO("worst weapon " << worstWeapon << " cm, worst guard " << worstGuard << " cm, largest capsule offset "
                         << largestCapsuleOffset << " cm");
    CHECK(largestCapsuleOffset > 0.3);
    CHECK(worstWeapon <= 1e-5);
    CHECK(worstGuard <= 1e-5);
}

#endif
