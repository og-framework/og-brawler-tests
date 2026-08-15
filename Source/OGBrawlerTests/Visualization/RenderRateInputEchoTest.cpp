// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include <cmath>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/BrawlerVisualizationInputSource.h"
#include "OGBrawler/DAttackAimVisualization.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include "../InputPackaging/MockInputSource.h"

// ---------------------------------------------------------------------------
// T13 / D5.4 caller half — render-side input echo.
//
// What this suite pins: the LOCAL-vs-REMOTE visualization input SOURCE RULE, and
// the fact that driving the real dAttackAimVisualization::visualize through it at
// render-frame rate makes the renderer observe a FRESH input on every frame for a
// local character, and a HELD-CONSTANT input for a remote proxy.
//
// Structural limitation, stated up front so no one over-reads a green run: this
// tree cannot link UE, so SimmableUpdateComponent.cpp (the translation unit that
// applies the rule) is NOT compiled here. What is verified is the rule itself plus
// its composition against the real visualize() signature. That the component calls
// it with the right two arguments is verified by inspection and in PIE — the same
// boundary T12 documented.
// ---------------------------------------------------------------------------

namespace renderechotests
{

// Renderer double recording the geometry the aim viz emits. The first drawLine in
// dAttackAimVisualization::visualize is the aim ray itself
// (rootTranslation - aim*30 .. rootTranslation + aim*outerRadius), so the recorded
// endpoints are a direct, observable function of the input's aim direction — which
// is what lets this suite assert "fresh input each call" from the RENDERER's side
// rather than by re-reading the value the test itself just constructed.
struct RecordingRenderer
{
	std::vector<glm::vec3>* lineEnds = nullptr;

	void drawLine(const glm::vec3&, const glm::vec3& end, unsigned int, float) { lineEnds->push_back(end); }
	void drawPoint(const glm::vec3&) {}
	void drawSphere(const glm::vec3&, float, unsigned int, float) {}
	void drawCircleArc(const glm::vec3&, const glm::vec3&, float, float, unsigned int, float) {}
	void drawMesh(const std::vector<glm::vec3>&, const std::vector<unsigned int>&, unsigned int, unsigned int = 150) {}
	void drawTriangle(const glm::vec3&, const glm::vec3&, const glm::vec3&, unsigned int) {}
};

struct SilentLogger
{
	void logVec3(const char*, const glm::vec3&) {}
	void logInt(const char*, int) {}
};

// Minimal radial fixture the aim viz needs. Empty sequence table with an invalid
// active sequence, so the (guarded) sequence lookup is never taken — this suite is
// about input plumbing, not attack-sequence geometry.
struct AimVizFixture
{
	std::vector<DAttackRadialSequence> sequences;
	DAttackCircle circle{ 8u, 50.f, 100.f, 10.f, false, 1.f };
	dAttackRadialSimulation::StaticData staticData{ sequences, circle };

	dAttackRadialSimulation::State state;
	dAttackRadialSimulation::DerivedState derived;
	dAttackRadialSimulation::InitialConditions ic;

	AimVizFixture()
	{
		state.bodyState.position = glm::vec3(0.f);
		state.bodyState.rotation = glm::quat(1.f, 0.f, 0.f, 0.f);
	}
};

// One render frame of the production composition, reproduced exactly:
//   select source -> unpack the machine sub-input -> build Input -> visualize.
//
// Returns the endpoint of the FIRST drawLine this frame emitted, or nullopt if the
// viz did not run (mirroring the has_value() gate at the call site). visualize()
// emits several lines per frame (aim ray, then weapon-indicator geometry); the aim
// ray is emitted first and is a direct function of the input's aim direction, which
// is the observable this suite asserts on. Recording per-frame rather than into one
// flat buffer matters: a flat buffer interleaves each frame's several draws and the
// n-th entry is NOT the n-th frame.
template <typename LiveSampler>
std::optional<glm::vec3> renderFrameAimRay(
	AimVizFixture& fx,
	dAttackAimVisualization::State& vizState,
	bool hasLiveLocalInput,
	LiveSampler&& sampleLive,
	const std::optional<simulatableBrawler::PlayerInput>& cachedInput)
{
	const std::optional<simulatableBrawler::PlayerInput> selected =
		simulatableBrawler::selectVisualizationInput(
			hasLiveLocalInput, std::forward<LiveSampler>(sampleLive), cachedInput);

	if (!selected.has_value())
		return std::nullopt;

	const auto& machineInput = selected->get<dAttackMachineSimulation::PlayerInput>();

	std::vector<glm::vec3> thisFrame;
	RecordingRenderer renderer{ &thisFrame };
	SilentLogger logger;
	dAttackAimVisualization::Input<RecordingRenderer, SilentLogger> input(
		1.f / 240.f,
		machineInput.aimDirection,
		renderer,
		logger,
		machineInput.moveDirection,
		machineInput.moveDirectionWorld);

	dAttackAimVisualization::visualize(
		input, fx.state, fx.ic, fx.derived, fx.staticData, vizState);

	REQUIRE_FALSE(thisFrame.empty()); // the aim ray is unconditional
	return thisFrame.front();
}

// A non-zero move direction. dAttackAimVisualization::visualize normalizes
// moveDirectionWorld unconditionally, so a zero vector yields NaN geometry — see
// the ZeroMoveDirectionWorld case at the bottom of this file, which pins that
// PRE-EXISTING behaviour rather than working around it silently.
inline glm::vec3 someMoveDirection() { return glm::vec3(0.f, 1.f, 0.f); }

// Builds the live sample a local character would produce this frame, through the
// real T12 packer — so anything the packer pins (discrete neutrality) holds here too.
inline simulatableBrawler::PlayerInput liveSampleWithAim(const glm::vec3& aim)
{
	MockInputSource src;
	src.aimDirection       = aim;
	src.moveStick          = glm::vec2(0.f, 1.f);
	src.moveDirectionWorld = someMoveDirection();
	return simulatableBrawler::makeVisualizationPlayerInput(
		simulatableBrawler::readContinuousInputFields(src));
}

// The tick-quantized value a correction cache would hold: one aim, held for the
// whole sim tick.
inline simulatableBrawler::PlayerInput cachedSampleWithAim(const glm::vec3& aim)
{
	MockInputSource src;
	src.aimDirection       = aim;
	src.moveStick          = glm::vec2(0.f, 1.f);
	src.moveDirectionWorld = someMoveDirection();
	return simulatableBrawler::makeSimPlayerInput(
		simulatableBrawler::readContinuousInputFields(src),
		/*leftAttack*/ false, /*rightAttack*/ false, inputSequence::kNoMatch);
}

// Four render frames per 60 Hz sim tick — the AC's 4x sampling rate.
constexpr int kRenderFramesPerSimTick = 4;

// Aim direction at render frame i, sweeping within a single sim tick. Distinct per
// frame, which is exactly the sub-tick motion the echo is supposed to expose.
inline glm::vec3 aimAtRenderFrame(int i)
{
	const float t = 0.05f * static_cast<float>(i + 1);
	return glm::normalize(glm::vec3(1.f, t, 0.f));
}

} // namespace renderechotests

// ---------------------------------------------------------------------------
// LOCAL: every render frame re-samples, and the renderer sees a different aim ray
// on each of the four frames within one sim tick.
// ---------------------------------------------------------------------------
TEST_CASE("Visualization.RenderRateInputEcho.LocalCharacterEchoesFreshInputEveryRenderFrame",
          "[CharacterViz][RenderRateInputEcho]")
{
	using namespace renderechotests;

	AimVizFixture fx;
	dAttackAimVisualization::State vizState;

	// The cache is frozen at the tick boundary — if the echo were not live, every
	// frame would draw THIS aim.
	const glm::vec3 staleAim = glm::normalize(glm::vec3(0.f, -1.f, 0.f));
	const std::optional<simulatableBrawler::PlayerInput> cached = cachedSampleWithAim(staleAim);

	int sampleCount = 0;
	std::vector<glm::vec3> aimRays;
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
	{
		const std::optional<glm::vec3> ray = renderFrameAimRay(fx, vizState,
			/*hasLiveLocalInput*/ true,
			[&]() { ++sampleCount; return liveSampleWithAim(aimAtRenderFrame(frame)); },
			cached);
		REQUIRE(ray.has_value());
		aimRays.push_back(*ray);
	}

	// Sampled once per render frame — not once per sim tick.
	REQUIRE(sampleCount == kRenderFramesPerSimTick);
	REQUIRE(aimRays.size() == static_cast<size_t>(kRenderFramesPerSimTick));

	const float outerRadius = fx.circle.getOuterRadius();
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
	{
		const glm::vec3 expected = aimAtRenderFrame(frame) * outerRadius;
		REQUIRE(aimRays[frame].x == Catch::Approx(expected.x).margin(1e-3));
		REQUIRE(aimRays[frame].y == Catch::Approx(expected.y).margin(1e-3));
	}

	// And every frame differs from its predecessor — the property that fails if the
	// source silently reverts to a tick-quantized read.
	for (int frame = 1; frame < kRenderFramesPerSimTick; ++frame)
		REQUIRE(glm::length(aimRays[frame] - aimRays[frame - 1]) > 1e-3f);

	// The stale cached aim was never drawn.
	const glm::vec3 staleEnd = staleAim * outerRadius;
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
		REQUIRE(glm::length(aimRays[frame] - staleEnd) > 1e-3f);
}

// ---------------------------------------------------------------------------
// REMOTE: the live sampler is never invoked, and all four frames draw the same
// cached aim. This is the "verified unchanged for remote proxies" acceptance
// criterion, asserted rather than argued.
// ---------------------------------------------------------------------------
TEST_CASE("Visualization.RenderRateInputEcho.RemoteProxyKeepsCachedInputAndNeverSamplesLive",
          "[CharacterViz][RenderRateInputEcho]")
{
	using namespace renderechotests;

	AimVizFixture fx;
	dAttackAimVisualization::State vizState;

	const glm::vec3 cachedAim = glm::normalize(glm::vec3(1.f, 0.f, 0.f));
	const std::optional<simulatableBrawler::PlayerInput> cached = cachedSampleWithAim(cachedAim);

	int sampleCount = 0;
	std::vector<glm::vec3> aimRays;
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
	{
		const std::optional<glm::vec3> ray = renderFrameAimRay(fx, vizState,
			/*hasLiveLocalInput*/ false,
			[&]() { ++sampleCount; return liveSampleWithAim(aimAtRenderFrame(frame)); },
			cached);
		REQUIRE(ray.has_value());
		aimRays.push_back(*ray);
	}

	// The contract is that a remote proxy does not merely IGNORE the live read — it
	// never performs one. Someone else's character must not observe our input at all.
	REQUIRE(sampleCount == 0);

	// Held constant across all four render frames — the tick-quantized behaviour,
	// unchanged by T13.
	const glm::vec3 expected = cachedAim * fx.circle.getOuterRadius();
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
	{
		REQUIRE(aimRays[frame].x == Catch::Approx(expected.x).margin(1e-3));
		REQUIRE(aimRays[frame].y == Catch::Approx(expected.y).margin(1e-3));
		REQUIRE(glm::length(aimRays[frame] - aimRays[0]) < 1e-4f);
	}
}

// ---------------------------------------------------------------------------
// REMOTE + cold cache: viz skipped, exactly as before T13. Pins that the rule
// forwards nullopt rather than substituting a neutral input (which would draw a
// bogus aim ray for a character we know nothing about).
// ---------------------------------------------------------------------------
TEST_CASE("Visualization.RenderRateInputEcho.RemoteProxyWithColdCacheSkipsVisualization",
          "[CharacterViz][RenderRateInputEcho]")
{
	using namespace renderechotests;

	AimVizFixture fx;
	dAttackAimVisualization::State vizState;

	int sampleCount = 0;
	const std::optional<glm::vec3> ray = renderFrameAimRay(fx, vizState,
		/*hasLiveLocalInput*/ false,
		[&]() { ++sampleCount; return liveSampleWithAim(aimAtRenderFrame(0)); },
		std::nullopt);

	REQUIRE_FALSE(ray.has_value()); // nothing drawn at all
	REQUIRE(sampleCount == 0);
}

// ---------------------------------------------------------------------------
// LISTEN-SERVER HOST: local character with NO cached input source at all. Before
// T13 this combination skipped the viz on every frame; now it echoes. This case
// IS the listen-server improvement, pinned so a regression is loud.
//
// (Historically the absent source was the correction cache's input column, which
// the authority never populated because it allocates no caches — `getLatestInput`
// answered nullopt there. [og-netcode-v2-input-relay T16] That column and that
// accessor are retired outright, so the nullopt this case models is now the
// ORDINARY answer on every role rather than an authority-specific one. The case
// itself passes `std::nullopt` directly and never named either symbol.)
// ---------------------------------------------------------------------------
TEST_CASE("Visualization.RenderRateInputEcho.ListenServerHostEchoesWithNoCorrectionCache",
          "[CharacterViz][RenderRateInputEcho]")
{
	using namespace renderechotests;

	AimVizFixture fx;
	dAttackAimVisualization::State vizState;

	int sampleCount = 0;
	std::vector<glm::vec3> aimRays;
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
	{
		const std::optional<glm::vec3> ray = renderFrameAimRay(fx, vizState,
			/*hasLiveLocalInput*/ true,
			[&]() { ++sampleCount; return liveSampleWithAim(aimAtRenderFrame(frame)); },
			std::nullopt); // authority: no correction cache at all
		REQUIRE(ray.has_value());
		aimRays.push_back(*ray);
	}

	REQUIRE(sampleCount == kRenderFramesPerSimTick);

	const float outerRadius = fx.circle.getOuterRadius();
	for (int frame = 0; frame < kRenderFramesPerSimTick; ++frame)
	{
		const glm::vec3 expected = aimAtRenderFrame(frame) * outerRadius;
		REQUIRE(aimRays[frame].x == Catch::Approx(expected.x).margin(1e-3));
		REQUIRE(aimRays[frame].y == Catch::Approx(expected.y).margin(1e-3));
	}
}

// ---------------------------------------------------------------------------
// Discrete input edges structurally cannot render-echo. The live source is at the
// "everything pressed" extreme; the echoed value must still carry neutral discrete
// fields on EVERY sub-input. (T12 pins this for the packer; this pins that the T13
// path did not reintroduce a discrete field on the way through.)
// ---------------------------------------------------------------------------
TEST_CASE("Visualization.RenderRateInputEcho.DiscreteEdgesNeverRenderEcho",
          "[CharacterViz][RenderRateInputEcho]")
{
	using namespace renderechotests;

	MockInputSource pressed;
	pressed.aimDirection       = glm::normalize(glm::vec3(1.f, 1.f, 0.f));
	pressed.moveStick          = glm::vec2(0.f, 1.f);
	pressed.moveDirectionWorld = someMoveDirection();
	pressed.leftAttack         = true;
	pressed.rightAttack        = true;
	pressed.triggeredActionId  = 4242u;

	// A cache value that DOES carry discrete edges, so a rule that wrongly fell back
	// to the cache on the local path would be caught here too.
	const std::optional<simulatableBrawler::PlayerInput> cachedWithEdges =
		simulatableBrawler::makeSimPlayerInput(
			simulatableBrawler::readContinuousInputFields(pressed),
			/*leftAttack*/ true, /*rightAttack*/ true, /*triggeredActionId*/ 4242u);

	const std::optional<simulatableBrawler::PlayerInput> echoed =
		simulatableBrawler::selectVisualizationInput(
			/*hasLiveLocalInput*/ true,
			[&]() {
				return simulatableBrawler::makeVisualizationPlayerInput(
					simulatableBrawler::readContinuousInputFields(pressed));
			},
			cachedWithEdges);

	REQUIRE(echoed.has_value());

	const auto& machine = echoed->get<dAttackMachineSimulation::PlayerInput>();
	REQUIRE_FALSE(machine.attackLeft);
	REQUIRE_FALSE(machine.attackRight);
	REQUIRE(machine.triggeredActionId == inputSequence::kNoMatch);

	const auto& radial = echoed->get<dAttackRadialSimulation::PlayerInput>();
	REQUIRE_FALSE(radial.attackLeft);
	REQUIRE_FALSE(radial.attackRight);

	// Continuous fields DID come through — otherwise the neutrality above would be
	// vacuous (an all-zero result would also pass it).
	REQUIRE(machine.aimDirection.x == Catch::Approx(pressed.aimDirection.x).margin(1e-5));
	REQUIRE(machine.aimDirection.y == Catch::Approx(pressed.aimDirection.y).margin(1e-5));
	REQUIRE(machine.moveDirection.y == Catch::Approx(pressed.moveStick.y).margin(1e-5));
}

// ---------------------------------------------------------------------------
// MESH-ONLY INVARIANT (proposal §2.3 / R-UE1), compile-anchored.
//
// Taking the address of the instantiated visualize() specialization pins its FULL
// signature, constness included. If anyone makes a simulation-side parameter
// mutable — the exact failure R-UE1 exists to prevent — this static_assert stops
// compiling. Only the visualization State& is non-const, which is the whole point:
// visualization owns its own state and nothing else.
// ---------------------------------------------------------------------------
namespace renderechotests
{

using AimVisualizeFn = void (*)(
	const dAttackAimVisualization::Input<RecordingRenderer, SilentLogger>&,
	const dAttackRadialSimulation::State&,
	const dAttackRadialSimulation::InitialConditions&,
	const dAttackRadialSimulation::DerivedState&,
	const dAttackRadialSimulation::StaticData&,
	dAttackAimVisualization::State&);

static_assert(
	std::is_same_v<
		decltype(&dAttackAimVisualization::visualize<RecordingRenderer, SilentLogger>),
		AimVisualizeFn>,
	"MESH-ONLY INVARIANT (R-UE1): dAttackAimVisualization::visualize must take every "
	"simulation-side parameter by const reference. Only its own visualization State "
	"may be mutable. See OGBrawler/VISUALIZATION_DISCIPLINE.md.");

} // namespace renderechotests

TEST_CASE("Visualization.RenderRateInputEcho.MeshOnlyInvariantIsCompileAnchored",
          "[CharacterViz][RenderRateInputEcho]")
{
	// The guarantee is the static_assert above; this case exists so the invariant is
	// visible in the test report rather than only in a header nobody runs.
	using namespace renderechotests;
	SUCCEED("dAttackAimVisualization::visualize signature pinned const on all sim-side params");
}

// ---------------------------------------------------------------------------
// PRE-EXISTING BEHAVIOUR, PINNED NOT FIXED: dAttackAimVisualization::visualize
// normalizes moveDirectionWorld unconditionally, so an idle move stick — which
// buildMoveDirectionWorld deliberately reports as the zero vector — yields NaN
// geometry. This predates T13 (the correction cache stored the same zero vector,
// so the tick-quantized path hit it too), so T13 does NOT change it. It is pinned
// here because T13 makes the path far more reachable: every render frame instead
// of every sim tick, and now on the listen-server host as well.
//
// If this case ever starts failing, someone added the guard — update the impl
// notes' open item rather than deleting the case.
// ---------------------------------------------------------------------------
TEST_CASE("Visualization.RenderRateInputEcho.ZeroMoveDirectionWorldStillNaNs_PreExisting",
          "[CharacterViz][RenderRateInputEcho]")
{
	using namespace renderechotests;

	MockInputSource idle; // moveStick zero, moveDirectionWorld zero — the defaults
	idle.aimDirection = glm::normalize(glm::vec3(1.f, 0.f, 0.f));

	const simulatableBrawler::PlayerInput echoed =
		simulatableBrawler::makeVisualizationPlayerInput(
			simulatableBrawler::readContinuousInputFields(idle));

	const auto& machine = echoed.get<dAttackMachineSimulation::PlayerInput>();
	REQUIRE(glm::length(machine.moveDirectionWorld) == Catch::Approx(0.f).margin(1e-6));

	// This is what the viz would then compute. Documented, not asserted through
	// visualize() itself — driving NaN geometry into the renderer proves nothing
	// extra and makes the failure mode harder to read.
	const glm::vec3 normalized = glm::normalize(machine.moveDirectionWorld);
	REQUIRE(std::isnan(normalized.x));
}

#endif // WITH_LOW_LEVEL_TESTS
