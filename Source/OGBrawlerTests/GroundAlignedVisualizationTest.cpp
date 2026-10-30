// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/trigonometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGBrawler/DAttackAimVisualization.h"
#include "OGBrawler/DAttackBlockPredictionVisualization.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackGuardSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/DAttackTargetVisualizationTwo.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SurfaceFrame.h"

namespace groundAlignedVisualizationTests
{
constexpr float kPlaneTolerance = 1e-4f;
constexpr glm::vec3 kRoot = glm::vec3(-0.35f, 0.42f, -1.2f);
const glm::vec3 kThirtyDegreeNormal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));
const glm::vec3 kTwentyDegreeNormal(std::sin(glm::radians(20.f)), std::cos(glm::radians(20.f)), 0.f);

enum class Op : std::uint32_t
{
	Line = 1,
	CircleArc = 2,
	Triangle = 3,
	Mesh = 4,
	Sphere = 5,
	Point = 6,
};

struct Call
{
	Op op = Op::Line;
	unsigned int colorId = 0;
	std::vector<glm::vec3> points;
	std::vector<std::uint32_t> words;
};

struct Recording
{
	std::vector<Call> calls;
};

void pushFloat(std::vector<std::uint32_t>& words, float value)
{
	words.push_back(std::bit_cast<std::uint32_t>(value));
}

void pushVec(std::vector<std::uint32_t>& words, const glm::vec3& v)
{
	pushFloat(words, v.x);
	pushFloat(words, v.y);
	pushFloat(words, v.z);
}

std::vector<glm::vec3> engineArcSamples(const glm::vec3& center, const glm::vec3& direction, float radius, float halfAngle)
{
	const glm::vec3 along = glm::normalize(direction);
	const glm::vec3 across = glm::normalize(glm::cross(simFrame::kUp, along));
	std::vector<glm::vec3> samples;
	for (const float angle : {-halfAngle, 0.f, halfAngle})
	{
		samples.push_back(center + radius * (std::cos(angle) * along + std::sin(angle) * across));
	}
	return samples;
}

struct RecordingRenderer
{
	Recording* recording = nullptr;

	void drawLine(const glm::vec3& start, const glm::vec3& end, unsigned int colorId, float thickness) const
	{
		Call call{Op::Line, colorId, {start, end}, {}};
		pushVec(call.words, start);
		pushVec(call.words, end);
		call.words.push_back(colorId);
		pushFloat(call.words, thickness);
		recording->calls.push_back(call);
	}
	void drawCircleArc(const glm::vec3& center, const glm::vec3& direction, float radius, float halfAngle,
		unsigned int colorId, float thickness) const
	{
		Call call{Op::CircleArc, colorId, engineArcSamples(center, direction, radius, halfAngle), {}};
		pushVec(call.words, center);
		pushVec(call.words, direction);
		pushFloat(call.words, radius);
		pushFloat(call.words, halfAngle);
		call.words.push_back(colorId);
		pushFloat(call.words, thickness);
		recording->calls.push_back(call);
	}
	void drawTriangle(const glm::vec3& one, const glm::vec3& two, const glm::vec3& three, unsigned int colorId) const
	{
		Call call{Op::Triangle, colorId, {one, two, three}, {}};
		pushVec(call.words, one);
		pushVec(call.words, two);
		pushVec(call.words, three);
		call.words.push_back(colorId);
		recording->calls.push_back(call);
	}
	void drawMesh(const std::vector<glm::vec3>& vertices, const std::vector<unsigned int>& indices, unsigned int colorId,
		unsigned int alpha = 150) const
	{
		Call call{Op::Mesh, colorId, vertices, {}};
		call.words.push_back(static_cast<std::uint32_t>(vertices.size()));
		for (const glm::vec3& vertex : vertices)
		{
			pushVec(call.words, vertex);
		}
		call.words.push_back(static_cast<std::uint32_t>(indices.size()));
		for (const unsigned int index : indices)
		{
			call.words.push_back(index);
		}
		call.words.push_back(colorId);
		call.words.push_back(alpha);
		recording->calls.push_back(call);
	}
	void drawSphere(const glm::vec3& position, float radius, unsigned int colorId, float lifetime = -1.f) const
	{
		Call call{Op::Sphere, colorId, {position}, {}};
		pushVec(call.words, position);
		pushFloat(call.words, radius);
		call.words.push_back(colorId);
		pushFloat(call.words, lifetime);
		recording->calls.push_back(call);
	}
	void drawPoint(const glm::vec3& point) const
	{
		Call call{Op::Point, 0u, {point}, {}};
		pushVec(call.words, point);
		recording->calls.push_back(call);
	}
};

struct NullLogger
{
	void logVec3(const char*, const glm::vec3&) const {}
	void logInt(const char*, int) const {}
};

struct MockQueryAdapter
{
	SpatialQueryReport report;
	void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
	SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return report; }
};

struct MockPhysicsReader
{
	glm::mat4 guardTransform{1.f};
	bool isBodyResolvable(BodyId) const { return true; }
	glm::mat4 getBodyTransform(BodyId) const { return guardTransform; }
};

std::uint64_t hashOf(const Recording& recording)
{
	std::uint64_t hash = 0xcbf29ce484222325ull;
	const auto mix = [&hash](std::uint32_t word) {
		for (int byte = 0; byte < 4; ++byte)
		{
			hash ^= (word >> (8 * byte)) & 0xffu;
			hash *= 0x100000001b3ull;
		}
	};
	mix(static_cast<std::uint32_t>(recording.calls.size()));
	for (const Call& call : recording.calls)
	{
		mix(static_cast<std::uint32_t>(call.op));
		mix(static_cast<std::uint32_t>(call.words.size()));
		for (const std::uint32_t word : call.words)
		{
			mix(word);
		}
	}
	return hash;
}

const dAttackRadialSimulation::StaticData& staticData()
{
	static const std::vector<DAttackRadialSequence> sequences = [] {
		std::vector<DAttackRadialSequence> out;
		out.emplace_back(
			std::vector<DAttackRadialSequencePoint>{
				{0.0f, 0.0f, DAttackRadialSequenceState::Damaging},
				{0.2f, 7.0f, DAttackRadialSequenceState::Idle}},
			0.1f,
			DAttackRadialSequence::turnAxis());
		return out;
	}();
	static const DAttackCircle circle(8u, 0.9f, 3.f, 0.7f, false, 1.f);
	static const dAttackRadialSimulation::StaticData data(sequences, circle);
	return data;
}

glm::vec3 aimAtDegrees(float degrees)
{
	const glm::vec3 forward = simFrame::kForward;
	return glm::vec3(glm::rotate(glm::mat4(1.f), glm::radians(degrees), simFrame::kTurnAxis) * glm::vec4(forward, 0.f));
}

struct LocalAim
{
	float angle = 0.f;
	glm::vec3 axis{0.f};
};

LocalAim localAimOn(const glm::vec3& normal, const glm::vec3& aim)
{
	const glm::vec3 heading = glm::normalize(simFrame::flatten(surfaceFrame::tiltFromSurface(normal, aim)));
	const glm::vec3 forward = simFrame::kForward;
	const float aimDot = glm::clamp(glm::dot(heading, forward), -1.f, 1.f);
	LocalAim out;
	out.angle = glm::acos(aimDot);
	out.axis = glm::abs(glm::abs(aimDot) - 1.f) < 0.0001f ? glm::vec3(0.f, -1.f, 0.f)
		: glm::normalize(glm::cross(forward, heading));
	return out;
}

dAttackGuardSimulation::State guardStateOn(const glm::vec3& normal, const glm::vec3& aim)
{
	const LocalAim local = localAimOn(normal, aim);
	glm::mat4 pose = glm::rotate(glm::mat4(1.f), local.angle, local.axis);
	if (!surfaceFrame::isFlat(normal))
	{
		pose = glm::mat4(surfaceFrame::tiltMatrix(normal)) * pose;
	}
	dAttackGuardSimulation::State guard;
	guard.bodyState.position = kRoot;
	guard.bodyState.rotation = glm::quat_cast(pose);
	return guard;
}

glm::vec3 guardUpOf(const dAttackGuardSimulation::State& guard)
{
	const glm::mat4 guardTransform = glm::translate(glm::mat4(1.f), guard.bodyState.position)
		* glm::mat4_cast(guard.bodyState.rotation);
	return surfaceFrame::surfaceUpOf(guardTransform);
}

dAttackRadialSimulation::InitialConditions idleIc()
{
	dAttackRadialSimulation::InitialConditions ic{};
	ic.activeAttackSequence = InvalidAttackSequenceId;
	return ic;
}

dAttackRadialSimulation::InitialConditions swingIcOn(const glm::vec3& normal, const glm::vec3& aim)
{
	const LocalAim local = localAimOn(normal, aim);
	dAttackRadialSimulation::InitialConditions ic{};
	ic.activeAttackSequence = 0u;
	if (surfaceFrame::isFlat(normal))
	{
		ic.initialAimAngle = local.angle;
		ic.initialAimRotationAxis = local.axis;
	}
	else
	{
		const surfaceFrame::AngleAxis frame = surfaceFrame::toAngleAxis(surfaceFrame::tiltMatrix(normal)
			* glm::mat3(glm::rotate(glm::mat4(1.f), local.angle, local.axis)));
		ic.initialAimAngle = frame.angle;
		ic.initialAimRotationAxis = frame.axis;
	}
	return ic;
}

glm::vec3 swingUpOf(const dAttackRadialSimulation::InitialConditions& ic)
{
	constexpr glm::vec3 up = simFrame::kUp;
	return glm::mat3(glm::rotate(glm::mat4(1.f), ic.initialAimAngle, ic.initialAimRotationAxis)) * up;
}

enum class MoveKind
{
	None,
	Left,
	Right,
};

const char* nameOf(MoveKind move)
{
	switch (move)
	{
	case MoveKind::None: return "none";
	case MoveKind::Left: return "left";
	case MoveKind::Right: return "right";
	}
	return "?";
}

glm::vec2 stickFor(MoveKind move)
{
	return move == MoveKind::None ? glm::vec2(0.f) : glm::vec2(1.f, 0.f);
}

glm::vec3 moveWorldFor(MoveKind move, float aimDegrees)
{
	if (move == MoveKind::None)
	{
		return glm::vec3(0.f);
	}
	return aimAtDegrees(aimDegrees + (move == MoveKind::Left ? -70.f : 70.f));
}

unsigned int expectedSequenceFor(MoveKind move)
{
	return move == MoveKind::None ? dAttackDirection::kForwardSequenceId
		: (move == MoveKind::Left ? dAttackDirection::kLeftSequenceId : dAttackDirection::kRightSequenceId);
}

Recording runAim(float aimDegrees, MoveKind move, const dAttackRadialSimulation::InitialConditions& ic,
	const dAttackGuardSimulation::State& guard)
{
	dAttackRadialSimulation::State radial;
	radial.bodyState.position = kRoot;
	radial.currenSequenceId = InvalidAttackSequenceId;
	dAttackRadialSimulation::DerivedState derived;
	Recording recording;
	const RecordingRenderer renderer{&recording};
	dAttackAimVisualization::Input<RecordingRenderer, NullLogger> input(1.f / 60.f, aimAtDegrees(aimDegrees), renderer,
		NullLogger{}, stickFor(move), moveWorldFor(move, aimDegrees));
	dAttackAimVisualization::State vizState;
	dAttackAimVisualization::visualize(input, radial, ic, derived, staticData(), guard, vizState);
	return recording;
}

constexpr std::uint32_t kDefenderRoot = 10u;
constexpr std::uint32_t kDefenderGuard = 11u;
constexpr std::uint32_t kDefenderBody = 12u;

SpatialQueryReport defenderReport(const glm::vec3& position)
{
	SpatialQueryReport report;
	report.hits.push_back({position, BodyId{kDefenderGuard}, BodyId{kDefenderRoot}, CollisionCategories::single(collisionCategory::guard)});
	report.hits.push_back({position, BodyId{kDefenderBody}, BodyId{kDefenderRoot}, CollisionCategories::single(collisionCategory::body)});
	return report;
}

Recording runBlockPrediction(float aimDegrees, MoveKind move, const dAttackGuardSimulation::State& guard)
{
	dAttackRadialSimulation::State radial;
	radial.bodyState.position = kRoot;
	radial.currenSequenceId = InvalidAttackSequenceId;
	const glm::vec3 defender = kRoot + aimAtDegrees(aimDegrees) * 1.5f;
	MockQueryAdapter query;
	query.report = defenderReport(defender);
	MockPhysicsReader reader;
	reader.guardTransform = glm::rotate(glm::mat4(1.f), glm::radians(aimDegrees + 185.f), simFrame::kTurnAxis);
	reader.guardTransform[3] = glm::vec4(defender, 1.f);
	Recording recording;
	const RecordingRenderer renderer{&recording};
	dAttackBlockPredictionVisualization::Input<MockPhysicsReader, MockQueryAdapter, RecordingRenderer> input(1.f / 60.f,
		aimAtDegrees(aimDegrees), stickFor(move), moveWorldFor(move, aimDegrees), reader, query, renderer);
	dAttackBlockPredictionVisualization::State vizState(std::vector<QueryVolumeId>{QueryVolumeId{3u}});
	dAttackBlockPredictionVisualization::visualize(input, vizState, staticData(), radial, guard);
	return recording;
}

Recording runLegacyGuardArcs(float aimDegrees, DAttackState machineState, const dAttackGuardSimulation::State& guard)
{
	dAttackRadialSimulation::State radial;
	radial.bodyState.position = kRoot;
	radial.currenSequenceId = InvalidAttackSequenceId;
	dAttackMachineSimulation::State machine;
	machine.m_currentState = machineState;
	MockQueryAdapter query;
	SpatialQueryReport report;
	report.hits.push_back({kRoot + aimAtDegrees(aimDegrees + 8.f) * 1.5f, BodyId{21u}, BodyId{20u},
		CollisionCategories::single(collisionCategory::guard)});
	report.hits.push_back({kRoot + aimAtDegrees(aimDegrees + 35.f) * 1.2f + simFrame::kUp * 0.2f, BodyId{23u}, BodyId{22u},
		CollisionCategories::single(collisionCategory::guard)});
	query.report = report;
	Recording recording;
	const RecordingRenderer renderer{&recording};
	dAttackTargetVisualizationTwo::Input<MockQueryAdapter, RecordingRenderer> input(1.f / 60.f, aimAtDegrees(aimDegrees),
		query, renderer, true);
	dAttackTargetVisualizationTwo::State vizState(std::vector<QueryVolumeId>{QueryVolumeId{3u}});
	dAttackTargetVisualizationTwo::visualize(input, vizState, staticData(), machine, radial, guard);
	return recording;
}

struct PlaneCheck
{
	int pointsChecked = 0;
	int nonFinite = 0;
	float worstDistance = 0.f;
};

void checkInPlane(const Recording& recording, const glm::vec3& planeNormal, PlaneCheck& check,
	bool (*selects)(const Call&))
{
	for (const Call& call : recording.calls)
	{
		if (!selects(call))
		{
			continue;
		}
		for (const glm::vec3& point : call.points)
		{
			const float distance = std::abs(glm::dot(point - kRoot, planeNormal));
			if (!std::isfinite(distance))
				++check.nonFinite;
			else
				check.worstDistance = std::max(check.worstDistance, distance);
			++check.pointsChecked;
		}
	}
}

bool anyCall(const Call&) { return true; }
bool isWeaponCall(const Call& call) { return call.colorId == 0u; }
bool isNotWeaponCall(const Call& call) { return call.colorId != 0u; }

int countColor(const Recording& recording, unsigned int colorId)
{
	int count = 0;
	for (const Call& call : recording.calls)
	{
		count += call.colorId == colorId ? 1 : 0;
	}
	return count;
}

void checkAimIndicator(const Recording& recording, MoveKind move, const glm::vec3& up, float aimDegrees, bool idle,
	PlaneCheck& ground, PlaneCheck& upright)
{
	if (idle && move == MoveKind::None)
	{
		checkInPlane(recording, up, ground, isNotWeaponCall);
		const glm::vec3 aim = aimAtDegrees(aimDegrees);
		const glm::vec3 heading = glm::normalize(aim - up * glm::dot(aim, up));
		checkInPlane(recording, glm::normalize(glm::cross(up, heading)), upright, isWeaponCall);
	}
	else
	{
		checkInPlane(recording, up, ground, anyCall);
	}
}

constexpr float kAims[] = {0.f, 37.f, 90.f, 200.f, 300.f};
constexpr MoveKind kMoves[] = {MoveKind::None, MoveKind::Left, MoveKind::Right};

struct FlatCase
{
	std::string name;
	Recording recording;
};

std::vector<FlatCase> flatCases()
{
	std::vector<FlatCase> cases;
	const glm::vec3 flat = simFrame::kUp;
	for (const float aim : kAims)
	{
		const dAttackGuardSimulation::State guard = guardStateOn(flat, aimAtDegrees(aim));
		for (const MoveKind move : kMoves)
		{
			const std::string suffix = std::to_string(static_cast<int>(aim)) + "/" + nameOf(move);
			cases.push_back({"aim.idle/" + suffix, runAim(aim, move, idleIc(), guard)});
			cases.push_back({"aim.swing/" + suffix, runAim(aim, move, swingIcOn(flat, aimAtDegrees(aim)), guard)});
			cases.push_back({"block/" + suffix, runBlockPrediction(aim, move, guard)});
		}
		const std::string aimName = std::to_string(static_cast<int>(aim));
		cases.push_back({"legacyArcs.idle/" + aimName, runLegacyGuardArcs(aim, DAttackState::Idle, guard)});
		cases.push_back({"legacyArcs.attacking/" + aimName, runLegacyGuardArcs(aim, DAttackState::Attacking, guard)});
	}
	return cases;
}

struct RecordedFlatHash
{
	const char* name;
	std::size_t calls;
	std::uint64_t hash;
};

constexpr RecordedFlatHash kRecordedFlatHashes[] = {
	{"aim.idle/0/none", 9u, 0x08d264ef47a59a76ull},
	{"aim.swing/0/none", 4u, 0xd7f5d179baaa1000ull},
	{"block/0/none", 4u, 0xb60dcaee7cfe7ad4ull},
	{"aim.idle/0/left", 10u, 0x79e640658ba256c6ull},
	{"aim.swing/0/left", 5u, 0x8c829a743c5fb429ull},
	{"block/0/left", 8u, 0xee69eba231183998ull},
	{"aim.idle/0/right", 10u, 0xf9381331d486ac96ull},
	{"aim.swing/0/right", 5u, 0x23c000e1055fa8ddull},
	{"block/0/right", 8u, 0xc818596f51b5092cull},
	{"legacyArcs.idle/0", 10u, 0x8dd2d1eb408f17d3ull},
	{"legacyArcs.attacking/0", 8u, 0x798cf1b54b62fff1ull},
	{"aim.idle/37/none", 9u, 0x4537bcd363fb4219ull},
	{"aim.swing/37/none", 4u, 0xb03f7baa02e364e6ull},
	{"block/37/none", 4u, 0xe27d8e29599f4acdull},
	{"aim.idle/37/left", 10u, 0x0d593c32854c9e52ull},
	{"aim.swing/37/left", 5u, 0x7301dcf43ea36419ull},
	{"block/37/left", 8u, 0xbbaf849fb9e61adcull},
	{"aim.idle/37/right", 10u, 0x933a742896657c7aull},
	{"aim.swing/37/right", 5u, 0x644e734a115bb28bull},
	{"block/37/right", 8u, 0x3ee21eeeeca7975aull},
	{"legacyArcs.idle/37", 10u, 0x0dc75643e55be7aeull},
	{"legacyArcs.attacking/37", 8u, 0x4d515a049d51c3c7ull},
	{"aim.idle/90/none", 9u, 0x9cb41d11c9ed8b05ull},
	{"aim.swing/90/none", 4u, 0x1517085100663bf3ull},
	{"block/90/none", 4u, 0xbae3855b88973359ull},
	{"aim.idle/90/left", 10u, 0xeedad4aa8290147bull},
	{"aim.swing/90/left", 5u, 0x1175ba4e03288fdfull},
	{"block/90/left", 8u, 0x41e604462c4d36f7ull},
	{"aim.idle/90/right", 10u, 0x6673625db6783655ull},
	{"aim.swing/90/right", 5u, 0x9d24cb24de24f83eull},
	{"block/90/right", 8u, 0xd4089f007302e4beull},
	{"legacyArcs.idle/90", 10u, 0x3cd27d42d9b9f54eull},
	{"legacyArcs.attacking/90", 8u, 0x0527450d5c667c9dull},
	{"aim.idle/200/none", 9u, 0xb1f9abc03a625b90ull},
	{"aim.swing/200/none", 4u, 0xa50c47c05f40a661ull},
	{"block/200/none", 4u, 0xc230a5db0a4786b3ull},
	{"aim.idle/200/left", 10u, 0x7553bf8f5e90887bull},
	{"aim.swing/200/left", 5u, 0x10d87ba1e77277e0ull},
	{"block/200/left", 8u, 0x13bb0e7142a38013ull},
	{"aim.idle/200/right", 10u, 0x0b8765e3b2d92093ull},
	{"aim.swing/200/right", 5u, 0x407fa57f32dd8864ull},
	{"block/200/right", 8u, 0x5ee248aad7564eadull},
	{"legacyArcs.idle/200", 10u, 0xb2defd8b7795b269ull},
	{"legacyArcs.attacking/200", 8u, 0x4a38f6b063ffcf19ull},
	{"aim.idle/300/none", 9u, 0xe1e10c9020a2191bull},
	{"aim.swing/300/none", 4u, 0xf8eb9ffc1f6c4045ull},
	{"block/300/none", 4u, 0xcfae89d353ba1407ull},
	{"aim.idle/300/left", 10u, 0x51b5797df362df7full},
	{"aim.swing/300/left", 5u, 0x8bb2a710b614acb3ull},
	{"block/300/left", 8u, 0x0cc9a9bc8c4adb02ull},
	{"aim.idle/300/right", 10u, 0xd8e5d0757497fd91ull},
	{"aim.swing/300/right", 5u, 0xc43f20ae9ba4844dull},
	{"block/300/right", 8u, 0x61a182037622a04eull},
	{"legacyArcs.idle/300", 10u, 0x572ccd5bebb49b4dull},
	{"legacyArcs.attacking/300", 8u, 0x766820599200b94dull},
};
}

TEST_CASE("GroundAlignedViz: the idle aim indicator lies in the guard body's plane on the 30-degree slope", "[DAttack]")
{
	using namespace groundAlignedVisualizationTests;
	PlaneCheck ground;
	PlaneCheck upright;
	int movesDrawn = 0;
	for (const float aim : kAims)
	{
		const dAttackGuardSimulation::State guard = guardStateOn(kThirtyDegreeNormal, aimAtDegrees(aim));
		const glm::vec3 up = guardUpOf(guard);
		REQUIRE(glm::length(up - kThirtyDegreeNormal) < 1e-5f);
		for (const MoveKind move : kMoves)
		{
			INFO("aim " << aim << " move " << nameOf(move));
			REQUIRE(dAttackDirection::classify(aimAtDegrees(aim), moveWorldFor(move, aim), stickFor(move)) == expectedSequenceFor(move));
			const Recording recording = runAim(aim, move, idleIc(), guard);
			REQUIRE(countColor(recording, 0u) > 0);
			movesDrawn += countColor(recording, 3u);
			checkAimIndicator(recording, move, up, aim, true, ground, upright);
		}
	}
	std::printf("[GroundAlignedViz.idleAim] ground points=%d worst=%.3g m, forward weapon points=%d worst=%.3g m, move lines=%d\n",
		ground.pointsChecked, ground.worstDistance, upright.pointsChecked, upright.worstDistance, movesDrawn);
	CHECK(ground.pointsChecked > 0);
	CHECK(upright.pointsChecked > 0);
	CHECK(movesDrawn == 10);
	CHECK(ground.nonFinite == 0);
	CHECK(ground.worstDistance <= kPlaneTolerance);
	CHECK(upright.nonFinite == 0);
	CHECK(upright.worstDistance <= kPlaneTolerance);
}

TEST_CASE("GroundAlignedViz: mid-swing the aim indicator lies in the swing frame's plane, not the guard's", "[DAttack]")
{
	using namespace groundAlignedVisualizationTests;
	PlaneCheck ground;
	PlaneCheck upright;
	for (const float aim : kAims)
	{
		const dAttackRadialSimulation::InitialConditions ic = swingIcOn(kThirtyDegreeNormal, aimAtDegrees(aim));
		const glm::vec3 up = swingUpOf(ic);
		REQUIRE(glm::length(up - kThirtyDegreeNormal) < 1e-5f);
		const dAttackGuardSimulation::State guard = guardStateOn(kTwentyDegreeNormal, aimAtDegrees(aim));
		REQUIRE(glm::length(guardUpOf(guard) - up) > 0.3f);
		for (const MoveKind move : kMoves)
		{
			INFO("aim " << aim << " move " << nameOf(move));
			const Recording recording = runAim(aim, move, ic, guard);
			REQUIRE(countColor(recording, 1u) > 0);
			CHECK(countColor(recording, 0u) == 0);
			checkAimIndicator(recording, move, up, aim, false, ground, upright);
		}
	}
	std::printf("[GroundAlignedViz.swingAim] points=%d worst=%.3g m\n", ground.pointsChecked, ground.worstDistance);
	CHECK(ground.pointsChecked > 0);
	CHECK(ground.nonFinite == 0);
	CHECK(ground.worstDistance <= kPlaneTolerance);
}

TEST_CASE("GroundAlignedViz: the block-prediction wedge and its blue block arc lie in the guard body's plane", "[DAttack]")
{
	using namespace groundAlignedVisualizationTests;
	PlaneCheck ground;
	int blueArcs = 0;
	for (const float aim : kAims)
	{
		const dAttackGuardSimulation::State guard = guardStateOn(kThirtyDegreeNormal, aimAtDegrees(aim));
		const glm::vec3 up = guardUpOf(guard);
		for (const MoveKind move : kMoves)
		{
			INFO("aim " << aim << " move " << nameOf(move));
			const Recording recording = runBlockPrediction(aim, move, guard);
			REQUIRE(countColor(recording, dAttackVisualizationUtils::kAttackDirectionColorId) > 0);
			blueArcs += countColor(recording, 2u);
			checkInPlane(recording, up, ground, anyCall);
		}
	}
	std::printf("[GroundAlignedViz.blockPrediction] points=%d worst=%.3g m blueArcs=%d\n", ground.pointsChecked,
		ground.worstDistance, blueArcs);
	CHECK(ground.pointsChecked > 0);
	CHECK(blueArcs == 5);
	CHECK(ground.nonFinite == 0);
	CHECK(ground.worstDistance <= kPlaneTolerance);
}

TEST_CASE("GroundAlignedViz: the legacy guard arcs lie in the guard body's plane, idle and attacking", "[DAttack]")
{
	using namespace groundAlignedVisualizationTests;
	PlaneCheck ground;
	for (const float aim : kAims)
	{
		const dAttackGuardSimulation::State guard = guardStateOn(kThirtyDegreeNormal, aimAtDegrees(aim));
		const glm::vec3 up = guardUpOf(guard);
		for (const DAttackState machineState : {DAttackState::Idle, DAttackState::Attacking})
		{
			INFO("aim " << aim << " attacking " << (machineState == DAttackState::Attacking));
			const Recording recording = runLegacyGuardArcs(aim, machineState, guard);
			REQUIRE(countColor(recording, 2u) > 0);
			checkInPlane(recording, up, ground, anyCall);
		}
	}
	std::printf("[GroundAlignedViz.legacyGuardArcs] points=%d worst=%.3g m\n", ground.pointsChecked, ground.worstDistance);
	CHECK(ground.pointsChecked > 0);
	CHECK(ground.nonFinite == 0);
	CHECK(ground.worstDistance <= kPlaneTolerance);
}

TEST_CASE("GroundAlignedViz: on flat ground every indicator's draw calls are bit-identical to the recorded ones", "[DAttack]")
{
	using namespace groundAlignedVisualizationTests;
	const std::vector<FlatCase> cases = flatCases();
	int matched = 0;
	for (const FlatCase& flatCase : cases)
	{
		const std::uint64_t hash = hashOf(flatCase.recording);
		std::printf("\t{\"%s\", %zuu, 0x%016llxull},\n", flatCase.name.c_str(), flatCase.recording.calls.size(),
			static_cast<unsigned long long>(hash));
		for (const RecordedFlatHash& recorded : kRecordedFlatHashes)
		{
			if (flatCase.name == recorded.name)
			{
				INFO(flatCase.name);
				CHECK(flatCase.recording.calls.size() == recorded.calls);
				CHECK(hash == recorded.hash);
				++matched;
			}
		}
	}
	CHECK(cases.size() == 55u);
	CHECK(matched == static_cast<int>(cases.size()));
}

#endif // WITH_LOW_LEVEL_TESTS
