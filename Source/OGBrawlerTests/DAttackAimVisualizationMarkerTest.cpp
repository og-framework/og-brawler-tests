// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackAimVisualization.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/DAttackVisualizationUtils.h"

#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"

#include <cmath>
#include <optional>
#include <vector>

namespace aimVizMarkerTest
{
using dAttackVisualizationUtils::aimRightMarkerEnd;
using dAttackVisualizationUtils::kAimRightMarkerOverhangCm;

constexpr float kInnerRadius = 50.f;
constexpr float kOuterRadius = 100.f;
constexpr float kAimArcOverhang = 0.1f;
constexpr float kTolerance = 1e-4f;

const glm::vec3 kRoot(120.f, -35.f, 42.f);

glm::vec3 aimAtDegrees(float degrees)
{
	const float radians = glm::radians(degrees);
	return glm::vec3(std::cos(radians), std::sin(radians), 0.f);
}

float crossZ(const glm::vec3& a, const glm::vec3& b)
{
	return a.x * b.y - a.y * b.x;
}

float signedAngleAboutUp(const glm::vec3& from, const glm::vec3& to)
{
	return std::atan2(crossZ(from, to), from.x * to.x + from.y * to.y);
}

struct RecordedLine
{
	glm::vec3 start;
	glm::vec3 end;
	unsigned int colorId;
	float thickness;
};

struct RecordedArc
{
	glm::vec3 center;
	glm::vec3 direction;
	float radius;
	float halfAngle;
	unsigned int colorId;
	float thickness;
};

struct Recording
{
	std::vector<RecordedLine> lines;
	std::vector<RecordedArc> arcs;
};

struct RecordingRenderer
{
	Recording* recording = nullptr;

	void drawLine(const glm::vec3& start, const glm::vec3& end, unsigned int colorId, float thickness)
	{
		recording->lines.push_back({start, end, colorId, thickness});
	}
	void drawPoint(const glm::vec3&) {}
	void drawSphere(const glm::vec3&, float, unsigned int, float) {}
	void drawCircleArc(const glm::vec3& center, const glm::vec3& direction, float radius, float halfAngle,
		unsigned int colorId, float thickness)
	{
		recording->arcs.push_back({center, direction, radius, halfAngle, colorId, thickness});
	}
	void drawMesh(const std::vector<glm::vec3>&, const std::vector<unsigned int>&, unsigned int, unsigned int = 150) {}
	void drawTriangle(const glm::vec3&, const glm::vec3&, const glm::vec3&, unsigned int) {}
};

struct NullLogger
{
	void logVec3(const char*, const glm::vec3&) {}
	void logInt(const char*, int) {}
};

Recording visualizeAim(const glm::vec3& aimDirection)
{
	std::vector<DAttackRadialSequence> sequences;
	DAttackCircle circle{8u, kInnerRadius, kOuterRadius, 10.f, false, 1.f};
	dAttackRadialSimulation::StaticData staticData{sequences, circle};
	dAttackRadialSimulation::State simState;
	dAttackRadialSimulation::DerivedState derived;
	dAttackRadialSimulation::InitialConditions ic;
	ic.activeAttackSequence = kHadoukenSequenceSentinel;
	simState.bodyState.position = kRoot;
	simState.bodyState.rotation = glm::quat(1.f, 0.f, 0.f, 0.f);

	Recording recording;
	RecordingRenderer renderer{&recording};
	dAttackAimVisualization::State vizState;
	dAttackAimVisualization::Input<RecordingRenderer, NullLogger> input(
		1.f / 60.f, aimDirection, renderer, NullLogger{}, glm::vec2(0.f), glm::vec3(0.f));
	dAttackAimVisualization::visualize(input, simState, ic, derived, staticData, vizState);
	return recording;
}

constexpr unsigned int kReferenceColorId = 1u;

std::vector<RecordedLine> referenceLines(const Recording& recording)
{
	std::vector<RecordedLine> reference;
	for (const RecordedLine& line : recording.lines)
	{
		if (line.colorId == kReferenceColorId)
		{
			reference.push_back(line);
		}
	}
	return reference;
}

std::vector<RecordedLine> referenceLinesFromRoot(const Recording& recording)
{
	std::vector<RecordedLine> fromRoot;
	for (const RecordedLine& line : referenceLines(recording))
	{
		if (line.start.x == kRoot.x && line.start.y == kRoot.y && line.start.z == kRoot.z)
		{
			fromRoot.push_back(line);
		}
	}
	return fromRoot;
}

std::vector<RecordedArc> innerArcs(const Recording& recording)
{
	std::vector<RecordedArc> inner;
	for (const RecordedArc& arc : recording.arcs)
	{
		if (arc.radius == kInnerRadius)
		{
			inner.push_back(arc);
		}
	}
	return inner;
}
}

TEST_CASE("AimVizMarker: aim +X puts the marker on +Y, Unreal's screen-right", "[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	const std::optional<glm::vec3> markerEnd = aimRightMarkerEnd(kRoot, glm::vec3(1.f, 0.f, 0.f), kInnerRadius);
	REQUIRE(markerEnd.has_value());
	CHECK(markerEnd->x == Catch::Approx(kRoot.x).margin(kTolerance));
	CHECK(markerEnd->y == Catch::Approx(kRoot.y + kInnerRadius + 10.f).margin(kTolerance));
	CHECK(markerEnd->z == kRoot.z);
	CHECK(kAimRightMarkerOverhangCm == 10.f);
}

TEST_CASE("AimVizMarker: a rotated aim gives a perpendicular marker of length innerRadius + 10 on the +pi/2 side",
	"[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	int aimsChecked = 0;
	for (const float degrees : {45.f, 200.f, 0.f, 90.f, 135.f, 270.f, 315.f})
	{
		INFO("aim at " << degrees << " degrees");
		const glm::vec3 aim = aimAtDegrees(degrees);
		const std::optional<glm::vec3> markerEnd = aimRightMarkerEnd(kRoot, aim, kInnerRadius);
		REQUIRE(markerEnd.has_value());
		const glm::vec3 marker = *markerEnd - kRoot;
		CHECK(glm::dot(aim, marker) == Catch::Approx(0.f).margin(kTolerance));
		CHECK(glm::length(marker) == Catch::Approx(kInnerRadius + 10.f).margin(kTolerance));
		CHECK(crossZ(aim, marker) > 0.f);
		CHECK(signedAngleAboutUp(aim, marker) == Catch::Approx(glm::pi<float>() / 2.f).margin(kTolerance));
		++aimsChecked;
	}
	CHECK(aimsChecked == 7);
}

TEST_CASE("AimVizMarker: an aim with a downward z still gives a horizontal marker", "[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	const glm::vec3 aim = glm::normalize(glm::vec3(0.3f, -0.5f, -0.8f));
	const std::optional<glm::vec3> markerEnd = aimRightMarkerEnd(kRoot, aim, kInnerRadius);
	REQUIRE(markerEnd.has_value());
	const glm::vec3 marker = *markerEnd - kRoot;
	const glm::vec3 aimXY = glm::normalize(glm::vec3(aim.x, aim.y, 0.f));
	CHECK(markerEnd->z == kRoot.z);
	CHECK(glm::length(marker) == Catch::Approx(kInnerRadius + 10.f).margin(kTolerance));
	CHECK(glm::dot(aimXY, marker) == Catch::Approx(0.f).margin(kTolerance));
	CHECK(crossZ(aimXY, marker) > 0.f);
}

TEST_CASE("AimVizMarker: an aim straight down or up gives no marker", "[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	CHECK_FALSE(aimRightMarkerEnd(kRoot, glm::vec3(0.f, 0.f, -1.f), kInnerRadius).has_value());
	CHECK_FALSE(aimRightMarkerEnd(kRoot, glm::vec3(0.f, 0.f, 1.f), kInnerRadius).has_value());
}

TEST_CASE("AimVizMarker: visualize draws the marker from the root in the reference-line colour and thickness",
	"[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	const Recording recording = visualizeAim(glm::vec3(1.f, 0.f, 0.f));
	const std::vector<RecordedLine> fromRoot = referenceLinesFromRoot(recording);
	REQUIRE(fromRoot.size() == 1);
	CHECK(fromRoot[0].end.x == Catch::Approx(kRoot.x).margin(kTolerance));
	CHECK(fromRoot[0].end.y == Catch::Approx(kRoot.y + kInnerRadius + 10.f).margin(kTolerance));
	CHECK(fromRoot[0].end.z == kRoot.z);
	CHECK(fromRoot[0].colorId == 1u);
	CHECK(fromRoot[0].thickness == 1.f);
}

TEST_CASE("AimVizMarker: visualize draws no marker for a vertical aim", "[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	const Recording verticalRecording = visualizeAim(glm::vec3(0.f, 0.f, -1.f));
	const Recording horizontalRecording = visualizeAim(glm::vec3(1.f, 0.f, 0.f));
	CHECK(referenceLinesFromRoot(verticalRecording).empty());
	CHECK(referenceLinesFromRoot(horizontalRecording).size() == 1);
	CHECK(referenceLines(verticalRecording).size() + 1 == referenceLines(horizontalRecording).size());
}

TEST_CASE("AimVizMarker: the inner arc spans -0.1 rad to pi/2 + 0.1 rad on the marker's side",
	"[DAttack][AimVizMarker]")
{
	using namespace aimVizMarkerTest;
	int aimsChecked = 0;
	for (const float degrees : {0.f, 45.f, 200.f})
	{
		INFO("aim at " << degrees << " degrees");
		const glm::vec3 aim = aimAtDegrees(degrees);
		const Recording recording = visualizeAim(aim);
		const std::vector<RecordedArc> inner = innerArcs(recording);
		REQUIRE(inner.size() == 1);

		const float halfAngle = inner[0].halfAngle;
		const float centerOffset = signedAngleAboutUp(aim, inner[0].direction);
		CHECK(halfAngle == Catch::Approx(glm::pi<float>() / 4.f + kAimArcOverhang).margin(kTolerance));
		CHECK(centerOffset == Catch::Approx(glm::pi<float>() / 4.f).margin(kTolerance));
		CHECK(centerOffset - halfAngle == Catch::Approx(-kAimArcOverhang).margin(kTolerance));
		CHECK(centerOffset + halfAngle == Catch::Approx(glm::pi<float>() / 2.f + kAimArcOverhang).margin(kTolerance));
		CHECK(inner[0].colorId == 1u);
		CHECK(inner[0].thickness == 1.f);

		const std::optional<glm::vec3> markerEnd = aimRightMarkerEnd(kRoot, aim, kInnerRadius);
		REQUIRE(markerEnd.has_value());
		CHECK(crossZ(inner[0].direction, *markerEnd - kRoot) > 0.f);
		CHECK(signedAngleAboutUp(aim, *markerEnd - kRoot) < centerOffset + halfAngle);
		++aimsChecked;
	}
	CHECK(aimsChecked == 3);
}

#endif // WITH_LOW_LEVEL_TESTS
