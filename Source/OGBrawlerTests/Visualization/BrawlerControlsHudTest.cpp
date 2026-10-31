// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerControlsHud.h"
#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/ControlModeTexts.h"
#include "OGBrawler/InputMapping/GameInputMapping.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace brawlerControlsHudTest
{
using brawlerControlsHud::CameraView;
using brawlerControlsHud::ControlsHudInk;
using brawlerControlsHud::ControlsHudInput;
using brawlerControlsHud::ControlsHudLayout;
using brawlerControlsHud::IndicatorRow;
using brawlerControlsHud::RowInk;
using brawlerControlsHud::bindingText;
using brawlerControlsHud::indicatorRows;
using brawlerControlsHud::legendLines;
using dAttackMachineSimulation::MovementScheme;
using dInput::ActionDescriptor;
using dInput::ActionMapping;
using dInput::KeyId;
using dInput::KeyModifier;
using dInput::MappingContext;

constexpr std::size_t kMaxDrawnStringLength = 64;

constexpr std::array<MovementScheme, 4> kAllSchemes = {
	MovementScheme::CameraRelative,
	MovementScheme::AimRelative,
	MovementScheme::MoveRelativeAim,
	MovementScheme::AimRelativeSwapped,
};

const char* inkName(RowInk ink)
{
	switch (ink)
	{
	case RowInk::Active: return "Active";
	case RowInk::Normal: return "Normal";
	case RowInk::Dim: return "Dim";
	}
	return "?";
}

ControlsHudInput defaultInput()
{
	ControlsHudInput input;
	input.scheme = MovementScheme::AimRelative;
	input.view = CameraView::Solo;
	input.cameraLookOn = false;
	input.legendOpen = false;
	input.legacyStickSwap = false;
	return input;
}

ControlsHudInput openLegend(ControlsHudInput input)
{
	input.legendOpen = true;
	return input;
}

ActionMapping& mappingOf(MappingContext& ctx, const ActionDescriptor& action)
{
	for (ActionMapping& mapping : ctx.actionMappings)
		if (mapping.action == &action)
			return mapping;
	FAIL("action not mapped: " << action.name);
	return ctx.actionMappings.front();
}

std::size_t countOf(const std::vector<std::string>& lines, std::string_view text)
{
	return static_cast<std::size_t>(std::count(lines.begin(), lines.end(), std::string(text)));
}

bool isSevenBitAscii(std::string_view text)
{
	return std::all_of(text.begin(), text.end(),
		[](char c) { return static_cast<unsigned char>(c) < 0x80u; });
}

bool containsDigit(std::string_view text)
{
	return std::any_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; });
}

void checkRow(const IndicatorRow& row, const char* keys, const char* label, const char* state, RowInk ink)
{
	CAPTURE(row.keys, row.label, row.state, inkName(row.ink));
	CHECK(row.keys == keys);
	CHECK(row.label == label);
	CHECK(row.state == state);
	CHECK(inkName(row.ink) == std::string(inkName(ink)));
}

bool sameInk(const ControlsHudInk& a, const ControlsHudInk& b)
{
	return a.r == b.r && a.g == b.g && a.b == b.b;
}

TEST_CASE("ControlsHud: the default mapping, solo, Aim-relative, look off and help expanded gives the six shipped rows",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	const std::vector<IndicatorRow> rows = indicatorRows(ctx, openLegend(defaultInput()));
	REQUIRE(rows.size() == 6u);
	checkRow(rows[0], "5 / Pad Y", "Camera look", "OFF", RowInk::Normal);
	checkRow(rows[1], "7", "Camera-relative", "", RowInk::Dim);
	checkRow(rows[2], "8", "Aim-relative", "<", RowInk::Active);
	checkRow(rows[3], "9", "Aim follows move", "", RowInk::Dim);
	checkRow(rows[4], "0", "Aim-relative, swapped sticks", "", RowInk::Dim);
	checkRow(rows[5], "H / View", "Controls help", "", RowInk::Normal);
}

TEST_CASE("ControlsHud: collapsed by default, the six rows keep their keys and inks and have no label and no state",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	const std::vector<IndicatorRow> rows = indicatorRows(ctx, defaultInput());
	REQUIRE(rows.size() == 6u);
	checkRow(rows[0], "5 / Pad Y", "", "", RowInk::Normal);
	checkRow(rows[1], "7", "", "", RowInk::Dim);
	checkRow(rows[2], "8", "", "", RowInk::Active);
	checkRow(rows[3], "9", "", "", RowInk::Dim);
	checkRow(rows[4], "0", "", "", RowInk::Dim);
	checkRow(rows[5], "H / View", "", "", RowInk::Normal);

	for (const MovementScheme scheme : kAllSchemes)
		for (const CameraView view : { CameraView::Solo, CameraView::Shared })
			for (const bool lookOn : { false, true })
				for (const bool swap : { false, true })
				{
					CAPTURE(static_cast<int>(scheme), static_cast<int>(view), lookOn, swap);
					const ControlsHudInput collapsed{ scheme, view, lookOn, false, swap };
					const std::vector<IndicatorRow> collapsedRows = indicatorRows(ctx, collapsed);
					const std::vector<IndicatorRow> expandedRows = indicatorRows(ctx, openLegend(collapsed));
					REQUIRE(collapsedRows.size() == expandedRows.size());
					for (std::size_t index = 0; index < collapsedRows.size(); ++index)
					{
						CAPTURE(index, collapsedRows[index].label, collapsedRows[index].state);
						CHECK(collapsedRows[index].keys == expandedRows[index].keys);
						CHECK(collapsedRows[index].ink == expandedRows[index].ink);
						CHECK(collapsedRows[index].label.empty());
						CHECK(collapsedRows[index].state.empty());
						CHECK_FALSE(expandedRows[index].label.empty());
					}
				}
}

TEST_CASE("ControlsHud: bindingText lists keyboard and mouse keys first, then gamepad keys, each in binding order, each key once",
	"[CharacterViz][ControlsHud]")
{
	const ActionDescriptor probe{ "ControlsHudProbe", dInput::ActionValueType::Boolean };
	const ActionDescriptor unbound{ "ControlsHudUnbound", dInput::ActionValueType::Boolean };
	const ActionDescriptor other{ "ControlsHudOther", dInput::ActionValueType::Boolean };

	MappingContext ctx{ "probe", {} };
	ctx.actionMappings.push_back({ &probe, { { KeyId::Gamepad_FaceBottom, KeyModifier::None },
		{ KeyId::Key_W, KeyModifier::None }, { KeyId::Gamepad_FaceBottom, KeyModifier::None } } });
	ctx.actionMappings.push_back({ &other, { { KeyId::Key_Q, KeyModifier::None } } });
	ctx.actionMappings.push_back({ &probe, { { KeyId::Key_W, KeyModifier::Negate },
		{ KeyId::Mouse_Left, KeyModifier::None }, { KeyId::Gamepad_LeftShoulder, KeyModifier::None } } });

	CHECK(bindingText(ctx, probe) == "W / LMB / Pad A / LB");
	CHECK(bindingText(ctx, other) == "Q");
	CHECK(bindingText(ctx, unbound).empty());
}

TEST_CASE("ControlsHud: the shared camera drops the camera-look row and the legend's camera-look section and shows the shared-camera line",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	const dInput::controlModeTexts::ControlModeEntry* cameraLook =
		dInput::controlModeTexts::findControlModeEntry(&dInput::gameMapping::BlockLook);
	REQUIRE(cameraLook != nullptr);

	for (const bool lookOn : { false, true })
	{
		CAPTURE(lookOn);
		ControlsHudInput shared = defaultInput();
		shared.view = CameraView::Shared;
		shared.cameraLookOn = lookOn;

		for (const bool expanded : { false, true })
		{
			CAPTURE(expanded);
			ControlsHudInput mode = shared;
			mode.legendOpen = expanded;
			const std::vector<IndicatorRow> rows = indicatorRows(ctx, mode);
			REQUIRE(rows.size() == 5u);
			for (const IndicatorRow& row : rows)
			{
				CAPTURE(row.keys, row.label);
				CHECK(row.label != std::string(cameraLook->shortLabel));
				CHECK(row.keys != "5 / Pad Y");
			}
			CHECK(rows.front().keys == "7");
			CHECK(rows.front().label == (expanded ? "Camera-relative" : ""));
		}

		const std::vector<std::string> sharedLegend = legendLines(ctx, openLegend(shared));
		CHECK(countOf(sharedLegend, brawlerControlsHud::kSharedCameraLine) == 1u);
		CHECK(countOf(sharedLegend, "5 / Pad Y Camera look (solo camera)") == 0u);
		for (const std::string_view line : cameraLook->lines)
			CHECK(countOf(sharedLegend, line) == 0u);
		for (const std::string_view line : cameraLook->linesWhenOn)
			CHECK(countOf(sharedLegend, line) == 0u);

		ControlsHudInput solo = shared;
		solo.view = CameraView::Solo;
		const std::vector<std::string> soloLegend = legendLines(ctx, openLegend(solo));
		CHECK(countOf(soloLegend, brawlerControlsHud::kSharedCameraLine) == 0u);
		CHECK(countOf(soloLegend, "5 / Pad Y Camera look (solo camera)") == 1u);
		CHECK(indicatorRows(ctx, solo).size() == 6u);
	}
}

TEST_CASE("ControlsHud: each of the four schemes makes exactly its own row Active, collapsed or expanded, and heads the legend",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	const std::array<const char*, 4> expectedKeys = { "7", "8", "9", "0" };
	const std::array<const char*, 4> expectedLabels = {
		"Camera-relative", "Aim-relative", "Aim follows move", "Aim-relative, swapped sticks" };
	const std::array<const char*, 4> expectedHeadings = {
		"7 Camera-relative", "8 Aim-relative", "9 Aim follows move", "0 Aim-relative, swapped sticks" };

	for (std::size_t index = 0; index < kAllSchemes.size(); ++index)
		for (const bool expanded : { false, true })
		{
			CAPTURE(index, expanded);
			ControlsHudInput input = defaultInput();
			input.scheme = kAllSchemes[index];
			input.legendOpen = expanded;

			const std::vector<IndicatorRow> rows = indicatorRows(ctx, input);
			REQUIRE(rows.size() == 6u);
			std::size_t activeCount = 0;
			for (std::size_t rowIndex = 1; rowIndex <= 4; ++rowIndex)
			{
				const IndicatorRow& row = rows[rowIndex];
				CAPTURE(rowIndex, row.keys, row.label, row.state, inkName(row.ink));
				CHECK(row.keys == expectedKeys[rowIndex - 1]);
				CHECK(row.label == (expanded ? expectedLabels[rowIndex - 1] : ""));
				const bool expectActive = rowIndex - 1 == index;
				if (row.ink == RowInk::Active)
					++activeCount;
				CHECK((row.ink == RowInk::Active) == expectActive);
				CHECK((row.ink == RowInk::Dim) == !expectActive);
				CHECK(row.state == (expanded && expectActive ? "<" : ""));
			}
			CHECK(activeCount == 1u);
			CHECK(rows[0].ink == RowInk::Normal);
			CHECK(rows[5].ink == RowInk::Normal);

			const std::vector<std::string> legend = legendLines(ctx, openLegend(input));
			REQUIRE_FALSE(legend.empty());
			CHECK(legend.front() == expectedHeadings[index]);
		}
}

TEST_CASE("ControlsHud: camera look on and off flips the camera-look row's state, its ink and the legend body",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	const std::vector<std::string> expectedOff = {
		"5 / Pad Y Camera look (solo camera)",
		"OFF: the camera holds still and the mouse cursor aims.",
		"H / View: close this help",
	};
	const std::vector<std::string> expectedOn = {
		"5 / Pad Y Camera look (solo camera)",
		"ON: the cursor hides; mouse and aim stick turn the camera.",
		"In Aim-relative you aim where the camera faces.",
		"H / View: close this help",
	};

	for (const bool lookOn : { false, true })
	{
		CAPTURE(lookOn);
		ControlsHudInput input = defaultInput();
		input.cameraLookOn = lookOn;

		const RowInk lookInk = lookOn ? RowInk::Active : RowInk::Normal;
		const std::vector<IndicatorRow> collapsedRows = indicatorRows(ctx, input);
		REQUIRE(collapsedRows.size() == 6u);
		checkRow(collapsedRows[0], "5 / Pad Y", "", "", lookInk);
		const std::vector<IndicatorRow> rows = indicatorRows(ctx, openLegend(input));
		REQUIRE(rows.size() == 6u);
		checkRow(rows[0], "5 / Pad Y", "Camera look", lookOn ? "ON" : "OFF", lookInk);

		const std::vector<std::string> legend = legendLines(ctx, openLegend(input));
		const auto blank = std::find(legend.begin(), legend.end(), std::string());
		REQUIRE(blank != legend.end());
		const std::vector<std::string> body(blank + 1, legend.end());
		CHECK(body == (lookOn ? expectedOn : expectedOff));
	}
}

TEST_CASE("ControlsHud: a rebinding reaches the rows, collapsed or expanded, and the legend, so the HUD has no second key list",
	"[CharacterViz][ControlsHud]")
{
	MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	mappingOf(ctx, dInput::gameMapping::SetSchemeAimRelative).bindings = { { KeyId::Key_P, KeyModifier::None } };
	mappingOf(ctx, dInput::gameMapping::BlockLook).bindings.push_back({ KeyId::Mouse_Middle, KeyModifier::None });

	const std::vector<IndicatorRow> collapsedRows = indicatorRows(ctx, defaultInput());
	REQUIRE(collapsedRows.size() == 6u);
	checkRow(collapsedRows[0], "5 / MMB / Pad Y", "", "", RowInk::Normal);
	checkRow(collapsedRows[2], "P", "", "", RowInk::Active);

	const std::vector<IndicatorRow> rows = indicatorRows(ctx, openLegend(defaultInput()));
	REQUIRE(rows.size() == 6u);
	checkRow(rows[0], "5 / MMB / Pad Y", "Camera look", "OFF", RowInk::Normal);
	checkRow(rows[2], "P", "Aim-relative", "<", RowInk::Active);

	const std::vector<std::string> legend = legendLines(ctx, openLegend(defaultInput()));
	REQUIRE_FALSE(legend.empty());
	CHECK(legend.front() == "P Aim-relative");
	CHECK(countOf(legend, "5 / MMB / Pad Y Camera look (solo camera)") == 1u);
}

TEST_CASE("ControlsHud: a closed legend has no lines; an open one starts with the active scheme and ends with the close line",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	for (const MovementScheme scheme : kAllSchemes)
		for (const CameraView view : { CameraView::Solo, CameraView::Shared })
			for (const bool lookOn : { false, true })
				for (const bool swap : { false, true })
				{
					CAPTURE(static_cast<int>(scheme), static_cast<int>(view), lookOn, swap);
					ControlsHudInput input{ scheme, view, lookOn, false, swap };
					CHECK(legendLines(ctx, input).empty());
					input.legendOpen = true;
					const std::vector<std::string> legend = legendLines(ctx, input);
					REQUIRE(legend.size() >= 3u);
					CHECK(legend.back() == "H / View: close this help");
				}

	const std::vector<std::string> expected = {
		"8 Aim-relative",
		"Aim: the mouse cursor, or the right stick (camera-relative).",
		"Move (movement keys / left stick): relative to your aim;",
		"  up walks toward it, left and right strafe around it.",
		"Gamepad: right stick released, the left stick moves and aims.",
		"",
		"5 / Pad Y Camera look (solo camera)",
		"OFF: the camera holds still and the mouse cursor aims.",
		"H / View: close this help",
	};
	CHECK(legendLines(ctx, openLegend(defaultInput())) == expected);
}

TEST_CASE("ControlsHud: the legacy stick-swap note appears only with the console swap on and a scheme that is not the swapped one",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	for (const MovementScheme scheme : kAllSchemes)
		for (const CameraView view : { CameraView::Solo, CameraView::Shared })
			for (const bool swap : { false, true })
			{
				CAPTURE(static_cast<int>(scheme), static_cast<int>(view), swap);
				const ControlsHudInput input{ scheme, view, false, true, swap };
				const std::vector<std::string> legend = legendLines(ctx, input);
				const bool expectNote = swap && scheme != MovementScheme::AimRelativeSwapped;
				CHECK(countOf(legend, brawlerControlsHud::kLegacyStickSwapNote) == (expectNote ? 1u : 0u));
				if (expectNote)
				{
					REQUIRE(legend.size() >= 2u);
					CHECK(legend[legend.size() - 2] == std::string(brawlerControlsHud::kLegacyStickSwapNote));
				}
			}
}

TEST_CASE("ControlsHud: scale 2 doubles every length and the text scale and keeps the origin",
	"[CharacterViz][ControlsHud]")
{
	ControlsHudLayout base;
	CHECK(base.originX == 12.f);
	CHECK(base.originY == 12.f);
	CHECK(base.rowHeight == 16.f);
	CHECK(base.keysX == 6.f);
	CHECK(base.labelX == 92.f);
	CHECK(base.stateX == 300.f);
	CHECK(base.panelWidth == 330.f);
	CHECK(base.compactPanelWidth == 80.f);
	CHECK(base.legendGapY == 8.f);
	CHECK(base.legendLineHeight == 15.f);
	CHECK(base.legendWidth == 470.f);
	CHECK(base.padding == 4.f);
	CHECK(base.textScale == 1.f);

	base.originX = 30.f;
	base.originY = 40.f;
	const ControlsHudLayout scaled = brawlerControlsHud::scaledControlsHudLayout(base, 2.f);
	CHECK(scaled.originX == 30.f);
	CHECK(scaled.originY == 40.f);
	CHECK(scaled.rowHeight == 2.f * base.rowHeight);
	CHECK(scaled.keysX == 2.f * base.keysX);
	CHECK(scaled.labelX == 2.f * base.labelX);
	CHECK(scaled.stateX == 2.f * base.stateX);
	CHECK(scaled.panelWidth == 2.f * base.panelWidth);
	CHECK(scaled.compactPanelWidth == 2.f * base.compactPanelWidth);
	CHECK(scaled.legendGapY == 2.f * base.legendGapY);
	CHECK(scaled.legendLineHeight == 2.f * base.legendLineHeight);
	CHECK(scaled.legendWidth == 2.f * base.legendWidth);
	CHECK(scaled.padding == 2.f * base.padding);
	CHECK(scaled.textScale == 2.f * base.textScale);

	using namespace brawlerControlsHud;
	CHECK(indicatorKeysX(scaled) - scaled.originX == 2.f * (indicatorKeysX(base) - base.originX));
	CHECK(indicatorLabelX(scaled) - scaled.originX == 2.f * (indicatorLabelX(base) - base.originX));
	CHECK(indicatorStateX(scaled) - scaled.originX == 2.f * (indicatorStateX(base) - base.originX));
	CHECK(legendTextX(scaled) - scaled.originX == 2.f * (legendTextX(base) - base.originX));
	CHECK(indicatorRowTopY(scaled, 3) - scaled.originY == 2.f * (indicatorRowTopY(base, 3) - base.originY));
	CHECK(indicatorHeight(scaled, 6) == 2.f * indicatorHeight(base, 6));
	CHECK(legendTopY(scaled, 6) - scaled.originY == 2.f * (legendTopY(base, 6) - base.originY));
	CHECK(legendHeight(scaled, 9) == 2.f * legendHeight(base, 9));
	CHECK(legendLineTopY(scaled, 6, 2) - scaled.originY == 2.f * (legendLineTopY(base, 6, 2) - base.originY));
}

TEST_CASE("ControlsHud: the indicator panel is the compact width collapsed and the full width expanded, at every scale",
	"[CharacterViz][ControlsHud]")
{
	using namespace brawlerControlsHud;
	const ControlsHudLayout base;
	CHECK(indicatorPanelWidth(base, false) == 80.f);
	CHECK(indicatorPanelWidth(base, true) == 330.f);

	const ControlsHudLayout doubled = scaledControlsHudLayout(base, 2.f);
	CHECK(indicatorPanelWidth(doubled, false) == 160.f);
	CHECK(indicatorPanelWidth(doubled, true) == 660.f);

	CHECK(base.keysX < base.compactPanelWidth);
	CHECK(base.compactPanelWidth < base.panelWidth);
}

TEST_CASE("ControlsHud: the scale and backdrop clamps pull out-of-range values and NaN to their stated range",
	"[CharacterViz][ControlsHud]")
{
	using namespace brawlerControlsHud;
	CHECK(kControlsHudDefaultScale == 1.f);
	CHECK(kControlsHudMinScale == 0.25f);
	CHECK(kControlsHudMaxScale == 4.f);
	CHECK(kControlsHudDefaultBackgroundAlpha == 0.4f);
	CHECK(kControlsHudMinBackgroundAlpha == 0.f);
	CHECK(kControlsHudMaxBackgroundAlpha == 1.f);

	volatile float nanSource = std::numeric_limits<float>::quiet_NaN();
	const float notANumber = nanSource;
	volatile float infinitySource = std::numeric_limits<float>::infinity();
	const float infinity = infinitySource;

	CHECK(clampControlsHudScale(0.1f) == 0.25f);
	CHECK(clampControlsHudScale(9.f) == 4.f);
	CHECK(clampControlsHudScale(notANumber) == kControlsHudMinScale);
	CHECK(clampControlsHudScale(-notANumber) == kControlsHudMinScale);
	CHECK(clampControlsHudScale(infinity) == kControlsHudMaxScale);
	CHECK(clampControlsHudScale(-infinity) == kControlsHudMinScale);
	CHECK(clampControlsHudScale(0.25f) == 0.25f);
	CHECK(clampControlsHudScale(2.5f) == 2.5f);
	CHECK(clampControlsHudScale(4.f) == 4.f);

	CHECK(clampControlsHudBackgroundAlpha(-0.5f) == 0.f);
	CHECK(clampControlsHudBackgroundAlpha(9.f) == 1.f);
	CHECK(clampControlsHudBackgroundAlpha(notANumber) == kControlsHudMinBackgroundAlpha);
	CHECK(clampControlsHudBackgroundAlpha(-notANumber) == kControlsHudMinBackgroundAlpha);
	CHECK(clampControlsHudBackgroundAlpha(infinity) == kControlsHudMaxBackgroundAlpha);
	CHECK(clampControlsHudBackgroundAlpha(0.f) == 0.f);
	CHECK(clampControlsHudBackgroundAlpha(0.4f) == 0.4f);
	CHECK(clampControlsHudBackgroundAlpha(1.f) == 1.f);
}

TEST_CASE("ControlsHud: the legend sits below the indicator's last row plus the row height plus the gap, at every scale",
	"[CharacterViz][ControlsHud]")
{
	using namespace brawlerControlsHud;
	for (const float scale : { kControlsHudMinScale, kControlsHudDefaultScale, 2.f, kControlsHudMaxScale })
		for (const std::size_t rowCount : { std::size_t{ 5 }, std::size_t{ 6 } })
		{
			CAPTURE(scale, rowCount);
			const ControlsHudLayout layout = scaledControlsHudLayout(ControlsHudLayout{}, scale);
			const float lastRowBottom = indicatorRowTopY(layout, rowCount - 1) + layout.rowHeight;
			CHECK(legendTopY(layout, rowCount) >= lastRowBottom + layout.legendGapY);
			CHECK(legendTopY(layout, rowCount) == layout.originY + indicatorHeight(layout, rowCount) + layout.legendGapY);
			CHECK(indicatorRowTopY(layout, 0) == layout.originY + layout.padding);
			CHECK(legendLineTopY(layout, rowCount, 0) == legendTopY(layout, rowCount) + layout.padding);
			CHECK(legendHeight(layout, 9) == 2.f * layout.padding + 9.f * layout.legendLineHeight);
		}

	const ControlsHudLayout layout;
	CHECK(layout.keysX < layout.labelX);
	CHECK(layout.labelX < layout.stateX);
	CHECK(layout.stateX < layout.panelWidth);
	CHECK(layout.padding <= layout.keysX);
}

TEST_CASE("ControlsHud: the inks, and every drawn line is 7-bit ASCII and at most 64 characters with no digit outside the key lists",
	"[CharacterViz][ControlsHud]")
{
	using namespace brawlerControlsHud;
	CHECK(sameInk(controlsHudRowInk(RowInk::Active), ControlsHudInk{ 1.0f, 0.85f, 0.2f }));
	CHECK(sameInk(controlsHudRowInk(RowInk::Normal), ControlsHudInk{ 0.9f, 0.9f, 0.9f }));
	CHECK(sameInk(controlsHudRowInk(RowInk::Dim), ControlsHudInk{ 0.55f, 0.55f, 0.55f }));
	CHECK(sameInk(kControlsHudBackdropInk, ControlsHudInk{ 0.f, 0.f, 0.f }));

	for (const std::string_view text : { kBindingSeparator, kKeysTitleSeparator, kCameraLookOnState, kCameraLookOffState,
			 kActiveSchemeState, kSharedCameraLine, kLegacyStickSwapNote, kCloseHelpSuffix })
	{
		CAPTURE(std::string(text));
		CHECK_FALSE(text.empty());
		CHECK(isSevenBitAscii(text));
		CHECK(text.size() <= kMaxDrawnStringLength);
		CHECK_FALSE(containsDigit(text));
	}

	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	for (const MovementScheme scheme : kAllSchemes)
		for (const CameraView view : { CameraView::Solo, CameraView::Shared })
			for (const bool lookOn : { false, true })
				for (const bool swap : { false, true })
				{
					const ControlsHudInput input{ scheme, view, lookOn, true, swap };
					for (const std::string& line : legendLines(ctx, input))
					{
						CAPTURE(static_cast<int>(scheme), static_cast<int>(view), lookOn, swap, line);
						CHECK(isSevenBitAscii(line));
						CHECK(line.size() <= kMaxDrawnStringLength);
					}
					for (const IndicatorRow& row : indicatorRows(ctx, input))
					{
						CAPTURE(row.keys, row.label, row.state);
						CHECK(isSevenBitAscii(row.keys + row.label + row.state));
						CHECK_FALSE(containsDigit(row.label + row.state));
					}
				}
}

} // namespace brawlerControlsHudTest

#endif // WITH_LOW_LEVEL_TESTS
