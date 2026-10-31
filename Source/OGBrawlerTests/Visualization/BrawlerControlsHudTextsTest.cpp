// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/ControlModeTexts.h"
#include "OGBrawler/InputMapping/GameInputMapping.h"
#include "OGBrawler/InputMapping/KeyGlyphs.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace brawlerControlsHudTextsTest
{
using dAttackMachineSimulation::MovementScheme;
using dInput::ActionDescriptor;
using dInput::KeyId;
using dInput::MappingContext;
using dInput::controlModeTexts::ControlModeEntry;
using dInput::controlModeTexts::ControlModeKind;
using dInput::controlModeTexts::controlModeEntries;
using dInput::controlModeTexts::findControlModeEntry;
using dInput::keyGlyphs::isGamepadKey;
using dInput::keyGlyphs::keyGlyph;

constexpr int kFirstKeyIdValue = static_cast<int>(KeyId::Key_A);
constexpr int kLastKeyIdValue = dInput::keyGlyphs::kLastKeyIdValueAsShipped;
constexpr std::size_t kMaxTableStringLength = 64;

const std::vector<std::pair<KeyId, std::string_view>> kExpectedGlyphs = {
	{ KeyId::Key_A, "A" }, { KeyId::Key_B, "B" }, { KeyId::Key_C, "C" }, { KeyId::Key_D, "D" },
	{ KeyId::Key_E, "E" }, { KeyId::Key_F, "F" }, { KeyId::Key_G, "G" }, { KeyId::Key_H, "H" },
	{ KeyId::Key_I, "I" }, { KeyId::Key_J, "J" }, { KeyId::Key_K, "K" }, { KeyId::Key_L, "L" },
	{ KeyId::Key_M, "M" }, { KeyId::Key_N, "N" }, { KeyId::Key_O, "O" }, { KeyId::Key_P, "P" },
	{ KeyId::Key_Q, "Q" }, { KeyId::Key_R, "R" }, { KeyId::Key_S, "S" }, { KeyId::Key_T, "T" },
	{ KeyId::Key_U, "U" }, { KeyId::Key_V, "V" }, { KeyId::Key_W, "W" }, { KeyId::Key_X, "X" },
	{ KeyId::Key_Y, "Y" }, { KeyId::Key_Z, "Z" },
	{ KeyId::Key_Space, "Space" }, { KeyId::Key_LeftShift, "LShift" },
	{ KeyId::Key_LeftControl, "LCtrl" }, { KeyId::Key_LeftAlt, "LAlt" },
	{ KeyId::Key_1, "1" }, { KeyId::Key_2, "2" }, { KeyId::Key_3, "3" }, { KeyId::Key_4, "4" },
	{ KeyId::Key_5, "5" }, { KeyId::Key_6, "6" }, { KeyId::Key_7, "7" }, { KeyId::Key_8, "8" },
	{ KeyId::Key_9, "9" }, { KeyId::Key_0, "0" },
	{ KeyId::Mouse_X, "Mouse X" }, { KeyId::Mouse_Y, "Mouse Y" }, { KeyId::Mouse_XY, "Mouse" },
	{ KeyId::Mouse_Left, "LMB" }, { KeyId::Mouse_Right, "RMB" }, { KeyId::Mouse_Middle, "MMB" },
	{ KeyId::Gamepad_LeftStick_XY, "LS" }, { KeyId::Gamepad_RightStick_XY, "RS" },
	{ KeyId::Gamepad_FaceBottom, "Pad A" }, { KeyId::Gamepad_FaceRight, "Pad B" },
	{ KeyId::Gamepad_FaceLeft, "Pad X" }, { KeyId::Gamepad_FaceTop, "Pad Y" },
	{ KeyId::Gamepad_DPad_Up, "D-pad Up" }, { KeyId::Gamepad_DPad_Down, "D-pad Down" },
	{ KeyId::Gamepad_DPad_Left, "D-pad Left" }, { KeyId::Gamepad_DPad_Right, "D-pad Right" },
	{ KeyId::Gamepad_LeftTrigger, "LT" }, { KeyId::Gamepad_RightTrigger, "RT" },
	{ KeyId::Gamepad_LeftTriggerAxis, "LT axis" }, { KeyId::Gamepad_RightTriggerAxis, "RT axis" },
	{ KeyId::Gamepad_LeftShoulder, "LB" }, { KeyId::Gamepad_RightShoulder, "RB" },
	{ KeyId::Gamepad_Special_Left, "View" },
};

const std::vector<KeyId> kGamepadKeys = {
	KeyId::Gamepad_LeftStick_XY, KeyId::Gamepad_RightStick_XY,
	KeyId::Gamepad_FaceBottom, KeyId::Gamepad_FaceRight, KeyId::Gamepad_FaceLeft, KeyId::Gamepad_FaceTop,
	KeyId::Gamepad_DPad_Up, KeyId::Gamepad_DPad_Down, KeyId::Gamepad_DPad_Left, KeyId::Gamepad_DPad_Right,
	KeyId::Gamepad_LeftTrigger, KeyId::Gamepad_RightTrigger,
	KeyId::Gamepad_LeftTriggerAxis, KeyId::Gamepad_RightTriggerAxis,
	KeyId::Gamepad_LeftShoulder, KeyId::Gamepad_RightShoulder,
	KeyId::Gamepad_Special_Left,
};

bool isSevenBitAscii(std::string_view text)
{
	return std::all_of(text.begin(), text.end(),
		[](char c) { return static_cast<unsigned char>(c) < 0x80u; });
}

bool containsDigit(std::string_view text)
{
	return std::any_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; });
}

std::size_t bindingCount(const MappingContext& ctx, const ActionDescriptor* action)
{
	std::size_t count = 0;
	for (const auto& mapping : ctx.actionMappings)
		if (mapping.action == action)
			count += mapping.bindings.size();
	return count;
}

std::vector<const ControlModeEntry*> entriesOfKind(ControlModeKind kind)
{
	std::vector<const ControlModeEntry*> found;
	for (const ControlModeEntry& entry : controlModeEntries())
		if (entry.kind == kind)
			found.push_back(&entry);
	return found;
}

TEST_CASE("ControlsHud: every KeyId from Key_A through the pinned last KeyId has a non-empty 7-bit ASCII glyph, and no two glyphs are equal",
	"[CharacterViz][ControlsHud]")
{
	CHECK(kFirstKeyIdValue == 0);
	std::set<std::string_view> seen;
	for (int value = kFirstKeyIdValue; value <= kLastKeyIdValue; ++value)
	{
		CAPTURE(value);
		const std::string_view glyph = keyGlyph(static_cast<KeyId>(value));
		CAPTURE(std::string(glyph));
		CHECK_FALSE(glyph.empty());
		CHECK(isSevenBitAscii(glyph));
		CHECK(seen.insert(glyph).second);
	}
	CHECK(seen.size() == static_cast<std::size_t>(kLastKeyIdValue - kFirstKeyIdValue + 1));
}

TEST_CASE("ControlsHud: keyGlyph spells each KeyId as the backlog lists it",
	"[CharacterViz][ControlsHud]")
{
	REQUIRE(kExpectedGlyphs.size() == static_cast<std::size_t>(kLastKeyIdValue - kFirstKeyIdValue + 1));
	for (std::size_t index = 0; index < kExpectedGlyphs.size(); ++index)
	{
		const auto& [key, expected] = kExpectedGlyphs[index];
		CAPTURE(index, static_cast<int>(key), std::string(expected));
		CHECK(static_cast<std::size_t>(key) == index);
		CHECK(std::string(keyGlyph(key)) == std::string(expected));
	}
}

TEST_CASE("ControlsHud: isGamepadKey is true exactly for the Gamepad_ enumerators",
	"[CharacterViz][ControlsHud]")
{
	CHECK(kGamepadKeys.size() == 17u);
	for (const KeyId listedKey : kGamepadKeys)
	{
		CAPTURE(static_cast<int>(listedKey));
		CHECK(static_cast<int>(listedKey) <= kLastKeyIdValue);
	}
	std::size_t gamepadCount = 0;
	for (int value = kFirstKeyIdValue; value <= kLastKeyIdValue; ++value)
	{
		const KeyId key = static_cast<KeyId>(value);
		const bool listed = std::find(kGamepadKeys.begin(), kGamepadKeys.end(), key) != kGamepadKeys.end();
		CAPTURE(value, std::string(keyGlyph(key)), listed);
		CHECK(isGamepadKey(key) == listed);
		if (isGamepadKey(key))
			++gamepadCount;
	}
	CHECK(gamepadCount == kGamepadKeys.size());
}

TEST_CASE("ControlsHud: the control-mode table lists BlockLook, the four scheme shortcuts and ToggleControlsLegend, in row order",
	"[CharacterViz][ControlsHud]")
{
	const std::vector<const ActionDescriptor*> expectedOrder = {
		&dInput::gameMapping::BlockLook,
		&dInput::gameMapping::SetSchemeCameraRelative,
		&dInput::gameMapping::SetSchemeAimRelative,
		&dInput::gameMapping::SetSchemeMoveRelativeAim,
		&dInput::gameMapping::SetSchemeAimRelativeSwapped,
		&dInput::gameMapping::ToggleControlsLegend,
	};
	const std::vector<ControlModeEntry>& entries = controlModeEntries();
	REQUIRE(entries.size() == expectedOrder.size());
	for (std::size_t index = 0; index < entries.size(); ++index)
	{
		CAPTURE(index, std::string(entries[index].shortLabel));
		CHECK(entries[index].action == expectedOrder[index]);
	}
}

TEST_CASE("ControlsHud: every table entry's action is bound in buildDefaultContext, and no two entries share an action",
	"[CharacterViz][ControlsHud]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();
	const std::vector<ControlModeEntry>& entries = controlModeEntries();
	std::set<const ActionDescriptor*> actions;
	for (const ControlModeEntry& entry : entries)
	{
		CAPTURE(std::string(entry.shortLabel));
		REQUIRE(entry.action != nullptr);
		CAPTURE(std::string(entry.action->name));
		CHECK(bindingCount(ctx, entry.action) >= 1u);
		CHECK(actions.insert(entry.action).second);
		CHECK(findControlModeEntry(entry.action) == &entry);
	}
	CHECK(findControlModeEntry(&dInput::gameMapping::Jump) == nullptr);
	CHECK(findControlModeEntry(nullptr) == nullptr);
}

TEST_CASE("ControlsHud: one MovementScheme entry per scheme, pinned to today's 7 / 8 / 9 / 0 actions",
	"[CharacterViz][ControlsHud]")
{
	const std::vector<std::pair<const ActionDescriptor*, MovementScheme>> expectedPairs = {
		{ &dInput::gameMapping::SetSchemeCameraRelative, MovementScheme::CameraRelative },
		{ &dInput::gameMapping::SetSchemeAimRelative, MovementScheme::AimRelative },
		{ &dInput::gameMapping::SetSchemeMoveRelativeAim, MovementScheme::MoveRelativeAim },
		{ &dInput::gameMapping::SetSchemeAimRelativeSwapped, MovementScheme::AimRelativeSwapped },
	};
	const std::vector<const ControlModeEntry*> schemeEntries = entriesOfKind(ControlModeKind::MovementScheme);
	CHECK(schemeEntries.size() == expectedPairs.size());
	for (const auto& [action, scheme] : expectedPairs)
	{
		CAPTURE(std::string(action->name), static_cast<int>(scheme));
		const std::size_t withScheme = static_cast<std::size_t>(std::count_if(schemeEntries.begin(), schemeEntries.end(),
			[scheme = scheme](const ControlModeEntry* entry) { return entry->scheme == scheme; }));
		CHECK(withScheme == 1u);
		const ControlModeEntry* entry = findControlModeEntry(action);
		REQUIRE(entry != nullptr);
		CHECK(entry->kind == ControlModeKind::MovementScheme);
		CHECK(entry->scheme == scheme);
		CHECK_FALSE(entry->lines.empty());
		CHECK(entry->linesWhenOn.empty());
	}
}

TEST_CASE("ControlsHud: exactly one CameraLook entry (BlockLook, both bodies) and one LegendToggle entry (ToggleControlsLegend, no body)",
	"[CharacterViz][ControlsHud]")
{
	const std::vector<const ControlModeEntry*> cameraLook = entriesOfKind(ControlModeKind::CameraLook);
	REQUIRE(cameraLook.size() == 1u);
	CHECK(cameraLook[0]->action == &dInput::gameMapping::BlockLook);
	CHECK_FALSE(cameraLook[0]->lines.empty());
	CHECK_FALSE(cameraLook[0]->linesWhenOn.empty());

	const std::vector<const ControlModeEntry*> legendToggle = entriesOfKind(ControlModeKind::LegendToggle);
	REQUIRE(legendToggle.size() == 1u);
	CHECK(legendToggle[0]->action == &dInput::gameMapping::ToggleControlsLegend);
	CHECK(legendToggle[0]->lines.empty());
	CHECK(legendToggle[0]->linesWhenOn.empty());
}

TEST_CASE("ControlsHud: every table string is 7-bit ASCII, at most 64 characters, and holds no digit",
	"[CharacterViz][ControlsHud]")
{
	for (const ControlModeEntry& entry : controlModeEntries())
	{
		std::vector<std::string_view> strings = { entry.shortLabel, entry.title };
		strings.insert(strings.end(), entry.lines.begin(), entry.lines.end());
		strings.insert(strings.end(), entry.linesWhenOn.begin(), entry.linesWhenOn.end());
		for (const std::string_view text : strings)
		{
			CAPTURE(std::string(entry.action->name), std::string(text), text.size());
			CHECK_FALSE(text.empty());
			CHECK(isSevenBitAscii(text));
			CHECK(text.size() <= kMaxTableStringLength);
			CHECK_FALSE(containsDigit(text));
		}
	}
}

} // namespace brawlerControlsHudTextsTest

#endif // WITH_LOW_LEVEL_TESTS
