// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/InputMapping/GameInputMapping.h"

#include <string>
#include <vector>

namespace gameInputMappingTest
{
using dInput::ActionDescriptor;
using dInput::KeyId;
using dInput::KeyModifier;
using dInput::MappingContext;

struct KeyUse
{
	const ActionDescriptor* action;
	KeyModifier modifier;
};

std::vector<KeyUse> usesOf(const MappingContext& ctx, KeyId key)
{
	std::vector<KeyUse> uses;
	for (const auto& mapping : ctx.actionMappings)
		for (const auto& binding : mapping.bindings)
			if (binding.key == key)
				uses.push_back({ mapping.action, binding.modifier });
	return uses;
}

std::vector<KeyId> keysOf(const MappingContext& ctx, const ActionDescriptor& action)
{
	std::vector<KeyId> keys;
	for (const auto& mapping : ctx.actionMappings)
		if (mapping.action == &action)
			for (const auto& binding : mapping.bindings)
				keys.push_back(binding.key);
	return keys;
}

TEST_CASE("GameInputMapping: the gamepad left stick is bound to MoveStick only",
	"[SimulatableBrawler][GameInputMapping]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	const std::vector<KeyUse> uses = usesOf(ctx, KeyId::Gamepad_LeftStick_XY);
	REQUIRE(uses.size() == 1);
	CHECK(uses[0].action == &dInput::gameMapping::MoveStick);
	CHECK(uses[0].modifier == KeyModifier::None);
	CHECK(dInput::gameMapping::MoveStick.valueType == dInput::ActionValueType::Axis2D);
	CHECK(std::string(dInput::gameMapping::MoveStick.name) == "MoveStick");
	CHECK(keysOf(ctx, dInput::gameMapping::MoveStick) == std::vector<KeyId>{ KeyId::Gamepad_LeftStick_XY });
}

TEST_CASE("GameInputMapping: Move keeps WASD and the D-pad and no stick",
	"[SimulatableBrawler][GameInputMapping]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	CHECK(keysOf(ctx, dInput::gameMapping::Move) == std::vector<KeyId>{
		KeyId::Key_W, KeyId::Key_S, KeyId::Key_A, KeyId::Key_D,
		KeyId::Gamepad_DPad_Up, KeyId::Gamepad_DPad_Down, KeyId::Gamepad_DPad_Left, KeyId::Gamepad_DPad_Right });

	const std::vector<KeyUse> rightStick = usesOf(ctx, KeyId::Gamepad_RightStick_XY);
	REQUIRE(rightStick.size() == 1);
	CHECK(rightStick[0].action == &dInput::gameMapping::Aim);
}

TEST_CASE("GameInputMapping: Key_0 selects AimRelativeSwapped",
	"[SimulatableBrawler][GameInputMapping]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	const std::vector<KeyUse> uses = usesOf(ctx, KeyId::Key_0);
	REQUIRE(uses.size() == 1);
	CHECK(uses[0].action == &dInput::gameMapping::SetSchemeAimRelativeSwapped);
	CHECK(uses[0].modifier == KeyModifier::None);
	CHECK(dInput::gameMapping::SetSchemeAimRelativeSwapped.valueType == dInput::ActionValueType::Boolean);
	CHECK(keysOf(ctx, dInput::gameMapping::SetSchemeAimRelativeSwapped) == std::vector<KeyId>{ KeyId::Key_0 });
}

TEST_CASE("GameInputMapping: BlockLook is bound to Key_5 and gamepad Y only, and LT and Left Alt are free",
	"[SimulatableBrawler][GameInputMapping]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	CHECK(dInput::gameMapping::BlockLook.valueType == dInput::ActionValueType::Boolean);
	CHECK(keysOf(ctx, dInput::gameMapping::BlockLook) == std::vector<KeyId>{ KeyId::Key_5, KeyId::Gamepad_FaceTop });

	for (const KeyId key : { KeyId::Key_5, KeyId::Gamepad_FaceTop })
	{
		CAPTURE(static_cast<int>(key));
		const std::vector<KeyUse> uses = usesOf(ctx, key);
		REQUIRE(uses.size() == 1);
		CHECK(uses[0].action == &dInput::gameMapping::BlockLook);
		CHECK(uses[0].modifier == KeyModifier::None);
	}

	for (const KeyId key : { KeyId::Key_LeftAlt, KeyId::Gamepad_LeftTriggerAxis, KeyId::Gamepad_LeftTrigger })
	{
		CAPTURE(static_cast<int>(key));
		CHECK(usesOf(ctx, key).empty());
	}
}

TEST_CASE("GameInputMapping: Key_8 selects AimRelative",
	"[SimulatableBrawler][GameInputMapping]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	const std::vector<KeyUse> uses = usesOf(ctx, KeyId::Key_8);
	REQUIRE(uses.size() == 1);
	CHECK(uses[0].action == &dInput::gameMapping::SetSchemeAimRelative);
	CHECK(uses[0].modifier == KeyModifier::None);
	CHECK(keysOf(ctx, dInput::gameMapping::SetSchemeAimRelative) == std::vector<KeyId>{ KeyId::Key_8 });
}

TEST_CASE("GameInputMapping: the four scheme switches are the digit keys 7, 8, 9 and 0, one key each",
	"[SimulatableBrawler][GameInputMapping]")
{
	const MappingContext ctx = dInput::gameMapping::buildDefaultContext();

	CHECK(keysOf(ctx, dInput::gameMapping::SetSchemeCameraRelative) == std::vector<KeyId>{ KeyId::Key_7 });
	CHECK(keysOf(ctx, dInput::gameMapping::SetSchemeAimRelative) == std::vector<KeyId>{ KeyId::Key_8 });
	CHECK(keysOf(ctx, dInput::gameMapping::SetSchemeMoveRelativeAim) == std::vector<KeyId>{ KeyId::Key_9 });
	CHECK(keysOf(ctx, dInput::gameMapping::SetSchemeAimRelativeSwapped) == std::vector<KeyId>{ KeyId::Key_0 });
}

} // namespace gameInputMappingTest

#endif // WITH_LOW_LEVEL_TESTS
