// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerSyncedPlayerInput.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "BrawlerTestInputs.h"

namespace syncedPlayerInputTest
{
using simulatableBrawler::SyncedPlayerInput;

static_assert(std::is_same_v<simulatableBrawler::PlayerInput, simulatableBrawler::SyncedPlayerInput>,
    "simulatableBrawler::PlayerInput must be the flat SyncedPlayerInput on the wire (og-syncedInput-rework "
    "task 3). Replaces the case ...ZeroBytesEqualTheLegacyMachineSliceAndMovementByte, which cross-checked "
    "these 39 bytes against bytes [14,52) ++ [76] of the retired six-slice input composite.");

struct ViewWithFrom
{
    glm::vec3 aimDirection{};
    static ViewWithFrom from(const SyncedPlayerInput& in) { return { .aimDirection = in.aimDirection }; }
};

struct EmptyViewWithFrom
{
    static EmptyViewWithFrom from(const SyncedPlayerInput&) { return {}; }
};

struct ViewWithoutFrom
{
    glm::vec3 aimDirection{};
};

struct SerializableViewWithFrom
{
    glm::vec3 aimDirection{};
    static SerializableViewWithFrom from(const SyncedPlayerInput& in) { return { .aimDirection = in.aimDirection }; }
};

struct OwningViewWithFrom
{
    std::vector<float> history;
    static OwningViewWithFrom from(const SyncedPlayerInput&) { return {}; }
};
} // namespace syncedPlayerInputTest

template <>
struct SerializableFields<syncedPlayerInputTest::SerializableViewWithFrom>
{
    static constexpr auto get()
    {
        using V = syncedPlayerInputTest::SerializableViewWithFrom;
        return std::make_tuple(SIM_MEMBER(V, aimDirection));
    }
};

namespace syncedPlayerInputTest
{
struct ByteBuffer
{
    std::vector<std::uint8_t> bytes;

    std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

    void bundleAddZeroedBytes(std::int32_t count)
    { bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u); }

    template <typename T>
    void writeToBuffer(std::uint32_t off, const T& value)
    { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

    template <typename T>
    T readFromBuffer(std::uint32_t off) const
    {
        T value;
        std::memcpy(&value, bytes.data() + off, sizeof(T));
        return value;
    }
};

constexpr std::uint32_t kSyncedBytes = 39u;

constexpr std::uint8_t kSyncedZeroBytes[kSyncedBytes] = {
    0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x80u,0x3Fu,
    0x00u, 0x00u,
    0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u,
    0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u, 0x00u,0x00u,0x00u,0x00u,
    0x00u,0x00u,0x00u,0x00u,
    0x00u,
};

std::vector<std::uint8_t> serialize(const SyncedPlayerInput& input)
{
    ByteBuffer buf;
    buf.bytes.assign(kSyncedBytes, 0xCDu);
    const std::uint32_t written = writeToSyncedBuffer(input, buf, 0u);
    REQUIRE(written == kSyncedBytes);
    return buf.bytes;
}

int firstDifference(const std::vector<std::uint8_t>& actual, const std::uint8_t* expected, std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i)
        if (actual[i] != expected[i]) return static_cast<int>(i);
    return -1;
}

bool sameInput(const SyncedPlayerInput& a, const SyncedPlayerInput& b)
{
    return a.aimDirection == b.aimDirection
        && a.attackLeft == b.attackLeft
        && a.attackRight == b.attackRight
        && a.moveStick == b.moveStick
        && a.moveDirectionWorld == b.moveDirectionWorld
        && a.triggeredActionId == b.triggeredActionId
        && a.flags == b.flags;
}

void requireEveryFieldEqual(const SyncedPlayerInput& actual, const SyncedPlayerInput& expected)
{
    REQUIRE(actual.aimDirection == expected.aimDirection);
    REQUIRE(actual.attackLeft == expected.attackLeft);
    REQUIRE(actual.attackRight == expected.attackRight);
    REQUIRE(actual.moveStick == expected.moveStick);
    REQUIRE(actual.moveDirectionWorld == expected.moveDirectionWorld);
    REQUIRE(actual.triggeredActionId == expected.triggeredActionId);
    REQUIRE(actual.flags == expected.flags);
}

SyncedPlayerInput richInput()
{
    return SyncedPlayerInput{
        .aimDirection       = glm::vec3(0.25f, -0.5f, 0.75f),
        .attackLeft         = true,
        .attackRight        = true,
        .moveStick          = glm::vec2(0.5f, -0.25f),
        .moveDirectionWorld = glm::vec3(-0.75f, 0.5f, 0.125f),
        .triggeredActionId  = 7u,
        .flags              = brawlerMovementSimulation::kInputFlagHoldGuard,
    };
}

void requireDiffersFromZeroAndDefaultInEveryField(const SyncedPlayerInput& in)
{
    const SyncedPlayerInput zero = SyncedPlayerInput::zero();
    const SyncedPlayerInput def{};
    for (const SyncedPlayerInput* other : { &zero, &def })
    {
        REQUIRE(in.aimDirection != other->aimDirection);
        REQUIRE(in.attackLeft != other->attackLeft);
        REQUIRE(in.attackRight != other->attackRight);
        REQUIRE(in.moveStick != other->moveStick);
        REQUIRE(in.moveDirectionWorld != other->moveDirectionWorld);
        REQUIRE(in.triggeredActionId != other->triggeredActionId);
        REQUIRE(in.flags != other->flags);
    }
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.ZeroSerializesToTheCapturedBytes",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    const std::vector<std::uint8_t> bytes = serialize(SyncedPlayerInput::zero());
    const int diffAt = firstDifference(bytes, kSyncedZeroBytes, kSyncedBytes);
    INFO("first differing byte index (-1 = identical): " << diffAt);
    REQUIRE(diffAt == -1);
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.ZeroIsNotValueInitialised",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    REQUIRE(SyncedPlayerInput::zero().aimDirection == glm::vec3(0.f, 0.f, 1.f));
    REQUIRE(SyncedPlayerInput{}.aimDirection == glm::vec3(0.f, 0.f, 0.f));
    REQUIRE_FALSE(sameInput(SyncedPlayerInput::zero(), SyncedPlayerInput{}));
    REQUIRE(firstDifference(serialize(SyncedPlayerInput{}), kSyncedZeroBytes, kSyncedBytes) != -1);
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.FlagsByteDiscriminatesAtIndex38",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    constexpr std::uint32_t kFlagsByte = 38u;

    SyncedPlayerInput guarded = SyncedPlayerInput::zero();
    guarded.flags = brawlerMovementSimulation::kInputFlagHoldGuard;
    const std::vector<std::uint8_t> bytes = serialize(guarded);

    INFO("first differing byte = " << firstDifference(bytes, kSyncedZeroBytes, kSyncedBytes)
         << ", byte[38] = " << static_cast<int>(bytes[kFlagsByte]));
    REQUIRE(firstDifference(bytes, kSyncedZeroBytes, kSyncedBytes) == static_cast<int>(kFlagsByte));
    REQUIRE(bytes[kFlagsByte] == 1u);
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.RingAndBundleRoundTripEveryField",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    const SyncedPlayerInput rich = richInput();
    requireDiffersFromZeroAndDefaultInEveryField(rich);

    SECTION("relayed input ring")
    {
        constexpr std::uint32_t kCaptureTick = 1234u;
        constexpr std::uint8_t kDA = 3u;

        ByteBuffer ring;
        REQUIRE(relayedInputRing::writeLatest<SyncedPlayerInput>(ring, kCaptureTick, kDA, rich, 1));
        REQUIRE(ring.bytes.size() == 46u);
        REQUIRE(relayedInputRing::getWireFormatVersion(ring) == relayedInputRing::kWireFormatVersion);

        std::uint8_t outDA = 0u;
        SyncedPlayerInput out = SyncedPlayerInput::zero();
        REQUIRE(relayedInputRing::findEntry<SyncedPlayerInput>(ring, kCaptureTick, outDA, out));
        REQUIRE(outDA == kDA);
        requireEveryFieldEqual(out, rich);
    }

    SECTION("input redundancy bundle")
    {
        constexpr std::uint32_t kCaptureTick = 4321u;

        ByteBuffer bundle;
        inputRedundancyBundle::appendSlot<SyncedPlayerInput>(bundle, kCaptureTick, rich);
        REQUIRE(bundle.bytes.size() == 45u);
        REQUIRE(inputRedundancyBundle::getWireFormatVersion(bundle) == inputRedundancyBundle::kWireFormatVersion);

        int visited = 0;
        std::uint32_t seenTick = 0u;
        SyncedPlayerInput out = SyncedPlayerInput::zero();
        inputRedundancyBundle::forEachSlot<SyncedPlayerInput>(bundle,
            [&](std::uint32_t captureTick, const SyncedPlayerInput& input)
            {
                ++visited;
                seenTick = captureTick;
                out = input;
            });
        REQUIRE(visited == 1);
        REQUIRE(seenTick == kCaptureTick);
        requireEveryFieldEqual(out, rich);
    }
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.CodecStrideAndSlotSize",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    STATIC_REQUIRE(relayedInputRing::detail::entryStride<SyncedPlayerInput>() == 44u);
    STATIC_REQUIRE(inputRedundancyBundle::detail::slotInputSize<SyncedPlayerInput>() == 39u);
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.TestHelperMakeCarriesFlags",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    REQUIRE(brawlerTestInputs::make({ .flags = brawlerMovementSimulation::kInputFlagHoldGuard }).flags
            == brawlerMovementSimulation::kInputFlagHoldGuard);
}

TEST_CASE("SimulatableBrawler.SyncedPlayerInput.BrawlerInputViewDiscriminates",
          "[SimulatableBrawler][SyncedPlayerInput]")
{
    STATIC_REQUIRE(simulatableBrawler::BrawlerInputView<ViewWithFrom>);
    STATIC_REQUIRE(simulatableBrawler::BrawlerInputView<EmptyViewWithFrom>);
    STATIC_REQUIRE_FALSE(simulatableBrawler::BrawlerInputView<ViewWithoutFrom>);
    STATIC_REQUIRE_FALSE(simulatableBrawler::BrawlerInputView<SerializableViewWithFrom>);
    STATIC_REQUIRE_FALSE(simulatableBrawler::BrawlerInputView<OwningViewWithFrom>);

    STATIC_REQUIRE(!Serializable<ViewWithoutFrom> && std::is_trivially_copyable_v<ViewWithoutFrom>);
    STATIC_REQUIRE(Serializable<SerializableViewWithFrom> && std::is_trivially_copyable_v<SerializableViewWithFrom>);
    STATIC_REQUIRE(!Serializable<OwningViewWithFrom> && !std::is_trivially_copyable_v<OwningViewWithFrom>);
}

} // namespace syncedPlayerInputTest

#endif // WITH_LOW_LEVEL_TESTS
