// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimNet/SimNetChannel.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/SimTransport.h"
#include "OGSimulation/SimulationNetSync.h"
#include "OGSimulation/SimulationQueues.h"

namespace simNetChannelBrawlerTest
{
    using BrawlerInput = simulatableBrawler::PlayerInput;

    struct TestBundleBuffer
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value;
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }
    };

    void buildBundleIntoTestBuffer(const PendingInputQueue<BrawlerInput>& queue, std::uint32_t currentTick,
        std::uint8_t redundancyDepth, TestBundleBuffer& outBundle)
    {
        outBundle.bytes.clear();
        const std::uint8_t depth = redundancyDepth < inputRedundancyBundle::kMaxSlots
            ? redundancyDepth
            : inputRedundancyBundle::kMaxSlots;
        queue.forEachRecent(currentTick, static_cast<std::size_t>(depth),
            [&outBundle](std::uint32_t captureTick, const BrawlerInput& input)
            {
                inputRedundancyBundle::appendSlot<BrawlerInput>(outBundle, captureTick, input);
            });
    }

    struct RecordingChannelTransport
    {
        std::vector<std::vector<std::uint8_t>>* sent;

        void sendInputBundle(std::span<const std::byte> bundle)
        {
            std::vector<std::uint8_t> bytes;
            for (const std::byte b : bundle)
                bytes.push_back(std::to_integer<std::uint8_t>(b));
            sent->push_back(std::move(bytes));
        }

        void endInputSends() {}
    };

    static_assert(ClientSimTransport<RecordingChannelTransport>);

    using BrawlerChannel = ogsimnet::SimNetChannel<SimulatableBrawler, RecordingChannelTransport>;

    static_assert(PredictionSyncedBufferOwnerConcept<BrawlerChannel, simulatableBrawler::State, BrawlerInput>,
        "SimNetChannel must satisfy NetSync's prediction owner concept for the brawler: since og-input-latency task P5-7 "
        "it is the brawler's owner (UESimNetChannel, SimulatableBrawlerOwnerTraits.h). Was 'as the brawler component does'");
    static_assert(AuthoritySyncedBufferOwnerConcept<BrawlerChannel, simulatableBrawler::State, BrawlerInput>,
        "SimNetChannel must satisfy NetSync's authority owner concept for the brawler: since og-input-latency task P5-7 "
        "it is the brawler's owner (UESimNetChannel, SimulatableBrawlerOwnerTraits.h). Was 'as the brawler component does'");

    BrawlerInput inputFor(std::uint32_t tick)
    {
        BrawlerInput input       = BrawlerInput::zero();
        input.attackLeft         = (tick % 2u) == 0u;
        input.attackRight        = (tick % 3u) == 0u;
        input.moveStick          = glm::vec2(0.25f * static_cast<float>(tick % 5u), -0.5f);
        input.moveDirectionWorld = glm::vec3(0.f, 1.f, 0.f);
        input.triggeredActionId  = tick * 7u;
        input.flags              = static_cast<std::uint8_t>(tick & 0x3Fu);
        return input;
    }

    TEST_CASE("SimNetChannel.BrawlerBundleBytesEqualBuildRedundancyBundle", "[WireFormat][SimulationNetSync]")
    {
        PendingInputQueue<BrawlerInput> queue;
        for (std::uint32_t tick = 3000u; tick < 3012u; ++tick)
            queue.enqueue(tick, inputFor(tick));

        std::vector<std::vector<std::uint8_t>> sent;
        BrawlerChannel channel(RecordingChannelTransport{&sent}, 3u, "brawler");

        const std::uint32_t depths[] = {0u, 1u, 3u, 5u, 8u, 9u, 60u, 256u, 259u};
        const std::uint32_t ticks[]  = {3011u, 3006u, 2999u};
        for (const std::uint32_t currentTick : ticks)
        {
            for (const std::uint32_t depth : depths)
            {
                INFO("currentTick " << currentTick << " depth " << depth);
                channel.sendLocalInputToAuthority(queue, currentTick, depth);
                REQUIRE(!sent.empty());

                TestBundleBuffer reference;
                buildBundleIntoTestBuffer(queue, currentTick, static_cast<std::uint8_t>(depth), reference);
                CHECK(sent.back() == reference.bytes);
            }
        }

        CHECK(sent.size() == 27u);
        CHECK(sent[2].size() == 2u + 3u * (4u + 39u));
        CHECK(sent[4].size() == 2u + 8u * (4u + 39u));
        CHECK(sent[7].empty());
        CHECK(sent[8].size() == 2u + 3u * (4u + 39u));
        CHECK(!channel.isDirty());
    }
} // namespace simNetChannelBrawlerTest

#endif
