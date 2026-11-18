// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/SimPayloadBuffers.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationNetSync.h"

namespace simPayloadBuffersBrawlerTest
{
    template <typename T>
    struct SpbBrawlerCompositeBytes;

    template <typename... Ts>
    struct SpbBrawlerCompositeBytes<SimulationComposite<Ts...>>
    {
        static constexpr std::uint32_t value = compositeSyncSize<Ts...>();
    };

    using BrawlerState = simulatableBrawler::State;
    using BrawlerInput = simulatableBrawler::PlayerInput;

    constexpr std::uint32_t kInboxMaxMessageBytes = 512u;

    constexpr std::uint32_t kCorrectionEnvelopeBytes = 4u;
    constexpr std::uint32_t kRelayRingEnvelopeBytes  = 3u;
    constexpr std::uint32_t kBundleSetEnvelopeBytes  = 3u;
    constexpr std::uint32_t kBundleSetEntryBytes     = 1u;

    constexpr std::uint32_t kCorrectionUsedBytes =
        correctionStateBuffer::kHeaderBytes + SpbBrawlerCompositeBytes<BrawlerState>::value;
    constexpr std::uint32_t kMaxRelayRingBytes =
        relayedInputRing::kHeaderBytes
        + static_cast<std::uint32_t>(relayedInputRing::kMaxDepth) * relayedInputRing::detail::entryStride<BrawlerInput>();
    constexpr std::uint32_t kMaxBundleBytes =
        inputRedundancyBundle::kHeaderBytes
        + static_cast<std::uint32_t>(inputRedundancyBundle::kMaxSlots)
            * (static_cast<std::uint32_t>(sizeof(std::uint32_t)) + inputRedundancyBundle::detail::slotInputSize<BrawlerInput>());

    static_assert(CorrectionStateSyncedBufferConcept<CorrectionStateBuffer, BrawlerState>,
        "CorrectionStateBuffer must satisfy NetSync's correction buffer concept for the brawler state: it is "
        "SimNetChannel's SyncedCorrectionBufferType. Was 'as FSimulationStateSyncBuffer does' (deleted by "
        "og-input-latency task P5-9)");

    static_assert(kCorrectionUsedBytes == 333u,
        "the brawler correction payload is 8 B of header plus the 325 B state composite. A change here is a state "
        "footprint change: re-check og-abilities' footprint and the 384 B buffer");
    static_assert(kMaxRelayRingBytes == 354u,
        "a full brawler relay ring is 2 + 8 x (4 + 1 + 39) B");
    static_assert(kMaxBundleBytes == 346u,
        "a full brawler redundancy bundle is 2 + 8 x (4 + 39) B");

    static_assert(kCorrectionEnvelopeBytes + kCorrectionUsedBytes <= kInboxMaxMessageBytes,
        "a brawler correction plus its envelope (version, kind, simId, corrVersion) exceeds the inbox's 512 B record. "
        "Re-check og-abilities' state footprint, which grows this payload");
    static_assert(kRelayRingEnvelopeBytes + kMaxRelayRingBytes <= kInboxMaxMessageBytes,
        "a full brawler relay ring plus its envelope (version, kind, simId) exceeds the inbox's 512 B record. Re-check "
        "og-abilities' input footprint, which grows each ring entry");
    static_assert(kBundleSetEnvelopeBytes + kBundleSetEntryBytes + kMaxBundleBytes <= kInboxMaxMessageBytes,
        "a one-bundle brawler input set plus its envelope (version, kind, count, simId) exceeds the inbox's 512 B "
        "record. Re-check og-abilities' input footprint, which grows each bundle slot");

    TEST_CASE("SimPayloadBuffers.BrawlerPayloadsFitTheirBuffersAndTheInbox", "[WireFormat][PacketBudget]")
    {
        BrawlerState state{};
        CorrectionStateBuffer sent;
        sent.write(state, 4242u, 4240u);
        CHECK(sent.usedBytes() == kCorrectionUsedBytes);

        CorrectionStateBuffer received;
        REQUIRE(received.assignFromWire(sent.payload()));
        BrawlerState decoded{};
        CHECK(received.readInto(decoded) == 4242u);
        CHECK(received.getAppliedCaptureTick() == 4240u);

        CorrectionStateBuffer rewritten;
        rewritten.write(decoded, 4242u, 4240u);
        const std::span<const std::byte> a = sent.payload();
        const std::span<const std::byte> b = rewritten.payload();
        REQUIRE(a.size() == b.size());
        std::vector<std::uint8_t> av;
        std::vector<std::uint8_t> bv;
        for (std::size_t i = 0; i < a.size(); ++i)
        {
            av.push_back(std::to_integer<std::uint8_t>(a[i]));
            bv.push_back(std::to_integer<std::uint8_t>(b[i]));
        }
        CHECK(av == bv);

        const BrawlerInput input = BrawlerInput::zero();
        RelayRingBuffer stage;
        RelayRingBuffer ring;
        for (std::uint32_t tick = 1000u; tick < 1000u + relayedInputRing::kMaxDepth; ++tick)
            CHECK(stage.stageArrival<BrawlerInput>(tick, 6u, input).accepted);
        CHECK(relayedInputRing::flushStagedInto(ring, stage, 2) == relayedInputRing::kMaxDepth);
        CHECK(ring.payload().size() == kMaxRelayRingBytes);

        InputBundleBuffer bundle;
        for (std::uint32_t tick = 2000u; tick < 2000u + inputRedundancyBundle::kMaxSlots; ++tick)
            CHECK(bundle.tryAppendSlot<BrawlerInput>(tick, input));
        CHECK(bundle.payload().size() == kMaxBundleBytes);
    }
} // namespace simPayloadBuffersBrawlerTest

#endif
