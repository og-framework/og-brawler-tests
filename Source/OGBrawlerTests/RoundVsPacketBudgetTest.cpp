// SPDX-License-Identifier: BUSL-1.1
// docs/RoundVsPacketBudgetTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SessionConstants.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGNet/Lanes/LaneConfig.h"
#include "OGSimNet/SimNetServer.h"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/Network/SimEnvelope.h"
#include "OGSimulation/Network/SimPayloadBuffers.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

namespace roundVsPacketBudgetTest
{
    using BrawlerState = simulatableBrawler::State;

    constexpr std::uint8_t kSimId = 7u;
    constexpr std::size_t kEncodeBufferBytes = 512u;

    constexpr std::uint32_t kShippedBytesPerSecond = 1250000u;
    constexpr ognet::lanes::BudgetConfig kShippedBudget{ kShippedBytesPerSecond };
    static_assert(kShippedBudget.bytesPerSecond > ognet::lanes::kDefaultBudget.bytesPerSecond
            && ognet::lanes::isValidBudget(kShippedBudget),
        "the shipped og-net budget mirrored here (OGNetUnreal's budgetFor of Config/DefaultEngine.ini's rates, pinned by "
        "EngineLaneSettings.h's static_assert) is a valid budget above og-net's floor kDefaultBudget. Was the old "
        "RoundVsPacketBudgetTest.cpp's mirrored engine constants, re-based by og-input-latency task P5-26");

    std::size_t encodedCorrectionBytes()
    {
        CorrectionStateBuffer buffer;
        buffer.write(BrawlerState{}, 4242u, 4240u);
        std::array<std::byte, kEncodeBufferBytes> out{};
        return simEnvelope::writeCorrectionState(out, kSimId, correctionStateBuffer::kWireFormatVersion, buffer.payload());
    }

    std::size_t candidateCorrectionBytes()
    {
        CorrectionStateBuffer buffer;
        buffer.write(BrawlerState{}, 4242u, 4240u);
        return simEnvelope::kHeaderBytes + 2u + buffer.payload().size();
    }

    std::size_t allowanceAt(const ognet::lanes::BudgetConfig& budget)
    {
        return ogsimnet::correctionAllowanceBytes(ognet::lanes::kDefaultLaneTable, budget, TimeConfig{}.tickFrequency);
    }

    std::size_t roundBytes(std::size_t characters)
    {
        return characters * encodedCorrectionBytes();
    }

    std::size_t largestFittingRound(std::size_t allowanceBytes)
    {
        return allowanceBytes / encodedCorrectionBytes();
    }

    TEST_CASE("PacketBudget.ABrawlerCorrectionIsTheSizeTheServerPricesIt", "[PacketBudget][SimNetServer]")
    {
        INFO("encoded=" << encodedCorrectionBytes() << " candidate=" << candidateCorrectionBytes());
        CHECK(encodedCorrectionBytes() == candidateCorrectionBytes());
        CHECK(encodedCorrectionBytes() == 337u);
    }

    TEST_CASE("PacketBudget.AtTheShippedBudgetEveryAdmittedCharactersCorrectionFitsOneFlush", "[PacketBudget][SimNetServer]")
    {
        const std::size_t allowance = allowanceAt(kShippedBudget);
        const std::size_t ceiling = static_cast<std::size_t>(og::brawler::session::hardPlayerCeiling);
        INFO("allowance=" << allowance << " ceiling=" << ceiling << " round=" << roundBytes(ceiling)
             << " largestFitting=" << largestFittingRound(allowance));
        CHECK(allowance == 6250u);
        CHECK(roundBytes(ceiling) <= allowance);
        CHECK(largestFittingRound(allowance) == 18u);
    }

    TEST_CASE("PacketBudget.AtOgNetsFloorBudgetTheTier1RoundFitsAndTheFenceBitesOneCharacterLater", "[PacketBudget][SimNetServer]")
    {
        const std::size_t allowance = allowanceAt(ognet::lanes::kDefaultBudget);
        const std::size_t tier1 = static_cast<std::size_t>(og::brawler::session::maxPlayersPerServer);
        const std::size_t tier2 = static_cast<std::size_t>(og::brawler::session::maxPlayersPerServerTier2);
        INFO("allowance=" << allowance << " tier1=" << tier1 << " round1=" << roundBytes(tier1)
             << " tier2=" << tier2 << " round2=" << roundBytes(tier2) << " largestFitting=" << largestFittingRound(allowance));
        CHECK(allowance == 1250u);
        CHECK(roundBytes(tier1) <= allowance);
        CHECK(largestFittingRound(allowance) == tier1);
        CHECK(roundBytes(tier1 + 1u) > allowance);
        CHECK(roundBytes(tier2) > allowance);
    }
} // namespace roundVsPacketBudgetTest

#endif // WITH_LOW_LEVEL_TESTS
