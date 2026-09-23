// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <string>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/OGBrawlerLog.h"
#include "OGSimulation/SimulationLog.h"

// ---------------------------------------------------------------------------
// [og-netcode-v2-field-defects task 16] THE (id, tick) PREFIX IS INSERTED BY THE EMITTER,
// NEVER AUTHORED IN A FORMAT STRING.
//
// Inside a simulationLog::IntegrateScope (opened by the integration executor around each
// character's integrate), every OGBLOG_G line gains `id=<id> tick=<tick> ` directly after its
// tag — past a leading `[Verbose]` / `[Warning]` verbosity marker, which the UE sink must still
// see first. Outside any scope the line is byte-identical to what the format prints.
// ---------------------------------------------------------------------------

namespace ogbrawlerlogscopetests
{
    // Captures every line OGBLOG_G emits while alive; restores the previous sink after.
    struct SinkCapture
    {
        std::vector<std::string> lines;
        std::function<void(const char*)> previous = ::ogblog::g_sink;

        SinkCapture() { ::ogblog::setGlobal([this](const char* msg) { lines.emplace_back(msg); }); }
        ~SinkCapture() { ::ogblog::setGlobal(std::move(previous)); }

        std::string only() const
        {
            REQUIRE(lines.size() == 1u);
            return lines.front();
        }
    };
}

TEST_CASE("OGBrawlerLog.IntegrateScopePrefixLandsAfterTheTag", "[DAttack][OGBrawlerLog]")
{
    using namespace ogbrawlerlogscopetests;
    simulationLog::IntegrateScope scope(7u, 9u);

    SECTION("a plain tag")
    {
        SinkCapture capture;
        OGBLOG_G("[Machine.transition] Idle -> Attacking seq=%u endTick=%u", 2u, 30u);
        CHECK(capture.only() == "[Machine.transition] id=7 tick=9 Idle -> Attacking seq=2 endTick=30");
    }
    SECTION("a [Verbose] marker is skipped: the prefix goes after the TAG, never before the marker")
    {
        SinkCapture capture;
        OGBLOG_G("[Verbose][Radial.branch] idle (state.curSeq invalid)");
        CHECK(capture.only() == "[Verbose][Radial.branch] id=7 tick=9 idle (state.curSeq invalid)");
    }
    SECTION("a [Warning] marker is skipped too")
    {
        SinkCapture capture;
        OGBLOG_G("[Warning][Movement.hover] omega=%.3f", 1.5f);
        CHECK(capture.only() == "[Warning][Movement.hover] id=7 tick=9 omega=1.500");
    }
    SECTION("a tagless line gets the prefix at column 0")
    {
        SinkCapture capture;
        OGBLOG_G("no tag here n=%u", 5u);
        CHECK(capture.only() == "id=7 tick=9 no tag here n=5");
    }
}

TEST_CASE("OGBrawlerLog.OutsideAnyScopeTheLineIsByteIdentical", "[DAttack][OGBrawlerLog]")
{
    using namespace ogbrawlerlogscopetests;
    REQUIRE_FALSE(simulationLog::currentIntegrateScope().has_value());

    SinkCapture capture;
    OGBLOG_G("[Machine.transition] Idle -> Attacking seq=%u endTick=%u", 2u, 30u);
    OGBLOG_G("[Verbose][Radial.branch] idle (state.curSeq invalid)");
    OGBLOG_G("[Warning][Movement.hover] omega=%.3f", 1.5f);
    OGBLOG_G("no tag here n=%u", 5u);

    const std::vector<std::string> expected = {
        "[Machine.transition] Idle -> Attacking seq=2 endTick=30",
        "[Verbose][Radial.branch] idle (state.curSeq invalid)",
        "[Warning][Movement.hover] omega=1.500",
        "no tag here n=5",
    };
    CHECK(capture.lines == expected);
}

// The buffer check reserves the WIDEST prefix (`id=4294967295 tick=4294967295 `, 30 chars) on
// top of every literal's worst case. A literal the compile-time check admits at its limit must
// therefore reach the sink whole with the widest prefix inserted — nothing clipped.
TEST_CASE("OGBrawlerLog.TheWidestPrefixOnTheLongestAdmittedLiteralDoesNotClip", "[DAttack][OGBrawlerLog]")
{
    using namespace ogbrawlerlogscopetests;
    static_assert(::ogblog::kIntegrateScopePrefixMaxBytes == 30u);

#define OGBLOG_SCOPE_TEST_X16  "xxxxxxxxxxxxxxxx"
#define OGBLOG_SCOPE_TEST_X128 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 \
                               OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16
    // "[T] " (4) + 7*128 (896) + 5*16 (80) + 13 = 993 chars; 993 + 30 = 1023 = kLineBufferBytes - 1.
#define OGBLOG_SCOPE_TEST_LONGEST "[T] " \
    OGBLOG_SCOPE_TEST_X128 OGBLOG_SCOPE_TEST_X128 OGBLOG_SCOPE_TEST_X128 OGBLOG_SCOPE_TEST_X128 \
    OGBLOG_SCOPE_TEST_X128 OGBLOG_SCOPE_TEST_X128 OGBLOG_SCOPE_TEST_X128 \
    OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 OGBLOG_SCOPE_TEST_X16 \
    "xxxxxxxxxxxxx"
    static_assert(sizeof(OGBLOG_SCOPE_TEST_LONGEST) - 1u == 993u);
    static_assert(sizeof(OGBLOG_SCOPE_TEST_LONGEST) - 1u + ::ogblog::kIntegrateScopePrefixMaxBytes
                  == ::ogblog::kLineBufferBytes - 1u);

    simulationLog::IntegrateScope scope(4294967295u, 4294967295u);
    SinkCapture capture;
    OGBLOG_G(OGBLOG_SCOPE_TEST_LONGEST);

    const std::string literal = OGBLOG_SCOPE_TEST_LONGEST;
    const std::string expected = "[T] id=4294967295 tick=4294967295 " + literal.substr(4);
    CHECK(capture.only().size() == ::ogblog::kLineBufferBytes - 1u);
    CHECK(capture.only() == expected);

#undef OGBLOG_SCOPE_TEST_LONGEST
#undef OGBLOG_SCOPE_TEST_X128
#undef OGBLOG_SCOPE_TEST_X16
}

#endif // WITH_LOW_LEVEL_TESTS
