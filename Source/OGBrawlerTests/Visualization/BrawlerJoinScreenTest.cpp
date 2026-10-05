// SPDX-License-Identifier: BUSL-1.1
// ../docs/BrawlerJoinScreenTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "OGBrawler/BrawlerJoinScreen.h"

namespace joinscreentests
{

using namespace brawlerJoinScreen;

AddressParseError parseErrorOf(std::string_view text)
{
    return parseServerAddress(text).error;
}

RecentAddressList recentsOf(std::initializer_list<std::string_view> newestLast)
{
    RecentAddressList recents;
    for (const std::string_view address : newestLast)
        recents.remember(address);
    return recents;
}

bool rectInside(const JoinScreenRect& inner, const JoinScreenRect& outer)
{
    constexpr float kSlack = 1e-3f;
    return inner.x >= outer.x - kSlack
        && inner.y >= outer.y - kSlack
        && inner.x + inner.width <= outer.x + outer.width + kSlack
        && inner.y + inner.height <= outer.y + outer.height + kSlack;
}

bool rectIsFinite(const JoinScreenRect& rect)
{
    return std::isfinite(rect.x) && std::isfinite(rect.y) && std::isfinite(rect.width)
        && std::isfinite(rect.height);
}

TEST_CASE("JoinScreen.AnIPv4AddressWithoutAPortGetsTheDefaultPort",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    const ServerAddressParse parsed = parseServerAddress("192.168.1.42");
    REQUIRE(parsed.ok());
    CHECK(parsed.address.host == "192.168.1.42");
    CHECK(parsed.address.port == kDefaultServerPort);
    CHECK(parsed.address.kind == HostKind::IPv4);
    CHECK(parsed.address.canonical() == "192.168.1.42:7777");
    CHECK(kDefaultServerPort == 7777u);
}

TEST_CASE("JoinScreen.AnExplicitPortIsKeptAndTheBoundariesOneAnd65535AreLegal",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    CHECK(parseServerAddress("10.0.0.1:17777").address.port == 17777u);
    CHECK(parseServerAddress("10.0.0.1:1").address.port == 1u);
    CHECK(parseServerAddress("10.0.0.1:65535").address.port == 65535u);
    CHECK(parseServerAddress("10.0.0.1:007777").address.port == 7777u);
    CHECK(parseServerAddress("0.0.0.0").ok());
    CHECK(parseServerAddress("255.255.255.255").ok());
}

TEST_CASE("JoinScreen.AHostnameIsLowercasedSoTwoSpellingsAreOneAddress",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    const ServerAddressParse parsed = parseServerAddress("PlayTest-01.Example.COM:9000");
    REQUIRE(parsed.ok());
    CHECK(parsed.address.kind == HostKind::Hostname);
    CHECK(parsed.address.host == "playtest-01.example.com");
    CHECK(parsed.address.canonical() == "playtest-01.example.com:9000");

    CHECK(parseServerAddress("localhost").address.canonical() == "localhost:7777");
    CHECK(parseServerAddress("1.2.3.4a").address.kind == HostKind::Hostname);
}

TEST_CASE("JoinScreen.SurroundingWhitespaceIsTrimmedBeforeParsing",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    const ServerAddressParse parsed = parseServerAddress(" \t192.168.1.42:7000\r\n");
    REQUIRE(parsed.ok());
    CHECK(parsed.address.canonical() == "192.168.1.42:7000");
}

TEST_CASE("JoinScreen.EveryParseErrorIsReachedByItsOwnInput",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    CHECK(parseErrorOf("") == AddressParseError::Empty);
    CHECK(parseErrorOf("   ") == AddressParseError::Empty);

    CHECK(parseErrorOf(std::string(kAddressMaxLength + 1u, 'a')) == AddressParseError::TooLong);
    CHECK(parseErrorOf(std::string(kAddressMaxLength, '1')) != AddressParseError::TooLong);

    CHECK(parseErrorOf("1.2.3.4/x") == AddressParseError::InvalidCharacter);
    CHECK(parseErrorOf("my server") == AddressParseError::InvalidCharacter);
    CHECK(parseErrorOf("127.0.0.1:7777?InitialConnectTimeout=8") == AddressParseError::InvalidCharacter);
    CHECK(parseErrorOf("/Game/ThirdPerson/Maps/ThirdPersonMap") == AddressParseError::InvalidCharacter);

    CHECK(parseErrorOf(":7777") == AddressParseError::MissingHost);

    CHECK(parseErrorOf("::1") == AddressParseError::TooManyColons);
    CHECK(parseErrorOf("host:1:2") == AddressParseError::TooManyColons);

    CHECK(parseErrorOf("host:") == AddressParseError::MissingPort);

    CHECK(parseErrorOf("host:77a7") == AddressParseError::PortNotNumeric);
    CHECK(parseErrorOf("host:-77") == AddressParseError::PortNotNumeric);

    CHECK(parseErrorOf("host:0") == AddressParseError::PortOutOfRange);
    CHECK(parseErrorOf("host:65536") == AddressParseError::PortOutOfRange);
    CHECK(parseErrorOf("host:99999999999999999999") == AddressParseError::PortOutOfRange);

    CHECK(parseErrorOf("256.1.1.1") == AddressParseError::InvalidIPv4);
    CHECK(parseErrorOf("1.2.3") == AddressParseError::InvalidIPv4);
    CHECK(parseErrorOf("1.2.3.4.5") == AddressParseError::InvalidIPv4);
    CHECK(parseErrorOf("01.2.3.4") == AddressParseError::InvalidIPv4);
    CHECK(parseErrorOf("1..2.3") == AddressParseError::InvalidIPv4);
    CHECK(parseErrorOf("1234.1.1.1") == AddressParseError::InvalidIPv4);
    CHECK(parseErrorOf("7777") == AddressParseError::InvalidIPv4);

    CHECK(parseErrorOf("a..b") == AddressParseError::InvalidHostname);
    CHECK(parseErrorOf(".example") == AddressParseError::InvalidHostname);
    CHECK(parseErrorOf("example.") == AddressParseError::InvalidHostname);
    CHECK(parseErrorOf("-example") == AddressParseError::InvalidHostname);
    CHECK(parseErrorOf("example-") == AddressParseError::InvalidHostname);
    CHECK(parseErrorOf(std::string(kMaxHostnameLabelLength + 1u, 'a')) == AddressParseError::InvalidHostname);
    CHECK(parseErrorOf(std::string(kMaxHostnameLabelLength, 'a')) == AddressParseError::None);
}

TEST_CASE("JoinScreen.EveryParseErrorHasItsOwnTextAndNoneHasNone",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    const std::vector<AddressParseError> errors{
        AddressParseError::Empty,          AddressParseError::TooLong,
        AddressParseError::InvalidCharacter, AddressParseError::MissingHost,
        AddressParseError::TooManyColons,  AddressParseError::MissingPort,
        AddressParseError::PortNotNumeric, AddressParseError::PortOutOfRange,
        AddressParseError::InvalidIPv4,    AddressParseError::InvalidHostname,
    };

    CHECK(addressParseErrorText(AddressParseError::None).empty());

    for (std::size_t first = 0u; first < errors.size(); ++first)
    {
        CAPTURE(first);
        CHECK_FALSE(addressParseErrorText(errors[first]).empty());
        for (std::size_t second = first + 1u; second < errors.size(); ++second)
            CHECK(std::string(addressParseErrorText(errors[first])) != std::string(addressParseErrorText(errors[second])));
    }
}

TEST_CASE("JoinScreen.ThisPcIsTheLoopbackAddressOnTheDefaultPort",
          "[BrawlerJoinScreen][JoinScreenParse]")
{
    const ServerAddressParse parsed = parseServerAddress(kThisPcAddress);
    REQUIRE(parsed.ok());
    CHECK(parsed.address.host == "127.0.0.1");
    CHECK(parsed.address.port == kDefaultServerPort);
    CHECK(parsed.address.canonical() == std::string(kThisPcAddress));
    CHECK(std::string(kThisPcLabel) == "This PC");
}

TEST_CASE("JoinScreen.TypedCharactersOutsideTheAddressSetAreRefused",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;

    for (const char32_t accepted : std::u32string_view(U"09AZaz.:-"))
        CHECK(buffer.insertTyped(accepted));
    CHECK(buffer.text() == "09AZaz.:-");

    for (const char32_t refused : std::u32string_view(U" /\\?_@,\t\x13A\x12E\xE9"))
    {
        CAPTURE(static_cast<unsigned int>(refused));
        CHECK_FALSE(buffer.insertTyped(refused));
    }
    CHECK(buffer.text() == "09AZaz.:-");
}

TEST_CASE("JoinScreen.AWideCharacterIsTestedBeforeAnyNarrowingSoU013ANeverBecomesAColon",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    CHECK(static_cast<char>(0x13A) == ':');
    CHECK(static_cast<char>(0x12E) == '.');

    AddressEditBuffer buffer;
    CHECK_FALSE(buffer.insertTyped(static_cast<wchar_t>(0x13A)));
    CHECK_FALSE(buffer.insertTyped(static_cast<char16_t>(0x12E)));
    CHECK_FALSE(buffer.insertTyped(static_cast<char16_t>(0xD83D)));
    CHECK(buffer.insertTyped(L'7'));
    CHECK(buffer.text() == "7");
}

TEST_CASE("JoinScreen.TypingInsertsAtTheCursorAndStopsAtTheMaximumLength",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;
    REQUIRE(buffer.assign("1.2.4"));
    CHECK(buffer.cursor() == 5u);

    REQUIRE(buffer.moveCursorLeft());
    REQUIRE(buffer.moveCursorLeft());
    CHECK(buffer.insertTyped(U'3'));
    CHECK(buffer.insertTyped(U'.'));
    CHECK(buffer.text() == "1.23..4");
    CHECK(buffer.cursor() == 5u);
    CHECK(std::string(buffer.textBeforeCursor()) == "1.23.");

    buffer.clear();
    for (std::size_t index = 0u; index < kAddressMaxLength; ++index)
        REQUIRE(buffer.insertTyped(U'a'));
    CHECK_FALSE(buffer.insertTyped(U'a'));
    CHECK(buffer.text().size() == kAddressMaxLength);
}

TEST_CASE("JoinScreen.BackspaceAndDeleteRemoveTheCharacterOnTheirSideOfTheCursor",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;
    REQUIRE(buffer.assign("abcd"));

    CHECK_FALSE(buffer.deleteForward());
    CHECK(buffer.backspace());
    CHECK(buffer.text() == "abc");
    CHECK(buffer.cursor() == 3u);

    REQUIRE(buffer.moveCursorToStart());
    CHECK_FALSE(buffer.backspace());
    CHECK(buffer.deleteForward());
    CHECK(buffer.text() == "bc");
    CHECK(buffer.cursor() == 0u);

    REQUIRE(buffer.moveCursorRight());
    CHECK(buffer.backspace());
    CHECK(buffer.text() == "c");
    CHECK(buffer.cursor() == 0u);
}

TEST_CASE("JoinScreen.TheCursorStaysInsideTheText",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;
    CHECK_FALSE(buffer.moveCursorLeft());
    CHECK_FALSE(buffer.moveCursorRight());
    CHECK_FALSE(buffer.moveCursorToStart());
    CHECK_FALSE(buffer.moveCursorToEnd());

    REQUIRE(buffer.assign("ab"));
    CHECK_FALSE(buffer.moveCursorRight());
    CHECK_FALSE(buffer.moveCursorToEnd());
    CHECK(buffer.moveCursorToStart());
    CHECK_FALSE(buffer.moveCursorLeft());
    CHECK(buffer.moveCursorRight());
    CHECK(buffer.cursor() == 1u);
    CHECK(buffer.moveCursorToEnd());
    CHECK(buffer.cursor() == 2u);
}

TEST_CASE("JoinScreen.APasteIsTrimmedAndInsertedAtTheCursor",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;
    REQUIRE(buffer.assign("host:"));
    CHECK(buffer.insertPasted(std::u16string_view(u"  7777\r\n")) == PasteOutcome::Inserted);
    CHECK(buffer.text() == "host:7777");
    CHECK(buffer.cursor() == 9u);

    REQUIRE(buffer.moveCursorToStart());
    CHECK(buffer.insertPasted(std::wstring_view(L"my-")) == PasteOutcome::Inserted);
    CHECK(buffer.text() == "my-host:7777");
    CHECK(buffer.cursor() == 3u);
}

TEST_CASE("JoinScreen.APasteIsAllOrNothing",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;
    REQUIRE(buffer.assign("ab"));

    CHECK(buffer.insertPasted(std::string_view("   \t")) == PasteOutcome::NothingToPaste);
    CHECK(buffer.insertPasted(std::string_view("")) == PasteOutcome::NothingToPaste);
    CHECK(buffer.insertPasted(std::string_view("1.2.3.4 7777")) == PasteOutcome::InvalidCharacter);
    CHECK(buffer.insertPasted(std::string_view("http://1.2.3.4")) == PasteOutcome::InvalidCharacter);
    CHECK(buffer.insertPasted(std::string_view("caf\xC3\xA9")) == PasteOutcome::InvalidCharacter);
    CHECK(buffer.insertPasted(std::wstring_view(L"a\x13A" L"7777")) == PasteOutcome::InvalidCharacter);
    CHECK(buffer.insertPasted(std::string_view(std::string(kAddressMaxLength - 1u, 'x')))
          == PasteOutcome::TooLong);
    CHECK(buffer.text() == "ab");
    CHECK(buffer.cursor() == 2u);

    CHECK(buffer.insertPasted(std::string_view(std::string(kAddressMaxLength - 2u, 'x')))
          == PasteOutcome::Inserted);
    CHECK(buffer.text().size() == kAddressMaxLength);
}

TEST_CASE("JoinScreen.AssignTrimsAndRefusesTextTheBufferCouldNotHold",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    AddressEditBuffer buffer;
    CHECK(buffer.assign("  10.0.0.2:7000 "));
    CHECK(buffer.text() == "10.0.0.2:7000");
    CHECK(buffer.cursor() == buffer.text().size());

    CHECK_FALSE(buffer.assign("bad address"));
    CHECK_FALSE(buffer.assign(std::string(kAddressMaxLength + 1u, 'a')));
    CHECK(buffer.text() == "10.0.0.2:7000");
}

TEST_CASE("JoinScreen.AsciiTextRefusesAnythingOutsideSevenBits",
          "[BrawlerJoinScreen][JoinScreenEdit]")
{
    CHECK(asciiText(std::wstring_view(L"1.2.3.4:7777")) == std::optional<std::string>("1.2.3.4:7777"));
    CHECK(asciiText(std::wstring_view(L"1.2.3.4\x13A" L"7777")) == std::nullopt);
    CHECK(asciiText(std::string_view("caf\xC3\xA9")) == std::nullopt);
    CHECK(asciiText(std::u16string_view(u"")) == std::optional<std::string>(""));
}

TEST_CASE("JoinScreen.RememberPutsTheAddressFirstInItsCanonicalForm",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    RecentAddressList recents;
    CHECK(recents.remember("10.0.0.1"));
    CHECK(recents.remember("Server.Example:9000"));
    REQUIRE(recents.entries().size() == 2u);
    CHECK(recents.entries()[0] == "server.example:9000");
    CHECK(recents.entries()[1] == "10.0.0.1:7777");
}

TEST_CASE("JoinScreen.RememberingAKnownAddressMovesItToTheFrontWithoutADuplicate",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    RecentAddressList recents = recentsOf({ "10.0.0.1", "10.0.0.2", "10.0.0.3" });

    CHECK(recents.remember("10.0.0.1:7777"));
    CHECK(recents.entries() == std::vector<std::string>{ "10.0.0.1:7777", "10.0.0.3:7777", "10.0.0.2:7777" });

    CHECK_FALSE(recents.remember("10.0.0.1"));
    CHECK(recents.entries().size() == 3u);
}

TEST_CASE("JoinScreen.TheRecentListKeepsTheFiveNewestAndDropsTheOldest",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    RecentAddressList recents =
        recentsOf({ "10.0.0.1", "10.0.0.2", "10.0.0.3", "10.0.0.4", "10.0.0.5", "10.0.0.6" });

    CHECK(kRecentAddressCapacity == 5u);
    CHECK(recents.entries()
          == std::vector<std::string>{ "10.0.0.6:7777", "10.0.0.5:7777", "10.0.0.4:7777", "10.0.0.3:7777",
                                       "10.0.0.2:7777" });
}

TEST_CASE("JoinScreen.ThisPcAndInvalidAddressesAreNeverStored",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    RecentAddressList recents;
    CHECK_FALSE(recents.remember(kThisPcAddress));
    CHECK_FALSE(recents.remember("127.0.0.1"));
    CHECK_FALSE(recents.remember("not an address"));
    CHECK(recents.empty());

    CHECK(recents.remember("127.0.0.1:17777"));
    CHECK(recents.entries() == std::vector<std::string>{ "127.0.0.1:17777" });
}

TEST_CASE("JoinScreen.TheRecentListRoundTripsThroughOneLineOfText",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    const RecentAddressList recents = recentsOf({ "10.0.0.1", "host.example:9000", "10.0.0.3:1" });
    const std::string line = recents.serialize();

    CHECK(line == "10.0.0.3:1,host.example:9000,10.0.0.1:7777");
    CHECK(line.find('\n') == std::string::npos);
    CHECK(RecentAddressList::deserialize(line).entries() == recents.entries());

    CHECK(RecentAddressList().serialize().empty());
    CHECK(RecentAddressList::deserialize("").empty());
}

TEST_CASE("JoinScreen.LoadingAHandEditedLineKeepsOnlyWhatRememberWouldHaveKept",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    const RecentAddressList loaded = RecentAddressList::deserialize(
        " 10.0.0.1 ,,bad address,127.0.0.1:7777,10.0.0.1:7777,HOST.example,10.0.0.2,10.0.0.3,10.0.0.4,10.0.0.5");

    CHECK(loaded.entries()
          == std::vector<std::string>{ "10.0.0.1:7777", "host.example:7777", "10.0.0.2:7777", "10.0.0.3:7777",
                                       "10.0.0.4:7777" });
}

TEST_CASE("JoinScreen.ThisPcIsAlwaysTheFirstListEntryAndTheRecentsFollowNewestFirst",
          "[BrawlerJoinScreen][JoinScreenRecent]")
{
    const std::vector<JoinListEntry> empty = joinListEntries(RecentAddressList());
    REQUIRE(empty.size() == 1u);
    CHECK(empty[0].isThisPc);
    CHECK(empty[0].label == std::string(kThisPcLabel));
    CHECK(empty[0].address == std::string(kThisPcAddress));

    const std::vector<JoinListEntry> full = joinListEntries(
        recentsOf({ "10.0.0.1", "10.0.0.2", "10.0.0.3", "10.0.0.4", "10.0.0.5", "10.0.0.6" }));
    REQUIRE(full.size() == kJoinListMaxEntries);
    CHECK(full[0].isThisPc);
    CHECK_FALSE(full[1].isThisPc);
    CHECK(full[1].address == "10.0.0.6:7777");
    CHECK(full[1].label == full[1].address);
    CHECK(full[5].address == "10.0.0.2:7777");
}

TEST_CASE("JoinScreen.TheCommandLineAddressIsTheFirstTokenThatIsNotASwitch",
          "[BrawlerJoinScreen][JoinScreenCommandLine]")
{
    CHECK(std::string(commandLineMapOverrideToken(std::string_view("127.0.0.1:17777 -game -PIEVIACONSOLE"))) == "127.0.0.1:17777");
    CHECK(std::string(commandLineMapOverrideToken(std::string_view("-log   10.0.0.2 -windowed"))) == "10.0.0.2");
    CHECK(commandLineMapOverrideToken(std::string_view("-game -log")).empty());
    CHECK(commandLineMapOverrideToken(std::string_view("")).empty());
    CHECK(commandLineMapOverrideToken(std::string_view("   \t ")).empty());
    CHECK(std::string(commandLineMapOverrideToken(std::string_view("-log \"10.0.0.3:7000\" x"))) == "10.0.0.3:7000");
    CHECK(std::string(commandLineMapOverrideToken(std::string_view("-ini:\"a b\" 10.0.0.4"))) == "10.0.0.4");
    CHECK(std::string(commandLineMapOverrideToken(std::string_view("-MAP=10.0.0.5 10.0.0.6"))) == "10.0.0.5");
    CHECK(std::wstring(commandLineMapOverrideToken(std::wstring_view(L" -log 10.0.0.7"))) == L"10.0.0.7");
}

TEST_CASE("JoinScreen.WithNoHistoryThisPcIsFocusedAndOnePressJoinsIt",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "", false);
    CHECK(model.phase() == JoinPhase::Editing);
    CHECK(model.focus().area == JoinFocusArea::List);
    CHECK(model.focus().listIndex == 0u);
    CHECK(model.field().text().empty());

    const std::optional<std::string> target = model.activate();
    REQUIRE(target.has_value());
    CHECK(*target == std::string(kThisPcAddress));
    CHECK(model.phase() == JoinPhase::Connecting);
    CHECK(model.targetAddress() == std::string(kThisPcAddress));
    CHECK(model.field().text() == std::string(kThisPcAddress));
}

TEST_CASE("JoinScreen.WithHistoryTheFieldHoldsTheNewestAddressAndHasFocus",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(recentsOf({ "10.0.0.1", "10.0.0.2" }), "", false);
    CHECK(model.phase() == JoinPhase::Editing);
    CHECK(model.focus().area == JoinFocusArea::Field);
    CHECK(model.field().text() == "10.0.0.2:7777");
    CHECK(model.field().cursor() == model.field().text().size());

    const std::optional<std::string> target = model.activate();
    REQUIRE(target.has_value());
    CHECK(*target == "10.0.0.2:7777");
}

TEST_CASE("JoinScreen.ACommandLineAddressStartsConnectingAtOnce",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    const JoinScreenModel model = JoinScreenModel::start(recentsOf({ "10.0.0.1" }), " Host.Example:9000 ", false);
    CHECK(model.phase() == JoinPhase::Connecting);
    CHECK(model.targetAddress() == "host.example:9000");
    CHECK(model.field().text() == "host.example:9000");
    CHECK(model.statusLine("dev").tone == JoinStatusTone::Progress);
    CHECK(model.statusLine("dev").text.headline == "Connecting to host.example:9000...");
}

TEST_CASE("JoinScreen.AfterAFailureReturnTheCommandLineAddressIsIgnored",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    const JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "10.0.0.9", true);
    CHECK(model.phase() == JoinPhase::Editing);
    CHECK(model.targetAddress().empty());
    CHECK(model.field().text().empty());
    CHECK(model.notice().empty());
}

TEST_CASE("JoinScreen.AnInvalidCommandLineAddressLeavesTheScreenEditingWithANotice",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    const JoinScreenModel model =
        JoinScreenModel::start(recentsOf({ "10.0.0.1" }), "/Game/ThirdPerson/Maps/ThirdPersonMap", false);
    CHECK(model.phase() == JoinPhase::Editing);
    CHECK(model.field().text() == "10.0.0.1:7777");
    CHECK_FALSE(model.notice().empty());

    const JoinStatusLine status = model.statusLine("dev");
    CHECK(status.tone == JoinStatusTone::Prompt);
    CHECK(status.text.headline == std::string(kEditingPrompt));
    CHECK(status.text.detail == model.notice());
}

TEST_CASE("JoinScreen.AnInvalidFieldIsRefusedAndTheErrorStaysUntilTheNextEdit",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "", false);
    REQUIRE(model.typeCharacter(U'1'));
    REQUIRE(model.typeCharacter(U':'));

    CHECK_FALSE(model.activate().has_value());
    CHECK(model.phase() == JoinPhase::Editing);
    CHECK(model.rejectedFieldError() == AddressParseError::MissingPort);

    const JoinStatusLine refused = model.statusLine("dev");
    CHECK(refused.tone == JoinStatusTone::Error);
    CHECK(refused.text.headline == std::string(addressParseErrorText(AddressParseError::MissingPort)));

    REQUIRE(model.backspace());
    CHECK(model.rejectedFieldError() == AddressParseError::None);
    CHECK(model.statusLine("dev").tone == JoinStatusTone::Prompt);
}

TEST_CASE("JoinScreen.WhileConnectingNoInputChangesAnything",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    REQUIRE(model.phase() == JoinPhase::Connecting);

    CHECK_FALSE(model.acceptsInput());
    CHECK_FALSE(model.typeCharacter(U'5'));
    CHECK(model.paste(std::string_view("5")) == PasteOutcome::NothingToPaste);
    CHECK_FALSE(model.backspace());
    CHECK_FALSE(model.deleteForward());
    CHECK_FALSE(model.moveCursorLeft());
    CHECK_FALSE(model.navigate(JoinNavigation::Up));
    CHECK_FALSE(model.activate().has_value());
    CHECK_FALSE(model.requestJoin("10.0.0.2").has_value());
    CHECK(model.field().text() == "10.0.0.1:7777");
    CHECK(model.targetAddress() == "10.0.0.1:7777");
}

TEST_CASE("JoinScreen.CancelReturnsAConnectingScreenToEditingAndALateFailureIsIgnored",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "", false);
    CHECK_FALSE(model.cancel());

    REQUIRE(model.requestJoin("10.0.0.1").has_value());
    CHECK(model.cancel());
    CHECK(model.phase() == JoinPhase::Editing);

    CHECK_FALSE(model.noteJoinFailed(JoinFailureReason::CannotReachServer, ""));
    CHECK(model.phase() == JoinPhase::Editing);
}

TEST_CASE("JoinScreen.ASuccessfulJoinIsRememberedOnlyWhenTheCallerAllowsIt",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel remembered = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    CHECK(remembered.noteJoinSucceeded(true));
    CHECK(remembered.phase() == JoinPhase::Joined);
    CHECK(remembered.recents().entries() == std::vector<std::string>{ "10.0.0.1:7777" });
    CHECK_FALSE(remembered.noteJoinSucceeded(true));

    JoinScreenModel editorSession = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    CHECK_FALSE(editorSession.noteJoinSucceeded(false));
    CHECK(editorSession.phase() == JoinPhase::Joined);
    CHECK(editorSession.recents().empty());

    JoinScreenModel thisPc = JoinScreenModel::start(RecentAddressList(), "", false);
    REQUIRE(thisPc.activate().has_value());
    CHECK_FALSE(thisPc.noteJoinSucceeded(true));
    CHECK(thisPc.phase() == JoinPhase::Joined);
    CHECK(thisPc.recents().empty());

    JoinScreenModel idle = JoinScreenModel::start(RecentAddressList(), "", false);
    CHECK_FALSE(idle.noteJoinSucceeded(true));
    CHECK(idle.phase() == JoinPhase::Editing);
}

TEST_CASE("JoinScreen.TheFirstFailureOfAnAttemptIsTheOneShown",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);

    CHECK(model.noteJoinFailed(JoinFailureReason::DifferentBuild, "OutdatedClient"));
    CHECK_FALSE(model.noteJoinFailed(JoinFailureReason::CannotReachServer, "PendingConnectionFailure"));
    CHECK(model.phase() == JoinPhase::Failed);
    CHECK(model.failureReason() == JoinFailureReason::DifferentBuild);
    CHECK(model.failureServerText() == "OutdatedClient");
}

TEST_CASE("JoinScreen.AConnectionLostWhilePlayingLandsOnTheFailedScreen",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    REQUIRE(model.noteJoinSucceeded(true));

    CHECK(model.noteJoinFailed(JoinFailureReason::ConnectionLost, ""));
    CHECK(model.phase() == JoinPhase::Failed);
    CHECK(model.statusLine("dev").tone == JoinStatusTone::Error);
    CHECK(model.statusLine("dev").text.headline == "Connection lost.");
}

TEST_CASE("JoinScreen.AFailureNobodyIsWaitingForIsIgnored",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "", false);
    CHECK_FALSE(model.noteJoinFailed(JoinFailureReason::Unknown, "stray"));
    CHECK(model.phase() == JoinPhase::Editing);
}

TEST_CASE("JoinScreen.FromTheFailedScreenAnEditReturnsToEditingAndAJoinRetries",
          "[BrawlerJoinScreen][JoinScreenState]")
{
    JoinScreenModel edited = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    REQUIRE(edited.noteJoinFailed(JoinFailureReason::CannotReachServer, ""));
    CHECK(edited.acceptsInput());
    CHECK(edited.backspace());
    CHECK(edited.phase() == JoinPhase::Editing);
    CHECK(edited.field().text() == "10.0.0.1:777");

    JoinScreenModel retried = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    REQUIRE(retried.noteJoinFailed(JoinFailureReason::CannotReachServer, ""));
    const std::optional<std::string> target = retried.activate();
    REQUIRE(target.has_value());
    CHECK(*target == "10.0.0.1:7777");
    CHECK(retried.phase() == JoinPhase::Connecting);
    CHECK(retried.failureServerText().empty());

    JoinScreenModel navigated = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    REQUIRE(navigated.noteJoinFailed(JoinFailureReason::CannotReachServer, ""));
    CHECK(navigated.navigate(JoinNavigation::Up));
    CHECK(navigated.phase() == JoinPhase::Failed);
}

TEST_CASE("JoinScreen.EachFailureReasonHasItsOwnHeadline",
          "[BrawlerJoinScreen][JoinScreenText]")
{
    for (std::size_t first = 0u; first < kJoinFailureReasonCount; ++first)
    {
        const JoinStatusText firstText =
            joinFailureText(static_cast<JoinFailureReason>(first), "dev", "", "10.0.0.1:7777");
        CAPTURE(first);
        CHECK_FALSE(firstText.headline.empty());

        for (std::size_t second = first + 1u; second < kJoinFailureReasonCount; ++second)
        {
            const JoinStatusText secondText =
                joinFailureText(static_cast<JoinFailureReason>(second), "dev", "", "10.0.0.1:7777");
            CHECK(firstText.headline != secondText.headline);
        }
    }
}

TEST_CASE("JoinScreen.TheFailureTextsSayWhatTheBacklogAsksFor",
          "[BrawlerJoinScreen][JoinScreenText]")
{
    const JoinStatusText unreachable =
        joinFailureText(JoinFailureReason::CannotReachServer, "dev", "", "10.0.0.1:7777");
    CHECK(unreachable.headline == "Can't reach server 10.0.0.1:7777.");

    const JoinStatusText differentBuild =
        joinFailureText(JoinFailureReason::DifferentBuild, "20260929-101500-abc1234", "", "10.0.0.1:7777");
    CHECK(differentBuild.headline == "Different build.");
    CHECK(differentBuild.detail == "This game is 20260929-101500-abc1234; the server runs a different build.");

    CHECK(joinFailureText(JoinFailureReason::DifferentBuild, "", "", "").detail
          == "This game is dev; the server runs a different build.");

    CHECK(joinFailureText(JoinFailureReason::ConnectionLost, "dev", "", "").headline == "Connection lost.");

    CHECK(joinFailureText(JoinFailureReason::ServerRefused, "dev", "Server full.", "").headline
          == "Server refused: Server full.");
    CHECK(joinFailureText(JoinFailureReason::ServerRefused, "dev", "", "").headline
          == "Server refused: no reason given");

    CHECK(joinFailureText(JoinFailureReason::Unknown, "dev", "NetGuidMismatch", "").detail == "NetGuidMismatch");
    CHECK(joinFailureText(JoinFailureReason::Unknown, "dev", "", "").detail == "Unknown error.");
}

TEST_CASE("JoinScreen.TheFailedStatusLineUsesTheAttemptsAddressAndTheServersText",
          "[BrawlerJoinScreen][JoinScreenText]")
{
    JoinScreenModel model = JoinScreenModel::start(RecentAddressList(), "10.0.0.1", false);
    REQUIRE(model.noteJoinFailed(JoinFailureReason::ServerRefused, "Maximum splitscreen players"));

    const JoinStatusLine status = model.statusLine("dev");
    CHECK(status.tone == JoinStatusTone::Error);
    CHECK(status.text.headline == "Server refused: Maximum splitscreen players");
    CHECK(joinStatusInk(status.tone).r == kJoinScreenErrorInk.r);
}

TEST_CASE("JoinScreen.TheLocalCoopHintNamesTheBoundKeysAndStatesNoLimit",
          "[BrawlerJoinScreen][JoinScreenText]")
{
    CHECK(localCoopHintText() == "After joining: Tab adds a local player, End removes one");
    CHECK(std::string(kLocalCoopKeyNames.addPlayer) == "Tab");
    CHECK(std::string(kLocalCoopKeyNames.removePlayer) == "End");
}

TEST_CASE("JoinScreen.TheLocalPlayerLimitNoticeStatesTheLimitAndFadesAfterItsWindow",
          "[BrawlerJoinScreen][JoinScreenText]")
{
    CHECK(localPlayerLimitNoticeText(4) == "No local player added: this PC already has 4, the most one PC can have.");

    CHECK_FALSE(localPlayerLimitNoticeVisible(-0.1f));
    CHECK(localPlayerLimitNoticeVisible(0.f));
    CHECK(localPlayerLimitNoticeVisible(kLocalPlayerLimitNoticeSeconds - 0.01f));
    CHECK_FALSE(localPlayerLimitNoticeVisible(kLocalPlayerLimitNoticeSeconds));
    CHECK_FALSE(localPlayerLimitNoticeVisible(std::numeric_limits<float>::quiet_NaN()));
}

TEST_CASE("JoinScreen.TheBuildLineShowsDevWhenThereIsNoLabel",
          "[BrawlerJoinScreen][JoinScreenText]")
{
    CHECK(buildLabelLine("") == "Build: dev");
    CHECK(buildLabelLine("20260929-101500-abc1234") == "Build: 20260929-101500-abc1234");
}

TEST_CASE("JoinScreen.FocusWalksListThenFieldThenJoinAndStopsAtBothEnds",
          "[BrawlerJoinScreen][JoinScreenFocus]")
{
    constexpr std::size_t kListCount = 3u;
    JoinFocus focus{ JoinFocusArea::List, 0u };

    focus = navigatedFocus(focus, JoinNavigation::Up, kListCount);
    CHECK(focus.area == JoinFocusArea::List);
    CHECK(focus.listIndex == 0u);

    focus = navigatedFocus(focus, JoinNavigation::Down, kListCount);
    CHECK(focus.listIndex == 1u);
    focus = navigatedFocus(focus, JoinNavigation::Down, kListCount);
    CHECK(focus.listIndex == 2u);
    focus = navigatedFocus(focus, JoinNavigation::Down, kListCount);
    CHECK(focus.area == JoinFocusArea::Field);
    focus = navigatedFocus(focus, JoinNavigation::Down, kListCount);
    CHECK(focus.area == JoinFocusArea::JoinButton);
    focus = navigatedFocus(focus, JoinNavigation::Down, kListCount);
    CHECK(focus.area == JoinFocusArea::JoinButton);

    focus = navigatedFocus(focus, JoinNavigation::Up, kListCount);
    CHECK(focus.area == JoinFocusArea::Field);
    focus = navigatedFocus(focus, JoinNavigation::Up, kListCount);
    CHECK(focus.area == JoinFocusArea::List);
    CHECK(focus.listIndex == kListCount - 1u);
}

TEST_CASE("JoinScreen.AListIndexPastTheEndIsPulledBackToTheLastEntry",
          "[BrawlerJoinScreen][JoinScreenFocus]")
{
    const JoinFocus stale{ JoinFocusArea::List, 9u };
    const JoinFocus up = navigatedFocus(stale, JoinNavigation::Up, 2u);
    CHECK(up.area == JoinFocusArea::List);
    CHECK(up.listIndex == 0u);

    const JoinFocus down = navigatedFocus(stale, JoinNavigation::Down, 2u);
    CHECK(down.area == JoinFocusArea::Field);
    CHECK(down.listIndex == 1u);
}

TEST_CASE("JoinScreen.TypingMovesFocusToTheFieldAndPickingAnEntryCopiesItThere",
          "[BrawlerJoinScreen][JoinScreenFocus]")
{
    JoinScreenModel model = JoinScreenModel::start(recentsOf({ "10.0.0.1", "10.0.0.2" }), "", false);
    REQUIRE(model.navigate(JoinNavigation::Up));
    CHECK(model.focus().area == JoinFocusArea::List);
    CHECK(model.focus().listIndex == 2u);

    REQUIRE(model.navigate(JoinNavigation::Up));
    const std::optional<std::string> target = model.activate();
    REQUIRE(target.has_value());
    CHECK(*target == "10.0.0.2:7777");
    CHECK(model.field().text() == "10.0.0.2:7777");

    JoinScreenModel typed = JoinScreenModel::start(RecentAddressList(), "", false);
    REQUIRE(typed.focus().area == JoinFocusArea::List);
    CHECK(typed.typeCharacter(U'9'));
    CHECK(typed.focus().area == JoinFocusArea::Field);

    JoinScreenModel viaButton = JoinScreenModel::start(recentsOf({ "10.0.0.1" }), "", false);
    REQUIRE(viaButton.navigate(JoinNavigation::Down));
    CHECK(viaButton.focus().area == JoinFocusArea::JoinButton);
    const std::optional<std::string> fromButton = viaButton.activate();
    REQUIRE(fromButton.has_value());
    CHECK(*fromButton == "10.0.0.1:7777");
}

TEST_CASE("JoinScreen.TheScaleClampPullsEveryValueIntoRangeIncludingNaN",
          "[BrawlerJoinScreen][JoinScreenLayout]")
{
    CHECK(clampJoinScreenScale(kJoinScreenDefaultScale) == kJoinScreenDefaultScale);
    CHECK(clampJoinScreenScale(2.f) == 2.f);
    CHECK(clampJoinScreenScale(0.1f) == kJoinScreenMinScale);
    CHECK(clampJoinScreenScale(-3.f) == kJoinScreenMinScale);
    CHECK(clampJoinScreenScale(100.f) == kJoinScreenMaxScale);
    CHECK(clampJoinScreenScale(std::numeric_limits<float>::infinity()) == kJoinScreenMaxScale);
    CHECK(clampJoinScreenScale(std::numeric_limits<float>::quiet_NaN()) == kJoinScreenMinScale);
}

TEST_CASE("JoinScreen.ThePanelGrowsWithTheCanvasHeightAndIsCentred",
          "[BrawlerJoinScreen][JoinScreenLayout]")
{
    const JoinScreenLayout at720 = placedJoinScreenLayout(3u, 1.f, 1280.f, 720.f);
    CHECK(at720.scale == Catch::Approx(1.f));

    const JoinScreenLayout at1080 = placedJoinScreenLayout(3u, 1.f, 1920.f, 1080.f);
    CHECK(at1080.scale == Catch::Approx(1.5f));
    CHECK(at1080.titleTextScale == Catch::Approx(1.5f * kJoinScreenTitleTextScale));
    CHECK(at1080.panel.width == Catch::Approx(JoinScreenMetrics{}.panelWidth * 1.5f));
    CHECK(at1080.panel.x + at1080.panel.width * 0.5f == Catch::Approx(960.f));
    CHECK(at1080.panel.y + at1080.panel.height * 0.5f == Catch::Approx(540.f));

    const JoinScreenLayout doubled = placedJoinScreenLayout(3u, 2.f, 1920.f, 1080.f);
    CHECK(doubled.scale == Catch::Approx(3.f));
}

TEST_CASE("JoinScreen.ThePanelShrinksToFitACanvasTooSmallForTheRequestedScale",
          "[BrawlerJoinScreen][JoinScreenLayout]")
{
    const JoinScreenMetrics metrics{};

    const JoinScreenLayout tiny = placedJoinScreenLayout(kJoinListMaxEntries, 1.f, 320.f, 180.f);
    const JoinScreenRect canvas{ 0.f, 0.f, 320.f, 180.f };
    CHECK(tiny.scale < 1.f);
    CHECK(tiny.scale < kJoinScreenMinScale);
    CHECK(rectInside(tiny.panel, canvas));

    const JoinScreenLayout huge = placedJoinScreenLayout(kJoinListMaxEntries, kJoinScreenMaxScale, 3840.f, 2160.f);
    const JoinScreenRect canvas4k{ 0.f, 0.f, 3840.f, 2160.f };
    CHECK(rectInside(huge.panel, canvas4k));
    CHECK(huge.panel.height == Catch::Approx(2160.f));
    CHECK(huge.scale == Catch::Approx(2160.f / joinScreenBaseHeight(metrics, kJoinListMaxEntries)));

    const JoinScreenLayout narrow = placedJoinScreenLayout(1u, 1.f, 400.f, 1080.f);
    CHECK(narrow.panel.width == Catch::Approx(400.f));
    CHECK(narrow.panel.x == Catch::Approx(0.f));
}

TEST_CASE("JoinScreen.AnUnsizedCanvasStillGivesAFiniteLayout",
          "[BrawlerJoinScreen][JoinScreenLayout]")
{
    for (const float side : { 0.f, -5.f, std::numeric_limits<float>::quiet_NaN() })
    {
        const JoinScreenLayout layout = placedJoinScreenLayout(2u, 1.f, side, side);
        CAPTURE(side);
        CHECK(layout.scale == kJoinScreenDefaultScale);
        CHECK(rectIsFinite(layout.panel));
        CHECK(rectIsFinite(layout.hint));
        CHECK(layout.panel.x == 0.f);
        CHECK(layout.panel.y == 0.f);
    }
}

TEST_CASE("JoinScreen.TheRowsStackTopToBottomInsideThePanelInFocusOrder",
          "[BrawlerJoinScreen][JoinScreenLayout]")
{
    const JoinScreenLayout layout = placedJoinScreenLayout(4u, 1.f, 1920.f, 1080.f);
    REQUIRE(layout.listRowCount == 4u);

    std::vector<JoinScreenRect> stack{ layout.title, layout.buildLabel };
    for (std::size_t row = 0u; row < layout.listRowCount; ++row)
        stack.push_back(layout.listRows[row]);
    stack.push_back(layout.field);
    stack.push_back(layout.joinButton);
    stack.push_back(layout.statusHeadline);
    stack.push_back(layout.statusDetail);
    stack.push_back(layout.hint);

    for (std::size_t index = 0u; index < stack.size(); ++index)
    {
        CAPTURE(index);
        CHECK(rectInside(stack[index], layout.panel));
        CHECK(stack[index].height > 0.f);
        if (index > 0u)
            CHECK(stack[index].y >= stack[index - 1u].y + stack[index - 1u].height - 1e-3f);
    }

    for (std::size_t row = 1u; row < layout.listRowCount; ++row)
        CHECK(layout.listRows[row].y == Catch::Approx(layout.listRows[row - 1u].y + layout.listRows[row - 1u].height));

    CHECK(layout.hint.y + layout.hint.height == Catch::Approx(layout.panel.y + layout.panel.height
                                                              - JoinScreenMetrics{}.padding * layout.scale));
}

TEST_CASE("JoinScreen.TheListRowCountIsCappedAtTheLongestPossibleList",
          "[BrawlerJoinScreen][JoinScreenLayout]")
{
    const JoinScreenMetrics metrics{};
    CHECK(placedJoinScreenLayout(99u, 1.f, 1280.f, 720.f).listRowCount == kJoinListMaxEntries);
    CHECK(joinScreenBaseHeight(metrics, 99u) == joinScreenBaseHeight(metrics, kJoinListMaxEntries));
    CHECK(joinScreenBaseHeight(metrics, 2u) - joinScreenBaseHeight(metrics, 1u) == Catch::Approx(metrics.rowHeight));
    CHECK(placedJoinScreenLayout(0u, 1.f, 1280.f, 720.f).listRowCount == 0u);
}

} // namespace joinscreentests

#endif // WITH_LOW_LEVEL_TESTS
