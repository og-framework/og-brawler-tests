<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerJoinScreenTest — rationale

Source: `Visualization/BrawlerJoinScreenTest.cpp`. It pins `OGBrawler/BrawlerJoinScreen.h`, the
pure model of the join screen; the reasoning behind each rule is in that header's rationale
document in the og-brawler submodule (`OGBrawler/docs/BrawlerJoinScreen-rationale.md`). This file
has no guards document: the test file carries no tags.

## 1. Tags, and why there is a new top-level one

Every case carries `[BrawlerJoinScreen]` plus one sub-tag for its group: `[JoinScreenParse]`,
`[JoinScreenEdit]`, `[JoinScreenRecent]`, `[JoinScreenCommandLine]`, `[JoinScreenState]`,
`[JoinScreenText]`, `[JoinScreenFocus]`, `[JoinScreenLayout]`.

`[@og]` in `OgTagAliases.cpp` is a whitelist, so `[BrawlerJoinScreen]` was appended to it in the
same change as the cases. The join screen is not a character visualization, so riding in on
`[CharacterViz]` (as the scoreboard does) would have filed the cases under the wrong heading.

**Measured 2026-09-29** on one build of this file:

| run | cases | assertions |
|---|---|---|
| `[@og]` with the file compiled in, before the alias append | 616 | 14467 |
| `[BrawlerJoinScreen]` directly | 52 | 556 |
| `[@og]` after the alias append (and `oglltest brawler`) | 668 | 15023 |

The difference is exactly the direct run (+52 cases, +556 assertions), and 668 equals the number
of `TEST_CASE` definitions in the module, the completeness check the alias file asks for.

## 2. What each group proves

* **Parse** — one input per `AddressParseError` value (several for the IPv4 and host-name rules),
  the port boundaries 1 and 65535, lowercasing, trimming, "This PC" parsing to itself, and every
  error having its own text.
* **Edit** — the allowed set, the maximum length, the cursor at both ends, backspace and delete,
  paste (trimmed, at the cursor, all or nothing), `assign`, `asciiText`, and the wide-character
  rule with the two narrowings it exists for (U+013A → `:`, U+012E → `.`).
* **Recent** — front insertion, move-to-front dedupe on canonical form, the cap of five, "This PC"
  and invalid text never stored, the one-line round trip, and a hand-edited line loading as
  `remember` would have built it.
* **Command line** — the first non-switch token, quoting, `-map=`, wide strings.
* **State** — every transition in the rationale's §7 table, including the start rules (history or
  not, a command-line address, one ignored after a failure return, an invalid one), the latch on
  the first failure, cancel followed by a late failure, and success with and without remembering.
* **Text** — each failure reason's headline distinct, the exact texts the backlog asks for, the
  local co-op hint, the limit notice and its time window, the build line.
* **Focus** — the list → field → Join chain, both ends, a stale index, typing moving focus, and
  activation from each area.
* **Layout** — the clamp (NaN, infinity, both ends), resolution scaling and centring, fit on a
  small canvas and at the maximum scale, an unsized canvas, the stacking order, and the row cap.

## 3. Poison arms, measured 2026-09-29

Each was applied, seen to fail, and restored (the file's md5 was compared before and after).

| poison | result |
|---|---|
| a test line `buffer.insertTyped(static_cast<char>(0x13A))` | build fails, C2338 with the wide-character `static_assert` message |
| `kRecentAddressSeparator` changed to `'.'` in the header | build fails, C2338 with the separator `static_assert` message |
| `noteJoinFailed` accepting failures in every phase except `Editing` | `JoinScreen.TheFirstFailureOfAnAttemptIsTheOneShown` fails (3 checks) |

## 4. Two traps met while writing the file

* **`std::string_view` in a `CHECK` does not link.** The Catch2 amalgamation built into this target
  does not define the `string_view` string maker, so `CHECK(view == "x")` compiles and then fails at
  link with `LNK2019 ... StringMaker<std::basic_string_view<...>>::convert`. Every comparison of a
  view converts it to `std::string` (or `std::wstring`) first.
* **The port is checked before the host.** `1:` is `MissingPort`, not `InvalidIPv4`; the case was
  first written the other way and failed.
