<!-- SPDX-License-Identifier: BUSL-1.1 -->
# RoundVsPacketBudgetTest — rationale

Source: `RoundVsPacketBudgetTest.cpp`. This file has no guards document: nothing in it carries a `⛔` tag.

## 1. What this file is for

It prices the brawler's **correction round**, the corrections the server sends one client in one flush, against the budget
og-sim-net gives that traffic: `ogsimnet::correctionAllowanceBytes(lanes, budget, tickFrequency)` in og-sim-net's
`SimNetServer.h`. That is og-net's SimState lane's class share of the connection budget, per tick (og-sim-net's
`SimNetServer-rationale.md` §10). The Unreal server session constructs its `SimNetServer` with it, and the server counts a
receiver's flush whose selected bytes exceed it as `overBudget` (the `[SimNet.Window]` line's `overBudget=`). The shipped
policy, `SendAllDirty`, sends every dirty character's correction anyway, so a round over the allowance is counted, not cut.

Tags: `[PacketBudget]` puts every case in the `[@og]` whitelist; `[SimNetServer]` selects them with og-sim-net's server.

## 2. The cases

- **`PacketBudget.ABrawlerCorrectionIsTheSizeTheServerPricesIt`.** A default `simulatableBrawler::State` written into a
  `CorrectionStateBuffer` and encoded by `simEnvelope::writeCorrectionState` is 337 B: the 4 B envelope (version, kind, sim
  id, correction version), the correction codec's 8 B header and the 325 B state composite (`SimPayloadBuffersBrawlerTest.cpp`
  pins the 333 B payload). The case also checks that this equals the size `SimNetServer::flushCorrections` gives the policy as
  the candidate's `bytes` (`simEnvelope::kHeaderBytes + 2 + payload`), so the budget is compared with what is sent.
- **`PacketBudget.AtTheShippedBudgetEveryAdmittedCharactersCorrectionFitsOneFlush`.** At the shipped budget, 1,250,000 B/s, the
  allowance at 60 Hz is 6,250 B: 18 brawler corrections. The server refuses the login after `hardPlayerCeiling` (8) players
  (`SessionConstants.h`), and a full round of 8 corrections is 2,696 B, so a session the server admits never counts
  `overBudget` on correction size alone.
- **`PacketBudget.AtOgNetsFloorBudgetTheTier1RoundFitsAndTheFenceBitesOneCharacterLater`.** At og-net's floor,
  `ognet::lanes::kDefaultBudget` (250,000 B/s), the allowance is 1,250 B (og-sim-net's `SimNetServerTest.cpp` pins the same
  number): 3 corrections, exactly Tier 1's `maxPlayersPerServer`. A fourth character (1,348 B) does not fit, and neither does
  Tier 2's round of 6 (2,022 B). These two are the negative controls: they show the comparison can fail.

The tick rate is `TimeConfig{}.tickFrequency` (60 Hz). The Unreal session passes its `NetConfig::tickFrequencyHz` instead,
which the game module's `UEBrawlerNetConfigConceptTest.cpp` pins at the same 60.

## 3. Why the floor, and the mirrored shipped budget

The budget the Unreal driver gives og-net is `budgetFor` (OGNetUnreal's `EngineLaneSettings.h`): the largest of the engine's
four rate settings plus headroom, clamped from below to `kDefaultBudget`. With `Config/DefaultEngine.ini`'s rates it is
1,250,000 B/s, and a `static_assert` there pins that value for those four rates. This target cannot include OGNetUnreal (it
links og-net and og-sim-net, not the Unreal plugin), so the file mirrors the value as a literal. A `static_assert` beside it
checks that the mirrored budget is valid and above the floor. Neither the literal nor that assertion reads the ini: a change
of the project's rates re-prices both, in the same change.

The floor case holds whatever the ini says: `budgetFor` never returns less than `kDefaultBudget`, so the floor allowance is the
least any Unreal configuration gives the correction round.

## 4. What it does not price

- **The relay rings.** They ride og-net's SimRelay lane, a different class with its own share. og-sim-net has no budget
  function for that lane and does not count a relay round against one, so there is nothing to test against. A full relay ring
  fitting one inbox record is `SimPayloadBuffersBrawlerTest.cpp`'s.
- **og-net's datagram packing and queue age.** The allowance is a per-tick share of the byte budget; how og-net packs messages
  into datagrams and when it drops an aged SimState message are og-net's (its lane scheduler and queue age), not this file's.

## 5. Provenance (og-input-latency task P5-26)

Until og-input-latency task P5-26 this file priced a different round: every remote character's relay ring plus one correction
state against Iris's single-bunch capacity (952 B, from `UNetConnection::GetMaxSingleBunchSizeBits`), with Iris's per-object and
per-packet framing (og-netcode-v2-input-relay T39, after T37's depth regression). That traffic has ridden og-net's sim lanes
since og-input-latency task P5-7, and the relay actors and the Iris buffers it priced were deleted by P5-9, so the old round
priced nothing that is sent. P5-10's finding F1 asked for the file to be retired or re-based; the lead's ruling was to re-base
it on og-sim-net's per-connection sim-lane budget if a budget function exists, and `correctionAllowanceBytes` is that function.
The file name and the `[PacketBudget]` tag are kept, so the references to it in `SimulatableBrawlerTest.cpp`'s state-footprint
assertion ("re-price RoundVsPacketBudgetTest.cpp") still hold: a state footprint change moves the 337 B correction.
