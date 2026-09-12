// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>

#include "catch_amalgamated.hpp"
#include "OGBrawler/SessionConstants.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/RelayedInputRingCodec.h"

// ---------------------------------------------------------------------------
// ⭐ [og-netcode-v2-input-relay / T39] THE ROUND-VS-PACKET BUDGET.
//
// WHY THIS FILE EXISTS. Every test in this repository priced a PAYLOAD. None
// priced the ROUND — the whole set of bytes one server->client connection tries
// to ship in one frame — and that omission is the entire content of T37:
// raising the relay ring's depth from 1 to 2 pushed the two-character round past
// the single-packet bunch capacity, Iris started skipping one character's whole
// batch on two frames out of three, and the symptom arrived as *fewer* bytes on
// the wire and a ⅓ cadence collapse on every snapshot channel simultaneously.
// The change had passed every payload-level suite in all four repositories.
// (finding_task37_depth_regression.md §7.1 asked for exactly this test.)
//
// WHAT IS ASSERTED — the INPUT GUARANTEE plus STATE LIVENESS, at the product
// target of six characters (design_task38_input_first_replication.md §13):
//
//     (N-1) x (ringWireBytes(depth) + perBatchOverhead)
//   +   1   x (stateWireBytes        + perBatchOverhead)
//   +          perPacketOverhead
//   <= usableSingleBunchBytes
//
// Read it as: on any connection, ALL the remote characters' relay rings at their
// worst size, PLUS at least one correction state, must always fit one packet.
//
//   * N-1 rings, not N: the owning client is not sent its own character's ring
//     (T39 moves it onto ASimulationInputRelay behind COND_SkipOwner; the owning
//     client provably never reads it — SimulationNetSync::registerPredictionOwner
//     creates a RemoteInputCache only for provider-ABSENT ids).
//   * ONE state, not K: the correction state is the SELF-HEALING payload — each
//     snapshot is a complete anchor, so a skipped one costs repair latency, not
//     repair ability. The rotation's K is therefore OPPORTUNISTIC above this
//     line and deliberately NOT asserted: pre-wire-diet, K=2 does NOT fit at six
//     characters, and that is designed degradation (states sort last at static
//     priority 1.0 against the ring's 4.0, get skipped, and rotate back via
//     Iris's priority accumulation), not a defect. Asserting K here would make
//     this test fail for a condition the design chose.
//   * The rings are what must never be displaced: a dropped relayed input has no
//     recovery path anywhere in the system.
//
// THE JOB OF THIS TEST, stated so nobody weakens it by accident: it MUST go red
// if anyone raises the compiled relay depth, or grows either payload past the
// packet. Both payload sizes below are computed from the CODECS' OWN constants
// against the REAL `simulatableBrawler` types, so a wire change anywhere
// upstream moves this arithmetic without anyone remembering to update a number.
//
// WHAT IT DOES NOT AND CANNOT CATCH: [og-netcode-v2-input-relay item 63 / RN-13,
// 2026-08-16] there is no longer an ini-configurable ring depth to worry about
// here — that knob is retired (its old identifier is on record in RN-13,
// ReviewNotes.md), and the flush stage's capacity is the compile-time constant
// `relayedInputRing::kMaxDepth` (see §4's fence below). The runtime arm of the
// packet-budget guard is the `[RelayProbe.Budget]` PIE gate (`outPackets ==
// frames`, `bytesPerPacket <= capacity - margin`), which fails in its first
// steady window — this test covers the COMPILED sizes, which are the values a
// wire-layout code change can move silently.
// ---------------------------------------------------------------------------

namespace
{
    // -----------------------------------------------------------------------
    // MIRRORED ENGINE CONSTANTS.
    //
    // This target cannot include engine headers, so each of these is a pinned
    // literal naming the engine symbol it mirrors — the pattern
    // `FRelayedInputRing::static_assert(kMaxWireBytes == 1066, ...)` established.
    // -----------------------------------------------------------------------

    // `UNetConnection::GetMaxSingleBunchSizeBits()` (NetConnection.h) =
    //   MaxPacket*8 - MAX_BUNCH_HEADER_BITS(256) - MAX_PACKET_TRAILER_BITS(1)
    //                - MAX_PACKET_HEADER_BITS(308) - MaxPacketHandlerBits
    // At MAX_PACKET_SIZE = 1024 (CoreNet.h; this project sets no override) and a
    // zero-reservation handler that is 7,627 bits, which
    // `UDataStreamChannel::WriteData` word-rounds to (7627/32)*4 = 952 bytes.
    //
    // ⚠ THIS IS AN UPPER BOUND, and the running engine is the referee. A nonzero
    // `MaxPacketHandlerBits` (encryption, packet handlers) can only LOWER it. The
    // authority logs the real pair once per session as
    //   [PacketBudget] usableSingleBunchBytes=%d handlerBits=%d
    // (Warning-level, one-shot, first client connection — SimulationManagerUImpl).
    // If that line ever reports a value BELOW this literal, this literal is
    // optimistic and must be lowered here, in the same change that observes it.
    // T37 published ~975 B for the same quantity; that figure does not reproduce
    // from the formula above and is superseded.
    constexpr std::uint32_t kUsableSingleBunchBytes = 952u;

    // Per ROOT-OBJECT batch: net handle id, 16-bit batch size, batch flags, and
    // the changemask (`FReplicationWriter::WriteObjectAndSubObjects` +
    // `WriteSparseBitArray`). Source decomposition puts it at ~5.6-6.6 B; pinned
    // at the top of the range because this is a budget, and a budget that
    // under-counts framing is the mistake T37 already paid for once.
    constexpr std::uint32_t kPerRootObjectBatchOverheadBytes = 7u;

    // Per PACKET, paid once: Iris stream headers + debug bits + batch count +
    // destroy count (`UDataStreamManager::FImpl::WriteData`,
    // `FReplicationWriter::Write`) at ~5.25 B, plus the real bunch header once the
    // channel is open-acked (`UNetConnection::SendRawBunch`) at ~3.4 B.
    constexpr std::uint32_t kPerPacketOverheadBytes = 9u;

    // -----------------------------------------------------------------------
    // PAYLOAD SIZES, COMPUTED — never typed in.
    // -----------------------------------------------------------------------

    // The relay ring on the wire: the u16 length prefix written by
    // `FRelayedInputRing::NetSerialize`, then the codec's own header
    // ([version u8][entryCount u8]) and `entries` fixed-stride entries
    // ([captureTick u32][dA u8][input]).
    //
    // ⚠ [T34] `entries` IS NOT A DEPTH ANY MORE. Under bare C1 flush-on-poll the
    // ring carries however many arrivals the last poll interval saw — 1 on a
    // healthy frame, 2 at the measured per-character p99, up to
    // `relayedInputRing::kMaxDepth` in a join burst. The old `depth` parameter
    // named a session constant; this one names a per-frame observation, which is
    // exactly why the pre-diet table at the bottom of this file exists.
    constexpr std::uint32_t ringWireBytes(std::uint32_t entries)
    {
        return static_cast<std::uint32_t>(sizeof(std::uint16_t))
             + relayedInputRing::kHeaderBytes
             + entries * relayedInputRing::detail::entryStride<simulatableBrawler::PlayerInput>();
    }

    // [T34] Everything a ring object costs EXCEPT its entries: the u16 prefix, the
    // codec header and the per-root-object batch framing. Split out because the
    // pre-diet table's bound is linear in total entries across all remote rings and
    // constant per ring, and keeping the two terms apart is what makes the
    // fractional average entry count (1.132, measured) expressible.
    constexpr std::uint32_t kRingFixedBytesPerObject =
          static_cast<std::uint32_t>(sizeof(std::uint16_t))
        + relayedInputRing::kHeaderBytes
        + kPerRootObjectBatchOverheadBytes;

    constexpr std::uint32_t kRingEntryBytes =
        relayedInputRing::detail::entryStride<simulatableBrawler::PlayerInput>();

    // The correction state on the wire: `FSimulationStateSyncBuffer::NetSerialize`
    // writes [version u8][used u16] and then `used` payload bytes, where the
    // payload is the correction codec's [tick u32][appliedCaptureTick u32] header
    // followed by the whole `simulatableBrawler::State` composite. The fixed
    // kBufferBytes capacity does NOT ride the wire — the watermark trim means only
    // the used prefix is sent.
    constexpr std::uint32_t kStateWireBytes =
          static_cast<std::uint32_t>(sizeof(std::uint8_t))     // wire-format version
        + static_cast<std::uint32_t>(sizeof(std::uint16_t))    // used-byte count
        + correctionStateBuffer::kHeaderBytes                  // tick + appliedCaptureTick
        + relayedInputRing::detail::CompositeSerializedSize<simulatableBrawler::State>::value;

    // THE ROUND. `characters` is N; every remote character contributes one ring
    // object carrying `entriesPerRing` entries, and one correction state is
    // required to fit beside them.
    constexpr std::uint32_t roundBytes(std::uint32_t characters, std::uint32_t entriesPerRing)
    {
        const std::uint32_t remoteRings = (characters == 0u) ? 0u : characters - 1u;
        return remoteRings * (ringWireBytes(entriesPerRing) + kPerRootObjectBatchOverheadBytes)
             + (kStateWireBytes + kPerRootObjectBatchOverheadBytes)
             + kPerPacketOverheadBytes;
    }

    // ⭐ [T34] THE INPUT-GUARANTEE BOUND, in MILLI-BYTES.
    //
    // The rings must fit the packet BY THEMSELVES. That is not a stylistic
    // preference: rings sort first (static priority 4.0 against the state's 1.0), so
    // a ring is only ever skipped when the rings ALONE overflow — and under R = 0 a
    // skipped ring round is a lost burst with no recovery path anywhere. States
    // absorb the overflow instead, and a skipped state costs repair latency, not
    // repair ability.
    //
    // `sumEntriesX1000` is total entries across all N-1 remote rings, scaled by
    // 1000 so the MEASURED average of 1.132 entries per ring per frame survives
    // integer arithmetic. Everything else is exact.
    constexpr std::uint64_t ringsOnlyBytesX1000(std::uint32_t characters,
                                                std::uint32_t sumEntriesX1000)
    {
        const std::uint64_t remoteRings = (characters == 0u) ? 0u : characters - 1u;
        return remoteRings * kRingFixedBytesPerObject * 1000ull
             + static_cast<std::uint64_t>(sumEntriesX1000) * kRingEntryBytes
             + kPerPacketOverheadBytes * 1000ull;
    }

    constexpr std::uint64_t kBudgetX1000 =
        static_cast<std::uint64_t>(kUsableSingleBunchBytes) * 1000ull;

    // The three per-ring entry counts the design models, all MEASURED (T22/T33 write
    // -path histograms) rather than assumed:
    //   average    1.132 entries per ring per frame in steady state
    //   p99        2
    //   join burst 5-8; the stage caps at kMaxDepth, so 8 is the worst a ring can
    //              ever carry and is what a joining character's first rounds look
    //              like while its client settles.
    constexpr std::uint32_t kAvgEntriesX1000  = 1132u;
    constexpr std::uint32_t kP99Entries       = 2u;
    constexpr std::uint32_t kJoinBurstEntries =
        static_cast<std::uint32_t>(relayedInputRing::kMaxDepth);

    // "A join, and nothing else unusual": the joiner's ring at the stage cap, every
    // other remote ring at its steady average. This is the scenario that decides the
    // cap, because it needs no server hitch and no correlated burst — it happens on
    // an ordinary join.
    constexpr std::uint32_t joinAloneSumEntriesX1000(std::uint32_t characters)
    {
        // N-1 remote rings: one of them is the joiner, the rest sit at the average.
        return (characters < 2u)
            ? 0u
            : kJoinBurstEntries * 1000u + (characters - 2u) * kAvgEntriesX1000;
    }

    // The pre-diet character cap. DERIVED here rather than typed: it is the largest
    // N whose join-alone round still fits, and the runtime fence
    // (`ASimulationManagerUImpl::kPreDietCharacterCap`) mirrors this value. Both are
    // deleted by item 40.
    constexpr std::uint32_t largestFittingCharacterCountUnderJoin()
    {
        std::uint32_t n = 2u;
        while (n < 16u && ringsOnlyBytesX1000(n + 1u, joinAloneSumEntriesX1000(n + 1u))
                          <= kBudgetX1000)
        {
            ++n;
        }
        return n;
    }

    // The product target: Tier 2 of the ratified player-count envelope. Read from
    // the source of truth rather than typed as a 6, so that raising the envelope
    // automatically re-prices the packet instead of silently leaving this test
    // guarding a character count the product left behind.
    constexpr std::uint32_t kTargetCharacters =
        static_cast<std::uint32_t>(og::brawler::session::maxPlayersPerServerTier2);

    // The steady-state residency of ONE ring: one arrival, published at the next
    // poll. [T34] This used to read the session-configurable ring-retention
    // depth (item 63 / RN-13, 2026-08-16: that field is now retired outright —
    // its old identifier is on record in RN-13, ReviewNotes.md), and
    // deliberately does not any more: under flush-on-poll that knob was already
    // INERT before its removal (the stage's capacity is
    // `relayedInputRing::kMaxDepth`, taken as a constant), so modelling the
    // round from it would make this fence fire on a change that has no wire
    // effect at all, and would quietly stop firing on the one that does.
    constexpr std::uint32_t kSteadyEntriesPerRing = 1u;

    // The rotation width a session runs at. Pre-diet this is 1 (T38 §16.2); item 40
    // restores 2.
    constexpr std::uint32_t kShippedRotationK =
        static_cast<std::uint32_t>(TimeConfig{}.correctionRotationK);

    constexpr std::uint32_t kStateBatchBytes =
        kStateWireBytes + kPerRootObjectBatchOverheadBytes;
}

// ---------------------------------------------------------------------------
// 1. THE INVARIANT.
// ---------------------------------------------------------------------------

TEST_CASE("PacketBudget: all remote rings plus one correction state fit one packet at the product target",
          "[PacketBudget][InputFirstReplication]")
{
    const std::uint32_t round = roundBytes(kTargetCharacters, kSteadyEntriesPerRing);

    INFO("characters=" << kTargetCharacters
         << " entriesPerRing=" << kSteadyEntriesPerRing
         << " ringWire=" << ringWireBytes(kSteadyEntriesPerRing)
         << " stateWire=" << kStateWireBytes
         << " round=" << round
         << " budget=" << kUsableSingleBunchBytes);

    REQUIRE(round <= kUsableSingleBunchBytes);

    // Also a compile-time fence, so the failure surfaces at build time for anyone
    // who changes a wire layout and does not run the suite. The REQUIRE above
    // survives beside it because it is what prints the arithmetic when it breaks.
    static_assert(roundBytes(kTargetCharacters, kSteadyEntriesPerRing) <= kUsableSingleBunchBytes,
        "The relay rings of all remote characters plus one correction state no longer fit a "
        "single Iris packet at the product character target. This is the T37 defect: the "
        "round, not the payload, is the budget. Shrink the wire (item 40's diet) before "
        "raising the character count or either payload.");

    // ⚠ [T34] THIS CASE PRICES THE STEADY FRAME, NOT THE WORST ONE. Under bare C1
    // flush-on-poll the ring's residency is a per-frame observation (1 on a healthy
    // frame, 2 at p99, up to 8 in a join burst), so "one entry per ring at the
    // product target" is the AVERAGE round and no longer an invariant on its own.
    // The invariant that actually binds — the rings fitting the packet BY
    // THEMSELVES under a join — is the pre-diet table at the bottom of this file,
    // and it is why six characters is not currently a shippable configuration.

    // The intermediate counts, pinned so a break says WHICH term moved rather than
    // just that the sum did. These are not independent facts — they are the same
    // arithmetic decomposed — and that is the point: a diff that changes one of
    // them is a wire change, and a wire change must be a deliberate, versioned act.
    REQUIRE(relayedInputRing::kHeaderBytes == 2u);
    REQUIRE(correctionStateBuffer::kHeaderBytes == 8u);
    // [movement-sim task 11, 2026-09-06] 85 -> 86 B. The movement sub-sim's PlayerInput
    // gained `holdGuard` as a `bool` member, so the INPUT composite went 76 -> 77 B and the
    // per-entry stride with it (81 -> 82). This is the first movement-sim task to move the
    // ring term at all: T1/T5/T29 all moved the STATE wire and left the input untouched.
    //
    // [movement-sim task 51, 2026-09-06] RE-QUOTED UNCHANGED at 86 B, and that is the whole
    // result of the task. `PlayerInput` went from a `bool` member to `uint8_t flags`, with
    // holdGuard as bit 0 and bits 1-7 reserved for wall-grab (20), jump (21), dash (31) and
    // ski-tuck (48). `bool` and `uint8_t` are both 1 B, so the input composite is still 77 B,
    // the entry stride is still 82 B and this line still reads 86. The four future signals
    // now cost ZERO further ring bytes instead of ~10.264 B of margin each.
    REQUIRE(ringWireBytes(1u) == 86u);
    // [movement-sim T1 -> T5, 2026-09-02] 311 -> 363 -> 335 B. T1 appended the
    // movement sub-sim's State carrying a placeholder 52 B PhysicsBodyState
    // (composite 300 -> 352 B); T5 swapped that slice to the 24 B LinearBodyState
    // (composite 352 -> 324 B, -28 B).
    //
    // [movement-sim T29, 2026-09-04] 335 -> 279 B. The guard sub-simulation went
    // entirely off the wire (composite 324 -> 268 B, -56 B: a 4 B phantom timer and
    // a 52 B derivable body state). 279 = 1 (version) + 2 (used count) + 8
    // (correction header) + 268 (composite). The composite's own absolute size and
    // its headroom against FSimulationStateSyncBuffer::kBufferBytes are fenced in
    // SimulatableBrawlerTest.cpp's WireFootprint case.
    //
    // [movement-sim task 11, 2026-09-06] 279 -> 320 B. The movement sub-simulation stopped
    // being a skeleton: its State went 24 -> 49 B (bodyState 24 + a sim-owned velocity 12 +
    // committedStepDir 8 + stepStartTick 4 + flags 1, per ruling #17 b) and its
    // InitialConditions 0 -> 16 B (the teleport seed), so the composite went 268 -> 309 B.
    // 320 = 1 (version) + 2 (used count) + 8 (correction header) + 309 (composite).
    //
    // [movement-sim task 50, 2026-09-06] 320 -> 332 B, and the INPUT wire DOES NOT MOVE:
    // `ringWireBytes(1u)` above is re-quoted UNCHANGED at 86 B. The movement sub-simulation's
    // `State::positionCmd` went from off-wire scratch onto the wire (+12 B, slice 49 -> 61,
    // composite 309 -> 321) because a correction restores by whole-struct assignment over a
    // default-constructed state, which zeroed the one operand step 6' cannot re-derive.
    // 332 = 1 (version) + 2 (used count) + 8 (correction header) + 321 (composite).
    REQUIRE(kStateWireBytes == 332u);
}

// ---------------------------------------------------------------------------
// 2. THE POSITIVE CONTROL — proof the fence can actually bite.
// ---------------------------------------------------------------------------

TEST_CASE("PacketBudget: the fence bites — two entries per ring at the product target does NOT fit",
          "[PacketBudget][InputFirstReplication]")
{
    // A budget test that only ever passes is indistinguishable from no test. This
    // case pins the OTHER side of the boundary permanently, so the falsification
    // does not have to be re-performed by hand (item 39 AC 1's "flip depth to 2
    // locally and show red") every time somebody wants to trust this file.
    //
    // [T34] Two entries per ring is no longer a config an operator has to choose —
    // it is the MEASURED per-character p99 of bare C1's own residency. So this row
    // stopped being "what happens if somebody sets the knob to 2" (the T33
    // experiment T37 refuted) and became "what a correlated p99 frame at six
    // characters would cost", which is the same arithmetic pointed at a live
    // scenario. Six characters is out of reach until item 40's diet lands, and this
    // is one of the two independent reasons why.
    REQUIRE(roundBytes(kTargetCharacters, kP99Entries) > kUsableSingleBunchBytes);

    // ...and the boundary is where the arithmetic says, not merely "somewhere
    // above 1". At six characters the rings alone at two entries are 5 x 175 = 875 B,
    // which leaves no room for a state (327 B) inside 952 B.
    // [movement-sim task 51, 2026-09-06] RE-QUOTED UNCHANGED at 168 B: the flags re-layout
    // is 1 B for 1 B, so neither the stride nor any multiple of it moved.
    // [movement-sim task 11, 2026-09-06] 166 -> 168 B (2 x the 81 -> 82 B entry stride);
    // the state batch is 327 B, not 342. Both terms moved, and the row's conclusion is
    // unchanged and now holds by a wider margin.
    REQUIRE(ringWireBytes(2u) == 168u);
    // [movement-sim T1 -> T5, 2026-09-02] 1192 -> 1244 -> 1216 B, all of it the state
    // term (318 -> 370 -> 342 B batched). The ring term never moved: the movement
    // sub-sim's PlayerInput slice is empty, so that task changed the STATE wire and
    // not the INPUT wire at all.
    // [movement-sim T29, 2026-09-04] 1216 -> 1160 B, again all state term
    // (342 -> 286 B batched), and again with the ring term untouched — the guard's
    // PlayerInput still carries its aim, only its State left the wire.
    // [movement-sim task 11, 2026-09-06] 1160 -> 1211 B, and this is the FIRST movement-sim
    // task to move BOTH terms: the state batch 286 -> 327 B (+41, the composite growth) and
    // the ring term 5 x (166+7) -> 5 x (168+7) = +10, because the movement PlayerInput
    // stopped being empty. 1211 = 875 (rings) + 327 (state batch) + 9 (per-packet overhead).
    // The row's conclusion — six characters at two entries does not fit 952 B — is
    // unchanged and now holds by 259 B instead of 208 B.
    // [movement-sim task 50, 2026-09-06] 1211 -> 1223 B, and ALL of it is the state term
    // (327 -> 339 B batched, the +12 B of `positionCmd`). The ring term is UNTOUCHED at
    // 5 x (168+7) = 875 B — this task moves the STATE wire and not one input byte, which is
    // the constraint the ordinary-join table below is now only ~2.5 input bytes away from.
    // 1223 = 875 (rings) + 339 (state batch) + 9 (per-packet overhead); the conclusion is
    // unchanged and now holds by 271 B.
    REQUIRE(roundBytes(kTargetCharacters, 2u) == 1223u);

    // The relay ring's own malformed-length ceiling is far above the packet, and
    // that is not a contradiction: kMaxWireBytes bounds what a RECEIVER will
    // accept from a hostile sender, while this file bounds what a SENDER may
    // schedule. Pinned together here so the two bounds are never confused for one
    // another the way "the rate allowance" and "the packet" were before T37.
    REQUIRE(relayedInputRing::kMaxWireBytes > kUsableSingleBunchBytes);
}

// ---------------------------------------------------------------------------
// 3. THE HEADROOM, REPORTED.
// ---------------------------------------------------------------------------

TEST_CASE("PacketBudget: the shipped round leaves the margin the PIE gate is set to",
          "[PacketBudget][InputFirstReplication]")
{
    // The PIE gate (item 39 AC 3) asks for `bytesPerPacket <= capacity - 32 B`.
    // That margin is only meaningful if the modelled round clears it too — a
    // model that just barely fits would make the live gate a coin flip on
    // framing noise rather than a check on the design.
    constexpr std::uint32_t kPieGateMarginBytes = 32u;

    const std::uint32_t round = roundBytes(kTargetCharacters, kSteadyEntriesPerRing);
    INFO("round=" << round << " headroom="
         << (kUsableSingleBunchBytes - round) << " B");

    REQUIRE(round + kPieGateMarginBytes <= kUsableSingleBunchBytes);

    // The two- and three-character cases, which are the ones the shipped gates
    // actually measure. Three characters is the case that is OVER budget before
    // this task (the T29 commit measured a 1,203 B round against the MTU) and is
    // therefore the cheapest proof that Stage 1 worked; both must clear the same
    // margin. Note these carry ONE state, as the invariant does — the rotation's
    // second state at K=2 is opportunistic on top.
    REQUIRE(roundBytes(2u, kSteadyEntriesPerRing) + kPieGateMarginBytes <= kUsableSingleBunchBytes);
    REQUIRE(roundBytes(3u, kSteadyEntriesPerRing) + kPieGateMarginBytes <= kUsableSingleBunchBytes);

    // At the SHIPPED rotation width every extra state must fit too. [T34] K is 1
    // pre-diet, so `extraStates` is 0 and these two rows are currently the same
    // statement as the two above — kept, and kept computed from the knob rather
    // than folded away, because item 40 restores K=2 and this is where that restore
    // gets priced against the small character counts the shipped gates measure.
    const std::uint32_t extraStates =
        (kShippedRotationK > 1u) ? (kShippedRotationK - 1u) : 0u;

    REQUIRE(roundBytes(2u, kSteadyEntriesPerRing) + extraStates * kStateBatchBytes
            <= kUsableSingleBunchBytes);
    REQUIRE(roundBytes(3u, kSteadyEntriesPerRing) + extraStates * kStateBatchBytes
            <= kUsableSingleBunchBytes);
}

// ---------------------------------------------------------------------------
// ⭐ 4. [og-netcode-v2-input-relay T34] THE PRE-DIET TABLE — the cap, from inside
//    the suite.
//
// ⛔ ITEM 40 REPLACES THIS WHOLE SECTION with the post-diet table (its AC 2), and
// deletes `ASimulationManagerUImpl::kPreDietCharacterCap` with it. Their joint
// absence is the "cap lifted" statement; there is no flag to flip.
//
// WHY A TABLE AND NOT A CONSTANT. Bare C1 makes the ring's residency VARIABLE, so
// the question "does the round fit" stopped having one answer and acquired a
// distribution. What binds is the INPUT GUARANTEE: the rings must fit the packet by
// themselves, because they sort ahead of the states (priority 4.0 vs 1.0) and are
// therefore only ever skipped when they alone overflow — and under R = 0 a skipped
// ring round is a lost burst with no recovery path. So the table prices the
// scenarios, and the cap falls out of it rather than being asserted.
//
// THE SCENARIO THAT DECIDES IT is an ORDINARY JOIN: the joining character's ring at
// the stage cap while everyone else sits at the measured steady average. It needs no
// server hitch and no correlated burst — it happens every time somebody connects.
// N = 4 clears it with about five sixths of one entry to spare; N = 5 does not.
//
// [movement-sim task 11, 2026-09-06] ⭐⭐ THE INPUT WIRE IS THE SCARCE ONE, AND THE NUMBER
// IS SMALL. Task 11's `PlayerInput::holdGuard` moved the entry stride 81 -> 82 B, and the
// margin at the cap went **78.616 -> 68.352 B = 0.970 -> 0.833 entries** -- hence "nine
// tenths" becoming "five sixths" above. Nothing is red and both guards below still hold,
// but the arithmetic underneath is the load-bearing part:
//   * `:472` guards `margin > half an entry` -- a floor that is now **41.0 B** against a
//     margin of **68.352 B**, so **27.352 B of slack**.
//   * One byte of INPUT composite costs **10.264 B of margin**, because it is multiplied
//     across every ring entry in the scenario (8 join-burst + 2 x 1.132 average = 10.264
//     entries at N = 4). An input byte is roughly **TEN TIMES more expensive than a state
//     byte**, which costs nothing here at all -- this row prices rings ONLY.
//   * The half-entry floor itself rises 0.5 B per stride byte, so net closure is
//     **10.764 B per input byte**.
//   * ⛔ **27.352 / 10.764 = 2.54 -- TWO more bytes of input wire and `:472` goes RED.**
// `holdGuard` is a single `bool` and it consumed A QUARTER of the entire remaining input
// budget. ⭐ STANDING RULE, recorded at the "ADDING A MOVEMENT MODEL" block in
// `OGBrawler/BrawlerMovementSimulation.h`: future per-tick input fields go in as BITS IN AN
// EXISTING FLAGS BYTE, never as new `bool` members. Jump (21), dash (31), wall-grab (20) and
// ski-tuck (48) as four separate bools would blow this fence twice over; as four bits in one
// byte they cost nothing beyond what `holdGuard` already spent.
//
// ⭐⭐ [movement-sim task 51, 2026-09-06] THE FLAGS BYTE NOW EXISTS, AND NONE OF THE
// ARITHMETIC ABOVE MOVED. `brawlerMovementSimulation::PlayerInput` is `uint8_t flags` with
// holdGuard as bit 0 and bits 1-7 reserved for exactly those four tasks. `bool` -> `uint8_t`
// is 1 B for 1 B, so the entry stride is still 82 B, `ringWireBytes(1u)` is still 86 B, and
// the margin at the cap is still 68.352 B with 27.352 B of slack over the half-entry floor.
// The four coming signals are now free at this fence rather than ~10.264 B of margin each,
// which is what turns "2.54 more input bytes, ever" into a budget the roadmap fits inside.
// The rule is no longer advisory: the type it names has nowhere to put a new `bool`.
// ---------------------------------------------------------------------------

TEST_CASE("PacketBudget: the pre-diet cap is 4, and it is where join-alone crosses the bound",
          "[PacketBudget][InputFirstReplication]")
{
    // ⭐ THE INVERTED ROW. N = 5's ordinary-join round EXCEEDS the budget — this is
    // the assertion that makes the cap a derived fact rather than a preference, and
    // the one that must go red first if anyone grows the entry payload.
    INFO("N=5 join-alone ringsOnly(x1000)=" << ringsOnlyBytesX1000(5u, joinAloneSumEntriesX1000(5u))
         << " budget(x1000)=" << kBudgetX1000);
    REQUIRE(ringsOnlyBytesX1000(5u, joinAloneSumEntriesX1000(5u)) > kBudgetX1000);

    // ...and the rows below it PASS, so the boundary is exactly where the cap says.
    for (std::uint32_t n = 2u; n <= 4u; ++n)
    {
        INFO("N=" << n << " join-alone ringsOnly(x1000)="
             << ringsOnlyBytesX1000(n, joinAloneSumEntriesX1000(n)));
        REQUIRE(ringsOnlyBytesX1000(n, joinAloneSumEntriesX1000(n)) <= kBudgetX1000);
    }

    // The cap, derived by walking the same bound rather than typed. The runtime
    // fence mirrors this value as `ASimulationManagerUImpl::kPreDietCharacterCap`;
    // the two are pinned together by this line and by that constant's comment,
    // because a pure-C++ target cannot see a UCLASS.
    REQUIRE(largestFittingCharacterCountUnderJoin() == 4u);

    // The margin at the cap, reported rather than asserted loosely: any payload
    // growth that eats it turns the row above red instead of silently narrowing it.
    const std::uint64_t marginX1000 =
        kBudgetX1000 - ringsOnlyBytesX1000(4u, joinAloneSumEntriesX1000(4u));
    INFO("margin at the cap = " << (marginX1000 / 1000u) << " B = "
         << (marginX1000 / kRingEntryBytes) << " milli-entries");
    REQUIRE(marginX1000 < kRingEntryBytes * 1000ull);   // under ONE entry of slack
    REQUIRE(marginX1000 > (kRingEntryBytes * 1000ull) / 2ull);  // and over half of one
}

TEST_CASE("PacketBudget: at the cap the whole round fits on average AND at correlated p99, with K=1",
          "[PacketBudget][InputFirstReplication]")
{
    // THE PASS ROWS. Rings plus the one correction state K=1 publishes, at the
    // measured steady average and at a fully correlated p99 frame (every remote ring
    // carrying 2). Both must clear the budget at every N up to the cap, or the
    // pre-diet window is not a shippable configuration at all.
    for (std::uint32_t n = 2u; n <= 4u; ++n)
    {
        const std::uint64_t avgRings = ringsOnlyBytesX1000(n, (n - 1u) * kAvgEntriesX1000);
        const std::uint64_t p99Rings = ringsOnlyBytesX1000(n, (n - 1u) * kP99Entries * 1000u);
        const std::uint64_t stateX1000 = kStateBatchBytes * 1000ull;

        INFO("N=" << n << " avgRound(x1000)=" << (avgRings + stateX1000)
             << " p99Round(x1000)=" << (p99Rings + stateX1000)
             << " budget(x1000)=" << kBudgetX1000);

        REQUIRE(avgRings + stateX1000 <= kBudgetX1000);
        REQUIRE(p99Rings + stateX1000 <= kBudgetX1000);
    }

    // And K really is 1 in this build — the packet arithmetic above is only the
    // shipped configuration if the rotation width agrees with it.
    REQUIRE(kShippedRotationK == 1u);
}

TEST_CASE("PacketBudget: the K=2 round at the cap does NOT fit — the premise K=1 shipped on holds again",
          "[PacketBudget][InputFirstReplication]")
{
    // ⭐ THIS ROW WAS AN INVERTED POSITIVE CONTROL AND IT HAS NOW FIRED. Read the
    // history before changing it again — the whole point of this case is that the
    // reasoning it retired stays legible.
    //
    // WHAT IT USED TO ASSERT, AND WHY. T38 §16.2's argument for shipping K=1 is
    // Iris's huge-object window, which this target cannot model (that is an engine
    // control-flow fact about `HandleObjectBatchFailure`, not an arithmetic one).
    // What COULD be asserted here was the necessary condition underneath it: at four
    // characters a K=2 round did not fit one packet even on an AVERAGE frame, so a
    // second state batch was attempted-and-failed on essentially every frame — which
    // is precisely the situation in which the window's 192-316 B fork gets reached.
    // So this row never proved §16.2; it proved the PREMISE §16.2 reasons from, and
    // it was written to go red the moment a diet made that premise false — which is
    // the signal that K may return to 2.
    //
    // ⭐ [movement-sim T29, 2026-09-04] THE DIET LANDED AND THE PREMISE WENT FALSE,
    // measured. Task 29 took the guard sub-simulation's State entirely off the wire
    // (composite 324 -> 268 B, -56 B), which moved kStateBatchBytes 342 -> 286 B and
    // this round from 1001.076 B — over a 952 B bunch by 49.076 B, where it had sat
    // red since the row was written — to 889.076 B, CLEARING by 62.924 B. The
    // comparison was therefore inverted to `<=` to record that measurement rather
    // than to keep asserting a falsehood.
    //
    // ⭐⭐ [movement-sim task 11, 2026-09-06] AND NOW IT IS TRUE AGAIN, measured. The
    // movement sub-simulation stopped being a skeleton: its State went 24 -> 49 B and
    // its InitialConditions 0 -> 16 B (composite 268 -> 309 B, +41 B), and its
    // PlayerInput gained one byte, which moved the ring entry stride 81 -> 82 B. Both
    // terms grew, so this round went 889.076 -> 974.472 B — OVER the 952 B bunch by
    // 22.472 B. `kStateBatchBytes` is 327 B, not 286.
    //
    // ⛔ THE COMPARISON IS THEREFORE BACK TO ITS ORIGINAL `>` FORM, and the case name
    // with it. This is NOT a fence being flipped to hide a failure: `>` is the
    // direction this row was WRITTEN in, the direction that is measured true today,
    // and the direction that goes red the moment a future diet clears it — which is
    // exactly the alarm it exists to raise. Task 29's `<=` recorded a fact that was
    // true for two days. Both measurements are kept above so the swing is legible.
    //
    // ⛔ SO K=2 IS OFF THE TABLE ON ARITHMETIC ALONE AGAIN, at N=4. K is still 1:
    // `TimeConfig::correctionRotationK` is untouched by task 11 and the preceding case
    // still asserts `kShippedRotationK == 1u` truthfully. Note that the arithmetic
    // objection was never the ONLY one — §16.2 also rested on the Iris huge-object
    // window, an engine fact this target cannot model — so this row returning to red
    // restores a sufficient objection, not merely a necessary one.
    //
    // ⚠ FOR THE LEAD: the shipped K=1 configuration is UNAFFECTED and still clears at
    // every N up to the cap on both the average and the correlated-p99 frame (the
    // preceding case, all green). What task 11 consumed is the hypothetical-K=2
    // headroom that task 29 had opened.
    //
    // ⭐ [movement-sim task 50, 2026-09-06] MEASURED A FOURTH TIME, AND THE DIRECTION IS
    // UNCHANGED. `kStateBatchBytes` 327 -> 339 B (+12, `positionCmd` onto the wire), so the
    // N=4 round goes 974.472 -> 998.472 B against a 952 B bunch — OVER by 46.472 B where
    // task 11 left it over by 22.472. The `>` comparison below stays the direction the row
    // was written in and the direction that is measured true. The four historical
    // measurements are all preserved above: 1001.076 B (as the row was first written, red)
    // -> 889.076 (T29's -56 B diet, cleared) -> 974.472 (task 11, red again) -> 998.472
    // (task 50, red by a further 24 B).
    // ⛔ The row is NOT restructured — that is task 28's job. The N=3 arm below still
    // clears: 2 x 339 B of state now, 894.648 B against 952.
    //
    // ⚠ Note the literal `2ull` below: this row prices a HYPOTHETICAL K=2 round and
    // is deliberately independent of the configured `kShippedRotationK`. Changing the
    // config does not change this arithmetic by a single byte.
    const std::uint64_t avgRingsAtCap = ringsOnlyBytesX1000(4u, 3u * kAvgEntriesX1000);
    INFO("N=4 K=2 avgRound(x1000)=" << (avgRingsAtCap + 2ull * kStateBatchBytes * 1000ull)
         << " budget(x1000)=" << kBudgetX1000);
    REQUIRE(avgRingsAtCap + 2ull * kStateBatchBytes * 1000ull > kBudgetX1000);

    // At three characters K=2 still fits on average, which is why §16.2 had to
    // reason about the window rather than about bytes to rule it out there.
    //
    // [movement-sim T1 -> T5, 2026-09-02] RESTORED. T1's placeholder 52 B body state
    // put this row 2.384 B over the bunch (954.384 vs 952 B) and neutralised the CASE
    // behind `[!shouldfail]` rather than inverting the claim; T5's LinearBodyState
    // swap took kStateBatchBytes 370 -> 342 B and this round to 898.384 B, clearing
    // by 53.616 B. The tag and its comment block are gone and the `<=` below is live
    // again. The row above (N=4) stayed red from then until movement-sim task 29 —
    // 1001.076 B, over by 49.076 B — so the positive control this case exists for
    // never lapsed while the premise it guarded was still true.
    //
    // [movement-sim T29, 2026-09-04] This arm was ALREADY clearing and still is; the
    // diet only widened its margin, 898.384 -> 786.384 B (214.384 B of rings plus
    // 2 x 286 B of state), so nothing here needed re-pointing. The N=4 arm above is
    // the one task 29 flipped.
    //
    // [movement-sim task 11, 2026-09-06] This arm STILL clears, which is what makes the
    // N=4 arm above a boundary rather than a blanket failure: task 11's growth narrows
    // this margin (2 x 327 B of state now, and a slightly wider ring term) without
    // crossing it. One character fewer is the whole difference.
    //
    // [movement-sim task 50, 2026-09-06] STILL clears, and by a margin worth recording:
    // 2 x 339 B of state now, so 870.648 -> 894.648 B against the 952 B bunch, clearing
    // by 57.352 B. The N=4 arm above went the other way in the same edit (974.472 ->
    // 998.472, over by 46.472), which is what keeps this pair a BOUNDARY.
    const std::uint64_t avgRingsAtThree = ringsOnlyBytesX1000(3u, 2u * kAvgEntriesX1000);
    INFO("N=3 K=2 avgRound(x1000)=" << (avgRingsAtThree + 2ull * kStateBatchBytes * 1000ull)
         << " budget(x1000)=" << kBudgetX1000);
    REQUIRE(avgRingsAtThree + 2ull * kStateBatchBytes * 1000ull <= kBudgetX1000);
}

TEST_CASE("PacketBudget: a connection running TWO local players is bounded by the one-local-player model",
          "[PacketBudget][InputFirstReplication]")
{
    // ⚠ THE TOPOLOGY THE VALIDATION ACTUALLY RUNS, and the one this file did not
    // price until now. Every row above models one local player per client: N-1
    // remote rings on every connection. The user's usual 3-character scenario is TWO
    // clients, one of them running two local players — and item 39's closing review
    // measured that couch-co-op connection at 917 B mean / 942 B max against a
    // 952 B bunch, ~176 B/tick beyond what the round model accounts for.
    //
    // ⭐ THE ANSWER, AND IT IS A ONE-LINER ONCE STATED: a connection owning C
    // characters receives N-C rings, not N-1, because COND_SkipOwner drops the ring
    // of every character that connection owns. So a multi-local-player connection
    // carries STRICTLY FEWER relay bytes than the single-local-player model this
    // file already asserts, at the same N and the same K. The input guarantee on
    // that connection is therefore covered by the existing rows, a fortiori.
    //
    // ⛔ WHAT THIS ROW DOES NOT COVER, said plainly rather than left implied: the
    // ~176 B/tick the measurement could not attribute. A second local player brings
    // its own PlayerController and PlayerState traffic, which is not relay traffic
    // and is not scheduled by anything this task touches — this file prices the
    // ROUND this system schedules, never the whole connection. The instrument for
    // the whole connection is the live `[RelayProbe.Budget]` gate
    // (`outPackets <= frames * 1.02`, `bytesPerPacket`), and that is where a
    // couch-co-op run is judged. Modelling an unattributed term here would be
    // inventing a number, which is how item 33 happened.
    for (std::uint32_t n = 3u; n <= 4u; ++n)
    {
        const std::uint32_t oneLocalRemoteRings = n - 1u;
        const std::uint32_t twoLocalRemoteRings = n - 2u;
        REQUIRE(twoLocalRemoteRings < oneLocalRemoteRings);

        // Same scenario, both topologies, at the correlated p99.
        const std::uint64_t oneLocal =
            ringsOnlyBytesX1000(n, oneLocalRemoteRings * kP99Entries * 1000u);
        const std::uint64_t twoLocal =
            twoLocalRemoteRings * kRingFixedBytesPerObject * 1000ull
            + static_cast<std::uint64_t>(twoLocalRemoteRings) * kP99Entries * 1000ull * kRingEntryBytes
            + kPerPacketOverheadBytes * 1000ull;

        INFO("N=" << n << " oneLocal(x1000)=" << oneLocal << " twoLocal(x1000)=" << twoLocal);
        REQUIRE(twoLocal < oneLocal);
        REQUIRE(twoLocal + kStateBatchBytes * 1000ull <= kBudgetX1000);
    }
}

TEST_CASE("PacketBudget: the flush stage capacity is kMaxDepth - a compile-time constant",
          "[PacketBudget][InputFirstReplication]")
{
    // [og-netcode-v2-input-relay item 63 / RN-13, 2026-08-16] This case used to
    // price a session-configurable ring-retention knob's compiled value against
    // kMaxDepth to show the knob was inert under flush-on-poll (T34). That knob
    // is now RETIRED outright (its old identifier is on record in RN-13,
    // ReviewNotes.md) rather than merely inert, so there is no compiled value
    // left to price. What survives — and is machine-fenced against ever
    // reopening T43 finding 1 — is that the stage's capacity is `kMaxDepth`,
    // taken as a constant by `relayedInputRing::stageArrival`, and that
    // `stageArrival` accepts no depth argument at all; see
    // `Network/RelayRedundancyDepthTest.cpp` for the compile-time half of that
    // fence. This case keeps the runtime, packet-budget-shaped half: one ring at
    // the stage cap must still be a legal payload on its own.
    REQUIRE(relayedInputRing::kMaxDepth == 8u);
    REQUIRE(relayedInputRing::isAcceptableWireLength(
        ringWireBytes(static_cast<std::uint32_t>(relayedInputRing::kMaxDepth))));
    REQUIRE(ringWireBytes(kJoinBurstEntries) < kUsableSingleBunchBytes);
}

#endif // WITH_LOW_LEVEL_TESTS
