<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerHitDedupPinsTest — rationale

Source: `BrawlerHitDedupPinsTest.cpp`. This file has no guards document: its two prohibitions are
`static_assert`s on the peer handles, so nothing in it needs a `⛔` tag.

<!-- lint-external-ref: attackHits -- RETIRED by og-netcode-v2-field-defects task 27: the radial's derived per-swing ledger. It must NOT resolve -->
<!-- lint-external-ref: activeRootBodyId -- RETIRED by og-netcode-v2-field-defects task 27: the dead radial InitialConditions field. It must NOT resolve -->

## 1. What this file is for

Two pins that must hold before and after the per-swing hit ledger moves from the radial's derived
`attackHits` onto the wire (og-netcode-v2-field-defects task 27, candidate A of that initiative's
hit-dedup design). Both were landed by task 26 and were seen GREEN on the tree without the ledger. Task
27 must keep them green.

Both cases are written only against APIs that exist on both trees: machine state, the radial's
per-tick `hitsThisTick` and `guardBlockedThisTick`, the inbound slice's `wasHitThisTick`, and the
synced `State` composite. Neither case reads `attackHits`, which task 27 deleted.

Task 27 added two more cases (§5, §6). They were written against the same APIs, so they compiled on
the tree before task 27 and were seen RED there, then GREEN after it.

Tags: `[SimulatableBrawler]` puts both cases in the `[@og]` whitelist; `[HitDedup]` selects them on
their own.

## 2. The rig

The rig is a parameterised copy of the one in `BrawlerHitDetectionBehaviourTest.cpp`, with the same
geometry, aims and swing timing (press at tick 16, contact on tick 40). It differs in one way: every
storage key and engine handle comes from a `Handles` value, so that two rigs can model two peers.
The original hard-codes keys 0/1 and bodies 30-33. That file is still in the old comment
convention, so extracting its rig into a shared header would have meant converting it too.

* `step(t)` runs `integrateAll(t)`, `firePostIntegrate(t)`, then the next step's
  `firePreIntegrate(t+1)`. That is production order, cut after the reduction over `t`, so a sample
  taken after `step(t)` sees what the machine reads on `t+1`.
* Weapon contact is scheduled (`weaponOverlapsTarget`). The query reports the target's body, and
  also its guard while the guard shape is enabled. The guard sub-simulation enables the shape
  whenever the target's machine is `Idle` (`DAttackGuardSimulation.h`, the shape toggle at the end
  of its integrate). So "guard dropped" in this file means the guard is **turned away** from the
  attacker, as in `BrawlerHitDetectionBehaviourTest.cpp` (`turnedAwayAim`). It does not mean the
  shape is disabled.
* Storage keys start at 1 because task 25 reserves `SimCharacterId` 0 for "no character". A
  `static_assert` enforces this.

## 3. Pin 1 — a guard block ends the swing, and the target is not hit later in it

**The measured rule.** When a swing is blocked on the reduction over tick T, the block is routed to
the attacker's own inbound slice (`BrawlerHitRoutingSystem.h`, branch 5). On `integrate(T+1)` the
machine goes `Attacking -> GuardFlinch` and sets the radial's `activeAttackSequence` to Invalid
(`DAttackMachineSimulation.h`, case `Attacking`). The radial integrates after the machine
(`SimulatableBrawler.h` sub-simulation order), so it deactivates on that same tick. Only `Attacking`
hosts a swing (`DAttackState` has four values). **A single attacker's swing therefore cannot hit a
target after a block**, whatever the target does next.

**The correction this pins.** The design's §1 P1 and the problem statement say a blocked target "may
still be hit later in the same swing". That is false for today's game: the detector does not record
the blocked target (the `break` of G-08 in `docs/BrawlerHitDetectionSystem-guards.md`), but the swing
it would be hit by is over. Task 26 was first specified as "block on t, guard dropped on t+1, hit on
t+1". A probe with the guard turned away on T, T+1 or T+2 never produced a hit, so that
specification could not be green. The user ruled on 2026-09-26 to pin the real rule instead.

**The case.**
* Section 1: the guard faces the attacker up to and including tick T (contact) and is turned away
  from T+1 on, with the weapon still overlapping. It requires the block on T, and it checks that the
  attacker is in `GuardFlinch` on T+1. Over every tick after T it checks that no hit is detected
  (`hitsThisTick` stays empty), that no hit is routed (`wasHitThisTick` stays false), and that the
  target never enters `HitFlinch`.
* Section 2 (control): the same swing with the guard turned away throughout and contact starting on
  T+1. The first hit is detected over T+1 and the target is in `HitFlinch` on T+2. This proves the
  weapon is still in the damaging segment on T+1 in this rig, so section 1's "no hit" is caused by
  the block and not by the swing's geometry.

**Poison, measured (task 26).** A temporary edit cleared the attacker's routed
`wasGuardBlockedThisTick` after every reduction, so the block was never routed. Section 1 went RED
on 3 checks: attacker `Attacking` on T+1, 1 tick with a hit, and the target in `HitFlinch` for 30
ticks. Section 2 stayed green. The edit was restored by hash. The poison also shows that without the
block's reaction the target *would* be hit later in the swing, which is what G-08 leaves possible.

**What task 27 could break here.** A synced ledger that recorded the blocked target, or an append
that read `guardHits`, would not show up in this case, because the swing ends. The case would catch
any change that lets a blocked swing continue: a machine or routing edit that drops the
`GuardFlinch` reaction.

## 4. Pin 2 — the attacker's synced bytes are identical on two peers whose engine handles differ

**The property (design §1 P3).** The correction gate compares integral fields exactly. A
per-process value in any synced field would therefore make every correction of that character
compare unequal: a permanent resim storm.

**What differs between the two rigs, and what does not.** Ruled by the user on 2026-09-26, and
enforced by `static_assert` on `kPeerA` / `kPeerB`:
* **Different:** every engine handle (root body, guard body, guard shape, radial query volume) and
  the registration order (peer B registers the target first). These stay per-process after task
  25.
* **The same:** the storage keys. From task 25 the key *is* the peer-stable `SimCharacterId`, so two
  rigs with different keys would model two different characters. Task 27's ledger would then
  differ legitimately and the pin would go red for the wrong reason. The brief's wording "different
  local ids" predates that; before task 25 the key was the per-process `GetUniqueID()`.

**The case.** Both rigs run the same swing, which hits the target on T (guard turned away). Every
tick from 0 to T+40, the case serializes each character's synced `State` composite with
`writeCompositeToSyncedBuffer` into a buffer pre-filled with `0xCD`, and compares the peers byte for
byte. The premises are that the target is in `HitFlinch` on T+1 in both rigs and that the attacker
is back to `Idle` by T+40, so the whole swing is sampled, including `deactivate`. Measured on task
26's tree: 326 B per attacker, 81 ticks, 0 mismatches for either character.

**What it catches after task 27.** A `hitTargets` entry written from a `BodyId`, or from any handle
other than the peer-stable `SimCharacterId`. It also catches a `BodyId -> SimCharacterId` map built
from the wrong key. The first mismatch appears on the hit tick, and the failure message gives that
tick and byte offset.

**Poison, measured (task 26).** A temporary edit wrote the attacker's `hitsThisTick[0].hitRootBodyId`
(a per-process `BodyId`) into the synced radial IC field `activeRootBodyId` after every reduction
that detected a hit. That is the shape of a ledger keyed by body. The case went RED: 41 attacker
mismatch ticks, the first on tick 40 at byte 20. Byte 20 is `activeRootBodyId`, the fourth radial IC
field, after 4 + 12 + 4 B. The edit was restored by hash. That field is retired by task 27, so the
poison cannot be re-run in that form after it. Plant the id into `hitTargets` instead.

**Poison, re-planted against the ledger (task 27).** After task 27, with the case GREEN at 325 B,
81 ticks and 0 mismatches, a temporary edit wrote `hitsThisTick[0].hitRootBodyId` (31 on peer A, 131
on peer B, cast to the 1-byte id) into `hitTargets[0]` after every step that detected a hit. The
case went RED: 19 attacker mismatch ticks (the hit tick up to the swing's `deactivate`), the first on
tick 40 at byte 80. Byte 80 is `hitTargets[0]`: the radial IC's 20 B, then `attackTimer` 4,
`currenSequenceId` 4 and `bodyState` 52. The edit was restored by hash.

## 5. Live adoption — an idle attacker adopted mid-swing hits on its next swing (task 27)

`HitDedup.AdoptingAnIdleAttackerMidSwingLetsTheNextSwingHit`. The swing hits the target over tick
40 (guard turned away, contact continuous from 40). After tick 45 (mid-swing, attacker `Attacking`)
the case overwrites the attacker's whole synced `State` with the one it held at the end of tick 15,
before the press: machine `Idle`, radial pair Invalid, ledger empty. That is what adopting a
correction whose swing has already ended looks like: capture 3's phantom. The attacker presses again
on tick 47.

* **RED before task 27:** 1 hit before the adoption, 0 after it. The derived ledger survived the
  overwrite, still held the target, and the new swing whiffed. Those were capture 3's three missed
  stuns.
* **GREEN after task 27:** 1 hit after the adoption, detected over tick 47, the press tick itself
  (the contact is continuous and this rig's weapon never rotates, so the first swing tick is
  already in the damaging segment). The adopted ledger is the empty one.

The first version of the case expected the hit at 47 + 24. That was the author's mistake, not the
code's: 24 is the first swing's press-to-contact gap, and it only exists because contact starts on
tick 40.

## 6. The cap — a swing hits at most three distinct targets (task 27, user ruling R3)

`HitDedup.ASwingHitsAtMostThreeDistinctTargets`, on its own rig: one attacker (key 1) and four
targets (keys 2-5), all at the same point in the swing's damaging segment, with guards turned away.
The query reports whichever targets the section schedules.

| section | before task 27 (RED) | after (GREEN) |
|---|---|---|
| four targets from tick 40, all in one pass | 4 detected in one pass, all 4 hit on 40 | 3 in one pass, targets 1-3 hit on 40, target 4 never |
| four targets on ticks 40, 41, 42, 43 | all 4 hit (40, 41, 42, 43) | 3 hit, target 4 never |
| control: only the fourth target, from tick 43 | hit on 43 | hit on 43 |

The control proves that the fourth target's "never" is the cap and not the rig. The rig needs its
own `Physics` guard body id (999): with the default 0, the targets' guard sub-simulations wrote body
0, which is also the attacker's unbound weapon body, and the swing never reached a damaging segment.
That was found by the control going RED on the first run.
