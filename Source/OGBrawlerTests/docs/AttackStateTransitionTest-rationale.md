<!-- SPDX-License-Identifier: BUSL-1.1 -->
# AttackStateTransitionTest — rationale

Source: `AttackStateTransitionTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for the attack state machine's transitions between attacks (`dAttackMachineSimulation::integrate3`),
added by og-attackstatetransition-cleanup. Task 1 wrote the rig and the rapid-fire cases (§3). Task 2
added the same-side fast follow-up cases (§4). Task 4 removed the shooter's recoil on a projectile
block and lengthened the Hadouken commitment to 53 ticks (§5). Task 6 shortened it to 45 ticks (§6).

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[AttackTransitions]` selects them on
their own.

## 2. The rig

`FAttackRig` drives one `SimulatableBrawler` through `SimulatableBrawler::integrate`, the same
character across ticks, like `DAttack.Integrate3.HadoukenCommitmentHoldsAttackingState`. Since task 4
the character lives in a `SimulationObjectStorage` (id 0), so the rig can run the production routing
system, `brawlerHitRouting::System`, over it (`character` is a reference into the storage). The file
has its own two stateless adapter mocks rather than sharing one with `IntegrateThreeAttackSelectionTest.cpp`,
which is still in the old comment convention.

* **Ticks start at 1.** A projectile slot whose `spawnTick` is 0 reads as free, so a shot on tick 0
  could not be told apart from no shot.
* **The input.** `aim` (east by default), `moveWorld` and `moveStick` persist across ticks; `tick`
  takes the two buttons and the matcher's `triggeredActionId` for that tick. `holdStick(angle)` sets
  the stick at an aim-relative angle in the convention of `InputSequence.h` (0 is the aim, `+π/2`
  is right of it, `π` is dead-back); `holdStickOffBack(offset)` is `holdStick(π + offset)`.
  `moveStick` is set to the XY of `moveWorld`, so the classifier's magnitude gate and the matcher's
  read agree.
* **The inbound slice is cleared after every tick**, the one-shot reset the routing pass performs in
  production. A case that delivers a hit or a block writes the slice before its tick.
* **`dt`** (task 4) is a rig member, `1.f / 60.f` by default. `tick` and `commitmentTicks` use it, so a
  case can run at the engine's fixed step 0.016667 as well.
* **A projectile block** (task 4). Setting `blockLiveShotOnNextTick` makes the next `tick` pose what
  the detector reports for a guard block: the newest live slot (spawned before this tick) gets the
  `BlockedByGuard` outcome in the projectile `DerivedState::detectedThisTick`, and the routing system's
  `preIntegrate` runs on that step before the integrate, as in production. The outcome is cleared after
  the tick, as the detector's next pass would. `blockedSlot()` is a copy of the blocked slot taken right
  after that tick's integrate (a later shot may reuse the slot). The injection point is the detector's
  output, so the same case compiled on the code before task 4, where routing turned the block into the
  shooter's recoil.
* `everIn(state)`: whether any recorded tick was in that state.
* **The log.** The rig installs an `ogblog` sink for its lifetime and restores the previous one in
  its destructor. `logLinesContaining` counts captured lines.
* **One `TickRecord` per tick**, read after the tick: machine state, active and queued sequence, end
  tick, and two derived flags.
  * `fired`: `Attacking`, the active sequence is `kHadoukenSequenceSentinel`, and
    `m_timeInCurrentState` is exactly 0. Every Hadouken launch resets it to 0 after the tick's
    `+= dt`, and no other tick under the sentinel leaves it at 0.
  * `spawned`: some projectile slot's `spawnTick` equals this tick. The projectile sub-simulation
    runs after the machine on the same tick and consumes the spawn request, so this is the request
    observed through its effect. `slotReuses` counts spawns into a slot that had spawned before.
* **Task 2's readers** (over `history`, which a case may clear to start counting afresh):
  * `swingSequences()`: the active sequence of every swing start, in order. A swing starts on a tick
    that is `Attacking` on a real sequence when the tick before was not, or held another sequence, or
    another end tick. A chain into the same id (3 → 3) is still a new start, because the chain
    rewrites the end tick.
  * `queuedSequences()`: every queued sequence seen, consecutive repeats folded.
  * `trace()`: one token per tick, `tick:State/active`, with `qN` for a queue, `H` for the projectile
    sentinel and `*` for a spawn. Every case prints it on failure.
  * `run(n, left, right)` holds the buttons for `n` ticks; `fireAndSettle` fires one projectile from
    dead-back and runs to two ticks past its `Idle`.
* **The authored follow-up ids are literals in the test** (`kAuthoredRightFollowUp` 2,
  `kAuthoredLeftFollowUp` 3), not the header's constants, so that the behaviour cases compiled on the
  code before task 2. `FollowUpPressIsTheClassifiedSideWithoutProjectileIntent` checks that the
  header's `kRightFollowUpSequenceId` and `kLeftFollowUpSequenceId` equal them. `kRightOfAim` (+π/2)
  and `kLeftOfAim` (−π/2) are the sticks `dAttackDirection::classify` calls right and left; every case
  `REQUIRE`s the classifier's answer before it relies on it.

## 3. Task 1 — rapid fire while back + attack 2 are held

The machine side is described in `DAttackMachineSimulation-rationale.md` §6.4 and §6.5. Every
expected tick count is spelled `swingTickCount(kHadoukenCommitmentSeconds, dt)`, never 18. So all
seven cases followed tasks 4 and 6 (53 ticks, then 45) unedited. `NoShotDroppedAtDefaultPool` still
passes: a slot lives 53 ticks, so at the 45-tick period two shots are in flight for 8 ticks of every
45, and the pool has 3 slots.

| case | what it pins | on the code before task 1 |
|---|---|---|
| `DAttack.AttackTransitions.HeldBackAndAttack2RefiresProjectile` | dead-back + attack 2 held, the matcher's action on the first tick only: 7 shots, one every period; the machine never holds a real sequence; every shot rewrites the end tick to shot tick + period | **failed**: one shot (tick 1), `Idle` on tick 19, sequence 4 from tick 20 |
| `DAttack.AttackTransitions.NoShotDroppedAtDefaultPool` | 10 shots at the shipped `StaticData` and 60 Hz: every shot tick is a distinct slot `spawnTick`, no `[Projectile.poolFull]`, and slots were reused (so recycling was exercised) | failed: one shot |
| `DAttack.AttackTransitions.ReleaseStopsFire` | release after the second shot: `Attacking` with the sentinel until exactly one period after it, then `Idle`, no further shot; stick moved 30° off dead-back with attack 2 held: no shot, and the sentinel is left on the commitment's last tick | the first section failed (no second shot); the second passed |
| `DAttack.AttackTransitions.HeldFromIdleFires` | `Idle`, no matcher action, back + attack 2 held: fires on that tick, end tick = tick + period | failed: sequence 4 |
| `DAttack.AttackTransitions.BackConeIsTheMatchersCone` | 15° off dead-back (inside the matcher's π/8, outside the classifier's 10° backpedal cone, both asserted) fires; 30° off gives the classified swing; both sides | failed: the 15° rows gave the classified side swing |
| `DAttack.AttackTransitions.StickMatchesStepAtTheBoundaries` | `inputSequence::stickMatchesStep` 0.1° inside and outside the tolerance on both sides, below and above the deadzone, the zero stick, an aim with a Z component, a stick with a Z component, an aim with no XY | passed (it tests the new predicate only) |
| `DAttack.AttackTransitions.StickMatchesStepIsTheMatchersPerFrameTest` | motion 3 of `kGameMotions` is exactly `kBackShortcutStep` (Back, π/8, 8); for every whole degree around a tilted aim, a one-entry `inputSequence::matchSequence` on motion 3 matches exactly when the predicate does, and the sweep has both outcomes | passed |

The last two pass on both sides of the change by construction: they pin the predicate and its
agreement with the matcher, so that the two cannot drift apart later.

## 4. Task 2 — the same-side fast follow-up, and none from a projectile

The machine side is described in `DAttackMachineSimulation-rationale.md` §14 and guards G-09 and G-10.
"Before task 2" is the code as task 1 left it. Attack 1 is `attackLeft` and attack 2 is `attackRight`.

| case | what it pins | on the code before task 2 |
|---|---|---|
| `DAttack.AttackTransitions.LeftSwingFollowUpWithAttack1` | left stick + attack 1 held: swings 1, 3, 3 | **failed**: swings `{ 1, 1, 1, 1, 1 }`, every swing restarted from `Idle` |
| `DAttack.AttackTransitions.RightSwingFollowUpWithAttack2` | right stick + attack 2 held: swings 0, 2, 2, and no projectile | **failed**: `{ 0, 0, 0, 0, 0 }` |
| `DAttack.AttackTransitions.PreviouslyWorkingFollowUpsStillChain` | right + attack 1: 0, 2, 2; left + attack 2: 1, 3, 3 | passed: these are the two pairings the button-keyed rule got right |
| `DAttack.AttackTransitions.OppositeOrForwardPressIsNotAFollowUp` | after one tick of a side swing, the stick moves to the other side, forward, or neutral, the button still held: nothing is queued, the machine is `Idle` on the swing's end tick, and the next swing is the classified one (other side, or 4). Both buttons × both sides × three sticks | **failed** on the 6 rows where the button matched the old rule (right + attack 1, left + attack 2): the follow-up was queued anyway |
| `DAttack.AttackTransitions.FirstSwingAfterProjectileIsNormal` | a projectile, its `Idle`, then a side swing held with either button: the first swing is 0 or 1, the second its follow-up | **failed** on 2 of 4 rows (left + attack 1, right + attack 2), for the same reason as the first two cases; the first swing was 0 or 1 on every row |
| `DAttack.AttackTransitions.ProjectileInputMidSwingIsNotAFollowUp` | during sequence 1 past 0.3 s, attack 2 held 15° off dead-back on the side the classifier calls left (both `REQUIRE`d): nothing queued, `Idle` on the swing's end tick, a projectile on the next | **failed**: 3 queued from tick 25, the swing chained into 3, no projectile |
| `DAttack.AttackTransitions.DualPressCannotHijackProjectile` | a three-tick dual press with a neutral stick during the commitment: the sentinel stays until the commitment ends, `Idle` then, no real sequence, one shot. Both buttons held with the stick back: a shot every period and no pool overflow | **failed** both sections: `Idle` on the press tick and sequence 4 from the next; with the stick back a shot every second tick and 35 `[Projectile.poolFull]` lines |
| `DAttack.AttackTransitions.FollowUpPressIsTheClassifiedSideWithoutProjectileIntent` | `followUpPressSequence` for every whole degree around a tilted aim, both buttons and none: no button gives none, a projectile input gives none (matcher action or the held test), anything else gives `classify`'s answer whichever button; the header's follow-up ids are 2 and 3; the sentinel maps to none | not compiled (the functions did not exist). It fails when the projectile test is deleted from `followUpPressSequence` (G-09 poison) |
| `DAttack.AttackTransitions.DualPressOnAGuardBlockTickQueuesNothing` | a dual press on the tick a guard block arrives, inside the 0.1 s restart window and past 0.3 s: `GuardFlinch` with nothing active or queued, nothing queued after the recoil, and the next swing chains into its own follow-up | added after the pre-change run. It fails on both rows when the dual-press gate is deleted (G-10 poison): 4 active, or 4 queued and chained after the recoil (`{ 0, 4, 0, 2, … }`) |

The two poison builds each failed only the cases named in their guard, and the header was restored
byte-identical (md5) after each.

⚠ **Task 4** changed one line in `ProjectileInputMidSwingIsNotAFollowUp`. The case needs a tick past
the swing's 0.3 s follow-up window and spelled it `1 + commitmentTicks() + 6`. That was right only
because the old commitment was also 0.3 s. It is now `1 + swingTickCount(kFollowUpWindowSeconds, dt) + 6`,
with `kFollowUpWindowSeconds` = 0.3, the `attackTimer > 0.3` test in `integrate3`. The value is
unchanged (tick 25).

## 5. Task 4 — no shooter recoil on a block; a 53-tick lock after every shot (45 since task 6)

The machine side is described in `DAttackMachineSimulation-rationale.md` §7.2 and §15, and in
guard G-04's retirement (`DAttackMachineSimulation-guards.md` §R). "Before task 4" is the code as task 2
left it.

| case | what it pins | on the code before task 4 |
|---|---|---|
| `DAttack.AttackTransitions.ProjectileBlockDoesNotFlinchShooter` | a block 13 ticks into the flight, through the production routing pass: mid-commitment, the shooter stays `Attacking` under the sentinel with its end tick unchanged and goes `Idle` exactly one period after the shot; in rapid fire, the second shot is blocked and the stream keeps its schedule; in `Idle` (a 0.05 s hit ends the commitment early: the block comes 13 ticks into the flight, long before the plain exit to `Idle`, so a hit is what puts the shooter in `Idle` with its shot still in the air) the shooter stays `Idle`. Every section: no `GuardFlinch`, the slot ends with `endReason` 4 on the block tick | **failed** all three sections: `GuardFlinch` on the block tick; in rapid fire `Idle` 18 ticks later and the shots `{ 1, 19, 51, 69 }` instead of every 18 |
| `DAttack.AttackTransitions.CommitmentIs45Ticks` (task 4 wrote it as CommitmentIs53Ticks, pinning 53; task 6 retargeted it, §6) | at `dt` = `1.f / 60.f` and 0.016667: the commitment is a literal tick count, on purpose; one shot holds `Attacking` for exactly that many ticks, its end tick is shot tick + that count, `Idle` on that tick; held fire shoots once per that count (task 4: ticks 1, 54, 107, 160) | **failed**: 18 at both `dt` values |
| `DAttack.AttackTransitions.MeleeGuardBlockRecoilIsUnchanged` | a guard block on a side swing's tenth tick: `GuardFlinch` on the block tick, `Idle` exactly 18 ticks later, at both `dt` values | passed, 18 (the measurement this pin was taken from) |
| `DAttack.AttackTransitions.CommitmentLeavesThePunishWindow` | for sequences 0, 1 and 4, k = `swingTickCount` of the start time of the segment holding local angle 0 (from the shipped `StaticData`, no physics); the commitment is at least the slowest k + `kAssumedInputDelayTicks` (6, the shipped `RelayDelayFloorTicks`) + `kPunishReactionTicks` (10) + `kMaxPunishableFlightTicks` (13 under task 4, a block at the 300 cm edge of swing reach; 5 since task 6, §6), at both `dt` values | **failed**: 18 ≥ 53 is false. Measured k = 24, 24, 18 |

The punish-window case also failed with the constant poisoned to `0.3f` on top of task 4's code (the
witness its acceptance criterion asks for).

The 53-tick lock also ended the point-blank projectile-hit → side-swing combo: the follow-up swing
connected 32 ticks after the target's 0.65 s stun ends (24 ticks after it under task 6's 45). The user
dropped the combo (2026-10-04). Its pin in `BrawlerHitRoutingTest.cpp` lost its follow-up half and is now
`HitRouting.ProjectilePointBlankHitStunsOnce` (`DAttackMachineSimulation-rationale.md` §15).

## 6. Task 6 — the lock shortened to 45 ticks

The machine side is described in `DAttackMachineSimulation-rationale.md` §15: the user shortened
`kHadoukenCommitmentSeconds` to `0.74167f` (45 ticks, a half-tick literal) on 2026-10-04, and the
guaranteed punish flight went from 13 ticks (about 3 m) to 5 (about 2 m). "Before task 6" is the code
as task 4 left it (53 ticks).

| case | change | on the code before task 6 |
|---|---|---|
| `DAttack.AttackTransitions.CommitmentIs45Ticks` | renamed from CommitmentIs53Ticks; its literal is 45 | **failed** at both `dt` values: commitment `53 == 45`, end tick `54 == 46`, shots `{ 1, 54, 107 } == { 1, 46, 91, 136 }` |
| `DAttack.AttackTransitions.CommitmentLeavesThePunishWindow` | `kMaxPunishableFlightTicks` 13 → 5, so the sum is 24 + 6 + 10 + 5 = 45 | passed (53 ≥ 45): the pin is an inequality, so a longer lock always meets it |

The punish-window sum is met exactly, with no slack: with the constant poisoned to `0.725f` (44 ticks)
on top of task 6's code, it failed at both `dt` values (`44 >= 45`), and so did
`CommitmentIs45Ticks`. Every other case that depends on the lock spells its counts with
`commitmentTicks()` and followed the change unedited; `NoShotDroppedAtDefaultPool` now runs with two
shots in flight at a time (§3).
