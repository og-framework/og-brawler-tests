<!-- SPDX-License-Identifier: BUSL-1.1 -->
# SupportLawScorecardTest — rationale

Source: `SupportLawScorecardTest.cpp`. This file has no guards document: nothing in it needs a `⛔` tag. The harness is
described in `GoldenTraceScene-rationale.md`, the slope statics in `GoldenTraceSlopeScenario-rationale.md`. The third
scorecard rig, the platform edge, lives in `PlatformEdgeTest.cpp` (`PlatformEdgeTest-rationale.md` §8).

`SupportLawScorecard.FlatKnockbacks` and `SupportLawScorecard.CreaseCrossings` (`[GoldenTrace]`, no golden) are two of
the three measuring rigs of the ground-aligned attacks initiative's task 31: the scorecard the support law v2 is gated
on (the initiative's design for the support law, §9). They knock a brawler back, or walk it, with real Jolt geometry
and the production path, and count what its support classification does. Each rig prints one totals line,
`[SupportLaw.scorecard] law=… rig=…`, and pins those totals for the Legacy law, which shipped until the initiative's task 36 (§6). Tasks 33 to 35 run the same rigs
under the support law v2 and compare.

## 1. The scene and the law

- **`RigScenario`** is a `goldenTraceScene::Scenario` whose characters, input script and teleports are set before each
  `goldenTraceScene::Scene` is built (`RigScenario::set`); the cases run one after another, as in `PlatformEdgeTest.cpp`.
  The target is id 1 (`kTarget`). Attackers, a stunner and a shooter take the next ids.
- **Two worlds.** `flatStatics` is one 60 m box whose top is at y = 0 (the flat slab). The crease rig uses
  `goldenTraceSlopeScenario::slopeStatics`, the slope golden's statics: the landing (top y = 2), the 30° face and the
  plateau (top y = 14). `slopeSurface` and `slopeNormal` give the analytic surface point and face normal under a
  horizontal position.
- **Static data** is `scorecardStaticData`, which is `goldenTraceScenario::scenarioStaticData`: the shipped movement,
  attack and reaction tables (melee knockback 20 m/s, `launchDecel` 40, so the knockback lockout is 0.5 s).
- **The law is read from the scene's movement StaticData only.** `supportLawName` returns `v2` when that
  StaticData's `supportLaw` is `SupportLaw::V2`, else `legacy`. Since og-ground-aligned-attacks task 33 each rig runs
  under both laws in one binary: `SupportLawScorecard.FlatKnockbacks` and `SupportLawScorecard.CreaseCrossings` under
  Legacy, `SupportLawScorecard.FlatKnockbacksUnderV2` and `SupportLawScorecard.CreaseCrossingsUnderV2` under V2. Each
  case sets `gSupportLaw` through a `SupportLawScope`, and `RigScenario::configureStaticData` writes it into the movement
  StaticData before the scene registers the characters (`GoldenTraceScene-rationale.md` §1; the StaticData cannot be
  copied, so `scorecardStaticData` cannot set it). The rigs observe the stepped state only. They never call a movement helper, never
  name the compile-time knob, and pass no explicit template argument, so they measure whatever law the scene runs.
- **Every knockback is production's.** A melee knockback comes from a real swing: hit detection, hit routing (the
  inbound-hit slice from the swing tangent) and the movement's knockback write. The projectile-on-flinch knockback comes
  from a real Hadouken that hits a target already in HitFlinch (§4.3).

## 2. The per-tick record (`runRig`)

- **Before each step,** the target's movement probe volume is swept 20 m (`kClearanceSweepM`) straight down from the
  body position. The fraction gives the clearance, the same quantity the movement's probe measures within its own
  0.5 m reach (`rideHeight` + `snapDistance`). The query sees statics only, so the other characters do not matter.
- **After the step** it records: the support bits of `State::flags`, `DerivedState::lastSupportState`, the post-step
  velocity, whether the attack machine is in HitFlinch, and the inbound-hit slice (hit, reaction, speed, world direction).
- **For the redirect check** (task 35, §6.2) it also records: the production estimate of the pre-step State
  (`estimateSupport`: its face normal and the probe's clearance), whether the movement is committed
  (`machineLaunchesMovement`), and the velocity that enters the classification and the redirect
  (`velocityEnteringTheRedirect`: the pre-step velocity after step 1's push-out, then V2's hit-tick write). These
  sweeps change nothing: the query is a read of the static scene.
- **The support cross-check.** The flags' support bits must equal `lastSupportState` on every recorded tick
  (`supportSourceMismatches`, asserted 0 in both cases). The flips and Unsupported counts read the flags; the check
  shows they read the right bits.

## 3. Definitions shared by both rigs

- **The hit tick** is the first tick (from a given tick on) whose inbound slice is a Knockback hit (`firstHitIndex`).
  **The lockout end** is the first later tick whose machine is no longer in HitFlinch (`lockoutEndIndex`).
- **Flips** (`transitionsIn`): ticks whose support differs from the tick before. The hit tick counts, so a detach on the
  hit tick is a flip.
- **Unsupported ticks** (`unsupportedIn`): ticks whose support is Unsupported.
- **Apex** (`apexIn`): the largest clearance above `rideHeight` over the ticks whose sweep hit the ground. A value at or
  below 0 means the target never rose above its hover height.
- **Slide-end ticks** (`slideEndTicksIn`): Unsupported ticks with ground within the probe's reach (clearance at most
  `rideHeight` + `snapDistance`) and a speed below 15 % of the melee knockback speed (`kSlideEndFraction`, 3 m/s): the
  target is detached while its probe still finds ground, and the slide has nearly stopped. Airborne ticks (a fall off an
  edge, a hop) are not slide-end ticks.

## 4. The rigs

### 4.1 The attacker search (`bestMelee`)

- The attacker stands beside the target, perpendicular to the wanted knockback direction, and aims at it horizontally.
  The radial hit's swing tangent is then along the wanted direction (`PlatformEdgeTest-rationale.md` §3).
- `meleeAttempt` places it at 1.6 m and then 1.2 m (`kAttackerRadii`), on either side, with either swing, shifted 0 or
  ±0.45 m along the wanted direction (`kAttackerShifts`). Each attempt is a full run. The search keeps the hit whose
  direction is closest to the wanted one (the in-face direction on a slope), requires a cosine of at least 0.82, and
  stops at 0.99.
- An attempt is skipped when the attacker would stand within 1 m (`kPathClearanceM`) of the target's path before the
  hit (the walk, or the first slide).
- **Measured on 2026-11-03:** whether a side swing connects with an unguarded target 1.6 m in front depends on the
  target's facing. With the attacker at (x, z) = (0.49, −1.52) aiming at a target at the origin, the right swing finds no hit
  for target facings from 315° to 15°, and the left swing none from 210° to 270°. The other facings connect at tick 85.
  The search tries the other side or swing, so every placement is reached; the cause was not investigated (routed to
  the initiative's lead).

### 4.2 Flat knockbacks (`runFlatRig`)

- **Directions:** 20, every 18° from +X (`kFlatDirections`, `kFlatStepDeg`).
- **Three target states:**
  - **rest:** the target settles for 60 ticks and is hit by a swing started at tick 61 (`kSwingTick`); the hit lands at
    tick 85;
  - **walk:** the target walks along +X at the shipped walk speed (1 m/s) from tick 30 until 6 ticks after the expected
    hit, and the attacker stands beside the place a dry run puts it at tick 85;
  - **midSlide:** a first swing knocks the target along +X (`kFirstKnockback`); a second attacker stands beside where the
    first slide puts the target 18 ticks (0.3 s, `kMidSlideDelayTicks`) after the first hit, and swings 24 ticks
    earlier, or 3, 6 or 9 ticks later still (`kMidSlideSwingDelays`). The second hit must land 14 to 22 ticks after the
    first. The first attacker is teleported 25 m away 2 ticks after its hit (`kFirstAttackerParking`), so no second
    slide can run into it.
- **Per run,** from the hit tick to the lockout end (`measureFlat`): flips, Unsupported ticks, the apex, the slide
  distance (horizontal travel from the hit tick to the lockout end), slide-end ticks, the clearance on the hit tick, and
  the vertical speed on the tick before the hit (`preHitVy`, the velocity the flat lift test reads on the hit tick).
- **Lines:** `[SupportLaw.flat.state]` per state (runs, flips, `flickerRuns` = runs with at least 2 flips, Unsupported
  ticks, apex maximum, slide minimum and maximum, slide-end ticks), and the totals line
  `[SupportLaw.scorecard] law=… rig=flat runs=… flips=… apexMaxM=… slideEndTicks=…`.

### 4.3 Crease crossings (`runCreaseRig`)

- **Four crossings** (`kCrossings`), all at x = 0 on the slope golden's statics:
  - `flatToSlope`: from the landing up across the concave crease (entering);
  - `slopeToFlat`: down the face across the concave crease;
  - `slopeToPlateau`: up the face across the convex crease (the ski jump);
  - `plateauToSlope`: from the plateau down across the convex crease.
- **Melee runs:** the target rests 0.3, 1.5, 3.5 or 4.5 m before the crease (`kCreaseDistances`), measured along the
  surface it stands on (`creaseStart`), and is knocked along the fall line or 30° either side of it
  (`kCreaseObliqueDeg`). 12 per crossing.
- **One projectile-on-flinch run** per crossing (`projectileOnFlinch`), at 0.3 m along the fall line:
  - a stunner beside the target swings the overhead (a Stun, 0.72 s, the target does not move), and a shooter 2 m
    behind the target fires a Hadouken along the fall line;
  - two dry runs time it: one finds the stun's tick, one finds the shot's flight time;
  - the final run fires so that the shot lands 12 ticks (`kShotDelayTicks`) after the stun. Hit routing then selects the
    projectile-on-flinch row, a 17.89 m/s Knockback along the shot's direction, which on the face is in the face plane.
- **One walk** per crossing: from 1 m before the crease along the fall line, for 120 ticks from tick 61.
- **Per run** (`measureCrease`):
  - **the crossing tick:** the first tick from the hit (or the walk's start) whose start position is past the crease
    line in z. A run that never reaches it prints `crossed=0`.
  - from the crossing tick to the lockout end (to the walk's end for a walk): Unsupported ticks and flips;
  - from the crossing tick to the end of the run: the apex, and **`hop`**, which is set when some Unsupported tick with
    ground below has a clearance more than 5 cm above `rideHeight` (`kHopApexM`); `offWorld` when the sweep found no
    ground (none did);
  - the speed on the tick before the crossing (`speedBefore`) and the speed in the destination face's plane on the
    crossing tick (`inPlaneAfter`);
  - the vertical speed on the crossing tick (`vyDepart`) against `vSinTheta`, `speedBefore` times sin 30°. At the ski jump
    a body leaving the face rigidly keeps +V sin θ; at the convex crease going down, following the face needs −V sin θ.
    `convexVyErrMaxMps` on a convex crossing's line is the largest |vyDepart − that value| of its runs (0 on the concave
    lines, which do not compute it);
  - slide-end ticks from the hit to the lockout end (none for a walk).
- **Lines:** `[SupportLaw.crease.crossing]` per crossing, and the totals line
  `[SupportLaw.scorecard] law=… rig=crease runs=… flips=… hops=… apexMaxM=… slideEndTicks=…`.

## 5. What it prints

- By default only the per-state and per-crossing lines and the totals lines (task 23's review asked for quiet `[@og]`
  logs).
- With the environment variable OG_SUPPORT_LAW_RUNS=1 it also prints every attempt (`[SupportLaw.attempt]`), every run
  (`[SupportLaw.flat.run]`, `[SupportLaw.crease.run]`), every run's ticks (`[SupportLaw.flat.tick]`,
  `[SupportLaw.crease.tick]`), and the pinned totals (`[SupportLaw.pin]`). The same switch turns on the platform-edge
  rig's per-run lines.
- A placement that no attempt reaches prints `[SupportLaw.flat.unreachable]` or `[SupportLaw.crease.unreachable]` and
  fails its CHECK.
- **Every line of a V2 case says so** (since task 33). A line whose format has a law field prints law=v2; every other
  line gets ` law=v2` right after its bracket tag (`lawTag`), which is empty under Legacy. So the Legacy lines are
  byte-identical to the tree before task 33, and dropping every line that contains law=v2 leaves exactly the Legacy
  output. The platform-edge rig follows the same rule (`PlatformEdgeTest-rationale.md` §8).

## 6. What it asserts

- Every placement is reached: 60 flat runs, and 56 crease runs (4 × (12 + 2)).
- The support cross-check of §2: 0 mismatches.
- **The Legacy law's totals are pinned** (`checkPin`, only while `supportLawName` is `legacy`; floats to 5e-4,
  `kPinTolerance`). The pins are what make every counter seen to fail (§7), and they make the rigs part of the
  bit-identity check of a task that must not change the law (tasks 32 and 33). A task that changes the Legacy law on
  purpose re-measures and re-pins them, and explains each change. Task 36 made V2 the shipped law without changing
  Legacy, and every rig line of both laws stayed byte-identical, so the pins stayed.
  - flat: flips 1803, flickerRuns 60, Unsupported 900, slide-end ticks 120, apex maximum 0 and sum −0.0214 m, slide
    minimum and maximum 5.1658 m and sum 309.9488 m;
  - crease: crossed 50, flips 79, hops 40, Unsupported 792, slide-end ticks 35, offWorld 0, apex maximum 5.0532 m and sum
    72.6148 m, in-plane speed minimum 0.5326, maximum 20.8478 and sum 592.9659 m/s, convex departure error maximum
    9.5914 and sum 88.2606 m/s.
- The measured lines and their reading are in the initiative workspace, impl/support_law_baseline.md.

### 6.1 The V2 checks (og-ground-aligned-attacks task 34)

Since task 34 the V2 cases check V2's own results:
- **flat:** flips, flickerRuns, Unsupported and slide-end ticks are all 0 (`checkPin`, while `supportLawName` is
  `v2`). Under Legacy the same counters are 1803, 60, 900 and 120.
- **crease** (`checkV2Crease`), over the committed convex crossings: the runs of slopeToPlateau and plateauToSlope by
  a melee or projectile hit whose crossing tick comes before the lockout ends (`committedCrossing`). Two slow
  slopeToPlateau runs cross on the tick the lockout ends; they are walkers by then, and follow the crease.
  - Each departs (`departTick`, the first Unsupported tick from the crossing tick to the lockout's end) on the
    crossing tick or the next one.
  - On the departure tick its vertical speed equals the previous tick's plus one gravity step, within
    `|gravity| · dt` (`ballisticErrMps`).
  - There are 22 such departures.
  - No concave crossing (flatToSlope, slopeToFlat) hops (`hop`, §4.3).
- **Lines** (V2 only). With OG_SUPPORT_LAW_RUNS=1 each committed convex crossing prints
  `[SupportLaw.crease.departure] law=v2 crossing=… kind=… d=… obl=… cross=… depart=… ballisticErrMps=…` after its
  run line. The case always prints one totals line,
  `[SupportLaw.crease.departures] law=v2 convexDepartures=… worstBallisticErrMps=… gravityStepMps=… concaveHops=…`.
- **Seen failing** (task 34, each a poison of `classifySupportV2` in og-brawler, built and run under the lock, then
  restored byte for byte):
  - the launch row deciding Unsupported at `dot(d, n) >= 0`, so a horizontal hit on flat ground launches: flat flips
    `120 == 0`, flickerRuns `60 == 0`, Unsupported `60 == 0`;
  - the separation row disabled: all 22 departures fail `r.departTick <= r.crossTick + 1u` (for example `89 <= 87`);
  - the separation row without its `clearance >= rideHeight` term (the hover rule of V2's transitions): `concaveHops == 0u`
    fails, `18 == 0`.

### 6.2 The redirect check (og-ground-aligned-attacks task 35)

Since task 35 `checkV2Crease` also checks the redirect into the face on each committed concave crossing (flatToSlope
and slopeToFlat, by a melee or projectile hit, crossing before the lockout ends):
- **0 Unsupported ticks** from the crossing to the lockout's end.
- **The redirect tick** is the first tick from the hit whose recorded estimate is Supported on the destination face
  (`kSameFaceCos`, within 1°). Two slow slopeToFlat runs (d = 4.5 m, ±30°) cross at about 1.3 m/s on the lockout's last
  ticks and reach the flat face only after it (`faceAfterTheLockout`, 2).
- **The speed along the face.** With `v` the velocity entering the redirect (§2), `V = ‖v‖` and `n` the destination
  face, the expectation is `sqrt(V² − dot(v, n)²)`, which is `V · cos θ` for the crossing's angle `θ` into the face.
  One tick of committed decay (`launchDecel · dt`, clamped at 0) is subtracted on a tick that is not the hit tick,
  because step 3 decays every committed tick except a hit tick. The measurement is the post-step velocity without its
  up channel, `velocity − kWorldUp · dot(velocity, n) / n.y`: that is the tangent channels' speed, which the servo does
  not touch. They agree within `kRedirectSpeedTolerance · V` (1e-3).
- **22 such crossings** and 2 that reach the face after the lockout, both pinned.
- **slopeToPlateau crosses 14 of 14** (since task 35's rework 1; `slopeToPlateauCrossed`, through `checkPin`). It is
  the pin of the redirect's hover term: without the term the two runs at d = 4.5 m, ±30°, float over the convex crest
  and stop short of the crease line (the movement's rationale §35).
- **Lines** (V2 only). With OG_SUPPORT_LAW_RUNS=1 each committed concave crossing prints
  `[SupportLaw.crease.redirect] law=v2 crossing=… kind=… d=… obl=… cross=… redirect=… speedIn=… thetaDeg=… expectedMps=…
  inPlaneMps=… errMps=… minProbeClearanceM=… unsupported=…` after its run line, where `minProbeClearanceM` is the
  smallest pre-step probe clearance from the redirect tick to the lockout's end. The case always prints
  `[SupportLaw.crease.redirects] law=v2 concaveCrossings=… faceAfterTheLockout=… unsupported=… worstErrOverV=…
  minProbeClearanceM=…`.
- **Seen failing** (task 35, the redirect's assignment removed from og-brawler's `integrate`, built and run under the
  lock, then restored byte for byte): the 11 flatToSlope crossings fail the speed check, for example `5.75135f <= 0.02f`
  at d = 0.3 m along the fall line (22.4273 m/s against 16.6760 m/s: the oblique split's `V / cos 30°` less one tick of
  decay). The slopeToFlat crossings pass without it: there the decomposition onto the flat face's tangent channels
  already keeps `V · cos θ`, and the up channel carries the rest.
  Task 35's rework 1, the redirect's hover term deleted (the same procedure): the slopeToPlateau pin fails, `12 == 14`,
  with the message `rig crease field slopeToPlateau.crossed measured 12 pinned 14`, and nothing else in `[@og]` fails.

## 7. Seen failing (task 31, built and run under the lock, then restored byte for byte)

One poison per counter, each built and run. Each failed its own pinned field and moved no other counter's pin, except
where the counters share the poisoned input:
- the support bits read one bit too high: the cross-check failed in both cases (`sourceMismatches == 0u`), and with it
  every count read from the flags (flat flips 0, Unsupported 1860, slide-end 300; crease flips 0, Unsupported 1180,
  slide-end 78);
- flips counting only Unsupported → Supported: flat 903, crease 28;
- Unsupported counting Supported ticks: flat 960, crease 388;
- slide-end with the speed test inverted: flat 780, crease 454;
- apex without subtracting `rideHeight`: flat maximum 0.1 and sum 5.9786, crease maximum 5.1532 and sum 77.6148;
- slide distance from x and y instead of x and z: flat minimum 0 and sum 192.877;
- flickerRuns at 40 flips instead of 2: 0;
- `hop` with the clearance test inverted: 27;
- `inPlaneAfter` as the whole speed: minimum 0.6663, maximum 20.8507, sum 646.065;
- `vSinTheta` from the fall line's z instead of its y: maximum 16.6679, sum 208.792;
- `offWorld` with the test inverted: 50;
- the crossing test inverted (every run crosses on its first tick): crossed 56, with flips, hops, Unsupported and the
  speed sums moving with it.
