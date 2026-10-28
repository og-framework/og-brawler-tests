<!-- SPDX-License-Identifier: BUSL-1.1 -->
# GoldenTraceSlopeScenario — rationale

Source: `GoldenTraceSlopeScenario.h`. This file has no guards document: nothing in the header needs a `⛔` tag. The
harness that runs the scenario is described in `GoldenTraceScene-rationale.md`, the test in
`BrawlerJoltSlopeGoldenTraceTest-rationale.md`. Written by og-ground-aligned-attacks task 3 as **today's baseline** on a
30° slope: the brawler builds its swings, guard and projectiles horizontally, and this golden records what that does on
the slope before the ground-aligned work changes it.

Every length, point and direction is a native literal: metres, og axes (+Y up, forward −Z, right +X), each the float
nearest the double-precision derivation given below. The static data and the box material are the flat scenario's
(`scenarioStaticData` and `staticBox` from `GoldenTraceScenario.h`: the game's defaults; friction 0.7, restitution 0.3).

## 1. The arena

- **The slope.** A box with half extents (6, 0.25, 12) (`kSlopeHalfExtents`), turned 30° about +X by
  `glm::quat(kHalfAngleCos, kHalfAngleSin, -0.f, 0.f)` = (cos 15°, sin 15°, −0, 0). Its top face normal is
  `kFaceNormal` = (0, cos 30°, sin 30°) = (0, 0.8660254, 0.5), the slope the iso camera looks straight at
  (design §1 of the initiative). Up the fall line is `kUpTheFallLine` = (0, 0.5, −0.8660254); the contour is +X.
  The centre `kSlopeCentre` = (0, 7.7834935, −0.125) is the face point `kFacePoint` = (0, 8, 0) moved 0.25 m (the half
  thickness) against the normal, so the top face contains (0, 8, 0). The face spans heights 2 to 14 (8 ± 12 sin 30°)
  and z from −10.392304 to 10.392304 (± 12 cos 30°); x from −6 to 6.
- **The landing** (`kLandingCentre`, `kLandingHalfExtents`): an unrotated box with its top at y = 2, from z = 9.392304
  to 22.392304, the same width. It reaches 1 m under the slope, so the **concave crease** at z = 10.392304
  (`kLowerCreaseZ`) has no gap: the slope's lower end face, which runs from the crease down to (y 1.567, z 10.142), lies
  inside the landing.
- **The plateau** (`kPlateauCentre`, `kPlateauHalfExtents`): an unrotated box with its top at y = 14, 8 m deep, from
  z = −18.392304 to −10.392304, the same width. The **convex crease** is at z = −10.392304 (`kUpperCreaseZ`); the
  slope's upper end face, down to y 13.567, lies inside the plateau (its bottom is at 13.5).
- The sides are open. The lowest walkable point (y = 2) is above the ring-out kill plane (−5 m).

## 2. The characters

Seven characters, ids 1 to 7 in body slots 0 to 6 (`kMaxSimulatableSlots` is 8). Each spawns `kSpawnHeight` (1.5 m)
straight above its face point; the ring-out spawn table holds the same seven points.

| character | face point | where |
|---|---|---|
| A (`kFaceA`) | (0, 8, 0) | the slope point |
| B (`kFaceB`) | (0, 9, −1.7320508) | 2 m up the fall line from A |
| C (`kFaceC`) | (0, 7, 1.7320508) | 2 m down the fall line |
| D (`kFaceD`) | (−2, 8, 0) | 2 m along the contour, −X |
| E (`kFaceE`) | (2, 8, 0) | 2 m along the contour, +X |
| F (`kLowerF`, then `kUpperF`) | (−1, 2.15, 10.132497), then (−1, 13.95, −10.305702) | along the face, 0.3 m from the concave crease, then 0.1 m from the convex one (§5) |
| G (`kLowerG`, then `kUpperG`) | (1, 2, 10.692305), then (1, 14, −10.692305) | 0.3 m from the crease on the flat side |

F and G stand 2 m apart along the contour, on either side of a crease: first the concave one, then the convex one.
Their aims (`kLowerFToG` and the others) are the horizontal directions to each other.

## 3. The script

The script (`kInputScript`) gives every character exactly one row on every tick (the rows are built by the helpers
`still`, `guard`, `walk`, `shoot` and `swing`). After 60 idle ticks (`kIdleTicks`) it runs ten **slots**. A slot starts
with the crease pair's teleport (§4), and its action comes `kSettleTicks` (35) ticks later (`actionTick`), by when every
character has been Supported and still for at least 20 ticks (measured: `unsettledPhaseStarts=0`,
`BrawlerJoltSlopeGoldenTraceTest-rationale.md` §2). In slots 1 to 8 the A group and the crease pair act on the same
tick, 10 m or more apart. The ticks are those of the recorded trace (`slotStart`, `actionTick`).

| slot | ticks | action | A group | crease pair | what the trace shows |
|---|---|---|---|---|---|
| — | 1–60 | — | all land and idle | F, G land at the concave crease | A's horizontal drift 0 |
| 1 | 61–240 | 96 | A right swing aimed up the fall line at B | F left swing at G | no hit |
| 2 | 241–350 | 276 | A right swing down the fall line at C | F right swing at G | no hit |
| 3 | 351–460 | 386 | A left swing at D (aim −X) | G right swing at F | D hit at 410, slides 5.17 m down the fall line; F hit at 410, hops up the slope (apex 2.63 m) |
| 4 | 461–640 | 496 | A left swing at E (aim +X) | G left swing at F | E hit at 520, hops up the fall line (apex 2.53 m); F hit at 520, thrown down onto the landing |
| 5 | 641–820 | 676 | B guards facing A; A overhead | F, G at the convex crease; F left swing at G | B's guard blocks the overhead at 694; G hit at 700, thrown along the plateau |
| 6 | 821–930 | 856 | C guards turned for a right swing; A right swing at C | F right swing at G | nothing on C; G hit at 880, thrown off the plateau edge onto the slope |
| 7 | 931–1040 | 966 | C guards turned for a left swing; A left swing at C | G right swing at F | nothing on C; F hit at 990, thrown up over the edge onto the plateau (a hop: apex 0.84 m, 0.72 s airborne) |
| 8 | 1041–1220 | 1076 | B fires a projectile at A | G left swing at F | the projectile neither hits nor is blocked; F hit at 1100, slides down the slope |
| 9 | 1221–1315 | 1256 | C guards facing A; A fires at C | — | the projectile neither hits nor is blocked |
| 10 | 1316–1480 | 1426 | A is teleported 1 m up the face from the concave crease (`kCreaseWalkStart`), walks down the fall line on ticks 1341–1400 and swings right across the crease | — | A stops with its capsule at z 10.3925, over the crease (10.3923) |

Directions in the trace's inbound hit direction are in the (forward, left) chart: (−1, 0) is +Z, down the fall line.
The knockback numbers are the test's `[GoldenTrace.slope]` line (`BrawlerJoltSlopeGoldenTraceTest-rationale.md` §3).

## 4. Teleports

Two characters serve both creases, and each crease hit throws its target about 5 m, so F and G are put back by a
scripted teleport (`goldenTraceScene::Teleport`, `GoldenTraceScene-rationale.md` §4) at the start of slots 2 to 8, to
`kTeleportHeight` (1.2 m) above their face points; slot 5 moves them to the convex crease. A is teleported once, at the
start of slot 10, next to the concave crease: walking there from (0, 8, 0) would take 12 m, about 720 ticks. Each
teleport is a short drop (0.08 to 0.14 m) to the hover height, settled well inside the 35 settle ticks.

## 5. Why these numbers

- **The tick budget.** The scenario must stay within 1500 ticks (a `static_assert`); it runs 1480. Slots 1, 4, 5 and 8 are
  `kLaunchSlotTicks` (180) long, the others `kSlideSlotTicks` (110), `kShotSlotTicks` (95) and `kCreaseWalkSlotTicks`
  (165). The four long slots were sized at task 3 for the crease hits whose swing tangent leaves the target's ground once
  the swing follows the attacker's slope (the initiative's design §3.6: such a hit launches, about 2 s airborne): the
  next teleport comes about 2 s (121 ticks) after the hit. Since task 3b the hits predicted to launch after the flip are
  those of slots 1, 4 and 8 (the crease windows below); slot 5 keeps its length. Today no long-slot hit launches; the
  convex hit of slot 7 hops 0.84 m and lands 0.72 s later, before slot 8's teleport.
- **Swing sides.** A knockback goes along the swing's tangent at the target, and the tangent depends on where the target
  is and on the swing's turning sense, not on the aim. Two targets on opposite sides of the attacker, struck by opposite
  swing sides, are thrown the same way: measured with a left swing at D and a right swing at E, both slid down the fall
  line with the same inbound direction (−1, 0). So A takes a left swing at both; D goes down the fall line and E up it.
  At the creases, with F aiming about +X, F's left swing throws G up the fall line (towards the slope or onto the
  plateau) and its right swing down; G aims about −X, so its right swing throws F up and its left swing down. Each pair
  runs up first, then down.
- **Which crease swings land (task 3b).** The melee detector measures from the attacker's weapon pivot to the target's
  root body: the radial body sits `attachmentOffset` (0.3 m up) above its capsule, and the query reports the position of
  the struck character's root, its capsule (`JoltSpatialQueryAdapter-guards.md` G-08). The hit needs that offset, along
  the swing axis, inside the ring's half thickness of 0.35 m (`getHalfThickness`), and the on-plane distance between the
  ring radii, in a damaging segment. While the axis is vertical (today) the target may therefore sit from 0.05 m below
  to 0.65 m above the attacker in capsule height, so a pair connects in **both** roles only when its two capsules rest
  within 0.05 m of the same height. At rest a capsule hovers 1.06 m above flat ground and 1.1234 m above a point of the
  30° face (the 0.0634 m more is r (1 / cos 30° − 1) for the ground probe's radius r = 0.41 m, `capsuleRadius` 0.42
  less `kProbeShrink`), except within about 0.2 m of a crease, where the probe touches the other face. Measured by
  `BrawlerJoltSlopeCreaseReach.PrintsTheConnectingWindows` (`BrawlerJoltSlopeGoldenTraceTest-rationale.md` §6): a
  flat-side capsule 0.05 and 0.10 m from the concave crease rests 1.0918 and 1.0657 m high, and a slope-side capsule
  0.05 to 0.20 m from the convex crease rests 1.0828 to 1.1216 m above its face point, its probe on the plateau's edge;
  its attack ground sample is then the plateau's up, so after the flip its swing stays horizontal.

  The sweep's windows (each side 0.05 to 1.00 m from the crease in 0.05 m steps, 400 pairs per row; both swing senses
  give the same windows; "after the flip" is task 5's ground-aligned initial conditions on each attacker's own ground
  sample):

  | crease | attacker | today | after the flip | predicted to launch after the flip |
  |---|---|---|---|---|
  | concave | F, slope side | none | all | F's up swing, G 0.15 m or more from the crease |
  | concave | G, flat side | all | all | G's down swing, G 0.15 m or more from the crease |
  | convex | F, slope side | all | F within 0.20 m, or G at 0.05 m | F's up swing, only F 0.25 m or more with G at 0.05 m |
  | convex | G, flat side | F within 0.20 m | F within 0.20 m | G's down swing, F within 0.20 m |

  A launch here is the design's rule (§3.6): the tangent the detector would write leaves the target's movement probe
  face by more than sin 2°.
- **The chosen placement (task 3b).** At the concave crease 2 of the 4 swings connect in both modes at every placement
  (the slope-side attacker never reaches today), and 0.3 m on each side also keeps two predicted launches, so the concave
  pair is unchanged. At the convex crease F moved from 0.3 m to 0.1 m from the edge (`kUpperF`, with the aims
  `kUpperFToG` and `kUpperGToF`); G stays at 0.3 m. Then all 4 convex swings connect in both modes, the most any
  placement gives, and G's down swing at F is predicted to launch after the flip (slot 8). 0.1 m is inside the measured
  0.05 to 0.20 m window with a sweep step on either side; the tightest band margin is G's swing at F, 0.309 m along the
  axis against 0.35 m. The cost: at 0.1 m F's attack ground sample is the plateau's, so the scenario has no tilted
  slope-side swing at the convex crease (that one connects only with G within about 0.05 m of the edge).

  With 0.3 m at both creases (task 3) four of the eight crease swings landed and none of the convex ones was predicted
  to connect after the flip. Now six land today (`creaseHits=6/8`, as the sweep predicts) and all eight are predicted to
  connect after the flip.
- **Stillness is judged horizontally.** A resting capsule's movement velocity flickers between 0 and 0.1633 m/s
  (g·dt) on some ticks, with a vertical position dither of about 3 mm (seen on F and G at the concave crease and on E
  after its hop); the horizontal position does not move. The settle check therefore requires Supported and at most 1 mm
  of horizontal travel per tick.
- **Sampling** is the flat scenario's: every tick through tick 300, every fifth tick after (`isSampleTick`).
