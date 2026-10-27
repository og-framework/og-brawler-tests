<!-- SPDX-License-Identifier: BUSL-1.1 -->
# GoldenTraceScenario — rationale

Source: `GoldenTraceScenario.h`. This file has no guards document: nothing in the header needs a `⛔`
tag. The harness that runs the scenario is described in `BrawlerJoltGoldenTraceTest-rationale.md`.

## 1. Native literals

Every length, point, direction and rotation in the scenario is a native literal: metres, og axes (+Y up,
forward −Z, right +X). Through the two flips (og-sim-metres-yup tasks 17 and 19) the scenario was written
in the legacy numbers (UE centimetres, Z up, x forward, y right) and passed through `simFrame`'s legacy
wrappers (deleted at task 24), so it meant the same world in every frame and unit the two knobs could select. Task 22 replaced each wrapped value by the
float the wrapper returned at (metres, Y up), measured bit for bit in the real build (a probe of every
static shape, spawn point, scripted input and static-data value before and after the rewrite: all equal).
So the golden recorded by task 19 still passes in exact mode.

- **Points and boxes** (`kSpawnA`, `kSpawnB`, `kRespawnB`, `kFloorCentre`/`kFloorHalfExtents`,
  `kRampCentre`/`kRampHalfExtents`, `kPlateauCentre`/`kPlateauHalfExtents`) are `glm::vec3` literals. The
  ramp's and the plateau's were derived, in centimetres, from the ramp angle and the step height: the run
  is `kStepHeight / tan(kRampAngleDeg)`, the half length is half the run over `cos(kRampAngleDeg)`, the
  centre is the top face's midpoint moved 0.1 m (the slab's half thickness) along the face normal, and the
  plateau spans the ramp's top (`kRampTop`) to `kStepEdge`. The literals are that derivation's floats, so
  they carry its rounding (for example `kRampTop` is `21.324244f`, not the exact 21.3242432…).
- **The ramp's rotation** is still computed: `glm::quat(cos(a/2), sin(a/2), -0, 0)` for the ramp angle `a`,
  a turn about the right axis (+X). It is the wrapped quaternion of the legacy rotation about −y written
  with `cos(a/2)` and `sin(a/2)` for `cos(−a/2)` and `−sin(−a/2)`; the probe shows the same matrix bits.
- **The aim and move directions** (`kTowardB`, `kTowardA`, `kAlongTheArena`, `kBackAlongTheArena`, `kStill`,
  `kGuardBlocksRightSwing`, `kGuardBlocksLeftSwing`) keep the `-0` components the wrapper's negation of a
  `+0` forward component produced.
- **The static data** (`scenarioStaticData`) takes walk speed `kMaxWalkSpeed` (1 m/s), step speed
  `kStepSpeed` (1 m/s) and gravity `kGravity` (−9.8 m/s²); the step period `kStepPeriodTicks` is a tick
  count. The stick (`moveStick`) is input space. The world's gravity is that static data's gravity along
  the frame's up.

The trace is still compared in the legacy (canonical) frame: the recorder converts every value back
(`GoldenTraceFormat-rationale.md`).

`scenarioStaticData` passes the five arguments of the `simulatableBrawler::StaticData` constructor: the
movement model (`kMovementModel`) and the four values above. Besides these, the harness overrides only
the ring-out spawn points of slots 0 and 1: the test's `Scene` constructor sets them to the scenario's
`kSpawnA` and `kRespawnB`. Everything else in the static data keeps the game's defaults, for example the
ring-out kill plane, `brawlerRingout::StaticData::killPlaneHeight` (−5 m), authored where it is defined
(`BrawlerRingoutSimulation.h`).

## 2. The arena

All of it sits about 20 m forward of the world origin (z ≈ −20), away from where the unused slot bodies
sit (the parked slots, and the projectile bodies of a character that has not fired). "Forward" below is
the distance along −Z.

- **Floor:** a box with its top at y = 0, forward from 19 m to 25 m (z from −25 to −19), x from −1.5 to 3.
  Its near edge at forward 19 m (`kFloorStart`) is **the ledge**: nothing is below it, so a character that
  leaves it falls past the ring-out kill plane (`brawlerRingout::StaticData::killPlaneHeight`, −5 m by
  default).
- **Ramp:** a slab turned 20° about the right axis (`kRampAngleDeg`), whose top face rises from the floor at
  forward 20.5 m (`kRampFoot`) to 0.3 m at forward ≈ 21.32 m (`kRampTop`). It is 1.2 m wide (x from −0.6
  to 0.6).
- **Plateau and step:** a 0.3 m high box (`kStepHeight`) from the top of the ramp to forward 21.8 m
  (`kStepEdge`), where it ends in a 0.3 m step down to the floor.

The statics are world-category boxes that block every brawler category, with friction 0.7 and
restitution 0.3, the values the og-simulation-tests Jolt rigs give an imported floor.

## 3. The script

Two characters: A (id 1, slot 0) spawns at (0, 1.5, −20) and B (id 2, slot 1) at (1.8, 1.5, −20).
B's ring-out respawn point (spawn slot 1) is (1.85, 1.5, −22.415), in front of where A's walk ends. The
script (`kInputScript`) gives every character exactly one row on every one of the 600 ticks. Each row
holds an aim, a move direction, a stick, the two attack buttons, a triggered action id and the
hold-guard flag. The schedule, with the ticks the recorded trace shows:

| ticks | what is scripted | what the trace shows |
|---|---|---|
| 1–25 | both fall from 1.5 m and land | hover at 1.06 m by tick 24 |
| 40 | A swings right at B; B holds guard aimed 0.6 rad right of A | B's guard blocks at tick 64; A enters GuardFlinch |
| 90 | A swings left; B's guard is aimed 0.6 rad to the other side | blocked at tick 114 |
| 140 | A swings right; B aims away | hit at tick 164: knockback backward (`hitDirectionXY` = (−1, 0) as the trace records it, in the (forward, right) chart; the live value in the native (forward, left) chart is its mirror, the same (−1, 0) up to the sign of the zero) |
| 164–238 | — | B slides off the ledge, falls, and dies at tick 238 (respawn tick 358) |
| 185–334 | A walks forward | up the ramp, along the plateau, down the step; stops at forward ≈ 22.415 m |
| 358 | — | B respawns at its spawn point and lands |
| 390 | A swings overhead; B aims away | hit at tick 408: stun |
| 462 | B fires a projectile at A; A holds guard facing B | blocked by A's guard at tick 466 |
| 518, 522 | A starts an overhead, B fires again | A is attacking, so its guard is off: the projectile hits A at tick 527 |

The guard aims are the two sides of `wouldGuardBlock`: the attacker 0.6 rad off the guard's forward,
inside the outer shield angle and outside the middle section, once on each side
(`kGuardBlocksRightSwing`, `kGuardBlocksLeftSwing`).

## 4. Why these numbers

- **The respawn spacing is 1.85 m, not 1.8.** `JoltSpatialQueryAdapter` sorts an overlap report by the
  peer-stable hit key (`JoltQueryHitKey`: simulatable id, then declaration index), so a character's body
  sphere (the radial's body, declaration 0) comes before its guard sphere (declaration 1). The projectile
  detector takes the first contact. So a projectile is blocked only
  on a tick where it overlaps the guard sphere (radius 0.4 m) and not yet the body sphere (radius 0.3 m).
  The projectile moves about 0.133 m a tick. At 1.8 m spacing its tick positions step over that 0.1 m band
  and the shot hits. At 1.85 m one tick lands inside it.
- **The knockback swing is a right swing.** It knocks B backward, toward the ledge 1 m away. A left
  swing knocks B forward across the floor, and B stops hanging on the far edge.
- **The walk ends at forward ≈ 22.415 m,** 0.6 m past the step: the character's ground probe (radius
  0.41 m) must clear the plateau's edge before it settles on the floor.

## 5. Coverage

`BrawlerJoltGoldenTrace.ScenarioCoversEveryScriptedMechanic` counts, from the trace in the legacy
frame, every mechanic above and requires each. The counts at record time: 121 walking ticks, 47 on the
ramp, 29 on the plateau, 4 sampled ticks of the step descent, 2 right, 1 left and 2 overhead swings,
2 projectiles, 1 guard block of each swing side, 1 projectile blocked and 1 that hit, 3 hit-flinches,
16 knockback ticks, 141 ticks off the ledge, 1 death and 1 respawn. The detectors' own counts agree:
2 melee hits, 2 guard blocks, 1 projectile hit and 1 projectile block.

The brawler has **no dash**: `brawlerMovementSimulation::StaticData` carries `dashSpeed`, `dashTicks`
and `dashCancelTick`, but nothing reads them and no input selects a dash. So the scenario cannot
exercise it and leaves it out.

Only the cadence movement model sets `committedStepDir`. Under the shipped model (continuous
accel/brake) the only write is the reset to zero on the spawn and respawn teleports, so that planar
field is recorded as zero throughout. The inbound hit direction is the planar field the trace
does exercise.
