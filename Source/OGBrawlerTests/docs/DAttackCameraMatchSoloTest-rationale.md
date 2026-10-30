<!-- SPDX-License-Identifier: BUSL-1.1 -->
# DAttackCameraMatchSoloTest — rationale

Source: `DAttackCameraMatchSoloTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-ground-aligned-attacks task 17 (2026-10-30): the shared iso camera can match the solo camera
(the Unreal host's `og.iso.matchSolo`, on by default). It then takes the solo camera's field of view and
uses margins equal to what the solo camera sees around its player, so one player is framed from the solo
boom and every outermost player of a group keeps that reach. The math is the engine-free
`dAttackCameraBehaviour::matchedIsoMargins`, `dAttackCameraBehaviour::sightReach`,
`dAttackCameraBehaviour::resolveIsoFraming` and `dAttackCameraBehaviour::cameraSight`. The Unreal actor
only calls them, and this target cannot compile it. The reasoning is in og-brawler's
`DAttackCamera-rationale.md` §16.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraMatchSolo]` selects them on their own.

## 2. The references are independent of the code under test

* `soloHalfTangents` is the solo camera's horizontal and vertical half-angle tangents in double precision,
  from the authored field of view and its axis: on the shorter axis of a landscape screen the vertical
  tangent is the authored one and the horizontal one is that times the aspect; otherwise the horizontal
  tangent is the authored one and the vertical one is that divided by the aspect. It does not call
  `horizontalFovDeg` or `verticalFovDeg`.
* `renderedHalfExtents` is what the iso camera shows at a distance, from the field of view the actor writes
  to its camera: the engine's projection with the cameras' MaintainXFOV constraint (the same model as
  `DAttackCameraFramingTest.cpp`'s, MaintainXFOV only): half-width `distance × tan(fov / 2)`, half-height
  that divided by the aspect.
* `kActorDefaults` is the Unreal actor's own framing defaults (`m_fovDegrees` 11.33 on the shorter axis,
  `m_marginHCm` 400, `m_marginVCm` 300, `m_minDistanceCm` 2000, `m_maxDistanceCm` 25000 in
  SharedIsometricCameraActor.h in the game module), kept equal by hand because this target cannot include
  that header. The matched cases use them, so the minimum distance being dropped and the maximum kept are
  exercised with the real values, not with the clamps set aside.

Tolerance: 1 cm on lengths (the task's), 0.001° on angles.

## 3. The cases

The four aspects are 4:3, 16:9, 21:9 and 9:16 (Android portrait); the booms 800 to 2000 cm in 200 cm
steps and 2500 cm (the default since og-ground-aligned-attacks task 18); the solo fields of view the
default 40° (since task 18) and task 17's default 58.72° on the shorter axis and 90° and 70° on the
horizontal one.

* `DAttackCamera.MatchedMarginsAreTheSoloReachAtTheLiveAspect`: the matched margins are the boom times the
  reference tangents, and `soloSightReach` returns the same bits as `matchedIsoMargins`.
* `DAttackCamera.MatchSoloFramesOnePlayerFromTheSoloBoom`: with matching on, an empty group (half-extents
  0) is framed from the boom within 1 cm; the resolved field of view and axis are the solo camera's, the
  minimum distance is 0 and the maximum the actor's. The all-defaults case frames one player from
  `kBoomLengthAtTargetPitch`.
* `DAttackCamera.MatchSoloKeepsTheSoloReachBeyondTheOutermostPlayers`: four groups (6 m along camera-right,
  6 m along camera-up, 6 m right and 2 m up, 30 m right and 1 m up). At the fitted distance, the rendered
  half-extents minus the group's half-extents (the reach beyond the outermost players on the camera
  plane) are at least the solo reach on both axes and equal to it within 1 cm on at least one. A group
  spread only along camera-right touches horizontally and keeps exactly `marginH` beyond it; one spread
  only along camera-up touches vertically and keeps exactly `marginV`.
* `DAttackCamera.MatchSoloOffIsTheIsoCamerasOwnFraming`: with matching off, `resolveIsoFraming` returns the
  iso settings bit for bit, the field of view the actor writes is the same bits, and the framing distance
  through the settings overload is bit-identical to the nine-argument `isoFramingDistance` that tasks 15
  and 16 call (7 aspects including 1, 3 and 0.4; 4 iso settings including the actor's and a 0.05° one for
  the tangent floor; 4 solo fields of view; 8 booms; 25 half-extent pairs: 22400 comparisons). That is the
  "`og.iso.matchSolo 0` reproduces task 15/16" criterion: the actor's call with matching off passes the same
  values to the same function.
* `DAttackCamera.CameraSightReportsTheSoloReachAndTheIsoDistance`: the readout's fields at the four aspects
  with the defaults: the boom, both solo fields of view and the reach against the references, the margins
  equal to the reach with matching on, the iso distance for a 6 m sideways group, and with matching off
  the actor's own settings and distance. On 9:16 the horizontal field of view is the authored default's
  bits (the shorter axis is the horizontal one).
* `DAttackCamera.MatchSoloFramesOnePlayerFromTheTunedDefaultBoom` (og-ground-aligned-attacks task 18): the
  defaults are boom 2500, field of view 40° on the shorter axis and 58° at the horizon; at the four aspects
  one player is framed from 2500 within 1 cm with the solo 40°, the resolved iso framing is the same bits
  whatever the horizon field of view (the iso camera uses the target pitch's values), and the readout's
  horizon field of view is `horizontalFovDeg(58°)` at the aspect.

## 4. Seen failing

(Task 17's record below predates task 18: the assertion totals were those of the five task-17 cases.)

Two poisons of `dAttackCameraBehaviour::sightReach` in `DAttackCamera.h`, each built and run under the UBT
lock and then restored from a copy taken before (md5 checked), 2026-10-30:

* **The vertical margin from the horizontal field of view** (`halfHeight = distance × tan(horizontalDeg /
  2)`): 4 of 5 cases, 771 of 2856 assertions failed. One player is framed from `aspect × boom` on every
  landscape screen (for example `16:9 fov 58.72 axis 1 boom 1200 distance 2133.33`), and the vertical
  margin and reach checks fail at all four aspects, 16:9 included. On 9:16 the horizontal term binds, so
  only the reach checks fail there.
* **The vertical field of view at a fixed 16:9 aspect** (`verticalFovDeg(horizontalDeg, 16.f / 9.f)`):
  4 of 5 cases, 433 of 2856 assertions failed, at 4:3 (112), 21:9 (208) and 9:16 (113) and none at 16:9,
  for example `21:9 fov 58.72 axis 1 boom 800 distance 1050` and `4:3 ... margins (600.055, 337.531)
  geometry (600.055, 450.041)`.

The matching-off case passed under both poisons, as it should: it never calls `sightReach`.
