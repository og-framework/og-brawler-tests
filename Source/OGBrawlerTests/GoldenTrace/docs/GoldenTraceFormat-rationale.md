<!-- SPDX-License-Identifier: BUSL-1.1 -->
# GoldenTraceFormat — rationale

Source: `GoldenTraceFormat.h`. This file has no guards document: nothing in the header needs a `⛔` tag.
The two generated data files (BrawlerJoltGolden.inl and BrawlerJoltGoldenEnvelope.inl) carry this
document as their docs pointer, because this header is what writes and reads them.

## 1. What the format is for

The golden trace checks that a change of the simulation's units or axes (`kUnitsPerMetre`, `kAxisFrame`)
leaves the brawler's behaviour unchanged.
One scenario (`GoldenTraceScenario.h`) runs the brawler on Jolt for 600 ticks. Its state is recorded
in one **canonical frame**, the legacy frame: UE centimetres, Z up, the (forward, right) chart. So a
trace recorded before a units or axis flip can be compared with one recorded after it.

Every recorded value is converted to the legacy frame in `double` through the inverses of `simFrame`
(`SimulationFrame-rationale.md` §8), chosen by the value's **type tag**:

| tag | meaning | converter | example |
|---|---|---|---|
| `p` | a vector with a length (position, linear velocity) | `toLegacyPosition` (maps the axes back, scales to cm) | body `position`, `linearVelocity`, movement `velocity` |
| `d` | a unitless direction | `toLegacyAxes` | (none recorded today) |
| `a` | an axial vector | `toLegacyAxial` | body `angularVelocity`, radial `initialAimRotationAxis` |
| `q` | a quaternion | `toLegacyRotation` | body `rotation` |
| `s` | a unitless scalar | none (`double` of the `float`) | radial `initialAimAngle` |
| `2` | a planar chart value | `toLegacyPlanar` | movement `committedStepDir`, the inbound hit's `hitDirectionXY` |
| `e` | an exact, discrete field | none, compared for equality | flags, machine state, hit counts and ticks, ring-out |

`toLegacyPlanar` is the identity under today's planar chart, P (forward, right). It is the one inverse
that changes if the chart becomes N (forward, left; `SimulationFrame-rationale.md` §3), so a planar
field recorded after that change is mirrored back into chart P and still compares with one recorded
before it.
No extents are recorded, so there is no extents tag.

## 2. The record

`goldenTrace::Recorder` builds a `goldenTrace::Trace`:

- the **schema** (`FieldSpec`: name, tag, component count, offset), fixed by the first sample;
- one **sample** per recorded tick: the tick and the flat list of `double` values.

Every later sample must record the same fields in the same order; `Recorder::endSample` checks it with
an `OG_CHECK`, so a scenario that records a field conditionally cannot produce a ragged trace.

## 3. The files

`writeGoldenInl` and `writeEnvelopeInl` emit C++ source that the test includes, CRLF like the rest of
the repository's sources:

- the golden: `kGoldenHeader` (format version, the two knobs `kAxisFrame` and `kUnitsPerMetre` at record
  time, the step dt, the scenario length, the source commit string from the recorder's environment and
  the generator name), `kGoldenFields` (the schema, with each tag's letter), `kGoldenTicks` and
  `kGoldenValues`;
- the envelope: `kEnvelopeHeader`, `kEnvelopeNudges` (character id, sim-frame component index, sign of
  each nudged run), `kEnvelopeTicks`, `kEnvelopeTagOrder` and `kEnvelopeValues` (one row per sample, one
  column per tag, in tag order).

Continuous values are written as C++17 hexadecimal floating literals (`std::to_chars` with
`std::chars_format::hex`). A hex literal is exact, so the compiled array holds the recorded bits, and a
`double` that came from a `float` needs at most six hex digits of mantissa. Discrete values are
written as decimal integers. `hexLiteral` refuses a value that is not finite with an `OG_CHECK` on the bit
test `isFiniteBits` (below), not on `std::isfinite`, which `/fp:fast` may fold away, so a golden cannot
record one.

## 4. The metrics

`fieldMetric` compares one field of two samples:

- vectors (`p d a 2`) and scalars (`s`): the largest absolute component difference, |Δ|∞;
- quaternions (`q`): the rotation angle between them, `2·acos|dot|`, computed as
  `4·asin(‖q₁ − σq₂‖ / 2)` with σ the sign of the dot product. The two are equal for unit quaternions
  (with φ = acos|dot|, ‖q₁ − σq₂‖ = 2·sin(φ/2)). The `asin` form keeps the resolution of a
  small angle, where `acos` of a dot product next to 1 loses it;
- discrete fields (`e`): the number of differing components.

## 5. The two modes

`compareExact` compares every value **bit for bit**, with one exception: in a float field (every tag but
`e`), `+0` and `−0` are equal. A float field matches when every component pair has the same bits or is a
pair of zeros, one of each sign or not (`sameBitsOrBothZero`). Any other difference still
fails: a 1-ULP change, a zero against a denormal, a NaN with a different payload. A `static_assert` beside
`sameBitsOrBothZero` pins these cases. Discrete fields (`e`) are compared with `std::memcmp` alone. It is the
mode for a change that must leave the trace bit-identical at the same knobs. The test refuses exact mode
when the golden's knobs differ from the build's.

Why zeros are the exception: the UE build compiles with `/fp:fast`, which lets the optimiser rewrite arithmetic
whose result is an exact zero, so the sign of that zero can depend on how the surrounding code was
optimised rather than on the source. A measured case: one aim axis's zero sign changed with an unrelated
header edit and no other value changed. Failing on it would make exact mode fail on edits whose only effect
is a zero's sign; ignoring it silently would hide where it happens. So exact mode counts it instead.

In exact mode the test prints one `[GoldenTrace.signedZero]` line: for each float tag (`p d a q s 2`) the
number of ±0-only differences (one per field and sample), then `first=` with the first one (tick, field
name, both values), or `first=none`. They are reported, never failed: `mismatchingFields` in the
`[GoldenTrace.compare]` line does not count them, and they add nothing to the largest metric per tag
(|Δ| = 0). The recorder's check that its two base runs agree requires no ±0-only difference either, so a
recorded golden comes from runs that agree on every bit.

`compareBounds` is the mode for a flip of `kUnitsPerMetre` or `kAxisFrame` (the test's `bounds` mode). It
holds every field, on every sample through tick 600, to an absolute bound for its **class**, in the legacy
frame, so a bound means the same thing at every knob combination:

| class | fields | bound (`kBounds`) |
|---|---|---|
| `p.position` | a `p` field whose last name segment is `position` | 1 cm (`kPositionBoundCm`) |
| `p.velocity` | a `p` field whose last name segment is `linearVelocity` or `velocity` | 33 cm/s (`kLinearVelocityBoundCmPerS`) |
| `a` | every `a` field: angular velocities and the aim rotation axis | 1e-4 (rad/s for an angular velocity) (`kAxialBound`) |
| `q` | every `q` field, by the angle metric of section 4 | 1e-5 rad (`kRotationBoundRadians`) |
| `s` | every `s` field | 1e-5 (`kScalarBound`) |
| `2` | every `2` field | 1e-5 (`kPlanarBound`) |
| `e` | every `e` field | 0: exact on every sample (`kDiscreteBound`) |

`boundClassOf` maps a tag and a field name to its class. The `p` tag holds both positions and velocities,
and a position bound on a velocity would be wrong, so `p` is split by the field name's last segment.
Any other `p` field, and every `d` field (none is recorded today, and no bound has been chosen for one),
has no class: `compareBounds` reports it as a schema problem, so the comparison fails rather than
guessing a bound. Two `static_assert`s pin this: one beside `boundClassOf` (each class, and the names
that must stay unclassified), and one in the test on the committed golden's field list
(`everyFieldHasABound`), so a recorded field with no class does not compile.

Why these bounds. A correct flip moves the trace by float-precision differences that the scenario
amplifies at a few discrete moments, and a units or axis defect moves it by orders of magnitude more:

- **What a correct flip costs**, measured against the centimetre, Z-up golden: positions at most
  0.273 cm, velocities at most 25.1 cm/s, `a` at most 5.7e-6, `q` at most 9e-7 rad, and every `s`, `2`
  and `e` value equal. The source of the large velocity differences is the hover servo, whose
  gravity feed-forward is present only when the ride-height error is zero or positive
  (`BrawlerMovementSimulation.h`). On the ramp that error stays within about 4e-4 cm of zero, so a
  rounding difference moves the tick on which a run's vertical velocity dips by g·dt (and rebounds
  after it). On a tick where one run dips or rebounds and the other does not, the velocities differ by
  up to about 1.5·g·dt (the measured 25.1 cm/s); the runs re-converge within a tick or two, so
  positions stay well under a centimetre.
- **The velocity bound** is therefore 2·g·dt (32.7 cm/s at the scenario's gravity and dt), rounded up
  to 33 cm/s, which leaves headroom over that one-tick difference. A `static_assert` in the test checks
  `kLinearVelocityBoundCmPerS` against 2·g·dt of `GoldenTraceScenario.h`.
- **The other bounds** sit an order of magnitude or more above the measured maxima, and far below a
  defect: a length left unconverted is wrong by a factor of 100 (centimetres against metres), and a
  vector left on the wrong axis is wrong by centimetres to metres. Such a defect also changes discrete
  outcomes (a hit, a guard block, a ring-out tick, a support flag), which `e` catches on every sample.

The envelope (section 6) is still read and printed, as a diagnostic only. For each continuous tag,
`compareBounds` stores the largest ratio of the metric to `max(4 · E(t, τ), 4 · ULP(|value|))`
(`envelopeRatio`; ULP is the `float` ULP of the golden value's largest component magnitude, of 1 for a
quaternion). It never fails on it. As a tolerance the envelope cannot pass a correct flip: its nudges
never reach the hover servo's branch in the centimetre build, they leave `a`, `q` and `s` at 0, and the
relative floor then gives a near-zero value a near-zero tolerance. Measured: a one-ULP decrease of
gravity at the legacy knobs (centimetres, Z up), with no flip at all, exceeds it.

`compareBounds` needs no exception for zeros: a ±0-only difference has a metric of 0 (|Δ| = 0, and a
quaternion's angle is 0), which no bound rejects.

A value that is not finite (±infinity or a NaN, golden or live) fails every class: `compareBounds` reports
it as "not finite" and records it in `classMaximum` as an infinite metric, and it tests the bits
(`isFiniteBits`: the exponent field all ones) rather than `std::isfinite`, `std::isnan` or `x != x`, because
`/fp:fast` lets the optimiser assume there is no NaN and fold those tests away (the metric cannot catch it
either: `std::max` drops a NaN component, and under `/fp:fast` a NaN metric can compare as within its bound).

Both return a `CompareResult`: the number of mismatches (one per field and sample), the first one
(tick, field name, both values), how many samples passed before it, and the largest metric per tag.
`compareExact` also fills the ±0-only count per tag (`signedZeroOnly`) and the first one
(`firstSignedZero`). `compareBounds` also fills, per class, the largest metric with its tick and field
(`classMaximum`, a `ClassMaximum`) and the envelope ratio per tag (`envelopeRatio`); a mismatch's text
names the metric, the bound, the class and E(t).

## 6. The envelope

`envelopeOf` takes the base run and 8 re-runs. Each re-run nudges one component of one character's
spawn position by one `float` ULP up or down (A and B, sim components 0 and 1, both signs; the list is
`kEnvelopeNudges`). For every sample and tag it stores the largest metric between the re-run and the
base. So E(t, τ) is how far a one-ULP change of the initial conditions has spread by tick t: a measure
of the scenario's sensitivity, read as a diagnostic (section 5). The `e` column counts the differing
discrete values, which shows where (if anywhere) the scenario's discrete outcome stops being robust to
a one-ULP perturbation.

Two limits of the nudges, which matter only when reading the diagnostic. The nudged components are
sim-frame component indices, so after an axis flip a re-recorded envelope nudges different physical
axes. In the Ue frame A's spawn coordinate on component 1 is 0; in the Y frame (the shipped frame since
the axis flip) its component 0 is 0 and component 1 is its spawn height. The two nudges of that zero
component move it to plus or minus the smallest `float` denormal, while the physics step runs with
denormals flushed to zero (`JoltStepFpScope`).

## 7. Versioning

`kFormatVersion` is checked against both headers. A change to the file layout bumps it and re-records.
A change to the recorded fields changes the schema; the schema check then fails with the first differing
field (or the two field counts, when the count changed), and the golden is re-recorded (see `BrawlerJoltGoldenTraceTest-rationale.md` §3).
