# Sensor Manager Numeric Subtypes: Design Rationale

This document records the design of the constrained numeric types introduced
in the SPARK/Ada Sensor Manager, the sanitization rules applied at the
message-ingestion boundary, and every place where the Ada implementation
deliberately diverges from the C++ `SensorManagerService`. Each divergence is
covered by a back-to-back (b2b) test marked `xfail` in its `b2b.yaml`, with
the mismatch documented there as well.

## Principles

1. **Units are types.** Degrees and radians are distinct derived types
   (`Degrees_64`, `Radians_64`). Mixing them requires an explicit conversion
   function. This decision is motivated by an actual bug in the C++ service:
   `FindSensorFootPrint` compares the requested elevation angle, documented
   as degrees in `mdms/UXTASK.xml` (`ElevationAngles ... Units="deg"`),
   directly against gimbal limits already converted to radians
   (`SensorManagerService.cpp:252-257`), without conversion.

2. **Wire values are unconstrained; working values are constrained.** LMCP
   `real32` fields can carry any IEEE-754 value, including NaN and
   infinities, and any magnitude. All constrained subtypes therefore apply
   to *working* values produced by total sanitization functions in
   `Sensor_Manager_Types` at the point of first use (`Gimbal_Sweep_Range`,
   `Apply_Override`, `Continuous_Candidates`, `Effective_Altitude`, ...).
   Stored state and the shared record types in `lmcp_messages.ads` remain
   wire-faithful and unconstrained. A wire value that fails finiteness or
   range checks is clamped or rejected according to a documented per-field
   rule below — never silently propagated into the footprint math. This
   matters doubly because release builds compile with `-gnatp`: the
   subtypes are established by explicit code, not by runtime checks.

3. **Constraints come from CMASI/UXTASK documentation first, physics
   second.** Where the message definition documents a range (or the C++
   code documents an assumption), that is the constraint. Where it does
   not, the constraint is the widest physically meaningful range, chosen
   generously so that no plausible real-world configuration is rejected.

## Types

### Angles

| Type | Base | Range | Rationale |
|------|------|-------|-----------|
| `Degrees_64` | `Real64` | full | Unit-distinct carrier for degree values. |
| `Radians_64` | `Real64` | full | Unit-distinct carrier for radian values. |
| `Elevation_Deg` | `Degrees_64` | `-180.0 .. 180.0` | CMASI `GimbalConfiguration.Min/MaxElevation` defaults are ±180 and the C++ code states `ASSUME:: min/max angles are between -180/180` (`SensorManagerService.cpp:239`). Wire values outside are clamped to this range at ingestion (the subsequent working-range clamp makes this behaviorally identical to C++ for values like −200°, which C++ maps to −179° via its own radian clamp). |
| `Working_Elevation_Rad` | `Radians_64` | `-(Pi - Pi/180) .. -(Pi/180)` (−179°..−1°) | The elevation range in which footprint geometry is computed. Bounds are the C++ code's own clamping constants (`SensorManagerService.cpp:241,249-250`): at least 1° below horizontal (else slant range diverges: slant = alt/sin(−elev) grows as 1/sin) and at least 1° above straight-down-backward (−180°, where the geometry mirrors). Within this range `Sin(−E) ∈ [sin 1°, 1]` is bounded away from zero, so slant-range division is well-defined **by construction** — no tolerance guard needed. |
| `FOV_Deg` | `Degrees_64` | `0.0 .. 179.0` with `Dynamic_Predicate => FOV_Deg > 0.0` | A camera's horizontal field of view is physically a nonzero angle strictly inside a half-turn: a 0° FOV images nothing, and at 180° the perspective model degenerates (footprint width `2·slant·tan(FOV/2)` → ∞; C++ reports ~6.8×10¹⁵ m). The 1° top margin mirrors the elevation margins and bounds the width by `tan(89.5°) ≈ 114.6`. CMASI documents `Units="degree"` but no range. Candidate FOVs outside (0°, 179°] are skipped at enumeration (D5). |

### Lengths and distances

| Type | Base | Range | Rationale |
|------|------|-------|-----------|
| `AGL_Altitude_M` | `Real64` | `0.0 .. 100_000.0` | Above-ground-level altitude in meters (UXTASK `AglAltitudes Units="meters"`). 100 km (the Kármán line) generously bounds any air vehicle this service plans for. |
| `Assigned_Altitude_M` | `AGL_Altitude_M` | `10.0 .. 100_000.0` | Post-gate altitude: the C++ service refuses to compute footprints below `MIMIMUM_ASSIGNED_ALTITUDE_M = 10.0` (`SensorManagerService.cpp:49,228`). Values above 100 km fail the same gate in Ada (divergence D6). |
| `Slant_Range_M` | `Real64` | `0.0 .. 5.8E6` | slant = alt / sin(−elev) ≤ 100 000 / sin(1°) ≈ 5.73×10⁶ m. Bound rounded up. |
| `Center_Distance_M` | `Real64` | `-5.8E6 .. 5.8E6` | Horizontal distance to the footprint center, alt/tan(−elev); over the working elevation range \|tan(−elev)\| ≥ tan(1°) (the minimum is attained at the range ends), so \|result\| ≤ 100 000/tan(1°) ≈ 5.73×10⁶ m. Negative when the boresight points backward (beyond −90°). |
| `Edge_Distance_M` | `Real64` | `-1.01E15 .. 1.01E15` | Leading/trailing-edge distances use the FOV-widened edge angles, which are clamped to [−π, 0] and can therefore sit at a zero of tan; the guarded division (tolerance 10⁻¹⁰, retained from C++) bounds the result by 100 000/10⁻¹⁰. |
| `Width_M` | `Real64` | `0.0 .. 1.4E9` | Width = 2·slant·tan(FOV/2) ≤ 2·(5.8×10⁶)·tan(89.5°) ≈ 1.33×10⁹ m, thanks to the 179° FOV cap. |

### Sampling and imaging

| Type | Base | Range | Rationale |
|------|------|-------|-----------|
| `Pixel_Count` | `UInt32` | `0 .. 65_536` | CMASI `VideoStreamHorizontal/VerticalResolution Units="pixel"`, `uint32`. No real sensor exceeds 65 536 px per axis. 0 is retained as "unknown/absent" — the C++ code has defined behavior for it (aspect ratio 1.0, worst-case GSD angle π/2) which Ada preserves. Larger wire values are clamped to 65 536 (D7). |
| `Aspect_Ratio_T` | `Real64` | `1.0/65_536.0 .. 65_536.0` | Horiz/vert resolution ratio of two in-range nonzero `Pixel_Count`s; 1.0 when vertical resolution is 0 (C++ rule, `SensorManagerService.cpp:278`). |
| `Desired_GSD_M` | `Real64` | `0.001 .. 1.0E6` | Effective desired ground sample distance. Requests below 0.001 m/px (including 0 = "unspecified" and negatives) become the C++ default 1000.0 (`SensorManagerService.cpp:220-224`). 10⁶ m/px generously exceeds any meaningful GSD (max achievable is slant ≤ 5.8×10⁶ times sin(alpha), and desired GSDs beyond ~10³ already accept anything). |
| `Achieved_GSD_M` | `Real64` | `0.0 .. 5.8E6` | GSD = slant·sin(alpha) ≤ slant. |

### Loop counts

| Type | Base | Range | Rationale |
|------|------|-------|-----------|
| `Elevation_Step_Count` | `Positive` | `1 .. 37` | Elevation sweep span ≤ 178° at 5° steps (`Gimbal_Step_Size_Rad`): ⌊178/5⌋+1 = 36. The value 37 ('Last) is returned only by the defensive guard against floating-point rounding, and any overshoot is absorbed by `Sweep_Elevation`'s clamp to the sweep's upper end. |
| `FOV_Step_Count` | `Natural` | `0 .. 37` | At most ⌊179/5⌋+1 = 36 grid points of the 5° candidate grid (`Horizontal_FOV_Step_Size_Deg`) fit in (0°, 179°]; 0 means "no valid candidates". In C++ this loop is unbounded in the configuration values: a very negative `MinHorizontalFieldOfView` with a very positive maximum iterates (max−min)/5 times — effectively a hang for adversarial values. The Ada bound makes that impossible by construction. Grids are anchored at the wire minimum (for candidate-for-candidate compatibility with C++); anchors beyond ±3600° (`Max_FOV_Anchor_Magnitude_Deg`, ten full turns, far past any grid whose points could reach the valid range from a sane configuration) yield no candidates. |

## Sanitization rules

Applied by the total functions in `Sensor_Manager_Types` at the point of
first use; stored configurations stay wire-faithful. "Non-finite" means
NaN or ±infinity (`Is_Finite`).

| Field | Rule (function) |
|-------|------|
| `GimbalConfig.Min/MaxElevation` | Non-finite → the gimbal's sweep is invalid, it contributes nothing (C++ effectively skips such gimbals: every comparison on NaN is false, so its elevation loop never runs). Out-of-range finite values → clamped to ±180°, then into the working range. (`Gimbal_Sweep_Range`) |
| `CameraConfig.Min/MaxHorizontalFOV` | Non-finite → no continuous FOV candidates (C++: `NaN <= max` is false, loop never runs). Finite values → the 5° grid anchored at the minimum, intersected with (0°, 179°]; anchors beyond ±3600° → no candidates. (`Continuous_Candidates`) |
| `CameraConfig.DiscreteHFOVList` entries | Entries that are non-finite or outside (0°, 179°] are skipped. (`Is_Valid_FOV`) |
| `CameraConfig.Horiz/VertResolution` | Values > 65 536 are clamped to 65 536 (D7). 0 keeps its C++ meaning. (`Clamped_Resolution`) |
| Request `AglAltitudes` entries | NaN → the altitude gate fails (degenerate footprint), matching C++ where `NaN >= 10.0` is false. < 0.001 (including −∞) → nominal altitude (C++ rule). Outside [10 m, 100 km] after defaulting (including +∞) → gate fails (D6). (`Effective_Altitude`) |
| Request `GroundSampleDistances` entries | Non-finite or < 0.001 → default 1000.0 (D8). > 10⁶ → clamped to 10⁶. (`Effective_Desired_GSD`) |
| Request `ElevationAngles` entries | Non-finite → treated as "no override" (full-range sweep); this matches C++ for NaN and +∞ (`NaN < 0.001` is false) and diverges for −∞ (C++ pins to the gimbal minimum). Finite values: see D2/D3. (`Apply_Override`) |
| `EntityConfig.NominalAltitude` | Used only as the default altitude; passes through the same altitude gate. |

## Documented divergences from C++

Every divergence below is deliberate, is asserted by a boundary test, and is
`xfail`-marked in b2b mode with a reason referencing this section.

**D1 — GSD selection uses real-valued absolute difference.**
C++ computes `abs(desiredGsd_m - gsd_m)` where the unqualified `abs` resolves
to the C integer `abs(int)` (`SensorManagerService.cpp:308,310`), truncating
both deltas toward zero before comparison. Candidates whose deltas fall in
the same integer bucket are considered equal, so C++ keeps the *first*
candidate in enumeration order rather than the closest one. The previous Ada
implementation replicated this truncation; per explicit direction, that
replication has been removed: Ada compares `abs (Desired - GSD)` in
`Real64`. Tests whose camera configurations produce multiple candidates
within 1 m/px of each other now select different footprints in the two
implementations and are XFAILed.

Note on ties: the best GSD so far is tracked in a full-precision `Real64`
local rather than re-read from the `Real32` message field, so exactly-tied
candidates compare equal and the *first* is kept (as in C++). Comparing
against the rounded stored value would let a tied candidate win by a
rounding ulp and drift the selection to the last tie.

**D2 — Requested elevation angles are converted from degrees to radians.**
UXTASK.xml documents `ElevationAngles` as degrees. C++ pins the gimbal range
using the raw degree value against radian limits (`SensorManagerService.cpp:255`),
so a request for −45° is compared as −45 *radians*, which is below any
gimbal minimum; the sweep then runs at the gimbal minimum elevation instead
of the requested angle (e.g. −179° for an unclamped gimbal — the "nonsense
gimbal angles"). Ada converts the request to radians before pinning.

**D3 — The elevation override is clamped into the working range and the
gimbal's clamped range.** C++ pins `min := max(request, gimbal_min)` and
`max := min`, ignoring `gimbal_max` entirely and allowing pinned elevations
arbitrarily close to 0° (slant range → ∞ as 1/sin). Ada clamps the converted
override into `Working_Elevation_Rad` ∩ [gimbal min, gimbal max]. A request
of −0.5° therefore evaluates at −1°, and a request above the gimbal's max
elevation evaluates at the max, not beyond it.

**D4 — Gimbal max elevation in (−1°, 0°] is clamped to −1°.** C++ clamps
`MaxElevation` to −1° only when it is strictly positive
(`SensorManagerService.cpp:243`), so a max of −0.5° survives and the sweep
may evaluate footprints at −0.5°, where horizontal distances reach
~115 × altitude and grow unboundedly as the angle approaches 0. Ada clamps
any max above −1° down to −1° (this matches the intent evidenced by the C++
code's own ±1° margin constants). This divergence predates this change; it
is now documented and covered by a boundary test.

**D5 — FOV candidates are restricted to (0°, 179°].** C++ enumerates
whatever the configuration supplies: a 0° FOV yields GSD 0, a negative FOV
yields a negative GSD and footprint width, a 180° FOV yields a ~6.8×10¹⁵ m
footprint width, and an extreme continuous range loops essentially forever
(see `FOV_Step_Count`). Ada enumerates the same 5°-pitch grid, anchored at
the wire minimum for candidate-for-candidate compatibility, but skips grid
points outside (0°, 179°]; a camera with no valid candidate contributes
nothing.

**D6 — Assigned altitude above 100 km fails the validity gate.** C++ only
checks the lower bound (10 m). Ada treats the altitude gate as membership in
`Assigned_Altitude_M`.

**D7 — Video resolutions are clamped to 65 536 px.** Affects aspect ratio
and GSD only for physically impossible configurations. Relatedly, a camera
with zero *horizontal* resolution reports aspect ratio 0.0 in C++
(0/vertical) but 1.0 in Ada (`Aspect_Ratio_T` excludes 0); the computed
geometry is identical because C++ guards a zero aspect ratio by using the
horizontal FOV as the vertical FOV, which is exactly the aspect-1.0
geometry.

**D8 — Non-finite requested GSD becomes the 1000.0 default.** In C++ a NaN
desired GSD poisons the comparison chain: `abs(NaN - gsd)` cast to `int` is
undefined behavior, and in practice the first enumerated candidate wins.

**D9 — Entity configuration updates take effect.** On re-receipt of an
`EntityConfiguration` with a known ID, C++ uses `std::map::insert`, which
silently keeps the *first* configuration; the Ada service replaces the
stored configuration with the update. (Pre-existing Ada behavior, now
documented and pinned by the `entity_configuration_update` b2b test.)

**D10 — The response footprint sequence is capped at its index type's
range.** Footprints beyond `Positive'Last` (2³¹−1) per response are dropped;
C++ grows its vector unboundedly. Unlike D1-D9 this divergence is not
covered by a b2b test: reaching the cap would require a request whose
dimension product exceeds 2×10⁹ footprints (hundreds of gigabytes of
response), which neither implementation could serialize. The guard exists
to make `Handle_SensorFootprintRequests` total and provable.

## Behaviors deliberately kept bug-compatible with C++

These are C++ quirks that the Ada implementation *preserves*, because
existing b2b tests pin them and they are not in scope of the mandated fixes:

- **The 0.0 "unspecified" elevation sentinel produces a degenerate
  footprint.** UXTASK.xml says an empty `ElevationAngles` list should use
  "an optimal elevation angle for achieving max GSD"; the C++ implementation
  instead pins min = max = 0.0, never enters the GSD loop, and emits an
  all-zero footprint. Ada preserves this dead path (a request elevation in
  [0°, 0.001°) behaves the same way).
- **Positive requested elevations ≥ 0.001° are silently ignored** (treated
  as "no override": full-range sweep), rather than rejected.
- **A camera with zero vertical resolution** gets aspect ratio 1.0 and
  worst-case GSD angle π/2.
- **The comparison tolerance** `1.0e-10` and the guarded-division pattern in
  `Calculate_Sensor_Footprint` (returning 0.0 when a trigonometric
  denominator vanishes) are retained where angles can legitimately reach the
  guard (footprint edge angles clamped to exactly 0 or −π). For the
  boresight itself the guards were *dropped*: over `Working_Elevation_Rad`
  both `Sin(−E)` and `Tan(−E)` are bounded away from zero by construction,
  which is the point of the subtype.

## Proof architecture

All Sensor Manager subprograms are proved to SPARK Silver (absence of
runtime errors, plus the light functional contracts on the sweep and
candidate math) at `--level=2`, resting on two kinds of deliberately
unproved leaves:

- **Boundary functions** (`Is_Finite`, `Is_Below_Nominal_Threshold` in
  `Sensor_Manager_Types`): SPARK's floating-point model has no NaN or
  infinities, so the predicates that classify raw wire values are
  `SPARK_Mode (Off)` bodies. They answer Boolean questions about a
  possibly special value without letting it into SPARK code; every
  conversion of a wire `Real32` into working `Real64` math is guarded by
  them.

- **Trigonometric axioms** (`Sensor_Manager_Trig`): GNATprove has no
  theory of `Sin`/`Tan` (the runtime's elementary functions carry no
  postconditions), so the four facts the geometry needs are ghost
  procedures whose null bodies are `SPARK_Mode (Off)`; their
  postconditions are *assumed* at call sites, not proved. Each is
  justified in the spec by elementary real analysis over its
  precondition interval, with numeric margins (≥ 10⁻⁵) that generously
  absorb libm implementation error (a few ulps, ~10⁻¹⁵ at these
  magnitudes):

  | Axiom | Fact |
  |-------|------|
  | `Axiom_Sin_Bounds_On_Working_Range` | sin(x) ∈ [0.0174, 1] on [1°, 179°] |
  | `Axiom_Sin_Bounds_On_Half_Turn` | sin(x) ∈ [0, 1] on [0, π] |
  | `Axiom_Tan_Magnitude_On_Working_Range` | \|tan(x)\| ≥ 0.0174 on [1°, 179°] |
  | `Axiom_Tan_Bounds_Below_Vertical` | tan(x) ∈ [0, 115] on [0, 1.5621] |

  In debug builds (`-gnata`) the postconditions are compiled and
  evaluated, so the axioms are exercised at run time rather than
  blindly trusted.

The proof replays via `tests/proof/proofs/sensor_manager` (the
`Width_Center` multiplication needs a 120 s prover timeout when proved
from scratch with `--no-replay`).

## Test strategy

The divergences above make naive back-to-back comparison of the two
implementations impossible wherever they are exercised, so the b2b suite is
arranged as follows (see the per-test `b2b.yaml` files):

- Most functional tests pin the requested elevation **at the gimbal
  minimum** — the one point where C++'s mis-pinning (D2) and Ada's correct
  conversion agree — and pin the desired GSD **below every achievable
  candidate**, so that the C++ integer-bucket selection (D1) and the Ada
  argmin selection both keep the first candidate. These tests compare real
  footprint geometry between the implementations.
- A small set of exemplar tests deliberately exercises one divergence each
  and is marked `xfail` with a tight `xfail_match`:
  `gimbal_elevation_range` and `extreme_gimbal_angles` (D2),
  `default_gsd` and `gsd_optimization_multiple` (D1),
  `entity_configuration_update` (D9), `invalid_fov_mode` (strict Ada enum
  checking, pre-existing).
- The `boundary_*` tests probe each subtype bound just before, at, and
  just across the boundary; the across-boundary vehicles/requests are
  ordered last so the `xfail_match` can name the exact footprint index
  expected to diverge. Their assertions are implementation-aware (via
  `UXAS_IMPL`): they pin the C++ nonsense values as documentation and the
  Ada sanitized values as the specification.
- Request fields not pinned by a test are filled with *random* values by
  pylmcp (`randomize=True`), so every sensor-manager test now pins all
  four request arrays explicitly; several also needed `FieldOfViewMode`
  corrected from 1 (Discrete — leaving the actual candidate list random)
  to 0 (Continuous), matching their stated intent.
