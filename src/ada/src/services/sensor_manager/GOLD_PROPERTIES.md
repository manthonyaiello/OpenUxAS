# Sensor Manager Gold Properties

This document proposes functional (SPARK Gold) properties for the Sensor
Manager, building on the completed Silver proof (absence of runtime errors
plus light contracts; see SUBTYPES.md for the type architecture, the
sanitization rules, and the documented divergences D1-D10 from the C++
`SensorManagerService`).

Each property is grounded in project documentation — `mdms/UXTASK.xml`,
the CMASI configuration structs (`GimbalConfiguration`,
`CameraConfiguration`), or the C++ reference behavior — and annotated with
provenance and expected proof effort. Properties are grouped into tiers by
increasing effort.

Status legend: `proposed` | `in progress` | `proved` | `abandoned (reason)`.

## Tier 1 — protocol and state properties

### P1. Config-store semantics of `Handle_EntityConfig` — proved

After `Handle_EntityConfig (State, Config)`:

- If `Config.ID` was already known, or the map held fewer than
  `Max_Entity_Configs` entries, then the map contains `Config.ID` and maps
  it to `Config`.
- Every other key's mapping is unchanged, and no key disappears.
- The length grows by at most one (and only when `Config.ID` was new).

Provenance: divergence D9 — C++ `std::map::insert` silently keeps the
*first* configuration for a re-announced entity ID; the Ada service
replaces it. This postcondition turns D9 from documented behavior into a
proved property.

Effort: low. SPARK formal hashed maps provide the model functions
(`Contains`, `Element`, `Length`, `Formal_Model`) and their operation
contracts are designed for exactly this style of postcondition.

### P2. Response correlation — proved

For `Handle_SensorFootprintRequests (State, Mailbox, Msg)`, the broadcast
response satisfies:

- `Response.ResponseID = Msg.RequestID`.
- Every footprint in `Response.Footprints` carries the
  `FootprintRequestID` and `VehicleID` of the request in `Msg.Footprints`
  it was computed for.

Provenance: the only functional requirement UXTASK.xml states verbatim —
`SensorFootprintResponse.ResponseID`: "ID matching initial request ID";
`SensorFootprint.FootprintResponseID`: "Each response must contain the
matching request ID in it's FootprintResponseID field".

Proved as the postcondition of the extracted `Build_Response` function
(the handler is now build-then-send, see P10): the response ID equals
`Msg.RequestID`, and for every footprint in the result there exists a
request index `J` in `Msg.Footprints` with matching `FootprintRequestID`
and `VehicleID`, whose `VehicleID` has a stored configuration. The
`VehicleID` correlation rests on the new ghost state invariant
`Configs_Keyed_By_ID` (every stored configuration is keyed by its own
ID), which holds of the default-initialized empty map and is preserved by
`Handle_EntityConfig` (added as a pre/postcondition pair there, on top of
the unweakened P1 contract).

### P3. Response shape and completeness — proved

The response contains exactly one footprint per combination of

    (request in Msg.Footprints with known VehicleID)
    x wavelength x GSD x altitude x elevation

where each empty request dimension is defaulted to a single "unspecified"
sentinel (so a dimension contributes `max (1, length)` combinations), in
request order; requests whose `VehicleID` is not in the config store
contribute nothing. Hence

    Length (Response.Footprints) =
      sum over known-vehicle requests of
        max(1,|W|) * max(1,|G|) * max(1,|A|) * max(1,|E|)

Note that even combinations for which no sensor qualifies produce a
(degenerate, all-zero) footprint — consumers rely on this for positional
correlation.

Provenance: C++ `FindSensorFootPrint` control flow; UXTASK.xml's empty-list
defaulting rules on all four request dimensions.

Proved as two additional postconditions of `Build_Response`, using the
ghost functions `Dim` (defaulted dimension count), `Combos` (their
product) and the recursive `Expected` (running sum over the leading
requests), all in big-integer arithmetic so the counting carries no
overflow obligations. The cap landed in the `Min` formulation: the
footprint count equals `Min (Expected (State, Msg, Last (Msg.Footprints)),
Positive'Last)`, which also covers responses truncated by the D10 cap.
The proof additionally strengthens P2's existential correlation to a
positional mapping: the footprints of request `J` occupy exactly the
response positions `Expected (J - 1) + 1 .. Expected (J)`, supported by a
recursive monotonicity lemma on `Expected` and mixed-radix counting
invariants in `Process_Request`, whose four nested loops are now indexed
rather than element iterations so each level's progress is nameable.

Side observation: UXTASK caps `SensorFootprintRequests.Footprints` at 16
and each request dimension at 8, so a maximal batch yields 16 * 8^4 =
65 536 footprints — one more than the response's documented
`MaxArrayLength="65535"`. If the wire caps are read as normative, the
response-side bound is unsatisfiable by exactly one footprint. This is a
spec-level finding in UXTASK.xml independent of either implementation.

## Tier 2 — per-footprint validity and realizability

### P4. Wire-level range guarantees on emitted footprints — proved

Every non-degenerate footprint in the response satisfies (as `Real32`
images of the constrained `Real64` working values):

| Field | Range | Source subtype |
|-------|-------|----------------|
| `AglAltitude` | [10.0, 100 000.0] m | `Assigned_Altitude_M` |
| `GimbalElevation` | [-179.0, -1.0] deg | `Working_Elevation_Rad` |
| `HorizontalFOV` | (0.0, 179.0] deg | `FOV_Deg` |
| `AspectRatio` | [1/65 536, 65 536] | `Aspect_Ratio_T` |
| `AchievedGSD` | [0, SlantRangeToCenter] | `Achieved_GSD_M` |
| `SlantRangeToCenter` | [AglAltitude, 5.8E6] m | `Slant_Range_M`; slant >= altitude since sin <= 1 |
| `WidthCenter` | >= 0 | `Width_M` |

This lifts the internal subtypes into the message-level contract — the
"no nonsense values on the bus" property, which the C++ service fails
(footprint widths of ~6.8E15 m, negative GSDs; see D5). State the bounds
as `Real32` conversions of the working-subtype bounds so floating-point
rounding does not force approximation reasoning.

Effort: medium. The information exists at each assignment; the work is
threading it through `Consider_Candidate` / `Find_Sensor_Footprint` /
`Process_Request` postconditions and loop invariants.

Proved as a `Build_Response` postcondition: every emitted footprint
satisfies the ghost predicate `Footprint_Wire_OK` — either
`Footprint_Geometry_Defaulted` (the degenerate all-default footprint,
also P8's forward vocabulary) or `Footprint_In_Wire_Ranges` (the table
above, including the two relational rows, plus bounds on the three
horizontal-distance fields). Supporting contracts:
`Slant_Range'Result >= Altitude` and `Compute_GSD'Result <= Slant`
(both from Sin <= 1 via the existing axioms), a degree-space image
bound on `To_Degrees` over the working elevation range (margins
`Working_Elevation_Deg_Lo/Hi` = -179.0001/-0.9999 absorb conversion
rounding), and a delta-aggregate frame on `Calculate_Sensor_Footprint`
("only the five geometry fields change"). The
`Consider_Candidate`/`Evaluate_Camera` preconditions pin
`Slant = Slant_Range (Altitude.Value, Elev)` so the achieved GSD can be
compared against the slant range recomputed inside
`Calculate_Sensor_Footprint`.

Two deviations from the table as first proposed:

- `HorizontalFOV` is proved in the closed interval [0.0, 179.0], not
  (0.0, 179.0]: strict positivity of the `Real32` image would require
  proving the `Real64` FOV candidates are bounded away from 2^-150
  degrees (true — grid points inherit `Real32` granularity from their
  wire anchors — but a sub-denormal rounding argument, not worth the
  effort). The internal `FOV_Deg` predicate still guarantees strict
  positivity in working precision.
- `GimbalElevation` bounds carry the 1.0E-4 degree rounding margins
  rather than exact -179.0/-1.0, since the working radian bounds do not
  convert to exact degree values.

Prover note: `Compute_GSD'Result <= Slant` (float multiply vs. one
factor) defeats the bit-blasting provers even at the campaign's
extended 120-second timeout; COLIBRI proves it instantly.

Configuration honesty: the campaign invocation is
`--level=2 --timeout=120 --prover=cvc5,z3,altergo,colibri` — the
level-2 *prover set* (plus COLIBRI) with a per-check time budget far
beyond stock level 2 (5 s) and even level 4 (60 s). These are not
"level 2" results; "proved" claims are relative to this configuration
(replay via the stored sessions is unaffected, since `--replay`
ignores user time limits). The Silver base does not need the extended
budget — it reproves from a fresh clone at stock level 2 except one
level-3 check. The extended configuration is accepted for the Gold
work for now, with an open item to measure which Gold checks actually
depend on it and pare the budget back down.

### P5. Realizability (provenance of the selected configuration) — proved

A non-degenerate footprint is actionable by the platform it names:

- `CameraID` names a camera in the vehicle's stored
  `EntityConfiguration`.
- That camera appears in the `ContainedPayloadList` of the gimbal named
  by `GimbalID`, which is a gimbal of that entity.
- `CameraWavelength` equals that camera's `SupportedWavelengthBand`, and
  satisfies the request's eligibility filter (equal to the requested
  band, or the request was `AllAny`).
- `HorizontalFOV` is one of that camera's valid FOV candidates: a member
  of its `DiscreteHorizontalFieldOfViewList` (discrete mode) or a point
  of the 5-degree grid within `[MinHorizontalFOV, MaxHorizontalFOV]`
  (continuous mode), in both cases inside (0, 179] degrees (D5).

Provenance: the CMASI payload-configuration model (cameras mounted on
gimbals via `ContainedPayloadList`); UXTASK's `EligibleWavelengths`
filter documentation.

Effort: medium-high. Wants a ghost "is a candidate" predicate shared with
P7.

Proved as a `Build_Response` postcondition mirroring P6's shape: for
every non-degenerate footprint in request J's positional segment,
`Footprint_Camera_Traceable` holds against the stored configuration of
J's vehicle and *some* entry of J's defaulted `EligibleWavelengths`
dimension — some camera of that configuration carries `FP.CameraID`,
that camera is mounted (`Camera_On_Gimbal`, via `ContainedPayloadList`)
on a gimbal carrying `FP.GimbalID`, `FP.CameraWavelength` equals the
camera's `SupportedWavelengthBand` and satisfies the eligibility filter
(equal to the requested band, or `AllAny`), and `FP.HorizontalFOV` is
the `Real32` image of one of the camera's valid FOV candidates
(`Camera_FOV_Image`: a `Continuous_Candidates` grid point in continuous
mode, a valid `DiscreteHFOVList` entry in discrete mode, both inside
(0, 179] per D5).

Two readings to note, neither a weakening of what was proposed:

- **Same-gimbal caveat — since closed by P7.** The gimbal witness in
  `Footprint_Camera_Traceable` is existential and independent of
  `Footprint_Elevation_Traceable`'s: if an entity announces two gimbals
  with the *same* `PayloadID`, the elevation could be witnessed by one
  and the mounting by the other. Under distinct gimbal payload IDs
  (the sane configuration) both predicates pin the same gimbal. P7's
  `Selection_Witness` now carries the joint enumeration — one
  `(gimbal, step, camera, FOV)` tuple explains mounting, elevation,
  wavelength, and FOV together — so the caveat no longer weakens the
  combined contract.
- The FOV clause is stated at the wire level (`Real32` image of a valid
  candidate), consistent with P4's image phrasing; the working-precision
  predicate `Valid_FOV_Of` carries the candidate-set semantics and is
  what P7 should quantify over.

Proof architecture: `Valid_FOV_Of` / `Camera_FOV_Image` /
`Camera_On_Gimbal` / `Footprint_Camera_Traceable` are default-hidden
expression functions with intro lemmas (`Lemma_Camera_On_Gimbal_Intro`,
`Lemma_Valid_FOV_Continuous_Intro`, `Lemma_Valid_FOV_Discrete_Intro`)
proved in `Sensor_Manager_Types` and called with ground index
witnesses; the `Cam_ID`/`Camera`/discrete-FOV loops are now indexed so
those witnesses are nameable, and `Evaluate_Camera` /
`Consider_Candidate` take the camera *index* (the camera record is
fetched locally) so no record equality crosses a call boundary. The
`First_GSD_Found`-keyed clause carries the third conjunct through the
same chain as P4/P6.

### P6. Commanded elevation is achievable by the named gimbal — proved

For every non-degenerate footprint, `GimbalElevation` (converted back to
radians) lies within the named gimbal's clamped elevation limits
intersected with the working range; and when the request supplies a
finite in-range elevation override, `GimbalElevation` equals that
override after the documented clamping (D2/D3 stated positively).

Provenance: CMASI `GimbalConfiguration.Min/MaxElevation`. Pointed
because the C++ service violates it: D3 (override pinning ignores
`gimbal_max`) and D2 (degrees compared against radians) can command
elevations the gimbal cannot reach. Proving this is a concrete assurance
delta over the reference implementation.

Effort: medium. Mostly strengthening the existing `Elevation_Sweep`
contracts (`Gimbal_Sweep_Range`, `Apply_Override`, `Sweep_Elevation`) to
relate the sweep to the gimbal's wire limits, then propagating.

Proved as a `Build_Response` postcondition, in provenance form: for
every non-degenerate footprint in request J's positional segment (P3),
`Footprint_Elevation_Traceable` holds against the stored configuration
of J's vehicle and *some* entry of J's defaulted `ElevationAngles`
dimension — some gimbal of that configuration carries `FP.GimbalID`,
its sweep under that override entry is valid, and `FP.GimbalElevation`
is the `Real32` degree image of one of that sweep's steps. The
headline properties then follow from the sweep contracts, all now
exact:

- `Gimbal_Sweep_Range` is characterized declaratively against the ghost
  repaired wire limits `Gimbal_Lo_Rad`/`Gimbal_Hi_Rad` (the C++-faithful
  repairs stated as functions of the CMASI fields), with validity
  exactly when the limits are finite and the repaired minimum points
  below horizontal.
- `Apply_Override` carries contract cases: non-finite and >= 0.001
  overrides are ignored, the [0, 0.001) sentinel band invalidates the
  sweep (the C++ dead path), and a negative override pins a valid sweep
  to `Pinned_Override` — the D2/D3 clamping chain stated positively.
- `Sweep_Elevation` lands in `[Sweep.Lo, Sweep.Hi]`, and
  `Sweep_Step_Count` of a pinned sweep is 1, so an active override is
  evaluated exactly at the pinned elevation.

Hence a traceable footprint's elevation lies within the named gimbal's
clamped limits intersected with the working range, and equals the
documented clamp of an active override.

Proof-architecture notes (the interesting part; see also the
"Managing the proof context" note below):

- The heavy ghost predicates (`Footprint_In_Wire_Ranges`,
  `Footprint_Elevation_Traceable`, `Elevation_Of_Gimbal`, `Sweep_Of`)
  are hidden by default via `pragma Annotate (GNATprove, Hide_Info,
  "Expression_Function_Body")` and disclosed (`Unhide_Info`) only where
  established: `Consider_Candidate` for the footprint predicates, and
  the null-bodied `Lemma_Elevation_Of_Gimbal_Intro` — proved inside
  `Sensor_Manager_Types`, called with ground index witnesses — for the
  elevation provenance.
- Opaque atoms cannot cross `FP = FP'Old`/`FP'Loop_Entry`: the record
  is mostly `Real32` fields, so that equality is component-wise IEEE
  `fp.eq`, which has no congruence rule. The `Find_Sensor_Footprint`
  contract chain is therefore keyed on `First_GSD_Found` ("a candidate
  has been accepted"): the flag is monotone, `FP` is untouched until it
  first rises, and while it is up the P4/P6 predicates hold of `FP`.
  Hidden atoms then transfer syntactically (unchanged `FP` is the same
  term; calls re-supply the atoms for the new term).
- `Evaluate_Gimbal` receives the gimbal *index* (ground witnesses) and
  the precomputed sweep, so the clamping machinery's contracts never
  enter its proof context; `Sweep_Of` exposes only `Valid implies
  Lo <= Hi`.

## Tier 3 — optimization and geometry

### P7. GSD argmin optimality — proved

The selected footprint's achieved GSD minimizes
`abs (Effective_Desired_GSD - gsd (c))` over the *entire* candidate set —
all valid sweep steps of all gimbals x eligible mounted cameras x valid
FOV candidates — with ties keeping the first candidate in enumeration
order.

Provenance: UXTASK `GroundSampleDistances` ("Desired ground sample
distance for an eligible sensor") — the service's core purpose. True of
the Ada code only because of D1 (real-valued comparison replacing C++'s
integer-truncated `abs`); the deliberate bug fix is what makes the spec
provable.

Proved, in two layers:

- **Selection layer** (`Find_Sensor_Footprint`'s postcondition, the
  full-strength result): when the effective altitude is valid, either
  the footprint is untouched and `No_Candidate` holds (the candidate
  set is empty), or `Footprint_GSD_Optimal` holds — there is a joint
  candidate witness `(gimbal K, sweep step S, camera CJ, FOV index FI)`
  such that the footprint's `GimbalID`, `GimbalElevation`, `CameraID`,
  `CameraWavelength`, `HorizontalFOV`, `AglAltitude`, and `AchievedGSD`
  all trace to that one tuple (`Selection_Witness`), and that tuple's
  exact `Real64` GSD dominates every candidate in the set
  (`Candidates_Covered`): for each candidate `c`,
  `abs (Desired - selected) <= abs (Desired - gsd (c))`. When the
  effective altitude is invalid, the footprint is untouched (P8
  support). Completeness is included: a degenerate footprint under a
  valid altitude *proves* the candidate set was empty, because the
  first candidate examined is always accepted.
- **Response layer** (`Build_Response`'s postcondition): every
  non-degenerate footprint in request J's positional segment satisfies
  `Footprint_GSD_Optimal` for some combination of J's four defaulted
  request dimensions (wavelength, GSD, altitude, elevation), with a
  valid effective altitude — mirroring the P5/P6 lift.

The joint witness also closes P5/P6's same-gimbal caveat: one candidate
tuple explains the mounting, the elevation, the wavelength, and the FOV
simultaneously, so duplicate gimbal payload IDs can no longer split the
witnesses.

Two documented deviations from the proposal:

- The tie-breaking clause ("ties keep the first candidate in
  enumeration order") is not formalized — that would need a
  lexicographic order over candidate tuples threaded through every
  prefix invariant. What is proved is that the selected GSD is a
  candidate GSD attaining the global minimum distance. (Tie-keeping
  remains true of the code — `Is_Better` is strict — just unproved.)
- The response-layer lift is existential over the request's
  combinations (as P5/P6 are), not positional per-combination; the
  per-combination statement would need P3's mixed-radix positional
  machinery replicated for the property clause.

Proof architecture (see also "Managing the proof context" below):

- **Joint candidate model** in `Sensor_Manager_Types`: `Is_Candidate`
  over `(K, S, CJ, FI)` with a unified 1-based FOV index space across
  continuous and discrete modes (`FOV_Candidate_Bound/Valid`,
  `Camera_FOV_At`), index-based mounting (`Mounted_On`), and
  `Candidate_GSD` recomputing exactly the `Compute_GSD` expression the
  implementation evaluates — so the accepted candidate's `Best_GSD`
  equals its `Candidate_GSD` definitionally.
- **One coverage family for optimality and completeness.** The
  hierarchical predicates (`Camera_Covered_Upto` through
  `Candidates_Covered`) state, for every candidate in a prefix: `Found`
  is up *and* `Best` dominates it. With `Found = False` the same
  predicate says the prefix has no candidates, so a single invariant
  family threads both halves of the property through all four loop
  levels; `No_Candidate` is the `(False, 0.0)` instance.
- **Preservation = extend + monotone.** `Best_GSD` only improves
  (`Is_Better` is strict), so prefix domination survives each call by
  distance-transitivity; per level, one Extend and one Monotone lemma
  (plus an empty-prefix intro — a hidden predicate needs an intro even
  for the trivial base). Monotone bodies above the camera level are
  ghost loops applying the inner level's lemma pointwise.
- **Witness thread.** Ghost `Sel_K/Sel_S/Sel_CJ/Sel_FI` locals record
  the accepted tuple; the flag-keyed invariant carries
  `Selection_Witness (FP, ..., Sel tuple, Best_GSD)` alongside the
  P4/P5/P6 chain. At the return, `Lemma_Footprint_GSD_Optimal_Intro`
  takes `Best` as a parameter and derives `Best = Candidate_GSD`
  internally, where `Selection_Witness` is disclosed — the equality is
  a `Real64` float equality inside a hidden atom and is deliberately
  never needed at call sites (the fp.eq congruence caveat).
- **Stepping stones before merges.** Asserts placed after the
  accept/reject branch of `Consider_Candidate` (merged paths, heavy
  floating-point context) time out even on trivialities; the same
  facts prove instantly asserted before the branch and inside each
  branch. This placement discipline, plus one `Slant` bridging assert
  per sweep step restating the slant range in the candidate tuple's
  own terms, was all the manual help the four-level threading needed —
  every stage's subprograms proved on their first full run at the
  campaign configuration (`Evaluate_Gimbal` 66/66,
  `Find_Sensor_Footprint` 18/18, `Process_Request` 193/193,
  `Build_Response` 156/156; unit totals 934 + 437).

### P8. Degenerate-footprint characterization — proposed

Forward direction (cheaper): if no candidate was accepted, the
footprint's geometry and selection fields hold their all-zero defaults —
consumers can rely on "all-zero geometry means no feasible sensor
configuration". Full characterization (harder): all-zero iff the
altitude gate failed, or no gimbal has a valid sweep, or the request hit
the 0.0-elevation sentinel dead path, or no eligible camera has a valid
FOV candidate.

Note: P7's contract on `Find_Sensor_Footprint` already carries most of
this at the selection layer — an invalid effective altitude leaves the
footprint untouched, and under a valid altitude the footprint is
untouched exactly when `No_Candidate` holds (the sweep-validity,
sentinel, mounting, eligibility, and FOV conditions are precisely
`Is_Candidate`'s conjuncts). What remains for P8 is lifting the
degenerate direction through `Process_Request`/`Build_Response` to the
response level.

Provenance: the C++ dead-path behavior documented in SUBTYPES.md
("Behaviors deliberately kept bug-compatible"); pinning it as *intended*
behavior rather than an accident.

Effort: forward direction low-medium once P4's plumbing exists; the iff
is high.

### P9. Geometric consistency of footprint fields — proposed

Two levels:

- Equational: each geometry field equals `Real32` of the documented
  formula — `SlantRangeToCenter = Alt / Sin (-E)`,
  `HorizontalToCenter = Alt / Tan (-E)`,
  `WidthCenter = 2 * Slant * Tan (FOV/2)`,
  `AchievedGSD = Slant * Sin (FOV_rad / Min_Res)` — stated via ghost
  functions, making "the implementation computes the UXTASK-documented
  quantities" a spec with the trig axioms as the only assumptions.
- Ordering: `HorizontalToTrailingEdge <= HorizontalToCenter <=
  HorizontalToLeadingEdge` whenever neither edge-division guard fires.
  Holds because `Alt * cot (-theta)` is monotone increasing in theta
  across (-Pi, 0), passing continuously through zero at -90 degrees.
  Needs one new trig axiom (cotangent monotonicity) in the style of the
  existing four in `Sensor_Manager_Trig`, plus a case split for the
  `Comparison_Tolerance` guards, which break the ordering by returning
  0.0.

Provenance: UXTASK field comments ("Distance out front of the entity to
the leading/trailing edge", "Width of the footprint at the vertical
center", etc.).

Effort: equational medium (mostly restating assignments as ghost-function
equalities); ordering high (new axiom, guard case analysis).

### P10. Single broadcast per request message — proved

`Handle_SensorFootprintRequests` broadcasts exactly one response per
handled `SensorFootprintRequests` message. Holds by construction after
the P2 refactor: the handler body is now the single statement
`sendBroadcastMessage (Mailbox, Build_Response (State, Msg))`.

## Proof architecture notes

- **Expose the response to contracts.** `sendBroadcastMessage` consumes
  the response into a `SPARK_Mode (Off)` pipe. The pure
  `Build_Response (State, Msg) return SensorFootprintResponse_Msg` has
  been extracted (done for P2/P10) so P3/P4/P5/P6 can be stated as
  further postconditions; the handler is build-then-send.
- **One ghost candidate model, three clients.** P5, P6, and P7 all
  quantify over "the candidates of an entity configuration under a
  request". Define a ghost predicate/enumeration once, reusing the
  existing step functions, and share it. (Done: P7's `Is_Candidate` /
  `Candidate_GSD` joint model, whose `Selection_Witness` also
  subsumes the P5/P6 per-field provenance for the selected candidate.)
- **State ranges as `Real32` images.** Message fields are `Real32`
  conversions of constrained `Real64` working values; phrasing
  postconditions as `Field = Real32 (X)` for a constrained ghost `X`, or
  as membership in `Real32`-converted bounds, keeps floating-point
  rounding out of the reasoning.
- **New axioms only in the established style.** Any additional trig fact
  (P9 ordering) should be a ghost procedure with a `SPARK_Mode (Off)`
  null body, justified in the spec by elementary real analysis with
  generous numeric margins, and exercised at runtime in debug builds —
  exactly like the four existing axioms.
- **Managing the proof context** (established during P6). Ghost
  predicates that quantify over configuration structure are hidden by
  default (`Annotate => (GNATprove, Hide_Info,
  "Expression_Function_Body")`) and disclosed per verifying entity with
  `Unhide_Info` only where they are established. Establishment goes
  through null-bodied intro lemmas proved next to the definition, called
  with ground (index) witnesses. Crucially, opaque atoms do not survive
  `Real32`-record equality (`fp.eq` has no congruence), so contract
  chains over `SensorFootprint_Msg` must be keyed on state flags
  (`First_GSD_Found`), never on `FP = FP'Old` disjunctions. P5/P7
  should extend `Elevation_Of_Gimbal`-style predicates and reuse these
  patterns.

## Suggested sequencing

1. P1 (quick win; pins D9).
2. P2 + P10 (requires the `Build_Response` extraction; P2 is the one
   property UXTASK states verbatim).
3. P4, P6, P5 (the assurance story versus the C++ bugs).
4. P3 (independent; its counting machinery is verbose — can be slotted
   anywhere; the wire-cap off-by-one finding stands regardless).
5. P7 (flagship Gold result — done), then P8 (its selection-layer core
   now falls out of P7's contract), P9 as stretch goals.
