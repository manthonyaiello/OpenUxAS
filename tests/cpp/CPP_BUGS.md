# C++ UxAS Bug Catalog

This file documents bugs and problematic behaviors found in the C++ UxAS
implementation during the Ada/SPARK reimplementation effort.  Each entry
notes whether the bug has been fixed in C++, and whether the corresponding
Ada implementation diverges intentionally (xfail) or matches the C++ behavior.

---

## SensorManagerService

### SM-1: MinElevation = -180° escapes the elevation clamp

**File**: `src/cpp/Services/SensorManagerService.cpp`, line 241

**Status**: Not fixed in C++.  Ada diverges intentionally (xfail).

**Tests**: `tests/cpp/tests/sensor-manager/wide_fov_179/`

**Description**

When clamping the minimum gimbal elevation, C++ uses a strict less-than
comparison:

```cpp
gimbalElevationMin_rad =
    (gimbalElevationMin_rad < -dPi())          // strictly less than -Pi
        ? -(dPi() - dDegreesToRadians())        // clamp to -179 degrees
        : gimbalElevationMin_rad;               // keep as-is
```

The intent appears to be to exclude the singularity at -180 degrees (where
`sin(-elevation) = sin(Pi) ~= 0` causes a degenerate slant-range computation),
but the strict `<` means that an input of exactly -180 degrees is not clamped.
C++ therefore sweeps the gimbal elevation starting from -180 degrees when
`MinElevation = -180`.

The consequence is that the C++ gimbal sweep sequence and the Ada gimbal sweep
sequence are offset by one 5-degree step.  Both find a step with the minimum
integer-truncated |GSD - desired|, but they find different steps (e.g. C++
picks -150 degrees, Ada picks -149 degrees).

Ada defines `ELEV_MIN_BOUND = -(Pi - Pi/180) ~= -179 degrees` and always
clamps to this value, matching the intended (not actual) C++ behavior.

**Degenerate-step handling**

C++ processes the -180-degree step via a separate zero-denominator guard:

```cpp
double dDenominator = sin(-gimbalElevation_rad);
double dSlantRangeMin_m =
    bCompareDouble(dDenominator, 0.0, enEqual)
        ? altitudeAgl_m          // fallback: use altitude as slant
        : altitudeAgl_m / dDenominator;
```

This fallback is also present in Ada's `Process_Gimbal`.  However,
`Calculate_Sensor_Footprint` (the geometry sub-step) contains SPARK assertions
that require `sin(-Elev_Rad) >= 0.01`, which would be violated at -180 degrees.
Replicating the full C++ degenerate-step behavior in Ada would require
significant proof changes for marginal benefit, since the -180-degree step is
almost never the best GSD match.

---

### SM-2: HFOV >= 180 degrees is processed without validation

**File**: `src/cpp/Services/SensorManagerService.cpp`, lines 305-330

**Status**: Not fixed in C++.  Ada diverges intentionally (xfail).

**Tests**:
- `tests/cpp/tests/sensor-manager/wide_fov_180/` (HFOV = 180 degrees)
- `tests/cpp/tests/sensor-manager/wide_fov_190/` (HFOV = 190 degrees)

**Description**

C++ passes any horizontal FOV value, however large, directly to the GSD and
geometry calculations without checking whether the FOV is physically
meaningful for a downward-facing sensor:

- **HFOV = 180 degrees**: `tan(Pi/2)` is called.  Because `Pi` has no exact
  IEEE 754 representation, `Pi/2` is not the exact mathematical Pi/2, and
  `tan` returns a very large but finite value (~1.633e16).  WidthCenter becomes
  astronomically large but does not overflow float.

- **HFOV > 180 degrees**: `tan(HFOV/2)` enters the second quadrant and returns
  a negative value.  C++ silently stores a negative WidthCenter -- a
  geometrically meaningless result indicating the sensor is looking past the
  horizon.

Ada rejects FOV entries >= 180 degrees in `Process_Camera`:

```ada
if FOV <= 179.0 then
    Update_Best (...);
end if;
```

This guard is required for the SPARK proof of `Calculate_Sensor_Footprint`,
which needs `Tan(HFOV/2)` to be positive and finite (bounded by ~114.6 at
HFOV = 179 degrees).  179 degrees is also the largest physically sensible FOV
for a downward-facing sensor.

---

### SM-3: Integer abs() used for floating-point GSD comparison

**File**: `src/cpp/Services/SensorManagerService.cpp`, lines 308 and 310

**Status**: Not fixed in C++.  Ada deliberately matches the behavior.

**Tests**: All sensor-manager b2b tests that exercise GSD selection.

**Description**

The GSD comparison uses unqualified `abs()` on `double` values:

```cpp
double gsdDeltaDesired_m = abs(desiredGsd_m - gsd_m);
if (!firstGsdInitialized ||
    abs(desiredGsd_m - sensorFootprint->getAchievedGSD()) > gsdDeltaDesired_m)
```

`SensorManagerService.cpp` does not include `<cmath>`, so `abs()` resolves to
the C integer `abs` pulled in transitively through `<cstdlib>` or similar.
Passing a `double` to C integer `abs` truncates the value towards zero before
taking the absolute value -- the comparison is effectively on integer-truncated
deltas, not the raw floating-point deltas.

For example, with `desiredGsd_m = 5.0`:
- `gsd_m = 4.3`: `abs(0.7)` as integer abs = `abs((int)0.7)` = `abs(0)` = 0
- `gsd_m = 5.7`: `abs(-0.7)` as integer abs = `abs((int)(-0.7))` = `abs(0)` = 0

Both deltas compare as 0 even though 4.3 and 5.7 differ from 5.0 by 0.7.
This makes "first found wins" for any two GSD values within 1.0 m of each
other -- the gimbal sweep order determines the result.

Mathematically, `abs((int)x)` for a double `x` equals `floor(abs(x))`, since
truncation-towards-zero followed by abs is equivalent to floor of the magnitude.
Ada replicates this exactly with `Integer(Real64'Floor(abs(...)))` in `Update_Best`.
Without this replication the b2b tests would fail whenever two elevation steps
produce GSD values that differ by less than 1.0 m from the desired value.

---

### SM-4: ElevationAngles field (degrees) compared and assigned against radians variables

**File**: `src/cpp/Services/SensorManagerService.cpp`, lines 252–256

**Status**: Not fixed in C++.  Ada diverges intentionally.

**Tests**: `tests/cpp/tests/sensor-manager/gimbal_elevation_range/` (test comment at lines
79–83 explicitly documents the misbehaviour and only asserts weak properties).

**Description**

The LMCP field `SensorFootprintRequest::ElevationAngles` is defined with `Units="deg"`
in `mdms/UXTASK.xml`.  The local variable `elevationAngle` therefore holds a value in
degrees.  However lines 252–256 compare and assign it directly against/to
`gimbalElevationMin_rad`, which is in radians:

```cpp
if (elevationAngle < 0.001)                           // (A) threshold in radians?
{
    // elevation angle specified
    gimbalElevationMin_rad =
        (elevationAngle <= gimbalElevationMin_rad)    // (B) degrees vs. radians
            ? (gimbalElevationMin_rad)
            : (elevationAngle);                       // (C) stores degrees in _rad variable
    gimbalElevationMax_rad = gimbalElevationMin_rad;
}
```

The threshold 0.001 at (A) is nonsensical as a degree limit (0.001° ≈ barely nonzero)
but makes sense as a radian limit (~0.057°).  The net effect:

**Sub-bug A — full-sweep sentinel is broken.**  When `ElevationAngles` is empty, the code
pushes sentinel `0.0` (line 177).  `0.0 < 0.001` is TRUE, so the block is entered.
`max(gimbalElevationMin_rad_negative, 0.0) = 0.0`, so both min and max are set to 0.0.
The outer guard `if (gimbalElevationMin_rad < 0.0)` at line 258 then evaluates to FALSE and
the entire gimbal sweep is skipped, producing `AchievedGSD = 0.0`.  The documented
behaviour ("uses an optimal elevation angle for achieving max GSD") is therefore completely
broken for the most common use case of an empty `ElevationAngles` list.

**Sub-bug B — specified angle is silently ignored for most practical values.**  For any
meaningful negative elevation angle in degrees (e.g., `-45.0`), the numeric comparison
`-45.0 <= gimbalElevationMin_rad` (where the radian value is something like `-1.57`)
evaluates TRUE (since −45 < −1.57 numerically), so `gimbalElevationMin_rad` is left at
the gimbal's pre-computed minimum instead of the requested angle.  The loop then sweeps
only the single gimbal-minimum step.  The output `GimbalElevation` reports the gimbal
minimum (e.g., −90°), not the requested −45°.

For the narrow range of degree values numerically greater than `gimbalElevationMin_rad`
(roughly −0.001° to −1°, depending on gimbal config), sub-bug B goes the other way:
the degree value is stored directly into `gimbalElevationMin_rad` and treated as radians,
producing a wildly wrong elevation (e.g., −0.5 degrees → −0.5 radians ≈ −28.6°).

**How it is masked**

- Tests that exercise the full gimbal sweep use `randomize=True`, which generates
  `ElevationAngles` values in `[0, 1)`.  These positive values satisfy
  `elevationAngle >= 0.001` and skip the block entirely — correctly triggering the full
  sweep by accident.
- Tests that pass an explicit negative elevation (e.g., `-45.0`) only assert that
  `AchievedGSD > 0` and `GimbalElevation < 0`; they do not assert that `GimbalElevation`
  approximates the requested angle.  The `gimbal_elevation_range` test comment explicitly
  notes that the service "pins to the gimbal's minimum elevation" for this input.

---

### SM-5: EntityConfiguration not updated when a second message arrives for the same entity

**File**: `src/cpp/Services/SensorManagerService.cpp`, line 111

**Status**: Not fixed in C++.  Ada diverges intentionally (uses a replacement insert).

**Tests**: `tests/cpp/tests/sensor-manager/entity_configuration_update/` (test does not
assert the new configuration is actually used).

**Description**

The entity configuration map is updated with:

```cpp
m_idVsEntityConfiguration.insert(
    std::make_pair(entityConfiguration->getID(), entityConfiguration));
```

`std::unordered_map::insert` is a no-op when the key already exists; it silently keeps
the original entry and returns an iterator to it.  If an `EntityConfiguration` message
arrives for an entity that was already configured (e.g., because a vehicle's payload
changed in flight), the new configuration is discarded and all subsequent footprint
calculations continue to use the stale first-seen configuration.

The correct idiom is `operator[]` or `insert_or_assign` (C++17).

**How it is masked**

The `entity_configuration_update` test sends two configurations for the same entity
(vehicle ID 400) and checks that responses are produced for both requests, but never
asserts that the second response actually reflects the second configuration.  Specifically,
the test captures `first_altitude` and `second_altitude` (lines 79 and 145) but never
compares them or checks that the second response uses `NominalAltitude=1500` rather than
`NominalAltitude=500`.  The test passes even when the stale config is used throughout.

---

### SM-6: Zero-denominator fallback value is inconsistent between GSD and geometry calculations

**File**: `src/cpp/Services/SensorManagerService.cpp`, lines 265 and 351

**Status**: Not fixed in C++.  Ada matches the GSD fallback (altitude) and asserts that
the geometry sub-step is never called with a zero denominator.

**Tests**: Only reachable via the SM-1 −180° path; no dedicated test exists.

**Description**

When `sin(-gimbalElevation_rad) ≈ 0` (i.e., the gimbal is at ≈ −180° or 0°), two
different fallback values are used in two different places:

In `FindSensorFootPrint` (line 265), the slant range used for GSD calculation falls back to
`altitudeAgl_m`:

```cpp
double dSlantRangeMin_m =
    bCompareDouble(dDenominator, 0.0, enEqual)
        ? altitudeAgl_m          // fallback: slant = altitude
        : altitudeAgl_m / dDenominator;
```

In `CalculateSensorFootprint` (line 351), the slant range stored in the footprint response
falls back to `0.0`:

```cpp
double slantRangeToCenter_m =
    bCompareDouble(denominator, 0.0, enEqual)
        ? 0.0                    // fallback: slant = 0
        : altitudeAgl_m / denominator;
```

If the degenerate step is selected as the best-GSD candidate (possible with the SM-1
−180° escape), the response will carry `SlantRangeToCenter = 0.0` while `AchievedGSD` was
computed using `altitudeAgl_m` as the slant.  The two fields are therefore mutually
inconsistent.

**How it is masked**

Requires the SM-1 −180° escape in order to reach the degenerate step.  In normal
configurations the −180° step is never the best GSD match, so the inconsistency is never
visible in the output.

---

### SM-7: Divide-by-zero in CalculateSensorFootprint when HorizontalResolution = 0, VerticalResolution > 0

**File**: `src/cpp/Services/SensorManagerService.cpp`, lines 278–279 and 344

**Status**: Not fixed in C++.

**Tests**: Not tested; all tests use nonzero horizontal resolution or both resolutions zero.

**Description**

The aspect ratio is computed as:

```cpp
double dAspectRatio =
    (cameraConfiguration->getVideoStreamVerticalResolution() == 0) ? (1.0) :
    (static_cast<double>(cameraConfiguration->getVideoStreamHorizontalResolution()) /
     static_cast<double>(cameraConfiguration->getVideoStreamVerticalResolution()));
```

The guard only handles the case where **vertical** resolution is zero.  If horizontal
resolution is zero but vertical is nonzero (e.g., H=0, V=1080), the result is
`dAspectRatio = 0.0 / 1080 = 0.0`.

`CalculateSensorFootprint` then computes:

```cpp
double verticalFov_rad = horizantalFov_rad / dAspectRatio;  // division by zero
```

Under IEEE 754, dividing a nonzero finite float by 0.0 yields ±infinity.  The immediately
following clamps (`> 0.0` → `0.0`, `< -π` → `-π`) happen to recover finite values for
`gimbalAngleMax_rad` and `dGimbalAngleMin_rad`, but subsequent `tan()` calls on `−π` yield
near-zero values (tan(−π) ≈ −1.2e-16) that may or may not be caught by `bCompareDouble`,
producing a numerically garbage footprint geometry rather than a clean error.

**How it is masked**

No test constructs a camera with H=0, V>0.  In practice such a configuration is
nonsensical (a sensor with zero horizontal pixels but nonzero vertical pixels), so it is
unlikely to appear in real deployments.  The symptom would be a footprint with
geometrically meaningless field values rather than a crash.

---

## AutomationRequestValidatorService

### ARV-1: ImpactAutomationResponse.ResponseID not copied

**File**: `src/cpp/Services/AutomationRequestValidatorService.cpp`, `SendResponse`

**Status**: Fixed in C++ during Ada/SPARK work.

**Tests**: `tests/cpp/tests/arv/`

**Description**

In the sandbox-response branch of `SendResponse`, the `ResponseID` field of
the outgoing `ImpactAutomationResponse` was not copied from the corresponding
`ImpactAutomationRequest`.  The response was sent with `ResponseID = 0`
regardless of the request ID.

This was discovered when writing Ada back-to-back tests: the C++ oracle was
producing a 0 ResponseID while the Ada challenger (which copied it correctly)
was producing the expected non-zero value.  The C++ code was fixed to match.
