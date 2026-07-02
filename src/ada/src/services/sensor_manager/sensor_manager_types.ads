with Ada.Numerics;
with Common;         use Common;
with LMCP_Messages;  use LMCP_Messages;

--  Constrained numeric types for the Sensor Manager, with the sanitization
--  functions that establish them from unconstrained LMCP wire values.
--
--  Design rationale, constraint provenance (CMASI/UXTASK documentation and
--  geometry), and every deliberate divergence from the C++
--  SensorManagerService are documented in SUBTYPES.md, in this directory.
--  Divergences are referred to below by their D-numbers from that file.

package Sensor_Manager_Types with SPARK_Mode is

   Pi : constant := Ada.Numerics.Pi;
   --  The mathematical constant pi

   Deg_To_Rad : constant := Pi / 180.0;
   --  Multiplier converting an angle in degrees to radians

   type Degrees_64 is new Real64;
   --  An angle in degrees. Degrees and radians are distinct types so that
   --  mixing them requires an explicit conversion, motivated by the C++
   --  units bug fixed by D2.

   type Radians_64 is new Real64;
   --  An angle in radians. Distinct from Degrees_64 (see above).

   ------------
   -- Angles --
   ------------

   subtype Elevation_Deg is Degrees_64 range -180.0 .. 180.0;
   --  A gimbal elevation angle. CMASI GimbalConfiguration documents
   --  Min/MaxElevation defaults of +/-180 deg and the C++ code assumes all
   --  angles lie within them.

   One_Degree_Rad  : constant := Deg_To_Rad;
   --  One degree expressed in radians

   Elev_Working_Hi : constant := -One_Degree_Rad;           --    -1 deg
   --  Upper end of the working elevation range: one degree below horizontal

   Elev_Working_Lo : constant := -(Pi - One_Degree_Rad);    --  -179 deg
   --  Lower end of the working elevation range: one degree short of
   --  straight back

   subtype Working_Elevation_Rad is
     Radians_64 range Elev_Working_Lo .. Elev_Working_Hi;
   --  Elevations at which footprint geometry is evaluated: at least one
   --  degree below horizontal and one degree short of straight back (the
   --  C++ code's own clamping margins). Within this range Sin (-E) is in
   --  [sin 1 deg, 1], bounded away from zero, so the slant-range division
   --  is well-defined by construction.

   Max_FOV_Deg : constant := 179.0;
   --  Largest permitted horizontal field of view, in degrees. A camera's
   --  horizontal FOV is physically a nonzero angle strictly inside a
   --  half-turn; a one-degree margin at the top mirrors the elevation
   --  margins, keeping footprint widths bounded (tan of the half-angle
   --  appears in the width formula). CMASI gives Units="degree" but no
   --  range. See D5.

   subtype FOV_Deg is Degrees_64 range 0.0 .. Max_FOV_Deg
     with Dynamic_Predicate => FOV_Deg > 0.0;
   --  A valid horizontal field of view. Zero is excluded by the predicate:
   --  a zero FOV images nothing. See D5.

   ---------------------------
   -- Lengths and distances --
   ---------------------------

   subtype AGL_Altitude_M is Real64 range 0.0 .. 100_000.0;
   --  Above-ground-level altitude in meters (UXTASK Units="meters").
   --  100 km (the Karman line) generously bounds any air vehicle.

   Minimum_Assigned_Altitude_M : constant := 10.0;
   --  Lowest altitude the service will plan for. The C++ service refuses to
   --  plan below 10 m; Ada additionally refuses above 100 km (D6).

   subtype Assigned_Altitude_M is
     AGL_Altitude_M range Minimum_Assigned_Altitude_M .. AGL_Altitude_M'Last;
   --  An AGL altitude that the service will actually plan for

   subtype Slant_Range_M is Real64 range 0.0 .. 5.8E6;
   --  Line-of-sight distance from sensor to footprint center, in meters:
   --  Altitude / Sin (-Elevation) <= 100_000 / sin (1 deg), about 5.73E6 m;
   --  bound rounded up.

   subtype Center_Distance_M is Real64 range -5.8E6 .. 5.8E6;
   --  Horizontal distance to footprint center: Altitude / Tan (-E) with
   --  abs Tan >= tan (1 deg) over the working elevation range (the minimum
   --  is attained at the range ends), hence abs result <= 5.73E6 m.

   subtype Edge_Distance_M is Real64 range -1.01E15 .. 1.01E15;
   --  Leading/trailing-edge horizontal distances. These use the
   --  FOV-widened gimbal angles, which are clamped to [-Pi, 0]; the guarded
   --  division (tolerance 1.0E-10, kept from C++) bounds them by
   --  100_000 / 1.0E-10.

   subtype Width_M is Real64 range 0.0 .. 1.4E9;
   --  Footprint width in meters: 2 * Slant * Tan (FOV/2), which is at most
   --  2 * 5.8E6 * tan (89.5 deg).

   ----------------------------
   -- Sampling and imaging   --
   ----------------------------

   Max_Pixel_Count : constant := 65_536;
   --  Largest per-axis pixel count kept as-is. CMASI video resolutions are
   --  uint32 pixel counts; no real sensor exceeds 65 536 px per axis (D7).
   --  Zero is kept, meaning "unknown/absent", with the C++ semantics
   --  (aspect ratio 1.0, worst-case GSD angle Pi/2).

   subtype Pixel_Count is UInt32 range 0 .. Max_Pixel_Count;
   --  A clamped per-axis pixel resolution

   subtype Aspect_Ratio_T is
     Real64 range 1.0 / Real64 (Max_Pixel_Count) .. Real64 (Max_Pixel_Count);
   --  Ratio of two nonzero in-range pixel counts, or exactly 1.0 when the
   --  vertical resolution is zero.

   Default_Acceptable_GSD : constant := 1000.0;
   --  Fallback desired ground sample distance. Requests below 0.001 m/px
   --  (including the 0.0 "unspecified" sentinel) become this C++ default;
   --  non-finite requests likewise (D8).

   subtype Desired_GSD_M is Real64 range 0.001 .. 1.0E6;
   --  An effective desired ground sample distance, in meters per pixel

   subtype Achieved_GSD_M is Real64 range 0.0 .. Slant_Range_M'Last;
   --  Ground sample distance actually achieved: Slant * Sin (alpha),
   --  which is at most Slant.

   -----------------
   -- Loop counts --
   -----------------

   Gimbal_Step_Size_Rad : constant := 5.0 * Deg_To_Rad;
   --  Elevation sweep step, in radians (5 degrees)

   subtype Elevation_Step_Count is Positive range 1 .. 37;
   --  Number of elevation sweep steps: span <= 178 deg at 5-degree steps.

   Horizontal_FOV_Step_Size_Deg : constant := 5.0;
   --  FOV sweep step, in degrees. Valid candidates lie in (0, 179] on a
   --  5-degree grid, so at most 36 of them; 0 means "no valid candidate".
   --  In C++ this loop is unbounded in the configuration values (a huge
   --  Min..Max range iterates essentially forever); the Ada bound makes
   --  that impossible.

   subtype FOV_Step_Count is Natural range 0 .. 37;
   --  Number of valid FOV candidates (0 when there are none)

   Max_FOV_Anchor_Magnitude_Deg : constant := 3_600.0;
   --  Sanity bound (10 full turns) on the continuous-mode FOV grid anchor.
   --  Continuous-mode grids are anchored at the wire MinHorizontalFOV to
   --  stay candidate-for-candidate compatible with C++; anchors beyond this
   --  bound yield no candidates (D5).

   ---------------------------------
   -- Wire-value validity queries --
   ---------------------------------

   function Is_Finite (X : Real32) return Boolean;
   --  Whether X is an ordinary number, the SPARK boundary check: everything
   --  downstream of it is free of special values.
   --  @param X A raw wire value that may be NaN or infinite
   --  @return True iff X is neither NaN nor infinite

   function Is_Below_Nominal_Threshold (X : Real32) return Boolean;
   --  Boundary function like Is_Finite: it answers a question about a
   --  possibly special wire value without letting that value into SPARK
   --  code.
   --  @param X A raw wire value that may be NaN or infinite
   --  @return True iff X is a number (not NaN) below the 0.001 "use the
   --    nominal altitude" threshold; in particular True for -Inf

   ----------------------
   -- Unit conversions --
   ----------------------

   function To_Radians (D : Degrees_64) return Radians_64 is
     (Radians_64 (Real64 (D) * Deg_To_Rad))
   with Pre => abs D <= 36_000.0;
   --  Convert an angle from degrees to radians.
   --  @param D Angle in degrees, bounded by 36_000
   --  @return D expressed in radians

   function To_Degrees (R : Radians_64) return Degrees_64 is
     (Degrees_64 (Real64 (R) / Deg_To_Rad))
   with Pre => abs R <= 700.0;
   --  Convert an angle from radians to degrees.
   --  @param R Angle in radians, bounded by 700
   --  @return R expressed in degrees

   --------------------------
   -- Elevation sweep math --
   --------------------------

   type Elevation_Sweep is record
      Valid : Boolean               := False;
      --  Whether the gimbal contributes a real footprint. When False the
      --  gimbal cannot point at the ground (or its limits are not finite)
      --  and only the degenerate all-zero response is produced.
      Lo    : Working_Elevation_Rad := Working_Elevation_Rad'Last;
      --  Lower (most negative) elevation of the interval to sweep
      Hi    : Working_Elevation_Rad := Working_Elevation_Rad'Last;
      --  Upper (least negative) elevation of the interval to sweep
   end record;
   --  The elevation interval a gimbal will be swept through, together with a
   --  validity flag.

   function Gimbal_Sweep_Range (Gimbal : GimbalConfig) return Elevation_Sweep
     with Post =>
       (if Gimbal_Sweep_Range'Result.Valid
        then Gimbal_Sweep_Range'Result.Lo <= Gimbal_Sweep_Range'Result.Hi);
   --  C++-faithful clamping of a gimbal's elevation limits (min below
   --  -180 deg raised, max above 0 dropped to -1 deg, crossed limits
   --  pinned, unclamped gimbals given the full working range), followed by
   --  the final clamp of both ends into the working range (D4 at both
   --  edges). Non-finite limits invalidate the sweep.
   --  @param Gimbal The gimbal configuration whose limits are clamped
   --  @return The elevation interval to sweep, or an invalid sweep

   function Apply_Override
     (Sweep    : Elevation_Sweep;
      Wire_Deg : Real32) return Elevation_Sweep
     with
       Pre  => (if Sweep.Valid then Sweep.Lo <= Sweep.Hi),
       Post =>
         (if Apply_Override'Result.Valid
          then Apply_Override'Result.Lo <= Apply_Override'Result.Hi);
   --  Apply a requested elevation override, per UXTASK degrees (D2), clamped
   --  into the working range and the gimbal's own range (D3). Values in
   --  [0, 0.001) preserve the C++ dead path (sweep invalidated, so the
   --  degenerate footprint is emitted); values >= 0.001 are ignored, as in
   --  C++; non-finite values are ignored (for -Inf this diverges from C++,
   --  which pins to the gimbal minimum).
   --  @param Sweep The sweep computed from the gimbal's own limits
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @return The sweep after applying the override

   function Sweep_Step_Count (Sweep : Elevation_Sweep)
     return Elevation_Step_Count
     with Pre => Sweep.Valid and then Sweep.Lo <= Sweep.Hi;
   --  Number of 5-degree steps the sweep is divided into.
   --  @param Sweep A valid, non-empty elevation sweep
   --  @return The number of elevation steps to evaluate

   function Sweep_Elevation
     (Sweep : Elevation_Sweep;
      Step  : Natural) return Working_Elevation_Rad
     with Pre => Sweep.Valid
                   and then Sweep.Lo <= Sweep.Hi
                   and then Step < Sweep_Step_Count (Sweep);
   --  The elevation of a given sweep step, clamped to the sweep's upper end
   --  to absorb floating-point rounding in the last step.
   --  @param Sweep A valid, non-empty elevation sweep
   --  @param Step The step index, in 0 .. Sweep_Step_Count (Sweep) - 1
   --  @return The elevation at that step

   ------------------------
   -- FOV candidate math --
   ------------------------

   type FOV_Candidates is record
      Count       : FOV_Step_Count := 0;
      --  Number of valid grid points (0 when the range is empty, its ends
      --  are not finite, or the anchor is beyond the sanity bound)
      First_Valid : FOV_Deg        := Max_FOV_Deg;
      --  First grid point inside the valid range
   end record;
   --  Valid FOV candidates of a continuous-mode camera: the 5-degree grid
   --  anchored at the wire minimum, intersected with (0, 179] (D5).

   function Continuous_Candidates
     (Wire_Min, Wire_Max : Real32) return FOV_Candidates;
   --  Compute the valid FOV candidates of a continuous-mode camera.
   --  @param Wire_Min The wire MinHorizontalFOV (grid anchor), in degrees
   --  @param Wire_Max The wire MaxHorizontalFOV (grid end), in degrees
   --  @return The valid FOV candidate grid

   function Candidate_FOV
     (Candidates : FOV_Candidates;
      Index      : Natural) return FOV_Deg
     with Pre => Index < Candidates.Count;
   --  The FOV of a given candidate on the grid.
   --  @param Candidates A computed candidate grid
   --  @param Index The candidate index, in 0 .. Candidates.Count - 1
   --  @return The field of view at that candidate

   function Is_Valid_FOV (Wire : Real32) return Boolean is
     (Is_Finite (Wire)
        and then Degrees_64 (Wire) > 0.0
        and then Degrees_64 (Wire) <= Max_FOV_Deg);
   --  Validity filter for discrete-mode FOV list entries (D5).
   --  @param Wire A wire FOV value, in degrees
   --  @return True iff Wire is finite and in (0, Max_FOV_Deg]

   ------------------------------
   -- Camera-derived quantities --
   ------------------------------

   function Clamped_Resolution (Wire : UInt32) return Pixel_Count is
     (if Wire > Max_Pixel_Count then Max_Pixel_Count else Wire);
   --  Clamp a wire pixel resolution to Max_Pixel_Count (D7).
   --  @param Wire A wire per-axis pixel count
   --  @return Wire, clamped to at most Max_Pixel_Count

   function Min_Resolution (Camera : CameraConfig) return Pixel_Count is
     (Pixel_Count'Min
        (Clamped_Resolution (Camera.HorizResolution),
         Clamped_Resolution (Camera.VertResolution)));
   --  The smaller of a camera's two clamped pixel resolutions.
   --  @param Camera A camera configuration
   --  @return The minimum of the clamped horizontal and vertical resolutions

   function Aspect_Ratio_Of (Camera : CameraConfig) return Aspect_Ratio_T is
     (if Clamped_Resolution (Camera.VertResolution) = 0
        or else Clamped_Resolution (Camera.HorizResolution) = 0
      then 1.0
      else Real64 (Clamped_Resolution (Camera.HorizResolution))
           / Real64 (Clamped_Resolution (Camera.VertResolution)));
   --  A camera's width/height ratio; exactly 1.0 when either clamped
   --  resolution is zero (C++ rule).
   --  @param Camera A camera configuration
   --  @return The horizontal-to-vertical resolution ratio

   -------------------------
   -- Request sanitization --
   -------------------------

   type Altitude_Result is record
      Valid : Boolean             := False;
      --  Whether the effective altitude is usable (in [10 m, 100 km] and
      --  finite)
      Value : Assigned_Altitude_M := Assigned_Altitude_M'First;
      --  The effective assigned altitude when Valid is True
   end record;
   --  Effective assigned altitude. Valid is False when the effective
   --  altitude (wire value, or nominal when the wire value is below the
   --  0.001 "unspecified" threshold) falls outside [10 m, 100 km] or is not
   --  finite; the C++ service checks only the lower bound (D6).

   function Effective_Altitude
     (Wire_M    : Real32;
      Nominal_M : Real32) return Altitude_Result;
   --  Resolve the effective assigned altitude from the request and the
   --  entity's nominal altitude.
   --  @param Wire_M The requested altitude, in meters
   --  @param Nominal_M The entity's nominal altitude, used when Wire_M is
   --    below the 0.001 "unspecified" threshold
   --  @return The effective altitude and whether it is usable

   function Effective_Desired_GSD (Wire_M : Real32) return Desired_GSD_M;
   --  Resolve the effective desired GSD: below-threshold and non-finite
   --  values take the 1000.0 default (D8); absurdly large values are
   --  clamped.
   --  @param Wire_M The requested ground sample distance, in meters/pixel
   --  @return The effective desired GSD

   ------------------------
   -- Footprint geometry --
   ------------------------

   function Slant_Range
     (Altitude : Assigned_Altitude_M;
      Elev     : Working_Elevation_Rad) return Slant_Range_M;
   --  Line-of-sight distance to the footprint center: Altitude / Sin (-Elev).
   --  Total because Sin (-Elev) >= sin (1 deg) over the working elevation
   --  range.
   --  @param Altitude The assigned AGL altitude, in meters
   --  @param Elev The boresight elevation, in radians
   --  @return The slant range, in meters

   function Compute_GSD
     (Slant   : Slant_Range_M;
      FOV     : FOV_Deg;
      Min_Res : Pixel_Count) return Achieved_GSD_M;
   --  GSD achieved by one camera pixel at the given slant range:
   --  Slant * Sin (alpha) with alpha = FOV_rad / resolution, or the
   --  worst-case Pi/2 when the resolution is unknown (C++ rule).
   --  @param Slant The slant range to the footprint center, in meters
   --  @param FOV The camera's horizontal field of view, in degrees
   --  @param Min_Res The smaller per-axis resolution, or 0 if unknown
   --  @return The achieved ground sample distance, in meters/pixel

   function Is_Better
     (Desired     : Desired_GSD_M;
      Candidate   : Achieved_GSD_M;
      Current     : Achieved_GSD_M;
      First_Found : Boolean) return Boolean
   is
     (not First_Found
        or else abs (Desired - Current) > abs (Desired - Candidate));
   --  Whether a candidate GSD is a better match for the desired GSD than the
   --  current best. D1: this is a real-valued comparison of distances from
   --  the desired value. The C++ code truncates both distances to integers
   --  first (its abs resolves to the C integer abs), making candidates
   --  within the same integer bucket "equal" so the first one found wins;
   --  that behavior was deliberately not reproduced.
   --  @param Desired The desired ground sample distance
   --  @param Candidate The candidate's achieved GSD
   --  @param Current The current best achieved GSD
   --  @param First_Found Whether any candidate has been accepted yet
   --  @return True iff Candidate should replace Current

end Sensor_Manager_Types;
