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

   Deg_To_Rad : constant := Pi / 180.0;

   --  Degrees and radians are distinct types: mixing them requires an
   --  explicit conversion. Motivated by the C++ units bug fixed by D2.
   type Degrees_64 is new Real64;
   type Radians_64 is new Real64;

   ------------
   -- Angles --
   ------------

   --  CMASI GimbalConfiguration documents Min/MaxElevation defaults of
   --  +/-180 deg and the C++ code assumes all angles lie within them.
   subtype Elevation_Deg is Degrees_64 range -180.0 .. 180.0;

   --  Elevations at which footprint geometry is evaluated: at least one
   --  degree below horizontal and one degree short of straight back
   --  (the C++ code's own clamping margins). Within this range
   --  Sin (-E) is in [sin 1 deg, 1], bounded away from zero, so the
   --  slant-range division is well-defined by construction.
   One_Degree_Rad  : constant := Deg_To_Rad;
   Elev_Working_Hi : constant := -One_Degree_Rad;           --    -1 deg
   Elev_Working_Lo : constant := -(Pi - One_Degree_Rad);    --  -179 deg

   subtype Working_Elevation_Rad is
     Radians_64 range Elev_Working_Lo .. Elev_Working_Hi;

   --  A camera's horizontal field of view is physically a nonzero angle
   --  strictly inside a half-turn; we keep a one-degree margin at the top,
   --  mirroring the elevation margins, so that footprint widths stay
   --  bounded (tan of the half-angle appears in the width formula).
   --  CMASI gives Units="degree" but no range. Zero is excluded by
   --  predicate: a zero FOV images nothing. See D5.
   Max_FOV_Deg : constant := 179.0;

   subtype FOV_Deg is Degrees_64 range 0.0 .. Max_FOV_Deg
     with Dynamic_Predicate => FOV_Deg > 0.0;

   ---------------------------
   -- Lengths and distances --
   ---------------------------

   --  Above-ground-level altitude in meters (UXTASK Units="meters").
   --  100 km (the Karman line) generously bounds any air vehicle.
   subtype AGL_Altitude_M is Real64 range 0.0 .. 100_000.0;

   --  The C++ service refuses to plan below 10 m; Ada additionally
   --  refuses above 100 km (D6).
   Minimum_Assigned_Altitude_M : constant := 10.0;

   subtype Assigned_Altitude_M is
     AGL_Altitude_M range Minimum_Assigned_Altitude_M .. AGL_Altitude_M'Last;

   --  Slant = Altitude / Sin (-Elevation) <= 100_000 / sin (1 deg),
   --  about 5.73E6 m; bound rounded up.
   subtype Slant_Range_M is Real64 range 0.0 .. 5.8E6;

   --  Horizontal distance to footprint center: Altitude / Tan (-E) with
   --  |Tan| >= tan (1 deg) over the working elevation range (the minimum
   --  is attained at the range ends), hence |result| <= 5.73E6 m.
   subtype Center_Distance_M is Real64 range -5.8E6 .. 5.8E6;

   --  Leading/trailing-edge distances use the FOV-widened gimbal angles,
   --  which are clamped to [-Pi, 0]; the guarded division (tolerance
   --  1.0E-10, kept from C++) bounds them by 100_000 / 1.0E-10.
   subtype Edge_Distance_M is Real64 range -1.01E15 .. 1.01E15;

   --  Width = 2 * Slant * Tan (FOV/2) <= 2 * 5.8E6 * tan (89.5 deg).
   subtype Width_M is Real64 range 0.0 .. 1.4E9;

   ----------------------------
   -- Sampling and imaging   --
   ----------------------------

   --  CMASI video resolutions are uint32 pixel counts; no real sensor
   --  exceeds 65 536 px per axis (D7). Zero is kept, meaning
   --  "unknown/absent", with the C++ semantics (aspect ratio 1.0,
   --  worst-case GSD angle Pi/2).
   Max_Pixel_Count : constant := 65_536;

   subtype Pixel_Count is UInt32 range 0 .. Max_Pixel_Count;

   --  Ratio of two nonzero in-range pixel counts, or exactly 1.0 when the
   --  vertical resolution is zero.
   subtype Aspect_Ratio_T is
     Real64 range 1.0 / Real64 (Max_Pixel_Count) .. Real64 (Max_Pixel_Count);

   --  Effective desired ground sample distance: requests below 0.001 m/px
   --  (including the 0.0 "unspecified" sentinel) become the C++ default
   --  of 1000.0; non-finite requests likewise (D8).
   Default_Acceptable_GSD : constant := 1000.0;

   subtype Desired_GSD_M is Real64 range 0.001 .. 1.0E6;

   --  Achieved GSD = Slant * Sin (alpha) <= Slant.
   subtype Achieved_GSD_M is Real64 range 0.0 .. Slant_Range_M'Last;

   -----------------
   -- Loop counts --
   -----------------

   --  Elevation sweep: span <= 178 deg at 5-degree steps.
   Gimbal_Step_Size_Rad : constant := 5.0 * Deg_To_Rad;

   subtype Elevation_Step_Count is Positive range 1 .. 37;

   --  FOV sweep: valid candidates lie in (0, 179] on a 5-degree grid, so
   --  at most 36 of them; 0 means "no valid candidate". In C++ this loop
   --  is unbounded in the configuration values (a huge Min..Max range
   --  iterates essentially forever); the Ada bound makes that impossible.
   Horizontal_FOV_Step_Size_Deg : constant := 5.0;

   subtype FOV_Step_Count is Natural range 0 .. 37;

   --  Continuous-mode grids are anchored at the wire MinHorizontalFOV to
   --  stay candidate-for-candidate compatible with C++; anchors beyond
   --  this sanity bound (10 full turns) yield no candidates (D5).
   Max_FOV_Anchor_Magnitude_Deg : constant := 3_600.0;

   ---------------------------------
   -- Wire-value validity queries --
   ---------------------------------

   --  True iff X is neither NaN nor infinite. This is the SPARK boundary:
   --  everything downstream of these checks is free of special values.
   function Is_Finite (X : Real32) return Boolean;

   ----------------------
   -- Unit conversions --
   ----------------------

   function To_Radians (D : Degrees_64) return Radians_64 is
     (Radians_64 (Real64 (D) * Deg_To_Rad))
   with Pre => abs D <= 36_000.0;

   function To_Degrees (R : Radians_64) return Degrees_64 is
     (Degrees_64 (Real64 (R) / Deg_To_Rad))
   with Pre => abs R <= 700.0;

   --------------------------
   -- Elevation sweep math --
   --------------------------

   --  The elevation interval a gimbal will be swept through. When Valid
   --  is False the gimbal cannot point at the ground (or its limits are
   --  not finite) and it contributes no footprint, only the degenerate
   --  all-zero response the C++ service also produces.
   type Elevation_Sweep is record
      Valid : Boolean               := False;
      Lo    : Working_Elevation_Rad := Working_Elevation_Rad'Last;
      Hi    : Working_Elevation_Rad := Working_Elevation_Rad'Last;
   end record;

   --  C++-faithful clamping of a gimbal's elevation limits (min below
   --  -180 deg raised, max above 0 dropped to -1 deg, crossed limits
   --  pinned, unclamped gimbals given the full working range), followed
   --  by the final clamp of both ends into the working range (D4 at both
   --  edges). Non-finite limits invalidate the sweep.
   function Gimbal_Sweep_Range (Gimbal : GimbalConfig) return Elevation_Sweep
     with Post =>
       (if Gimbal_Sweep_Range'Result.Valid
        then Gimbal_Sweep_Range'Result.Lo <= Gimbal_Sweep_Range'Result.Hi);

   --  Requested elevation override, per UXTASK degrees (D2), clamped into
   --  the working range and the gimbal's own range (D3). Values in
   --  [0, 0.001) preserve the C++ dead path (sweep invalidated, so the
   --  degenerate footprint is emitted); values >= 0.001 are ignored, as
   --  in C++; non-finite values are ignored (for -Inf this diverges from
   --  C++, which pins to the gimbal minimum).
   function Apply_Override
     (Sweep    : Elevation_Sweep;
      Wire_Deg : Real32) return Elevation_Sweep
     with
       Pre  => (if Sweep.Valid then Sweep.Lo <= Sweep.Hi),
       Post =>
         (if Apply_Override'Result.Valid
          then Apply_Override'Result.Lo <= Apply_Override'Result.Hi);

   function Sweep_Step_Count (Sweep : Elevation_Sweep)
     return Elevation_Step_Count
     with Pre => Sweep.Valid and then Sweep.Lo <= Sweep.Hi;

   --  The elevation of a given sweep step, clamped to the sweep's upper
   --  end to absorb floating-point rounding in the last step.
   function Sweep_Elevation
     (Sweep : Elevation_Sweep;
      Step  : Natural) return Working_Elevation_Rad
     with Pre => Sweep.Valid
                   and then Sweep.Lo <= Sweep.Hi
                   and then Step < Sweep_Step_Count (Sweep);

   ------------------------
   -- FOV candidate math --
   ------------------------

   --  Valid FOV candidates of a continuous-mode camera: the 5-degree grid
   --  anchored at the wire minimum, intersected with (0, 179] (D5).
   --  First_Valid is the first grid point inside the valid range; Count
   --  is the number of valid grid points (0 when the range is empty, its
   --  ends are not finite, or the anchor is beyond the sanity bound).
   type FOV_Candidates is record
      Count       : FOV_Step_Count := 0;
      First_Valid : FOV_Deg        := Max_FOV_Deg;
   end record;

   function Continuous_Candidates
     (Wire_Min, Wire_Max : Real32) return FOV_Candidates;

   function Candidate_FOV
     (Candidates : FOV_Candidates;
      Index      : Natural) return FOV_Deg
     with Pre => Index < Candidates.Count;

   --  Validity filter for discrete-mode FOV list entries (D5).
   function Is_Valid_FOV (Wire : Real32) return Boolean is
     (Is_Finite (Wire)
        and then Degrees_64 (Wire) > 0.0
        and then Degrees_64 (Wire) <= Max_FOV_Deg);

   ------------------------------
   -- Camera-derived quantities --
   ------------------------------

   --  Resolutions above Max_Pixel_Count are clamped (D7).
   function Clamped_Resolution (Wire : UInt32) return Pixel_Count is
     (if Wire > Max_Pixel_Count then Max_Pixel_Count else Wire);

   function Min_Resolution (Camera : CameraConfig) return Pixel_Count is
     (Pixel_Count'Min
        (Clamped_Resolution (Camera.HorizResolution),
         Clamped_Resolution (Camera.VertResolution)));

   --  Width/height ratio; exactly 1.0 when the vertical resolution is
   --  zero (C++ rule).
   function Aspect_Ratio_Of (Camera : CameraConfig) return Aspect_Ratio_T is
     (if Clamped_Resolution (Camera.VertResolution) = 0
        or else Clamped_Resolution (Camera.HorizResolution) = 0
      then 1.0
      else Real64 (Clamped_Resolution (Camera.HorizResolution))
           / Real64 (Clamped_Resolution (Camera.VertResolution)));

   -------------------------
   -- Request sanitization --
   -------------------------

   --  Effective assigned altitude. Valid is False when the effective
   --  altitude (wire value, or nominal when the wire value is below the
   --  0.001 "unspecified" threshold) falls outside [10 m, 100 km] or is
   --  not finite; the C++ service checks only the lower bound (D6).
   type Altitude_Result is record
      Valid : Boolean             := False;
      Value : Assigned_Altitude_M := Assigned_Altitude_M'First;
   end record;

   function Effective_Altitude
     (Wire_M    : Real32;
      Nominal_M : Real32) return Altitude_Result;

   --  Effective desired GSD: below-threshold and non-finite values take
   --  the 1000.0 default (D8); absurdly large values are clamped.
   function Effective_Desired_GSD (Wire_M : Real32) return Desired_GSD_M;

   ------------------------
   -- Footprint geometry --
   ------------------------

   --  Altitude / Sin (-Elev); total because Sin (-Elev) >= sin (1 deg)
   --  over the working elevation range.
   function Slant_Range
     (Altitude : Assigned_Altitude_M;
      Elev     : Working_Elevation_Rad) return Slant_Range_M;

   --  GSD achieved by one camera pixel at the given slant range:
   --  Slant * Sin (alpha) with alpha = FOV_rad / resolution, or the
   --  worst-case Pi/2 when the resolution is unknown (C++ rule).
   function Compute_GSD
     (Slant   : Slant_Range_M;
      FOV     : FOV_Deg;
      Min_Res : Pixel_Count) return Achieved_GSD_M;

   --  D1: real-valued comparison of GSD distances from the desired value.
   --  The C++ code truncates both distances to integers first (its abs
   --  resolves to the C integer abs), making candidates within the same
   --  integer bucket "equal" so the first one found wins; that behavior
   --  was deliberately not reproduced.
   function Is_Better
     (Desired     : Desired_GSD_M;
      Candidate   : Achieved_GSD_M;
      Current     : Achieved_GSD_M;
      First_Found : Boolean) return Boolean
   is
     (not First_Found
        or else abs (Desired - Current) > abs (Desired - Candidate));

end Sensor_Manager_Types;
