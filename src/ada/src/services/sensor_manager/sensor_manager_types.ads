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

   Working_Elevation_Deg_Lo : constant := -179.0001;
   --  Lower degree-space bound of the working elevation range's image
   --  under To_Degrees. Mathematically the image is [-179, -1] degrees;
   --  the 1.0E-4 margin generously absorbs the floating-point rounding of
   --  the radian subtype bounds and of the conversion's division (a few
   --  parts in 1.0E13 at these magnitudes).

   Working_Elevation_Deg_Hi : constant := -0.9999;
   --  Upper degree-space bound of the working elevation range's image
   --  under To_Degrees, with the same rounding margin as
   --  Working_Elevation_Deg_Lo.

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
   with
     Pre  => abs R <= 700.0,
     Post =>
       (if R in Working_Elevation_Rad
        then To_Degrees'Result
               in Working_Elevation_Deg_Lo .. Working_Elevation_Deg_Hi);
   --  Convert an angle from radians to degrees. The postcondition bounds
   --  the image of the working elevation range, within the documented
   --  rounding margins (P4 support).
   --  @param R Angle in radians, bounded by 700
   --  @return R expressed in degrees

   --------------------------
   -- Elevation sweep math --
   --------------------------

   function Clamped_Elevation (Wire : Real32) return Elevation_Deg is
     (Degrees_64'Min
        (Degrees_64'Max (Degrees_64 (Wire), Elevation_Deg'First),
         Elevation_Deg'Last))
   with Pre => Is_Finite (Wire);
   --  Clamp a finite wire elevation into the documented CMASI range.
   --  @param Wire A finite wire elevation, in degrees
   --  @return Wire clamped into [-180, 180] degrees

   function Clamp_Working (R : Radians_64) return Working_Elevation_Rad is
     (Radians_64'Min
        (Radians_64'Max (R, Working_Elevation_Rad'First),
         Working_Elevation_Rad'Last));
   --  Clamp a radian elevation into the working range (D4).
   --  @param R An elevation, in radians
   --  @return R clamped into the working elevation range

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

   function Gimbal_Limits_Known (Gimbal : GimbalConfig) return Boolean is
     (Is_Finite (Gimbal.MinElevation)
      and then Is_Finite (Gimbal.MaxElevation))
   with Ghost;
   --  Whether both of a gimbal's wire elevation limits are ordinary
   --  numbers; non-finite limits invalidate the sweep (P6 support).
   --  @param Gimbal The gimbal configuration to examine
   --  @return True iff both elevation limits are finite

   function Gimbal_Lo_Rad (Gimbal : GimbalConfig) return Radians_64 is
     (if Gimbal.IsElevationClamped
      then To_Radians (Clamped_Elevation (Gimbal.MinElevation))
      else Working_Elevation_Rad'First)
   with Ghost, Pre => Gimbal_Limits_Known (Gimbal);
   --  A gimbal's repaired lower elevation limit, before the final working-
   --  range clamp: the CMASI-clamped wire minimum, or the bottom of the
   --  working range for a gimbal free to rotate 360 degrees (P6 support).
   --  @param Gimbal The gimbal configuration whose limits are read
   --  @return The repaired lower elevation limit, in radians

   function Gimbal_Hi_Rad (Gimbal : GimbalConfig) return Radians_64 is
     (if not Gimbal.IsElevationClamped
      then Working_Elevation_Rad'Last
      else
        Radians_64'Max
          (Gimbal_Lo_Rad (Gimbal),
           (if To_Radians (Clamped_Elevation (Gimbal.MaxElevation)) > 0.0
            then Working_Elevation_Rad'Last
            else To_Radians (Clamped_Elevation (Gimbal.MaxElevation)))))
   with Ghost, Pre => Gimbal_Limits_Known (Gimbal);
   --  A gimbal's repaired upper elevation limit, before the final working-
   --  range clamp, per the C++-faithful repairs: a max above horizontal is
   --  dropped to the top of the working range, and crossed limits collapse
   --  onto the min (P6 support).
   --  @param Gimbal The gimbal configuration whose limits are read
   --  @return The repaired upper elevation limit, in radians

   function Gimbal_Sweep_Range (Gimbal : GimbalConfig) return Elevation_Sweep
     with Post =>

       --  The sweep is valid exactly when both limits are finite and the
       --  repaired minimum points below horizontal, and it then covers the
       --  working-range clamp of the repaired limit interval (P6)

       Gimbal_Sweep_Range'Result.Valid =
         (Gimbal_Limits_Known (Gimbal)
            and then Gimbal_Lo_Rad (Gimbal) < 0.0)
       and then
         (if Gimbal_Sweep_Range'Result.Valid
          then Gimbal_Sweep_Range'Result.Lo
                 = Clamp_Working (Gimbal_Lo_Rad (Gimbal))
            and then Gimbal_Sweep_Range'Result.Hi
                 = Clamp_Working (Gimbal_Hi_Rad (Gimbal))
            and then Gimbal_Sweep_Range'Result.Lo
                 <= Gimbal_Sweep_Range'Result.Hi);
   --  C++-faithful clamping of a gimbal's elevation limits (min below
   --  -180 deg raised, max above 0 dropped to -1 deg, crossed limits
   --  pinned, unclamped gimbals given the full working range), followed by
   --  the final clamp of both ends into the working range (D4 at both
   --  edges). Non-finite limits invalidate the sweep. The postcondition
   --  characterizes the sweep exactly, in terms of the ghost repaired
   --  limits (Gold property P6: emitted elevations are achievable by the
   --  named gimbal).
   --  @param Gimbal The gimbal configuration whose limits are clamped
   --  @return The elevation interval to sweep, or an invalid sweep

   function Pinned_Override
     (Sweep    : Elevation_Sweep;
      Wire_Deg : Real32) return Working_Elevation_Rad
   is
     (Radians_64'Min
        (Radians_64'Max
           (Clamp_Working
              (To_Radians
                 (Degrees_64'Max
                    (Degrees_64 (Wire_Deg), Elevation_Deg'First))),
            Sweep.Lo),
         Sweep.Hi))
   with
     Ghost,
     Pre => Is_Finite (Wire_Deg)
              and then Degrees_64 (Wire_Deg) < 0.0
              and then Sweep.Lo <= Sweep.Hi;
   --  The elevation an active override pins the sweep to: the override in
   --  radians (D2), clamped into the working range (D3) and into the
   --  sweep's own interval — the documented clamping chain, stated
   --  positively (P6 support).
   --  @param Sweep The sweep computed from the gimbal's own limits
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @return The single elevation the overridden sweep is pinned to

   function Apply_Override
     (Sweep    : Elevation_Sweep;
      Wire_Deg : Real32) return Elevation_Sweep
     with
       Pre  => (if Sweep.Valid then Sweep.Lo <= Sweep.Hi),
       Post =>
         (if Apply_Override'Result.Valid
          then Apply_Override'Result.Lo <= Apply_Override'Result.Hi),
       Contract_Cases =>

         --  Non-finite and at-or-above-threshold overrides are ignored

         (not Is_Finite (Wire_Deg)
            =>
          Apply_Override'Result = Sweep,

          Is_Finite (Wire_Deg)
            and then Degrees_64 (Wire_Deg) >= 0.001
            =>
          Apply_Override'Result = Sweep,

          --  The [0, 0.001) sentinel band preserves the C++ dead path:
          --  the sweep is invalidated

          Is_Finite (Wire_Deg)
            and then Degrees_64 (Wire_Deg) >= 0.0
            and then Degrees_64 (Wire_Deg) < 0.001
            =>
          not Apply_Override'Result.Valid,

          --  A negative override pins a valid sweep to the documented
          --  clamp of the requested elevation (P6); an invalid sweep is
          --  returned unchanged

          Is_Finite (Wire_Deg)
            and then Degrees_64 (Wire_Deg) < 0.0
            =>
          (if Sweep.Valid
           then Apply_Override'Result.Valid
             and then Apply_Override'Result.Lo
               = Pinned_Override (Sweep, Wire_Deg)
             and then Apply_Override'Result.Hi
               = Pinned_Override (Sweep, Wire_Deg)
           else Apply_Override'Result = Sweep));
   --  Apply a requested elevation override, per UXTASK degrees (D2), clamped
   --  into the working range and the gimbal's own range (D3). Values in
   --  [0, 0.001) preserve the C++ dead path (sweep invalidated, so the
   --  degenerate footprint is emitted); values >= 0.001 are ignored, as in
   --  C++; non-finite values are ignored (for -Inf this diverges from C++,
   --  which pins to the gimbal minimum). The contract cases characterize
   --  the override exactly (Gold property P6: an in-range override is
   --  honored after the documented clamping).
   --  @param Sweep The sweep computed from the gimbal's own limits
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @return The sweep after applying the override

   function Sweep_Step_Count (Sweep : Elevation_Sweep)
     return Elevation_Step_Count
     with
       Pre  => Sweep.Valid and then Sweep.Lo <= Sweep.Hi,
       Post =>
         (if Sweep.Lo = Sweep.Hi then Sweep_Step_Count'Result = 1);
   --  Number of 5-degree steps the sweep is divided into. A pinned
   --  (single-point) sweep has exactly one step, so an override-pinned
   --  sweep is evaluated exactly at the pinned elevation (P6 support).
   --  @param Sweep A valid, non-empty elevation sweep
   --  @return The number of elevation steps to evaluate

   function Sweep_Elevation
     (Sweep : Elevation_Sweep;
      Step  : Natural) return Working_Elevation_Rad
     with
       Pre  => Sweep.Valid
                 and then Sweep.Lo <= Sweep.Hi
                 and then Step < Sweep_Step_Count (Sweep),
       Post => Sweep_Elevation'Result in Sweep.Lo .. Sweep.Hi;
   --  The elevation of a given sweep step, clamped to the sweep's upper end
   --  to absorb floating-point rounding in the last step. Always within
   --  the sweep interval, hence within the named gimbal's clamped limits
   --  (P6 support).
   --  @param Sweep A valid, non-empty elevation sweep
   --  @param Step The step index, in 0 .. Sweep_Step_Count (Sweep) - 1
   --  @return The elevation at that step

   function Sweep_Of
     (Gimbal   : GimbalConfig;
      Wire_Deg : Real32) return Elevation_Sweep
   is
     (Apply_Override (Gimbal_Sweep_Range (Gimbal), Wire_Deg))
   with
     Post =>
       (if Sweep_Of'Result.Valid
        then Sweep_Of'Result.Lo <= Sweep_Of'Result.Hi),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  The elevation sweep a gimbal is evaluated over under a request:
   --  its own clamped limits with the requested override applied (P6
   --  support). The body is hidden by default so that users of the sweep
   --  see only the explicit postcondition, not the clamping machinery's
   --  contracts; the full characterization is recovered by disclosing
   --  the body together with the Gimbal_Sweep_Range and Apply_Override
   --  contracts.
   --  @param Gimbal The gimbal configuration whose limits are swept
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @return The sweep the gimbal contributes under the request

   function Elevation_Of_Gimbal
     (Entity_Cfg : EntityConfig;
      Wire_Deg   : Real32;
      Gimbal_ID  : Int64;
      Elev       : Working_Elevation_Rad) return Boolean
   is
     (for some K in 1 .. Last (Entity_Cfg.Gimbals) =>
        Get (Entity_Cfg.Gimbals, K).PayloadID = Gimbal_ID
        and then Sweep_Of (Get (Entity_Cfg.Gimbals, K), Wire_Deg).Valid
        and then
          (for some S in
             0 .. Sweep_Step_Count
                    (Sweep_Of (Get (Entity_Cfg.Gimbals, K), Wire_Deg)) - 1
           =>
             Elev = Sweep_Elevation
                      (Sweep_Of (Get (Entity_Cfg.Gimbals, K), Wire_Deg),
                       S)))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether a working elevation traces to the entity's own gimbals: some
   --  gimbal of Entity_Cfg carries Gimbal_ID, its sweep under the request
   --  override is valid, and Elev is one of that sweep's steps — hence,
   --  by the sweep contracts, within the gimbal's clamped elevation
   --  limits and equal to the pinned override when one is active (Gold
   --  property P6).
   --  @param Entity_Cfg The entity configuration owning the gimbals
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @param Gimbal_ID The payload ID the footprint names as its gimbal
   --  @param Elev The boresight elevation to trace
   --  @return True iff Elev is a sweep step of an Entity_Cfg gimbal
   --    carrying Gimbal_ID

   procedure Lemma_Elevation_Of_Gimbal_Intro
     (Entity_Cfg   : EntityConfig;
      Wire_Deg     : Real32;
      Gimbal_Index : Positive;
      Step         : Natural)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Sweep_Of
                      (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                       Wire_Deg).Valid
           and then Step < Sweep_Step_Count
                             (Sweep_Of
                                (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                 Wire_Deg)),
       Post =>
         Elevation_Of_Gimbal
           (Entity_Cfg, Wire_Deg,
            Get (Entity_Cfg.Gimbals, Gimbal_Index).PayloadID,
            Sweep_Elevation
              (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index), Wire_Deg),
               Step));
   --  Introduction lemma for Elevation_Of_Gimbal: gimbal number
   --  Gimbal_Index of Entity_Cfg and sweep step Step are the witnesses.
   --  Proved here, where Elevation_Of_Gimbal's definition is in scope
   --  with little else; callers can keep the predicate's body hidden
   --  (P6 support).
   --  @param Entity_Cfg The entity configuration owning the gimbals
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @param Gimbal_Index Index of the witnessing gimbal
   --  @param Step The witnessing sweep step

   function Footprint_Elevation_Traceable
     (FP         : SensorFootprint_Msg;
      Entity_Cfg : EntityConfig;
      Wire_Deg   : Real32) return Boolean
   is
     (for some K in 1 .. Last (Entity_Cfg.Gimbals) =>
        Get (Entity_Cfg.Gimbals, K).PayloadID = FP.GimbalID
        and then Sweep_Of (Get (Entity_Cfg.Gimbals, K), Wire_Deg).Valid
        and then
          (for some S in
             0 .. Sweep_Step_Count
                    (Sweep_Of (Get (Entity_Cfg.Gimbals, K), Wire_Deg)) - 1
           =>
             FP.GimbalElevation
               = Real32
                   (To_Degrees
                      (Sweep_Elevation
                         (Sweep_Of (Get (Entity_Cfg.Gimbals, K), Wire_Deg),
                          S)))))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether a footprint's commanded elevation traces to the gimbal it
   --  names: some gimbal of Entity_Cfg carries FP.GimbalID, its sweep
   --  under the request override is valid, and FP.GimbalElevation is the
   --  Real32 degree image of one of that sweep's steps (Gold property P6:
   --  the commanded elevation is achievable by the named gimbal, and
   --  honors an active override after the documented clamping).
   --  @param FP The footprint whose commanded elevation is traced
   --  @param Entity_Cfg The entity configuration owning the gimbals
   --  @param Wire_Deg The requested elevation override, in degrees
   --  @return True iff FP's elevation is the degree image of a sweep step
   --    of an Entity_Cfg gimbal carrying FP.GimbalID

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
      Elev     : Working_Elevation_Rad) return Slant_Range_M
     with Post => Slant_Range'Result >= Altitude;
   --  Line-of-sight distance to the footprint center: Altitude / Sin (-Elev).
   --  Total because Sin (-Elev) >= sin (1 deg) over the working elevation
   --  range; at least Altitude because Sin (-Elev) <= 1 (P4 support).
   --  @param Altitude The assigned AGL altitude, in meters
   --  @param Elev The boresight elevation, in radians
   --  @return The slant range, in meters

   function Compute_GSD
     (Slant   : Slant_Range_M;
      FOV     : FOV_Deg;
      Min_Res : Pixel_Count) return Achieved_GSD_M
     with Post => Compute_GSD'Result <= Slant;
   --  GSD achieved by one camera pixel at the given slant range:
   --  Slant * Sin (alpha) with alpha = FOV_rad / resolution, or the
   --  worst-case Pi/2 when the resolution is unknown (C++ rule). At most
   --  Slant because Sin (alpha) <= 1 (P4 support).
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

   ----------------------------------
   -- Wire-level footprint ranges  --
   ----------------------------------

   function Footprint_Geometry_Defaulted
     (FP : SensorFootprint_Msg) return Boolean
   is
     (FP.CameraID = 0
      and then FP.GimbalID = 0
      and then FP.HorizontalFOV = 0.0
      and then FP.AglAltitude = 0.0
      and then FP.GimbalElevation = 0.0
      and then FP.AspectRatio = 0.0
      and then FP.AchievedGSD = 0.0
      and then FP.CameraWavelength = AllAny
      and then FP.HorizontalToLeadingEdge = 0.0
      and then FP.HorizontalToTrailingEdge = 0.0
      and then FP.HorizontalToCenter = 0.0
      and then FP.WidthCenter = 0.0
      and then FP.SlantRangeToCenter = 0.0)
   with Ghost;
   --  Whether a footprint's selection and geometry fields all hold their
   --  default (all-zero) values, as in a freshly built footprint for
   --  which no sensor candidate was accepted — the degenerate footprint
   --  consumers rely on for positional correlation (P4/P8 support). The
   --  correlation fields FootprintResponseID and VehicleID are not
   --  constrained.
   --  @param FP The footprint to examine
   --  @return True iff every selection and geometry field is defaulted

   function Footprint_In_Wire_Ranges
     (FP : SensorFootprint_Msg) return Boolean
   is
     (FP.AglAltitude
        in Real32 (Assigned_Altitude_M'First)
           .. Real32 (Assigned_Altitude_M'Last)
      and then FP.GimbalElevation
        in Working_Elevation_Deg_Lo .. Working_Elevation_Deg_Hi
      and then FP.HorizontalFOV in 0.0 .. Real32 (Max_FOV_Deg)
      and then FP.AspectRatio
        in Real32 (Aspect_Ratio_T'First) .. Real32 (Aspect_Ratio_T'Last)
      and then FP.AchievedGSD in 0.0 .. FP.SlantRangeToCenter
      and then FP.SlantRangeToCenter
        in FP.AglAltitude .. Real32 (Slant_Range_M'Last)
      and then FP.HorizontalToCenter
        in Real32 (Center_Distance_M'First)
           .. Real32 (Center_Distance_M'Last)
      and then FP.HorizontalToLeadingEdge
        in Real32 (Edge_Distance_M'First) .. Real32 (Edge_Distance_M'Last)
      and then FP.HorizontalToTrailingEdge
        in Real32 (Edge_Distance_M'First) .. Real32 (Edge_Distance_M'Last)
      and then FP.WidthCenter in 0.0 .. Real32 (Width_M'Last))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether a footprint's selection and geometry fields lie within the
   --  Real32 images of their constrained Real64 working subtypes — the
   --  "no nonsense values on the bus" property (Gold property P4). Two
   --  clauses are relational: the achieved GSD never exceeds the slant
   --  range (Sin <= 1), and the slant range is at least the altitude
   --  (again Sin <= 1). HorizontalFOV is bounded by [0, 179] rather than
   --  the working subtype's strict (0, 179]: strict positivity of the
   --  Real32 image would require reasoning about sub-denormal FOV grid
   --  anchors (a Real64 FOV below 2.0**(-150) degrees rounds to 0.0 in
   --  Real32), see GOLD_PROPERTIES.md.
   --  @param FP The footprint to examine
   --  @return True iff every selection and geometry field is in range

   function Footprint_Wire_OK (FP : SensorFootprint_Msg) return Boolean is
     (Footprint_Geometry_Defaulted (FP)
      or else Footprint_In_Wire_Ranges (FP))
   with Ghost;
   --  Wire-level validity of one emitted footprint (Gold property P4):
   --  either no sensor candidate was accepted and the footprint is the
   --  degenerate all-default one, or every selection and geometry field
   --  lies within the Real32 image of its working subtype.
   --  @param FP The footprint to examine
   --  @return True iff FP is degenerate or within all wire-level ranges

   -------------------------------
   -- Camera and FOV provenance --
   -------------------------------

   function Camera_On_Gimbal
     (Entity_Cfg   : EntityConfig;
      Gimbal_ID    : Int64;
      Camera_Index : Positive) return Boolean
   is
     (Camera_Index <= Last (Entity_Cfg.Cameras)
      and then
        (for some K in 1 .. Last (Entity_Cfg.Gimbals) =>
           Get (Entity_Cfg.Gimbals, K).PayloadID = Gimbal_ID
           and then
             (for some CI in
                1 .. Last (Get (Entity_Cfg.Gimbals, K)
                             .ContainedPayloadList)
              =>
                Get (Get (Entity_Cfg.Gimbals, K).ContainedPayloadList,
                     CI)
                  = Get (Entity_Cfg.Cameras, Camera_Index).PayloadID)))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether camera number Camera_Index of Entity_Cfg is mounted on a
   --  gimbal of Entity_Cfg carrying Gimbal_ID: the camera's payload ID
   --  appears in that gimbal's ContainedPayloadList — the CMASI
   --  payload-configuration mounting model (Gold property P5).
   --  @param Entity_Cfg The entity configuration owning gimbals and
   --    cameras
   --  @param Gimbal_ID The payload ID of the mounting gimbal
   --  @param Camera_Index Index of the camera in Entity_Cfg.Cameras
   --  @return True iff the camera is in the contained-payload list of an
   --    Entity_Cfg gimbal carrying Gimbal_ID

   function Valid_FOV_Of
     (Camera : CameraConfig;
      FOV    : FOV_Deg) return Boolean
   is
     (if Camera.FieldOfViewMode = Continuous
      then
        (for some Index in
           0 .. Continuous_Candidates
                  (Camera.MinHorizontalFOV,
                   Camera.MaxHorizontalFOV).Count - 1
         =>
           FOV = Candidate_FOV
                   (Continuous_Candidates
                      (Camera.MinHorizontalFOV, Camera.MaxHorizontalFOV),
                    Index))
      else
        (for some DI in 1 .. Last (Camera.DiscreteHFOVList) =>
           Is_Valid_FOV (Get (Camera.DiscreteHFOVList, DI))
           and then FOV
             = FOV_Deg (Degrees_64 (Get (Camera.DiscreteHFOVList, DI)))))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether FOV is one of a camera's valid field-of-view candidates:
   --  a point of the 5-degree grid within the wire min/max in continuous
   --  mode, or a valid entry of DiscreteHorizontalFieldOfViewList in
   --  discrete mode, in both cases inside (0, 179] degrees (D5; Gold
   --  property P5).
   --  @param Camera The camera configuration supplying the candidates
   --  @param FOV The working-precision field of view to trace
   --  @return True iff FOV is a valid candidate of Camera

   function Camera_FOV_Image
     (Camera   : CameraConfig;
      Wire_FOV : Real32) return Boolean
   is
     (if Camera.FieldOfViewMode = Continuous
      then
        (for some Index in
           0 .. Continuous_Candidates
                  (Camera.MinHorizontalFOV,
                   Camera.MaxHorizontalFOV).Count - 1
         =>
           Wire_FOV
             = Real32 (Candidate_FOV
                         (Continuous_Candidates
                            (Camera.MinHorizontalFOV,
                             Camera.MaxHorizontalFOV),
                          Index)))
      else
        (for some DI in 1 .. Last (Camera.DiscreteHFOVList) =>
           Is_Valid_FOV (Get (Camera.DiscreteHFOVList, DI))
           and then Wire_FOV
             = Real32 (Degrees_64 (Get (Camera.DiscreteHFOVList, DI)))))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Wire-level image of Valid_FOV_Of: whether Wire_FOV is the Real32
   --  image of one of the camera's valid FOV candidates (Gold property
   --  P5).
   --  @param Camera The camera configuration supplying the candidates
   --  @param Wire_FOV The wire field of view to trace, in degrees
   --  @return True iff Wire_FOV is the Real32 image of a valid candidate

   function Footprint_Camera_Traceable
     (FP              : SensorFootprint_Msg;
      Entity_Cfg      : EntityConfig;
      Wavelength_Wire : WavelengthBandEnum) return Boolean
   is
     (for some CJ in 1 .. Last (Entity_Cfg.Cameras) =>
        Get (Entity_Cfg.Cameras, CJ).PayloadID = FP.CameraID
        and then FP.CameraWavelength
          = Get (Entity_Cfg.Cameras, CJ).SupportedWavelengthBand
        and then (Get (Entity_Cfg.Cameras, CJ).SupportedWavelengthBand
                    = Wavelength_Wire
                  or else Wavelength_Wire = AllAny)
        and then Camera_On_Gimbal (Entity_Cfg, FP.GimbalID, CJ)
        and then Camera_FOV_Image (Get (Entity_Cfg.Cameras, CJ),
                                   FP.HorizontalFOV))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether a footprint's selected camera traces to the entity's own
   --  configuration (Gold property P5): some camera of Entity_Cfg
   --  carries FP.CameraID, that camera is mounted on a gimbal carrying
   --  FP.GimbalID, FP.CameraWavelength is that camera's supported band
   --  and satisfies the request's eligibility filter (equal to the
   --  requested band, or the request was AllAny), and FP.HorizontalFOV
   --  is the Real32 image of one of that camera's valid FOV candidates.
   --  Caveat: the gimbal witness here is existential and separate from
   --  Footprint_Elevation_Traceable's; if an entity announces two
   --  gimbals with the same PayloadID, the two predicates may be
   --  witnessed by different gimbals (see GOLD_PROPERTIES.md).
   --  @param FP The footprint whose camera selection is traced
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Wavelength_Wire The requested eligible wavelength band
   --  @return True iff FP's camera fields trace to a mounted, eligible
   --    camera of Entity_Cfg with a valid FOV candidate

   procedure Lemma_Camera_On_Gimbal_Intro
     (Entity_Cfg    : EntityConfig;
      Gimbal_Index  : Positive;
      List_Position : Positive;
      Camera_Index  : Positive)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Camera_Index <= Last (Entity_Cfg.Cameras)
           and then List_Position
             <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList)
           and then Get (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                           .ContainedPayloadList,
                         List_Position)
             = Get (Entity_Cfg.Cameras, Camera_Index).PayloadID,
       Post =>
         Camera_On_Gimbal
           (Entity_Cfg,
            Get (Entity_Cfg.Gimbals, Gimbal_Index).PayloadID,
            Camera_Index);
   --  Introduction lemma for Camera_On_Gimbal: gimbal number
   --  Gimbal_Index and position List_Position of its contained-payload
   --  list are the witnesses. Proved here, where the predicate's
   --  definition is in scope with little else; callers keep the
   --  predicate hidden (P5 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Gimbal_Index Index of the witnessing gimbal
   --  @param List_Position Witnessing position in the gimbal's
   --    contained-payload list
   --  @param Camera_Index Index of the mounted camera

   procedure Lemma_Valid_FOV_Continuous_Intro
     (Camera : CameraConfig;
      Index  : Natural)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Camera.FieldOfViewMode = Continuous
           and then Index < Continuous_Candidates
                              (Camera.MinHorizontalFOV,
                               Camera.MaxHorizontalFOV).Count,
       Post =>
         Valid_FOV_Of
           (Camera,
            Candidate_FOV
              (Continuous_Candidates
                 (Camera.MinHorizontalFOV, Camera.MaxHorizontalFOV),
               Index));
   --  Introduction lemma for Valid_FOV_Of in continuous mode: grid
   --  candidate number Index is the witness (P5 support).
   --  @param Camera A continuous-mode camera configuration
   --  @param Index The witnessing candidate index

   procedure Lemma_Valid_FOV_Discrete_Intro
     (Camera : CameraConfig;
      DI     : Positive)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Camera.FieldOfViewMode /= Continuous
           and then DI <= Last (Camera.DiscreteHFOVList)
           and then Is_Valid_FOV (Get (Camera.DiscreteHFOVList, DI)),
       Post =>
         Valid_FOV_Of
           (Camera,
            FOV_Deg (Degrees_64 (Get (Camera.DiscreteHFOVList, DI))));
   --  Introduction lemma for Valid_FOV_Of in discrete mode: list entry
   --  DI is the witness (P5 support).
   --  @param Camera A discrete-mode camera configuration
   --  @param DI The witnessing list position

   ------------------------------
   -- GSD candidate model (P7) --
   ------------------------------

   --  The joint candidate model for Gold property P7: a candidate is one
   --  (gimbal, sweep step, camera, FOV) tuple of the entity configuration
   --  under a request, and Candidate_GSD is the ground sample distance
   --  the service computes for it. The unified 1-based FOV candidate
   --  index space covers both FOV modes: index FI names continuous grid
   --  candidate FI - 1, or discrete list entry FI (invalid discrete
   --  entries are indexed but filtered out by FOV_Candidate_Valid).

   function FOV_Candidate_Bound (Camera : CameraConfig) return Natural is
     (if Camera.FieldOfViewMode = Continuous
      then Continuous_Candidates
             (Camera.MinHorizontalFOV, Camera.MaxHorizontalFOV).Count
      else Last (Camera.DiscreteHFOVList))
   with Ghost;
   --  Size of a camera's unified FOV candidate index space (P7 support).
   --  @param Camera The camera configuration supplying the candidates
   --  @return The largest candidate index (0 when there are none)

   function FOV_Candidate_Valid
     (Camera    : CameraConfig;
      FOV_Index : Positive) return Boolean
   is
     (FOV_Index <= FOV_Candidate_Bound (Camera)
      and then
        (Camera.FieldOfViewMode = Continuous
         or else Is_Valid_FOV (Get (Camera.DiscreteHFOVList, FOV_Index))))
   with Ghost;
   --  Whether an index of the unified FOV candidate space names a valid
   --  candidate: continuous grid points always do; discrete entries must
   --  pass the Is_Valid_FOV filter (D5; P7 support).
   --  @param Camera The camera configuration supplying the candidates
   --  @param FOV_Index The candidate index to test
   --  @return True iff FOV_Index names a valid FOV candidate of Camera

   function Camera_FOV_At
     (Camera    : CameraConfig;
      FOV_Index : Positive) return FOV_Deg
   is
     (if Camera.FieldOfViewMode = Continuous
      then Candidate_FOV
             (Continuous_Candidates
                (Camera.MinHorizontalFOV, Camera.MaxHorizontalFOV),
              FOV_Index - 1)
      else FOV_Deg (Degrees_64 (Get (Camera.DiscreteHFOVList, FOV_Index))))
   with Ghost, Pre => FOV_Candidate_Valid (Camera, FOV_Index);
   --  The field of view of a valid candidate of the unified index space
   --  (P7 support).
   --  @param Camera The camera configuration supplying the candidates
   --  @param FOV_Index A valid candidate index
   --  @return The field of view of that candidate

   function Wavelength_Eligible
     (Camera              : CameraConfig;
      Eligible_Wavelength : WavelengthBandEnum) return Boolean
   is
     (Camera.SupportedWavelengthBand = Eligible_Wavelength
      or else Eligible_Wavelength = AllAny)
   with Ghost;
   --  Whether a camera passes the request's wavelength eligibility
   --  filter: its supported band is the requested one, or the request
   --  accepts any band (P7 support).
   --  @param Camera The camera configuration to test
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @return True iff the camera is eligible under the request

   function Mounted_On
     (Entity_Cfg   : EntityConfig;
      Gimbal_Index : Positive;
      Camera_Index : Positive) return Boolean
   is
     (Gimbal_Index <= Last (Entity_Cfg.Gimbals)
      and then Camera_Index <= Last (Entity_Cfg.Cameras)
      and then
        (for some P in
           1 .. Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList)
         =>
           Get (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                  .ContainedPayloadList,
                P)
             = Get (Entity_Cfg.Cameras, Camera_Index).PayloadID))
   with Ghost;
   --  Whether camera number Camera_Index is mounted on gimbal number
   --  Gimbal_Index: the camera's payload ID appears in that gimbal's
   --  contained-payload list. The index-based counterpart of the
   --  ID-based Camera_On_Gimbal (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Camera_Index Index of the camera in Entity_Cfg.Cameras
   --  @return True iff both indices are in range and the camera is in
   --    the gimbal's contained-payload list

   function Dominates
     (Desired : Desired_GSD_M;
      Best    : Achieved_GSD_M;
      Cand    : Achieved_GSD_M) return Boolean
   is
     (abs (Desired - Best) <= abs (Desired - Cand))
   with Ghost;
   --  Whether one achieved GSD matches the desired GSD at least as well
   --  as another: its distance from the desired value is no larger.
   --  Deliberately not hidden: the plain arithmetic (in particular its
   --  transitivity) must stay visible to the provers (P7 support).
   --  @param Desired The desired ground sample distance
   --  @param Best The dominating achieved GSD
   --  @param Cand The dominated achieved GSD
   --  @return True iff Best is at least as close to Desired as Cand

   function Is_Candidate
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Gimbal_Index        : Positive;
      Step                : Natural;
      Camera_Index        : Positive;
      FOV_Index           : Positive) return Boolean
   is
     (Gimbal_Index <= Last (Entity_Cfg.Gimbals)
      and then Camera_Index <= Last (Entity_Cfg.Cameras)
      and then Sweep_Of
                 (Get (Entity_Cfg.Gimbals, Gimbal_Index), Elev_Wire).Valid
      and then Step < Sweep_Step_Count
                        (Sweep_Of
                           (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                            Elev_Wire))
      and then Mounted_On (Entity_Cfg, Gimbal_Index, Camera_Index)
      and then Wavelength_Eligible
                 (Get (Entity_Cfg.Cameras, Camera_Index),
                  Eligible_Wavelength)
      and then FOV_Candidate_Valid
                 (Get (Entity_Cfg.Cameras, Camera_Index), FOV_Index))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether a (gimbal, sweep step, camera, FOV) tuple is a candidate
   --  of the entity configuration under a request: both payload indices
   --  are in range, the gimbal's sweep under the override is valid and
   --  Step is one of its steps, the camera is mounted on the gimbal and
   --  wavelength-eligible, and FOV_Index names a valid FOV candidate
   --  (Gold property P7).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Step The sweep step index
   --  @param Camera_Index Index of the camera in Entity_Cfg.Cameras
   --  @param FOV_Index The FOV candidate index
   --  @return True iff the tuple is a candidate under the request

   function Candidate_GSD
     (Entity_Cfg   : EntityConfig;
      Altitude     : Assigned_Altitude_M;
      Elev_Wire    : Real32;
      Gimbal_Index : Positive;
      Step         : Natural;
      Camera_Index : Positive;
      FOV_Index    : Positive) return Achieved_GSD_M
   is
     (Compute_GSD
        (Slant_Range
           (Altitude,
            Sweep_Elevation
              (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                         Elev_Wire),
               Step)),
         Camera_FOV_At (Get (Entity_Cfg.Cameras, Camera_Index),
                        FOV_Index),
         Min_Resolution (Get (Entity_Cfg.Cameras, Camera_Index))))
   with
     Ghost,
     Pre => Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then Camera_Index <= Last (Entity_Cfg.Cameras)
              and then Sweep_Of
                         (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                          Elev_Wire).Valid
              and then Step < Sweep_Step_Count
                                (Sweep_Of
                                   (Get (Entity_Cfg.Gimbals,
                                         Gimbal_Index),
                                    Elev_Wire))
              and then FOV_Candidate_Valid
                         (Get (Entity_Cfg.Cameras, Camera_Index),
                          FOV_Index),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  The ground sample distance the service computes for a candidate:
   --  Compute_GSD at the candidate's slant range, field of view and
   --  minimum resolution — definitionally the value Consider_Candidate
   --  evaluates (Gold property P7). The precondition repeats the range
   --  and validity conjuncts of Is_Candidate (not eligibility or
   --  mounting, which do not affect the value).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Step The sweep step index
   --  @param Camera_Index Index of the camera in Entity_Cfg.Cameras
   --  @param FOV_Index The FOV candidate index
   --  @return The achieved GSD of the candidate

   -----------------------------
   -- Candidate coverage (P7) --
   -----------------------------

   --  Each predicate of this family states that every candidate of a
   --  prefix of the service's enumeration order (a) forces Found and
   --  (b) is dominated by Best: its GSD is no closer to Desired. With
   --  Found = False the same predicate states that the prefix has no
   --  candidates at all, so one invariant family carries optimality and
   --  completeness together (Gold property P7).

   function Camera_Covered_Upto
     (Camera  : CameraConfig;
      FI_Hi   : Natural;
      Slant   : Slant_Range_M;
      Desired : Desired_GSD_M;
      Found   : Boolean;
      Best    : Achieved_GSD_M) return Boolean
   is
     (for all FI in 1 .. FI_Hi =>
        (if FOV_Candidate_Valid (Camera, FI)
         then Found
           and then Dominates
                      (Desired, Best,
                       Compute_GSD
                         (Slant,
                          Camera_FOV_At (Camera, FI),
                          Min_Resolution (Camera)))))
   with
     Ghost,
     Pre => FI_Hi <= FOV_Candidate_Bound (Camera),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether every valid FOV candidate of Camera with index at most
   --  FI_Hi, evaluated at Slant, forces Found and is dominated by Best
   --  (P7 support).
   --  @param Camera The camera configuration supplying the candidates
   --  @param FI_Hi The covered prefix of the FOV candidate index space
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the prefix is covered

   function Camera_Covered
     (Camera  : CameraConfig;
      Slant   : Slant_Range_M;
      Desired : Desired_GSD_M;
      Found   : Boolean;
      Best    : Achieved_GSD_M) return Boolean
   is
     (Camera_Covered_Upto
        (Camera, FOV_Candidate_Bound (Camera), Slant, Desired, Found,
         Best))
   with Ghost;
   --  Whether every valid FOV candidate of Camera, evaluated at Slant,
   --  forces Found and is dominated by Best: Camera_Covered_Upto at the
   --  full candidate bound (thin wrapper, deliberately not hidden; P7
   --  support).
   --  @param Camera The camera configuration supplying the candidates
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the camera's whole candidate set is covered

   function Position_Covered_Upto
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Position            : Positive;
      CJ_Hi               : Natural;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M) return Boolean
   is
     (for all CJ in 1 .. CJ_Hi =>
        (if Get (Entity_Cfg.Cameras, CJ).PayloadID
              = Get (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                       .ContainedPayloadList,
                     Position)
           and then Wavelength_Eligible
                      (Get (Entity_Cfg.Cameras, CJ),
                       Eligible_Wavelength)
         then Camera_Covered
                (Get (Entity_Cfg.Cameras, CJ), Slant, Desired, Found,
                 Best)))
   with
     Ghost,
     Pre => Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then Position
                <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                           .ContainedPayloadList)
              and then CJ_Hi <= Last (Entity_Cfg.Cameras),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether every camera with index at most CJ_Hi that matches one
   --  contained-payload position of a gimbal and passes the wavelength
   --  filter is covered at Slant (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Position Position in the gimbal's contained-payload list
   --  @param CJ_Hi The covered prefix of Entity_Cfg.Cameras
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the camera prefix is covered for this position

   function Positions_Covered_Upto
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      P_Hi                : Natural;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M) return Boolean
   is
     (for all P in 1 .. P_Hi =>
        Position_Covered_Upto
          (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, P,
           Last (Entity_Cfg.Cameras), Slant, Desired, Found, Best))
   with
     Ghost,
     Pre => Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then P_Hi
                <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                           .ContainedPayloadList),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether every contained-payload position of a gimbal with index
   --  at most P_Hi is covered over the full camera list at Slant (P7
   --  support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param P_Hi The covered prefix of the contained-payload list
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the position prefix is covered

   function Steps_Covered_Upto
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      S_Hi                : Natural;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M) return Boolean
   is
     (for all S in 0 .. S_Hi - 1 =>
        Positions_Covered_Upto
          (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
           Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                   .ContainedPayloadList),
           Slant_Range
             (Altitude,
              Sweep_Elevation
                (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                           Elev_Wire),
                 S)),
           Desired, Found, Best))
   with
     Ghost,
     Pre => Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then Sweep_Of
                         (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                          Elev_Wire).Valid
              and then S_Hi <= Sweep_Step_Count
                                 (Sweep_Of
                                    (Get (Entity_Cfg.Gimbals,
                                          Gimbal_Index),
                                     Elev_Wire)),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether every sweep step of a gimbal below S_Hi is covered over
   --  the gimbal's full contained-payload list, at the slant range of
   --  that step's elevation (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param S_Hi The number of covered sweep steps
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the step prefix is covered

   function Gimbal_Covered
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M) return Boolean
   is
     (if Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                   Elev_Wire).Valid
      then Steps_Covered_Upto
             (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
              Gimbal_Index,
              Sweep_Step_Count
                (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                           Elev_Wire)),
              Desired, Found, Best))
   with
     Ghost,
     Pre => Gimbal_Index <= Last (Entity_Cfg.Gimbals),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether one gimbal's whole candidate set is covered: a valid
   --  sweep must be covered at its full step count; an invalid sweep
   --  contributes no candidates, so it is covered vacuously (P7
   --  support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the gimbal's candidate set is covered

   function Gimbals_Covered_Upto
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      K_Hi                : Natural;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M) return Boolean
   is
     (for all K in 1 .. K_Hi =>
        Gimbal_Covered
          (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, K,
           Desired, Found, Best))
   with
     Ghost,
     Pre => K_Hi <= Last (Entity_Cfg.Gimbals),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether every gimbal with index at most K_Hi is covered (P7
   --  support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param K_Hi The covered prefix of Entity_Cfg.Gimbals
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the gimbal prefix is covered

   function Candidates_Covered
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M) return Boolean
   is
     (Gimbals_Covered_Upto
        (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
         Last (Entity_Cfg.Gimbals), Desired, Found, Best))
   with Ghost;
   --  Whether the entire candidate set of the entity configuration
   --  under the request is covered: Gimbals_Covered_Upto at the full
   --  gimbal list (thin wrapper, deliberately not hidden; Gold property
   --  P7).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far
   --  @return True iff the whole candidate set is covered

   --------------------------------------------
   -- Selection witness and optimality (P7)  --
   --------------------------------------------

   function Selection_Witness
     (FP           : SensorFootprint_Msg;
      Entity_Cfg   : EntityConfig;
      Elev_Wire    : Real32;
      Altitude     : Assigned_Altitude_M;
      Gimbal_Index : Positive;
      Step         : Natural;
      Camera_Index : Positive;
      FOV_Index    : Positive;
      Best         : Achieved_GSD_M) return Boolean
   is
     (Best = Candidate_GSD
               (Entity_Cfg, Altitude, Elev_Wire, Gimbal_Index, Step,
                Camera_Index, FOV_Index)
      and then FP.AchievedGSD = Real32 (Best)
      and then FP.CameraID
        = Get (Entity_Cfg.Cameras, Camera_Index).PayloadID
      and then FP.GimbalID
        = Get (Entity_Cfg.Gimbals, Gimbal_Index).PayloadID
      and then FP.CameraWavelength
        = Get (Entity_Cfg.Cameras, Camera_Index).SupportedWavelengthBand
      and then FP.HorizontalFOV
        = Real32 (Camera_FOV_At
                    (Get (Entity_Cfg.Cameras, Camera_Index), FOV_Index))
      and then FP.GimbalElevation
        = Real32 (To_Degrees
                    (Sweep_Elevation
                       (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                  Elev_Wire),
                        Step)))
      and then FP.AglAltitude = Real32 (Altitude))
   with
     Ghost,
     Pre => Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then Camera_Index <= Last (Entity_Cfg.Cameras)
              and then Sweep_Of
                         (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                          Elev_Wire).Valid
              and then Step < Sweep_Step_Count
                                (Sweep_Of
                                   (Get (Entity_Cfg.Gimbals,
                                         Gimbal_Index),
                                    Elev_Wire))
              and then FOV_Candidate_Valid
                         (Get (Entity_Cfg.Cameras, Camera_Index),
                          FOV_Index),
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether one candidate tuple jointly explains a footprint's
   --  selection fields: Best is that candidate's GSD, and the
   --  footprint's achieved GSD, camera and gimbal IDs, wavelength,
   --  FOV, elevation and altitude are the wire images of that same
   --  tuple's values — one witness for all fields, which closes the
   --  P5/P6 same-gimbal caveat (Gold property P7).
   --  @param FP The footprint whose selection fields are witnessed
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Step The sweep step index
   --  @param Camera_Index Index of the camera in Entity_Cfg.Cameras
   --  @param FOV_Index The FOV candidate index
   --  @param Best The best achieved GSD tracked by the search
   --  @return True iff the tuple explains every selection field of FP

   function Footprint_GSD_Optimal
     (FP                  : SensorFootprint_Msg;
      Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired             : Desired_GSD_M;
      Altitude            : Assigned_Altitude_M;
      Elev_Wire           : Real32) return Boolean
   is
     (for some K in 1 .. Last (Entity_Cfg.Gimbals) =>
        Sweep_Of (Get (Entity_Cfg.Gimbals, K), Elev_Wire).Valid
        and then
          (for some S in 0 .. Elevation_Step_Count'Last - 1 =>
             S < Sweep_Step_Count
                   (Sweep_Of (Get (Entity_Cfg.Gimbals, K), Elev_Wire))
             and then
               (for some CJ in 1 .. Last (Entity_Cfg.Cameras) =>
                  (for some FI in
                     1 .. FOV_Candidate_Bound
                            (Get (Entity_Cfg.Cameras, CJ))
                   =>
                     FOV_Candidate_Valid
                       (Get (Entity_Cfg.Cameras, CJ), FI)
                     and then Is_Candidate
                                (Entity_Cfg, Eligible_Wavelength,
                                 Elev_Wire, K, S, CJ, FI)
                     and then Selection_Witness
                                (FP, Entity_Cfg, Elev_Wire, Altitude, K,
                                 S, CJ, FI,
                                 Candidate_GSD
                                   (Entity_Cfg, Altitude, Elev_Wire, K,
                                    S, CJ, FI))
                     and then Candidates_Covered
                                (Entity_Cfg, Eligible_Wavelength,
                                 Elev_Wire, Altitude, Desired, True,
                                 Candidate_GSD
                                   (Entity_Cfg, Altitude, Elev_Wire, K,
                                    S, CJ, FI))))))
   with
     Ghost,
     Annotate => (GNATprove, Hide_Info, "Expression_Function_Body");
   --  Whether a footprint's selected GSD attains the minimum distance
   --  to the desired GSD over the entire candidate set: some candidate
   --  tuple is a Selection_Witness for FP and every candidate is
   --  dominated by that tuple's GSD (Gold property P7). The sweep
   --  validity, step and FOV validity conjuncts restate consequences of
   --  Is_Candidate; they are semantically redundant but must appear
   --  explicitly because Is_Candidate is hidden and could not otherwise
   --  guard the partial calls that follow.
   --  @param FP The footprint whose selection is claimed optimal
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Desired The desired ground sample distance
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @return True iff FP's selection attains the global GSD minimum

   function No_Candidate
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Desired             : Desired_GSD_M) return Boolean
   is
     (Candidates_Covered
        (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, Desired,
         False, 0.0))
   with Ghost;
   --  Whether the entity configuration has no candidates at all under
   --  the request: full coverage with Found = False (thin wrapper,
   --  deliberately not hidden). The Altitude and Desired arguments are
   --  semantically inert here but syntactically required (Gold property
   --  P7).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Desired The desired ground sample distance
   --  @return True iff the candidate set is empty

   -------------------------------
   -- Coverage lemmas (P7)      --
   -------------------------------

   --  Two lemma shapes per coverage level, proved here where the
   --  predicate definitions are in scope with little else, and called
   --  with ground scope arguments. Extend: coverage of the prefix below
   --  Hi plus the Hi-th element's obligation (vacuous when filtered
   --  out) extends coverage to Hi. Monotone: coverage carries over to a
   --  later search state whose flag is at least as high and whose best
   --  GSD dominates the earlier one; with Found0 = False the covered
   --  prefix has no candidates, so any target state follows — this
   --  instance also rewrites full coverage with Found = False to the
   --  literal No_Candidate form.

   procedure Lemma_Camera_Covered_Extend
     (Camera  : CameraConfig;
      FI_Hi   : Positive;
      Slant   : Slant_Range_M;
      Desired : Desired_GSD_M;
      Found   : Boolean;
      Best    : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         FI_Hi <= FOV_Candidate_Bound (Camera)
           and then Camera_Covered_Upto
                      (Camera, FI_Hi - 1, Slant, Desired, Found, Best)
           and then
             (if FOV_Candidate_Valid (Camera, FI_Hi)
              then Found
                and then Dominates
                           (Desired, Best,
                            Compute_GSD
                              (Slant,
                               Camera_FOV_At (Camera, FI_Hi),
                               Min_Resolution (Camera)))),
       Post =>
         Camera_Covered_Upto (Camera, FI_Hi, Slant, Desired, Found, Best);
   --  Extension lemma for Camera_Covered_Upto: the FI_Hi-th candidate's
   --  obligation (vacuous for an invalid index) extends the covered
   --  prefix by one (P7 support).
   --  @param Camera The camera configuration supplying the candidates
   --  @param FI_Hi The new prefix bound
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Camera_Covered_Monotone
     (Camera  : CameraConfig;
      FI_Hi   : Natural;
      Slant   : Slant_Range_M;
      Desired : Desired_GSD_M;
      Found0  : Boolean;
      Best0   : Achieved_GSD_M;
      Found1  : Boolean;
      Best1   : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         FI_Hi <= FOV_Candidate_Bound (Camera)
           and then Camera_Covered_Upto
                      (Camera, FI_Hi, Slant, Desired, Found0, Best0)
           and then (if Found0
                     then Found1
                       and then Dominates (Desired, Best1, Best0)),
       Post =>
         Camera_Covered_Upto
           (Camera, FI_Hi, Slant, Desired, Found1, Best1);
   --  Monotonicity lemma for Camera_Covered_Upto: coverage carries over
   --  to a dominating later search state (P7 support).
   --  @param Camera The camera configuration supplying the candidates
   --  @param FI_Hi The covered prefix bound
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found0 The earlier search state's flag
   --  @param Best0 The earlier search state's best GSD
   --  @param Found1 The later search state's flag
   --  @param Best1 The later search state's best GSD

   procedure Lemma_Position_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Position            : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Position
             <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList),
       Post =>
         Position_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, Position, 0,
            Slant, Desired, Found, Best);
   --  Introduction lemma for the empty Position_Covered_Upto prefix: a
   --  covered camera range with no indices holds trivially, but the
   --  predicate is hidden, so even the trivial base case must be
   --  introduced where the definition is disclosed (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Position Position in the gimbal's contained-payload list
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Position_Covered_Extend
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Position            : Positive;
      CJ_Hi               : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Position
             <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList)
           and then CJ_Hi <= Last (Entity_Cfg.Cameras)
           and then Position_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
                       Position, CJ_Hi - 1, Slant, Desired, Found, Best)
           and then
             (if Get (Entity_Cfg.Cameras, CJ_Hi).PayloadID
                   = Get (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                            .ContainedPayloadList,
                          Position)
                and then Wavelength_Eligible
                           (Get (Entity_Cfg.Cameras, CJ_Hi),
                            Eligible_Wavelength)
              then Camera_Covered
                     (Get (Entity_Cfg.Cameras, CJ_Hi), Slant, Desired,
                      Found, Best)),
       Post =>
         Position_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, Position,
            CJ_Hi, Slant, Desired, Found, Best);
   --  Extension lemma for Position_Covered_Upto: the CJ_Hi-th camera's
   --  obligation (vacuous when it does not match the position or fails
   --  the wavelength filter) extends the covered prefix by one (P7
   --  support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Position Position in the gimbal's contained-payload list
   --  @param CJ_Hi The new prefix bound
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Position_Covered_Monotone
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Position            : Positive;
      CJ_Hi               : Natural;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found0              : Boolean;
      Best0               : Achieved_GSD_M;
      Found1              : Boolean;
      Best1               : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Position
             <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList)
           and then CJ_Hi <= Last (Entity_Cfg.Cameras)
           and then Position_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
                       Position, CJ_Hi, Slant, Desired, Found0, Best0)
           and then (if Found0
                     then Found1
                       and then Dominates (Desired, Best1, Best0)),
       Post =>
         Position_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, Position,
            CJ_Hi, Slant, Desired, Found1, Best1);
   --  Monotonicity lemma for Position_Covered_Upto: coverage carries
   --  over to a dominating later search state (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Position Position in the gimbal's contained-payload list
   --  @param CJ_Hi The covered prefix bound
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found0 The earlier search state's flag
   --  @param Best0 The earlier search state's best GSD
   --  @param Found1 The later search state's flag
   --  @param Best1 The later search state's best GSD

   procedure Lemma_Positions_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  => Gimbal_Index <= Last (Entity_Cfg.Gimbals),
       Post =>
         Positions_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, 0, Slant,
            Desired, Found, Best);
   --  Introduction lemma for the empty Positions_Covered_Upto prefix: a
   --  covered position range with no indices holds trivially, but the
   --  predicate is hidden, so even the trivial base case must be
   --  introduced where the definition is disclosed (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Positions_Covered_Extend
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      P_Hi                : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then P_Hi
             <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList)
           and then Positions_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
                       P_Hi - 1, Slant, Desired, Found, Best)
           and then Position_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
                       P_Hi, Last (Entity_Cfg.Cameras), Slant, Desired,
                       Found, Best),
       Post =>
         Positions_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, P_Hi, Slant,
            Desired, Found, Best);
   --  Extension lemma for Positions_Covered_Upto: coverage of position
   --  P_Hi over the full camera list extends the covered prefix by one
   --  (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param P_Hi The new prefix bound
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Positions_Covered_Monotone
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      P_Hi                : Natural;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found0              : Boolean;
      Best0               : Achieved_GSD_M;
      Found1              : Boolean;
      Best1               : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then P_Hi
             <= Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                        .ContainedPayloadList)
           and then Positions_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
                       P_Hi, Slant, Desired, Found0, Best0)
           and then (if Found0
                     then Found1
                       and then Dominates (Desired, Best1, Best0)),
       Post =>
         Positions_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, P_Hi, Slant,
            Desired, Found1, Best1);
   --  Monotonicity lemma for Positions_Covered_Upto: coverage carries
   --  over to a dominating later search state (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param P_Hi The covered prefix bound
   --  @param Slant The slant range the candidates are evaluated at
   --  @param Desired The desired ground sample distance
   --  @param Found0 The earlier search state's flag
   --  @param Best0 The earlier search state's best GSD
   --  @param Found1 The later search state's flag
   --  @param Best1 The later search state's best GSD

   procedure Lemma_Steps_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Sweep_Of
                      (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                       Elev_Wire).Valid,
       Post =>
         Steps_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
            Gimbal_Index, 0, Desired, Found, Best);
   --  Introduction lemma for the empty Steps_Covered_Upto prefix: a
   --  covered step range with no indices holds trivially, but the
   --  predicate is hidden, so even the trivial base case must be
   --  introduced where the definition is disclosed (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Steps_Covered_Extend
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      S_Hi                : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Sweep_Of
                      (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                       Elev_Wire).Valid
           and then S_Hi <= Sweep_Step_Count
                              (Sweep_Of
                                 (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                  Elev_Wire))
           and then Steps_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, Gimbal_Index, S_Hi - 1, Desired, Found,
                       Best)
           and then Positions_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
                       Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                               .ContainedPayloadList),
                       Slant_Range
                         (Altitude,
                          Sweep_Elevation
                            (Sweep_Of
                               (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                Elev_Wire),
                             S_Hi - 1)),
                       Desired, Found, Best),
       Post =>
         Steps_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
            Gimbal_Index, S_Hi, Desired, Found, Best);
   --  Extension lemma for Steps_Covered_Upto: coverage of step S_Hi - 1
   --  over the gimbal's full contained-payload list, at that step's
   --  slant range, extends the covered prefix by one (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param S_Hi The new number of covered sweep steps
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Steps_Covered_Monotone
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      S_Hi                : Natural;
      Desired             : Desired_GSD_M;
      Found0              : Boolean;
      Best0               : Achieved_GSD_M;
      Found1              : Boolean;
      Best1               : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Sweep_Of
                      (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                       Elev_Wire).Valid
           and then S_Hi <= Sweep_Step_Count
                              (Sweep_Of
                                 (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                  Elev_Wire))
           and then Steps_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, Gimbal_Index, S_Hi, Desired, Found0,
                       Best0)
           and then (if Found0
                     then Found1
                       and then Dominates (Desired, Best1, Best0)),
       Post =>
         Steps_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
            Gimbal_Index, S_Hi, Desired, Found1, Best1);
   --  Monotonicity lemma for Steps_Covered_Upto: coverage carries over
   --  to a dominating later search state (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param S_Hi The number of covered sweep steps
   --  @param Desired The desired ground sample distance
   --  @param Found0 The earlier search state's flag
   --  @param Best0 The earlier search state's best GSD
   --  @param Found1 The later search state's flag
   --  @param Best1 The later search state's best GSD

   procedure Lemma_Gimbal_Covered_Intro
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then
             (if Sweep_Of
                   (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                    Elev_Wire).Valid
              then Steps_Covered_Upto
                     (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                      Altitude, Gimbal_Index,
                      Sweep_Step_Count
                        (Sweep_Of
                           (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                            Elev_Wire)),
                      Desired, Found, Best)),
       Post =>
         Gimbal_Covered
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
            Gimbal_Index, Desired, Found, Best);
   --  Introduction lemma for Gimbal_Covered, in two cases: a valid
   --  sweep covered at its full step count, or an invalid sweep, which
   --  contributes no candidates and is covered vacuously (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Gimbal_Covered_Monotone
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      Desired             : Desired_GSD_M;
      Found0              : Boolean;
      Best0               : Achieved_GSD_M;
      Found1              : Boolean;
      Best1               : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Gimbal_Covered
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, Gimbal_Index, Desired, Found0, Best0)
           and then (if Found0
                     then Found1
                       and then Dominates (Desired, Best1, Best0)),
       Post =>
         Gimbal_Covered
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
            Gimbal_Index, Desired, Found1, Best1);
   --  Monotonicity lemma for Gimbal_Covered: coverage carries over to a
   --  dominating later search state (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Gimbal_Index Index of the gimbal in Entity_Cfg.Gimbals
   --  @param Desired The desired ground sample distance
   --  @param Found0 The earlier search state's flag
   --  @param Best0 The earlier search state's best GSD
   --  @param Found1 The later search state's flag
   --  @param Best1 The later search state's best GSD

   procedure Lemma_Gimbals_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Post =>
         Gimbals_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, 0,
            Desired, Found, Best);
   --  Introduction lemma for the empty Gimbals_Covered_Upto prefix: a
   --  covered gimbal range with no indices holds trivially, but the
   --  predicate is hidden, so even the trivial base case must be
   --  introduced where the definition is disclosed (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Gimbals_Covered_Extend
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      K_Hi                : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         K_Hi <= Last (Entity_Cfg.Gimbals)
           and then Gimbals_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, K_Hi - 1, Desired, Found, Best)
           and then Gimbal_Covered
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, K_Hi, Desired, Found, Best),
       Post =>
         Gimbals_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, K_Hi,
            Desired, Found, Best);
   --  Extension lemma for Gimbals_Covered_Upto: coverage of gimbal K_Hi
   --  extends the covered prefix by one (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param K_Hi The new prefix bound
   --  @param Desired The desired ground sample distance
   --  @param Found Whether any candidate has been accepted yet
   --  @param Best The best achieved GSD so far

   procedure Lemma_Gimbals_Covered_Monotone
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      K_Hi                : Natural;
      Desired             : Desired_GSD_M;
      Found0              : Boolean;
      Best0               : Achieved_GSD_M;
      Found1              : Boolean;
      Best1               : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         K_Hi <= Last (Entity_Cfg.Gimbals)
           and then Gimbals_Covered_Upto
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, K_Hi, Desired, Found0, Best0)
           and then (if Found0
                     then Found1
                       and then Dominates (Desired, Best1, Best0)),
       Post =>
         Gimbals_Covered_Upto
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, K_Hi,
            Desired, Found1, Best1);
   --  Monotonicity lemma for Gimbals_Covered_Upto: coverage carries
   --  over to a dominating later search state. Called at the full
   --  gimbal list, this also rewrites Candidates_Covered with
   --  Found = False to the literal No_Candidate form (P7 support).
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Altitude The effective assigned altitude, in meters
   --  @param K_Hi The covered prefix bound
   --  @param Desired The desired ground sample distance
   --  @param Found0 The earlier search state's flag
   --  @param Best0 The earlier search state's best GSD
   --  @param Found1 The later search state's flag
   --  @param Best1 The later search state's best GSD

   procedure Lemma_Footprint_GSD_Optimal_Intro
     (FP                  : SensorFootprint_Msg;
      Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired             : Desired_GSD_M;
      Altitude            : Assigned_Altitude_M;
      Elev_Wire           : Real32;
      Gimbal_Index        : Positive;
      Step                : Natural;
      Camera_Index        : Positive;
      FOV_Index           : Positive;
      Best                : Achieved_GSD_M)
     with
       Ghost,
       Always_Terminates,
       Global => null,
       Pre  =>
         Gimbal_Index <= Last (Entity_Cfg.Gimbals)
           and then Camera_Index <= Last (Entity_Cfg.Cameras)
           and then Sweep_Of
                      (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                       Elev_Wire).Valid
           and then Step < Sweep_Step_Count
                             (Sweep_Of
                                (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                 Elev_Wire))
           and then FOV_Candidate_Valid
                      (Get (Entity_Cfg.Cameras, Camera_Index), FOV_Index)
           and then Is_Candidate
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Gimbal_Index, Step, Camera_Index, FOV_Index)
           and then Selection_Witness
                      (FP, Entity_Cfg, Elev_Wire, Altitude, Gimbal_Index,
                       Step, Camera_Index, FOV_Index, Best)
           and then Candidates_Covered
                      (Entity_Cfg, Eligible_Wavelength, Elev_Wire,
                       Altitude, Desired, True, Best),
       Post =>
         Footprint_GSD_Optimal
           (FP, Entity_Cfg, Eligible_Wavelength, Desired, Altitude,
            Elev_Wire);
   --  Introduction lemma for Footprint_GSD_Optimal: the selected
   --  candidate's indices are the existential witnesses, and Best is
   --  the search's tracked best GSD, equal to the witnessing tuple's
   --  own GSD inside Selection_Witness — the lemma derives that
   --  equality internally, where the witness definition is disclosed,
   --  because callers hold coverage in terms of Best and cannot cross
   --  the hidden equality themselves. The explicit range and validity
   --  conjuncts restate consequences of Is_Candidate that must appear
   --  separately because Is_Candidate is hidden and could not
   --  otherwise guard the partial calls that follow (P7 support).
   --  @param FP The footprint whose selection is claimed optimal
   --  @param Entity_Cfg The entity configuration owning the payloads
   --  @param Eligible_Wavelength The requested eligible wavelength band
   --  @param Desired The desired ground sample distance
   --  @param Altitude The effective assigned altitude, in meters
   --  @param Elev_Wire The requested elevation override, in degrees
   --  @param Gimbal_Index Index of the witnessing gimbal
   --  @param Step The witnessing sweep step
   --  @param Camera_Index Index of the witnessing camera
   --  @param FOV_Index The witnessing FOV candidate index
   --  @param Best The best achieved GSD tracked by the search

end Sensor_Manager_Types;
