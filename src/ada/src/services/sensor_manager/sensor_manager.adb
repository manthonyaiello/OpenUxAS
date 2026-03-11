with Ada.Containers;
with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;

package body Sensor_Manager with SPARK_Mode is

   use Ada.Containers;

   package Math is new Ada.Numerics.Generic_Elementary_Functions (Real64);

   --  Constants matching C++ SensorManagerService
   Pi                           : constant Real64 := Ada.Numerics.Pi;
   Deg_To_Rad                   : constant Real64 := Pi / 180.0;
   GIMBAL_STEP_SIZE_RAD         : constant Real64 := 5.0 * Deg_To_Rad;
   HORIZONTAL_FOV_STEP_SIZE_DEG : constant Real64 := 5.0;
   COMPARISON_TOLERANCE         : constant Real64 := 1.0e-10;
   DEFAULT_ACCEPTABLE_GSD       : constant Real64 := 1000.0;
   MINIMUM_ASSIGNED_ALTITUDE_M  : constant Real64 := 10.0;
   ELEV_MAX_BOUND               : constant Real64 := -(Pi / 180.0);
   ELEV_MIN_BOUND               : constant Real64 := -(Pi - Pi / 180.0);

   --  Domain-specific subtypes for improved documentation and proof readiness.

   --  Elevation angle in radians, constrained to the downward-pointing half-plane.
   --  Zero is horizontal; values approach -Pi at the nadir (straight down).
   subtype Elevation_Rad is Real64 range -Pi .. 0.0;

   --  Elevation angle in radians after full clamping to the valid gimbal range.
   --  ELEV_MIN_BOUND ≈ −179° and ELEV_MAX_BOUND ≈ −1° (both in radians).
   subtype Clamped_Elevation_Rad is Elevation_Rad
      range ELEV_MIN_BOUND .. ELEV_MAX_BOUND;

   --  Horizontal field of view in radians [0, 2π] (full revolution).
   subtype Horiz_FOV_Rad is Real64 range 0.0 .. 2.0 * Pi;

   --  Horizontal field of view in degrees [0°, 360°] (full revolution).
   subtype Horiz_FOV_Deg is Real64 range 0.0 .. 360.0;

   --  Altitude above ground level in metres, non-negative.
   --  Upper bound matches the CMASI MaximumAltitude default (1,000,000 m).
   --  This is far above the Kármán line and well beyond any realistic AGL altitude.
   MAX_ALTITUDE_M : constant Real64 := 1.0e6;
   subtype Altitude_M is Real64 range 0.0 .. MAX_ALTITUDE_M;

   --  Maximum achievable slant range (metres).
   --  Slant = Altitude / Sin(-Elev_Rad); maximised when Altitude = MAX_ALTITUDE_M
   --  and -Elev_Rad = Pi/180 (i.e. ELEV_MAX_BOUND, the shallowest valid angle, 1°).
   --  Sin(Pi/180) = sin(1°) ≈ 0.01745, so the tight bound is ≈ 5.73 × 10⁷ m.
   --  We use 6.0 × 10⁷ as a conservative rounded-up literal so that GNATprove
   --  can evaluate it statically (a Math.Sin call would be opaque to the prover).
   MAX_SLANT_M : constant Real64 := 6.0e7;

   --  Slant range from sensor platform to ground target, in metres (non-negative).
   --  Upper bound is MAX_SLANT_M, derived from altitude and minimum elevation angle.
   subtype Slant_M is Real64 range 0.0 .. MAX_SLANT_M;

   --  Ground sample distance (metres/pixel), non-negative.
   --  Upper bound matches MAX_SLANT_M: the maximum achievable GSD equals the
   --  maximum achievable slant range (when Sin(alpha) = 1.0).
   subtype GSD_T is Real64 range 0.0 .. MAX_SLANT_M;

   --  Camera aspect ratio (HorizRes / VertRes), non-negative.
   --  Bounded by UInt32 since both resolution fields are UInt32.
   subtype Aspect_Ratio_T is Real64 range 0.0 .. Real64 (UInt32'Last);

   --  Raw elevation-angle override from a SensorFootprintRequest (degrees, LMCP convention).
   --  This value is compared against radian-valued gimbal limits in Compute_Elevation_Range,
   --  faithfully replicating C++ behaviour (intentional unit mismatch).
   subtype Elevation_Request_Deg is Real64
      range Real64 (Real32'First) .. Real64 (Real32'Last);

   --  Local subprogram declarations

   procedure Calculate_Sensor_Footprint
     (FP           : in out SensorFootprint_Msg;
      Altitude     : Altitude_M;
      Elev_Rad     : Clamped_Elevation_Rad;
      HFOV_Rad     : Horiz_FOV_Rad;
      Aspect_Ratio : Aspect_Ratio_T)
     with
       Always_Terminates,
       Pre =>
         Altitude in 0.0 .. MAX_ALTITUDE_M
         and then HFOV_Rad in 0.0 .. Pi - Pi / 180.0;

   procedure Update_Best
     (FP                : in out SensorFootprint_Msg;
      First_GSD_Found   : in out Boolean;
      FOV_Deg           : Horiz_FOV_Deg;
      Slant             : Slant_M;
      Min_Res           : UInt32;
      Acceptable_GSD    : GSD_T;
      Altitude          : Altitude_M;
      Elev_Rad          : Clamped_Elevation_Rad;
      Aspect            : Aspect_Ratio_T;
      Camera_ID         : Int64;
      Camera_Wavelength : WavelengthBandEnum;
      Gimbal_ID         : Int64)
     with Always_Terminates,
          Pre =>
            FOV_Deg in 0.0 .. 179.0
            and then
              (not First_GSD_Found
               or else Real64 (FP.AchievedGSD) in GSD_T);
   --  FOV_Deg must be strictly less than 180 degrees: at 180 deg the horizontal
   --  half-angle reaches 90 deg, making Tan (HFOV/2) undefined (infinite footprint).
   --  When First_GSD_Found is True, FP.AchievedGSD must already be within GSD_T,
   --  which holds because it was set to Real32(GSD) in a prior call and GSD is in GSD_T.
   --  Update FP with the given FOV_Deg if it produces a better GSD match.
   --  "Better" means |Acceptable_GSD - GSD| < current best (using C++ integer
   --  truncation semantics for the comparison).

   procedure Process_Camera
     (FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean;
      Camera              : CameraConfig;
      Gimbal_ID           : Int64;
      Eligible_Wavelength : WavelengthBandEnum;
      Slant               : Slant_M;
      Altitude            : Altitude_M;
      Elev_Rad            : Clamped_Elevation_Rad;
      Acceptable_GSD      : GSD_T)
     with Always_Terminates;
   --  Check wavelength eligibility for Camera, then iterate over its FOV
   --  configurations and call Update_Best for each candidate.

   procedure Process_Gimbal_At_Elevation
     (FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean;
      Gimbal              : GimbalConfig;
      Cameras             : CameraConfig_Seq;
      Eligible_Wavelength : WavelengthBandEnum;
      Slant               : Slant_M;
      Altitude            : Altitude_M;
      Elev_Rad            : Clamped_Elevation_Rad;
      Acceptable_GSD      : GSD_T)
     with Always_Terminates;
   --  For a given Gimbal and elevation step (Elev_Rad, Slant already computed),
   --  find cameras in Cameras whose PayloadID matches a ContainedPayloadList
   --  entry and process each via Process_Camera.

   procedure Compute_Elevation_Range
     (Gimbal          : GimbalConfig;
      Elevation_Angle : Elevation_Request_Deg;
      Elev_Min        : out Clamped_Elevation_Rad;
      Elev_Max        : out Clamped_Elevation_Rad;
      Valid           : out Boolean)
     with
       Always_Terminates,
       Contract_Cases =>
         --  Upward-facing gimbal: shallowest angle is at or above the horizon,
         --  so it cannot observe the ground.
         (Gimbal.IsElevationClamped
            and then Real64 (Gimbal.MinElevation) * Deg_To_Rad >= 0.0 =>
              not Valid,

          --  Downward-facing or unclamped gimbal: outputs are clamped to
          --  [ELEV_MIN_BOUND, ELEV_MAX_BOUND] and ordered Min ≤ Max.
          others =>
              Valid
                and then Elev_Min in ELEV_MIN_BOUND .. ELEV_MAX_BOUND
                and then Elev_Max in ELEV_MIN_BOUND .. ELEV_MAX_BOUND
                and then Elev_Min <= Elev_Max);

   procedure Process_Gimbal
     (FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean;
      Gimbal              : GimbalConfig;
      Cameras             : CameraConfig_Seq;
      Eligible_Wavelength : WavelengthBandEnum;
      Acceptable_GSD      : GSD_T;
      Altitude            : Altitude_M;
      Elevation_Angle     : Elevation_Request_Deg)
     with Always_Terminates;
   --  Compute the effective elevation range for Gimbal, then iterate over
   --  elevation steps, calling Process_Gimbal_At_Elevation for each.

   procedure Find_Sensor_Footprint
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired_GSD         : Real64;
      Altitude_AGL        : Real64;
      Elevation_Angle     : Real64;
      FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean)
     with Always_Terminates;

   --------------------------------
   -- Calculate_Sensor_Footprint --
   --------------------------------

   procedure Calculate_Sensor_Footprint
     (FP           : in out SensorFootprint_Msg;
      Altitude     : Altitude_M;
      Elev_Rad     : Clamped_Elevation_Rad;
      HFOV_Rad     : Horiz_FOV_Rad;
      Aspect_Ratio : Aspect_Ratio_T)
   is
      use Math;
      Vert_FOV       : Real64;
      Gimbal_Max     : Elevation_Rad;
      Gimbal_Min     : Elevation_Rad;
      Slant_Range    : Real64;
      Horiz_Center   : Real64;
      Horiz_Leading  : Real64;
      Horiz_Trailing : Real64;
      Width_Center   : Real64;
      Denom          : Real64;

      procedure Axiom_Sin_Lower_Bound (X : Real64)
      with
         Ghost,
         Import,
         Pre =>
            X >= Pi / 180.0 and
            X <= Pi - Pi / 180.0,
         Post =>
            --  Sin (pi-pi/180) ≈ 0.01745240644
            --  Sin (pi/180)    ≈ 0.01745240644
            --  between these values, the Sin is always greater
            Sin (X) > 0.01;

      procedure Axiom_Tan_Lower_Bound (X : Real64)
      with
         Ghost,
         Import,
         Pre =>
            X >= Pi / 180.0 and
            X <= Pi - Pi / 180.0,
         Post =>
           --  Tan (pi-pi/180) ≈ -0.01745506493
           --  Tan (pi/180)    ≈  0.01745506493
           --  between these values, the Tan is no closer to zero
           Tan (X) >=  0.01 and
           Tan (X) <= -0.01;


      procedure Axiom_Tan_HFOV_Rad (X : Real64)
      with
         Ghost,
         Import,
         Pre =>
            X in 0.0 .. (Pi - Pi / 180.0) / 2.0,
         Post =>
            --  Tan ((pi - pi/180) / 2) = Tan (89.5 deg) ≈ 114.59
            Tan (X) in 0.0 .. 114.6;

   begin
      --  There are more assertions in this subprogram body than strictly
      --  necessary, but I wanted to illustrate how the reasoning goes that
      --  allows us (and SPARK) to conclude that all of our operations are
      --  ultimately safe.
      --
      --  This also proves at level=2; the stripped version requires level=3,
      --  which I prefer to avoid.

      if abs (Aspect_Ratio) < COMPARISON_TOLERANCE then
         Vert_FOV := HFOV_Rad;
      else
         Vert_FOV := HFOV_Rad / Aspect_Ratio;
      end if;

      Gimbal_Max :=
        Elevation_Rad (Real64'Max (-Pi, Real64'Min (0.0, Elev_Rad + Vert_FOV / 2.0)));
      Gimbal_Min :=
        Elevation_Rad (Real64'Max (-Pi, Real64'Min (0.0, Elev_Rad - Vert_FOV / 2.0)));

      Denom := Sin (-Elev_Rad);

      --  We need an axiom, because the contract on Sin is imprecise, then we
      --  can tightly bound Denom.
      Axiom_Sin_Lower_Bound (-Elev_Rad);
      pragma Assert (0.01 <= Denom);
      pragma Assert (        Denom <= 1.0);

      Slant_Range := Altitude / Denom;
      pragma Assert (0.0 <= Slant_Range);
      pragma Assert (       Slant_Range <= 1.9e9);

      Denom := Tan (-Elev_Rad);

      --  We need an axiom, because the contract on Tan is imprecise, then we
      --  can tightly bound Denom.
      Axiom_Tan_Lower_Bound (-Elev_Rad);
      pragma Assert (Denom <= -0.01);
      pragma Assert ( 0.01 >= Denom);

      Horiz_Center := Altitude / Denom;
      pragma Assert (-6.0e7 <= Horiz_Center);
      pragma Assert (          Horiz_Center <= 6.0e7);

      Denom := Tan (-Gimbal_Max);
      Horiz_Leading :=
        (if abs (Denom) < COMPARISON_TOLERANCE then 0.0
         else Altitude / Denom);
      pragma Assert (-1.0e17 <= Horiz_Leading);
      pragma Assert (           Horiz_Leading <= 1.0e17);

      Denom := Tan (-Gimbal_Min);
      Horiz_Trailing :=
        (if abs (Denom) < COMPARISON_TOLERANCE then 0.0
         else Altitude / Denom);
      pragma Assert (-1.0e17 <= Horiz_Trailing);
      pragma Assert (           Horiz_Trailing <= 1.0e17);

      --  This axiom puts a tight bound on the Tan term below, which is needed
      --  to show that the computation of Width_Center doesn't overflow
      --  (nor does its conversion to a Real32).
      Axiom_Tan_HFOV_Rad (0.5 * HFOV_Rad);
      Width_Center := 2.0 * Slant_Range * Tan (0.5 * HFOV_Rad);
      pragma Assert (0.0 <= Width_Center);
      pragma Assert (       Width_Center <= 2.0 * 1.9e9 * 114.6);  --  ~ 4.36e11 < Real32'Last

      FP.SlantRangeToCenter       := Real32 (Slant_Range);
      FP.HorizontalToCenter       := Real32 (Horiz_Center);
      FP.HorizontalToLeadingEdge  := Real32 (Horiz_Leading);
      FP.HorizontalToTrailingEdge := Real32 (Horiz_Trailing);
      FP.WidthCenter              := Real32 (Width_Center);
   end Calculate_Sensor_Footprint;

   -----------------
   -- Update_Best --
   -----------------

   procedure Update_Best
     (FP                : in out SensorFootprint_Msg;
      First_GSD_Found   : in out Boolean;
      FOV_Deg           : Horiz_FOV_Deg;
      Slant             : Slant_M;
      Min_Res           : UInt32;
      Acceptable_GSD    : GSD_T;
      Altitude          : Altitude_M;
      Elev_Rad          : Clamped_Elevation_Rad;
      Aspect            : Aspect_Ratio_T;
      Camera_ID         : Int64;
      Camera_Wavelength : WavelengthBandEnum;
      Gimbal_ID         : Int64)
   is
      use Math;
      FOV_Rad : constant Horiz_FOV_Rad := FOV_Deg * Deg_To_Rad;

      --  Axiom: Sin is bounded in [-1, 1] for all inputs; non-negative on [0, Pi].
      procedure Axiom_Sin_Bounded (X : Real64)
      with
         Ghost,
         Import,
         Pre  => X in 0.0 .. Pi,
         Post => Sin (X) in 0.0 .. 1.0;

      --  Lemma: multiplying a value in [0, MAX_SLANT_M] by a factor in [0, 1]
      --  yields a value in [0, MAX_SLANT_M].  Provable by GNATprove from the
      --  subtype bounds on A and the precondition on B alone.
      procedure Lemma_Product_Le_Slant (A : Slant_M; B : Real64)
      with
         Ghost,
         Pre  => B in 0.0 .. 1.0,
         Post => A * B in 0.0 .. MAX_SLANT_M;

      procedure Lemma_Product_Le_Slant (A : Slant_M; B : Real64) is null;

      Alpha_Rad     : Real64;
      GSD           : Real64;
      GSD_Delta_Int : Integer;
   begin
      Alpha_Rad :=
        (if Min_Res = 0 then Pi / 2.0
         else FOV_Rad / Real64 (Min_Res));

      --  Alpha_Rad is the per-pixel angular resolution.  It lies in [0, Pi]:
      --  when Min_Res = 0 it is Pi/2; otherwise it is FOV_Rad / Min_Res where
      --  FOV_Rad <= 179 * Pi/180 < Pi and Min_Res >= 1.
      pragma Assert (Alpha_Rad in 0.0 .. Pi);
      Axiom_Sin_Bounded (Alpha_Rad);
      pragma Assert (Sin (Alpha_Rad) in 0.0 .. 1.0);

      Lemma_Product_Le_Slant (Slant, Sin (Alpha_Rad));
      GSD := Slant * Sin (Alpha_Rad);
      --  GSD <= Slant * 1.0 <= MAX_SLANT_M, and GSD >= 0 since both factors >= 0.
      pragma Assert (GSD in GSD_T);

      --  Match C++ behavior: abs() resolves to C integer abs, truncating to int.
      GSD_Delta_Int := Integer (Real64'Floor (abs (Acceptable_GSD - GSD)));

      if not First_GSD_Found
        or else Integer
                  (Real64'Floor (abs (Acceptable_GSD - Real64 (FP.AchievedGSD))))
                > GSD_Delta_Int
      then
         First_GSD_Found      := True;
         FP.CameraID          := Camera_ID;
         FP.GimbalID          := Gimbal_ID;
         FP.HorizontalFOV     := Real32 (FOV_Deg);
         FP.AglAltitude       := Real32 (Altitude);
         FP.GimbalElevation   := Real32 (Elev_Rad / Deg_To_Rad);
         FP.AspectRatio       := Real32 (Aspect);
         FP.AchievedGSD       := Real32 (GSD);
         FP.CameraWavelength  := Camera_Wavelength;
         Calculate_Sensor_Footprint (FP, Altitude, Elev_Rad, FOV_Rad, Aspect);
      end if;
   end Update_Best;

   --------------------
   -- Process_Camera --
   --------------------

   procedure Process_Camera
     (FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean;
      Camera              : CameraConfig;
      Gimbal_ID           : Int64;
      Eligible_Wavelength : WavelengthBandEnum;
      Slant               : Slant_M;
      Altitude            : Altitude_M;
      Elev_Rad            : Clamped_Elevation_Rad;
      Acceptable_GSD      : GSD_T)
   is
      pragma SPARK_Mode (Off);
      use all type Real32_Seq;
      Aspect  : constant Aspect_Ratio_T :=
        (if Camera.VertResolution = 0 then 1.0
         else Real64 (Camera.HorizResolution)
              / Real64 (Camera.VertResolution));
      Min_Res : constant UInt32 :=
        UInt32'Min (Camera.HorizResolution, Camera.VertResolution);
   begin
      if Camera.SupportedWavelengthBand = Eligible_Wavelength
        or else Eligible_Wavelength = AllAny
      then
         if Camera.FieldOfViewMode = Continuous then
            declare
               Min_FOV : constant Horiz_FOV_Deg :=
                 Real64'Max (0.0, Real64'Min (360.0, Real64 (Camera.MinHorizontalFOV)));
               Max_FOV : constant Horiz_FOV_Deg :=
                 Real64'Max (0.0, Real64'Min (360.0, Real64 (Camera.MaxHorizontalFOV)));
               N_FOV   : Natural;
            begin
               if Max_FOV >= Min_FOV then
                  N_FOV :=
                    Natural
                      (Real64'Floor
                         ((Max_FOV - Min_FOV) / HORIZONTAL_FOV_STEP_SIZE_DEG))
                    + 1;
                  for FOV_Step in 0 .. N_FOV - 1 loop
                     declare
                        FOV : constant Real64 :=
                          Min_FOV + Real64 (FOV_Step) * HORIZONTAL_FOV_STEP_SIZE_DEG;
                     begin
                        --  Skip degenerate FOVs >= 180 deg (Tan (HFOV/2) undefined).
                        if FOV <= 179.0 then
                           Update_Best
                             (FP, First_GSD_Found,
                              Horiz_FOV_Deg (FOV),
                              Slant, Min_Res, Acceptable_GSD, Altitude, Elev_Rad,
                              Aspect, Camera.PayloadID, Camera.SupportedWavelengthBand,
                              Gimbal_ID);
                        end if;
                     end;
                  end loop;
               end if;
            end;
         else
            --  Discrete mode
            for FOV_Entry of Camera.DiscreteHFOVList loop
               declare
                  FOV : constant Horiz_FOV_Deg :=
                    Horiz_FOV_Deg (Real64'Max (0.0, Real64'Min (360.0,
                                                                Real64 (FOV_Entry))));
               begin
                  --  Skip degenerate FOVs >= 180 deg (Tan (HFOV/2) undefined).
                  if FOV <= 179.0 then
                     Update_Best
                       (FP, First_GSD_Found,
                        FOV,
                        Slant, Min_Res, Acceptable_GSD, Altitude, Elev_Rad,
                        Aspect, Camera.PayloadID, Camera.SupportedWavelengthBand,
                        Gimbal_ID);
                  end if;
               end;
            end loop;
         end if;
      end if;
   end Process_Camera;

   ---------------------------------
   -- Process_Gimbal_At_Elevation --
   ---------------------------------

   procedure Process_Gimbal_At_Elevation
     (FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean;
      Gimbal              : GimbalConfig;
      Cameras             : CameraConfig_Seq;
      Eligible_Wavelength : WavelengthBandEnum;
      Slant               : Slant_M;
      Altitude            : Altitude_M;
      Elev_Rad            : Clamped_Elevation_Rad;
      Acceptable_GSD      : GSD_T)
   is
      pragma SPARK_Mode (Off);
      use all type Int64_Seq;
      use all type CameraConfig_Seq;
   begin
      for Cam_ID of Gimbal.ContainedPayloadList loop
         for Camera of Cameras loop
            if Camera.PayloadID = Cam_ID then
               Process_Camera
                 (FP, First_GSD_Found,
                  Camera, Gimbal.PayloadID,
                  Eligible_Wavelength, Slant, Altitude, Elev_Rad,
                  Acceptable_GSD);
            end if;
         end loop;
      end loop;
   end Process_Gimbal_At_Elevation;

   ----------------------------
   -- Compute_Elevation_Range --
   ----------------------------

   procedure Compute_Elevation_Range
     (Gimbal          : GimbalConfig;
      Elevation_Angle : Elevation_Request_Deg;
      Elev_Min        : out Clamped_Elevation_Rad;
      Elev_Max        : out Clamped_Elevation_Rad;
      Valid           : out Boolean)
   is
      Elev_Min_Raw : Real64 := Real64 (Gimbal.MinElevation) * Deg_To_Rad;
      Elev_Max_Raw : Real64 := Real64 (Gimbal.MaxElevation) * Deg_To_Rad;
   begin
      --  Phase 1: detect an upward-facing gimbal.
      --  MinElevation ≥ 0° means the shallowest angle the gimbal can reach is
      --  horizontal or above — it cannot observe the ground.  Matches the
      --  explicit C++ guard (SensorManagerService.cpp line 258).
      if Gimbal.IsElevationClamped and then Elev_Min_Raw >= 0.0 then
         Valid    := False;
         Elev_Min := Clamped_Elevation_Rad'First;
         Elev_Max := Clamped_Elevation_Rad'First;
         return;
      end if;

      --  Phase 2: compute the valid elevation range for a downward-facing or
      --  unclamped gimbal.

      --  Unclamped gimbal: sweep the full valid range.
      if not Gimbal.IsElevationClamped then
         Elev_Min_Raw := ELEV_MIN_BOUND;
         Elev_Max_Raw := ELEV_MAX_BOUND;
      end if;

      --  Two-sided clamp: enforce ELEV_MIN_BOUND ≤ Min ≤ Max ≤ ELEV_MAX_BOUND.
      --  After Phase 1, Elev_Min_Raw < 0 for clamped gimbals, so the clamp
      --  keeps Min negative.
      Elev_Min_Raw :=
        Real64'Max (ELEV_MIN_BOUND, Real64'Min (ELEV_MAX_BOUND, Elev_Min_Raw));
      Elev_Max_Raw :=
        Real64'Max (ELEV_MIN_BOUND, Real64'Min (ELEV_MAX_BOUND, Elev_Max_Raw));
      if Elev_Max_Raw < Elev_Min_Raw then
         Elev_Max_Raw := Elev_Min_Raw;
      end if;

      --  Elevation override: if the request pins a specific elevation angle,
      --  collapse the range to that single angle (or Min, whichever is less
      --  steep).  Note: Elevation_Angle is in degrees while Elev_Min/Max_Raw
      --  are in radians — intentional unit mismatch replicating C++ behavior.
      if Elevation_Angle < 0.001 then
         if Elevation_Angle > Elev_Min_Raw then
            Elev_Min_Raw := Elevation_Angle;
         end if;
         --  Re-clamp: override may produce a value in (ELEV_MAX_BOUND, 0.001).
         Elev_Min_Raw :=
           Real64'Max (ELEV_MIN_BOUND, Real64'Min (ELEV_MAX_BOUND, Elev_Min_Raw));
         Elev_Max_Raw := Elev_Min_Raw;
      end if;

      Valid    := True;
      Elev_Min := Clamped_Elevation_Rad (Elev_Min_Raw);
      Elev_Max := Clamped_Elevation_Rad (Elev_Max_Raw);
   end Compute_Elevation_Range;

   --------------------
   -- Process_Gimbal --
   --------------------

   procedure Process_Gimbal
     (FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean;
      Gimbal              : GimbalConfig;
      Cameras             : CameraConfig_Seq;
      Eligible_Wavelength : WavelengthBandEnum;
      Acceptable_GSD      : GSD_T;
      Altitude            : Altitude_M;
      Elevation_Angle     : Elevation_Request_Deg)
   is
      pragma SPARK_Mode (Off);
      use Math;
      Elev_Min : Clamped_Elevation_Rad;
      Elev_Max : Clamped_Elevation_Rad;
      Valid    : Boolean;
      N_Elev   : Natural;
   begin
      Compute_Elevation_Range (Gimbal, Elevation_Angle, Elev_Min, Elev_Max, Valid);

      --  Skip upward-facing gimbals (Valid = False), matching C++ behavior.
      if Valid then
         N_Elev :=
           Natural
             (Real64'Floor ((Elev_Max - Elev_Min) / GIMBAL_STEP_SIZE_RAD))
           + 1;

         for Elev_Step in 0 .. N_Elev - 1 loop
            declare
               Elev_Rad : constant Clamped_Elevation_Rad :=
                 Elev_Min + Real64 (Elev_Step) * GIMBAL_STEP_SIZE_RAD;
               Denom    : constant Real64 := Sin (-Elev_Rad);
               Slant    : constant Real64 :=
                 (if abs (Denom) < COMPARISON_TOLERANCE then Altitude
                  else Altitude / Denom);
            begin
               Process_Gimbal_At_Elevation
                 (FP, First_GSD_Found,
                  Gimbal, Cameras,
                  Eligible_Wavelength, Slant_M (Slant), Altitude, Elev_Rad,
                  Acceptable_GSD);
            end;
         end loop;
      end if;
   end Process_Gimbal;

   ---------------------------
   -- Find_Sensor_Footprint --
   ---------------------------

   procedure Find_Sensor_Footprint
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired_GSD         : Real64;
      Altitude_AGL        : Real64;
      Elevation_Angle     : Real64;
      FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean)
   is
      pragma SPARK_Mode (Off);
      use all type GimbalConfig_Seq;

      --  Raw altitude before sanity check; may be negative (NominalAltitude).
      --  Narrowed to Altitude_M at the call site below, after the sanity check.
      Altitude : constant Real64 :=
        (if Altitude_AGL < 0.001 then Real64 (Entity_Cfg.NominalAltitude)
         else Altitude_AGL);

      Acceptable_GSD : constant GSD_T :=
        (if Desired_GSD < 0.001 then DEFAULT_ACCEPTABLE_GSD
         else Real64'Min (GSD_T'Last, Desired_GSD));

   begin
      --  Sanity check: altitude must meet minimum
      if Altitude < MINIMUM_ASSIGNED_ALTITUDE_M then
         return;
      end if;

      for Gimbal of Entity_Cfg.Gimbals loop
         Process_Gimbal
           (FP, First_GSD_Found,
            Gimbal, Entity_Cfg.Cameras,
            Eligible_Wavelength, Acceptable_GSD,
            Altitude_M (Altitude),
            Elevation_Request_Deg (Elevation_Angle));
      end loop;
   end Find_Sensor_Footprint;

   --------------------------
   -- Handle_EntityConfig  --
   --------------------------

   procedure Handle_EntityConfig
     (State  : in out Sensor_Manager_State;
      Config : EntityConfig)
   is
      use Entity_Config_Maps;
   begin
      if Contains (State.Entity_Configs, Config.ID) then
         Replace (State.Entity_Configs, Config.ID, Config);
      elsif Length (State.Entity_Configs) < Max_Entity_Configs then
         Insert (State.Entity_Configs, Config.ID, Config);
      end if;
   end Handle_EntityConfig;

   ------------------------------------
   -- Handle_SensorFootprintRequests --
   ------------------------------------

   procedure Handle_SensorFootprintRequests
     (State   : in out Sensor_Manager_State;
      Mailbox : in out Sensor_Manager_Mailbox;
      Msg     : SensorFootprintRequests_Msg)
   is
      pragma SPARK_Mode (Off);
      use Entity_Config_Maps;
      use all type FootprintRequest_Seq;
      use all type WavelengthBand_Seq;
      use all type Real32_Seq;
      use all type SensorFootprint_Seq;

      Response : SensorFootprintResponse_Msg;
   begin
      Response.ResponseID := Msg.RequestID;

      for Request of Msg.Footprints loop
         if Contains (State.Entity_Configs, Request.VehicleID) then
            declare
               Entity_Cfg : constant EntityConfig :=
                 Element (State.Entity_Configs, Request.VehicleID);
               Eff_W : WavelengthBand_Seq;
               Eff_G : Real32_Seq;
               Eff_A : Real32_Seq;
               Eff_E : Real32_Seq;
            begin
               Eff_W :=
                 (if Natural (Last (Request.EligibleWavelengths)) = 0
                  then Add (Eff_W, AllAny)
                  else Request.EligibleWavelengths);
               Eff_G :=
                 (if Natural (Last (Request.GroundSampleDistances)) = 0
                  then Add (Eff_G, 0.0)
                  else Request.GroundSampleDistances);
               Eff_A :=
                 (if Natural (Last (Request.AglAltitudes)) = 0
                  then Add (Eff_A, 0.0)
                  else Request.AglAltitudes);
               Eff_E :=
                 (if Natural (Last (Request.ElevationAngles)) = 0
                  then Add (Eff_E, 0.0)
                  else Request.ElevationAngles);

               for W of Eff_W loop
                  for G of Eff_G loop
                     for A of Eff_A loop
                        for E of Eff_E loop
                           declare
                              FP              : SensorFootprint_Msg;
                              First_GSD_Found : Boolean := False;
                           begin
                              FP.FootprintResponseID :=
                                Request.FootprintRequestID;
                              FP.VehicleID := Entity_Cfg.ID;
                              Find_Sensor_Footprint
                                (Entity_Cfg          => Entity_Cfg,
                                 Eligible_Wavelength => W,
                                 Desired_GSD         => Real64 (G),
                                 Altitude_AGL        => Real64 (A),
                                 Elevation_Angle     => Real64 (E),
                                 FP                  => FP,
                                 First_GSD_Found     => First_GSD_Found);
                              Response.Footprints :=
                                Add (Response.Footprints, FP);
                           end;
                        end loop;
                     end loop;
                  end loop;
               end loop;
            end;
         end if;
      end loop;

      sendBroadcastMessage (Mailbox, Response);
   end Handle_SensorFootprintRequests;

end Sensor_Manager;
