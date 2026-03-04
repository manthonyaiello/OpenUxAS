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
   --  Upper bound matches the range of any LMCP Real32 altitude field.
   subtype Altitude_M is Real64 range 0.0 .. Real64 (Real32'Last);

   --  Slant range from sensor platform to ground target, in metres (non-negative).
   subtype Slant_M is Real64 range 0.0 .. Real64 (Real32'Last);

   --  Ground sample distance (metres/pixel), non-negative.
   subtype GSD_T is Real64 range 0.0 .. Real64 (Real32'Last);

   --  Camera aspect ratio (HorizRes / VertRes), non-negative.
   --  Bounded by UInt32 since both resolution fields are UInt32.
   subtype Aspect_Ratio_T is Real64 range 0.0 .. Real64 (UInt32'Last);

   --  Minimum camera pixel resolution (min of horizontal and vertical), non-negative.
   subtype Min_Resolution_T is Real64 range 0.0 .. Real64 (UInt32'Last);

   --  Raw elevation-angle override from a SensorFootprintRequest (degrees, LMCP convention).
   --  This value is compared against radian-valued gimbal limits in Compute_Elevation_Range,
   --  faithfully replicating C++ behaviour (intentional unit mismatch).
   subtype Elevation_Request_Deg is Real64
      range Real64 (Real32'First) .. Real64 (Real32'Last);

   --  Local subprogram declarations

   procedure Calculate_Sensor_Footprint
     (FP           : in out SensorFootprint_Msg;
      Altitude     : Altitude_M;
      Elev_Rad     : Elevation_Rad;
      HFOV_Rad     : Horiz_FOV_Rad;
      Aspect_Ratio : Aspect_Ratio_T)
     with Always_Terminates;

   procedure Update_Best
     (FP                : in out SensorFootprint_Msg;
      First_GSD_Found   : in out Boolean;
      FOV_Deg           : Horiz_FOV_Deg;
      Slant             : Slant_M;
      Min_Res           : Min_Resolution_T;
      Acceptable_GSD    : GSD_T;
      Altitude          : Altitude_M;
      Elev_Rad          : Elevation_Rad;
      Aspect            : Aspect_Ratio_T;
      Camera_ID         : Int64;
      Camera_Wavelength : WavelengthBandEnum;
      Gimbal_ID         : Int64)
     with Always_Terminates;
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
      Elev_Rad            : Elevation_Rad;
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
      Elev_Rad            : Elevation_Rad;
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
   --  Phase 1: returns Valid = False immediately if IsElevationClamped and
   --  MinElevation ≥ 0° (gimbal cannot observe the ground).
   --  Phase 2: two-sided clamps both limits into [ELEV_MIN_BOUND, ELEV_MAX_BOUND],
   --  fixes any inversion, then applies the elevation-angle override from the
   --  request (replicating C++ behavior, including its degree/radian unit mismatch).
   --  Sets Valid := False (and assigns dummy values to Elev_Min/Elev_Max)
   --  when the gimbal has no downward-facing coverage.

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
      Elev_Rad     : Elevation_Rad;
      HFOV_Rad     : Horiz_FOV_Rad;
      Aspect_Ratio : Aspect_Ratio_T)
   is
      pragma SPARK_Mode (Off);
      use Math;
      Vert_FOV       : Real64;           --  Can exceed 2π for small aspect ratios
      Gimbal_Max     : Elevation_Rad;
      Gimbal_Min     : Elevation_Rad;
      Slant_Range    : Real64;           --  May exceed Real32'Last for extreme inputs
      Horiz_Center   : Real64;
      Horiz_Leading  : Real64;
      Horiz_Trailing : Real64;
      Width_Center   : Real64;           --  Unbounded near FOV = π
      Denom          : Real64;
   begin
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
      Slant_Range :=
        (if abs (Denom) < COMPARISON_TOLERANCE then 0.0
         else Altitude / Denom);

      Denom := Tan (-Elev_Rad);
      Horiz_Center :=
        (if abs (Denom) < COMPARISON_TOLERANCE then 0.0
         else Altitude / Denom);

      Denom := Tan (-Gimbal_Max);
      Horiz_Leading :=
        (if abs (Denom) < COMPARISON_TOLERANCE then 0.0
         else Altitude / Denom);

      Denom := Tan (-Gimbal_Min);
      Horiz_Trailing :=
        (if abs (Denom) < COMPARISON_TOLERANCE then 0.0
         else Altitude / Denom);

      Width_Center := 2.0 * Slant_Range * Tan (0.5 * HFOV_Rad);

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
      Min_Res           : Min_Resolution_T;
      Acceptable_GSD    : GSD_T;
      Altitude          : Altitude_M;
      Elev_Rad          : Elevation_Rad;
      Aspect            : Aspect_Ratio_T;
      Camera_ID         : Int64;
      Camera_Wavelength : WavelengthBandEnum;
      Gimbal_ID         : Int64)
   is
      pragma SPARK_Mode (Off);
      use Math;
      FOV_Rad       : constant Horiz_FOV_Rad := FOV_Deg * Deg_To_Rad;
      Alpha_Rad     : constant Real64 :=
        (if Min_Res <= 0.0 then Pi / 2.0
         else FOV_Rad / Min_Res);
      GSD           : constant Real64 := Slant * Sin (Alpha_Rad);
      --  Match C++ behavior: abs() resolves to C integer abs, truncating to int
      GSD_Delta_Int : constant Integer :=
        Integer (Real64'Floor (abs (Acceptable_GSD - GSD)));
   begin
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
      Elev_Rad            : Elevation_Rad;
      Acceptable_GSD      : GSD_T)
   is
      pragma SPARK_Mode (Off);
      use all type Real32_Seq;
      Aspect  : constant Aspect_Ratio_T :=
        (if Camera.VertResolution = 0 then 1.0
         else Real64 (Camera.HorizResolution)
              / Real64 (Camera.VertResolution));
      Min_Res : constant Min_Resolution_T :=
        Real64'Min (Real64 (Camera.HorizResolution),
                    Real64 (Camera.VertResolution));
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
                     Update_Best
                       (FP, First_GSD_Found,
                        Horiz_FOV_Deg
                          (Min_FOV + Real64 (FOV_Step) * HORIZONTAL_FOV_STEP_SIZE_DEG),
                        Slant, Min_Res, Acceptable_GSD, Altitude, Elev_Rad,
                        Aspect, Camera.PayloadID, Camera.SupportedWavelengthBand,
                        Gimbal_ID);
                  end loop;
               end if;
            end;
         else
            --  Discrete mode
            for FOV_Entry of Camera.DiscreteHFOVList loop
               Update_Best
                 (FP, First_GSD_Found,
                  Horiz_FOV_Deg (Real64'Max (0.0, Real64'Min (360.0,
                                                               Real64 (FOV_Entry)))),
                  Slant, Min_Res, Acceptable_GSD, Altitude, Elev_Rad,
                  Aspect, Camera.PayloadID, Camera.SupportedWavelengthBand,
                  Gimbal_ID);
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
      Elev_Rad            : Elevation_Rad;
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
         else Desired_GSD);

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
