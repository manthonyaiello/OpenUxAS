with Ada.Numerics.Generic_Elementary_Functions;
with Sensor_Manager_Types; use Sensor_Manager_Types;

package body Sensor_Manager with SPARK_Mode is

   use Ada.Containers;

   package Math is new Ada.Numerics.Generic_Elementary_Functions (Real64);

   --  Tolerance of the guarded divisions in Calculate_Sensor_Footprint,
   --  matching the C++ double comparison; it is reachable only for the
   --  FOV-widened footprint edge angles, which are clamped to [-Pi, 0]
   --  and so can sit exactly at a zero of Tan.
   Comparison_Tolerance : constant Real64 := 1.0e-10;

   --  Local subprogram declarations

   procedure Calculate_Sensor_Footprint
     (FP           : in out SensorFootprint_Msg;
      Altitude     : Assigned_Altitude_M;
      Elev         : Working_Elevation_Rad;
      Horiz_FOV    : FOV_Deg;
      Aspect_Ratio : Aspect_Ratio_T)
     with Always_Terminates;

   procedure Find_Sensor_Footprint
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired_GSD_Wire    : Real32;
      Altitude_Wire       : Real32;
      Elevation_Wire      : Real32;
      FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean)
     with Always_Terminates;

   --------------------------------
   -- Calculate_Sensor_Footprint --
   --------------------------------

   procedure Calculate_Sensor_Footprint
     (FP           : in out SensorFootprint_Msg;
      Altitude     : Assigned_Altitude_M;
      Elev         : Working_Elevation_Rad;
      Horiz_FOV    : FOV_Deg;
      Aspect_Ratio : Aspect_Ratio_T)
   is
      pragma SPARK_Mode (Off);  --  elementary-function calls
      use Math;

      function Guarded_Ratio (Denom : Real64) return Edge_Distance_M is
        (if abs Denom < Comparison_Tolerance then 0.0
         else Real64 (Altitude) / Denom);

      Horiz_FOV_Rad : constant Real64 := Real64 (To_Radians (Horiz_FOV));

      --  Aspect_Ratio_T is bounded away from zero, so the C++ guard
      --  against a zero aspect ratio is not needed.
      Vert_FOV : constant Real64 := Horiz_FOV_Rad / Real64 (Aspect_Ratio);

      --  Footprint edge angles: the boresight elevation widened by half
      --  the vertical FOV, clamped to [-Pi, 0] as in C++.
      Gimbal_Max : constant Real64 :=
        Real64'Max (-Pi, Real64'Min (0.0, Real64 (Elev) + Vert_FOV / 2.0));
      Gimbal_Min : constant Real64 :=
        Real64'Max (-Pi, Real64'Min (0.0, Real64 (Elev) - Vert_FOV / 2.0));

      --  Over the working elevation range Sin and Tan of -Elev are
      --  bounded away from zero, so center distances need no guard.
      Slant_To_Center : constant Slant_Range_M   :=
        Slant_Range (Altitude, Elev);
      Horiz_Center    : constant Center_Distance_M :=
        Real64 (Altitude) / Tan (-Real64 (Elev));

      Horiz_Leading  : constant Edge_Distance_M :=
        Guarded_Ratio (Tan (-Gimbal_Max));
      Horiz_Trailing : constant Edge_Distance_M :=
        Guarded_Ratio (Tan (-Gimbal_Min));

      Width_Center : constant Width_M :=
        2.0 * Real64 (Slant_To_Center) * Tan (0.5 * Horiz_FOV_Rad);
   begin
      FP.SlantRangeToCenter       := Real32 (Slant_To_Center);
      FP.HorizontalToCenter       := Real32 (Horiz_Center);
      FP.HorizontalToLeadingEdge  := Real32 (Horiz_Leading);
      FP.HorizontalToTrailingEdge := Real32 (Horiz_Trailing);
      FP.WidthCenter              := Real32 (Width_Center);
   end Calculate_Sensor_Footprint;

   ---------------------------
   -- Find_Sensor_Footprint --
   ---------------------------

   procedure Find_Sensor_Footprint
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired_GSD_Wire    : Real32;
      Altitude_Wire       : Real32;
      Elevation_Wire      : Real32;
      FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean)
   is
      Altitude : constant Altitude_Result :=
        Effective_Altitude (Altitude_Wire, Entity_Cfg.NominalAltitude);

      Acceptable_GSD : constant Desired_GSD_M :=
        Effective_Desired_GSD (Desired_GSD_Wire);

      --  Best GSD so far, tracked in full precision: FP.AchievedGSD is
      --  Real32 on the wire, and comparing a fresh Real64 candidate
      --  against the rounded stored value would let exactly-tied
      --  candidates win by a rounding ulp (ties must keep the first
      --  candidate, as in C++).
      Best_GSD : Achieved_GSD_M := 0.0;

      --  Record the candidate if it beats the best GSD found so far.

      procedure Consider_Candidate
        (Camera    : CameraConfig;
         Gimbal_ID : Int64;
         Elev      : Working_Elevation_Rad;
         Slant     : Slant_Range_M;
         FOV       : FOV_Deg)
        with Always_Terminates;

      --  Enumerate the valid FOV candidates of one camera at one
      --  boresight elevation.

      procedure Evaluate_Camera
        (Camera    : CameraConfig;
         Gimbal_ID : Int64;
         Elev      : Working_Elevation_Rad;
         Slant     : Slant_Range_M)
        with Always_Terminates;

      --  Sweep one gimbal's elevation range and evaluate every eligible
      --  camera mounted on it.

      procedure Evaluate_Gimbal (Gimbal : GimbalConfig)
        with Always_Terminates;

      ------------------------
      -- Consider_Candidate --
      ------------------------

      procedure Consider_Candidate
        (Camera    : CameraConfig;
         Gimbal_ID : Int64;
         Elev      : Working_Elevation_Rad;
         Slant     : Slant_Range_M;
         FOV       : FOV_Deg)
      is
         GSD : constant Achieved_GSD_M :=
           Compute_GSD (Slant, FOV, Min_Resolution (Camera));
      begin
         if Is_Better
              (Desired     => Acceptable_GSD,
               Candidate   => GSD,
               Current     => Best_GSD,
               First_Found => First_GSD_Found)
         then
            First_GSD_Found     := True;
            Best_GSD            := GSD;
            FP.CameraID         := Camera.PayloadID;
            FP.GimbalID         := Gimbal_ID;
            FP.HorizontalFOV    := Real32 (FOV);
            FP.AglAltitude      := Real32 (Altitude.Value);
            FP.GimbalElevation  := Real32 (To_Degrees (Elev));
            FP.AspectRatio      := Real32 (Aspect_Ratio_Of (Camera));
            FP.AchievedGSD      := Real32 (GSD);
            FP.CameraWavelength := Camera.SupportedWavelengthBand;
            Calculate_Sensor_Footprint
              (FP           => FP,
               Altitude     => Altitude.Value,
               Elev         => Elev,
               Horiz_FOV    => FOV,
               Aspect_Ratio => Aspect_Ratio_Of (Camera));
         end if;
      end Consider_Candidate;

      ---------------------
      -- Evaluate_Camera --
      ---------------------

      procedure Evaluate_Camera
        (Camera    : CameraConfig;
         Gimbal_ID : Int64;
         Elev      : Working_Elevation_Rad;
         Slant     : Slant_Range_M)
      is
      begin
         if Camera.FieldOfViewMode = Continuous then
            declare
               Candidates : constant FOV_Candidates :=
                 Continuous_Candidates
                   (Camera.MinHorizontalFOV, Camera.MaxHorizontalFOV);
            begin
               for Index in 0 .. Candidates.Count - 1 loop
                  Consider_Candidate
                    (Camera, Gimbal_ID, Elev, Slant,
                     Candidate_FOV (Candidates, Index));
               end loop;
            end;
         else
            --  Discrete mode: entries outside (0, 179] degrees are
            --  skipped (D5).
            for FOV_Entry of Camera.DiscreteHFOVList loop
               if Is_Valid_FOV (FOV_Entry) then
                  Consider_Candidate
                    (Camera, Gimbal_ID, Elev, Slant,
                     FOV_Deg (Degrees_64 (FOV_Entry)));
               end if;
            end loop;
         end if;
      end Evaluate_Camera;

      ---------------------
      -- Evaluate_Gimbal --
      ---------------------

      procedure Evaluate_Gimbal (Gimbal : GimbalConfig) is
         Sweep : constant Elevation_Sweep :=
           Apply_Override (Gimbal_Sweep_Range (Gimbal), Elevation_Wire);
      begin
         if not Sweep.Valid then
            return;
         end if;

         for Step in 0 .. Sweep_Step_Count (Sweep) - 1 loop
            declare
               Elev  : constant Working_Elevation_Rad :=
                 Sweep_Elevation (Sweep, Step);
               Slant : constant Slant_Range_M         :=
                 Slant_Range (Altitude.Value, Elev);
            begin
               for Cam_ID of Gimbal.ContainedPayloadList loop
                  for Camera of Entity_Cfg.Cameras loop
                     if Camera.PayloadID = Cam_ID
                       and then
                         (Camera.SupportedWavelengthBand = Eligible_Wavelength
                          or else Eligible_Wavelength = AllAny)
                     then
                        Evaluate_Camera (Camera, Gimbal.PayloadID, Elev,
                                         Slant);
                     end if;
                  end loop;
               end loop;
            end;
         end loop;
      end Evaluate_Gimbal;

   begin
      if not Altitude.Valid then
         return;
      end if;

      for Gimbal of Entity_Cfg.Gimbals loop
         Evaluate_Gimbal (Gimbal);
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

      --  Empty request dimensions take a single default element (the
      --  0.0 / AllAny "unspecified" sentinels), as in C++.

      function Defaulted (Seq : Real32_Seq) return Real32_Seq is
        (if Natural (Last (Seq)) = 0 then Add (Empty_Sequence, 0.0)
         else Seq);

      function Defaulted (Seq : WavelengthBand_Seq) return WavelengthBand_Seq
      is
        (if Natural (Last (Seq)) = 0 then Add (Empty_Sequence, AllAny)
         else Seq);

      procedure Process_Request
        (Request  : FootprintRequest_Msg;
         Response : in out SensorFootprintResponse_Msg);

      ---------------------
      -- Process_Request --
      ---------------------

      procedure Process_Request
        (Request  : FootprintRequest_Msg;
         Response : in out SensorFootprintResponse_Msg)
      is
         Entity_Cfg : constant EntityConfig :=
           Element (State.Entity_Configs, Request.VehicleID);
      begin
         for W of Defaulted (Request.EligibleWavelengths) loop
            for G of Defaulted (Request.GroundSampleDistances) loop
               for A of Defaulted (Request.AglAltitudes) loop
                  for E of Defaulted (Request.ElevationAngles) loop
                     declare
                        FP              : SensorFootprint_Msg;
                        First_GSD_Found : Boolean := False;
                     begin
                        FP.FootprintResponseID := Request.FootprintRequestID;
                        FP.VehicleID           := Entity_Cfg.ID;
                        Find_Sensor_Footprint
                          (Entity_Cfg          => Entity_Cfg,
                           Eligible_Wavelength => W,
                           Desired_GSD_Wire    => G,
                           Altitude_Wire       => A,
                           Elevation_Wire      => E,
                           FP                  => FP,
                           First_GSD_Found     => First_GSD_Found);
                        Response.Footprints :=
                          Add (Response.Footprints, FP);
                     end;
                  end loop;
               end loop;
            end loop;
         end loop;
      end Process_Request;

      Response : SensorFootprintResponse_Msg;
   begin
      Response.ResponseID := Msg.RequestID;

      for Request of Msg.Footprints loop
         if Contains (State.Entity_Configs, Request.VehicleID) then
            Process_Request (Request, Response);
         end if;
      end loop;

      sendBroadcastMessage (Mailbox, Response);
   end Handle_SensorFootprintRequests;

end Sensor_Manager;
