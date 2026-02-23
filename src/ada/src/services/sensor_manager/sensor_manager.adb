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

   --  Local subprogram declarations
   procedure Calculate_Sensor_Footprint
     (FP            : in out SensorFootprint_Msg;
      Altitude      : Real64;
      Elev_Rad      : Real64;
      Horiz_FOV_Rad : Real64;
      Aspect_Ratio  : Real64)
   with
      Always_Terminates,
      Pre => Altitude >= MINIMUM_ASSIGNED_ALTITUDE_M;

   procedure Find_Sensor_Footprint
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired_GSD         : Real64;
      Altitude_AGL        : Real64;
      Elevation_Angle     : Real64;
      FP                  : in out SensorFootprint_Msg;
      First_GSD_Found     : in out Boolean)
   with
      Always_Terminates,
      Pre =>
         Entity_Cfg.NominalAltitude >= 10.0;

   --------------------------------
   -- Calculate_Sensor_Footprint --
   --------------------------------

   procedure Calculate_Sensor_Footprint
     (FP            : in out SensorFootprint_Msg;
      Altitude      : Real64;
      Elev_Rad      : Real64;
      Horiz_FOV_Rad : Real64;
      Aspect_Ratio  : Real64)
   is
      use Math;
      Vert_FOV       : Real64;
      Gimbal_Max     : Real64;
      Gimbal_Min     : Real64;
      Slant_Range    : Real64;
      Horiz_Center   : Real64;
      Horiz_Leading  : Real64;
      Horiz_Trailing : Real64;
      Width_Center   : Real64;
      Denom          : Real64;
   begin
      if abs (Aspect_Ratio) < COMPARISON_TOLERANCE then
         Vert_FOV := Horiz_FOV_Rad;
      else
         Vert_FOV := Horiz_FOV_Rad / Aspect_Ratio;
      end if;

      Gimbal_Max :=
        Real64'Max (-Pi, Real64'Min (0.0, Elev_Rad + Vert_FOV / 2.0));
      Gimbal_Min :=
        Real64'Max (-Pi, Real64'Min (0.0, Elev_Rad - Vert_FOV / 2.0));

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

      Width_Center := 2.0 * Slant_Range * Tan (0.5 * Horiz_FOV_Rad);

      FP.SlantRangeToCenter       := Real32 (Slant_Range);
      FP.HorizontalToCenter       := Real32 (Horiz_Center);
      FP.HorizontalToLeadingEdge  := Real32 (Horiz_Leading);
      FP.HorizontalToTrailingEdge := Real32 (Horiz_Trailing);
      FP.WidthCenter              := Real32 (Width_Center);
   end Calculate_Sensor_Footprint;

   procedure Lemma_Bounded_Difference
     (MIN_BOUND : Real64;
      Min       : Real64;
      Max       : Real64;
      MAX_BOUND : Real64)
   with
      Pre =>
         MIN_BOUND >= -1_000.0 and then
         MAX_BOUND <=  1_000.0 and then
         MIN_BOUND <= Min and then
                      Min <= Max and then
                             Max <= MAX_BOUND,
      Post =>
         Max - Min <= MAX_BOUND - MIN_BOUND,
      Ghost
   is
   begin
      null;
   end Lemma_Bounded_Difference;


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
      use Math;
      use all type GimbalConfig_Seq;
      use all type CameraConfig_Seq;
      use all type Real32_Seq;
      use all type Int64_Seq;

      Altitude : constant Real64 :=
        (if Altitude_AGL < 0.001 then Real64 (Entity_Cfg.NominalAltitude)
         else Altitude_AGL);

      Acceptable_GSD : constant Real64 :=
        (if Desired_GSD < 0.001 then DEFAULT_ACCEPTABLE_GSD
         else Desired_GSD);

      procedure Update_Best
        (FOV_Deg         : Real64;
         Min_Res         : UInt32;
         Slant           : Real64;
         Acceptable_GSD  : Real64;
         Altitude        : Real64;
         Elev_Rad        : Real64;
         Aspect          : Real64;
         Camera          : CameraConfig;
         Gimbal          : GimbalConfig;
         FP              : in out SensorFootprint_Msg;
         First_GSD_Found : in out Boolean)
      with
         Pre => Acceptable_GSD >= DEFAULT_ACCEPTABLE_GSD;

      procedure Update_Best
        (FOV_Deg         : Real64;
         Min_Res         : UInt32;
         Slant           : Real64;
         Acceptable_GSD  : Real64;
         Altitude        : Real64;
         Elev_Rad        : Real64;
         Aspect          : Real64;
         Camera          : CameraConfig;
         Gimbal          : GimbalConfig;
         FP              : in out SensorFootprint_Msg;
         First_GSD_Found : in out Boolean)
      is
         FOV_Rad       : constant Real64 := FOV_Deg * Deg_To_Rad;
         Alpha_Rad     : constant Real64 :=
           (if Min_Res <= 0 then
               Pi / 2.0
            else
               FOV_Rad / Real64 (Min_Res));
         GSD           : constant Real64 := Slant * Sin (Alpha_Rad);
         GSD_Delta     : constant Real64 := abs (Acceptable_GSD - GSD);
      begin
         if not First_GSD_Found
            or else abs (Acceptable_GSD - Real64 (FP.AchievedGSD))
                    > GSD_Delta
         then
            First_GSD_Found    := True;
            FP.CameraID        := Camera.PayloadID;
            FP.GimbalID        := Gimbal.PayloadID;
            FP.HorizontalFOV   := Real32 (FOV_Deg);
            FP.AglAltitude     := Real32 (Altitude);
            FP.GimbalElevation := Real32 (Elev_Rad / Deg_To_Rad);
            FP.AspectRatio     := Real32 (Aspect);
            FP.AchievedGSD     := Real32 (GSD);
            FP.CameraWavelength := Camera.SupportedWavelengthBand;
            Calculate_Sensor_Footprint (FP, Altitude, Elev_Rad, FOV_Rad, Aspect);
         end if;
      end Update_Best;

   begin
      --  TODO: eliminate via precondition?
      --  Sanity check: altitude must meet minimum
      if Altitude < MINIMUM_ASSIGNED_ALTITUDE_M then
         return;
      end if;

      --  Entailed by the above.
      pragma Assert (Altitude >= MINIMUM_ASSIGNED_ALTITUDE_M);

      for Gimbal of Entity_Cfg.Gimbals loop
         declare
            Elev_Min : Real64;
            Elev_Max : Real64;
            N_Elev   : Natural;
         begin
            Elev_Min := Real64 (Gimbal.MinElevation) * Deg_To_Rad;
            Elev_Max := Real64 (Gimbal.MaxElevation) * Deg_To_Rad;

            --  This differs from the C++: C++ makes Elev_Max = Elev_Min in
            --  this case, but only *after* the clamping has been applied -
            --  which means that there is no upper bound on Elev_Min or
            --  Elev_Max when Elev_Max < Elev_Min. I think this is a bug.
            if Elev_Max < Elev_Min then
               declare
                  Temp : constant Real64 := Elev_Max;
               begin
                  Elev_Max := Elev_Min;
                  Elev_Min := Temp;
               end;
            end if;

            --  Clamp to valid downward-facing range
            if Elev_Min < ELEV_MIN_BOUND then
               Elev_Min := ELEV_MIN_BOUND;
            end if;

            if Elev_Max > ELEV_MAX_BOUND then
               Elev_Max := ELEV_MAX_BOUND;
            end if;

            --  Unclamped gimbal: use full elevation range
            if not Gimbal.IsElevationClamped then
               Elev_Max := ELEV_MAX_BOUND;
               Elev_Min := ELEV_MIN_BOUND;
            end if;

            --  Elevation override: replicate C++ behavior
            if Elevation_Angle < 0.001 then
               if Elevation_Angle > Elev_Min then
                  Elev_Min := Elevation_Angle;
               end if;
               Elev_Max := Elev_Min;
            end if;

            --  Full bounds on Elev_Min and Elev_Max
            pragma Assert (
               ELEV_MIN_BOUND <= Elev_Min and then
                                 Elev_Min <= Elev_Max and then
                                             Elev_Max <= ELEV_MAX_BOUND
            );

            --  Only process gimbals pointing downward
            if Elev_Min < 0.0 then
               N_Elev :=
                  Natural
                     (Real64'Floor ((Elev_Max - Elev_Min) / GIMBAL_STEP_SIZE_RAD));

               Lemma_Bounded_Difference (ELEV_MIN_BOUND, Elev_Min, Elev_Max, ELEV_MAX_BOUND);
               pragma Assert (Elev_Max - Elev_Min <= ELEV_MAX_BOUND - ELEV_MIN_BOUND);
               pragma Assert ((Elev_Max - Elev_Min) / GIMBAL_STEP_SIZE_RAD <=
                              (ELEV_MAX_BOUND - ELEV_MIN_BOUND) / GIMBAL_STEP_SIZE_RAD);
               pragma Assert (N_Elev <= Natural (Real64'Floor ((ELEV_MAX_BOUND - ELEV_MIN_BOUND) / GIMBAL_STEP_SIZE_RAD)));

               --  In order for us to be sure that Elev_Rad is strictly
               --  negative in the loop below, we will need to know that
               --  Elev_Min + Elev_Step * Gimbal_Step is also negative. But
               --  of course we cannot know that, since the test above is only
               --  on Elev_Min. Thus this assertion fails to prove:
               pragma Assert (Elev_Min + Real64 (N_Elev) * GIMBAL_STEP_SIZE_RAD < 0.0);

               for Elev_Step in 0 .. N_Elev loop
                  declare
                     Elev_Rad : constant Real64 :=
                       Elev_Min + Real64 (Elev_Step) * GIMBAL_STEP_SIZE_RAD;
                     Denom    : constant Real64 := Sin (-Elev_Rad);
                     Slant    : constant Real64 :=
                       (if abs (Denom) < COMPARISON_TOLERANCE then Altitude
                        else Altitude / Denom);
                  begin
                     pragma Assert (Elev_Rad < 0.0);
                     pragma Assert (Denom > 0.0);
                     pragma Assert (Slant <= Altitude);

                     for Cam_ID of Gimbal.ContainedPayloadList loop
                        for Camera of Entity_Cfg.Cameras loop
                           if Camera.PayloadID = Cam_ID then
                              if Camera.SupportedWavelengthBand = Eligible_Wavelength
                                or else Eligible_Wavelength = AllAny
                              then
                                 declare
                                    Aspect  : constant Real64 :=
                                      (if Camera.VertResolution = 0 then 1.0
                                       else Real64 (Camera.HorizResolution)
                                            / Real64 (Camera.VertResolution));
                                    Min_Res : constant UInt32 :=
                                      UInt32'Min
                                        (Camera.HorizResolution,
                                         Camera.VertResolution);
                                 begin
                                    if Camera.FieldOfViewMode = Continuous then
                                       declare
                                          Min_FOV : constant Real64 :=
                                            Real64 (Camera.MinHorizontalFOV);
                                          Max_FOV : constant Real64 :=
                                            Real64 (Camera.MaxHorizontalFOV);
                                          N_FOV   : Natural;
                                       begin
                                          if Max_FOV >= Min_FOV then
                                             N_FOV :=
                                               Natural
                                                 (Real64'Floor
                                                    ((Max_FOV - Min_FOV)
                                                     / HORIZONTAL_FOV_STEP_SIZE_DEG))
                                               + 1;
                                             for FOV_Step in 0 .. N_FOV - 1 loop
                                                Update_Best
                                                  (Min_FOV
                                                   + Real64 (FOV_Step)
                                                   * HORIZONTAL_FOV_STEP_SIZE_DEG,
                                                   Min_Res, Slant, Acceptable_GSD,
                                                   Altitude, Elev_Rad, Aspect,
                                                   Camera, Gimbal, FP, First_GSD_Found);
                                             end loop;
                                          end if;
                                       end;
                                    else
                                       --  Discrete mode
                                       for FOV_Entry of Camera.DiscreteHFOVList loop
                                          Update_Best
                                            (Real64 (FOV_Entry),
                                             Min_Res, Slant, Acceptable_GSD,
                                             Altitude, Elev_Rad, Aspect,
                                             Camera, Gimbal, FP, First_GSD_Found);
                                       end loop;
                                    end if;
                                 end;
                              end if;
                           end if;
                        end loop;
                     end loop;
                  end;
               end loop;
            end if;
         end;
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
