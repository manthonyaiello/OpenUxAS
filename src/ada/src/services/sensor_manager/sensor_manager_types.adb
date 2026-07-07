with Sensor_Manager_Trig; use Sensor_Manager_Trig;

package body Sensor_Manager_Types with SPARK_Mode is

   ---------------
   -- Is_Finite --
   ---------------

   function Is_Finite (X : Real32) return Boolean is
      pragma SPARK_Mode (Off);  --  X may be NaN or infinite here
   begin
      --  NaN fails X = X; infinities fail the range comparisons.
      return X = X and then X >= Real32'First and then X <= Real32'Last;
   end Is_Finite;

   --------------------------------
   -- Is_Below_Nominal_Threshold --
   --------------------------------

   function Is_Below_Nominal_Threshold (X : Real32) return Boolean is
      pragma SPARK_Mode (Off);  --  X may be NaN or infinite here
   begin
      --  NaN compares False, -Inf compares True, exactly like the
      --  C++ "altitude < 0.001" test this mirrors.
      return X < 0.001;
   end Is_Below_Nominal_Threshold;

   ------------------------
   -- Gimbal_Sweep_Range --
   ------------------------

   function Gimbal_Sweep_Range (Gimbal : GimbalConfig) return Elevation_Sweep
   is
      Result       : Elevation_Sweep;
      Min_R, Max_R : Radians_64;
   begin
      if not Is_Finite (Gimbal.MinElevation)
        or else not Is_Finite (Gimbal.MaxElevation)
      then
         return Result;
      end if;

      Min_R := To_Radians (Clamped_Elevation (Gimbal.MinElevation));
      Max_R := To_Radians (Clamped_Elevation (Gimbal.MaxElevation));

      --  C++-faithful limit repair (SensorManagerService.cpp:243-244): a
      --  max above horizontal cannot see the ground, and crossed limits
      --  collapse onto the min. (The C++ min repair at line 241 is
      --  subsumed by the working-range clamp below.)
      if Max_R > 0.0 then
         Max_R := Working_Elevation_Rad'Last;
      end if;
      if Max_R < Min_R then
         Max_R := Min_R;
      end if;

      --  A gimbal free to rotate 360 degrees can always point down.
      if not Gimbal.IsElevationClamped then
         Min_R := Working_Elevation_Rad'First;
         Max_R := Working_Elevation_Rad'Last;
      end if;

      --  C++ gate: only gimbals that can point below horizontal
      --  contribute a footprint.
      Result.Valid := Min_R < 0.0;

      if Result.Valid then
         --  D4: both ends land in the working range. C++ leaves a min in
         --  [-180, -179) deg and a max in (-1, 0] deg unclamped and then
         --  evaluates geometry there (slant range diverging near 0 deg,
         --  mirror geometry at -180 deg).
         Result.Lo := Clamp_Working (Min_R);
         Result.Hi := Clamp_Working (Max_R);
      end if;

      return Result;
   end Gimbal_Sweep_Range;

   --------------------
   -- Apply_Override --
   --------------------

   function Apply_Override
     (Sweep    : Elevation_Sweep;
      Wire_Deg : Real32) return Elevation_Sweep
   is
      Result : Elevation_Sweep := Sweep;
   begin
      if not Is_Finite (Wire_Deg) then
         return Sweep;
      end if;

      declare
         E_Deg : constant Degrees_64 := Degrees_64 (Wire_Deg);
      begin
         --  C++ treats a request at or above its 0.001 threshold as
         --  "no override" (positive elevations are silently ignored).
         if E_Deg >= 0.001 then
            return Sweep;
         end if;

         --  C++ dead path preserved: the 0.0 "unspecified" sentinel (and
         --  anything up to the threshold) pins the sweep at or above
         --  horizontal, so no footprint is computed and the degenerate
         --  all-zero footprint is emitted.
         if E_Deg >= 0.0 then
            Result.Valid := False;
            return Result;
         end if;

         --  D2: the request is documented as degrees (UXTASK.xml); C++
         --  compares the raw degree value against radian limits.
         --  D3: the converted angle is confined to the working range and
         --  to the gimbal's own range; C++ pins min := max(request, min)
         --  and ignores the gimbal max entirely.
         if Result.Valid then
            declare
               Pinned : Working_Elevation_Rad :=
                 Clamp_Working
                   (To_Radians
                      (Degrees_64'Max (E_Deg, Elevation_Deg'First)));
            begin
               Pinned    :=
                 Radians_64'Min (Radians_64'Max (Pinned, Sweep.Lo), Sweep.Hi);
               Result.Lo := Pinned;
               Result.Hi := Pinned;
            end;
         end if;

         return Result;
      end;
   end Apply_Override;

   ----------------------
   -- Sweep_Step_Count --
   ----------------------

   function Sweep_Step_Count (Sweep : Elevation_Sweep)
     return Elevation_Step_Count
   is
      Steps : constant Real64 :=
        Real64'Floor
          (Real64 (Sweep.Hi - Sweep.Lo) / Gimbal_Step_Size_Rad);
   begin
      --  Working-range spans are at most 178 deg, so Steps lies in
      --  [0, 35]; the guard keeps the function total under
      --  floating-point rounding (release builds run without checks).
      if Steps < 0.0 or else Steps > 35.0 then
         return Elevation_Step_Count'Last;
      end if;
      return Elevation_Step_Count (Integer (Steps) + 1);
   end Sweep_Step_Count;

   ---------------------
   -- Sweep_Elevation --
   ---------------------

   function Sweep_Elevation
     (Sweep : Elevation_Sweep;
      Step  : Natural) return Working_Elevation_Rad
   is
      Raw : constant Radians_64 :=
        Sweep.Lo + Radians_64 (Step) * Gimbal_Step_Size_Rad;
   begin
      --  Mathematically Raw <= Hi; 'Min absorbs floating-point rounding
      --  in the final step.
      return Radians_64'Min (Raw, Sweep.Hi);
   end Sweep_Elevation;

   -------------------------------------
   -- Lemma_Elevation_Of_Gimbal_Intro --
   -------------------------------------

   procedure Lemma_Elevation_Of_Gimbal_Intro
     (Entity_Cfg   : EntityConfig;
      Wire_Deg     : Real32;
      Gimbal_Index : Positive;
      Step         : Natural)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  introduction is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Elevation_Of_Gimbal);

   ----------------------------------
   -- Lemma_Camera_On_Gimbal_Intro --
   ----------------------------------

   procedure Lemma_Camera_On_Gimbal_Intro
     (Entity_Cfg    : EntityConfig;
      Gimbal_Index  : Positive;
      List_Position : Positive;
      Camera_Index  : Positive)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  introduction is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Camera_On_Gimbal);

   --------------------------------------
   -- Lemma_Valid_FOV_Continuous_Intro --
   --------------------------------------

   procedure Lemma_Valid_FOV_Continuous_Intro
     (Camera : CameraConfig;
      Index  : Natural)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  introduction is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Valid_FOV_Of);

   ------------------------------------
   -- Lemma_Valid_FOV_Discrete_Intro --
   ------------------------------------

   procedure Lemma_Valid_FOV_Discrete_Intro
     (Camera : CameraConfig;
      DI     : Positive)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  introduction is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Valid_FOV_Of);

   ---------------------------
   -- Continuous_Candidates --
   ---------------------------

   function Continuous_Candidates
     (Wire_Min, Wire_Max : Real32) return FOV_Candidates
   is
      None         : constant FOV_Candidates := (others => <>);

      --  Grid indices are bounded by the anchor sanity bound:
      --  (3600 + 179) / 5 + 2 < 1_002.
      subtype Grid_Index is Integer range 0 .. 1_002;

      Min_D, Max_D : Real64;
      Hi_End       : Real64;
      K0, K_Last   : Grid_Index;
      First        : Real64;
   begin
      if not Is_Finite (Wire_Min) or else not Is_Finite (Wire_Max) then
         return None;
      end if;

      Min_D := Real64 (Wire_Min);
      Max_D := Real64 (Wire_Max);

      if Max_D < Min_D                                     --  empty range
        or else Max_D <= 0.0                               --  all invalid
        or else Min_D > Max_FOV_Deg                        --  all invalid
        or else abs Min_D > Max_FOV_Anchor_Magnitude_Deg   --  D5 sanity
      then
         return None;
      end if;

      Hi_End := Real64'Min (Max_D, Max_FOV_Deg);

      --  Grid indices K such that Min_D + K * 5 lies in (0, 179]. The
      --  grid is anchored at the wire minimum so that valid candidates
      --  coincide with the C++ enumeration. The float-to-integer
      --  guards are unreachable for the sanity-bounded anchors accepted
      --  above; they keep the function total under floating-point
      --  rounding (release builds run without checks).
      if Min_D > 0.0 then
         K0 := 0;
      else
         declare
            F : constant Real64 :=
              Real64'Floor (-Min_D / Horizontal_FOV_Step_Size_Deg);
         begin
            if F < 0.0 or else F > 1_000.0 then
               return None;
            end if;
            K0 := Integer (F) + 1;
         end;
      end if;

      declare
         F : constant Real64 :=
           Real64'Floor
             ((Hi_End - Min_D) / Horizontal_FOV_Step_Size_Deg);
      begin
         if F < 0.0 or else F > 1_000.0 then
            return None;
         end if;
         K_Last := Integer (F);
      end;

      if K_Last < K0 then
         return None;
      end if;

      First := Min_D + Real64 (K0) * Horizontal_FOV_Step_Size_Deg;

      --  The first grid index above zero yields a positive grid point;
      --  it can round to a nonpositive value only by floating-point
      --  error, in which case the next grid point is the first valid.
      if First <= 0.0 then
         K0    := K0 + 1;
         First := First + Horizontal_FOV_Step_Size_Deg;
      end if;

      if K_Last < K0 or else First <= 0.0 or else First > Max_FOV_Deg then
         return None;
      end if;

      --  At most 36 grid points fit in (0, 179] at a 5-degree pitch;
      --  the cap is provably redundant for in-range values.
      return
        (Count       => FOV_Step_Count (Integer'Min (K_Last - K0 + 1, 36)),
         First_Valid => FOV_Deg (Degrees_64 (First)));
   end Continuous_Candidates;

   -------------------
   -- Candidate_FOV --
   -------------------

   function Candidate_FOV
     (Candidates : FOV_Candidates;
      Index      : Natural) return FOV_Deg
   is
      Raw : constant Degrees_64 :=
        Candidates.First_Valid
          + Degrees_64 (Index) * Horizontal_FOV_Step_Size_Deg;
   begin
      --  Mathematically in (0, 179]; 'Min absorbs rounding at the end.
      return Degrees_64'Min (Raw, Max_FOV_Deg);
   end Candidate_FOV;

   ------------------------
   -- Effective_Altitude --
   ------------------------

   --  D6: the gate is membership in [10 m, 100 km]; C++ checks only
   --  the 10 m floor (and so happily plans at +Inf altitude).
   function Gated_Altitude (Alt : Real64) return Altitude_Result is
     (if Alt >= Assigned_Altitude_M'First
        and then Alt <= Assigned_Altitude_M'Last
      then (Valid => True, Value => Alt)
      else (others => <>));

   function Effective_Altitude
     (Wire_M    : Real32;
      Nominal_M : Real32) return Altitude_Result
   is
   begin
      --  C++ rule: below the 0.001 threshold (which includes the 0.0
      --  "use nominal" sentinel, negatives and -Inf), substitute the
      --  entity's nominal altitude. A non-finite nominal (like any
      --  out-of-range one) fails the gate.
      if Is_Below_Nominal_Threshold (Wire_M) then
         return
           (if Is_Finite (Nominal_M)
            then Gated_Altitude (Real64 (Nominal_M))
            else (others => <>));

      elsif Is_Finite (Wire_M) then
         return Gated_Altitude (Real64 (Wire_M));

      else
         --  NaN (fails the C++ altitude gate too: NaN >= 10 is false)
         --  or +Inf (above the D6 ceiling).
         return (others => <>);
      end if;
   end Effective_Altitude;

   ---------------------------
   -- Effective_Desired_GSD --
   ---------------------------

   function Effective_Desired_GSD (Wire_M : Real32) return Desired_GSD_M is
   begin
      --  D8: non-finite values take the default, like the sub-threshold
      --  values do in C++ (whose comparison chain they would poison).
      if not Is_Finite (Wire_M) then
         return Default_Acceptable_GSD;
      end if;

      declare
         GSD : constant Real64 := Real64 (Wire_M);
      begin
         if GSD < Desired_GSD_M'First then
            return Default_Acceptable_GSD;
         end if;
         return Real64'Min (GSD, Desired_GSD_M'Last);
      end;
   end Effective_Desired_GSD;

   -----------------
   -- Slant_Range --
   -----------------

   function Slant_Range
     (Altitude : Assigned_Altitude_M;
      Elev     : Working_Elevation_Rad) return Slant_Range_M
   is
      Neg_Elev : constant Real64 := -Real64 (Elev);
   begin
      --  Sin (-Elev) is in [sin (1 deg), 1] over the working range, so
      --  the division needs no guard: the result is within
      --  [Altitude, Altitude / sin (1 deg)].
      Axiom_Sin_Bounds_On_Working_Range (Neg_Elev);
      return Real64 (Altitude) / Sin (Neg_Elev);
   end Slant_Range;

   -----------------
   -- Compute_GSD --
   -----------------

   function Compute_GSD
     (Slant   : Slant_Range_M;
      FOV     : FOV_Deg;
      Min_Res : Pixel_Count) return Achieved_GSD_M
   is
      Alpha : constant Real64 :=
        (if Min_Res = 0
         then Pi / 2.0    --  C++ worst case for unknown resolution
         else Real64 (To_Radians (FOV)) / Real64 (Min_Res));
   begin
      --  Alpha is in (0, Pi), so Sin (Alpha) is in (0, 1] and the
      --  result cannot exceed the slant range.
      Axiom_Sin_Bounds_On_Half_Turn (Alpha);
      return Real64 (Slant) * Sin (Alpha);
   end Compute_GSD;

   ---------------------------------
   -- Lemma_Camera_Covered_Extend --
   ---------------------------------

   procedure Lemma_Camera_Covered_Extend
     (Camera  : CameraConfig;
      FI_Hi   : Positive;
      Slant   : Slant_Range_M;
      Desired : Desired_GSD_M;
      Found   : Boolean;
      Best    : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  extension is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Camera_Covered_Upto);

   -----------------------------------
   -- Lemma_Camera_Covered_Monotone --
   -----------------------------------

   procedure Lemma_Camera_Covered_Monotone
     (Camera  : CameraConfig;
      FI_Hi   : Natural;
      Slant   : Slant_Range_M;
      Desired : Desired_GSD_M;
      Found0  : Boolean;
      Best0   : Achieved_GSD_M;
      Found1  : Boolean;
      Best1   : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  monotonicity is proved (transitivity of Dominates, which is not
   --  hidden).
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Camera_Covered_Upto);

   ----------------------------------
   -- Lemma_Position_Covered_Empty --
   ----------------------------------

   procedure Lemma_Position_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Position            : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  trivial base case is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Position_Covered_Upto);

   -----------------------------------
   -- Lemma_Position_Covered_Extend --
   -----------------------------------

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
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  extension is proved. The camera-level predicate stays hidden: its
   --  atoms transfer syntactically between hypothesis and conclusion.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Position_Covered_Upto);

   -------------------------------------
   -- Lemma_Position_Covered_Monotone --
   -------------------------------------

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
   is
      --  The predicate is hidden by default; disclose it here, where its
      --  monotonicity is proved. The camera-level predicate stays
      --  hidden: the loop rewrites it pointwise via its own
      --  monotonicity lemma.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Position_Covered_Upto);
   begin
      for CJ in 1 .. CJ_Hi loop
         if Get (Entity_Cfg.Cameras, CJ).PayloadID
              = Get (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                       .ContainedPayloadList,
                     Position)
           and then Wavelength_Eligible
                      (Get (Entity_Cfg.Cameras, CJ), Eligible_Wavelength)
         then
            Lemma_Camera_Covered_Monotone
              (Get (Entity_Cfg.Cameras, CJ),
               FOV_Candidate_Bound (Get (Entity_Cfg.Cameras, CJ)),
               Slant, Desired, Found0, Best0, Found1, Best1);
         end if;
         pragma Loop_Invariant
           (Position_Covered_Upto
              (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, Position,
               CJ, Slant, Desired, Found1, Best1));
      end loop;
   end Lemma_Position_Covered_Monotone;

   -----------------------------------
   -- Lemma_Positions_Covered_Empty --
   -----------------------------------

   procedure Lemma_Positions_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  trivial base case is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Positions_Covered_Upto);

   ------------------------------------
   -- Lemma_Positions_Covered_Extend --
   ------------------------------------

   procedure Lemma_Positions_Covered_Extend
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Gimbal_Index        : Positive;
      P_Hi                : Positive;
      Slant               : Slant_Range_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  extension is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Positions_Covered_Upto);

   --------------------------------------
   -- Lemma_Positions_Covered_Monotone --
   --------------------------------------

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
   is
      --  The predicate is hidden by default; disclose it here, where its
      --  monotonicity is proved; the loop rewrites the position-level
      --  predicate pointwise via its own monotonicity lemma.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Positions_Covered_Upto);
   begin
      for P in 1 .. P_Hi loop
         Lemma_Position_Covered_Monotone
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, P,
            Last (Entity_Cfg.Cameras), Slant, Desired,
            Found0, Best0, Found1, Best1);
         pragma Loop_Invariant
           (Positions_Covered_Upto
              (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, P, Slant,
               Desired, Found1, Best1));
      end loop;
   end Lemma_Positions_Covered_Monotone;

   -------------------------------
   -- Lemma_Steps_Covered_Empty --
   -------------------------------

   procedure Lemma_Steps_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  trivial base case is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Steps_Covered_Upto);

   --------------------------------
   -- Lemma_Steps_Covered_Extend --
   --------------------------------

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
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  extension is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Steps_Covered_Upto);

   ----------------------------------
   -- Lemma_Steps_Covered_Monotone --
   ----------------------------------

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
   is
      --  The predicate is hidden by default; disclose it here, where its
      --  monotonicity is proved; the loop rewrites the positions-level
      --  predicate pointwise via its own monotonicity lemma.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Steps_Covered_Upto);
   begin
      for S in 0 .. S_Hi - 1 loop
         Lemma_Positions_Covered_Monotone
           (Entity_Cfg, Eligible_Wavelength, Gimbal_Index,
            Last (Get (Entity_Cfg.Gimbals, Gimbal_Index)
                    .ContainedPayloadList),
            Slant_Range
              (Altitude,
               Sweep_Elevation
                 (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                            Elev_Wire),
                  S)),
            Desired, Found0, Best0, Found1, Best1);
         pragma Loop_Invariant
           (Steps_Covered_Upto
              (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
               Gimbal_Index, S + 1, Desired, Found1, Best1));
      end loop;
   end Lemma_Steps_Covered_Monotone;

   --------------------------------
   -- Lemma_Gimbal_Covered_Intro --
   --------------------------------

   procedure Lemma_Gimbal_Covered_Intro
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Gimbal_Index        : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  introduction is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Gimbal_Covered);

   -----------------------------------
   -- Lemma_Gimbal_Covered_Monotone --
   -----------------------------------

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
   is
      --  The predicate is hidden by default; disclose it here, where its
      --  monotonicity is proved; a valid sweep is rewritten via the
      --  steps-level monotonicity lemma, an invalid one is vacuous.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Gimbal_Covered);
   begin
      if Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                   Elev_Wire).Valid
      then
         Lemma_Steps_Covered_Monotone
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
            Gimbal_Index,
            Sweep_Step_Count
              (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                         Elev_Wire)),
            Desired, Found0, Best0, Found1, Best1);
      end if;
   end Lemma_Gimbal_Covered_Monotone;

   ---------------------------------
   -- Lemma_Gimbals_Covered_Empty --
   ---------------------------------

   procedure Lemma_Gimbals_Covered_Empty
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  trivial base case is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Gimbals_Covered_Upto);

   ----------------------------------
   -- Lemma_Gimbals_Covered_Extend --
   ----------------------------------

   procedure Lemma_Gimbals_Covered_Extend
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Elev_Wire           : Real32;
      Altitude            : Assigned_Altitude_M;
      K_Hi                : Positive;
      Desired             : Desired_GSD_M;
      Found               : Boolean;
      Best                : Achieved_GSD_M)
   is null;
   --  The predicate is hidden by default; disclose it here, where its
   --  extension is proved.
   pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                    Gimbals_Covered_Upto);

   ------------------------------------
   -- Lemma_Gimbals_Covered_Monotone --
   ------------------------------------

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
   is
      --  The predicate is hidden by default; disclose it here, where its
      --  monotonicity is proved; the loop rewrites the gimbal-level
      --  predicate pointwise via its own monotonicity lemma.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Gimbals_Covered_Upto);
   begin
      for K in 1 .. K_Hi loop
         Lemma_Gimbal_Covered_Monotone
           (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, K,
            Desired, Found0, Best0, Found1, Best1);
         pragma Loop_Invariant
           (Gimbals_Covered_Upto
              (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude, K,
               Desired, Found1, Best1));
      end loop;
   end Lemma_Gimbals_Covered_Monotone;

   ---------------------------------------
   -- Lemma_Footprint_GSD_Optimal_Intro --
   ---------------------------------------

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
   is
      --  The optimality predicate is hidden by default; disclose it
      --  here, where its introduction is proved. Selection_Witness is
      --  disclosed too: it holds the equality between Best and the
      --  witnessing tuple's own GSD, which licenses rewriting the
      --  coverage atom from Best to Candidate_GSD terms via the
      --  monotonicity lemma below (equal GSDs dominate each other).
      --  Is_Candidate and the coverage predicates stay hidden: their
      --  atoms transfer syntactically from the precondition and the
      --  lemma call.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Footprint_GSD_Optimal);
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Selection_Witness);
   begin
      Lemma_Gimbals_Covered_Monotone
        (Entity_Cfg, Eligible_Wavelength, Elev_Wire, Altitude,
         Last (Entity_Cfg.Gimbals), Desired, True, Best, True,
         Candidate_GSD
           (Entity_Cfg, Altitude, Elev_Wire, Gimbal_Index, Step,
            Camera_Index, FOV_Index));
   end Lemma_Footprint_GSD_Optimal_Intro;

end Sensor_Manager_Types;
