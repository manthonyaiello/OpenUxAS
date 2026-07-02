with Ada.Numerics.Generic_Elementary_Functions;

package body Sensor_Manager_Types with SPARK_Mode is

   package Math is new Ada.Numerics.Generic_Elementary_Functions (Real64);

   --  Clamp a finite wire elevation into the documented CMASI range.
   function Clamped_Elevation (Wire : Real32) return Elevation_Deg is
     (Degrees_64'Min
        (Degrees_64'Max (Degrees_64 (Wire), Elevation_Deg'First),
         Elevation_Deg'Last))
   with Pre => Is_Finite (Wire);

   --  Clamp a radian elevation into the working range.
   function Clamp_Working (R : Radians_64) return Working_Elevation_Rad is
     (Radians_64'Min
        (Radians_64'Max (R, Working_Elevation_Rad'First),
         Working_Elevation_Rad'Last));

   ---------------
   -- Is_Finite --
   ---------------

   function Is_Finite (X : Real32) return Boolean is
      pragma SPARK_Mode (Off);  --  X may be NaN or infinite here
   begin
      --  NaN fails X = X; infinities fail the range comparisons.
      return X = X and then X >= Real32'First and then X <= Real32'Last;
   end Is_Finite;

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

   function Effective_Altitude
     (Wire_M    : Real32;
      Nominal_M : Real32) return Altitude_Result
   is
      pragma SPARK_Mode (Off);  --  wire values may be NaN or infinite
      Invalid : constant Altitude_Result := (others => <>);
      Alt     : Real64 := Real64 (Wire_M);
   begin
      --  NaN requests fail the C++ altitude gate too (NaN >= 10 is
      --  false); reproduce that as an invalid result.
      if Alt /= Alt then
         return Invalid;
      end if;

      --  C++ rule: below the 0.001 threshold (which includes the 0.0
      --  "use nominal" sentinel, negatives and -Inf), substitute the
      --  entity's nominal altitude.
      if Alt < 0.001 then
         Alt := Real64 (Nominal_M);
      end if;

      --  D6: the gate is membership in [10 m, 100 km]; C++ checks only
      --  the 10 m floor (and so happily plans at +Inf altitude).
      if Alt /= Alt
        or else Alt < Assigned_Altitude_M'First
        or else Alt > Assigned_Altitude_M'Last
      then
         return Invalid;
      end if;

      return (Valid => True, Value => Alt);
   end Effective_Altitude;

   ---------------------------
   -- Effective_Desired_GSD --
   ---------------------------

   function Effective_Desired_GSD (Wire_M : Real32) return Desired_GSD_M is
      pragma SPARK_Mode (Off);  --  wire values may be NaN or infinite
      GSD : constant Real64 := Real64 (Wire_M);
   begin
      --  D8: non-finite values take the default, like the sub-threshold
      --  values do in C++ (whose comparison chain they would poison).
      if GSD /= GSD or else GSD < Desired_GSD_M'First
        or else GSD > Real64 (Real32'Last)
      then
         return Default_Acceptable_GSD;
      end if;
      return Real64'Min (GSD, Desired_GSD_M'Last);
   end Effective_Desired_GSD;

   -----------------
   -- Slant_Range --
   -----------------

   function Slant_Range
     (Altitude : Assigned_Altitude_M;
      Elev     : Working_Elevation_Rad) return Slant_Range_M
   is
      pragma SPARK_Mode (Off);  --  elementary-function call
   begin
      --  Sin (-Elev) is in [sin (1 deg), 1] over the working range, so
      --  the division needs no guard: the result is within
      --  [Altitude, Altitude / sin (1 deg)].
      return Real64 (Altitude) / Math.Sin (-Real64 (Elev));
   end Slant_Range;

   -----------------
   -- Compute_GSD --
   -----------------

   function Compute_GSD
     (Slant   : Slant_Range_M;
      FOV     : FOV_Deg;
      Min_Res : Pixel_Count) return Achieved_GSD_M
   is
      pragma SPARK_Mode (Off);  --  elementary-function call
      Alpha : constant Real64 :=
        (if Min_Res = 0
         then Pi / 2.0    --  C++ worst case for unknown resolution
         else Real64 (To_Radians (FOV)) / Real64 (Min_Res));
   begin
      --  Alpha is in (0, Pi), so Sin (Alpha) is in (0, 1] and the
      --  result cannot exceed the slant range.
      return Real64 (Slant) * Math.Sin (Alpha);
   end Compute_GSD;

end Sensor_Manager_Types;
