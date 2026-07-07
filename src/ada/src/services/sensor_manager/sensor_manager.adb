with Sensor_Manager_Types; use Sensor_Manager_Types;
with Sensor_Manager_Trig;  use Sensor_Manager_Trig;

package body Sensor_Manager with SPARK_Mode is

   use Ada.Containers;

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
     with
       Always_Terminates,
       Post =>

         --  Only the five geometry fields are computed; every other
         --  field — in particular the correlation fields set by
         --  Process_Request (P2 support) — is untouched

         FP = (FP'Old with delta
                 SlantRangeToCenter       => FP.SlantRangeToCenter,
                 HorizontalToCenter       => FP.HorizontalToCenter,
                 HorizontalToLeadingEdge  => FP.HorizontalToLeadingEdge,
                 HorizontalToTrailingEdge => FP.HorizontalToTrailingEdge,
                 WidthCenter              => FP.WidthCenter)

         --  The geometry fields are Real32 images of their constrained
         --  Real64 working values (P4 support)

         and then FP.SlantRangeToCenter
           = Real32 (Slant_Range (Altitude, Elev))
         and then FP.HorizontalToCenter
           in Real32 (Center_Distance_M'First)
              .. Real32 (Center_Distance_M'Last)
         and then FP.HorizontalToLeadingEdge
           in Real32 (Edge_Distance_M'First)
              .. Real32 (Edge_Distance_M'Last)
         and then FP.HorizontalToTrailingEdge
           in Real32 (Edge_Distance_M'First)
              .. Real32 (Edge_Distance_M'Last)
         and then FP.WidthCenter in 0.0 .. Real32 (Width_M'Last);

   procedure Find_Sensor_Footprint
     (Entity_Cfg          : EntityConfig;
      Eligible_Wavelength : WavelengthBandEnum;
      Desired_GSD_Wire    : Real32;
      Altitude_Wire       : Real32;
      Elevation_Wire      : Real32;
      FP                  : in out SensorFootprint_Msg)
     with
       Always_Terminates,
       Post =>

         --  The selection and geometry fields are computed here, but the
         --  correlation fields set by Process_Request are untouched (P2
         --  support)

         FP.FootprintResponseID = FP.FootprintResponseID'Old
         and then FP.VehicleID = FP.VehicleID'Old

         --  Either no candidate was accepted and FP is untouched, or the
         --  selection and geometry fields lie within the Real32 images of
         --  their working subtypes (P4 support), the commanded elevation
         --  traces to the gimbal the footprint names under the request's
         --  elevation override (P6 support), and the selected camera
         --  traces to a mounted, eligible camera with a valid FOV
         --  candidate (P5 support)

         and then (FP = FP'Old
                   or else (Footprint_In_Wire_Ranges (FP)
                            and then Footprint_Elevation_Traceable
                                       (FP, Entity_Cfg, Elevation_Wire)
                            and then Footprint_Camera_Traceable
                                       (FP, Entity_Cfg,
                                        Eligible_Wavelength)))

         --  Without a valid effective altitude no candidate is even
         --  evaluated and FP is untouched (P8 support); with one,
         --  either the candidate set is empty and FP is untouched, or
         --  FP's selection attains the global GSD minimum over all
         --  candidates, jointly witnessed by one (gimbal, sweep step,
         --  camera, FOV) tuple (P7)

         and then
           (if not Effective_Altitude
                     (Altitude_Wire, Entity_Cfg.NominalAltitude).Valid
            then FP = FP'Old)
         and then
           (if Effective_Altitude
                 (Altitude_Wire, Entity_Cfg.NominalAltitude).Valid
            then
              (FP = FP'Old
               and then No_Candidate
                          (Entity_Cfg, Eligible_Wavelength,
                           Elevation_Wire,
                           Effective_Altitude
                             (Altitude_Wire,
                              Entity_Cfg.NominalAltitude).Value,
                           Effective_Desired_GSD (Desired_GSD_Wire)))
              or else Footprint_GSD_Optimal
                        (FP, Entity_Cfg, Eligible_Wavelength,
                         Effective_Desired_GSD (Desired_GSD_Wire),
                         Effective_Altitude
                           (Altitude_Wire,
                            Entity_Cfg.NominalAltitude).Value,
                         Elevation_Wire));

   procedure Lemma_Expected_Monotonic
     (State : Sensor_Manager_State;
      Msg   : SensorFootprintRequests_Msg;
      J     : Natural)
     with
       Ghost,
       Always_Terminates,
       Pre  => J <= Last (Msg.Footprints),
       Post =>
         (for all K in 0 .. J =>
            Expected (State, Msg, K) <= Expected (State, Msg, J)),
       Subprogram_Variant => (Decreases => J);
   --  Expected is monotonic in its request-count argument: every request
   --  contributes a non-negative footprint count. Needed by the positional-
   --  correlation invariant of Build_Response, whose earlier segments must
   --  be known to lie below the footprints appended for the current
   --  request (P3 support).
   --  @param State Service state supplying the entity configurations
   --  @param Msg The batch of footprint requests being counted
   --  @param J Number of leading requests of Msg counted

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
      function Guarded_Ratio (Denom : Real64) return Edge_Distance_M is
        (if abs Denom < Comparison_Tolerance then 0.0
         else Real64 (Altitude) / Denom);

      Horiz_FOV_Rad : constant Real64 := Real64 (To_Radians (Horiz_FOV));
      Half_FOV_Rad  : constant Real64 := 0.5 * Horiz_FOV_Rad;
      Neg_Elev      : constant Real64 := -Real64 (Elev);

      --  Aspect_Ratio_T is bounded away from zero, so the C++ guard
      --  against a zero aspect ratio is not needed.
      Vert_FOV : constant Real64 := Horiz_FOV_Rad / Real64 (Aspect_Ratio);

      --  Footprint edge angles: the boresight elevation widened by half
      --  the vertical FOV, clamped to [-Pi, 0] as in C++.
      Gimbal_Max : constant Real64 :=
        Real64'Max (-Pi, Real64'Min (0.0, Real64 (Elev) + Vert_FOV / 2.0));
      Gimbal_Min : constant Real64 :=
        Real64'Max (-Pi, Real64'Min (0.0, Real64 (Elev) - Vert_FOV / 2.0));

      Slant_To_Center : constant Slant_Range_M :=
        Slant_Range (Altitude, Elev);

      Horiz_Center   : Center_Distance_M;
      Horiz_Leading  : Edge_Distance_M;
      Horiz_Trailing : Edge_Distance_M;
      Width_Center   : Width_M;
   begin
      --  Over the working elevation range Sin and Tan of -Elev are
      --  bounded away from zero, so center distances need no guard.
      Axiom_Tan_Magnitude_On_Working_Range (Neg_Elev);
      Horiz_Center := Real64 (Altitude) / Tan (Neg_Elev);

      Horiz_Leading  := Guarded_Ratio (Tan (-Gimbal_Max));
      Horiz_Trailing := Guarded_Ratio (Tan (-Gimbal_Min));

      Axiom_Tan_Bounds_Below_Vertical (Half_FOV_Rad);
      declare
         Tan_Half_FOV : constant Real64 := Tan (Half_FOV_Rad);
         Two_Slant    : constant Real64 :=
           2.0 * Real64 (Slant_To_Center);
      begin
         pragma Assert (Two_Slant in 0.0 .. 2.0 * Slant_Range_M'Last);
         Width_Center := Two_Slant * Tan_Half_FOV;
      end;

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
      FP                  : in out SensorFootprint_Msg)
   is
      Altitude : constant Altitude_Result :=
        Effective_Altitude (Altitude_Wire, Entity_Cfg.NominalAltitude);

      Acceptable_GSD : constant Desired_GSD_M :=
        Effective_Desired_GSD (Desired_GSD_Wire);

      First_GSD_Found : Boolean := False;

      --  Best GSD so far, tracked in full precision: FP.AchievedGSD is
      --  Real32 on the wire, and comparing a fresh Real64 candidate
      --  against the rounded stored value would let exactly-tied
      --  candidates win by a rounding ulp (ties must keep the first
      --  candidate, as in C++).
      Best_GSD : Achieved_GSD_M := 0.0;

      --  Selection witness for P7: the (gimbal, sweep step, camera,
      --  FOV) indices of the candidate currently held by FP, assigned
      --  by Consider_Candidate on acceptance and meaningful only while
      --  First_GSD_Found is up. The WITNESS clause of every contract
      --  and loop invariant of the chain states that this tuple is a
      --  candidate and jointly explains FP's selection fields and
      --  Best_GSD.

      Sel_K  : Positive := 1 with Ghost;
      Sel_S  : Natural  := 0 with Ghost;
      Sel_CJ : Positive := 1 with Ghost;
      Sel_FI : Positive := 1 with Ghost;

      --  What the Evaluate_* chain guarantees, keyed by First_GSD_Found
      --  ("a candidate has been accepted into FP") in every contract and
      --  loop invariant below: the flag is monotone, FP is untouched
      --  until it first rises, and while it is up FP is within its
      --  wire-level ranges (P4), with a commanded elevation traceable to
      --  the gimbal it names under the request's override (P6), and with
      --  a camera selection traceable to a mounted, eligible camera with
      --  a valid FOV candidate (P5). The predicates are hidden by
      --  default; keying on the flag rather than on FP = FP'Old lets the
      --  hidden atoms transfer over calls syntactically instead of
      --  across floating-point record equality, which opaque atoms
      --  cannot cross.

      --  Record the candidate if it beats the best GSD found so far. The
      --  postconditions (shared by the whole Evaluate_* chain) are that
      --  the correlation fields of FP are never touched (P2 support) and
      --  that FP is either untouched or within its wire-level ranges (P4
      --  support). The candidate is designated by its (gimbal, sweep
      --  step, camera, FOV) indices, and the precondition ties Elev, FOV
      --  and Slant to that tuple's own values, so the achieved GSD
      --  computed here is definitionally Candidate_GSD of the tuple and
      --  can be related to the slant range recomputed by
      --  Calculate_Sensor_Footprint. On return the first candidate has
      --  always been accepted, the best GSD dominates both this
      --  candidate and the entry best (P7 support).

      procedure Consider_Candidate
        (Camera_Index : Positive;
         Gimbal_Index : Positive;
         Step         : Natural;
         FOV_Index    : Positive;
         Elev         : Working_Elevation_Rad;
         Slant        : Slant_Range_M;
         FOV          : FOV_Deg)
        with
          Always_Terminates,
          Pre  =>

            --  The tuple's range and validity conjuncts, restated
            --  explicitly (and ahead of the partial calls below)
            --  because Is_Candidate is hidden and cannot guard them

            Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then Camera_Index <= Last (Entity_Cfg.Cameras)
              and then Sweep_Of
                         (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                          Elevation_Wire).Valid
              and then Step < Sweep_Step_Count
                                (Sweep_Of
                                   (Get (Entity_Cfg.Gimbals,
                                         Gimbal_Index),
                                    Elevation_Wire))
              and then FOV_Candidate_Valid
                         (Get (Entity_Cfg.Cameras, Camera_Index),
                          FOV_Index)
              and then Is_Candidate
                         (Entity_Cfg, Eligible_Wavelength,
                          Elevation_Wire, Gimbal_Index, Step,
                          Camera_Index, FOV_Index)

              --  Elev, FOV and Slant are the tuple's own values

              and then Elev
                = Sweep_Elevation
                    (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                               Elevation_Wire),
                     Step)
              and then FOV
                = Camera_FOV_At
                    (Get (Entity_Cfg.Cameras, Camera_Index), FOV_Index)
              and then Slant = Slant_Range (Altitude.Value, Elev)

              --  P5/P6 provenance of the candidate

              and then Elevation_Of_Gimbal
                         (Entity_Cfg, Elevation_Wire,
                          Get (Entity_Cfg.Gimbals, Gimbal_Index)
                            .PayloadID,
                          Elev)
              and then Camera_On_Gimbal
                         (Entity_Cfg,
                          Get (Entity_Cfg.Gimbals, Gimbal_Index)
                            .PayloadID,
                          Camera_Index)
              and then (Get (Entity_Cfg.Cameras, Camera_Index)
                          .SupportedWavelengthBand
                            = Eligible_Wavelength
                        or else Eligible_Wavelength = AllAny)
              and then Valid_FOV_Of
                         (Get (Entity_Cfg.Cameras, Camera_Index),
                          FOV)
              and then
                (if First_GSD_Found
                 then Footprint_In_Wire_Ranges (FP)
                   and then Footprint_Elevation_Traceable
                              (FP, Entity_Cfg, Elevation_Wire)
                   and then Footprint_Camera_Traceable
                              (FP, Entity_Cfg, Eligible_Wavelength))

              --  WITNESS: the current selection is explained by the
              --  Sel_* tuple (with its own restated validity conjuncts,
              --  as above)

              and then
                (if First_GSD_Found
                 then Sel_K <= Last (Entity_Cfg.Gimbals)
                   and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                   and then Sweep_Of
                              (Get (Entity_Cfg.Gimbals, Sel_K),
                               Elevation_Wire).Valid
                   and then Sel_S < Sweep_Step_Count
                                      (Sweep_Of
                                         (Get (Entity_Cfg.Gimbals,
                                               Sel_K),
                                          Elevation_Wire))
                   and then FOV_Candidate_Valid
                              (Get (Entity_Cfg.Cameras, Sel_CJ),
                               Sel_FI)
                   and then Is_Candidate
                              (Entity_Cfg, Eligible_Wavelength,
                               Elevation_Wire, Sel_K, Sel_S, Sel_CJ,
                               Sel_FI)
                   and then Selection_Witness
                              (FP, Entity_Cfg, Elevation_Wire,
                               Altitude.Value, Sel_K, Sel_S, Sel_CJ,
                               Sel_FI, Best_GSD)),
          Post => FP.FootprintResponseID = FP.FootprintResponseID'Old
                    and then FP.VehicleID = FP.VehicleID'Old
                    and then (if First_GSD_Found'Old then First_GSD_Found)
                    and then (if not First_GSD_Found then FP = FP'Old)
                    and then
                      (if First_GSD_Found
                       then Footprint_In_Wire_Ranges (FP)
                         and then Footprint_Elevation_Traceable
                                    (FP, Entity_Cfg, Elevation_Wire)
                         and then Footprint_Camera_Traceable
                                    (FP, Entity_Cfg, Eligible_Wavelength))

                    --  P7: the first candidate is always accepted, the
                    --  best GSD dominates this candidate, and it
                    --  dominates the entry best (MONO)

                    and then First_GSD_Found
                    and then Dominates
                               (Acceptable_GSD, Best_GSD,
                                Candidate_GSD
                                  (Entity_Cfg, Altitude.Value,
                                   Elevation_Wire, Gimbal_Index, Step,
                                   Camera_Index, FOV_Index))
                    and then (if First_GSD_Found'Old
                              then Dominates
                                     (Acceptable_GSD, Best_GSD,
                                      Best_GSD'Old))

                    --  WITNESS: the selection is explained by the
                    --  (possibly updated) Sel_* tuple

                    and then
                      (if First_GSD_Found
                       then Sel_K <= Last (Entity_Cfg.Gimbals)
                         and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                         and then Sweep_Of
                                    (Get (Entity_Cfg.Gimbals, Sel_K),
                                     Elevation_Wire).Valid
                         and then Sel_S < Sweep_Step_Count
                                            (Sweep_Of
                                               (Get (Entity_Cfg.Gimbals,
                                                     Sel_K),
                                                Elevation_Wire))
                         and then FOV_Candidate_Valid
                                    (Get (Entity_Cfg.Cameras, Sel_CJ),
                                     Sel_FI)
                         and then Is_Candidate
                                    (Entity_Cfg, Eligible_Wavelength,
                                     Elevation_Wire, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI)
                         and then Selection_Witness
                                    (FP, Entity_Cfg, Elevation_Wire,
                                     Altitude.Value, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI, Best_GSD));

      --  Enumerate the valid FOV candidates of one camera at one
      --  boresight elevation. On return every FOV candidate of the
      --  camera at this slant range is covered: it forced acceptance and
      --  is dominated by the best GSD (P7 support).

      procedure Evaluate_Camera
        (Camera_Index : Positive;
         Gimbal_Index : Positive;
         Step         : Natural;
         Elev         : Working_Elevation_Rad;
         Slant        : Slant_Range_M)
        with
          Always_Terminates,
          Pre  =>

            --  The (gimbal, step, camera) scope of this call, restated
            --  explicitly because Is_Candidate is hidden (P7)

            Gimbal_Index <= Last (Entity_Cfg.Gimbals)
              and then Camera_Index <= Last (Entity_Cfg.Cameras)
              and then Sweep_Of
                         (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                          Elevation_Wire).Valid
              and then Step < Sweep_Step_Count
                                (Sweep_Of
                                   (Get (Entity_Cfg.Gimbals,
                                         Gimbal_Index),
                                    Elevation_Wire))
              and then Elev
                = Sweep_Elevation
                    (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                               Elevation_Wire),
                     Step)
              and then Slant = Slant_Range (Altitude.Value, Elev)
              and then Mounted_On
                         (Entity_Cfg, Gimbal_Index, Camera_Index)

              --  P5/P6 provenance (existing chain)

              and then Elevation_Of_Gimbal
                         (Entity_Cfg, Elevation_Wire,
                          Get (Entity_Cfg.Gimbals, Gimbal_Index)
                            .PayloadID,
                          Elev)
              and then Camera_On_Gimbal
                         (Entity_Cfg,
                          Get (Entity_Cfg.Gimbals, Gimbal_Index)
                            .PayloadID,
                          Camera_Index)
              and then (Get (Entity_Cfg.Cameras, Camera_Index)
                          .SupportedWavelengthBand
                            = Eligible_Wavelength
                        or else Eligible_Wavelength = AllAny)
              and then
                (if First_GSD_Found
                 then Footprint_In_Wire_Ranges (FP)
                   and then Footprint_Elevation_Traceable
                              (FP, Entity_Cfg, Elevation_Wire)
                   and then Footprint_Camera_Traceable
                              (FP, Entity_Cfg, Eligible_Wavelength))

              --  WITNESS

              and then
                (if First_GSD_Found
                 then Sel_K <= Last (Entity_Cfg.Gimbals)
                   and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                   and then Sweep_Of
                              (Get (Entity_Cfg.Gimbals, Sel_K),
                               Elevation_Wire).Valid
                   and then Sel_S < Sweep_Step_Count
                                      (Sweep_Of
                                         (Get (Entity_Cfg.Gimbals,
                                               Sel_K),
                                          Elevation_Wire))
                   and then FOV_Candidate_Valid
                              (Get (Entity_Cfg.Cameras, Sel_CJ),
                               Sel_FI)
                   and then Is_Candidate
                              (Entity_Cfg, Eligible_Wavelength,
                               Elevation_Wire, Sel_K, Sel_S, Sel_CJ,
                               Sel_FI)
                   and then Selection_Witness
                              (FP, Entity_Cfg, Elevation_Wire,
                               Altitude.Value, Sel_K, Sel_S, Sel_CJ,
                               Sel_FI, Best_GSD)),
          Post => FP.FootprintResponseID = FP.FootprintResponseID'Old
                    and then FP.VehicleID = FP.VehicleID'Old
                    and then (if First_GSD_Found'Old then First_GSD_Found)
                    and then (if not First_GSD_Found then FP = FP'Old)
                    and then
                      (if First_GSD_Found
                       then Footprint_In_Wire_Ranges (FP)
                         and then Footprint_Elevation_Traceable
                                    (FP, Entity_Cfg, Elevation_Wire)
                         and then Footprint_Camera_Traceable
                                    (FP, Entity_Cfg, Eligible_Wavelength))

                    --  P7: the camera's whole candidate set is covered,
                    --  and the best GSD dominates the entry best (MONO)

                    and then Camera_Covered
                               (Get (Entity_Cfg.Cameras, Camera_Index),
                                Slant, Acceptable_GSD, First_GSD_Found,
                                Best_GSD)
                    and then (if First_GSD_Found'Old
                              then Dominates
                                     (Acceptable_GSD, Best_GSD,
                                      Best_GSD'Old))

                    --  WITNESS

                    and then
                      (if First_GSD_Found
                       then Sel_K <= Last (Entity_Cfg.Gimbals)
                         and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                         and then Sweep_Of
                                    (Get (Entity_Cfg.Gimbals, Sel_K),
                                     Elevation_Wire).Valid
                         and then Sel_S < Sweep_Step_Count
                                            (Sweep_Of
                                               (Get (Entity_Cfg.Gimbals,
                                                     Sel_K),
                                                Elevation_Wire))
                         and then FOV_Candidate_Valid
                                    (Get (Entity_Cfg.Cameras, Sel_CJ),
                                     Sel_FI)
                         and then Is_Candidate
                                    (Entity_Cfg, Eligible_Wavelength,
                                     Elevation_Wire, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI)
                         and then Selection_Witness
                                    (FP, Entity_Cfg, Elevation_Wire,
                                     Altitude.Value, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI, Best_GSD));

      --  Introduction lemma for the empty Camera_Covered_Upto prefix:
      --  a covered range with no indices holds trivially, but the
      --  predicate is hidden, so even the trivial base case must be
      --  introduced where the definition is disclosed (P7 support).
      --  Declared here (not in Sensor_Manager_Types) because it is a
      --  proof-local artifact of the loop structure below.

      procedure Lemma_Camera_Covered_Empty
        (Camera  : CameraConfig;
         Slant   : Slant_Range_M;
         Desired : Desired_GSD_M;
         Found   : Boolean;
         Best    : Achieved_GSD_M)
        with
          Ghost,
          Always_Terminates,
          Global => null,
          Post =>
            Camera_Covered_Upto (Camera, 0, Slant, Desired, Found, Best);

      --  Sweep one gimbal's elevation range and evaluate every eligible
      --  camera mounted on it. The gimbal is designated by its index in
      --  Entity_Cfg.Gimbals so the provenance witnesses of
      --  Elevation_Of_Gimbal are ground terms, and its sweep is computed
      --  by the caller so this subprogram's proof context carries only
      --  Sweep_Of's minimal contract, not the clamping machinery's (P6
      --  support).

      procedure Evaluate_Gimbal
        (Gimbal_Index : Positive;
         Sweep        : Elevation_Sweep)
        with
          Always_Terminates,
          Pre  => Gimbal_Index <= Last (Entity_Cfg.Gimbals)
                    and then Sweep = Sweep_Of
                                       (Get (Entity_Cfg.Gimbals,
                                             Gimbal_Index),
                                        Elevation_Wire)
                    and then Sweep.Valid
                    and then
                      (if First_GSD_Found
                       then Footprint_In_Wire_Ranges (FP)
                         and then Footprint_Elevation_Traceable
                                    (FP, Entity_Cfg, Elevation_Wire)
                         and then Footprint_Camera_Traceable
                                    (FP, Entity_Cfg, Eligible_Wavelength))

                    --  WITNESS

                    and then
                      (if First_GSD_Found
                       then Sel_K <= Last (Entity_Cfg.Gimbals)
                         and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                         and then Sweep_Of
                                    (Get (Entity_Cfg.Gimbals, Sel_K),
                                     Elevation_Wire).Valid
                         and then Sel_S < Sweep_Step_Count
                                            (Sweep_Of
                                               (Get (Entity_Cfg.Gimbals,
                                                     Sel_K),
                                                Elevation_Wire))
                         and then FOV_Candidate_Valid
                                    (Get (Entity_Cfg.Cameras, Sel_CJ),
                                     Sel_FI)
                         and then Is_Candidate
                                    (Entity_Cfg, Eligible_Wavelength,
                                     Elevation_Wire, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI)
                         and then Selection_Witness
                                    (FP, Entity_Cfg, Elevation_Wire,
                                     Altitude.Value, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI, Best_GSD)),
          Post => FP.FootprintResponseID = FP.FootprintResponseID'Old
                    and then FP.VehicleID = FP.VehicleID'Old
                    and then (if First_GSD_Found'Old then First_GSD_Found)
                    and then (if not First_GSD_Found then FP = FP'Old)
                    and then
                      (if First_GSD_Found
                       then Footprint_In_Wire_Ranges (FP)
                         and then Footprint_Elevation_Traceable
                                    (FP, Entity_Cfg, Elevation_Wire)
                         and then Footprint_Camera_Traceable
                                    (FP, Entity_Cfg, Eligible_Wavelength))

                    --  P7: the gimbal's whole candidate set is covered,
                    --  and the best GSD dominates the entry best (MONO)

                    and then Gimbal_Covered
                               (Entity_Cfg, Eligible_Wavelength,
                                Elevation_Wire, Altitude.Value,
                                Gimbal_Index, Acceptable_GSD,
                                First_GSD_Found, Best_GSD)
                    and then (if First_GSD_Found'Old
                              then Dominates
                                     (Acceptable_GSD, Best_GSD,
                                      Best_GSD'Old))

                    --  WITNESS

                    and then
                      (if First_GSD_Found
                       then Sel_K <= Last (Entity_Cfg.Gimbals)
                         and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                         and then Sweep_Of
                                    (Get (Entity_Cfg.Gimbals, Sel_K),
                                     Elevation_Wire).Valid
                         and then Sel_S < Sweep_Step_Count
                                            (Sweep_Of
                                               (Get (Entity_Cfg.Gimbals,
                                                     Sel_K),
                                                Elevation_Wire))
                         and then FOV_Candidate_Valid
                                    (Get (Entity_Cfg.Cameras, Sel_CJ),
                                     Sel_FI)
                         and then Is_Candidate
                                    (Entity_Cfg, Eligible_Wavelength,
                                     Elevation_Wire, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI)
                         and then Selection_Witness
                                    (FP, Entity_Cfg, Elevation_Wire,
                                     Altitude.Value, Sel_K, Sel_S,
                                     Sel_CJ, Sel_FI, Best_GSD));

      --------------------------------
      -- Lemma_Camera_Covered_Empty --
      --------------------------------

      procedure Lemma_Camera_Covered_Empty
        (Camera  : CameraConfig;
         Slant   : Slant_Range_M;
         Desired : Desired_GSD_M;
         Found   : Boolean;
         Best    : Achieved_GSD_M)
      is null;
      --  The predicate is hidden by default; disclose it here, where
      --  its trivial base case is proved.
      pragma Annotate (GNATprove, Unhide_Info, "Expression_Function_Body",
                       Camera_Covered_Upto);

      ------------------------
      -- Consider_Candidate --
      ------------------------

      procedure Consider_Candidate
        (Camera_Index : Positive;
         Gimbal_Index : Positive;
         Step         : Natural;
         FOV_Index    : Positive;
         Elev         : Working_Elevation_Rad;
         Slant        : Slant_Range_M;
         FOV          : FOV_Deg)
      is
         --  This is where an accepted footprint's P4/P5/P6 properties
         --  and its P7 selection witness are established, so the
         --  default-hidden predicates are disclosed. Camera_On_Gimbal
         --  stays hidden: it transfers from the precondition as an
         --  opaque atom (its Camera_Index and gimbal payload ID
         --  arguments are integers).
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Footprint_In_Wire_Ranges);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Footprint_Elevation_Traceable);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Elevation_Of_Gimbal);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Footprint_Camera_Traceable);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Camera_FOV_Image);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Valid_FOV_Of);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Is_Candidate);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Candidate_GSD);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Selection_Witness);

         Camera    : constant CameraConfig :=
           Get (Entity_Cfg.Cameras, Camera_Index);
         Gimbal_ID : constant Int64 :=
           Get (Entity_Cfg.Gimbals, Gimbal_Index).PayloadID;
         GSD       : constant Achieved_GSD_M :=
           Compute_GSD (Slant, FOV, Min_Resolution (Camera));

         --  Entry snapshots for the per-branch MONO stepping stones

         Found_Entry : constant Boolean := First_GSD_Found with Ghost;
         Best_Entry  : constant Achieved_GSD_M := Best_GSD with Ghost;
      begin
         --  The GSD computed here is definitionally the candidate
         --  tuple's: the precondition pins Elev, FOV and Slant to the
         --  tuple's own values. The Dominates form is established here,
         --  before the branch, where the proof context is light: after
         --  the branch only Dominates transitivity is needed (P7
         --  support)

         pragma Assert
           (GSD = Candidate_GSD
                    (Entity_Cfg, Altitude.Value, Elevation_Wire,
                     Gimbal_Index, Step, Camera_Index, FOV_Index));
         pragma Assert
           (Dominates
              (Acceptable_GSD, GSD,
               Candidate_GSD
                 (Entity_Cfg, Altitude.Value, Elevation_Wire,
                  Gimbal_Index, Step, Camera_Index, FOV_Index)));

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

            --  Record the accepted tuple as the selection witness (P7)

            Sel_K  := Gimbal_Index;
            Sel_S  := Step;
            Sel_CJ := Camera_Index;
            Sel_FI := FOV_Index;

            Calculate_Sensor_Footprint
              (FP           => FP,
               Altitude     => Altitude.Value,
               Elev         => Elev,
               Horiz_FOV    => FOV,
               Aspect_Ratio => Aspect_Ratio_Of (Camera));

            --  Stepping stones for the postcondition: the freshly
            --  accepted tuple explains the freshly assigned selection
            --  fields (Calculate_Sensor_Footprint touches only the
            --  geometry fields), and the new best dominates the
            --  candidate by reflexivity through the pre-branch
            --  assertion

            pragma Assert
              (Selection_Witness
                 (FP, Entity_Cfg, Elevation_Wire, Altitude.Value,
                  Sel_K, Sel_S, Sel_CJ, Sel_FI, Best_GSD));
            pragma Assert
              (Dominates
                 (Acceptable_GSD, Best_GSD,
                  Candidate_GSD
                    (Entity_Cfg, Altitude.Value, Elevation_Wire,
                     Gimbal_Index, Step, Camera_Index, FOV_Index)));
            pragma Assert
              (if Found_Entry
               then Dominates (Acceptable_GSD, Best_GSD, Best_Entry));
         else
            --  Stepping stone for the postcondition: the rejected
            --  candidate is dominated by the standing best (the failed
            --  Is_Better comparison), hence so is the candidate's GSD
            --  by transitivity through the pre-branch assertion

            pragma Assert
              (Dominates
                 (Acceptable_GSD, Best_GSD,
                  Candidate_GSD
                    (Entity_Cfg, Altitude.Value, Elevation_Wire,
                     Gimbal_Index, Step, Camera_Index, FOV_Index)));
            pragma Assert
              (if Found_Entry
               then Dominates (Acceptable_GSD, Best_GSD, Best_Entry));
         end if;

         pragma Assert (First_GSD_Found);
      end Consider_Candidate;

      ---------------------
      -- Evaluate_Camera --
      ---------------------

      procedure Evaluate_Camera
        (Camera_Index : Positive;
         Gimbal_Index : Positive;
         Step         : Natural;
         Elev         : Working_Elevation_Rad;
         Slant        : Slant_Range_M)
      is
         --  This subprogram passes the P4/P5/P6 footprint predicates
         --  along (precondition to precondition, postcondition to
         --  postcondition); they stay hidden, so the transfers are
         --  propositional on the opaque atoms. The FOV provenance of
         --  each candidate is established via the Valid_FOV_Of intro
         --  lemmas (P5 support). For P7, Is_Candidate and Candidate_GSD
         --  are disclosed: the candidate tuple handed to
         --  Consider_Candidate is assembled from this subprogram's own
         --  precondition, and the candidate's GSD (opaque in
         --  Consider_Candidate's postcondition) is rewritten to the
         --  Compute_GSD form the coverage lemmas expect. The coverage
         --  predicate itself stays hidden and is threaded through the
         --  Extend/Monotone lemmas.
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Is_Candidate);
         pragma Annotate (GNATprove, Unhide_Info,
                          "Expression_Function_Body",
                          Candidate_GSD);

         Camera : constant CameraConfig :=
           Get (Entity_Cfg.Cameras, Camera_Index);
      begin
         --  Rewrite the slant range to the candidate tuple's own terms
         --  (the precondition pins Elev to the tuple's sweep step), and
         --  start the covered prefix empty

         pragma Assert
           (Slant
              = Slant_Range
                  (Altitude.Value,
                   Sweep_Elevation
                     (Sweep_Of (Get (Entity_Cfg.Gimbals, Gimbal_Index),
                                Elevation_Wire),
                      Step)));
         Lemma_Camera_Covered_Empty
           (Camera, Slant, Acceptable_GSD, First_GSD_Found, Best_GSD);

         if Camera.FieldOfViewMode = Continuous then
            declare
               Candidates : constant FOV_Candidates :=
                 Continuous_Candidates
                   (Camera.MinHorizontalFOV, Camera.MaxHorizontalFOV);
            begin
               for Index in 0 .. Candidates.Count - 1 loop
                  pragma Loop_Invariant
                    (FP.FootprintResponseID
                       = FP.FootprintResponseID'Loop_Entry
                     and then FP.VehicleID = FP.VehicleID'Loop_Entry
                     and then (if First_GSD_Found'Loop_Entry
                               then First_GSD_Found)
                     and then (if not First_GSD_Found
                               then FP = FP'Loop_Entry)
                     and then
                       (if First_GSD_Found
                        then Footprint_In_Wire_Ranges (FP)
                          and then Footprint_Elevation_Traceable
                                     (FP, Entity_Cfg, Elevation_Wire)
                          and then Footprint_Camera_Traceable
                                     (FP, Entity_Cfg,
                                      Eligible_Wavelength))

                     --  P7: the processed FOV prefix is covered, and the
                     --  best GSD dominates the loop-entry best (MONO)

                     and then Camera_Covered_Upto
                                (Camera, Index, Slant, Acceptable_GSD,
                                 First_GSD_Found, Best_GSD)
                     and then (if First_GSD_Found'Loop_Entry
                               then Dominates
                                      (Acceptable_GSD, Best_GSD,
                                       Best_GSD'Loop_Entry))

                     --  WITNESS

                     and then
                       (if First_GSD_Found
                        then Sel_K <= Last (Entity_Cfg.Gimbals)
                          and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                          and then Sweep_Of
                                     (Get (Entity_Cfg.Gimbals, Sel_K),
                                      Elevation_Wire).Valid
                          and then Sel_S
                            < Sweep_Step_Count
                                (Sweep_Of
                                   (Get (Entity_Cfg.Gimbals, Sel_K),
                                    Elevation_Wire))
                          and then FOV_Candidate_Valid
                                     (Get (Entity_Cfg.Cameras, Sel_CJ),
                                      Sel_FI)
                          and then Is_Candidate
                                     (Entity_Cfg, Eligible_Wavelength,
                                      Elevation_Wire, Sel_K, Sel_S,
                                      Sel_CJ, Sel_FI)
                          and then Selection_Witness
                                     (FP, Entity_Cfg, Elevation_Wire,
                                      Altitude.Value, Sel_K, Sel_S,
                                      Sel_CJ, Sel_FI, Best_GSD)));
                  declare
                     Found_Prev : constant Boolean := First_GSD_Found
                       with Ghost;
                     Best_Prev  : constant Achieved_GSD_M := Best_GSD
                       with Ghost;
                  begin
                     Lemma_Valid_FOV_Continuous_Intro (Camera, Index);
                     Consider_Candidate
                       (Camera_Index => Camera_Index,
                        Gimbal_Index => Gimbal_Index,
                        Step         => Step,
                        FOV_Index    => Index + 1,
                        Elev         => Elev,
                        Slant        => Slant,
                        FOV          => Candidate_FOV (Candidates,
                                                       Index));

                     --  Rewrite the accepted obligation to the coverage
                     --  lemma's Compute_GSD form, carry the covered
                     --  prefix across the possible Best improvement,
                     --  then extend it by this candidate

                     pragma Assert
                       (Candidate_GSD
                          (Entity_Cfg, Altitude.Value, Elevation_Wire,
                           Gimbal_Index, Step, Camera_Index, Index + 1)
                          = Compute_GSD
                              (Slant,
                               Camera_FOV_At (Camera, Index + 1),
                               Min_Resolution (Camera)));
                     Lemma_Camera_Covered_Monotone
                       (Camera, Index, Slant, Acceptable_GSD,
                        Found_Prev, Best_Prev,
                        First_GSD_Found, Best_GSD);
                     Lemma_Camera_Covered_Extend
                       (Camera, Index + 1, Slant, Acceptable_GSD,
                        First_GSD_Found, Best_GSD);
                  end;
               end loop;
            end;
         else
            --  Discrete mode: entries outside (0, 179] degrees are
            --  skipped (D5) and extend the covered prefix vacuously.
            --  Indexed rather than element iteration, so the FOV
            --  provenance witness is a ground term (P5).
            for DI in 1 .. Last (Camera.DiscreteHFOVList) loop
               pragma Loop_Invariant
                 (FP.FootprintResponseID = FP.FootprintResponseID'Loop_Entry
                  and then FP.VehicleID = FP.VehicleID'Loop_Entry
                  and then (if First_GSD_Found'Loop_Entry
                            then First_GSD_Found)
                  and then (if not First_GSD_Found
                            then FP = FP'Loop_Entry)
                  and then
                    (if First_GSD_Found
                     then Footprint_In_Wire_Ranges (FP)
                       and then Footprint_Elevation_Traceable
                                  (FP, Entity_Cfg, Elevation_Wire)
                       and then Footprint_Camera_Traceable
                                  (FP, Entity_Cfg, Eligible_Wavelength))

                  --  P7: the processed FOV prefix is covered, and the
                  --  best GSD dominates the loop-entry best (MONO)

                  and then Camera_Covered_Upto
                             (Camera, DI - 1, Slant, Acceptable_GSD,
                              First_GSD_Found, Best_GSD)
                  and then (if First_GSD_Found'Loop_Entry
                            then Dominates
                                   (Acceptable_GSD, Best_GSD,
                                    Best_GSD'Loop_Entry))

                  --  WITNESS

                  and then
                    (if First_GSD_Found
                     then Sel_K <= Last (Entity_Cfg.Gimbals)
                       and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                       and then Sweep_Of
                                  (Get (Entity_Cfg.Gimbals, Sel_K),
                                   Elevation_Wire).Valid
                       and then Sel_S
                         < Sweep_Step_Count
                             (Sweep_Of
                                (Get (Entity_Cfg.Gimbals, Sel_K),
                                 Elevation_Wire))
                       and then FOV_Candidate_Valid
                                  (Get (Entity_Cfg.Cameras, Sel_CJ),
                                   Sel_FI)
                       and then Is_Candidate
                                  (Entity_Cfg, Eligible_Wavelength,
                                   Elevation_Wire, Sel_K, Sel_S,
                                   Sel_CJ, Sel_FI)
                       and then Selection_Witness
                                  (FP, Entity_Cfg, Elevation_Wire,
                                   Altitude.Value, Sel_K, Sel_S,
                                   Sel_CJ, Sel_FI, Best_GSD)));
               if Is_Valid_FOV (Get (Camera.DiscreteHFOVList, DI)) then
                  declare
                     Found_Prev : constant Boolean := First_GSD_Found
                       with Ghost;
                     Best_Prev  : constant Achieved_GSD_M := Best_GSD
                       with Ghost;
                  begin
                     Lemma_Valid_FOV_Discrete_Intro (Camera, DI);
                     Consider_Candidate
                       (Camera_Index => Camera_Index,
                        Gimbal_Index => Gimbal_Index,
                        Step         => Step,
                        FOV_Index    => DI,
                        Elev         => Elev,
                        Slant        => Slant,
                        FOV          =>
                          FOV_Deg (Degrees_64
                                     (Get (Camera.DiscreteHFOVList,
                                           DI))));

                     --  As in the continuous branch: rewrite, carry,
                     --  extend

                     pragma Assert
                       (Candidate_GSD
                          (Entity_Cfg, Altitude.Value, Elevation_Wire,
                           Gimbal_Index, Step, Camera_Index, DI)
                          = Compute_GSD
                              (Slant,
                               Camera_FOV_At (Camera, DI),
                               Min_Resolution (Camera)));
                     Lemma_Camera_Covered_Monotone
                       (Camera, DI - 1, Slant, Acceptable_GSD,
                        Found_Prev, Best_Prev,
                        First_GSD_Found, Best_GSD);
                     Lemma_Camera_Covered_Extend
                       (Camera, DI, Slant, Acceptable_GSD,
                        First_GSD_Found, Best_GSD);
                  end;
               else
                  --  An invalid entry is not a candidate: the covered
                  --  prefix extends vacuously

                  Lemma_Camera_Covered_Extend
                    (Camera, DI, Slant, Acceptable_GSD,
                     First_GSD_Found, Best_GSD);
               end if;
            end loop;
         end if;
      end Evaluate_Camera;

      ---------------------
      -- Evaluate_Gimbal --
      ---------------------

      procedure Evaluate_Gimbal
        (Gimbal_Index : Positive;
         Sweep        : Elevation_Sweep)
      is
         --  The footprint-level predicates stay hidden in this subprogram:
         --  they are only shuttled through, and Elevation_Of_Gimbal is
         --  established via its intro lemma rather than by unfolding.

         Gimbal : constant GimbalConfig :=
           Get (Entity_Cfg.Gimbals, Gimbal_Index);
      begin
         --  Start the covered step prefix empty (P7)

         Lemma_Steps_Covered_Empty
           (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
            Altitude.Value, Gimbal_Index, Acceptable_GSD,
            First_GSD_Found, Best_GSD);

         for Step in 0 .. Sweep_Step_Count (Sweep) - 1 loop
            pragma Loop_Invariant
              (FP.FootprintResponseID = FP.FootprintResponseID'Loop_Entry
               and then FP.VehicleID = FP.VehicleID'Loop_Entry
               and then (if First_GSD_Found'Loop_Entry
                         then First_GSD_Found)
               and then (if not First_GSD_Found
                         then FP = FP'Loop_Entry)
               and then
                 (if First_GSD_Found
                  then Footprint_In_Wire_Ranges (FP)
                    and then Footprint_Elevation_Traceable
                               (FP, Entity_Cfg, Elevation_Wire)
                    and then Footprint_Camera_Traceable
                               (FP, Entity_Cfg, Eligible_Wavelength))

               --  P7: the processed step prefix is covered, and the
               --  best GSD dominates the loop-entry best (MONO)

               and then Steps_Covered_Upto
                          (Entity_Cfg, Eligible_Wavelength,
                           Elevation_Wire, Altitude.Value, Gimbal_Index,
                           Step, Acceptable_GSD, First_GSD_Found,
                           Best_GSD)
               and then (if First_GSD_Found'Loop_Entry
                         then Dominates
                                (Acceptable_GSD, Best_GSD,
                                 Best_GSD'Loop_Entry))

               --  WITNESS

               and then
                 (if First_GSD_Found
                  then Sel_K <= Last (Entity_Cfg.Gimbals)
                    and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                    and then Sweep_Of
                               (Get (Entity_Cfg.Gimbals, Sel_K),
                                Elevation_Wire).Valid
                    and then Sel_S
                      < Sweep_Step_Count
                          (Sweep_Of
                             (Get (Entity_Cfg.Gimbals, Sel_K),
                              Elevation_Wire))
                    and then FOV_Candidate_Valid
                               (Get (Entity_Cfg.Cameras, Sel_CJ),
                                Sel_FI)
                    and then Is_Candidate
                               (Entity_Cfg, Eligible_Wavelength,
                                Elevation_Wire, Sel_K, Sel_S,
                                Sel_CJ, Sel_FI)
                    and then Selection_Witness
                               (FP, Entity_Cfg, Elevation_Wire,
                                Altitude.Value, Sel_K, Sel_S,
                                Sel_CJ, Sel_FI, Best_GSD)));
            declare
               Elev  : constant Working_Elevation_Rad :=
                 Sweep_Elevation (Sweep, Step);
               Slant : constant Slant_Range_M         :=
                 Slant_Range (Altitude.Value, Elev);
            begin
               --  Establish the provenance of Elev for the calls below:
               --  Gimbal is gimbal number Gimbal_Index of Entity_Cfg,
               --  its sweep under the override is Sweep (precondition),
               --  and Elev is its sweep step Step. The intro lemma keeps
               --  the predicate's definition out of this subprogram's
               --  proof context (P6 support).

               Lemma_Elevation_Of_Gimbal_Intro
                 (Entity_Cfg, Elevation_Wire, Gimbal_Index, Step);
               pragma Assert
                 (Elevation_Of_Gimbal
                    (Entity_Cfg, Elevation_Wire, Gimbal.PayloadID, Elev));

               --  Rewrite the slant range to the candidate tuple's own
               --  terms (Sweep is the gimbal's own sweep by the
               --  precondition), and start the covered position prefix
               --  of this step empty (P7)

               pragma Assert
                 (Slant
                    = Slant_Range
                        (Altitude.Value,
                         Sweep_Elevation
                           (Sweep_Of (Get (Entity_Cfg.Gimbals,
                                           Gimbal_Index),
                                      Elevation_Wire),
                            Step)));
               Lemma_Positions_Covered_Empty
                 (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, Slant,
                  Acceptable_GSD, First_GSD_Found, Best_GSD);

               --  Indexed rather than element iteration, so the mounting
               --  witnesses of Camera_On_Gimbal are ground terms (P5)

               for CI in 1 .. Last (Gimbal.ContainedPayloadList) loop
                  pragma Loop_Invariant
                    (FP.FootprintResponseID
                       = FP.FootprintResponseID'Loop_Entry
                     and then FP.VehicleID = FP.VehicleID'Loop_Entry
                     and then (if First_GSD_Found'Loop_Entry
                               then First_GSD_Found)
                     and then (if not First_GSD_Found
                               then FP = FP'Loop_Entry)
                     and then
                       (if First_GSD_Found
                        then Footprint_In_Wire_Ranges (FP)
                          and then Footprint_Elevation_Traceable
                                     (FP, Entity_Cfg, Elevation_Wire)
                          and then Footprint_Camera_Traceable
                                     (FP, Entity_Cfg,
                                      Eligible_Wavelength))

                     --  P7: the earlier steps and the processed position
                     --  prefix of this step are covered, and the best
                     --  GSD dominates the loop-entry best (MONO)

                     and then Steps_Covered_Upto
                                (Entity_Cfg, Eligible_Wavelength,
                                 Elevation_Wire, Altitude.Value,
                                 Gimbal_Index, Step, Acceptable_GSD,
                                 First_GSD_Found, Best_GSD)
                     and then Positions_Covered_Upto
                                (Entity_Cfg, Eligible_Wavelength,
                                 Gimbal_Index, CI - 1, Slant,
                                 Acceptable_GSD, First_GSD_Found,
                                 Best_GSD)
                     and then (if First_GSD_Found'Loop_Entry
                               then Dominates
                                      (Acceptable_GSD, Best_GSD,
                                       Best_GSD'Loop_Entry))

                     --  WITNESS

                     and then
                       (if First_GSD_Found
                        then Sel_K <= Last (Entity_Cfg.Gimbals)
                          and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                          and then Sweep_Of
                                     (Get (Entity_Cfg.Gimbals, Sel_K),
                                      Elevation_Wire).Valid
                          and then Sel_S
                            < Sweep_Step_Count
                                (Sweep_Of
                                   (Get (Entity_Cfg.Gimbals, Sel_K),
                                    Elevation_Wire))
                          and then FOV_Candidate_Valid
                                     (Get (Entity_Cfg.Cameras, Sel_CJ),
                                      Sel_FI)
                          and then Is_Candidate
                                     (Entity_Cfg, Eligible_Wavelength,
                                      Elevation_Wire, Sel_K, Sel_S,
                                      Sel_CJ, Sel_FI)
                          and then Selection_Witness
                                     (FP, Entity_Cfg, Elevation_Wire,
                                      Altitude.Value, Sel_K, Sel_S,
                                      Sel_CJ, Sel_FI, Best_GSD)));

                  --  Start the covered camera prefix of position CI
                  --  empty (P7)

                  Lemma_Position_Covered_Empty
                    (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, CI,
                     Slant, Acceptable_GSD, First_GSD_Found, Best_GSD);

                  for CJ in 1 .. Last (Entity_Cfg.Cameras) loop
                     pragma Loop_Invariant
                       (FP.FootprintResponseID
                          = FP.FootprintResponseID'Loop_Entry
                        and then FP.VehicleID = FP.VehicleID'Loop_Entry
                        and then (if First_GSD_Found'Loop_Entry
                                  then First_GSD_Found)
                        and then (if not First_GSD_Found
                                  then FP = FP'Loop_Entry)
                        and then
                          (if First_GSD_Found
                           then Footprint_In_Wire_Ranges (FP)
                             and then Footprint_Elevation_Traceable
                                        (FP, Entity_Cfg, Elevation_Wire)
                             and then Footprint_Camera_Traceable
                                        (FP, Entity_Cfg,
                                         Eligible_Wavelength))

                        --  P7: the earlier steps, the earlier positions
                        --  of this step and the processed camera prefix
                        --  of position CI are covered, and the best GSD
                        --  dominates the loop-entry best (MONO)

                        and then Steps_Covered_Upto
                                   (Entity_Cfg, Eligible_Wavelength,
                                    Elevation_Wire, Altitude.Value,
                                    Gimbal_Index, Step, Acceptable_GSD,
                                    First_GSD_Found, Best_GSD)
                        and then Positions_Covered_Upto
                                   (Entity_Cfg, Eligible_Wavelength,
                                    Gimbal_Index, CI - 1, Slant,
                                    Acceptable_GSD, First_GSD_Found,
                                    Best_GSD)
                        and then Position_Covered_Upto
                                   (Entity_Cfg, Eligible_Wavelength,
                                    Gimbal_Index, CI, CJ - 1, Slant,
                                    Acceptable_GSD, First_GSD_Found,
                                    Best_GSD)
                        and then (if First_GSD_Found'Loop_Entry
                                  then Dominates
                                         (Acceptable_GSD, Best_GSD,
                                          Best_GSD'Loop_Entry))

                        --  WITNESS

                        and then
                          (if First_GSD_Found
                           then Sel_K <= Last (Entity_Cfg.Gimbals)
                             and then Sel_CJ
                               <= Last (Entity_Cfg.Cameras)
                             and then Sweep_Of
                                        (Get (Entity_Cfg.Gimbals,
                                              Sel_K),
                                         Elevation_Wire).Valid
                             and then Sel_S
                               < Sweep_Step_Count
                                   (Sweep_Of
                                      (Get (Entity_Cfg.Gimbals, Sel_K),
                                       Elevation_Wire))
                             and then FOV_Candidate_Valid
                                        (Get (Entity_Cfg.Cameras,
                                              Sel_CJ),
                                         Sel_FI)
                             and then Is_Candidate
                                        (Entity_Cfg,
                                         Eligible_Wavelength,
                                         Elevation_Wire, Sel_K, Sel_S,
                                         Sel_CJ, Sel_FI)
                             and then Selection_Witness
                                        (FP, Entity_Cfg,
                                         Elevation_Wire,
                                         Altitude.Value, Sel_K, Sel_S,
                                         Sel_CJ, Sel_FI, Best_GSD)));
                     if Get (Entity_Cfg.Cameras, CJ).PayloadID
                          = Get (Gimbal.ContainedPayloadList, CI)
                       and then
                         (Get (Entity_Cfg.Cameras, CJ)
                            .SupportedWavelengthBand = Eligible_Wavelength
                          or else Eligible_Wavelength = AllAny)
                     then
                        declare
                           Found_Prev : constant Boolean :=
                             First_GSD_Found with Ghost;
                           Best_Prev  : constant Achieved_GSD_M :=
                             Best_GSD with Ghost;
                        begin
                           Lemma_Camera_On_Gimbal_Intro
                             (Entity_Cfg, Gimbal_Index, CI, CJ);
                           Evaluate_Camera (CJ, Gimbal_Index, Step, Elev,
                                            Slant);

                           --  Carry the covered prefixes at every level
                           --  across the possible Best improvement, then
                           --  extend position CI's camera prefix by this
                           --  camera (Evaluate_Camera's postcondition)

                           Lemma_Position_Covered_Monotone
                             (Entity_Cfg, Eligible_Wavelength,
                              Gimbal_Index, CI, CJ - 1, Slant,
                              Acceptable_GSD, Found_Prev, Best_Prev,
                              First_GSD_Found, Best_GSD);
                           Lemma_Positions_Covered_Monotone
                             (Entity_Cfg, Eligible_Wavelength,
                              Gimbal_Index, CI - 1, Slant,
                              Acceptable_GSD, Found_Prev, Best_Prev,
                              First_GSD_Found, Best_GSD);
                           Lemma_Steps_Covered_Monotone
                             (Entity_Cfg, Eligible_Wavelength,
                              Elevation_Wire, Altitude.Value,
                              Gimbal_Index, Step, Acceptable_GSD,
                              Found_Prev, Best_Prev,
                              First_GSD_Found, Best_GSD);
                           Lemma_Position_Covered_Extend
                             (Entity_Cfg, Eligible_Wavelength,
                              Gimbal_Index, CI, CJ, Slant,
                              Acceptable_GSD, First_GSD_Found,
                              Best_GSD);
                        end;
                     else
                        --  A mismatched or ineligible camera holds no
                        --  candidate at this position: the covered
                        --  prefix extends vacuously

                        Lemma_Position_Covered_Extend
                          (Entity_Cfg, Eligible_Wavelength,
                           Gimbal_Index, CI, CJ, Slant,
                           Acceptable_GSD, First_GSD_Found, Best_GSD);
                     end if;
                  end loop;

                  --  Position CI is covered over the full camera list:
                  --  extend the covered position prefix by one (P7)

                  Lemma_Positions_Covered_Extend
                    (Entity_Cfg, Eligible_Wavelength, Gimbal_Index, CI,
                     Slant, Acceptable_GSD, First_GSD_Found, Best_GSD);
               end loop;

               --  This step is covered over the gimbal's full
               --  contained-payload list: extend the covered step
               --  prefix by one (P7)

               Lemma_Steps_Covered_Extend
                 (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
                  Altitude.Value, Gimbal_Index, Step + 1,
                  Acceptable_GSD, First_GSD_Found, Best_GSD);
            end;
         end loop;

         --  Every sweep step is covered: conclude the gimbal is (P7)

         Lemma_Gimbal_Covered_Intro
           (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
            Altitude.Value, Gimbal_Index, Acceptable_GSD,
            First_GSD_Found, Best_GSD);
      end Evaluate_Gimbal;

   begin
      if not Altitude.Valid then
         return;
      end if;

      --  Start the covered gimbal prefix empty (P7)

      Lemma_Gimbals_Covered_Empty
        (Entity_Cfg, Eligible_Wavelength, Elevation_Wire, Altitude.Value,
         Acceptable_GSD, First_GSD_Found, Best_GSD);

      --  Indexed rather than element iteration, so the gimbal-membership
      --  witness of Evaluate_Gimbal's precondition is nameable (P6)

      for K in 1 .. Last (Entity_Cfg.Gimbals) loop
         pragma Loop_Invariant
           (FP.FootprintResponseID = FP.FootprintResponseID'Loop_Entry
            and then FP.VehicleID = FP.VehicleID'Loop_Entry
            and then (if First_GSD_Found'Loop_Entry
                      then First_GSD_Found)
            and then (if not First_GSD_Found
                      then FP = FP'Loop_Entry)
            and then
              (if First_GSD_Found
               then Footprint_In_Wire_Ranges (FP)
                 and then Footprint_Elevation_Traceable
                            (FP, Entity_Cfg, Elevation_Wire)
                 and then Footprint_Camera_Traceable
                            (FP, Entity_Cfg, Eligible_Wavelength))

            --  P7: the processed gimbal prefix is covered, and the
            --  best GSD dominates the loop-entry best (MONO)

            and then Gimbals_Covered_Upto
                       (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
                        Altitude.Value, K - 1, Acceptable_GSD,
                        First_GSD_Found, Best_GSD)
            and then (if First_GSD_Found'Loop_Entry
                      then Dominates
                             (Acceptable_GSD, Best_GSD,
                              Best_GSD'Loop_Entry))

            --  WITNESS

            and then
              (if First_GSD_Found
               then Sel_K <= Last (Entity_Cfg.Gimbals)
                 and then Sel_CJ <= Last (Entity_Cfg.Cameras)
                 and then Sweep_Of
                            (Get (Entity_Cfg.Gimbals, Sel_K),
                             Elevation_Wire).Valid
                 and then Sel_S
                   < Sweep_Step_Count
                       (Sweep_Of
                          (Get (Entity_Cfg.Gimbals, Sel_K),
                           Elevation_Wire))
                 and then FOV_Candidate_Valid
                            (Get (Entity_Cfg.Cameras, Sel_CJ),
                             Sel_FI)
                 and then Is_Candidate
                            (Entity_Cfg, Eligible_Wavelength,
                             Elevation_Wire, Sel_K, Sel_S,
                             Sel_CJ, Sel_FI)
                 and then Selection_Witness
                            (FP, Entity_Cfg, Elevation_Wire,
                             Altitude.Value, Sel_K, Sel_S,
                             Sel_CJ, Sel_FI, Best_GSD)));
         declare
            Sweep : constant Elevation_Sweep :=
              Sweep_Of (Get (Entity_Cfg.Gimbals, K), Elevation_Wire);
         begin
            if Sweep.Valid then
               declare
                  Found_Prev : constant Boolean :=
                    First_GSD_Found with Ghost;
                  Best_Prev  : constant Achieved_GSD_M :=
                    Best_GSD with Ghost;
               begin
                  Evaluate_Gimbal (K, Sweep);

                  --  Carry the covered gimbal prefix across the
                  --  possible Best improvement, then extend it by this
                  --  gimbal (Evaluate_Gimbal's postcondition)

                  Lemma_Gimbals_Covered_Monotone
                    (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
                     Altitude.Value, K - 1, Acceptable_GSD, Found_Prev,
                     Best_Prev, First_GSD_Found, Best_GSD);
                  Lemma_Gimbals_Covered_Extend
                    (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
                     Altitude.Value, K, Acceptable_GSD, First_GSD_Found,
                     Best_GSD);
               end;
            else
               --  A gimbal with an invalid sweep holds no candidates:
               --  the covered prefix extends vacuously (P7)

               Lemma_Gimbal_Covered_Intro
                 (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
                  Altitude.Value, K, Acceptable_GSD, First_GSD_Found,
                  Best_GSD);
               Lemma_Gimbals_Covered_Extend
                 (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
                  Altitude.Value, K, Acceptable_GSD, First_GSD_Found,
                  Best_GSD);
            end if;
         end;
      end loop;

      if First_GSD_Found then

         --  The selected candidate's Sel_* indices witness that FP's
         --  selection attains the global GSD minimum (P7)

         Lemma_Footprint_GSD_Optimal_Intro
           (FP, Entity_Cfg, Eligible_Wavelength, Acceptable_GSD,
            Altitude.Value, Elevation_Wire, Sel_K, Sel_S, Sel_CJ,
            Sel_FI, Best_GSD);
      else
         --  No candidate was ever accepted: rewrite the full coverage
         --  with Found = False to the literal No_Candidate form (its
         --  improvement hypotheses are vacuous with the flag down)

         Lemma_Gimbals_Covered_Monotone
           (Entity_Cfg, Eligible_Wavelength, Elevation_Wire,
            Altitude.Value, Last (Entity_Cfg.Gimbals), Acceptable_GSD,
            False, Best_GSD, False, 0.0);
      end if;
   end Find_Sensor_Footprint;

   ------------------------------
   -- Lemma_Expected_Monotonic --
   ------------------------------

   procedure Lemma_Expected_Monotonic
     (State : Sensor_Manager_State;
      Msg   : SensorFootprintRequests_Msg;
      J     : Natural)
   is
   begin
      if J > 0 then
         Lemma_Expected_Monotonic (State, Msg, J - 1);
      end if;
   end Lemma_Expected_Monotonic;

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

   --------------------
   -- Build_Response --
   --------------------

   function Build_Response
     (State : Sensor_Manager_State;
      Msg   : SensorFootprintRequests_Msg) return SensorFootprintResponse_Msg
   is
      --  The per-footprint predicates are only carried from
      --  Process_Request's postcondition into the loop invariants and
      --  the final postcondition; they stay hidden here.

      procedure Process_Request
        (Entity_Cfg : EntityConfig;
         Request    : FootprintRequest_Msg;
         Response   : in out SensorFootprintResponse_Msg)
        with
          Always_Terminates,
          Post =>

            --  The response ID and the footprints already emitted for
            --  earlier requests are unchanged

            Response.ResponseID = Response.ResponseID'Old
            and then Equal_Prefix (Response.Footprints'Old,
                                   Response.Footprints)

            --  Every appended footprint carries Request's ID and the ID of
            --  the configuration it was computed against (P2 support), is
            --  degenerate or within its wire-level ranges (P4 support),
            --  and, unless degenerate, its commanded elevation traces to
            --  a gimbal of Entity_Cfg under one of Request's (defaulted)
            --  elevation entries (P6 support), its camera selection
            --  traces to a mounted, eligible camera of Entity_Cfg under
            --  one of Request's (defaulted) wavelength entries (P5
            --  support), and its selected GSD attains the global minimum
            --  over the candidate set under one of Request's (defaulted)
            --  wavelength x GSD x altitude x elevation combinations,
            --  whose effective altitude is valid (P7)

            and then
              (for all I in 1 .. Last (Response.Footprints) =>
                 (if I > Last (Response.Footprints'Old) then
                    Get (Response.Footprints, I).FootprintResponseID
                      = Request.FootprintRequestID
                    and then Get (Response.Footprints, I).VehicleID
                      = Entity_Cfg.ID
                    and then Footprint_Wire_OK
                      (Get (Response.Footprints, I))
                    and then
                      (Footprint_Geometry_Defaulted
                         (Get (Response.Footprints, I))
                       or else
                         ((for some EJ in
                             1 .. Last (Defaulted
                                          (Request.ElevationAngles))
                           =>
                             Footprint_Elevation_Traceable
                               (Get (Response.Footprints, I), Entity_Cfg,
                                Get (Defaulted (Request.ElevationAngles),
                                     EJ)))
                          and then
                            (for some WJ in
                               1 .. Last (Defaulted
                                            (Request.EligibleWavelengths))
                             =>
                               Footprint_Camera_Traceable
                                 (Get (Response.Footprints, I),
                                  Entity_Cfg,
                                  Get (Defaulted
                                         (Request.EligibleWavelengths),
                                       WJ)))
                          and then
                            (for some WJ in
                               1 .. Last (Defaulted
                                            (Request.EligibleWavelengths))
                             =>
                               (for some GJ in
                                  1 .. Last (Defaulted
                                               (Request.GroundSampleDistances))
                                =>
                                  (for some EJ in
                                     1 .. Last (Defaulted
                                                  (Request.ElevationAngles))
                                   =>
                                     (for some AJ in
                                        1 .. Last (Defaulted
                                                     (Request.AglAltitudes))
                                      =>
                                        Effective_Altitude
                                          (Get (Defaulted
                                                  (Request.AglAltitudes),
                                                AJ),
                                           Entity_Cfg.NominalAltitude).Valid
                                        and then
                                          Footprint_GSD_Optimal
                                            (Get (Response.Footprints, I),
                                             Entity_Cfg,
                                             Get (Defaulted
                                                    (Request
                                                       .EligibleWavelengths),
                                                  WJ),
                                             Effective_Desired_GSD
                                               (Get (Defaulted
                                                     (Request
                                                       .GroundSampleDistances),
                                                     GJ)),
                                             Effective_Altitude
                                               (Get (Defaulted
                                                       (Request.AglAltitudes),
                                                     AJ),
                                                Entity_Cfg.NominalAltitude)
                                               .Value,
                                             Get (Defaulted
                                                    (Request.ElevationAngles),
                                                  EJ))))))))))

            --  Exactly one footprint is appended per wavelength x GSD x
            --  altitude x elevation combination of Request, up to the D10
            --  cap of Positive'Last footprints (P3 support)

            and then To_Big_Integer (Last (Response.Footprints))
              = Min (To_Big_Integer (Last (Response.Footprints'Old))
                       + Combos (Request),
                     To_Big_Integer (Positive'Last));

      ---------------------
      -- Process_Request --
      ---------------------

      procedure Process_Request
        (Entity_Cfg : EntityConfig;
         Request    : FootprintRequest_Msg;
         Response   : in out SensorFootprintResponse_Msg)
      is
         --  The P4/P6/P7 predicates arrive as opaque facts from
         --  Find_Sensor_Footprint's postcondition and are only shuttled
         --  into the loop invariants; they stay hidden here.

         --  The four defaulted request dimensions, iterated by position
         --  below so the counting invariants (P3) can name each loop's
         --  progress; functional-vector iteration is positional, so the
         --  indexed loops visit the same elements in the same order as
         --  the previous of-loops.

         Ws : constant WavelengthBand_Seq :=
           Defaulted (Request.EligibleWavelengths);
         Gs : constant Real32_Seq := Defaulted (Request.GroundSampleDistances);
         As : constant Real32_Seq := Defaulted (Request.AglAltitudes);
         Es : constant Real32_Seq := Defaulted (Request.ElevationAngles);

         NW : constant Big_Positive := To_Big_Integer (Last (Ws)) with Ghost;
         NG : constant Big_Positive := To_Big_Integer (Last (Gs)) with Ghost;
         NA : constant Big_Positive := To_Big_Integer (Last (As)) with Ghost;
         NE : constant Big_Positive := To_Big_Integer (Last (Es)) with Ghost;
         --  Combination count of each defaulted dimension (equal to the
         --  corresponding Dim of Request, asserted below)

         Base : constant Big_Natural :=
           To_Big_Integer (Last (Response.Footprints)) with Ghost;
         --  Number of footprints already in the response on entry

         Cap : constant Big_Natural :=
           To_Big_Integer (Positive'Last) with Ghost;
         --  The D10 cap on the number of footprints in the response
      begin
         pragma Assert (NW = Dim (Request.EligibleWavelengths));
         pragma Assert (NG = Dim (Request.GroundSampleDistances));
         pragma Assert (NA = Dim (Request.AglAltitudes));
         pragma Assert (NE = Dim (Request.ElevationAngles));

         for WI in 1 .. Last (Ws) loop
            pragma Loop_Invariant
              (Response.ResponseID = Response.ResponseID'Loop_Entry);
            pragma Loop_Invariant
              (Equal_Prefix (Response.Footprints'Loop_Entry,
                             Response.Footprints));
            pragma Loop_Invariant
              (for all I in 1 .. Last (Response.Footprints) =>
                 (if I > Last (Response.Footprints'Loop_Entry) then
                    Get (Response.Footprints, I).FootprintResponseID
                      = Request.FootprintRequestID
                    and then Get (Response.Footprints, I).VehicleID
                      = Entity_Cfg.ID
                    and then Footprint_Wire_OK
                      (Get (Response.Footprints, I))
                    and then
                      (Footprint_Geometry_Defaulted
                         (Get (Response.Footprints, I))
                       or else
                         ((for some EJ in 1 .. Last (Es) =>
                             Footprint_Elevation_Traceable
                               (Get (Response.Footprints, I), Entity_Cfg,
                                Get (Es, EJ)))
                          and then
                            (for some WJ in 1 .. Last (Ws) =>
                               Footprint_Camera_Traceable
                                 (Get (Response.Footprints, I),
                                  Entity_Cfg, Get (Ws, WJ)))
                          and then
                            (for some WJ in 1 .. Last (Ws) =>
                               (for some GJ in 1 .. Last (Gs) =>
                                  (for some EJ in 1 .. Last (Es) =>
                                     (for some AJ in 1 .. Last (As) =>
                                        Effective_Altitude
                                          (Get (As, AJ),
                                           Entity_Cfg.NominalAltitude)
                                          .Valid
                                        and then
                                          Footprint_GSD_Optimal
                                            (Get (Response.Footprints,
                                                  I),
                                             Entity_Cfg, Get (Ws, WJ),
                                             Effective_Desired_GSD
                                               (Get (Gs, GJ)),
                                             Effective_Altitude
                                               (Get (As, AJ),
                                                Entity_Cfg
                                                  .NominalAltitude)
                                               .Value,
                                             Get (Es, EJ))))))))));
            pragma Loop_Invariant
              (To_Big_Integer (Last (Response.Footprints))
                 = Min (Base + To_Big_Integer (WI - 1) * NG * NA * NE,
                        Cap));
            for GI in 1 .. Last (Gs) loop
               pragma Loop_Invariant
                 (Response.ResponseID = Response.ResponseID'Loop_Entry);
               pragma Loop_Invariant
                 (Equal_Prefix (Response.Footprints'Loop_Entry,
                                Response.Footprints));
               pragma Loop_Invariant
                 (for all I in 1 .. Last (Response.Footprints) =>
                    (if I > Last (Response.Footprints'Loop_Entry) then
                       Get (Response.Footprints, I).FootprintResponseID
                         = Request.FootprintRequestID
                       and then Get (Response.Footprints, I).VehicleID
                         = Entity_Cfg.ID
                       and then Footprint_Wire_OK
                         (Get (Response.Footprints, I))
                       and then
                         (Footprint_Geometry_Defaulted
                            (Get (Response.Footprints, I))
                          or else
                            ((for some EJ in 1 .. Last (Es) =>
                                Footprint_Elevation_Traceable
                                  (Get (Response.Footprints, I),
                                   Entity_Cfg, Get (Es, EJ)))
                             and then
                               (for some WJ in 1 .. Last (Ws) =>
                                  Footprint_Camera_Traceable
                                    (Get (Response.Footprints, I),
                                     Entity_Cfg, Get (Ws, WJ)))
                             and then
                               (for some WJ in 1 .. Last (Ws) =>
                                  (for some GJ in 1 .. Last (Gs) =>
                                     (for some EJ in 1 .. Last (Es) =>
                                        (for some AJ in 1 .. Last (As) =>
                                           Effective_Altitude
                                             (Get (As, AJ),
                                              Entity_Cfg.NominalAltitude)
                                             .Valid
                                           and then
                                             Footprint_GSD_Optimal
                                               (Get (Response.Footprints,
                                                     I),
                                                Entity_Cfg, Get (Ws, WJ),
                                                Effective_Desired_GSD
                                                  (Get (Gs, GJ)),
                                                Effective_Altitude
                                                  (Get (As, AJ),
                                                   Entity_Cfg
                                                     .NominalAltitude)
                                                  .Value,
                                                Get (Es, EJ))))))))));
               pragma Loop_Invariant
                 (To_Big_Integer (Last (Response.Footprints))
                    = Min (Base
                             + To_Big_Integer (WI - 1) * NG * NA * NE
                             + To_Big_Integer (GI - 1) * NA * NE,
                           Cap));
               for AI in 1 .. Last (As) loop
                  pragma Loop_Invariant
                    (Response.ResponseID = Response.ResponseID'Loop_Entry);
                  pragma Loop_Invariant
                    (Equal_Prefix (Response.Footprints'Loop_Entry,
                                   Response.Footprints));
                  pragma Loop_Invariant
                    (for all I in 1 .. Last (Response.Footprints) =>
                       (if I > Last (Response.Footprints'Loop_Entry) then
                          Get (Response.Footprints, I).FootprintResponseID
                            = Request.FootprintRequestID
                          and then Get (Response.Footprints, I).VehicleID
                            = Entity_Cfg.ID
                          and then Footprint_Wire_OK
                            (Get (Response.Footprints, I))
                          and then
                            (Footprint_Geometry_Defaulted
                               (Get (Response.Footprints, I))
                             or else
                               ((for some EJ in 1 .. Last (Es) =>
                                   Footprint_Elevation_Traceable
                                     (Get (Response.Footprints, I),
                                      Entity_Cfg, Get (Es, EJ)))
                                and then
                                  (for some WJ in 1 .. Last (Ws) =>
                                     Footprint_Camera_Traceable
                                       (Get (Response.Footprints, I),
                                        Entity_Cfg, Get (Ws, WJ)))
                                and then
                                  (for some WJ in 1 .. Last (Ws) =>
                                     (for some GJ in 1 .. Last (Gs) =>
                                        (for some EJ in 1 .. Last (Es) =>
                                           (for some AJ in 1 .. Last (As) =>
                                              Effective_Altitude
                                                (Get (As, AJ),
                                                 Entity_Cfg.NominalAltitude)
                                                .Valid
                                              and then
                                                Footprint_GSD_Optimal
                                                  (Get (Response.Footprints,
                                                        I),
                                                   Entity_Cfg, Get (Ws, WJ),
                                                   Effective_Desired_GSD
                                                     (Get (Gs, GJ)),
                                                   Effective_Altitude
                                                     (Get (As, AJ),
                                                      Entity_Cfg
                                                        .NominalAltitude)
                                                     .Value,
                                                   Get (Es, EJ))))))))));
                  pragma Loop_Invariant
                    (To_Big_Integer (Last (Response.Footprints))
                       = Min (Base
                                + To_Big_Integer (WI - 1) * NG * NA * NE
                                + To_Big_Integer (GI - 1) * NA * NE
                                + To_Big_Integer (AI - 1) * NE,
                              Cap));
                  for EI in 1 .. Last (Es) loop
                     pragma Loop_Invariant
                       (Response.ResponseID = Response.ResponseID'Loop_Entry);
                     pragma Loop_Invariant
                       (Equal_Prefix (Response.Footprints'Loop_Entry,
                                      Response.Footprints));
                     pragma Loop_Invariant
                       (for all I in 1 .. Last (Response.Footprints) =>
                          (if I > Last (Response.Footprints'Loop_Entry) then
                             Get (Response.Footprints, I).FootprintResponseID
                               = Request.FootprintRequestID
                             and then Get (Response.Footprints, I).VehicleID
                               = Entity_Cfg.ID
                             and then Footprint_Wire_OK
                               (Get (Response.Footprints, I))
                             and then
                               (Footprint_Geometry_Defaulted
                                  (Get (Response.Footprints, I))
                                or else
                                  ((for some EJ in 1 .. Last (Es) =>
                                      Footprint_Elevation_Traceable
                                        (Get (Response.Footprints, I),
                                         Entity_Cfg, Get (Es, EJ)))
                                   and then
                                     (for some WJ in 1 .. Last (Ws) =>
                                        Footprint_Camera_Traceable
                                          (Get (Response.Footprints, I),
                                           Entity_Cfg,
                                           Get (Ws, WJ)))
                                   and then
                                     (for some WJ in 1 .. Last (Ws) =>
                                        (for some GJ in 1 .. Last (Gs) =>
                                           (for some EJ in 1 .. Last (Es) =>
                                              (for some AJ in
                                                 1 .. Last (As)
                                               =>
                                                 Effective_Altitude
                                                   (Get (As, AJ),
                                                    Entity_Cfg
                                                      .NominalAltitude)
                                                   .Valid
                                                 and then
                                                   Footprint_GSD_Optimal
                                                     (Get
                                                        (Response.Footprints,
                                                         I),
                                                      Entity_Cfg,
                                                      Get (Ws, WJ),
                                                      Effective_Desired_GSD
                                                        (Get (Gs, GJ)),
                                                      Effective_Altitude
                                                        (Get (As, AJ),
                                                         Entity_Cfg
                                                           .NominalAltitude)
                                                        .Value,
                                                      Get (Es, EJ))))))))));
                     pragma Loop_Invariant
                       (To_Big_Integer (Last (Response.Footprints))
                          = Min (Base
                                   + To_Big_Integer (WI - 1) * NG * NA * NE
                                   + To_Big_Integer (GI - 1) * NA * NE
                                   + To_Big_Integer (AI - 1) * NE
                                   + To_Big_Integer (EI - 1),
                                 Cap));
                     declare
                        FP : SensorFootprint_Msg;
                     begin
                        FP.FootprintResponseID := Request.FootprintRequestID;
                        FP.VehicleID           := Entity_Cfg.ID;
                        pragma Assert (Footprint_Geometry_Defaulted (FP));
                        Find_Sensor_Footprint
                          (Entity_Cfg          => Entity_Cfg,
                           Eligible_Wavelength => Get (Ws, WI),
                           Desired_GSD_Wire    => Get (Gs, GI),
                           Altitude_Wire       => Get (As, AI),
                           Elevation_Wire      => Get (Es, EI),
                           FP                  => FP);
                        pragma Assert (Footprint_Wire_OK (FP));
                        --  P7 lift: FP entered the call degenerate, so a
                        --  non-degenerate FP rules out the FP-untouched
                        --  disjuncts of Find_Sensor_Footprint's Post: the
                        --  effective altitude must be valid, and FP's
                        --  selection attains the global GSD minimum for
                        --  this (WI, GI, AI, EI) combination.
                        pragma Assert
                          (if not Footprint_Geometry_Defaulted (FP) then
                             Effective_Altitude
                               (Get (As, AI),
                                Entity_Cfg.NominalAltitude).Valid);
                        pragma Assert
                          (if not Footprint_Geometry_Defaulted (FP) then
                             Footprint_GSD_Optimal
                               (FP, Entity_Cfg, Get (Ws, WI),
                                Effective_Desired_GSD (Get (Gs, GI)),
                                Effective_Altitude
                                  (Get (As, AI),
                                   Entity_Cfg.NominalAltitude).Value,
                                Get (Es, EI)));
                        --  D10: the response sequence is capped at its
                        --  index type's range; footprints beyond
                        --  Positive'Last are dropped (unreachable in
                        --  practice: such a response could not be
                        --  serialized anyway).
                        if Last (Response.Footprints) < Positive'Last then
                           Response.Footprints :=
                             Add (Response.Footprints, FP);
                        end if;
                     end;
                  end loop;
                  pragma Assert
                    (To_Big_Integer (AI - 1) * NE + NE
                       = To_Big_Integer (AI) * NE);
               end loop;
               pragma Assert
                 (To_Big_Integer (GI - 1) * NA * NE + NA * NE
                    = To_Big_Integer (GI) * NA * NE);
            end loop;
            pragma Assert
              (To_Big_Integer (WI - 1) * NG * NA * NE + NG * NA * NE
                 = To_Big_Integer (WI) * NG * NA * NE);
         end loop;
         pragma Assert (Combos (Request) = NW * NG * NA * NE);
      end Process_Request;

      Response : SensorFootprintResponse_Msg;
   begin
      Response.ResponseID := Msg.RequestID;

      for J in 1 .. Last (Msg.Footprints) loop
         pragma Loop_Invariant (Response.ResponseID = Msg.RequestID);
         pragma Loop_Invariant
           (for all I in 1 .. Last (Response.Footprints) =>
              (for some K in 1 .. Last (Msg.Footprints) =>
                 Get (Response.Footprints, I).FootprintResponseID
                   = Get (Msg.Footprints, K).FootprintRequestID
                 and then Get (Response.Footprints, I).VehicleID
                   = Get (Msg.Footprints, K).VehicleID
                 and then Contains (State.Entity_Configs,
                                    Get (Msg.Footprints, K).VehicleID)));
         pragma Loop_Invariant
           (To_Big_Integer (Last (Response.Footprints))
              = Min (Expected (State, Msg, J - 1),
                     To_Big_Integer (Positive'Last)));
         pragma Loop_Invariant
           (for all I in 1 .. Last (Response.Footprints) =>
              Footprint_Wire_OK (Get (Response.Footprints, I)));
         pragma Loop_Invariant
           (for all K in 1 .. J - 1 =>
              (for all I in 1 .. Last (Response.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, K - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, K)
                    and then Contains (State.Entity_Configs,
                                       Get (Msg.Footprints, K).VehicleID)
                    and then not Footprint_Geometry_Defaulted
                                   (Get (Response.Footprints, I))
                  then
                    (for some EJ in
                       1 .. Last (Defaulted
                                    (Get (Msg.Footprints, K)
                                       .ElevationAngles))
                     =>
                       Footprint_Elevation_Traceable
                         (Get (Response.Footprints, I),
                          Element (State.Entity_Configs,
                                   Get (Msg.Footprints, K).VehicleID),
                          Get (Defaulted
                                 (Get (Msg.Footprints, K).ElevationAngles),
                               EJ))))));
         pragma Loop_Invariant
           (for all K in 1 .. J - 1 =>
              (for all I in 1 .. Last (Response.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, K - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, K)
                    and then Contains (State.Entity_Configs,
                                       Get (Msg.Footprints, K).VehicleID)
                    and then not Footprint_Geometry_Defaulted
                                   (Get (Response.Footprints, I))
                  then
                    (for some WJ in
                       1 .. Last (Defaulted
                                    (Get (Msg.Footprints, K)
                                       .EligibleWavelengths))
                     =>
                       Footprint_Camera_Traceable
                         (Get (Response.Footprints, I),
                          Element (State.Entity_Configs,
                                   Get (Msg.Footprints, K).VehicleID),
                          Get (Defaulted
                                 (Get (Msg.Footprints, K)
                                    .EligibleWavelengths),
                               WJ))))));
         pragma Loop_Invariant
           (for all K in 1 .. J - 1 =>
              (for all I in 1 .. Last (Response.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, K - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, K)
                    and then Contains (State.Entity_Configs,
                                       Get (Msg.Footprints, K).VehicleID)
                    and then not Footprint_Geometry_Defaulted
                                   (Get (Response.Footprints, I))
                  then
                    (for some WJ in
                       1 .. Last (Defaulted
                                    (Get (Msg.Footprints, K)
                                       .EligibleWavelengths))
                     =>
                       (for some GJ in
                          1 .. Last (Defaulted
                                       (Get (Msg.Footprints, K)
                                          .GroundSampleDistances))
                        =>
                          (for some EJ in
                             1 .. Last (Defaulted
                                          (Get (Msg.Footprints, K)
                                             .ElevationAngles))
                           =>
                             (for some AJ in
                                1 .. Last (Defaulted
                                             (Get (Msg.Footprints, K)
                                                .AglAltitudes))
                              =>
                                Effective_Altitude
                                  (Get (Defaulted
                                          (Get (Msg.Footprints, K)
                                             .AglAltitudes),
                                        AJ),
                                   Element (State.Entity_Configs,
                                            Get (Msg.Footprints, K)
                                              .VehicleID)
                                     .NominalAltitude).Valid
                                and then
                                  Footprint_GSD_Optimal
                                    (Get (Response.Footprints, I),
                                     Element (State.Entity_Configs,
                                              Get (Msg.Footprints, K)
                                                .VehicleID),
                                     Get (Defaulted
                                            (Get (Msg.Footprints, K)
                                               .EligibleWavelengths),
                                          WJ),
                                     Effective_Desired_GSD
                                       (Get (Defaulted
                                               (Get (Msg.Footprints, K)
                                                  .GroundSampleDistances),
                                             GJ)),
                                     Effective_Altitude
                                       (Get (Defaulted
                                               (Get (Msg.Footprints, K)
                                                  .AglAltitudes),
                                             AJ),
                                        Element (State.Entity_Configs,
                                                 Get (Msg.Footprints, K)
                                                   .VehicleID)
                                          .NominalAltitude).Value,
                                     Get (Defaulted
                                            (Get (Msg.Footprints, K)
                                               .ElevationAngles),
                                          EJ)))))))));
         pragma Loop_Invariant
           (for all K in 1 .. J - 1 =>
              (for all I in 1 .. Last (Response.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, K - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, K)
                  then
                    Get (Response.Footprints, I).FootprintResponseID
                      = Get (Msg.Footprints, K).FootprintRequestID
                    and then Get (Response.Footprints, I).VehicleID
                      = Get (Msg.Footprints, K).VehicleID)));

         --  Earlier requests' segments end at or below the current
         --  footprint count, so the appended segment starts above them

         Lemma_Expected_Monotonic (State, Msg, J - 1);

         declare
            Request : constant FootprintRequest_Msg := Get (Msg.Footprints, J);
         begin
            if Contains (State.Entity_Configs, Request.VehicleID) then
               Process_Request
                 (Entity_Cfg => Element (State.Entity_Configs,
                                         Request.VehicleID),
                  Request    => Request,
                  Response   => Response);
            end if;
         end;
      end loop;

      return Response;
   end Build_Response;

   ------------------------------------
   -- Handle_SensorFootprintRequests --
   ------------------------------------

   procedure Handle_SensorFootprintRequests
     (State   : Sensor_Manager_State;
      Mailbox : in out Sensor_Manager_Mailbox;
      Msg     : SensorFootprintRequests_Msg)
   is
   begin
      --  P10: exactly one broadcast per handled message, whose content is
      --  Build_Response's result — by construction.
      sendBroadcastMessage (Mailbox, Build_Response (State, Msg));
   end Handle_SensorFootprintRequests;

end Sensor_Manager;
