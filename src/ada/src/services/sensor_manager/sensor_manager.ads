with Ada.Containers;
with SPARK.Big_Integers;       use SPARK.Big_Integers;
with SPARK.Containers.Formal.Hashed_Maps;
with Common;                   use Common;
with LMCP_Messages;            use LMCP_Messages;
with Sensor_Manager_Mailboxes; use Sensor_Manager_Mailboxes;
with Sensor_Manager_Types;     use Sensor_Manager_Types;

--  Core SPARK logic of the Sensor Manager service: it stores the entity
--  configurations it is told about and, on each footprint request, computes
--  the best-matching sensor footprints and broadcasts the response. The
--  geometry and the sanitization of wire values live in Sensor_Manager_Types
--  and Sensor_Manager_Trig; message passing lives in Sensor_Manager_Mailboxes.

package Sensor_Manager with SPARK_Mode is

   pragma Unevaluated_Use_Of_Old (Allow);

   Max_Entity_Configs : constant Ada.Containers.Count_Type := 200;
   --  Maximum number of entity configurations the service will store

   package Entity_Config_Maps is new SPARK.Containers.Formal.Hashed_Maps
     (Key_Type     => Int64,
      Element_Type => EntityConfig,
      Hash         => Int64_Hash);
   --  Formal hashed map from entity ID to its EntityConfig

   subtype Entity_Config_Map is Entity_Config_Maps.Map
     (Max_Entity_Configs,
      Entity_Config_Maps.Default_Modulus (Max_Entity_Configs));
   --  An Entity_Config_Maps.Map sized for at most Max_Entity_Configs entries

   use Entity_Config_Maps;
   use Entity_Config_Maps.Formal_Model;
   use type Ada.Containers.Count_Type;

   type Sensor_Manager_Configuration_Data is record
      null;
   end record;
   --  Placeholder for the service's XML configuration. The Sensor Manager
   --  has no configurable parameters, so the record is empty.

   type Sensor_Manager_State is record
      Entity_Configs : Entity_Config_Map;
      --  The entity configurations received so far, keyed by entity ID
   end record;
   --  Mutable state of the service: the set of known entity configurations

   function Configs_Keyed_By_ID (State : Sensor_Manager_State) return Boolean
   is
     (for all K of Model (State.Entity_Configs) =>
        Element (Model (State.Entity_Configs), K).ID = K)
   with Ghost;
   --  State invariant: every stored entity configuration is keyed by its
   --  own ID, so a lookup by vehicle ID returns that vehicle's own
   --  configuration. It holds trivially of a default-initialized (empty)
   --  map, is preserved by Handle_EntityConfig, and is what lets
   --  Build_Response correlate each response footprint with its request
   --  (Gold property P2).
   --  @param State Service state whose entity-config map is examined
   --  @return True iff each stored configuration's ID equals its key

   procedure Handle_EntityConfig
     (State  : in out Sensor_Manager_State;
      Config : EntityConfig)
     with
       Always_Terminates,
       Pre  => Configs_Keyed_By_ID (State),
       Post =>

         --  Config.ID is stored exactly when it was already known or the
         --  map had room, and it is then mapped to Config (P1, clause 1)

         Contains (State.Entity_Configs, Config.ID) =
           (Contains (State.Entity_Configs'Old, Config.ID)
              or else Length (State.Entity_Configs'Old) < Max_Entity_Configs)
         and then
           (if Contains (State.Entity_Configs, Config.ID)
            then Element (State.Entity_Configs, Config.ID) = Config)

         --  Every other key keeps its mapping: no key disappears, and no
         --  key other than Config.ID appears (P1, clause 2)

         and then M.Keys_Included
               (Model (State.Entity_Configs'Old),
                Model (State.Entity_Configs))
         and then M.Elements_Equal_Except
               (Model (State.Entity_Configs),
                Model (State.Entity_Configs'Old),
                Config.ID)

         --  The length grows by one exactly when Config.ID is new and the
         --  map had room, and is unchanged otherwise (P1, clause 3)

         and then Length (State.Entity_Configs) =
               (if Contains (State.Entity_Configs'Old, Config.ID)
                  or else Length (State.Entity_Configs'Old)
                            = Max_Entity_Configs
                then Length (State.Entity_Configs'Old)
                else Length (State.Entity_Configs'Old) + 1)

         --  Every stored configuration is still keyed by its own ID, so
         --  the correlation invariant is maintained (P2 support)

         and then Configs_Keyed_By_ID (State);
   --  Store or replace an entity configuration, indexed by entity ID. A
   --  configuration whose ID is already known replaces the existing entry;
   --  otherwise it is inserted while the map is below Max_Entity_Configs.
   --  The postcondition states Gold property P1 (GOLD_PROPERTIES.md): the
   --  configuration is stored whenever possible, every other mapping is
   --  preserved, and the length grows only on a new insertion. It also
   --  maintains the Configs_Keyed_By_ID invariant needed by P2.
   --  @param State Service state whose entity-config map is updated
   --  @param Config Entity configuration to store, keyed by Config.ID

   function Defaulted (Seq : Real32_Seq) return Real32_Seq is
     (if Last (Seq) = 0 then Add (Empty_Sequence, 0.0)
      else Seq);
   --  A Real32 request dimension with the empty-list defaulting rule
   --  applied: an empty dimension becomes the single 0.0 "unspecified"
   --  sentinel, as in C++ (P3/P6 support).
   --  @param Seq A Real32 request dimension sequence
   --  @return Seq, or the singleton 0.0 sequence if Seq is empty

   function Defaulted (Seq : WavelengthBand_Seq) return WavelengthBand_Seq
   is
     (if Last (Seq) = 0 then Add (Empty_Sequence, AllAny)
      else Seq);
   --  The wavelength request dimension with the empty-list defaulting
   --  rule applied: an empty dimension becomes the single AllAny
   --  sentinel, as in C++ (P3 support).
   --  @param Seq The wavelength request dimension sequence
   --  @return Seq, or the singleton AllAny sequence if Seq is empty

   function Dim (Seq : Real32_Seq) return Big_Positive is
     (if Last (Seq) = 0 then 1 else To_Big_Integer (Last (Seq)))
   with Ghost;
   --  Number of combinations a Real32 request dimension contributes: an
   --  empty dimension is defaulted to a single "unspecified" sentinel, so
   --  it counts one (P3 support).
   --  @param Seq A Real32 request dimension sequence
   --  @return max (1, Last (Seq)), as a big integer

   function Dim (Seq : WavelengthBand_Seq) return Big_Positive is
     (if Last (Seq) = 0 then 1 else To_Big_Integer (Last (Seq)))
   with Ghost;
   --  Number of combinations the wavelength request dimension contributes:
   --  an empty dimension is defaulted to a single AllAny sentinel, so it
   --  counts one (P3 support).
   --  @param Seq The wavelength request dimension sequence
   --  @return max (1, Last (Seq)), as a big integer

   function Combos (Request : FootprintRequest_Msg) return Big_Positive is
     (Dim (Request.EligibleWavelengths)
      * Dim (Request.GroundSampleDistances)
      * Dim (Request.AglAltitudes)
      * Dim (Request.ElevationAngles))
   with Ghost;
   --  Number of wavelength x GSD x altitude x elevation combinations of one
   --  footprint request, each defaulted dimension counting one (P3
   --  support). Big-integer arithmetic keeps the product free of overflow
   --  concerns.
   --  @param Request The footprint request whose combinations are counted
   --  @return The product of the four defaulted dimension counts

   function Expected
     (State : Sensor_Manager_State;
      Msg   : SensorFootprintRequests_Msg;
      J     : Natural) return Big_Natural
   is
     (if J = 0 then Big_Natural'(0)
      else Expected (State, Msg, J - 1)
        + (if Contains (State.Entity_Configs,
                        Get (Msg.Footprints, J).VehicleID)
           then Combos (Get (Msg.Footprints, J))
           else Big_Natural'(0)))
   with
     Ghost,
     Pre => J <= Last (Msg.Footprints),
     Subprogram_Variant => (Decreases => J);
   --  Number of footprints the first J requests of Msg must produce: each
   --  request whose VehicleID has a stored configuration contributes its
   --  combination count, and every other request contributes nothing (P3
   --  support).
   --  @param State Service state supplying the entity configurations
   --  @param Msg The batch of footprint requests being counted
   --  @param J Number of leading requests of Msg to count
   --  @return The sum of Combos over the known-vehicle requests among the
   --    first J requests of Msg

   function Build_Response
     (State : Sensor_Manager_State;
      Msg   : SensorFootprintRequests_Msg) return SensorFootprintResponse_Msg
     with
       Pre  => Configs_Keyed_By_ID (State),
       Post =>

         --  The response's ID matches the batch's request ID (P2, clause 1)

         Build_Response'Result.ResponseID = Msg.RequestID

         --  Every footprint in the response carries the FootprintRequestID
         --  and VehicleID of a request in Msg it was computed for, and that
         --  request's vehicle has a stored configuration (P2, clause 2)

         and then
           (for all I in 1 .. Last (Build_Response'Result.Footprints) =>
              (for some J in 1 .. Last (Msg.Footprints) =>
                 Get (Build_Response'Result.Footprints, I).FootprintResponseID
                   = Get (Msg.Footprints, J).FootprintRequestID
                 and then Get (Build_Response'Result.Footprints, I).VehicleID
                   = Get (Msg.Footprints, J).VehicleID
                 and then Contains (State.Entity_Configs,
                                    Get (Msg.Footprints, J).VehicleID)))

         --  The response contains exactly one footprint per wavelength x
         --  GSD x altitude x elevation combination of every request whose
         --  vehicle has a stored configuration, capped at Positive'Last
         --  footprints (the D10 cap) (P3)

         and then To_Big_Integer (Last (Build_Response'Result.Footprints))
           = Min (Expected (State, Msg, Last (Msg.Footprints)),
                  To_Big_Integer (Positive'Last))

         --  Positional correlation: the footprints computed for request J
         --  occupy exactly the response positions Expected (J - 1) + 1
         --  .. Expected (J); positions beyond the D10 cap do not exist,
         --  so the guard on I covers the capped case as well (P3)

         and then
           (for all J in 1 .. Last (Msg.Footprints) =>
              (for all I in 1 .. Last (Build_Response'Result.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, J - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, J)
                  then
                    Get (Build_Response'Result.Footprints, I)
                      .FootprintResponseID
                        = Get (Msg.Footprints, J).FootprintRequestID
                    and then Get (Build_Response'Result.Footprints, I)
                      .VehicleID = Get (Msg.Footprints, J).VehicleID)))

         --  Every emitted footprint is either the degenerate all-default
         --  footprint (no sensor candidate qualified) or lies within the
         --  Real32 images of its constrained working subtypes — no
         --  nonsense values on the bus (P4)

         and then
           (for all I in 1 .. Last (Build_Response'Result.Footprints) =>
              Footprint_Wire_OK (Get (Build_Response'Result.Footprints, I)))

         --  Every non-degenerate footprint's commanded elevation traces to
         --  a gimbal of the stored configuration of the vehicle it was
         --  computed for, under one of its request's (defaulted) elevation
         --  entries: it is achievable by the gimbal the footprint names,
         --  and honors an active override after the documented clamping
         --  (P6)

         and then
           (for all J in 1 .. Last (Msg.Footprints) =>
              (for all I in 1 .. Last (Build_Response'Result.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, J - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, J)
                    and then Contains (State.Entity_Configs,
                                       Get (Msg.Footprints, J).VehicleID)
                    and then not Footprint_Geometry_Defaulted
                                   (Get (Build_Response'Result.Footprints,
                                         I))
                  then
                    (for some EJ in
                       1 .. Last (Defaulted
                                    (Get (Msg.Footprints, J)
                                       .ElevationAngles))
                     =>
                       Footprint_Elevation_Traceable
                         (Get (Build_Response'Result.Footprints, I),
                          Element (State.Entity_Configs,
                                   Get (Msg.Footprints, J).VehicleID),
                          Get (Defaulted
                                 (Get (Msg.Footprints, J).ElevationAngles),
                               EJ))))))

         --  Every non-degenerate footprint's camera selection traces to
         --  the stored configuration of the vehicle it was computed for,
         --  under one of its request's (defaulted) eligible-wavelength
         --  entries: some camera of that configuration carries the
         --  footprint's CameraID, is mounted (via ContainedPayloadList)
         --  on a gimbal carrying its GimbalID, supports the wavelength
         --  the footprint reports subject to the eligibility filter, and
         --  the footprint's HorizontalFOV is the Real32 image of one of
         --  that camera's valid FOV candidates (P5)

         and then
           (for all J in 1 .. Last (Msg.Footprints) =>
              (for all I in 1 .. Last (Build_Response'Result.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, J - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, J)
                    and then Contains (State.Entity_Configs,
                                       Get (Msg.Footprints, J).VehicleID)
                    and then not Footprint_Geometry_Defaulted
                                   (Get (Build_Response'Result.Footprints,
                                         I))
                  then
                    (for some WJ in
                       1 .. Last (Defaulted
                                    (Get (Msg.Footprints, J)
                                       .EligibleWavelengths))
                     =>
                       Footprint_Camera_Traceable
                         (Get (Build_Response'Result.Footprints, I),
                          Element (State.Entity_Configs,
                                   Get (Msg.Footprints, J).VehicleID),
                          Get (Defaulted
                                 (Get (Msg.Footprints, J)
                                    .EligibleWavelengths),
                               WJ))))))

         --  Every non-degenerate footprint's selected GSD attains the
         --  minimum distance to the desired GSD over the entire candidate
         --  set of the stored configuration of the vehicle it was computed
         --  for, under one of its request's (defaulted) wavelength x GSD x
         --  altitude x elevation combinations, whose effective altitude is
         --  valid; the selection is jointly witnessed by one (gimbal,
         --  sweep step, camera, FOV) candidate tuple (P7)

         and then
           (for all J in 1 .. Last (Msg.Footprints) =>
              (for all I in 1 .. Last (Build_Response'Result.Footprints) =>
                 (if To_Big_Integer (I) > Expected (State, Msg, J - 1)
                    and then To_Big_Integer (I) <= Expected (State, Msg, J)
                    and then Contains (State.Entity_Configs,
                                       Get (Msg.Footprints, J).VehicleID)
                    and then not Footprint_Geometry_Defaulted
                                   (Get (Build_Response'Result.Footprints,
                                         I))
                  then
                    (for some WJ in
                       1 .. Last (Defaulted
                                    (Get (Msg.Footprints, J)
                                       .EligibleWavelengths))
                     =>
                       (for some GJ in
                          1 .. Last (Defaulted
                                       (Get (Msg.Footprints, J)
                                          .GroundSampleDistances))
                        =>
                          (for some EJ in
                             1 .. Last (Defaulted
                                          (Get (Msg.Footprints, J)
                                             .ElevationAngles))
                           =>
                             (for some AJ in
                                1 .. Last (Defaulted
                                             (Get (Msg.Footprints, J)
                                                .AglAltitudes))
                              =>
                                Effective_Altitude
                                  (Get (Defaulted
                                          (Get (Msg.Footprints, J)
                                             .AglAltitudes),
                                        AJ),
                                   Element (State.Entity_Configs,
                                            Get (Msg.Footprints, J)
                                              .VehicleID)
                                     .NominalAltitude).Valid
                                and then
                                  Footprint_GSD_Optimal
                                    (Get (Build_Response'Result.Footprints,
                                          I),
                                     Element (State.Entity_Configs,
                                              Get (Msg.Footprints, J)
                                                .VehicleID),
                                     Get (Defaulted
                                            (Get (Msg.Footprints, J)
                                               .EligibleWavelengths),
                                          WJ),
                                     Effective_Desired_GSD
                                       (Get (Defaulted
                                               (Get (Msg.Footprints, J)
                                                  .GroundSampleDistances),
                                             GJ)),
                                     Effective_Altitude
                                       (Get (Defaulted
                                               (Get (Msg.Footprints, J)
                                                  .AglAltitudes),
                                             AJ),
                                        Element (State.Entity_Configs,
                                                 Get (Msg.Footprints, J)
                                                   .VehicleID)
                                          .NominalAltitude).Value,
                                     Get (Defaulted
                                            (Get (Msg.Footprints, J)
                                               .ElevationAngles),
                                          EJ)))))))));
   --  Compute the SensorFootprintResponse for a batch of footprint
   --  requests: for each request whose VehicleID has a stored entity
   --  configuration, the best-matching sensor footprint of every
   --  wavelength x GSD x altitude x elevation combination is appended, in
   --  request order. The postcondition states Gold properties P2 and P3
   --  (GOLD_PROPERTIES.md): the response ID echoes the batch's request ID,
   --  every emitted footprint is correlated with a request in Msg, the
   --  number of emitted footprints is exactly the number of combinations
   --  of the known-vehicle requests (capped at Positive'Last, per D10),
   --  and the footprints of each request occupy exactly their positional
   --  segment of the response. Further clauses state the per-footprint
   --  properties: wire-level sanity (P4), elevation traceability (P6),
   --  camera traceability (P5), and GSD argmin optimality over the
   --  candidate set under one of the request's (defaulted) combinations
   --  (P7).
   --  @param State Service state supplying the entity configurations
   --  @param Msg The batch of footprint requests to service
   --  @return The response to broadcast for Msg

   procedure Handle_SensorFootprintRequests
     (State   : Sensor_Manager_State;
      Mailbox : in out Sensor_Manager_Mailbox;
      Msg     : SensorFootprintRequests_Msg)
     with
       Always_Terminates,
       Pre => Configs_Keyed_By_ID (State);
   --  Broadcast exactly one SensorFootprintResponse for Msg, whose content
   --  is Build_Response (State, Msg) — the handler is a single build-then-
   --  send (Gold property P10); the response's contents are covered by
   --  Build_Response's postcondition (Gold property P2).
   --  @param State Service state supplying the entity configurations
   --  @param Mailbox Mailbox used to broadcast the response message
   --  @param Msg The batch of footprint requests to service

end Sensor_Manager;
