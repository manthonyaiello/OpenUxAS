with Ada.Containers;
with SPARK.Containers.Formal.Hashed_Maps;
with Common;                   use Common;
with LMCP_Messages;            use LMCP_Messages;
with Sensor_Manager_Mailboxes; use Sensor_Manager_Mailboxes;

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

   procedure Handle_EntityConfig
     (State  : in out Sensor_Manager_State;
      Config : EntityConfig)
     with Always_Terminates;
   --  Store or replace an entity configuration, indexed by entity ID. A
   --  configuration whose ID is already known replaces the existing entry;
   --  otherwise it is inserted while the map is below Max_Entity_Configs.
   --  @param State Service state whose entity-config map is updated
   --  @param Config Entity configuration to store, keyed by Config.ID

   procedure Handle_SensorFootprintRequests
     (State   : Sensor_Manager_State;
      Mailbox : in out Sensor_Manager_Mailbox;
      Msg     : SensorFootprintRequests_Msg)
     with Always_Terminates;
   --  Process every footprint request in Msg against the known entity
   --  configurations and broadcast a single SensorFootprintResponse.
   --  @param State Service state supplying the entity configurations
   --  @param Mailbox Mailbox used to broadcast the response message
   --  @param Msg The batch of footprint requests to service

end Sensor_Manager;
