with Ada.Containers;
with SPARK.Containers.Formal.Hashed_Maps;
with Common;                   use Common;
with LMCP_Messages;            use LMCP_Messages;
with Sensor_Manager_Mailboxes; use Sensor_Manager_Mailboxes;

package Sensor_Manager with SPARK_Mode is

   pragma Unevaluated_Use_Of_Old (Allow);

   --  Maximum number of entity configs to store
   Max_Entity_Configs : constant Ada.Containers.Count_Type := 200;

   package Entity_Config_Maps is new SPARK.Containers.Formal.Hashed_Maps
     (Key_Type     => Int64,
      Element_Type => EntityConfig,
      Hash         => Int64_Hash);

   subtype Entity_Config_Map is Entity_Config_Maps.Map
     (Max_Entity_Configs,
      Entity_Config_Maps.Default_Modulus (Max_Entity_Configs));

   type Sensor_Manager_Configuration_Data is record
      null;
   end record;

   type Sensor_Manager_State is record
      Entity_Configs : Entity_Config_Map;
   end record;

   --  Store or replace an entity configuration indexed by entity ID
   procedure Handle_EntityConfig
     (State  : in out Sensor_Manager_State;
      Config : EntityConfig)
     with Always_Terminates;

   --  Process all footprint requests and broadcast the response
   procedure Handle_SensorFootprintRequests
     (State   : in out Sensor_Manager_State;
      Mailbox : in out Sensor_Manager_Mailbox;
      Msg     : SensorFootprintRequests_Msg)
     with Always_Terminates;

end Sensor_Manager;
