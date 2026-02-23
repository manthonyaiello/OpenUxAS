with DOM.Core;

with Common;                   use Common;
with Sensor_Manager;           use Sensor_Manager;
with Sensor_Manager_Mailboxes; use Sensor_Manager_Mailboxes;

package UxAS.Comms.LMCP_Net_Client.Service.Sensor_Manager_Interfacing is

   type Sensor_Manager_Service is new Service_Base with private;

   Type_Name : constant String := "SensorManagerService";

   Directory_Name : constant String := "";

   --  static const std::vector<std::string>
   --  s_registryServiceTypeNames()
   function Registry_Service_Type_Names return Service_Type_Names_List;

   --  static ServiceBase*
   --  create()
   function Create return Any_Service;

private

   type Sensor_Manager_Service is new Service_Base with record

      --  the following types are defined in SPARK code
      Mailbox : Sensor_Manager_Mailbox;
      State   : Sensor_Manager_State;
      Config  : Sensor_Manager_Configuration_Data;
   end record;

   overriding
   procedure Configure
     (This     : in out Sensor_Manager_Service;
      XML_Node : DOM.Core.Element;
      Result   : out Boolean);

   overriding
   procedure Initialize
     (This   : in out Sensor_Manager_Service;
      Result : out Boolean);

   overriding
   procedure Process_Received_LMCP_Message
     (This             : in out Sensor_Manager_Service;
      Received_Message : not null Any_LMCP_Message;
      Should_Terminate : out Boolean);

end UxAS.Comms.LMCP_Net_Client.Service.Sensor_Manager_Interfacing;
