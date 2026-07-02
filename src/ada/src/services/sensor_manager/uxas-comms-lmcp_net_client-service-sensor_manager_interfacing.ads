with DOM.Core;

with Common;                   use Common;
with Sensor_Manager;           use Sensor_Manager;
with Sensor_Manager_Mailboxes; use Sensor_Manager_Mailboxes;

--  Glue between the UxAS C++/Ada service framework and the SPARK Sensor
--  Manager. It registers the service, receives LMCP messages off the bus,
--  and dispatches them to the SPARK logic in Sensor_Manager.

package UxAS.Comms.LMCP_Net_Client.Service.Sensor_Manager_Interfacing is

   type Sensor_Manager_Service is new Service_Base with private;
   --  The Sensor Manager service as seen by the UxAS service framework

   Type_Name : constant String := "SensorManagerService";
   --  The registered type name of this service

   Directory_Name : constant String := "";
   --  Working-directory name for this service (none required)

   function Registry_Service_Type_Names return Service_Type_Names_List;
   --  The type names this service registers under. Corresponds to the C++
   --  static s_registryServiceTypeNames().
   --  @return The list of registered service type names

   function Create return Any_Service;
   --  Factory that constructs a fresh service instance. Corresponds to the
   --  C++ static create().
   --  @return A new Sensor Manager service instance

private

   type Sensor_Manager_Service is new Service_Base with record
      Mailbox : Sensor_Manager_Mailbox;
      --  Message-passing state (defined in SPARK code)
      State   : Sensor_Manager_State;
      --  Service state: the known entity configurations (SPARK code)
      Config  : Sensor_Manager_Configuration_Data;
      --  Parsed XML configuration (SPARK code)
   end record;
   --  Full view: a framework service holding the SPARK mailbox, state, and
   --  configuration.

   overriding
   procedure Configure
     (This     : in out Sensor_Manager_Service;
      XML_Node : DOM.Core.Element;
      Result   : out Boolean);
   --  Parse the service's XML configuration.
   --  @param This The service being configured
   --  @param XML_Node The service's XML configuration element
   --  @param Result Set True iff configuration succeeded

   overriding
   procedure Initialize
     (This   : in out Sensor_Manager_Service;
      Result : out Boolean);
   --  Initialize the service after configuration (sets up the mailbox).
   --  @param This The service being initialized
   --  @param Result Set True iff initialization succeeded

   overriding
   procedure Process_Received_LMCP_Message
     (This             : in out Sensor_Manager_Service;
      Received_Message : not null Any_LMCP_Message;
      Should_Terminate : out Boolean);
   --  Dispatch a received LMCP message to the SPARK Sensor Manager logic.
   --  @param This The service receiving the message
   --  @param Received_Message The LMCP message just received off the bus
   --  @param Should_Terminate Set True iff the service should shut down

end UxAS.Comms.LMCP_Net_Client.Service.Sensor_Manager_Interfacing;
