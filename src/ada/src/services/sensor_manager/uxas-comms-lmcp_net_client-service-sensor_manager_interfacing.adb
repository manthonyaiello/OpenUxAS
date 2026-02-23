with LMCP_Message_Conversions;   use LMCP_Message_Conversions;
with LMCP_Messages;

with AFRL.CMASI.EntityConfiguration;
  use AFRL.CMASI.EntityConfiguration;
with AFRL.CMASI.RemoveTasks;
  use AFRL.CMASI.RemoveTasks;
with UxAS.Messages.lmcptask.SensorFootprintRequests;
  use UxAS.Messages.lmcptask.SensorFootprintRequests;

package body UxAS.Comms.LMCP_Net_Client.Service.Sensor_Manager_Interfacing is

   ---------------
   -- Configure --
   ---------------

   overriding
   procedure Configure
     (This     : in out Sensor_Manager_Service;
      XML_Node : DOM.Core.Element;
      Result   : out Boolean)
   is
      pragma Unreferenced (XML_Node);
      Unused : Boolean;
   begin
      --  addSubscriptionAddress(afrl::cmasi::RemoveTasks::Subscription);
      This.Add_Subscription_Address (AFRL.CMASI.RemoveTasks.Subscription, Unused);

      --  addSubscriptionAddress(afrl::cmasi::EntityConfiguration::Subscription);
      This.Add_Subscription_Address (AFRL.CMASI.EntityConfiguration.Subscription, Unused);
      for Descendant of EntityConfiguration_Descendants loop
         This.Add_Subscription_Address (Descendant, Unused);
      end loop;

      --  addSubscriptionAddress(uxas::messages::task::SensorFootprintRequests::Subscription);
      This.Add_Subscription_Address
        (UxAS.Messages.lmcptask.SensorFootprintRequests.Subscription, Unused);

      Result := True;
   end Configure;

   ------------
   -- Create --
   ------------

   function Create return Any_Service is
      Result : Any_Service;
   begin
      Result := new Sensor_Manager_Service;
      Result.Construct_Service
        (Service_Type        => Type_Name,
         Work_Directory_Name => Directory_Name);
      return Result;
   end Create;

   ----------------
   -- Initialize --
   ----------------

   overriding
   procedure Initialize
     (This   : in out Sensor_Manager_Service;
      Result : out Boolean)
   is
   begin
      Result := True;

      Sensor_Manager_Mailboxes.Initialize
        (This.Mailbox,
         Source_Group => Value (This.Message_Source_Group),
         Unique_Id    => Common.Int64 (UxAS.Comms.LMCP_Net_Client.Unique_Entity_Send_Message_Id),
         Entity_Id    => Common.UInt32 (This.Entity_Id),
         Service_Id   => Common.UInt32 (This.Network_Id));
   end Initialize;

   -----------------------------------
   -- Process_Received_LMCP_Message --
   -----------------------------------

   overriding
   procedure Process_Received_LMCP_Message
     (This             : in out Sensor_Manager_Service;
      Received_Message :        not null Any_LMCP_Message;
      Should_Terminate :    out Boolean)
   is
   begin
      if Received_Message.Payload.all in EntityConfiguration'Class then
         declare
            use all type LMCP_Messages.GimbalConfig_Seq;
            use all type LMCP_Messages.CameraConfig_Seq;
            EC : constant LMCP_Messages.EntityConfig :=
              As_EntityConfig_Message
                (EntityConfiguration_Any (Received_Message.Payload));
         begin
            Handle_EntityConfig (This.State, EC);
         end;

      elsif Received_Message.Payload.all in SensorFootprintRequests'Class then
         Handle_SensorFootprintRequests
           (This.State,
            This.Mailbox,
            As_SensorFootprintRequests_Message
              (SensorFootprintRequests_Any (Received_Message.Payload)));

      end if;
      --  RemoveTasks is silently ignored (no state to clear in Ada)

      Should_Terminate := False;
   end Process_Received_LMCP_Message;

   ---------------------------------
   -- Registry_Service_Type_Names --
   ---------------------------------

   function Registry_Service_Type_Names return Service_Type_Names_List is
      (Service_Type_Names_List'(1 => Instance (Service_Type_Name_Max_Length, Content => Type_Name)));

   -----------------------------
   -- Package Executable Part --
   -----------------------------

   --  This is the executable part for the package, invoked automatically and only once.
begin
   --  All concrete service subclasses must call this procedure in their
   --  own package like this, with their own params.
   Register_Service_Creation_Function_Pointers (Registry_Service_Type_Names, Create'Access);
end UxAS.Comms.LMCP_Net_Client.Service.Sensor_Manager_Interfacing;
