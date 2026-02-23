with AVTAS.LMCP.Types;
with LMCP_Message_Conversions; use LMCP_Message_Conversions;

package body Sensor_Manager_Mailboxes is

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize
     (This         : out Sensor_Manager_Mailbox;
      Source_Group : String;
      Unique_Id    : Int64;
      Entity_Id    : UInt32;
      Service_Id   : UInt32)
   is
   begin
      This.Message_Sender_Pipe.Initialize_Push
        (Source_Group => Source_Group,
         Entity_Id    => AVTAS.LMCP.Types.UInt32 (Entity_Id),
         Service_Id   => AVTAS.LMCP.Types.UInt32 (Service_Id));

      This.Unique_Entity_Send_Message_Id := Unique_Id;
   end Initialize;

   --------------------------
   -- sendBroadcastMessage --
   --------------------------

   procedure sendBroadcastMessage
     (This : in out Sensor_Manager_Mailbox;
      Msg  : Message_Root'Class)
   is
   begin
      This.Unique_Entity_Send_Message_Id :=
        This.Unique_Entity_Send_Message_Id + 1;
      This.Message_Sender_Pipe.Send_Shared_Broadcast_Message
        (As_Object_Any (Msg));
   end sendBroadcastMessage;

end Sensor_Manager_Mailboxes;
