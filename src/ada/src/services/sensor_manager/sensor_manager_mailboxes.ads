with Common;        use Common;
with LMCP_Messages; use LMCP_Messages;

private with Ada.Strings.Unbounded;
private with UxAS.Comms.LMCP_Object_Message_Sender_Pipes;

--  Package only concerned with message passing. It defines its own state,
--  named Mailbox here, which is not mixed with the state of the service.

package Sensor_Manager_Mailboxes with SPARK_Mode is

   type Sensor_Manager_Mailbox is limited private;
   --  Message-passing state of the Sensor Manager: the send pipe and the
   --  identity used when broadcasting responses on the message bus.

   procedure Initialize
     (This         : out Sensor_Manager_Mailbox;
      Source_Group : String;
      Unique_Id    : Int64;
      Entity_Id    : UInt32;
      Service_Id   : UInt32);
   --  Set up the mailbox's send pipe and sender identity.
   --  @param This Mailbox to initialize
   --  @param Source_Group Message-bus source group to publish under
   --  @param Unique_Id Unique entity-send message ID seed for this sender
   --  @param Entity_Id ID of the entity this service instance runs for
   --  @param Service_Id ID of this service instance

   procedure sendBroadcastMessage
     (This : in out Sensor_Manager_Mailbox;
      Msg  : Message_Root'Class)
     with Always_Terminates;
   --  Broadcast an LMCP message to all subscribers on the message bus.
   --  @param This Mailbox whose send pipe carries the message
   --  @param Msg The LMCP message to broadcast

private
   pragma SPARK_Mode (Off);

   use Ada.Strings.Unbounded;
   use UxAS.Comms.LMCP_Object_Message_Sender_Pipes;

   type Sensor_Manager_Mailbox is tagged limited record
      Message_Sender_Pipe           : LMCP_Object_Message_Sender_Pipe;
      --  Pipe over which broadcast messages are sent
      Source_Group                  : Unbounded_String;
      --  Message-bus source group this sender publishes under
      Unique_Entity_Send_Message_Id : Int64;
      --  Unique ID stamped on messages sent from this mailbox
   end record;
   --  Full view of the mailbox: the send pipe plus the sender identity.

end Sensor_Manager_Mailboxes;
