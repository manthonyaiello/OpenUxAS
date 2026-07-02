package body Sensor_Manager_Trig with SPARK_Mode is

   --  The axiom bodies are null and outside SPARK: GNATprove assumes
   --  their postconditions at call sites instead of proving them. The
   --  mathematical justification of each is with its declaration.

   ---------------------------------------
   -- Axiom_Sin_Bounds_On_Working_Range --
   ---------------------------------------

   procedure Axiom_Sin_Bounds_On_Working_Range (X : Real64) is
      pragma SPARK_Mode (Off);
   begin
      null;
   end Axiom_Sin_Bounds_On_Working_Range;

   ------------------------------------
   -- Axiom_Sin_Bounds_On_Half_Turn  --
   ------------------------------------

   procedure Axiom_Sin_Bounds_On_Half_Turn (X : Real64) is
      pragma SPARK_Mode (Off);
   begin
      null;
   end Axiom_Sin_Bounds_On_Half_Turn;

   ------------------------------------------
   -- Axiom_Tan_Magnitude_On_Working_Range --
   ------------------------------------------

   procedure Axiom_Tan_Magnitude_On_Working_Range (X : Real64) is
      pragma SPARK_Mode (Off);
   begin
      null;
   end Axiom_Tan_Magnitude_On_Working_Range;

   -------------------------------------
   -- Axiom_Tan_Bounds_Below_Vertical --
   -------------------------------------

   procedure Axiom_Tan_Bounds_Below_Vertical (X : Real64) is
      pragma SPARK_Mode (Off);
   begin
      null;
   end Axiom_Tan_Bounds_Below_Vertical;

end Sensor_Manager_Trig;
