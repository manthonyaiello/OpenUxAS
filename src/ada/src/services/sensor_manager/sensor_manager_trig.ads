with Ada.Numerics.Generic_Elementary_Functions;
with Common;               use Common;
with Sensor_Manager_Types; use Sensor_Manager_Types;

--  The shared Real64 elementary-function instance of the Sensor Manager,
--  together with the trigonometric facts its footprint geometry relies
--  on, stated as axioms.
--
--  GNATprove has no theory of Sin and Tan (the runtime's
--  Ada.Numerics.Generic_Elementary_Functions carries no postconditions),
--  so each fact is a ghost procedure whose body is outside SPARK: the
--  postcondition is assumed at call sites, not proved. Each axiom is
--  justified below by elementary real analysis, with a numeric margin
--  (at least 1.0E-5) that generously absorbs the few-ulp error (~1.0E-15
--  at these magnitudes) of any faithful libm implementation. In debug
--  builds (-gnata) the postconditions are compiled and evaluated, so the
--  axioms are exercised at run time rather than blindly trusted.

package Sensor_Manager_Trig with SPARK_Mode is

   package Math is new Ada.Numerics.Generic_Elementary_Functions (Real64);
   --  Real64 instance of the runtime elementary functions

   function Sin (X : Real64) return Real64 renames Math.Sin;
   --  Sine of X, from the Real64 elementary-function instance.
   --  @param X Angle in radians
   --  @return Sine of X

   function Tan (X : Real64) return Real64 renames Math.Tan;
   --  Tangent of X, from the Real64 elementary-function instance.
   --  @param X Angle in radians
   --  @return Tangent of X

   Sin_Working_Floor : constant := 0.0174;
   --  Safe lower bound for Sin over the working elevation range:
   --  sin (1 deg) = 0.0174524..., which 0.0174 floors.

   Tan_Working_Floor : constant := 0.0174;
   --  Safe lower bound for abs Tan over the working elevation range: the
   --  same floor works, since tan (1 deg) = 0.0174550... .

   Half_FOV_Rad_Bound : constant := 1.5621;
   --  Upper end of the half-FOV range: FOV_Deg caps fields of view at
   --  179 deg, so half of the radian value is at most
   --  179 * Pi / 360 = 1.56206968... (89.5 deg); 1.5621 rounds that up
   --  and stays strictly below Pi / 2 = 1.57079632... .

   Tan_Half_FOV_Ceiling : constant := 115.0;
   --  Upper bound for Tan on [0, Half_FOV_Rad_Bound]: Tan is increasing on
   --  [0, Pi/2), so on [0, 1.5621] it is at most tan (1.5621) = 114.988...

   procedure Axiom_Sin_Bounds_On_Working_Range (X : Real64)
     with
       Ghost,
       Global => null,
       Always_Terminates,
       Pre  => X >= One_Degree_Rad and then X <= Pi - One_Degree_Rad,
       Post => Sin (X) >= Sin_Working_Floor and then Sin (X) <= 1.0;
   --  Assumes Sin (X) lies in [Sin_Working_Floor, 1] on the working range.
   --  Sin is concave on [0, Pi] with zeros exactly at the ends, so on
   --  [1 deg, 179 deg] its minimum is at the ends (sin (1 deg), by
   --  symmetry); the maximum of Sin anywhere is 1.
   --  @param X Angle in radians, in [1 deg, 179 deg]

   procedure Axiom_Sin_Bounds_On_Half_Turn (X : Real64)
     with
       Ghost,
       Global => null,
       Always_Terminates,
       Pre  => X >= 0.0 and then X <= Pi,
       Post => Sin (X) >= 0.0 and then Sin (X) <= 1.0;
   --  Assumes Sin (X) lies in [0, 1] on the half-turn [0, Pi]: Sin is
   --  nonnegative there and never exceeds 1.
   --  @param X Angle in radians, in [0, Pi]

   procedure Axiom_Tan_Magnitude_On_Working_Range (X : Real64)
     with
       Ghost,
       Global => null,
       Always_Terminates,
       Pre  => X >= One_Degree_Rad and then X <= Pi - One_Degree_Rad,
       Post => abs Tan (X) >= Tan_Working_Floor;
   --  Assumes abs Tan (X) >= Tan_Working_Floor on the working range.
   --  Since abs tan = abs sin / abs cos, on [1 deg, 179 deg] we have
   --  abs sin >= sin (1 deg) (previous axiom) and abs cos <= 1, so
   --  abs tan (X) >= tan (1 deg); the minimum is attained at the interval
   --  ends, and abs tan diverges at Pi/2 in between.
   --  @param X Angle in radians, in [1 deg, 179 deg]

   procedure Axiom_Tan_Bounds_Below_Vertical (X : Real64)
     with
       Ghost,
       Global => null,
       Always_Terminates,
       Pre  => X >= 0.0 and then X <= Half_FOV_Rad_Bound,
       Post => Tan (X) >= 0.0 and then Tan (X) <= Tan_Half_FOV_Ceiling;
   --  Assumes Tan (X) lies in [0, Tan_Half_FOV_Ceiling] below the vertical.
   --  Tan is zero at 0 and strictly increasing on [0, Pi/2), so on
   --  [0, 1.5621] it lies in [0, tan (1.5621)], which is within [0, 115.0].
   --  @param X Angle in radians, in [0, Half_FOV_Rad_Bound]

end Sensor_Manager_Trig;
