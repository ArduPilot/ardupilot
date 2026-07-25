
pragma Ada_2022;

with Interfaces;
private with Ada.Unchecked_Conversion;

package MAVLink.Raw_Floats is

   pragma Pure;

   type Raw_Float is private;

   function Is_Valid (Value : Raw_Float) return Boolean;

   function To_Float (Value : Raw_Float) return Interfaces.IEEE_Float_32
     with Pre => Is_Valid (Value);

   function To_Raw (Value : Interfaces.IEEE_Float_32) return Raw_Float;

private
   type Raw_Float is new Interfaces.Unsigned_32;

   function Unchecked_To_Float is new Ada.Unchecked_Conversion
     (Raw_Float, Interfaces.IEEE_Float_32);

   function Unchecked_To_Raw is new Ada.Unchecked_Conversion
     (Interfaces.IEEE_Float_32, Raw_Float);

   function Is_Valid (Value : Raw_Float) return Boolean is
     (Unchecked_To_Float (Value)'Valid);

   function To_Float (Value : Raw_Float) return Interfaces.IEEE_Float_32 is
     (Unchecked_To_Float (Value));

   function To_Raw (Value : Interfaces.IEEE_Float_32) return Raw_Float is
      (Unchecked_To_Raw (Value));

end MAVLink.Raw_Floats;
