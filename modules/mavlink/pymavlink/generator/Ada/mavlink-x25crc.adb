--  Package body for crc checksum
--  Copyright Fil Andrii root.fi36@gmail.com 2022

package body MAVLink.X25CRC is

   procedure Reset (Element : in out Checksum) is
   begin
      Element.Low := 16#FF#;
      Element.High := 16#FF#;
   end Reset;

   procedure Update (Element : in out Checksum; Value : Unsigned_8) is
      use type Unsigned_16;

      Tmp : Unsigned_16;
   begin
      Tmp := Unsigned_16 (Value) xor (Element.Value and 16#FF#);
      Tmp := (Tmp xor Shift_Left (Tmp, 4)) and 16#FF#;
      Element.Value :=
        Shift_Right (Element.Value, 8) xor Shift_Left (Tmp, 8) xor
        Shift_Left (Tmp, 3) xor Shift_Right (Tmp, 4);
   end Update;

end MAVLink.X25CRC;
