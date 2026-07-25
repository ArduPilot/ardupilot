--  Package for crc checksum
--  Copyright Fil Andrii root.fi36@gmail.com 2022

with Interfaces; use Interfaces;

package MAVLink.X25CRC is

   pragma Pure (X25CRC);

   type Checksum (Repr : Boolean := True) is record
      case Repr is
         when True =>
            Low, High : Unsigned_8 := 16#FF#;
         when others =>
            Value     : Unsigned_16 := 16#FFFF#;
      end case;
   end record with Size => 16, Unchecked_Union;
   for Checksum use record
      High at 0 range 0 .. 7;
      Low  at 0 range 8 .. 15;

      Value at 0 range 0 .. 15;
   end record;

   procedure Reset (Element : in out Checksum);
   procedure Update (Element : in out Checksum; Value : Unsigned_8);

end MAVLink.X25CRC;
