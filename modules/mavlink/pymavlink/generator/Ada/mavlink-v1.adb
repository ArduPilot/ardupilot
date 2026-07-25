--  MAVLink connection
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

package body MAVLink.V1 is

   -------------------
   -- Set_System_Id --
   -------------------

   procedure Set_System_Id
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8) is
   begin
      Self.System_Id := Value;
   end Set_System_Id;

   ----------------------
   -- Set_Component_Id --
   ----------------------

   procedure Set_Component_Id
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8) is
   begin
      Self.Component_Id := Value;
   end Set_Component_Id;

   ---------------------
   -- Set_Sequency_Id --
   ---------------------

   procedure Set_Sequency_Id
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8) is
   begin
      Self.Out_Sequency := Value;
   end Set_Sequency_Id;

   -------------------
   -- Set_System_Id --
   -------------------

   procedure Set_System_Id
     (Self  : in out Out_Connection;
      Value : Interfaces.Unsigned_8) is
   begin
      Self.System_Id := Value;
   end Set_System_Id;

   ----------------------
   -- Set_Component_Id --
   ----------------------

   procedure Set_Component_Id
     (Self  : in out Out_Connection;
      Value : Interfaces.Unsigned_8) is
   begin
      Self.Component_Id := Value;
   end Set_Component_Id;

   ---------------------
   -- Set_Sequency_Id --
   ---------------------

   procedure Set_Sequency_Id
     (Self  : in out Out_Connection;
      Value : Interfaces.Unsigned_8) is
   begin
      Self.Out_Sequency := Value;
   end Set_Sequency_Id;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Incoming : in out Incoming_Data;
      Val      : Interfaces.Unsigned_8)
      return Boolean
   is
      use type Interfaces.Unsigned_8;
   begin
      if Incoming.In_Ptr = 0
        and then Val /= Version_1_Code
      then
         return False;
      end if;

      Incoming.In_Ptr := Incoming.In_Ptr + 1;
      Incoming.In_Buf (Incoming.In_Ptr) := Val;

      if Incoming.In_Ptr = Pos_Len then
         Incoming.Len := Incoming.In_Buf'First +
           Packet_Payload_First + --  header
             Natural (Incoming.In_Buf (Pos_Len)) + --  data len
           1; --  x25crc checksum -1

         X25CRC.Reset (Incoming.Checksum);
         X25CRC.Update (Incoming.Checksum, Val);
         Incoming.Extras_Added := False;

      elsif Incoming.In_Ptr = Incoming.Len then
         Incoming.In_Ptr := 0;
         return True;

      elsif Incoming.In_Ptr <= Incoming.Len - 2 then
         X25CRC.Update (Incoming.Checksum, Val);
      end if;

      return False;
   end Parse_Byte;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Conn : in out Connection;
      Val  : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Parse_Byte (Conn.Incoming, Val);
   end Parse_Byte;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Conn : in out In_Connection;
      Val  : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Parse_Byte (Conn.Incoming, Val);
   end Parse_Byte;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer;
      Last     : out Natural)
   is
      Len : constant Natural := Natural'Min
        (Buffer'Length,
         (if Incoming.In_Ptr = 0 then Incoming.Len else Incoming.In_Ptr));
   begin
      Last := Buffer'First + Len - 1;
      Buffer (Buffer'First .. Last) := Incoming.In_Buf
        (Incoming.In_Buf'First .. Incoming.In_Buf'First + Len - 1);
   end Get_Buffer;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Conn   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) is
   begin
      Get_Buffer (Conn.Incoming, Buffer, Last);
   end Get_Buffer;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Conn   : In_Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) is
   begin
      Get_Buffer (Conn.Incoming, Buffer, Last);
   end Get_Buffer;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Incoming : in out Incoming_Data;
      Extras   : Interfaces.Unsigned_8)
      return Boolean is
   begin
      if not Incoming.Extras_Added then
         X25CRC.Update (Incoming.Checksum, Extras);
         Incoming.Extras_Added := True;
      end if;

      return Incoming.Checksum.High = Incoming.In_Buf (Incoming.Len - 1)
        and Incoming.Checksum.Low = Incoming.In_Buf (Incoming.Len);
   end Is_CRC_Valid;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Conn   : in out Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Is_CRC_Valid (Conn.Incoming, Extras);
   end Is_CRC_Valid;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Conn   : in out In_Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Is_CRC_Valid (Conn.Incoming, Extras);
   end Is_CRC_Valid;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer)
   is
      Header    : constant V1_Header with Import,
        Address => Incoming.In_Buf'Address;
      Last_Data : constant Positive := Incoming.In_Buf'First +
        Packet_Payload_First +
          Natural (Header.Len - 1);
      Last : constant Natural := Buffer'First + Natural (Header.Len - 1);
   begin
      Buffer (Buffer'First .. Last) := Incoming.In_Buf
        (Incoming.In_Buf'First + Packet_Payload_First .. Last_Data);
   end Get_Message_Data;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Conn   : Connection;
      Buffer : out Data_Buffer) is
   begin
      Get_Message_Data (Conn.Incoming, Buffer);
   end Get_Message_Data;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Conn   : In_Connection;
      Buffer : out Data_Buffer) is
   begin
      Get_Message_Data (Conn.Incoming, Buffer);
   end Get_Message_Data;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Conn   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode (Conn.System_Id, Conn.Component_Id, Conn.Out_Sequency,
              Id, Extras, Buffer, Last);
      Conn.Out_Sequency := Conn.Out_Sequency + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Conn   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode (Conn.System_Id, Conn.Component_Id, Conn.Out_Sequency,
              Id, Extras, Buffer, Last);
      Conn.Out_Sequency := Conn.Out_Sequency + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8;
      Sequency     : Interfaces.Unsigned_8;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive)
   is
      Header   : V1_Header with Import,
        Address => Buffer (Buffer'First)'Address;
      Checksum : X25CRC.Checksum;
   begin
      Header.Stx     := Version_1_Code;
      Header.Len     := Interfaces.Unsigned_8 (Last - Packet_Payload_First);
      Header.Seq     := Sequency;
      Header.Sys_Id  := System_Id;
      Header.Comp_Id := Component_Id;
      Header.Msg_Id  := Id;

      for B of Buffer (Buffer'First + 1 .. Last) loop
         X25CRC.Update (Checksum, B);
      end loop;
      X25CRC.Update (Checksum, Extras);

      Buffer (Last + 1) := Checksum.High;
      Buffer (Last + 2) := Checksum.Low;
      Last := Last + 2;
   end Encode;

end MAVLink.V1;
