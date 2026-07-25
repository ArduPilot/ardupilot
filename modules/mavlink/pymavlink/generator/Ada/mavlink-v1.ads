--  MAVLink
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

pragma Ada_2022;

with Interfaces;              use Interfaces;
with MAVLink.Raw_Floats;      use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats; use MAVLink.Raw_Long_Floats;

private with MAVLink.X25CRC;

package MAVLink.V1 is

   pragma Pure;

   subtype Msg_Id is Interfaces.Unsigned_8;

   -- Arrays --

   type Data_Buffer is array (Positive range <>) of Interfaces.Unsigned_8;

   Maximum_Buffer_Len : constant Positive := 263;

   Version_1_Code     : constant Interfaces.Unsigned_8 := 16#FE#;

   type Unsigned_8_Array is array (Natural range <>) of Interfaces.Unsigned_8
     with Component_Size => 8;
   type Unsigned_16_Array is array (Natural range <>) of Interfaces.Unsigned_16
     with Component_Size => 16;
   type Unsigned_32_Array is array (Natural range <>) of Interfaces.Unsigned_32
     with Component_Size => 32;
   type Unsigned_64_Array is array (Natural range <>) of Interfaces.Unsigned_64
     with Component_Size => 64;
   type Integer_8_Array is array (Natural range <>) of Interfaces.Integer_8
     with Component_Size => 8;
   type Integer_16_Array is array (Natural range <>) of Interfaces.Integer_16
     with Component_Size => 16;
   type Integer_32_Array is array (Natural range <>) of Interfaces.Integer_32
     with Component_Size => 32;
   type Integer_64_Array is array (Natural range <>) of Interfaces.Integer_64
     with Component_Size => 64;
   type Short_Float_Array is array (Natural range <>) of Raw_Float
     with Component_Size => 32;
   type Long_Float_Array is array (Natural range <>) of Raw_Long_Float
     with Component_Size => 64;

   ----------------
   -- Connection --
   ----------------

   type Connection is private;

   procedure Set_System_Id
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8);

   procedure Set_Component_Id
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8);

   procedure Set_Sequency_Id
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8);

   function Parse_Byte
     (Conn : in out Connection;
      Val  : Interfaces.Unsigned_8)
      return Boolean with Inline;
   --  Returns True when the full message has been loaded to the internal
   --  buffer. After this, you can use Check_CRC and Decode from the
   --  corresponding message according to the Get_Msg_Id result.

   function Get_Target_System_Id
     (Conn : Connection) return Interfaces.Unsigned_8 with Inline;

   function Get_Target_Component_Id
     (Conn : Connection) return Interfaces.Unsigned_8 with Inline;

   function Get_Message_Sequnce
     (Conn : Connection) return Interfaces.Unsigned_8 with Inline;
   --  Returns message's Seq

   function Get_Msg_Id (Conn : Connection) return Msg_Id with Inline;

   procedure Get_Buffer
     (Conn   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) with Inline;
   --  Returns data from the internal buffer filled with Parse_Byte.

   -------------------
   -- In_Connection --
   -------------------

   type In_Connection is private;

   function Parse_Byte
     (Conn : in out In_Connection;
      Val  : Interfaces.Unsigned_8)
      return Boolean with Inline;

   function Get_Target_System_Id
     (Conn : In_Connection) return Interfaces.Unsigned_8 with Inline;

   function Get_Target_Component_Id
     (Conn : In_Connection) return Interfaces.Unsigned_8 with Inline;

   function Get_Message_Sequnce
     (Conn : In_Connection) return Interfaces.Unsigned_8 with Inline;
   --  Returns message's Seq

   function Get_Msg_Id (Conn : In_Connection) return Msg_Id with Inline;

   procedure Get_Buffer
     (Conn   : In_Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) with Inline;

   --------------------
   -- Out_Connection --
   --------------------

   type Out_Connection is private;

   procedure Set_System_Id
     (Self  : in out Out_Connection;
      Value : Interfaces.Unsigned_8);

   procedure Set_Component_Id
     (Self  : in out Out_Connection;
      Value : Interfaces.Unsigned_8);

   procedure Set_Sequency_Id
     (Self  : in out Out_Connection;
      Value : Interfaces.Unsigned_8);

private

   type V1_Header is record
      Stx     : Interfaces.Unsigned_8;
      Len     : Interfaces.Unsigned_8;
      Seq     : Interfaces.Unsigned_8;
      Sys_Id  : Interfaces.Unsigned_8;
      Comp_Id : Interfaces.Unsigned_8;
      Msg_Id  : Interfaces.Unsigned_8;
   end record;

   for V1_Header use record
      Stx       at 0 range 0 .. 7;
      Len       at 1 range 0 .. 7;
      Seq       at 2 range 0 .. 7;
      Sys_Id    at 3 range 0 .. 7;
      Comp_Id   at 4 range 0 .. 7;
      Msg_Id    at 5 range 0 .. 7;
   end record;

   Packet_Marker_Length     : constant := 1;
   Packet_Header_Length     : constant := 5;
   Packet_Checksum_Length   : constant := 2;

   Packet_Control_Info_Size : constant :=
     Packet_Marker_Length + Packet_Header_Length + Packet_Checksum_Length;


   Pos_Len                 : constant Natural := 2;
   Pos_Sequence            : constant Natural := 3;
   Pos_Target_System_Id    : constant Natural := 4;
   Pos_Target_Component_Id : constant Natural := 5;
   Pos_Msg_Id              : constant Natural := 6;

   type Incoming_Data is record
      In_Buf       : Data_Buffer (1 .. Maximum_Buffer_Len) := [others => 0];
      In_Ptr       : Natural := 0;
      Len          : Natural := 0;
      Checksum     : X25CRC.Checksum;
      Extras_Added : Boolean := False;
   end record;

   -- Connection --

   type Connection is record
      System_Id    : Interfaces.Unsigned_8 := 1;
      Component_Id : Interfaces.Unsigned_8 := 1;
      Incoming     : Incoming_Data;
      Out_Sequency : Interfaces.Unsigned_8 := 0;
   end record;

   function Get_Target_System_Id
     (Conn : Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Target_System_Id));

   function Get_Target_Component_Id
     (Conn : Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Target_Component_Id));

   function Get_Message_Sequnce
     (Conn : Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Sequence));

   function Get_Msg_Id (Conn : Connection) return Msg_Id
   is (Conn.Incoming.In_Buf (Pos_Msg_Id));

   function Get_Msg_Len (Conn : Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Len));

   procedure Encode
     (Conn   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);

   function Is_CRC_Valid
     (Conn   : in out Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Message_Data
     (Conn   : Connection;
      Buffer : out Data_Buffer);

   -- In_Connection --

   type In_Connection is record
      Incoming : Incoming_Data;
   end record;

   function Get_Target_System_Id
     (Conn : In_Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Target_System_Id));

   function Get_Target_Component_Id
     (Conn : In_Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Target_Component_Id));

   function Get_Message_Sequnce
     (Conn : In_Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Sequence));

   function Get_Msg_Id (Conn : In_Connection) return Msg_Id
   is (Conn.Incoming.In_Buf (Pos_Msg_Id));

   function Get_Msg_Len (Conn : In_Connection) return Interfaces.Unsigned_8
   is (Conn.Incoming.In_Buf (Pos_Len));

   function Is_CRC_Valid
     (Conn   : in out In_Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Message_Data
     (Conn   : In_Connection;
      Buffer : out Data_Buffer);

   -- Out_Connection --

   type Out_Connection is record
      System_Id    : Interfaces.Unsigned_8 := 1;
      Component_Id : Interfaces.Unsigned_8 := 1;
      Out_Sequency : Interfaces.Unsigned_8 := 0;
   end record;

   procedure Encode
     (Conn   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);

   -- Utils --

   function Parse_Byte
     (Incoming : in out Incoming_Data;
      Val      : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Buffer
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer;
      Last     : out Natural);

   function Is_CRC_Valid
     (Incoming : in out Incoming_Data;
      Extras   : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Message_Data
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer);

   procedure Encode
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8;
      Sequency     : Interfaces.Unsigned_8;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive);

   --  Positions of the messages parts minus 1, to use in construction like
   --  Buff'First + Packet_Payload_First
   Packet_Payload_First : constant :=
     Packet_Header_Length + Packet_Marker_Length;

end MAVLink.V1;
