--  Connects to ardupilot (baud rate 115200) via ttyUSB0 and read all params
--  Copyright Fil Andrii root.fi36@gmail.com 2022

pragma Ada_2022;

with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Strings.Maps;
with Ada.Streams;
with Ada.Text_IO;

with GNAT.Serial_Communications;
with Interfaces;

with MAVLink.V1;
with MAVLink.V1.Common.Param_Request_Lists;
with MAVLink.V1.Common.Param_Values;

procedure Param_Request_List is
   use type Ada.Streams.Stream_Element_Offset;
   use type Interfaces.Unsigned_8;
   use type Interfaces.IEEE_Float_32;

   package IEEE_Text_IO is new Ada.Text_IO.Float_IO (Interfaces.IEEE_Float_32);

   Ser         : GNAT.Serial_Communications.Serial_Port;
   Input       : Ada.Streams.Stream_Element_Array (1 .. 1024);
   Input_Last  : Ada.Streams.Stream_Element_Offset;
   Output      : MAVLink.V1.Data_Buffer (1 .. 1024);
   Output_Last : Natural := Output'First;

   Mav_Conn    : MAVLink.V1.Connection;

   procedure Handler_Param_Value is
      Param_Value : MAVLink.V1.Common.Param_Values.Param_Value;
   begin
      pragma Assert
        (MAVLink.V1.Common.Param_Values.Check_CRC (Mav_Conn));

      MAVLink.V1.Common.Param_Values.Decode (Param_Value, Mav_Conn);
      Ada.Text_IO.Put (Param_Value.Param_Id & " = ");
      IEEE_Text_IO.Put (Param_Value.Param_Value, Aft => 4, Exp => 0);
      Ada.Text_IO.New_Line;
   end Handler_Param_Value;

   Param_Request_List : MAVLink.V1.Common.
     Param_Request_Lists.Param_Request_List;

begin
   MAVLink.V1.Set_System_Id (Mav_Conn, 250);
   MAVLink.V1.Set_Component_Id (Mav_Conn, 1);

   Ada.Text_IO.Put_Line
     ("Connects to ardupilot (baud rate 115200) via"
      & " ttyUSB0 and read all params");

   GNAT.Serial_Communications.Open (Port => Ser, Name => "/dev/ttyUSB0");

   GNAT.Serial_Communications.Set
     (Port  => Ser,
      Rate  => GNAT.Serial_Communications.B115200,
      Block => True, Timeout => 0.0);

   Param_Request_List.Target_System := 1;
   Param_Request_List.Target_Component := 0;

   MAVLink.V1.Common.Param_Request_Lists.Encode
     (Param_Request_List, Mav_Conn, Output, Output_Last);

   declare
      Buffer : Ada.Streams.Stream_Element_Array
        (Ada.Streams.Stream_Element_Offset (Output'First) ..
             Ada.Streams.Stream_Element_Offset (Output_Last))
        with Import, Address => Output'Address;
   begin
      GNAT.Serial_Communications.Write
        (Port   => Ser,
         Buffer => Buffer);
   end;

   loop
      GNAT.Serial_Communications.Read
        (Port => Ser, Buffer => Input, Last => Input_Last);

      for B of Input (Input'First .. Input_Last) loop
         if MAVLink.V1.Parse_Byte (Mav_Conn, Interfaces.Unsigned_8 (B)) then
            if MAVLink.V1.Get_Msg_Id (Mav_Conn) =
              MAVLink.V1.Common.Param_Value_Id
            then
               Handler_Param_Value;
            end if;
         end if;
      end loop;
   end loop;
end Param_Request_List;
