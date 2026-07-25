--  Connects to ardupilot (baud rate 115200) via ttyUSB0 and set SR0_PARAMS to 11.
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

with MAVLink.V1.Common.Types;
with MAVLink.V1.Common.Param_Sets;
with MAVLink.V1.Common.Param_Values;

procedure Param_Set_SR0_PARAMS is
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

   function Handler_Param_Value return Boolean is
      Param_Value : MAVLink.V1.Common.Param_Values.Param_Value;
   begin
      pragma Assert
        (MAVLink.V1.Common.Param_Values.Check_CRC (Mav_Conn));

      MAVLink.V1.Common.Param_Values.Decode (Param_Value, Mav_Conn);

      if Ada.Strings.Fixed.Trim
        (Source => Param_Value.Param_Id,
         Left   => Ada.Strings.Maps.Null_Set,
         Right  => Ada.Strings.Maps.To_Set (ASCII.Nul)) = "SR0_PARAMS"
      then
         Ada.Text_IO.Put (Param_Value.Param_Id & " = ");
         IEEE_Text_IO.Put (Param_Value.Param_Value, Aft => 4, Exp => 0);
         Ada.Text_IO.New_Line;

         return True;
      end if;
      return False;
   end Handler_Param_Value;

   Param_Set : MAVLink.V1.Common.Param_Sets.Param_Set;

begin
   MAVLink.V1.Set_System_Id (Mav_Conn, 250);
   MAVLink.V1.Set_Component_Id (Mav_Conn, 1);

   Ada.Text_IO.Put_Line
     ("Connects to ardupilot (baud rate 115200) via "
      & "ttyUSB0 and set SR0_PARAMS to 11.");
   Ada.Text_IO.Put_Line ("Warning this app change param SR0_PARAMS to 11!");

   GNAT.Serial_Communications.Open (Port => Ser, Name => "/dev/ttyUSB0");

   GNAT.Serial_Communications.Set
     (Port    => Ser,
      Rate    => GNAT.Serial_Communications.B115200,
      Block   => True,
      Timeout => 0.0);

   Param_Set.Target_System := 1;
   Param_Set.Target_Component := 0;
   Param_Set.Param_Id := Ada.Strings.Fixed.Head
     (Source => "SR0_PARAMS",
      Count  => 16,
      Pad    => ASCII.Nul);
   Param_Set.Param_Value := 11.0;
   Param_Set.Param_Type := MAVLink.V1.Common.Types.Real32;

   MAVLink.V1.Common.Param_Sets.Encode
     (Param_Set, Mav_Conn, Output, Output_Last);

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

   Main_Loop: loop
      GNAT.Serial_Communications.Read
        (Port   => Ser,
         Buffer => Input,
         Last   => Input_Last);

      for B of Input (Input'First .. Input_Last) loop
         if MAVLink.V1.Parse_Byte(Mav_Conn, Interfaces.Unsigned_8 (B)) then
            if MAVLink.V1.Get_Msg_Id (Mav_Conn) =
              MAVLink.V1.Common.Param_Value_Id
            then
               if Handler_Param_Value then
                  exit Main_Loop;
               end if;
            end if;
         end if;
      end loop;
   end loop Main_Loop;
end Param_Set_SR0_PARAMS;
