--  Connects to ardupilot (baud rate 115200) via ttyUSB0 and read all params
--  Copyright Fil Andrii root.fi36@gmail.com 2022s

pragma Ada_2022;

with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Streams;
with Ada.Text_IO;

with GNAT.Serial_Communications;
with Interfaces;

with MAVLink.V1;
with MAVLink.V1.Common.Attitudes;

procedure Attitude is
   use type Ada.Streams.Stream_Element_Offset;
   use type Interfaces.Unsigned_8;
   use type Interfaces.IEEE_Float_32;

   package IEEE_Text_IO is new Ada.Text_IO.Float_IO (Interfaces.IEEE_Float_32);

   Ser        : GNAT.Serial_Communications.Serial_Port;
   Input      : Ada.Streams.Stream_Element_Array(1 .. 1024);
   Input_Last : Ada.Streams.Stream_Element_Offset;

   Mav_Conn : MAVLink.V1.Connection;

   procedure Handler_Attitude is
      Attitude  : MAVLink.V1.Common.Attitudes.Attitude;
      K_Rad2Deg : Interfaces.IEEE_Float_32 := 180.0 / Ada.Numerics.Pi;
   begin
      pragma Assert (MAVLink.V1.Common.Attitudes.Check_CRC (Mav_Conn));

      MAVLink.V1.Common.Attitudes.Decode (Attitude, Mav_Conn);

      Ada.Text_IO.Put ("Pitch: ");
      IEEE_Text_IO.Put (Attitude.Pitch * K_Rad2Deg, Aft => 4, Exp => 0);
      Ada.Text_IO.Put ("   Roll: ");
      IEEE_Text_IO.Put (Attitude.Roll * K_Rad2Deg, Aft => 4, Exp => 0);
      Ada.Text_IO.Put ("   Yaw: ");
      IEEE_Text_IO.Put (Attitude.Yaw * K_Rad2Deg, Aft => 4, Exp => 0);
      Ada.Text_IO.New_Line;
   end Handler_Attitude;

begin
   MAVLink.V1.Set_System_Id (Mav_Conn, 250);
   MAVLink.V1.Set_Component_Id (Mav_Conn, 1);

   Ada.Text_IO.Put_Line
     ("Connects to ardupilot (baud rate 115200) via "
      & "ttyUSB0 and reads attitude angles");

   GNAT.Serial_Communications.Open (Port => Ser, Name => "/dev/ttyUSB0");

   GNAT.Serial_Communications.Set
     (Port    => Ser,
      Rate    => GNAT.Serial_Communications.B115200,
      Block   => True,
      Timeout => 0.0);

   loop
      GNAT.Serial_Communications.Read
        (Port => Ser, Buffer => Input, Last => Input_Last);

      for B of Input (Input'First .. Input_Last) loop
         if MAVLink.V1.Parse_Byte (Mav_Conn, Interfaces.Unsigned_8 (B)) then
            if MAVLink.V1.Get_Msg_Id (Mav_Conn) =
              MAVLink.V1.Common.Attitude_Id
            then
               Handler_Attitude;
            end if;
         end if;
      end loop;
   end loop;
end Attitude;
