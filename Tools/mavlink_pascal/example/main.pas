{
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
}

{
 //  Example MAVLink data communication for APM.
 //  Author: Hiroshi Takey, August 2016
 //
}

unit main;

interface

uses
  System.SysUtils, System.Types, System.UITypes, System.Classes, System.Variants, FMX.Forms, FMX.StdCtrls, CPort, FMX.Memo, MAVLink,
  FMX.Types, FMX.Controls, FMX.Controls.Presentation, FMX.ScrollBox;

type

  TForm1 = class(TForm)
    ComPort1: TComPort;
    Button1: TButton;
    Memo1: TMemo;
    Button2: TButton;
    procedure Button1Click(Sender: TObject);
    procedure ComPort1RxChar(Sender: TObject; Count: Integer);
    procedure Button2Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);

  private
    hit:boolean;
    count2:integer;
    inproc:boolean;
    s1: string;
    s2: string;
    datain:string;
    helpers:THelpers;
     { Private declarations }
  public
    packetcount:integer;
    armed:boolean ;

    { Public declarations }
  end;

var
  Form1: TForm1;

implementation

{$R *.fmx}

procedure TForm1.Button1Click(Sender: TObject);
begin
  ComPort1.BaudRate := TBaudRate.br115200;
  ComPort1.Port:= 'COM23';
  ComPort1.Timeouts.ReadInterval:= 100;
  ComPort1.Connected:= true;
end;

procedure TForm1.Button2Click(Sender: TObject);
var
  indata: mavlink_command_long_t;
  indata2: mavlink_param_request_list_t;
  packet:System.TArray<System.Byte>;
  data: System.TArray<System.Byte>;
  i:integer;
  b:byte;
  checksum: UInt16;
  ck_a: byte;
  ck_b: byte;
  bytetrans:integer;
  dataout:string;
begin
    dataout:='';

    indata.target_system:= 1;
    indata.target_component:= 1;
    indata.command:= MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
    indata.param1:= 1;

    indata2.target_system:= 1;
    indata2.target_component:= 1;

    SetLength(data, SizeOf(indata2));
    SetLength(packet, (SizeOf(indata2)+ 6 + 2));
    data:= BytesOf(@indata2, SizeOf(indata2));

    packet[0]:= 254;
    packet[1]:= length(data);
    packet[2]:= byte(packetcount);
    packetcount:= packetcount + 1;
    packet[3]:= 255;
    packet[4]:= MAV_COMPONENT.MAV_COMPONENT_MAV_COMP_ID_MISSIONPLANNER;
    packet[5]:= MAVLINK_MSG_ID.MAVLINK_MSG_ID_PARAM_REQUEST_LIST;

    i:= 6;
    for b in data do
    begin
      packet[i]:= b;
      inc(i);
    end;

    checksum:=  helpers.crc_calculate(packet, packet[1] + 6);
    checksum:= helpers.crc_accumulate(MAVLINK_MESSAGE_CRCS[packet[5]], checksum);

    ck_a := byte(checksum and $FF); ///< High byte
    ck_b := byte(checksum shr 8); ///< Low byte
    packet[i]:= ck_a;
    i := i + 1;
    packet[i]:= ck_b;
    for i := 0 to (length(packet)-1) do
    begin
      dataout:= dataout + char(packet[i]);
    end;
    bytetrans:= ComPort1.WriteStr(dataout);
end;

procedure TForm1.ComPort1RxChar(Sender: TObject; Count: Integer);
var
  packet:System.TArray<System.Byte>;
  i,c:integer;
  checksum: UInt16;
  readbytes:integer;
  b:byte;
  cadena1:string;
begin
    if (count > 5)and(inproc = false) then
    begin
      inproc:= true;
      readbytes:= ComPort1.ReadStr(s1, count);
      for i := 1 to readbytes do
      begin
        if (byte(s1[i]) = MAVLINK_STX) then
        begin
          count2:= byte(s1[i+1]) + 2;
          ComPort1.ReadStr(s2, count2);
          datain:= s1 + s2;
          SetLength(packet, count2 + 6);
          for c := 0 to ((count2 + 6)-1) do
          begin
            packet[c]:= byte(datain[c+i]);
          end;
          checksum:= helpers.crc_calculate(packet, length(packet)-2);
          checksum := helpers.crc_accumulate(MAVLINK_MESSAGE_CRCS[packet[5]], checksum);
          if ((length(packet) < 5) or (packet[length(packet) - 1] <> (checksum shr 8)) or (packet[length(packet) - 2] <> (checksum and $ff))) then
          begin
            helpers.packet_msg_data_set(packet,packet[5],packet[1]);
            Memo1.Lines.Add('FALLA!  msgdata: ' + cadena1 + ' cantidad:' + helpers.msg_data_PARAM_VALUE.param_count.ToString + ' INDEX:'  + helpers.msg_data_PARAM_VALUE.param_index.ToString);
          end
          else
          begin
            helpers.packet_msg_data_set(packet,packet[5],packet[1]);
            cadena1:='';
            for b in helpers.msg_data_PARAM_VALUE.param_id do
            begin
              cadena1:= cadena1 + char(b);
            end;
            Memo1.Lines.Add('msgdata: ' + cadena1 + ' cantidad:' + helpers.msg_data_PARAM_VALUE.param_count.ToString + ' INDEX:'  + helpers.msg_data_PARAM_VALUE.param_index.ToString);
          end;
          inproc:= false;
          break;
        end;
      end;
      if (inproc = true) then
      begin
        inproc:= false;
      end;
    end;
end;

procedure TForm1.FormCreate(Sender: TObject);
begin
  packetcount:= 0;
  armed:= false;
  hit:= false;
  count2:=200;
  inproc:=false;
  s1:= '';
  s2:= '';
  datain:= '';
  helpers:= THelpers.Create;
end;

end.
