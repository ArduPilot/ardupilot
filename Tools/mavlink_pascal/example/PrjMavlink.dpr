program PrjMavlink;

uses
  System.StartUpCopy,
  FMX.Forms,
  main in 'main.pas' {Form1},
  MAVLink in '..\MAVLink.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TForm1, Form1);
  Application.Run;
end.
