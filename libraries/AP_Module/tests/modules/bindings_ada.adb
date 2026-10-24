-- SPDX-License-Identifier: GPL-3.0-or-later
with AP_Module.Bindings;
with AP_Module.Statuses;
with AP_Module.Types;
with Interfaces;
with Interfaces.C;

package body Bindings_Ada is
   use AP_Module;
   use type Status;
   use type Interfaces.C.C_float;
   use type Interfaces.Unsigned_8;
   use type Interfaces.Unsigned_16;
   use type Interfaces.Unsigned_32;

   function Run (Ctx : Context) return Status is
      V : Types.Vector3f := (X => 3.0, Y => 4.0, Z => 12.0);
      Gyro : Types.Vector3f := (X => 99.0, Y => 99.0, Z => 99.0);
      Magnitude : Interfaces.C.C_float;
      Code : Status;
      Filename : constant String := "ada-fixture.bin";
      Path : Types.Array_uint8_t_256 := (others => 0);
      Stat : Types.Optional_filesystem_statOutput;
      Data : Types.Optional_filesystem_read_atOutput;
      Message : constant String (5 .. 24) := "Ada bindings checked";
   begin
      Code := Bindings.Vector3f.Length (Ctx, V, Magnitude);
      if Code /= OK or else Magnitude /= 13.0 then
         return Statuses.HOST_ERROR;
      end if;
      Code := Bindings.Vector3f.Normalize (Ctx, V);
      if Code /= OK then
         return Code;
      end if;
      Code := Bindings.Vector3f.Length (Ctx, V, Magnitude);
      if Code /= OK or else abs (Magnitude - 1.0) > 0.00001 then
         return Statuses.HOST_ERROR;
      end if;
      Code := Bindings.Libm.Sqrtf (Ctx, 169.0, Magnitude);
      if Code /= OK or else Magnitude /= 13.0 then
         return Statuses.HOST_ERROR;
      end if;
      Code := Bindings.Ahrs.Get_Gyro (Ctx, Gyro);
      if Code = OK then
         Code := Send_Text (Ctx, Info, "Ada AHRS checked");
      elsif Code /= Statuses.UNAVAILABLE or else Gyro.X /= 0.0 or else Gyro.Y /= 0.0 or else Gyro.Z /= 0.0 then
         return Statuses.HOST_ERROR;
      end if;
      for I in Filename'Range loop
         Path (I - 1) := Interfaces.Unsigned_8 (Character'Pos (Filename (I)));
      end loop;
      Code := Bindings.Filesystem.Stat (Ctx, Path, Stat);
      if Code /= OK or else not Boolean (Stat.Present) or else Stat.Value.Size /= 3
        or else Boolean (Stat.Value.Directory)
      then
         return Statuses.HOST_ERROR;
      end if;
      Code := Bindings.Filesystem.Read_At (Ctx, Path, 1, 2, Data);
      if Code /= OK or else not Boolean (Data.Present) or else Data.Value.Length /= 2
        or else Data.Value.Data (0) /= Character'Pos ('b') or else Data.Value.Data (1) /= Character'Pos ('c')
      then
         return Statuses.HOST_ERROR;
      end if;
      Path (0) := Character'Pos ('!');
      Code := Bindings.Filesystem.Stat (Ctx, Path, Stat);
      if Code /= OK or else Boolean (Stat.Present) or else Stat.Value.Size /= 0 then
         return Statuses.HOST_ERROR;
      end if;
      Code := Bindings.Gcs.Send_Text (Ctx, Types.MavSeverity_INFO, Message);
      if Code /= OK and then Code /= Statuses.UNAVAILABLE then
         return Code;
      end if;
      return Send_Text (Ctx, Info, "MATH ok");
   end Run;
end Bindings_Ada;
