-- SPDX-License-Identifier: GPL-3.0-or-later
-- AI-assisted sine demo sharing SINE_FREQ with Lua, Rust and Fortran.
with AP_Module.Bindings;
with AP_Module.Statuses;
with AP_Module.Types;
with Interfaces.C;

package body Sine is
   use AP_Module;
   use type Status;
   use type Interfaces.Unsigned_64;
   use type Interfaces.C.C_float;

   package B renames AP_Module.Bindings;
   package S renames AP_Module.Statuses;
   subtype Float32 is Interfaces.C.C_float;
   Pi : constant Float32 := 3.141592653589793;

   function Register (Ctx : Context) return Status is
      Code : Status;
      Added : Interfaces.C.C_bool;
      Frequency : AP_Module.Types.Optional_Float;
   begin
      Code := B.Param.Get (Ctx, "SINE_FREQ", Frequency);
      if Code /= OK then return Code; end if;
      if Boolean (Frequency.Present) then return OK; end if;
      Code := B.Param.Add_Table (Ctx, 120, "SINE_", 1, Added);
      if Code /= OK then return Code; end if;
      -- Lua may have created its table but not yet published the entry.
      if not Boolean (Added) then return S.BUSY; end if;
      Code := B.Param.Add_Param (Ctx, 120, 1, "FREQ", 0.2, Added);
      if Code /= OK then return Code; end if;
      return (if Boolean (Added) then OK else S.HOST_ERROR);
   end Register;

   function Sample (Ctx : Context) return Status is
      Code : Status;
      Frequency : AP_Module.Types.Optional_Float;
      Seconds, Value : Float32;
   begin
      Code := B.Param.Get (Ctx, "SINE_FREQ", Frequency);
      if Code /= OK then return Code; end if;
      if not Boolean (Frequency.Present) then return S.INVALID; end if;
      Seconds := Float32 (Interfaces.Unsigned_32 (Time_Us (Ctx) / 1000 mod 2 ** 32)) * 0.001;
      Code := B.Libm.Sinf (Ctx, 2.0 * Pi * Frequency.Value * Seconds + Pi, Value);
      if Code /= OK then return Code; end if;
      return B.Gcs.Send_Named_Float (Ctx, "Sine_ada", Value);
   end Sample;

   function Run (Ctx : Context) return Status is
      Code : Status;
   begin
      loop
         Code := Register (Ctx);
         exit when Code /= S.BUSY;
         Code := Sleep_Ms (Ctx, 10);
         if Code /= OK then return Code; end if;
      end loop;
      if Code /= OK then return Code; end if;
      loop
         Code := Sample (Ctx);
         if Code /= OK and then Code /= S.BUSY then return Code; end if;
         Code := Sleep_Ms (Ctx, 100);
         if Code /= OK then return Code; end if;
      end loop;
   end Run;
end Sine;
