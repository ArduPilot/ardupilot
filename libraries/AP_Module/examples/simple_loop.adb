-- SPDX-License-Identifier: GPL-3.0-or-later
with AP_Module.Bindings;
with AP_Module.Types;
package body Simple_Loop is
   use AP_Module;
   use type Status;

   function Run (Ctx : Context) return Status is
      Result : Status;
   begin
      while not Stop_Requested (Ctx) loop
         Result := Bindings.Gcs.Send_Text (Ctx, Types.MavSeverity_INFO, "Ada tick");
         if Result /= OK then
            return Result;
         end if;
         Result := Sleep_Ms (Ctx, 1000);
         if Result /= OK then
            return Result;
         end if;
      end loop;
      return Stopped;
   end Run;
end Simple_Loop;
