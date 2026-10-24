-- SPDX-License-Identifier: GPL-3.0-or-later
-- Experimental small API, not a complete generated Ada binding catalogue.
with Interfaces;
with Interfaces.C;
with System;

package AP_Module is
   pragma Preelaborate;

   -- Borrows the worker entry frame: do not retain after Run returns.
   subtype Context is System.Address;
   subtype Status is Interfaces.C.int;
   OK      : constant Status := 0;
   Stopped : constant Status := 1;

   type Severity is (Emergency, Alert, Critical, Error, Warning, Notice, Info, Debug);
   for Severity use (Emergency => 0, Alert => 1, Critical => 2, Error => 3,
                     Warning => 4, Notice => 5, Info => 6, Debug => 7);

   function Send_Text (Ctx : Context; Level : Severity; Message : String) return Status;
   function Stop_Requested (Ctx : Context) return Boolean;
   function Time_Us (Ctx : Context) return Interfaces.Unsigned_64
     with Import, Convention => C, External_Name => "ap_module_ada_time_us";
   function Sleep_Ms (Ctx : Context; Milliseconds : Interfaces.C.unsigned) return Status
     with Import, Convention => C, External_Name => "ap_module_ada_sleep_ms";
end AP_Module;
