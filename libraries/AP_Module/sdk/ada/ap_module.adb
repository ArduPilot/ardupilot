-- SPDX-License-Identifier: GPL-3.0-or-later
package body AP_Module is
   use type Interfaces.C.int;

   function Raw_Text
     (Ctx : Context; Level : Interfaces.C.int; Message : System.Address;
      Length : Interfaces.C.unsigned) return Status
     with Import, Convention => C, External_Name => "ap_module_ada_text";

   function Raw_Stop_Requested (Ctx : Context) return Interfaces.C.int
     with Import, Convention => C, External_Name => "ap_module_ada_stop_requested";

   procedure Panic_Assertion
     with Import, Convention => C, External_Name => "ap_module_ada_assertion_failed", No_Return;

   -- Keep GNAT's unconstrained String calling convention on the Ada side.
   procedure Raise_Assertion (Message : String)
     with Export, External_Name => "system__assertions__raise_assert_failure", No_Return;

   procedure Raise_Assertion (Message : String) is
      pragma Unreferenced (Message);
   begin
      Panic_Assertion;
   end Raise_Assertion;

   function Send_Text (Ctx : Context; Level : Severity; Message : String) return Status is
   begin
      return Raw_Text (Ctx, Severity'Pos (Level), Message'Address, Message'Length);
   end Send_Text;

   function Stop_Requested (Ctx : Context) return Boolean is
   begin
      return Raw_Stop_Requested (Ctx) /= 0;
   end Stop_Requested;
end AP_Module;
