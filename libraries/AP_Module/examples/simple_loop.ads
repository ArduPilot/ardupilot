-- SPDX-License-Identifier: GPL-3.0-or-later
with AP_Module;

package Simple_Loop is
   function Run (Ctx : AP_Module.Context) return AP_Module.Status
     with Export, Convention => C, External_Name => "ap_module_ada_run";
end Simple_Loop;
