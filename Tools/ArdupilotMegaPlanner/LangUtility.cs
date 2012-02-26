//this file contains some simple extension methods

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Globalization;
using System.ComponentModel;
using System.Windows.Forms;

namespace ArdupilotMega
{
    static class CultureInfoEx
    {
        public static CultureInfo GetCultureInfo(string name)
        {
            try { return new CultureInfo(name); }
            catch (Exception) { return null; }
        }

        public static bool IsChildOf(this CultureInfo cX, CultureInfo cY)
        {
            
            if (cX == null || cY == null)
                return false;

            CultureInfo c = cX;
            while (!c.Equals(CultureInfo.InvariantCulture))
            {
                if (c.Equals(cY))
                    return true;
                c = c.Parent;
            }
            return false;
        }
    }

    static class ComponentResourceManagerEx
    {
        public static void ApplyResource(this ComponentResourceManager rm, Control ctrl)
        {
            rm.ApplyResources(ctrl, ctrl.Name);
            foreach (Control subctrl in ctrl.Controls)
                ApplyResource(rm, subctrl);

            if (ctrl.ContextMenu != null)
                ApplyResource(rm, ctrl.ContextMenu);


            if (ctrl is DataGridView)
            {
                foreach (DataGridViewColumn col in (ctrl as DataGridView).Columns)
                    rm.ApplyResources(col, col.Name);
            }
        }

        public static void ApplyResource(this ComponentResourceManager rm, Menu menu)
        {
            rm.ApplyResources(menu, menu.Name);
            foreach (MenuItem submenu in menu.MenuItems)
                ApplyResource(rm, submenu);
        }
    }
}
