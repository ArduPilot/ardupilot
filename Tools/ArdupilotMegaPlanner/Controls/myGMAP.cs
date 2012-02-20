using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    class myGMAP : GMap.NET.WindowsForms.GMapControl
    {
        public bool inOnPaint = false;
        string otherthread = "";

        protected override void OnPaint(System.Windows.Forms.PaintEventArgs e)
        {
            if (inOnPaint)
            {
                Console.WriteLine("Was in onpaint Gmap th:" + System.Threading.Thread.CurrentThread.Name + " in " + otherthread);
                return;
            }

            otherthread = System.Threading.Thread.CurrentThread.Name;

            inOnPaint = true;

            try
            {
                base.OnPaint(e);
            }
            catch (Exception ex) { Console.WriteLine(ex.ToString()); }

            inOnPaint = false;
        }

        protected override void OnMouseMove(System.Windows.Forms.MouseEventArgs e)
        {
            try
            {
                base.OnMouseMove(e);
            }
            catch (Exception ex) { Console.WriteLine(ex.ToString()); }
        }
    }
}
