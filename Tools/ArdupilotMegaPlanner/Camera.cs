using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    public partial class Camera : Form
    {
        float mm_2_feet = 1 / 304.8f;
        float feet_2_mm = 304.8f;
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        public Camera()
        {
            InitializeComponent();
        }

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            doCalc();
        }

        void doCalc()
        {

            var film_width = 36.0f;
            var film_height = 27.0f;
            var film_diag = 0.0f;

            var flen = (float)num_focallength.Value;
            var flen_mult = (float)num_focalmultip.Value;
            var subj_dist = (float)num_agl.Value;

            //if (isNaN(flen_mult) || flen_mult<=0)
            {
                //f.flen_mult = 1;
                //flen_mult = 1;
            }

            // convert distance to mm
            /*
	if (f.units.value.search(/feet/i) != -1)
	{
		//user input in feet
		subj_dist = subj_dist * feet_2_mm;
	}
	else */
            {
                //user input in m
                subj_dist = subj_dist * 1000;
            }

            //Account for focal length multiplier (actually, a film/sensor size multiplier)
            film_width = film_width / flen_mult;
            film_height = film_height / flen_mult;
            film_diag = (int)(Math.Sqrt((film_width * film_width) + (film_height * film_height)));

            var half_fov_h = (Math.Atan(film_width / (2 * flen)));
            var fov_h = 2 * (subj_dist * Math.Tan(half_fov_h));

            var half_fov_v = (Math.Atan(film_height / (2 * flen)));
            var fov_v = 2 * (subj_dist * Math.Tan(half_fov_v));

            var half_fov_d = (Math.Atan(film_diag / (2 * flen)));
            var fov_d = 2 * (subj_dist * Math.Tan(half_fov_d));

            //convert answer (currently in mm) back to feet
            fov_h = fov_h * mm_2_feet;
            fov_v = fov_v * mm_2_feet;
            fov_d = fov_d * mm_2_feet;
            /*
	if (f.units.value.search(/feet/i) != -1)
    {
    	f.fov_h.value = feet_inches(fov_h);
    	f.fov_v.value = feet_inches(fov_v);
    	f.fov_d.value = feet_inches(fov_d);
    }
    else */
            {
                TXT_fovH.Text = meters(fov_h);
                TXT_fovV.Text = meters(fov_v);
                TXT_fovD.Text = meters(fov_d);

                TXT_fovAH.Text = (half_fov_h * 2 * rad2deg).ToString("0.00");
                TXT_fovAV.Text = (half_fov_v * 2 * rad2deg).ToString("0.00");
                TXT_fovAD.Text = (half_fov_d * 2 * rad2deg).ToString("0.00");

                float test1 = (float)Math.Sqrt((float)num_megapixel.Value * 1000000 * (film_height / film_width));

                TXT_imgwidth.Text = test1.ToString("0");
                TXT_imgheight.Text = (((float)num_megapixel.Value * 1000000) / test1).ToString("0");


                TXT_cmpixel.Text = (((fov_h * feet_2_mm) / 10.0) / test1).ToString("0.000 cm");
            }
        }


        //Takes a distance in feet and converts to string representation in meters/cm
        string meters(double aNumber)
        {
            //if (isNaN(aNumber))
            //return aNumber;

            var mm = aNumber * feet_2_mm;

            var m = Math.Floor(mm / 1000);
            var cm = (mm / 10) % 100;

            return m + "m " + cm.ToString("0.00") + "cm";
        }

        private void num_agl_ValueChanged(object sender, EventArgs e)
        {
            doCalc();
        }

        private void num_megapixel_ValueChanged(object sender, EventArgs e)
        {
            doCalc();
        }

        private void num_focallength_ValueChanged(object sender, EventArgs e)
        {
            doCalc();
        }

        private void num_focalmultip_ValueChanged(object sender, EventArgs e)
        {
            doCalc();
        }
    }
}