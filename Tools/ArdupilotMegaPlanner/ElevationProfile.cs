using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using ZedGraph;
using GMap.NET;
using System.Xml; // GE xml alt reader

namespace ArdupilotMega
{
    public partial class ElevationProfile : Form
    {
        List<PointLatLngAlt> gelocs = new List<PointLatLngAlt>();
        List<PointLatLngAlt> planlocs = new List<PointLatLngAlt>();
        PointPairList list1 = new PointPairList();
        PointPairList list2 = new PointPairList();
        int distance = 0;
        double homealt = 0;

        public ElevationProfile(List<PointLatLngAlt> locs, double homealt)
        {
            InitializeComponent();

            planlocs = locs;

            if (planlocs.Count <= 1)
            {
                CustomMessageBox.Show("Please plan something first");
                return;
            }

            // get total distance
            distance = 0;
            PointLatLngAlt lastloc = null;
            foreach (PointLatLngAlt loc in planlocs)
            {
                if (lastloc != null) {
                    distance += (int)loc.GetDistance(lastloc);
                }
                lastloc = loc;
            }

            this.homealt = homealt / MainV2.cs.multiplierdist;

            Form frm = Common.LoadingBox("Loading", "Downloading Google Earth Data");

            gelocs = getGEAltPath(planlocs);

            frm.Close();
        }

        private void ElevationProfile_Load(object sender, EventArgs e)
        {
            if (planlocs.Count <= 1)
            {
                this.Close();
                return;
            }
            // GE plot
            double a = 0;
            double increment = (distance / (gelocs.Count - 1));

            foreach (PointLatLngAlt geloc in gelocs)
            {
                list2.Add(a,geloc.Alt);

                Console.WriteLine(geloc.Lng + "," + geloc.Lat + "," + geloc.Alt);

                a+=increment;
            }
            // Planner Plot
            a=0;
            int count = 0;
            PointLatLngAlt lastloc = null;
            foreach (PointLatLngAlt planloc in planlocs)
            {
                if (lastloc != null)
                {
                    a += planloc.GetDistance(lastloc);
                }

                list1.Add(a, planloc.Alt / MainV2.cs.multiplierdist, 0, planloc.Tag); // homealt

                lastloc = planloc;
                count++;
            }
            // draw graph
            CreateChart(zg1);
        }

        List<PointLatLngAlt> getGEAltPath(List<PointLatLngAlt> list)
        {
            double alt = 0;
            double lat = 0;
            double lng = 0;

            int pos = 0;

            List<PointLatLngAlt> answer = new List<PointLatLngAlt>();

            //http://code.google.com/apis/maps/documentation/elevation/
            //http://maps.google.com/maps/api/elevation/xml
            string coords = "";

            foreach (PointLatLngAlt loc in list)
            {
                coords = coords + loc.Lat.ToString(new System.Globalization.CultureInfo("en-US")) + "," + loc.Lng.ToString(new System.Globalization.CultureInfo("en-US")) + "|";
            }
            coords = coords.Remove(coords.Length - 1);

            if (list.Count <= 2 || coords.Length > (2048 - 256) || distance > 50000)
            {
                CustomMessageBox.Show("To many/few WP's or to Big a Distance " + (distance/1000) + "km");
                return answer;
            }

            try
            {
                using (XmlTextReader xmlreader = new XmlTextReader("http://maps.google.com/maps/api/elevation/xml?path=" + coords + "&samples=" + (distance / 100).ToString(new System.Globalization.CultureInfo("en-US")) + "&sensor=false"))
                {
                    while (xmlreader.Read())
                    {
                        xmlreader.MoveToElement();
                        switch (xmlreader.Name)
                        {
                            case "elevation":
                                alt = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                Console.WriteLine("DO it " + lat + " " + lng + " " + alt);
                                PointLatLngAlt loc = new PointLatLngAlt(lat,lng,alt,"");
                                answer.Add(loc);
                                pos++;
                                break;
                            case "lat":
                                lat = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                break;
                            case "lng":
                                lng = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
            catch { CustomMessageBox.Show("Error getting GE data"); }

            return answer;
        }

        public void CreateChart(ZedGraphControl zgc)
        {
            GraphPane myPane = zgc.GraphPane;

            // Set the titles and axis labels
            myPane.Title.Text = "Elevation above ground";
            myPane.XAxis.Title.Text = "Distance (m)";
            myPane.YAxis.Title.Text = "Elevation (m)";

            LineItem myCurve;

            myCurve = myPane.AddCurve("Planner", list1, Color.Red, SymbolType.None);
            myCurve = myPane.AddCurve("GE", list2, Color.Green, SymbolType.None);

            foreach (PointPair pp in list1)
            {
                // Add a another text item to to point out a graph feature
                TextObj text = new TextObj((string)pp.Tag, pp.X, pp.Y);
                // rotate the text 90 degrees
                text.FontSpec.Angle = 90;
                text.FontSpec.FontColor = Color.White;
                // Align the text such that the Right-Center is at (700, 50) in user scale coordinates
                text.Location.AlignH = AlignH.Right;
                text.Location.AlignV = AlignV.Center;
                // Disable the border and background fill options for the text
                text.FontSpec.Fill.IsVisible = false;
                text.FontSpec.Border.IsVisible = false;
                myPane.GraphObjList.Add(text);
            }

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;

            myPane.XAxis.Scale.Min = 0;
            myPane.XAxis.Scale.Max = distance;

            // Make the Y axis scale red
            myPane.YAxis.Scale.FontSpec.FontColor = Color.Red;
            myPane.YAxis.Title.FontSpec.FontColor = Color.Red;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Fill the axis background with a gradient
            //myPane.Chart.Fill = new Fill(Color.White, Color.LightGray, 45.0f);

            // Calculate the Axis Scale Ranges
            try
            {
                zg1.AxisChange();
            }
            catch { }



        }
    }
}
