using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System.Drawing.Imaging;
using System.Drawing;
using GMap.NET;
using GMap.NET.WindowsForms;

namespace ArdupilotMega.Controls
{
    public class OpenGLtest : GLControl
    {
        public static OpenGLtest instance;

        // terrain image
        Bitmap _terrain = new Bitmap(640,480);
        int texture = 0;

        float _angle = 0;
        double cameraX, cameraY, cameraZ;       // camera coordinates
        double lookX, lookY, lookZ;             // camera look-at coordinates

        double step = 1 / 1200.0;

        // image zoom level
        int zoom = 11;

        RectLatLng area = new RectLatLng(-35.04286,117.84262,0.1,0.1);

        double _alt = 0;
        public PointLatLngAlt LocationCenter { 
            get { 
                return new PointLatLngAlt(area.LocationMiddle.Lat, area.LocationMiddle.Lng,_alt,"");
            }
            set {

                if (area.LocationMiddle.Lat == value.Lat && area.LocationMiddle.Lng == value.Lng)
                    return;

                if (value.Lat == 0 && value.Lng == 0)
                    return;

                _alt = value.Alt;
                area = new RectLatLng(value.Lat + 0.15, value.Lng - 0.15, 0.3, 0.3);
               // Console.WriteLine(area.LocationMiddle + " " + value.ToString());
                this.Invalidate();
            } 
        }

        public Vector3 rpy = new Vector3();

        public OpenGLtest()
        {
            instance = this;

            InitializeComponent();

            GL.GenTextures(1, out texture);
        }

        void getImage()
        {
            MapType type = MapType.GoogleSatellite;
            PureProjection prj = null;
            int maxZoom;

            GMaps.Instance.AdjustProjection(type, ref prj, out maxZoom);
          //int zoom = 14; // 12
            if (!area.IsEmpty)
            {
                try
                {
                    List<GPoint> tileArea = prj.GetAreaTileList(area, zoom, 0);
                    //string bigImage = zoom + "-" + type + "-vilnius.png";

                    //Console.WriteLine("Preparing: " + bigImage);
                    //Console.WriteLine("Zoom: " + zoom);
                    //Console.WriteLine("Type: " + type.ToString());
                    //Console.WriteLine("Area: " + area);

                    var types = GMaps.Instance.GetAllLayersOfType(type);

                    // current area
                    GPoint topLeftPx = prj.FromLatLngToPixel(area.LocationTopLeft, zoom);
                    GPoint rightButtomPx = prj.FromLatLngToPixel(area.Bottom, area.Right, zoom);
                    GPoint pxDelta = new GPoint(rightButtomPx.X - topLeftPx.X, rightButtomPx.Y - topLeftPx.Y);

                    DateTime startimage = DateTime.Now;

                    int padding = 0;
                    {
                        using (Bitmap bmpDestination = new Bitmap(pxDelta.X + padding * 2, pxDelta.Y + padding * 2))
                        {
                            using (Graphics gfx = Graphics.FromImage(bmpDestination))
                            {
                                gfx.CompositingMode = System.Drawing.Drawing2D.CompositingMode.SourceOver;

                                // get tiles & combine into one
                                foreach (var p in tileArea)
                                {
                                   Console.WriteLine("Downloading[" + p + "]: " + tileArea.IndexOf(p) + " of " + tileArea.Count);

                                    foreach (MapType tp in types)
                                    {
                                        Exception ex;
                                        WindowsFormsImage tile = GMaps.Instance.GetImageFrom(tp, p, zoom, out ex) as WindowsFormsImage;
                                        if (tile != null)
                                        {
                                            using (tile)
                                            {
                                                int x = p.X * prj.TileSize.Width - topLeftPx.X + padding;
                                                int y = p.Y * prj.TileSize.Width - topLeftPx.Y + padding;
                                                {
                                                    gfx.DrawImage(tile.Img, x, y, prj.TileSize.Width, prj.TileSize.Height);
                                                }
                                            }
                                        }
                                    }
                                    if ((DateTime.Now - startimage).TotalMilliseconds > 200)
                                        break;
                                }
                            }
                            _terrain = new Bitmap(bmpDestination, 512, 512);


                            GL.BindTexture(TextureTarget.Texture2D, texture);

                            BitmapData data = _terrain.LockBits(new System.Drawing.Rectangle(0, 0, _terrain.Width, _terrain.Height),
                    ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                            //Console.WriteLine("w {0} h {1}",data.Width, data.Height);

                            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, data.Width, data.Height, 0,
                                OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, data.Scan0);

                            _terrain.UnlockBits(data);

                            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
                            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);


                        }
                    }
                    if ((DateTime.Now - startimage).TotalMilliseconds > 200)
                    {
                        zoom--;
                    }
                    else
                    {
                        //zoom++;
                    }
                }
                catch { }
            }
        }

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        public Vector3 Normal(Vector3 a, Vector3 b, Vector3 c)
        {
                var dir = Vector3.Cross(b - a, c - a);
                var norm = Vector3.Normalize(dir);
                return norm;
        }


        protected override void OnPaint(System.Windows.Forms.PaintEventArgs e)
        {
            if (this.DesignMode)
                return;

            if (area.LocationMiddle.Lat == 0 && area.LocationMiddle.Lng == 0)
                return;

            _angle+=1f;

           // area.LocationTopLeft = new PointLatLng(area.LocationTopLeft.Lat + 0.0001,area.LocationTopLeft.Lng);

            //area.Size = new SizeLatLng(0.1, 0.1);

            try
            {
                base.OnPaint(e);

            }
            catch { return;  }

            double heightscale = (step / 90.0) * 4;

            float scale = 1.0f;

            float radians = (float)(Math.PI * (rpy.Z * -1) / 180.0f);

             //radians = 0;

            float mouseY = (float)(0.1 * scale);
 
      cameraX = area.LocationMiddle.Lng;     // multiplying by mouseY makes the
      cameraZ = area.LocationMiddle.Lat;    // camera get closer/farther away with mouseY
      cameraY = (LocationCenter.Alt < srtm.getAltitude(cameraZ, cameraX, 20)) ? (srtm.getAltitude(cameraZ, cameraX, 20)+ 0.2) * heightscale : LocationCenter.Alt * heightscale;// (srtm.getAltitude(lookZ, lookX, 20) + 100) * heighscale;


      lookX = area.LocationMiddle.Lng + Math.Sin(radians) * mouseY; ;
      lookY = cameraY;
      lookZ = area.LocationMiddle.Lat + Math.Cos(radians) * mouseY; ;


            MakeCurrent();


            GL.MatrixMode(MatrixMode.Projection);

            OpenTK.Matrix4 projection = OpenTK.Matrix4.CreatePerspectiveFieldOfView(60 * deg2rad, 1f, 0.00001f, 5000.0f);
            GL.LoadMatrix(ref projection);

            Matrix4 modelview = Matrix4.LookAt((float)cameraX, (float)cameraY, (float)cameraZ, (float)lookX, (float)lookY, (float)lookZ, 0,1,0);
            GL.MatrixMode(MatrixMode.Modelview);

            // roll
            modelview = Matrix4.Mult(modelview,Matrix4.CreateRotationZ (rpy.X * deg2rad));
            // pitch
            modelview = Matrix4.Mult(modelview, Matrix4.CreateRotationX(rpy.Y * -deg2rad));

            GL.LoadMatrix(ref modelview);

            GL.ClearColor(Color.Blue);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            GL.LightModel(LightModelParameter.LightModelAmbient,new float[] {1f,1f,1f,1f});

            GL.Enable(EnableCap.Texture2D);
            GL.BindTexture(TextureTarget.Texture2D, texture);
            /*
            GL.Begin(BeginMode.LineStrip);

            GL.Color3(Color.White);
            GL.Vertex3(0, 0, 0);

            GL.Vertex3(area.Bottom, 0, area.Left);

            GL.Vertex3(lookX, lookY, lookZ);

            //GL.Vertex3(cameraX, cameraY, cameraZ);

            GL.End();
            */
            System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

            sw.Start();

            getImage();

            sw.Stop();

            Console.WriteLine("img " +sw.ElapsedMilliseconds);

            sw.Start();

           double increment = step * 1;

            double cleanup = area.Bottom % increment;
            double cleanup2 = area.Left % increment;

            for (double z = (area.Bottom - cleanup); z < area.Top - step; z += increment)
            {
                //Makes OpenGL draw a triangle at every three consecutive vertices
                GL.Begin(BeginMode.TriangleStrip);
                for (double x = (area.Left - cleanup2); x < area.Right - step; x += increment)
                {
                    int heightl = srtm.getAltitude(z, area.Right + area.Left - x, 20);

                 //   Console.WriteLine(x + " " + z);

                    GL.Color3(Color.White);


                   // int heightl = 0;

                    double scale2 = (Math.Abs(x - area.Left) / area.WidthLng);// / (float)_terrain.Width;

                    double scale3 = (Math.Abs(z - area.Bottom) / area.HeightLat);// / (float)_terrain.Height;

                    double imgx = 1 - scale2;
                    double imgy = 1 - scale3;
                   // GL.Color3(Color.Red);

                    //GL.Color3(_terrain.GetPixel(imgx, imgy));
                    GL.TexCoord2(imgx,imgy);
                    GL.Vertex3(x, heightl * heightscale, z); //  _terrain.GetPixel(x, z).R
            
                    try
                    {
                        heightl = srtm.getAltitude(z + increment, area.Right + area.Left - x, 20);

                        //scale2 = (Math.Abs(x - area.Left) / area.WidthLng) * (float)_terrain.Width;

                        scale3 = (Math.Abs(((z + increment) - area.Bottom)) / area.HeightLat);// / (float)_terrain.Height;

                        imgx = 1- scale2;
                        imgy = 1 - scale3;
                       // GL.Color3(Color.Green);
                       //GL.Color3(_terrain.GetPixel(imgx, imgy));
                        GL.TexCoord2(imgx,imgy);
                        GL.Vertex3(x, heightl * heightscale, z + increment);

                      //  Console.WriteLine(x + " " + (z + step));
                    }
                    catch { break; }
                    
                }
                GL.End();
            }

            GL.Enable(EnableCap.Blend);
            GL.DepthMask(false);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.One);
            GL.DepthMask(true);
            GL.Disable(EnableCap.Blend);

            GL.Flush();


            sw.Stop();

            Console.WriteLine("GL  "+sw.ElapsedMilliseconds);

            this.SwapBuffers();

            //this.Invalidate();
        }

        private void InitializeComponent()
        {
            this.SuspendLayout();
            // 
            // OpenGLtest
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.Name = "OpenGLtest";
            this.Load += new System.EventHandler(this.test_Load);
            this.Resize += new System.EventHandler(this.test_Resize);
            this.ResumeLayout(false);

        }

        private void test_Load(object sender, EventArgs e)
        {
            GL.Enable(EnableCap.DepthTest);
           // GL.Enable(EnableCap.Light0);
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.ColorMaterial);
            GL.Enable(EnableCap.Normalize);

            //GL.Enable(EnableCap.LineSmooth);
            //GL.Enable(EnableCap.PointSmooth);
            //GL.Enable(EnableCap.PolygonSmooth);
            GL.ShadeModel(ShadingModel.Smooth);
            GL.Enable(EnableCap.CullFace);
            GL.Enable(EnableCap.Texture2D);

        }

        private void test_Resize(object sender, EventArgs e)
        {
            MakeCurrent();

            GL.Viewport(0, 0, this.Width, this.Height);

            this.Invalidate();
        }
    }
}
