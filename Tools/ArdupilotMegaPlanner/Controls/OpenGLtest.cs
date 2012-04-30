using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System.Drawing.Imaging;
using System.Drawing;

namespace ArdupilotMega.Controls
{
    public class OpenGLtest : GLControl
    {
        int fixme;
        Bitmap _terrain = new Bitmap(640,480);

        public OpenGLtest()
        {
            InitializeComponent();

            try
            {
                _terrain = new Bitmap(@"C:\Users\hog\Pictures\Denmark\[Group 1]-P1020169_P1020174-6 images.jpg");
            }
            catch {  }

            _terrain = new Bitmap(_terrain, 512, 512);


            GL.GenTextures(1, out texture);
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

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        int[,] getElevationData(double lat, double lng, double direction)
        {
            int[,] answer = new int[400,400];

            double step = 0.00083333333333333;

            for (int y = 0; y < answer.GetLength(0) - 1; y++)
            {
                Console.WriteLine(y);
                for (int x = 0; x < answer.GetLength(1) - 1; x++)
                {
                    double mlat = lat + step * (float)y + Math.Sin(direction * deg2rad) * step * (float)y;
                    double mlng = lng + step * (float)x + Math.Cos(direction * deg2rad) * step * (float)x;

                  //  Console.WriteLine(mlat + " "+mlng);

                    int alt = srtm.getAltitude(mlat, mlng, 20);
                    answer[x,y] = alt;
                }
            }

            return answer;
        }

        int texture = 0;
        private System.ComponentModel.IContainer components;

        public Vector3 Normal(Vector3 a, Vector3 b, Vector3 c)
        {
                var dir = Vector3.Cross(b - a, c - a);
                var norm = Vector3.Normalize(dir);
                return norm;
        }

        float _angle = 0;
        double cameraX, cameraY, cameraZ;       // camera coordinates
        double lookX, lookY, lookZ;             // camera look-at coordinates


        protected override void OnPaint(System.Windows.Forms.PaintEventArgs e)
        {
            if (this.DesignMode)
                return;

            _angle+=1;

            try
            {
                base.OnPaint(e);

            }
            catch { return;  }

            float scale = 1.0f;

             float radians =  (float)(Math.PI*(_angle-90.0f)/180.0f);

            int mouseY = (int)(900 * scale);
  
      // calculate the camera's position
      cameraX = lookX + Math.Sin(radians)*mouseY;     // multiplying by mouseY makes the
      cameraZ = lookZ + Math.Cos(radians)*mouseY;    // camera get closer/farther away with mouseY
      cameraY = lookY + mouseY / 2.0f;
  
      // calculate the camera look-at coordinates as the center of the terrain map
      lookX = (_terrain.Width * scale) / 2.0f;
      lookY = 0 * scale;
      lookZ = (_terrain.Height * scale) / 2.0f;



            MakeCurrent();

            GL.ClearColor(Color.Green);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            GL.LoadIdentity();

            OpenTK.Graphics.Glu.LookAt(cameraX, cameraY, cameraZ, lookX, lookY, lookZ, 0.0, 1.0, 0.0);

            GL.Enable(EnableCap.Texture2D);
            GL.BindTexture(TextureTarget.Texture2D, texture);         
            
            double mlat = -34.73306;
            double mlng = 117.8864897;
            double step = 0.00083333333333333;

            int increment =50;

            for (int z = 0; z < _terrain.Height - 1; z += increment)
            {
                //Makes OpenGL draw a triangle at every three consecutive vertices
                GL.Begin(BeginMode.TriangleStrip);
                for (int x = 0; x < _terrain.Width - 1; x += increment)
                {

                   // Console.WriteLine(mlat + step * z +" "+ mlng + step * x);

                    int heightl = srtm.getAltitude(mlat + step * z, mlng + step * x, 20);

                    //GL.Color3(_terrain.GetPixel(x, z));
                    GL.TexCoord2((x / (float)_terrain.Width),  (z / (float)_terrain.Height));
                    GL.Vertex3(x * scale, heightl, z * scale); //  _terrain.GetPixel(x, z).R
            
                    try
                    {
                        heightl = srtm.getAltitude(mlat + step * (z + increment), mlng + step * (x), 20);

                        //GL.Color3(_terrain.GetPixel(x, z + increment));
                        GL.TexCoord2((x / (float)_terrain.Width),  ((z + increment) / (float)_terrain.Height));
                        GL.Vertex3(x * scale, heightl, z + increment * scale);
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
             

            this.SwapBuffers();

          //  this.Invalidate();
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
          //  GL.Enable(EnableCap.Lighting);
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
            GL.Viewport(0, 0, this.Width, this.Height);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            if (Height == 0)
                Height = 1;

            OpenTK.Graphics.Glu.Perspective(54.0f, this.Width / this.Height, 1.0f, 5000.0f);

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();
        }
    }
}
