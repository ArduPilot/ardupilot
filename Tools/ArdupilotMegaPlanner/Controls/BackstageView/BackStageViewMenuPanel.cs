using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArdupilotMega.Controls.BackstageView
{
    public class BackStageViewMenuPanel : Panel
    {
        internal Color GradColor = Color.White;
        internal Color PencilBorderColor = Color.White;

        private const int GradientWidth = 20;

        public BackStageViewMenuPanel()
        {
            this.SetStyle(ControlStyles.UserPaint, true);
        }

        protected override void OnPaintBackground(PaintEventArgs pevent)
        {
            base.OnPaintBackground(pevent);

            var rc = new Rectangle(ClientSize.Width - GradientWidth, 0, GradientWidth, this.ClientSize.Height);

            using (var brush = new LinearGradientBrush(rc, BackColor, GradColor, LinearGradientMode.Horizontal))
            {
                pevent.Graphics.FillRectangle(brush, rc);
            }

            pevent.Graphics.DrawLine(new Pen(PencilBorderColor), Width-1,0,Width-1,Height);
        }

        protected override void OnResize(System.EventArgs eventargs)
        {
            base.OnResize(eventargs);
            this.Invalidate();
        }

        public void PaintBackground(PaintEventArgs pevent)
        {
            OnPaintBackground(pevent);
        }
    }
}