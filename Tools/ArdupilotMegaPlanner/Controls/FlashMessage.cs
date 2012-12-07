using System;
using System.ComponentModel;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Collections;

namespace ArdupilotMega.Controls
{
    public partial class FlashMessage : Label
    {
        private const int fadeInterval = 20;    //time in ms between "frames" of animation
        private const int fadeDuration = 200;   //overall animation duration in ms in one direction
        private const int desiredHeight = 20;   //desired height to animate to
        private const int inOutDelay = 800 / fadeInterval; //number in ms between in and out
        private const float fadeStepValue = desiredHeight * (float)fadeInterval / (float)fadeDuration;
        private int inOutDelayCounter;
        private int direction;
        private int heightVisible;
        private bool disposeOnComplete;
        private Timer fadeTimer;
        private bool inOut;
        private Queue messages = new Queue();

        public FlashMessage()
        {
            Visible = false;

            Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left)
            | System.Windows.Forms.AnchorStyles.Right)));
            Font = new System.Drawing.Font("Microsoft Sans Serif", 9.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(238)));
            Location = new Point(0, 0);
            TextAlign = System.Drawing.ContentAlignment.MiddleCenter;

            fadeTimer = new Timer();
            fadeTimer.Interval = fadeInterval;
            fadeTimer.Tick += new EventHandler(fadeTimer_Tick);
        }

        public void FadeInOut(string msg, bool success)
        {
            if (Visible)
            {
                messages.Enqueue(new { msg = msg, success = success });
            }
            else
            {
                if (this.Parent != null) Width = this.Parent.ClientSize.Width;
                BringToFront();
                direction = 1;
                inOut = true;
                inOutDelayCounter = inOutDelay;
                heightVisible = Height = 0;
                Text = msg;
                BackColor = success ? Color.YellowGreen : Color.Coral;
            }
            Visible = true;
            fadeTimer.Enabled = true;
        }

        //public void FadeIn(string msg, bool success)
        //{
        //    if (this.Parent != null) Width = this.Parent.ClientSize.Width;
        //    BringToFront();
        //    direction = 1;
        //    inOut = false;
        //    heightVisible = Height = 0;
        //    if (Visible)
        //    {
        //        messages.Enqueue(msg);
        //    }
        //    else
        //    {
        //        Text = msg;
        //    }
        //    BackColor = success ? Color.YellowGreen : Color.Coral;
        //    Visible = true;
        //    fadeTimer.Enabled = true;
        //}

        private void FadeOut(bool disposeOnComplete = false)
        {
            if (this.Parent != null) Width = this.Parent.ClientSize.Width;
            BringToFront();
            direction = -1;
            inOut = false;
            heightVisible = Height = desiredHeight;
            this.disposeOnComplete = disposeOnComplete;
            fadeTimer.Enabled = true;
        }

        private void fadeTimer_Tick(object sender, EventArgs e)
        {
            bool done = false;
            heightVisible += (int)Math.Round(direction * fadeStepValue);
            if (heightVisible < 0) { done = true; heightVisible = 0; }
            if (heightVisible > desiredHeight) { done = true; heightVisible = desiredHeight; }
            Height = heightVisible;
            if (done)
            {
                if (inOut)
                {
                    if (--inOutDelayCounter == 0)
                    {
                        FadeOut(disposeOnComplete);
                    }
                }
                else
                {
                    if (messages.Count > 0)
                    {
                        Object o = messages.Dequeue();
                        Text = (string)o.GetType().GetProperty("msg").GetValue(o, null);
                        bool success = (bool)o.GetType().GetProperty("success").GetValue(o, null);
                        BackColor = success ? Color.YellowGreen : Color.Coral;

                        direction = 1;
                        inOut = true;
                        inOutDelayCounter = inOutDelay;
                        heightVisible = Height = 0;

                        //Text = (messages.Dequeue()).msg;
                    }
                    else
                    {
                        fadeTimer.Enabled = false;
                        Visible = false;
                        if (direction == -1)    //hide after fadeOut
                        {
                            if (disposeOnComplete) this.Dispose();
                        }
                    }
                }
            }
            else
            {
                //this.Height = heightVisible;
                //this.Invalidate();
            }
        }
    }
}
