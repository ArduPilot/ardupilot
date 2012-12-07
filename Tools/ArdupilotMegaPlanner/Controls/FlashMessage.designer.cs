using System.Windows.Forms;
using System.Drawing;
namespace ArdupilotMega.Controls
{
    partial class FlashMessage
    {
        public AutoScaleMode AutoScaleMode = AutoScaleMode.Font;

        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.SuspendLayout();
            // 
            // FlashMessage
            // 
            this.BackColor = System.Drawing.Color.Coral;
            this.Name = "FM_info";
            this.ResumeLayout(false);
        }

        #endregion
    }
}
