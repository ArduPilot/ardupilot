namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigParamParams
   {
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
         this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
         this.BUT_rerequestparams = new ArdupilotMega.Controls.MyButton();
         this.BUT_writePIDS = new ArdupilotMega.Controls.MyButton();
         this.SuspendLayout();
         // 
         // tableLayoutPanel1
         // 
         this.tableLayoutPanel1.AutoScroll = true;
         this.tableLayoutPanel1.ColumnCount = 1;
         this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
         this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 20F));
         this.tableLayoutPanel1.Location = new System.Drawing.Point(12, 36);
         this.tableLayoutPanel1.Name = "tableLayoutPanel1";
         this.tableLayoutPanel1.RowCount = 1;
         this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
         this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 110F));
         this.tableLayoutPanel1.Size = new System.Drawing.Size(514, 110);
         this.tableLayoutPanel1.TabIndex = 0;
         // 
         // BUT_rerequestparams
         // 
         this.BUT_rerequestparams.ImeMode = System.Windows.Forms.ImeMode.NoControl;
         this.BUT_rerequestparams.Location = new System.Drawing.Point(121, 11);
         this.BUT_rerequestparams.Name = "BUT_rerequestparams";
         this.BUT_rerequestparams.Padding = new System.Windows.Forms.Padding(0, 15, 0, 0);
         this.BUT_rerequestparams.Size = new System.Drawing.Size(103, 19);
         this.BUT_rerequestparams.TabIndex = 73;
         this.BUT_rerequestparams.Text = "Refresh Params";
         this.BUT_rerequestparams.UseVisualStyleBackColor = true;
         // 
         // BUT_writePIDS
         // 
         this.BUT_writePIDS.ImeMode = System.Windows.Forms.ImeMode.NoControl;
         this.BUT_writePIDS.Location = new System.Drawing.Point(12, 11);
         this.BUT_writePIDS.Name = "BUT_writePIDS";
         this.BUT_writePIDS.Padding = new System.Windows.Forms.Padding(0, 15, 0, 0);
         this.BUT_writePIDS.Size = new System.Drawing.Size(103, 19);
         this.BUT_writePIDS.TabIndex = 74;
         this.BUT_writePIDS.Text = "Write Params";
         this.BUT_writePIDS.UseVisualStyleBackColor = true;
         // 
         // ConfigFriendlyParams
         // 
         this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
         this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
         this.AutoSize = true;
         this.Controls.Add(this.BUT_rerequestparams);
         this.Controls.Add(this.BUT_writePIDS);
         this.Controls.Add(this.tableLayoutPanel1);
         this.Name = "ConfigFriendlyParams";
         this.Size = new System.Drawing.Size(673, 177);
         this.ResumeLayout(false);

      }

      #endregion

      private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
      private Controls.MyButton BUT_rerequestparams;
      private Controls.MyButton BUT_writePIDS;
   }
}
