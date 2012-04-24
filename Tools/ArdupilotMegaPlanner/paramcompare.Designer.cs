namespace ArdupilotMega
{
    partial class ParamCompare
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ParamCompare));
            this.Params = new System.Windows.Forms.DataGridView();
            this.Command = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Value = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.newvalue = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Use = new System.Windows.Forms.DataGridViewCheckBoxColumn();
            this.BUT_save = new ArdupilotMega.Controls.MyButton();
            this.CHK_toggleall = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.Params)).BeginInit();
            this.SuspendLayout();
            // 
            // Params
            // 
            this.Params.AllowUserToAddRows = false;
            this.Params.AllowUserToDeleteRows = false;
            this.Params.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.Params.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.Command,
            this.Value,
            this.newvalue,
            this.Use});
            this.Params.Location = new System.Drawing.Point(12, 12);
            this.Params.Name = "Params";
            this.Params.RowHeadersVisible = false;
            this.Params.Size = new System.Drawing.Size(428, 467);
            this.Params.TabIndex = 0;
            // 
            // Command
            // 
            this.Command.HeaderText = "Command";
            this.Command.Name = "Command";
            this.Command.ReadOnly = true;
            // 
            // Value
            // 
            this.Value.HeaderText = "Value";
            this.Value.Name = "Value";
            this.Value.ReadOnly = true;
            // 
            // newvalue
            // 
            this.newvalue.HeaderText = "New Value";
            this.newvalue.Name = "newvalue";
            this.newvalue.ReadOnly = true;
            // 
            // Use
            // 
            this.Use.FillWeight = 30F;
            this.Use.HeaderText = "Use";
            this.Use.Name = "Use";
            this.Use.Width = 50;
            // 
            // BUT_save
            // 
            this.BUT_save.Location = new System.Drawing.Point(185, 485);
            this.BUT_save.Name = "BUT_save";
            this.BUT_save.Size = new System.Drawing.Size(75, 23);
            this.BUT_save.TabIndex = 1;
            this.BUT_save.Text = "Continue";
            this.BUT_save.UseVisualStyleBackColor = true;
            this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);
            // 
            // CHK_toggleall
            // 
            this.CHK_toggleall.AutoSize = true;
            this.CHK_toggleall.Checked = true;
            this.CHK_toggleall.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_toggleall.Location = new System.Drawing.Point(320, 489);
            this.CHK_toggleall.Name = "CHK_toggleall";
            this.CHK_toggleall.Size = new System.Drawing.Size(120, 17);
            this.CHK_toggleall.TabIndex = 2;
            this.CHK_toggleall.Text = "Check/Uncheck All";
            this.CHK_toggleall.UseVisualStyleBackColor = true;
            this.CHK_toggleall.CheckedChanged += new System.EventHandler(this.CHK_toggleall_CheckedChanged);
            // 
            // ParamCompare
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(457, 516);
            this.Controls.Add(this.CHK_toggleall);
            this.Controls.Add(this.BUT_save);
            this.Controls.Add(this.Params);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "ParamCompare";
            this.Text = "ParamCompare";
            ((System.ComponentModel.ISupportInitialize)(this.Params)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.DataGridView Params;
        private ArdupilotMega.Controls.MyButton BUT_save;
        private System.Windows.Forms.DataGridViewTextBoxColumn Command;
        private System.Windows.Forms.DataGridViewTextBoxColumn Value;
        private System.Windows.Forms.DataGridViewTextBoxColumn newvalue;
        private System.Windows.Forms.DataGridViewCheckBoxColumn Use;
        private System.Windows.Forms.CheckBox CHK_toggleall;
    }
}