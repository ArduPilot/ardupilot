namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigRawParams
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
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            this.BUT_compare = new ArdupilotMega.MyButton();
            this.BUT_rerequestparams = new ArdupilotMega.MyButton();
            this.BUT_writePIDS = new ArdupilotMega.MyButton();
            this.BUT_save = new ArdupilotMega.MyButton();
            this.BUT_load = new ArdupilotMega.MyButton();
            this.Params = new System.Windows.Forms.DataGridView();
            this.Command = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Value = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Default = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.mavScale = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.RawValue = new System.Windows.Forms.DataGridViewTextBoxColumn();
            ((System.ComponentModel.ISupportInitialize)(this.Params)).BeginInit();
            this.SuspendLayout();
            // 
            // BUT_compare
            // 
            this.BUT_compare.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_compare.Location = new System.Drawing.Point(341, 119);
            this.BUT_compare.Name = "BUT_compare";
            this.BUT_compare.Size = new System.Drawing.Size(103, 19);
            this.BUT_compare.TabIndex = 72;
            this.BUT_compare.Text = "Compare Params";
            this.BUT_compare.UseVisualStyleBackColor = true;
            this.BUT_compare.Click += new System.EventHandler(this.BUT_compare_Click);
            // 
            // BUT_rerequestparams
            // 
            this.BUT_rerequestparams.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_rerequestparams.Location = new System.Drawing.Point(341, 94);
            this.BUT_rerequestparams.Name = "BUT_rerequestparams";
            this.BUT_rerequestparams.Size = new System.Drawing.Size(103, 19);
            this.BUT_rerequestparams.TabIndex = 67;
            this.BUT_rerequestparams.Text = "Refresh Params";
            this.BUT_rerequestparams.UseVisualStyleBackColor = true;
            this.BUT_rerequestparams.Click += new System.EventHandler(this.BUT_rerequestparams_Click);
            // 
            // BUT_writePIDS
            // 
            this.BUT_writePIDS.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_writePIDS.Location = new System.Drawing.Point(341, 69);
            this.BUT_writePIDS.Name = "BUT_writePIDS";
            this.BUT_writePIDS.Size = new System.Drawing.Size(103, 19);
            this.BUT_writePIDS.TabIndex = 69;
            this.BUT_writePIDS.Text = "Write Params";
            this.BUT_writePIDS.UseVisualStyleBackColor = true;
            this.BUT_writePIDS.Click += new System.EventHandler(this.BUT_writePIDS_Click);
            // 
            // BUT_save
            // 
            this.BUT_save.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_save.Location = new System.Drawing.Point(341, 35);
            this.BUT_save.Margin = new System.Windows.Forms.Padding(0);
            this.BUT_save.Name = "BUT_save";
            this.BUT_save.Size = new System.Drawing.Size(104, 19);
            this.BUT_save.TabIndex = 70;
            this.BUT_save.Text = "Save";
            this.BUT_save.UseVisualStyleBackColor = true;
            this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);
            // 
            // BUT_load
            // 
            this.BUT_load.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_load.Location = new System.Drawing.Point(341, 7);
            this.BUT_load.Margin = new System.Windows.Forms.Padding(0);
            this.BUT_load.Name = "BUT_load";
            this.BUT_load.Size = new System.Drawing.Size(104, 19);
            this.BUT_load.TabIndex = 71;
            this.BUT_load.Text = "Load";
            this.BUT_load.UseVisualStyleBackColor = true;
            this.BUT_load.Click += new System.EventHandler(this.BUT_load_Click);
            // 
            // Params
            // 
            this.Params.AllowUserToAddRows = false;
            this.Params.AllowUserToDeleteRows = false;
            this.Params.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            dataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            dataGridViewCellStyle1.BackColor = System.Drawing.Color.Maroon;
            dataGridViewCellStyle1.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            dataGridViewCellStyle1.ForeColor = System.Drawing.Color.White;
            dataGridViewCellStyle1.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle1.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle1.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.Params.ColumnHeadersDefaultCellStyle = dataGridViewCellStyle1;
            this.Params.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.Params.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.Command,
            this.Value,
            this.Default,
            this.mavScale,
            this.RawValue});
            this.Params.Location = new System.Drawing.Point(14, 3);
            this.Params.Name = "Params";
            dataGridViewCellStyle2.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            dataGridViewCellStyle2.BackColor = System.Drawing.SystemColors.ActiveCaption;
            dataGridViewCellStyle2.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            dataGridViewCellStyle2.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle2.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle2.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle2.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.Params.RowHeadersDefaultCellStyle = dataGridViewCellStyle2;
            this.Params.RowHeadersVisible = false;
            this.Params.RowHeadersWidth = 150;
            this.Params.Size = new System.Drawing.Size(321, 302);
            this.Params.TabIndex = 68;
            // 
            // Command
            // 
            this.Command.HeaderText = "Command";
            this.Command.Name = "Command";
            this.Command.ReadOnly = true;
            this.Command.Width = 150;
            // 
            // Value
            // 
            this.Value.HeaderText = "Value";
            this.Value.Name = "Value";
            this.Value.Width = 80;
            // 
            // Default
            // 
            this.Default.HeaderText = "Default";
            this.Default.Name = "Default";
            this.Default.Visible = false;
            // 
            // mavScale
            // 
            this.mavScale.HeaderText = "mavScale";
            this.mavScale.Name = "mavScale";
            this.mavScale.Visible = false;
            // 
            // RawValue
            // 
            this.RawValue.HeaderText = "RawValue";
            this.RawValue.Name = "RawValue";
            this.RawValue.Visible = false;
            // 
            // ConfigRawParams
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_compare);
            this.Controls.Add(this.BUT_rerequestparams);
            this.Controls.Add(this.BUT_writePIDS);
            this.Controls.Add(this.BUT_save);
            this.Controls.Add(this.BUT_load);
            this.Controls.Add(this.Params);
            this.Name = "ConfigRawParams";
            this.Size = new System.Drawing.Size(460, 305);
            this.Load += new System.EventHandler(this.ConfigRawParams_Load);
            this.ControlAdded += new System.Windows.Forms.ControlEventHandler(this.ConfigRawParams_ControlAdded);
            this.ControlRemoved += new System.Windows.Forms.ControlEventHandler(this.ConfigRawParams_ControlRemoved);
            ((System.ComponentModel.ISupportInitialize)(this.Params)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private MyButton BUT_compare;
        private MyButton BUT_rerequestparams;
        private MyButton BUT_writePIDS;
        private MyButton BUT_save;
        private MyButton BUT_load;
        private System.Windows.Forms.DataGridView Params;
        private System.Windows.Forms.DataGridViewTextBoxColumn Command;
        private System.Windows.Forms.DataGridViewTextBoxColumn Value;
        private System.Windows.Forms.DataGridViewTextBoxColumn Default;
        private System.Windows.Forms.DataGridViewTextBoxColumn mavScale;
        private System.Windows.Forms.DataGridViewTextBoxColumn RawValue;
    }
}
