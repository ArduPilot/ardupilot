namespace ArdupilotMega.GCSViews
{
    partial class Configuration
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
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Configuration));
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            this.Params = new System.Windows.Forms.DataGridView();
            this.Command = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Value = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Default = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.mavScale = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.RawValue = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.ConfigTabs = new System.Windows.Forms.TabControl();
            this.TabAPM2 = new System.Windows.Forms.TabPage();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.THR_FS_VALUE = new System.Windows.Forms.NumericUpDown();
            this.label5 = new System.Windows.Forms.Label();
            this.THR_MAX = new System.Windows.Forms.NumericUpDown();
            this.label6 = new System.Windows.Forms.Label();
            this.THR_MIN = new System.Windows.Forms.NumericUpDown();
            this.label7 = new System.Windows.Forms.Label();
            this.TRIM_THROTTLE = new System.Windows.Forms.NumericUpDown();
            this.label8 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.ARSPD_RATIO = new System.Windows.Forms.NumericUpDown();
            this.label1 = new System.Windows.Forms.Label();
            this.ARSPD_FBW_MAX = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.ARSPD_FBW_MIN = new System.Windows.Forms.NumericUpDown();
            this.label3 = new System.Windows.Forms.Label();
            this.TRIM_ARSPD_CM = new System.Windows.Forms.NumericUpDown();
            this.label4 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.LIM_PITCH_MIN = new System.Windows.Forms.NumericUpDown();
            this.label39 = new System.Windows.Forms.Label();
            this.LIM_PITCH_MAX = new System.Windows.Forms.NumericUpDown();
            this.label38 = new System.Windows.Forms.Label();
            this.LIM_ROLL_CD = new System.Windows.Forms.NumericUpDown();
            this.label37 = new System.Windows.Forms.Label();
            this.groupBox15 = new System.Windows.Forms.GroupBox();
            this.XTRK_ANGLE_CD = new System.Windows.Forms.NumericUpDown();
            this.label79 = new System.Windows.Forms.Label();
            this.XTRK_GAIN_SC = new System.Windows.Forms.NumericUpDown();
            this.label80 = new System.Windows.Forms.Label();
            this.groupBox16 = new System.Windows.Forms.GroupBox();
            this.KFF_PTCH2THR = new System.Windows.Forms.NumericUpDown();
            this.label83 = new System.Windows.Forms.Label();
            this.KFF_RDDRMIX = new System.Windows.Forms.NumericUpDown();
            this.label78 = new System.Windows.Forms.Label();
            this.KFF_PTCHCOMP = new System.Windows.Forms.NumericUpDown();
            this.label81 = new System.Windows.Forms.Label();
            this.groupBox14 = new System.Windows.Forms.GroupBox();
            this.ENRGY2THR_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label73 = new System.Windows.Forms.Label();
            this.ENRGY2THR_D = new System.Windows.Forms.NumericUpDown();
            this.label74 = new System.Windows.Forms.Label();
            this.ENRGY2THR_I = new System.Windows.Forms.NumericUpDown();
            this.label75 = new System.Windows.Forms.Label();
            this.ENRGY2THR_P = new System.Windows.Forms.NumericUpDown();
            this.label76 = new System.Windows.Forms.Label();
            this.groupBox13 = new System.Windows.Forms.GroupBox();
            this.ALT2PTCH_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label69 = new System.Windows.Forms.Label();
            this.ALT2PTCH_D = new System.Windows.Forms.NumericUpDown();
            this.label70 = new System.Windows.Forms.Label();
            this.ALT2PTCH_I = new System.Windows.Forms.NumericUpDown();
            this.label71 = new System.Windows.Forms.Label();
            this.ALT2PTCH_P = new System.Windows.Forms.NumericUpDown();
            this.label72 = new System.Windows.Forms.Label();
            this.groupBox12 = new System.Windows.Forms.GroupBox();
            this.ARSP2PTCH_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label65 = new System.Windows.Forms.Label();
            this.ARSP2PTCH_D = new System.Windows.Forms.NumericUpDown();
            this.label66 = new System.Windows.Forms.Label();
            this.ARSP2PTCH_I = new System.Windows.Forms.NumericUpDown();
            this.label67 = new System.Windows.Forms.Label();
            this.ARSP2PTCH_P = new System.Windows.Forms.NumericUpDown();
            this.label68 = new System.Windows.Forms.Label();
            this.groupBox11 = new System.Windows.Forms.GroupBox();
            this.HDNG2RLL_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label61 = new System.Windows.Forms.Label();
            this.HDNG2RLL_D = new System.Windows.Forms.NumericUpDown();
            this.label62 = new System.Windows.Forms.Label();
            this.HDNG2RLL_I = new System.Windows.Forms.NumericUpDown();
            this.label63 = new System.Windows.Forms.Label();
            this.HDNG2RLL_P = new System.Windows.Forms.NumericUpDown();
            this.label64 = new System.Windows.Forms.Label();
            this.groupBox10 = new System.Windows.Forms.GroupBox();
            this.YW2SRV_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label57 = new System.Windows.Forms.Label();
            this.YW2SRV_D = new System.Windows.Forms.NumericUpDown();
            this.label58 = new System.Windows.Forms.Label();
            this.YW2SRV_I = new System.Windows.Forms.NumericUpDown();
            this.label59 = new System.Windows.Forms.Label();
            this.YW2SRV_P = new System.Windows.Forms.NumericUpDown();
            this.label60 = new System.Windows.Forms.Label();
            this.groupBox9 = new System.Windows.Forms.GroupBox();
            this.PTCH2SRV_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label53 = new System.Windows.Forms.Label();
            this.PTCH2SRV_D = new System.Windows.Forms.NumericUpDown();
            this.label54 = new System.Windows.Forms.Label();
            this.PTCH2SRV_I = new System.Windows.Forms.NumericUpDown();
            this.label55 = new System.Windows.Forms.Label();
            this.PTCH2SRV_P = new System.Windows.Forms.NumericUpDown();
            this.label56 = new System.Windows.Forms.Label();
            this.groupBox8 = new System.Windows.Forms.GroupBox();
            this.RLL2SRV_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label49 = new System.Windows.Forms.Label();
            this.RLL2SRV_D = new System.Windows.Forms.NumericUpDown();
            this.label50 = new System.Windows.Forms.Label();
            this.RLL2SRV_I = new System.Windows.Forms.NumericUpDown();
            this.label51 = new System.Windows.Forms.Label();
            this.RLL2SRV_P = new System.Windows.Forms.NumericUpDown();
            this.label52 = new System.Windows.Forms.Label();
            this.TabAC2 = new System.Windows.Forms.TabPage();
            this.myLabel1 = new ArdupilotMega.MyLabel();
            this.CH7_OPT = new System.Windows.Forms.ComboBox();
            this.groupBox17 = new System.Windows.Forms.GroupBox();
            this.ACRO_PIT_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label27 = new System.Windows.Forms.Label();
            this.ACRO_PIT_I = new System.Windows.Forms.NumericUpDown();
            this.label29 = new System.Windows.Forms.Label();
            this.ACRO_PIT_P = new System.Windows.Forms.NumericUpDown();
            this.label33 = new System.Windows.Forms.Label();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.label14 = new System.Windows.Forms.Label();
            this.THR_RATE_IMAX = new System.Windows.Forms.NumericUpDown();
            this.THR_RATE_I = new System.Windows.Forms.NumericUpDown();
            this.label20 = new System.Windows.Forms.Label();
            this.THR_RATE_P = new System.Windows.Forms.NumericUpDown();
            this.label25 = new System.Windows.Forms.Label();
            this.groupBox18 = new System.Windows.Forms.GroupBox();
            this.ACRO_RLL_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label40 = new System.Windows.Forms.Label();
            this.ACRO_RLL_I = new System.Windows.Forms.NumericUpDown();
            this.label44 = new System.Windows.Forms.Label();
            this.ACRO_RLL_P = new System.Windows.Forms.NumericUpDown();
            this.label48 = new System.Windows.Forms.Label();
            this.CHK_lockrollpitch = new System.Windows.Forms.CheckBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.WP_SPEED_MAX = new System.Windows.Forms.NumericUpDown();
            this.label9 = new System.Windows.Forms.Label();
            this.NAV_LAT_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label13 = new System.Windows.Forms.Label();
            this.NAV_LAT_I = new System.Windows.Forms.NumericUpDown();
            this.label15 = new System.Windows.Forms.Label();
            this.NAV_LAT_P = new System.Windows.Forms.NumericUpDown();
            this.label16 = new System.Windows.Forms.Label();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.XTRK_GAIN_SC1 = new System.Windows.Forms.NumericUpDown();
            this.label18 = new System.Windows.Forms.Label();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.THR_ALT_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label19 = new System.Windows.Forms.Label();
            this.THR_ALT_I = new System.Windows.Forms.NumericUpDown();
            this.label21 = new System.Windows.Forms.Label();
            this.THR_ALT_P = new System.Windows.Forms.NumericUpDown();
            this.label22 = new System.Windows.Forms.Label();
            this.groupBox19 = new System.Windows.Forms.GroupBox();
            this.HLD_LAT_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label28 = new System.Windows.Forms.Label();
            this.HLD_LAT_I = new System.Windows.Forms.NumericUpDown();
            this.label30 = new System.Windows.Forms.Label();
            this.HLD_LAT_P = new System.Windows.Forms.NumericUpDown();
            this.label31 = new System.Windows.Forms.Label();
            this.groupBox20 = new System.Windows.Forms.GroupBox();
            this.STB_YAW_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label32 = new System.Windows.Forms.Label();
            this.STB_YAW_I = new System.Windows.Forms.NumericUpDown();
            this.label34 = new System.Windows.Forms.Label();
            this.STB_YAW_P = new System.Windows.Forms.NumericUpDown();
            this.label35 = new System.Windows.Forms.Label();
            this.groupBox21 = new System.Windows.Forms.GroupBox();
            this.STB_PIT_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label36 = new System.Windows.Forms.Label();
            this.STB_PIT_I = new System.Windows.Forms.NumericUpDown();
            this.label41 = new System.Windows.Forms.Label();
            this.STB_PIT_P = new System.Windows.Forms.NumericUpDown();
            this.label42 = new System.Windows.Forms.Label();
            this.groupBox22 = new System.Windows.Forms.GroupBox();
            this.STB_RLL_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label43 = new System.Windows.Forms.Label();
            this.STB_RLL_I = new System.Windows.Forms.NumericUpDown();
            this.label45 = new System.Windows.Forms.Label();
            this.STB_RLL_P = new System.Windows.Forms.NumericUpDown();
            this.label46 = new System.Windows.Forms.Label();
            this.groupBox23 = new System.Windows.Forms.GroupBox();
            this.RATE_YAW_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label47 = new System.Windows.Forms.Label();
            this.RATE_YAW_I = new System.Windows.Forms.NumericUpDown();
            this.label77 = new System.Windows.Forms.Label();
            this.RATE_YAW_P = new System.Windows.Forms.NumericUpDown();
            this.label82 = new System.Windows.Forms.Label();
            this.groupBox24 = new System.Windows.Forms.GroupBox();
            this.RATE_PIT_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label84 = new System.Windows.Forms.Label();
            this.RATE_PIT_I = new System.Windows.Forms.NumericUpDown();
            this.label86 = new System.Windows.Forms.Label();
            this.RATE_PIT_P = new System.Windows.Forms.NumericUpDown();
            this.label87 = new System.Windows.Forms.Label();
            this.groupBox25 = new System.Windows.Forms.GroupBox();
            this.RATE_RLL_IMAX = new System.Windows.Forms.NumericUpDown();
            this.label88 = new System.Windows.Forms.Label();
            this.RATE_RLL_I = new System.Windows.Forms.NumericUpDown();
            this.label90 = new System.Windows.Forms.Label();
            this.RATE_RLL_P = new System.Windows.Forms.NumericUpDown();
            this.label91 = new System.Windows.Forms.Label();
            this.TabPlanner = new System.Windows.Forms.TabPage();
            this.label26 = new System.Windows.Forms.Label();
            this.CMB_videoresolutions = new System.Windows.Forms.ComboBox();
            this.label12 = new System.Windows.Forms.Label();
            this.CHK_GDIPlus = new System.Windows.Forms.CheckBox();
            this.label24 = new System.Windows.Forms.Label();
            this.CHK_loadwponconnect = new System.Windows.Forms.CheckBox();
            this.label23 = new System.Windows.Forms.Label();
            this.NUM_tracklength = new System.Windows.Forms.NumericUpDown();
            this.CHK_speechaltwarning = new System.Windows.Forms.CheckBox();
            this.label108 = new System.Windows.Forms.Label();
            this.CHK_resetapmonconnect = new System.Windows.Forms.CheckBox();
            this.CHK_mavdebug = new System.Windows.Forms.CheckBox();
            this.label107 = new System.Windows.Forms.Label();
            this.CMB_raterc = new System.Windows.Forms.ComboBox();
            this.label104 = new System.Windows.Forms.Label();
            this.label103 = new System.Windows.Forms.Label();
            this.label102 = new System.Windows.Forms.Label();
            this.label101 = new System.Windows.Forms.Label();
            this.CMB_ratestatus = new System.Windows.Forms.ComboBox();
            this.CMB_rateposition = new System.Windows.Forms.ComboBox();
            this.CMB_rateattitude = new System.Windows.Forms.ComboBox();
            this.label99 = new System.Windows.Forms.Label();
            this.label98 = new System.Windows.Forms.Label();
            this.label97 = new System.Windows.Forms.Label();
            this.CMB_speedunits = new System.Windows.Forms.ComboBox();
            this.CMB_distunits = new System.Windows.Forms.ComboBox();
            this.label96 = new System.Windows.Forms.Label();
            this.label95 = new System.Windows.Forms.Label();
            this.CHK_speechbattery = new System.Windows.Forms.CheckBox();
            this.CHK_speechcustom = new System.Windows.Forms.CheckBox();
            this.CHK_speechmode = new System.Windows.Forms.CheckBox();
            this.CHK_speechwaypoint = new System.Windows.Forms.CheckBox();
            this.label94 = new System.Windows.Forms.Label();
            this.CMB_osdcolor = new System.Windows.Forms.ComboBox();
            this.CMB_language = new System.Windows.Forms.ComboBox();
            this.label93 = new System.Windows.Forms.Label();
            this.CHK_enablespeech = new System.Windows.Forms.CheckBox();
            this.CHK_hudshow = new System.Windows.Forms.CheckBox();
            this.label92 = new System.Windows.Forms.Label();
            this.CMB_videosources = new System.Windows.Forms.ComboBox();
            this.BUT_Joystick = new ArdupilotMega.MyButton();
            this.BUT_videostop = new ArdupilotMega.MyButton();
            this.BUT_videostart = new ArdupilotMega.MyButton();
            this.TabSetup = new System.Windows.Forms.TabPage();
            this.label109 = new System.Windows.Forms.Label();
            this.BUT_rerequestparams = new ArdupilotMega.MyButton();
            this.BUT_writePIDS = new ArdupilotMega.MyButton();
            this.BUT_save = new ArdupilotMega.MyButton();
            this.BUT_load = new ArdupilotMega.MyButton();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.BUT_compare = new ArdupilotMega.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.Params)).BeginInit();
            this.ConfigTabs.SuspendLayout();
            this.TabAPM2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.THR_FS_VALUE)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_MAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_MIN)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRIM_THROTTLE)).BeginInit();
            this.groupBox1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ARSPD_RATIO)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSPD_FBW_MAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSPD_FBW_MIN)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRIM_ARSPD_CM)).BeginInit();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_PITCH_MIN)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_PITCH_MAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_ROLL_CD)).BeginInit();
            this.groupBox15.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.XTRK_ANGLE_CD)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.XTRK_GAIN_SC)).BeginInit();
            this.groupBox16.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.KFF_PTCH2THR)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.KFF_RDDRMIX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.KFF_PTCHCOMP)).BeginInit();
            this.groupBox14.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_P)).BeginInit();
            this.groupBox13.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_P)).BeginInit();
            this.groupBox12.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_P)).BeginInit();
            this.groupBox11.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_P)).BeginInit();
            this.groupBox10.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_P)).BeginInit();
            this.groupBox9.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_P)).BeginInit();
            this.groupBox8.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_D)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_P)).BeginInit();
            this.TabAC2.SuspendLayout();
            this.groupBox17.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_PIT_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_PIT_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_PIT_P)).BeginInit();
            this.groupBox5.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.THR_RATE_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_RATE_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_RATE_P)).BeginInit();
            this.groupBox18.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_RLL_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_RLL_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_RLL_P)).BeginInit();
            this.groupBox4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.WP_SPEED_MAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NAV_LAT_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NAV_LAT_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NAV_LAT_P)).BeginInit();
            this.groupBox6.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.XTRK_GAIN_SC1)).BeginInit();
            this.groupBox7.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.THR_ALT_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_ALT_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_ALT_P)).BeginInit();
            this.groupBox19.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.HLD_LAT_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.HLD_LAT_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.HLD_LAT_P)).BeginInit();
            this.groupBox20.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.STB_YAW_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_YAW_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_YAW_P)).BeginInit();
            this.groupBox21.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.STB_PIT_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_PIT_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_PIT_P)).BeginInit();
            this.groupBox22.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.STB_RLL_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_RLL_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_RLL_P)).BeginInit();
            this.groupBox23.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_YAW_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_YAW_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_YAW_P)).BeginInit();
            this.groupBox24.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_PIT_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_PIT_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_PIT_P)).BeginInit();
            this.groupBox25.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_RLL_IMAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_RLL_I)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_RLL_P)).BeginInit();
            this.TabPlanner.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUM_tracklength)).BeginInit();
            this.SuspendLayout();
            // 
            // Params
            // 
            this.Params.AllowUserToAddRows = false;
            this.Params.AllowUserToDeleteRows = false;
            resources.ApplyResources(this.Params, "Params");
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
            this.Params.CellValueChanged += new System.Windows.Forms.DataGridViewCellEventHandler(this.Params_CellValueChanged);
            // 
            // Command
            // 
            resources.ApplyResources(this.Command, "Command");
            this.Command.Name = "Command";
            this.Command.ReadOnly = true;
            // 
            // Value
            // 
            resources.ApplyResources(this.Value, "Value");
            this.Value.Name = "Value";
            // 
            // Default
            // 
            resources.ApplyResources(this.Default, "Default");
            this.Default.Name = "Default";
            // 
            // mavScale
            // 
            resources.ApplyResources(this.mavScale, "mavScale");
            this.mavScale.Name = "mavScale";
            // 
            // RawValue
            // 
            resources.ApplyResources(this.RawValue, "RawValue");
            this.RawValue.Name = "RawValue";
            // 
            // ConfigTabs
            // 
            resources.ApplyResources(this.ConfigTabs, "ConfigTabs");
            this.ConfigTabs.Controls.Add(this.TabAPM2);
            this.ConfigTabs.Controls.Add(this.TabAC2);
            this.ConfigTabs.Controls.Add(this.TabPlanner);
            this.ConfigTabs.Controls.Add(this.TabSetup);
            this.ConfigTabs.Name = "ConfigTabs";
            this.ConfigTabs.SelectedIndex = 0;
            this.ConfigTabs.SelectedIndexChanged += new System.EventHandler(this.Planner_TabIndexChanged);
            // 
            // TabAPM2
            // 
            this.TabAPM2.BackColor = System.Drawing.Color.Transparent;
            this.TabAPM2.Controls.Add(this.groupBox3);
            this.TabAPM2.Controls.Add(this.groupBox1);
            this.TabAPM2.Controls.Add(this.groupBox2);
            this.TabAPM2.Controls.Add(this.groupBox15);
            this.TabAPM2.Controls.Add(this.groupBox16);
            this.TabAPM2.Controls.Add(this.groupBox14);
            this.TabAPM2.Controls.Add(this.groupBox13);
            this.TabAPM2.Controls.Add(this.groupBox12);
            this.TabAPM2.Controls.Add(this.groupBox11);
            this.TabAPM2.Controls.Add(this.groupBox10);
            this.TabAPM2.Controls.Add(this.groupBox9);
            this.TabAPM2.Controls.Add(this.groupBox8);
            resources.ApplyResources(this.TabAPM2, "TabAPM2");
            this.TabAPM2.Name = "TabAPM2";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.THR_FS_VALUE);
            this.groupBox3.Controls.Add(this.label5);
            this.groupBox3.Controls.Add(this.THR_MAX);
            this.groupBox3.Controls.Add(this.label6);
            this.groupBox3.Controls.Add(this.THR_MIN);
            this.groupBox3.Controls.Add(this.label7);
            this.groupBox3.Controls.Add(this.TRIM_THROTTLE);
            this.groupBox3.Controls.Add(this.label8);
            resources.ApplyResources(this.groupBox3, "groupBox3");
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.TabStop = false;
            // 
            // THR_FS_VALUE
            // 
            resources.ApplyResources(this.THR_FS_VALUE, "THR_FS_VALUE");
            this.THR_FS_VALUE.Name = "THR_FS_VALUE";
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // THR_MAX
            // 
            resources.ApplyResources(this.THR_MAX, "THR_MAX");
            this.THR_MAX.Name = "THR_MAX";
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            // 
            // THR_MIN
            // 
            resources.ApplyResources(this.THR_MIN, "THR_MIN");
            this.THR_MIN.Name = "THR_MIN";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            // 
            // TRIM_THROTTLE
            // 
            resources.ApplyResources(this.TRIM_THROTTLE, "TRIM_THROTTLE");
            this.TRIM_THROTTLE.Name = "TRIM_THROTTLE";
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.Name = "label8";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.ARSPD_RATIO);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.ARSPD_FBW_MAX);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.ARSPD_FBW_MIN);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.TRIM_ARSPD_CM);
            this.groupBox1.Controls.Add(this.label4);
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // ARSPD_RATIO
            // 
            resources.ApplyResources(this.ARSPD_RATIO, "ARSPD_RATIO");
            this.ARSPD_RATIO.Name = "ARSPD_RATIO";
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // ARSPD_FBW_MAX
            // 
            resources.ApplyResources(this.ARSPD_FBW_MAX, "ARSPD_FBW_MAX");
            this.ARSPD_FBW_MAX.Name = "ARSPD_FBW_MAX";
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // ARSPD_FBW_MIN
            // 
            resources.ApplyResources(this.ARSPD_FBW_MIN, "ARSPD_FBW_MIN");
            this.ARSPD_FBW_MIN.Name = "ARSPD_FBW_MIN";
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // TRIM_ARSPD_CM
            // 
            resources.ApplyResources(this.TRIM_ARSPD_CM, "TRIM_ARSPD_CM");
            this.TRIM_ARSPD_CM.Name = "TRIM_ARSPD_CM";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.LIM_PITCH_MIN);
            this.groupBox2.Controls.Add(this.label39);
            this.groupBox2.Controls.Add(this.LIM_PITCH_MAX);
            this.groupBox2.Controls.Add(this.label38);
            this.groupBox2.Controls.Add(this.LIM_ROLL_CD);
            this.groupBox2.Controls.Add(this.label37);
            resources.ApplyResources(this.groupBox2, "groupBox2");
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.TabStop = false;
            // 
            // LIM_PITCH_MIN
            // 
            resources.ApplyResources(this.LIM_PITCH_MIN, "LIM_PITCH_MIN");
            this.LIM_PITCH_MIN.Name = "LIM_PITCH_MIN";
            // 
            // label39
            // 
            resources.ApplyResources(this.label39, "label39");
            this.label39.Name = "label39";
            // 
            // LIM_PITCH_MAX
            // 
            resources.ApplyResources(this.LIM_PITCH_MAX, "LIM_PITCH_MAX");
            this.LIM_PITCH_MAX.Name = "LIM_PITCH_MAX";
            // 
            // label38
            // 
            resources.ApplyResources(this.label38, "label38");
            this.label38.Name = "label38";
            // 
            // LIM_ROLL_CD
            // 
            resources.ApplyResources(this.LIM_ROLL_CD, "LIM_ROLL_CD");
            this.LIM_ROLL_CD.Name = "LIM_ROLL_CD";
            // 
            // label37
            // 
            resources.ApplyResources(this.label37, "label37");
            this.label37.Name = "label37";
            // 
            // groupBox15
            // 
            this.groupBox15.Controls.Add(this.XTRK_ANGLE_CD);
            this.groupBox15.Controls.Add(this.label79);
            this.groupBox15.Controls.Add(this.XTRK_GAIN_SC);
            this.groupBox15.Controls.Add(this.label80);
            resources.ApplyResources(this.groupBox15, "groupBox15");
            this.groupBox15.Name = "groupBox15";
            this.groupBox15.TabStop = false;
            // 
            // XTRK_ANGLE_CD
            // 
            resources.ApplyResources(this.XTRK_ANGLE_CD, "XTRK_ANGLE_CD");
            this.XTRK_ANGLE_CD.Name = "XTRK_ANGLE_CD";
            // 
            // label79
            // 
            resources.ApplyResources(this.label79, "label79");
            this.label79.Name = "label79";
            // 
            // XTRK_GAIN_SC
            // 
            resources.ApplyResources(this.XTRK_GAIN_SC, "XTRK_GAIN_SC");
            this.XTRK_GAIN_SC.Name = "XTRK_GAIN_SC";
            // 
            // label80
            // 
            resources.ApplyResources(this.label80, "label80");
            this.label80.Name = "label80";
            // 
            // groupBox16
            // 
            this.groupBox16.Controls.Add(this.KFF_PTCH2THR);
            this.groupBox16.Controls.Add(this.label83);
            this.groupBox16.Controls.Add(this.KFF_RDDRMIX);
            this.groupBox16.Controls.Add(this.label78);
            this.groupBox16.Controls.Add(this.KFF_PTCHCOMP);
            this.groupBox16.Controls.Add(this.label81);
            resources.ApplyResources(this.groupBox16, "groupBox16");
            this.groupBox16.Name = "groupBox16";
            this.groupBox16.TabStop = false;
            // 
            // KFF_PTCH2THR
            // 
            resources.ApplyResources(this.KFF_PTCH2THR, "KFF_PTCH2THR");
            this.KFF_PTCH2THR.Name = "KFF_PTCH2THR";
            // 
            // label83
            // 
            resources.ApplyResources(this.label83, "label83");
            this.label83.Name = "label83";
            // 
            // KFF_RDDRMIX
            // 
            resources.ApplyResources(this.KFF_RDDRMIX, "KFF_RDDRMIX");
            this.KFF_RDDRMIX.Name = "KFF_RDDRMIX";
            // 
            // label78
            // 
            resources.ApplyResources(this.label78, "label78");
            this.label78.Name = "label78";
            // 
            // KFF_PTCHCOMP
            // 
            resources.ApplyResources(this.KFF_PTCHCOMP, "KFF_PTCHCOMP");
            this.KFF_PTCHCOMP.Name = "KFF_PTCHCOMP";
            // 
            // label81
            // 
            resources.ApplyResources(this.label81, "label81");
            this.label81.Name = "label81";
            // 
            // groupBox14
            // 
            this.groupBox14.Controls.Add(this.ENRGY2THR_IMAX);
            this.groupBox14.Controls.Add(this.label73);
            this.groupBox14.Controls.Add(this.ENRGY2THR_D);
            this.groupBox14.Controls.Add(this.label74);
            this.groupBox14.Controls.Add(this.ENRGY2THR_I);
            this.groupBox14.Controls.Add(this.label75);
            this.groupBox14.Controls.Add(this.ENRGY2THR_P);
            this.groupBox14.Controls.Add(this.label76);
            resources.ApplyResources(this.groupBox14, "groupBox14");
            this.groupBox14.Name = "groupBox14";
            this.groupBox14.TabStop = false;
            // 
            // ENRGY2THR_IMAX
            // 
            resources.ApplyResources(this.ENRGY2THR_IMAX, "ENRGY2THR_IMAX");
            this.ENRGY2THR_IMAX.Name = "ENRGY2THR_IMAX";
            // 
            // label73
            // 
            resources.ApplyResources(this.label73, "label73");
            this.label73.Name = "label73";
            // 
            // ENRGY2THR_D
            // 
            resources.ApplyResources(this.ENRGY2THR_D, "ENRGY2THR_D");
            this.ENRGY2THR_D.Name = "ENRGY2THR_D";
            // 
            // label74
            // 
            resources.ApplyResources(this.label74, "label74");
            this.label74.Name = "label74";
            // 
            // ENRGY2THR_I
            // 
            resources.ApplyResources(this.ENRGY2THR_I, "ENRGY2THR_I");
            this.ENRGY2THR_I.Name = "ENRGY2THR_I";
            // 
            // label75
            // 
            resources.ApplyResources(this.label75, "label75");
            this.label75.Name = "label75";
            // 
            // ENRGY2THR_P
            // 
            resources.ApplyResources(this.ENRGY2THR_P, "ENRGY2THR_P");
            this.ENRGY2THR_P.Name = "ENRGY2THR_P";
            // 
            // label76
            // 
            resources.ApplyResources(this.label76, "label76");
            this.label76.Name = "label76";
            // 
            // groupBox13
            // 
            this.groupBox13.Controls.Add(this.ALT2PTCH_IMAX);
            this.groupBox13.Controls.Add(this.label69);
            this.groupBox13.Controls.Add(this.ALT2PTCH_D);
            this.groupBox13.Controls.Add(this.label70);
            this.groupBox13.Controls.Add(this.ALT2PTCH_I);
            this.groupBox13.Controls.Add(this.label71);
            this.groupBox13.Controls.Add(this.ALT2PTCH_P);
            this.groupBox13.Controls.Add(this.label72);
            resources.ApplyResources(this.groupBox13, "groupBox13");
            this.groupBox13.Name = "groupBox13";
            this.groupBox13.TabStop = false;
            // 
            // ALT2PTCH_IMAX
            // 
            resources.ApplyResources(this.ALT2PTCH_IMAX, "ALT2PTCH_IMAX");
            this.ALT2PTCH_IMAX.Name = "ALT2PTCH_IMAX";
            // 
            // label69
            // 
            resources.ApplyResources(this.label69, "label69");
            this.label69.Name = "label69";
            // 
            // ALT2PTCH_D
            // 
            resources.ApplyResources(this.ALT2PTCH_D, "ALT2PTCH_D");
            this.ALT2PTCH_D.Name = "ALT2PTCH_D";
            // 
            // label70
            // 
            resources.ApplyResources(this.label70, "label70");
            this.label70.Name = "label70";
            // 
            // ALT2PTCH_I
            // 
            resources.ApplyResources(this.ALT2PTCH_I, "ALT2PTCH_I");
            this.ALT2PTCH_I.Name = "ALT2PTCH_I";
            // 
            // label71
            // 
            resources.ApplyResources(this.label71, "label71");
            this.label71.Name = "label71";
            // 
            // ALT2PTCH_P
            // 
            resources.ApplyResources(this.ALT2PTCH_P, "ALT2PTCH_P");
            this.ALT2PTCH_P.Name = "ALT2PTCH_P";
            // 
            // label72
            // 
            resources.ApplyResources(this.label72, "label72");
            this.label72.Name = "label72";
            // 
            // groupBox12
            // 
            this.groupBox12.Controls.Add(this.ARSP2PTCH_IMAX);
            this.groupBox12.Controls.Add(this.label65);
            this.groupBox12.Controls.Add(this.ARSP2PTCH_D);
            this.groupBox12.Controls.Add(this.label66);
            this.groupBox12.Controls.Add(this.ARSP2PTCH_I);
            this.groupBox12.Controls.Add(this.label67);
            this.groupBox12.Controls.Add(this.ARSP2PTCH_P);
            this.groupBox12.Controls.Add(this.label68);
            resources.ApplyResources(this.groupBox12, "groupBox12");
            this.groupBox12.Name = "groupBox12";
            this.groupBox12.TabStop = false;
            // 
            // ARSP2PTCH_IMAX
            // 
            resources.ApplyResources(this.ARSP2PTCH_IMAX, "ARSP2PTCH_IMAX");
            this.ARSP2PTCH_IMAX.Name = "ARSP2PTCH_IMAX";
            // 
            // label65
            // 
            resources.ApplyResources(this.label65, "label65");
            this.label65.Name = "label65";
            // 
            // ARSP2PTCH_D
            // 
            resources.ApplyResources(this.ARSP2PTCH_D, "ARSP2PTCH_D");
            this.ARSP2PTCH_D.Name = "ARSP2PTCH_D";
            // 
            // label66
            // 
            resources.ApplyResources(this.label66, "label66");
            this.label66.Name = "label66";
            // 
            // ARSP2PTCH_I
            // 
            resources.ApplyResources(this.ARSP2PTCH_I, "ARSP2PTCH_I");
            this.ARSP2PTCH_I.Name = "ARSP2PTCH_I";
            // 
            // label67
            // 
            resources.ApplyResources(this.label67, "label67");
            this.label67.Name = "label67";
            // 
            // ARSP2PTCH_P
            // 
            resources.ApplyResources(this.ARSP2PTCH_P, "ARSP2PTCH_P");
            this.ARSP2PTCH_P.Name = "ARSP2PTCH_P";
            // 
            // label68
            // 
            resources.ApplyResources(this.label68, "label68");
            this.label68.Name = "label68";
            // 
            // groupBox11
            // 
            this.groupBox11.Controls.Add(this.HDNG2RLL_IMAX);
            this.groupBox11.Controls.Add(this.label61);
            this.groupBox11.Controls.Add(this.HDNG2RLL_D);
            this.groupBox11.Controls.Add(this.label62);
            this.groupBox11.Controls.Add(this.HDNG2RLL_I);
            this.groupBox11.Controls.Add(this.label63);
            this.groupBox11.Controls.Add(this.HDNG2RLL_P);
            this.groupBox11.Controls.Add(this.label64);
            resources.ApplyResources(this.groupBox11, "groupBox11");
            this.groupBox11.Name = "groupBox11";
            this.groupBox11.TabStop = false;
            // 
            // HDNG2RLL_IMAX
            // 
            resources.ApplyResources(this.HDNG2RLL_IMAX, "HDNG2RLL_IMAX");
            this.HDNG2RLL_IMAX.Name = "HDNG2RLL_IMAX";
            // 
            // label61
            // 
            resources.ApplyResources(this.label61, "label61");
            this.label61.Name = "label61";
            // 
            // HDNG2RLL_D
            // 
            resources.ApplyResources(this.HDNG2RLL_D, "HDNG2RLL_D");
            this.HDNG2RLL_D.Name = "HDNG2RLL_D";
            // 
            // label62
            // 
            resources.ApplyResources(this.label62, "label62");
            this.label62.Name = "label62";
            // 
            // HDNG2RLL_I
            // 
            resources.ApplyResources(this.HDNG2RLL_I, "HDNG2RLL_I");
            this.HDNG2RLL_I.Name = "HDNG2RLL_I";
            // 
            // label63
            // 
            resources.ApplyResources(this.label63, "label63");
            this.label63.Name = "label63";
            // 
            // HDNG2RLL_P
            // 
            resources.ApplyResources(this.HDNG2RLL_P, "HDNG2RLL_P");
            this.HDNG2RLL_P.Name = "HDNG2RLL_P";
            // 
            // label64
            // 
            resources.ApplyResources(this.label64, "label64");
            this.label64.Name = "label64";
            // 
            // groupBox10
            // 
            this.groupBox10.Controls.Add(this.YW2SRV_IMAX);
            this.groupBox10.Controls.Add(this.label57);
            this.groupBox10.Controls.Add(this.YW2SRV_D);
            this.groupBox10.Controls.Add(this.label58);
            this.groupBox10.Controls.Add(this.YW2SRV_I);
            this.groupBox10.Controls.Add(this.label59);
            this.groupBox10.Controls.Add(this.YW2SRV_P);
            this.groupBox10.Controls.Add(this.label60);
            resources.ApplyResources(this.groupBox10, "groupBox10");
            this.groupBox10.Name = "groupBox10";
            this.groupBox10.TabStop = false;
            // 
            // YW2SRV_IMAX
            // 
            resources.ApplyResources(this.YW2SRV_IMAX, "YW2SRV_IMAX");
            this.YW2SRV_IMAX.Name = "YW2SRV_IMAX";
            // 
            // label57
            // 
            resources.ApplyResources(this.label57, "label57");
            this.label57.Name = "label57";
            // 
            // YW2SRV_D
            // 
            resources.ApplyResources(this.YW2SRV_D, "YW2SRV_D");
            this.YW2SRV_D.Name = "YW2SRV_D";
            // 
            // label58
            // 
            resources.ApplyResources(this.label58, "label58");
            this.label58.Name = "label58";
            // 
            // YW2SRV_I
            // 
            resources.ApplyResources(this.YW2SRV_I, "YW2SRV_I");
            this.YW2SRV_I.Name = "YW2SRV_I";
            // 
            // label59
            // 
            resources.ApplyResources(this.label59, "label59");
            this.label59.Name = "label59";
            // 
            // YW2SRV_P
            // 
            resources.ApplyResources(this.YW2SRV_P, "YW2SRV_P");
            this.YW2SRV_P.Name = "YW2SRV_P";
            // 
            // label60
            // 
            resources.ApplyResources(this.label60, "label60");
            this.label60.Name = "label60";
            // 
            // groupBox9
            // 
            this.groupBox9.Controls.Add(this.PTCH2SRV_IMAX);
            this.groupBox9.Controls.Add(this.label53);
            this.groupBox9.Controls.Add(this.PTCH2SRV_D);
            this.groupBox9.Controls.Add(this.label54);
            this.groupBox9.Controls.Add(this.PTCH2SRV_I);
            this.groupBox9.Controls.Add(this.label55);
            this.groupBox9.Controls.Add(this.PTCH2SRV_P);
            this.groupBox9.Controls.Add(this.label56);
            resources.ApplyResources(this.groupBox9, "groupBox9");
            this.groupBox9.Name = "groupBox9";
            this.groupBox9.TabStop = false;
            // 
            // PTCH2SRV_IMAX
            // 
            resources.ApplyResources(this.PTCH2SRV_IMAX, "PTCH2SRV_IMAX");
            this.PTCH2SRV_IMAX.Name = "PTCH2SRV_IMAX";
            // 
            // label53
            // 
            resources.ApplyResources(this.label53, "label53");
            this.label53.Name = "label53";
            // 
            // PTCH2SRV_D
            // 
            resources.ApplyResources(this.PTCH2SRV_D, "PTCH2SRV_D");
            this.PTCH2SRV_D.Name = "PTCH2SRV_D";
            // 
            // label54
            // 
            resources.ApplyResources(this.label54, "label54");
            this.label54.Name = "label54";
            // 
            // PTCH2SRV_I
            // 
            resources.ApplyResources(this.PTCH2SRV_I, "PTCH2SRV_I");
            this.PTCH2SRV_I.Name = "PTCH2SRV_I";
            // 
            // label55
            // 
            resources.ApplyResources(this.label55, "label55");
            this.label55.Name = "label55";
            // 
            // PTCH2SRV_P
            // 
            resources.ApplyResources(this.PTCH2SRV_P, "PTCH2SRV_P");
            this.PTCH2SRV_P.Name = "PTCH2SRV_P";
            // 
            // label56
            // 
            resources.ApplyResources(this.label56, "label56");
            this.label56.Name = "label56";
            // 
            // groupBox8
            // 
            this.groupBox8.Controls.Add(this.RLL2SRV_IMAX);
            this.groupBox8.Controls.Add(this.label49);
            this.groupBox8.Controls.Add(this.RLL2SRV_D);
            this.groupBox8.Controls.Add(this.label50);
            this.groupBox8.Controls.Add(this.RLL2SRV_I);
            this.groupBox8.Controls.Add(this.label51);
            this.groupBox8.Controls.Add(this.RLL2SRV_P);
            this.groupBox8.Controls.Add(this.label52);
            resources.ApplyResources(this.groupBox8, "groupBox8");
            this.groupBox8.Name = "groupBox8";
            this.groupBox8.TabStop = false;
            // 
            // RLL2SRV_IMAX
            // 
            resources.ApplyResources(this.RLL2SRV_IMAX, "RLL2SRV_IMAX");
            this.RLL2SRV_IMAX.Name = "RLL2SRV_IMAX";
            // 
            // label49
            // 
            resources.ApplyResources(this.label49, "label49");
            this.label49.Name = "label49";
            // 
            // RLL2SRV_D
            // 
            resources.ApplyResources(this.RLL2SRV_D, "RLL2SRV_D");
            this.RLL2SRV_D.Name = "RLL2SRV_D";
            // 
            // label50
            // 
            resources.ApplyResources(this.label50, "label50");
            this.label50.Name = "label50";
            // 
            // RLL2SRV_I
            // 
            resources.ApplyResources(this.RLL2SRV_I, "RLL2SRV_I");
            this.RLL2SRV_I.Name = "RLL2SRV_I";
            // 
            // label51
            // 
            resources.ApplyResources(this.label51, "label51");
            this.label51.Name = "label51";
            // 
            // RLL2SRV_P
            // 
            resources.ApplyResources(this.RLL2SRV_P, "RLL2SRV_P");
            this.RLL2SRV_P.Name = "RLL2SRV_P";
            // 
            // label52
            // 
            resources.ApplyResources(this.label52, "label52");
            this.label52.Name = "label52";
            // 
            // TabAC2
            // 
            this.TabAC2.Controls.Add(this.myLabel1);
            this.TabAC2.Controls.Add(this.CH7_OPT);
            this.TabAC2.Controls.Add(this.groupBox17);
            this.TabAC2.Controls.Add(this.groupBox5);
            this.TabAC2.Controls.Add(this.groupBox18);
            this.TabAC2.Controls.Add(this.CHK_lockrollpitch);
            this.TabAC2.Controls.Add(this.groupBox4);
            this.TabAC2.Controls.Add(this.groupBox6);
            this.TabAC2.Controls.Add(this.groupBox7);
            this.TabAC2.Controls.Add(this.groupBox19);
            this.TabAC2.Controls.Add(this.groupBox20);
            this.TabAC2.Controls.Add(this.groupBox21);
            this.TabAC2.Controls.Add(this.groupBox22);
            this.TabAC2.Controls.Add(this.groupBox23);
            this.TabAC2.Controls.Add(this.groupBox24);
            this.TabAC2.Controls.Add(this.groupBox25);
            resources.ApplyResources(this.TabAC2, "TabAC2");
            this.TabAC2.Name = "TabAC2";
            // 
            // myLabel1
            // 
            resources.ApplyResources(this.myLabel1, "myLabel1");
            this.myLabel1.Name = "myLabel1";
            this.myLabel1.resize = false;
            // 
            // CH7_OPT
            // 
            this.CH7_OPT.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CH7_OPT.FormattingEnabled = true;
            this.CH7_OPT.Items.AddRange(new object[] {
            resources.GetString("CH7_OPT.Items"),
            resources.GetString("CH7_OPT.Items1"),
            resources.GetString("CH7_OPT.Items2"),
            resources.GetString("CH7_OPT.Items3"),
            resources.GetString("CH7_OPT.Items4"),
            resources.GetString("CH7_OPT.Items5"),
            resources.GetString("CH7_OPT.Items6"),
            resources.GetString("CH7_OPT.Items7")});
            resources.ApplyResources(this.CH7_OPT, "CH7_OPT");
            this.CH7_OPT.Name = "CH7_OPT";
            // 
            // groupBox17
            // 
            this.groupBox17.Controls.Add(this.ACRO_PIT_IMAX);
            this.groupBox17.Controls.Add(this.label27);
            this.groupBox17.Controls.Add(this.ACRO_PIT_I);
            this.groupBox17.Controls.Add(this.label29);
            this.groupBox17.Controls.Add(this.ACRO_PIT_P);
            this.groupBox17.Controls.Add(this.label33);
            resources.ApplyResources(this.groupBox17, "groupBox17");
            this.groupBox17.Name = "groupBox17";
            this.groupBox17.TabStop = false;
            // 
            // ACRO_PIT_IMAX
            // 
            resources.ApplyResources(this.ACRO_PIT_IMAX, "ACRO_PIT_IMAX");
            this.ACRO_PIT_IMAX.Name = "ACRO_PIT_IMAX";
            // 
            // label27
            // 
            resources.ApplyResources(this.label27, "label27");
            this.label27.Name = "label27";
            // 
            // ACRO_PIT_I
            // 
            resources.ApplyResources(this.ACRO_PIT_I, "ACRO_PIT_I");
            this.ACRO_PIT_I.Name = "ACRO_PIT_I";
            // 
            // label29
            // 
            resources.ApplyResources(this.label29, "label29");
            this.label29.Name = "label29";
            // 
            // ACRO_PIT_P
            // 
            resources.ApplyResources(this.ACRO_PIT_P, "ACRO_PIT_P");
            this.ACRO_PIT_P.Name = "ACRO_PIT_P";
            // 
            // label33
            // 
            resources.ApplyResources(this.label33, "label33");
            this.label33.Name = "label33";
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.label14);
            this.groupBox5.Controls.Add(this.THR_RATE_IMAX);
            this.groupBox5.Controls.Add(this.THR_RATE_I);
            this.groupBox5.Controls.Add(this.label20);
            this.groupBox5.Controls.Add(this.THR_RATE_P);
            this.groupBox5.Controls.Add(this.label25);
            resources.ApplyResources(this.groupBox5, "groupBox5");
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.TabStop = false;
            // 
            // label14
            // 
            resources.ApplyResources(this.label14, "label14");
            this.label14.Name = "label14";
            // 
            // THR_RATE_IMAX
            // 
            resources.ApplyResources(this.THR_RATE_IMAX, "THR_RATE_IMAX");
            this.THR_RATE_IMAX.Name = "THR_RATE_IMAX";
            // 
            // THR_RATE_I
            // 
            resources.ApplyResources(this.THR_RATE_I, "THR_RATE_I");
            this.THR_RATE_I.Name = "THR_RATE_I";
            // 
            // label20
            // 
            resources.ApplyResources(this.label20, "label20");
            this.label20.Name = "label20";
            // 
            // THR_RATE_P
            // 
            resources.ApplyResources(this.THR_RATE_P, "THR_RATE_P");
            this.THR_RATE_P.Name = "THR_RATE_P";
            // 
            // label25
            // 
            resources.ApplyResources(this.label25, "label25");
            this.label25.Name = "label25";
            // 
            // groupBox18
            // 
            this.groupBox18.Controls.Add(this.ACRO_RLL_IMAX);
            this.groupBox18.Controls.Add(this.label40);
            this.groupBox18.Controls.Add(this.ACRO_RLL_I);
            this.groupBox18.Controls.Add(this.label44);
            this.groupBox18.Controls.Add(this.ACRO_RLL_P);
            this.groupBox18.Controls.Add(this.label48);
            resources.ApplyResources(this.groupBox18, "groupBox18");
            this.groupBox18.Name = "groupBox18";
            this.groupBox18.TabStop = false;
            // 
            // ACRO_RLL_IMAX
            // 
            resources.ApplyResources(this.ACRO_RLL_IMAX, "ACRO_RLL_IMAX");
            this.ACRO_RLL_IMAX.Name = "ACRO_RLL_IMAX";
            // 
            // label40
            // 
            resources.ApplyResources(this.label40, "label40");
            this.label40.Name = "label40";
            // 
            // ACRO_RLL_I
            // 
            resources.ApplyResources(this.ACRO_RLL_I, "ACRO_RLL_I");
            this.ACRO_RLL_I.Name = "ACRO_RLL_I";
            // 
            // label44
            // 
            resources.ApplyResources(this.label44, "label44");
            this.label44.Name = "label44";
            // 
            // ACRO_RLL_P
            // 
            resources.ApplyResources(this.ACRO_RLL_P, "ACRO_RLL_P");
            this.ACRO_RLL_P.Name = "ACRO_RLL_P";
            // 
            // label48
            // 
            resources.ApplyResources(this.label48, "label48");
            this.label48.Name = "label48";
            // 
            // CHK_lockrollpitch
            // 
            resources.ApplyResources(this.CHK_lockrollpitch, "CHK_lockrollpitch");
            this.CHK_lockrollpitch.Checked = true;
            this.CHK_lockrollpitch.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_lockrollpitch.Name = "CHK_lockrollpitch";
            this.CHK_lockrollpitch.UseVisualStyleBackColor = true;
            this.CHK_lockrollpitch.CheckedChanged += new System.EventHandler(this.CHK_lockrollpitch_CheckedChanged);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.WP_SPEED_MAX);
            this.groupBox4.Controls.Add(this.label9);
            this.groupBox4.Controls.Add(this.NAV_LAT_IMAX);
            this.groupBox4.Controls.Add(this.label13);
            this.groupBox4.Controls.Add(this.NAV_LAT_I);
            this.groupBox4.Controls.Add(this.label15);
            this.groupBox4.Controls.Add(this.NAV_LAT_P);
            this.groupBox4.Controls.Add(this.label16);
            resources.ApplyResources(this.groupBox4, "groupBox4");
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.TabStop = false;
            // 
            // WP_SPEED_MAX
            // 
            resources.ApplyResources(this.WP_SPEED_MAX, "WP_SPEED_MAX");
            this.WP_SPEED_MAX.Name = "WP_SPEED_MAX";
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            // 
            // NAV_LAT_IMAX
            // 
            resources.ApplyResources(this.NAV_LAT_IMAX, "NAV_LAT_IMAX");
            this.NAV_LAT_IMAX.Name = "NAV_LAT_IMAX";
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.Name = "label13";
            // 
            // NAV_LAT_I
            // 
            resources.ApplyResources(this.NAV_LAT_I, "NAV_LAT_I");
            this.NAV_LAT_I.Name = "NAV_LAT_I";
            // 
            // label15
            // 
            resources.ApplyResources(this.label15, "label15");
            this.label15.Name = "label15";
            // 
            // NAV_LAT_P
            // 
            resources.ApplyResources(this.NAV_LAT_P, "NAV_LAT_P");
            this.NAV_LAT_P.Name = "NAV_LAT_P";
            // 
            // label16
            // 
            resources.ApplyResources(this.label16, "label16");
            this.label16.Name = "label16";
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.XTRK_GAIN_SC1);
            this.groupBox6.Controls.Add(this.label18);
            resources.ApplyResources(this.groupBox6, "groupBox6");
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.TabStop = false;
            // 
            // XTRK_GAIN_SC1
            // 
            resources.ApplyResources(this.XTRK_GAIN_SC1, "XTRK_GAIN_SC1");
            this.XTRK_GAIN_SC1.Name = "XTRK_GAIN_SC1";
            // 
            // label18
            // 
            resources.ApplyResources(this.label18, "label18");
            this.label18.Name = "label18";
            // 
            // groupBox7
            // 
            this.groupBox7.Controls.Add(this.THR_ALT_IMAX);
            this.groupBox7.Controls.Add(this.label19);
            this.groupBox7.Controls.Add(this.THR_ALT_I);
            this.groupBox7.Controls.Add(this.label21);
            this.groupBox7.Controls.Add(this.THR_ALT_P);
            this.groupBox7.Controls.Add(this.label22);
            resources.ApplyResources(this.groupBox7, "groupBox7");
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.TabStop = false;
            // 
            // THR_ALT_IMAX
            // 
            resources.ApplyResources(this.THR_ALT_IMAX, "THR_ALT_IMAX");
            this.THR_ALT_IMAX.Name = "THR_ALT_IMAX";
            // 
            // label19
            // 
            resources.ApplyResources(this.label19, "label19");
            this.label19.Name = "label19";
            // 
            // THR_ALT_I
            // 
            resources.ApplyResources(this.THR_ALT_I, "THR_ALT_I");
            this.THR_ALT_I.Name = "THR_ALT_I";
            // 
            // label21
            // 
            resources.ApplyResources(this.label21, "label21");
            this.label21.Name = "label21";
            // 
            // THR_ALT_P
            // 
            resources.ApplyResources(this.THR_ALT_P, "THR_ALT_P");
            this.THR_ALT_P.Name = "THR_ALT_P";
            // 
            // label22
            // 
            resources.ApplyResources(this.label22, "label22");
            this.label22.Name = "label22";
            // 
            // groupBox19
            // 
            this.groupBox19.Controls.Add(this.HLD_LAT_IMAX);
            this.groupBox19.Controls.Add(this.label28);
            this.groupBox19.Controls.Add(this.HLD_LAT_I);
            this.groupBox19.Controls.Add(this.label30);
            this.groupBox19.Controls.Add(this.HLD_LAT_P);
            this.groupBox19.Controls.Add(this.label31);
            resources.ApplyResources(this.groupBox19, "groupBox19");
            this.groupBox19.Name = "groupBox19";
            this.groupBox19.TabStop = false;
            // 
            // HLD_LAT_IMAX
            // 
            resources.ApplyResources(this.HLD_LAT_IMAX, "HLD_LAT_IMAX");
            this.HLD_LAT_IMAX.Name = "HLD_LAT_IMAX";
            // 
            // label28
            // 
            resources.ApplyResources(this.label28, "label28");
            this.label28.Name = "label28";
            // 
            // HLD_LAT_I
            // 
            resources.ApplyResources(this.HLD_LAT_I, "HLD_LAT_I");
            this.HLD_LAT_I.Name = "HLD_LAT_I";
            // 
            // label30
            // 
            resources.ApplyResources(this.label30, "label30");
            this.label30.Name = "label30";
            // 
            // HLD_LAT_P
            // 
            resources.ApplyResources(this.HLD_LAT_P, "HLD_LAT_P");
            this.HLD_LAT_P.Name = "HLD_LAT_P";
            // 
            // label31
            // 
            resources.ApplyResources(this.label31, "label31");
            this.label31.Name = "label31";
            // 
            // groupBox20
            // 
            this.groupBox20.Controls.Add(this.STB_YAW_IMAX);
            this.groupBox20.Controls.Add(this.label32);
            this.groupBox20.Controls.Add(this.STB_YAW_I);
            this.groupBox20.Controls.Add(this.label34);
            this.groupBox20.Controls.Add(this.STB_YAW_P);
            this.groupBox20.Controls.Add(this.label35);
            resources.ApplyResources(this.groupBox20, "groupBox20");
            this.groupBox20.Name = "groupBox20";
            this.groupBox20.TabStop = false;
            // 
            // STB_YAW_IMAX
            // 
            resources.ApplyResources(this.STB_YAW_IMAX, "STB_YAW_IMAX");
            this.STB_YAW_IMAX.Name = "STB_YAW_IMAX";
            // 
            // label32
            // 
            resources.ApplyResources(this.label32, "label32");
            this.label32.Name = "label32";
            // 
            // STB_YAW_I
            // 
            resources.ApplyResources(this.STB_YAW_I, "STB_YAW_I");
            this.STB_YAW_I.Name = "STB_YAW_I";
            // 
            // label34
            // 
            resources.ApplyResources(this.label34, "label34");
            this.label34.Name = "label34";
            // 
            // STB_YAW_P
            // 
            resources.ApplyResources(this.STB_YAW_P, "STB_YAW_P");
            this.STB_YAW_P.Name = "STB_YAW_P";
            // 
            // label35
            // 
            resources.ApplyResources(this.label35, "label35");
            this.label35.Name = "label35";
            // 
            // groupBox21
            // 
            this.groupBox21.Controls.Add(this.STB_PIT_IMAX);
            this.groupBox21.Controls.Add(this.label36);
            this.groupBox21.Controls.Add(this.STB_PIT_I);
            this.groupBox21.Controls.Add(this.label41);
            this.groupBox21.Controls.Add(this.STB_PIT_P);
            this.groupBox21.Controls.Add(this.label42);
            resources.ApplyResources(this.groupBox21, "groupBox21");
            this.groupBox21.Name = "groupBox21";
            this.groupBox21.TabStop = false;
            // 
            // STB_PIT_IMAX
            // 
            resources.ApplyResources(this.STB_PIT_IMAX, "STB_PIT_IMAX");
            this.STB_PIT_IMAX.Name = "STB_PIT_IMAX";
            // 
            // label36
            // 
            resources.ApplyResources(this.label36, "label36");
            this.label36.Name = "label36";
            // 
            // STB_PIT_I
            // 
            resources.ApplyResources(this.STB_PIT_I, "STB_PIT_I");
            this.STB_PIT_I.Name = "STB_PIT_I";
            // 
            // label41
            // 
            resources.ApplyResources(this.label41, "label41");
            this.label41.Name = "label41";
            // 
            // STB_PIT_P
            // 
            resources.ApplyResources(this.STB_PIT_P, "STB_PIT_P");
            this.STB_PIT_P.Name = "STB_PIT_P";
            // 
            // label42
            // 
            resources.ApplyResources(this.label42, "label42");
            this.label42.Name = "label42";
            // 
            // groupBox22
            // 
            this.groupBox22.Controls.Add(this.STB_RLL_IMAX);
            this.groupBox22.Controls.Add(this.label43);
            this.groupBox22.Controls.Add(this.STB_RLL_I);
            this.groupBox22.Controls.Add(this.label45);
            this.groupBox22.Controls.Add(this.STB_RLL_P);
            this.groupBox22.Controls.Add(this.label46);
            resources.ApplyResources(this.groupBox22, "groupBox22");
            this.groupBox22.Name = "groupBox22";
            this.groupBox22.TabStop = false;
            // 
            // STB_RLL_IMAX
            // 
            resources.ApplyResources(this.STB_RLL_IMAX, "STB_RLL_IMAX");
            this.STB_RLL_IMAX.Name = "STB_RLL_IMAX";
            // 
            // label43
            // 
            resources.ApplyResources(this.label43, "label43");
            this.label43.Name = "label43";
            // 
            // STB_RLL_I
            // 
            resources.ApplyResources(this.STB_RLL_I, "STB_RLL_I");
            this.STB_RLL_I.Name = "STB_RLL_I";
            // 
            // label45
            // 
            resources.ApplyResources(this.label45, "label45");
            this.label45.Name = "label45";
            // 
            // STB_RLL_P
            // 
            resources.ApplyResources(this.STB_RLL_P, "STB_RLL_P");
            this.STB_RLL_P.Name = "STB_RLL_P";
            // 
            // label46
            // 
            resources.ApplyResources(this.label46, "label46");
            this.label46.Name = "label46";
            // 
            // groupBox23
            // 
            this.groupBox23.Controls.Add(this.RATE_YAW_IMAX);
            this.groupBox23.Controls.Add(this.label47);
            this.groupBox23.Controls.Add(this.RATE_YAW_I);
            this.groupBox23.Controls.Add(this.label77);
            this.groupBox23.Controls.Add(this.RATE_YAW_P);
            this.groupBox23.Controls.Add(this.label82);
            resources.ApplyResources(this.groupBox23, "groupBox23");
            this.groupBox23.Name = "groupBox23";
            this.groupBox23.TabStop = false;
            // 
            // RATE_YAW_IMAX
            // 
            resources.ApplyResources(this.RATE_YAW_IMAX, "RATE_YAW_IMAX");
            this.RATE_YAW_IMAX.Name = "RATE_YAW_IMAX";
            // 
            // label47
            // 
            resources.ApplyResources(this.label47, "label47");
            this.label47.Name = "label47";
            // 
            // RATE_YAW_I
            // 
            resources.ApplyResources(this.RATE_YAW_I, "RATE_YAW_I");
            this.RATE_YAW_I.Name = "RATE_YAW_I";
            // 
            // label77
            // 
            resources.ApplyResources(this.label77, "label77");
            this.label77.Name = "label77";
            // 
            // RATE_YAW_P
            // 
            resources.ApplyResources(this.RATE_YAW_P, "RATE_YAW_P");
            this.RATE_YAW_P.Name = "RATE_YAW_P";
            // 
            // label82
            // 
            resources.ApplyResources(this.label82, "label82");
            this.label82.Name = "label82";
            // 
            // groupBox24
            // 
            this.groupBox24.Controls.Add(this.RATE_PIT_IMAX);
            this.groupBox24.Controls.Add(this.label84);
            this.groupBox24.Controls.Add(this.RATE_PIT_I);
            this.groupBox24.Controls.Add(this.label86);
            this.groupBox24.Controls.Add(this.RATE_PIT_P);
            this.groupBox24.Controls.Add(this.label87);
            resources.ApplyResources(this.groupBox24, "groupBox24");
            this.groupBox24.Name = "groupBox24";
            this.groupBox24.TabStop = false;
            // 
            // RATE_PIT_IMAX
            // 
            resources.ApplyResources(this.RATE_PIT_IMAX, "RATE_PIT_IMAX");
            this.RATE_PIT_IMAX.Name = "RATE_PIT_IMAX";
            // 
            // label84
            // 
            resources.ApplyResources(this.label84, "label84");
            this.label84.Name = "label84";
            // 
            // RATE_PIT_I
            // 
            resources.ApplyResources(this.RATE_PIT_I, "RATE_PIT_I");
            this.RATE_PIT_I.Name = "RATE_PIT_I";
            // 
            // label86
            // 
            resources.ApplyResources(this.label86, "label86");
            this.label86.Name = "label86";
            // 
            // RATE_PIT_P
            // 
            resources.ApplyResources(this.RATE_PIT_P, "RATE_PIT_P");
            this.RATE_PIT_P.Name = "RATE_PIT_P";
            // 
            // label87
            // 
            resources.ApplyResources(this.label87, "label87");
            this.label87.Name = "label87";
            // 
            // groupBox25
            // 
            this.groupBox25.Controls.Add(this.RATE_RLL_IMAX);
            this.groupBox25.Controls.Add(this.label88);
            this.groupBox25.Controls.Add(this.RATE_RLL_I);
            this.groupBox25.Controls.Add(this.label90);
            this.groupBox25.Controls.Add(this.RATE_RLL_P);
            this.groupBox25.Controls.Add(this.label91);
            resources.ApplyResources(this.groupBox25, "groupBox25");
            this.groupBox25.Name = "groupBox25";
            this.groupBox25.TabStop = false;
            // 
            // RATE_RLL_IMAX
            // 
            resources.ApplyResources(this.RATE_RLL_IMAX, "RATE_RLL_IMAX");
            this.RATE_RLL_IMAX.Name = "RATE_RLL_IMAX";
            // 
            // label88
            // 
            resources.ApplyResources(this.label88, "label88");
            this.label88.Name = "label88";
            // 
            // RATE_RLL_I
            // 
            resources.ApplyResources(this.RATE_RLL_I, "RATE_RLL_I");
            this.RATE_RLL_I.Name = "RATE_RLL_I";
            // 
            // label90
            // 
            resources.ApplyResources(this.label90, "label90");
            this.label90.Name = "label90";
            // 
            // RATE_RLL_P
            // 
            resources.ApplyResources(this.RATE_RLL_P, "RATE_RLL_P");
            this.RATE_RLL_P.Name = "RATE_RLL_P";
            // 
            // label91
            // 
            resources.ApplyResources(this.label91, "label91");
            this.label91.Name = "label91";
            // 
            // TabPlanner
            // 
            this.TabPlanner.Controls.Add(this.label26);
            this.TabPlanner.Controls.Add(this.CMB_videoresolutions);
            this.TabPlanner.Controls.Add(this.label12);
            this.TabPlanner.Controls.Add(this.CHK_GDIPlus);
            this.TabPlanner.Controls.Add(this.label24);
            this.TabPlanner.Controls.Add(this.CHK_loadwponconnect);
            this.TabPlanner.Controls.Add(this.label23);
            this.TabPlanner.Controls.Add(this.NUM_tracklength);
            this.TabPlanner.Controls.Add(this.CHK_speechaltwarning);
            this.TabPlanner.Controls.Add(this.label108);
            this.TabPlanner.Controls.Add(this.CHK_resetapmonconnect);
            this.TabPlanner.Controls.Add(this.CHK_mavdebug);
            this.TabPlanner.Controls.Add(this.label107);
            this.TabPlanner.Controls.Add(this.CMB_raterc);
            this.TabPlanner.Controls.Add(this.label104);
            this.TabPlanner.Controls.Add(this.label103);
            this.TabPlanner.Controls.Add(this.label102);
            this.TabPlanner.Controls.Add(this.label101);
            this.TabPlanner.Controls.Add(this.CMB_ratestatus);
            this.TabPlanner.Controls.Add(this.CMB_rateposition);
            this.TabPlanner.Controls.Add(this.CMB_rateattitude);
            this.TabPlanner.Controls.Add(this.label99);
            this.TabPlanner.Controls.Add(this.label98);
            this.TabPlanner.Controls.Add(this.label97);
            this.TabPlanner.Controls.Add(this.CMB_speedunits);
            this.TabPlanner.Controls.Add(this.CMB_distunits);
            this.TabPlanner.Controls.Add(this.label96);
            this.TabPlanner.Controls.Add(this.label95);
            this.TabPlanner.Controls.Add(this.CHK_speechbattery);
            this.TabPlanner.Controls.Add(this.CHK_speechcustom);
            this.TabPlanner.Controls.Add(this.CHK_speechmode);
            this.TabPlanner.Controls.Add(this.CHK_speechwaypoint);
            this.TabPlanner.Controls.Add(this.label94);
            this.TabPlanner.Controls.Add(this.CMB_osdcolor);
            this.TabPlanner.Controls.Add(this.CMB_language);
            this.TabPlanner.Controls.Add(this.label93);
            this.TabPlanner.Controls.Add(this.CHK_enablespeech);
            this.TabPlanner.Controls.Add(this.CHK_hudshow);
            this.TabPlanner.Controls.Add(this.label92);
            this.TabPlanner.Controls.Add(this.CMB_videosources);
            this.TabPlanner.Controls.Add(this.BUT_Joystick);
            this.TabPlanner.Controls.Add(this.BUT_videostop);
            this.TabPlanner.Controls.Add(this.BUT_videostart);
            resources.ApplyResources(this.TabPlanner, "TabPlanner");
            this.TabPlanner.Name = "TabPlanner";
            // 
            // label26
            // 
            resources.ApplyResources(this.label26, "label26");
            this.label26.Name = "label26";
            // 
            // CMB_videoresolutions
            // 
            this.CMB_videoresolutions.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_videoresolutions.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_videoresolutions, "CMB_videoresolutions");
            this.CMB_videoresolutions.Name = "CMB_videoresolutions";
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.Name = "label12";
            // 
            // CHK_GDIPlus
            // 
            resources.ApplyResources(this.CHK_GDIPlus, "CHK_GDIPlus");
            this.CHK_GDIPlus.Name = "CHK_GDIPlus";
            this.toolTip1.SetToolTip(this.CHK_GDIPlus, resources.GetString("CHK_GDIPlus.ToolTip"));
            this.CHK_GDIPlus.UseVisualStyleBackColor = true;
            this.CHK_GDIPlus.CheckedChanged += new System.EventHandler(this.CHK_GDIPlus_CheckedChanged);
            // 
            // label24
            // 
            resources.ApplyResources(this.label24, "label24");
            this.label24.Name = "label24";
            // 
            // CHK_loadwponconnect
            // 
            resources.ApplyResources(this.CHK_loadwponconnect, "CHK_loadwponconnect");
            this.CHK_loadwponconnect.Name = "CHK_loadwponconnect";
            this.CHK_loadwponconnect.UseVisualStyleBackColor = true;
            this.CHK_loadwponconnect.CheckedChanged += new System.EventHandler(this.CHK_loadwponconnect_CheckedChanged);
            // 
            // label23
            // 
            resources.ApplyResources(this.label23, "label23");
            this.label23.Name = "label23";
            // 
            // NUM_tracklength
            // 
            this.NUM_tracklength.Increment = new decimal(new int[] {
            100,
            0,
            0,
            0});
            resources.ApplyResources(this.NUM_tracklength, "NUM_tracklength");
            this.NUM_tracklength.Maximum = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            this.NUM_tracklength.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            0});
            this.NUM_tracklength.Name = "NUM_tracklength";
            this.toolTip1.SetToolTip(this.NUM_tracklength, resources.GetString("NUM_tracklength.ToolTip"));
            this.NUM_tracklength.Value = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.NUM_tracklength.ValueChanged += new System.EventHandler(this.NUM_tracklength_ValueChanged);
            // 
            // CHK_speechaltwarning
            // 
            resources.ApplyResources(this.CHK_speechaltwarning, "CHK_speechaltwarning");
            this.CHK_speechaltwarning.Name = "CHK_speechaltwarning";
            this.CHK_speechaltwarning.UseVisualStyleBackColor = true;
            this.CHK_speechaltwarning.CheckedChanged += new System.EventHandler(this.CHK_speechaltwarning_CheckedChanged);
            // 
            // label108
            // 
            resources.ApplyResources(this.label108, "label108");
            this.label108.Name = "label108";
            // 
            // CHK_resetapmonconnect
            // 
            this.CHK_resetapmonconnect.Checked = true;
            this.CHK_resetapmonconnect.CheckState = System.Windows.Forms.CheckState.Checked;
            resources.ApplyResources(this.CHK_resetapmonconnect, "CHK_resetapmonconnect");
            this.CHK_resetapmonconnect.Name = "CHK_resetapmonconnect";
            this.CHK_resetapmonconnect.UseVisualStyleBackColor = true;
            this.CHK_resetapmonconnect.CheckedChanged += new System.EventHandler(this.CHK_resetapmonconnect_CheckedChanged);
            // 
            // CHK_mavdebug
            // 
            resources.ApplyResources(this.CHK_mavdebug, "CHK_mavdebug");
            this.CHK_mavdebug.Name = "CHK_mavdebug";
            this.CHK_mavdebug.UseVisualStyleBackColor = true;
            this.CHK_mavdebug.CheckedChanged += new System.EventHandler(this.CHK_mavdebug_CheckedChanged);
            // 
            // label107
            // 
            resources.ApplyResources(this.label107, "label107");
            this.label107.Name = "label107";
            // 
            // CMB_raterc
            // 
            this.CMB_raterc.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_raterc.FormattingEnabled = true;
            this.CMB_raterc.Items.AddRange(new object[] {
            resources.GetString("CMB_raterc.Items"),
            resources.GetString("CMB_raterc.Items1"),
            resources.GetString("CMB_raterc.Items2"),
            resources.GetString("CMB_raterc.Items3")});
            resources.ApplyResources(this.CMB_raterc, "CMB_raterc");
            this.CMB_raterc.Name = "CMB_raterc";
            this.CMB_raterc.SelectedIndexChanged += new System.EventHandler(this.CMB_raterc_SelectedIndexChanged);
            // 
            // label104
            // 
            resources.ApplyResources(this.label104, "label104");
            this.label104.Name = "label104";
            // 
            // label103
            // 
            resources.ApplyResources(this.label103, "label103");
            this.label103.Name = "label103";
            // 
            // label102
            // 
            resources.ApplyResources(this.label102, "label102");
            this.label102.Name = "label102";
            // 
            // label101
            // 
            resources.ApplyResources(this.label101, "label101");
            this.label101.Name = "label101";
            // 
            // CMB_ratestatus
            // 
            this.CMB_ratestatus.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_ratestatus.FormattingEnabled = true;
            this.CMB_ratestatus.Items.AddRange(new object[] {
            resources.GetString("CMB_ratestatus.Items"),
            resources.GetString("CMB_ratestatus.Items1"),
            resources.GetString("CMB_ratestatus.Items2"),
            resources.GetString("CMB_ratestatus.Items3")});
            resources.ApplyResources(this.CMB_ratestatus, "CMB_ratestatus");
            this.CMB_ratestatus.Name = "CMB_ratestatus";
            this.CMB_ratestatus.SelectedIndexChanged += new System.EventHandler(this.CMB_ratestatus_SelectedIndexChanged);
            // 
            // CMB_rateposition
            // 
            this.CMB_rateposition.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_rateposition.FormattingEnabled = true;
            this.CMB_rateposition.Items.AddRange(new object[] {
            resources.GetString("CMB_rateposition.Items"),
            resources.GetString("CMB_rateposition.Items1"),
            resources.GetString("CMB_rateposition.Items2"),
            resources.GetString("CMB_rateposition.Items3")});
            resources.ApplyResources(this.CMB_rateposition, "CMB_rateposition");
            this.CMB_rateposition.Name = "CMB_rateposition";
            this.CMB_rateposition.SelectedIndexChanged += new System.EventHandler(this.CMB_rateposition_SelectedIndexChanged);
            // 
            // CMB_rateattitude
            // 
            this.CMB_rateattitude.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_rateattitude.FormattingEnabled = true;
            this.CMB_rateattitude.Items.AddRange(new object[] {
            resources.GetString("CMB_rateattitude.Items"),
            resources.GetString("CMB_rateattitude.Items1"),
            resources.GetString("CMB_rateattitude.Items2"),
            resources.GetString("CMB_rateattitude.Items3")});
            resources.ApplyResources(this.CMB_rateattitude, "CMB_rateattitude");
            this.CMB_rateattitude.Name = "CMB_rateattitude";
            this.CMB_rateattitude.SelectedIndexChanged += new System.EventHandler(this.CMB_rateattitude_SelectedIndexChanged);
            // 
            // label99
            // 
            resources.ApplyResources(this.label99, "label99");
            this.label99.Name = "label99";
            // 
            // label98
            // 
            resources.ApplyResources(this.label98, "label98");
            this.label98.Name = "label98";
            // 
            // label97
            // 
            resources.ApplyResources(this.label97, "label97");
            this.label97.Name = "label97";
            // 
            // CMB_speedunits
            // 
            this.CMB_speedunits.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_speedunits.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_speedunits, "CMB_speedunits");
            this.CMB_speedunits.Name = "CMB_speedunits";
            this.CMB_speedunits.SelectedIndexChanged += new System.EventHandler(this.CMB_speedunits_SelectedIndexChanged);
            // 
            // CMB_distunits
            // 
            this.CMB_distunits.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_distunits.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_distunits, "CMB_distunits");
            this.CMB_distunits.Name = "CMB_distunits";
            this.CMB_distunits.SelectedIndexChanged += new System.EventHandler(this.CMB_distunits_SelectedIndexChanged);
            // 
            // label96
            // 
            resources.ApplyResources(this.label96, "label96");
            this.label96.Name = "label96";
            // 
            // label95
            // 
            resources.ApplyResources(this.label95, "label95");
            this.label95.Name = "label95";
            // 
            // CHK_speechbattery
            // 
            resources.ApplyResources(this.CHK_speechbattery, "CHK_speechbattery");
            this.CHK_speechbattery.Name = "CHK_speechbattery";
            this.CHK_speechbattery.UseVisualStyleBackColor = true;
            this.CHK_speechbattery.CheckedChanged += new System.EventHandler(this.CHK_speechbattery_CheckedChanged);
            // 
            // CHK_speechcustom
            // 
            resources.ApplyResources(this.CHK_speechcustom, "CHK_speechcustom");
            this.CHK_speechcustom.Name = "CHK_speechcustom";
            this.CHK_speechcustom.UseVisualStyleBackColor = true;
            this.CHK_speechcustom.CheckedChanged += new System.EventHandler(this.CHK_speechcustom_CheckedChanged);
            // 
            // CHK_speechmode
            // 
            resources.ApplyResources(this.CHK_speechmode, "CHK_speechmode");
            this.CHK_speechmode.Name = "CHK_speechmode";
            this.CHK_speechmode.UseVisualStyleBackColor = true;
            this.CHK_speechmode.CheckedChanged += new System.EventHandler(this.CHK_speechmode_CheckedChanged);
            // 
            // CHK_speechwaypoint
            // 
            resources.ApplyResources(this.CHK_speechwaypoint, "CHK_speechwaypoint");
            this.CHK_speechwaypoint.Name = "CHK_speechwaypoint";
            this.CHK_speechwaypoint.UseVisualStyleBackColor = true;
            this.CHK_speechwaypoint.CheckedChanged += new System.EventHandler(this.CHK_speechwaypoint_CheckedChanged);
            // 
            // label94
            // 
            resources.ApplyResources(this.label94, "label94");
            this.label94.Name = "label94";
            // 
            // CMB_osdcolor
            // 
            this.CMB_osdcolor.DrawMode = System.Windows.Forms.DrawMode.OwnerDrawFixed;
            this.CMB_osdcolor.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_osdcolor.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_osdcolor, "CMB_osdcolor");
            this.CMB_osdcolor.Name = "CMB_osdcolor";
            this.CMB_osdcolor.DrawItem += new System.Windows.Forms.DrawItemEventHandler(this.CMB_osdcolor_DrawItem);
            this.CMB_osdcolor.SelectedIndexChanged += new System.EventHandler(this.CMB_osdcolor_SelectedIndexChanged);
            // 
            // CMB_language
            // 
            this.CMB_language.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_language.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_language, "CMB_language");
            this.CMB_language.Name = "CMB_language";
            // 
            // label93
            // 
            resources.ApplyResources(this.label93, "label93");
            this.label93.Name = "label93";
            // 
            // CHK_enablespeech
            // 
            resources.ApplyResources(this.CHK_enablespeech, "CHK_enablespeech");
            this.CHK_enablespeech.Name = "CHK_enablespeech";
            this.CHK_enablespeech.UseVisualStyleBackColor = true;
            this.CHK_enablespeech.CheckedChanged += new System.EventHandler(this.CHK_enablespeech_CheckedChanged);
            // 
            // CHK_hudshow
            // 
            this.CHK_hudshow.Checked = true;
            this.CHK_hudshow.CheckState = System.Windows.Forms.CheckState.Checked;
            resources.ApplyResources(this.CHK_hudshow, "CHK_hudshow");
            this.CHK_hudshow.Name = "CHK_hudshow";
            this.CHK_hudshow.UseVisualStyleBackColor = true;
            this.CHK_hudshow.CheckedChanged += new System.EventHandler(this.CHK_hudshow_CheckedChanged);
            // 
            // label92
            // 
            resources.ApplyResources(this.label92, "label92");
            this.label92.Name = "label92";
            // 
            // CMB_videosources
            // 
            this.CMB_videosources.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_videosources.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_videosources, "CMB_videosources");
            this.CMB_videosources.Name = "CMB_videosources";
            this.CMB_videosources.SelectedIndexChanged += new System.EventHandler(this.CMB_videosources_SelectedIndexChanged);
            this.CMB_videosources.MouseClick += new System.Windows.Forms.MouseEventHandler(this.CMB_videosources_MouseClick);
            // 
            // BUT_Joystick
            // 
            resources.ApplyResources(this.BUT_Joystick, "BUT_Joystick");
            this.BUT_Joystick.Name = "BUT_Joystick";
            this.BUT_Joystick.UseVisualStyleBackColor = true;
            this.BUT_Joystick.Click += new System.EventHandler(this.BUT_Joystick_Click);
            // 
            // BUT_videostop
            // 
            resources.ApplyResources(this.BUT_videostop, "BUT_videostop");
            this.BUT_videostop.Name = "BUT_videostop";
            this.BUT_videostop.UseVisualStyleBackColor = true;
            this.BUT_videostop.Click += new System.EventHandler(this.BUT_videostop_Click);
            // 
            // BUT_videostart
            // 
            resources.ApplyResources(this.BUT_videostart, "BUT_videostart");
            this.BUT_videostart.Name = "BUT_videostart";
            this.BUT_videostart.UseVisualStyleBackColor = true;
            this.BUT_videostart.Click += new System.EventHandler(this.BUT_videostart_Click);
            // 
            // TabSetup
            // 
            resources.ApplyResources(this.TabSetup, "TabSetup");
            this.TabSetup.Name = "TabSetup";
            this.TabSetup.UseVisualStyleBackColor = true;
            // 
            // label109
            // 
            resources.ApplyResources(this.label109, "label109");
            this.label109.Name = "label109";
            // 
            // BUT_rerequestparams
            // 
            resources.ApplyResources(this.BUT_rerequestparams, "BUT_rerequestparams");
            this.BUT_rerequestparams.Name = "BUT_rerequestparams";
            this.toolTip1.SetToolTip(this.BUT_rerequestparams, resources.GetString("BUT_rerequestparams.ToolTip"));
            this.BUT_rerequestparams.UseVisualStyleBackColor = true;
            this.BUT_rerequestparams.Click += new System.EventHandler(this.BUT_rerequestparams_Click);
            // 
            // BUT_writePIDS
            // 
            resources.ApplyResources(this.BUT_writePIDS, "BUT_writePIDS");
            this.BUT_writePIDS.Name = "BUT_writePIDS";
            this.toolTip1.SetToolTip(this.BUT_writePIDS, resources.GetString("BUT_writePIDS.ToolTip"));
            this.BUT_writePIDS.UseVisualStyleBackColor = true;
            this.BUT_writePIDS.Click += new System.EventHandler(this.BUT_writePIDS_Click);
            // 
            // BUT_save
            // 
            resources.ApplyResources(this.BUT_save, "BUT_save");
            this.BUT_save.Name = "BUT_save";
            this.toolTip1.SetToolTip(this.BUT_save, resources.GetString("BUT_save.ToolTip"));
            this.BUT_save.UseVisualStyleBackColor = true;
            this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);
            // 
            // BUT_load
            // 
            resources.ApplyResources(this.BUT_load, "BUT_load");
            this.BUT_load.Name = "BUT_load";
            this.toolTip1.SetToolTip(this.BUT_load, resources.GetString("BUT_load.ToolTip"));
            this.BUT_load.UseVisualStyleBackColor = true;
            this.BUT_load.Click += new System.EventHandler(this.BUT_load_Click);
            // 
            // BUT_compare
            // 
            resources.ApplyResources(this.BUT_compare, "BUT_compare");
            this.BUT_compare.Name = "BUT_compare";
            this.BUT_compare.UseVisualStyleBackColor = true;
            this.BUT_compare.Click += new System.EventHandler(this.BUT_compare_Click);
            // 
            // Configuration
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.Control;
            this.Controls.Add(this.BUT_compare);
            this.Controls.Add(this.BUT_rerequestparams);
            this.Controls.Add(this.ConfigTabs);
            this.Controls.Add(this.BUT_writePIDS);
            this.Controls.Add(this.BUT_save);
            this.Controls.Add(this.BUT_load);
            this.Controls.Add(this.Params);
            this.Name = "Configuration";
            this.Load += new System.EventHandler(this.Configuration_Load);
            ((System.ComponentModel.ISupportInitialize)(this.Params)).EndInit();
            this.ConfigTabs.ResumeLayout(false);
            this.TabAPM2.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.THR_FS_VALUE)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_MAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_MIN)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRIM_THROTTLE)).EndInit();
            this.groupBox1.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.ARSPD_RATIO)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSPD_FBW_MAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSPD_FBW_MIN)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRIM_ARSPD_CM)).EndInit();
            this.groupBox2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.LIM_PITCH_MIN)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_PITCH_MAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_ROLL_CD)).EndInit();
            this.groupBox15.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.XTRK_ANGLE_CD)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.XTRK_GAIN_SC)).EndInit();
            this.groupBox16.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.KFF_PTCH2THR)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.KFF_RDDRMIX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.KFF_PTCHCOMP)).EndInit();
            this.groupBox14.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ENRGY2THR_P)).EndInit();
            this.groupBox13.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ALT2PTCH_P)).EndInit();
            this.groupBox12.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ARSP2PTCH_P)).EndInit();
            this.groupBox11.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.HDNG2RLL_P)).EndInit();
            this.groupBox10.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.YW2SRV_P)).EndInit();
            this.groupBox9.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PTCH2SRV_P)).EndInit();
            this.groupBox8.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_D)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RLL2SRV_P)).EndInit();
            this.TabAC2.ResumeLayout(false);
            this.TabAC2.PerformLayout();
            this.groupBox17.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_PIT_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_PIT_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_PIT_P)).EndInit();
            this.groupBox5.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.THR_RATE_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_RATE_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_RATE_P)).EndInit();
            this.groupBox18.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_RLL_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_RLL_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ACRO_RLL_P)).EndInit();
            this.groupBox4.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.WP_SPEED_MAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NAV_LAT_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NAV_LAT_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NAV_LAT_P)).EndInit();
            this.groupBox6.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.XTRK_GAIN_SC1)).EndInit();
            this.groupBox7.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.THR_ALT_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_ALT_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.THR_ALT_P)).EndInit();
            this.groupBox19.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.HLD_LAT_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.HLD_LAT_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.HLD_LAT_P)).EndInit();
            this.groupBox20.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.STB_YAW_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_YAW_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_YAW_P)).EndInit();
            this.groupBox21.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.STB_PIT_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_PIT_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_PIT_P)).EndInit();
            this.groupBox22.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.STB_RLL_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_RLL_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.STB_RLL_P)).EndInit();
            this.groupBox23.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.RATE_YAW_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_YAW_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_YAW_P)).EndInit();
            this.groupBox24.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.RATE_PIT_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_PIT_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_PIT_P)).EndInit();
            this.groupBox25.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.RATE_RLL_IMAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_RLL_I)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.RATE_RLL_P)).EndInit();
            this.TabPlanner.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.NUM_tracklength)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private MyButton BUT_save;
        private MyButton BUT_load;
        private System.Windows.Forms.DataGridView Params;
        private MyButton BUT_writePIDS;
        private System.Windows.Forms.TabControl ConfigTabs;
        private System.Windows.Forms.TabPage TabAPM2;
        private System.Windows.Forms.TabPage TabAC2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.NumericUpDown THR_FS_VALUE;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown THR_MAX;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.NumericUpDown THR_MIN;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.NumericUpDown TRIM_THROTTLE;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.NumericUpDown ARSPD_RATIO;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.NumericUpDown ARSPD_FBW_MAX;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.NumericUpDown ARSPD_FBW_MIN;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.NumericUpDown TRIM_ARSPD_CM;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.NumericUpDown LIM_PITCH_MIN;
        private System.Windows.Forms.Label label39;
        private System.Windows.Forms.NumericUpDown LIM_PITCH_MAX;
        private System.Windows.Forms.Label label38;
        private System.Windows.Forms.NumericUpDown LIM_ROLL_CD;
        private System.Windows.Forms.Label label37;
        private System.Windows.Forms.GroupBox groupBox15;
        private System.Windows.Forms.NumericUpDown XTRK_ANGLE_CD;
        private System.Windows.Forms.Label label79;
        private System.Windows.Forms.NumericUpDown XTRK_GAIN_SC;
        private System.Windows.Forms.Label label80;
        private System.Windows.Forms.GroupBox groupBox16;
        private System.Windows.Forms.NumericUpDown KFF_PTCH2THR;
        private System.Windows.Forms.Label label83;
        private System.Windows.Forms.NumericUpDown KFF_RDDRMIX;
        private System.Windows.Forms.Label label78;
        private System.Windows.Forms.NumericUpDown KFF_PTCHCOMP;
        private System.Windows.Forms.Label label81;
        private System.Windows.Forms.GroupBox groupBox14;
        private System.Windows.Forms.NumericUpDown ENRGY2THR_IMAX;
        private System.Windows.Forms.Label label73;
        private System.Windows.Forms.NumericUpDown ENRGY2THR_D;
        private System.Windows.Forms.Label label74;
        private System.Windows.Forms.NumericUpDown ENRGY2THR_I;
        private System.Windows.Forms.Label label75;
        private System.Windows.Forms.NumericUpDown ENRGY2THR_P;
        private System.Windows.Forms.Label label76;
        private System.Windows.Forms.GroupBox groupBox13;
        private System.Windows.Forms.NumericUpDown ALT2PTCH_IMAX;
        private System.Windows.Forms.Label label69;
        private System.Windows.Forms.NumericUpDown ALT2PTCH_D;
        private System.Windows.Forms.Label label70;
        private System.Windows.Forms.NumericUpDown ALT2PTCH_I;
        private System.Windows.Forms.Label label71;
        private System.Windows.Forms.NumericUpDown ALT2PTCH_P;
        private System.Windows.Forms.Label label72;
        private System.Windows.Forms.GroupBox groupBox12;
        private System.Windows.Forms.NumericUpDown ARSP2PTCH_IMAX;
        private System.Windows.Forms.Label label65;
        private System.Windows.Forms.NumericUpDown ARSP2PTCH_D;
        private System.Windows.Forms.Label label66;
        private System.Windows.Forms.NumericUpDown ARSP2PTCH_I;
        private System.Windows.Forms.Label label67;
        private System.Windows.Forms.NumericUpDown ARSP2PTCH_P;
        private System.Windows.Forms.Label label68;
        private System.Windows.Forms.GroupBox groupBox11;
        private System.Windows.Forms.NumericUpDown HDNG2RLL_IMAX;
        private System.Windows.Forms.Label label61;
        private System.Windows.Forms.NumericUpDown HDNG2RLL_D;
        private System.Windows.Forms.Label label62;
        private System.Windows.Forms.NumericUpDown HDNG2RLL_I;
        private System.Windows.Forms.Label label63;
        private System.Windows.Forms.NumericUpDown HDNG2RLL_P;
        private System.Windows.Forms.Label label64;
        private System.Windows.Forms.GroupBox groupBox10;
        private System.Windows.Forms.NumericUpDown YW2SRV_IMAX;
        private System.Windows.Forms.Label label57;
        private System.Windows.Forms.NumericUpDown YW2SRV_D;
        private System.Windows.Forms.Label label58;
        private System.Windows.Forms.NumericUpDown YW2SRV_I;
        private System.Windows.Forms.Label label59;
        private System.Windows.Forms.NumericUpDown YW2SRV_P;
        private System.Windows.Forms.Label label60;
        private System.Windows.Forms.GroupBox groupBox9;
        private System.Windows.Forms.NumericUpDown PTCH2SRV_IMAX;
        private System.Windows.Forms.Label label53;
        private System.Windows.Forms.NumericUpDown PTCH2SRV_D;
        private System.Windows.Forms.Label label54;
        private System.Windows.Forms.NumericUpDown PTCH2SRV_I;
        private System.Windows.Forms.Label label55;
        private System.Windows.Forms.NumericUpDown PTCH2SRV_P;
        private System.Windows.Forms.Label label56;
        private System.Windows.Forms.GroupBox groupBox8;
        private System.Windows.Forms.NumericUpDown RLL2SRV_IMAX;
        private System.Windows.Forms.Label label49;
        private System.Windows.Forms.NumericUpDown RLL2SRV_D;
        private System.Windows.Forms.Label label50;
        private System.Windows.Forms.NumericUpDown RLL2SRV_I;
        private System.Windows.Forms.Label label51;
        private System.Windows.Forms.NumericUpDown RLL2SRV_P;
        private System.Windows.Forms.Label label52;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.NumericUpDown NAV_LAT_IMAX;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.NumericUpDown NAV_LAT_I;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.NumericUpDown NAV_LAT_P;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.NumericUpDown XTRK_GAIN_SC1;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.NumericUpDown THR_ALT_IMAX;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.NumericUpDown THR_ALT_I;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.NumericUpDown THR_ALT_P;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.GroupBox groupBox19;
        private System.Windows.Forms.NumericUpDown HLD_LAT_IMAX;
        private System.Windows.Forms.Label label28;
        private System.Windows.Forms.NumericUpDown HLD_LAT_I;
        private System.Windows.Forms.Label label30;
        private System.Windows.Forms.NumericUpDown HLD_LAT_P;
        private System.Windows.Forms.Label label31;
        private System.Windows.Forms.GroupBox groupBox20;
        private System.Windows.Forms.NumericUpDown STB_YAW_IMAX;
        private System.Windows.Forms.Label label32;
        private System.Windows.Forms.NumericUpDown STB_YAW_I;
        private System.Windows.Forms.Label label34;
        private System.Windows.Forms.NumericUpDown STB_YAW_P;
        private System.Windows.Forms.Label label35;
        private System.Windows.Forms.GroupBox groupBox21;
        private System.Windows.Forms.NumericUpDown STB_PIT_IMAX;
        private System.Windows.Forms.Label label36;
        private System.Windows.Forms.NumericUpDown STB_PIT_I;
        private System.Windows.Forms.Label label41;
        private System.Windows.Forms.NumericUpDown STB_PIT_P;
        private System.Windows.Forms.Label label42;
        private System.Windows.Forms.GroupBox groupBox22;
        private System.Windows.Forms.NumericUpDown STB_RLL_IMAX;
        private System.Windows.Forms.Label label43;
        private System.Windows.Forms.NumericUpDown STB_RLL_I;
        private System.Windows.Forms.Label label45;
        private System.Windows.Forms.NumericUpDown STB_RLL_P;
        private System.Windows.Forms.Label label46;
        private System.Windows.Forms.GroupBox groupBox23;
        private System.Windows.Forms.NumericUpDown RATE_YAW_IMAX;
        private System.Windows.Forms.Label label47;
        private System.Windows.Forms.NumericUpDown RATE_YAW_I;
        private System.Windows.Forms.Label label77;
        private System.Windows.Forms.NumericUpDown RATE_YAW_P;
        private System.Windows.Forms.Label label82;
        private System.Windows.Forms.GroupBox groupBox24;
        private System.Windows.Forms.NumericUpDown RATE_PIT_IMAX;
        private System.Windows.Forms.Label label84;
        private System.Windows.Forms.NumericUpDown RATE_PIT_I;
        private System.Windows.Forms.Label label86;
        private System.Windows.Forms.NumericUpDown RATE_PIT_P;
        private System.Windows.Forms.Label label87;
        private System.Windows.Forms.GroupBox groupBox25;
        private System.Windows.Forms.NumericUpDown RATE_RLL_IMAX;
        private System.Windows.Forms.Label label88;
        private System.Windows.Forms.NumericUpDown RATE_RLL_I;
        private System.Windows.Forms.Label label90;
        private System.Windows.Forms.NumericUpDown RATE_RLL_P;
        private System.Windows.Forms.Label label91;
        private System.Windows.Forms.TabPage TabPlanner;
        private System.Windows.Forms.Label label92;
        private System.Windows.Forms.ComboBox CMB_videosources;
        private MyButton BUT_videostop;
        private MyButton BUT_videostart;
        private System.Windows.Forms.CheckBox CHK_hudshow;
        private System.Windows.Forms.CheckBox CHK_enablespeech;
        private System.Windows.Forms.ComboBox CMB_language;
        private System.Windows.Forms.Label label93;
        private System.Windows.Forms.Label label94;
        private System.Windows.Forms.ComboBox CMB_osdcolor;
        private System.Windows.Forms.CheckBox CHK_speechwaypoint;
        private System.Windows.Forms.CheckBox CHK_speechmode;
        private System.Windows.Forms.CheckBox CHK_speechcustom;
        private MyButton BUT_rerequestparams;
        private System.Windows.Forms.CheckBox CHK_speechbattery;
        private MyButton BUT_Joystick;
        private System.Windows.Forms.Label label96;
        private System.Windows.Forms.Label label95;
        private System.Windows.Forms.ComboBox CMB_speedunits;
        private System.Windows.Forms.ComboBox CMB_distunits;
        private System.Windows.Forms.Label label98;
        private System.Windows.Forms.Label label97;
        private System.Windows.Forms.Label label99;
        private System.Windows.Forms.TabPage TabSetup;
        private System.Windows.Forms.Label label101;
        private System.Windows.Forms.ComboBox CMB_ratestatus;
        private System.Windows.Forms.ComboBox CMB_rateposition;
        private System.Windows.Forms.ComboBox CMB_rateattitude;
        private System.Windows.Forms.Label label104;
        private System.Windows.Forms.Label label103;
        private System.Windows.Forms.Label label102;
        private System.Windows.Forms.Label label107;
        private System.Windows.Forms.ComboBox CMB_raterc;
        private System.Windows.Forms.CheckBox CHK_mavdebug;
        private System.Windows.Forms.CheckBox CHK_resetapmonconnect;
        private System.Windows.Forms.Label label108;
        private System.Windows.Forms.CheckBox CHK_lockrollpitch;
        private System.Windows.Forms.NumericUpDown WP_SPEED_MAX;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.CheckBox CHK_speechaltwarning;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.NumericUpDown NUM_tracklength;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.CheckBox CHK_loadwponconnect;
        private System.Windows.Forms.DataGridViewTextBoxColumn Command;
        private System.Windows.Forms.DataGridViewTextBoxColumn Value;
        private System.Windows.Forms.DataGridViewTextBoxColumn Default;
        private System.Windows.Forms.DataGridViewTextBoxColumn mavScale;
        private System.Windows.Forms.DataGridViewTextBoxColumn RawValue;
        private MyButton BUT_compare;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.CheckBox CHK_GDIPlus;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.NumericUpDown THR_RATE_IMAX;
        private System.Windows.Forms.NumericUpDown THR_RATE_I;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.NumericUpDown THR_RATE_P;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.ComboBox CMB_videoresolutions;
        private System.Windows.Forms.Label label109;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.GroupBox groupBox17;
        private System.Windows.Forms.NumericUpDown ACRO_PIT_IMAX;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.NumericUpDown ACRO_PIT_I;
        private System.Windows.Forms.Label label29;
        private System.Windows.Forms.NumericUpDown ACRO_PIT_P;
        private System.Windows.Forms.Label label33;
        private System.Windows.Forms.GroupBox groupBox18;
        private System.Windows.Forms.NumericUpDown ACRO_RLL_IMAX;
        private System.Windows.Forms.Label label40;
        private System.Windows.Forms.NumericUpDown ACRO_RLL_I;
        private System.Windows.Forms.Label label44;
        private System.Windows.Forms.NumericUpDown ACRO_RLL_P;
        private System.Windows.Forms.Label label48;
        private MyLabel myLabel1;
        private System.Windows.Forms.ComboBox CH7_OPT;
    }
}
