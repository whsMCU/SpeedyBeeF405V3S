﻿namespace SpeedyBeeF405V3S_GUI
{
    partial class Form1
    {
        /// <summary>
        /// 필수 디자이너 변수입니다.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 사용 중인 모든 리소스를 정리합니다.
        /// </summary>
        /// <param name="disposing">관리되는 리소스를 삭제해야 하면 true이고, 그렇지 않으면 false입니다.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        public bool create_waypoint_list;
        public int time_counter, receive_buffer_counter, receive_start_detect;
        public byte[] receive_buffer = new byte[50];
        public byte[] send_buffer = new byte[20];
        public byte receive_byte_previous, webbrouwser_1_ready, webbrouwser_2_ready, home_gps_set;

        public byte check_byte, temp_byte, start;
        public byte first_receive, received_data, webbrouwser_active;

        public long milliseconds, last_receive;
        public double ground_distance, los_distance;

        public int zoom = 17;
        public short temperature;
        public int error, flight_mode, roll_angle, pitch_angle;
        public int altitude_meters,
            max_altitude_meters,
            takeoff_throttle,
            actual_compass_heading,
            heading_lock,
            number_used_sats,
            fix_type,
            l_lat_gps,
            l_lon_gps,
            home_lat_gps,
            home_lon_gps;

        public int click_lat, click_lon;
        public int waypoint_list_counter, send_telemetry_data_counter, waypoint_send_step;
        public int[] waypoint_click_lat = new int[10];
        public int[] waypoint_click_lon = new int[10];
        public int flight_timer_seconds;
        public int program_timer_seconds;

        public int new_telemetry_data_to_send;
        public int battery_bar_level;


        public float battery_voltage,
            adjustable_setting_1,
            adjustable_setting_2,
            adjustable_setting_3;

        #region Windows Form 디자이너에서 생성한 코드

        /// <summary>
        /// 디자이너 지원에 필요한 메서드입니다. 
        /// 이 메서드의 내용을 코드 편집기로 수정하지 마세요.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.OpenClose = new System.Windows.Forms.Button();
            this.comboBox_port = new System.Windows.Forms.ComboBox();
            this.serialPort = new System.IO.Ports.SerialPort(this.components);
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.lb_rc_throttle = new System.Windows.Forms.Label();
            this.lb_roll = new System.Windows.Forms.Label();
            this.label44 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.lb_rc_yaw = new System.Windows.Forms.Label();
            this.lb_pitch = new System.Windows.Forms.Label();
            this.label42 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.lb_rc_pitch = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.lb_rc_roll = new System.Windows.Forms.Label();
            this.label27 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.button7 = new System.Windows.Forms.Button();
            this.button6 = new System.Windows.Forms.Button();
            this.bt_reset_waypoints = new System.Windows.Forms.Button();
            this.label25 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.lb_althold = new System.Windows.Forms.Label();
            this.label91 = new System.Windows.Forms.Label();
            this.label75 = new System.Windows.Forms.Label();
            this.label82 = new System.Windows.Forms.Label();
            this.lb_rangefinder = new System.Windows.Forms.Label();
            this.lb_gyro_Z = new System.Windows.Forms.Label();
            this.lb_gyro_Y = new System.Windows.Forms.Label();
            this.lb_gyro_X = new System.Windows.Forms.Label();
            this.label60 = new System.Windows.Forms.Label();
            this.label59 = new System.Windows.Forms.Label();
            this.label58 = new System.Windows.Forms.Label();
            this.label37 = new System.Windows.Forms.Label();
            this.label38 = new System.Windows.Forms.Label();
            this.label22 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label20 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.lb_altitude = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.lb_heading = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.lb_long = new System.Windows.Forms.Label();
            this.lb_lat = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label24 = new System.Windows.Forms.Label();
            this.panel1 = new System.Windows.Forms.Panel();
            this.panel4 = new System.Windows.Forms.Panel();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.label77 = new System.Windows.Forms.Label();
            this.tb_rx_error = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.lb_fail = new System.Windows.Forms.Label();
            this.panel3 = new System.Windows.Forms.Panel();
            this.label81 = new System.Windows.Forms.Label();
            this.lb_fc_load = new System.Windows.Forms.Label();
            this.label55 = new System.Windows.Forms.Label();
            this.label36 = new System.Windows.Forms.Label();
            this.textBox11 = new System.Windows.Forms.TextBox();
            this.panel2 = new System.Windows.Forms.Panel();
            this.label92 = new System.Windows.Forms.Label();
            this.lb_bat_mAh = new System.Windows.Forms.Label();
            this.label90 = new System.Windows.Forms.Label();
            this.lb_bat_A = new System.Windows.Forms.Label();
            this.lb_gps_fix = new System.Windows.Forms.Label();
            this.label56 = new System.Windows.Forms.Label();
            this.label79 = new System.Windows.Forms.Label();
            this.label57 = new System.Windows.Forms.Label();
            this.lb_sat_num = new System.Windows.Forms.Label();
            this.lb_armed = new System.Windows.Forms.Label();
            this.pictureBox6 = new System.Windows.Forms.PictureBox();
            this.pictureBox5 = new System.Windows.Forms.PictureBox();
            this.pictureBox4 = new System.Windows.Forms.PictureBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.lb_bat_V = new System.Windows.Forms.Label();
            this.panel7 = new System.Windows.Forms.Panel();
            this.panel6 = new System.Windows.Forms.Panel();
            this.panel5 = new System.Windows.Forms.Panel();
            this.textBox10 = new System.Windows.Forms.TextBox();
            this.bt_zoom_p = new System.Windows.Forms.Button();
            this.bt_zoom_m = new System.Windows.Forms.Button();
            this.label10 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.lb_debug0 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.tb_FC_ALT_Range_D = new System.Windows.Forms.TextBox();
            this.tb_FC_ALT_Range_I = new System.Windows.Forms.TextBox();
            this.tb_FC_ALT_Range_P = new System.Windows.Forms.TextBox();
            this.label96 = new System.Windows.Forms.Label();
            this.label97 = new System.Windows.Forms.Label();
            this.tb_ALT_Range_D = new System.Windows.Forms.TextBox();
            this.tb_ALT_Range_I = new System.Windows.Forms.TextBox();
            this.tb_ALT_Range_P = new System.Windows.Forms.TextBox();
            this.tb_FC_POS_Opflow_D = new System.Windows.Forms.TextBox();
            this.tb_FC_POS_Opflow_I = new System.Windows.Forms.TextBox();
            this.tb_FC_POS_Opflow_P = new System.Windows.Forms.TextBox();
            this.label93 = new System.Windows.Forms.Label();
            this.label94 = new System.Windows.Forms.Label();
            this.label95 = new System.Windows.Forms.Label();
            this.tb_POS_Opflow_D = new System.Windows.Forms.TextBox();
            this.tb_POS_Opflow_I = new System.Windows.Forms.TextBox();
            this.tb_POS_Opflow_P = new System.Windows.Forms.TextBox();
            this.tb_FC_ALT_D = new System.Windows.Forms.TextBox();
            this.tb_FC_ALT_I = new System.Windows.Forms.TextBox();
            this.tb_FC_ALT_P = new System.Windows.Forms.TextBox();
            this.label89 = new System.Windows.Forms.Label();
            this.label88 = new System.Windows.Forms.Label();
            this.label87 = new System.Windows.Forms.Label();
            this.tb_ALT_D = new System.Windows.Forms.TextBox();
            this.tb_ALT_I = new System.Windows.Forms.TextBox();
            this.tb_ALT_P = new System.Windows.Forms.TextBox();
            this.cB_RP_Coupling = new System.Windows.Forms.CheckBox();
            this.bt_pid_save = new System.Windows.Forms.Button();
            this.bt_pid_copy = new System.Windows.Forms.Button();
            this.tb_FC_Y_R_D = new System.Windows.Forms.TextBox();
            this.tb_FC_Y_R_I = new System.Windows.Forms.TextBox();
            this.tb_FC_Y_R_P = new System.Windows.Forms.TextBox();
            this.label46 = new System.Windows.Forms.Label();
            this.label47 = new System.Windows.Forms.Label();
            this.tb_FC_Y_A_D = new System.Windows.Forms.TextBox();
            this.tb_FC_Y_A_I = new System.Windows.Forms.TextBox();
            this.tb_FC_Y_A_P = new System.Windows.Forms.TextBox();
            this.tb_FC_P_O_D = new System.Windows.Forms.TextBox();
            this.tb_FC_P_O_I = new System.Windows.Forms.TextBox();
            this.tb_FC_P_O_P = new System.Windows.Forms.TextBox();
            this.label48 = new System.Windows.Forms.Label();
            this.label49 = new System.Windows.Forms.Label();
            this.tb_FC_P_I_D = new System.Windows.Forms.TextBox();
            this.tb_FC_P_I_I = new System.Windows.Forms.TextBox();
            this.tb_FC_P_I_P = new System.Windows.Forms.TextBox();
            this.tb_FC_R_O_D = new System.Windows.Forms.TextBox();
            this.tb_FC_R_O_I = new System.Windows.Forms.TextBox();
            this.tb_FC_R_O_P = new System.Windows.Forms.TextBox();
            this.label50 = new System.Windows.Forms.Label();
            this.label51 = new System.Windows.Forms.Label();
            this.label52 = new System.Windows.Forms.Label();
            this.tb_FC_R_I_D = new System.Windows.Forms.TextBox();
            this.label53 = new System.Windows.Forms.Label();
            this.tb_FC_R_I_I = new System.Windows.Forms.TextBox();
            this.label54 = new System.Windows.Forms.Label();
            this.tb_FC_R_I_P = new System.Windows.Forms.TextBox();
            this.tb_Y_R_D = new System.Windows.Forms.TextBox();
            this.tb_Y_R_I = new System.Windows.Forms.TextBox();
            this.tb_Y_R_P = new System.Windows.Forms.TextBox();
            this.label43 = new System.Windows.Forms.Label();
            this.label45 = new System.Windows.Forms.Label();
            this.tb_Y_A_D = new System.Windows.Forms.TextBox();
            this.tb_Y_A_I = new System.Windows.Forms.TextBox();
            this.tb_Y_A_P = new System.Windows.Forms.TextBox();
            this.tb_P_O_D = new System.Windows.Forms.TextBox();
            this.tb_P_O_I = new System.Windows.Forms.TextBox();
            this.tb_P_O_P = new System.Windows.Forms.TextBox();
            this.label34 = new System.Windows.Forms.Label();
            this.label35 = new System.Windows.Forms.Label();
            this.tb_P_I_D = new System.Windows.Forms.TextBox();
            this.tb_P_I_I = new System.Windows.Forms.TextBox();
            this.tb_P_I_P = new System.Windows.Forms.TextBox();
            this.tb_R_O_D = new System.Windows.Forms.TextBox();
            this.tb_R_O_I = new System.Windows.Forms.TextBox();
            this.tb_R_O_P = new System.Windows.Forms.TextBox();
            this.label33 = new System.Windows.Forms.Label();
            this.label32 = new System.Windows.Forms.Label();
            this.label31 = new System.Windows.Forms.Label();
            this.label30 = new System.Windows.Forms.Label();
            this.label29 = new System.Windows.Forms.Label();
            this.bt_pid_recive = new System.Windows.Forms.Button();
            this.bt_pid_send = new System.Windows.Forms.Button();
            this.label39 = new System.Windows.Forms.Label();
            this.tb_R_I_D = new System.Windows.Forms.TextBox();
            this.label40 = new System.Windows.Forms.Label();
            this.tb_R_I_I = new System.Windows.Forms.TextBox();
            this.label41 = new System.Windows.Forms.Label();
            this.tb_R_I_P = new System.Windows.Forms.TextBox();
            this.rx_timer_blink = new System.Windows.Forms.Timer(this.components);
            this.timer_status = new System.Windows.Forms.Timer(this.components);
            this.flight_timer = new System.Windows.Forms.Timer(this.components);
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.Gauge_RR = new LiveCharts.WinForms.SolidGauge();
            this.Gauge_RF = new LiveCharts.WinForms.SolidGauge();
            this.Gauge_LR = new LiveCharts.WinForms.SolidGauge();
            this.Gauge_LF = new LiveCharts.WinForms.SolidGauge();
            this.lb_motor3 = new System.Windows.Forms.Label();
            this.lb_motor2 = new System.Windows.Forms.Label();
            this.lb_motor1 = new System.Windows.Forms.Label();
            this.lb_motor0 = new System.Windows.Forms.Label();
            this.label28 = new System.Windows.Forms.Label();
            this.label23 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.label98 = new System.Windows.Forms.Label();
            this.lb_debug7 = new System.Windows.Forms.Label();
            this.label100 = new System.Windows.Forms.Label();
            this.lb_debug6 = new System.Windows.Forms.Label();
            this.label102 = new System.Windows.Forms.Label();
            this.lb_debug5 = new System.Windows.Forms.Label();
            this.label104 = new System.Windows.Forms.Label();
            this.lb_debug4 = new System.Windows.Forms.Label();
            this.label21 = new System.Windows.Forms.Label();
            this.lb_debug3 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.lb_debug2 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.lb_debug1 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.contextMenuStrip1 = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.tabPage1 = new System.Windows.Forms.TabPage();
            this.groupBox10 = new System.Windows.Forms.GroupBox();
            this.headingIndicatorInstrumentControl1 = new AvionicsInstrumentControlDemo.HeadingIndicatorInstrumentControl();
            this.attitudeIndicatorInstrumentControl1 = new AvionicsInstrumentControlDemo.AttitudeIndicatorInstrumentControl();
            this.tabPage2 = new System.Windows.Forms.TabPage();
            this.label62 = new System.Windows.Forms.Label();
            this.lb_route_distance = new System.Windows.Forms.Label();
            this.label61 = new System.Windows.Forms.Label();
            this.gMapControl1 = new GMap.NET.WindowsForms.GMapControl();
            this.tabPage3 = new System.Windows.Forms.TabPage();
            this.comboBox_Debug = new System.Windows.Forms.ComboBox();
            this.rb_alt_range_setpoint = new System.Windows.Forms.RadioButton();
            this.rb_alt_setpoint = new System.Windows.Forms.RadioButton();
            this.lb_PID_Test_Target_Time = new System.Windows.Forms.Label();
            this.lb_PID_Test_Progress_Time = new System.Windows.Forms.Label();
            this.label86 = new System.Windows.Forms.Label();
            this.label85 = new System.Windows.Forms.Label();
            this.bt_open_folder = new System.Windows.Forms.Button();
            this.lb_PID_Test_Progress = new System.Windows.Forms.Label();
            this.lb_PID_Test_Status = new System.Windows.Forms.Label();
            this.bt_start_pid_test = new System.Windows.Forms.Button();
            this.label84 = new System.Windows.Forms.Label();
            this.label83 = new System.Windows.Forms.Label();
            this.label80 = new System.Windows.Forms.Label();
            this.tb_PID_ms = new System.Windows.Forms.TextBox();
            this.tb_PID_Deg = new System.Windows.Forms.TextBox();
            this.tb_PID_Throttle = new System.Windows.Forms.TextBox();
            this.panel8 = new System.Windows.Forms.Panel();
            this.cb_record = new System.Windows.Forms.CheckBox();
            this.rb_debug = new System.Windows.Forms.RadioButton();
            this.rb_motor = new System.Windows.Forms.RadioButton();
            this.rb_gyro = new System.Windows.Forms.RadioButton();
            this.rb_altitude = new System.Windows.Forms.RadioButton();
            this.cb_autoscale = new System.Windows.Forms.CheckBox();
            this.rb_none = new System.Windows.Forms.RadioButton();
            this.rb_yaw_setpoint = new System.Windows.Forms.RadioButton();
            this.rb_pitch_setpoint = new System.Windows.Forms.RadioButton();
            this.rb_roll_setpoint = new System.Windows.Forms.RadioButton();
            this.rb_roll_pitch = new System.Windows.Forms.RadioButton();
            this.rb_yaw = new System.Windows.Forms.RadioButton();
            this.rb_pitch = new System.Windows.Forms.RadioButton();
            this.rb_roll = new System.Windows.Forms.RadioButton();
            this.zedGraphControl1 = new ZedGraph.ZedGraphControl();
            this.tabPage4 = new System.Windows.Forms.TabPage();
            this.groupBox9 = new System.Windows.Forms.GroupBox();
            this.label69 = new System.Windows.Forms.Label();
            this.label76 = new System.Windows.Forms.Label();
            this.lb_bodyrate_Y = new System.Windows.Forms.Label();
            this.lb_bodyrate_X = new System.Windows.Forms.Label();
            this.label72 = new System.Windows.Forms.Label();
            this.label73 = new System.Windows.Forms.Label();
            this.lb_flowrate_Y = new System.Windows.Forms.Label();
            this.lb_flowrate_X = new System.Windows.Forms.Label();
            this.groupBox8 = new System.Windows.Forms.GroupBox();
            this.label67 = new System.Windows.Forms.Label();
            this.label68 = new System.Windows.Forms.Label();
            this.lb_mag_Z = new System.Windows.Forms.Label();
            this.lb_mag_Y = new System.Windows.Forms.Label();
            this.lb_mag_X = new System.Windows.Forms.Label();
            this.label74 = new System.Windows.Forms.Label();
            this.lb_magcal_remain_time = new System.Windows.Forms.Label();
            this.label66 = new System.Windows.Forms.Label();
            this.label71 = new System.Windows.Forms.Label();
            this.label70 = new System.Windows.Forms.Label();
            this.lb_magZero_Z = new System.Windows.Forms.Label();
            this.lb_magZero_Y = new System.Windows.Forms.Label();
            this.lb_magZero_X = new System.Windows.Forms.Label();
            this.label78 = new System.Windows.Forms.Label();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.label65 = new System.Windows.Forms.Label();
            this.label63 = new System.Windows.Forms.Label();
            this.label64 = new System.Windows.Forms.Label();
            this.lb_accTrim_Y = new System.Windows.Forms.Label();
            this.lb_accTrim_Z = new System.Windows.Forms.Label();
            this.lb_accTrim_X = new System.Windows.Forms.Label();
            this.bt_mag_cal = new System.Windows.Forms.Button();
            this.bt_acc_cal = new System.Windows.Forms.Button();
            this.tabPage5 = new System.Windows.Forms.TabPage();
            this.bt_ReScan = new System.Windows.Forms.Button();
            this.textBox5 = new System.Windows.Forms.TextBox();
            this.tb_msp_error = new System.Windows.Forms.TextBox();
            this.label99 = new System.Windows.Forms.Label();
            this.label101 = new System.Windows.Forms.Label();
            this.lb_gps_time = new System.Windows.Forms.Label();
            this.groupBox3.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.panel4.SuspendLayout();
            this.panel3.SuspendLayout();
            this.panel2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox6)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox5)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.panel7.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.tabControl1.SuspendLayout();
            this.tabPage1.SuspendLayout();
            this.groupBox10.SuspendLayout();
            this.tabPage2.SuspendLayout();
            this.tabPage3.SuspendLayout();
            this.tabPage4.SuspendLayout();
            this.groupBox9.SuspendLayout();
            this.groupBox8.SuspendLayout();
            this.groupBox7.SuspendLayout();
            this.SuspendLayout();
            // 
            // OpenClose
            // 
            this.OpenClose.Location = new System.Drawing.Point(140, 12);
            this.OpenClose.Name = "OpenClose";
            this.OpenClose.Size = new System.Drawing.Size(156, 23);
            this.OpenClose.TabIndex = 33;
            this.OpenClose.Text = "Open";
            this.OpenClose.UseVisualStyleBackColor = true;
            this.OpenClose.Click += new System.EventHandler(this.OpenClose_Click);
            // 
            // comboBox_port
            // 
            this.comboBox_port.FormattingEnabled = true;
            this.comboBox_port.Location = new System.Drawing.Point(13, 12);
            this.comboBox_port.Name = "comboBox_port";
            this.comboBox_port.Size = new System.Drawing.Size(121, 20);
            this.comboBox_port.TabIndex = 31;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.lb_rc_throttle);
            this.groupBox3.Controls.Add(this.lb_roll);
            this.groupBox3.Controls.Add(this.label44);
            this.groupBox3.Controls.Add(this.label11);
            this.groupBox3.Controls.Add(this.lb_rc_yaw);
            this.groupBox3.Controls.Add(this.lb_pitch);
            this.groupBox3.Controls.Add(this.label42);
            this.groupBox3.Controls.Add(this.label2);
            this.groupBox3.Controls.Add(this.lb_rc_pitch);
            this.groupBox3.Controls.Add(this.label4);
            this.groupBox3.Controls.Add(this.label18);
            this.groupBox3.Controls.Add(this.lb_rc_roll);
            this.groupBox3.Font = new System.Drawing.Font("Arial", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBox3.Location = new System.Drawing.Point(13, 293);
            this.groupBox3.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Padding = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.groupBox3.Size = new System.Drawing.Size(327, 98);
            this.groupBox3.TabIndex = 68;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Attitude";
            // 
            // lb_rc_throttle
            // 
            this.lb_rc_throttle.AutoSize = true;
            this.lb_rc_throttle.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_rc_throttle.Location = new System.Drawing.Point(105, 74);
            this.lb_rc_throttle.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_rc_throttle.Name = "lb_rc_throttle";
            this.lb_rc_throttle.Size = new System.Drawing.Size(16, 21);
            this.lb_rc_throttle.TabIndex = 59;
            this.lb_rc_throttle.Text = "-";
            // 
            // lb_roll
            // 
            this.lb_roll.AutoSize = true;
            this.lb_roll.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_roll.Location = new System.Drawing.Point(105, 27);
            this.lb_roll.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_roll.Name = "lb_roll";
            this.lb_roll.Size = new System.Drawing.Size(16, 21);
            this.lb_roll.TabIndex = 53;
            this.lb_roll.Text = "-";
            // 
            // label44
            // 
            this.label44.AutoSize = true;
            this.label44.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label44.Location = new System.Drawing.Point(12, 74);
            this.label44.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label44.Name = "label44";
            this.label44.Size = new System.Drawing.Size(69, 21);
            this.label44.TabIndex = 58;
            this.label44.Text = "RC_Thro";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label11.Location = new System.Drawing.Point(10, 27);
            this.label11.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(79, 21);
            this.label11.TabIndex = 52;
            this.label11.Text = "Roll angle";
            // 
            // lb_rc_yaw
            // 
            this.lb_rc_yaw.AutoSize = true;
            this.lb_rc_yaw.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_rc_yaw.Location = new System.Drawing.Point(283, 74);
            this.lb_rc_yaw.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_rc_yaw.Name = "lb_rc_yaw";
            this.lb_rc_yaw.Size = new System.Drawing.Size(16, 21);
            this.lb_rc_yaw.TabIndex = 57;
            this.lb_rc_yaw.Text = "-";
            // 
            // lb_pitch
            // 
            this.lb_pitch.AutoSize = true;
            this.lb_pitch.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_pitch.Location = new System.Drawing.Point(283, 27);
            this.lb_pitch.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_pitch.Name = "lb_pitch";
            this.lb_pitch.Size = new System.Drawing.Size(16, 21);
            this.lb_pitch.TabIndex = 51;
            this.lb_pitch.Text = "-";
            // 
            // label42
            // 
            this.label42.AutoSize = true;
            this.label42.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label42.Location = new System.Drawing.Point(164, 74);
            this.label42.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label42.Name = "label42";
            this.label42.Size = new System.Drawing.Size(69, 21);
            this.label42.TabIndex = 56;
            this.label42.Text = "RC_YAW";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(164, 27);
            this.label2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(86, 21);
            this.label2.TabIndex = 50;
            this.label2.Text = "Pitch angle";
            // 
            // lb_rc_pitch
            // 
            this.lb_rc_pitch.AutoSize = true;
            this.lb_rc_pitch.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_rc_pitch.Location = new System.Drawing.Point(283, 51);
            this.lb_rc_pitch.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_rc_pitch.Name = "lb_rc_pitch";
            this.lb_rc_pitch.Size = new System.Drawing.Size(16, 21);
            this.lb_rc_pitch.TabIndex = 55;
            this.lb_rc_pitch.Text = "-";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.Location = new System.Drawing.Point(10, 51);
            this.label4.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(75, 21);
            this.label4.TabIndex = 52;
            this.label4.Text = "RC_ROLL";
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label18.Location = new System.Drawing.Point(164, 51);
            this.label18.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(78, 21);
            this.label18.TabIndex = 54;
            this.label18.Text = "RC_PITCH";
            // 
            // lb_rc_roll
            // 
            this.lb_rc_roll.AutoSize = true;
            this.lb_rc_roll.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_rc_roll.Location = new System.Drawing.Point(105, 51);
            this.lb_rc_roll.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_rc_roll.Name = "lb_rc_roll";
            this.lb_rc_roll.Size = new System.Drawing.Size(16, 21);
            this.lb_rc_roll.TabIndex = 53;
            this.lb_rc_roll.Text = "-";
            // 
            // label27
            // 
            this.label27.AutoSize = true;
            this.label27.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label27.ForeColor = System.Drawing.Color.Red;
            this.label27.Location = new System.Drawing.Point(37, 56);
            this.label27.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(225, 21);
            this.label27.TabIndex = 67;
            this.label27.Text = "Waiting for telemetry signal";
            this.label27.Visible = false;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.button7);
            this.groupBox2.Controls.Add(this.button6);
            this.groupBox2.Controls.Add(this.bt_reset_waypoints);
            this.groupBox2.Controls.Add(this.label25);
            this.groupBox2.Controls.Add(this.label26);
            this.groupBox2.Font = new System.Drawing.Font("Arial", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBox2.Location = new System.Drawing.Point(13, 395);
            this.groupBox2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Padding = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.groupBox2.Size = new System.Drawing.Size(327, 104);
            this.groupBox2.TabIndex = 66;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Waypoints";
            // 
            // button7
            // 
            this.button7.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button7.Location = new System.Drawing.Point(222, 65);
            this.button7.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.button7.Name = "button7";
            this.button7.Size = new System.Drawing.Size(97, 21);
            this.button7.TabIndex = 57;
            this.button7.Text = "Fly list";
            this.button7.UseVisualStyleBackColor = true;
            // 
            // button6
            // 
            this.button6.Enabled = false;
            this.button6.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button6.Location = new System.Drawing.Point(118, 65);
            this.button6.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.button6.Name = "button6";
            this.button6.Size = new System.Drawing.Size(97, 21);
            this.button6.TabIndex = 56;
            this.button6.Text = "Create list";
            this.button6.UseVisualStyleBackColor = true;
            // 
            // bt_reset_waypoints
            // 
            this.bt_reset_waypoints.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_reset_waypoints.Location = new System.Drawing.Point(14, 65);
            this.bt_reset_waypoints.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.bt_reset_waypoints.Name = "bt_reset_waypoints";
            this.bt_reset_waypoints.Size = new System.Drawing.Size(97, 21);
            this.bt_reset_waypoints.TabIndex = 55;
            this.bt_reset_waypoints.Text = "Reset";
            this.bt_reset_waypoints.UseVisualStyleBackColor = true;
            // 
            // label25
            // 
            this.label25.AutoSize = true;
            this.label25.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label25.Location = new System.Drawing.Point(9, 34);
            this.label25.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(90, 21);
            this.label25.TabIndex = 50;
            this.label25.Text = "Send status";
            // 
            // label26
            // 
            this.label26.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label26.Location = new System.Drawing.Point(147, 34);
            this.label26.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(173, 19);
            this.label26.TabIndex = 51;
            this.label26.Text = "-";
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.lb_althold);
            this.groupBox4.Controls.Add(this.label91);
            this.groupBox4.Controls.Add(this.label75);
            this.groupBox4.Controls.Add(this.label82);
            this.groupBox4.Controls.Add(this.lb_rangefinder);
            this.groupBox4.Controls.Add(this.lb_gyro_Z);
            this.groupBox4.Controls.Add(this.lb_gyro_Y);
            this.groupBox4.Controls.Add(this.lb_gyro_X);
            this.groupBox4.Controls.Add(this.label60);
            this.groupBox4.Controls.Add(this.label59);
            this.groupBox4.Controls.Add(this.label58);
            this.groupBox4.Controls.Add(this.label37);
            this.groupBox4.Controls.Add(this.label38);
            this.groupBox4.Controls.Add(this.label22);
            this.groupBox4.Controls.Add(this.label5);
            this.groupBox4.Controls.Add(this.label6);
            this.groupBox4.Controls.Add(this.label20);
            this.groupBox4.Controls.Add(this.label7);
            this.groupBox4.Controls.Add(this.lb_altitude);
            this.groupBox4.Controls.Add(this.label8);
            this.groupBox4.Controls.Add(this.lb_heading);
            this.groupBox4.Controls.Add(this.label9);
            this.groupBox4.Controls.Add(this.lb_long);
            this.groupBox4.Controls.Add(this.lb_lat);
            this.groupBox4.Controls.Add(this.label13);
            this.groupBox4.Font = new System.Drawing.Font("Arial", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBox4.Location = new System.Drawing.Point(13, 86);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.groupBox4.Size = new System.Drawing.Size(327, 201);
            this.groupBox4.TabIndex = 65;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Telemetry passed_data";
            // 
            // lb_althold
            // 
            this.lb_althold.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_althold.Location = new System.Drawing.Point(251, 96);
            this.lb_althold.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_althold.Name = "lb_althold";
            this.lb_althold.Size = new System.Drawing.Size(76, 20);
            this.lb_althold.TabIndex = 98;
            this.lb_althold.Text = "-";
            // 
            // label91
            // 
            this.label91.AutoSize = true;
            this.label91.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label91.Location = new System.Drawing.Point(163, 96);
            this.label91.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label91.Name = "label91";
            this.label91.Size = new System.Drawing.Size(62, 21);
            this.label91.TabIndex = 97;
            this.label91.Text = "AltHold";
            // 
            // label75
            // 
            this.label75.AutoSize = true;
            this.label75.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label75.Location = new System.Drawing.Point(293, 119);
            this.label75.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label75.Name = "label75";
            this.label75.Size = new System.Drawing.Size(31, 21);
            this.label75.TabIndex = 96;
            this.label75.Text = "cm";
            // 
            // label82
            // 
            this.label82.AutoSize = true;
            this.label82.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label82.Location = new System.Drawing.Point(145, 119);
            this.label82.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label82.Name = "label82";
            this.label82.Size = new System.Drawing.Size(98, 21);
            this.label82.TabIndex = 60;
            this.label82.Text = "RangeFinder";
            // 
            // lb_rangefinder
            // 
            this.lb_rangefinder.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_rangefinder.Location = new System.Drawing.Point(251, 120);
            this.lb_rangefinder.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_rangefinder.Name = "lb_rangefinder";
            this.lb_rangefinder.Size = new System.Drawing.Size(45, 20);
            this.lb_rangefinder.TabIndex = 59;
            this.lb_rangefinder.Text = "-";
            // 
            // lb_gyro_Z
            // 
            this.lb_gyro_Z.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_gyro_Z.Location = new System.Drawing.Point(242, 70);
            this.lb_gyro_Z.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_gyro_Z.Name = "lb_gyro_Z";
            this.lb_gyro_Z.Size = new System.Drawing.Size(57, 21);
            this.lb_gyro_Z.TabIndex = 58;
            this.lb_gyro_Z.Text = "-";
            this.lb_gyro_Z.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lb_gyro_Y
            // 
            this.lb_gyro_Y.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_gyro_Y.Location = new System.Drawing.Point(242, 46);
            this.lb_gyro_Y.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_gyro_Y.Name = "lb_gyro_Y";
            this.lb_gyro_Y.Size = new System.Drawing.Size(57, 21);
            this.lb_gyro_Y.TabIndex = 57;
            this.lb_gyro_Y.Text = "-";
            this.lb_gyro_Y.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lb_gyro_X
            // 
            this.lb_gyro_X.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_gyro_X.Location = new System.Drawing.Point(242, 24);
            this.lb_gyro_X.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_gyro_X.Name = "lb_gyro_X";
            this.lb_gyro_X.Size = new System.Drawing.Size(57, 21);
            this.lb_gyro_X.TabIndex = 56;
            this.lb_gyro_X.Text = "-";
            this.lb_gyro_X.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label60
            // 
            this.label60.AutoSize = true;
            this.label60.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label60.Location = new System.Drawing.Point(164, 70);
            this.label60.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label60.Name = "label60";
            this.label60.Size = new System.Drawing.Size(63, 21);
            this.label60.TabIndex = 55;
            this.label60.Text = "Gyro[Z]";
            // 
            // label59
            // 
            this.label59.AutoSize = true;
            this.label59.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label59.Location = new System.Drawing.Point(164, 46);
            this.label59.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label59.Name = "label59";
            this.label59.Size = new System.Drawing.Size(63, 21);
            this.label59.TabIndex = 54;
            this.label59.Text = "Gyro[Y]";
            // 
            // label58
            // 
            this.label58.AutoSize = true;
            this.label58.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label58.Location = new System.Drawing.Point(164, 23);
            this.label58.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label58.Name = "label58";
            this.label58.Size = new System.Drawing.Size(63, 21);
            this.label58.TabIndex = 53;
            this.label58.Text = "Gyro[X]";
            // 
            // label37
            // 
            this.label37.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label37.Location = new System.Drawing.Point(120, 164);
            this.label37.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label37.Name = "label37";
            this.label37.Size = new System.Drawing.Size(45, 20);
            this.label37.TabIndex = 51;
            this.label37.Text = "-";
            // 
            // label38
            // 
            this.label38.AutoSize = true;
            this.label38.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label38.Location = new System.Drawing.Point(9, 164);
            this.label38.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label38.Name = "label38";
            this.label38.Size = new System.Drawing.Size(97, 21);
            this.label38.TabIndex = 50;
            this.label38.Text = "Temperature";
            // 
            // label22
            // 
            this.label22.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label22.Location = new System.Drawing.Point(120, 142);
            this.label22.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(45, 20);
            this.label22.TabIndex = 49;
            this.label22.Text = "-";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.Location = new System.Drawing.Point(9, 24);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(66, 21);
            this.label5.TabIndex = 16;
            this.label5.Text = "Latitude";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.Location = new System.Drawing.Point(9, 48);
            this.label6.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(80, 21);
            this.label6.TabIndex = 18;
            this.label6.Text = "Longitude";
            // 
            // label20
            // 
            this.label20.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label20.Location = new System.Drawing.Point(120, 120);
            this.label20.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(45, 20);
            this.label20.TabIndex = 47;
            this.label20.Text = "-";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.Location = new System.Drawing.Point(9, 72);
            this.label7.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(68, 21);
            this.label7.TabIndex = 20;
            this.label7.Text = "Heading";
            // 
            // lb_altitude
            // 
            this.lb_altitude.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_altitude.Location = new System.Drawing.Point(120, 96);
            this.lb_altitude.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_altitude.Name = "lb_altitude";
            this.lb_altitude.Size = new System.Drawing.Size(45, 20);
            this.lb_altitude.TabIndex = 46;
            this.lb_altitude.Text = "-";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.Location = new System.Drawing.Point(9, 96);
            this.label8.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(64, 21);
            this.label8.TabIndex = 27;
            this.label8.Text = "Altitude";
            // 
            // lb_heading
            // 
            this.lb_heading.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_heading.Location = new System.Drawing.Point(120, 72);
            this.lb_heading.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_heading.Name = "lb_heading";
            this.lb_heading.Size = new System.Drawing.Size(45, 20);
            this.lb_heading.TabIndex = 45;
            this.lb_heading.Text = "-";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.Location = new System.Drawing.Point(9, 120);
            this.label9.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(95, 21);
            this.label9.TabIndex = 28;
            this.label9.Text = "Max altitude";
            // 
            // lb_long
            // 
            this.lb_long.Font = new System.Drawing.Font("Segoe UI", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_long.Location = new System.Drawing.Point(92, 51);
            this.lb_long.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_long.Name = "lb_long";
            this.lb_long.Size = new System.Drawing.Size(73, 20);
            this.lb_long.TabIndex = 44;
            this.lb_long.Text = "-";
            // 
            // lb_lat
            // 
            this.lb_lat.Font = new System.Drawing.Font("Segoe UI", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_lat.Location = new System.Drawing.Point(99, 27);
            this.lb_lat.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_lat.Name = "lb_lat";
            this.lb_lat.Size = new System.Drawing.Size(73, 20);
            this.lb_lat.TabIndex = 43;
            this.lb_lat.Text = "-";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label13.Location = new System.Drawing.Point(8, 142);
            this.label13.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(99, 21);
            this.label13.TabIndex = 39;
            this.label13.Text = "LOS distance";
            // 
            // label24
            // 
            this.label24.AutoSize = true;
            this.label24.Font = new System.Drawing.Font("Segoe UI", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label24.ForeColor = System.Drawing.Color.Red;
            this.label24.Location = new System.Drawing.Point(47, 52);
            this.label24.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(201, 30);
            this.label24.TabIndex = 64;
            this.label24.Text = "!!Connection Lost!!";
            this.label24.Visible = false;
            // 
            // panel1
            // 
            this.panel1.Location = new System.Drawing.Point(303, 16);
            this.panel1.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(21, 19);
            this.panel1.TabIndex = 69;
            // 
            // panel4
            // 
            this.panel4.Controls.Add(this.textBox2);
            this.panel4.Controls.Add(this.label77);
            this.panel4.Controls.Add(this.tb_rx_error);
            this.panel4.Controls.Add(this.label1);
            this.panel4.Controls.Add(this.label3);
            this.panel4.Controls.Add(this.textBox3);
            this.panel4.Controls.Add(this.lb_fail);
            this.panel4.Location = new System.Drawing.Point(13, 506);
            this.panel4.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel4.Name = "panel4";
            this.panel4.Size = new System.Drawing.Size(327, 89);
            this.panel4.TabIndex = 73;
            // 
            // textBox2
            // 
            this.textBox2.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox2.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBox2.Location = new System.Drawing.Point(104, 8);
            this.textBox2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox2.Name = "textBox2";
            this.textBox2.ReadOnly = true;
            this.textBox2.Size = new System.Drawing.Size(386, 18);
            this.textBox2.TabIndex = 7;
            this.textBox2.Text = "-";
            // 
            // label77
            // 
            this.label77.AutoSize = true;
            this.label77.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label77.Location = new System.Drawing.Point(4, 56);
            this.label77.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label77.Name = "label77";
            this.label77.Size = new System.Drawing.Size(71, 21);
            this.label77.TabIndex = 14;
            this.label77.Text = "RX_Error";
            // 
            // tb_rx_error
            // 
            this.tb_rx_error.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.tb_rx_error.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.tb_rx_error.Location = new System.Drawing.Point(103, 54);
            this.tb_rx_error.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.tb_rx_error.Name = "tb_rx_error";
            this.tb_rx_error.ReadOnly = true;
            this.tb_rx_error.Size = new System.Drawing.Size(46, 22);
            this.tb_rx_error.TabIndex = 13;
            this.tb_rx_error.Text = "-";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(4, 8);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(93, 21);
            this.label1.TabIndex = 8;
            this.label1.Text = "Flight mode";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(4, 35);
            this.label3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(45, 21);
            this.label3.TabIndex = 11;
            this.label3.Text = "Error";
            // 
            // textBox3
            // 
            this.textBox3.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox3.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBox3.Location = new System.Drawing.Point(104, 29);
            this.textBox3.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(96, 22);
            this.textBox3.TabIndex = 12;
            this.textBox3.Text = "-";
            // 
            // lb_fail
            // 
            this.lb_fail.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_fail.Location = new System.Drawing.Point(255, 32);
            this.lb_fail.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_fail.Name = "lb_fail";
            this.lb_fail.Size = new System.Drawing.Size(64, 19);
            this.lb_fail.TabIndex = 85;
            this.lb_fail.Text = "-";
            // 
            // panel3
            // 
            this.panel3.Controls.Add(this.lb_gps_time);
            this.panel3.Controls.Add(this.label101);
            this.panel3.Controls.Add(this.label99);
            this.panel3.Controls.Add(this.label81);
            this.panel3.Controls.Add(this.lb_fc_load);
            this.panel3.Controls.Add(this.label55);
            this.panel3.Controls.Add(this.label36);
            this.panel3.Controls.Add(this.textBox11);
            this.panel3.Location = new System.Drawing.Point(351, 3);
            this.panel3.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel3.Name = "panel3";
            this.panel3.Size = new System.Drawing.Size(738, 56);
            this.panel3.TabIndex = 72;
            // 
            // label81
            // 
            this.label81.AutoSize = true;
            this.label81.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label81.Location = new System.Drawing.Point(679, 7);
            this.label81.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label81.Name = "label81";
            this.label81.Size = new System.Drawing.Size(23, 21);
            this.label81.TabIndex = 91;
            this.label81.Text = "%";
            // 
            // lb_fc_load
            // 
            this.lb_fc_load.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_fc_load.Location = new System.Drawing.Point(657, 7);
            this.lb_fc_load.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_fc_load.Name = "lb_fc_load";
            this.lb_fc_load.Size = new System.Drawing.Size(36, 19);
            this.lb_fc_load.TabIndex = 92;
            this.lb_fc_load.Text = "-";
            // 
            // label55
            // 
            this.label55.AutoSize = true;
            this.label55.Location = new System.Drawing.Point(601, 13);
            this.label55.Name = "label55";
            this.label55.Size = new System.Drawing.Size(61, 12);
            this.label55.TabIndex = 91;
            this.label55.Text = "FC 부하 : ";
            // 
            // label36
            // 
            this.label36.Font = new System.Drawing.Font("Consolas", 18F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label36.Location = new System.Drawing.Point(595, 28);
            this.label36.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label36.Name = "label36";
            this.label36.Size = new System.Drawing.Size(139, 25);
            this.label36.TabIndex = 51;
            this.label36.Text = "00:00:00";
            // 
            // textBox11
            // 
            this.textBox11.BackColor = System.Drawing.SystemColors.Control;
            this.textBox11.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox11.Font = new System.Drawing.Font("Microsoft Sans Serif", 21.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBox11.Location = new System.Drawing.Point(139, 3);
            this.textBox11.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox11.Name = "textBox11";
            this.textBox11.ReadOnly = true;
            this.textBox11.Size = new System.Drawing.Size(372, 33);
            this.textBox11.TabIndex = 0;
            this.textBox11.Text = "MCU Drone Flight Monitor";
            this.textBox11.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // panel2
            // 
            this.panel2.Controls.Add(this.label92);
            this.panel2.Controls.Add(this.lb_bat_mAh);
            this.panel2.Controls.Add(this.label90);
            this.panel2.Controls.Add(this.lb_bat_A);
            this.panel2.Controls.Add(this.lb_gps_fix);
            this.panel2.Controls.Add(this.label56);
            this.panel2.Controls.Add(this.label79);
            this.panel2.Controls.Add(this.label57);
            this.panel2.Controls.Add(this.lb_sat_num);
            this.panel2.Controls.Add(this.lb_armed);
            this.panel2.Controls.Add(this.pictureBox6);
            this.panel2.Controls.Add(this.pictureBox5);
            this.panel2.Controls.Add(this.pictureBox4);
            this.panel2.Controls.Add(this.pictureBox2);
            this.panel2.Controls.Add(this.pictureBox1);
            this.panel2.Controls.Add(this.lb_bat_V);
            this.panel2.Controls.Add(this.panel7);
            this.panel2.Controls.Add(this.textBox10);
            this.panel2.Controls.Add(this.bt_zoom_p);
            this.panel2.Controls.Add(this.bt_zoom_m);
            this.panel2.Controls.Add(this.label10);
            this.panel2.Controls.Add(this.label15);
            this.panel2.Location = new System.Drawing.Point(1087, 3);
            this.panel2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(71, 596);
            this.panel2.TabIndex = 84;
            // 
            // label92
            // 
            this.label92.AutoSize = true;
            this.label92.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label92.Location = new System.Drawing.Point(27, 423);
            this.label92.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label92.Name = "label92";
            this.label92.Size = new System.Drawing.Size(43, 21);
            this.label92.TabIndex = 96;
            this.label92.Text = "mAh";
            // 
            // lb_bat_mAh
            // 
            this.lb_bat_mAh.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_bat_mAh.Location = new System.Drawing.Point(6, 423);
            this.lb_bat_mAh.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_bat_mAh.Name = "lb_bat_mAh";
            this.lb_bat_mAh.Size = new System.Drawing.Size(64, 19);
            this.lb_bat_mAh.TabIndex = 95;
            this.lb_bat_mAh.Text = "-";
            // 
            // label90
            // 
            this.label90.AutoSize = true;
            this.label90.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label90.Location = new System.Drawing.Point(47, 401);
            this.label90.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label90.Name = "label90";
            this.label90.Size = new System.Drawing.Size(20, 21);
            this.label90.TabIndex = 94;
            this.label90.Text = "A";
            // 
            // lb_bat_A
            // 
            this.lb_bat_A.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_bat_A.Location = new System.Drawing.Point(6, 401);
            this.lb_bat_A.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_bat_A.Name = "lb_bat_A";
            this.lb_bat_A.Size = new System.Drawing.Size(64, 19);
            this.lb_bat_A.TabIndex = 93;
            this.lb_bat_A.Text = "-";
            // 
            // lb_gps_fix
            // 
            this.lb_gps_fix.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_gps_fix.Location = new System.Drawing.Point(23, 328);
            this.lb_gps_fix.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_gps_fix.Name = "lb_gps_fix";
            this.lb_gps_fix.Size = new System.Drawing.Size(35, 19);
            this.lb_gps_fix.TabIndex = 92;
            this.lb_gps_fix.Text = "-";
            // 
            // label56
            // 
            this.label56.AutoSize = true;
            this.label56.Location = new System.Drawing.Point(5, 228);
            this.label56.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label56.Name = "label56";
            this.label56.Size = new System.Drawing.Size(63, 12);
            this.label56.TabIndex = 91;
            this.label56.Text = "SAT_NUM";
            // 
            // label79
            // 
            this.label79.AutoSize = true;
            this.label79.Font = new System.Drawing.Font("굴림", 10F);
            this.label79.Location = new System.Drawing.Point(8, 366);
            this.label79.Name = "label79";
            this.label79.Size = new System.Drawing.Size(49, 14);
            this.label79.TabIndex = 90;
            this.label79.Text = "배터리";
            // 
            // label57
            // 
            this.label57.AutoSize = true;
            this.label57.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label57.Location = new System.Drawing.Point(47, 380);
            this.label57.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label57.Name = "label57";
            this.label57.Size = new System.Drawing.Size(20, 21);
            this.label57.TabIndex = 89;
            this.label57.Text = "V";
            // 
            // lb_sat_num
            // 
            this.lb_sat_num.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_sat_num.Location = new System.Drawing.Point(27, 246);
            this.lb_sat_num.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_sat_num.Name = "lb_sat_num";
            this.lb_sat_num.Size = new System.Drawing.Size(35, 19);
            this.lb_sat_num.TabIndex = 87;
            this.lb_sat_num.Text = "-";
            // 
            // lb_armed
            // 
            this.lb_armed.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_armed.Location = new System.Drawing.Point(17, 62);
            this.lb_armed.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_armed.Name = "lb_armed";
            this.lb_armed.Size = new System.Drawing.Size(22, 19);
            this.lb_armed.TabIndex = 57;
            this.lb_armed.Text = "-";
            // 
            // pictureBox6
            // 
            this.pictureBox6.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox6.Image")));
            this.pictureBox6.Location = new System.Drawing.Point(18, 273);
            this.pictureBox6.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.pictureBox6.Name = "pictureBox6";
            this.pictureBox6.Size = new System.Drawing.Size(35, 28);
            this.pictureBox6.TabIndex = 56;
            this.pictureBox6.TabStop = false;
            // 
            // pictureBox5
            // 
            this.pictureBox5.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox5.Image")));
            this.pictureBox5.Location = new System.Drawing.Point(18, 273);
            this.pictureBox5.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.pictureBox5.Name = "pictureBox5";
            this.pictureBox5.Size = new System.Drawing.Size(35, 28);
            this.pictureBox5.TabIndex = 55;
            this.pictureBox5.TabStop = false;
            // 
            // pictureBox4
            // 
            this.pictureBox4.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox4.Image")));
            this.pictureBox4.Location = new System.Drawing.Point(18, 273);
            this.pictureBox4.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.pictureBox4.Name = "pictureBox4";
            this.pictureBox4.Size = new System.Drawing.Size(35, 28);
            this.pictureBox4.TabIndex = 54;
            this.pictureBox4.TabStop = false;
            // 
            // pictureBox2
            // 
            this.pictureBox2.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox2.Image")));
            this.pictureBox2.InitialImage = ((System.Drawing.Image)(resources.GetObject("pictureBox2.InitialImage")));
            this.pictureBox2.Location = new System.Drawing.Point(10, 13);
            this.pictureBox2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(47, 46);
            this.pictureBox2.TabIndex = 52;
            this.pictureBox2.TabStop = false;
            this.pictureBox2.Visible = false;
            // 
            // pictureBox1
            // 
            this.pictureBox1.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox1.Image")));
            this.pictureBox1.InitialImage = ((System.Drawing.Image)(resources.GetObject("pictureBox1.InitialImage")));
            this.pictureBox1.Location = new System.Drawing.Point(10, 13);
            this.pictureBox1.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(47, 46);
            this.pictureBox1.TabIndex = 51;
            this.pictureBox1.TabStop = false;
            // 
            // lb_bat_V
            // 
            this.lb_bat_V.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_bat_V.Location = new System.Drawing.Point(6, 380);
            this.lb_bat_V.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_bat_V.Name = "lb_bat_V";
            this.lb_bat_V.Size = new System.Drawing.Size(64, 19);
            this.lb_bat_V.TabIndex = 48;
            this.lb_bat_V.Text = "-";
            // 
            // panel7
            // 
            this.panel7.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel7.Controls.Add(this.panel6);
            this.panel7.Controls.Add(this.panel5);
            this.panel7.Location = new System.Drawing.Point(18, 457);
            this.panel7.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel7.Name = "panel7";
            this.panel7.Size = new System.Drawing.Size(40, 127);
            this.panel7.TabIndex = 50;
            // 
            // panel6
            // 
            this.panel6.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.panel6.Location = new System.Drawing.Point(1, 1);
            this.panel6.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel6.Name = "panel6";
            this.panel6.Size = new System.Drawing.Size(40, 124);
            this.panel6.TabIndex = 50;
            // 
            // panel5
            // 
            this.panel5.BackColor = System.Drawing.Color.Lime;
            this.panel5.Location = new System.Drawing.Point(1, 1);
            this.panel5.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel5.Name = "panel5";
            this.panel5.Size = new System.Drawing.Size(40, 124);
            this.panel5.TabIndex = 49;
            // 
            // textBox10
            // 
            this.textBox10.Location = new System.Drawing.Point(12, 152);
            this.textBox10.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox10.Name = "textBox10";
            this.textBox10.ReadOnly = true;
            this.textBox10.Size = new System.Drawing.Size(50, 21);
            this.textBox10.TabIndex = 33;
            this.textBox10.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // bt_zoom_p
            // 
            this.bt_zoom_p.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_zoom_p.Location = new System.Drawing.Point(12, 128);
            this.bt_zoom_p.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.bt_zoom_p.Name = "bt_zoom_p";
            this.bt_zoom_p.Size = new System.Drawing.Size(50, 18);
            this.bt_zoom_p.TabIndex = 32;
            this.bt_zoom_p.Text = "+";
            this.bt_zoom_p.UseVisualStyleBackColor = true;
            this.bt_zoom_p.Click += new System.EventHandler(this.bt_zoom_p_Click);
            // 
            // bt_zoom_m
            // 
            this.bt_zoom_m.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_zoom_m.Location = new System.Drawing.Point(12, 176);
            this.bt_zoom_m.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.bt_zoom_m.Name = "bt_zoom_m";
            this.bt_zoom_m.Size = new System.Drawing.Size(50, 18);
            this.bt_zoom_m.TabIndex = 31;
            this.bt_zoom_m.Text = "-";
            this.bt_zoom_m.UseVisualStyleBackColor = true;
            this.bt_zoom_m.Click += new System.EventHandler(this.bt_zoom_m_Click);
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(16, 113);
            this.label10.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(38, 12);
            this.label10.TabIndex = 30;
            this.label10.Text = "Zoom";
            // 
            // label15
            // 
            this.label15.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label15.Location = new System.Drawing.Point(6, 309);
            this.label15.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(60, 19);
            this.label15.TabIndex = 42;
            this.label15.Text = "GPS_FIX";
            this.label15.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lb_debug0
            // 
            this.lb_debug0.AutoSize = true;
            this.lb_debug0.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug0.Location = new System.Drawing.Point(119, 13);
            this.lb_debug0.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug0.Name = "lb_debug0";
            this.lb_debug0.Size = new System.Drawing.Size(16, 21);
            this.lb_debug0.TabIndex = 86;
            this.lb_debug0.Text = "-";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.tb_FC_ALT_Range_D);
            this.groupBox1.Controls.Add(this.tb_FC_ALT_Range_I);
            this.groupBox1.Controls.Add(this.tb_FC_ALT_Range_P);
            this.groupBox1.Controls.Add(this.label96);
            this.groupBox1.Controls.Add(this.label97);
            this.groupBox1.Controls.Add(this.tb_ALT_Range_D);
            this.groupBox1.Controls.Add(this.tb_ALT_Range_I);
            this.groupBox1.Controls.Add(this.tb_ALT_Range_P);
            this.groupBox1.Controls.Add(this.tb_FC_POS_Opflow_D);
            this.groupBox1.Controls.Add(this.tb_FC_POS_Opflow_I);
            this.groupBox1.Controls.Add(this.tb_FC_POS_Opflow_P);
            this.groupBox1.Controls.Add(this.label93);
            this.groupBox1.Controls.Add(this.label94);
            this.groupBox1.Controls.Add(this.label95);
            this.groupBox1.Controls.Add(this.tb_POS_Opflow_D);
            this.groupBox1.Controls.Add(this.tb_POS_Opflow_I);
            this.groupBox1.Controls.Add(this.tb_POS_Opflow_P);
            this.groupBox1.Controls.Add(this.tb_FC_ALT_D);
            this.groupBox1.Controls.Add(this.tb_FC_ALT_I);
            this.groupBox1.Controls.Add(this.tb_FC_ALT_P);
            this.groupBox1.Controls.Add(this.label89);
            this.groupBox1.Controls.Add(this.label88);
            this.groupBox1.Controls.Add(this.label87);
            this.groupBox1.Controls.Add(this.tb_ALT_D);
            this.groupBox1.Controls.Add(this.tb_ALT_I);
            this.groupBox1.Controls.Add(this.tb_ALT_P);
            this.groupBox1.Controls.Add(this.cB_RP_Coupling);
            this.groupBox1.Controls.Add(this.bt_pid_save);
            this.groupBox1.Controls.Add(this.bt_pid_copy);
            this.groupBox1.Controls.Add(this.tb_FC_Y_R_D);
            this.groupBox1.Controls.Add(this.tb_FC_Y_R_I);
            this.groupBox1.Controls.Add(this.tb_FC_Y_R_P);
            this.groupBox1.Controls.Add(this.label46);
            this.groupBox1.Controls.Add(this.label47);
            this.groupBox1.Controls.Add(this.tb_FC_Y_A_D);
            this.groupBox1.Controls.Add(this.tb_FC_Y_A_I);
            this.groupBox1.Controls.Add(this.tb_FC_Y_A_P);
            this.groupBox1.Controls.Add(this.tb_FC_P_O_D);
            this.groupBox1.Controls.Add(this.tb_FC_P_O_I);
            this.groupBox1.Controls.Add(this.tb_FC_P_O_P);
            this.groupBox1.Controls.Add(this.label48);
            this.groupBox1.Controls.Add(this.label49);
            this.groupBox1.Controls.Add(this.tb_FC_P_I_D);
            this.groupBox1.Controls.Add(this.tb_FC_P_I_I);
            this.groupBox1.Controls.Add(this.tb_FC_P_I_P);
            this.groupBox1.Controls.Add(this.tb_FC_R_O_D);
            this.groupBox1.Controls.Add(this.tb_FC_R_O_I);
            this.groupBox1.Controls.Add(this.tb_FC_R_O_P);
            this.groupBox1.Controls.Add(this.label50);
            this.groupBox1.Controls.Add(this.label51);
            this.groupBox1.Controls.Add(this.label52);
            this.groupBox1.Controls.Add(this.tb_FC_R_I_D);
            this.groupBox1.Controls.Add(this.label53);
            this.groupBox1.Controls.Add(this.tb_FC_R_I_I);
            this.groupBox1.Controls.Add(this.label54);
            this.groupBox1.Controls.Add(this.tb_FC_R_I_P);
            this.groupBox1.Controls.Add(this.tb_Y_R_D);
            this.groupBox1.Controls.Add(this.tb_Y_R_I);
            this.groupBox1.Controls.Add(this.tb_Y_R_P);
            this.groupBox1.Controls.Add(this.label43);
            this.groupBox1.Controls.Add(this.label45);
            this.groupBox1.Controls.Add(this.tb_Y_A_D);
            this.groupBox1.Controls.Add(this.tb_Y_A_I);
            this.groupBox1.Controls.Add(this.tb_Y_A_P);
            this.groupBox1.Controls.Add(this.tb_P_O_D);
            this.groupBox1.Controls.Add(this.tb_P_O_I);
            this.groupBox1.Controls.Add(this.tb_P_O_P);
            this.groupBox1.Controls.Add(this.label34);
            this.groupBox1.Controls.Add(this.label35);
            this.groupBox1.Controls.Add(this.tb_P_I_D);
            this.groupBox1.Controls.Add(this.tb_P_I_I);
            this.groupBox1.Controls.Add(this.tb_P_I_P);
            this.groupBox1.Controls.Add(this.tb_R_O_D);
            this.groupBox1.Controls.Add(this.tb_R_O_I);
            this.groupBox1.Controls.Add(this.tb_R_O_P);
            this.groupBox1.Controls.Add(this.label33);
            this.groupBox1.Controls.Add(this.label32);
            this.groupBox1.Controls.Add(this.label31);
            this.groupBox1.Controls.Add(this.label30);
            this.groupBox1.Controls.Add(this.label29);
            this.groupBox1.Controls.Add(this.bt_pid_recive);
            this.groupBox1.Controls.Add(this.bt_pid_send);
            this.groupBox1.Controls.Add(this.label39);
            this.groupBox1.Controls.Add(this.tb_R_I_D);
            this.groupBox1.Controls.Add(this.label40);
            this.groupBox1.Controls.Add(this.tb_R_I_I);
            this.groupBox1.Controls.Add(this.label41);
            this.groupBox1.Controls.Add(this.tb_R_I_P);
            this.groupBox1.Location = new System.Drawing.Point(11, 209);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(699, 354);
            this.groupBox1.TabIndex = 84;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "PID상수";
            // 
            // tb_FC_ALT_Range_D
            // 
            this.tb_FC_ALT_Range_D.Location = new System.Drawing.Point(617, 249);
            this.tb_FC_ALT_Range_D.Name = "tb_FC_ALT_Range_D";
            this.tb_FC_ALT_Range_D.ReadOnly = true;
            this.tb_FC_ALT_Range_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_ALT_Range_D.TabIndex = 175;
            // 
            // tb_FC_ALT_Range_I
            // 
            this.tb_FC_ALT_Range_I.Location = new System.Drawing.Point(546, 249);
            this.tb_FC_ALT_Range_I.Name = "tb_FC_ALT_Range_I";
            this.tb_FC_ALT_Range_I.ReadOnly = true;
            this.tb_FC_ALT_Range_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_ALT_Range_I.TabIndex = 174;
            // 
            // tb_FC_ALT_Range_P
            // 
            this.tb_FC_ALT_Range_P.Location = new System.Drawing.Point(475, 249);
            this.tb_FC_ALT_Range_P.Name = "tb_FC_ALT_Range_P";
            this.tb_FC_ALT_Range_P.ReadOnly = true;
            this.tb_FC_ALT_Range_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_ALT_Range_P.TabIndex = 173;
            // 
            // label96
            // 
            this.label96.AutoSize = true;
            this.label96.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label96.Location = new System.Drawing.Point(403, 252);
            this.label96.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label96.Name = "label96";
            this.label96.Size = new System.Drawing.Size(65, 17);
            this.label96.TabIndex = 172;
            this.label96.Text = "Alt_Range";
            // 
            // label97
            // 
            this.label97.AutoSize = true;
            this.label97.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label97.Location = new System.Drawing.Point(46, 253);
            this.label97.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label97.Name = "label97";
            this.label97.Size = new System.Drawing.Size(70, 17);
            this.label97.TabIndex = 171;
            this.label97.Tag = "Z";
            this.label97.Text = "ALT_Range";
            // 
            // tb_ALT_Range_D
            // 
            this.tb_ALT_Range_D.Location = new System.Drawing.Point(264, 250);
            this.tb_ALT_Range_D.Name = "tb_ALT_Range_D";
            this.tb_ALT_Range_D.Size = new System.Drawing.Size(45, 21);
            this.tb_ALT_Range_D.TabIndex = 170;
            // 
            // tb_ALT_Range_I
            // 
            this.tb_ALT_Range_I.Location = new System.Drawing.Point(193, 250);
            this.tb_ALT_Range_I.Name = "tb_ALT_Range_I";
            this.tb_ALT_Range_I.Size = new System.Drawing.Size(45, 21);
            this.tb_ALT_Range_I.TabIndex = 169;
            // 
            // tb_ALT_Range_P
            // 
            this.tb_ALT_Range_P.Location = new System.Drawing.Point(122, 250);
            this.tb_ALT_Range_P.Name = "tb_ALT_Range_P";
            this.tb_ALT_Range_P.Size = new System.Drawing.Size(45, 21);
            this.tb_ALT_Range_P.TabIndex = 168;
            // 
            // tb_FC_POS_Opflow_D
            // 
            this.tb_FC_POS_Opflow_D.Location = new System.Drawing.Point(617, 287);
            this.tb_FC_POS_Opflow_D.Name = "tb_FC_POS_Opflow_D";
            this.tb_FC_POS_Opflow_D.ReadOnly = true;
            this.tb_FC_POS_Opflow_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_POS_Opflow_D.TabIndex = 167;
            // 
            // tb_FC_POS_Opflow_I
            // 
            this.tb_FC_POS_Opflow_I.Location = new System.Drawing.Point(546, 287);
            this.tb_FC_POS_Opflow_I.Name = "tb_FC_POS_Opflow_I";
            this.tb_FC_POS_Opflow_I.ReadOnly = true;
            this.tb_FC_POS_Opflow_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_POS_Opflow_I.TabIndex = 166;
            // 
            // tb_FC_POS_Opflow_P
            // 
            this.tb_FC_POS_Opflow_P.Location = new System.Drawing.Point(475, 287);
            this.tb_FC_POS_Opflow_P.Name = "tb_FC_POS_Opflow_P";
            this.tb_FC_POS_Opflow_P.ReadOnly = true;
            this.tb_FC_POS_Opflow_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_POS_Opflow_P.TabIndex = 165;
            // 
            // label93
            // 
            this.label93.AutoSize = true;
            this.label93.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label93.Location = new System.Drawing.Point(396, 290);
            this.label93.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label93.Name = "label93";
            this.label93.Size = new System.Drawing.Size(79, 17);
            this.label93.TabIndex = 164;
            this.label93.Text = "POS_Opflow";
            // 
            // label94
            // 
            this.label94.AutoSize = true;
            this.label94.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label94.Location = new System.Drawing.Point(0, 288);
            this.label94.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label94.Name = "label94";
            this.label94.Size = new System.Drawing.Size(41, 21);
            this.label94.TabIndex = 163;
            this.label94.Text = "POS";
            // 
            // label95
            // 
            this.label95.AutoSize = true;
            this.label95.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label95.Location = new System.Drawing.Point(42, 291);
            this.label95.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label95.Name = "label95";
            this.label95.Size = new System.Drawing.Size(79, 17);
            this.label95.TabIndex = 162;
            this.label95.Text = "POS_Opflow";
            // 
            // tb_POS_Opflow_D
            // 
            this.tb_POS_Opflow_D.Location = new System.Drawing.Point(264, 288);
            this.tb_POS_Opflow_D.Name = "tb_POS_Opflow_D";
            this.tb_POS_Opflow_D.Size = new System.Drawing.Size(45, 21);
            this.tb_POS_Opflow_D.TabIndex = 161;
            // 
            // tb_POS_Opflow_I
            // 
            this.tb_POS_Opflow_I.Location = new System.Drawing.Point(193, 288);
            this.tb_POS_Opflow_I.Name = "tb_POS_Opflow_I";
            this.tb_POS_Opflow_I.Size = new System.Drawing.Size(45, 21);
            this.tb_POS_Opflow_I.TabIndex = 160;
            // 
            // tb_POS_Opflow_P
            // 
            this.tb_POS_Opflow_P.Location = new System.Drawing.Point(122, 288);
            this.tb_POS_Opflow_P.Name = "tb_POS_Opflow_P";
            this.tb_POS_Opflow_P.Size = new System.Drawing.Size(45, 21);
            this.tb_POS_Opflow_P.TabIndex = 159;
            // 
            // tb_FC_ALT_D
            // 
            this.tb_FC_ALT_D.Location = new System.Drawing.Point(617, 216);
            this.tb_FC_ALT_D.Name = "tb_FC_ALT_D";
            this.tb_FC_ALT_D.ReadOnly = true;
            this.tb_FC_ALT_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_ALT_D.TabIndex = 158;
            // 
            // tb_FC_ALT_I
            // 
            this.tb_FC_ALT_I.Location = new System.Drawing.Point(546, 216);
            this.tb_FC_ALT_I.Name = "tb_FC_ALT_I";
            this.tb_FC_ALT_I.ReadOnly = true;
            this.tb_FC_ALT_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_ALT_I.TabIndex = 157;
            // 
            // tb_FC_ALT_P
            // 
            this.tb_FC_ALT_P.Location = new System.Drawing.Point(475, 216);
            this.tb_FC_ALT_P.Name = "tb_FC_ALT_P";
            this.tb_FC_ALT_P.ReadOnly = true;
            this.tb_FC_ALT_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_ALT_P.TabIndex = 156;
            // 
            // label89
            // 
            this.label89.AutoSize = true;
            this.label89.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label89.Location = new System.Drawing.Point(408, 219);
            this.label89.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label89.Name = "label89";
            this.label89.Size = new System.Drawing.Size(55, 17);
            this.label89.TabIndex = 155;
            this.label89.Text = "Alt_Baro";
            // 
            // label88
            // 
            this.label88.AutoSize = true;
            this.label88.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label88.Location = new System.Drawing.Point(7, 235);
            this.label88.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label88.Name = "label88";
            this.label88.Size = new System.Drawing.Size(37, 21);
            this.label88.TabIndex = 154;
            this.label88.Text = "ALT";
            // 
            // label87
            // 
            this.label87.AutoSize = true;
            this.label87.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label87.Location = new System.Drawing.Point(51, 220);
            this.label87.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label87.Name = "label87";
            this.label87.Size = new System.Drawing.Size(60, 17);
            this.label87.TabIndex = 153;
            this.label87.Text = "ALT_Baro";
            // 
            // tb_ALT_D
            // 
            this.tb_ALT_D.Location = new System.Drawing.Point(264, 217);
            this.tb_ALT_D.Name = "tb_ALT_D";
            this.tb_ALT_D.Size = new System.Drawing.Size(45, 21);
            this.tb_ALT_D.TabIndex = 152;
            // 
            // tb_ALT_I
            // 
            this.tb_ALT_I.Location = new System.Drawing.Point(193, 217);
            this.tb_ALT_I.Name = "tb_ALT_I";
            this.tb_ALT_I.Size = new System.Drawing.Size(45, 21);
            this.tb_ALT_I.TabIndex = 151;
            // 
            // tb_ALT_P
            // 
            this.tb_ALT_P.Location = new System.Drawing.Point(122, 217);
            this.tb_ALT_P.Name = "tb_ALT_P";
            this.tb_ALT_P.Size = new System.Drawing.Size(45, 21);
            this.tb_ALT_P.TabIndex = 150;
            // 
            // cB_RP_Coupling
            // 
            this.cB_RP_Coupling.AutoSize = true;
            this.cB_RP_Coupling.Checked = true;
            this.cB_RP_Coupling.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cB_RP_Coupling.Location = new System.Drawing.Point(11, 18);
            this.cB_RP_Coupling.Name = "cB_RP_Coupling";
            this.cB_RP_Coupling.Size = new System.Drawing.Size(102, 16);
            this.cB_RP_Coupling.TabIndex = 149;
            this.cB_RP_Coupling.Text = "R, P Coupling";
            this.cB_RP_Coupling.UseVisualStyleBackColor = true;
            this.cB_RP_Coupling.CheckedChanged += new System.EventHandler(this.cB_RP_Coupling_CheckedChanged);
            // 
            // bt_pid_save
            // 
            this.bt_pid_save.Location = new System.Drawing.Point(212, 326);
            this.bt_pid_save.Name = "bt_pid_save";
            this.bt_pid_save.Size = new System.Drawing.Size(102, 23);
            this.bt_pid_save.TabIndex = 148;
            this.bt_pid_save.Text = "데이터 저장하기";
            this.bt_pid_save.UseVisualStyleBackColor = true;
            this.bt_pid_save.Click += new System.EventHandler(this.bt_pid_save_Click);
            // 
            // bt_pid_copy
            // 
            this.bt_pid_copy.Location = new System.Drawing.Point(323, 155);
            this.bt_pid_copy.Name = "bt_pid_copy";
            this.bt_pid_copy.Size = new System.Drawing.Size(81, 41);
            this.bt_pid_copy.TabIndex = 147;
            this.bt_pid_copy.Text = "←Copy←";
            this.bt_pid_copy.UseVisualStyleBackColor = true;
            this.bt_pid_copy.Click += new System.EventHandler(this.bt_pid_copy_Click);
            // 
            // tb_FC_Y_R_D
            // 
            this.tb_FC_Y_R_D.Location = new System.Drawing.Point(617, 180);
            this.tb_FC_Y_R_D.Name = "tb_FC_Y_R_D";
            this.tb_FC_Y_R_D.ReadOnly = true;
            this.tb_FC_Y_R_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_Y_R_D.TabIndex = 146;
            // 
            // tb_FC_Y_R_I
            // 
            this.tb_FC_Y_R_I.Location = new System.Drawing.Point(546, 180);
            this.tb_FC_Y_R_I.Name = "tb_FC_Y_R_I";
            this.tb_FC_Y_R_I.ReadOnly = true;
            this.tb_FC_Y_R_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_Y_R_I.TabIndex = 145;
            // 
            // tb_FC_Y_R_P
            // 
            this.tb_FC_Y_R_P.Location = new System.Drawing.Point(475, 180);
            this.tb_FC_Y_R_P.Name = "tb_FC_Y_R_P";
            this.tb_FC_Y_R_P.ReadOnly = true;
            this.tb_FC_Y_R_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_Y_R_P.TabIndex = 144;
            // 
            // label46
            // 
            this.label46.AutoSize = true;
            this.label46.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label46.Location = new System.Drawing.Point(418, 183);
            this.label46.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label46.Name = "label46";
            this.label46.Size = new System.Drawing.Size(34, 17);
            this.label46.TabIndex = 143;
            this.label46.Text = "Rate";
            // 
            // label47
            // 
            this.label47.AutoSize = true;
            this.label47.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label47.Location = new System.Drawing.Point(415, 159);
            this.label47.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label47.Name = "label47";
            this.label47.Size = new System.Drawing.Size(41, 17);
            this.label47.TabIndex = 142;
            this.label47.Text = "Angle";
            // 
            // tb_FC_Y_A_D
            // 
            this.tb_FC_Y_A_D.Location = new System.Drawing.Point(617, 156);
            this.tb_FC_Y_A_D.Name = "tb_FC_Y_A_D";
            this.tb_FC_Y_A_D.ReadOnly = true;
            this.tb_FC_Y_A_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_Y_A_D.TabIndex = 141;
            // 
            // tb_FC_Y_A_I
            // 
            this.tb_FC_Y_A_I.Location = new System.Drawing.Point(546, 156);
            this.tb_FC_Y_A_I.Name = "tb_FC_Y_A_I";
            this.tb_FC_Y_A_I.ReadOnly = true;
            this.tb_FC_Y_A_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_Y_A_I.TabIndex = 140;
            // 
            // tb_FC_Y_A_P
            // 
            this.tb_FC_Y_A_P.Location = new System.Drawing.Point(475, 156);
            this.tb_FC_Y_A_P.Name = "tb_FC_Y_A_P";
            this.tb_FC_Y_A_P.ReadOnly = true;
            this.tb_FC_Y_A_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_Y_A_P.TabIndex = 139;
            // 
            // tb_FC_P_O_D
            // 
            this.tb_FC_P_O_D.Location = new System.Drawing.Point(617, 121);
            this.tb_FC_P_O_D.Name = "tb_FC_P_O_D";
            this.tb_FC_P_O_D.ReadOnly = true;
            this.tb_FC_P_O_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_P_O_D.TabIndex = 138;
            // 
            // tb_FC_P_O_I
            // 
            this.tb_FC_P_O_I.Location = new System.Drawing.Point(546, 121);
            this.tb_FC_P_O_I.Name = "tb_FC_P_O_I";
            this.tb_FC_P_O_I.ReadOnly = true;
            this.tb_FC_P_O_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_P_O_I.TabIndex = 137;
            // 
            // tb_FC_P_O_P
            // 
            this.tb_FC_P_O_P.Location = new System.Drawing.Point(475, 121);
            this.tb_FC_P_O_P.Name = "tb_FC_P_O_P";
            this.tb_FC_P_O_P.ReadOnly = true;
            this.tb_FC_P_O_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_P_O_P.TabIndex = 136;
            // 
            // label48
            // 
            this.label48.AutoSize = true;
            this.label48.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label48.Location = new System.Drawing.Point(415, 124);
            this.label48.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label48.Name = "label48";
            this.label48.Size = new System.Drawing.Size(41, 17);
            this.label48.TabIndex = 135;
            this.label48.Text = "Outer";
            // 
            // label49
            // 
            this.label49.AutoSize = true;
            this.label49.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label49.Location = new System.Drawing.Point(417, 100);
            this.label49.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label49.Name = "label49";
            this.label49.Size = new System.Drawing.Size(37, 17);
            this.label49.TabIndex = 134;
            this.label49.Text = "Inner";
            // 
            // tb_FC_P_I_D
            // 
            this.tb_FC_P_I_D.Location = new System.Drawing.Point(617, 97);
            this.tb_FC_P_I_D.Name = "tb_FC_P_I_D";
            this.tb_FC_P_I_D.ReadOnly = true;
            this.tb_FC_P_I_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_P_I_D.TabIndex = 133;
            // 
            // tb_FC_P_I_I
            // 
            this.tb_FC_P_I_I.Location = new System.Drawing.Point(546, 97);
            this.tb_FC_P_I_I.Name = "tb_FC_P_I_I";
            this.tb_FC_P_I_I.ReadOnly = true;
            this.tb_FC_P_I_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_P_I_I.TabIndex = 132;
            // 
            // tb_FC_P_I_P
            // 
            this.tb_FC_P_I_P.Location = new System.Drawing.Point(475, 97);
            this.tb_FC_P_I_P.Name = "tb_FC_P_I_P";
            this.tb_FC_P_I_P.ReadOnly = true;
            this.tb_FC_P_I_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_P_I_P.TabIndex = 131;
            // 
            // tb_FC_R_O_D
            // 
            this.tb_FC_R_O_D.Location = new System.Drawing.Point(617, 62);
            this.tb_FC_R_O_D.Name = "tb_FC_R_O_D";
            this.tb_FC_R_O_D.ReadOnly = true;
            this.tb_FC_R_O_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_R_O_D.TabIndex = 130;
            // 
            // tb_FC_R_O_I
            // 
            this.tb_FC_R_O_I.Location = new System.Drawing.Point(546, 62);
            this.tb_FC_R_O_I.Name = "tb_FC_R_O_I";
            this.tb_FC_R_O_I.ReadOnly = true;
            this.tb_FC_R_O_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_R_O_I.TabIndex = 129;
            // 
            // tb_FC_R_O_P
            // 
            this.tb_FC_R_O_P.Location = new System.Drawing.Point(475, 62);
            this.tb_FC_R_O_P.Name = "tb_FC_R_O_P";
            this.tb_FC_R_O_P.ReadOnly = true;
            this.tb_FC_R_O_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_R_O_P.TabIndex = 128;
            // 
            // label50
            // 
            this.label50.AutoSize = true;
            this.label50.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label50.Location = new System.Drawing.Point(415, 65);
            this.label50.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label50.Name = "label50";
            this.label50.Size = new System.Drawing.Size(41, 17);
            this.label50.TabIndex = 127;
            this.label50.Text = "Outer";
            // 
            // label51
            // 
            this.label51.AutoSize = true;
            this.label51.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label51.Location = new System.Drawing.Point(417, 41);
            this.label51.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label51.Name = "label51";
            this.label51.Size = new System.Drawing.Size(37, 17);
            this.label51.TabIndex = 126;
            this.label51.Text = "Inner";
            // 
            // label52
            // 
            this.label52.AutoSize = true;
            this.label52.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label52.Location = new System.Drawing.Point(636, 14);
            this.label52.Name = "label52";
            this.label52.Size = new System.Drawing.Size(17, 14);
            this.label52.TabIndex = 125;
            this.label52.Text = "D";
            // 
            // tb_FC_R_I_D
            // 
            this.tb_FC_R_I_D.Location = new System.Drawing.Point(617, 38);
            this.tb_FC_R_I_D.Name = "tb_FC_R_I_D";
            this.tb_FC_R_I_D.ReadOnly = true;
            this.tb_FC_R_I_D.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_R_I_D.TabIndex = 124;
            // 
            // label53
            // 
            this.label53.AutoSize = true;
            this.label53.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label53.Location = new System.Drawing.Point(564, 14);
            this.label53.Name = "label53";
            this.label53.Size = new System.Drawing.Size(10, 14);
            this.label53.TabIndex = 123;
            this.label53.Text = "I";
            // 
            // tb_FC_R_I_I
            // 
            this.tb_FC_R_I_I.Location = new System.Drawing.Point(546, 38);
            this.tb_FC_R_I_I.Name = "tb_FC_R_I_I";
            this.tb_FC_R_I_I.ReadOnly = true;
            this.tb_FC_R_I_I.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_R_I_I.TabIndex = 122;
            // 
            // label54
            // 
            this.label54.AutoSize = true;
            this.label54.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label54.Location = new System.Drawing.Point(492, 14);
            this.label54.Name = "label54";
            this.label54.Size = new System.Drawing.Size(17, 14);
            this.label54.TabIndex = 121;
            this.label54.Text = "P";
            // 
            // tb_FC_R_I_P
            // 
            this.tb_FC_R_I_P.Location = new System.Drawing.Point(475, 38);
            this.tb_FC_R_I_P.Name = "tb_FC_R_I_P";
            this.tb_FC_R_I_P.ReadOnly = true;
            this.tb_FC_R_I_P.Size = new System.Drawing.Size(45, 21);
            this.tb_FC_R_I_P.TabIndex = 120;
            // 
            // tb_Y_R_D
            // 
            this.tb_Y_R_D.Location = new System.Drawing.Point(264, 184);
            this.tb_Y_R_D.Name = "tb_Y_R_D";
            this.tb_Y_R_D.Size = new System.Drawing.Size(45, 21);
            this.tb_Y_R_D.TabIndex = 119;
            // 
            // tb_Y_R_I
            // 
            this.tb_Y_R_I.Location = new System.Drawing.Point(193, 184);
            this.tb_Y_R_I.Name = "tb_Y_R_I";
            this.tb_Y_R_I.Size = new System.Drawing.Size(45, 21);
            this.tb_Y_R_I.TabIndex = 118;
            // 
            // tb_Y_R_P
            // 
            this.tb_Y_R_P.Location = new System.Drawing.Point(122, 184);
            this.tb_Y_R_P.Name = "tb_Y_R_P";
            this.tb_Y_R_P.Size = new System.Drawing.Size(45, 21);
            this.tb_Y_R_P.TabIndex = 117;
            // 
            // label43
            // 
            this.label43.AutoSize = true;
            this.label43.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label43.Location = new System.Drawing.Point(64, 187);
            this.label43.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label43.Name = "label43";
            this.label43.Size = new System.Drawing.Size(34, 17);
            this.label43.TabIndex = 116;
            this.label43.Text = "Rate";
            // 
            // label45
            // 
            this.label45.AutoSize = true;
            this.label45.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label45.Location = new System.Drawing.Point(61, 163);
            this.label45.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label45.Name = "label45";
            this.label45.Size = new System.Drawing.Size(41, 17);
            this.label45.TabIndex = 115;
            this.label45.Text = "Angle";
            // 
            // tb_Y_A_D
            // 
            this.tb_Y_A_D.Location = new System.Drawing.Point(264, 160);
            this.tb_Y_A_D.Name = "tb_Y_A_D";
            this.tb_Y_A_D.Size = new System.Drawing.Size(45, 21);
            this.tb_Y_A_D.TabIndex = 114;
            // 
            // tb_Y_A_I
            // 
            this.tb_Y_A_I.Location = new System.Drawing.Point(193, 160);
            this.tb_Y_A_I.Name = "tb_Y_A_I";
            this.tb_Y_A_I.Size = new System.Drawing.Size(45, 21);
            this.tb_Y_A_I.TabIndex = 113;
            // 
            // tb_Y_A_P
            // 
            this.tb_Y_A_P.Location = new System.Drawing.Point(122, 160);
            this.tb_Y_A_P.Name = "tb_Y_A_P";
            this.tb_Y_A_P.Size = new System.Drawing.Size(45, 21);
            this.tb_Y_A_P.TabIndex = 112;
            // 
            // tb_P_O_D
            // 
            this.tb_P_O_D.Location = new System.Drawing.Point(264, 125);
            this.tb_P_O_D.Name = "tb_P_O_D";
            this.tb_P_O_D.Size = new System.Drawing.Size(45, 21);
            this.tb_P_O_D.TabIndex = 111;
            this.tb_P_O_D.TextChanged += new System.EventHandler(this.tb_P_O_D_TextChanged);
            // 
            // tb_P_O_I
            // 
            this.tb_P_O_I.Location = new System.Drawing.Point(193, 125);
            this.tb_P_O_I.Name = "tb_P_O_I";
            this.tb_P_O_I.Size = new System.Drawing.Size(45, 21);
            this.tb_P_O_I.TabIndex = 110;
            this.tb_P_O_I.TextChanged += new System.EventHandler(this.tb_P_O_I_TextChanged);
            // 
            // tb_P_O_P
            // 
            this.tb_P_O_P.Location = new System.Drawing.Point(122, 125);
            this.tb_P_O_P.Name = "tb_P_O_P";
            this.tb_P_O_P.Size = new System.Drawing.Size(45, 21);
            this.tb_P_O_P.TabIndex = 109;
            this.tb_P_O_P.TextChanged += new System.EventHandler(this.tb_P_O_P_TextChanged);
            // 
            // label34
            // 
            this.label34.AutoSize = true;
            this.label34.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label34.Location = new System.Drawing.Point(61, 128);
            this.label34.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label34.Name = "label34";
            this.label34.Size = new System.Drawing.Size(41, 17);
            this.label34.TabIndex = 108;
            this.label34.Text = "Outer";
            // 
            // label35
            // 
            this.label35.AutoSize = true;
            this.label35.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label35.Location = new System.Drawing.Point(63, 104);
            this.label35.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label35.Name = "label35";
            this.label35.Size = new System.Drawing.Size(37, 17);
            this.label35.TabIndex = 107;
            this.label35.Text = "Inner";
            // 
            // tb_P_I_D
            // 
            this.tb_P_I_D.Location = new System.Drawing.Point(264, 101);
            this.tb_P_I_D.Name = "tb_P_I_D";
            this.tb_P_I_D.Size = new System.Drawing.Size(45, 21);
            this.tb_P_I_D.TabIndex = 106;
            this.tb_P_I_D.TextChanged += new System.EventHandler(this.tb_P_I_D_TextChanged);
            // 
            // tb_P_I_I
            // 
            this.tb_P_I_I.Location = new System.Drawing.Point(193, 101);
            this.tb_P_I_I.Name = "tb_P_I_I";
            this.tb_P_I_I.Size = new System.Drawing.Size(45, 21);
            this.tb_P_I_I.TabIndex = 105;
            this.tb_P_I_I.TextChanged += new System.EventHandler(this.tb_P_I_I_TextChanged);
            // 
            // tb_P_I_P
            // 
            this.tb_P_I_P.Location = new System.Drawing.Point(122, 101);
            this.tb_P_I_P.Name = "tb_P_I_P";
            this.tb_P_I_P.Size = new System.Drawing.Size(45, 21);
            this.tb_P_I_P.TabIndex = 104;
            this.tb_P_I_P.TextChanged += new System.EventHandler(this.tb_P_I_P_TextChanged);
            // 
            // tb_R_O_D
            // 
            this.tb_R_O_D.Location = new System.Drawing.Point(264, 66);
            this.tb_R_O_D.Name = "tb_R_O_D";
            this.tb_R_O_D.Size = new System.Drawing.Size(45, 21);
            this.tb_R_O_D.TabIndex = 103;
            this.tb_R_O_D.TextChanged += new System.EventHandler(this.tb_R_O_D_TextChanged);
            // 
            // tb_R_O_I
            // 
            this.tb_R_O_I.Location = new System.Drawing.Point(193, 66);
            this.tb_R_O_I.Name = "tb_R_O_I";
            this.tb_R_O_I.Size = new System.Drawing.Size(45, 21);
            this.tb_R_O_I.TabIndex = 102;
            this.tb_R_O_I.TextChanged += new System.EventHandler(this.tb_R_O_I_TextChanged);
            // 
            // tb_R_O_P
            // 
            this.tb_R_O_P.Location = new System.Drawing.Point(122, 66);
            this.tb_R_O_P.Name = "tb_R_O_P";
            this.tb_R_O_P.Size = new System.Drawing.Size(45, 21);
            this.tb_R_O_P.TabIndex = 101;
            this.tb_R_O_P.TextChanged += new System.EventHandler(this.tb_R_O_P_TextChanged);
            // 
            // label33
            // 
            this.label33.AutoSize = true;
            this.label33.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label33.Location = new System.Drawing.Point(61, 69);
            this.label33.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label33.Name = "label33";
            this.label33.Size = new System.Drawing.Size(41, 17);
            this.label33.TabIndex = 100;
            this.label33.Text = "Outer";
            // 
            // label32
            // 
            this.label32.AutoSize = true;
            this.label32.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label32.Location = new System.Drawing.Point(63, 45);
            this.label32.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label32.Name = "label32";
            this.label32.Size = new System.Drawing.Size(37, 17);
            this.label32.TabIndex = 99;
            this.label32.Text = "Inner";
            // 
            // label31
            // 
            this.label31.AutoSize = true;
            this.label31.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label31.Location = new System.Drawing.Point(7, 171);
            this.label31.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label31.Name = "label31";
            this.label31.Size = new System.Drawing.Size(45, 21);
            this.label31.TabIndex = 98;
            this.label31.Text = "YAW";
            // 
            // label30
            // 
            this.label30.AutoSize = true;
            this.label30.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label30.Location = new System.Drawing.Point(7, 118);
            this.label30.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label30.Name = "label30";
            this.label30.Size = new System.Drawing.Size(55, 21);
            this.label30.TabIndex = 97;
            this.label30.Text = "PITCH";
            // 
            // label29
            // 
            this.label29.AutoSize = true;
            this.label29.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label29.Location = new System.Drawing.Point(7, 61);
            this.label29.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label29.Name = "label29";
            this.label29.Size = new System.Drawing.Size(48, 21);
            this.label29.TabIndex = 96;
            this.label29.Text = "ROLL";
            // 
            // bt_pid_recive
            // 
            this.bt_pid_recive.Location = new System.Drawing.Point(495, 325);
            this.bt_pid_recive.Name = "bt_pid_recive";
            this.bt_pid_recive.Size = new System.Drawing.Size(115, 23);
            this.bt_pid_recive.TabIndex = 53;
            this.bt_pid_recive.Text = "데이터 가져오기";
            this.bt_pid_recive.UseVisualStyleBackColor = true;
            this.bt_pid_recive.Click += new System.EventHandler(this.bt_pid_recive_Click);
            // 
            // bt_pid_send
            // 
            this.bt_pid_send.Location = new System.Drawing.Point(84, 325);
            this.bt_pid_send.Name = "bt_pid_send";
            this.bt_pid_send.Size = new System.Drawing.Size(102, 23);
            this.bt_pid_send.TabIndex = 52;
            this.bt_pid_send.Text = "데이터 보내기";
            this.bt_pid_send.UseVisualStyleBackColor = true;
            this.bt_pid_send.Click += new System.EventHandler(this.bt_pid_send_Click);
            // 
            // label39
            // 
            this.label39.AutoSize = true;
            this.label39.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label39.Location = new System.Drawing.Point(277, 18);
            this.label39.Name = "label39";
            this.label39.Size = new System.Drawing.Size(17, 14);
            this.label39.TabIndex = 51;
            this.label39.Text = "D";
            // 
            // tb_R_I_D
            // 
            this.tb_R_I_D.Location = new System.Drawing.Point(264, 42);
            this.tb_R_I_D.Name = "tb_R_I_D";
            this.tb_R_I_D.Size = new System.Drawing.Size(45, 21);
            this.tb_R_I_D.TabIndex = 50;
            this.tb_R_I_D.TextChanged += new System.EventHandler(this.tb_R_I_D_TextChanged);
            // 
            // label40
            // 
            this.label40.AutoSize = true;
            this.label40.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label40.Location = new System.Drawing.Point(205, 18);
            this.label40.Name = "label40";
            this.label40.Size = new System.Drawing.Size(10, 14);
            this.label40.TabIndex = 49;
            this.label40.Text = "I";
            // 
            // tb_R_I_I
            // 
            this.tb_R_I_I.Location = new System.Drawing.Point(193, 42);
            this.tb_R_I_I.Name = "tb_R_I_I";
            this.tb_R_I_I.Size = new System.Drawing.Size(45, 21);
            this.tb_R_I_I.TabIndex = 48;
            this.tb_R_I_I.TextChanged += new System.EventHandler(this.tb_R_I_I_TextChanged);
            // 
            // label41
            // 
            this.label41.AutoSize = true;
            this.label41.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label41.Location = new System.Drawing.Point(133, 18);
            this.label41.Name = "label41";
            this.label41.Size = new System.Drawing.Size(17, 14);
            this.label41.TabIndex = 47;
            this.label41.Text = "P";
            // 
            // tb_R_I_P
            // 
            this.tb_R_I_P.Location = new System.Drawing.Point(122, 42);
            this.tb_R_I_P.Name = "tb_R_I_P";
            this.tb_R_I_P.Size = new System.Drawing.Size(45, 21);
            this.tb_R_I_P.TabIndex = 46;
            this.tb_R_I_P.TextChanged += new System.EventHandler(this.tb_R_I_P_TextChanged);
            // 
            // rx_timer_blink
            // 
            this.rx_timer_blink.Enabled = true;
            this.rx_timer_blink.Tick += new System.EventHandler(this.Rx_timer_blink_Tick);
            // 
            // timer_status
            // 
            this.timer_status.Enabled = true;
            this.timer_status.Interval = 50;
            this.timer_status.Tick += new System.EventHandler(this.timer_status_Tick);
            // 
            // flight_timer
            // 
            this.flight_timer.Enabled = true;
            this.flight_timer.Interval = 1000;
            this.flight_timer.Tick += new System.EventHandler(this.flight_timer_Tick);
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.Gauge_RR);
            this.groupBox5.Controls.Add(this.Gauge_RF);
            this.groupBox5.Controls.Add(this.Gauge_LR);
            this.groupBox5.Controls.Add(this.Gauge_LF);
            this.groupBox5.Controls.Add(this.lb_motor3);
            this.groupBox5.Controls.Add(this.lb_motor2);
            this.groupBox5.Controls.Add(this.lb_motor1);
            this.groupBox5.Controls.Add(this.lb_motor0);
            this.groupBox5.Controls.Add(this.label28);
            this.groupBox5.Controls.Add(this.label23);
            this.groupBox5.Controls.Add(this.label19);
            this.groupBox5.Controls.Add(this.label16);
            this.groupBox5.Location = new System.Drawing.Point(318, 26);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(215, 177);
            this.groupBox5.TabIndex = 85;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "MOTOR";
            // 
            // Gauge_RR
            // 
            this.Gauge_RR.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.Gauge_RR.Location = new System.Drawing.Point(126, 88);
            this.Gauge_RR.Name = "Gauge_RR";
            this.Gauge_RR.Size = new System.Drawing.Size(50, 50);
            this.Gauge_RR.TabIndex = 163;
            this.Gauge_RR.Text = "solidGauge4";
            // 
            // Gauge_RF
            // 
            this.Gauge_RF.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.Gauge_RF.Location = new System.Drawing.Point(122, 11);
            this.Gauge_RF.Name = "Gauge_RF";
            this.Gauge_RF.Size = new System.Drawing.Size(50, 50);
            this.Gauge_RF.TabIndex = 162;
            this.Gauge_RF.Text = "solidGauge3";
            // 
            // Gauge_LR
            // 
            this.Gauge_LR.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.Gauge_LR.Location = new System.Drawing.Point(24, 88);
            this.Gauge_LR.Name = "Gauge_LR";
            this.Gauge_LR.Size = new System.Drawing.Size(50, 50);
            this.Gauge_LR.TabIndex = 161;
            this.Gauge_LR.Text = "solidGauge2";
            // 
            // Gauge_LF
            // 
            this.Gauge_LF.Font = new System.Drawing.Font("굴림", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.Gauge_LF.Location = new System.Drawing.Point(28, 11);
            this.Gauge_LF.Name = "Gauge_LF";
            this.Gauge_LF.Size = new System.Drawing.Size(50, 50);
            this.Gauge_LF.TabIndex = 160;
            this.Gauge_LF.Text = "solidGauge1";
            // 
            // lb_motor3
            // 
            this.lb_motor3.AutoSize = true;
            this.lb_motor3.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.lb_motor3.Location = new System.Drawing.Point(122, 137);
            this.lb_motor3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor3.Name = "lb_motor3";
            this.lb_motor3.Size = new System.Drawing.Size(15, 19);
            this.lb_motor3.TabIndex = 95;
            this.lb_motor3.Text = "-";
            // 
            // lb_motor2
            // 
            this.lb_motor2.AutoSize = true;
            this.lb_motor2.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.lb_motor2.Location = new System.Drawing.Point(118, 58);
            this.lb_motor2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor2.Name = "lb_motor2";
            this.lb_motor2.Size = new System.Drawing.Size(15, 19);
            this.lb_motor2.TabIndex = 94;
            this.lb_motor2.Text = "-";
            // 
            // lb_motor1
            // 
            this.lb_motor1.AutoSize = true;
            this.lb_motor1.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.lb_motor1.Location = new System.Drawing.Point(20, 137);
            this.lb_motor1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor1.Name = "lb_motor1";
            this.lb_motor1.Size = new System.Drawing.Size(15, 19);
            this.lb_motor1.TabIndex = 93;
            this.lb_motor1.Text = "-";
            // 
            // lb_motor0
            // 
            this.lb_motor0.AutoSize = true;
            this.lb_motor0.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.lb_motor0.Location = new System.Drawing.Point(24, 58);
            this.lb_motor0.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor0.Name = "lb_motor0";
            this.lb_motor0.Size = new System.Drawing.Size(15, 19);
            this.lb_motor0.TabIndex = 92;
            this.lb_motor0.Text = "-";
            // 
            // label28
            // 
            this.label28.AutoSize = true;
            this.label28.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.label28.Location = new System.Drawing.Point(116, 153);
            this.label28.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(73, 19);
            this.label28.TabIndex = 91;
            this.label28.Text = "MOTOR[3]";
            // 
            // label23
            // 
            this.label23.AutoSize = true;
            this.label23.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.label23.Location = new System.Drawing.Point(19, 152);
            this.label23.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(73, 19);
            this.label23.TabIndex = 90;
            this.label23.Text = "MOTOR[1]";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.label19.Location = new System.Drawing.Point(116, 71);
            this.label19.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(73, 19);
            this.label19.TabIndex = 89;
            this.label19.Text = "MOTOR[2]";
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Font = new System.Drawing.Font("Segoe UI", 10F);
            this.label16.Location = new System.Drawing.Point(19, 71);
            this.label16.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(73, 19);
            this.label16.TabIndex = 88;
            this.label16.Text = "MOTOR[0]";
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.label98);
            this.groupBox6.Controls.Add(this.lb_debug7);
            this.groupBox6.Controls.Add(this.label100);
            this.groupBox6.Controls.Add(this.lb_debug6);
            this.groupBox6.Controls.Add(this.label102);
            this.groupBox6.Controls.Add(this.lb_debug5);
            this.groupBox6.Controls.Add(this.label104);
            this.groupBox6.Controls.Add(this.lb_debug4);
            this.groupBox6.Controls.Add(this.label21);
            this.groupBox6.Controls.Add(this.lb_debug3);
            this.groupBox6.Controls.Add(this.label17);
            this.groupBox6.Controls.Add(this.lb_debug2);
            this.groupBox6.Controls.Add(this.label14);
            this.groupBox6.Controls.Add(this.lb_debug1);
            this.groupBox6.Controls.Add(this.label12);
            this.groupBox6.Controls.Add(this.lb_debug0);
            this.groupBox6.Location = new System.Drawing.Point(545, 26);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(165, 177);
            this.groupBox6.TabIndex = 86;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "DEBUG";
            // 
            // label98
            // 
            this.label98.AutoSize = true;
            this.label98.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label98.Location = new System.Drawing.Point(5, 152);
            this.label98.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label98.Name = "label98";
            this.label98.Size = new System.Drawing.Size(79, 21);
            this.label98.TabIndex = 101;
            this.label98.Text = "DEBUG[7]";
            // 
            // lb_debug7
            // 
            this.lb_debug7.AutoSize = true;
            this.lb_debug7.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug7.Location = new System.Drawing.Point(120, 152);
            this.lb_debug7.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug7.Name = "lb_debug7";
            this.lb_debug7.Size = new System.Drawing.Size(16, 21);
            this.lb_debug7.TabIndex = 100;
            this.lb_debug7.Text = "-";
            // 
            // label100
            // 
            this.label100.AutoSize = true;
            this.label100.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label100.Location = new System.Drawing.Point(5, 132);
            this.label100.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label100.Name = "label100";
            this.label100.Size = new System.Drawing.Size(79, 21);
            this.label100.TabIndex = 99;
            this.label100.Text = "DEBUG[6]";
            // 
            // lb_debug6
            // 
            this.lb_debug6.AutoSize = true;
            this.lb_debug6.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug6.Location = new System.Drawing.Point(120, 132);
            this.lb_debug6.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug6.Name = "lb_debug6";
            this.lb_debug6.Size = new System.Drawing.Size(16, 21);
            this.lb_debug6.TabIndex = 98;
            this.lb_debug6.Text = "-";
            // 
            // label102
            // 
            this.label102.AutoSize = true;
            this.label102.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label102.Location = new System.Drawing.Point(5, 112);
            this.label102.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label102.Name = "label102";
            this.label102.Size = new System.Drawing.Size(79, 21);
            this.label102.TabIndex = 97;
            this.label102.Text = "DEBUG[5]";
            // 
            // lb_debug5
            // 
            this.lb_debug5.AutoSize = true;
            this.lb_debug5.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug5.Location = new System.Drawing.Point(120, 112);
            this.lb_debug5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug5.Name = "lb_debug5";
            this.lb_debug5.Size = new System.Drawing.Size(16, 21);
            this.lb_debug5.TabIndex = 96;
            this.lb_debug5.Text = "-";
            // 
            // label104
            // 
            this.label104.AutoSize = true;
            this.label104.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label104.Location = new System.Drawing.Point(4, 92);
            this.label104.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label104.Name = "label104";
            this.label104.Size = new System.Drawing.Size(79, 21);
            this.label104.TabIndex = 95;
            this.label104.Text = "DEBUG[4]";
            // 
            // lb_debug4
            // 
            this.lb_debug4.AutoSize = true;
            this.lb_debug4.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug4.Location = new System.Drawing.Point(119, 92);
            this.lb_debug4.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug4.Name = "lb_debug4";
            this.lb_debug4.Size = new System.Drawing.Size(16, 21);
            this.lb_debug4.TabIndex = 94;
            this.lb_debug4.Text = "-";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label21.Location = new System.Drawing.Point(4, 72);
            this.label21.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(79, 21);
            this.label21.TabIndex = 93;
            this.label21.Text = "DEBUG[3]";
            // 
            // lb_debug3
            // 
            this.lb_debug3.AutoSize = true;
            this.lb_debug3.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug3.Location = new System.Drawing.Point(119, 73);
            this.lb_debug3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug3.Name = "lb_debug3";
            this.lb_debug3.Size = new System.Drawing.Size(16, 21);
            this.lb_debug3.TabIndex = 92;
            this.lb_debug3.Text = "-";
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label17.Location = new System.Drawing.Point(4, 52);
            this.label17.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(79, 21);
            this.label17.TabIndex = 91;
            this.label17.Text = "DEBUG[2]";
            // 
            // lb_debug2
            // 
            this.lb_debug2.AutoSize = true;
            this.lb_debug2.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug2.Location = new System.Drawing.Point(119, 52);
            this.lb_debug2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug2.Name = "lb_debug2";
            this.lb_debug2.Size = new System.Drawing.Size(16, 21);
            this.lb_debug2.TabIndex = 90;
            this.lb_debug2.Text = "-";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label14.Location = new System.Drawing.Point(4, 32);
            this.label14.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(79, 21);
            this.label14.TabIndex = 89;
            this.label14.Text = "DEBUG[1]";
            // 
            // lb_debug1
            // 
            this.lb_debug1.AutoSize = true;
            this.lb_debug1.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug1.Location = new System.Drawing.Point(119, 32);
            this.lb_debug1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug1.Name = "lb_debug1";
            this.lb_debug1.Size = new System.Drawing.Size(16, 21);
            this.lb_debug1.TabIndex = 88;
            this.lb_debug1.Text = "-";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label12.Location = new System.Drawing.Point(4, 13);
            this.label12.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(79, 21);
            this.label12.TabIndex = 87;
            this.label12.Text = "DEBUG[0]";
            // 
            // contextMenuStrip1
            // 
            this.contextMenuStrip1.Name = "contextMenuStrip1";
            this.contextMenuStrip1.Size = new System.Drawing.Size(61, 4);
            // 
            // tabControl1
            // 
            this.tabControl1.Controls.Add(this.tabPage1);
            this.tabControl1.Controls.Add(this.tabPage2);
            this.tabControl1.Controls.Add(this.tabPage3);
            this.tabControl1.Controls.Add(this.tabPage4);
            this.tabControl1.Controls.Add(this.tabPage5);
            this.tabControl1.Location = new System.Drawing.Point(351, 65);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(733, 595);
            this.tabControl1.TabIndex = 88;
            // 
            // tabPage1
            // 
            this.tabPage1.Controls.Add(this.groupBox10);
            this.tabPage1.Controls.Add(this.groupBox5);
            this.tabPage1.Controls.Add(this.groupBox6);
            this.tabPage1.Controls.Add(this.groupBox1);
            this.tabPage1.Location = new System.Drawing.Point(4, 22);
            this.tabPage1.Name = "tabPage1";
            this.tabPage1.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage1.Size = new System.Drawing.Size(725, 569);
            this.tabPage1.TabIndex = 0;
            this.tabPage1.Text = "PID 및 디버그";
            this.tabPage1.UseVisualStyleBackColor = true;
            // 
            // groupBox10
            // 
            this.groupBox10.Controls.Add(this.headingIndicatorInstrumentControl1);
            this.groupBox10.Controls.Add(this.attitudeIndicatorInstrumentControl1);
            this.groupBox10.Location = new System.Drawing.Point(11, 26);
            this.groupBox10.Name = "groupBox10";
            this.groupBox10.Size = new System.Drawing.Size(301, 177);
            this.groupBox10.TabIndex = 87;
            this.groupBox10.TabStop = false;
            this.groupBox10.Text = "ATTITUDE";
            // 
            // headingIndicatorInstrumentControl1
            // 
            this.headingIndicatorInstrumentControl1.Location = new System.Drawing.Point(153, 19);
            this.headingIndicatorInstrumentControl1.Name = "headingIndicatorInstrumentControl1";
            this.headingIndicatorInstrumentControl1.Size = new System.Drawing.Size(141, 142);
            this.headingIndicatorInstrumentControl1.TabIndex = 1;
            this.headingIndicatorInstrumentControl1.Text = "headingIndicatorInstrumentControl1";
            // 
            // attitudeIndicatorInstrumentControl1
            // 
            this.attitudeIndicatorInstrumentControl1.Location = new System.Drawing.Point(5, 18);
            this.attitudeIndicatorInstrumentControl1.Name = "attitudeIndicatorInstrumentControl1";
            this.attitudeIndicatorInstrumentControl1.Size = new System.Drawing.Size(141, 143);
            this.attitudeIndicatorInstrumentControl1.TabIndex = 0;
            this.attitudeIndicatorInstrumentControl1.Text = "attitudeIndicatorInstrumentControl1";
            // 
            // tabPage2
            // 
            this.tabPage2.Controls.Add(this.label62);
            this.tabPage2.Controls.Add(this.lb_route_distance);
            this.tabPage2.Controls.Add(this.label61);
            this.tabPage2.Controls.Add(this.gMapControl1);
            this.tabPage2.Location = new System.Drawing.Point(4, 22);
            this.tabPage2.Name = "tabPage2";
            this.tabPage2.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage2.Size = new System.Drawing.Size(725, 569);
            this.tabPage2.TabIndex = 1;
            this.tabPage2.Text = "MAP";
            this.tabPage2.UseVisualStyleBackColor = true;
            // 
            // label62
            // 
            this.label62.AutoSize = true;
            this.label62.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label62.Location = new System.Drawing.Point(178, 426);
            this.label62.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label62.Name = "label62";
            this.label62.Size = new System.Drawing.Size(33, 21);
            this.label62.TabIndex = 59;
            this.label62.Text = "Km";
            // 
            // lb_route_distance
            // 
            this.lb_route_distance.AutoSize = true;
            this.lb_route_distance.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_route_distance.Location = new System.Drawing.Point(132, 426);
            this.lb_route_distance.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_route_distance.Name = "lb_route_distance";
            this.lb_route_distance.Size = new System.Drawing.Size(16, 21);
            this.lb_route_distance.TabIndex = 58;
            this.lb_route_distance.Text = "-";
            // 
            // label61
            // 
            this.label61.AutoSize = true;
            this.label61.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label61.Location = new System.Drawing.Point(7, 426);
            this.label61.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label61.Name = "label61";
            this.label61.Size = new System.Drawing.Size(117, 21);
            this.label61.TabIndex = 57;
            this.label61.Text = "Route_Distance";
            // 
            // gMapControl1
            // 
            this.gMapControl1.Bearing = 0F;
            this.gMapControl1.CanDragMap = true;
            this.gMapControl1.EmptyTileColor = System.Drawing.Color.Navy;
            this.gMapControl1.GrayScaleMode = false;
            this.gMapControl1.HelperLineOption = GMap.NET.WindowsForms.HelperLineOptions.DontShow;
            this.gMapControl1.LevelsKeepInMemory = 5;
            this.gMapControl1.Location = new System.Drawing.Point(7, 7);
            this.gMapControl1.MarkersEnabled = true;
            this.gMapControl1.MaxZoom = 2;
            this.gMapControl1.MinZoom = 2;
            this.gMapControl1.MouseWheelZoomEnabled = true;
            this.gMapControl1.MouseWheelZoomType = GMap.NET.MouseWheelZoomType.MousePositionAndCenter;
            this.gMapControl1.Name = "gMapControl1";
            this.gMapControl1.NegativeMode = false;
            this.gMapControl1.PolygonsEnabled = true;
            this.gMapControl1.RetryLoadTile = 0;
            this.gMapControl1.RoutesEnabled = true;
            this.gMapControl1.ScaleMode = GMap.NET.WindowsForms.ScaleModes.Integer;
            this.gMapControl1.SelectedAreaFillColor = System.Drawing.Color.FromArgb(((int)(((byte)(33)))), ((int)(((byte)(65)))), ((int)(((byte)(105)))), ((int)(((byte)(225)))));
            this.gMapControl1.ShowTileGridLines = false;
            this.gMapControl1.Size = new System.Drawing.Size(712, 409);
            this.gMapControl1.TabIndex = 0;
            this.gMapControl1.Zoom = 0D;
            this.gMapControl1.OnMarkerClick += new GMap.NET.WindowsForms.MarkerClick(this.gMapControl1_OnMarkerClick);
            this.gMapControl1.OnRouteClick += new GMap.NET.WindowsForms.RouteClick(this.gMapControl1_OnRouteClick);
            this.gMapControl1.MouseClick += new System.Windows.Forms.MouseEventHandler(this.gMapControl1_MouseClick);
            // 
            // tabPage3
            // 
            this.tabPage3.Controls.Add(this.comboBox_Debug);
            this.tabPage3.Controls.Add(this.rb_alt_range_setpoint);
            this.tabPage3.Controls.Add(this.rb_alt_setpoint);
            this.tabPage3.Controls.Add(this.lb_PID_Test_Target_Time);
            this.tabPage3.Controls.Add(this.lb_PID_Test_Progress_Time);
            this.tabPage3.Controls.Add(this.label86);
            this.tabPage3.Controls.Add(this.label85);
            this.tabPage3.Controls.Add(this.bt_open_folder);
            this.tabPage3.Controls.Add(this.lb_PID_Test_Progress);
            this.tabPage3.Controls.Add(this.lb_PID_Test_Status);
            this.tabPage3.Controls.Add(this.bt_start_pid_test);
            this.tabPage3.Controls.Add(this.label84);
            this.tabPage3.Controls.Add(this.label83);
            this.tabPage3.Controls.Add(this.label80);
            this.tabPage3.Controls.Add(this.tb_PID_ms);
            this.tabPage3.Controls.Add(this.tb_PID_Deg);
            this.tabPage3.Controls.Add(this.tb_PID_Throttle);
            this.tabPage3.Controls.Add(this.panel8);
            this.tabPage3.Controls.Add(this.cb_record);
            this.tabPage3.Controls.Add(this.rb_debug);
            this.tabPage3.Controls.Add(this.rb_motor);
            this.tabPage3.Controls.Add(this.rb_gyro);
            this.tabPage3.Controls.Add(this.rb_altitude);
            this.tabPage3.Controls.Add(this.cb_autoscale);
            this.tabPage3.Controls.Add(this.rb_none);
            this.tabPage3.Controls.Add(this.rb_yaw_setpoint);
            this.tabPage3.Controls.Add(this.rb_pitch_setpoint);
            this.tabPage3.Controls.Add(this.rb_roll_setpoint);
            this.tabPage3.Controls.Add(this.rb_roll_pitch);
            this.tabPage3.Controls.Add(this.rb_yaw);
            this.tabPage3.Controls.Add(this.rb_pitch);
            this.tabPage3.Controls.Add(this.rb_roll);
            this.tabPage3.Controls.Add(this.zedGraphControl1);
            this.tabPage3.Location = new System.Drawing.Point(4, 22);
            this.tabPage3.Name = "tabPage3";
            this.tabPage3.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage3.Size = new System.Drawing.Size(725, 569);
            this.tabPage3.TabIndex = 2;
            this.tabPage3.Text = "그래프";
            this.tabPage3.UseVisualStyleBackColor = true;
            // 
            // comboBox_Debug
            // 
            this.comboBox_Debug.FormattingEnabled = true;
            this.comboBox_Debug.Location = new System.Drawing.Point(101, 510);
            this.comboBox_Debug.Name = "comboBox_Debug";
            this.comboBox_Debug.Size = new System.Drawing.Size(121, 20);
            this.comboBox_Debug.TabIndex = 87;
            this.comboBox_Debug.SelectedIndexChanged += new System.EventHandler(this.comboBox_Debug_SelectedIndexChanged);
            // 
            // rb_alt_range_setpoint
            // 
            this.rb_alt_range_setpoint.AutoSize = true;
            this.rb_alt_range_setpoint.Location = new System.Drawing.Point(112, 488);
            this.rb_alt_range_setpoint.Name = "rb_alt_range_setpoint";
            this.rb_alt_range_setpoint.Size = new System.Drawing.Size(138, 16);
            this.rb_alt_range_setpoint.TabIndex = 86;
            this.rb_alt_range_setpoint.Text = "Alt_Range / Setpoint";
            this.rb_alt_range_setpoint.UseVisualStyleBackColor = true;
            this.rb_alt_range_setpoint.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_alt_range_setpoint_MouseDown);
            // 
            // rb_alt_setpoint
            // 
            this.rb_alt_setpoint.AutoSize = true;
            this.rb_alt_setpoint.Location = new System.Drawing.Point(112, 466);
            this.rb_alt_setpoint.Name = "rb_alt_setpoint";
            this.rb_alt_setpoint.Size = new System.Drawing.Size(96, 16);
            this.rb_alt_setpoint.TabIndex = 85;
            this.rb_alt_setpoint.Text = "Alt / Setpoint";
            this.rb_alt_setpoint.UseVisualStyleBackColor = true;
            this.rb_alt_setpoint.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_alt_setpoint_MouseDown);
            // 
            // lb_PID_Test_Target_Time
            // 
            this.lb_PID_Test_Target_Time.AutoSize = true;
            this.lb_PID_Test_Target_Time.Location = new System.Drawing.Point(499, 494);
            this.lb_PID_Test_Target_Time.Name = "lb_PID_Test_Target_Time";
            this.lb_PID_Test_Target_Time.Size = new System.Drawing.Size(11, 12);
            this.lb_PID_Test_Target_Time.TabIndex = 84;
            this.lb_PID_Test_Target_Time.Text = "-";
            // 
            // lb_PID_Test_Progress_Time
            // 
            this.lb_PID_Test_Progress_Time.AutoSize = true;
            this.lb_PID_Test_Progress_Time.Location = new System.Drawing.Point(499, 472);
            this.lb_PID_Test_Progress_Time.Name = "lb_PID_Test_Progress_Time";
            this.lb_PID_Test_Progress_Time.Size = new System.Drawing.Size(11, 12);
            this.lb_PID_Test_Progress_Time.TabIndex = 83;
            this.lb_PID_Test_Progress_Time.Text = "-";
            // 
            // label86
            // 
            this.label86.AutoSize = true;
            this.label86.Location = new System.Drawing.Point(323, 494);
            this.label86.Name = "label86";
            this.label86.Size = new System.Drawing.Size(144, 12);
            this.label86.TabIndex = 82;
            this.label86.Text = "PID_Test_Target_Time : ";
            // 
            // label85
            // 
            this.label85.AutoSize = true;
            this.label85.Location = new System.Drawing.Point(323, 473);
            this.label85.Name = "label85";
            this.label85.Size = new System.Drawing.Size(159, 12);
            this.label85.TabIndex = 81;
            this.label85.Text = "PID_Test_Progress_Time : ";
            // 
            // bt_open_folder
            // 
            this.bt_open_folder.Location = new System.Drawing.Point(547, 479);
            this.bt_open_folder.Name = "bt_open_folder";
            this.bt_open_folder.Size = new System.Drawing.Size(95, 23);
            this.bt_open_folder.TabIndex = 80;
            this.bt_open_folder.Text = "Open Folder";
            this.bt_open_folder.UseVisualStyleBackColor = true;
            this.bt_open_folder.Click += new System.EventHandler(this.bt_open_folder_Click);
            // 
            // lb_PID_Test_Progress
            // 
            this.lb_PID_Test_Progress.AutoSize = true;
            this.lb_PID_Test_Progress.Location = new System.Drawing.Point(545, 447);
            this.lb_PID_Test_Progress.Name = "lb_PID_Test_Progress";
            this.lb_PID_Test_Progress.Size = new System.Drawing.Size(108, 12);
            this.lb_PID_Test_Progress.TabIndex = 79;
            this.lb_PID_Test_Progress.Text = "PID Test Progress";
            // 
            // lb_PID_Test_Status
            // 
            this.lb_PID_Test_Status.AutoSize = true;
            this.lb_PID_Test_Status.Location = new System.Drawing.Point(545, 425);
            this.lb_PID_Test_Status.Name = "lb_PID_Test_Status";
            this.lb_PID_Test_Status.Size = new System.Drawing.Size(92, 12);
            this.lb_PID_Test_Status.TabIndex = 78;
            this.lb_PID_Test_Status.Text = "PID Test Status";
            // 
            // bt_start_pid_test
            // 
            this.bt_start_pid_test.Location = new System.Drawing.Point(544, 392);
            this.bt_start_pid_test.Name = "bt_start_pid_test";
            this.bt_start_pid_test.Size = new System.Drawing.Size(98, 23);
            this.bt_start_pid_test.TabIndex = 77;
            this.bt_start_pid_test.Text = "Start PID Test";
            this.bt_start_pid_test.UseVisualStyleBackColor = true;
            this.bt_start_pid_test.Click += new System.EventHandler(this.bt_start_pid_test_Click);
            // 
            // label84
            // 
            this.label84.AutoSize = true;
            this.label84.Location = new System.Drawing.Point(485, 444);
            this.label84.Name = "label84";
            this.label84.Size = new System.Drawing.Size(23, 12);
            this.label84.TabIndex = 76;
            this.label84.Text = "ms";
            // 
            // label83
            // 
            this.label83.AutoSize = true;
            this.label83.Location = new System.Drawing.Point(485, 422);
            this.label83.Name = "label83";
            this.label83.Size = new System.Drawing.Size(27, 12);
            this.label83.TabIndex = 75;
            this.label83.Text = "Deg";
            // 
            // label80
            // 
            this.label80.AutoSize = true;
            this.label80.Location = new System.Drawing.Point(485, 400);
            this.label80.Name = "label80";
            this.label80.Size = new System.Drawing.Size(47, 12);
            this.label80.TabIndex = 74;
            this.label80.Text = "Throttle";
            // 
            // tb_PID_ms
            // 
            this.tb_PID_ms.Location = new System.Drawing.Point(440, 441);
            this.tb_PID_ms.Name = "tb_PID_ms";
            this.tb_PID_ms.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.tb_PID_ms.Size = new System.Drawing.Size(42, 21);
            this.tb_PID_ms.TabIndex = 73;
            this.tb_PID_ms.Text = "2000";
            // 
            // tb_PID_Deg
            // 
            this.tb_PID_Deg.Location = new System.Drawing.Point(440, 418);
            this.tb_PID_Deg.Name = "tb_PID_Deg";
            this.tb_PID_Deg.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.tb_PID_Deg.Size = new System.Drawing.Size(42, 21);
            this.tb_PID_Deg.TabIndex = 72;
            this.tb_PID_Deg.Text = "20";
            // 
            // tb_PID_Throttle
            // 
            this.tb_PID_Throttle.Location = new System.Drawing.Point(440, 395);
            this.tb_PID_Throttle.Name = "tb_PID_Throttle";
            this.tb_PID_Throttle.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.tb_PID_Throttle.Size = new System.Drawing.Size(42, 21);
            this.tb_PID_Throttle.TabIndex = 71;
            this.tb_PID_Throttle.Text = "1200";
            // 
            // panel8
            // 
            this.panel8.Location = new System.Drawing.Point(621, 19);
            this.panel8.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel8.Name = "panel8";
            this.panel8.Size = new System.Drawing.Size(18, 18);
            this.panel8.TabIndex = 70;
            // 
            // cb_record
            // 
            this.cb_record.AutoSize = true;
            this.cb_record.Location = new System.Drawing.Point(514, 22);
            this.cb_record.Name = "cb_record";
            this.cb_record.Size = new System.Drawing.Size(100, 16);
            this.cb_record.TabIndex = 13;
            this.cb_record.Text = "Excel Record";
            this.cb_record.UseVisualStyleBackColor = true;
            this.cb_record.CheckedChanged += new System.EventHandler(this.cb_record_CheckedChanged);
            // 
            // rb_debug
            // 
            this.rb_debug.AutoSize = true;
            this.rb_debug.Location = new System.Drawing.Point(13, 511);
            this.rb_debug.Name = "rb_debug";
            this.rb_debug.Size = new System.Drawing.Size(59, 16);
            this.rb_debug.TabIndex = 12;
            this.rb_debug.Text = "Debug";
            this.rb_debug.UseVisualStyleBackColor = true;
            this.rb_debug.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_debug_MouseDown);
            // 
            // rb_motor
            // 
            this.rb_motor.AutoSize = true;
            this.rb_motor.Location = new System.Drawing.Point(237, 444);
            this.rb_motor.Name = "rb_motor";
            this.rb_motor.Size = new System.Drawing.Size(55, 16);
            this.rb_motor.TabIndex = 11;
            this.rb_motor.Text = "Motor";
            this.rb_motor.UseVisualStyleBackColor = true;
            this.rb_motor.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_motor_MouseDown);
            // 
            // rb_gyro
            // 
            this.rb_gyro.AutoSize = true;
            this.rb_gyro.Location = new System.Drawing.Point(237, 422);
            this.rb_gyro.Name = "rb_gyro";
            this.rb_gyro.Size = new System.Drawing.Size(50, 16);
            this.rb_gyro.TabIndex = 10;
            this.rb_gyro.Text = "Gyro";
            this.rb_gyro.UseVisualStyleBackColor = true;
            this.rb_gyro.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_gyro_MouseDown);
            // 
            // rb_altitude
            // 
            this.rb_altitude.AutoSize = true;
            this.rb_altitude.Location = new System.Drawing.Point(237, 400);
            this.rb_altitude.Name = "rb_altitude";
            this.rb_altitude.Size = new System.Drawing.Size(64, 16);
            this.rb_altitude.TabIndex = 9;
            this.rb_altitude.Text = "Altitude";
            this.rb_altitude.UseVisualStyleBackColor = true;
            this.rb_altitude.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_altitude_MouseDown);
            // 
            // cb_autoscale
            // 
            this.cb_autoscale.AutoSize = true;
            this.cb_autoscale.Location = new System.Drawing.Point(325, 401);
            this.cb_autoscale.Name = "cb_autoscale";
            this.cb_autoscale.Size = new System.Drawing.Size(85, 16);
            this.cb_autoscale.TabIndex = 8;
            this.cb_autoscale.Text = "Auto Scale";
            this.cb_autoscale.UseVisualStyleBackColor = true;
            // 
            // rb_none
            // 
            this.rb_none.AutoSize = true;
            this.rb_none.Location = new System.Drawing.Point(13, 533);
            this.rb_none.Name = "rb_none";
            this.rb_none.Size = new System.Drawing.Size(53, 16);
            this.rb_none.TabIndex = 7;
            this.rb_none.Text = "None";
            this.rb_none.UseVisualStyleBackColor = true;
            this.rb_none.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_none_MouseDown);
            // 
            // rb_yaw_setpoint
            // 
            this.rb_yaw_setpoint.AutoSize = true;
            this.rb_yaw_setpoint.Location = new System.Drawing.Point(112, 444);
            this.rb_yaw_setpoint.Name = "rb_yaw_setpoint";
            this.rb_yaw_setpoint.Size = new System.Drawing.Size(107, 16);
            this.rb_yaw_setpoint.TabIndex = 6;
            this.rb_yaw_setpoint.Text = "Yaw / Setpoint";
            this.rb_yaw_setpoint.UseVisualStyleBackColor = true;
            this.rb_yaw_setpoint.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_yaw_setpoint_MouseDown);
            // 
            // rb_pitch_setpoint
            // 
            this.rb_pitch_setpoint.AutoSize = true;
            this.rb_pitch_setpoint.Location = new System.Drawing.Point(112, 422);
            this.rb_pitch_setpoint.Name = "rb_pitch_setpoint";
            this.rb_pitch_setpoint.Size = new System.Drawing.Size(110, 16);
            this.rb_pitch_setpoint.TabIndex = 5;
            this.rb_pitch_setpoint.Text = "Pitch / Setpoint";
            this.rb_pitch_setpoint.UseVisualStyleBackColor = true;
            this.rb_pitch_setpoint.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_pitch_setpoint_MouseDown);
            // 
            // rb_roll_setpoint
            // 
            this.rb_roll_setpoint.AutoSize = true;
            this.rb_roll_setpoint.Location = new System.Drawing.Point(112, 400);
            this.rb_roll_setpoint.Name = "rb_roll_setpoint";
            this.rb_roll_setpoint.Size = new System.Drawing.Size(103, 16);
            this.rb_roll_setpoint.TabIndex = 4;
            this.rb_roll_setpoint.Text = "Roll / Setpoint";
            this.rb_roll_setpoint.UseVisualStyleBackColor = true;
            this.rb_roll_setpoint.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_roll_setpoint_MouseDown);
            // 
            // rb_roll_pitch
            // 
            this.rb_roll_pitch.AutoSize = true;
            this.rb_roll_pitch.Location = new System.Drawing.Point(13, 466);
            this.rb_roll_pitch.Name = "rb_roll_pitch";
            this.rb_roll_pitch.Size = new System.Drawing.Size(86, 16);
            this.rb_roll_pitch.TabIndex = 3;
            this.rb_roll_pitch.Text = "Roll / Pitch";
            this.rb_roll_pitch.UseVisualStyleBackColor = true;
            this.rb_roll_pitch.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_roll_pitch_MouseDown);
            // 
            // rb_yaw
            // 
            this.rb_yaw.AutoSize = true;
            this.rb_yaw.Location = new System.Drawing.Point(13, 444);
            this.rb_yaw.Name = "rb_yaw";
            this.rb_yaw.Size = new System.Drawing.Size(48, 16);
            this.rb_yaw.TabIndex = 2;
            this.rb_yaw.Text = "Yaw";
            this.rb_yaw.UseVisualStyleBackColor = true;
            this.rb_yaw.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_yaw_MouseDown);
            // 
            // rb_pitch
            // 
            this.rb_pitch.AutoSize = true;
            this.rb_pitch.Location = new System.Drawing.Point(13, 422);
            this.rb_pitch.Name = "rb_pitch";
            this.rb_pitch.Size = new System.Drawing.Size(51, 16);
            this.rb_pitch.TabIndex = 1;
            this.rb_pitch.Text = "Pitch";
            this.rb_pitch.UseVisualStyleBackColor = true;
            this.rb_pitch.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_pitch_MouseDown);
            // 
            // rb_roll
            // 
            this.rb_roll.AutoSize = true;
            this.rb_roll.Checked = true;
            this.rb_roll.Location = new System.Drawing.Point(13, 400);
            this.rb_roll.Name = "rb_roll";
            this.rb_roll.Size = new System.Drawing.Size(44, 16);
            this.rb_roll.TabIndex = 0;
            this.rb_roll.TabStop = true;
            this.rb_roll.Text = "Roll";
            this.rb_roll.UseVisualStyleBackColor = true;
            this.rb_roll.MouseDown += new System.Windows.Forms.MouseEventHandler(this.rb_roll_MouseDown);
            // 
            // zedGraphControl1
            // 
            this.zedGraphControl1.Location = new System.Drawing.Point(7, 6);
            this.zedGraphControl1.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.zedGraphControl1.Name = "zedGraphControl1";
            this.zedGraphControl1.ScrollGrace = 0D;
            this.zedGraphControl1.ScrollMaxX = 0D;
            this.zedGraphControl1.ScrollMaxY = 0D;
            this.zedGraphControl1.ScrollMaxY2 = 0D;
            this.zedGraphControl1.ScrollMinX = 0D;
            this.zedGraphControl1.ScrollMinY = 0D;
            this.zedGraphControl1.ScrollMinY2 = 0D;
            this.zedGraphControl1.Size = new System.Drawing.Size(711, 380);
            this.zedGraphControl1.TabIndex = 0;
            this.zedGraphControl1.UseExtendedPrintDialog = true;
            // 
            // tabPage4
            // 
            this.tabPage4.Controls.Add(this.groupBox9);
            this.tabPage4.Controls.Add(this.groupBox8);
            this.tabPage4.Controls.Add(this.groupBox7);
            this.tabPage4.Controls.Add(this.bt_mag_cal);
            this.tabPage4.Controls.Add(this.bt_acc_cal);
            this.tabPage4.Location = new System.Drawing.Point(4, 22);
            this.tabPage4.Name = "tabPage4";
            this.tabPage4.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage4.Size = new System.Drawing.Size(725, 569);
            this.tabPage4.TabIndex = 3;
            this.tabPage4.Text = "캘리브레이션";
            this.tabPage4.UseVisualStyleBackColor = true;
            // 
            // groupBox9
            // 
            this.groupBox9.Controls.Add(this.label69);
            this.groupBox9.Controls.Add(this.label76);
            this.groupBox9.Controls.Add(this.lb_bodyrate_Y);
            this.groupBox9.Controls.Add(this.lb_bodyrate_X);
            this.groupBox9.Controls.Add(this.label72);
            this.groupBox9.Controls.Add(this.label73);
            this.groupBox9.Controls.Add(this.lb_flowrate_Y);
            this.groupBox9.Controls.Add(this.lb_flowrate_X);
            this.groupBox9.Location = new System.Drawing.Point(6, 259);
            this.groupBox9.Name = "groupBox9";
            this.groupBox9.Size = new System.Drawing.Size(335, 177);
            this.groupBox9.TabIndex = 154;
            this.groupBox9.TabStop = false;
            this.groupBox9.Text = "옵티컬플로우";
            // 
            // label69
            // 
            this.label69.AutoSize = true;
            this.label69.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label69.Location = new System.Drawing.Point(174, 134);
            this.label69.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label69.Name = "label69";
            this.label69.Size = new System.Drawing.Size(95, 21);
            this.label69.TabIndex = 100;
            this.label69.Text = "BodyRate[Y]";
            // 
            // label76
            // 
            this.label76.AutoSize = true;
            this.label76.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label76.Location = new System.Drawing.Point(56, 134);
            this.label76.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label76.Name = "label76";
            this.label76.Size = new System.Drawing.Size(95, 21);
            this.label76.TabIndex = 99;
            this.label76.Text = "BodyRate[X]";
            // 
            // lb_bodyrate_Y
            // 
            this.lb_bodyrate_Y.AutoSize = true;
            this.lb_bodyrate_Y.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_bodyrate_Y.Location = new System.Drawing.Point(211, 107);
            this.lb_bodyrate_Y.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_bodyrate_Y.Name = "lb_bodyrate_Y";
            this.lb_bodyrate_Y.Size = new System.Drawing.Size(16, 21);
            this.lb_bodyrate_Y.TabIndex = 98;
            this.lb_bodyrate_Y.Text = "-";
            // 
            // lb_bodyrate_X
            // 
            this.lb_bodyrate_X.AutoSize = true;
            this.lb_bodyrate_X.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_bodyrate_X.Location = new System.Drawing.Point(92, 107);
            this.lb_bodyrate_X.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_bodyrate_X.Name = "lb_bodyrate_X";
            this.lb_bodyrate_X.Size = new System.Drawing.Size(16, 21);
            this.lb_bodyrate_X.TabIndex = 97;
            this.lb_bodyrate_X.Text = "-";
            // 
            // label72
            // 
            this.label72.AutoSize = true;
            this.label72.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label72.Location = new System.Drawing.Point(178, 60);
            this.label72.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label72.Name = "label72";
            this.label72.Size = new System.Drawing.Size(93, 21);
            this.label72.TabIndex = 96;
            this.label72.Text = "FlowRate[Y]";
            // 
            // label73
            // 
            this.label73.AutoSize = true;
            this.label73.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label73.Location = new System.Drawing.Point(58, 60);
            this.label73.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label73.Name = "label73";
            this.label73.Size = new System.Drawing.Size(93, 21);
            this.label73.TabIndex = 95;
            this.label73.Text = "FlowRate[X]";
            // 
            // lb_flowrate_Y
            // 
            this.lb_flowrate_Y.AutoSize = true;
            this.lb_flowrate_Y.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_flowrate_Y.Location = new System.Drawing.Point(211, 33);
            this.lb_flowrate_Y.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_flowrate_Y.Name = "lb_flowrate_Y";
            this.lb_flowrate_Y.Size = new System.Drawing.Size(16, 21);
            this.lb_flowrate_Y.TabIndex = 94;
            this.lb_flowrate_Y.Text = "-";
            // 
            // lb_flowrate_X
            // 
            this.lb_flowrate_X.AutoSize = true;
            this.lb_flowrate_X.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_flowrate_X.Location = new System.Drawing.Point(92, 33);
            this.lb_flowrate_X.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_flowrate_X.Name = "lb_flowrate_X";
            this.lb_flowrate_X.Size = new System.Drawing.Size(16, 21);
            this.lb_flowrate_X.TabIndex = 92;
            this.lb_flowrate_X.Text = "-";
            // 
            // groupBox8
            // 
            this.groupBox8.Controls.Add(this.label67);
            this.groupBox8.Controls.Add(this.label68);
            this.groupBox8.Controls.Add(this.lb_mag_Z);
            this.groupBox8.Controls.Add(this.lb_mag_Y);
            this.groupBox8.Controls.Add(this.lb_mag_X);
            this.groupBox8.Controls.Add(this.label74);
            this.groupBox8.Controls.Add(this.lb_magcal_remain_time);
            this.groupBox8.Controls.Add(this.label66);
            this.groupBox8.Controls.Add(this.label71);
            this.groupBox8.Controls.Add(this.label70);
            this.groupBox8.Controls.Add(this.lb_magZero_Z);
            this.groupBox8.Controls.Add(this.lb_magZero_Y);
            this.groupBox8.Controls.Add(this.lb_magZero_X);
            this.groupBox8.Controls.Add(this.label78);
            this.groupBox8.Location = new System.Drawing.Point(361, 12);
            this.groupBox8.Name = "groupBox8";
            this.groupBox8.Size = new System.Drawing.Size(335, 177);
            this.groupBox8.TabIndex = 153;
            this.groupBox8.TabStop = false;
            this.groupBox8.Text = "지자계";
            // 
            // label67
            // 
            this.label67.AutoSize = true;
            this.label67.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label67.Location = new System.Drawing.Point(249, 46);
            this.label67.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label67.Name = "label67";
            this.label67.Size = new System.Drawing.Size(64, 21);
            this.label67.TabIndex = 105;
            this.label67.Text = "MAG[Z]";
            // 
            // label68
            // 
            this.label68.AutoSize = true;
            this.label68.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label68.Location = new System.Drawing.Point(128, 46);
            this.label68.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label68.Name = "label68";
            this.label68.Size = new System.Drawing.Size(64, 21);
            this.label68.TabIndex = 104;
            this.label68.Text = "MAG[Y]";
            // 
            // lb_mag_Z
            // 
            this.lb_mag_Z.AutoSize = true;
            this.lb_mag_Z.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_mag_Z.Location = new System.Drawing.Point(273, 25);
            this.lb_mag_Z.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_mag_Z.Name = "lb_mag_Z";
            this.lb_mag_Z.Size = new System.Drawing.Size(16, 21);
            this.lb_mag_Z.TabIndex = 103;
            this.lb_mag_Z.Text = "-";
            // 
            // lb_mag_Y
            // 
            this.lb_mag_Y.AutoSize = true;
            this.lb_mag_Y.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_mag_Y.Location = new System.Drawing.Point(152, 21);
            this.lb_mag_Y.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_mag_Y.Name = "lb_mag_Y";
            this.lb_mag_Y.Size = new System.Drawing.Size(16, 21);
            this.lb_mag_Y.TabIndex = 102;
            this.lb_mag_Y.Text = "-";
            // 
            // lb_mag_X
            // 
            this.lb_mag_X.AutoSize = true;
            this.lb_mag_X.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_mag_X.Location = new System.Drawing.Point(48, 21);
            this.lb_mag_X.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_mag_X.Name = "lb_mag_X";
            this.lb_mag_X.Size = new System.Drawing.Size(16, 21);
            this.lb_mag_X.TabIndex = 101;
            this.lb_mag_X.Text = "-";
            // 
            // label74
            // 
            this.label74.AutoSize = true;
            this.label74.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label74.Location = new System.Drawing.Point(29, 46);
            this.label74.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label74.Name = "label74";
            this.label74.Size = new System.Drawing.Size(64, 21);
            this.label74.TabIndex = 100;
            this.label74.Text = "MAG[X]";
            // 
            // lb_magcal_remain_time
            // 
            this.lb_magcal_remain_time.AutoSize = true;
            this.lb_magcal_remain_time.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_magcal_remain_time.Location = new System.Drawing.Point(231, 140);
            this.lb_magcal_remain_time.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_magcal_remain_time.Name = "lb_magcal_remain_time";
            this.lb_magcal_remain_time.Size = new System.Drawing.Size(16, 21);
            this.lb_magcal_remain_time.TabIndex = 99;
            this.lb_magcal_remain_time.Text = "-";
            // 
            // label66
            // 
            this.label66.AutoSize = true;
            this.label66.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label66.Location = new System.Drawing.Point(7, 140);
            this.label66.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label66.Name = "label66";
            this.label66.Size = new System.Drawing.Size(185, 21);
            this.label66.TabIndex = 98;
            this.label66.Text = "캘리브레이션 남은 시간 :";
            // 
            // label71
            // 
            this.label71.AutoSize = true;
            this.label71.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label71.Location = new System.Drawing.Point(249, 103);
            this.label71.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label71.Name = "label71";
            this.label71.Size = new System.Drawing.Size(68, 21);
            this.label71.TabIndex = 97;
            this.label71.Text = "ZERO[Z]";
            // 
            // label70
            // 
            this.label70.AutoSize = true;
            this.label70.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label70.Location = new System.Drawing.Point(124, 103);
            this.label70.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label70.Name = "label70";
            this.label70.Size = new System.Drawing.Size(68, 21);
            this.label70.TabIndex = 96;
            this.label70.Text = "ZERO[Y]";
            // 
            // lb_magZero_Z
            // 
            this.lb_magZero_Z.AutoSize = true;
            this.lb_magZero_Z.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_magZero_Z.Location = new System.Drawing.Point(273, 82);
            this.lb_magZero_Z.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_magZero_Z.Name = "lb_magZero_Z";
            this.lb_magZero_Z.Size = new System.Drawing.Size(16, 21);
            this.lb_magZero_Z.TabIndex = 95;
            this.lb_magZero_Z.Text = "-";
            // 
            // lb_magZero_Y
            // 
            this.lb_magZero_Y.AutoSize = true;
            this.lb_magZero_Y.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_magZero_Y.Location = new System.Drawing.Point(152, 78);
            this.lb_magZero_Y.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_magZero_Y.Name = "lb_magZero_Y";
            this.lb_magZero_Y.Size = new System.Drawing.Size(16, 21);
            this.lb_magZero_Y.TabIndex = 94;
            this.lb_magZero_Y.Text = "-";
            // 
            // lb_magZero_X
            // 
            this.lb_magZero_X.AutoSize = true;
            this.lb_magZero_X.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_magZero_X.Location = new System.Drawing.Point(48, 78);
            this.lb_magZero_X.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_magZero_X.Name = "lb_magZero_X";
            this.lb_magZero_X.Size = new System.Drawing.Size(16, 21);
            this.lb_magZero_X.TabIndex = 92;
            this.lb_magZero_X.Text = "-";
            // 
            // label78
            // 
            this.label78.AutoSize = true;
            this.label78.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label78.Location = new System.Drawing.Point(29, 103);
            this.label78.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label78.Name = "label78";
            this.label78.Size = new System.Drawing.Size(68, 21);
            this.label78.TabIndex = 88;
            this.label78.Text = "ZERO[X]";
            // 
            // groupBox7
            // 
            this.groupBox7.Controls.Add(this.label65);
            this.groupBox7.Controls.Add(this.label63);
            this.groupBox7.Controls.Add(this.label64);
            this.groupBox7.Controls.Add(this.lb_accTrim_Y);
            this.groupBox7.Controls.Add(this.lb_accTrim_Z);
            this.groupBox7.Controls.Add(this.lb_accTrim_X);
            this.groupBox7.Location = new System.Drawing.Point(6, 12);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Size = new System.Drawing.Size(335, 177);
            this.groupBox7.TabIndex = 152;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "가속도계";
            // 
            // label65
            // 
            this.label65.AutoSize = true;
            this.label65.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label65.Location = new System.Drawing.Point(258, 65);
            this.label65.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label65.Name = "label65";
            this.label65.Size = new System.Drawing.Size(60, 21);
            this.label65.TabIndex = 97;
            this.label65.Text = "Trim[Z]";
            // 
            // label63
            // 
            this.label63.AutoSize = true;
            this.label63.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label63.Location = new System.Drawing.Point(135, 65);
            this.label63.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label63.Name = "label63";
            this.label63.Size = new System.Drawing.Size(60, 21);
            this.label63.TabIndex = 96;
            this.label63.Text = "Trim[Y]";
            // 
            // label64
            // 
            this.label64.AutoSize = true;
            this.label64.Font = new System.Drawing.Font("Segoe UI", 12F);
            this.label64.Location = new System.Drawing.Point(19, 65);
            this.label64.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label64.Name = "label64";
            this.label64.Size = new System.Drawing.Size(60, 21);
            this.label64.TabIndex = 95;
            this.label64.Text = "Trim[X]";
            // 
            // lb_accTrim_Y
            // 
            this.lb_accTrim_Y.AutoSize = true;
            this.lb_accTrim_Y.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_accTrim_Y.Location = new System.Drawing.Point(159, 38);
            this.lb_accTrim_Y.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_accTrim_Y.Name = "lb_accTrim_Y";
            this.lb_accTrim_Y.Size = new System.Drawing.Size(16, 21);
            this.lb_accTrim_Y.TabIndex = 94;
            this.lb_accTrim_Y.Text = "-";
            // 
            // lb_accTrim_Z
            // 
            this.lb_accTrim_Z.AutoSize = true;
            this.lb_accTrim_Z.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_accTrim_Z.Location = new System.Drawing.Point(276, 38);
            this.lb_accTrim_Z.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_accTrim_Z.Name = "lb_accTrim_Z";
            this.lb_accTrim_Z.Size = new System.Drawing.Size(16, 21);
            this.lb_accTrim_Z.TabIndex = 93;
            this.lb_accTrim_Z.Text = "-";
            // 
            // lb_accTrim_X
            // 
            this.lb_accTrim_X.AutoSize = true;
            this.lb_accTrim_X.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_accTrim_X.Location = new System.Drawing.Point(40, 38);
            this.lb_accTrim_X.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_accTrim_X.Name = "lb_accTrim_X";
            this.lb_accTrim_X.Size = new System.Drawing.Size(16, 21);
            this.lb_accTrim_X.TabIndex = 92;
            this.lb_accTrim_X.Text = "-";
            // 
            // bt_mag_cal
            // 
            this.bt_mag_cal.Location = new System.Drawing.Point(450, 205);
            this.bt_mag_cal.Name = "bt_mag_cal";
            this.bt_mag_cal.Size = new System.Drawing.Size(133, 23);
            this.bt_mag_cal.TabIndex = 149;
            this.bt_mag_cal.Text = "지자계 캘리브레이션";
            this.bt_mag_cal.UseVisualStyleBackColor = true;
            this.bt_mag_cal.Click += new System.EventHandler(this.bt_mag_cal_Click);
            // 
            // bt_acc_cal
            // 
            this.bt_acc_cal.Location = new System.Drawing.Point(81, 205);
            this.bt_acc_cal.Name = "bt_acc_cal";
            this.bt_acc_cal.Size = new System.Drawing.Size(133, 23);
            this.bt_acc_cal.TabIndex = 150;
            this.bt_acc_cal.Text = "가속도 캘리브레이션";
            this.bt_acc_cal.UseVisualStyleBackColor = true;
            this.bt_acc_cal.Click += new System.EventHandler(this.bt_acc_cal_Click);
            // 
            // tabPage5
            // 
            this.tabPage5.Location = new System.Drawing.Point(4, 22);
            this.tabPage5.Name = "tabPage5";
            this.tabPage5.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage5.Size = new System.Drawing.Size(725, 569);
            this.tabPage5.TabIndex = 4;
            this.tabPage5.Text = "실험실";
            this.tabPage5.UseVisualStyleBackColor = true;
            // 
            // bt_ReScan
            // 
            this.bt_ReScan.Location = new System.Drawing.Point(12, 36);
            this.bt_ReScan.Name = "bt_ReScan";
            this.bt_ReScan.Size = new System.Drawing.Size(122, 23);
            this.bt_ReScan.TabIndex = 90;
            this.bt_ReScan.Text = "ReScan";
            this.bt_ReScan.UseVisualStyleBackColor = true;
            this.bt_ReScan.Click += new System.EventHandler(this.bt_ReScan_Click);
            // 
            // textBox5
            // 
            this.textBox5.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox5.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBox5.Location = new System.Drawing.Point(15, 601);
            this.textBox5.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox5.Name = "textBox5";
            this.textBox5.ReadOnly = true;
            this.textBox5.Size = new System.Drawing.Size(97, 22);
            this.textBox5.TabIndex = 94;
            this.textBox5.Text = "MSP_Error :";
            // 
            // tb_msp_error
            // 
            this.tb_msp_error.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.tb_msp_error.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.tb_msp_error.Location = new System.Drawing.Point(124, 601);
            this.tb_msp_error.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.tb_msp_error.Name = "tb_msp_error";
            this.tb_msp_error.ReadOnly = true;
            this.tb_msp_error.Size = new System.Drawing.Size(55, 22);
            this.tb_msp_error.TabIndex = 93;
            this.tb_msp_error.Text = "-";
            // 
            // label99
            // 
            this.label99.AutoSize = true;
            this.label99.Font = new System.Drawing.Font("굴림", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label99.Location = new System.Drawing.Point(497, 34);
            this.label99.Name = "label99";
            this.label99.Size = new System.Drawing.Size(99, 16);
            this.label99.TabIndex = 93;
            this.label99.Text = "Flight Time : ";
            // 
            // label101
            // 
            this.label101.AutoSize = true;
            this.label101.Font = new System.Drawing.Font("굴림", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label101.ImageAlign = System.Drawing.ContentAlignment.MiddleLeft;
            this.label101.Location = new System.Drawing.Point(4, 33);
            this.label101.Name = "label101";
            this.label101.Size = new System.Drawing.Size(92, 16);
            this.label101.TabIndex = 94;
            this.label101.Text = "GPS Time : ";
            // 
            // lb_gps_time
            // 
            this.lb_gps_time.Font = new System.Drawing.Font("Consolas", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_gps_time.Location = new System.Drawing.Point(86, 30);
            this.lb_gps_time.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_gps_time.Name = "lb_gps_time";
            this.lb_gps_time.Size = new System.Drawing.Size(238, 25);
            this.lb_gps_time.TabIndex = 95;
            this.lb_gps_time.Text = "2025-01-01 00:00:00";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1184, 681);
            this.Controls.Add(this.textBox5);
            this.Controls.Add(this.tb_msp_error);
            this.Controls.Add(this.bt_ReScan);
            this.Controls.Add(this.tabControl1);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.panel4);
            this.Controls.Add(this.panel3);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.OpenClose);
            this.Controls.Add(this.label27);
            this.Controls.Add(this.comboBox_port);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.label24);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Form1";
            this.Text = "Form1";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.panel4.ResumeLayout(false);
            this.panel4.PerformLayout();
            this.panel3.ResumeLayout(false);
            this.panel3.PerformLayout();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox6)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox5)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.panel7.ResumeLayout(false);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.tabControl1.ResumeLayout(false);
            this.tabPage1.ResumeLayout(false);
            this.groupBox10.ResumeLayout(false);
            this.tabPage2.ResumeLayout(false);
            this.tabPage2.PerformLayout();
            this.tabPage3.ResumeLayout(false);
            this.tabPage3.PerformLayout();
            this.tabPage4.ResumeLayout(false);
            this.groupBox9.ResumeLayout(false);
            this.groupBox9.PerformLayout();
            this.groupBox8.ResumeLayout(false);
            this.groupBox8.PerformLayout();
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.Button OpenClose;
        private System.Windows.Forms.ComboBox comboBox_port;
        private System.IO.Ports.SerialPort serialPort;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Label lb_roll;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label lb_pitch;
        private System.Windows.Forms.Label label2;
        public System.Windows.Forms.Label label27;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button button7;
        private System.Windows.Forms.Button button6;
        private System.Windows.Forms.Button bt_reset_waypoints;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Label label37;
        private System.Windows.Forms.Label label38;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label lb_altitude;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label lb_heading;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label lb_long;
        private System.Windows.Forms.Label lb_lat;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label24;
        public System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Panel panel4;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label3;
        public System.Windows.Forms.TextBox textBox2;
        public System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.Panel panel3;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.PictureBox pictureBox6;
        private System.Windows.Forms.PictureBox pictureBox5;
        private System.Windows.Forms.PictureBox pictureBox4;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Label lb_bat_V;
        private System.Windows.Forms.Panel panel7;
        private System.Windows.Forms.Panel panel6;
        private System.Windows.Forms.Panel panel5;
        private System.Windows.Forms.TextBox textBox10;
        private System.Windows.Forms.Button bt_zoom_p;
        private System.Windows.Forms.Button bt_zoom_m;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label label36;
        private System.Windows.Forms.TextBox textBox11;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button bt_pid_recive;
        private System.Windows.Forms.Button bt_pid_send;
        private System.Windows.Forms.Label label39;
        private System.Windows.Forms.TextBox tb_R_I_D;
        private System.Windows.Forms.Label label40;
        private System.Windows.Forms.TextBox tb_R_I_I;
        private System.Windows.Forms.Label label41;
        private System.Windows.Forms.TextBox tb_R_I_P;
        private System.Windows.Forms.Timer rx_timer_blink;
        private System.Windows.Forms.Label lb_rc_yaw;
        private System.Windows.Forms.Label label42;
        private System.Windows.Forms.Label lb_rc_pitch;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.Label lb_rc_roll;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label lb_rc_throttle;
        private System.Windows.Forms.Label label44;
        private System.Windows.Forms.Label lb_fail;
        private System.Windows.Forms.Label lb_armed;
        private System.Windows.Forms.Timer timer_status;
        private System.Windows.Forms.Timer flight_timer;
        private System.Windows.Forms.Label lb_debug0;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.Label lb_debug3;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.Label lb_debug2;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label lb_debug1;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.ContextMenuStrip contextMenuStrip1;
        private System.Windows.Forms.Label lb_motor0;
        private System.Windows.Forms.Label label28;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label lb_motor3;
        private System.Windows.Forms.Label lb_motor2;
        private System.Windows.Forms.Label lb_motor1;
        private System.Windows.Forms.Label label33;
        private System.Windows.Forms.Label label32;
        private System.Windows.Forms.Label label31;
        private System.Windows.Forms.Label label30;
        private System.Windows.Forms.Label label29;
        private System.Windows.Forms.Button bt_pid_copy;
        private System.Windows.Forms.TextBox tb_FC_Y_R_D;
        private System.Windows.Forms.TextBox tb_FC_Y_R_I;
        private System.Windows.Forms.TextBox tb_FC_Y_R_P;
        private System.Windows.Forms.Label label46;
        private System.Windows.Forms.Label label47;
        private System.Windows.Forms.TextBox tb_FC_Y_A_D;
        private System.Windows.Forms.TextBox tb_FC_Y_A_I;
        private System.Windows.Forms.TextBox tb_FC_Y_A_P;
        private System.Windows.Forms.TextBox tb_FC_P_O_D;
        private System.Windows.Forms.TextBox tb_FC_P_O_I;
        private System.Windows.Forms.TextBox tb_FC_P_O_P;
        private System.Windows.Forms.Label label48;
        private System.Windows.Forms.Label label49;
        private System.Windows.Forms.TextBox tb_FC_P_I_D;
        private System.Windows.Forms.TextBox tb_FC_P_I_I;
        private System.Windows.Forms.TextBox tb_FC_P_I_P;
        private System.Windows.Forms.TextBox tb_FC_R_O_D;
        private System.Windows.Forms.TextBox tb_FC_R_O_I;
        private System.Windows.Forms.TextBox tb_FC_R_O_P;
        private System.Windows.Forms.Label label50;
        private System.Windows.Forms.Label label51;
        private System.Windows.Forms.Label label52;
        private System.Windows.Forms.TextBox tb_FC_R_I_D;
        private System.Windows.Forms.Label label53;
        private System.Windows.Forms.TextBox tb_FC_R_I_I;
        private System.Windows.Forms.Label label54;
        private System.Windows.Forms.TextBox tb_FC_R_I_P;
        private System.Windows.Forms.TextBox tb_Y_R_D;
        private System.Windows.Forms.TextBox tb_Y_R_I;
        private System.Windows.Forms.TextBox tb_Y_R_P;
        private System.Windows.Forms.Label label43;
        private System.Windows.Forms.Label label45;
        private System.Windows.Forms.TextBox tb_Y_A_D;
        private System.Windows.Forms.TextBox tb_Y_A_I;
        private System.Windows.Forms.TextBox tb_Y_A_P;
        private System.Windows.Forms.TextBox tb_P_O_D;
        private System.Windows.Forms.TextBox tb_P_O_I;
        private System.Windows.Forms.TextBox tb_P_O_P;
        private System.Windows.Forms.Label label34;
        private System.Windows.Forms.Label label35;
        private System.Windows.Forms.TextBox tb_P_I_D;
        private System.Windows.Forms.TextBox tb_P_I_I;
        private System.Windows.Forms.TextBox tb_P_I_P;
        private System.Windows.Forms.TextBox tb_R_O_D;
        private System.Windows.Forms.TextBox tb_R_O_I;
        private System.Windows.Forms.TextBox tb_R_O_P;
        private System.Windows.Forms.Button bt_pid_save;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.TabPage tabPage1;
        private System.Windows.Forms.TabPage tabPage2;
        private System.Windows.Forms.Label lb_sat_num;
        private System.Windows.Forms.Label label57;
        private System.Windows.Forms.Label lb_gyro_Z;
        private System.Windows.Forms.Label lb_gyro_Y;
        private System.Windows.Forms.Label lb_gyro_X;
        private System.Windows.Forms.Label label60;
        private System.Windows.Forms.Label label59;
        private System.Windows.Forms.Label label58;
        private System.Windows.Forms.TabPage tabPage3;
        private ZedGraph.ZedGraphControl zedGraphControl1;
        private System.Windows.Forms.CheckBox cb_autoscale;
        private System.Windows.Forms.RadioButton rb_none;
        private System.Windows.Forms.RadioButton rb_yaw_setpoint;
        private System.Windows.Forms.RadioButton rb_pitch_setpoint;
        private System.Windows.Forms.RadioButton rb_roll_setpoint;
        private System.Windows.Forms.RadioButton rb_roll_pitch;
        private System.Windows.Forms.RadioButton rb_yaw;
        private System.Windows.Forms.RadioButton rb_pitch;
        private System.Windows.Forms.RadioButton rb_roll;
        private System.Windows.Forms.RadioButton rb_motor;
        private System.Windows.Forms.RadioButton rb_gyro;
        private System.Windows.Forms.RadioButton rb_altitude;
        private GMap.NET.WindowsForms.GMapControl gMapControl1;
        private System.Windows.Forms.Label lb_route_distance;
        private System.Windows.Forms.Label label61;
        private System.Windows.Forms.Label label62;
        private System.Windows.Forms.Label label82;
        private System.Windows.Forms.Label lb_rangefinder;
        private System.Windows.Forms.Label label75;
        private System.Windows.Forms.RadioButton rb_debug;
        private System.Windows.Forms.Label label77;
        public System.Windows.Forms.TextBox tb_rx_error;
        private System.Windows.Forms.CheckBox cB_RP_Coupling;
        private System.Windows.Forms.Button bt_ReScan;
        private System.Windows.Forms.CheckBox cb_record;
        public System.Windows.Forms.Panel panel8;
        private System.Windows.Forms.Label label79;
        private System.Windows.Forms.Label label81;
        private System.Windows.Forms.Label lb_fc_load;
        private System.Windows.Forms.Label label55;
        private System.Windows.Forms.Button bt_start_pid_test;
        private System.Windows.Forms.Label label84;
        private System.Windows.Forms.Label label83;
        private System.Windows.Forms.Label label80;
        private System.Windows.Forms.TextBox tb_PID_ms;
        private System.Windows.Forms.TextBox tb_PID_Deg;
        private System.Windows.Forms.TextBox tb_PID_Throttle;
        private System.Windows.Forms.Label lb_PID_Test_Status;
        private System.Windows.Forms.Button bt_open_folder;
        private System.Windows.Forms.Label lb_PID_Test_Progress;
        private System.Windows.Forms.Label lb_PID_Test_Target_Time;
        private System.Windows.Forms.Label lb_PID_Test_Progress_Time;
        private System.Windows.Forms.Label label86;
        private System.Windows.Forms.Label label85;
        public System.Windows.Forms.TextBox textBox5;
        public System.Windows.Forms.TextBox tb_msp_error;
        private System.Windows.Forms.Label label88;
        private System.Windows.Forms.Label label87;
        private System.Windows.Forms.TextBox tb_ALT_D;
        private System.Windows.Forms.TextBox tb_ALT_I;
        private System.Windows.Forms.TextBox tb_ALT_P;
        private System.Windows.Forms.TextBox tb_FC_ALT_D;
        private System.Windows.Forms.TextBox tb_FC_ALT_I;
        private System.Windows.Forms.TextBox tb_FC_ALT_P;
        private System.Windows.Forms.Label label89;
        private System.Windows.Forms.RadioButton rb_alt_setpoint;
        private System.Windows.Forms.Label lb_althold;
        private System.Windows.Forms.Label label91;
        private System.Windows.Forms.Label label56;
        private System.Windows.Forms.Label lb_gps_fix;
        private System.Windows.Forms.GroupBox groupBox10;
        private AvionicsInstrumentControlDemo.AttitudeIndicatorInstrumentControl attitudeIndicatorInstrumentControl1;
        private AvionicsInstrumentControlDemo.HeadingIndicatorInstrumentControl headingIndicatorInstrumentControl1;
        private System.Windows.Forms.TabPage tabPage4;
        private System.Windows.Forms.GroupBox groupBox9;
        private System.Windows.Forms.Label label69;
        private System.Windows.Forms.Label label76;
        private System.Windows.Forms.Label lb_bodyrate_Y;
        private System.Windows.Forms.Label lb_bodyrate_X;
        private System.Windows.Forms.Label label72;
        private System.Windows.Forms.Label label73;
        private System.Windows.Forms.Label lb_flowrate_Y;
        private System.Windows.Forms.Label lb_flowrate_X;
        private System.Windows.Forms.GroupBox groupBox8;
        private System.Windows.Forms.Label label67;
        private System.Windows.Forms.Label label68;
        private System.Windows.Forms.Label lb_mag_Z;
        private System.Windows.Forms.Label lb_mag_Y;
        private System.Windows.Forms.Label lb_mag_X;
        private System.Windows.Forms.Label label74;
        private System.Windows.Forms.Label lb_magcal_remain_time;
        private System.Windows.Forms.Label label66;
        private System.Windows.Forms.Label label71;
        private System.Windows.Forms.Label label70;
        private System.Windows.Forms.Label lb_magZero_Z;
        private System.Windows.Forms.Label lb_magZero_Y;
        private System.Windows.Forms.Label lb_magZero_X;
        private System.Windows.Forms.Label label78;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.Label label65;
        private System.Windows.Forms.Label label63;
        private System.Windows.Forms.Label label64;
        private System.Windows.Forms.Label lb_accTrim_Y;
        private System.Windows.Forms.Label lb_accTrim_Z;
        private System.Windows.Forms.Label lb_accTrim_X;
        private System.Windows.Forms.Button bt_mag_cal;
        private System.Windows.Forms.Button bt_acc_cal;
        private LiveCharts.WinForms.SolidGauge Gauge_RR;
        private LiveCharts.WinForms.SolidGauge Gauge_RF;
        private LiveCharts.WinForms.SolidGauge Gauge_LR;
        private LiveCharts.WinForms.SolidGauge Gauge_LF;
        private System.Windows.Forms.Label label90;
        private System.Windows.Forms.Label lb_bat_A;
        private System.Windows.Forms.Label label92;
        private System.Windows.Forms.Label lb_bat_mAh;
        private System.Windows.Forms.TextBox tb_FC_ALT_Range_D;
        private System.Windows.Forms.TextBox tb_FC_ALT_Range_I;
        private System.Windows.Forms.TextBox tb_FC_ALT_Range_P;
        private System.Windows.Forms.Label label96;
        private System.Windows.Forms.Label label97;
        private System.Windows.Forms.TextBox tb_ALT_Range_D;
        private System.Windows.Forms.TextBox tb_ALT_Range_I;
        private System.Windows.Forms.TextBox tb_ALT_Range_P;
        private System.Windows.Forms.TextBox tb_FC_POS_Opflow_D;
        private System.Windows.Forms.TextBox tb_FC_POS_Opflow_I;
        private System.Windows.Forms.TextBox tb_FC_POS_Opflow_P;
        private System.Windows.Forms.Label label93;
        private System.Windows.Forms.Label label94;
        private System.Windows.Forms.Label label95;
        private System.Windows.Forms.TextBox tb_POS_Opflow_D;
        private System.Windows.Forms.TextBox tb_POS_Opflow_I;
        private System.Windows.Forms.TextBox tb_POS_Opflow_P;
        private System.Windows.Forms.TabPage tabPage5;
        private System.Windows.Forms.RadioButton rb_alt_range_setpoint;
        private System.Windows.Forms.ComboBox comboBox_Debug;
        private System.Windows.Forms.Label label98;
        private System.Windows.Forms.Label lb_debug7;
        private System.Windows.Forms.Label label100;
        private System.Windows.Forms.Label lb_debug6;
        private System.Windows.Forms.Label label102;
        private System.Windows.Forms.Label lb_debug5;
        private System.Windows.Forms.Label label104;
        private System.Windows.Forms.Label lb_debug4;
        private System.Windows.Forms.Label label99;
        private System.Windows.Forms.Label lb_gps_time;
        private System.Windows.Forms.Label label101;
    }
}

