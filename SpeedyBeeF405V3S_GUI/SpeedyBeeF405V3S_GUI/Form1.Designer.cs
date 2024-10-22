namespace SpeedyBeeF405V3S_GUI
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
            this.label1 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.panel3 = new System.Windows.Forms.Panel();
            this.label36 = new System.Windows.Forms.Label();
            this.textBox11 = new System.Windows.Forms.TextBox();
            this.panel2 = new System.Windows.Forms.Panel();
            this.lb_fail = new System.Windows.Forms.Label();
            this.lb_armed = new System.Windows.Forms.Label();
            this.pictureBox6 = new System.Windows.Forms.PictureBox();
            this.pictureBox5 = new System.Windows.Forms.PictureBox();
            this.pictureBox4 = new System.Windows.Forms.PictureBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.lb_bat = new System.Windows.Forms.Label();
            this.panel7 = new System.Windows.Forms.Panel();
            this.panel6 = new System.Windows.Forms.Panel();
            this.panel5 = new System.Windows.Forms.Panel();
            this.textBox10 = new System.Windows.Forms.TextBox();
            this.button4 = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.label10 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.lb_debug0 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.bt_pid_recive = new System.Windows.Forms.Button();
            this.bt_pid_send = new System.Windows.Forms.Button();
            this.label39 = new System.Windows.Forms.Label();
            this.tb_D = new System.Windows.Forms.TextBox();
            this.label40 = new System.Windows.Forms.Label();
            this.tb_I = new System.Windows.Forms.TextBox();
            this.label41 = new System.Windows.Forms.Label();
            this.tb_P = new System.Windows.Forms.TextBox();
            this.rx_timer_blink = new System.Windows.Forms.Timer(this.components);
            this.timer_status = new System.Windows.Forms.Timer(this.components);
            this.flight_timer = new System.Windows.Forms.Timer(this.components);
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.lb_motor0 = new System.Windows.Forms.Label();
            this.label28 = new System.Windows.Forms.Label();
            this.label23 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.label21 = new System.Windows.Forms.Label();
            this.lb_debug3 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.lb_debug2 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.lb_debug1 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.contextMenuStrip1 = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.lb_motor1 = new System.Windows.Forms.Label();
            this.lb_motor2 = new System.Windows.Forms.Label();
            this.lb_motor3 = new System.Windows.Forms.Label();
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
            this.groupBox4.Text = "Telemetry data";
            // 
            // label37
            // 
            this.label37.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label37.Location = new System.Drawing.Point(147, 164);
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
            this.label22.Location = new System.Drawing.Point(147, 142);
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
            this.label20.Location = new System.Drawing.Point(147, 120);
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
            this.lb_altitude.Location = new System.Drawing.Point(147, 96);
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
            this.lb_heading.Location = new System.Drawing.Point(147, 72);
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
            this.lb_long.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_long.Location = new System.Drawing.Point(147, 48);
            this.lb_long.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_long.Name = "lb_long";
            this.lb_long.Size = new System.Drawing.Size(45, 20);
            this.lb_long.TabIndex = 44;
            this.lb_long.Text = "-";
            // 
            // lb_lat
            // 
            this.lb_lat.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_lat.Location = new System.Drawing.Point(147, 24);
            this.lb_lat.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_lat.Name = "lb_lat";
            this.lb_lat.Size = new System.Drawing.Size(45, 20);
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
            this.panel4.Controls.Add(this.label1);
            this.panel4.Controls.Add(this.label3);
            this.panel4.Controls.Add(this.textBox2);
            this.panel4.Controls.Add(this.textBox3);
            this.panel4.Location = new System.Drawing.Point(351, 589);
            this.panel4.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel4.Name = "panel4";
            this.panel4.Size = new System.Drawing.Size(808, 60);
            this.panel4.TabIndex = 73;
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
            this.label3.Location = new System.Drawing.Point(4, 28);
            this.label3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(45, 21);
            this.label3.TabIndex = 11;
            this.label3.Text = "Error";
            // 
            // textBox2
            // 
            this.textBox2.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox2.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBox2.Location = new System.Drawing.Point(119, 8);
            this.textBox2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox2.Name = "textBox2";
            this.textBox2.ReadOnly = true;
            this.textBox2.Size = new System.Drawing.Size(386, 22);
            this.textBox2.TabIndex = 7;
            this.textBox2.Text = "-";
            // 
            // textBox3
            // 
            this.textBox3.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox3.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBox3.Location = new System.Drawing.Point(119, 29);
            this.textBox3.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(386, 22);
            this.textBox3.TabIndex = 12;
            this.textBox3.Text = "-";
            // 
            // panel3
            // 
            this.panel3.Controls.Add(this.label36);
            this.panel3.Controls.Add(this.textBox11);
            this.panel3.Location = new System.Drawing.Point(351, 23);
            this.panel3.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel3.Name = "panel3";
            this.panel3.Size = new System.Drawing.Size(738, 56);
            this.panel3.TabIndex = 72;
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
            this.textBox11.Location = new System.Drawing.Point(99, 9);
            this.textBox11.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox11.Name = "textBox11";
            this.textBox11.ReadOnly = true;
            this.textBox11.Size = new System.Drawing.Size(488, 33);
            this.textBox11.TabIndex = 0;
            this.textBox11.Text = "MCU Drone flight monitor";
            this.textBox11.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // panel2
            // 
            this.panel2.Controls.Add(this.lb_fail);
            this.panel2.Controls.Add(this.lb_armed);
            this.panel2.Controls.Add(this.pictureBox6);
            this.panel2.Controls.Add(this.pictureBox5);
            this.panel2.Controls.Add(this.pictureBox4);
            this.panel2.Controls.Add(this.pictureBox2);
            this.panel2.Controls.Add(this.pictureBox1);
            this.panel2.Controls.Add(this.lb_bat);
            this.panel2.Controls.Add(this.panel7);
            this.panel2.Controls.Add(this.textBox10);
            this.panel2.Controls.Add(this.button4);
            this.panel2.Controls.Add(this.button1);
            this.panel2.Controls.Add(this.label10);
            this.panel2.Controls.Add(this.label15);
            this.panel2.Location = new System.Drawing.Point(1088, 23);
            this.panel2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(71, 560);
            this.panel2.TabIndex = 84;
            // 
            // lb_fail
            // 
            this.lb_fail.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_fail.Location = new System.Drawing.Point(3, 349);
            this.lb_fail.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_fail.Name = "lb_fail";
            this.lb_fail.Size = new System.Drawing.Size(64, 19);
            this.lb_fail.TabIndex = 85;
            this.lb_fail.Text = "-";
            // 
            // lb_armed
            // 
            this.lb_armed.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_armed.Location = new System.Drawing.Point(4, 328);
            this.lb_armed.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_armed.Name = "lb_armed";
            this.lb_armed.Size = new System.Drawing.Size(64, 19);
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
            // lb_bat
            // 
            this.lb_bat.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_bat.Location = new System.Drawing.Point(6, 372);
            this.lb_bat.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_bat.Name = "lb_bat";
            this.lb_bat.Size = new System.Drawing.Size(64, 19);
            this.lb_bat.TabIndex = 48;
            this.lb_bat.Text = "-";
            // 
            // panel7
            // 
            this.panel7.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel7.Controls.Add(this.panel6);
            this.panel7.Controls.Add(this.panel5);
            this.panel7.Location = new System.Drawing.Point(17, 416);
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
            // button4
            // 
            this.button4.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button4.Location = new System.Drawing.Point(12, 128);
            this.button4.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(50, 18);
            this.button4.TabIndex = 32;
            this.button4.Text = "+";
            this.button4.UseVisualStyleBackColor = true;
            // 
            // button1
            // 
            this.button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button1.Location = new System.Drawing.Point(12, 176);
            this.button1.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(50, 18);
            this.button1.TabIndex = 31;
            this.button1.Text = "-";
            this.button1.UseVisualStyleBackColor = true;
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
            this.label15.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label15.Location = new System.Drawing.Point(9, 309);
            this.label15.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(51, 19);
            this.label15.TabIndex = 42;
            this.label15.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lb_debug0
            // 
            this.lb_debug0.AutoSize = true;
            this.lb_debug0.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_debug0.Location = new System.Drawing.Point(115, 28);
            this.lb_debug0.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_debug0.Name = "lb_debug0";
            this.lb_debug0.Size = new System.Drawing.Size(16, 21);
            this.lb_debug0.TabIndex = 86;
            this.lb_debug0.Text = "-";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.bt_pid_recive);
            this.groupBox1.Controls.Add(this.bt_pid_send);
            this.groupBox1.Controls.Add(this.label39);
            this.groupBox1.Controls.Add(this.tb_D);
            this.groupBox1.Controls.Add(this.label40);
            this.groupBox1.Controls.Add(this.tb_I);
            this.groupBox1.Controls.Add(this.label41);
            this.groupBox1.Controls.Add(this.tb_P);
            this.groupBox1.Location = new System.Drawing.Point(13, 505);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(327, 144);
            this.groupBox1.TabIndex = 84;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "PID상수";
            // 
            // bt_pid_recive
            // 
            this.bt_pid_recive.Location = new System.Drawing.Point(151, 69);
            this.bt_pid_recive.Name = "bt_pid_recive";
            this.bt_pid_recive.Size = new System.Drawing.Size(115, 23);
            this.bt_pid_recive.TabIndex = 53;
            this.bt_pid_recive.Text = "데이터 가져오기";
            this.bt_pid_recive.UseVisualStyleBackColor = true;
            // 
            // bt_pid_send
            // 
            this.bt_pid_send.Location = new System.Drawing.Point(33, 69);
            this.bt_pid_send.Name = "bt_pid_send";
            this.bt_pid_send.Size = new System.Drawing.Size(102, 23);
            this.bt_pid_send.TabIndex = 52;
            this.bt_pid_send.Text = "데이터 보내기";
            this.bt_pid_send.UseVisualStyleBackColor = true;
            // 
            // label39
            // 
            this.label39.AutoSize = true;
            this.label39.Location = new System.Drawing.Point(170, 34);
            this.label39.Name = "label39";
            this.label39.Size = new System.Drawing.Size(13, 12);
            this.label39.TabIndex = 51;
            this.label39.Text = "D";
            // 
            // tb_D
            // 
            this.tb_D.Location = new System.Drawing.Point(190, 29);
            this.tb_D.Name = "tb_D";
            this.tb_D.Size = new System.Drawing.Size(45, 21);
            this.tb_D.TabIndex = 50;
            // 
            // label40
            // 
            this.label40.AutoSize = true;
            this.label40.Location = new System.Drawing.Point(88, 32);
            this.label40.Name = "label40";
            this.label40.Size = new System.Drawing.Size(8, 12);
            this.label40.TabIndex = 49;
            this.label40.Text = "I";
            // 
            // tb_I
            // 
            this.tb_I.Location = new System.Drawing.Point(104, 27);
            this.tb_I.Name = "tb_I";
            this.tb_I.Size = new System.Drawing.Size(50, 21);
            this.tb_I.TabIndex = 48;
            // 
            // label41
            // 
            this.label41.AutoSize = true;
            this.label41.Location = new System.Drawing.Point(14, 32);
            this.label41.Name = "label41";
            this.label41.Size = new System.Drawing.Size(13, 12);
            this.label41.TabIndex = 47;
            this.label41.Text = "P";
            // 
            // tb_P
            // 
            this.tb_P.Location = new System.Drawing.Point(33, 27);
            this.tb_P.Name = "tb_P";
            this.tb_P.Size = new System.Drawing.Size(44, 21);
            this.tb_P.TabIndex = 46;
            // 
            // rx_timer_blink
            // 
            this.rx_timer_blink.Enabled = true;
            this.rx_timer_blink.Tick += new System.EventHandler(this.Rx_timer_blink_Tick);
            // 
            // timer_status
            // 
            this.timer_status.Enabled = true;
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
            this.groupBox5.Controls.Add(this.lb_motor3);
            this.groupBox5.Controls.Add(this.lb_motor2);
            this.groupBox5.Controls.Add(this.lb_motor1);
            this.groupBox5.Controls.Add(this.lb_motor0);
            this.groupBox5.Controls.Add(this.label28);
            this.groupBox5.Controls.Add(this.label23);
            this.groupBox5.Controls.Add(this.label19);
            this.groupBox5.Controls.Add(this.label16);
            this.groupBox5.Location = new System.Drawing.Point(369, 110);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(335, 177);
            this.groupBox5.TabIndex = 85;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "MOTOR";
            // 
            // lb_motor0
            // 
            this.lb_motor0.AutoSize = true;
            this.lb_motor0.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_motor0.Location = new System.Drawing.Point(117, 65);
            this.lb_motor0.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor0.Name = "lb_motor0";
            this.lb_motor0.Size = new System.Drawing.Size(16, 21);
            this.lb_motor0.TabIndex = 92;
            this.lb_motor0.Text = "-";
            // 
            // label28
            // 
            this.label28.AutoSize = true;
            this.label28.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label28.Location = new System.Drawing.Point(174, 140);
            this.label28.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(83, 21);
            this.label28.TabIndex = 91;
            this.label28.Text = "MOTOR[3]";
            // 
            // label23
            // 
            this.label23.AutoSize = true;
            this.label23.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label23.Location = new System.Drawing.Point(19, 139);
            this.label23.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(83, 21);
            this.label23.TabIndex = 90;
            this.label23.Text = "MOTOR[1]";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label19.Location = new System.Drawing.Point(174, 65);
            this.label19.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(83, 21);
            this.label19.TabIndex = 89;
            this.label19.Text = "MOTOR[2]";
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label16.Location = new System.Drawing.Point(19, 65);
            this.label16.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(83, 21);
            this.label16.TabIndex = 88;
            this.label16.Text = "MOTOR[0]";
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.label21);
            this.groupBox6.Controls.Add(this.lb_debug3);
            this.groupBox6.Controls.Add(this.label17);
            this.groupBox6.Controls.Add(this.lb_debug2);
            this.groupBox6.Controls.Add(this.label14);
            this.groupBox6.Controls.Add(this.lb_debug1);
            this.groupBox6.Controls.Add(this.label12);
            this.groupBox6.Controls.Add(this.lb_debug0);
            this.groupBox6.Location = new System.Drawing.Point(746, 110);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(335, 177);
            this.groupBox6.TabIndex = 86;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "DEBUG";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label21.Location = new System.Drawing.Point(7, 128);
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
            this.lb_debug3.Location = new System.Drawing.Point(115, 128);
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
            this.label17.Location = new System.Drawing.Point(7, 96);
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
            this.lb_debug2.Location = new System.Drawing.Point(115, 96);
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
            this.label14.Location = new System.Drawing.Point(7, 61);
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
            this.lb_debug1.Location = new System.Drawing.Point(115, 61);
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
            this.label12.Location = new System.Drawing.Point(7, 28);
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
            // lb_motor1
            // 
            this.lb_motor1.AutoSize = true;
            this.lb_motor1.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_motor1.Location = new System.Drawing.Point(117, 140);
            this.lb_motor1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor1.Name = "lb_motor1";
            this.lb_motor1.Size = new System.Drawing.Size(16, 21);
            this.lb_motor1.TabIndex = 93;
            this.lb_motor1.Text = "-";
            // 
            // lb_motor2
            // 
            this.lb_motor2.AutoSize = true;
            this.lb_motor2.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_motor2.Location = new System.Drawing.Point(267, 65);
            this.lb_motor2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor2.Name = "lb_motor2";
            this.lb_motor2.Size = new System.Drawing.Size(16, 21);
            this.lb_motor2.TabIndex = 94;
            this.lb_motor2.Text = "-";
            // 
            // lb_motor3
            // 
            this.lb_motor3.AutoSize = true;
            this.lb_motor3.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lb_motor3.Location = new System.Drawing.Point(267, 139);
            this.lb_motor3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lb_motor3.Name = "lb_motor3";
            this.lb_motor3.Size = new System.Drawing.Size(16, 21);
            this.lb_motor3.TabIndex = 95;
            this.lb_motor3.Text = "-";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1184, 661);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.groupBox1);
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
        private System.Windows.Forms.Label lb_bat;
        private System.Windows.Forms.Panel panel7;
        private System.Windows.Forms.Panel panel6;
        private System.Windows.Forms.Panel panel5;
        private System.Windows.Forms.TextBox textBox10;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label label36;
        private System.Windows.Forms.TextBox textBox11;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button bt_pid_recive;
        private System.Windows.Forms.Button bt_pid_send;
        private System.Windows.Forms.Label label39;
        private System.Windows.Forms.TextBox tb_D;
        private System.Windows.Forms.Label label40;
        private System.Windows.Forms.TextBox tb_I;
        private System.Windows.Forms.Label label41;
        private System.Windows.Forms.TextBox tb_P;
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
    }
}

