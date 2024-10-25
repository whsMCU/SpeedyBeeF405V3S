using Ball_Ballancer_CS.Class;
using GMap.NET;
using GMap.NET.MapProviders;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using System;
using System.Collections;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Timers;
using System.Windows.Forms;

namespace SpeedyBeeF405V3S_GUI
{
    public partial class Form1 : Form
    {
        private static System.Timers.Timer AHRS_Timer;
        float[] passed_data = new float[30];
        UTF8 UTF8 = new UTF8();
        DataPassing data = new DataPassing();

        private ArrayList al;
        private Point p;

        float[] pb_point = new float[2];

        bool pid_recive_flag = false;
        bool pid_send_flag = false;
        bool pid_save_flag = false;

        public GMapOverlay MarkerOverlay = new GMapOverlay("markers");

        public Form1()
        {
            InitializeComponent();
            al = new ArrayList();
            this.Text = "MCU Drone 제어프로그램";
            received_data = 2;
            textBox10.Text = (zoom - 14).ToString();

            PointLatLng p = new PointLatLng(37.497872, 127.0275142);
            gMapControl1.Overlays.Add(MarkerOverlay);
            gMapControl1.MapProvider = GMapProviders.GoogleMap;
            gMapControl1.Position = p;
            gMapControl1.MinZoom = 5;
            gMapControl1.MaxZoom = 19;
            gMapControl1.Zoom = 14;
            AddMarker(p, "test");
        }

        private void Form1_Load(object sender, EventArgs e)  //폼이 로드되면
        {
            try
            {
                comboBox_port.DataSource = SerialPort.GetPortNames(); //연결 가능한 시리얼포트 이름을 콤보박스에 가져오기 
            }
            catch { }
        }
        public void AddMarker(PointLatLng p, string text)
        {
            GMarkerGoogle gMarker = new GMarkerGoogle(p, GMarkerGoogleType.blue_dot);
            gMarker.ToolTipMode = MarkerTooltipMode.OnMouseOver;
            gMarker.ToolTipText = text;
            gMarker.ToolTip.TextPadding = new Size(10, 10);
            gMarker.ToolTip.Fill = new SolidBrush(Color.DimGray);
            gMarker.ToolTip.Foreground = new SolidBrush(Color.White);
            gMarker.ToolTip.Offset = new Point(10, -30);
            gMarker.ToolTip.Stroke = new Pen(Color.Transparent, .0f);
            MarkerOverlay.Markers.Add(gMarker);
        }

        private void OpenClose_Click(object sender, EventArgs e)  //통신 연결하기 버튼
        {
            try
            {
                if (!serialPort.IsOpen && OpenClose.Text == "Open")  //시리얼포트가 열려 있지 않으면
                {
                    if(comboBox_port.Text.Length > 1)
                    {
                        serialPort.PortName = comboBox_port.Text;  //콤보박스의 선택된 COM포트명을 시리얼포트명으로 지정
                        serialPort.BaudRate = 115200;  //보레이트 변경이 필요하면 숫자 변경하기
                        serialPort.DataBits = 8;
                        serialPort.StopBits = StopBits.One;
                        serialPort.Parity = Parity.None;
                        serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler); //이것이 꼭 필요하다

                        serialPort.Open();  //시리얼포트 열기
                        OpenClose.Text = "Close";
                        comboBox_port.Enabled = false;  //COM포트설정 콤보박스 비활성화

                        AHRS_Timer = new System.Timers.Timer(100);
                        AHRS_Timer.Elapsed += OnTimedEvent;
                        AHRS_Timer.AutoReset = true;
                        AHRS_Timer.Enabled = true;
                    }
                    else
                    {
                        string message = "No port selected";
                        string title = "Error";
                        MessageBox.Show(message, title);
                        return;
                    }

                }
                else  //시리얼포트가 열려 있으면
                {
                    AHRS_Timer.Enabled = false;
                    //clear_waypoint_labels();
                    serialPort.Close();
                    OpenClose.Text = "Open";
                    label12.Visible = false;
                    label24.Visible = false;
                    //first_receive = 0;
                    button6.Enabled = false;
                    //flight_timer.Enabled = false;
                    comboBox_port.Enabled = true;  //COM포트설정 콤보박스 활성화
                }
            }
            catch (System.Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)  //수신 이벤트가 발생하면 이 부분이 실행된다.
        {
            try
            {
                this.Invoke(new EventHandler(MySerialReceived));  //메인 쓰레드와 수신 쓰레드의 충돌 방지를 위해 Invoke 사용. MySerialReceived로 이동하여 추가 작업 실행.
            }
            catch { Console.WriteLine("SerailHandler Error"); }
        }

        private void MySerialReceived(object s, EventArgs e)  //여기에서 수신 데이타를 사용자의 용도에 따라 처리한다.
        {
            //int ReceiveData = serialPort1.ReadByte();  //시리얼 버터에 수신된 데이타를 ReceiveData 읽어오기
            //richTextBox_received.Text = richTextBox_received.Text + string.Format("{0:X2}", ReceiveData);  //int 형식을 string형식으로 변환하여 출력
            try
            {
                if (received_data == 0) received_data = 1;
                int iRecSize = serialPort.BytesToRead; // 수신된 데이터 갯수

                if (iRecSize != 0) // 수신된 데이터의 수가 0이 아닐때만 처리하자
                {
                    byte[] buff = new byte[iRecSize];
                    try
                    {
                        serialPort.Read(buff, 0, iRecSize);
                        passed_data = data.dataPassing(buff, iRecSize);
                        if (passed_data != null)
                        {
                            if (passed_data[0] == 0)
                            {
                                lb_roll.Text = passed_data[1].ToString();
                                lb_pitch.Text = passed_data[2].ToString();
                                lb_heading.Text = passed_data[3].ToString();
                                lb_altitude.Text = passed_data[4].ToString();
                                lb_rc_roll.Text = passed_data[5].ToString();
                                lb_rc_pitch.Text = passed_data[6].ToString();
                                lb_rc_yaw.Text = passed_data[7].ToString();
                                lb_rc_throttle.Text = passed_data[8].ToString();
                                lb_lat.Text = passed_data[9].ToString();
                                lb_long.Text = passed_data[10].ToString();
                                lb_bat.Text = passed_data[11].ToString();
                                battery_bar_level = (int)passed_data[11];
                                lb_fail.Text = passed_data[12].ToString();
                                lb_armed.Text = passed_data[13].ToString();
                                start = (byte)passed_data[13];

                                lb_motor0.Text = passed_data[14].ToString();
                                lb_motor1.Text = passed_data[15].ToString();
                                lb_motor2.Text = passed_data[16].ToString();
                                lb_motor3.Text = passed_data[17].ToString();

                                lb_debug0.Text = passed_data[18].ToString();
                                lb_debug1.Text = passed_data[19].ToString();
                                lb_debug2.Text = passed_data[20].ToString();
                                lb_debug3.Text = passed_data[21].ToString();
                            }
                            else if (passed_data[0] == 1)
                            {
                                tb_FC_R_I_P.Text = passed_data[1].ToString();
                                tb_FC_R_I_I.Text = passed_data[2].ToString();
                                tb_FC_R_I_D.Text = passed_data[3].ToString();

                                tb_FC_R_O_P.Text = passed_data[4].ToString();
                                tb_FC_R_O_I.Text = passed_data[5].ToString();
                                tb_FC_R_O_D.Text = passed_data[6].ToString();

                                tb_FC_P_I_P.Text = passed_data[7].ToString();
                                tb_FC_P_I_I.Text = passed_data[8].ToString();
                                tb_FC_P_I_D.Text = passed_data[9].ToString();

                                tb_FC_P_O_P.Text = passed_data[10].ToString();
                                tb_FC_P_O_I.Text = passed_data[11].ToString();
                                tb_FC_P_O_D.Text = passed_data[12].ToString();

                                tb_FC_Y_A_P.Text = passed_data[13].ToString();
                                tb_FC_Y_A_I.Text = passed_data[14].ToString();
                                tb_FC_Y_A_D.Text = passed_data[15].ToString();

                                tb_FC_Y_R_P.Text = passed_data[16].ToString();
                                tb_FC_Y_R_I.Text = passed_data[17].ToString();
                                tb_FC_Y_R_D.Text = passed_data[18].ToString();
                            }

                        }
                    }
                    catch { Console.WriteLine("Data Passing Error1"); }
                }
            }
            catch (System.Exception)
            {
                Console.WriteLine("Data Passing Error2");
            }
        }

        private void OnTimedEvent(object source, ElapsedEventArgs e)
        {
            byte[] buff = new byte[20];
            try
            {
                buff[0] = 0x47;
                buff[1] = 0x53;
                buff[2] = 0x20;
                buff[3] = 0;
                buff[4] = 0;
                buff[5] = 0;
                buff[6] = 0;
                buff[7] = 0;
                buff[8] = 0;
                buff[9] = 0;
                buff[10] = 0;
                buff[11] = 0;
                buff[12] = 0;
                buff[13] = 0;
                buff[14] = 0;
                buff[15] = 0;
                buff[16] = 0;
                buff[17] = 0;
                buff[18] = 0;
                buff[19] = 0xff;

                for (int i = 0; i < 19; i++)
                {
                    buff[19] -= buff[i];
                }

                serialPort.Write(Encoding.UTF8.GetString(buff));
            }
            catch { Console.WriteLine("Telemetry Data Requset Error"); }
            if(pid_recive_flag == true)
            {
                pid_recive_flag = false;
                try
                {
                    buff[0] = 0x47;
                    buff[1] = 0x53;
                    buff[2] = 0x10;
                    buff[3] = 0;
                    buff[4] = 0;
                    buff[5] = 0;
                    buff[6] = 0;
                    buff[7] = 0;
                    buff[8] = 0;
                    buff[9] = 0;
                    buff[10] = 0;
                    buff[11] = 0;
                    buff[12] = 0;
                    buff[13] = 0;
                    buff[14] = 0;
                    buff[15] = 0;
                    buff[16] = 0;
                    buff[17] = 0;
                    buff[18] = 0;
                    buff[19] = 0xff;

                    for (int i = 0; i < 19; i++)
                    {
                        buff[19] -= buff[i];
                    }

                    serialPort.Write(Encoding.UTF8.GetString(buff));
                    Console.WriteLine("PID값 수신명령 전송 완료");
                }
                catch { Console.WriteLine("PID Data Requset Error"); }
            }
            if (pid_save_flag == true)
            {
                pid_save_flag = false;
                try
                {
                    buff[0] = 0x47;
                    buff[1] = 0x53;
                    buff[2] = 0x00;
                    buff[3] = 0;
                    buff[4] = 0;
                    buff[5] = 0;
                    buff[6] = 0;
                    buff[7] = 0;
                    buff[8] = 0;
                    buff[9] = 0;
                    buff[10] = 0;
                    buff[11] = 0;
                    buff[12] = 0;
                    buff[13] = 0;
                    buff[14] = 0;
                    buff[15] = 0;
                    buff[16] = 0;
                    buff[17] = 0;
                    buff[18] = 0;
                    buff[19] = 0xff;

                    for (int i = 0; i < 19; i++)
                    {
                        buff[19] -= buff[i];
                    }
                    serialPort.Write(Encoding.UTF8.GetString(buff));
                    Console.WriteLine("PID값 저장명령 전송 완료");
                }
                catch { Console.WriteLine("PID Save Requset Error"); }
            }
            if(pid_send_flag == true)
            {
                pid_send_flag = false;
                byte[] pid_buff = new byte[76];
                float float_buff;
                byte[] tmp = new byte[4];
                try
                {
                    pid_buff[0] = 0x47;
                    pid_buff[1] = 0x53;
                    pid_buff[2] = 0x30;

                    float_buff = float.Parse(tb_R_I_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[3] = tmp[0];
                    pid_buff[4] = tmp[1];
                    pid_buff[5] = tmp[2];
                    pid_buff[6] = tmp[3];
                    float_buff = float.Parse(tb_R_I_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[7] = tmp[0];
                    pid_buff[8] = tmp[1];
                    pid_buff[9] = tmp[2];
                    pid_buff[10] = tmp[3];
                    float_buff = float.Parse(tb_R_I_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[11] = tmp[0];
                    pid_buff[12] = tmp[1];
                    pid_buff[13] = tmp[2];
                    pid_buff[14] = tmp[3];

                    float_buff = float.Parse(tb_R_O_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[15] = tmp[0];
                    pid_buff[16] = tmp[1];
                    pid_buff[17] = tmp[2];
                    pid_buff[18] = tmp[3];
                    float_buff = float.Parse(tb_R_O_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[19] = tmp[0];
                    pid_buff[20] = tmp[1];
                    pid_buff[21] = tmp[2];
                    pid_buff[22] = tmp[3];
                    float_buff = float.Parse(tb_R_O_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[23] = tmp[0];
                    pid_buff[24] = tmp[1];
                    pid_buff[25] = tmp[2];
                    pid_buff[26] = tmp[3];

                    float_buff = float.Parse(tb_P_I_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[27] = tmp[0];
                    pid_buff[28] = tmp[1];
                    pid_buff[29] = tmp[2];
                    pid_buff[30] = tmp[3];
                    float_buff = float.Parse(tb_P_I_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[31] = tmp[0];
                    pid_buff[32] = tmp[1];
                    pid_buff[33] = tmp[2];
                    pid_buff[34] = tmp[3];
                    float_buff = float.Parse(tb_P_I_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[35] = tmp[0];
                    pid_buff[36] = tmp[1];
                    pid_buff[37] = tmp[2];
                    pid_buff[38] = tmp[3];

                    float_buff = float.Parse(tb_P_O_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[39] = tmp[0];
                    pid_buff[40] = tmp[1];
                    pid_buff[41] = tmp[2];
                    pid_buff[42] = tmp[3];
                    float_buff = float.Parse(tb_P_O_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[43] = tmp[0];
                    pid_buff[44] = tmp[1];
                    pid_buff[45] = tmp[2];
                    pid_buff[46] = tmp[3];
                    float_buff = float.Parse(tb_P_O_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[47] = tmp[0];
                    pid_buff[48] = tmp[1];
                    pid_buff[49] = tmp[2];
                    pid_buff[50] = tmp[3];

                    float_buff = float.Parse(tb_Y_A_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[51] = tmp[0];
                    pid_buff[52] = tmp[1];
                    pid_buff[53] = tmp[2];
                    pid_buff[54] = tmp[3];
                    float_buff = float.Parse(tb_Y_A_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[55] = tmp[0];
                    pid_buff[56] = tmp[1];
                    pid_buff[57] = tmp[2];
                    pid_buff[58] = tmp[3];
                    float_buff = float.Parse(tb_Y_A_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[59] = tmp[0];
                    pid_buff[60] = tmp[1];
                    pid_buff[61] = tmp[2];
                    pid_buff[62] = tmp[3];

                    float_buff = float.Parse(tb_Y_R_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[63] = tmp[0];
                    pid_buff[64] = tmp[1];
                    pid_buff[65] = tmp[2];
                    pid_buff[66] = tmp[3];
                    float_buff = float.Parse(tb_Y_R_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[67] = tmp[0];
                    pid_buff[68] = tmp[1];
                    pid_buff[69] = tmp[2];
                    pid_buff[70] = tmp[3];
                    float_buff = float.Parse(tb_Y_R_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[71] = tmp[0];
                    pid_buff[72] = tmp[1];
                    pid_buff[73] = tmp[2];
                    pid_buff[74] = tmp[3];

                    pid_buff[75] = 0xff;

                    for (int i = 0; i < 75; i++)
                    {
                        pid_buff[75] -= pid_buff[i];
                    }

                    serialPort.Write(Encoding.UTF8.GetString(pid_buff));
                    Console.WriteLine("PID값 전송 완료");
                }
                catch { Console.WriteLine("PID Data Send Error"); }
            }
        }

        private void indicator_on()
        {
            Graphics g = panel1.CreateGraphics();
            Pen p = new Pen(Color.Blue);
            SolidBrush sb = new SolidBrush(Color.LightBlue);
            g.DrawEllipse(p, 1, 1, 10, 10);
            g.FillEllipse(sb, 1, 1, 10, 10);
        }

        private void indicator_off()
        {
            Graphics g = panel1.CreateGraphics();
            Pen p = new Pen(Color.DarkBlue);
            SolidBrush sb = new SolidBrush(Color.DarkBlue);
            g.DrawEllipse(p, 1, 1, 10, 10);
            g.FillEllipse(sb, 1, 1, 10, 10);
        }

        private void Rx_timer_blink_Tick(object sender, EventArgs e)
        {
            if (received_data > 0) received_data++;

            if (received_data == 2)
            {
                indicator_on();
            }
            if (received_data == 3)
            {
                indicator_off();
            }
            if (received_data == 5)
            {
                received_data = 0;
            }
        }

        private void timer_status_Tick(object sender, EventArgs e)
        {
            if (start == 0)
            {
                pictureBox1.Visible = true;
                pictureBox2.Visible = false;
            }
            if (start == 1)
            {
                pictureBox1.Visible = false;
                pictureBox2.Visible = true;
            }

            if (battery_bar_level > 124) battery_bar_level = 124;
            if (battery_bar_level < 85) battery_bar_level = 85;
            if (battery_bar_level > 108) panel5.BackColor = Color.Lime;
            else if (battery_bar_level > 100) panel5.BackColor = Color.Yellow;
            else panel5.BackColor = Color.Red;

            panel6.Size = new Size(34, 134 - ((battery_bar_level - 80) * 3));
        }

        private void flight_timer_Tick(object sender, EventArgs e)
        {
            if(start == 1)
            {
                flight_timer_seconds++;
                label36.Text = ("00:" + (flight_timer_seconds / 60).ToString("00.") + ":" + (flight_timer_seconds % 60).ToString("00."));
            }
        }

        private void bt_pid_recive_Click(object sender, EventArgs e)
        {
            pid_recive_flag = true;
        }

        private void bt_pid_send_Click(object sender, EventArgs e)
        {
            pid_send_flag = true;
        }

        private void bt_pid_save_Click(object sender, EventArgs e)
        {
            pid_save_flag = true;
        }

        private void bt_pid_copy_Click(object sender, EventArgs e)
        {
            tb_R_I_P.Text = tb_FC_R_I_P.Text;
            tb_R_I_I.Text = tb_FC_R_I_I.Text;
            tb_R_I_D.Text = tb_FC_R_I_D.Text;

            tb_R_O_P.Text = tb_FC_R_O_P.Text;
            tb_R_O_I.Text = tb_FC_R_O_I.Text;
            tb_R_O_D.Text = tb_FC_R_O_D.Text;

            tb_P_I_P.Text = tb_FC_P_I_P.Text;
            tb_P_I_I.Text = tb_FC_P_I_I.Text;
            tb_P_I_D.Text = tb_FC_P_I_D.Text;

            tb_P_O_P.Text = tb_FC_P_O_P.Text;
            tb_P_O_I.Text = tb_FC_P_O_I.Text;
            tb_P_O_D.Text =  tb_FC_P_O_D.Text;

            tb_Y_A_P.Text = tb_FC_Y_A_P.Text;
            tb_Y_A_I.Text = tb_FC_Y_A_I.Text;
            tb_Y_A_D.Text = tb_FC_Y_A_D.Text;

            tb_Y_R_P.Text = tb_FC_Y_R_P.Text;
            tb_Y_R_I.Text = tb_FC_Y_R_I.Text;
            tb_Y_R_D.Text = tb_FC_Y_R_D.Text;
        }
    }
}
