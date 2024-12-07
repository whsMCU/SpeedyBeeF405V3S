using Ball_Ballancer_CS.Class;
using GMap.NET;
using GMap.NET.MapProviders;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using System.Text;
using System.Timers;
using System.Windows.Forms;
using ZedGraph;
using static GMap.NET.Entity.OpenStreetMapGraphHopperGeocodeEntity;
using static System.Net.Mime.MediaTypeNames;

namespace SpeedyBeeF405V3S_GUI
{
    public partial class Form1 : Form
    {
        private static System.Timers.Timer AHRS_Timer;
        float[] passed_data = new float[30];
        UTF8 UTF8 = new UTF8();
        DataPassing data = new DataPassing();

        private ArrayList al;

        bool pid_recive_flag = false;
        bool pid_send_flag = false;
        bool pid_save_flag = false;

        List<PointLatLng> map_points = new List<PointLatLng>();
        GMarkerGoogle marker;
        GMapOverlay markersOverlay = new GMapOverlay("markers");

        UInt32 time_count = 0;
        GraphPane _myPane;
        PointPairList _roll_angle_points = new PointPairList();
        LineItem _roll_angle_curve;
        PointPairList _pitch_angle_points = new PointPairList();
        LineItem _pitch_angle_curve;
        PointPairList _yaw_angle_points = new PointPairList();
        LineItem _yaw_angle_curve;
        PointPairList _rc_roll_points = new PointPairList();
        LineItem _rc_roll_curve;
        PointPairList _rc_pitch_points = new PointPairList();
        LineItem _rc_pitch_curve;
        PointPairList _rc_yaw_points = new PointPairList();
        LineItem _rc_yaw_curve;
        PointPairList _rc_thro_points = new PointPairList();
        LineItem _rc_thro_curve;
        PointPairList _altitude_points = new PointPairList();
        LineItem _altitude_curve;
        PointPairList _gyro_x_points = new PointPairList();
        LineItem _gyro_x_curve;
        PointPairList _gyro_y_points = new PointPairList();
        LineItem _gyro_y_curve;
        PointPairList _gyro_z_points = new PointPairList();
        LineItem _gyro_z_curve;
        PointPairList _motor_0_points = new PointPairList();
        LineItem _motor_0_curve;
        PointPairList _motor_1_points = new PointPairList();
        LineItem _motor_1_curve;
        PointPairList _motor_2_points = new PointPairList();
        LineItem _motor_2_curve;
        PointPairList _motor_3_points = new PointPairList();
        LineItem _motor_3_curve;
        PointPairList _debug_0_points = new PointPairList();
        LineItem _debug_0_curve;
        PointPairList _debug_1_points = new PointPairList();
        LineItem _debug_1_curve;
        PointPairList _debug_2_points = new PointPairList();
        LineItem _debug_2_curve;
        PointPairList _debug_3_points = new PointPairList();
        LineItem _debug_3_curve;

        public Form1()
        {
            InitializeComponent();
            al = new ArrayList();
            this.Text = "MCU Drone 제어프로그램";
            received_data = 2;
            textBox10.Text = (zoom - 14).ToString();

            InitGmap();
            InitGraph();
        }

        public void InitGmap()
        {
            gMapControl1.MapProvider = GMapProviders.GoogleMap;
            PointLatLng p = new PointLatLng(35.1965882, 126.8295163);
            gMapControl1.Position = p;
            gMapControl1.MinZoom = 5;
            gMapControl1.MaxZoom = 100;
            gMapControl1.Zoom = 15;

        }

        private void gMapControl1_MouseClick(object sender, MouseEventArgs e)
        {
            if(e.Button == MouseButtons.Left)
            {
                // 클릭한 위치의 위도, 경도 가져오기
                PointLatLng point = gMapControl1.FromLocalToLatLng(e.X, e.Y);

                // 클릭한 위치에 마커 추가
                marker = new GMarkerGoogle(point, GMarkerGoogleType.red);
                marker.ToolTipText = $"위도: {point.Lat}, 경도: {point.Lng}";
                //marker.ToolTipMode = MarkerTooltipMode.Always;
                marker.ToolTipMode = MarkerTooltipMode.OnMouseOver;
                marker.ToolTip.TextPadding = new Size(10, 10);
                marker.ToolTip.Fill = new SolidBrush(Color.DimGray);
                marker.ToolTip.Foreground = new SolidBrush(Color.White);
                //gMarker.ToolTip.Offset = new Point(10, -30);
                marker.ToolTip.Stroke = new Pen(Color.Transparent, .0f);

                // 마커를 GMapControl의 Overlay에 추가
                markersOverlay.Markers.Add(marker);
                gMapControl1.Overlays.Add(markersOverlay);

                map_points.Add(point);
                GMapRoute route = new GMapRoute(map_points, "route");
                route.Stroke = new Pen(Color.Red, 2);
                markersOverlay.Routes.Add(route);
                gMapControl1.Overlays.Add(markersOverlay);
                lb_route_distance.Text = route.Distance.ToString();

                // 지도 새로고침
                gMapControl1.Refresh();
                gMapControl1.Zoom++;
                gMapControl1.Zoom--;
            }
        }
        public void RemoveMarker(GMapMarker gMarker)
        {
            markersOverlay.Markers.Remove(gMarker);

        }

        public void RemoveRoute(GMapRoute gRoute)
        {
            markersOverlay.Routes.Remove(gRoute);
        }

        private void gMapControl1_OnMarkerClick(GMapMarker item, MouseEventArgs e)
        {
            if(e.Button == MouseButtons.Right)
            {
                RemoveMarker(item);
            }
        }


        private void gMapControl1_OnRouteClick(GMapRoute item, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right)
            {
                RemoveRoute(item);
            }
        }

        public void InitGraph()
        {
            _myPane = zedGraphControl1.GraphPane;
            _myPane.Title.Text = "Trend";
            _myPane.Title.FontSpec.Size = 15;
            _myPane.Title.FontSpec.IsBold = true;
            //X축 설정
            _myPane.XAxis.Title.FontSpec.Size = 12;
            _myPane.XAxis.Title.Text = "X Axis";
            //실시간으로 Scale 변경 자동으로 하도록
            _myPane.XAxis.Scale.Min = 0;
            _myPane.XAxis.Scale.Max = 300;
            //_myPane.XAxis.Scale.MinAuto = true;
            //_myPane.XAxis.Scale.MaxAuto = true;
            _myPane.XAxis.Scale.MajorStepAuto = true;
            _myPane.XAxis.MajorGrid.IsVisible = true;
            _myPane.XAxis.MinorGrid.IsVisible = false;
            _myPane.XAxis.MajorTic.Color = Color.Black;
            //Y축 설정
            _myPane.YAxis.Title.FontSpec.Size = 12;
            _myPane.YAxis.Title.Text = "Y Axis";
            //실시간으로 Scale 변경 자동으로 하도록
            _myPane.YAxis.Scale.Min = -60;
            _myPane.YAxis.Scale.Max = 60;
            //_myPane.YAxis.Scale.MinAuto = true;
            //_myPane.YAxis.Scale.MaxAuto = true;
            _myPane.YAxis.Scale.MajorStepAuto = true;
            _myPane.YAxis.MajorGrid.IsVisible = true;
            _myPane.YAxis.MinorGrid.IsVisible = false;
            _myPane.YAxis.MajorTic.Color = Color.Black;

            //그래프 Chart 색, Border 색/굵기 설정
            _myPane.Fill = new Fill(Color.FromArgb(255, 238, 238, 238));
            _myPane.Chart.Fill = new Fill(Color.LightGray, Color.LightGray, 180.0f);
            _myPane.Chart.Border.Color = Color.Black;
            _myPane.Chart.Border.Width = 2;
            //Point 리스트를 그래프 Curve 리스트에 추가
            _myPane.CurveList.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
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
            //gMarker.ToolTip.Offset = new Point(10, -30);
            gMarker.ToolTip.Stroke = new Pen(Color.Transparent, .0f);
            markersOverlay.Markers.Add(gMarker);
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
                                DateTime date_time = DateTime.Now;
                                int ms = date_time.Millisecond;
                                time_count++;
                                lb_roll.Text = passed_data[1].ToString();
                                lb_pitch.Text = passed_data[2].ToString();
                                lb_heading.Text = passed_data[3].ToString();

                                if(rb_roll.Checked == true)
                                {
                                    _roll_angle_points.Add(time_count+150, passed_data[1]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }
                                if (rb_pitch.Checked == true)
                                {
                                    _pitch_angle_points.Add(time_count + 150, passed_data[2]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }
                                if (rb_yaw.Checked == true)
                                {
                                    _yaw_angle_points.Add(time_count + 150, passed_data[3]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_roll_pitch.Checked == true)
                                {
                                    _roll_angle_points.Add(time_count + 150, passed_data[1]);
                                    _pitch_angle_points.Add(time_count + 150, passed_data[2]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_roll_setpoint.Checked == true)
                                {
                                    _roll_angle_points.Add(time_count + 150, passed_data[1]);
                                    _rc_roll_points.Add(time_count + 150, passed_data[5]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_pitch_setpoint.Checked == true)
                                {
                                    _pitch_angle_points.Add(time_count + 150, passed_data[2]);
                                    _rc_pitch_points.Add(time_count + 150, passed_data[6]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_yaw_setpoint.Checked == true)
                                {
                                    _yaw_angle_points.Add(time_count + 150, passed_data[3]);
                                    _rc_yaw_points.Add(time_count + 150, passed_data[7]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_altitude.Checked == true)
                                {
                                    _altitude_points.Add(time_count + 150, passed_data[4]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_gyro.Checked == true)
                                {
                                    _gyro_x_points.Add(time_count + 150, passed_data[23]);
                                    _gyro_y_points.Add(time_count + 150, passed_data[24]);
                                    _gyro_z_points.Add(time_count + 150, passed_data[25]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                if (rb_motor.Checked == true)
                                {
                                    _motor_0_points.Add(time_count + 150, passed_data[15]);
                                    _motor_1_points.Add(time_count + 150, passed_data[16]);
                                    _motor_2_points.Add(time_count + 150, passed_data[17]);
                                    _motor_3_points.Add(time_count + 150, passed_data[18]);
                                    _myPane.XAxis.Scale.Min = time_count;
                                    _myPane.XAxis.Scale.Max = 300 + time_count;
                                }

                                lb_altitude.Text = passed_data[4].ToString();
                                lb_rc_roll.Text = passed_data[5].ToString();
                                lb_rc_pitch.Text = passed_data[6].ToString();
                                lb_rc_yaw.Text = passed_data[7].ToString();
                                lb_rc_throttle.Text = passed_data[8].ToString();
                                lb_lat.Text = passed_data[9].ToString();
                                lb_long.Text = passed_data[10].ToString();
                                lb_bat.Text = ((passed_data[11]*4)/100).ToString();
                                battery_bar_level = (int)(passed_data[11]*4)/10;

                                label55.Text = passed_data[12].ToString();
                                flight_mode = (UInt16)passed_data[12];

                                lb_fail.Text = passed_data[13].ToString();
                                error = (UInt16)passed_data[13];

                                lb_armed.Text = passed_data[14].ToString();
                                start = (byte)passed_data[14];

                                lb_motor0.Text = passed_data[15].ToString();
                                lb_motor1.Text = passed_data[16].ToString();
                                lb_motor2.Text = passed_data[17].ToString();
                                lb_motor3.Text = passed_data[18].ToString();

                                lb_debug0.Text = passed_data[19].ToString();
                                lb_debug1.Text = passed_data[20].ToString();
                                lb_debug2.Text = passed_data[21].ToString();
                                lb_debug3.Text = passed_data[22].ToString();

                                lb_gyro_X.Text = passed_data[23].ToString();
                                lb_gyro_Y.Text = passed_data[24].ToString();
                                lb_gyro_Z.Text = passed_data[25].ToString();

                                if(rb_roll.Checked == true || rb_pitch.Checked == true ||
                                   rb_yaw.Checked == true || rb_roll_pitch.Checked == true ||
                                   rb_roll_setpoint.Checked == true || rb_pitch_setpoint.Checked == true ||
                                   rb_yaw_setpoint.Checked == true || rb_altitude.Checked == true ||
                                   rb_gyro.Checked == true || rb_motor.Checked == true)
                                {
                                    zedGraphControl1.AxisChange();
                                    zedGraphControl1.Invalidate();
                                    zedGraphControl1.Refresh();
                                }
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

                //serialPort.Write(Encoding.UTF8.GetString(buff));
                serialPort.Write(buff, 0, 20);
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

                    //serialPort.Write(Encoding.UTF8.GetString(buff));
                    serialPort.Write(buff, 0, 20);
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
                    //serialPort.Write(Encoding.UTF8.GetString(buff));
                    serialPort.Write(buff, 0, 20);
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

                    //serialPort.Write(Encoding.UTF8.GetString(pid_buff));
                    serialPort.Write(pid_buff, 0, 76);
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

            if (flight_mode == 1) textBox2.Text = "ANGLE_MODE";
            if (flight_mode == 65) textBox2.Text = "HEADFREE_MODE";

            if (error == 0) textBox3.Text = "No error";
            if (error == 2) textBox3.Text = "RX_LOSS_DETECTED";
            if (error == 4) textBox3.Text = "RX_SWITCH";
            if (error == 8) textBox3.Text = "Battery LOW";
            //if (error == 2) textBox3.Text = "Program loop time";

            if (battery_bar_level > 164) battery_bar_level = 124;
            if (battery_bar_level < 110) battery_bar_level = 85;
            if (battery_bar_level > 142) panel5.BackColor = Color.Lime;
            else if (battery_bar_level > 132) panel5.BackColor = Color.Yellow;
            else panel5.BackColor = Color.Red;

            panel6.Size = new Size(34, 134 - ((battery_bar_level - 80) * 3));

            textBox10.Text = gMapControl1.Zoom.ToString();
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

        private void rb_yaw_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -10;
            _myPane.YAxis.Scale.Max = 370;
            _yaw_angle_curve = _myPane.AddCurve("YAW_Angle", _yaw_angle_points, Color.Blue, SymbolType.None);
            _yaw_angle_curve.Line.Width = 2;
            _yaw_angle_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_roll_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -60;
            _myPane.YAxis.Scale.Max = 60;
            _roll_angle_curve = _myPane.AddCurve("ROLL_Angle", _roll_angle_points, Color.Green, SymbolType.None);
            _roll_angle_curve.Line.Width = 2;
            _roll_angle_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_pitch_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -60;
            _myPane.YAxis.Scale.Max = 60;
            _pitch_angle_curve = _myPane.AddCurve("PITCH_Angle", _pitch_angle_points, Color.Red, SymbolType.None);
            _pitch_angle_curve.Line.Width = 2;
            _pitch_angle_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_roll_pitch_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -60;
            _myPane.YAxis.Scale.Max = 60;
            _roll_angle_curve = _myPane.AddCurve("ROLL_Angle", _roll_angle_points, Color.Green, SymbolType.None);
            _roll_angle_curve.Line.Width = 2;
            _roll_angle_points.Clear();

            _pitch_angle_curve = _myPane.AddCurve("PITCH_Angle", _pitch_angle_points, Color.Red, SymbolType.None);
            _pitch_angle_curve.Line.Width = 2;
            _pitch_angle_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_roll_setpoint_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -60;
            _myPane.YAxis.Scale.Max = 60;

            _roll_angle_curve = _myPane.AddCurve("ROLL_Angle", _roll_angle_points, Color.Blue, SymbolType.None);
            _roll_angle_curve.Line.Width = 2;
            _roll_angle_points.Clear();
            _rc_roll_curve = _myPane.AddCurve("ROLL_Setpoint", _rc_roll_points, Color.Red, SymbolType.None);
            _rc_roll_curve.Line.Width = 2;
            _rc_roll_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_pitch_setpoint_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -60;
            _myPane.YAxis.Scale.Max = 60;

            _pitch_angle_curve = _myPane.AddCurve("PITCH_Angle", _pitch_angle_points, Color.Blue, SymbolType.None);
            _pitch_angle_curve.Line.Width = 2;
            _pitch_angle_points.Clear();
            _rc_pitch_curve = _myPane.AddCurve("PITCH_Setpoint", _rc_pitch_points, Color.Red, SymbolType.None);
            _rc_pitch_curve.Line.Width = 2;
            _rc_pitch_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_yaw_setpoint_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -500;
            _myPane.YAxis.Scale.Max = 500;

            _yaw_angle_curve = _myPane.AddCurve("YAW_Angle", _yaw_angle_points, Color.Blue, SymbolType.None);
            _yaw_angle_curve.Line.Width = 2;
            _yaw_angle_points.Clear();
            _rc_yaw_curve = _myPane.AddCurve("YAW_Setpoint", _rc_yaw_points, Color.Red, SymbolType.None);
            _rc_yaw_curve.Line.Width = 2;
            _rc_yaw_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_none_MouseDown(object sender, MouseEventArgs e)
        {
            zedGraphControl1.GraphPane.CurveList.Clear();

            _myPane.YAxis.Scale.Min = -45;
            _myPane.YAxis.Scale.Max = 45;

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_altitude_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -10;
            _myPane.YAxis.Scale.Max = 200;

            _altitude_curve = _myPane.AddCurve("Altitude", _altitude_points, Color.Blue, SymbolType.None);
            _altitude_curve.Line.Width = 2;
            _altitude_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_gyro_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -500;
            _myPane.YAxis.Scale.Max = 500;

            _gyro_x_curve = _myPane.AddCurve("Gyro_[X]", _gyro_x_points, Color.Blue, SymbolType.None);
            _gyro_x_curve.Line.Width = 2;
            _gyro_x_points.Clear();
            _gyro_y_curve = _myPane.AddCurve("Gyro_[Y]", _gyro_y_points, Color.Red, SymbolType.None);
            _gyro_y_curve.Line.Width = 2;
            _gyro_y_points.Clear();
            _gyro_z_curve = _myPane.AddCurve("Gyro_[Z]", _gyro_z_points, Color.Green, SymbolType.None);
            _gyro_z_curve.Line.Width = 2;
            _gyro_z_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_motor_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = 10000;
            _myPane.YAxis.Scale.Max = 23000;

            _motor_0_curve = _myPane.AddCurve("MOTOR[0]", _motor_0_points, Color.Blue, SymbolType.None);
            _motor_0_curve.Line.Width = 2;
            _motor_0_points.Clear();
            _motor_1_curve = _myPane.AddCurve("MOTOR[1]", _motor_1_points, Color.Red, SymbolType.None);
            _motor_1_curve.Line.Width = 2;
            _motor_1_points.Clear();
            _motor_2_curve = _myPane.AddCurve("MOTOR[2]", _motor_2_points, Color.Green, SymbolType.None);
            _motor_2_curve.Line.Width = 2;
            _motor_2_points.Clear();
            _motor_3_curve = _myPane.AddCurve("MOTOR[3]", _motor_3_points, Color.Brown, SymbolType.None);
            _motor_3_curve.Line.Width = 2;
            _motor_3_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void bt_zoom_p_Click(object sender, EventArgs e)
        {
            gMapControl1.Zoom++;
        }

        private void bt_zoom_m_Click(object sender, EventArgs e)
        {
            gMapControl1.Zoom--;
        }
    }
}
