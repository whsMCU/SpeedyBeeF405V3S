using Ball_Ballancer_CS.Class;
using SpeedyBeeF405V3S_GUI.Class;
using GMap.NET;
using GMap.NET.MapProviders;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO.Ports;
using System.Timers;
using System.Windows.Forms;
using ZedGraph;
using OfficeOpenXml;
using System.IO;
using System.Text.RegularExpressions;
using System.Diagnostics;
using System.Globalization;
using System.Security.Cryptography;
using System.Threading.Tasks;

namespace SpeedyBeeF405V3S_GUI
{
    public partial class Form1 : Form
    {
        private static System.Timers.Timer AHRS_Timer;
        float[] passed_data = new float[50];
        float[] float_data = new float[10];
        float[] float_data_pid = new float[18];
        UTF8 UTF8 = new UTF8();
        Msp_Protocol protocol = new Msp_Protocol();
        MspProtocol mspProtocol = new MspProtocol();

        DateTime temp, end;

        /// <ExcelDataSave>
        string filePath = "RealTimeData.xlsx";
        string PID_log_filePath;
        int row = 2;

        private ArrayList al;

        bool drone_status_flag = true;
        bool pid_recive_flag = false;
        bool pid_send_flag = false;
        bool pid_save_flag = false;
        bool acc_cal_flag = false;
        bool mag_cal_flag = false;
        bool mag_cal_remain_time_flag = false;
        bool RP_Coupling = true;
        int mag_cal_remain_time = 0;
        bool pid_test_flag = false;
        bool pid_test_flag_temp = false;
        bool pid_test_request_flag = false;
        int pid_test_time = 0;
        int pid_test_setting_time_temp = 0;
        int pid_test_setting_deg_temp = 0;
        int pid_test_setting_time = 0;
        int pid_test_setting_deg = 0;
        int pid_test_setting_throttle = 0;

        enum pidState_e
        {
            TEST_IDLE,
            TEST_Step1,
            TEST_Step2,
            TEST_Step3,
            TEST_Step4,
            TEST_Step5,
            TEST_Step6,
            TEST_Step7,
            TEST_Step8,
            TEST_Step9,
            TEST_Step10,
            TEST_FINISH
        }

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
        PointPairList _yaw_reference_points = new PointPairList();
        LineItem _yaw_reference_curve;
        PointPairList _alt_points = new PointPairList();
        LineItem _alt_curve;
        PointPairList _alt_reference_points = new PointPairList();
        LineItem _alt_reference_curve;

        PointPairList _Throttle_points = new PointPairList();
        LineItem _Throttle_curve;
        PointPairList _Throttle_Hold_points = new PointPairList();
        LineItem _Throttle_Hold_curve;
        PointPairList _alt_pidresult_points = new PointPairList();
        LineItem _alt_pidresult_curve;

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
            InitExcel();

            mspProtocol.OnPacketReceived += (command, payload) =>
            {
                switch (command)
                {
                    case 6:
                        Msp_pid_data(payload);
                        break;
                    case 101: // MSP_RAW_IMU
                        Msp_raw_data(payload);
                        break;
                    default:
                        break;
                }
            };
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

        public void InitExcel()
        {
            ExcelPackage.LicenseContext = LicenseContext.NonCommercial;
        }
        public void InitLogger(string filePath)
        {
            using (StreamWriter writer = new StreamWriter(filePath, append: true))
            {
                writer.AutoFlush = true;

                float_data_pid[0] = float.Parse(tb_R_I_P.Text);
                float_data_pid[1] = float.Parse(tb_R_I_I.Text);
                float_data_pid[2] = float.Parse(tb_R_I_D.Text);

                float_data_pid[3] = float.Parse(tb_R_O_P.Text);
                float_data_pid[4] = float.Parse(tb_R_O_I.Text);
                float_data_pid[5] = float.Parse(tb_R_O_D.Text);

                float_data_pid[6] = float.Parse(tb_P_I_P.Text);
                float_data_pid[7] = float.Parse(tb_P_I_I.Text);
                float_data_pid[8] = float.Parse(tb_P_I_D.Text);

                float_data_pid[9] = float.Parse(tb_P_O_P.Text);
                float_data_pid[10] = float.Parse(tb_P_O_I.Text);
                float_data_pid[11] = float.Parse(tb_P_O_D.Text);

                float_data_pid[12] = float.Parse(tb_Y_A_P.Text);
                float_data_pid[13] = float.Parse(tb_Y_A_I.Text);
                float_data_pid[14] = float.Parse(tb_Y_A_D.Text);

                float_data_pid[15] = float.Parse(tb_Y_R_P.Text);
                float_data_pid[16] = float.Parse(tb_Y_R_I.Text);
                float_data_pid[17] = float.Parse(tb_Y_R_D.Text);

                string log = "R_I_P, R_I_I, R_I_D";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
                log = $"{float_data_pid[0]}, {float_data_pid[1]}, {float_data_pid[2]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력

                log = "R_O_P, R_O_I, R_O_D";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
                log = $"{float_data_pid[3]}, {float_data_pid[4]}, {float_data_pid[5]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력

                log = "P_I_P, P_I_I, P_I_D";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
                log = $"{float_data_pid[6]}, {float_data_pid[7]}, {float_data_pid[8]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력

                log = "P_O_P, P_O_I, P_O_D";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
                log = $"{float_data_pid[9]}, {float_data_pid[10]}, {float_data_pid[11]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력

                log = "Y_A_P, Y_A_I, Y_A_D";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
                log = $"{float_data_pid[12]}, {float_data_pid[13]}, {float_data_pid[14]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력

                log = "Y_R_P, Y_R_I, Y_R_D";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
                log = $"{float_data_pid[15]}, {float_data_pid[16]}, {float_data_pid[17]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력

                log = "DateTime, Roll, Pitch, Yaw, Alt, RollSetPoint, PitchSetPoint, Yaw SetPoint, Thorttle, yaw_heading_reference, altHold, lattitude, longitude, Sat_Num, gps_fix, Throttle_Hold_point, Alt_PID_Result, Debug[0], Debug[1], Debug[2], Debug[3]";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
            }
        }

        static void Data_Log(string filePath, float[] data, float setpoint)
        {
            using (StreamWriter writer = new StreamWriter(filePath, append: true))
            {
                writer.AutoFlush = true;
                data[1] /= 10;
                data[2] /= 10;

                string log = $"{DateTime.Now:HH:mm:ss.fff}, {data[1]}, {data[2]}, {data[3]}, {data[4]}, 0, {setpoint},0";
                writer.WriteLine(log);
                //Console.WriteLine(log); // 콘솔에도 출력
            }
        }

        static void Check_Data_Log(string filePath, float[] data)
        {
            using (StreamWriter writer = new StreamWriter(filePath, append: true))
            {
                writer.AutoFlush = true;
                data[0] /= 10;
                data[1] /= 10;
                data[4] /= 10;
                data[5] /= 10;
                string log = $"{DateTime.Now:HH:mm:ss.fff}, {data[0]}, {data[1]}, {data[2]}, {data[3]}, {data[4]}, {data[5]}, {data[6]}, {data[7]}, {data[41]}, {data[42]}, {data[8]}, {data[9]}, {data[43]}, {data[44]}, {data[45]}, {data[46]},{data[18]}, {data[19]}, {data[20]}, {data[21]}";
                writer.WriteLine(log);
                Console.WriteLine(log); // 콘솔에도 출력
            }
        }

        private void gMapControl1_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
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
            if (e.Button == MouseButtons.Right)
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
                    if (comboBox_port.Text.Length > 1)
                    {
                        serialPort.PortName = comboBox_port.Text;  //콤보박스의 선택된 COM포트명을 시리얼포트명으로 지정
                        serialPort.BaudRate = 115200;  //보레이트 변경이 필요하면 숫자 변경하기
                        serialPort.DataBits = 8;
                        serialPort.StopBits = StopBits.One;
                        serialPort.Parity = Parity.None;
                        serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler); //이것이 꼭 필요하다

                        serialPort.Open();  //시리얼포트 열기

                        mspProtocol.Open(serialPort); // MSP Protocol 객체에 시리얼포트 정보 전송

                        OpenClose.Text = "Close";
                        comboBox_port.Enabled = false;  //COM포트설정 콤보박스 비활성화

                        AHRS_Timer = new System.Timers.Timer(50);
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
            try
            {
                if (received_data == 0) received_data = 1;

                while (serialPort.BytesToRead > 0)
                {
                    byte b = (byte)serialPort.ReadByte();
                    mspProtocol.ParseByte(b);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Serial receive error: {ex.Message}");
            }
        }

        pidState_e pidState = pidState_e.TEST_IDLE;

        private void OnTimedEvent(object source, ElapsedEventArgs e)
        {
            byte[] buff = new byte[20];
            if(drone_status_flag == true)
            {
                try
                {
                    mspProtocol.SendMspCommand(101);
                }
                catch { Console.WriteLine("STATUS DATA Requset Error"); }
            }

            if (pid_recive_flag == true)
            {
                pid_recive_flag = false;
                drone_status_flag = true;

                try
                {
                    mspProtocol.SendMspCommand(6);
                }
                catch { Console.WriteLine("PID Data Requset Error"); }
            }
            if (pid_save_flag == true)
            {
                pid_save_flag = false;
                drone_status_flag = true;
                try
                {
                    mspProtocol.SendMspCommand(7);
                    Console.WriteLine("PID값 저장명령 전송 완료");
                }
                catch { Console.WriteLine("PID Save Requset Error"); }
            }
            if (pid_send_flag == true)
            {
                pid_send_flag = false;
                drone_status_flag = true;
                byte[] pid_buff = new byte[84];
                float float_buff;
                byte[] tmp = new byte[4];
                try
                {
                    float_buff = float.Parse(tb_R_I_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[0] = tmp[0];
                    pid_buff[1] = tmp[1];
                    pid_buff[2] = tmp[2];
                    pid_buff[3] = tmp[3];
                    float_buff = float.Parse(tb_R_I_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[4] = tmp[0];
                    pid_buff[5] = tmp[1];
                    pid_buff[6] = tmp[2];
                    pid_buff[7] = tmp[3];
                    float_buff = float.Parse(tb_R_I_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[8] = tmp[0];
                    pid_buff[9] = tmp[1];
                    pid_buff[10] = tmp[2];
                    pid_buff[11] = tmp[3];

                    float_buff = float.Parse(tb_R_O_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[12] = tmp[0];
                    pid_buff[13] = tmp[1];
                    pid_buff[14] = tmp[2];
                    pid_buff[15] = tmp[3];
                    float_buff = float.Parse(tb_R_O_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[16] = tmp[0];
                    pid_buff[17] = tmp[1];
                    pid_buff[18] = tmp[2];
                    pid_buff[19] = tmp[3];
                    float_buff = float.Parse(tb_R_O_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[20] = tmp[0];
                    pid_buff[21] = tmp[1];
                    pid_buff[22] = tmp[2];
                    pid_buff[23] = tmp[3];

                    float_buff = float.Parse(tb_P_I_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[24] = tmp[0];
                    pid_buff[25] = tmp[1];
                    pid_buff[26] = tmp[2];
                    pid_buff[27] = tmp[3];

                    float_buff = float.Parse(tb_P_I_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[28] = tmp[0];
                    pid_buff[29] = tmp[1];
                    pid_buff[30] = tmp[2];
                    pid_buff[31] = tmp[3];

                    float_buff = float.Parse(tb_P_I_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[32] = tmp[0];
                    pid_buff[33] = tmp[1];
                    pid_buff[34] = tmp[2];
                    pid_buff[35] = tmp[3];

                    float_buff = float.Parse(tb_P_O_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[36] = tmp[0];
                    pid_buff[37] = tmp[1];
                    pid_buff[38] = tmp[2];
                    pid_buff[39] = tmp[3];
                    float_buff = float.Parse(tb_P_O_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[40] = tmp[0];
                    pid_buff[41] = tmp[1];
                    pid_buff[42] = tmp[2];
                    pid_buff[43] = tmp[3];
                    float_buff = float.Parse(tb_P_O_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[44] = tmp[0];
                    pid_buff[45] = tmp[1];
                    pid_buff[46] = tmp[2];
                    pid_buff[47] = tmp[3];

                    float_buff = float.Parse(tb_Y_A_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[48] = tmp[0];
                    pid_buff[49] = tmp[1];
                    pid_buff[50] = tmp[2];
                    pid_buff[51] = tmp[3];
                    float_buff = float.Parse(tb_Y_A_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[52] = tmp[0];
                    pid_buff[53] = tmp[1];
                    pid_buff[54] = tmp[2];
                    pid_buff[55] = tmp[3];
                    float_buff = float.Parse(tb_Y_A_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[56] = tmp[0];
                    pid_buff[57] = tmp[1];
                    pid_buff[58] = tmp[2];
                    pid_buff[59] = tmp[3];

                    float_buff = float.Parse(tb_Y_R_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[60] = tmp[0];
                    pid_buff[61] = tmp[1];
                    pid_buff[62] = tmp[2];
                    pid_buff[63] = tmp[3];
                    float_buff = float.Parse(tb_Y_R_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[64] = tmp[0];
                    pid_buff[65] = tmp[1];
                    pid_buff[66] = tmp[2];
                    pid_buff[67] = tmp[3];
                    float_buff = float.Parse(tb_Y_R_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[68] = tmp[0];
                    pid_buff[69] = tmp[1];
                    pid_buff[70] = tmp[2];
                    pid_buff[71] = tmp[3];

                    float_buff = float.Parse(tb_ALT_P.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[72] = tmp[0];
                    pid_buff[73] = tmp[1];
                    pid_buff[74] = tmp[2];
                    pid_buff[75] = tmp[3];

                    float_buff = float.Parse(tb_ALT_I.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[76] = tmp[0];
                    pid_buff[77] = tmp[1];
                    pid_buff[78] = tmp[2];
                    pid_buff[79] = tmp[3];

                    float_buff = float.Parse(tb_ALT_D.Text);
                    tmp = BitConverter.GetBytes(float_buff);
                    pid_buff[80] = tmp[0];
                    pid_buff[81] = tmp[1];
                    pid_buff[82] = tmp[2];
                    pid_buff[83] = tmp[3];

                    mspProtocol.SendMspCommand(95, pid_buff);

                    Console.WriteLine("PID값 전송 완료");
                }
                catch { Console.WriteLine("PID Data Send Error"); }
            }

            if (acc_cal_flag == true)
            {
                
                acc_cal_flag = false;
                drone_status_flag = true;
                try
                {
                    mspProtocol.SendMspCommand(205);
                    Console.WriteLine("가속도센서 캘리브레이션 명령 전송 완료");
                }
                catch { Console.WriteLine("ACC Calibration Requset Error"); }
            }

            if (mag_cal_flag == true)
            {
                mag_cal_flag = false;
                drone_status_flag = true;
                try
                {
                    mspProtocol.SendMspCommand(206);
                    Console.WriteLine("지자계 캘리브레이션 명령 전송 완료");
                }
                catch { Console.WriteLine("MAG Calibration Requset Error"); }
            }

            if (pid_test_flag == true)
            {
                if (pid_test_request_flag == true)
                {
                    byte[] pid_buff = new byte[9];
                    byte[] tmp = new byte[4];
                    pid_test_request_flag = false;
                    drone_status_flag = true;
                    try
                    {
                        pid_buff[0] = Convert.ToByte(pid_test_flag_temp);

                        tmp = BitConverter.GetBytes(pid_test_setting_throttle);
                        pid_buff[1] = tmp[0];
                        pid_buff[2] = tmp[1];
                        pid_buff[3] = tmp[2];
                        pid_buff[4] = tmp[3];

                        tmp = BitConverter.GetBytes(pid_test_setting_deg_temp);
                        pid_buff[5] = tmp[0];
                        pid_buff[6] = tmp[1];
                        pid_buff[7] = tmp[2];
                        pid_buff[8] = tmp[3];

                        mspProtocol.SendMspCommand(8, pid_buff);

                        Console.WriteLine($"PID 테스트 Step flag : {pid_test_flag_temp}, throttle : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp} 신호 전송");
                    }
                    catch { Console.WriteLine("PID 테스트 Step1 Signal Requset Error"); }
                }

            }

            if (pid_test_flag == true)
            {
                pid_test_time++;
                this.Invoke((MethodInvoker)delegate
                {
                    lb_PID_Test_Status.Text = "PID_Control_Testing...";
                    lb_PID_Test_Progress_Time.Text = (pid_test_time * 50).ToString();
                    lb_PID_Test_Target_Time.Text = (pid_test_setting_time_temp * 50).ToString();
                });
                switch (pidState)
                {
                    case pidState_e.TEST_IDLE:
                        this.Invoke((MethodInvoker)delegate
                        {
                            tb_PID_ms.Enabled = false;
                            tb_PID_Deg.Enabled = false;
                            tb_PID_Throttle.Enabled = false;
                        });
                        pid_test_time = 0;
                        pid_test_setting_time = int.Parse(tb_PID_ms.Text) / 50;
                        pid_test_setting_time_temp = pid_test_setting_time;
                        pid_test_setting_deg = int.Parse(tb_PID_Deg.Text);
                        pid_test_setting_deg_temp = 0;
                        pid_test_setting_throttle = int.Parse(tb_PID_Throttle.Text);
                        pid_test_flag_temp = pid_test_flag;
                        pid_test_request_flag = true;

                        this.Invoke((MethodInvoker)delegate
                        {
                            lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 0";
                        });

                        Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 0");
                        pidState = pidState_e.TEST_Step1;
                        break;

                    case pidState_e.TEST_Step1:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_deg_temp = pid_test_setting_deg;
                            pid_test_request_flag = true;
                            temp = DateTime.Now;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 1";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 1");
                            pidState = pidState_e.TEST_Step2;
                        }
                        break;

                    case pidState_e.TEST_Step2:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_deg_temp = 0;
                            pid_test_request_flag = true;
                            end = DateTime.Now;
                            Console.WriteLine($"DT: {(end - temp).TotalMilliseconds}");
                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 2";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 2");
                            pidState = pidState_e.TEST_Step3;
                        }
                        break;

                    case pidState_e.TEST_Step3:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_deg_temp = -pid_test_setting_deg;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 3";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 3");
                            pidState = pidState_e.TEST_Step4;
                        }
                        break;

                    case pidState_e.TEST_Step4:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_deg_temp = 0;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 4";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 4");
                            pidState = pidState_e.TEST_Step5;
                        }
                        break;

                    case pidState_e.TEST_Step5:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time / 50;
                            pid_test_setting_deg_temp = pid_test_setting_deg;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 5";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 5");
                            pidState = pidState_e.TEST_Step6;
                        }
                        break;

                    case pidState_e.TEST_Step6:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_deg_temp = 0;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 6";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 6");
                            pidState = pidState_e.TEST_Step7;
                        }
                        break;

                    case pidState_e.TEST_Step7:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time / 50;
                            pid_test_setting_deg_temp = -pid_test_setting_deg;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 7";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 7");
                            pidState = pidState_e.TEST_Step8;
                        }
                        break;

                    case pidState_e.TEST_Step8:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_deg_temp = 0;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 8";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 8");
                            pidState = pidState_e.TEST_Step9;
                        }
                        break;

                    case pidState_e.TEST_Step9:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_time_temp += pid_test_setting_time;
                            pid_test_setting_throttle = 0;
                            pid_test_setting_deg_temp = 0;
                            pid_test_flag_temp = false;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 9";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 9");
                            pidState = pidState_e.TEST_Step10;
                        }
                        break;

                    case pidState_e.TEST_Step10:
                        if (pid_test_time >= pid_test_setting_time_temp)
                        {
                            pid_test_setting_throttle = 0;
                            pid_test_setting_deg_temp = 0;
                            pid_test_flag_temp = false;
                            pid_test_request_flag = true;

                            this.Invoke((MethodInvoker)delegate
                            {
                                lb_PID_Test_Progress.Text = $"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 10";
                            });

                            Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : 10");
                            pidState = pidState_e.TEST_FINISH;
                        }
                        break;

                    case pidState_e.TEST_FINISH:
                        this.Invoke((MethodInvoker)delegate
                        {
                            tb_PID_ms.Enabled = true;
                            tb_PID_Deg.Enabled = true;
                            tb_PID_Throttle.Enabled = true;
                        });

                        pid_test_flag = false;
                        pid_test_time = 0;
                        pid_test_setting_time_temp = 0;
                        pid_test_request_flag = false;

                        this.Invoke((MethodInvoker)delegate
                        {
                            lb_PID_Test_Status.Text = "PID_Control_Testing_Finished!";
                        });

                        Console.WriteLine($"Thr : {pid_test_setting_throttle}, Deg : {pid_test_setting_deg_temp}, Step : FINISH");
                        pidState = pidState_e.TEST_IDLE;
                        break;

                    default:
                        tb_PID_ms.Enabled = true;
                        tb_PID_Deg.Enabled = true;
                        tb_PID_Throttle.Enabled = true;
                        pid_test_flag = false;
                        pid_test_time = 0;
                        pid_test_setting_time_temp = 0;
                        pid_test_request_flag = false;
                        pidState = pidState_e.TEST_IDLE;
                        break;
                }
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


        private void record_indicator_on()
        {
            Graphics g = panel8.CreateGraphics();
            Pen p = new Pen(Color.Red);
            SolidBrush sb = new SolidBrush(Color.Red);
            g.DrawEllipse(p, 1, 1, 15, 15);
            g.FillEllipse(sb, 1, 1, 15, 15);
        }

        private void record_indicator_off()
        {
            Graphics g = panel8.CreateGraphics();
            Pen p = new Pen(Color.LightGray);
            SolidBrush sb = new SolidBrush(Color.LightGray);
            g.DrawEllipse(p, 1, 1, 15, 15);
            g.FillEllipse(sb, 1, 1, 15, 15);
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

            if ((passed_data[43] >= 6) && (passed_data[44] == 2)) // GPS_Num > 6 && GPS_FIX == 2
            {
                pictureBox4.Visible = false;
                pictureBox5.Visible = false;
                pictureBox6.Visible = true;

            }
            else if (passed_data[43] > 3) // GPS_Num > 3
            {
                pictureBox4.Visible = false;
                pictureBox5.Visible = true;
                pictureBox6.Visible = false;

            }
            else
            {
                pictureBox4.Visible = true;
                pictureBox5.Visible = false;
                pictureBox6.Visible = false;

            }

            if (battery_bar_level > 164) battery_bar_level = 124;
            if (battery_bar_level < 110) battery_bar_level = 85;
            if (battery_bar_level > 142) panel5.BackColor = Color.Lime;
            else if (battery_bar_level > 132) panel5.BackColor = Color.Yellow;
            else panel5.BackColor = Color.Red;

            panel6.Size = new Size(34, 134 - ((battery_bar_level - 80) * 3));

            textBox10.Text = gMapControl1.Zoom.ToString();
            tb_msp_error.Text = mspProtocol.GetMspError().ToString();
        }

        private void flight_timer_Tick(object sender, EventArgs e)
        {
            if (start == 1)
            {
                flight_timer_seconds++;
                label36.Text = ("00:" + (flight_timer_seconds / 60).ToString("00.") + ":" + (flight_timer_seconds % 60).ToString("00."));
            }

            if (mag_cal_remain_time_flag == true)
            {
                lb_magcal_remain_time.Text = mag_cal_remain_time.ToString();
                mag_cal_remain_time--;
                if (mag_cal_remain_time < 0)
                {
                    mag_cal_remain_time_flag = false;
                    lb_magcal_remain_time.Text = "완료";
                }
            }

            if (cb_record.Checked == true)
            {
                program_timer_seconds++;
                if (program_timer_seconds % 2 == 0)
                {
                    record_indicator_on();
                }
                else
                {
                    record_indicator_off();
                }
            }
            else
            {
                record_indicator_off();
            }
        }

        private void bt_pid_recive_Click(object sender, EventArgs e)
        {
            pid_recive_flag = true;
            drone_status_flag = false;
        }

        private void bt_pid_send_Click(object sender, EventArgs e)
        {
            pid_send_flag = true;
            drone_status_flag = false;
        }

        private void bt_pid_save_Click(object sender, EventArgs e)
        {
            pid_save_flag = true;
            drone_status_flag = false;
        }

        private void bt_acc_cal_Click(object sender, EventArgs e)
        {
            acc_cal_flag = true;
            drone_status_flag = false;
        }

        private void bt_mag_cal_Click(object sender, EventArgs e)
        {
            mag_cal_flag = true;
            mag_cal_remain_time_flag = true;
            mag_cal_remain_time = 31;
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
            tb_P_O_D.Text = tb_FC_P_O_D.Text;

            tb_Y_A_P.Text = tb_FC_Y_A_P.Text;
            tb_Y_A_I.Text = tb_FC_Y_A_I.Text;
            tb_Y_A_D.Text = tb_FC_Y_A_D.Text;

            tb_Y_R_P.Text = tb_FC_Y_R_P.Text;
            tb_Y_R_I.Text = tb_FC_Y_R_I.Text;
            tb_Y_R_D.Text = tb_FC_Y_R_D.Text;

            tb_ALT_P.Text = tb_FC_ALT_P.Text;
            tb_ALT_I.Text = tb_FC_ALT_I.Text;
            tb_ALT_D.Text = tb_FC_ALT_D.Text;
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
            _yaw_reference_curve = _myPane.AddCurve("YAW_Reference", _yaw_reference_points, Color.Yellow, SymbolType.None);
            _yaw_reference_curve.Line.Width = 2;
            _yaw_reference_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_alt_setpoint_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.Min = -500;
            _myPane.YAxis.Scale.Max = 500;

            _alt_curve = _myPane.AddCurve("ALT(CM)", _alt_points, Color.Blue, SymbolType.None);
            _alt_curve.Line.Width = 2;
            _alt_points.Clear();

            _alt_reference_curve = _myPane.AddCurve("ALT_Reference", _alt_reference_points, Color.Yellow, SymbolType.None);
            _alt_reference_curve.Line.Width = 2;
            _alt_reference_points.Clear();

            _Throttle_Hold_curve = _myPane.AddCurve("Throttle_Hold", _Throttle_Hold_points, Color.Green, SymbolType.None);
            _Throttle_Hold_curve.Line.Width = 2;
            _Throttle_Hold_points.Clear();

            _alt_pidresult_curve = _myPane.AddCurve("ALT_PID_Result", _alt_pidresult_points, Color.Black, SymbolType.None);
            _alt_pidresult_curve.Line.Width = 2;
            _alt_pidresult_points.Clear();

            _Throttle_curve = _myPane.AddCurve("Throttle_Hold", _Throttle_points, Color.Red, SymbolType.None);
            _Throttle_curve.Line.Width = 2;
            _Throttle_points.Clear();


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

            _motor_0_curve = _myPane.AddCurve("MOTOR[0][RR]", _motor_0_points, Color.Blue, SymbolType.None);
            _motor_0_curve.Line.Width = 2;
            _motor_0_points.Clear();
            _motor_1_curve = _myPane.AddCurve("MOTOR[1][RF]", _motor_1_points, Color.Red, SymbolType.None);
            _motor_1_curve.Line.Width = 2;
            _motor_1_points.Clear();
            _motor_2_curve = _myPane.AddCurve("MOTOR[2][LR]", _motor_2_points, Color.Green, SymbolType.None);
            _motor_2_curve.Line.Width = 2;
            _motor_2_points.Clear();
            _motor_3_curve = _myPane.AddCurve("MOTOR[3][LF]", _motor_3_points, Color.Brown, SymbolType.None);
            _motor_3_curve.Line.Width = 2;
            _motor_3_points.Clear();

            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            zedGraphControl1.Refresh();
        }

        private void rb_debug_MouseDown(object sender, MouseEventArgs e)
        {
            _myPane.CurveList.Clear();
            _myPane.YAxis.Scale.MinAuto = true;
            _myPane.YAxis.Scale.MaxAuto = true;

            _debug_0_curve = _myPane.AddCurve("DEBUG_[0]", _debug_0_points, Color.Blue, SymbolType.None);
            _debug_0_curve.Line.Width = 2;
            _debug_0_curve.Clear();
            _debug_1_curve = _myPane.AddCurve("DEBUG_[1]", _debug_1_points, Color.Red, SymbolType.None);
            _debug_1_curve.Line.Width = 2;
            _debug_1_curve.Clear();
            _debug_2_curve = _myPane.AddCurve("DEBUG_[2]", _debug_2_points, Color.Green, SymbolType.None);
            _debug_2_curve.Line.Width = 2;
            _debug_2_curve.Clear();
            _debug_3_curve = _myPane.AddCurve("DEBUG_[3]", _debug_3_points, Color.Black, SymbolType.None);
            _debug_3_curve.Line.Width = 2;
            _debug_3_curve.Clear();

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

        private void cB_RP_Coupling_CheckedChanged(object sender, EventArgs e)
        {
            if (RP_Coupling == false)
            {
                RP_Coupling = true;
            }
            else
            {
                RP_Coupling = false;
            }
        }

        private void tb_R_I_P_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_R_I_P_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_I_P_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_R_I_P.Text;
                tb_P_I_P.Text = text;
            }

        }

        private void tb_P_I_P_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_P_I_P_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_P_I_P_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_P_I_P.Text;
                tb_R_I_P.Text = text;
            }
        }

        private void tb_R_I_I_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_R_I_I_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_I_I_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_R_I_I.Text;
                tb_P_I_I.Text = text;
            }
        }

        private void tb_P_I_I_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_P_I_I_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_P_I_I_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_P_I_I.Text;
                tb_R_I_I.Text = text;
            }
        }

        private void tb_R_I_D_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_R_I_D_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_I_D_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_R_I_D.Text;
                tb_P_I_D.Text = text;
            }
        }

        private void tb_P_I_D_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_P_I_D_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_P_I_D_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_P_I_D.Text;
                tb_R_I_D.Text = text;
            }
        }

        private void tb_R_O_P_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_R_O_P_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_O_P_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_R_O_P.Text;
                tb_P_O_P.Text = text;
            }
        }

        private void tb_P_O_P_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_P_O_P_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_O_P_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_P_O_P.Text;
                tb_R_O_P.Text = text;
            }
        }

        private void tb_R_O_I_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_R_O_I_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_O_I_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_R_O_I.Text;
                tb_P_O_I.Text = text;
            }
        }

        private void tb_P_O_I_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_P_O_I_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_P_O_I_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_P_O_I.Text;
                tb_R_O_I.Text = text;
            }
        }

        private void tb_R_O_D_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_R_O_D_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_R_O_D_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_R_O_D.Text;
                tb_P_O_D.Text = text;
            }
        }

        private void tb_P_O_D_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = sender as TextBox;
            string text_temp = textBox.Text;

            // 소수점 숫자만 허용: "", ".", ".5", "0.5", "123", "123.456"
            string pattern = @"^(\d+)?(\.\d*)?$";

            if (Regex.IsMatch(textBox.Text, pattern))
            {
                text_temp = textBox.Text;
            }
            else
            {
                int cursorPos = textBox.SelectionStart - 1;
                textBox.TextChanged -= tb_P_O_D_TextChanged; // 이벤트 일시 해제
                textBox.Text = text_temp;
                textBox.TextChanged += tb_P_O_D_TextChanged; // 다시 연결

                if (cursorPos >= 0 && cursorPos <= textBox.Text.Length)
                {
                    textBox.SelectionStart = cursorPos;
                }
            }

            if (RP_Coupling == true)
            {
                string text = tb_P_O_D.Text;
                tb_R_O_D.Text = text;
            }
        }

        private void bt_ReScan_Click(object sender, EventArgs e)
        {
            try
            {
                comboBox_port.DataSource = SerialPort.GetPortNames(); //연결 가능한 시리얼포트 이름을 콤보박스에 가져오기 
            }
            catch { }
        }

        private void bt_start_pid_test_Click(object sender, EventArgs e)
        {
            // 바탕화면 경로 가져오기
            string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

            // 로그 폴더 만들기 (선택 사항)
            string folderPath = Path.Combine(desktopPath, "PID_Log");
            if (!Directory.Exists(folderPath))
            {
                Directory.CreateDirectory(folderPath);
            }

            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string fileName = $"log_{timestamp}.txt";

            PID_log_filePath = Path.Combine(folderPath, fileName);
            InitLogger(PID_log_filePath);

            pid_test_flag = true;
        }

        private void cb_record_CheckedChanged(object sender, EventArgs e)
        {
            if (cb_record.Checked == true)
            {
                // 바탕화면 경로 가져오기
                string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

                // 로그 폴더 만들기 (선택 사항)
                string folderPath = Path.Combine(desktopPath, "PID_Log");
                if (!Directory.Exists(folderPath))
                {
                    Directory.CreateDirectory(folderPath);
                }

                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string fileName = $"log_{timestamp}.txt";

                PID_log_filePath = Path.Combine(folderPath, fileName);
                InitLogger(PID_log_filePath);
            }
        }

        private void bt_open_folder_Click(object sender, EventArgs e)
        {
            // 실행 중인 프로그램의 폴더 경로 가져오기
            string folderPath = AppDomain.CurrentDomain.BaseDirectory;

            // 파일 탐색기로 폴더 열기
            Process.Start("explorer.exe", folderPath);
        }

        bool Msp_raw_data(byte[] payload)
        {
            DateTime date_time = DateTime.Now;
            int ms = date_time.Millisecond;
            time_count++;

            passed_data[0] = BitConverter.ToInt16(payload, 0) / 10;                  // attitude_roll
            passed_data[1] = BitConverter.ToInt16(payload, 2) / 10;                  // attitude_pitch
            passed_data[2] = BitConverter.ToUInt16(payload, 4) / 100;                // attitude_yaw
            passed_data[3] = BitConverter.ToInt16(payload, 6) / 10;                  // Altitude_Cm
            passed_data[4] = BitConverter.ToInt16(payload, 8) / 10;                  // RC_ROLL
            passed_data[5] = BitConverter.ToInt16(payload, 10) / 10;                 // RC_PITCH
            passed_data[6] = BitConverter.ToInt16(payload, 12);                      // RC_YAW
            passed_data[7] = (BitConverter.ToInt16(payload, 14) -1000) / 10;         // RC_Throttole
            passed_data[8] = BitConverter.ToInt32(payload, 16);     // posllh.lat
            passed_data[9] = BitConverter.ToInt32(payload, 20);     // posllh.lon
            passed_data[10] = BitConverter.ToInt16(payload, 24);    // batteryAverageCellVoltage
            passed_data[11] = BitConverter.ToUInt16(payload, 26);   // flightMode_Flags
            passed_data[12] = BitConverter.ToInt16(payload, 28);    // failsafe_Flags
            passed_data[13] = BitConverter.ToInt16(payload, 30);    // ARMING_Flags
            passed_data[14] = BitConverter.ToUInt16(payload, 32);   // Motor[R_R]
            passed_data[15] = BitConverter.ToUInt16(payload, 34);   // Motor[R_F]
            passed_data[16] = BitConverter.ToUInt16(payload, 36);   // Motor[L_R]
            passed_data[17] = BitConverter.ToUInt16(payload, 38);   // Motor[L_F]
            passed_data[18] = BitConverter.ToInt32(payload, 40);    // Debug[0]
            passed_data[19] = BitConverter.ToInt32(payload, 44);    // Debug[1]
            passed_data[20] = BitConverter.ToInt32(payload, 48);    // Debug[2]
            passed_data[21] = BitConverter.ToInt32(payload, 52);    // Debug[3]

            passed_data[22] = (float)(BitConverter.ToInt32(payload, 56)) / 1000;   // gyroADCf[X]
            passed_data[23] = (float)(BitConverter.ToInt32(payload, 60)) / 1000;   // gyroADCf[Y]
            passed_data[24] = (float)(BitConverter.ToInt32(payload, 64)) / 1000;   // gyroADCf[Z]

            passed_data[25] = BitConverter.ToInt16(payload, 68);    // accel_Trim[X]
            passed_data[26] = BitConverter.ToInt16(payload, 70);    // accel_Trim[Y]
            passed_data[27] = BitConverter.ToInt16(payload, 72);    // accel_Trim[Z]

            passed_data[28] = BitConverter.ToInt16(payload, 74);    // mag_Zero[X]
            passed_data[29] = BitConverter.ToInt16(payload, 76);    // mag_Zero[Y]
            passed_data[30] = BitConverter.ToInt16(payload, 78);    // mag_Zero[Z]

            passed_data[31] = (float)(BitConverter.ToInt32(payload, 80)) / 1000;   // mag_ADC[X]
            passed_data[32] = (float)(BitConverter.ToInt32(payload, 84)) / 1000;   // mag_ADC[Y]
            passed_data[33] = (float)(BitConverter.ToInt32(payload, 88)) / 1000;   // mag_ADC[Z]

            passed_data[34] = (float)(BitConverter.ToInt32(payload, 92)) / 1000;   // opflow_Rate[X]
            passed_data[35] = (float)(BitConverter.ToInt32(payload, 96)) / 1000;   // opflow_Rate[Y]
            passed_data[36] = (float)(BitConverter.ToInt32(payload, 100)) / 1000;  // opflow_bodyRate[X]
            passed_data[37] = (float)(BitConverter.ToInt32(payload, 104)) / 1000;  // opflow_bodyRate[Y]

            passed_data[38] = BitConverter.ToInt32(payload, 108);   // rangefinder_cm

            passed_data[39] = BitConverter.ToInt32(payload, 112);   // overren_cnt

            passed_data[40] = BitConverter.ToUInt16(payload, 116);  // CPU_LOAD

            passed_data[41] = (float)(BitConverter.ToInt32(payload, 118)) / 1000;  // yaw_heading_reference

            passed_data[42] = BitConverter.ToInt32(payload, 122);  // altHold

            passed_data[43] = BitConverter.ToUInt16(payload, 126);  // GPS_SAT_NUM

            passed_data[44] = BitConverter.ToUInt16(payload, 128);  // GPS_FIX

            passed_data[45] = BitConverter.ToInt32(payload, 130);  // initialThrottleHold

            passed_data[46] = BitConverter.ToInt32(payload, 134);  // _ALT.result

            if (cb_record.Checked == true)
            {
                Check_Data_Log(PID_log_filePath, passed_data);
            }

            float_data[0] = passed_data[0] / 10;
            lb_roll.Text = float_data[0].ToString("F1", CultureInfo.InvariantCulture);
            float_data[1] = passed_data[1] / 10;
            lb_pitch.Text = float_data[1].ToString("F1", CultureInfo.InvariantCulture);
            lb_heading.Text = passed_data[2].ToString();

            if (rb_roll.Checked == true)
            {
                _roll_angle_points.Add(time_count + 150, passed_data[0] / 10);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }
            if (rb_pitch.Checked == true)
            {
                _pitch_angle_points.Add(time_count + 150, passed_data[1] / 10);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }
            if (rb_yaw.Checked == true)
            {
                _yaw_angle_points.Add(time_count + 150, passed_data[2]);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_roll_pitch.Checked == true)
            {
                _roll_angle_points.Add(time_count + 150, passed_data[0] / 10);
                _pitch_angle_points.Add(time_count + 150, passed_data[1] / 10);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_roll_setpoint.Checked == true)
            {
                _roll_angle_points.Add(time_count + 150, passed_data[0] / 10);
                _rc_roll_points.Add(time_count + 150, passed_data[4] / 10);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_pitch_setpoint.Checked == true)
            {
                if (pid_test_flag == true)
                {
                    _rc_pitch_points.Add(time_count + 150, pid_test_setting_deg_temp);
                }
                else
                {
                    _rc_pitch_points.Add(time_count + 150, passed_data[5] / 10);
                }
                _pitch_angle_points.Add(time_count + 150, passed_data[1] / 10);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_yaw_setpoint.Checked == true)
            {
                _yaw_angle_points.Add(time_count + 150, passed_data[2]);
                _rc_yaw_points.Add(time_count + 150, passed_data[6]);
                _yaw_reference_points.Add(time_count + 150, passed_data[41]);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_alt_setpoint.Checked == true)
            {
                _alt_points.Add(time_count + 150, passed_data[3]);
                _alt_reference_points.Add(time_count + 150, passed_data[42]);
                _Throttle_Hold_points.Add(time_count + 150, passed_data[45]);
                _alt_pidresult_points.Add(time_count + 150, passed_data[46]);
                _Throttle_points.Add(time_count + 150, passed_data[7]*10);

                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_altitude.Checked == true)
            {
                _altitude_points.Add(time_count + 150, passed_data[3]);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_gyro.Checked == true)
            {
                _gyro_x_points.Add(time_count + 150, passed_data[22]);
                _gyro_y_points.Add(time_count + 150, passed_data[23]);
                _gyro_z_points.Add(time_count + 150, passed_data[24]);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_motor.Checked == true)
            {
                _motor_0_points.Add(time_count + 150, passed_data[14]);
                _motor_1_points.Add(time_count + 150, passed_data[15]);
                _motor_2_points.Add(time_count + 150, passed_data[16]);
                _motor_3_points.Add(time_count + 150, passed_data[17]);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            if (rb_debug.Checked == true)
            {
                _debug_0_points.Add(time_count + 150, passed_data[18]);
                _debug_1_points.Add(time_count + 150, passed_data[19]);
                _debug_2_points.Add(time_count + 150, passed_data[20]);
                _debug_3_points.Add(time_count + 150, passed_data[21]);
                _myPane.XAxis.Scale.Min = time_count;
                _myPane.XAxis.Scale.Max = 300 + time_count;
            }

            lb_altitude.Text = passed_data[3].ToString();
            float_data[4] = passed_data[4] / 10;
            lb_rc_roll.Text = float_data[4].ToString("F1", CultureInfo.InvariantCulture);
            float_data[5] = passed_data[5] / 10;
            lb_rc_pitch.Text = float_data[5].ToString("F1", CultureInfo.InvariantCulture);
            lb_rc_yaw.Text = passed_data[6].ToString();
            lb_rc_throttle.Text = passed_data[7].ToString();
            lb_lat.Text = passed_data[8].ToString();
            lb_long.Text = passed_data[9].ToString();
            lb_bat.Text = ((passed_data[10] * 4) / 100).ToString();
            battery_bar_level = (int)(passed_data[10] * 4) / 10;

            flight_mode = (UInt16)passed_data[11];

            lb_fail.Text = passed_data[12].ToString();
            error = (UInt16)passed_data[12];

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

            lb_gyro_X.Text = passed_data[22].ToString("0.00");
            lb_gyro_Y.Text = passed_data[23].ToString("0.00");
            lb_gyro_Z.Text = passed_data[24].ToString("0.00");

            lb_accTrim_X.Text = passed_data[25].ToString();
            lb_accTrim_Y.Text = passed_data[26].ToString();
            lb_accTrim_Z.Text = passed_data[27].ToString();

            lb_magZero_X.Text = passed_data[28].ToString();
            lb_magZero_Y.Text = passed_data[29].ToString();
            lb_magZero_Z.Text = passed_data[30].ToString();

            lb_mag_X.Text = passed_data[31].ToString();
            lb_mag_Y.Text = passed_data[32].ToString();
            lb_mag_Z.Text = passed_data[33].ToString();

            lb_flowrate_X.Text = passed_data[34].ToString();
            lb_flowrate_Y.Text = passed_data[35].ToString();
            lb_bodyrate_X.Text = passed_data[36].ToString();
            lb_bodyrate_Y.Text = passed_data[37].ToString();

            lb_rangefinder.Text = passed_data[38].ToString();

            tb_rx_error.Text = passed_data[39].ToString();
            lb_fc_load.Text = passed_data[40].ToString();

            lb_althold.Text = passed_data[42].ToString();

            lb_sat_num.Text = passed_data[43].ToString();

            lb_gps_fix.Text = passed_data[44].ToString();

            if (rb_roll.Checked == true || rb_pitch.Checked == true ||
               rb_yaw.Checked == true || rb_roll_pitch.Checked == true ||
               rb_roll_setpoint.Checked == true || rb_pitch_setpoint.Checked == true ||
               rb_yaw_setpoint.Checked == true || rb_altitude.Checked == true ||
               rb_gyro.Checked == true || rb_motor.Checked == true ||
               rb_debug.Checked == true || rb_alt_setpoint.Checked == true)
            {
                zedGraphControl1.AxisChange();
                zedGraphControl1.Invalidate();
                zedGraphControl1.Refresh();
            }
            return true;
        }

        bool Msp_pid_data(byte[] payload)
        {
            DateTime date_time = DateTime.Now;
            int ms = date_time.Millisecond;
            time_count++;

            passed_data[0] = (float)(BitConverter.ToInt32(payload, 0)) / 10;     // R_I_P
            passed_data[1] = (float)(BitConverter.ToInt32(payload, 4)) / 10;     // R_I_I
            passed_data[2] = (float)(BitConverter.ToInt32(payload, 8)) / 10;    // R_I_D

            passed_data[3] = (float)(BitConverter.ToInt32(payload, 12)) / 10;    // R_O_P
            passed_data[4] = (float)(BitConverter.ToInt32(payload, 16)) / 10;    // R_O_I
            passed_data[5] = (float)(BitConverter.ToInt32(payload, 20)) / 10;    // R_O_D

            passed_data[6] = (float)(BitConverter.ToInt32(payload, 24)) / 10;    // P_I_P
            passed_data[7] = (float)(BitConverter.ToInt32(payload, 28)) / 10;    // P_I_I
            passed_data[8] = (float)(BitConverter.ToInt32(payload, 32)) / 10;    // P_I_D

            passed_data[9] = (float)(BitConverter.ToInt32(payload, 36)) / 10;    // P_O_P
            passed_data[10] = (float)(BitConverter.ToInt32(payload, 40)) / 10;   // P_O_I
            passed_data[11] = (float)(BitConverter.ToInt32(payload, 44)) / 10;   // P_O_D

            passed_data[12] = (float)(BitConverter.ToInt32(payload, 48)) / 10;   // Y_A_P
            passed_data[13] = (float)(BitConverter.ToInt32(payload, 52)) / 10;   // Y_A_I
            passed_data[14] = (float)(BitConverter.ToInt32(payload, 56)) / 10;   // Y_A_D

            passed_data[15] = (float)(BitConverter.ToInt32(payload, 60)) / 10;   // Y_A_P
            passed_data[16] = (float)(BitConverter.ToInt32(payload, 64)) / 10;   // Y_A_I
            passed_data[17] = (float)(BitConverter.ToInt32(payload, 68)) / 10;   // Y_A_D

            passed_data[18] = (float)(BitConverter.ToInt32(payload, 72)) / 10;   // ALT_P
            passed_data[19] = (float)(BitConverter.ToInt32(payload, 76)) / 10;   // ALT_I
            passed_data[20] = (float)(BitConverter.ToInt32(payload, 80)) / 10;   // ALT_D

            tb_FC_R_I_P.Text = passed_data[0].ToString();
            tb_FC_R_I_I.Text = passed_data[1].ToString();
            tb_FC_R_I_D.Text = passed_data[2].ToString();

            tb_FC_R_O_P.Text = passed_data[3].ToString();
            tb_FC_R_O_I.Text = passed_data[4].ToString();
            tb_FC_R_O_D.Text = passed_data[5].ToString();

            tb_FC_P_I_P.Text = passed_data[6].ToString();
            tb_FC_P_I_I.Text = passed_data[7].ToString();
            tb_FC_P_I_D.Text = passed_data[8].ToString();

            tb_FC_P_O_P.Text = passed_data[9].ToString();
            tb_FC_P_O_I.Text = passed_data[10].ToString();
            tb_FC_P_O_D.Text = passed_data[11].ToString();

            tb_FC_Y_A_P.Text = passed_data[12].ToString();
            tb_FC_Y_A_I.Text = passed_data[13].ToString();
            tb_FC_Y_A_D.Text = passed_data[14].ToString();

            tb_FC_Y_R_P.Text = passed_data[15].ToString();
            tb_FC_Y_R_I.Text = passed_data[16].ToString();
            tb_FC_Y_R_D.Text = passed_data[17].ToString();

            tb_FC_ALT_P.Text = passed_data[18].ToString();
            tb_FC_ALT_I.Text = passed_data[19].ToString();
            tb_FC_ALT_D.Text = passed_data[20].ToString();

            return true;
        }
    }
}