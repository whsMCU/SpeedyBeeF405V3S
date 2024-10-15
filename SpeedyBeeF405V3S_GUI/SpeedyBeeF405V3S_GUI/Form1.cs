using Ball_Ballancer_CS.Class;
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
        float[] passed_data = new float[9];
        UTF8 UTF8 = new UTF8();
        DataPassing data = new DataPassing();

        private ArrayList al;
        private Point p;

        float[] pb_point = new float[2];

        public Form1()
        {
            InitializeComponent();
            al = new ArrayList();
            this.Text = "MCU Drone 제어프로그램";
            ((Control)webBrowser1).Enabled = false;
            ((Control)webBrowser2).Enabled = false;
            webBrowser1.Navigate("https://www.openstreetmap.org/#map=19/35.196569/126.829348");
            webBrowser1.Visible = true;
            webBrowser2.Visible = false;
            received_data = 2;
            textBox10.Text = (zoom - 14).ToString();
        }

        private void Form1_Load(object sender, EventArgs e)  //폼이 로드되면
        {
            try
            {
                comboBox_port.DataSource = SerialPort.GetPortNames(); //연결 가능한 시리얼포트 이름을 콤보박스에 가져오기 
            }
            catch { }
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
                    //Location_update_timer.Enabled = false;
                    webBrowser1.Navigate("https://www.openstreetmap.org/#map=2/50.8/5.6");
                    webBrowser1.Visible = true;
                    webBrowser2.Visible = false;
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
            catch { }
        }

        private void MySerialReceived(object s, EventArgs e)  //여기에서 수신 데이타를 사용자의 용도에 따라 처리한다.
        {
            //int ReceiveData = serialPort1.ReadByte();  //시리얼 버터에 수신된 데이타를 ReceiveData 읽어오기
            //richTextBox_received.Text = richTextBox_received.Text + string.Format("{0:X2}", ReceiveData);  //int 형식을 string형식으로 변환하여 출력
            try
            {
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
                                if (passed_data[1] != 0)
                                {

                                }
                            }
                            else if (passed_data[0] == 1)
                            {
                                //lb_roll.Text = passed_data[1].ToString();
                                //lb_pitch.Text = passed_data[2].ToString();
                                //tb_D.Text = passed_data[3].ToString();
                            }

                        }
                    }
                    catch { }
                }
            }
            catch (System.Exception)
            {
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
            catch { }
        }

        private void bt_pid_recive_Click(object sender, EventArgs e)
        {
            byte[] buff = new byte[20];
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
            }
            catch { }

            for (int i = 0; i < 19; i++)
            {
                buff[19] -= buff[i];
            }

            try
            {
                serialPort.Write(Encoding.UTF8.GetString(buff));
            }
            catch { }
        }

        private void bt_pid_send_Click(object sender, EventArgs e)
        {
            byte[] buff = new byte[20];
            float float_buff;
            byte[] tmp = new byte[4];
            try
            {
                buff[0] = 0x47;
                buff[1] = 0x53;
                buff[2] = 0x00;
                float_buff = float.Parse(tb_P.Text);
                tmp = BitConverter.GetBytes(float_buff);
                buff[3] = tmp[0];
                buff[4] = tmp[1];
                buff[5] = tmp[2];
                buff[6] = tmp[3];
                float_buff = float.Parse(tb_I.Text);
                tmp = BitConverter.GetBytes(float_buff);
                buff[7] = tmp[0];
                buff[8] = tmp[1];
                buff[9] = tmp[2];
                buff[10] = tmp[3];
                float_buff = float.Parse(tb_D.Text);
                tmp = BitConverter.GetBytes(float_buff);
                buff[11] = tmp[0];
                buff[12] = tmp[1];
                buff[13] = tmp[2];
                buff[14] = tmp[3];
                buff[15] = 0;
                buff[16] = 0;
                buff[17] = 0;
                buff[18] = 0;
                buff[19] = 0xff;
            }
            catch { }

            for (int i = 0; i < 19; i++)
            {
                buff[19] -= buff[i];
            }

            try
            {
                serialPort.Write(buff, 0, 20);
            }
            catch { }
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
            if (received_data == 10)
            {
                received_data = 0;
            }
        }
    }
}
