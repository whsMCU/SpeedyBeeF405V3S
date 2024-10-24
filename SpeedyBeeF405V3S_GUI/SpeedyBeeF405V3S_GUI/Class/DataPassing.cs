using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading.Tasks;

namespace Ball_Ballancer_CS.Class
{
    public class DataPassing
    {
        public static readonly byte
        P = 1,
        I = 2,
        D = 3,
        X = 1,
        Y = 2,
        Z = 3;

        enum PID_e
        {
            R_I_P = 1,
            R_I_I,
            R_I_D,

            R_O_P,
            R_O_I,
            R_O_D,

            P_I_P,
            P_I_I,
            P_I_D,

            P_O_P,
            P_O_I,
            P_O_D,

            Y_A_P,
            Y_A_I,
            Y_A_D,

            Y_R_P,
            Y_R_I,
            Y_R_D
        }

        List<byte> recvData = new List<byte>();
        byte[] buff = new byte[1024];
        byte[] buff_pass = new byte[100];
        int cnt = 0;
        byte checksum = 0xff;

        float[] data = new float[30];

        public DataPassing()
        {

        }

        public DataPassing(List<byte> recvData, byte checksum)
        {
            this.recvData = recvData;
            this.checksum = checksum;
        }


        enum gcsState_e
        {
            GCS_IDLE,
            GCS_HEADER_SYNC,
            GCS_HEADER_ID,

            GCS_PAYLOAD,

            GCS_CHECKSUM,

            GCS_DATA_RECEIVED
        }

        enum gcsData_e
        {
            GCS_Tlemetry,
            GCS_PID_recive,
            GCS_PID_send
        }

        gcsState_e gcsState = gcsState_e.GCS_IDLE;
        gcsData_e gcsData = gcsData_e.GCS_Tlemetry;

        public float[] dataPassing(byte[] data, int size)
        {
            bool recived_data_flag = false;
            int ii = 0;
            Array.Copy(data, buff, size);
            for (int i = 0; i < size; i++)
            {

                switch(gcsState)
                {
                    case gcsState_e.GCS_IDLE:
                        if (buff[i] == 'F')
                        {
                            buff_pass[cnt] = buff[i];
                            gcsState = gcsState_e.GCS_HEADER_SYNC;
                            cnt++;
                        }
                        break;
                    case gcsState_e.GCS_HEADER_SYNC:
                        if (buff[i] == 'C')
                        {
                            buff_pass[cnt++] = buff[i];
                            gcsState = gcsState_e.GCS_HEADER_ID;
                        }
                        else
                        {
                            gcsState = gcsState_e.GCS_IDLE;
                            cnt = 0;
                        }
                        break;
                    case gcsState_e.GCS_HEADER_ID:
                        switch (buff[i])
                        {
                            case 0x10:
                                buff_pass[cnt++] = buff[i];
                                gcsState = gcsState_e.GCS_PAYLOAD;
                                gcsData = gcsData_e.GCS_Tlemetry;
                                break;
                            case 0x20:
                                buff_pass[cnt++] = buff[i];
                                gcsState = gcsState_e.GCS_PAYLOAD;
                                gcsData = gcsData_e.GCS_PID_recive;
                                break;
                            default:
                                gcsState = gcsState_e.GCS_IDLE;
                                gcsData = gcsData_e.GCS_Tlemetry;
                                cnt = 0;
                                break;
                        }
                        break;
                    case gcsState_e.GCS_PAYLOAD:
                        buff_pass[cnt++] = buff[i];
                        if (gcsData == gcsData_e.GCS_Tlemetry && cnt == 59)
                        {
                            gcsState = gcsState_e.GCS_CHECKSUM;
                        }
                        else if (gcsData == gcsData_e.GCS_PID_recive && cnt == 75)
                        {
                            gcsState = gcsState_e.GCS_CHECKSUM;
                        }
                        break;
                    case gcsState_e.GCS_CHECKSUM:
                        buff_pass[cnt] = buff[i];
                        if (gcsData == gcsData_e.GCS_PID_recive)
                        {
                            byte chksum = 0xff;
                            for (ii = 0; ii < 75; ii++) chksum = (byte)(chksum - buff_pass[ii]);
                            if (chksum == buff_pass[cnt])
                            {
                                recived_data_flag = true;
                            }
                        }
                        else if (gcsData == gcsData_e.GCS_Tlemetry)
                        {
                            byte chksum = 0xff;
                            for (ii = 0; ii < 59; ii++) chksum = (byte)(chksum - buff_pass[ii]);
                            if (chksum == buff_pass[cnt])
                            {
                                recived_data_flag = true;
                            }
                        }

                        gcsState = gcsState_e.GCS_IDLE;
                        cnt = 0;
                        break;
                }
            }
            if(recived_data_flag == true)
            {

                return Passing();
            }
            return null;
        }

        public float[] Passing()
        {
            if (buff_pass[2] == 0x10) //AHRS Data 수신
            {
                data[0] = 0;
                data[1] = BitConverter.ToInt16(buff_pass, 3)/100;
                data[2] = BitConverter.ToInt16(buff_pass, 5)/100;
                data[3] = BitConverter.ToUInt16(buff_pass, 7)/100;
                data[4] = BitConverter.ToInt16(buff_pass, 9)/10;
                data[5] = BitConverter.ToInt16(buff_pass, 11)/100;
                data[6] = BitConverter.ToInt16(buff_pass, 13)/100;
                data[7] = (BitConverter.ToInt16(buff_pass, 15)/10);
                data[8] = (BitConverter.ToInt16(buff_pass, 17)/10)-1000;
                data[9] = BitConverter.ToInt32(buff_pass, 19);
                data[10] = BitConverter.ToInt32(buff_pass, 23);
                data[11] = BitConverter.ToInt16(buff_pass, 27);
                data[12] = BitConverter.ToInt16(buff_pass, 31);
                data[13] = BitConverter.ToInt16(buff_pass, 33);
                data[14] = BitConverter.ToUInt16(buff_pass, 35);
                data[15] = BitConverter.ToUInt16(buff_pass, 37);
                data[16] = BitConverter.ToUInt16(buff_pass, 39);
                data[17] = BitConverter.ToUInt16(buff_pass, 41);
                data[18] = BitConverter.ToUInt32(buff_pass, 43);
                data[19] = BitConverter.ToUInt32(buff_pass, 47);
                data[20] = BitConverter.ToUInt32(buff_pass, 51);
                data[21] = BitConverter.ToUInt32(buff_pass, 55);
            }
            else if (buff_pass[2] == 0x20) // PID Data 수신
            {
                data[0] = 1;
                data[(int)PID_e.R_I_P] = BitConverter.ToSingle(buff_pass, 3);
                data[(int)PID_e.R_I_I] = BitConverter.ToSingle(buff_pass, 7);
                data[(int)PID_e.R_I_D] = BitConverter.ToSingle(buff_pass, 11);

                data[(int)PID_e.R_O_P] = BitConverter.ToSingle(buff_pass, 15);
                data[(int)PID_e.R_O_I] = BitConverter.ToSingle(buff_pass, 19);
                data[(int)PID_e.R_O_D] = BitConverter.ToSingle(buff_pass, 23);

                data[(int)PID_e.P_I_P] = BitConverter.ToSingle(buff_pass, 27);
                data[(int)PID_e.P_I_I] = BitConverter.ToSingle(buff_pass, 31);
                data[(int)PID_e.P_I_D] = BitConverter.ToSingle(buff_pass, 35);

                data[(int)PID_e.P_O_P] = BitConverter.ToSingle(buff_pass, 39);
                data[(int)PID_e.P_O_I] = BitConverter.ToSingle(buff_pass, 43);
                data[(int)PID_e.P_O_D] = BitConverter.ToSingle(buff_pass, 47);

                data[(int)PID_e.Y_A_P] = BitConverter.ToSingle(buff_pass, 51);
                data[(int)PID_e.Y_A_I] = BitConverter.ToSingle(buff_pass, 55);
                data[(int)PID_e.Y_A_D] = BitConverter.ToSingle(buff_pass, 59);

                data[(int)PID_e.Y_R_P] = BitConverter.ToSingle(buff_pass, 63);
                data[(int)PID_e.Y_R_I] = BitConverter.ToSingle(buff_pass, 67);
                data[(int)PID_e.Y_R_D] = BitConverter.ToSingle(buff_pass, 71);
            }

            return data;
        }

        ~DataPassing()
        {

        }
    }
}
