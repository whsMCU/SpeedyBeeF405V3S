using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
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

        List<byte> recvData = new List<byte>();
        byte[] buff = new byte[1024];
        byte[] buff_pass = new byte[64];
        int cnt = 0;
        byte checksum = 0xff;

        float[] data = new float[20];

        public DataPassing()
        {

        }

        public DataPassing(List<byte> recvData, byte checksum)
        {
            this.recvData = recvData;
            this.checksum = checksum;
        }

        public float[] dataPassing(byte[] data, int size)
        {
            bool recived_data_flag = false;
            Array.Copy(data, buff, size);
            for (int i = 0; i < size; i++)
            {
                switch (cnt)
                {
                    case 0:
                        if (buff[i] == 0x46)
                        {
                            buff_pass[cnt] = buff[i];
                            cnt++;
                        }
                        else
                        {
                            cnt = 0;
                        }
                        break;

                    case 1:
                        if (buff[i] == 0x43)
                        {
                            buff_pass[cnt] = buff[i];
                            cnt++;
                        }
                        else
                        {
                            cnt = 0;
                        }
                        break;

                    case 35:
                        buff_pass[cnt] = buff[i];
                        cnt = 0;
                        recived_data_flag = true;
                        break;

                    default:
                        buff_pass[cnt] = buff[i];
                        cnt++;
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
            }
            else if (buff_pass[2] == 0x11) // GPS Data 수신
            {
                data[0] = 1;
                data[P] = BitConverter.ToSingle(buff_pass, 3);
                data[I] = BitConverter.ToSingle(buff_pass, 7);
                data[D] = BitConverter.ToSingle(buff_pass, 11);
            }

            return data;
        }

        ~DataPassing()
        {

        }
    }
}
