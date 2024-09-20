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
        P  = 1,
        I  = 2,
        D  = 3,
        X  = 1,
        Y  = 2,
        Z  = 3;

        List<byte> recvData = new List<byte>();
        byte[] buff = new byte[1024];
        byte[] buff_pass = new byte[64];
        int cnt = 0;
        byte checksum = 0xff;

        float[] data = new float[4];

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

                    case 19:
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
            if (buff_pass[2] == 0x10) //터치패널 좌표 데이터 수신
            {
                data[0] = 0;
                data[X] = BitConverter.ToInt16(buff_pass, 3);
                data[Y] = BitConverter.ToInt16(buff_pass, 5);
                data[Z] = BitConverter.ToInt16(buff_pass, 7);
            }
            else if (buff_pass[2] == 0x20) // PID 게인값 수신
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
