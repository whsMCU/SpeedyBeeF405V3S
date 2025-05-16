using GMap.NET.MapProviders;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Net.NetworkInformation;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using System.Threading.Tasks;
using ZedGraph;
using static System.Windows.Forms.AxHost;

namespace SpeedyBeeF405V3S_GUI.Class
{
    public class MspProtocol
    {
        private SerialPort _serialPort;

        private int msp_error;

        public delegate void MspPacketReceivedHandler(byte command, byte[] payload);
        public event MspPacketReceivedHandler OnPacketReceived;

        private enum ParseState
        {
            IDLE,
            HEADER_START,
            HEADER_M,
            HEADER_ARROW,
            HEADER_SIZE,
            HEADER_CMD,
            PAYLOAD,
            CHECKSUM
        }

        private ParseState c_state = ParseState.IDLE;


        private static int _INBUF_SIZE = 256;
        private byte[] _inBuf = new byte[256];
        private byte _cmdMSP;
        private byte _checksum = 0;
        private int _offset = 0, _dataSize = 0;


        public MspProtocol()
        {

        }

        public void Open(SerialPort serialPort)
        {
            _serialPort = serialPort;
        }

        public void ParseByte(byte c)
        {
            switch (c_state)
            {
                case ParseState.IDLE:
                    c_state = (c == '$') ? ParseState.HEADER_START : ParseState.IDLE;
                    break;
                case ParseState.HEADER_START:
                    c_state = (c == 'M') ? ParseState.HEADER_M : ParseState.IDLE;
                     break;
                case ParseState.HEADER_M:
                    c_state = (c == '>') ? ParseState.HEADER_ARROW : ParseState.IDLE;
                    _offset = 0;
                    _checksum = 0;
                    _dataSize = 0;
                    break;
                case ParseState.HEADER_ARROW:
                    _dataSize = c;
                    _checksum ^= c;
                    c_state = ParseState.HEADER_SIZE;
                    break;
                case ParseState.HEADER_SIZE:
                    _cmdMSP = c;
                    _checksum ^= c;
                    c_state = (_dataSize > 0) ? ParseState.PAYLOAD : ParseState.CHECKSUM;
                    break;
                case ParseState.PAYLOAD:
                    if (_offset < _inBuf.Length)
                    {
                        _inBuf[_offset++] = c;
                        _checksum ^= c;
                        if (_offset >= _dataSize)
                        {
                            c_state = ParseState.CHECKSUM;
                        }
                    }
                    else
                    {
                        c_state = ParseState.IDLE;
                        _offset = 0;
                        _checksum = 0;
                        _dataSize = 0;
                    }

                    break;
                case ParseState.CHECKSUM:
                    if (_checksum == c)
                    {
                        byte[] payload = new byte[_dataSize];
                        Array.Copy(_inBuf, payload, _dataSize);
                        OnPacketReceived?.Invoke(_cmdMSP, payload);
                    }
                    else
                    {
                        msp_error++;
                    }
                    c_state = ParseState.IDLE;
                    _offset = 0;
                    _checksum = 0;
                    _dataSize = 0;
                    break;
            }
        }

        public int GetMspError()
        {
            return msp_error;
        }

        public void SendMspCommand(byte command, byte[] payload = null)
        {
            if (payload == null)
            {
                payload = new byte[0];
            }
            byte size = (byte)payload.Length;
            byte checksum = (byte)(size ^ command);

            foreach (var b in payload)
            {
                checksum ^= b;
            }

            var packet = new byte[6 + payload.Length];
            packet[0] = (byte)'$';
            packet[1] = (byte)'M';
            packet[2] = (byte)'<';  // 송신 방향
            packet[3] = size;
            packet[4] = command;

            Array.Copy(payload, 0, packet, 5, payload.Length);
            packet[5 + payload.Length] = checksum;

            _serialPort.Write(packet, 0, packet.Length);
        }

    }
}
