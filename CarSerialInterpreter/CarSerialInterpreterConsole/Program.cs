using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace CarSerialInterpreterConsole
{
    struct Message
    {
        public byte Pattern;
        public sbyte Angle;
        public sbyte MotorLeft;
        public sbyte MotorRight;
        public byte Sensor;
        public byte TraceMask;
        public byte MessageByte;
        public byte MessageData;
        public short EndOfMessage;
    }

    class Program
    {
        static void Main(string[] args)
        {
            const string comPort = "COM9";

            Console.WriteLine($"Connecting on {comPort}...");
            System.IO.Ports.SerialPort port = new System.IO.Ports.SerialPort(comPort, 9600, System.IO.Ports.Parity.None, 8, System.IO.Ports.StopBits.One);
            port.Open();

            bool byte1Received = false;
            bool byte2Received = false;
            while (true)
            {
                if (!byte2Received)
                {
                    while (port.BytesToRead > 1)
                    {
                        byte readByte = (byte)port.ReadByte();
                        if (byte1Received && readByte== 0xff)
                        {
                            byte2Received = true;
                            break;
                        }
                        else if (byte1Received)
                        {
                            byte1Received = false;
                        }
                        else if(readByte == 0xff)
                        {
                            byte1Received = true;
                        }
                    }
                }
                else
                {
                    while (port.BytesToRead > 10)
                    {
                        byte[] buffer = new byte[10];
                        port.Read(buffer, 0, 10);
                        Message m = BufferToMessage(buffer);

                        PrintPrettyMessage(m);
                    }
                }
                Thread.Sleep(100);
            }
        }

        private static void PrintPrettyMessage(Message m)
        {
            if (m.EndOfMessage != 0)
            {
                Console.WriteLine("{0,22} {1,4}°   [{2,4}% {3,4}%]   {4} {5}",
                    PatternName(m.Pattern),
                    m.Angle,
                    m.MotorLeft,
                    m.MotorRight,
                    Convert.ToString(m.Sensor, 2).PadLeft(8, '0').Replace('0', '.').Replace('1', 'X'),
                    Convert.ToString(m.TraceMask, 2).PadLeft(8, '0').Replace('0', '_').Replace('1', 'X')
                    );
            }
        }

        private static string PatternName(byte pattern)
        {
            switch (pattern)
            {
                case 0: return "WAIT_FOR_SWITCH";
                case 1: return "WAIT_FOR_STARTBAR";
                case 2: return "WAIT_FOR_LOST_TRACK";
                case 3: return "NORMAL_TRACE";
                case 4: return "DETECT_SHARP_CORNER";
                case 5: return "SHARP_CORNER_LEFT";
                case 6: return "SHARP_CORNER_RIGHT";
                case 8: return "WAIT_HALF_LINE";
                case 10: return "SEARCH_LINE_RIGHT";
                case 11: return "SEARCH_LINE_LEFT";
                case 0x0f: return "RIGHT_LINE";
                case 0xf0: return "LEFT_LINE";
                case 0xff: return "CROSS_LINE";
                default: return pattern.ToString("X");
            }
        }

        static Message BufferToMessage(byte[] buffer)
        {
            Message m = new Message()
            {
                Pattern = buffer[0],
                Angle = (sbyte)buffer[1],
                MotorLeft = (sbyte)buffer[2],
                MotorRight = (sbyte)buffer[3],
                Sensor = buffer[4],
                TraceMask = buffer[5],
                MessageByte = buffer[6],
                MessageData = buffer[7],
                EndOfMessage = (short)((buffer[8] << 8) | buffer[9])
            };
            return m;
        }
    }
}
