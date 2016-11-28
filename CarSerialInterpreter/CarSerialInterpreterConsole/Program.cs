using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
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
        const string comPort = "COM9";

        const int Baud9600 = 9600;
        const int Baud38400 = 38400;

        static SerialPort port;

        static void Main(string[] args)
        {
            int MessageSize = System.Runtime.InteropServices.Marshal.SizeOf<Message>();

            port = new SerialPort(comPort, Baud9600, Parity.None, 8, StopBits.One);
            port.ReadBufferSize = 16384;

            Console.WriteLine($"Connecting on {comPort}...");
            port.Open();
            Console.WriteLine("Connected. Wait for data...");
            
            List<Message> messages = new List<Message>();

            try
            {
                // wait for data
                while (port.BytesToRead <= 10)
                {
                    Thread.Sleep(100);
                }

                Console.WriteLine("Receiving data...");

                // read while we receive data
                while (true)
                {
                    // read messages
                    while (port.BytesToRead >= MessageSize)
                    {
                        byte[] buffer = new byte[MessageSize];
                        port.Read(buffer, 0, MessageSize);
                        Message m = BufferToMessage(buffer);

                        messages.Add(m);
                        PrintPrettyMessage(m);
                    }

                    // wait if new data is incoming
                    int sheepCounter = 100;

                    // sleep a short time and check if new data is received
                    while (sheepCounter > 0)
                    {
                        sheepCounter--;
                        Thread.Sleep(10);

                        if (port.BytesToRead > 0)
                        {
                            // break sleep loop
                            break;
                        }
                    }

                    // if no new data, exit receive loop
                    if (port.BytesToRead == 0)
                    {
                        break;
                    }
                }
            }
            finally
            {
                port.Close();
            }

            using (var sw = new StreamWriter(File.OpenWrite(string.Format("run-{0:yyyyMMdd-HHmmss}.csv", DateTime.Now))))
            {
                sw.WriteLine("Pattern;Angle;MotorLeft;MotorRight;Sensor;TraceMask");
                foreach (Message m in messages)
                {
                    sw.WriteLine("0x{0:X2};{1};{2};{3};0x{4:X2};0x{5:X2}",
                        m.Pattern, m.Angle, m.MotorLeft, m.MotorRight, m.Sensor, m.TraceMask);
                }
            }

            Console.WriteLine($"Received {messages.Count} messages.");
            Console.WriteLine("Done");
            Console.ReadLine();
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
                    BitPattern(m.Sensor, '.', 'X'),
                    BitPattern(m.TraceMask, '_', 'X')
                    );
            }
        }

        private static string BitPattern(byte v, char zeroChar, char oneChar)
        {
            return Convert.ToString(v, 2)
                          .PadLeft(8, '0')
                          .Replace('0', zeroChar)
                          .Replace('1', oneChar);
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
                default: return "0x" + pattern.ToString("X");
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
