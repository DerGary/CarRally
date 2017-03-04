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
        public UInt32 SysTime;
    }

    class Program
    {
        const string comPort = "COM3";

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
            Console.BackgroundColor = ConsoleColor.Green;
            Console.WriteLine("Connected. Wait for data...");
            
            List<Message> messages = new List<Message>();
            int printedMessageCount = 0;

            try
            {
                // wait for data
                while (port.BytesToRead <= 10)
                {
                    Thread.Sleep(100);
                }

                Console.BackgroundColor = ConsoleColor.Black;
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
                        if (PrintPrettyMessage(m))
                        {
                            printedMessageCount++;
                        }
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
                sw.WriteLine("Time;Pattern;Angle;MotorLeft;MotorRight;Sensor;TraceMask");
                foreach (Message m in messages.OrderBy(m => m.SysTime))
                {
                    sw.WriteLine("{0};0x{1:X2};{2};{3};{4};0x{5:X2};0x{6:X2}",
                        m.SysTime, m.Pattern, m.Angle, m.MotorLeft, m.MotorRight, m.Sensor, m.TraceMask);
                }
            }
            using (var sw = new StreamWriter(File.OpenWrite(string.Format("run-{0:yyyyMMdd-HHmmss}-pretty.txt", DateTime.Now))))
            {
                sw.WriteLine("Time;Pattern;Angle;MotorLeft;MotorRight;Sensor;TraceMask");
                foreach (Message m in messages.OrderBy(m => m.SysTime))
                {
                    if (m.SysTime != 0)
                    {
                        sw.WriteLine(PrintPretty(m));
                    }
                    
                }
            }

            Console.WriteLine($"Printed {printedMessageCount} of {messages.Count} received messages.");
            Console.WriteLine("Done");
            Console.ReadLine();
        }

        private static bool PrintPrettyMessage(Message m)
        {
            if (m.SysTime != 0)
            {
                Console.WriteLine(PrintPretty(m));

                return true;
            }

            return false;
        }
        private static string PrintPretty(Message m)
        {
            return string.Format("{0,7}: {1,22} {2,4}°   [{3,4}% {4,4}%]   {5} {6}",
                    m.SysTime,
                    PatternName(m.Pattern),
                    m.Angle,
                    m.MotorLeft,
                    m.MotorRight,
                    BitPattern(m.Sensor, '.', 'X'),
                    BitPattern(m.TraceMask, '_', 'X'));
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
                case 0x1f: return "RIGHT_LINE";
                case 0xf8: return "LEFT_LINE";
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
                SysTime = (uint)(buffer[8] << 24 | buffer[9] << 16 | buffer[10] << 8 | buffer[11])
            };
            return m;
        }
    }
}
