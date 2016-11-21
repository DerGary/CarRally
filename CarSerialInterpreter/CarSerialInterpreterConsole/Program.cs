using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace CarSerialInterpreterConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            System.IO.Ports.SerialPort port = new System.IO.Ports.SerialPort("COM9", 9600, System.IO.Ports.Parity.None, 8, System.IO.Ports.StopBits.One);
            
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
                        string value = string.Join(" ", buffer.Select(x => (int)x));
                        Console.WriteLine(value);
                    }
                }
                Thread.Sleep(100);
            }
            
        }
    }
}
