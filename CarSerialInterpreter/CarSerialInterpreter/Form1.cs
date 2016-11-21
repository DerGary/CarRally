using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace CarSerialInterpreter
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            byte[] buffer = new byte[9];
            int readBytes = sp.Read(buffer, 0, 9);
            Debug.WriteLine(readBytes);

            Dispatch(() =>
            {
                string value = string.Join(" ", buffer.Select(x => (int)x));
                textBox1.AppendText(value);
                textBox1.AppendText(Environment.NewLine);
            });            
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            serialPort1.ReadBufferSize = 16;
            serialPort1.ReceivedBytesThreshold = 16;
            serialPort1.Open();
        }

        private void Dispatch(Action action)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(action);
            }
            else
            {
                action();
            }
        }
    }
}
