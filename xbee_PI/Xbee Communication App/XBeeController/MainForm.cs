using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.Runtime.InteropServices;

namespace XBee
{
    public partial class MainForm : Form
    {
        private Timer timer = new Timer();
        private StringBuilder recievedData = new StringBuilder();

        public MainForm()
        {
            InitializeComponent();
            RegisterHidNotification();
        }

        #region Private Methods
        protected override void WndProc(ref Message m)
        {
            switch (m.Msg)
            {
                case Win32.WM_DEVICECHANGE: OnDeviceChange(ref m); break;
            }
            base.WndProc(ref m);
        }

        private void OnDeviceChange(ref Message msg)
        {
            int wParam = (int)msg.WParam;
            if (wParam == Win32.DBT_DEVICEARRIVAL) txtDataIn.Text += "USB Device Detected" + Environment.NewLine;
            else if (wParam == Win32.DBT_DEVICEREMOVECOMPLETE) txtDataIn.Text += "USB Device Removed" + Environment.NewLine;

            InitPorts();
        }

        private void RegisterHidNotification()
        {
            Win32.DEV_BROADCAST_DEVICEINTERFACE dbi = new Win32.DEV_BROADCAST_DEVICEINTERFACE();
            int size = Marshal.SizeOf(dbi);
            dbi.dbcc_size = size;
            dbi.dbcc_devicetype = Win32.DBT_DEVTYP_DEVICEINTERFACE;
            dbi.dbcc_reserved = 0;
            dbi.dbcc_classguid = Win32.GUID_DEVINTERFACE_HID;
            dbi.dbcc_name = 0;
            IntPtr buffer = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(dbi, buffer, true);
            IntPtr r = Win32.RegisterDeviceNotification(Handle, buffer, Win32.DEVICE_NOTIFY_WINDOW_HANDLE);
            if (r == IntPtr.Zero) txtDataIn.Text += Win32.GetLastError().ToString() + Environment.NewLine;
        }

        private void ClosePort()
        {
            if (serialPort1.IsOpen) serialPort1.Close();
        }

        private void InitPorts()
        {
            cmbCOMPort.Items.Clear();
            string[] ports = SerialPort.GetPortNames();
            var sortedList = ports.OrderBy(port => Convert.ToInt32(port.Replace("COM", string.Empty)));

            foreach (string portname in sortedList)
            {
                cmbCOMPort.Items.Add(portname);
            }
        }

        private void OpenPort()
        {
            serialPort1.PortName = cmbCOMPort.Text;

            if (!serialPort1.IsOpen)
                serialPort1.Open();

            timer.Interval = 1000;
            timer.Tick += new EventHandler(timer_Tick); 
            timer.Start();
        }

        private void ReceiveData()
        {
            string data = serialPort1.ReadExisting();
            recievedData.Append(data);
            //if (!data.Contains("Got it!")) SendData("Got it!");
        }

        private void SendData(string text)
        {
            if (serialPort1.IsOpen)
            {
                serialPort1.WriteLine(text);
            }
        }

        private void UpdateDataIn()
        {
            txtDataIn.Text = recievedData.ToString();
        }
        #endregion

        #region Form Handlers
        private void MainForm_Load(object sender, EventArgs e)
        {
            InitPorts();
        }

        private void btnOpenPort_Click(object sender, EventArgs e)
        {
            OpenPort();
        }

        private void btnClosePort_Click(object sender, EventArgs e)
        {
            ClosePort();
        }

        private void btnSend_Click(object sender, EventArgs e)
        {
            SendData(txtDataOut.Text);
        }

        private void timer_Tick(object sender, EventArgs e)
        {
            UpdateDataIn();
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            ClosePort();
        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            ReceiveData();
        }

        private void txtDataOut_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)(Keys.Return))
            {
                SendData(txtDataOut.Text);
            }
        }
        #endregion

        
    }
}
