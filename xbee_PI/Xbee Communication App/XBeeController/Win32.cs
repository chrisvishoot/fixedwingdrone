using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;

namespace XBee
{
    class Win32
    {
        public const int WM_DEVICECHANGE = 0x0219;
        public const int DBT_DEVICEARRIVAL = 0x8000;
        public const int DBT_DEVICEREMOVECOMPLETE = 0x8004;
        public const int DEVICE_NOTIFY_WINDOW_HANDLE = 0;
        public const int DEVICE_NOTIFY_SERVICE_HANDLE = 1;
        public const int DBT_DEVTYP_DEVICEINTERFACE = 5;
        public static Guid GUID_DEVINTERFACE_HID = new Guid("4D1E55B2-F16F-11CF-88CB-001111000030");
        public static Guid GUID_DEVINTERFACE_USB_DEVICE = new Guid("A5DCBF10-6530-11D2-901F-00C04FB951ED");
        public static Guid GUID_DEVCLASS_KEYBOARD = new Guid("4D36E96B-E325-11CE-BFC1-08002BE10318");

        [StructLayout(LayoutKind.Sequential)]
        public class DEV_BROADCAST_DEVICEINTERFACE
        {
            public int dbcc_size;
            public int dbcc_devicetype;
            public int dbcc_reserved;
            public Guid dbcc_classguid;
            public short dbcc_name;
        }

        [DllImport("user32.dll", SetLastError = true)]
        public static extern IntPtr RegisterDeviceNotification(
        IntPtr hRecipient,
        IntPtr NotificationFilter,
        Int32 Flags);

        [DllImport("kernel32.dll")]
        public static extern int GetLastError();

        public const int CR_SUCCESS = 0;
        [DllImport("cfgmgr32.dll")]
        public static extern int CM_Get_Device_ID(int DevInst, IntPtr Buffer, int BufferLen, int Flags);
        public static string CM_Get_Device_ID(int DevInst)
        {
            string s = null;
            int len = 300;
            IntPtr buffer = Marshal.AllocHGlobal(len);
            int r = CM_Get_Device_ID(DevInst, buffer, len, 0);
            if (r == CR_SUCCESS) s = Marshal.PtrToStringAnsi(buffer);
            return s;
        }
    }
}
