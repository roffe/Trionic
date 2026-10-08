using System;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace Lawicel
{
    /// <summary>
    /// Lawicel CANUSB API (in-tree replacement for canusbdrv_net.dll).
    /// Windows calls Lawicel's native canusbdrv(64).dll, everywhere else the same calls are
    /// served by CanusbVcp, which speaks the ASCII protocol over the FTDI virtual COM port.
    /// </summary>
    public class CANUSB
    {
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CANMsg
        {
            public uint id;
            public uint timestamp;
            public byte flags;
            public byte len;
            public ulong data;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CANMsgEx
        {
            public uint id;
            public uint timestamp;
            public byte flags;
            public byte len;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CANMsgCallback
        {
            public uint id;
            public uint timestamp;
            public byte flags;
            public byte len;
            public ulong data;
        }

        public delegate void CANMsgCallbackDef(ref CANMsgCallback msg);

        public const string CAN_BAUD_BTR_1M = "0x00:0x14";
        public const string CAN_BAUD_BTR_500K = "0x00:0x1C";
        public const string CAN_BAUD_BTR_250K = "0x01:0x1C";
        public const string CAN_BAUD_BTR_125K = "0x03:0x1C";
        public const string CAN_BAUD_BTR_100K = "0x43:0x2F";
        public const string CAN_BAUD_BTR_50K = "0x47:0x2F";
        public const string CAN_BAUD_BTR_20K = "0x53:0x2F";
        public const string CAN_BAUD_BTR_10K = "0x67:0x2F";
        public const string CAN_BAUD_BTR_5K = "0x7F:0x7F";

        public const string CAN_BAUD_1M = "1000";
        public const string CAN_BAUD_800K = "800";
        public const string CAN_BAUD_500K = "500";
        public const string CAN_BAUD_250K = "250";
        public const string CAN_BAUD_125K = "125";
        public const string CAN_BAUD_100K = "100";
        public const string CAN_BAUD_50K = "50";
        public const string CAN_BAUD_20K = "20";
        public const string CAN_BAUD_10K = "10";

        public const int ERROR_CANUSB_OK = 1;
        public const int ERROR_CANUSB_OPEN_SUBSYSTEM = -2;
        public const int ERROR_CANUSB_COMMAND_SUBSYSTEM = -3;
        public const int ERROR_CANUSB_NOT_OPEN = -4;
        public const int ERROR_CANUSB_TX_FIFO_FULL = -5;
        public const int ERROR_CANUSB_INVALID_PARAM = -6;
        public const int ERROR_CANUSB_NO_MESSAGE = -7;
        public const int ERROR_CANUSB_MEMORY_ERROR = -8;
        public const int ERROR_CANUSB_NO_DEVICE = -9;
        public const int ERROR_CANUSB_TIMEOUT = -10;
        public const int ERROR_CANUSB_INVALID_HARDWARE = -11;

        public const byte CANMSG_EXTENDED = 128;
        public const byte CANMSG_RTR = 64;

        public const byte CANUSB_FLAG_TIMESTAMP = 1;
        public const byte CANUSB_FLAG_QUEUE_REPLACE = 2;
        public const byte CANUSB_FLAG_BLOCK = 4;
        public const byte CANUSB_FLAG_SLOW = 8;
        public const byte CANUSB_FLAG_NO_LOCAL_SEND = 16;

        public const uint CANUSB_ACCEPTANCE_CODE_ALL = 0u;
        public const uint CANUSB_ACCEPTANCE_MASK_ALL = uint.MaxValue;

        public const uint FLUSH_WAIT = 0u;
        public const uint FLUSH_DONTWAIT = 1u;
        public const uint FLUSH_EMPTY_INQUEUE = 2u;

        public static uint canusb_Open(string szID, string szBitrate, uint acceptance_code, uint acceptance_mask, uint flags)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Open(szID, szBitrate, acceptance_code, acceptance_mask, flags);
            return CanusbVcp.Open(szID, szBitrate, acceptance_code, acceptance_mask, flags);
        }

        public static uint canusb_Open(IntPtr szID, string szBitrate, uint acceptance_code, uint acceptance_mask, uint flags)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Open(szID, szBitrate, acceptance_code, acceptance_mask, flags);
            return CanusbVcp.Open(Marshal.PtrToStringAnsi(szID), szBitrate, acceptance_code, acceptance_mask, flags);
        }

        public static int canusb_Close(uint handle)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Close(handle);
            return CanusbVcp.Close(handle);
        }

        public static int canusb_Read(uint handle, out CANMsg msg)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Read(handle, out msg);
            return CanusbVcp.Read(handle, out msg);
        }

        public static int canusb_ReadFirst(uint h, uint id, uint flags, out CANMsg msg)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_ReadFirst(h, id, flags, out msg);
            throw new PlatformNotSupportedException("canusb_ReadFirst needs canusbdrv.dll"); // ponytail: unused, VCP serves only what CANUSBDevice calls
        }

        /// <summary>
        /// Not a canusbdrv call: how a receive loop waits after ERROR_CANUSB_NO_MESSAGE. The DLL keeps the
        /// 1 ms poll CANUSBDevice always had, the VCP wakes as soon as its reader thread queues a frame.
        /// </summary>
        public static void WaitReceive(uint handle)
        {
            if (OperatingSystem.IsWindows()) Thread.Sleep(1);
            else CanusbVcp.WaitReceive(handle, 50);
        }

        public static int canusb_Write(uint handle, ref CANMsg msg)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Write(handle, ref msg);
            return CanusbVcp.Write(handle, ref msg);
        }

        public static int canusb_Status(uint handle)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Status(handle);
            return CanusbVcp.Status(handle);
        }

        public static int canusb_VersionInfo(uint handle, StringBuilder verinfo)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_VersionInfo(handle, verinfo);
            return CanusbVcp.VersionInfo(handle, verinfo);
        }

        public static int canusb_Flush(uint h, byte flushflags)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_Flush(h, flushflags);
            return CanusbVcp.Flush(h, flushflags);
        }

        public static int canusb_SetTimeouts(uint h, uint receiveTimeout, uint transmitTimeout)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_SetTimeouts(h, receiveTimeout, transmitTimeout);
            return ERROR_CANUSB_OK; // VCP reads never block
        }

        public static int canusb_getFirstAdapter(StringBuilder szAdapter, int size)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_getFirstAdapter(szAdapter, size);
            return CanusbVcp.GetFirstAdapter(szAdapter, size);
        }

        public static int canusb_getNextAdapter(StringBuilder szAdapter, int size)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_getNextAdapter(szAdapter, size);
            return CanusbVcp.GetNextAdapter(szAdapter, size);
        }

        public static int canusb_setReceiveCallBack(uint handle, CANMsgCallbackDef rxMsgCallback)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_setReceiveCallBack(handle, rxMsgCallback);
            throw new PlatformNotSupportedException("canusb_setReceiveCallBack needs canusbdrv.dll"); // ponytail: unused
        }

        public static int canusb_setReceiveCallBack(uint handle, IntPtr szID)
        {
            if (OperatingSystem.IsWindows()) return Dll.canusb_setReceiveCallBack(handle, szID);
            throw new PlatformNotSupportedException("canusb_setReceiveCallBack needs canusbdrv.dll"); // ponytail: unused
        }

        // Same declarations as Lawicel's canusbdrv_net 2.0.0.1; NativeLibs maps "canusbdrv" to canusbdrv.dll / canusbdrv64.dll
        static class Dll
        {
            [DllImport("canusbdrv")]
            public static extern uint canusb_Open(string szID, string szBitrate, uint acceptance_code, uint acceptance_mask, uint flags);

            [DllImport("canusbdrv")]
            public static extern uint canusb_Open(IntPtr szID, string szBitrate, uint acceptance_code, uint acceptance_mask, uint flags);

            [DllImport("canusbdrv")]
            public static extern int canusb_Close(uint handle);

            [DllImport("canusbdrv")]
            public static extern int canusb_Read(uint handle, out CANMsg msg);

            [DllImport("canusbdrv")]
            public static extern int canusb_ReadFirst(uint h, uint id, uint flags, out CANMsg msg);

            [DllImport("canusbdrv")]
            public static extern int canusb_Write(uint handle, ref CANMsg msg);

            [DllImport("canusbdrv")]
            public static extern int canusb_Status(uint handle);

            [DllImport("canusbdrv")]
            public static extern int canusb_VersionInfo(uint handle, StringBuilder verinfo);

            [DllImport("canusbdrv")]
            public static extern int canusb_Flush(uint h, byte flushflags);

            [DllImport("canusbdrv")]
            public static extern int canusb_SetTimeouts(uint h, uint receiveTimeout, uint transmitTimeout);

            [DllImport("canusbdrv")]
            public static extern int canusb_getFirstAdapter(StringBuilder szAdapter, int size);

            [DllImport("canusbdrv")]
            public static extern int canusb_getNextAdapter(StringBuilder szAdapter, int size);

            [DllImport("canusbdrv")]
            public static extern int canusb_setReceiveCallBack(uint handle, CANMsgCallbackDef rxMsgCallback);

            [DllImport("canusbdrv")]
            public static extern int canusb_setReceiveCallBack(uint handle, IntPtr szID);
        }
    }
}
