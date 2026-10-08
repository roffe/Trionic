using System;
using System.Runtime.InteropServices;

namespace canlibCLSNET
{
    /// <summary>
    /// In-tree replacement for Kvaser's Windows-only canlibCLSNET.dll, same names and values,
    /// limited to what KvaserCANDevice uses. "canlib32" is mapped by NativeLibs to canlib32.dll
    /// (Windows) or libcanlib.so.1 (linuxcan). C long is 32-bit on Windows and 64-bit on Linux,
    /// hence CLong/CULong. Default calling convention = stdcall on win-x86, like CANLIBAPI.
    /// </summary>
    public static class Canlib
    {
        public enum canStatus
        {
            canOK = 0,
            canERR_PARAM = -1,
            canERR_NOMSG = -2,
            canERR_NOTFOUND = -3,
            canERR_NOMEM = -4,
            canERR_NOCHANNELS = -5,
            canERR_RESERVED_3 = -6,
            canERR_TIMEOUT = -7,
            canERR_NOTINITIALIZED = -8,
            canERR_NOHANDLES = -9,
            canERR_INVHANDLE = -10,
            canERR_INIFILE = -11,
            canERR_DRIVER = -12,
            canERR_TXBUFOFL = -13,
            canERR_RESERVED_1 = -14,
            canERR_HARDWARE = -15,
            canERR_DYNALOAD = -16,
            canERR_DYNALIB = -17,
            canERR_DYNAINIT = -18,
            canERR_RESERVED_4 = -19,
            canERR_RESERVED_5 = -20,
            canERR_RESERVED_6 = -21,
            canERR_RESERVED_2 = -22,
            canERR_DRIVERLOAD = -23,
            canERR_DRIVERFAILED = -24,
            canERR_NOCONFIGMGR = -25,
            canERR_NOCARD = -26,
            canERR_RESERVED_7 = -27,
            canERR_REGISTRY = -28,
            canERR_LICENSE = -29,
            canERR_INTERNAL = -30,
            canERR_NO_ACCESS = -31,
            canERR_NOT_IMPLEMENTED = -32,
            canERR_DEVICE_FILE = -33,
            canERR_HOST_FILE = -34,
            canERR_DISK = -35,
            canERR_CRC = -36,
            canERR_CONFIG = -37,
            canERR_MEMO_FAIL = -38,
            canERR_SCRIPT_FAIL = -39,
            canERR_SCRIPT_WRONG_VERSION = -40,
            canERR__RESERVED = -41
        }

        public const int canMSG_ERROR_FRAME = 0x20;
        public const int canBITRATE_500K = -2;
        public const int canCHANNELDATA_CHANNEL_CAP = 1;
        public const int canCHANNELDATA_CHANNEL_NAME = 13;
        public const int canCHANNEL_CAP_VIRTUAL = 0x10000;
        public const int canIOCTL_SET_LOCAL_TXECHO = 32;

        const int canFDMSG_FDF = 0x10000;

        static unsafe class Native
        {
            const string Lib = "canlib32";
            [DllImport(Lib)] public static extern void canInitializeLibrary();
            [DllImport(Lib)] public static extern canStatus canGetNumberOfChannels(int* channelCount);
            [DllImport(Lib)] public static extern canStatus canGetChannelData(int channel, int item, void* buffer, nuint bufsize);
            [DllImport(Lib)] public static extern int canOpenChannel(int channel, int flags);
            [DllImport(Lib)] public static extern canStatus canSetBusParams(int hnd, CLong freq, uint tseg1, uint tseg2, uint sjw, uint noSamp, uint syncmode);
            [DllImport(Lib)] public static extern canStatus canSetBusParamsC200(int hnd, byte btr0, byte btr1);
            [DllImport(Lib)] public static extern canStatus canSetAcceptanceFilter(int hnd, uint code, uint mask, int is_extended);
            [DllImport(Lib)] public static extern canStatus canIoCtl(int hnd, uint func, void* buf, uint buflen);
            [DllImport(Lib)] public static extern canStatus canBusOn(int hnd);
            [DllImport(Lib)] public static extern canStatus canBusOff(int hnd);
            [DllImport(Lib)] public static extern canStatus canClose(int hnd);
            [DllImport(Lib)] public static extern canStatus canReadWait(int hnd, CLong* id, void* msg, uint* dlc, uint* flag, CULong* time, CULong timeout);
            [DllImport(Lib)] public static extern canStatus canWrite(int hnd, CLong id, void* msg, uint dlc, uint flag);
        }

        public static void canInitializeLibrary()
        {
            Native.canInitializeLibrary();
        }

        public static unsafe canStatus canGetNumberOfChannels(out int channelCount)
        {
            int n = 0;
            canStatus status = Native.canGetNumberOfChannels(&n);
            channelCount = n;
            return status;
        }

        /// <summary>buffer is a uint for canCHANNELDATA_CHANNEL_CAP, a string for canCHANNELDATA_CHANNEL_NAME, null on failure.</summary>
        public static unsafe canStatus canGetChannelData(int channel, int item, out object buffer)
        {
            buffer = null;
            canStatus status;
            switch (item)
            {
                case canCHANNELDATA_CHANNEL_CAP:
                {
                    uint cap = 0;
                    status = Native.canGetChannelData(channel, item, &cap, sizeof(uint));
                    if (status == canStatus.canOK) buffer = cap;
                    return status;
                }
                case canCHANNELDATA_CHANNEL_NAME:
                {
                    // zeroed: linuxcan strncpy's bufsize - 1 bytes and leaves the last one alone
                    byte[] name = new byte[1024];
                    fixed (byte* p = name)
                    {
                        status = Native.canGetChannelData(channel, item, p, (nuint)name.Length);
                        if (status == canStatus.canOK) buffer = Marshal.PtrToStringAnsi((IntPtr)p);
                    }
                    return status;
                }
                default:
                    // ponytail: only the items KvaserCANDevice reads; canlibCLSNET also decoded serials, versions and unicode names
                    return canStatus.canERR_PARAM;
            }
        }

        public static int canOpenChannel(int channel, int flags)
        {
            return Native.canOpenChannel(channel, flags);
        }

        public static canStatus canSetBusParams(int handle, int freq, int tseg1, int tseg2, int sjw, int noSamp, int syncmode)
        {
            return Native.canSetBusParams(handle, new CLong(freq), (uint)tseg1, (uint)tseg2, (uint)sjw, (uint)noSamp, (uint)syncmode);
        }

        public static canStatus canSetBusParamsC200(int hnd, byte btr0, byte btr1)
        {
            return Native.canSetBusParamsC200(hnd, btr0, btr1);
        }

        public static canStatus canSetAcceptanceFilter(int hnd, int code, int mask, int is_extended)
        {
            return Native.canSetAcceptanceFilter(hnd, (uint)code, (uint)mask, is_extended);
        }

        public static unsafe canStatus canIoCtl(int handle, int func, int val)
        {
            return Native.canIoCtl(handle, (uint)func, &val, sizeof(int));
        }

        public static canStatus canBusOn(int handle)
        {
            return Native.canBusOn(handle);
        }

        public static canStatus canBusOff(int handle)
        {
            return Native.canBusOff(handle);
        }

        public static canStatus canClose(int handle)
        {
            return Native.canClose(handle);
        }

        public static unsafe canStatus canReadWait(int handle, out int id, [Out] byte[] msg, out int dlc, out int flag, out long time, long timeout)
        {
            CLong nid = default;
            CULong ntime = default;
            uint ndlc = 0, nflag = 0;
            byte* data = stackalloc byte[64]; // a CAN FD frame fits
            canStatus status = Native.canReadWait(handle, &nid, data, &ndlc, &nflag, &ntime, new CULong((uint)timeout));
            if (status != canStatus.canOK)
            {
                id = dlc = flag = 0;
                time = 0;
                return status;
            }
            id = (int)nid.Value;
            dlc = (int)ndlc;
            flag = (int)nflag;
            time = (long)ntime.Value;
            int n = Math.Min(Math.Min(dlc, (flag & canFDMSG_FDF) != 0 ? 64 : 8), msg.Length);
            new ReadOnlySpan<byte>(data, n).CopyTo(msg);
            return status;
        }

        public static unsafe canStatus canWrite(int handle, int id, [In] byte[] msg, int dlc, int flag)
        {
            byte* data = stackalloc byte[64];
            int n = Math.Min(Math.Min(dlc, (flag & canFDMSG_FDF) != 0 ? 64 : 8), msg.Length);
            msg.AsSpan(0, n).CopyTo(new Span<byte>(data, 64));
            return Native.canWrite(handle, new CLong(id), data, (uint)dlc, (uint)flag);
        }
    }
}
