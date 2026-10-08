using System;
using System.Runtime.InteropServices;

namespace TrionicCANLib.CAN.J2534
{
    /// <summary>
    /// Binds a vendor J2534 v04.04 library (Windows DLL, Linux/macOS shared object) at runtime.
    /// The name is kept from J2534DotNet, which this replaces.
    /// </summary>
    public class J2534Extended
    {
        // WINAPI: stdcall on Windows x86, the platform default everywhere else
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int OpenFn(IntPtr name, ref int deviceId);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int CloseFn(int deviceId);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int ConnectFn(int deviceId, int protocolId, int flags, int baudRate, ref int channelId);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int DisconnectFn(int channelId);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int MsgsFn(int channelId, IntPtr pMessages, ref int numMsgs, int timeout);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int StartMsgFilterFn(int channelId, int filterType, IntPtr maskMsg, IntPtr patternMsg, IntPtr flowControlMsg, ref int filterId);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int ReadVersionFn(int deviceId, IntPtr firmwareVersion, IntPtr dllVersion, IntPtr apiVersion);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int GetLastErrorFn(IntPtr errorDescription);
        [UnmanagedFunctionPointer(CallingConvention.Winapi)] delegate int IoctlFn(int channelId, int ioctlID, IntPtr input, IntPtr output);

        IntPtr m_pDll;
        OpenFn m_open;
        CloseFn m_close;
        ConnectFn m_connect;
        DisconnectFn m_disconnect;
        MsgsFn m_readMsgs;
        MsgsFn m_writeMsgs;
        StartMsgFilterFn m_startMsgFilter;
        ReadVersionFn m_readVersion;
        GetLastErrorFn m_getLastError;
        IoctlFn m_ioctl;

        /// <summary>
        /// Loads the device's FunctionLibrary. False when it cannot be loaded or is not a J2534 library.
        /// </summary>
        public bool LoadLibrary(J2534Device device)
        {
            FreeLibrary();
            if (string.IsNullOrEmpty(device.FunctionLibrary) || !NativeLibrary.TryLoad(device.FunctionLibrary, out m_pDll))
            {
                return false;
            }
            if (Bind("PassThruOpen", out m_open) &&
                Bind("PassThruClose", out m_close) &&
                Bind("PassThruConnect", out m_connect) &&
                Bind("PassThruDisconnect", out m_disconnect) &&
                Bind("PassThruReadMsgs", out m_readMsgs) &&
                Bind("PassThruWriteMsgs", out m_writeMsgs) &&
                Bind("PassThruStartMsgFilter", out m_startMsgFilter) &&
                Bind("PassThruReadVersion", out m_readVersion) &&
                Bind("PassThruGetLastError", out m_getLastError) &&
                Bind("PassThruIoctl", out m_ioctl))
            {
                return true;
            }
            // Not a J2534 library, don't leave it mapped
            FreeLibrary();
            return false;
        }

        bool Bind<T>(string name, out T fn) where T : Delegate
        {
            fn = NativeLibrary.TryGetExport(m_pDll, name, out IntPtr address) ? Marshal.GetDelegateForFunctionPointer<T>(address) : null;
            return fn != null;
        }

        public bool FreeLibrary()
        {
            if (m_pDll == IntPtr.Zero)
            {
                return false;
            }
            // A read or write thread that outlived close() gets a NullReferenceException, not unmapped code
            m_readMsgs = m_writeMsgs = null;
            NativeLibrary.Free(m_pDll);
            m_pDll = IntPtr.Zero;
            return true;
        }

        public J2534Err PassThruOpen(IntPtr name, ref int deviceId)
        {
            return (J2534Err)m_open(name, ref deviceId);
        }

        public J2534Err PassThruClose(int deviceId)
        {
            return (J2534Err)m_close(deviceId);
        }

        public J2534Err PassThruConnect(int deviceId, ProtocolID protocolId, ConnectFlag flags, BaudRate baudRate, ref int channelId)
        {
            return (J2534Err)m_connect(deviceId, (int)protocolId, (int)flags, (int)baudRate, ref channelId);
        }

        public J2534Err PassThruDisconnect(int channelId)
        {
            return (J2534Err)m_disconnect(channelId);
        }

        public J2534Err PassThruReadMsgs(int channelId, IntPtr msgs, ref int numMsgs, int timeout)
        {
            return (J2534Err)m_readMsgs(channelId, msgs, ref numMsgs, timeout);
        }

        public J2534Err PassThruWriteMsgs(int channelId, IntPtr msgs, ref int numMsgs, int timeout)
        {
            return (J2534Err)m_writeMsgs(channelId, msgs, ref numMsgs, timeout);
        }

        public J2534Err PassThruStartMsgFilter(int channelid, FilterType filterType, IntPtr maskMsg, IntPtr patternMsg, IntPtr flowControlMsg, ref int filterId)
        {
            return (J2534Err)m_startMsgFilter(channelid, (int)filterType, maskMsg, patternMsg, flowControlMsg, ref filterId);
        }

        /// <summary>Each buffer must hold 80 chars.</summary>
        public J2534Err PassThruReadVersion(int deviceId, IntPtr firmwareVersion, IntPtr dllVersion, IntPtr apiVersion)
        {
            return (J2534Err)m_readVersion(deviceId, firmwareVersion, dllVersion, apiVersion);
        }

        /// <summary>The buffer must hold 80 chars.</summary>
        public J2534Err PassThruGetLastError(IntPtr errorDescription)
        {
            return (J2534Err)m_getLastError(errorDescription);
        }

        public J2534Err PassThruIoctl(int channelId, int ioctlID, IntPtr input, IntPtr output)
        {
            return (J2534Err)m_ioctl(channelId, ioctlID, input, output);
        }
    }
}
