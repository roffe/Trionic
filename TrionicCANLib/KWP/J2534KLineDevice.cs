using System;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using TrionicCANLib.CAN.J2534;

namespace TrionicCANLib.KWP
{
    /// <summary>
    /// A J2534 device's ISO14230 channel, the K-line on DLC pin 7. The device does the fast init, the checksum, the
    /// echo and the P3 and P4 timing.
    /// </summary>
    public class J2534KLineDevice : KLineDevice
    {
        [StructLayout(LayoutKind.Sequential)]
        private struct SConfig
        {
            public uint Parameter;
            public uint Value;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct SConfigList
        {
            public uint NumOfParams;
            public IntPtr ConfigPtr;
        }

        private const uint P3_MIN = 0x0A, P4_MIN = 0x0C; // in 0.5 ms
        private const int WriteTimeoutMs = 5000;

        private readonly J2534Extended m_passThru = new J2534Extended();
        private int m_deviceId;
        private int m_channelId = -1;
        private bool m_open;

        public static string[] GetAdapterNames()
        {
            return J2534Detect.ListDevices().Where(d => d.IsKLineSupported).Select(d => d.Name).ToArray();
        }

        public override bool open()
        {
            close();
            J2534Device device = J2534Detect.ListDevices().Find(d => d.Name == ForcedComport);
            if (device == null || !m_passThru.LoadLibrary(device))
            {
                logger.Debug("J2534 device " + ForcedComport + " not found or its library could not be loaded");
                return false;
            }
            J2534Err status = m_passThru.PassThruOpen(IntPtr.Zero, ref m_deviceId);
            if (status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug("PassThruOpen: " + status);
                m_passThru.FreeLibrary();
                return false;
            }
            m_open = true;
            status = m_passThru.PassThruConnect(m_deviceId, ProtocolID.ISO14230, ConnectFlag.NONE, BaudRate.ISO14230, ref m_channelId);
            if (status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug("PassThruConnect ISO14230: " + status + lastError());
                m_channelId = -1;
                close();
                return false;
            }
            // a new channel passes nothing; the K-line only carries our own conversation, with the ECU and DICE
            IntPtr all = new PassThruMsg(ProtocolID.ISO14230, TxFlag.NONE, new byte[3]).ToIntPtr();
            try
            {
                int filterId = 0;
                status = m_passThru.PassThruStartMsgFilter(m_channelId, FilterType.PASS_FILTER, all, all, IntPtr.Zero, ref filterId);
            }
            finally
            {
                Marshal.FreeHGlobal(all);
            }
            if (status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug("PassThruStartMsgFilter: " + status);
                close();
                return false;
            }
            return true;
        }

        public override bool close()
        {
            if (!m_open)
            {
                return true;
            }
            m_open = false;
            if (m_channelId >= 0)
            {
                m_passThru.PassThruDisconnect(m_channelId);
                m_channelId = -1;
            }
            m_passThru.PassThruClose(m_deviceId);
            m_passThru.FreeLibrary();
            return true;
        }

        public override bool isOpen()
        {
            return m_open;
        }

        protected override void setTiming(bool a_fast)
        {
            SConfig[] config =
            {
                new SConfig { Parameter = P3_MIN, Value = a_fast ? 10u : 110u },
                new SConfig { Parameter = P4_MIN, Value = a_fast ? 0u : 10u },
            };
            GCHandle pinned = GCHandle.Alloc(config, GCHandleType.Pinned);
            IntPtr list = Marshal.AllocHGlobal(Marshal.SizeOf<SConfigList>());
            try
            {
                Marshal.StructureToPtr(new SConfigList { NumOfParams = (uint)config.Length, ConfigPtr = pinned.AddrOfPinnedObject() }, list, false);
                J2534Err status = m_passThru.PassThruIoctl(m_channelId, (int)Ioctl.SET_CONFIG, list, IntPtr.Zero);
                if (status != J2534Err.STATUS_NOERROR)
                {
                    logger.Debug("SET_CONFIG P3_MIN / P4_MIN: " + status);
                }
            }
            finally
            {
                Marshal.FreeHGlobal(list);
                pinned.Free();
            }
        }

        protected override byte[] fastInit(byte[] a_frame)
        {
            IntPtr input = new PassThruMsg(ProtocolID.ISO14230, TxFlag.NONE, a_frame).ToIntPtr();
            IntPtr output = new PassThruMsg(ProtocolID.ISO14230, TxFlag.NONE, Array.Empty<byte>()).ToIntPtr();
            try
            {
                J2534Err status = m_passThru.PassThruIoctl(m_channelId, (int)Ioctl.FAST_INIT, input, output);
                if (status != J2534Err.STATUS_NOERROR)
                {
                    logger.Debug("FAST_INIT: " + status + lastError());
                    return null;
                }
                return payload(output.AsStruct<PassThruMsg>().GetBytes());
            }
            finally
            {
                Marshal.FreeHGlobal(input);
                Marshal.FreeHGlobal(output);
            }
        }

        protected override bool send(byte[] a_frame)
        {
            // a late reply to a request that timed out isn't the answer to this one
            m_passThru.PassThruIoctl(m_channelId, (int)Ioctl.CLEAR_RX_BUFFER, IntPtr.Zero, IntPtr.Zero);
            IntPtr msg = new PassThruMsg(ProtocolID.ISO14230, TxFlag.NONE, a_frame).ToIntPtr();
            try
            {
                int numMsgs = 1;
                J2534Err status = m_passThru.PassThruWriteMsgs(m_channelId, msg, ref numMsgs, WriteTimeoutMs);
                if (status != J2534Err.STATUS_NOERROR)
                {
                    logger.Debug("PassThruWriteMsgs: " + status + lastError());
                    return false;
                }
                return true;
            }
            finally
            {
                Marshal.FreeHGlobal(msg);
            }
        }

        protected override byte[] receive()
        {
            IntPtr msg = Marshal.AllocHGlobal(Marshal.SizeOf<PassThruMsg>());
            try
            {
                Stopwatch sw = Stopwatch.StartNew();
                for (int left; (left = ReadDeadlineMs - (int)sw.ElapsedMilliseconds) > 0;)
                {
                    int numMsgs = 1;
                    long started = sw.ElapsedMilliseconds;
                    J2534Err status = m_passThru.PassThruReadMsgs(m_channelId, msg, ref numMsgs, left);
                    if (status != J2534Err.STATUS_NOERROR && status != J2534Err.ERR_TIMEOUT && status != J2534Err.ERR_BUFFER_EMPTY)
                    {
                        logger.Debug("PassThruReadMsgs: " + status);
                        return null;
                    }
                    if (numMsgs < 1)
                    {
                        // a driver that ignores the timeout would spin a core
                        if (sw.ElapsedMilliseconds == started)
                        {
                            Thread.Sleep(1);
                        }
                        continue;
                    }
                    PassThruMsg rx = msg.AsStruct<PassThruMsg>();
                    if ((rx.RxStatus & RxStatus.TX_MSG_TYPE) != 0)
                    {
                        continue; // the loopback of our request
                    }
                    byte[] p = payload(rx.GetBytes());
                    if (p != null && p.Length > 0)
                    {
                        return p;
                    }
                }
                return null;
            }
            finally
            {
                Marshal.FreeHGlobal(msg);
            }
        }

        private string lastError()
        {
            IntPtr text = Marshal.AllocHGlobal(80);
            try
            {
                return m_passThru.PassThruGetLastError(text) == J2534Err.STATUS_NOERROR ? " (" + Marshal.PtrToStringAnsi(text) + ")" : "";
            }
            finally
            {
                Marshal.FreeHGlobal(text);
            }
        }
    }
}
