using System;
using System.Collections.Generic;
using System.Threading;
using System.Runtime.InteropServices;
using NLog;
using J2534DotNet;

namespace TrionicCANLib.CAN
{
    /// <summary>
    /// All incomming messages are published to registered ICANListeners.
    /// </summary>
    ///
    public class J2534CANDevice : ICANDevice
    {
        private readonly static Logger logger = LogManager.GetCurrentClassLogger();

        // J2534 puts the CAN id big-endian in the first four data bytes of a message,
        // mask and pattern messages use the same layout.
        private static readonly byte[] IdMask = { 0x00, 0x00, 0x07, 0xFF };
        private static readonly byte[] PassAll = { 0x00, 0x00, 0x00, 0x00 };
        private const int RxBatch = 32;

        volatile bool m_deviceIsOpen = false;
        volatile bool m_endThread;
        Thread m_readThread;
        readonly object m_txLock = new object();

        private int m_forcedBaudrate = 38400;

        readonly J2534Extended passThru = new J2534Extended();
        static List<J2534Device> availableJ2534Devices;
        J2534Device m_selectedDevice;
        int m_deviceId;
        int m_channelId = -1;
        J2534Err m_status;

        public override int ForcedBaudrate
        {
            get
            {
                return m_forcedBaudrate;
            }
            set
            {
                m_forcedBaudrate = value;
            }
        }

        private bool m_filterBypass = false;
        public override bool bypassCANfilters
        {
            get
            {
                return m_filterBypass;
            }
            set
            {
                m_filterBypass = value;
            }
        }

        // The list is swapped while open by the CAN logger and T8 recovery, so the hardware filters follow it
        public override List<uint> AcceptOnlyMessageIds
        {
            get { return m_AcceptedMessageIds; }
            set
            {
                m_AcceptedMessageIds = value;
                if (m_deviceIsOpen)
                {
                    applyFilters();
                }
            }
        }

        // not supported by J2534
        public override float GetADCValue(uint channel)
        {
            return 0F;
        }

        // not supported by J2534
        public override float GetThermoValue()
        {
            return 0F;
        }

        public static new string[] GetAdapterNames()
        {
            // Find all of the installed J2534 passthru devices
            availableJ2534Devices = J2534Detect.ListDevices();

            List<string> names = new List<string>();
            foreach (J2534Device device in availableJ2534Devices)
            {
                if (device.IsCANSupported)
                {
                    names.Add(device.Name);
                    logger.Debug(String.Format("Found device with CAN support {0}", device.Name));
                }
                else
                {
                    logger.Debug(String.Format("Skipped device without CAN support {0}", device.Name));
                }
            }
            return names.ToArray();
        }

        public override void SetSelectedAdapter(string adapter)
        {
            if (availableJ2534Devices == null)
            {
                GetAdapterNames();
            }
            m_selectedDevice = availableJ2534Devices.Find(x => x.Name == adapter);
            if (m_selectedDevice == null)
            {
                logger.Debug(String.Format("J2534 adapter {0} not found", adapter));
            }
        }

        /// <summary>
        /// readMessages is the "run" method of this class. It reads all incomming messages
        /// and publishes them to registered ICANListeners.
        /// </summary>
        public void readMessages()
        {
            IntPtr rxMsgs = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(PassThruMsg)) * RxBatch);
            CANMessage canMessage = new CANMessage();
            logger.Debug("readMessages started");
            try
            {
                while (!m_endThread)
                {
                    try
                    {
                        // In: room in the buffer. Out: messages actually read. Must be reset before every call.
                        int numMsgs = RxBatch;
                        J2534Err status = passThru.PassThruReadMsgs(m_channelId, rxMsgs, ref numMsgs, 0);
                        if (status != J2534Err.STATUS_NOERROR && status != J2534Err.ERR_TIMEOUT && status != J2534Err.ERR_BUFFER_EMPTY)
                        {
                            logger.Debug(String.Format("PassThruReadMsgs, status:{0}", status));
                            Thread.Sleep(10);
                            continue;
                        }
                        if (numMsgs <= 0)
                        {
                            Thread.Sleep(1);
                            continue;
                        }

                        foreach (PassThruMsg msg in rxMsgs.AsList<PassThruMsg>(Math.Min(numMsgs, RxBatch)))
                        {
                            // Skip echoes of our own frames and anything that is not id + 0..8 data bytes
                            if ((msg.RxStatus & (RxStatus.TX_MSG_TYPE | RxStatus.TX_INDICATION)) != 0 || msg.DataSize < 4 || msg.DataSize > 12)
                            {
                                continue;
                            }

                            byte[] all = msg.GetBytes();
                            uint id = (uint)(all[2] << 8 | all[3]);
                            if (!acceptMessageId(id))
                            {
                                continue;
                            }

                            byte length = (byte)(msg.DataSize - 4);
                            byte[] data = new byte[length];
                            Array.Copy(all, 4, data, 0, length);

                            canMessage.setID(id);
                            canMessage.setTimeStamp(msg.Timestamp);
                            canMessage.setCanData(data, length);

                            receivedMessage(canMessage);
                        }
                    }
                    catch (Exception e)
                    {
                        // An unhandled exception here would take the whole process down
                        logger.Debug(e, "readMessages");
                        Thread.Sleep(10);
                    }
                }
            }
            finally
            {
                Marshal.FreeHGlobal(rxMsgs);
            }
            logger.Debug("readMessages thread ended");
        }

        /// <summary>
        /// </summary>
        /// <returns>OpenResult.OK is returned on success. Otherwise OpenResult.OpenError is
        /// returned.</returns>
        override public OpenResult open()
        {
            if (isOpen())
            {
                close();
            }

            if (m_selectedDevice == null || !passThru.LoadLibrary(m_selectedDevice))
            {
                logger.Debug("No J2534 adapter selected or its DLL could not be loaded");
                return OpenResult.OpenError;
            }

            m_deviceId = 0;
            m_channelId = -1;
            m_status = passThru.PassThruOpen(IntPtr.Zero, ref m_deviceId);
            if (m_status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug(String.Format("PassThruOpen, status:{0}", m_status));
                passThru.FreeLibrary();
                return OpenResult.OpenError;
            }

            // From here on close() releases whatever has been claimed
            m_deviceIsOpen = true;
            if (!connect())
            {
                close();
                return OpenResult.OpenError;
            }

            logger.Debug("P bus connected");
            m_endThread = false;
            m_readThread = new Thread(readMessages) { Name = "J2534CANDevice.m_readThread", IsBackground = true };
            m_readThread.Start();
            return OpenResult.OK;
        }

        private bool connect()
        {
            BaudRate baudRate = TrionicECU == API.ECU.TRIONIC5 ? BaudRate.CAN_615000 : BaudRate.CAN_500000;
            m_status = passThru.PassThruConnect(m_deviceId, ProtocolID.CAN, ConnectFlag.NONE, baudRate, ref m_channelId);
            if (m_status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug(String.Format("PassThruConnect, status:{0}", m_status));
                m_channelId = -1;
                return false;
            }

            if (!applyFilters())
            {
                return false;
            }

            m_status = passThru.PassThruIoctl(m_channelId, (int)Ioctl.CLEAR_RX_BUFFER, IntPtr.Zero, IntPtr.Zero);
            if (m_status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug(String.Format("CLEAR_RX_BUFFER, status:{0}", m_status));
                return false;
            }
            return true;
        }

        /// <summary>
        /// A CAN channel receives nothing until at least one PASS_FILTER is set.
        /// One exact filter per accepted id keeps the rest of the bus out of the read loop.
        /// </summary>
        private bool applyFilters()
        {
            lock (m_txLock)
            {
                passThru.PassThruIoctl(m_channelId, (int)Ioctl.CLEAR_MSG_FILTERS, IntPtr.Zero, IntPtr.Zero);

                bool filtered = !m_filterBypass && AcceptOnlyMessageIds != null;
                if (filtered)
                {
                    foreach (uint id in AcceptOnlyMessageIds)
                    {
                        if (!addFilter(IdMask, new byte[] { 0x00, 0x00, (byte)(id >> 8), (byte)id }))
                        {
                            logger.Debug("Id filter rejected, falling back to pass all");
                            filtered = false;
                            break;
                        }
                    }
                }
                return filtered || addFilter(PassAll, PassAll);
            }
        }

        private bool addFilter(byte[] mask, byte[] pattern)
        {
            // ToIntPtr allocates unmanaged memory, we own the free
            IntPtr maskPtr = new PassThruMsg(ProtocolID.CAN, TxFlag.NONE, mask).ToIntPtr();
            IntPtr patternPtr = new PassThruMsg(ProtocolID.CAN, TxFlag.NONE, pattern).ToIntPtr();
            try
            {
                int filterId = 0;
                m_status = passThru.PassThruStartMsgFilter(m_channelId, FilterType.PASS_FILTER, maskPtr, patternPtr, IntPtr.Zero, ref filterId);
                if (m_status != J2534Err.STATUS_NOERROR)
                {
                    logger.Debug(String.Format("PassThruStartMsgFilter {0}, status:{1}", BitConverter.ToString(pattern), m_status));
                    return false;
                }
                return true;
            }
            finally
            {
                Marshal.FreeHGlobal(maskPtr);
                Marshal.FreeHGlobal(patternPtr);
            }
        }

        /// <summary>
        /// The close method closes the device.
        /// </summary>
        /// <returns>CloseResult.OK on success, otherwise CloseResult.CloseError.</returns>
        override public CloseResult close()
        {
            if (!m_deviceIsOpen)
            {
                return CloseResult.OK;
            }
            m_deviceIsOpen = false;
            m_endThread = true;
            if (m_readThread != null)
            {
                m_readThread.Join(2000);
                m_readThread = null;
            }

            lock (m_txLock)
            {
                if (m_channelId >= 0)
                {
                    passThru.PassThruDisconnect(m_channelId);
                    m_channelId = -1;
                }
                m_status = passThru.PassThruClose(m_deviceId);
            }
            passThru.FreeLibrary();
            return m_status == J2534Err.STATUS_NOERROR ? CloseResult.OK : CloseResult.CloseError;
        }

        /// <summary>
        /// isOpen checks if the device is open.
        /// </summary>
        /// <returns>true if the device is open, otherwise false.</returns>
        override public bool isOpen()
        {
            return m_deviceIsOpen;
        }

        /// <summary>
        /// sendMessage send a CANMessage.
        /// </summary>
        /// <param name="a_message">A CANMessage.</param>
        /// <returns>true on success, othewise false.</returns>
        override protected bool sendMessageDevice(CANMessage a_message)
        {
            byte[] msg = a_message.getHeaderAndData();
            PassThruMsg txMsg = new PassThruMsg(ProtocolID.CAN, TxFlag.NONE, msg);
            J2534Err status;

            // The T8 keep alive timer sends from another thread
            lock (m_txLock)
            {
                if (!m_deviceIsOpen)
                {
                    return false;
                }
                // ToIntPtr allocates unmanaged memory, we own the free
                IntPtr txPtr = txMsg.ToIntPtr();
                try
                {
                    int numMsgs = 1;
                    status = passThru.PassThruWriteMsgs(m_channelId, txPtr, ref numMsgs, 0);
                }
                finally
                {
                    Marshal.FreeHGlobal(txPtr);
                }
            }

            if (status != J2534Err.STATUS_NOERROR)
            {
                logger.Debug(String.Format("tx failed with status {0} {1}", status, BitConverter.ToString(msg)));
                return false;
            }
            return true;
        }

        /// <summary>
        /// waitForMessage waits for a specific CAN message give by a CAN id.
        /// </summary>
        /// <param name="a_canID">The CAN id to listen for</param>
        /// <param name="timeout">Listen timeout</param>
        /// <param name="r_canMsg">The CAN message with a_canID that we where listening for.</param>
        /// <returns>The CAN id for the message we where listening for, otherwise 0.</returns>
        public override uint waitForMessage(uint a_canID, uint timeout, out CANMessage canMsg)
        {
            canMsg = new CANMessage();
            return 0;
        }
    }
}
