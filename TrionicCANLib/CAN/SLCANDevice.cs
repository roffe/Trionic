using NLog;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using TrionicCANLib.API;

namespace TrionicCANLib.CAN
{
    // SLCAN Device class by Roffe
    public class SLCANDevice : ICANDevice
    {
        bool m_deviceIsOpen = false;
        SerialPort m_serialPort = new SerialPort();
        Thread m_readThread;
        Object m_synchObject = new Object();
        bool m_endThread = false;
        private Logger logger = LogManager.GetCurrentClassLogger();

        private int m_forcedBaudrate = 3000000;
        private string m_forcedComport = string.Empty;

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

        public static new string[] GetAdapterNames()
        {
            return SerialPort.GetPortNames();
        }

        public override void SetSelectedAdapter(string adapter)
        {
            m_forcedComport = adapter;
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

        public SLCANDevice()
        { 
        }

        ~SLCANDevice()
        {
            lock (m_synchObject)
            {
                m_endThread = true;
            }
            close();
        }

        public void readMessages()
        {
            CANMessage canMessage = new CANMessage();
            string rxMessage = string.Empty;

            logger.Debug("readMessages started");
            while (true)
            {
                lock (m_synchObject)
                {
                    if (m_endThread)
                    {
                        logger.Debug("readMessages ended");
                        return;
                    }
                }
                try
                {
                    if (m_serialPort.IsOpen)
                    {
                        do
                        {
                            rxMessage = m_serialPort.ReadTo("\r");
                            rxMessage = rxMessage.Replace("\r", ""); // remove prompt characters... we don't need that stuff
                        } while (rxMessage.StartsWith("t") == false);

                        uint id = Convert.ToUInt32(rxMessage.Substring(1, 3), 16);
                        if (acceptMessageId(id))
                        {
                            canMessage.setID(id);
                            canMessage.setLength(8);
                            canMessage.setData(0x0000000000000000);
                            for (uint i = 0; i < 8; i++)
                            {
                                canMessage.setCanData(Convert.ToByte(rxMessage.Substring(5 + (2 * (int)i), 2), 16), i);
                            }
                            receivedMessage(canMessage);
                        }
                    }
                }
                catch (Exception)
                {
                    logger.Debug("MSG: " + rxMessage);
                }
            }
        }

        public override uint waitForMessage(uint a_canID, uint timeout, out CANMessage canMsg)
        {
            canMsg = new CANMessage();
            return 0;
        }

        protected override bool sendMessageDevice(CANMessage a_message)
        {
            if (!m_serialPort.IsOpen)
            {
                return false;
            }
            string sendString = "t";
            sendString += a_message.getID().ToString("X3");
            sendString += a_message.getLength().ToString("X1");
            for (uint i = 0; i < a_message.getLength(); i++)
            {
                sendString += a_message.getCanData(i).ToString("X2");
            }
            sendString += "\r";
            m_serialPort.Write(sendString);
            return true;
        }

        public override float GetThermoValue()
        {
            return 0F;
        }

        public override float GetADCValue(uint channel)
        {
            return 0F;
        }

        public override bool isOpen()
        {
            return m_deviceIsOpen;
        }

        public override OpenResult open()
        {
            m_serialPort.BaudRate = m_forcedBaudrate;
            m_serialPort.Handshake = Handshake.None;
            m_serialPort.ReadTimeout = 10;
            if (m_forcedComport != string.Empty)
            {

                logger.Debug("Opening com: " + m_forcedComport);

                if (m_serialPort.IsOpen)
                {
                    m_serialPort.Close();
                }
                    
                m_serialPort.PortName = m_forcedComport;

                try
                {
                    m_serialPort.Open();
                }
                catch (UnauthorizedAccessException)
                {
                    return OpenResult.OpenError;
                }

                m_deviceIsOpen = true;

                if (!UseOnlyPBus && TrionicECU != ECU.TRIONIC5)
                {
                    m_serialPort.Write("S0\r");         // Set Just4trionic CAN speed to 47,619 bits (I-BUS)
                    Thread.Sleep(10);
                    Flush();                       // Flush 'junk' in serial port buffers

                    try
                    {
                        m_serialPort.ReadLine();
                        logger.Debug("Connected to CAN at 47,619 speed");
                        CastInformationEvent("Connected to CAN I-BUS using " + m_forcedComport);

                        if (m_readThread != null)
                        {
                            logger.Debug(m_readThread.ThreadState.ToString());
                        }
                        m_readThread = new Thread(readMessages) { Name = "SLCANDevice.m_readThread" };
                        m_endThread = false; // reset for next tries :)
                        if (m_readThread.ThreadState == ThreadState.Unstarted)
                            m_readThread.Start();

                        Thread.Sleep(10);
                        Flush();                       // Flush 'junk' in serial port buffers
                        return OpenResult.OK;
                    }
                    catch (Exception)
                    {
                        logger.Debug("Unable to connect to the I-BUS");
                    }
                }

                if (TrionicECU == ECU.TRIONIC5)
                {
                    m_serialPort.Write("S9\r");         // Set CAN speed to 615,384 bits (T5-SFI)
                    logger.Debug("Connected to CAN at 615,384 kbits speed");
                }
                else
                {
                    m_serialPort.Write("S6\r");         // Set CAN speed to 500,000 kbits (P-BUS)
                    logger.Debug("Connected to CAN at 500,000 kbits speed");
                }

                Thread.Sleep(10);
                Flush();                       // Flush 'junk' in serial port buffers

                try
                {
                    CastInformationEvent("Connected to CAN P-BUS using " + m_forcedComport);

                    if (m_readThread != null)
                    {
                        CastInformationEvent(m_readThread.ThreadState.ToString());
                        logger.Debug(m_readThread.ThreadState.ToString());
                    }

                    m_readThread = new Thread(readMessages) { Name = "SLCANDevice.m_readThread" };
                    m_endThread = false; // reset for next tries :)
                    if (m_readThread.ThreadState == ThreadState.Unstarted)
                        m_readThread.Start();

                    m_serialPort.Write("O\r");          // 'open'
                    return OpenResult.OK;
                }
                catch (Exception)
                {
                    logger.Debug("Unable to connect to the P-BUS");
                }

                CastInformationEvent("SLCAN cannot connect to a CAN bus.");
                close();
                return OpenResult.OpenError;
            }
            CastInformationEvent("SLCAN doesn't seem to be connected to your computer.");
            close();
            return OpenResult.OpenError;
        }

        public void Flush()
        {
            if (m_deviceIsOpen)
            {
                m_serialPort.DiscardInBuffer();
                m_serialPort.DiscardOutBuffer();
            }
        }

        public override CloseResult close()
        {
            if (m_deviceIsOpen)
                m_serialPort.Write("C\r");          // mbed ESCape CAN interface
            m_endThread = true;
            m_serialPort.Close();
            m_deviceIsOpen = false;

            return CloseResult.OK;
        }
    }
}
