using NLog;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Threading;
using TrionicCANLib.API;
using TrionicCANLib.WMI;

namespace TrionicCANLib.CAN
{
    /// <summary>
    /// ScanTool.net STN11xx/STN21xx adapters (OBDLink SX/EX) with the STN's own commands: STBR to 2 Mbit, every frame as
    /// STPX with its header and response count, so no ATSH / ATR switching between frames. The roffe/gocan scantool
    /// adapter on TrionicCANLib's listener model.
    /// The STN is command/response: it only hears the bus inside a frame's reply window, which its '>' prompt ends. One
    /// reader thread hands the reply frames to the listeners as they arrive and frees the ready slot on each prompt; the
    /// next frame waits for it. Plain ELM327s keep CANELM327Device.
    /// </summary>
    public class CANScantoolDevice : ICANDevice
    {
        const int TargetBaudrate = 2000000;
        // power-up default (PP 0C, where close()'s ATZ leaves it) first, then the target (a crashed session), then the rest
        static readonly int[] Baudrates = { 115200, 2000000, 38400, 230400, 921600, 1000000, 57600 };
        const int PromptTimeout = 300; // ms a frame waits for the previous one's prompt

        private Logger logger = LogManager.GetCurrentClassLogger();
        SerialPort m_serialPort = new SerialPort();
        Thread m_readThread;
        object lockObj = new object();
        // taken by a frame, freed by the prompt that ends its reply window
        SemaphoreSlim readySlot = new SemaphoreSlim(1, 1);
        // set on the prompt after STOPPED or ?: the answer to an interrupt
        AutoResetEvent interruptAnswered = new AutoResetEvent(false);
        volatile bool m_open = false;  // the reader thread owns the port
        volatile bool listening = false; // the last frame's reply window may still be open
        private string m_forcedComport = string.Empty;
        readonly bool m_ex; // OBDLink EX (STN2120): other CAN timing registers than the SX (STN1130)

        public CANScantoolDevice(bool obdlinkEx = false)
        {
            m_ex = obdlinkEx;
        }

        public static new string[] GetAdapterNames()
        {
            return COMPortInfo.GetPortNames();
        }

        public override void SetSelectedAdapter(string adapter)
        {
            m_forcedComport = adapter;
        }

        // always 2 Mbit
        public override int ForcedBaudrate { get; set; }

        private bool m_filterBypass = false;
        public override bool bypassCANfilters
        {
            get { return m_filterBypass; }
            set { m_filterBypass = value; }
        }

        public override bool isOpen()
        {
            return m_open;
        }

        public override OpenResult open()
        {
            if (m_serialPort.IsOpen)
                close();
            // before Open: SerialPort opens the tty exclusive (TIOCEXCL), the ioctl would get EBUSY after it. The driver keeps the setting until unplug
            SerialLowLatency.TryEnable(m_forcedComport);
            try
            {
                m_serialPort.PortName = m_forcedComport;
                m_serialPort.BaudRate = Baudrates[0];
                m_serialPort.Handshake = Handshake.None;
                m_serialPort.ReadTimeout = 10;
                m_serialPort.WriteTimeout = 1000;
                m_serialPort.Open();
            }
            catch (Exception e)
            {
                logger.Debug("OPEN: " + e);
                return OpenResult.OpenError;
            }

            string banner = Hunt();
            if (banner == null)
            {
                CastInformationEvent("No OBDLink (STN) adapter answered on " + m_forcedComport + ". For other ELM327 adapters pick ELM327");
                m_serialPort.Close();
                return OpenResult.OpenError;
            }
            CastInformationEvent("Connected on com: " + m_forcedComport + " with baudrate: " + m_serialPort.BaudRate + ", " + banner);

            var init = new List<string>
            {
                "ATE0",     // echo off (the probe already sent it)
                "ATS0",     // spaces off
                "ATV1",     // variable DLC: a frame goes out with its own length, as on the other adapters
                "STP33",    // ISO 15765, 11-bit, 500 kbit/s
                "ATH1",     // headers on: the reply lines carry their CAN id
                "ATAT2",    // aggressive adaptive timing: how long a frame sent without a response count listens after the last reply
                "ATCAF0",   // raw frames, Trionic builds its own PCI bytes. Automatic flow control stays on: the T8 reads leave the 0x30 to the STN
                "STCMM1",   // ACK received frames: an unACKed ECU retransmits back-to-back (gocan, seen on STN1130 v5.10.1)
            };
            if (TrionicECU == ECU.TRIONIC5)
                init.Add(m_ex ? "STCTR82239F" : "STCTR8101FC"); // 615 kbit/s, gocan's values
            else if (TrionicECU == ECU.TRIONIC7 && !UseOnlyPBus)
            {
                if (m_ex)
                {
                    // ponytail: no 47.6 kbit/s timing known for the STN2120; the SX value would put it on the wrong speed. Add it when someone measures one
                    CastInformationEvent("The OBDLink EX has no I-bus timing yet, use Only P-bus");
                    m_serialPort.Close();
                    return OpenResult.OpenError;
                }
                init.Add("STCTR290284"); // 47.6 kbit/s I-bus
            }
            // ID 0x000 would thrash the filter calculation, bypass the filter if it's there
            if (!m_filterBypass && AcceptOnlyMessageIds != null && AcceptOnlyMessageIds.Count > 0 && !AcceptOnlyMessageIds.Contains(0))
            {
                uint filt = 0xFFF, mask = 0;
                foreach (var id in AcceptOnlyMessageIds)
                {
                    filt &= id;
                    mask |= id;
                }
                init.Add("ATCM" + ((~mask & 0x7FF) | filt).ToString("X3"));
                init.Add("ATCF" + filt.ToString("X3"));
            }
            foreach (var cmd in init)
            {
                var lines = Exec(cmd, 200);
                if (!lines.Contains("OK"))
                {
                    // the bus settings must take, the rest is tuning
                    logger.Debug("OPEN: " + cmd + " answered " + string.Join("|", lines));
                    if (cmd.StartsWith("STP") || cmd.StartsWith("STCTR"))
                    {
                        CastInformationEvent("The adapter refused " + cmd);
                        m_serialPort.Close();
                        return OpenResult.OpenError;
                    }
                }
            }

            m_serialPort.ReadTimeout = 100; // the reader's wake-up to notice close()
            readySlot.Wait(0);
            readySlot.Release();
            listening = false;
            m_open = true;
            m_readThread = new Thread(readMessages) { Name = "CANScantoolDevice.m_readThread", IsBackground = true };
            m_readThread.Start();
            return OpenResult.OK;
        }

        public override CloseResult close()
        {
            if (m_serialPort.IsOpen)
            {
                lock (lockObj)
                {
                    // a reply window would swallow the ATZ line
                    if (m_open && listening)
                        Interrupt();
                    m_open = false;
                    try
                    {
                        // full reset, back to 115.2 kbit/s where the next open looks first. gocan: a warm-started STN1130 v5.10.1 misbehaved
                        WriteToSerialWithTrace("ATZ\r");
                        Thread.Sleep(10);
                        m_serialPort.Close();
                    }
                    catch (Exception e)
                    {
                        logger.Debug("CLOSE: " + e.Message);
                    }
                }
                m_readThread?.Join(500);
            }
            m_open = false;
            return CloseResult.OK;
        }

        protected override bool sendMessageDevice(CANMessage a_message)
        {
            lock (lockObj)
            {
                if (!m_open)
                    return false;
                if (!readySlot.Wait(PromptTimeout))
                {
                    // writing into an open reply window would only interrupt it and lose the frame
                    logger.Debug("No prompt in " + PromptTimeout + " ms, interrupting");
                    Interrupt();
                    readySlot.Wait(PromptTimeout);
                }
                // -1: no count, the STN listens until its adaptive timeout. 0 answers with the prompt right away
                int responses = a_message.elmExpectedResponses;
                var cmd = new StringBuilder("STPXh:", 48).Append(a_message.getID().ToString("X3")).Append(",d:");
                ulong data = a_message.getData();
                for (int i = 0; i < a_message.getLength(); i++)
                    cmd.Append(((byte)(data >> (i * 8))).ToString("X2"));
                if (responses >= 0)
                    cmd.Append(",r:").Append(responses);
                listening = responses != 0;
                WriteToSerialWithTrace(cmd.Append('\r').ToString());
                return true;
            }
        }

        /// <summary>
        /// Ends a reply window when the caller has all it waited for but the STN would listen on: Legion's skip tag is one
        /// frame where 19 were asked for.
        /// </summary>
        public override void RequestDeviceReady()
        {
            lock (lockObj)
            {
                if (m_open && listening)
                    Interrupt();
            }
        }

        // Any character ends a reply window: the STN answers STOPPED and drops the rest of the line. When the window already
        // closed, the line is an unknown command, answered with ?. Either way the reader frees the ready slot on the prompt
        private void Interrupt()
        {
            interruptAnswered.Reset();
            WriteToSerialWithTrace("##########\r");
            if (!interruptAnswered.WaitOne(PromptTimeout))
                logger.Debug("Interrupt: no answer in " + PromptTimeout + " ms");
        }

        private void readMessages()
        {
            var buffer = new byte[1024];
            var line = new StringBuilder();
            var canMessage = new CANMessage();
            bool interrupted = false; // STOPPED or ? since the last prompt

            logger.Debug("readMessages started");
            while (m_open)
            {
                int read;
                try
                {
                    // A pending read keeps .NET's serial loop in poll(), which wakes on data; DataReceived + ReadExisting wait out
                    // two of its 1 ms idle sleeps and a thread pool hop (3.2 ms vs 1.1 ms per frame against an instant ELM)
                    read = m_serialPort.BaseStream.Read(buffer, 0, buffer.Length);
                }
                catch (TimeoutException)
                {
                    continue;
                }
                catch (Exception e)
                {
                    if (m_open)
                    {
                        logger.Debug("readMessages: " + e.Message);
                        Thread.Sleep(10); // unplugged: don't spin
                    }
                    continue;
                }
                for (int i = 0; i < read; i++)
                {
                    char c = (char)buffer[i];
                    if (c == '\r' || c == '>')
                    {
                        if (line.Length > 0)
                        {
                            interrupted |= HandleLine(line.ToString(), canMessage);
                            line.Clear();
                        }
                        if (c == '>')
                        {
                            listening = false;
                            if (interrupted)
                            {
                                interrupted = false;
                                interruptAnswered.Set();
                            }
                            readySlot.Wait(0);
                            readySlot.Release();
                        }
                    }
                    else if (c != '\n')
                    {
                        line.Append(c);
                    }
                }
            }
            logger.Debug("readMessages ended");
        }

        // true for STOPPED and ?, whose prompt answers an interrupt
        private bool HandleLine(string line, CANMessage canMessage)
        {
            logger.Debug("SERRX: " + line);
            if (line == "STOPPED" || line == "?")
                return true;
            if (TryParseFrame(line, canMessage) && acceptMessageId(canMessage.getID()))
                receivedMessage(canMessage);
            return false;
        }

        /// <summary>
        /// A reply line "iiiDDDD..": 3 hex digits of id, 0 to 8 data bytes (ATS0, ATH1). Status lines (OK, NO DATA,
        /// CAN ERROR, BUFFER FULL, ...) aren't hex.
        /// </summary>
        internal static bool TryParseFrame(string line, CANMessage canMessage)
        {
            if (line.Length < 3 || line.Length > 19 || line.Length % 2 == 0)
                return false;
            if (!uint.TryParse(line.AsSpan(0, 3), NumberStyles.AllowHexSpecifier, null, out uint id))
                return false;
            int length = (line.Length - 3) / 2;
            ulong data = 0;
            for (int i = 0; i < length; i++)
            {
                if (!byte.TryParse(line.AsSpan(3 + i * 2, 2), NumberStyles.AllowHexSpecifier, null, out byte b))
                    return false;
                data |= (ulong)b << (i * 8);
            }
            canMessage.setID(id);
            canMessage.setLength((byte)length);
            canMessage.setData(data);
            return true;
        }

        /// <summary>
        /// Finds the STN at whatever rate it is at and walks the STBR handshake to 2 Mbit. A failed handshake leaves it at its
        /// old rate (STBRT) and a device mid-reboot answers nothing, so the sweep repeats until the deadline.
        /// </summary>
        /// <returns>the STI banner, null when no STN answered</returns>
        private string Hunt()
        {
            long deadline = Environment.TickCount64 + 4000;
            do
            {
                foreach (int from in Baudrates)
                {
                    string banner = TrySpeed(from);
                    if (banner != null)
                        return banner;
                }
            }
            while (Environment.TickCount64 < deadline);
            return null;
        }

        private string TrySpeed(int from)
        {
            m_serialPort.BaudRate = from;
            if (!Probe())
                return null;
            logger.Debug("OPEN: adapter at " + from);
            if (from == TargetBaudrate)
                return Exec("STI", 200).Find(l => l.Contains("STN"));
            // 250 ms to switch and answer, without the 1 s per connect firmware that delays the banner by STBRT costs
            if (!Exec("STBRT250", 200).Contains("OK"))
                return null; // not an STN
            // OK (or ? when the rate can't be made) at the old rate, then the STI banner at the new one; a CR inside the
            // STBRT window keeps it there, else it reverts and the hunt retries
            m_serialPort.DiscardInBuffer();
            WriteToSerialWithTrace("STBR" + TargetBaudrate + "\r");
            if (ReadLine(200, l => l == "OK" || l == "?") == "?")
                return null;
            m_serialPort.BaudRate = TargetBaudrate;
            string banner = ReadLine(1500, l => l.Contains("STN"));
            if (banner == null)
                return null;
            WriteToSerialWithTrace("\r");
            return Probe() ? banner : null;
        }

        // ATE0, not a bare CR: an empty line repeats the last command. Twice: the first may land on a dirty line buffer and answer ?
        private bool Probe()
        {
            for (int i = 0; i < 2; i++)
            {
                if (Exec("ATE0", 100).Contains("OK"))
                    return true;
            }
            return false;
        }

        // open() only, before the reader thread owns the port
        private List<string> Exec(string cmd, int timeout)
        {
            m_serialPort.DiscardInBuffer();
            WriteToSerialWithTrace(cmd + "\r");
            var lines = new List<string>();
            ScanLines(timeout, (line, prompt) =>
            {
                if (line.Length > 0)
                    lines.Add(line);
                return prompt;
            });
            logger.Debug("SERRX: " + string.Join("|", lines));
            return lines;
        }

        private string ReadLine(int timeout, Func<string, bool> match)
        {
            return ScanLines(timeout, (line, prompt) => match(line));
        }

        // feeds each line (and each prompt, with the line before it) to done until it says so or the time is up
        private string ScanLines(int timeout, Func<string, bool, bool> done)
        {
            long deadline = Environment.TickCount64 + timeout;
            var line = new StringBuilder();
            var buffer = new byte[256];
            while (Environment.TickCount64 < deadline)
            {
                int read;
                try
                {
                    read = m_serialPort.BaseStream.Read(buffer, 0, buffer.Length);
                }
                catch (TimeoutException)
                {
                    continue;
                }
                for (int i = 0; i < read; i++)
                {
                    char c = (char)buffer[i];
                    if (c == '\r' || c == '\n' || c == '>')
                    {
                        string s = line.ToString();
                        line.Clear();
                        if (done(s, c == '>'))
                            return s;
                    }
                    else
                    {
                        line.Append(c);
                    }
                }
            }
            return null;
        }

        private void WriteToSerialWithTrace(string line)
        {
            m_serialPort.Write(line);
            logger.Debug("SERTX: " + line);
        }

        public override uint waitForMessage(uint a_canID, uint timeout, out CANMessage canMsg)
        {
            canMsg = new CANMessage();
            return 0;
        }

        public override float GetThermoValue()
        {
            return 0F;
        }

        public override float GetADCValue(uint channel)
        {
            return 0F;
        }
    }
}
