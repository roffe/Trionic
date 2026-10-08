using System;
using System.Buffers.Binary;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using NLog;
using TrionicCANLib;

namespace Lawicel
{
    /// <summary>
    /// The Lawicel CANUSB driven through its FTDI virtual COM port with the ASCII command set
    /// (CANUSB manual 1.0D), for Linux/macOS where canusbdrv does not exist. Ported from gocan
    /// adapters/canusb. Handles and adapter names behave like the DLL's: the adapter name is the
    /// FTDI serial number (e.g. LWSXKNIO), a null/empty name opens the first adapter found.
    /// </summary>
    internal sealed class CanusbVcp
    {
        const byte CR = 0x0D, BELL = 0x07;
        // Manual §1.5: the USB command parser takes "one or two" commands at a time, and a full CAN
        // transmit FIFO (8 frames) answers BELL and drops the frame. With one command in flight a
        // BELLed frame is resent before anything else goes out, so frames are never lost or reordered.
        // Over System.IO.Ports (macOS) each reply costs 1-2 ms, so there two commands are pipelined
        // instead and every reply hands a credit back (ponytail: a BELLed frame is lost there).
        const int PipelinedCredits = 2;
        const int ReplyTimeoutMs = 500;
        // how long a frame is resent while the FIFO stays full (foreign traffic, bus off) before the write fails
        const int TxFullRetryMs = 100;
        // z only means queued, so transmit is also metered to the bit rate (worst case stuffing) with
        // this many frames ahead of the wire: 8, the FIFO depth, keeps the bus busy through a 1 ms
        // sleep without filling the FIFO while the bus is ours. Pipelined, where a BELL loses the frame: 4.
        const int TxBacklog = 8, PipelinedTxBacklog = 4;

        static readonly Logger logger = LogManager.GetCurrentClassLogger();
        static readonly ConcurrentDictionary<uint, CanusbVcp> s_open = new ConcurrentDictionary<uint, CanusbVcp>();
        static int s_lastHandle;
        static List<KeyValuePair<string, string>> s_adapters = new List<KeyValuePair<string, string>>();
        static int s_nextAdapter;

        VcpPort m_port;
        bool m_pipelined;
        Thread m_readThread;
        volatile bool m_endThread;
        readonly object m_writeLock = new object();
        readonly AutoResetEvent m_rxReady = new AutoResetEvent(false); // set for every frame queued in Received
        readonly AutoResetEvent m_replied = new AutoResetEvent(false);
        readonly AutoResetEvent m_statusReplied = new AutoResetEvent(false);
        volatile bool m_bell; // the last reply was BELL
        volatile string m_reply;
        volatile int m_status;
        string m_version = string.Empty;
        double m_bitTime; // seconds
        long m_txFree;    // Stopwatch timestamp when everything written so far is on the wire
        readonly byte[] m_line = new byte[64];
        int m_lineLen;

        internal readonly ConcurrentQueue<CANUSB.CANMsg> Received = new ConcurrentQueue<CANUSB.CANMsg>();
        internal readonly SemaphoreSlim Credits = new SemaphoreSlim(PipelinedCredits, PipelinedCredits);

        internal string LastReply { get { return m_reply; } }
        internal bool LastWasBell { get { return m_bell; } }
        internal int LastStatus { get { return m_status; } }

        internal static uint Open(string szID, string szBitrate, uint acceptance_code, uint acceptance_mask, uint flags)
        {
            double bitrate;
            string rateCmd = BitrateCommand(szBitrate, out bitrate);
            if (rateCmd == null)
            {
                logger.Debug("canusb: unsupported bitrate " + szBitrate);
                return 0;
            }
            string port = null;
            foreach (var a in FindAdapters())
            {
                if (string.IsNullOrEmpty(szID) || a.Key == szID)
                {
                    port = a.Value;
                    break;
                }
            }
            if (port == null)
            {
                logger.Debug("canusb: adapter '" + szID + "' not found");
                return 0;
            }
            return OpenTty(port, rateCmd, bitrate, acceptance_code, acceptance_mask);
        }

        /// <summary>Open on a known tty (the tests use a pty), rateCmd/bitrate from BitrateCommand.</summary>
        internal static uint OpenTty(string port, string rateCmd, double bitrate, uint acceptance_code, uint acceptance_mask)
        {
            var dev = new CanusbVcp();
            dev.m_bitTime = 1.0 / bitrate;
            try
            {
                string version;
                if (!dev.OpenPort(port, out version))
                {
                    dev.ClosePort();
                    return 0;
                }
                dev.m_version = version;
                // Z0: our host side timestamps, the frame parser ignores a device timestamp anyway.
                // Old firmware may BELL on it, so its reply is not checked.
                dev.Command("Z0");
                foreach (var c in new[] { rateCmd, Acceptance('M', acceptance_code), Acceptance('m', acceptance_mask), "O" })
                {
                    if (dev.Command(c) == null)
                    {
                        logger.Debug("canusb " + port + ": " + c + " failed");
                        dev.ClosePort();
                        return 0;
                    }
                }
            }
            catch (Exception e)
            {
                logger.Debug("canusb " + port + ": open failed: " + e.Message);
                dev.ClosePort();
                return 0;
            }
            uint h = (uint)Interlocked.Increment(ref s_lastHandle);
            s_open[h] = dev;
            logger.Debug("canusb " + port + ": open " + rateCmd + " handle " + h);
            return h;
        }

        /// <summary>
        /// Opens the port and runs the identification part of the setup (manual §1.5): CRs to flush a
        /// half sent command, C in case a crashed session left the channel open, then V and N.
        /// Sends nothing that opens the channel or touches the bus.
        /// </summary>
        internal bool OpenPort(string port, out string version)
        {
            version = null;
            // before Open: the port is opened exclusive (TIOCEXCL), a second open for the ioctl gets EBUSY afterwards
            SerialLowLatency.TryEnable(port);
            m_port = VcpPort.Open(port);
            m_pipelined = m_port.SlowReplies;
            foreach (var c in new[] { "\r", "\r", "\r", "C\r" })
            {
                WriteAscii(c);
                Thread.Sleep(15);
            }
            Thread.Sleep(50);
            m_port.DiscardInBuffer();
            m_readThread = new Thread(ReadLoop) { Name = "CanusbVcp.m_readThread", IsBackground = true };
            m_readThread.Start();

            string v = Command("V");
            if (v == null || v.Length < 2 || v[0] != 'V')
            {
                logger.Debug("canusb " + port + ": no version reply, not a CANUSB?");
                return false;
            }
            string n = Command("N");
            version = v + " " + n;
            logger.Debug("canusb " + port + ": " + version);
            return true;
        }

        internal void ClosePort()
        {
            m_endThread = true;
            if (m_readThread != null) m_readThread.Join(1000);
            try
            {
                if (m_port != null) m_port.Close();
            }
            catch (Exception e)
            {
                logger.Debug("canusb: close failed: " + e.Message);
            }
        }

        internal static int Close(uint h)
        {
            CanusbVcp dev;
            if (!s_open.TryRemove(h, out dev)) return CANUSB.ERROR_CANUSB_NOT_OPEN;
            dev.m_rxReady.Set(); // a receive loop waiting on this handle rechecks its stop flag now
            dev.FlushWait();
            lock (dev.m_writeLock)
            {
                try
                {
                    dev.WriteAscii("C\r");
                }
                catch (Exception e)
                {
                    logger.Debug("canusb: close channel failed: " + e.Message);
                }
            }
            Thread.Sleep(50);
            dev.ClosePort();
            return CANUSB.ERROR_CANUSB_OK;
        }

        // ponytail: an unknown/closed handle reads as "no message" too, CANUSBDevice's wait loops only
        // count ERROR_CANUSB_NO_MESSAGE towards their timeout and would spin forever on anything else
        internal static int Read(uint h, out CANUSB.CANMsg msg)
        {
            CanusbVcp dev;
            if (s_open.TryGetValue(h, out dev) && dev.Received.TryDequeue(out msg)) return CANUSB.ERROR_CANUSB_OK;
            msg = new CANUSB.CANMsg();
            return CANUSB.ERROR_CANUSB_NO_MESSAGE;
        }

        /// <summary>
        /// Blocks until the reader thread queues a frame, the handle is closed or timeoutMs passes; may
        /// return early for a frame that was already read. An unknown handle sleeps 1 ms, so a receive
        /// loop cannot spin.
        /// </summary>
        internal static void WaitReceive(uint h, int timeoutMs)
        {
            CanusbVcp dev;
            if (s_open.TryGetValue(h, out dev)) dev.m_rxReady.WaitOne(timeoutMs);
            else Thread.Sleep(1);
        }

        internal static int Write(uint h, ref CANUSB.CANMsg msg)
        {
            CanusbVcp dev;
            if (!s_open.TryGetValue(h, out dev)) return CANUSB.ERROR_CANUSB_NOT_OPEN;
            string frame = EncodeFrame(msg);
            if (frame == null) return CANUSB.ERROR_CANUSB_INVALID_PARAM;
            return dev.Send(frame, FrameBits(msg.len, (msg.flags & CANUSB.CANMSG_EXTENDED) != 0));
        }

        internal static int Status(uint h)
        {
            CanusbVcp dev;
            if (!s_open.TryGetValue(h, out dev)) return CANUSB.ERROR_CANUSB_NOT_OPEN;
            dev.m_statusReplied.Reset();
            int rc = dev.Send("F\r", 0);
            if (rc != CANUSB.ERROR_CANUSB_OK) return rc;
            return dev.m_statusReplied.WaitOne(ReplyTimeoutMs) ? dev.m_status : CANUSB.ERROR_CANUSB_TIMEOUT;
        }

        internal static int VersionInfo(uint h, StringBuilder verinfo)
        {
            CanusbVcp dev;
            if (!s_open.TryGetValue(h, out dev)) return CANUSB.ERROR_CANUSB_NOT_OPEN;
            verinfo.Clear().Append(dev.m_version);
            return CANUSB.ERROR_CANUSB_OK;
        }

        internal static int Flush(uint h, byte flushflags)
        {
            CanusbVcp dev;
            if (!s_open.TryGetValue(h, out dev)) return CANUSB.ERROR_CANUSB_NOT_OPEN;
            if ((flushflags & CANUSB.FLUSH_EMPTY_INQUEUE) != 0) dev.Received.Clear();
            // no host side transmit queue to empty, frames go straight to the port
            if ((flushflags & CANUSB.FLUSH_DONTWAIT) == 0) dev.FlushWait();
            return CANUSB.ERROR_CANUSB_OK;
        }

        internal static int GetFirstAdapter(StringBuilder szAdapter, int size)
        {
            s_adapters = FindAdapters();
            s_nextAdapter = 0;
            if (s_adapters.Count == 0) return 0;
            return GetNextAdapter(szAdapter, size) == CANUSB.ERROR_CANUSB_OK ? s_adapters.Count : 0;
        }

        internal static int GetNextAdapter(StringBuilder szAdapter, int size)
        {
            var list = s_adapters;
            if (s_nextAdapter >= list.Count || size < 1) return CANUSB.ERROR_CANUSB_NO_DEVICE;
            string name = list[s_nextAdapter++].Key;
            szAdapter.Clear().Append(name.Length < size ? name : name.Substring(0, size - 1)); // C string, size includes the NUL
            return CANUSB.ERROR_CANUSB_OK;
        }

        /// <summary>CANUSB adapters as (FTDI serial number, serial port) in port order.</summary>
        internal static List<KeyValuePair<string, string>> FindAdapters()
        {
            var found = new List<KeyValuePair<string, string>>();
            try
            {
                if (OperatingSystem.IsLinux() && Directory.Exists("/sys/bus/usb-serial/devices"))
                {
                    var ttys = Directory.GetFileSystemEntries("/sys/bus/usb-serial/devices");
                    Array.Sort(ttys, StringComparer.Ordinal);
                    foreach (var tty in ttys)
                    {
                        // /sys/bus/usb-serial/devices/ttyUSBn -> .../<usb device>/<interface>/ttyUSBn
                        var target = new DirectoryInfo(tty).ResolveLinkTarget(true);
                        if (target == null) continue;
                        string usb = Path.GetDirectoryName(Path.GetDirectoryName(target.FullName));
                        if (ReadSysfs(usb, "product") != "CANUSB") continue;
                        string serial = ReadSysfs(usb, "serial");
                        if (!string.IsNullOrEmpty(serial)) found.Add(new KeyValuePair<string, string>(serial, "/dev/" + Path.GetFileName(tty)));
                    }
                }
                else if (OperatingSystem.IsMacOS())
                {
                    // ponytail: no IOKit lookup of the USB product string, Lawicel's FTDI serial numbers start with LW
                    const string prefix = "cu.usbserial-";
                    var ports = Directory.GetFiles("/dev", prefix + "LW*");
                    Array.Sort(ports, StringComparer.Ordinal);
                    foreach (var p in ports)
                    {
                        found.Add(new KeyValuePair<string, string>(Path.GetFileName(p).Substring(prefix.Length), p));
                    }
                }
            }
            catch (Exception e)
            {
                logger.Debug("canusb: adapter scan failed: " + e.Message);
            }
            return found;
        }

        static string ReadSysfs(string dir, string name)
        {
            string path = Path.Combine(dir, name);
            return File.Exists(path) ? File.ReadAllText(path).Trim() : null;
        }

        /// <summary>Sends a setup command and waits for its reply. Returns the reply line ("" for a bare CR), null on BELL or timeout.</summary>
        internal string Command(string cmd)
        {
            m_replied.Reset();
            m_reply = null;
            lock (m_writeLock)
            {
                WriteAscii(cmd + "\r");
            }
            return m_replied.WaitOne(ReplyTimeoutMs) ? m_reply : null;
        }

        void WriteAscii(string s)
        {
            byte[] b = Encoding.ASCII.GetBytes(s);
            m_port.Write(b, b.Length);
        }

        /// <summary>
        /// Writes a command and waits for its reply. Frames (bits > 0) are metered to the bit rate and
        /// resent while the device BELLs (transmit FIFO full) for up to TxFullRetryMs. Pipelined: only
        /// waits for a credit.
        /// </summary>
        int Send(string cmd, int bits)
        {
            byte[] b = Encoding.ASCII.GetBytes(cmd);
            lock (m_writeLock)
            {
                Pace(bits);
                long start = Stopwatch.GetTimestamp();
                while (true)
                {
                    if (m_pipelined && !Credits.Wait(ReplyTimeoutMs))
                    {
                        // ponytail: a lost reply is recovered by timing out and carrying on
                        logger.Debug("canusb: no reply to the last commands, sending anyway");
                    }
                    m_replied.Reset();
                    try
                    {
                        m_port.Write(b, b.Length);
                    }
                    catch (Exception e)
                    {
                        logger.Debug("canusb: write failed: " + e.Message);
                        return CANUSB.ERROR_CANUSB_COMMAND_SUBSYSTEM;
                    }
                    if (m_pipelined) return CANUSB.ERROR_CANUSB_OK;
                    if (!m_replied.WaitOne(ReplyTimeoutMs))
                    {
                        // ponytail: a lost reply is recovered by timing out and carrying on
                        logger.Debug("canusb: no reply to " + cmd.TrimEnd('\r'));
                        return CANUSB.ERROR_CANUSB_OK;
                    }
                    if (!m_bell) return CANUSB.ERROR_CANUSB_OK;
                    if (bits == 0) return CANUSB.ERROR_CANUSB_COMMAND_SUBSYSTEM; // F on a closed channel
                    if (Stopwatch.GetTimestamp() - start > TxFullRetryMs * Stopwatch.Frequency / 1000)
                    {
                        logger.Debug("canusb: transmit FIFO still full after " + TxFullRetryMs + " ms, frame dropped");
                        return CANUSB.ERROR_CANUSB_TX_FIFO_FULL;
                    }
                    // resent at once: the round trip (0.1 ms) is shorter than a frame on the wire (0.25 ms at
                    // 500k), so a few resends BELL again until the oldest queued frame has gone out
                }
            }
        }

        // Waits (bounded) until the outstanding commands are answered and the frames written are on the wire.
        void FlushWait()
        {
            lock (m_writeLock)
            {
                var sw = Stopwatch.StartNew();
                while (Credits.CurrentCount < PipelinedCredits && sw.ElapsedMilliseconds < 200) Thread.Sleep(1);
                long left = (m_txFree - Stopwatch.GetTimestamp()) * 1000 / Stopwatch.Frequency;
                if (left > 0) Thread.Sleep((int)Math.Min(left + 1, 200));
            }
        }

        // ponytail: models the bus as ours alone, sustained foreign traffic drains the FIFO slower than this
        void Pace(int bits)
        {
            if (bits == 0) return;
            long now = Stopwatch.GetTimestamp();
            long d = (long)(bits * m_bitTime * Stopwatch.Frequency);
            if (m_txFree < now) m_txFree = now;
            long wait = m_txFree - now - (m_pipelined ? PipelinedTxBacklog : TxBacklog) * d;
            m_txFree += d;
            if (wait > 0) Thread.Sleep((int)((wait * 1000 + Stopwatch.Frequency - 1) / Stopwatch.Frequency));
        }

        // Worst case bits on the wire: 47 (+20 extended) fixed plus data, plus a stuff bit per 4 bits from SOF to CRC.
        internal static int FrameBits(int len, bool extended)
        {
            int fixedBits = 47 + 8 * len, stuffed = 34 + 8 * len;
            if (extended)
            {
                fixedBits += 20;
                stuffed += 20;
            }
            return fixedBits + (stuffed - 1) / 4;
        }

        void ReadLoop()
        {
            var buf = new byte[4096];
            while (!m_endThread)
            {
                int n;
                try
                {
                    n = m_port.Read(buf, 50); // 50 ms: how long ClosePort waits for this thread at most
                }
                catch (Exception e)
                {
                    if (!m_endThread) logger.Debug("canusb: read failed: " + e.Message);
                    return;
                }
                Feed(buf, n);
            }
        }

        /// <summary>Reply parser: accumulates CR terminated lines across reads, BELL is a reply on its own.</summary>
        internal void Feed(byte[] buf, int count)
        {
            for (int i = 0; i < count; i++)
            {
                byte b = buf[i];
                if (b == BELL)
                {
                    logger.Debug("canusb: command error (BELL)");
                    Reply(null);
                }
                else if (b == CR)
                {
                    Dispatch(Encoding.ASCII.GetString(m_line, 0, m_lineLen));
                    m_lineLen = 0;
                }
                else if (m_lineLen < m_line.Length)
                {
                    m_line[m_lineLen++] = b;
                }
            }
        }

        void Dispatch(string line)
        {
            if (line.Length == 0)
            {
                Reply(line);
                return;
            }
            switch (line[0])
            {
                case 't':
                case 'T':
                case 'r':
                case 'R':
                    CANUSB.CANMsg msg;
                    if (DecodeFrame(line, out msg))
                    {
                        msg.timestamp = (uint)Environment.TickCount; // ponytail: host ms clock, not the device's 0-59999 one
                        Received.Enqueue(msg);
                        m_rxReady.Set();
                    }
                    else
                    {
                        logger.Debug("canusb: bad frame " + line);
                    }
                    return;
                case 'F':
                    int flags;
                    if (int.TryParse(line.AsSpan(1), NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out flags))
                    {
                        m_status = flags;
                        // bit 6, arbitration lost, is normal bus contention
                        if ((flags & ~0x40) != 0) logger.Debug("canusb: status flags " + line.Substring(1));
                    }
                    Reply(line);
                    m_statusReplied.Set();
                    return;
                default: // z/Z transmit acks, V, N, ...
                    Reply(line);
                    return;
            }
        }

        // The answer to the command in flight (pipelined: the oldest one); null = BELL
        void Reply(string line)
        {
            if (Credits.CurrentCount < PipelinedCredits) Credits.Release(); // only this thread releases, so no overshoot
            m_bell = line == null;
            m_reply = line;
            m_replied.Set();
        }

        /// <summary>Decodes "tiiildd.." / "Tiiiiiiiildd.." (r/R: remote, no data); trailing bytes such as a timestamp are ignored.</summary>
        internal static bool DecodeFrame(string line, out CANUSB.CANMsg msg)
        {
            msg = new CANUSB.CANMsg();
            bool ext = line[0] == 'T' || line[0] == 'R';
            bool rtr = line[0] == 'r' || line[0] == 'R';
            int idLen = ext ? 8 : 3;
            if (line.Length < idLen + 2) return false;
            if (!uint.TryParse(line.AsSpan(1, idLen), NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out msg.id)) return false;
            int dlc = line[idLen + 1] - '0';
            if (dlc < 0 || dlc > 8) return false;
            msg.len = (byte)dlc;
            msg.flags = (byte)((ext ? CANUSB.CANMSG_EXTENDED : 0) | (rtr ? CANUSB.CANMSG_RTR : 0));
            if (rtr) return true;
            if (line.Length < idLen + 2 + dlc * 2) return false;
            for (int i = 0; i < dlc; i++)
            {
                byte b;
                if (!byte.TryParse(line.AsSpan(idLen + 2 + i * 2, 2), NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out b)) return false;
                msg.data |= (ulong)b << (8 * i);
            }
            return true;
        }

        /// <summary>Encodes the transmit command, CR included, or null when len is over 8.</summary>
        internal static string EncodeFrame(CANUSB.CANMsg msg)
        {
            if (msg.len > 8) return null;
            bool ext = (msg.flags & CANUSB.CANMSG_EXTENDED) != 0;
            bool rtr = (msg.flags & CANUSB.CANMSG_RTR) != 0;
            var sb = new StringBuilder(27);
            sb.Append(ext ? (rtr ? 'R' : 'T') : (rtr ? 'r' : 't'));
            sb.Append(ext ? (msg.id & 0x1FFFFFFF).ToString("X8") : (msg.id & 0x7FF).ToString("X3"));
            sb.Append((char)('0' + msg.len));
            if (!rtr)
            {
                for (int i = 0; i < msg.len; i++) sb.Append(((byte)(msg.data >> (8 * i))).ToString("X2"));
            }
            return sb.Append('\r').ToString();
        }

        /// <summary>
        /// Maps a canusbdrv bitrate string to the Sn / sxxyy command: "10".."1000" kbit/s or a
        /// "btr0:btr1" pair ("0xcb:0x9a" or "3:28"). Null when unsupported.
        /// </summary>
        internal static string BitrateCommand(string rate, out double bitsPerSecond)
        {
            bitsPerSecond = 0;
            if (string.IsNullOrEmpty(rate)) return null;
            int colon = rate.IndexOf(':');
            if (colon < 0)
            {
                int kbps;
                if (!int.TryParse(rate.Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out kbps)) return null;
                int n = Array.IndexOf(new[] { 10, 20, 50, 100, 125, 250, 500, 800, 1000 }, kbps);
                if (n < 0) return null;
                bitsPerSecond = kbps * 1000.0;
                return "S" + n;
            }
            uint btr0, btr1;
            if (!ParseByte(rate.Substring(0, colon), out btr0) || !ParseByte(rate.Substring(colon + 1), out btr1)) return null;
            // SJA1000 at 16 MHz: tq = 2 * (BRP + 1) / 16 MHz, one bit = 1 + (TSEG1 + 1) + (TSEG2 + 1) tq
            double tq = 2.0 * ((btr0 & 0x3F) + 1) / 16e6;
            int tqPerBit = 3 + (int)(btr1 & 0x0F) + (int)((btr1 >> 4) & 0x07);
            bitsPerSecond = 1.0 / (tq * tqPerBit);
            return "s" + btr0.ToString("X2") + btr1.ToString("X2");
        }

        static bool ParseByte(string s, out uint v)
        {
            s = s.Trim();
            bool ok = s.StartsWith("0x", StringComparison.OrdinalIgnoreCase)
                ? uint.TryParse(s.AsSpan(2), NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out v)
                : uint.TryParse(s, NumberStyles.None, CultureInfo.InvariantCulture, out v);
            return ok && v <= 0xFF;
        }

        /// <summary>
        /// M/m command for a canusbdrv acceptance DWORD. The DLL byte swaps it before printing, so
        /// the low byte is ACR0/AMR0 (gocan adapters/canusb/dll_windows.go, dllAcceptance).
        /// </summary>
        internal static string Acceptance(char cmd, uint value)
        {
            return cmd + BinaryPrimitives.ReverseEndianness(value).ToString("X8");
        }
    }
}
