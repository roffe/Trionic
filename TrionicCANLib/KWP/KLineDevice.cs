using System;
using System.Diagnostics;
using System.Threading;
using NLog;
using TrionicCANLib.CAN;

namespace TrionicCANLib.KWP
{
    /// <summary>
    /// KWP2000 (ISO 14230) on the K-line to a Trionic 7 behind DICE, the OG 9-3 / 9-5 body module that relays
    /// K-line requests to the ECU on CAN: fast init, 10400 baud, ECU 0x11, tester 0xF1. The transports only move
    /// frames: KKLDevice (a KKL cable) and J2534KLineDevice (a J2534 device's ISO14230 channel).
    /// ForcedComport names the serial port or the J2534 device.
    /// </summary>
    public abstract class KLineDevice : IKWPDevice
    {
        protected static readonly Logger logger = LogManager.GetCurrentClassLogger();

        protected const int Baud = 10400;
        protected const int ReadDeadlineMs = 2000; // a 248 byte reply at DICE's 6 ms per byte takes 1.5 s
        private const byte Ecu = 0x11, Dice = 0x41, Tester = 0xF1;
        // ISO 14230 P3max is 5 s: a session left idle longer (the Edit parameters dialog) is gone
        private const int SessionIdleMs = 4000;
        private const int MaxPending = 30;

        private bool m_session;
        private readonly Stopwatch m_lastReply = new Stopwatch();

        /// <summary>Wake-up pattern, then StartCommunication (a_frame) without waiting P3. The reply's payload, or null.</summary>
        protected abstract byte[] fastInit(byte[] a_frame);

        /// <summary>One frame, header and payload; the transport adds the checksum and waits out P3.</summary>
        protected abstract bool send(byte[] a_frame);

        /// <summary>The next frame's payload (from the response SID on) within ReadDeadlineMs, or null.</summary>
        protected abstract byte[] receive();

        /// <summary>fast: P3min 5 ms and back to back bytes, after the ECU accepted P3min 0 and P4min 0; else the ISO defaults.</summary>
        protected abstract void setTiming(bool a_fast);

        public override bool EnableLog { get; set; }
        public override int ForcedBaudrate { get; set; }
        public override string ForcedComport { get; set; } = string.Empty;
        public override void setCANDevice(ICANDevice a_canDevice) { }

        public override bool startSession()
        {
            // T7Flasher starts a session in the one openDevice started: another fast init would only reset it
            if (m_session && m_lastReply.ElapsedMilliseconds < SessionIdleMs)
            {
                return true;
            }
            m_session = false;
            for (int attempt = 0; attempt < 3 && !m_session; attempt++)
            {
                if (attempt > 0)
                {
                    Thread.Sleep(1000);
                }
                setTiming(false);
                byte[] reply = guarded(() => fastInit(frame(Ecu, new byte[] { 0x81 })));
                m_session = reply != null && reply.Length > 0 && reply[0] == 0xC1;
                logger.Debug("K-line fast init " + (m_session ? "answered" : "not answered"));
            }
            if (!m_session)
            {
                return false;
            }
            m_lastReply.Restart();

            // at the default P3, before the timing below, as saabctl does it: sent after 83 03 at P3 5 ms, DICE never
            // took its 2 ms per byte (bench 2026-11-05, KKL cable: a read at 6 ms per byte, 56 min)
            fastDice();

            // the T7 allows P2min, P3min and P4min of 0 (83 00): P2max 50 ms, P3min 0, P3max 5 s. Best effort
            byte[] timing = request(Ecu, new byte[] { 0x83, 0x03, 0x00, 0x02, 0x00, 0x14, 0x00 });
            bool fast = timing != null && timing[0] == 0xC3;
            setTiming(fast);
            logger.Debug("K-line timing: " + (fast ? "P3min 5 ms" : "module defaults"));
            return true;
        }

        public override bool stopSession()
        {
            if (!isOpen())
            {
                return false;
            }
            byte[] reply = request(Ecu, new byte[] { 0x82 });
            m_session = false;
            return reply != null && reply[0] == 0xC2;
        }

        public override RequestResult sendRequest(KWPRequest a_request, out KWPReply r_reply)
        {
            r_reply = new KWPReply();
            if (m_session && m_lastReply.ElapsedMilliseconds >= SessionIdleMs)
            {
                logger.Debug("K-line session idle for " + m_lastReply.ElapsedMilliseconds + " ms, starting it again");
                m_session = false;
                if (!startSession())
                {
                    return RequestResult.ErrorSending;
                }
            }
            if (!m_session)
            {
                return RequestResult.ErrorSending;
            }
            byte[] req = a_request.getData();
            byte[] payload = request(Ecu, req.AsSpan(1, req[0]).ToArray());
            if (payload == null)
            {
                return RequestResult.Timeout;
            }
            if (payload[0] == 0xC2)
            {
                m_session = false;
            }
            byte[] reply = new byte[payload.Length + 1];
            reply[0] = (byte)payload.Length;
            payload.CopyTo(reply, 1);
            r_reply = new KWPReply(reply, a_request.getNrOfPID());
            return RequestResult.NoError;
        }

        /// <summary>
        /// One request to a node on the K-line; the reply payload after any responsePending (7F xx 78), or null.
        /// </summary>
        private byte[] request(byte a_target, byte[] a_payload)
        {
            return guarded(() =>
            {
                if (!send(frame(a_target, a_payload)))
                {
                    return null;
                }
                for (int pending = 0; ; pending++)
                {
                    byte[] reply = receive();
                    if (reply == null || reply.Length == 0)
                    {
                        return null;
                    }
                    m_lastReply.Restart();
                    if (reply.Length >= 3 && reply[0] == 0x7F && reply[2] == 0x78 && pending < MaxPending)
                    {
                        continue;
                    }
                    return reply;
                }
            });
        }

        // an unplugged cable throws from the port
        private static byte[] guarded(Func<byte[]> a_io)
        {
            try
            {
                return a_io();
            }
            catch (Exception e)
            {
                logger.Debug(e, "K-line");
                return null;
            }
        }

        /// <summary>
        /// The length rides in the format byte up to 7 bytes, a length byte follows above that (TWICE reads only 3 bits of
        /// the format byte, so the same rule as saabctl's).
        /// </summary>
        private static byte[] frame(byte a_target, byte[] a_payload)
        {
            int header = a_payload.Length <= 7 ? 3 : 4;
            byte[] f = new byte[header + a_payload.Length];
            f[0] = (byte)(a_payload.Length <= 7 ? 0x80 | a_payload.Length : 0x80);
            f[1] = a_target;
            f[2] = Tester;
            if (header == 4)
            {
                f[3] = (byte)a_payload.Length;
            }
            a_payload.CopyTo(f, header);
            return f;
        }

        /// <summary>The payload of a received frame, cut at the length its header declares. Null if it is shorter.</summary>
        protected static byte[] payload(byte[] a_frame)
        {
            if (a_frame.Length < 4)
            {
                return null;
            }
            int length = a_frame[0] & 0x3F;
            int offset = 3;
            if (length == 0)
            {
                length = a_frame[3];
                offset = 4;
            }
            return a_frame.Length < offset + length ? null : a_frame.AsSpan(offset, length).ToArray();
        }

        /// <summary>
        /// DICE sends a K-line byte every 6 ms, also the ECU replies it relays; bit 3 of its RAM $5F makes that 2 ms (a
        /// 512 KiB read takes 23 min instead of 57). Every StartCommunication clears it, a power cycle too. Inside the ECU's
        /// session DICE only answers once a plain 81 gave it a session slot of its own; a fast init would reset the ECU's.
        /// </summary>
        private void fastDice()
        {
            byte[] reply = request(Dice, new byte[] { 0x81 });
            if (reply == null || reply[0] != 0xC1)
            {
                logger.Debug("DICE didn't answer, the K-line stays at 6 ms per byte");
                return;
            }
            reply = request(Dice, new byte[] { 0x23, 0x00, 0x00, 0x5F, 0x01 });
            if (reply == null || reply.Length < 2 || reply[0] != 0x63)
            {
                logger.Debug("DICE $5F unreadable, the K-line stays at 6 ms per byte");
                return;
            }
            if ((reply[1] & 0x08) != 0)
            {
                return;
            }
            reply = request(Dice, new byte[] { 0x3D, 0x00, 0x00, 0x5F, 0x01, (byte)(reply[1] | 0x08) });
            logger.Debug(reply != null && reply[0] == 0x7D ? "DICE K-line at 2 ms per byte" : "DICE refused the fast K-line");
        }
    }
}
