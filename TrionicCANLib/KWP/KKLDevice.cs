using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading;
using TrionicCANLib.WMI;

namespace TrionicCANLib.KWP
{
    /// <summary>
    /// A KKL (VAG-COM style) USB serial cable. The host does the timing: the fast init is a 25 ms break, and the
    /// K-line is half duplex, so every byte sent comes back as echo.
    /// </summary>
    // ponytail: System.IO.Ports costs about 1 ms per Read and Write on Linux, a few ms per request against DICE's 2 to 6 ms
    // per byte. PosixTty (CAN/Lawicel/VcpPort.cs) if a flash shows it.
    public class KKLDevice : KLineDevice
    {
        private readonly SerialPort m_port = new SerialPort();
        private int m_p3 = 55;

        public static string[] GetAdapterNames()
        {
            return COMPortInfo.GetPortNames();
        }

        public override bool open()
        {
            close();
            // before Open: SerialPort opens the tty exclusive, the ioctl would get EBUSY after it
            SerialLowLatency.TryEnable(ForcedComport);
            try
            {
                m_port.PortName = ForcedComport;
                m_port.BaudRate = Baud;
                m_port.DataBits = 8;
                m_port.Parity = Parity.None;
                m_port.StopBits = StopBits.One;
                m_port.Handshake = Handshake.None;
                // raised, as a plain tty open leaves them (saabctl's cable works that way); SerialPort defaults to off
                m_port.DtrEnable = true;
                m_port.RtsEnable = true;
                m_port.Open();
                return true;
            }
            catch (Exception e)
            {
                logger.Debug(e, "KKL cable " + ForcedComport);
                return false;
            }
        }

        public override bool close()
        {
            if (m_port.IsOpen)
            {
                m_port.Close();
            }
            return true;
        }

        public override bool isOpen()
        {
            return m_port.IsOpen;
        }

        protected override void setTiming(bool a_fast)
        {
            m_p3 = a_fast ? 5 : 55;
        }

        protected override byte[] fastInit(byte[] a_frame)
        {
            m_port.DiscardInBuffer();
            Thread.Sleep(300);  // idle bus before the wake-up
            m_port.BreakState = true;
            Thread.Sleep(25);   // TiniL, it must really be 25 ms
            m_port.BreakState = false;
            Thread.Sleep(25);   // TWuP
            m_port.DiscardInBuffer(); // some chips put a stray byte in the RX stream at the break
            return write(a_frame) ? receive() : null;
        }

        protected override bool send(byte[] a_frame)
        {
            Thread.Sleep(m_p3);
            m_port.DiscardInBuffer();
            return write(a_frame);
        }

        private bool write(byte[] a_frame)
        {
            byte[] f = new byte[a_frame.Length + 1];
            a_frame.CopyTo(f, 0);
            f[a_frame.Length] = checksum(a_frame, a_frame.Length);
            m_port.Write(f, 0, f.Length);
            return read(f.Length).Length == f.Length; // the echo
        }

        protected override byte[] receive()
        {
            byte[] header = read(3);
            if (header.Length < 3)
            {
                return null;
            }
            int length = header[0] & 0x3F;
            byte[] lengthByte = Array.Empty<byte>();
            if (length == 0)
            {
                lengthByte = read(1);
                if (lengthByte.Length < 1)
                {
                    return null;
                }
                length = lengthByte[0];
            }
            byte[] rest = read(length + 1);
            if (rest.Length < length + 1)
            {
                return null;
            }
            byte sum = (byte)(checksum(header, 3) + checksum(lengthByte, lengthByte.Length) + checksum(rest, length));
            if (sum != rest[length])
            {
                logger.Debug("K-line checksum error");
                return null;
            }
            return rest.AsSpan(0, length).ToArray();
        }

        private byte[] read(int a_count)
        {
            byte[] buffer = new byte[a_count];
            int got = 0;
            Stopwatch sw = Stopwatch.StartNew();
            while (got < a_count)
            {
                int left = ReadDeadlineMs - (int)sw.ElapsedMilliseconds;
                if (left <= 0)
                {
                    break;
                }
                m_port.ReadTimeout = left;
                try
                {
                    got += m_port.Read(buffer, got, a_count - got);
                }
                catch (TimeoutException)
                {
                    break;
                }
            }
            return got == a_count ? buffer : buffer.AsSpan(0, got).ToArray();
        }

        private static byte checksum(byte[] a_data, int a_length)
        {
            byte sum = 0;
            for (int i = 0; i < a_length; i++)
            {
                sum += a_data[i];
            }
            return sum;
        }
    }
}
