using System;
using System.IO;
using System.IO.Ports;
using System.Runtime.InteropServices;

namespace Lawicel
{
    /// <summary>
    /// The CANUSB's byte stream under CanusbVcp. Linux talks to the tty directly (PosixTty): the Unix
    /// SerialStream of System.IO.Ports hands every Read and Write to an IOLoop that polls with a 1 ms
    /// timeout and sleeps 1 ms whenever its queue is empty, which made a T7 flash read 2.5x slower than
    /// libj2534_canusb on the same tty. macOS keeps System.IO.Ports, it can't be tested here.
    /// </summary>
    internal abstract class VcpPort
    {
        internal static VcpPort Open(string port)
        {
            if (OperatingSystem.IsLinux()) return new PosixTty(port);
            return new SerialPortVcp(port);
        }

        /// <summary>A reply takes a millisecond or two to arrive (System.IO.Ports): CanusbVcp then pipelines commands instead of waiting for each reply.</summary>
        internal virtual bool SlowReplies { get { return false; } }

        /// <summary>Writes count bytes or throws.</summary>
        internal abstract void Write(byte[] buf, int count);

        /// <summary>Waits up to timeoutMs for input. Returns the bytes read, 0 if none came; throws when the port is gone.</summary>
        internal abstract int Read(byte[] buf, int timeoutMs);

        internal abstract void DiscardInBuffer();

        internal abstract void Close();
    }

    /// <summary>
    /// Raw tty through libc for Linux: non-blocking fd, the reader blocks in poll() and gets each USB
    /// packet as ftdi_sio delivers it, writes go straight to write(). Constants are the asm-generic
    /// ones (x86, x86-64, arm, arm64, riscv); termios is handled as an opaque buffer except c_cflag.
    /// </summary>
    internal sealed unsafe class PosixTty : VcpPort
    {
        const int O_RDWR = 2, O_NOCTTY = 0x100, O_NONBLOCK = 0x800, O_CLOEXEC = 0x80000;
        const uint TIOCEXCL = 0x540C, TIOCNXCL = 0x540D;
        const int TCSANOW = 0, TCIFLUSH = 0, TCIOFLUSH = 2;
        const short POLLIN = 0x1, POLLOUT = 0x4;
        const int EINTR = 4, EAGAIN = 11;
        // glibc/musl struct termios { tcflag_t c_iflag, c_oflag, c_cflag, c_lflag; cc_t c_line, c_cc[32]; speed_t c_ispeed, c_ospeed; }
        const int CflagOffset = 8;
        const uint CSIZE = 0x30, CS8 = 0x30, CREAD = 0x80, CLOCAL = 0x800, CRTSCTS = 0x80000000;
        const int WriteTimeoutMs = 1000;

        [StructLayout(LayoutKind.Sequential)]
        struct PollFd
        {
            public int fd;
            public short events;
            public short revents;
        }

        [DllImport("libc", SetLastError = true)] static extern int open(string path, int flags);
        [DllImport("libc", SetLastError = true)] static extern int close(int fd);
        [DllImport("libc", SetLastError = true)] static extern nint read(int fd, byte* buf, nint count);
        [DllImport("libc", SetLastError = true)] static extern nint write(int fd, byte* buf, nint count);
        [DllImport("libc", SetLastError = true)] static extern int poll(PollFd* fds, nuint nfds, int timeout);
        [DllImport("libc", SetLastError = true)] static extern int ioctl(int fd, nuint request, nint arg);
        [DllImport("libc", SetLastError = true)] static extern int tcgetattr(int fd, byte[] termios);
        [DllImport("libc", SetLastError = true)] static extern int tcsetattr(int fd, int optionalActions, byte[] termios);
        [DllImport("libc")] static extern void cfmakeraw(byte[] termios);
        [DllImport("libc", SetLastError = true)] static extern int tcflush(int fd, int queueSelector);

        int m_fd;

        internal PosixTty(string port)
        {
            m_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
            if (m_fd < 0) throw Error("open " + port);
            try
            {
                // exclusive like SerialPort: a second opener would steal half of the replies
                if (ioctl(m_fd, TIOCEXCL, 0) != 0) throw Error("TIOCEXCL");
                var t = new byte[256]; // glibc's struct termios is 60 bytes
                if (tcgetattr(m_fd, t) != 0) throw Error("tcgetattr");
                cfmakeraw(t);
                uint cflag = BitConverter.ToUInt32(t, CflagOffset);
                if ((cflag & CSIZE) == CS8) // cfmakeraw sets CS8, so c_cflag is where we expect it
                {
                    // CLOCAL: ftdi_sio hangs the tty up when DCD drops otherwise, the FT245 has no modem lines
                    BitConverter.GetBytes((cflag | CLOCAL | CREAD) & ~CRTSCTS).CopyTo(t, CflagOffset);
                }
                if (tcsetattr(m_fd, TCSANOW, t) != 0) throw Error("tcsetattr");
                tcflush(m_fd, TCIOFLUSH);
            }
            catch
            {
                close(m_fd);
                m_fd = -1;
                throw;
            }
        }

        internal override void Write(byte[] buf, int count)
        {
            long deadline = Environment.TickCount64 + WriteTimeoutMs;
            int done = 0;
            fixed (byte* b = buf)
            {
                while (done < count)
                {
                    nint n = write(m_fd, b + done, count - done);
                    if (n > 0)
                    {
                        done += (int)n;
                        continue;
                    }
                    int e = n < 0 ? Marshal.GetLastPInvokeError() : EAGAIN;
                    if (e == EINTR) continue;
                    if (e != EAGAIN) throw Error("write", e);
                    int left = (int)(deadline - Environment.TickCount64);
                    if (left <= 0) throw new TimeoutException("canusb: tty write timed out");
                    var p = new PollFd { fd = m_fd, events = POLLOUT };
                    poll(&p, 1, left); // an error shows up on the next write
                }
            }
        }

        internal override int Read(byte[] buf, int timeoutMs)
        {
            var p = new PollFd { fd = m_fd, events = POLLIN };
            int r = poll(&p, 1, timeoutMs);
            if (r < 0)
            {
                int e = Marshal.GetLastPInvokeError();
                if (e == EINTR) return 0;
                throw Error("poll", e);
            }
            if (r == 0) return 0;
            // POLLHUP/POLLERR without POLLIN: unplugged; with POLLIN the data is read first
            if ((p.revents & POLLIN) == 0) throw new IOException("canusb: tty gone (poll revents 0x" + p.revents.ToString("X") + ")");
            nint n;
            fixed (byte* b = buf) n = read(m_fd, b, buf.Length);
            if (n > 0) return (int)n;
            if (n == 0) throw new IOException("canusb: tty hung up");
            int err = Marshal.GetLastPInvokeError();
            if (err == EAGAIN || err == EINTR) return 0;
            throw Error("read", err);
        }

        internal override void DiscardInBuffer()
        {
            tcflush(m_fd, TCIFLUSH);
        }

        internal override void Close()
        {
            int fd = m_fd;
            m_fd = -1;
            if (fd < 0) return;
            // the flag lives as long as the tty_struct, which a pty master (or another fd) keeps past our close
            ioctl(fd, TIOCNXCL, 0);
            close(fd);
        }

        static IOException Error(string what)
        {
            return Error(what, Marshal.GetLastPInvokeError());
        }

        static IOException Error(string what, int errno)
        {
            return new IOException("canusb: " + what + ": " + Marshal.GetPInvokeErrorMessage(errno), errno);
        }
    }

    /// <summary>System.IO.Ports, everywhere but Linux.</summary>
    internal sealed class SerialPortVcp : VcpPort
    {
        readonly SerialPort m_port;

        internal SerialPortVcp(string port)
        {
            // The CANUSB is an FT245 FIFO, the baud rate is not used for anything
            m_port = new SerialPort(port, 3000000, Parity.None, 8, StopBits.One) { ReadTimeout = 50, WriteTimeout = 1000 };
            m_port.Open();
        }

        internal override bool SlowReplies { get { return true; } }

        internal override void Write(byte[] buf, int count)
        {
            m_port.Write(buf, 0, count);
        }

        internal override int Read(byte[] buf, int timeoutMs)
        {
            if (m_port.ReadTimeout != timeoutMs) m_port.ReadTimeout = timeoutMs;
            try
            {
                return m_port.Read(buf, 0, buf.Length);
            }
            catch (TimeoutException)
            {
                return 0;
            }
        }

        internal override void DiscardInBuffer()
        {
            m_port.DiscardInBuffer();
        }

        internal override void Close()
        {
            m_port.Close();
        }
    }
}
