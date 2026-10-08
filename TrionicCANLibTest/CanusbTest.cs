using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Lawicel;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace TrionicCANLibTest
{
    [TestClass]
    public class CanusbTest
    {
        [TestMethod]
        public void BitrateStringsMapToLawicelCommands()
        {
            double bps;
            Assert.AreEqual("s8B2F", CanusbVcp.BitrateCommand("0x8B:0x2F", out bps)); // CANUSBDevice 33.3k
            Assert.AreEqual(33333, Math.Round(bps));
            Assert.AreEqual("sCB9A", CanusbVcp.BitrateCommand("0xcb:0x9a", out bps)); // 47.6k
            Assert.AreEqual(47619, Math.Round(bps));
            Assert.AreEqual("s4037", CanusbVcp.BitrateCommand("0x40:0x37", out bps)); // 615k
            Assert.AreEqual(615385, Math.Round(bps));
            Assert.AreEqual("s031C", CanusbVcp.BitrateCommand("3:28", out bps));
            Assert.AreEqual(125000, Math.Round(bps));
            Assert.AreEqual("S6", CanusbVcp.BitrateCommand(CANUSB.CAN_BAUD_500K, out bps));
            Assert.AreEqual(500000, bps);
            Assert.AreEqual("S0", CanusbVcp.BitrateCommand(CANUSB.CAN_BAUD_10K, out bps));
            Assert.AreEqual("S8", CanusbVcp.BitrateCommand(CANUSB.CAN_BAUD_1M, out bps));
            Assert.IsNull(CanusbVcp.BitrateCommand("33", out bps));
            Assert.IsNull(CanusbVcp.BitrateCommand("0x100:0x1C", out bps));
            Assert.IsNull(CanusbVcp.BitrateCommand(null, out bps));
        }

        [TestMethod]
        public void AcceptanceIsByteSwappedLikeTheDll()
        {
            // CANUSBDevice.CalcAcceptanceFilters for id 0x7E8 gives 0x00FD00FD -> ACR0..3 = FD 00 FD 00
            Assert.AreEqual("MFD00FD00", CanusbVcp.Acceptance('M', 0x00FD00FD));
            Assert.AreEqual("m12345678", CanusbVcp.Acceptance('m', 0x78563412));
            Assert.AreEqual("M00000000", CanusbVcp.Acceptance('M', CANUSB.CANUSB_ACCEPTANCE_CODE_ALL));
            Assert.AreEqual("mFFFFFFFF", CanusbVcp.Acceptance('m', CANUSB.CANUSB_ACCEPTANCE_MASK_ALL));
        }

        [TestMethod]
        public void EncodeFrames()
        {
            var msg = new CANUSB.CANMsg { id = 0x220, len = 8, data = 0x000040021100813f };
            Assert.AreEqual("t22083F81001102400000\r", CanusbVcp.EncodeFrame(msg));
            msg = new CANUSB.CANMsg { id = 0x7E0, len = 2, data = 0xFFFFFFFFFFFF3E01 };
            Assert.AreEqual("t7E02013E\r", CanusbVcp.EncodeFrame(msg));
            msg = new CANUSB.CANMsg { id = 0x18DAF110, len = 1, flags = CANUSB.CANMSG_EXTENDED, data = 0xAA };
            Assert.AreEqual("T18DAF1101AA\r", CanusbVcp.EncodeFrame(msg));
            msg = new CANUSB.CANMsg { id = 0x123, len = 4, flags = CANUSB.CANMSG_RTR, data = 0x1122 };
            Assert.AreEqual("r1234\r", CanusbVcp.EncodeFrame(msg));
            msg = new CANUSB.CANMsg { id = 0x123, len = 9 };
            Assert.IsNull(CanusbVcp.EncodeFrame(msg));
        }

        [TestMethod]
        public void ParserSurvivesSplitReadsAndInterleavedFrames()
        {
            var dev = new CanusbVcp();
            Assert.IsTrue(dev.Credits.Wait(0));
            Assert.IsTrue(dev.Credits.Wait(0)); // pipelined: two commands outstanding

            // frame split across reads, ack and a second frame in one read, a timestamped frame, garbage, BELL
            Feed(dev, "t7E88021");
            Assert.IsTrue(dev.Received.IsEmpty);
            Feed(dev, "0FF0000000000\rz\rT18DAF1101AA\r");
            Assert.AreEqual("z", dev.LastReply);
            Assert.IsFalse(dev.LastWasBell);
            Assert.AreEqual(1, dev.Credits.CurrentCount);
            Feed(dev, "t2580EA60\rtXYZ1\r\a");
            Assert.IsNull(dev.LastReply);
            Assert.IsTrue(dev.LastWasBell);
            Assert.AreEqual(2, dev.Credits.CurrentCount);
            Feed(dev, "z\rr1230\r"); // received frames are not replies; no credit outstanding, must not overshoot
            Assert.AreEqual("z", dev.LastReply);
            Assert.IsFalse(dev.LastWasBell);
            Assert.AreEqual(2, dev.Credits.CurrentCount);

            CANUSB.CANMsg m;
            Assert.IsTrue(dev.Received.TryDequeue(out m));
            Assert.AreEqual(0x7E8u, m.id);
            Assert.AreEqual(8, m.len);
            Assert.AreEqual(0, m.flags);
            Assert.AreEqual(0x0000000000FF1002ul, m.data);
            Assert.IsTrue(dev.Received.TryDequeue(out m));
            Assert.AreEqual(0x18DAF110u, m.id);
            Assert.AreEqual(CANUSB.CANMSG_EXTENDED, m.flags);
            Assert.AreEqual(0xAAul, m.data);
            Assert.IsTrue(dev.Received.TryDequeue(out m)); // trailing timestamp ignored
            Assert.AreEqual(0x258u, m.id);
            Assert.AreEqual(0, m.len);
            Assert.IsTrue(dev.Received.TryDequeue(out m)); // bad hex dropped, remote frame kept
            Assert.AreEqual(0x123u, m.id);
            Assert.AreEqual(CANUSB.CANMSG_RTR, m.flags);
            Assert.IsTrue(dev.Received.IsEmpty);

            Feed(dev, "V1011\r");
            Assert.AreEqual("V1011", dev.LastReply);
            Feed(dev, "\r");
            Assert.AreEqual("", dev.LastReply);
            Feed(dev, "F0");
            Feed(dev, "4\r");
            Assert.AreEqual(0x04, dev.LastStatus);
            Assert.AreEqual("F04", dev.LastReply);
        }

        [TestMethod]
        public void EncodeDecodeRoundTrip()
        {
            var msg = new CANUSB.CANMsg { id = 0x5E8, len = 7, data = 0xFF11223344556677 };
            string line = CanusbVcp.EncodeFrame(msg);
            CANUSB.CANMsg back;
            Assert.IsTrue(CanusbVcp.DecodeFrame(line.TrimEnd('\r'), out back));
            Assert.AreEqual(msg.id, back.id);
            Assert.AreEqual(msg.len, back.len);
            Assert.AreEqual(0x0011223344556677ul, back.data); // bytes past len are not sent
        }

        static void Feed(CanusbVcp dev, string s)
        {
            var b = Encoding.ASCII.GetBytes(s);
            dev.Feed(b, b.Length);
        }

        [TestMethod]
        public void PosixTtyDrivesAFakeCanusbOverAPty()
        {
            if (!OperatingSystem.IsLinux()) Assert.Inconclusive("PosixTty is the Linux backend");
            using (var fake = new FakeCanusb())
            {
                uint h = CanusbVcp.OpenTty(fake.SlavePath, "S6", 500000, 0x00FD00FD, 0x00000000);
                Assert.AreNotEqual(0u, h);
                try
                {
                    // the setup the DLL would do, in order (CRs and C flush the device first)
                    CollectionAssert.AreEqual(new[] { "", "", "", "C", "V", "N", "Z0", "S6", "MFD00FD00", "m00000000", "O" }, fake.Commands.ToArray());
                    var info = new StringBuilder();
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_OK, CanusbVcp.VersionInfo(h, info));
                    Assert.AreEqual("V1011 NFAKE", info.ToString());

                    // exclusive while open, like SerialPort: the J2534 .so or a second app gets EBUSY
                    Assert.ThrowsExactly<IOException>(() => new PosixTty(fake.SlavePath));

                    var msg = new CANUSB.CANMsg { id = 0x266, len = 8, data = 0x000000008B3FA140 };
                    for (int i = 0; i < 20; i++) Assert.AreEqual(CANUSB.ERROR_CANUSB_OK, CanusbVcp.Write(h, ref msg)); // > 2 credits: z acks hand them back
                    Assert.IsTrue(fake.WaitCommand("t26684"), "frame not seen by the device");

                    // a frame line split across two USB packets wakes the receive wait, no polling
                    CANUSB.CANMsg rx;
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_NO_MESSAGE, CanusbVcp.Read(h, out rx));
                    var sw = Stopwatch.StartNew();
                    var sender = new Thread(() =>
                    {
                        Thread.Sleep(50);
                        fake.Send("t2588C0BF02", "6CF0000000\r", 20);
                    });
                    sender.Start();
                    while (CanusbVcp.Read(h, out rx) != CANUSB.ERROR_CANUSB_OK && sw.ElapsedMilliseconds < 3000) CanusbVcp.WaitReceive(h, 1000);
                    sender.Join();
                    Assert.IsTrue(sw.ElapsedMilliseconds >= 60 && sw.ElapsedMilliseconds < 900, "receive wait did not wake on the frame: " + sw.ElapsedMilliseconds + " ms");
                    Assert.AreEqual(0x258u, rx.id);
                    Assert.AreEqual(8, rx.len);
                    Assert.AreEqual(0x000000F06C02BFC0ul, rx.data);
                }
                finally
                {
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_OK, CanusbVcp.Close(h));
                }
                var cmds = fake.Commands.ToArray();
                Assert.AreEqual("C", cmds[cmds.Length - 1], "channel not closed"); // written before Close's 50 ms settle
                Assert.AreEqual(CANUSB.ERROR_CANUSB_NOT_OPEN, CanusbVcp.Close(h));
                var sw2 = Stopwatch.StartNew();
                CanusbVcp.WaitReceive(h, 1000); // closed handle: no wait
                Assert.IsTrue(sw2.ElapsedMilliseconds < 500);
                new PosixTty(fake.SlavePath).Close(); // released: the J2534 .so can open it now
            }
        }

        [TestMethod, Timeout(10000)] // a resend loop that never gives up would hang the test run
        public void FullTransmitFifoResendsInOrderThenGivesUp()
        {
            if (!OperatingSystem.IsLinux()) Assert.Inconclusive("PosixTty is the Linux backend");
            using (var fake = new FakeCanusb())
            {
                uint h = CanusbVcp.OpenTty(fake.SlavePath, "S6", 500000, 0, 0xFFFFFFFF);
                Assert.AreNotEqual(0u, h);
                try
                {
                    int setup = fake.Commands.Count;
                    var a = new CANUSB.CANMsg { id = 0x240, len = 8, data = 0x1111111111111111 };
                    var b = new CANUSB.CANMsg { id = 0x240, len = 8, data = 0x2222222222222222 };
                    // BELL = the CAN transmit FIFO is full and dropped the frame: resent before b goes out
                    fake.BellFrames = 3;
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_OK, CanusbVcp.Write(h, ref a));
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_OK, CanusbVcp.Write(h, ref b));
                    var sent = fake.Commands.ToArray();
                    string ta = CanusbVcp.EncodeFrame(a).TrimEnd('\r'), tb = CanusbVcp.EncodeFrame(b).TrimEnd('\r');
                    CollectionAssert.AreEqual(new[] { ta, ta, ta, ta, tb }, sent[setup..]);

                    // a FIFO that never drains (bus off) fails the write instead of hanging
                    fake.BellFrames = int.MaxValue;
                    var sw = Stopwatch.StartNew();
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_TX_FIFO_FULL, CanusbVcp.Write(h, ref a));
                    Assert.IsTrue(sw.ElapsedMilliseconds >= 100 && sw.ElapsedMilliseconds < 1000, sw.ElapsedMilliseconds + " ms");
                    fake.BellFrames = 0;
                    Assert.AreEqual(CANUSB.ERROR_CANUSB_OK, CanusbVcp.Write(h, ref b));
                }
                finally
                {
                    CanusbVcp.Close(h);
                }
            }
        }

        [TestMethod]
        public void PosixTtyReadTimesOutThenReportsHangup()
        {
            if (!OperatingSystem.IsLinux()) Assert.Inconclusive("PosixTty is the Linux backend");
            var buf = new byte[64];
            PosixTty tty;
            using (var fake = new FakeCanusb(answer: false))
            {
                tty = new PosixTty(fake.SlavePath);
                var sw = Stopwatch.StartNew();
                Assert.AreEqual(0, tty.Read(buf, 30));
                Assert.IsTrue(sw.ElapsedMilliseconds >= 25, "poll did not wait");
                fake.Send("z\r", null, 0);
                Assert.AreEqual(2, tty.Read(buf, 1000));
                tty.Write(Encoding.ASCII.GetBytes("V\r"), 2);
            }
            // master gone = adapter unplugged: the reader thread must stop, not spin on zero byte reads
            Assert.ThrowsExactly<IOException>(() => tty.Read(buf, 1000));
            tty.Close();
        }

        /// <summary>A CANUSB on the master side of a pty: answers the setup commands, z for frames, F00 for F.</summary>
        sealed class FakeCanusb : IDisposable
        {
            const int O_RDWR = 2, O_NOCTTY = 0x100;
            [StructLayout(LayoutKind.Sequential)] struct PollFd { public int fd; public short events, revents; }
            [DllImport("libc", SetLastError = true)] static extern int posix_openpt(int flags);
            [DllImport("libc")] static extern int grantpt(int fd);
            [DllImport("libc")] static extern int unlockpt(int fd);
            [DllImport("libc")] static extern int ptsname_r(int fd, byte[] buf, nuint len);
            [DllImport("libc", SetLastError = true)] static extern nint read(int fd, byte[] buf, nint count);
            [DllImport("libc", SetLastError = true)] static extern nint write(int fd, byte[] buf, nint count);
            [DllImport("libc")] static extern int poll(ref PollFd fds, nuint nfds, int timeout);
            [DllImport("libc")] static extern int close(int fd);

            readonly int m_master;
            readonly bool m_answer;
            readonly Thread m_thread;
            volatile bool m_stop;
            readonly object m_writeLock = new object();
            internal readonly ConcurrentQueue<string> Commands = new ConcurrentQueue<string>();
            internal readonly string SlavePath;
            internal volatile int BellFrames; // answer this many t frames with BELL (transmit FIFO full)

            internal FakeCanusb(bool answer = true)
            {
                m_answer = answer;
                m_master = posix_openpt(O_RDWR | O_NOCTTY);
                Assert.IsTrue(m_master >= 0 && grantpt(m_master) == 0 && unlockpt(m_master) == 0, "no pty");
                var name = new byte[128];
                Assert.AreEqual(0, ptsname_r(m_master, name, (nuint)name.Length));
                SlavePath = Encoding.ASCII.GetString(name, 0, Array.IndexOf(name, (byte)0));
                m_thread = new Thread(Run) { IsBackground = true };
                m_thread.Start();
            }

            internal void Send(string first, string second, int gapMs)
            {
                Write(first);
                if (second == null) return;
                Thread.Sleep(gapMs);
                Write(second);
            }

            internal bool WaitCommand(string prefix)
            {
                var sw = Stopwatch.StartNew();
                while (sw.ElapsedMilliseconds < 2000)
                {
                    foreach (var c in Commands) if (c.StartsWith(prefix, StringComparison.Ordinal)) return true;
                    Thread.Sleep(5);
                }
                return false;
            }

            void Write(string s)
            {
                var b = Encoding.ASCII.GetBytes(s);
                lock (m_writeLock) write(m_master, b, b.Length);
            }

            void Run()
            {
                var buf = new byte[1024];
                var line = new StringBuilder();
                while (!m_stop)
                {
                    var p = new PollFd { fd = m_master, events = 1 };
                    if (poll(ref p, 1, 20) <= 0) continue;
                    nint n = read(m_master, buf, buf.Length);
                    if (n <= 0)
                    {
                        Thread.Sleep(1); // EIO while no slave is open
                        continue;
                    }
                    for (int i = 0; i < n; i++)
                    {
                        char c = (char)buf[i];
                        if (c != '\r')
                        {
                            line.Append(c);
                            continue;
                        }
                        string cmd = line.ToString();
                        line.Clear();
                        Commands.Enqueue(cmd);
                        if (!m_answer) continue;
                        if (cmd == "V") Write("V1011\r");
                        else if (cmd == "N") Write("NFAKE\r");
                        else if (cmd == "F") Write("F00\r");
                        else if (cmd.StartsWith("t", StringComparison.Ordinal) && BellFrames > 0)
                        {
                            BellFrames--;
                            Write("\a");
                        }
                        else if (cmd.StartsWith("t", StringComparison.Ordinal)) Write("z\r");
                        else if (cmd.StartsWith("T", StringComparison.Ordinal)) Write("Z\r");
                        else Write("\r");
                    }
                }
            }

            public void Dispose()
            {
                m_stop = true;
                m_thread.Join(1000);
                close(m_master);
            }
        }
    }
}
