using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Microsoft.Win32.SafeHandles;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.API;
using TrionicCANLib.CAN;

namespace TrionicCANLibTest
{
    // CANScantoolDevice against a fake STN on a pseudo terminal (Linux / macOS): the STBR handshake, STPX with header and
    // count, and the reply window an interrupt ends (the OBDLink SX answers STOPPED and drops the rest of the line).
    [TestClass]
    public class ScantoolTest
    {
        [TestMethod]
        public void ParsesReplyLines()
        {
            var m = new CANMessage();
            Assert.IsTrue(CANScantoolDevice.TryParseFrame("7E81082610000D200D2", m));
            Assert.AreEqual(0x7E8u, m.getID());
            Assert.AreEqual((byte)8, m.getLength());
            Assert.AreEqual(0xD200D20000618210UL, m.getData());
            Assert.IsTrue(CANScantoolDevice.TryParseFrame("7E80176", m));
            Assert.AreEqual((byte)2, m.getLength());
            Assert.AreEqual(0x7601UL, m.getData());
            foreach (var status in new[] { "OK", "?", "STOPPED", "NO DATA", "CAN ERROR", "BUFFER FULL", "STN1130 v5.10.1", "7E8123" })
                Assert.IsFalse(CANScantoolDevice.TryParseFrame(status, m), status);
        }

        [TestMethod]
        public void TalksToAnStn()
        {
            if (OperatingSystem.IsWindows())
                Assert.Inconclusive("needs a pseudo terminal");
            using var stn = new FakeStn();
            var device = new CANScantoolDevice { TrionicECU = ECU.TRIONIC8, AcceptOnlyMessageIds = new List<uint> { 0x7E0, 0x7E8 } };
            device.SetSelectedAdapter(stn.Port);
            var frames = new Frames();
            device.addListener(frames);

            Assert.AreEqual(OpenResult.OK, device.open());
            CollectionAssert.IsSubsetOf(new[] { "STBRT250", "STBR2000000", "STP33", "ATCAF0" }, stn.Commands.ToArray());

            // a T8 read: 19 frames where the count ends the window
            var read = new CANMessage(0x7E0, 0, 7) { elmExpectedResponses = 19 };
            read.setData(0x003F0300802106UL);
            Assert.IsTrue(device.sendMessage(read));
            Assert.AreEqual(19, frames.Take(19));
            Assert.AreEqual("STPXh:7E0,d:06218000033F00,r:19", stn.Commands.Last());

            // Legion's skip tag: one frame where 19 were asked for. The interrupt's STOPPED prompt frees the next frame at
            // once; waiting for an answer after STOPPED cost two 300 ms timeouts
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < 5; i++)
            {
                var skip = new CANMessage(0x7E0, 0, 7) { elmExpectedResponses = 19 };
                skip.setData(0xEE3F0300802106UL);
                Assert.IsTrue(device.sendMessage(skip));
                Assert.AreEqual(1, frames.Take(1));
                device.RequestDeviceReady();
            }
            Assert.IsTrue(device.sendMessage(read));
            Assert.AreEqual(19, frames.Take(19));
            Assert.IsTrue(sw.ElapsedMilliseconds < 250, sw.ElapsedMilliseconds + " ms for five skip tags");
            Assert.AreEqual(5, stn.Commands.Count(c => c.StartsWith("####")));

            // no reply expected: the prompt comes at once, nothing to interrupt
            var cf = new CANMessage(0x7E0, 0, 8) { elmExpectedResponses = 0 };
            cf.setData(0x0706050403020121UL);
            Assert.IsTrue(device.sendMessage(cf));
            device.RequestDeviceReady();
            Assert.IsTrue(SpinWait.SpinUntil(() => stn.Commands.LastOrDefault() == "STPXh:7E0,d:2101020304050607,r:0", 1000), stn.Commands.LastOrDefault());

            device.close();
            Assert.IsFalse(device.isOpen());
            Assert.IsTrue(SpinWait.SpinUntil(() => stn.Commands.LastOrDefault() == "ATZ", 1000));
        }

        class Frames : ICANListener
        {
            readonly SemaphoreSlim received = new SemaphoreSlim(0);
            public override bool messagePending() { return received.CurrentCount > 0; }
            public override void dumpQueue() { }
            public override void FlushQueue() { }
            public override void handleMessage(CANMessage a_canMessage) { received.Release(); }

            public int Take(int count)
            {
                int got = 0;
                while (got < count && received.Wait(1000))
                    got++;
                return got;
            }
        }

        // Answers at once; a reply window whose count isn't reached listens on for 25 ms, as the OBDLink SX's adaptive timeout
        sealed class FakeStn : IDisposable
        {
            const int O_RDWR = 2, O_NOCTTY_LINUX = 0x100, O_NOCTTY_MAC = 0x20000;
            [DllImport("libc", SetLastError = true)] static extern int posix_openpt(int flags);
            [DllImport("libc", SetLastError = true)] static extern int grantpt(int fd);
            [DllImport("libc", SetLastError = true)] static extern int unlockpt(int fd);
            [DllImport("libc")] static extern IntPtr ptsname(int fd);
            [DllImport("libc", SetLastError = true)] static extern int open(string path, int flags);
            [DllImport("libc")] static extern int close(int fd);
            [DllImport("libc")] static extern int tcgetattr(int fd, byte[] termios);
            [DllImport("libc")] static extern void cfmakeraw(byte[] termios);
            [DllImport("libc")] static extern int tcsetattr(int fd, int actions, byte[] termios);

            public readonly string Port;
            public readonly ConcurrentQueue<string> Commands = new ConcurrentQueue<string>();
            readonly FileStream master;
            readonly int slave; // held open: a master with no slave reads EIO, and it carries the raw line settings
            readonly BlockingCollection<string> lines = new BlockingCollection<string>();
            readonly Thread reader, worker;
            bool awaitCr;

            public FakeStn()
            {
                int fd = posix_openpt(O_RDWR | (OperatingSystem.IsMacOS() ? O_NOCTTY_MAC : O_NOCTTY_LINUX));
                if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0)
                    throw new IOException("no pseudo terminal");
                Port = Marshal.PtrToStringAnsi(ptsname(fd));
                slave = open(Port, O_RDWR | (OperatingSystem.IsMacOS() ? O_NOCTTY_MAC : O_NOCTTY_LINUX));
                var termios = new byte[256]; // opaque: only libc reads it
                if (slave < 0 || tcgetattr(slave, termios) != 0)
                    throw new IOException("no pseudo terminal slave");
                cfmakeraw(termios);
                tcsetattr(slave, 0, termios);
                master = new FileStream(new SafeFileHandle((IntPtr)fd, true), FileAccess.ReadWrite, 1);
                reader = new Thread(Read) { IsBackground = true };
                worker = new Thread(Work) { IsBackground = true };
                reader.Start();
                worker.Start();
            }

            void Read()
            {
                var line = new StringBuilder();
                var buffer = new byte[256];
                try
                {
                    int read;
                    while ((read = master.Read(buffer, 0, buffer.Length)) > 0)
                    {
                        for (int i = 0; i < read; i++)
                        {
                            if (buffer[i] == '\r')
                            {
                                lines.Add(line.ToString());
                                line.Clear();
                            }
                            else
                            {
                                line.Append((char)buffer[i]);
                            }
                        }
                    }
                }
                catch (Exception) { } // disposed
                lines.CompleteAdding();
            }

            void Write(string s)
            {
                var b = Encoding.ASCII.GetBytes(s);
                master.Write(b, 0, b.Length);
                master.Flush();
            }

            void Work()
            {
                foreach (var cmd in lines.GetConsumingEnumerable())
                {
                    Commands.Enqueue(cmd);
                    if (awaitCr && cmd == "")
                    {
                        awaitCr = false;
                        Write("OK\r>");
                    }
                    else if (cmd == "STI")
                        Write("STN1130 v5.10.1\r>");
                    else if (cmd.StartsWith("STBRT"))
                        Write("OK\r>");
                    else if (cmd.StartsWith("STBR"))
                    {
                        // OK at the old rate, the banner at the new one, then a CR must confirm it
                        Write("OK\r");
                        Thread.Sleep(75);
                        Write("STN1130 v5.10.1\r");
                        awaitCr = true;
                    }
                    else if (cmd.StartsWith("STPX"))
                        Stpx(cmd.Substring(4).Split(',').ToDictionary(p => p.Split(':')[0], p => p.Split(':')[1]));
                    else if (cmd.StartsWith("AT") || cmd.StartsWith("ST"))
                        Write("OK\r>");
                    else
                        Write("?\r>");
                }
            }

            void Stpx(Dictionary<string, string> p)
            {
                int? r = p.TryGetValue("r", out var n) ? int.Parse(n) : null;
                if (r == 0)
                {
                    Write("\r>");
                    return;
                }
                var reply = p["d"].EndsWith("EE")
                    ? new List<string> { "7E81006611400005D80" } // Legion's skip tag
                    : Enumerable.Range(0, r ?? 1).Select(i => "7E8" + (0x21 + i).ToString("X2") + "11111111111111").ToList();
                Write(string.Concat(reply.Select(l => l + "\r")));
                if (r != null && reply.Count >= r)
                    Write("\r>");
                else if (lines.TryTake(out var interrupting, 25))
                {
                    Commands.Enqueue(interrupting); // dropped by the STN
                    Write("STOPPED\r\r>");
                }
                else
                    Write("\r>");
            }

            public void Dispose()
            {
                master.Dispose();
                close(slave);
            }
        }
    }
}
