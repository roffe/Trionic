using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using Combi;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.API;
using TrionicCANLib.CAN;
using TrionicCANLib.Flasher;
using TrionicCANLib.KWP;

namespace TrionicCANLibTest
{
    [TestClass]
    public class T7FlasherTest
    {
        // Fake T7 KWP device. By default it answers the way the bench ECU did right after a flash,
        // still in its EOL loop: session and security access work, 21 F0 is refused (7F 21 12).
        // requestDownload and transferData replies are scripted per address for the write tests.
        class FakeEcu : IKWPDevice
        {
            public readonly List<string> Requests = new List<string>();
            public readonly Dictionary<uint, Queue<string>> DownloadReplies = new Dictionary<uint, Queue<string>>();
            public readonly Dictionary<uint, Queue<string>> TransferReplies = new Dictionary<uint, Queue<string>>();
            public readonly List<string> Transfers = new List<string>(); // "address:data" of every 0x36
            public readonly byte[] Flash = new byte[0x80000];            // what the accepted 0x36 programmed
            public string ReadReply = "037F2112";                        // 21 F0, as logged on hardware
            public Action OnStopCommunication;                           // called when 0x82 arrives
            public Action<byte[]> OnRequest;                             // called first for every request
            public Func<byte[], bool> NoReply;                           // true: the request times out
            public bool Open = true;
            public int TransferBlocks;
            private uint m_writeAddress;
            private byte[] m_keptBlock;

            public FakeEcu()
            {
                Array.Fill(Flash, (byte)0xFF);
            }

            public override bool startSession() { return true; }
            public override bool EnableLog { get; set; }
            public override int ForcedBaudrate { get; set; }
            public override string ForcedComport { get; set; }
            public override void setCANDevice(ICANDevice a_canDevice) { }
            public override bool open() { return true; }
            public override bool close() { return true; }
            public override bool isOpen() { return Open; }

            public override RequestResult sendRequest(KWPRequest a_request, out KWPReply r_reply)
            {
                byte[] req = a_request.getData();
                OnRequest?.Invoke(req);
                if (NoReply != null && NoReply(req))
                {
                    Requests.Add(Convert.ToHexString(req));
                    r_reply = new KWPReply();
                    return RequestResult.Timeout;
                }
                string reply;
                switch (req[1])
                {
                    case 0x27: reply = req[2] == 0x05 ? "0467055A64" : "03670634"; break;
                    case 0x2C: reply = "026CF0"; break;
                    case 0x21: reply = ReadReply; break;
                    case 0x31: reply = "0271" + req[2].ToString("X2"); break;
                    case 0x3E: reply = "017E"; break;
                    case 0x34:
                        uint addr = (uint)(req[2] << 16 | req[3] << 8 | req[4]);
                        reply = DownloadReplies.TryGetValue(addr, out var q) && q.Count > 0 ? q.Dequeue() : "0174";
                        if (reply == "0174") m_writeAddress = addr;
                        break;
                    case 0x36:
                        TransferBlocks++;
                        Transfers.Add(m_writeAddress.ToString("X5") + ":" + Convert.ToHexString(req, 2, req.Length - 2));
                        reply = TransferReplies.TryGetValue(m_writeAddress, out var tq) && tq.Count > 0 ? tq.Dequeue() : "0176";
                        // like the firmware: a busy block is kept, and the next accepted 0x36 programs
                        // the kept copy, whatever that request carried
                        if (reply == "037F3621" && m_keptBlock == null)
                        {
                            m_keptBlock = req;
                        }
                        else if (reply == "0176")
                        {
                            byte[] block = m_keptBlock ?? req;
                            m_keptBlock = null;
                            Array.Copy(block, 2, Flash, m_writeAddress, block.Length - 2);
                            m_writeAddress += (uint)(block.Length - 2);
                        }
                        break;
                    case 0x82: OnStopCommunication?.Invoke(); reply = "01C2"; break;
                    default: reply = "037F" + req[1].ToString("X2") + "11"; break;
                }
                if (req[1] != 0x36) Requests.Add(Convert.ToHexString(req));
                r_reply = new KWPReply(Convert.FromHexString(reply), a_request.getNrOfPID());
                return RequestResult.NoError;
            }
        }

        static T7Flasher NewFlasher(FakeEcu ecu)
        {
            KWPHandler.setKWPDevice(ecu);
            T7Flasher.setKWPHandler(KWPHandler.getInstance());
            return new T7Flasher();
        }

        static IFlasher.FlashStatus WaitWhileBusy(T7Flasher flasher)
        {
            var sw = Stopwatch.StartNew();
            IFlasher.FlashStatus st;
            while (((st = flasher.getStatus()) == IFlasher.FlashStatus.DoinNuthin || st == IFlasher.FlashStatus.Eraseing ||
                st == IFlasher.FlashStatus.Writing || st == IFlasher.FlashStatus.Reading) && sw.ElapsedMilliseconds < 20000)
            {
                Thread.Sleep(10);
            }
            return st;
        }

        static string TempBin()
        {
            string file = Path.Combine(Path.GetTempPath(), "t7flashertest-" + Guid.NewGuid().ToString("N") + ".bin");
            File.WriteAllBytes(file, new byte[0x80000]);
            return file;
        }

        static string PatternBin()
        {
            // every 128-byte block differs, so a skipped, shifted or repeated block shows
            byte[] image = new byte[0x80000];
            for (int i = 0; i < image.Length; i++)
            {
                image[i] = (byte)(i / 128 + i);
            }
            string file = NewFileName();
            File.WriteAllBytes(file, image);
            return file;
        }

        static string NewFileName()
        {
            return Path.Combine(Path.GetTempPath(), "t7flashertest-" + Guid.NewGuid().ToString("N") + ".bin");
        }

        // Trionic7 as openDevice leaves it, with the given flasher, whose status texts go to onCanInfo (the GUI log)
        static Trionic7 NewTrionic7(IFlasher flasher)
        {
            Trionic7 t7 = new Trionic7();
            typeof(Trionic7).GetField("flash", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t7, flasher);
            flasher.onStatusChanged += (IFlasher.StatusChanged)Delegate.CreateDelegate(typeof(IFlasher.StatusChanged), t7,
                typeof(Trionic7).GetMethod("flash_onStatusChanged", BindingFlags.NonPublic | BindingFlags.Instance));
            return t7;
        }

        // ReadFlash as the GUI calls it; returns the FinishedDownloadingFlash event that
        // tmrReadProcessChecker raised and the ms it took, or null if the read never finished
        static (ITrionic.CanInfoEventArgs e, long ms) ReadUntilFinished(Trionic7 t7, string file)
        {
            ITrionic.CanInfoEventArgs finished = null;
            long ms = 0;
            var sw = Stopwatch.StartNew();
            var done = new ManualResetEventSlim(false);
            t7.onCanInfo += (s, e) =>
            {
                if (e.Type == ActivityType.FinishedDownloadingFlash && !done.IsSet)
                {
                    ms = sw.ElapsedMilliseconds;
                    finished = e;
                    done.Set();
                }
            };
            t7.ReadFlash(file);
            done.Wait(15000);
            return (finished, ms);
        }

        [TestMethod]
        public void ReadFlashEndsCleanlyOnNegativeReadReply()
        {
            var ecu = new FakeEcu();
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.readFlash(file);

                // before the fix the flasher thread threw ArgumentOutOfRangeException in FileStream.Write,
                // and then the failed read still ended as Completed
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, WaitWhileBusy(flasher));
                Thread.Sleep(200);
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, flasher.getStatus(), "the error must stay visible to the read timer");
                Assert.IsFalse(File.Exists(file), "a failed read must not leave a partial file behind");
                CollectionAssert.Contains(ecu.Requests, "0221F0");
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }
    
        [TestMethod]
        public void WriteFlashResendsBusyRequestDownload()
        {
            // hardware, combi-kwp: the 0x7FE00 requestDownload sent 0.4 ms after the last 0x76
            // came back 7F 34 21 and the PI area was never written
            var ecu = new FakeEcu();
            ecu.DownloadReplies[0x7FE00] = new Queue<string>(new[] { "037F3421" });
            string file = TempBin();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.Completed, WaitWhileBusy(flasher));
                Assert.AreEqual(2, ecu.Requests.FindAll(r => r == "083407FE0000000200").Count, "busy requestDownload must be resent");
                Assert.AreEqual(0x7B000 / 128 + 0x200 / 128, ecu.TransferBlocks);
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void WriteFlashReportsRefusedRequestDownloadAsWriteError()
        {
            // before the fix run() overwrote WriteError with Completed straight away, so
            // tmrWriteProcessChecker announced "Finished FLASH session"
            var ecu = new FakeEcu();
            ecu.DownloadReplies[0x7FE00] = new Queue<string>(new[] { "037F3442" });
            string file = TempBin();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, WaitWhileBusy(flasher));
                Thread.Sleep(200);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, flasher.getStatus(), "the error must stay visible to the write timer");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        // a full read whose content fails ChecksumT7.VerifyChecksum, as the GUI does it: returns the infos
        // the GUI log got, the read timer's finish event and the file afterwards
        static (List<string> infos, string finished, byte[] file, string md5) ReadImage(string a_blockHex)
        {
            var ecu = new FakeEcu { ReadReply = "4261F0" + a_blockHex };
            string file = NewFileName();
            string md5File = Path.ChangeExtension(file, ".md5");
            var flasher = NewFlasher(ecu);
            Trionic7 t7 = NewTrionic7(flasher);
            var infos = new List<string>();
            t7.onCanInfo += (s, e) => { lock (infos) infos.Add(e.Info); };
            try
            {
                var (e, ms) = ReadUntilFinished(t7, file);
                Assert.AreEqual(512 * 1024 / 64, ecu.Requests.FindAll(r => r == "0221F0").Count, "the whole flash was read");
                lock (infos)
                {
                    return (new List<string>(infos), e?.Info, File.Exists(file) ? File.ReadAllBytes(file) : null,
                        File.Exists(md5File) ? File.ReadAllText(md5File) : null);
                }
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
                if (File.Exists(md5File)) File.Delete(md5File);
            }
        }

        static void AssertKeptWithWarning((List<string> infos, string finished, byte[] file, string md5) a_read, byte a_fill)
        {
            Assert.AreEqual("Finished download of FLASH", a_read.finished, string.Join(" | ", a_read.infos));
            Assert.IsNotNull(a_read.file, "a complete read is kept even if its checksums don't verify");
            Assert.AreEqual(512 * 1024, a_read.file.Length);
            Assert.IsTrue(a_read.file.All(b => b == a_fill), "the file must be exactly what the ECU sent: no checksum fixed");
            Assert.AreEqual(Convert.ToHexString(System.Security.Cryptography.MD5.HashData(a_read.file)), a_read.md5?.ToUpperInvariant());
            List<string> warnings = a_read.infos.FindAll(i => i.StartsWith("Warning: checksum verification failed"));
            Assert.AreEqual(1, warnings.Count, string.Join(" | ", a_read.infos));
            StringAssert.Contains(warnings[0], "saved as read from the ECU");
            CollectionAssert.DoesNotContain(a_read.infos, "Failed to download FLASH content");
        }

        [TestMethod]
        public void ReadFlashKeepsCompleteReadWithBadChecksum()
        {
            // an ECU holding a hand-patched image must still be backed up: the read used to be deleted
            // and reported as failed (and before that, VerifyChecksum's unset delegate threw on the thread)
            var read = ReadImage(new string('0', 128));
            AssertKeptWithWarning(read, 0x00);
            Assert.IsTrue(read.infos.Exists(i => i.Contains("(ChecksumF")), string.Join(" | ", read.infos));
        }

        [TestMethod]
        public void ReadFlashKeepsCompleteReadWhoseChecksumCheckThrows()
        {
            // a footer VerifyChecksum can't parse throws; on the flasher thread that ended the process
            string probe = NewFileName();
            try
            {
                File.WriteAllBytes(probe, Enumerable.Repeat((byte)0x01, 512 * 1024).ToArray());
                Assert.Throws<Exception>(() => TrionicCANLib.Checksum.ChecksumT7.VerifyChecksum(probe, false, false, (l, f, r) => false));
            }
            finally
            {
                File.Delete(probe);
            }
            AssertKeptWithWarning(ReadImage(string.Concat(Enumerable.Repeat("01", 64))), 0x01);
        }

        [TestMethod]
        public void ReadMemoryEndsCleanlyOnShortReply()
        {
            // same 7F 21 12 as ReadFlashEndsCleanlyOnNegativeReadReply, on the SRAM read
            var ecu = new FakeEcu();
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.readMemory(file, 0xF00000, 0x200);
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, WaitWhileBusy(flasher));
                Assert.IsFalse(File.Exists(file), "a failed read must not leave a partial file behind");
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void FailedReadEndsSessionBeforeReportingError()
        {
            // the read timer closes the device once it sees ReadError, and its stopFlasher (DoinNuthin)
            // let run() announce "Flasing procedure completed": the 0x82 must go out before the error shows
            foreach (string readReply in new[] { "037F2112" })
            {
                foreach (bool memory in new[] { false, true })
                {
                    var ecu = new FakeEcu { ReadReply = readReply };
                    string file = NewFileName();
                    var flasher = NewFlasher(ecu);
                    var atStop = new List<IFlasher.FlashStatus>();
                    ecu.OnStopCommunication = () => atStop.Add(flasher.getStatus());
                    try
                    {
                        if (memory)
                            flasher.readMemory(file, 0xF00000, 0x200);
                        else
                            flasher.readFlash(file);
                        Assert.AreEqual(IFlasher.FlashStatus.ReadError, WaitWhileBusy(flasher));
                        CollectionAssert.AreEqual(new[] { IFlasher.FlashStatus.Reading }, atStop, readReply + " memory=" + memory);
                    }
                    finally
                    {
                        flasher.cleanup();
                        if (File.Exists(file)) File.Delete(file);
                    }
                }
            }
        }

        [TestMethod]
        public void WriteFlashResendsBusyTransferDataUnchanged()
        {
            // a 7F 36 21 block was skipped (continue): the ECU programs the block it kept when the next
            // one arrives and drops that one, so 128 bytes stayed erased
            var ecu = new FakeEcu();
            ecu.TransferReplies[0x1000] = new Queue<string>(new[] { "037F3621", "037F3621" });
            ecu.TransferReplies[0x7FF80] = new Queue<string>(new[] { "037F3621" });
            string file = PatternBin();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.Completed, WaitWhileBusy(flasher));
                List<string> sent = ecu.Transfers.FindAll(t => t.StartsWith("01000:"));
                Assert.AreEqual(3, sent.Count, "a busy block is resent until the ECU takes it");
                Assert.AreEqual(1, sent.Distinct().Count(), "the resend must be the identical block");
                Assert.AreEqual(2, ecu.Transfers.FindAll(t => t.StartsWith("7FF80:")).Count);
                Assert.AreEqual(2, ecu.Requests.FindAll(r => r.StartsWith("0834")).Count, "no requestDownload instead of the resend");
                byte[] image = File.ReadAllBytes(file);
                Assert.IsTrue(image.AsSpan(0, 0x7B000).SequenceEqual(ecu.Flash.AsSpan(0, 0x7B000)), "0x0-0x7B000 programmed as in the file");
                Assert.IsTrue(image.AsSpan(0x7FE00).SequenceEqual(ecu.Flash.AsSpan(0x7FE00)), "0x7FE00-0x80000 programmed as in the file");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void WriteFlashFailsWhenTransferDataStaysBusy()
        {
            var ecu = new FakeEcu();
            ecu.TransferReplies[0x1000] = new Queue<string>(Enumerable.Repeat("037F3621", 1000));
            string file = PatternBin();
            int delay = KWPHandler.m_busyRetryDelay;
            KWPHandler.m_busyRetryDelay = 1;
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, WaitWhileBusy(flasher));
                Assert.AreEqual(601, ecu.Transfers.FindAll(t => t.StartsWith("01000:")).Count, "the block and 600 resends");
                Assert.IsTrue(ecu.Transfers.Last().StartsWith("01000:"), "nothing may be written past the failed block");
                Thread.Sleep(200);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, flasher.getStatus(), "the error must stay visible to the write timer");
            }
            finally
            {
                KWPHandler.m_busyRetryDelay = delay;
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void WriteFlashStopsAtRefusedTransferData()
        {
            // before the fix the write went on past a refused block and left a 128-byte gap
            var ecu = new FakeEcu();
            ecu.TransferReplies[0x1000] = new Queue<string>(new[] { "037F3622" });
            string file = PatternBin();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, WaitWhileBusy(flasher));
                Assert.AreEqual(0x1000 / 128 + 1, ecu.Transfers.Count);
                Assert.IsTrue(ecu.Transfers.Last().StartsWith("01000:"), "nothing may be written past the failed block");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void WriteFlashStopsAtRefusedTransferDataInPiArea()
        {
            // same for the 0x7FE00-0x80000 loop, which also used to continue
            var ecu = new FakeEcu();
            ecu.TransferReplies[0x7FE80] = new Queue<string>(new[] { "037F3622" });
            string file = PatternBin();
            var flasher = NewFlasher(ecu);
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, WaitWhileBusy(flasher));
                Assert.AreEqual(0x7B000 / 128 + 2, ecu.Transfers.Count, "nothing may be written past the failed block");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void ReadTimerReportsFailedKwpRead()
        {
            // the read above ended as "Finished download of FLASH" with no file written
            var ecu = new FakeEcu();
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            Trionic7 t7 = NewTrionic7(flasher);
            try
            {
                var (e, ms) = ReadUntilFinished(t7, file);
                Assert.IsNotNull(e, "the read timer must finish");
                Assert.AreEqual("Failed to download FLASH content", e.Info);
                Assert.IsFalse(File.Exists(file));
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void ReadTimerReportsFailedCombiSessionStart()
        {
            // on-device read with the ECU ignoring startCommunication: getStatus() said ReadError and the
            // read timer, which only finished on Completed, sat at 0% for good
            var combi = new caCombiAdapter(); // never opened: commands go nowhere and wait for their reply
            var flasher = new T7CombiFlasher(combi);
            Trionic7 t7 = NewTrionic7(flasher);
            MethodInfo receive = typeof(caCombiAdapter).GetMethod("process_packet", BindingFlags.NonPublic | BindingFlags.Instance);
            const int nakAfter = 1500;
            var stop = new ManualResetEventSlim(false);
            var adapter = new Thread(() =>
            {
                // the adapter fails the ECU connect (0x89) only after the read timer's first tick
                if (stop.Wait(nakAfter)) return;
                do
                {
                    receive.Invoke(combi, new object[] { (byte)0x89, new byte[0], caCombiAdapter.term_nak });
                }
                while (!stop.Wait(20));
            });
            adapter.Start();
            string file = NewFileName();
            try
            {
                var (e, ms) = ReadUntilFinished(t7, file);
                Assert.IsNotNull(e, "the read timer must finish");
                Assert.AreEqual("Failed to download FLASH content", e.Info);
                Assert.IsTrue(ms >= nakAfter, "still connecting is no failure, but the read finished after " + ms + " ms");
            }
            finally
            {
                stop.Set();
                adapter.Join();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void CombiStartFailureAfterSucceededOperationIsAnError()
        {
            // the adapter keeps OperationSucceeded() from its previous operation, so without the stored
            // start error a failed connect on the same adapter read as that operation's Completed
            var combi = new caCombiAdapter();
            typeof(caCombiAdapter).GetField("operation_succeeded", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(combi, true);
            var flasher = new T7CombiFlasher(combi);
            MethodInfo receive = typeof(caCombiAdapter).GetMethod("process_packet", BindingFlags.NonPublic | BindingFlags.Instance);
            var stop = new ManualResetEventSlim(false);
            var adapter = new Thread(() =>
            {
                while (!stop.Wait(50))
                {
                    receive.Invoke(combi, new object[] { (byte)0x89, new byte[0], caCombiAdapter.term_nak });
                }
            });
            adapter.Start();
            string file = NewFileName();
            try
            {
                flasher.readFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, flasher.getStatus());
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, flasher.getStatus());
            }
            finally
            {
                stop.Set();
                adapter.Join();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        // The GUI calls ReadFlash/WriteFlash on its UI thread, and the Combi on-device flasher connects on that
        // thread; the GUI's progress handler (Dispatcher.Invoke) blocks every tick until the call returns, so the
        // ticks of a 2.5 s failed connect resume together. Returns the finish events raised.
        static List<string> FinishEventsAfterFailedCombiConnect(bool a_write)
        {
            var combi = new caCombiAdapter();
            var flasher = new T7CombiFlasher(combi);
            Trionic7 t7 = NewTrionic7(flasher);
            MethodInfo receive = typeof(caCombiAdapter).GetMethod("process_packet", BindingFlags.NonPublic | BindingFlags.Instance);
            var callerFree = new ManualResetEventSlim(false);
            t7.onReadProgress += (s, e) => callerFree.Wait(10000);
            t7.onWriteProgress += (s, e) => callerFree.Wait(10000);
            var finished = new List<string>();
            t7.onCanInfo += (s, e) =>
            {
                if (e.Type == (a_write ? ActivityType.FinishedFlashing : ActivityType.FinishedDownloadingFlash))
                    lock (finished) finished.Add(e.Info);
            };
            var stop = new ManualResetEventSlim(false);
            var adapter = new Thread(() =>
            {
                if (stop.Wait(2500)) return;
                do
                {
                    receive.Invoke(combi, new object[] { (byte)0x89, new byte[0], caCombiAdapter.term_nak });
                }
                while (!stop.Wait(20));
            });
            adapter.Start();
            string file = NewFileName();
            try
            {
                if (a_write)
                    t7.WriteFlash(file);
                else
                    t7.ReadFlash(file);
                callerFree.Set();
                Thread.Sleep(2500);
                lock (finished)
                {
                    return new List<string>(finished);
                }
            }
            finally
            {
                stop.Set();
                callerFree.Set();
                adapter.Join();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void ReadTimerFinishesOnceAfterTicksQueuedBehindCaller()
        {
            // only one tick may finish: the second one finished again, and after the first one's stopFlasher
            // T7CombiFlasher reports Completed, so it could even say "Finished download of FLASH"
            List<string> finished = FinishEventsAfterFailedCombiConnect(false);
            CollectionAssert.AreEqual(new[] { "Failed to download FLASH content" }, finished, string.Join(" | ", finished));
        }

        [TestMethod]
        public void WriteTimerFinishesOnceAfterTicksQueuedBehindCaller()
        {
            List<string> finished = FinishEventsAfterFailedCombiConnect(true);
            CollectionAssert.AreEqual(new[] { "A write error occured, please retry to FLASH without cutting power to the ECU" }, finished,
                string.Join(" | ", finished));
        }

        // the read timer's verdict on a read started like the GUI does, how long the flasher took to give
        // up (ms from the first unanswered request to ReadError), and whether the file is left behind
        static (string finished, long giveUpMs, bool fileLeft) ReadWithDeadEcuAfter(FakeEcu ecu, int a_goodBlocks)
        {
            int reads = 0;
            var firstMiss = new Stopwatch();
            ecu.NoReply = req =>
            {
                if (req[1] != 0x21 || ++reads <= a_goodBlocks) return false;
                if (!firstMiss.IsRunning) firstMiss.Start();
                return true;
            };
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            Trionic7 t7 = NewTrionic7(flasher);
            long giveUpMs = -1;
            var watch = new Thread(() =>
            {
                while (flasher.getStatus() != IFlasher.FlashStatus.ReadError && giveUpMs < 0) Thread.Sleep(1);
                giveUpMs = firstMiss.ElapsedMilliseconds;
            }) { IsBackground = true };
            try
            {
                watch.Start();
                var (e, ms) = ReadUntilFinished(t7, file);
                watch.Join(1000);
                return (e?.Info, giveUpMs, File.Exists(file));
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void ReadGivesUpOnPersistentNoReplyWithinBound()
        {
            // while (!sendRequestDataByOffset(...)) retried forever: an ECU that stopped answering (BDM halt,
            // reboot) left the read at its percentage for good
            int budget = T7Flasher.m_retryTimeMs;
            T7Flasher.m_retryTimeMs = 1000;
            try
            {
                var ecu = new FakeEcu { ReadReply = "4261F0" + new string('0', 128) };
                var (finished, giveUpMs, fileLeft) = ReadWithDeadEcuAfter(ecu, 100);
                Assert.AreEqual("Failed to download FLASH content", finished);
                Assert.IsFalse(fileLeft, "the partial file must be deleted");
                Assert.IsTrue(giveUpMs >= 1000 && giveUpMs < 1500, "gave up after " + giveUpMs + " ms, bound 1000 ms");
                Assert.AreEqual(101, ecu.Requests.FindAll(r => r.StartsWith("082CF003")).Count, "block 101 defined once, its 21 F0 retried");
                CollectionAssert.DoesNotContain(ecu.Requests, "028200", "an ECU that stopped answering gets no stop from the flasher");
            }
            finally
            {
                T7Flasher.m_retryTimeMs = budget;
            }
        }

        [TestMethod]
        public void ReadRetriesAtLeastThreeTimes()
        {
            // a device whose failed call alone outlasts the time budget (ELM327 K-line: ~9 s) still gets retries
            int budget = T7Flasher.m_retryTimeMs;
            T7Flasher.m_retryTimeMs = 0;
            try
            {
                var ecu = new FakeEcu { ReadReply = "4261F0" + new string('0', 128) };
                var (finished, giveUpMs, fileLeft) = ReadWithDeadEcuAfter(ecu, 3);
                Assert.AreEqual("Failed to download FLASH content", finished);
                // 3 attempts, each a KWPHandler call that tries the device 3 times
                Assert.AreEqual(3 + 3 * 3, ecu.Requests.FindAll(r => r == "0221F0").Count);
            }
            finally
            {
                T7Flasher.m_retryTimeMs = budget;
            }
        }

        [TestMethod]
        public void ReadEndsAtOnceWhenDeviceIsClosed()
        {
            // the adapter closed under the flasher: every request returned DeviceNotConnected at once and
            // the read loop spun on it, at 100% CPU, forever
            var ecu = new FakeEcu { ReadReply = "4261F0" + new string('0', 128) };
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            int reads = 0;
            var closed = new Stopwatch();
            ecu.NoReply = req =>
            {
                if (req[1] != 0x21 || ++reads <= 50) return false;
                ecu.Open = false;
                closed.Start();
                return true;
            };
            try
            {
                flasher.readFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, WaitWhileBusy(flasher));
                Assert.IsTrue(closed.ElapsedMilliseconds < 500, "the read must end when the device closes, took " + closed.ElapsedMilliseconds + " ms");
                Assert.IsFalse(File.Exists(file), "the partial file must be deleted");
                // the request in flight (KWPHandler tries 3 times), then nothing
                Assert.AreEqual(50 + 3, ecu.Requests.FindAll(r => r == "0221F0").Count);
                Thread.Sleep(200);
                Assert.AreEqual(50 + 3, ecu.Requests.FindAll(r => r == "0221F0").Count);
                Thread thread = (Thread)typeof(T7Flasher).GetField("m_thread", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(flasher);
                Assert.IsTrue((thread.ThreadState & System.Threading.ThreadState.WaitSleepJoin) != 0, "the flasher thread waits for the next command: " + thread.ThreadState);
                flasher.cleanup();
                Assert.IsTrue(thread.Join(2000), "cleanup ends the flasher thread");
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void StopFlasherStopsARead()
        {
            // a stop used to continue through the remaining blocks without reading them
            var ecu = new FakeEcu { ReadReply = "4261F0" + new string('0', 128) };
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            int reads = 0;
            ecu.OnRequest = req => { if (req[1] == 0x21 && ++reads == 20) flasher.stopFlasher(); };
            try
            {
                flasher.readFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, WaitWhileBusy(flasher));
                Assert.AreEqual(20, ecu.Requests.FindAll(r => r == "0221F0").Count, "no request after the stop");
                Assert.IsFalse(File.Exists(file), "a stopped read leaves no file");
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void ExceptionInReadEndsTheReadNotTheProcess()
        {
            // an adapter exception on the flasher thread ended the process, and the request mutex it held
            // would have hung every later request
            var ecu = new FakeEcu { ReadReply = "4261F0" + new string('0', 128) };
            string file = NewFileName();
            var flasher = NewFlasher(ecu);
            int reads = 0;
            ecu.OnRequest = req => { if (req[1] == 0x21 && ++reads == 30) throw new InvalidOperationException("adapter gone"); };
            try
            {
                flasher.readFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.ReadError, WaitWhileBusy(flasher));
                Assert.IsFalse(File.Exists(file), "the partial file must be deleted");
                var other = System.Threading.Tasks.Task.Run(() => KWPHandler.getInstance().sendUnknownRequest());
                Assert.IsTrue(other.Wait(2000), "the request mutex must be free after the exception");
                Assert.AreEqual(KWPResult.OK, other.Result);
            }
            finally
            {
                flasher.cleanup();
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void ExceptionInWriteEndsTheWriteNotTheProcess()
        {
            // the write path has no catch of its own: only run()'s keeps an adapter exception from ending the process
            var ecu = new FakeEcu();
            string file = PatternBin();
            var flasher = NewFlasher(ecu);
            int blocks = 0;
            ecu.OnRequest = req => { if (req[1] == 0x36 && ++blocks == 10) throw new InvalidOperationException("adapter gone"); };
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, WaitWhileBusy(flasher));
                Assert.AreEqual(9, ecu.TransferBlocks, "no block after the exception");
                Thread thread = (Thread)typeof(T7Flasher).GetField("m_thread", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(flasher);
                Thread.Sleep(100);
                Assert.IsTrue((thread.ThreadState & System.Threading.ThreadState.WaitSleepJoin) != 0, "the flasher thread waits for the next command: " + thread.ThreadState);
                var other = System.Threading.Tasks.Task.Run(() => KWPHandler.getInstance().sendUnknownRequest());
                Assert.IsTrue(other.Wait(2000), "the request mutex must be free after the exception");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        // WriteFlash as the GUI calls it; returns the FinishedFlashing events the write timer raised in a_ms
        static List<string> WriteFinishEvents(Trionic7 t7, string file, int a_ms)
        {
            var finished = new List<string>();
            t7.onCanInfo += (s, e) =>
            {
                if (e.Type == ActivityType.FinishedFlashing)
                    lock (finished) finished.Add(e.Info);
            };
            t7.WriteFlash(file);
            Thread.Sleep(a_ms);
            lock (finished)
            {
                return new List<string>(finished);
            }
        }

        [TestMethod]
        public void EraseFailureStopsTheWrite()
        {
            // "Failed to erase flash..." was followed by a commented-out break: every block was still sent,
            // and run() then overwrote EraseError with Completed, "Finished FLASH session"
            var ecu = new FakeEcu();
            ecu.NoReply = req => req[1] == 0x31 && req[2] == 0x53;
            string file = PatternBin();
            var flasher = NewFlasher(ecu);
            Trionic7 t7 = NewTrionic7(flasher);
            try
            {
                List<string> finished = WriteFinishEvents(t7, file, 2500);
                CollectionAssert.AreEqual(new[] { "An erase error occured" }, finished, string.Join(" | ", finished));
                Assert.AreEqual(0, ecu.Requests.FindAll(r => r.StartsWith("0834")).Count, "no requestDownload after a failed erase");
                Assert.AreEqual(0, ecu.TransferBlocks, "no block after a failed erase");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void StopFlasherStopsAWrite()
        {
            // the write loops' stop check was a continue: the write went on with the next block
            var ecu = new FakeEcu();
            string file = PatternBin();
            var flasher = NewFlasher(ecu);
            int blocks = 0;
            ecu.OnRequest = req => { if (req[1] == 0x36 && ++blocks == 10) flasher.stopFlasher(); };
            try
            {
                flasher.writeFlash(file);
                Assert.AreEqual(IFlasher.FlashStatus.WriteError, WaitWhileBusy(flasher));
                Assert.AreEqual(10, ecu.TransferBlocks, "no block after the stop");
                Thread.Sleep(200);
                Assert.AreEqual(10, ecu.TransferBlocks);
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void WriteWithoutSecurityAccessIsNotCompleted()
        {
            // WriteCommand reported a write it didn't do as Completed ("Finished FLASH session")
            var ecu = new FakeEcu();
            string file = TempBin();
            var flasher = NewFlasher(ecu);
            try
            {
                typeof(T7Flasher).GetField("m_fileName", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(flasher, file);
                typeof(T7Flasher).GetMethod("WriteCommand", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(flasher, null);
                Assert.AreEqual(IFlasher.FlashStatus.NoSequrityAccess, flasher.getStatus());
                Assert.AreEqual(0, ecu.Requests.Count, "nothing is sent");
            }
            finally
            {
                flasher.cleanup();
                File.Delete(file);
            }
        }

        [TestMethod]
        public void SramSnapshotReadsFullBlocks()
        {
            var ecu = new FakeEcu { ReadReply = "8261F0" + new string('5', 256) };
            KWPHandler.setKWPDevice(ecu);
            Trionic7 t7 = new Trionic7();
            string file = NewFileName();
            try
            {
                Assert.IsTrue(t7.GetSRAMSnapshot(file));
                byte[] ram = File.ReadAllBytes(file);
                Assert.AreEqual(0x10000, ram.Length);
                Assert.IsTrue(ram.All(b => b == 0x55));
            }
            finally
            {
                if (File.Exists(file)) File.Delete(file);
            }
        }

        [TestMethod]
        public void SramSnapshotFailsOnShortReply()
        {
            // GUI "Read SRAM" on an ECU still in its EOL loop: 7F 21 12 wrote one byte per 128-byte
            // block and the snapshot was announced as downloaded
            var ecu = new FakeEcu();
            KWPHandler.setKWPDevice(ecu);
            Trionic7 t7 = new Trionic7();
            var infos = new List<string>();
            t7.onCanInfo += (s, e) => infos.Add(e.Info);
            string file = NewFileName();
            try
            {
                Assert.IsFalse(t7.GetSRAMSnapshot(file));
                CollectionAssert.DoesNotContain(infos, "Snapshot downloaded");
            }
            finally
            {
                if (File.Exists(file)) File.Delete(file);
            }
        }
    }
}
