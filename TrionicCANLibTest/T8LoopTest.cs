using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Runtime.ExceptionServices;
using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib;
using TrionicCANLib.API;
using TrionicCANLib.CAN;

namespace TrionicCANLibTest
{
    // Waits in Trionic8 that went on forever while the ECU said nothing (T8 Get ECU info with the ECU unpowered,
    // the ME9.6 pre-reads, a loader that died mid-flash), with the window that can't be closed during an operation.
    // They now give up after P2*CAN (5 s, Trionic8.timeoutP2ce) without an answer, restarted by every 0x78
    // response-pending. The tests run the real timings.
    [TestClass]
    public class T8LoopTest
    {
        const int P2ce = 5000;
        const string Pending1A = "7E8 037F1A7800000000";

        // Fake T8 / ME9.6 / CIM. Answer gets each frame sent ("7E0 021A710000000000") and returns the frames the ECU
        // sends back at once ("7E8 ..."), none by default: an unpowered ECU. Later sends one after a delay.
        class FakeEcu : ICANDevice
        {
            public readonly List<string> Sent = new List<string>();
            public Func<string, string[]> Answer = f => null;

            public override OpenResult open() { return OpenResult.OK; }
            public override CloseResult close() { return CloseResult.OK; }
            public override bool isOpen() { return true; }
            public override uint waitForMessage(uint a_canID, uint timeout, out CANMessage canMsg) { canMsg = new CANMessage(); return 0; }
            public override float GetThermoValue() { return 0; }
            public override float GetADCValue(uint channel) { return 0; }
            public override void SetSelectedAdapter(string adapter) { }
            public override int ForcedBaudrate { get; set; }
            public override bool bypassCANfilters { get; set; }

            protected override bool sendMessageDevice(CANMessage a_message)
            {
                byte[] d = new byte[8];
                for (uint i = 0; i < 8; i++)
                {
                    d[i] = a_message.getCanData(i);
                }
                string frame = a_message.getID().ToString("X3") + " " + Convert.ToHexString(d);
                lock (Sent)
                {
                    Sent.Add(frame);
                }
                string[] replies = Answer(frame);
                if (replies != null)
                {
                    foreach (string reply in replies)
                    {
                        Reply(reply);
                    }
                }
                return true;
            }

            public void Reply(string a_frame)
            {
                CANMessage msg = new CANMessage(Convert.ToUInt32(a_frame.Substring(0, 3), 16), 0, 8);
                msg.setCanData(Convert.FromHexString(a_frame.Substring(4)), 8);
                receivedMessage(msg);
            }

            public void Later(int a_ms, string a_frame)
            {
                new Thread(() => { Thread.Sleep(a_ms); Reply(a_frame); }) { IsBackground = true }.Start();
            }
        }

        // Trionic8 as setCANDevice leaves it, on the fake instead of an adapter
        static Trionic8 NewTrionic8(FakeEcu a_ecu)
        {
            Trionic8 t8 = new Trionic8();
            CANListener listener = new CANListener();
            a_ecu.addListener(listener);
            typeof(Trionic8).GetField("m_canListener", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t8, listener);
            typeof(ITrionic).GetField("canUsbDevice", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t8, a_ecu);
            return t8;
        }

        static object Invoke(Trionic8 a_t8, string a_method, params object[] a_args)
        {
            try
            {
                return typeof(Trionic8).GetMethod(a_method, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(a_t8, a_args);
            }
            catch (TargetInvocationException e)
            {
                ExceptionDispatchInfo.Capture(e.InnerException).Throw();
                throw;
            }
        }

        // a_call on a thread of its own, so a loop without end fails the test after a_limitMs instead of hanging the run
        static T Bounded<T>(int a_limitMs, Func<T> a_call, out long r_elapsedMs)
        {
            T result = default(T);
            ExceptionDispatchInfo error = null;
            Stopwatch sw = Stopwatch.StartNew();
            Thread t = new Thread(() =>
            {
                try
                {
                    result = a_call();
                }
                catch (Exception e)
                {
                    error = ExceptionDispatchInfo.Capture(e);
                }
            }) { IsBackground = true };
            t.Start();
            bool done = t.Join(a_limitMs);
            r_elapsedMs = sw.ElapsedMilliseconds;
            Assert.IsTrue(done, "still waiting for the ECU after " + a_limitMs + " ms");
            error?.Throw();
            return result;
        }

        static void AssertBetween(long a_min, long a_max, long a_ms)
        {
            Assert.IsTrue(a_ms >= a_min && a_ms <= a_max, a_ms + " ms, expected " + a_min + " to " + a_max);
        }

        // the ECU side of a TransferData ($36): flow control for the first frame, a_done's frames after the last
        // consecutive one
        static Func<string, string[]> TransferData(Func<string[]> a_done)
        {
            int remaining = 0;
            return f =>
            {
                byte[] d = Convert.FromHexString(f.Substring(4));
                if (f.StartsWith("7E0") && (d[0] & 0xF0) == 0x10 && d[2] == 0x36)
                {
                    remaining = ((d[0] & 0x0F) << 8 | d[1]) - 6;
                    return new[] { "7E8 3000000000000000" };
                }
                if (f.StartsWith("7E0") && (d[0] & 0xF0) == 0x20 && remaining > 0)
                {
                    remaining -= 7;
                    if (remaining <= 0)
                    {
                        return a_done();
                    }
                }
                return null;
            };
        }

        static string TempFile(int a_length)
        {
            string path = Path.Combine(Path.GetTempPath(), "t8looptest-" + Guid.NewGuid().ToString("N") + ".bin");
            byte[] bytes = new byte[a_length];
            for (int i = 0; i < bytes.Length; i++)
            {
                bytes[i] = (byte)i;
            }
            File.WriteAllBytes(path, bytes);
            return path;
        }

        [TestMethod]
        public void GetEcuInfoGivesUpOnASilentEcu()
        {
            // T8 Get ECU info with the ECU unpowered: its first read, ECU hardware, never came back
            FakeEcu ecu = new FakeEcu();
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            string hardware = Bounded(3 * P2ce, () => t8.GetECUHardware(), out ms);
            Assert.AreEqual("", hardware, "the GUI shows \"ECU connection issue\" for this");
            AssertBetween(P2ce, P2ce + 1500, ms);
            CollectionAssert.AreEqual(new[] { "7E0 021A710000000000" }, ecu.Sent);
        }

        [TestMethod]
        public void ReadDataByIdentifierGivesUpOnASilentEcu()
        {
            // the byte reads (speed limiter, oil quality, PI 01, DIDs) share the loop
            FakeEcu ecu = new FakeEcu();
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            byte[] data = Bounded(3 * P2ce, () => t8.RequestECUInfo(0x02), out ms);
            CollectionAssert.AreEqual(new byte[2], data);
            AssertBetween(P2ce, P2ce + 1500, ms);
            CollectionAssert.AreEqual(new[] { "7E0 021A020000000000" }, ecu.Sent);
        }

        [TestMethod]
        public void GetEcuInfoAnswersAreUnchanged()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f == "7E0 021A710000000000") return new[] { "7E8 100B5A7141424344" };
                if (f.StartsWith("7E0 30")) return new[] { "7E8 2145464748494A4B" };
                if (f == "7E0 021A020000000000") return new[] { "7E8 045A020A8C000000" };
                if (f == "7E0 021A720000000000") return new[] { "7E8 037F1A3100000000" };
                return null;
            };
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            Assert.AreEqual("ABCDEFGHIJ", Bounded(P2ce, () => t8.GetECUHardware(), out ms));
            Assert.AreEqual(270, Bounded(P2ce, () => t8.GetTopSpeed(), out ms));
            Assert.AreEqual("", Bounded(P2ce, () => t8.GetECUDescription(), out ms), "a refusal is an answer");
            Assert.IsTrue(ms < 1000, ms + " ms");
        }

        [TestMethod]
        public void GetEcuInfoWaitsAsLongAsTheEcuSaysPending()
        {
            // 0x78 at 0 and 3 s, the answer 6 s after the request: more than P2*CAN in all, 3 s after the last 0x78
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f != "7E0 021A710000000000") return null;
                ecu.Later(3000, Pending1A);
                ecu.Later(6000, "7E8 075A714142434445");
                return new[] { Pending1A };
            };
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            Assert.AreEqual("ABCD", Bounded(4 * P2ce, () => t8.GetECUHardware(), out ms));
            AssertBetween(6000, 7500, ms);
        }

        [TestMethod]
        public void ReadDataByIdentifierWaitsAsLongAsTheEcuSaysPending()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f != "7E0 021A020000000000") return null;
                ecu.Later(3000, Pending1A);
                ecu.Later(6000, "7E8 045A020A8C000000");
                return new[] { Pending1A };
            };
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            Assert.AreEqual(270, Bounded(4 * P2ce, () => t8.GetTopSpeed(), out ms));
            AssertBetween(6000, 7500, ms);
        }

        // 0x78 to a_request at once and 0.5, 1 and 1.5 s later, then nothing. The wait grew *3 with each 0x78,
        // 12 s after the fourth (hours after ten), and the give-up only came when a wait ran out
        static Func<string, string[]> PendingFourTimes(FakeEcu a_ecu, string a_request)
        {
            return f =>
            {
                if (f != a_request) return null;
                a_ecu.Later(500, Pending1A);
                a_ecu.Later(1000, Pending1A);
                a_ecu.Later(1500, Pending1A);
                return new[] { Pending1A };
            };
        }

        [TestMethod]
        public void GetEcuInfoGivesUpP2ceAfterTheLastOfManyPendings()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = PendingFourTimes(ecu, "7E0 021A710000000000");
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            Assert.AreEqual("", Bounded(2 * P2ce, () => t8.GetECUHardware(), out ms));
            AssertBetween(1500 + P2ce, 1500 + P2ce + 1500, ms);
        }

        [TestMethod]
        public void ReadDataByIdentifierGivesUpP2ceAfterTheLastOfManyPendings()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = PendingFourTimes(ecu, "7E0 021A020000000000");
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            CollectionAssert.AreEqual(new byte[2], Bounded(2 * P2ce, () => t8.RequestECUInfo(0x02), out ms));
            AssertBetween(1500 + P2ce, 1500 + P2ce + 1500, ms);
        }

        [TestMethod]
        public void Pi01IsNotWrittenWithoutTheCurrentValue()
        {
            // Edit Parameters: SetPI01 writes its bits over the PI 01 it reads first. Unanswered, that read gave
            // zeros, and the bits it doesn't set were written back cleared
            FakeEcu ecu = new FakeEcu();
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            Assert.IsFalse(Bounded(3 * P2ce, () => t8.SetPI01(false, false, false, false, DiagnosticType.None, false, TankType.EU), out ms));
            CollectionAssert.AreEqual(new[] { "7E0 021A010000000000" }, ecu.Sent, "no WriteDataByIdentifier");
        }

        [TestMethod]
        public void Pi01IsWrittenOverTheCurrentValue()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f == "7E0 021A010000000000") return new[] { "7E8 045A018281000000" };
                if (f.StartsWith("7E0 063B01")) return new[] { "7E8 027B010000000000" };
                return null;
            };
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            Assert.IsTrue(Bounded(P2ce, () => t8.SetPI01(false, false, false, false, DiagnosticType.None, false, TankType.EU), out ms));
            Assert.AreEqual("7E0 063B0192CB000000", ecu.Sent[1], "82 81 read: bits 7 and 1, 7 and 0 kept");
        }

        [TestMethod]
        public void DidBackupStopsWhenTheEcuStopsAnswering()
        {
            // ME9.6 read and calibration read start with SaveAllDID: 255 DIDs, P2*CAN each once bounded. The ME9.6
            // read opens without a word to the ECU, so with it unpowered this got no DID at all, and the empty
            // file it wrote replaced a .did file saved before under the same name
            FakeEcu ecu = new FakeEcu();
            Trionic8 t8 = NewTrionic8(ecu);
            string bin = Path.Combine(Path.GetTempPath(), "t8looptest-" + Guid.NewGuid().ToString("N") + ".bin");
            string did = Path.ChangeExtension(bin, ".did");
            File.WriteAllLines(did, new[] { "144,QUJDRA==" });
            try
            {
                long ms;
                Assert.IsFalse(Bounded(5 * P2ce, () => t8.SaveAllDID(bin), out ms));
                AssertBetween(3 * P2ce, 3 * P2ce + 2500, ms);
                CollectionAssert.AreEqual(new[] { "7E0 021A000000000000", "7E0 021A010000000000", "7E0 021A020000000000" }, ecu.Sent);
                CollectionAssert.AreEqual(new[] { "144,QUJDRA==" }, File.ReadAllLines(did), "the earlier backup is kept");
            }
            finally
            {
                File.Delete(did);
            }
        }

        [TestMethod]
        public void DidBackupGoesOnAfterOneLostAnswer()
        {
            // a refusal is an answer; the answer to DID 05 is lost, the read goes on, gets DID 90 and saves it
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f == "7E0 021A050000000000") return null;
                if (f == "7E0 021A900000000000") return new[] { "7E8 065A904142434400" };
                if (f.StartsWith("7E0 021A")) return new[] { "7E8 037F1A3100000000" };
                return null;
            };
            Trionic8 t8 = NewTrionic8(ecu);
            string bin = Path.Combine(Path.GetTempPath(), "t8looptest-" + Guid.NewGuid().ToString("N") + ".bin");
            string did = Path.ChangeExtension(bin, ".did");
            try
            {
                long ms;
                Assert.IsTrue(Bounded(4 * P2ce, () => t8.SaveAllDID(bin), out ms));
                CollectionAssert.AreEqual(new[] { "144,QUJDRA==" }, File.ReadAllLines(did), "DID 0x90, ABCD");
                Assert.AreEqual(255, ecu.Sent.Count);
            }
            finally
            {
                File.Delete(did);
            }
        }

        [TestMethod]
        public void Me96FlashFailsWhenTheEcuStopsAnswering()
        {
            // after a block's data the wait for 01 76 had no end
            FakeEcu ecu = new FakeEcu();
            Trionic8 t8 = NewTrionic8(ecu);
            string file = TempFile(0x20);
            try
            {
                long ms;
                bool ok = Bounded(5 * P2ce, () => (bool)Invoke(t8, "ProgramFlashME96", file, 0, 0x10), out ms);
                Assert.IsFalse(ok);
                AssertBetween(2 * P2ce, 2 * P2ce + 2000, ms);
            }
            finally
            {
                File.Delete(file);
            }
        }

        [TestMethod]
        public void Me96FlashWaitsOutAPendingBlock()
        {
            // 0x78 for the block, 01 76 6 s later: one P2*CAN window without a frame is not the end yet
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = TransferData(() =>
            {
                ecu.Later(6000, "7E8 0176000000000000");
                return new[] { "7E8 037F367800000000" };
            });
            Trionic8 t8 = NewTrionic8(ecu);
            string file = TempFile(0x20);
            try
            {
                long ms;
                Assert.IsTrue(Bounded(5 * P2ce, () => (bool)Invoke(t8, "ProgramFlashME96", file, 0, 0x10), out ms));
                AssertBetween(6000, 7500, ms);
                CollectionAssert.AreEqual(new[] { "7E0 1015360000000000", "7E0 2101020304050607", "7E0 2208090A0B0C0D0E",
                    "7E0 230F000000000000", "7E0 013E000000000000" }, ecu.Sent, "the block, then a tester present");
            }
            finally
            {
                File.Delete(file);
            }
        }

        [TestMethod]
        public void Me96FlashThatGaveUpIsReportedFailed()
        {
            // security access and erase answered, then the ECU went quiet in the first block: "FLASH upload failed"
            // was followed by the GUI's "Operation done", Result true with the FLASH half-written
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f == "7E0 0227010000000000") return new[] { "7E8 0467010000000000" };
                if (f == "7E0 0534001800000000") return new[] { "7E8 0174000000000000" };
                return null;
            };
            Trionic8 t8 = NewTrionic8(ecu);
            string file = TempFile(0x20);
            try
            {
                DoWorkEventArgs args = new DoWorkEventArgs(new FlashReadArguments() { FileName = file, start = 0, end = 0x10 });
                long ms;
                Bounded(6 * P2ce, () => { t8.WriteFlashME96(null, args); return true; }, out ms);
                Assert.AreEqual(false, args.Result);
                Assert.IsTrue(ecu.Sent.Contains("7E0 1015360000000000"), "it got as far as the block");
            }
            finally
            {
                File.Delete(file);
            }
        }

        static BlockManager NewBlockManager(string a_file)
        {
            BlockManager bm = new BlockManager();
            Assert.IsTrue(bm.SetFilename(a_file));
            return bm;
        }

        [TestMethod]
        public void LegionFlashFailsWhenTheLoaderStopsAnswering()
        {
            // a silent loader retried the block's TransferData without end
            FakeEcu ecu = new FakeEcu();
            Trionic8 t8 = NewTrionic8(ecu);
            typeof(Trionic8).GetField("formatmask", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t8, 0x1FFu);
            string file = TempFile((int)TrionicCANLib.Firmware.FileT8.Length);
            try
            {
                BlockManager bm = NewBlockManager(file);
                long ms;
                bool ok = Bounded(P2ce, () => (bool)Invoke(t8, "ProgramFlashLeg", 0x80, bm, Trionic8.EcuByte_T8), out ms);
                Assert.IsFalse(ok);
                Assert.AreEqual(20, ecu.Sent.Count);
                Assert.IsTrue(ecu.Sent.TrueForAll(f => f == "7E0 1088360000000000"));
                AssertBetween(20 * 150, 20 * 150 + 1500, ms);
            }
            finally
            {
                File.Delete(file);
            }
        }

        [TestMethod]
        public void LegionFlashOfAnAnsweredBlockIsUnchanged()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = TransferData(() => new[] { "7E8 0176000000000000" });
            Trionic8 t8 = NewTrionic8(ecu);
            typeof(Trionic8).GetField("formatmask", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t8, 0x1FFu);
            string file = TempFile((int)TrionicCANLib.Firmware.FileT8.Length);
            try
            {
                BlockManager bm = NewBlockManager(file);
                long ms;
                Assert.IsTrue(Bounded(P2ce, () => (bool)Invoke(t8, "ProgramFlashLeg", 0x80, bm, Trionic8.EcuByte_T8), out ms));
                Assert.AreEqual(20, ecu.Sent.Count, "first frame and 19 consecutive frames");
            }
            finally
            {
                File.Delete(file);
            }
        }

        [TestMethod]
        public void SramSnapshotStopsAfterItsRetries()
        {
            // it reported the failure after maxRetries refused reads and went on reading
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (f.StartsWith("7E0 013E")) return new[] { "7E8 017E000000000000" };
                if (f.StartsWith("7E0 0623")) return new[] { "7E8 037F232200000000" };
                return null;
            };
            Trionic8 t8 = NewTrionic8(ecu);
            string file = Path.Combine(Path.GetTempPath(), "t8looptest-" + Guid.NewGuid().ToString("N") + ".RAM");
            DoWorkEventArgs args = new DoWorkEventArgs(file);
            long ms;
            Bounded(30000, () => { t8.GetSRAMSnapshot(null, args); return true; }, out ms);
            Assert.AreEqual(false, args.Result);
            Assert.IsFalse(File.Exists(file));
            Assert.AreEqual(100, ecu.Sent.FindAll(f => f.StartsWith("7E0 0623")).Count);
        }

        [TestMethod]
        public void CimDtcReadEndsWhenTheCimGoesQuiet()
        {
            // 0x78, then nothing: every empty wait was listed as DTC P0000, without end
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f => f.StartsWith("245 03A981") ? new[] { "545 037FA97800000000" } : null;
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            string[] dtcs = Bounded(3 * P2ce, () => t8.readDTCCodesCIM(), out ms);
            Assert.AreEqual(0, dtcs.Length);
            AssertBetween(P2ce, P2ce + 2000, ms);
        }

        [TestMethod]
        public void CimDtcReadWaitsForABusyCim()
        {
            FakeEcu ecu = new FakeEcu();
            ecu.Answer = f =>
            {
                if (!f.StartsWith("245 03A981")) return null;
                ecu.Later(1000, "545 810107006F000000");
                ecu.Later(1200, "545 81000000FF000000");
                return new[] { "545 037FA97800000000" };
            };
            Trionic8 t8 = NewTrionic8(ecu);
            long ms;
            CollectionAssert.AreEqual(new[] { "DTC: P0107 StatusByte: 6F", "0xFF No more errors!" },
                Bounded(3 * P2ce, () => t8.readDTCCodesCIM(), out ms));
        }
    }
}
