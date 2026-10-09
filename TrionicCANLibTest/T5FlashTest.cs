using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.API;
using TrionicCANLib.CAN;

namespace TrionicCANLibTest
{
    [TestClass]
    public class T5FlashTest
    {
        // Fake T5.5 with AMD 28F010 FLASH, as on the bench. Until the C1 jump it is the ECU's own loader: an upload
        // frame is answered with its first byte and zeros. Then MyBooty (MyBooty.asm v1.3), answers as logged:
        // A5 stores the FLASH address and count; data frame nn + 7 bytes is copied into its buffer at offset nn,
        // the frame that fills the buffer programs the block (a resent one programs it again); C0 erases, C7 reads,
        // C8 sums ROM offset..code end from the footer, C9 identifies, C2 exits.
        class FakeT5 : ICANDevice
        {
            public readonly List<string> Sent = new List<string>();
            public readonly byte[] Flash = new byte[0x80000];
            public readonly Dictionary<uint, int> Programmed = new Dictionary<uint, int>(); // block address: times programmed
            public bool BootyRunning;
            public Func<string, bool> Lost = s => false;  // the ECU never gets the frame
            public Func<string, bool> Mute = s => false;  // the ECU acts on the frame, its answer is lost
            public Func<string, bool> Late = s => false;  // the answer comes in once the next frame has been sent
            public Func<string, bool> Unplug = s => false; // the adapter throws on this frame, as a vanished port can
            public string Identity = "000004000001A7"; // C9 after its echo: FLASH start 0x40000, AMD 28F010 (256 kB)
            private string m_late;
            private uint m_address;
            private int m_length;
            private readonly byte[] m_buffer = new byte[0x100];

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
                Sent.Add(frame);
                if (Unplug(frame))
                {
                    throw new IOException("The port is closed.");
                }
                if (m_late != null)
                {
                    Reply(m_late);
                    m_late = null;
                }
                if (a_message.getID() != 0x005 || Lost(frame))
                {
                    return true;
                }
                string reply = BootyRunning ? Booty(d) : Loader(d);
                if (reply == null || Mute(frame))
                {
                    return true;
                }
                if (Late(frame))
                {
                    m_late = reply;
                    return true;
                }
                Reply(reply);
                return true;
            }

            void Reply(string reply)
            {
                CANMessage msg = new CANMessage(0x00C, 0, 8);
                msg.setCanData(Convert.FromHexString(reply), 8);
                receivedMessage(msg);
            }

            static uint Address(byte[] d) { return (uint)(d[1] << 24 | d[2] << 16 | d[3] << 8 | d[4]); }

            static string Echo(byte cmd, string rest) { return cmd.ToString("X2") + rest; }

            string Loader(byte[] d)
            {
                if (d[0] == 0xA5 || d[0] <= 0x7F)
                {
                    return Echo(d[0], "00000000000000");
                }
                if (d[0] == 0xC1 && Address(d) == 0x5000)
                {
                    BootyRunning = true; // jumps to MyBooty, no answer
                }
                return null;
            }

            string Booty(byte[] d)
            {
                byte c = d[0];
                if (c <= 0x7F)
                {
                    int n = c;
                    for (int k = 1; k < 8; k++)
                    {
                        m_buffer[n++] = d[k];
                        if ((byte)n == (byte)m_length)
                        {
                            return Echo(c, Program() ? "00080808080808" : "01000000C00808");
                        }
                    }
                    return Echo(c, "00080808080808");
                }
                switch (c)
                {
                    case 0xA5:
                        m_address = Address(d);
                        m_length = d[5];
                        return Echo(c, "00080808080808");
                    case 0xC0:
                        Array.Fill(Flash, (byte)0xFF, 0x40000, 0x40000);
                        return Echo(c, "00080808080808");
                    case 0xC2:
                        BootyRunning = false;
                        return Echo(c, "00080808080808");
                    case 0xC7:
                        uint a = Address(d);
                        return Echo(c, "00" + Convert.ToHexString(new[] { Flash[a], Flash[a - 1], Flash[a - 2], Flash[a - 3], Flash[a - 4], Flash[a - 5] }));
                    case 0xC8:
                        return Echo(c, Checksum());
                    case 0xC9:
                        return Echo(c, Identity);
                    default:
                        return Echo(c, "09080808080808");
                }
            }

            // MyBooty refuses addresses below FLASH; a 28F010 can't program a 0 back to 1
            bool Program()
            {
                if (m_address < 0x40000 || m_address + m_length > 0x80000)
                {
                    return false;
                }
                Programmed[m_address] = Programmed.GetValueOrDefault(m_address) + 1;
                for (int i = 0; i < m_length; i++)
                {
                    byte v = m_buffer[i];
                    if (v != 0xFF)
                    {
                        if ((Flash[m_address + i] & v) != v)
                        {
                            return false;
                        }
                        Flash[m_address + i] = v;
                    }
                }
                return true;
            }

            // Get_Checksum: from 0x7FFFB down, length, identifier, text (most significant digit highest);
            // ROM offset (FD), then code end (FE) further down
            string Checksum()
            {
                uint p = 0x7FFFB;
                uint[] range = new uint[2];
                for (int k = 0; k < 2; k++)
                {
                    while (true)
                    {
                        int len = Flash[p];
                        if (len == 0 || len == 0xFF)
                        {
                            return "01080808080808";
                        }
                        byte id = Flash[p - 1];
                        uint s = p - (uint)len - 2;
                        p = s;
                        if (id == 0xFD + k)
                        {
                            string text = "";
                            for (int j = len; j > 0; j--)
                            {
                                text += (char)Flash[s + j];
                            }
                            range[k] = Convert.ToUInt32(text, 16);
                            break;
                        }
                    }
                }
                if (range[1] <= range[0] || range[1] >= 0x7FFFF)
                {
                    return "01080808080808";
                }
                uint sum = 0;
                for (uint a = range[0]; a <= range[1]; a++)
                {
                    sum += Flash[a];
                }
                uint stored = (uint)(Flash[0x7FFFC] << 24 | Flash[0x7FFFD] << 16 | Flash[0x7FFFE] << 8 | Flash[0x7FFFF]);
                return sum == stored ? "00" + sum.ToString("X8") + "0808" : "01080808080808";
            }
        }

        // From tonight's bench flash of a T5.5 (logs s2-vcp-flash, s2-combi-flash, s2-j2534-flash: the same frames).
        // The upload of MyBooty, 359 frames up to and including the C1 jump, as SHA-256 of the frames joined by \n
        const string UploadSha256 = "8416f9dcf0ecdf5beeff6ece4a2fa28053a8b0aa961a731327640694c469a62f";
        const int UploadFrames = 359;
        const string Jump = "005 C100005000000000";
        const string ChipId = "005 C900000000000000";
        const string Erase = "005 C000000000000000";
        const string Checksum = "005 C800000000000000";
        const string Exit = "005 C200000000000000";
        const string FooterAddress = "005 A50007FF80800000";

        static readonly string[] FooterReads = Enumerable.Range(0, 21).Select(i => "005 C70007" + (0xFF85 + 6 * i).ToString("X4") + "000000")
            .Append("005 C70007FFFF000000").ToArray();

        // the first two blocks the bench BIN was programmed with; its last frame repeats bytes of the one before
        static readonly string[] Block0 =
        {
            "005 A500040000800000", "005 00FFFFF7FC000769", "005 070400076A060007", "005 0E6A1A00076A2E00", "005 15076A4200076A56",
            "005 1C00076A6A00076A", "005 237E00076ACE0007", "005 2A6AE200076AF600", "005 31076A9200076AA6", "005 3800076ABA00076C",
            "005 3FFE00076CFE0007", "005 466CFE00076CFE00", "005 4D076CFE00076CFE", "005 5400076CFE00076C", "005 5BFEFFFFFFFF0007",
            "005 626B0A00076B1E00", "005 69076B320004962C", "005 7000076B5A00076B", "005 776E00076B820007", "005 7E6B96076B820007",
        };
        static readonly string[] Block1 =
        {
            "005 A500040080800000", "005 0000076BAA00076B", "005 07BE00076BD20007", "005 0E6BE600076BFA00", "005 15076C0E00076C22",
            "005 1C00076C3600076C", "005 234A00076C5E0007", "005 2A6C7200076C8600", "005 31076C9A00076CAE", "005 3800076CC200076C",
            "005 3FD600076CEA0007", "005 466CEA00076CEA00", "005 4D076CEA00076CEA", "005 5400076CEA00076C", "005 5BEA00076CEA0007",
            "005 626CEA00076CEA00", "005 69076CEA00076CFE", "005 7000076CFE00076C", "005 77FE00076CFE0007", "005 7E6CFE076CFE0007",
        };

        // T5.5 BIN: the two bench blocks, FF, and a footer with made-up identifiers (code end 0x400FF) and
        // the checksum of 0x40000-0x400FF that C8 compares. Another ROM offset makes the FLASH look like a T5.2's.
        static byte[] TestBin(string romOffset = "040000")
        {
            byte[] bin = new byte[0x40000];
            Array.Fill(bin, (byte)0xFF);
            foreach (string[] block in new[] { Block0, Block1 })
            {
                int start = Convert.ToInt32(block[0].Substring(6, 8), 16) - 0x40000;
                foreach (string frame in block.Skip(1))
                {
                    byte[] d = Convert.FromHexString(frame.Substring(4));
                    for (int k = 1; k < 8 && d[0] + k - 1 < 0x80; k++)
                    {
                        bin[start + d[0] + k - 1] = d[k];
                    }
                }
            }
            int p = 0x3FFFB;
            foreach (var (id, text) in new[] { (0x01, "1234567"), (0x02, "7654321"), (0x03, "A55TEST.00C"), (0x04, "B204L TEST"),
                (0xFC, "07FFFF"), (0xFD, romOffset), (0xFE, "0400FF") })
            {
                bin[p] = (byte)text.Length;
                bin[p - 1] = (byte)id;
                for (int j = 0; j < text.Length; j++)
                {
                    bin[p - 2 - j] = (byte)text[j];
                }
                p -= text.Length + 2;
            }
            uint sum = 0;
            for (int a = 0; a < 0x100; a++)
            {
                sum += bin[a];
            }
            bin[0x3FFFC] = (byte)(sum >> 24);
            bin[0x3FFFD] = (byte)(sum >> 16);
            bin[0x3FFFE] = (byte)(sum >> 8);
            bin[0x3FFFF] = (byte)sum;
            return bin;
        }

        static uint BinChecksum(byte[] bin) { return (uint)(bin[0x3FFFC] << 24 | bin[0x3FFFD] << 16 | bin[0x3FFFE] << 8 | bin[0x3FFFF]); }

        // the ECU before the flash: same footer, different code
        static FakeT5 NewEcu(byte[] bin)
        {
            FakeT5 ecu = new FakeT5();
            Array.Copy(bin, 0, ecu.Flash, 0x40000, bin.Length);
            Array.Fill(ecu.Flash, (byte)0x00, 0x40000, 0x100);
            return ecu;
        }

        // builds the device and listener like SetGenericOptions does, then puts the fake ECU in place
        static Trionic5 Connect(FakeT5 ecu, List<string> log)
        {
            Trionic5 t5 = new Trionic5();
            t5.setCANDevice(CANBusAdapter.SLCAN);
            object listener = typeof(Trionic5).GetField("m_canListener", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(t5);
            typeof(ITrionic).GetField("canUsbDevice", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t5, ecu);
            ecu.addListener((ICANListener)listener);
            t5.onCanInfo += (sender, e) => log.Add(e.Info);
            return t5;
        }

        // WriteFlash as the GUI calls it; a conversion question is answered No and its text added to asked
        static WriteFlashResult WriteFlash(FakeT5 ecu, byte[] bin, List<string> log, List<string> asked = null)
        {
            Trionic5 t5 = Connect(ecu, log);
            string file = Path.Combine(Path.GetTempPath(), "t5flashtest-" + Guid.NewGuid().ToString("N") + ".bin");
            File.WriteAllBytes(file, bin);
            Func<string, string, bool> yesNo = UserPrompt.YesNo;
            UserPrompt.YesNo = (text, caption) => { asked?.Add(text); return false; };
            try
            {
                return t5.WriteFlash(file);
            }
            finally
            {
                UserPrompt.YesNo = yesNo;
                File.Delete(file);
            }
        }

        static byte[] FlashContent(FakeT5 ecu) { return ecu.Flash.Skip(0x40000).ToArray(); }

        static string Sha256(IEnumerable<string> frames) { return Convert.ToHexString(SHA256.HashData(Encoding.ASCII.GetBytes(string.Join("\n", frames)))).ToLowerInvariant(); }

        static int CountInARow(List<string> sent, string frame)
        {
            int at = sent.IndexOf(frame);
            int n = 0;
            while (at >= 0 && at + n < sent.Count && sent[at + n] == frame)
            {
                n++;
            }
            return n;
        }

        [TestMethod]
        public void WriteFlashSendsTheFramesOfTheBenchFlash()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Done, WriteFlash(ecu, bin, log));

            List<string> sent = ecu.Sent;
            Assert.AreEqual(UploadSha256, Sha256(sent.Take(UploadFrames)), "MyBooty upload as on the bench");
            Assert.AreEqual(Jump, sent[UploadFrames - 1]);
            // one C9 (identify), the footer, erase, then the blocks
            string[] start = new[] { ChipId }.Concat(FooterReads).Append(Erase).Concat(Block0).Concat(Block1).Append(FooterAddress).ToArray();
            CollectionAssert.AreEqual(start, sent.Skip(UploadFrames).Take(start.Length).ToArray());
            string[] offsets = sent.Skip(UploadFrames + start.Length).Take(19).Select(s => s.Substring(4, 2)).ToArray();
            CollectionAssert.AreEqual(Block0.Skip(1).Select(s => s.Substring(4, 2)).ToArray(), offsets);
            // checksum, then GetECUInfo: footer, C9, checksum, exit
            string[] end = new[] { Checksum }.Concat(FooterReads).Append(ChipId).Append(Checksum).Append(Exit).ToArray();
            CollectionAssert.AreEqual(end, sent.Skip(UploadFrames + start.Length + 19).ToArray());

            CollectionAssert.AreEqual(bin, FlashContent(ecu));
            CollectionAssert.AreEquivalent(new uint[] { 0x40000, 0x40080, 0x7FF80 }, ecu.Programmed.Keys.ToArray(), "FF blocks are skipped");
            Assert.IsTrue(ecu.Programmed.Values.All(n => n == 1));
            CollectionAssert.Contains(log, "FLASH Checksum OK: " + BinChecksum(bin).ToString("X8"));
            CollectionAssert.Contains(log, "!!! SUCCESS !!!");
            CollectionAssert.Contains(log, "ECU is reset");
        }

        [TestMethod]
        public void WriteFlashStopsWhenTheBootloaderUploadFails()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            byte[] before = FlashContent(ecu);
            ecu.Lost = s => ecu.Sent.Count > 20; // the ECU goes quiet during the upload
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            Assert.AreEqual(ChipId, ecu.Sent[ecu.Sent.Count - 1], "nothing after the bootloader check");
            Assert.IsFalse(ecu.Sent.Contains(Erase));
            CollectionAssert.Contains(log, "!!! ERROR !!! The bootloader is not answering, this attempt did not touch the FLASH");
            // an unreachable MyBooty left running by a failed flash looks the same: never suggest it is safe to switch off
            CollectionAssert.Contains(log, "If an earlier FLASH attempt failed, don't switch the ECU off: check the connection and retry !!!");
            CollectionAssert.AreEqual(before, FlashContent(ecu));
        }

        [TestMethod]
        public void WriteFlashUsesTheBootloaderAnEarlierFlashLeftRunning()
        {
            // a flash that failed after the first block: MyBooty still runs and refuses the upload
            byte[] bin = TestBin();
            FakeT5 ecu = new FakeT5();
            Array.Fill(ecu.Flash, (byte)0xFF, 0x40000, 0x40000);
            Array.Copy(bin, 0, ecu.Flash, 0x40000, 0x80);
            ecu.BootyRunning = true;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Done, WriteFlash(ecu, bin, log));

            Assert.IsTrue(log.Any(s => s.StartsWith("Could not upload bootloader...")), "MyBooty refuses to program the upload at 0x5000");
            CollectionAssert.Contains(log, "The bootloader is still running from an earlier attempt, using it");
            CollectionAssert.Contains(log, "!!! SUCCESS !!!");
            CollectionAssert.AreEqual(bin, FlashContent(ecu));
            Assert.IsTrue(ecu.Programmed.Keys.All(a => a >= 0x40000));
        }

        [TestMethod]
        public void WriteFlashSaysDontSwitchOffWhenAnEarlierFlashsBootloaderIsOutOfReach()
        {
            // a flash failed after the first block because CAN went down, and it is still down for the retry
            byte[] bin = TestBin();
            FakeT5 ecu = new FakeT5();
            Array.Fill(ecu.Flash, (byte)0xFF, 0x40000, 0x40000);
            Array.Copy(bin, 0, ecu.Flash, 0x40000, 0x80);
            ecu.BootyRunning = true;
            ecu.Lost = s => true;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            Assert.AreEqual(ChipId, ecu.Sent[ecu.Sent.Count - 1], "nothing after the bootloader check");
            Assert.IsFalse(log.Any(s => s.Contains("not running") || s.Contains("was not touched")), "MyBooty runs, the FLASH is half written");
            CollectionAssert.Contains(log, "If an earlier FLASH attempt failed, don't switch the ECU off: check the connection and retry !!!");
        }

        // No to the conversion question: nothing erased, and the bootloader this attempt uploaded is exited, as master did
        [TestMethod]
        [DataRow("040000", 0x20000, "Do you want to upload a T5.2 BIN file to your T5.5 ECU")]
        [DataRow("060000", 0x40000, "Do you want to upload a T5.5 BIN file to your ECU that has been used as a T5.2?")]
        public void WriteFlashIsCancelledWhenTheUserSaysNo(string ecuRomOffset, int fileLength, string question)
        {
            FakeT5 ecu = NewEcu(TestBin(ecuRomOffset));
            byte[] before = FlashContent(ecu);
            List<string> log = new List<string>();
            List<string> asked = new List<string>();

            Assert.AreEqual(WriteFlashResult.Cancelled, WriteFlash(ecu, TestBin().Take(fileLength).ToArray(), log, asked));

            CollectionAssert.AreEqual(new[] { question }, asked);
            CollectionAssert.AreEqual(new[] { ChipId }.Concat(FooterReads).Append(Exit).ToArray(), ecu.Sent.Skip(UploadFrames).ToArray(), "identify, footer, exit: no erase");
            CollectionAssert.Contains(log, "ECU is reset");
            Assert.IsFalse(log.Any(s => s.StartsWith("Starting FLASH update session")));
            CollectionAssert.AreEqual(before, FlashContent(ecu));
        }

        // a file that doesn't fit the ECU: nothing asked, nothing erased, the bootloader exited
        [TestMethod]
        [DataRow("040000", "000004000001A7", 0x30000, "Not a Trionic 5.5 BIN File!")]
        [DataRow("060000", "000006000089B8", 0x40000, "Not a Trionic 5.2 BIN File!")]
        [DataRow("060000", "000004000001A7", 0x30000, "Not a Trionic BIN File!")]
        public void WriteFlashIsCancelledForAFileThatDoesNotFitTheECU(string ecuRomOffset, string identity, int fileLength, string message)
        {
            FakeT5 ecu = NewEcu(TestBin(ecuRomOffset));
            ecu.Identity = identity;
            byte[] before = FlashContent(ecu);
            List<string> log = new List<string>();
            List<string> asked = new List<string>();

            Assert.AreEqual(WriteFlashResult.Cancelled, WriteFlash(ecu, new byte[fileLength], log, asked));

            Assert.AreEqual(0, asked.Count);
            CollectionAssert.Contains(log, message);
            CollectionAssert.AreEqual(new[] { ChipId }.Concat(FooterReads).Append(Exit).ToArray(), ecu.Sent.Skip(UploadFrames).ToArray(), "identify, footer, exit: no erase");
            CollectionAssert.Contains(log, "ECU is reset");
            CollectionAssert.AreEqual(before, FlashContent(ecu));
        }

        [TestMethod]
        public void WriteFlashCancelledOnTheBootloaderOfAnEarlierAttemptDoesNotResetTheECU()
        {
            // an earlier flash failed and left MyBooty running, the FLASH maybe half written; the retry picks a file
            // that doesn't fit: a reset now would leave the ECU to BDM
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            ecu.BootyRunning = true;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Cancelled, WriteFlash(ecu, new byte[0x30000], log));

            Assert.IsFalse(ecu.Sent.Contains(Erase));
            Assert.IsFalse(ecu.Sent.Contains(Exit), "no reset");
            Assert.IsTrue(ecu.BootyRunning);
            CollectionAssert.Contains(log, "The bootloader is still running from an earlier attempt, using it");
            CollectionAssert.Contains(log, "ECU not reset, the bootloader from the earlier attempt keeps running");
            CollectionAssert.Contains(log, "If that FLASH attempt failed, don't switch the ECU off: retry with a BIN file for this ECU !!!");
            Assert.IsFalse(log.Contains("ECU is reset"));
        }

        [TestMethod]
        public void WriteFlashResendsALostAddressCommand()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            int n = 0;
            ecu.Lost = s => s == Block1[0] && n++ == 0;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Done, WriteFlash(ecu, bin, log));

            Assert.AreEqual(2, CountInARow(ecu.Sent, Block1[0]));
            CollectionAssert.AreEqual(bin, FlashContent(ecu), "block 1 programmed at its own address");
            Assert.AreEqual(1, ecu.Programmed[0x40080]);
        }

        [TestMethod]
        public void WriteFlashFailsWhenTheAddressCommandStaysUnanswered()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            bool dead = false;
            ecu.Mute = s => dead |= s == Block1[0]; // nothing gets back from here on
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            Assert.AreEqual(3, CountInARow(ecu.Sent, Block1[0]), "sent and resent twice");
            Assert.AreEqual(Block1[0], ecu.Sent[ecu.Sent.Count - 1], "no data frames after an unanswered address");
            CollectionAssert.Contains(log, "FLASHing Failed after: 0x000080 Bytes, the bootloader did not answer !!!");
            CollectionAssert.Contains(log, "!!! FAILURE !!! Could not program the FLASH in your ECU :-(");
            Assert.IsFalse(log.Contains("!!! SUCCESS !!!"));
        }

        [TestMethod]
        public void WriteFlashResendsALostDataFrame()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            int n = 0;
            ecu.Lost = s => s == Block0[2] && n++ == 0;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Done, WriteFlash(ecu, bin, log));

            Assert.AreEqual(2, CountInARow(ecu.Sent, Block0[2]));
            CollectionAssert.AreEqual(bin, FlashContent(ecu));
            Assert.AreEqual(1, ecu.Programmed[0x40000]);
        }

        [TestMethod]
        public void WriteFlashFailsWhenADataFrameStaysUnanswered()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            bool dead = false;
            ecu.Mute = s => dead |= s == Block0[2];
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            Assert.AreEqual(3, CountInARow(ecu.Sent, Block0[2]));
            Assert.AreEqual(Block0[2], ecu.Sent[ecu.Sent.Count - 1]);
            CollectionAssert.Contains(log, "FLASHing Failed after: 0x000000 Bytes, the bootloader did not answer !!!");
            Assert.IsFalse(ecu.Programmed.ContainsKey(0x40000));
            Assert.IsFalse(log.Contains("!!! SUCCESS !!!"));
        }

        [TestMethod]
        public void WriteFlashNeverResendsTheFrameThatProgramsABlock()
        {
            // the ECU programs block 0, its answer is lost: a resend would program the block a second time
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            ecu.Mute = s => s == Block0[19];
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            Assert.AreEqual(1, ecu.Sent.Count(s => s == Block0[19]));
            Assert.AreEqual(Block0[19], ecu.Sent[ecu.Sent.Count - 1], "nothing after it, no checksum");
            Assert.AreEqual(1, ecu.Programmed[0x40000]);
            CollectionAssert.Contains(log, "FLASHing Failed after: 0x000000 Bytes, the bootloader did not answer !!!");
            Assert.IsFalse(log.Contains("!!! SUCCESS !!!"));
        }

        [TestMethod]
        public void WriteFlashReportsAnInterruptedFlashAsAFailure()
        {
            // the adapter goes away mid-block: the exception must not skip the failure and the retry advice
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            ecu.Unplug = s => s == Block1[3];
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            Assert.AreEqual(Block1[3], ecu.Sent[ecu.Sent.Count - 1], "nothing after it, no checksum and no reset");
            CollectionAssert.Contains(log, "FLASHing was interrupted: The port is closed.");
            CollectionAssert.Contains(log, "!!! FAILURE !!! Could not program the FLASH in your ECU :-(");
            CollectionAssert.Contains(log, "Don't switch the ECU off before you retry !!!");
            Assert.IsFalse(log.Contains("!!! SUCCESS !!!"));
        }

        [TestMethod]
        public void WriteFlashSkipsALateAnswer()
        {
            // block 1's address is answered after the wait timed out, so the resend gets two answers
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            int n = 0;
            ecu.Late = s => s == Block1[0] && n++ == 0;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Done, WriteFlash(ecu, bin, log));

            Assert.AreEqual(2, CountInARow(ecu.Sent, Block1[0]));
            CollectionAssert.AreEqual(bin, FlashContent(ecu));
        }

        [TestMethod]
        public void WriteFlashReportsAnUnansweredChecksumAsNotRead()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            ecu.Mute = s => s == Checksum;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Failed, WriteFlash(ecu, bin, log));

            CollectionAssert.Contains(log, "Could NOT read the FLASH Checksum, the bootloader did not answer !!!");
            Assert.IsFalse(log.Any(s => s.StartsWith("FLASH Checksum OK")));
            Assert.IsFalse(log.Contains("!!! SUCCESS !!!"));
            Assert.AreEqual(Checksum, ecu.Sent[ecu.Sent.Count - 1], "no reset after a failed check");
        }

        [TestMethod]
        public void WriteFlashTellsWhenTheResetIsNotConfirmed()
        {
            byte[] bin = TestBin();
            FakeT5 ecu = NewEcu(bin);
            ecu.Mute = s => s == Exit;
            List<string> log = new List<string>();

            Assert.AreEqual(WriteFlashResult.Done, WriteFlash(ecu, bin, log));

            Assert.AreEqual(Exit, ecu.Sent[ecu.Sent.Count - 1]);
            CollectionAssert.Contains(log, "Bootloader did not confirm the reset, switch the ECU off and on");
            Assert.IsFalse(log.Contains("ECU is reset"));
        }

        [TestMethod]
        public void UploadBootLoaderFailsWhenTheFirstFrameOfARecordIsUnanswered()
        {
            // the first data frame of a record echoes 00: unanswered, it must not pass as an answer
            FakeT5 ecu = new FakeT5();
            string firstFrame = "005 004EB8500C4EB850";
            ecu.Mute = s => s == firstFrame;
            Trionic5 t5 = Connect(ecu, new List<string>());

            Assert.IsFalse(t5.UploadBootLoader());
            Assert.AreEqual(firstFrame, ecu.Sent[ecu.Sent.Count - 1]);
        }
    }
}
