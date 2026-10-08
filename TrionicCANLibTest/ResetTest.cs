using System;
using System.Collections.Generic;
using System.Reflection;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.API;
using TrionicCANLib.CAN;
using TrionicCANLib.KWP;

namespace TrionicCANLibTest
{
    [TestClass]
    public class ResetTest
    {
        // Fake CAN bus: Answer maps a sent frame ("ID DATA") to the reply frame ("ID DATA") or null.
        class FakeBus : ICANDevice
        {
            public readonly List<string> Sent = new List<string>();
            public Func<string, string> Answer = s => null;

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
                string reply = Answer(frame);
                if (reply != null)
                {
                    CANMessage msg = new CANMessage(Convert.ToUInt32(reply.Substring(0, 3), 16), 0, 8);
                    msg.setCanData(Convert.FromHexString(reply.Substring(4)), 8);
                    receivedMessage(msg);
                }
                return true;
            }
        }

        // builds the device and listener like SetGenericOptions does, then puts the fake bus in place
        static FakeBus UseFakeBus(ITrionic t)
        {
            t.setCANDevice(CANBusAdapter.SLCAN);
            FakeBus bus = new FakeBus();
            object listener = t.GetType().GetField("m_canListener", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(t);
            typeof(ITrionic).GetField("canUsbDevice", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t, bus);
            bus.addListener((ICANListener)listener);
            return bus;
        }

        const string LegionPing = "7E0 EFBE000000003366";
        const string ResetCpid = "7E0 02AE160000000000";
        const string ReturnToNormal = "7E0 0120000000000000";

        [TestMethod]
        public void T8ResetSendsDeviceControl16()
        {
            Trionic8 t8 = new Trionic8();
            FakeBus bus = UseFakeBus(t8);
            bus.Answer = s => s == ResetCpid ? "7E8 02EE160000000000" : null;

            Assert.IsTrue(t8.ResetECU());
            CollectionAssert.AreEqual(new[] { LegionPing, LegionPing, LegionPing, LegionPing, ResetCpid }, bus.Sent);
        }

        [TestMethod]
        public void T8ResetRefusedWhileEngineRuns()
        {
            Trionic8 t8 = new Trionic8();
            FakeBus bus = UseFakeBus(t8);
            bus.Answer = s => s == ResetCpid ? "7E8 037FAE2200000000" : null;

            Assert.IsFalse(t8.ResetECU());
            Assert.AreEqual(ResetCpid, bus.Sent[bus.Sent.Count - 1]);
            Assert.IsFalse(bus.Sent.Contains(ReturnToNormal), "a refused reset must not be followed by anything else");
        }

        [TestMethod]
        public void T8ResetFailsWithoutReply()
        {
            Trionic8 t8 = new Trionic8();
            FakeBus bus = UseFakeBus(t8);

            Assert.IsFalse(t8.ResetECU());
            Assert.AreEqual(ResetCpid, bus.Sent[bus.Sent.Count - 1]);
        }

        [TestMethod]
        public void T8ResetExitsRunningLegion()
        {
            // a legion loader left running gets the exit request it gets after a flash, no CPID $16
            Trionic8 t8 = new Trionic8();
            FakeBus bus = UseFakeBus(t8);
            bus.Answer = s => s == LegionPing ? "7E8 DEADF00F00000000" : s == ReturnToNormal ? "7E8 0160000000000000" : null;

            Assert.IsTrue(t8.ResetECU());
            CollectionAssert.AreEqual(new[] { LegionPing, ReturnToNormal }, bus.Sent);
        }

        // MyBooty as the T5 sees it: every command echoed with status 00, the rest 08
        static string MyBooty(string frame)
        {
            return frame.StartsWith("005 ") ? "00C " + frame.Substring(4, 2) + "00080808080808" : null;
        }

        [TestMethod]
        public void T5ResetUploadsBootloaderThenExits()
        {
            Trionic5 t5 = new Trionic5();
            FakeBus bus = UseFakeBus(t5);
            bus.Answer = MyBooty;

            Assert.IsTrue(t5.ResetECU());
            Assert.AreEqual("005 A500000000000000", bus.Sent[0], "bootloader upload starts with the S0 address command");
            Assert.AreEqual("005 C100005000000000", bus.Sent[bus.Sent.Count - 2], "jump to MyBooty at 0x5000");
            Assert.AreEqual("005 C200000000000000", bus.Sent[bus.Sent.Count - 1]);
        }

        [TestMethod]
        public void T5ResetFailsWhenExitIsNotConfirmed()
        {
            Trionic5 t5 = new Trionic5();
            FakeBus bus = UseFakeBus(t5);
            bus.Answer = s => s.StartsWith("005 C2") ? null : MyBooty(s);

            Assert.IsFalse(t5.ResetECU());
            Assert.AreEqual("005 C200000000000000", bus.Sent[bus.Sent.Count - 1]);
        }

        // T7 on the P-bus. Like the firmware: 0x81 only answered without a live session, 11 01 in a
        // session still in the EOL state of a flash ends that state with 7F 11 22, otherwise 51 81 and
        // the ECU reboots (session gone, 0x82 unanswered).
        class FakeT7
        {
            public readonly FakeBus Bus = new FakeBus();
            public bool SessionAlive, Eol, Mute;
            public int Resets;

            public FakeT7()
            {
                Bus.Answer = Answer;
            }

            string Answer(string frame)
            {
                if (frame == "220 3F81001102400000" && !SessionAlive)
                {
                    SessionAlive = true;
                    return "238 40BF21C100110258";
                }
                if (frame == ResetRequest && SessionAlive && !Mute)
                {
                    if (Eol)
                    {
                        Eol = false;
                        return "258 C0BF037F11220000";
                    }
                    SessionAlive = false;
                    Resets++;
                    return "258 C0BF025181000000";
                }
                if (frame == "240 40A1018200000000" && SessionAlive)
                {
                    SessionAlive = false;
                    return "258 C0BF01C200000000";
                }
                return null;
            }

            public Trionic7 Open()
            {
                Trionic7 t7 = new Trionic7();
                // builds the KWP stack like SetGenericOptions does, then puts the fake bus under the KWPHandler singleton
                t7.setCANDevice(CANBusAdapter.SLCAN);
                KWPCANDevice kwp = new KWPCANDevice() { Latency = Latency.Low };
                kwp.setCANDevice(Bus);
                KWPHandler.setKWPDevice(kwp);
                Assert.IsTrue(t7.openDevice());
                return t7;
            }

            public int Count(string frame) { return Bus.Sent.FindAll(s => s == frame).Count; }
        }

        const string ResetRequest = "240 40A1031101000000";
        const string StopRequest = "240 40A1018200000000";

        [TestMethod]
        public void T7ResetSendsEcuReset()
        {
            FakeT7 ecu = new FakeT7();
            Trionic7 t7 = ecu.Open();
            Assert.IsTrue(t7.ResetECU());
            t7.Cleanup();
            Assert.AreEqual(1, ecu.Resets);
            Assert.AreEqual(1, ecu.Count(ResetRequest));
            // the session went with the reboot: Cleanup's stop only waited for its reply timeout
            Assert.AreEqual(0, ecu.Count(StopRequest), "no stop to a rebooting ECU");
        }

        [TestMethod]
        public void T7ResetInFlashSessionNeedsSecondRequest()
        {
            // reset straight after a flash, before its session ended: the first 11 01 only ends the EOL state
            FakeT7 ecu = new FakeT7();
            Trionic7 t7 = ecu.Open();
            ecu.Eol = true;
            Assert.IsTrue(t7.ResetECU());
            t7.Cleanup();
            Assert.AreEqual(1, ecu.Resets);
            Assert.AreEqual(2, ecu.Count(ResetRequest));
            Assert.AreEqual(0, ecu.Count(StopRequest), "no stop to a rebooting ECU");
        }

        [TestMethod]
        public void T7ResetFailsWithoutReply()
        {
            FakeT7 ecu = new FakeT7();
            Trionic7 t7 = ecu.Open();
            ecu.Mute = true;
            Assert.IsFalse(t7.ResetECU());
            t7.Cleanup();
            Assert.AreEqual(0, ecu.Resets);
            Assert.IsFalse(ecu.SessionAlive, "Cleanup still ends the session");
            Assert.AreEqual(1, ecu.Count(StopRequest));
        }

        [TestMethod]
        public void T5ResetStopsWhenBootloaderUploadFails()
        {
            Trionic5 t5 = new Trionic5();
            FakeBus bus = UseFakeBus(t5);

            Assert.IsFalse(t5.ResetECU());
            CollectionAssert.AreEqual(new[] { "005 A500000000000000" }, bus.Sent, "no C2 without a running bootloader");
        }
    }
}
