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
    public class T7SessionTest
    {
        // Fake P-bus with a T7 on it. Like the firmware, startCommunication on 0x220 is only
        // answered while no session is alive, and stopCommunication on 0x240 ends the session.
        class FakeT7Bus : ICANDevice
        {
            public readonly List<string> Sent = new List<string>();
            public bool SessionAlive;
            public bool RefuseAcks;    // the adapter doesn't take 0x266 frames
            private bool m_open;

            public override OpenResult open() { m_open = true; return OpenResult.OK; }
            public override CloseResult close() { m_open = false; return CloseResult.OK; }
            public override bool isOpen() { return m_open; }
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
                Sent.Add(a_message.getID().ToString("X3") + " " + Convert.ToHexString(d));
                if (a_message.getID() == 0x266 && RefuseAcks)
                    return false;
                if (a_message.getID() == 0x220 && d[1] == 0x81 && !SessionAlive)
                {
                    SessionAlive = true;
                    Reply(0x238, "40BF21C100110258");
                }
                else if (a_message.getID() == 0x240 && SessionAlive && d[3] == 0x82)
                {
                    SessionAlive = false;
                    Reply(0x258, "C0BF01C200000000");
                }
                else if (a_message.getID() == 0x240 && SessionAlive && d[3] == 0x21 && d[4] == 0xF0)
                {
                    // two-frame reply 61 F0 AA BB CC DD EE FF
                    Reply(0x258, "C1BF0861F0AABBCC");
                }
                else if (a_message.getID() == 0x266 && d[3] == 0x81)
                {
                    // the ECU sends its next frame when it sees the ack; the CombiAdapter and the
                    // CANUSB VCP can deliver it before the ack's sendMessage has returned
                    Reply(0x258, "80BFDDEEFF000000");
                }
                return true;
            }

            private void Reply(uint a_id, string a_hex)
            {
                CANMessage msg = new CANMessage(a_id, 0, 8);
                msg.setCanData(Convert.FromHexString(a_hex), 8);
                receivedMessage(msg);
            }
        }

        static KWPCANDevice NewKwpDevice(FakeT7Bus bus)
        {
            KWPCANDevice kwp = new KWPCANDevice();
            kwp.setCANDevice(bus);
            KWPHandler.setKWPDevice(kwp);
            return kwp;
        }

        // a stopCommunication that throws, as KWPCANDevice did when the adapter refused the 0x266 ack
        class ThrowingStopDevice : KWPCANDevice
        {
            public override bool stopSession() { throw new Exception("Error sending ack"); }
        }

        // Trionic7 as SetGenericOptions leaves it, with a_kwp over a_bus under the KWPHandler singleton and
        // a separate open fake adapter as its canUsbDevice, so the device and the adapter close are told apart
        static Trionic7 NewTrionic7(KWPCANDevice a_kwp, FakeT7Bus a_bus, FakeT7Bus a_adapter)
        {
            Trionic7 t7 = new Trionic7();
            t7.setCANDevice(CANBusAdapter.SLCAN);
            a_kwp.setCANDevice(a_bus);
            KWPHandler.setKWPDevice(a_kwp);
            typeof(ITrionic).GetField("canUsbDevice", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(t7, a_adapter);
            a_adapter.open();
            return t7;
        }

        [TestMethod]
        public void CleanupEndsSessionSoTheNextOpenWorks()
        {
            // the GUI's Get ECU info pressed twice: before the fix the second openDevice got no
            // 0x238 because the first session was still alive
            FakeT7Bus bus = new FakeT7Bus();
            Trionic7 t7 = new Trionic7();
            for (int press = 0; press < 2; press++)
            {
                // builds the KWP stack and its T7Flasher like SetGenericOptions does, then swaps
                // the CAN device under the KWPHandler singleton for the fake bus
                t7.setCANDevice(CANBusAdapter.SLCAN);
                NewKwpDevice(bus);
                Assert.IsTrue(t7.openDevice(), "openDevice " + press);
                Assert.IsTrue(bus.SessionAlive);
                t7.Cleanup();
                Assert.IsFalse(bus.SessionAlive, "Cleanup must end the session");
            }
            CollectionAssert.AreEqual(new[]
            {
                "220 3F81001102400000", "240 40A1018200000000", "266 40A13F8000000000",
                "220 3F81001102400000", "240 40A1018200000000", "266 40A13F8000000000",
            }, bus.Sent);

            // frmMain_FormClosing calls Cleanup again: nothing is left to stop
            t7.Cleanup();
            Assert.AreEqual(6, bus.Sent.Count);
        }

        [TestMethod]
        public void OpenEndsASessionTheEcuStillHolds()
        {
            // a session left behind by another program (or by a stop whose ack got lost)
            FakeT7Bus bus = new FakeT7Bus() { SessionAlive = true };
            Trionic7 t7 = new Trionic7();
            t7.setCANDevice(CANBusAdapter.SLCAN);
            NewKwpDevice(bus);
            Assert.IsTrue(t7.openDevice());
            t7.Cleanup();
            CollectionAssert.AreEqual(new[]
            {
                "220 3F81001102400000", "240 40A1018200000000", "266 40A13F8000000000", "220 3F81001102400000",
                "240 40A1018200000000", "266 40A13F8000000000",
            }, bus.Sent);
        }

        [TestMethod]
        public void StopIsNotSentTwiceAfterReadEnd()
        {
            // T7Flasher ends a read with sendDataTransferExitRequest, which is stopCommunication
            FakeT7Bus bus = new FakeT7Bus();
            NewKwpDevice(bus);
            KWPHandler handler = KWPHandler.getInstance();
            Assert.IsTrue(handler.openDevice());
            Assert.IsTrue(handler.startSession());
            Assert.IsTrue(handler.sendDataTransferExitRequest());
            Assert.IsFalse(bus.SessionAlive);
            Assert.IsFalse(handler.stopSession());
            CollectionAssert.AreEqual(new[] { "220 3F81001102400000", "240 40A1028200000000", "266 40A13F8000000000" }, bus.Sent);

            // and never into a closed device
            Assert.IsTrue(handler.startSession());
            handler.closeDevice();
            Assert.IsFalse(handler.stopSession());
            Assert.AreEqual(4, bus.Sent.Count);
            Assert.IsTrue(bus.SessionAlive);
        }

        [TestMethod]
        public void NextReplyFrameArrivingDuringTheAckIsKept()
        {
            // bench: about once per full read a 21 F0 reply lost its second frame and was resent,
            // because the wait for it was armed after the ack that triggers it
            FakeT7Bus bus = new FakeT7Bus();
            KWPCANDevice kwp = NewKwpDevice(bus);
            KWPHandler handler = KWPHandler.getInstance();
            Assert.IsTrue(handler.openDevice());
            Assert.IsTrue(handler.startSession());
            KWPReply reply;
            Assert.AreEqual(RequestResult.NoError, kwp.sendRequest(new KWPRequest(0x21, 0xF0), out reply));
            Assert.AreEqual(0x61, reply.getMode());
            CollectionAssert.AreEqual(new byte[] { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }, reply.getData());
            CollectionAssert.AreEqual(new[] { "220 3F81001102400000", "240 40A10221F0000000", "266 40A13F8100000000", "266 40A13F8000000000" }, bus.Sent);
            handler.closeDevice();
        }

        [TestMethod]
        public void RefusedAckFailsTheRequestInsteadOfThrowing()
        {
            // sendAck threw "Error sending ack": out of KWPHandler.stopSession, so Cleanup skipped closing
            // the device and the adapter, and out of the flasher thread, which ended the process
            FakeT7Bus bus = new FakeT7Bus();
            KWPCANDevice kwp = NewKwpDevice(bus);
            KWPHandler handler = KWPHandler.getInstance();
            Assert.IsTrue(handler.openDevice());
            Assert.IsTrue(handler.startSession());
            bus.RefuseAcks = true;
            KWPReply reply;
            Assert.AreEqual(RequestResult.ErrorSending, kwp.sendRequest(new KWPRequest(0x21, 0xF0), out reply));
            Assert.IsFalse(handler.stopSession(), "a stop whose ack the adapter refused is not confirmed");
            handler.closeDevice();
        }

        [TestMethod]
        public void CleanupClosesDeviceAndAdapterWhenStopFails()
        {
            foreach (bool throwing in new[] { false, true })
            {
                FakeT7Bus bus = new FakeT7Bus(), adapter = new FakeT7Bus();
                Trionic7 t7 = NewTrionic7(throwing ? new ThrowingStopDevice() : new KWPCANDevice(), bus, adapter);
                Assert.IsTrue(t7.openDevice());
                bus.RefuseAcks = true;
                t7.Cleanup();
                Assert.IsFalse(bus.isOpen(), "the KWP device must be closed, throwing stop: " + throwing);
                Assert.IsFalse(adapter.isOpen(), "the adapter must be closed, throwing stop: " + throwing);
                if (!throwing)
                    Assert.AreEqual("240 40A1018200000000", bus.Sent.FindLast(f => f.StartsWith("240")), "the stop was tried");
            }
        }

        [TestMethod]
        public void OpenDoesNotThrowWhenTheStopAndRetryFails()
        {
            // the ECU still holds a session, so startCommunication goes unanswered and openDevice sends a
            // stop: a stop that threw went straight into the GUI's click handler
            FakeT7Bus bus = new FakeT7Bus() { SessionAlive = true }, adapter = new FakeT7Bus();
            Trionic7 t7 = NewTrionic7(new ThrowingStopDevice() { Latency = Latency.Low }, bus, adapter);
            Assert.IsFalse(t7.openDevice());
            Assert.IsFalse(bus.isOpen(), "a failed open closes the KWP device");
            Assert.IsFalse(adapter.isOpen(), "a failed open closes the adapter");
            t7.Cleanup();
        }
    }
}
