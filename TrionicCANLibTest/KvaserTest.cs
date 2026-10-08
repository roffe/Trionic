using System;
using canlibCLSNET;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.CAN;

namespace TrionicCANLibTest
{
    // Never opens a channel: only enumeration and calls on an invalid handle.
    [TestClass]
    public class KvaserTest
    {
        [TestMethod]
        public void StatusValuesMatchCanstatH()
        {
            Assert.AreEqual(0, (int)Canlib.canStatus.canOK);
            Assert.AreEqual(-2, (int)Canlib.canStatus.canERR_NOMSG);
            Assert.AreEqual(-10, (int)Canlib.canStatus.canERR_INVHANDLE);
            Assert.AreEqual(-16, (int)Canlib.canStatus.canERR_DYNALOAD);
            Assert.AreEqual(-41, (int)Canlib.canStatus.canERR__RESERVED);
        }

        [TestMethod]
        public void GetAdapterNamesNeverThrows()
        {
            // empty without CANlib (macOS, no driver) and with only virtual channels
            Assert.IsNotNull(KvaserCANDevice.GetAdapterNames());
        }

        [TestMethod]
        public void InvalidHandleRoundTripsThroughMarshaling()
        {
            try
            {
                Canlib.canInitializeLibrary();
            }
            catch (DllNotFoundException)
            {
                Assert.Inconclusive("CANlib not installed");
            }

            byte[] msg = new byte[8];
            Assert.AreEqual(Canlib.canStatus.canERR_INVHANDLE, Canlib.canReadWait(-1, out int id, msg, out int dlc, out int flag, out long time, 1));
            Assert.AreEqual(0, id);
            Assert.AreEqual(0, flag);
            Assert.AreEqual(Canlib.canStatus.canERR_INVHANDLE, Canlib.canWrite(-1, 0x7E0, msg, 8, 0));
            Assert.AreEqual(Canlib.canStatus.canERR_INVHANDLE, Canlib.canSetBusParams(-1, Canlib.canBITRATE_500K, 0, 0, 0, 0, 0));

            Assert.AreEqual(Canlib.canStatus.canOK, Canlib.canGetNumberOfChannels(out int channels));
            for (int i = 0; i < channels; i++)
            {
                Assert.AreEqual(Canlib.canStatus.canOK, Canlib.canGetChannelData(i, Canlib.canCHANNELDATA_CHANNEL_NAME, out object name));
                Assert.IsInstanceOfType(name, typeof(string));
                Assert.AreEqual(Canlib.canStatus.canOK, Canlib.canGetChannelData(i, Canlib.canCHANNELDATA_CHANNEL_CAP, out object cap));
                Assert.IsInstanceOfType(cap, typeof(uint));
            }
        }
    }
}
