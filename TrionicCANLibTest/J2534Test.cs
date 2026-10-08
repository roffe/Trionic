using System;
using System.IO;
using System.Runtime.InteropServices;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.CAN.J2534;

namespace TrionicCANLibTest
{
    [TestClass]
    public class J2534Test
    {
        [TestMethod]
        public void PassThruMsgMatchesTheCLayout()
        {
            // Six 32-bit fields then 4128 data bytes, the same on Windows x86/x64 and the Linux drivers
            Assert.AreEqual(4152, Marshal.SizeOf<PassThruMsg>());
            Assert.AreEqual(12, (int)Marshal.OffsetOf<PassThruMsg>("Timestamp"));
            Assert.AreEqual(16, (int)Marshal.OffsetOf<PassThruMsg>("DataSize"));
            Assert.AreEqual(20, (int)Marshal.OffsetOf<PassThruMsg>("ExtraDataIndex"));
            Assert.AreEqual(24, (int)Marshal.OffsetOf<PassThruMsg>("Data"));
        }

        [TestMethod]
        public void PassThruMsgRoundTripsThroughUnmanagedMemory()
        {
            byte[] frame = { 0x00, 0x00, 0x02, 0x40, 0x3F, 0x81 };
            IntPtr ptr = new PassThruMsg(ProtocolID.CAN, TxFlag.CAN_29BIT_ID, frame).ToIntPtr();
            try
            {
                Assert.AreEqual(5, Marshal.ReadInt32(ptr, 0));
                Assert.AreEqual(0x100, Marshal.ReadInt32(ptr, 8));
                Assert.AreEqual(6, Marshal.ReadInt32(ptr, 16));
                Assert.AreEqual(6, Marshal.ReadInt32(ptr, 20));
                Assert.AreEqual(0x40, Marshal.ReadByte(ptr, 24 + 3));

                PassThruMsg msg = ptr.AsStruct<PassThruMsg>();
                Assert.AreEqual(ProtocolID.CAN, msg.ProtocolID);
                CollectionAssert.AreEqual(frame, msg.GetBytes());
            }
            finally
            {
                Marshal.FreeHGlobal(ptr);
            }
        }

        [TestMethod]
        public void ListJsonDevicesReadsThePassthruConvention()
        {
            string dir = Path.Combine(Path.GetTempPath(), "j2534test-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                string lib = Path.Combine(dir, "libfake.so");
                File.WriteAllText(lib, "");
                File.WriteAllText(Path.Combine(dir, "b.json"), "{\"NAME\":\"Fake\",\"VENDOR\":\"Acme\",\"CAN\":true,\"FUNCTION_LIB\":\"" + lib.Replace("\\", "\\\\") + "\"}");
                File.WriteAllText(Path.Combine(dir, "a.json"), "{\"NAME\":\"KLine\",\"CAN\":false,\"FUNCTION_LIB\":\"" + lib.Replace("\\", "\\\\") + "\"}");
                File.WriteAllText(Path.Combine(dir, "missing.json"), "{\"NAME\":\"Gone\",\"CAN\":true,\"FUNCTION_LIB\":\"/nonexistent/lib.so\"}");
                File.WriteAllText(Path.Combine(dir, "broken.json"), "{not json");
                File.WriteAllText(Path.Combine(dir, "array.json"), "[]");
                File.WriteAllText(Path.Combine(dir, "notes.txt"), "ignored");

                var devices = J2534Detect.ListJsonDevices(dir);

                Assert.AreEqual(2, devices.Count);
                Assert.AreEqual("KLine", devices[0].Name);
                Assert.IsFalse(devices[0].IsCANSupported);
                Assert.AreEqual("Acme Fake", devices[1].Name);
                Assert.AreEqual("Acme", devices[1].Vendor);
                Assert.AreEqual(lib, devices[1].FunctionLibrary);
                Assert.IsTrue(devices[1].IsCANSupported);
                Assert.AreEqual(0, J2534Detect.ListJsonDevices(Path.Combine(dir, "nope")).Count);
            }
            finally
            {
                Directory.Delete(dir, true);
            }
        }

        [TestMethod]
        public void LoadLibraryFailsSoftly()
        {
            J2534Extended passThru = new J2534Extended();
            Assert.IsFalse(passThru.LoadLibrary(new J2534Device { FunctionLibrary = Path.Combine(Path.GetTempPath(), "no-such-j2534-lib") }));
            Assert.IsFalse(passThru.LoadLibrary(new J2534Device()));
            Assert.IsFalse(passThru.FreeLibrary());
        }
    }
}
