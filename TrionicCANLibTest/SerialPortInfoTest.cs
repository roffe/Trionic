using System;
using System.IO;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using TrionicCANLib.WMI;

namespace TrionicCANLibTest
{
    [TestClass]
    public class SerialPortInfoTest
    {
        private string m_root;

        [TestInitialize]
        public void Setup()
        {
            if (OperatingSystem.IsWindows())
            {
                Assert.Inconclusive("sysfs layout uses symlinks, Linux/macOS only");
            }
            m_root = Directory.CreateTempSubdirectory("sysfs").FullName;
            Directory.CreateDirectory(Path.Combine(m_root, "class", "tty"));
        }

        [TestCleanup]
        public void Cleanup()
        {
            if (m_root != null)
            {
                Directory.Delete(m_root, true);
            }
        }

        // Mirrors the kernel: class/tty/<tty> -> devices/.../<tty>/tty/<tty>, whose "device" link points back up
        private void AddTty(string usbDevice, string ttyParent, string tty, string deviceLink, params string[] attrs)
        {
            string usb = Path.Combine(m_root, "devices", "pci0000:00", "usb1", usbDevice);
            string node = Path.Combine(usb, ttyParent, "tty", tty);
            Directory.CreateDirectory(node);
            Directory.CreateSymbolicLink(Path.Combine(node, "device"), deviceLink);
            Directory.CreateSymbolicLink(Path.Combine(m_root, "class", "tty", tty),
                Path.Combine("..", "..", "devices", "pci0000:00", "usb1", usbDevice, ttyParent, "tty", tty));
            for (int i = 0; i < attrs.Length; i += 2)
            {
                File.WriteAllText(Path.Combine(usb, attrs[i]), attrs[i + 1] + "\n");
            }
        }

        private COMPortInfo Info(string tty)
        {
            return COMPortInfo.FromSysfs("/dev/" + tty, Path.Combine(m_root, "class", "tty"));
        }

        [TestMethod]
        public void UsbSerialPortGetsUsbStrings()
        {
            // ftdi_sio: the tty hangs off a ttyUSBn port device below the interface
            AddTty("1-5.3", Path.Combine("1-5.3:1.0", "ttyUSB3"), "ttyUSB3", Path.Combine("..", "..", "..", "ttyUSB3"),
                "idVendor", "0403", "idProduct", "6015", "manufacturer", "FTDI", "product", "FT231X USB UART", "serial", "D30F8H7A");

            COMPortInfo info = Info("ttyUSB3");
            Assert.AreEqual("/dev/ttyUSB3", info.Name);
            Assert.AreEqual("FTDI FT231X USB UART D30F8H7A", info.Description);
            Assert.AreEqual("0403:6015", info.UsbId);
        }

        [TestMethod]
        public void AcmPortGetsUsbStrings()
        {
            // cdc_acm: the tty hangs straight off the interface, this is how the mbed shows up
            AddTty("1-5.1", "1-5.1:1.1", "ttyACM0", Path.Combine("..", "..", "..", "1-5.1:1.1"),
                "idVendor", "0d28", "idProduct", "0204", "manufacturer", "MBED", "product", "MBED CMSIS-DAP", "serial", "1010");

            COMPortInfo info = Info("ttyACM0");
            Assert.AreEqual("MBED MBED CMSIS-DAP 1010", info.Description);
            Assert.AreEqual("0d28:0204", info.UsbId);
        }

        [TestMethod]
        public void MissingAndDuplicateStringsAreSkipped()
        {
            AddTty("1-4.3.3", Path.Combine("1-4.3.3:1.0", "ttyUSB1"), "ttyUSB1", Path.Combine("..", "..", "..", "ttyUSB1"),
                "idVendor", "1a86", "idProduct", "7523", "product", "USB Serial");
            AddTty("1-5.4.3", "1-5.4.3:1.0", "ttyACM1", Path.Combine("..", "..", "..", "1-5.4.3:1.0"),
                "idVendor", "18e1", "idProduct", "0107", "manufacturer", "Drew Technologies Inc.", "product", "Drew Technologies Inc.");

            Assert.AreEqual("USB Serial", Info("ttyUSB1").Description);
            Assert.AreEqual("Drew Technologies Inc.", Info("ttyACM1").Description);
        }

        [TestMethod]
        public void NonUsbPortKeepsItsName()
        {
            AddTty("serial8250", "serial8250:0.0", "ttyS0", Path.Combine("..", "..", "..", "serial8250:0.0"));

            COMPortInfo info = Info("ttyS0");
            Assert.AreEqual("/dev/ttyS0", info.Description);
            Assert.IsNull(info.UsbId);
            Assert.AreEqual("/dev/ttyXYZ", Info("ttyXYZ").Description);
        }
    }
}
