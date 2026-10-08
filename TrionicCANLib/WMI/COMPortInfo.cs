using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Management;
using System.Runtime.Versioning;

// Serial ports with a human readable description.
//  Windows: WMI friendly name, e.g. "mbed Serial Port (COM5)"
//   (https://dariosantarelli.wordpress.com/2010/10/18/c-how-to-programmatically-find-a-com-port-by-friendly-name/)
//  Linux: USB manufacturer, product and serial number from sysfs, e.g. "FTDI FT231X USB UART D30F8H7A"
//  macOS: just the /dev/cu.* name
//
// Example of how to use:
//  foreach (COMPortInfo comPort in COMPortInfo.GetCOMPortsInfo())
//  {
//      logger.Debug(string.Format("{0} – {1}", comPort.Name, comPort.Description));
//  }

namespace TrionicCANLib.WMI
{
    public class COMPortInfo
    {
        public string Name { get; set; }
        public string Description { get; set; }
        /// <summary>USB "vid:pid" in lowercase hex as lsusb prints it, null when unknown (only filled in on Linux)</summary>
        public string UsbId { get; set; }

        /// <summary>
        /// The serial ports to offer the user, named the way SerialPort.PortName wants them
        /// </summary>
        public static string[] GetPortNames()
        {
            string[] names = SerialPort.GetPortNames();
            if (OperatingSystem.IsMacOS())
            {
                // open() on a tty.* node blocks until carrier detect, cu.* is the one to use
                names = Array.FindAll(names, n => n.StartsWith("/dev/cu.", StringComparison.Ordinal));
            }
            Array.Sort(names, StringComparer.Ordinal);
            return names;
        }

        public static List<COMPortInfo> GetCOMPortsInfo()
        {
            if (OperatingSystem.IsWindows())
            {
                return GetWMIPortsInfo();
            }

            List<COMPortInfo> comPortInfoList = new List<COMPortInfo>();
            foreach (string name in GetPortNames())
            {
                comPortInfoList.Add(OperatingSystem.IsLinux() ? FromSysfs(name, "/sys/class/tty") : new COMPortInfo() { Name = name, Description = name });
            }
            return comPortInfoList;
        }

        /// <summary>
        /// /sys/class/tty/ttyUSB0 links to /sys/devices/.../1-4.2/1-4.2:1.0/ttyUSB0/tty/ttyUSB0 and its "device" link points
        /// at the usb-serial port or CDC-ACM interface. The USB device is the first parent that has an idVendor file.
        /// </summary>
        internal static COMPortInfo FromSysfs(string portName, string sysClassTty)
        {
            COMPortInfo info = new COMPortInfo() { Name = portName, Description = portName };
            try
            {
                // One link at a time: the ".." in a link target must be applied to the physical directory, the path
                // functions here collapse it textually
                string tty = Path.Combine(sysClassTty, Path.GetFileName(portName));
                string dir = new DirectoryInfo(tty).ResolveLinkTarget(true)?.FullName ?? tty;
                dir = new DirectoryInfo(Path.Combine(dir, "device")).ResolveLinkTarget(true)?.FullName;
                while (dir != null && !File.Exists(Path.Combine(dir, "idVendor")))
                {
                    dir = Path.GetDirectoryName(dir);
                }
                if (dir == null)
                {
                    return info;
                }

                info.UsbId = ReadSysfs(dir, "idVendor") + ":" + ReadSysfs(dir, "idProduct");
                List<string> parts = new List<string>();
                foreach (string attr in new[] { "manufacturer", "product", "serial" })
                {
                    string value = ReadSysfs(dir, attr);
                    if (value.Length > 0 && !parts.Contains(value))
                    {
                        parts.Add(value);
                    }
                }
                if (parts.Count > 0)
                {
                    info.Description = string.Join(" ", parts);
                }
            }
            catch (Exception)
            {
                // not a USB device or sysfs not readable, keep the plain name
            }
            return info;
        }

        private static string ReadSysfs(string dir, string attr)
        {
            string path = Path.Combine(dir, attr);
            return File.Exists(path) ? File.ReadAllText(path).Trim() : string.Empty;
        }

        [SupportedOSPlatform("windows")]
        private static List<COMPortInfo> GetWMIPortsInfo()
        {
            List<COMPortInfo> comPortInfoList = new List<COMPortInfo>();

            // default scope is the local \root\CIMV2
            ObjectQuery objectQuery = new ObjectQuery("SELECT * FROM Win32_PnPEntity WHERE ConfigManagerErrorCode = 0");
            ManagementObjectSearcher comPortSearcher = new ManagementObjectSearcher(objectQuery);

            using (comPortSearcher)
            {
                string caption = null;
                foreach (ManagementObject obj in comPortSearcher.Get())
                {
                    if (obj != null)
                    {
                        object captionObj = obj["Caption"];
                        if (captionObj != null)
                        {
                            caption = captionObj.ToString();
                            if (caption.Contains("(COM"))
                            {
                                COMPortInfo comPortInfo = new COMPortInfo()
                                {
                                    Name = caption.Substring(caption.LastIndexOf("(COM")).Replace("(", string.Empty).Replace(")", string.Empty),
                                    Description = caption
                                };
                                comPortInfoList.Add(comPortInfo);
                            }
                        }
                    }
                }
            }
            return comPortInfoList;
        }
    }
}
