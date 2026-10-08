using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Versioning;
using System.Text.Json;
using Microsoft.Win32;
using NLog;

namespace TrionicCANLib.CAN.J2534
{
    public class J2534Device
    {
        public string Vendor { get; set; }

        public string Name { get; set; }

        public string FunctionLibrary { get; set; }

        public int CANChannels { get; set; }

        public bool IsCANSupported => CANChannels > 0;

        public override string ToString()
        {
            return Name;
        }
    }

    /// <summary>
    /// Installed J2534 v04.04 drivers: the PassThruSupport registry key on Windows,
    /// ~/.passthru/*.json elsewhere (the rnd-ash convention gocan also reads).
    /// </summary>
    public static class J2534Detect
    {
        private readonly static Logger logger = LogManager.GetCurrentClassLogger();

        private const string PASSTHRU_REGISTRY_PATH = "Software\\PassThruSupport.04.04";
        private const string PASSTHRU_REGISTRY_PATH_6432 = "Software\\Wow6432Node\\PassThruSupport.04.04";

        public static List<J2534Device> ListDevices()
        {
            if (OperatingSystem.IsWindows())
            {
                return ListRegistryDevices();
            }
            return ListJsonDevices(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".passthru"));
        }

        [SupportedOSPlatform("windows")]
        private static List<J2534Device> ListRegistryDevices()
        {
            List<J2534Device> devices = new List<J2534Device>();
            RegistryKey root = Registry.LocalMachine.OpenSubKey(PASSTHRU_REGISTRY_PATH, false) ?? Registry.LocalMachine.OpenSubKey(PASSTHRU_REGISTRY_PATH_6432, false);
            if (root == null)
            {
                return devices;
            }
            using (root)
            {
                foreach (string name in root.GetSubKeyNames())
                {
                    using (RegistryKey key = root.OpenSubKey(name))
                    {
                        if (key == null)
                        {
                            continue;
                        }
                        devices.Add(new J2534Device
                        {
                            Vendor = key.GetValue("Vendor", "") as string,
                            Name = key.GetValue("Name", "") as string,
                            FunctionLibrary = key.GetValue("FunctionLibrary", "") as string,
                            CANChannels = key.GetValue("CAN", 0) is int can ? can : 0
                        });
                    }
                }
            }
            return devices;
        }

        /// <summary>
        /// One device per json file whose FUNCTION_LIB exists, a leading ~/ is the home directory.
        /// Unreadable or malformed files are skipped.
        /// </summary>
        public static List<J2534Device> ListJsonDevices(string directory)
        {
            List<J2534Device> devices = new List<J2534Device>();
            if (!Directory.Exists(directory))
            {
                return devices;
            }
            string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            string[] files = Directory.GetFiles(directory, "*.json");
            Array.Sort(files, StringComparer.Ordinal);
            foreach (string file in files)
            {
                try
                {
                    using (JsonDocument doc = JsonDocument.Parse(File.ReadAllText(file)))
                    {
                        JsonElement root = doc.RootElement;
                        string lib = JsonString(root, "FUNCTION_LIB");
                        if (lib.StartsWith("~/", StringComparison.Ordinal))
                        {
                            lib = Path.Combine(home, lib.Substring(2));
                        }
                        if (!File.Exists(lib))
                        {
                            logger.Debug(String.Format("{0}: FUNCTION_LIB {1} not found", file, lib));
                            continue;
                        }
                        devices.Add(new J2534Device
                        {
                            Vendor = JsonString(root, "VENDOR"),
                            Name = (JsonString(root, "VENDOR") + " " + JsonString(root, "NAME")).Trim(),
                            FunctionLibrary = lib,
                            CANChannels = root.TryGetProperty("CAN", out JsonElement can) && can.ValueKind == JsonValueKind.True ? 1 : 0
                        });
                    }
                }
                catch (Exception e)
                {
                    logger.Debug(e, file);
                }
            }
            return devices;
        }

        private static string JsonString(JsonElement root, string name)
        {
            return root.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.String ? value.GetString() : "";
        }
    }
}
