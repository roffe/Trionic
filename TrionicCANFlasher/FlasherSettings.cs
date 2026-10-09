using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Runtime.Versioning;
using System.Text.Json;
using Microsoft.Win32;
using NLog;
using TrionicCANLib.API;

namespace TrionicCANFlasher
{
    /// <summary>
    /// Application settings, persisted as JSON next to the logs. Edited through frmSettings.
    /// </summary>
    public class FlasherSettings
    {
        private static Logger logger = LogManager.GetCurrentClassLogger();

        private static readonly string m_folder = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "MattiasC", "TrionicCANFlasher");
        private static readonly string m_file = Path.Combine(m_folder, "settings.json");

        // Item lists of the settings dialog combo boxes. Index == enum value.
        public static readonly string[] AdapterTypeNames = GetAdapterTypeNames();
        public static readonly string[] ComSpeedNames = { "115200", "230400", "1Mbit", "2Mbit", "3Mbit" };
        public static readonly string[] InterframeDelayNames =
        {
            "50", "100", "200", "300", "400", "500", "600", "700", "800", "900",
            "1000 (Lowest safe delay)", "1100", "1200 (Default)", "1300", "1400",
            "1500", "1600", "1700", "1800", "1900", "2000"
        };

        private m_adaptertype _m_adaptertype = new m_adaptertype();
        private m_interframe _m_interframe = new m_interframe();
        private m_adapter _m_adapter = new m_adapter();
        private m_selecu _m_selecu = new m_selecu();
        private m_baud _m_baud = new m_baud();

        // Default settings
        private bool m_fullscreen = false;
        private bool m_collapsed  = false;
        private int  m_width  = -1;
        private int  m_height = -1;

        private bool m_enablelog = true;  // "Enable logging"
        private bool m_canlogging = false; // "CAN logging": every frame to canLog*.txt, verbose
        private bool m_onlypbus  = true;  // "Only P-Bus connection"
        private bool m_onbflash  = true;  // "Use flasher on device" (CombiAdapter)
        private bool m_uselegion = true;  // "Use Legion bootloader"
        private bool m_poweruser = false; // "I am a power user"
        private bool m_unlocksys = false; // "Unlock system partitions"
        private bool m_unlckboot = false; // "Unlock boot partition"
        private bool m_autocsum  = false; // "Auto update checksum"
        private bool m_remember  = false; // "Remember dimensions"

        // Hidden features
        private bool m_enablesufeatures = false; // enable / disable su features
        private bool m_verifychecksum   = true;  // Check checksum of file before flashing
        private bool m_uselastpointer   = true;  // Legion. Use the "last address of bin" feature or just regular partition md5
        private bool m_faster           = false; // Legion. Speed up certain tasks

        public class m_interframe
        {
            // "1200 (Default)". Was 9, which pointed at 900 after 50/100/200 were added in front.
            public const int DefaultIndex = 12;

            private string m_name = "1200 (Default)";
            private int m_index = DefaultIndex;
            private static uint[] m_dels =
            {
                50, 100, 200,
                300, 400, 500, 600, 700,
                800, 900,1000,1100,1200, // (Default)
               1300,1400,1500,1600,1700,
               1800,1900,2000
            };

            public int Index
            {
                get { return m_index; }
                set { m_index = value; }
            }

            public string Name
            {
                get { return m_name; }
                set { m_name = value; }
            }

            public uint Value
            {
                get
                {
                    if (m_index >= 0 && m_index < m_dels.Length)
                    {
                        return m_dels[m_index];
                    }
                    else
                    {
                        return 1200;
                    }
                }
            }
        }

        public class m_selecu
        {
            private string m_name = null;
            private int m_index = -1;

            public int Index
            {
                get { return m_index; }
                set { m_index = value; }
            }
            public string Name
            {
                get { return m_name; }
                set { m_name = value; }
            }
        }

        public class m_adaptertype
        {
            private string m_name = null;
            private int m_index = -1;

            public int Index
            {
                get { return m_index;  }
                set { m_index = value; }
            }
            public string Name
            {
                get { return m_name;  }
                set { m_name = value; }
            }
        }

        public class m_adapter
        {
            private string m_name = null;
            private int m_index = -1;

            public int Index
            {
                get { return m_index;  }
                set { m_index = value; }
            }
            public string Name
            {
                get { return m_name;  }
                set { m_name = value; }
            }
        }

        public class m_baud
        {
            private string m_name = null;
            private int m_index = -1;

            public int Index
            {
                get { return m_index;  }
                set { m_index = value; }
            }
            public string Name
            {
                get { return m_name;  }
                set { m_name = value; }
            }
        }

        public m_selecu SelectedECU
        {
            get { return _m_selecu;  }
            set { _m_selecu = value; }
        }

        public m_adaptertype AdapterType
        {
            get { return _m_adaptertype;  }
            set { _m_adaptertype = value; }
        }

        public m_adapter Adapter
        {
            get { return _m_adapter;  }
            set { _m_adapter = value; }
        }

        public m_baud Baudrate
        {
            get { return _m_baud;  }
            set { _m_baud = value; }
        }

        public m_interframe InterframeDelay
        {
            get { return _m_interframe;  }
            set { _m_interframe = value; }
        }

        public bool RememberDimensions
        {
            get { return m_remember;  }
            set { m_remember = value; }
        }

        public bool VerifyChecksum
        {
            get { return m_verifychecksum;  }
            set { m_verifychecksum = value; }
        }

        public bool Faster
        {
            get { return m_faster;  }
            set { m_faster = value; }
        }

        public bool UseLastMarker
        {
            get { return m_uselastpointer;  }
            set { m_uselastpointer = value; }
        }

        /// <summary>Hidden "super user" mode, unlocked by clicking "Advanced features" in the settings dialog.</summary>
        public bool SuperUser
        {
            get { return m_enablesufeatures;  }
            set { m_enablesufeatures = value; }
        }

        public int MainWidth
        {
            get { return m_width;  }
            set { m_width = value; }
        }

        public int MainHeight
        {
            get { return m_height;  }
            set { m_height = value; }
        }

        public bool Fullscreen
        {
            get { return m_fullscreen;  }
            set { m_fullscreen = value; }
        }

        public bool Collapsed
        {
            get { return m_collapsed; }
            set { m_collapsed = value; }
        }

        public bool CombiFlasher
        {
            get { return m_onbflash;  }
            set { m_onbflash = value; }
        }

        public bool OnlyPBus
        {
            get { return m_onlypbus;  }
            set { m_onlypbus = value; }
        }

        public bool EnableLogging
        {
            get { return m_enablelog;  }
            set { m_enablelog = value; }
        }

        public bool CanLogging
        {
            get { return m_canlogging; }
            set { m_canlogging = value; }
        }

        public bool UseLegion
        {
            get { return m_uselegion;  }
            set { m_uselegion = value; }
        }

        public bool PowerUser
        {
            get { return m_poweruser;  }
            set { m_poweruser = value; }
        }

        public bool UnlockSys
        {
            get { return m_unlocksys;  }
            set { m_unlocksys = value; }
        }

        public bool UnlockBoot
        {
            get { return m_unlckboot;  }
            set { m_unlckboot = value; }
        }

        public bool AutoChecksum
        {
            get { return m_autocsum;  }
            set { m_autocsum = value; }
        }

        private static string[] GetAdapterTypeNames()
        {
            // Fetch adapter types from TrionicCANLib.API
            List<string> names = new List<string>();
            foreach (var AdapterType in Enum.GetValues(typeof(CANBusAdapter)))
            {
                try
                {
                    names.Add(((DescriptionAttribute)AdapterType.GetType().GetField(AdapterType.ToString()).GetCustomAttributes(typeof(DescriptionAttribute), false)[0]).Description.ToString());
                }

                catch (Exception ex)
                {
                    logger.Debug(ex.Message);
                }
            }
            return names.ToArray();
        }

        /// <summary>
        /// Same keys (and string forms) as the old HKCU\Software\MattiasC\TrionicCANFlasher values.
        /// </summary>
        private void LoadSetting(string a, string value)
        {
            try
            {
                if (a == "AdapterType")
                {
                    AdapterType.Name = value;
                }
                else if (a == "Adapter")
                {
                    Adapter.Name = value;
                }
                else if (a == "ComSpeed")
                {
                    Baudrate.Name = value;
                }
                else if (a == "ECU")
                {
                    SelectedECU.Name = value;
                }

                else if (a == "EnableLogging")
                {
                    m_enablelog = Convert.ToBoolean(value);
                }
                else if (a == "CanLogging")
                {
                    m_canlogging = Convert.ToBoolean(value);
                }
                else if (a == "OnboardFlasher")
                {
                    m_onbflash = Convert.ToBoolean(value);
                }
                else if (a == "OnlyPBus")
                {
                    m_onlypbus = Convert.ToBoolean(value);
                }
                else if (a == "UseLegionBootloader")
                {
                    m_uselegion = Convert.ToBoolean(value);
                }
                else if (a == "PowerUser")
                {
                    m_poweruser = Convert.ToBoolean(value);
                }
                else if (a == "FormatSystemPartitions")
                {
                    m_unlocksys = Convert.ToBoolean(value);
                }
                else if (a == "FormatBootPartition")
                {
                    m_unlckboot = Convert.ToBoolean(value);
                }
                else if (a == "AutoChecksum")
                {
                    m_autocsum = Convert.ToBoolean(value);
                }
                else if (a == "SuperUser")
                {
                    m_enablesufeatures = Convert.ToBoolean(value);
                }
                else if (a == "InterframeDelay")
                {
                    InterframeDelay.Name = value;
                }
                else if (a == "UseLastAddressPointer")
                {
                    m_uselastpointer = Convert.ToBoolean(value);
                }
                else if (a == "Faster")
                {
                    m_faster = Convert.ToBoolean(value);
                }
                else if (a == "ViewRemember")
                {
                    m_remember = Convert.ToBoolean(value);
                }
                else if (a == "ViewWidth")
                {
                    m_width = Convert.ToInt32(value);
                }
                else if (a == "ViewHeight")
                {
                    m_height = Convert.ToInt32(value);
                }
                else if (a == "ViewFullscreen")
                {
                    m_fullscreen = Convert.ToBoolean(value);
                }
                else if (a == "ViewCollapsed")
                {
                    m_collapsed = Convert.ToBoolean(value);
                }
            }

            catch (Exception ex)
            {
                logger.Debug(ex.Message);
            }
        }

        [SupportedOSPlatform("windows")]
        private void ImportRegistrySettings()
        {
            using (RegistryKey Settings = Registry.CurrentUser.OpenSubKey("Software\\MattiasC\\TrionicCANFlasher"))
            {
                if (Settings != null)
                {
                    foreach (string a in Settings.GetValueNames())
                    {
                        LoadSetting(a, Convert.ToString(Settings.GetValue(a)));
                    }
                    logger.Debug("Imported settings from registry");
                }
            }
        }

        public void Load()
        {
            try
            {
                if (File.Exists(m_file))
                {
                    using (JsonDocument doc = JsonDocument.Parse(File.ReadAllText(m_file)))
                    {
                        foreach (JsonProperty p in doc.RootElement.EnumerateObject())
                        {
                            // JsonElement.ToString() gives "True"/"False" for booleans, the raw text for numbers
                            LoadSetting(p.Name, p.Value.ToString());
                        }
                    }
                }
                // First start after upgrading from the registry based versions
                else if (OperatingSystem.IsWindows())
                {
                    ImportRegistrySettings();
                }
            }

            catch (Exception ex)
            {
                logger.Debug(ex, "Failed to load " + m_file);
            }

            // What the settings dialog combo boxes used to resolve the names to
            AdapterType.Index = Array.IndexOf(AdapterTypeNames, AdapterType.Name);
            Baudrate.Index = Array.IndexOf(ComSpeedNames, Baudrate.Name);
            Adapter.Index = -1;
            if (AdapterType.Index >= 0 && Adapter.Name != null)
            {
                string[] adapters = ITrionic.GetAdapterNames((CANBusAdapter)AdapterType.Index);
                Adapter.Index = Array.IndexOf(adapters, Adapter.Name);

                // The combo kept the first adapter selected when the saved one was missing
                if (Adapter.Index < 0 && adapters.Length > 0)
                {
                    Adapter.Index = 0;
                }
            }
            if (Array.IndexOf(InterframeDelayNames, InterframeDelay.Name) >= 0)
            {
                InterframeDelay.Index = Array.IndexOf(InterframeDelayNames, InterframeDelay.Name);
            }

            /////////////////////////////////////////////
            // Recover from strange settings in registry

            // Make sure settings are returned to safe values in case power user is not enabled
            if (!m_poweruser)
            {
                m_unlocksys = false;
                m_unlckboot = false;
                m_autocsum = false;

                m_enablesufeatures = false;
            }

            // We have to plan this section..
            if (!m_enablesufeatures)
            {
                m_verifychecksum = true;
                m_uselastpointer = true;
                m_faster  = false;
                InterframeDelay.Index = m_interframe.DefaultIndex;
                InterframeDelay.Name = InterframeDelayNames[m_interframe.DefaultIndex];
            }

            // Maybe we should have different unlock sys for ME9 and T8?
            if (!m_unlocksys)
            {
                m_unlckboot = false;
            }

            if ((m_fullscreen && m_collapsed) || !m_remember)
            {
                m_collapsed = false;
                m_fullscreen = false;
            }
        }

        public void Save()
        {
            try
            {
                Directory.CreateDirectory(m_folder);

                // Write a temp file and swap it in so a crash mid-write doesn't eat the settings
                string tmp = m_file + ".tmp";
                using (FileStream fs = File.Create(tmp))
                using (Utf8JsonWriter w = new Utf8JsonWriter(fs, new JsonWriterOptions { Indented = true }))
                {
                    w.WriteStartObject();
                    w.WriteString("AdapterType", AdapterType.Name ?? String.Empty);
                    w.WriteString("Adapter", Adapter.Name ?? String.Empty);
                    w.WriteString("ComSpeed", Baudrate.Name ?? String.Empty);
                    w.WriteString("ECU", SelectedECU.Name ?? String.Empty);

                    w.WriteBoolean("EnableLogging", m_enablelog);
                    w.WriteBoolean("CanLogging", m_canlogging);
                    w.WriteBoolean("OnboardFlasher", m_onbflash);
                    w.WriteBoolean("OnlyPBus", m_onlypbus);
                    w.WriteBoolean("UseLegionBootloader", m_uselegion);

                    w.WriteBoolean("PowerUser", m_poweruser);
                    w.WriteBoolean("FormatSystemPartitions", m_unlocksys);
                    w.WriteBoolean("FormatBootPartition", m_unlckboot);
                    w.WriteBoolean("AutoChecksum", m_autocsum);

                    w.WriteBoolean("SuperUser", m_enablesufeatures);
                    w.WriteBoolean("ViewRemember", m_remember);

                    w.WriteString("InterframeDelay", InterframeDelay.Name ?? String.Empty);
                    w.WriteBoolean("UseLastAddressPointer", m_uselastpointer);
                    w.WriteBoolean("Faster", m_faster);

                    if (m_remember)
                    {
                        w.WriteNumber("ViewWidth", m_width);
                        w.WriteNumber("ViewHeight", m_height);
                        w.WriteBoolean("ViewFullscreen", m_fullscreen);
                        w.WriteBoolean("ViewCollapsed", m_collapsed);
                    }
                    w.WriteEndObject();
                }
                File.Move(tmp, m_file, true);
            }

            catch (Exception ex)
            {
                logger.Error(ex, "Failed to save " + m_file);
            }
        }
    }
}
