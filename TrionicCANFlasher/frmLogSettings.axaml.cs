using System.Collections.Generic;
using System.Linq;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Layout;
using NLog;
using NLog.Config;

namespace TrionicCANFlasher
{
    /// <summary>The log files NLog.config writes, each at a level of its own, and the presets for them.</summary>
    public static class FlasherLogs
    {
        public sealed record LogFile(string Target, string Caption, string File, string Tip);

        public const string Off = "Off";

        /// <summary>The least detailed first; a log at a level gets the ones before it too.</summary>
        public static readonly string[] Levels = { Off, "Error", "Info", "Debug", "Trace" };

        public static readonly LogFile[] Files =
        {
            new("uilogfile", "Program", "uiLog", "What the main window shows, and the program's own errors."),
            new("apilogfile", "ECU sessions", "apiLog", "The T5, T7 and T8 sessions: connecting, security access, ECU info, reading and flashing."),
            new("kwplogfile", "KWP requests", "kwpLog", "Trionic 7's KWP2000 sessions, at Trace every request and its answer."),
            new("flasherlogfile", "Flashing", "flasherLog", "Reading and flashing the ECU step by step, at Trace every block."),
            new("devicelogfile", "CAN adapter", "deviceLog", "The adapter's driver: opening it, its settings, its errors."),
            new("canlogfile", "CAN frames", "canLog",
                "Every CAN frame sent and received, at Trace. It grows fast: for finding communication problems. Log data always writes it."),
        };

        // a level per Files entry
        private static readonly (string Name, string[] Levels)[] s_presets =
        {
            ("Normal", new[] { "Info", "Info", "Info", "Info", "Info", Off }),
            ("Verbose", new[] { "Debug", "Debug", "Debug", "Debug", "Debug", Off }),
            ("Debug", new[] { "Trace", "Trace", "Trace", "Trace", "Debug", "Trace" }),
        };

        public static IEnumerable<string> PresetNames => s_presets.Select(p => p.Name);

        public const string DefaultPreset = "Verbose";

        public static Dictionary<string, string> Preset(string a_name)
        {
            string[] levels = s_presets.First(p => p.Name == a_name).Levels;
            return Files.Select((f, i) => (f.Target, levels[i])).ToDictionary();
        }

        /// <summary>The preset these levels are, or null.</summary>
        public static string PresetOf(IReadOnlyDictionary<string, string> a_levels)
        {
            return s_presets.FirstOrDefault(p => Files.Select((f, i) => a_levels[f.Target] == p.Levels[i]).All(same => same)).Name;
        }

        /// <summary>
        /// The settings into NLog's rules: every rule writing to a log gets its level and the ones after it, none with Enable logging off.
        /// Log data (a_logData) writes the CAN frames and the rest as set, whatever Enable logging says.
        /// </summary>
        public static void Apply(FlasherSettings a_settings, bool a_logData = false)
        {
            LoggingConfiguration config = LogManager.Configuration;
            if (config == null)
            {
                return;
            }
            foreach (LogFile log in Files)
            {
                string level = a_settings.EnableLogging || a_logData ? a_settings.LogLevels.GetValueOrDefault(log.Target, Off) : Off;
                if (a_logData && log.Target == "canlogfile")
                {
                    level = "Trace";
                }
                foreach (LoggingRule rule in config.LoggingRules.Where(r => r.Targets.Any(t => t.Name == log.Target)))
                {
                    rule.DisableLoggingForLevels(LogLevel.Trace, LogLevel.Fatal);
                    if (level != Off)
                    {
                        rule.EnableLoggingForLevels(LogLevel.FromString(level), LogLevel.Fatal);
                    }
                }
            }
            LogManager.ReconfigExistingLoggers();
        }
    }

    /// <summary>Settings → Log settings: a level per log file, or a preset. OK returns the levels, Cancel null.</summary>
    public partial class frmLogSettings : Window
    {
        private const string Custom = "Custom";
        private readonly ComboBox[] m_levels = new ComboBox[FlasherLogs.Files.Length];
        private bool m_lockout;

        // the XAML previewer
        public frmLogSettings() : this(FlasherLogs.Preset(FlasherLogs.DefaultPreset)) { }

        public frmLogSettings(IReadOnlyDictionary<string, string> a_levels)
        {
            InitializeComponent();
            m_lockout = true;
            cbxPreset.ItemsSource = FlasherLogs.PresetNames.Append(Custom).ToArray();
            for (int i = 0; i < FlasherLogs.Files.Length; i++)
            {
                FlasherLogs.LogFile log = FlasherLogs.Files[i];
                gridLogs.RowDefinitions.Add(new RowDefinition(GridLength.Auto));
                var caption = new TextBlock { Text = log.Caption, VerticalAlignment = VerticalAlignment.Center };
                var level = new ComboBox { ItemsSource = FlasherLogs.Levels, SelectedItem = a_levels[log.Target], HorizontalAlignment = HorizontalAlignment.Stretch };
                var file = new TextBlock { Text = log.File + "<date>.txt", VerticalAlignment = VerticalAlignment.Center, Opacity = 0.6 };
                level.SelectionChanged += (_, _) => ShowPreset();
                foreach (Control c in new Control[] { caption, level, file })
                {
                    ToolTip.SetTip(c, log.Tip);
                    Grid.SetRow(c, i);
                    gridLogs.Children.Add(c);
                }
                Grid.SetColumn(level, 1);
                Grid.SetColumn(file, 2);
                m_levels[i] = level;
            }
            m_lockout = false;
            ShowPreset();
        }

        private Dictionary<string, string> Levels()
        {
            return FlasherLogs.Files.Select((f, i) => (f.Target, (string)m_levels[i].SelectedItem)).ToDictionary();
        }

        private void ShowPreset()
        {
            if (m_lockout)
            {
                return;
            }
            m_lockout = true;
            cbxPreset.SelectedItem = FlasherLogs.PresetOf(Levels()) ?? Custom;
            m_lockout = false;
        }

        private void cbxPreset_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (m_lockout || cbxPreset.SelectedItem is not string name || name == Custom)
            {
                return;
            }
            m_lockout = true;
            Dictionary<string, string> preset = FlasherLogs.Preset(name);
            for (int i = 0; i < m_levels.Length; i++)
            {
                m_levels[i].SelectedItem = preset[FlasherLogs.Files[i].Target];
            }
            m_lockout = false;
        }

        private void btnOk_Click(object sender, RoutedEventArgs e)
        {
            Close(Levels());
        }

        private void btnCancel_Click(object sender, RoutedEventArgs e)
        {
            Close(null);
        }
    }
}
