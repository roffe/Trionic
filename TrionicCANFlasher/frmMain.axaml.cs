using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Reflection;
using Microsoft.Win32;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Platform;
using Avalonia.Threading;
using TrionicCANLib;
using TrionicCANLib.API;
using TrionicCANLib.Firmware;
using TrionicCANLib.Checksum;
using NLog;
using CommonSuite;

namespace TrionicCANFlasher
{
    public delegate void DelegateUpdateStatus(ITrionic.CanInfoEventArgs e);
    public delegate void DelegateProgressStatus(int percentage);

    public partial class frmMain : Window
    {
        readonly Trionic8 trionic8 = new Trionic8();
        readonly Trionic7 trionic7 = new Trionic7();
        readonly Trionic5 trionic5 = new Trionic5();
        FlasherSettings AppSettings = new FlasherSettings();

        DateTime dtstart;
        public DelegateUpdateStatus m_DelegateUpdateStatus;
        public DelegateProgressStatus m_DelegateProgressStatus;
        public ChecksumDelegate.ChecksumUpdate m_ShouldUpdateChecksum;
        private readonly Logger logger = LogManager.GetCurrentClassLogger();

        MsiUpdater m_msiUpdater;
        BackgroundWorker bgworkerLogCanData;
        private bool m_bypassCANfilters = false; // Christian: Stop-gap solution for now.
        private WindowState LastWindowState = WindowState.Normal;

        // set by EnableUserInput: every ECU operation starts with EnableUserInput(false) and ends with EnableUserInput(true)
        private bool m_operationRunning = false;
        private bool m_closeRefusedShown = false;
        // a T7 flash or read, which Trionic7 runs on its own threads and ends in updateStatusInBox
        private bool m_t7FlashRunning = false;

        // compact view is the button panel only, its height follows the content
        private const double CompactWidth = 360;

        // what WinForms Application.ProductVersion returned
        private static readonly string ProductVersion = (typeof(frmMain).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion
            ?? typeof(frmMain).Assembly.GetCustomAttribute<AssemblyFileVersionAttribute>()?.Version ?? "0.0.0.0").Split('+')[0];

        public frmMain()
        {
            try { Thread.CurrentThread.Priority = ThreadPriority.AboveNormal; } catch (Exception) { }
            Dispatcher.UIThread.UnhandledException += Dispatcher_UnhandledException;
            AppDomain.CurrentDomain.UnhandledException += CurrentDomain_UnhandledException;
            TaskScheduler.UnobservedTaskException += TaskScheduler_UnobservedTaskException;
            InitializeComponent();
            m_DelegateUpdateStatus = updateStatusInBox;
            m_DelegateProgressStatus = updateProgress;
            m_ShouldUpdateChecksum = ShouldUpdateChecksum;
            UserPrompt.YesNo = (text, caption) => Dialogs.Wait(() => Dialogs.YesNo(this, text, caption));
            UserPrompt.Notify = (text, caption) => Dialogs.Wait(async () => { await Dialogs.Info(this, text, caption); return true; });
            EnableUserInput(true);
            frmMain_Load();
        }

        private void frmMain_Load()
        {
            Title = "TrionicCANFlasher v" + ProductVersion;
            logger.Trace(Title);
            logger.Trace(".dot net CLR " + System.Environment.Version);

            // get additional info from registry if available
            AppSettings.Load();

            // Fetch ECUs from TrionicCANLib.API
            cbxEcuType.Items.Clear();

            // ECU values without a [Description] (internal ones) are not offered
            foreach (var Target in Enum.GetValues(typeof(ECU)))
            {
                var description = Target.GetType().GetField(Target.ToString()).GetCustomAttribute<DescriptionAttribute>();
                if (description != null)
                    cbxEcuType.Items.Add(description.Description);
            }

            // Fetch last selected ECU from registry and pass its index back to AppSettings
            if (AppSettings.SelectedECU.Name != null)
            {
                try
                {
                    cbxEcuType.SelectedItem = AppSettings.SelectedECU.Name;
                    AppSettings.SelectedECU.Index = cbxEcuType.SelectedIndex;
                }

                catch (Exception ex)
                {
                    AddLogItem(ex.Message);
                }
            }

            trionic5.onReadProgress += trionicCan_onReadProgress;
            trionic5.onWriteProgress += trionicCan_onWriteProgress;
            trionic5.onCanInfo += trionicCan_onCanInfo;

            trionic7.onReadProgress += trionicCan_onReadProgress;
            trionic7.onWriteProgress += trionicCan_onWriteProgress;
            trionic7.onCanInfo += trionicCan_onCanInfo;

            trionic8.onReadProgress += trionicCan_onReadProgress;
            trionic8.onWriteProgress += trionicCan_onWriteProgress;
            trionic8.onCanInfo += trionicCan_onCanInfo;

            RestoreView();
            UpdateLogManager();
            EnableUserInput(true);
        }

        private void frmMain_FormClosing(object sender, WindowClosingEventArgs e)
        {
            // the window stays responsive during an operation, closing it mid-flash would leave the ECU half written
            if (m_operationRunning)
            {
                e.Cancel = true;
                if (!m_closeRefusedShown)
                {
                    m_closeRefusedShown = true;
                    // a logger that ended by itself leaves the button saying Stop
                    bool logging = (string)btnLogData.Content == "Stop" && bgworkerLogCanData != null && bgworkerLogCanData.IsBusy;
                    string text = logging ? "Logging is running, press Stop before closing." :
                        "An ECU operation is running, wait until it finishes before closing.";
                    Dispatcher.UIThread.Post(async () =>
                    {
                        await Dialogs.Info(this, text, "Operation running");
                        m_closeRefusedShown = false;
                    });
                }
                return;
            }

            AppSettings.Save();
            trionic8.Cleanup();
            trionic7.Cleanup();
            trionic5.Cleanup();
        }

        void CurrentDomain_UnhandledException(object sender, UnhandledExceptionEventArgs u)
        {
            logger.Trace(u.ExceptionObject);
            LogManager.Flush();
        }

        void TaskScheduler_UnobservedTaskException(object sender, UnobservedTaskExceptionEventArgs e)
        {
            logger.Trace(e.Exception);
            AddLogItem(e.Exception.GetBaseException().Message);
            e.SetObserved();
        }

        // Like WinForms Application.ThreadException: log it and keep running
        void Dispatcher_UnhandledException(object sender, DispatcherUnhandledExceptionEventArgs t)
        {
            logger.Trace(t.Exception);
            t.Handled = true;
            AddLogItem(t.Exception.Message);
        }

        private async void frmMain_Shown(object sender, EventArgs e)
        {
            await CheckRegistryFTDI();
            try
            {
                // the numeric version (tag padded to 0.1.74.0), ProductVersion may carry a -<sha> suffix
                m_msiUpdater = new MsiUpdater(typeof(frmMain).Assembly.GetName().Version);
                m_msiUpdater.onDataPump += new MsiUpdater.DataPump(m_msiUpdater_onDataPump);
                m_msiUpdater.CheckForUpdates("https://api.github.com/repos/mattiasclaesson/trionic/releases/latest");
            }
            catch (Exception E)
            {
                AddLogItem(E.Message);
            }
        }

        // runs on the update check thread
        void m_msiUpdater_onDataPump(MsiUpdater.MSIUpdaterEventArgs e)
        {
            if (e.UpdateAvailable)
            {
                Dispatcher.UIThread.Post(async () =>
                {
                    frmUpdateAvailable frmUpdate = new frmUpdateAvailable();
                    frmUpdate.SetVersionNumber(e.Version.ToString());
                    if (m_msiUpdater != null)
                    {
                        m_msiUpdater.Blockauto_updates = false;
                    }
                    if (await frmUpdate.ShowDialog<bool>(this))
                    {
                        if (m_msiUpdater != null)
                        {
                            if (!m_operationRunning && !trionic5.isOpen() && !trionic7.isOpen() && !trionic8.isOpen())
                            {
                                m_msiUpdater.ExecuteUpdate(e.MSIFile);
                                // the msi replaces us; elsewhere only the release page was opened
                                if (OperatingSystem.IsWindows())
                                {
                                    Close();
                                }
                            }
                        }
                    }
                    else
                    {
                        // user choose "NO", don't bug him again!
                        if (m_msiUpdater != null)
                        {
                            m_msiUpdater.Blockauto_updates = false;
                        }
                    }
                });
            }
        }

        // Linux sets the FTDI latency itself (SerialLowLatency), only Windows needs the user to do it
        private async Task CheckRegistryFTDI()
        {
            if (!OperatingSystem.IsWindows())
            {
                return;
            }

            if (AppSettings.AdapterType.Index == (int)CANBusAdapter.ELM327)
            {
                bool warn = false;
                using (RegistryKey FTDIBUSKey = Registry.LocalMachine.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum\\FTDIBUS"))
                {
                    if (FTDIBUSKey != null)
                    {
                        string[] vals = FTDIBUSKey.GetSubKeyNames();
                        foreach (string name in vals)
                        {
                            if (name.StartsWith("VID_0403+PID_6001"))
                            {
                                using (RegistryKey NameKey = FTDIBUSKey.OpenSubKey(name + "\\0000\\Device Parameters"))
                                {
                                    if (NameKey != null)
                                    {
                                        String PortName = NameKey.GetValue("PortName").ToString();
                                        if (AppSettings.Adapter.Name != null && AppSettings.Adapter.Name.Equals(PortName))
                                        {
                                            String Latency = NameKey.GetValue("LatencyTimer").ToString();
                                            AddLogItem(String.Format("ELM327 FTDI setting for {0} LatencyTimer {1}ms.", PortName, Latency));
                                            if (!Latency.Equals("2") && !Latency.Equals("1"))
                                            {
                                                warn = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if (warn)
                {
                    await Dialogs.Info(this, "Warning LatencyTimer should be set to 2 ms", "ELM327 FTDI setting");
                }
            }
        }

        private static string TotalDuration(TimeSpan ts)
        {
            int minutes = (int)ts.TotalMinutes, seconds = ts.Seconds;
            return "Total duration: " + minutes + (minutes == 1 ? " minute " : " minutes ") + seconds + (seconds == 1 ? " second" : " seconds");
        }

        private readonly List<string> m_logLines = new List<string>();

        private void logCopy_Click(object sender, RoutedEventArgs e) => textBoxLog.Copy();

        private void logSelectAll_Click(object sender, RoutedEventArgs e) => textBoxLog.SelectAll();

        private void logCopyAll_Click(object sender, RoutedEventArgs e)
        {
            textBoxLog.SelectAll();
            textBoxLog.Copy();
        }

        // ECU strings arrive NUL-padded (T8 info: "55561881\0\0"); a NUL in a selection makes the
        // Linux clipboard come back empty, so control characters never reach the log
        private static string CleanForLog(string s)
        {
            System.Text.StringBuilder sb = null;
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                bool bad = char.IsControl(c) && c != '\t' && c != '\r' && c != '\n';
                if (bad && sb == null)
                {
                    sb = new System.Text.StringBuilder(s, 0, i, s.Length);
                }
                if (sb != null && c != '\0')
                {
                    sb.Append(bad ? '?' : c);
                }
            }
            return sb == null ? s : sb.ToString();
        }

        public void AddLogItem(string item)
        {
            // T5/T7 checksum callbacks pass a null layer; WinForms threw on that in compact view
            if (item == null)
            {
                return;
            }
            item = CleanForLog(item);

            if (!Dispatcher.UIThread.CheckAccess())
            {
                Dispatcher.UIThread.Post(() => AddLogItem(item));
                return;
            }

            if (AppSettings.MainWidth > 740)
            {
                m_logLines.Add(DateTime.Now.ToString("HH:mm:ss.fff") + " - " + item);
            }
            else
            {
                m_logLines.Add(item);
            }

            if (AppSettings.Collapsed)
            {
                string Lastmsg = item;

                if (Lastmsg.Length > 57)
                {
                    Lastmsg = Lastmsg.Remove(57, Lastmsg.Length - 57);
                }

                Minilog.Text = Lastmsg;
                Minilog.IsVisible = true;
            }

            // keep the last 100 lines like the old list; a selection the user is copying survives new lines
            int removed = 0;
            while (m_logLines.Count > 100)
            {
                removed += m_logLines[0].Length + Environment.NewLine.Length;
                m_logLines.RemoveAt(0);
            }
            int selStart = textBoxLog.SelectionStart, selEnd = textBoxLog.SelectionEnd;
            textBoxLog.Text = string.Join(Environment.NewLine, m_logLines);
            if (selStart != selEnd)
            {
                textBoxLog.SelectionStart = Math.Max(0, selStart - removed);
                textBoxLog.SelectionEnd = Math.Max(0, selEnd - removed);
            }
            else
            {
                textBoxLog.CaretIndex = textBoxLog.Text.Length; // follow the newest line
            }
            logger.Trace(item);
        }

        void trionicCan_onWriteProgress(object sender, ITrionic.WriteProgressEventArgs e)
        {
            UpdateProgressStatus(e.Percentage);
        }

        void trionicCan_onCanInfo(object sender, ITrionic.CanInfoEventArgs e)
        {
            UpdateFlashStatus(e);
        }

        void trionicCan_onReadProgress(object sender, ITrionic.ReadProgressEventArgs e)
        {
            UpdateProgressStatus(e.Percentage);
        }

        private async void updateStatusInBox(ITrionic.CanInfoEventArgs e)
        {
            AddLogItem(e.Info);
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                if (e.Type == ActivityType.FinishedFlashing || e.Type == ActivityType.FinishedDownloadingFlash)
                {
                    TimeSpan ts = DateTime.Now - dtstart;
                    AddLogItem(TotalDuration(ts));
                    // Read SRAM reports FinishedDownloadingFlash too, its worker closes its own connection
                    if (m_t7FlashRunning)
                    {
                        m_t7FlashRunning = false;
                        await RunOnWorker(trionic7, trionic7.Cleanup);
                        AddLogItem("Connection closed");
                        EnableUserInput(true);
                    }
                }
            }
        }

        // Like AddLogItem: the library's threads queue it behind the lines they logged before and go on. Waiting for
        // the window like Control.Invoke made them deadlocks against a UI thread that waits for them.
        private void UpdateFlashStatus(ITrionic.CanInfoEventArgs e)
        {
            if (!Dispatcher.UIThread.CheckAccess())
            {
                Dispatcher.UIThread.Post(() => UpdateFlashStatus(e));
                return;
            }

            try
            {
                m_DelegateUpdateStatus(e);
            }
            catch (Exception ex)
            {
                AddLogItem(ex.Message);
            }
        }

        private void updateProgress(int percentage)
        {
            if (progressBar1.Value != percentage)
            {
                progressBar1.Value = percentage;
            }
            if (AppSettings.EnableLogging)
            {
                logger.Trace("progress: " + percentage.ToString("F0") + "%");
            }
        }

        private void UpdateProgressStatus(int percentage)
        {
            if (!Dispatcher.UIThread.CheckAccess())
            {
                Dispatcher.UIThread.Post(() => UpdateProgressStatus(percentage));
                return;
            }

            try
            {
                m_DelegateProgressStatus(percentage);
            }
            catch (Exception e)
            {
                logger.Trace(e.Message);
            }
        }

        private void StartBGWorkerLog(ITrionic trionic)
        {
            AddLogItem("Logging in progress");
            bgworkerLogCanData = new BackgroundWorker
            {
                WorkerReportsProgress = true,
                WorkerSupportsCancellation = true
            };
            bgworkerLogCanData.DoWork += trionic.LogCANData;
            bgworkerLogCanData.RunWorkerCompleted += bgWorker_RunWorkerCompleted;
            bgworkerLogCanData.RunWorkerAsync();
        }

        async void bgWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            // reported at the click handler's priority, ahead of the operation's last log lines and progress that
            // are still queued at Default: let those come first
            await Dispatcher.UIThread.InvokeAsync(() => { }, DispatcherPriority.Default);

            if (e.Cancelled)
            {
                AddLogItem("Stopped");
            }
            else if (e.Error == null && e.Result != null && (bool)e.Result)
            {
                AddLogItem("Operation done");
            }
            else
            {
                // e.Result rethrows a DoWork exception, which skipped the Cleanup/EnableUserInput below
                if (e.Error != null)
                {
                    logger.Trace(e.Error);
                    AddLogItem(e.Error.Message);
                }
                AddLogItem("Operation failed");
                SetViewMode(false);
            }

            TimeSpan ts = DateTime.Now - dtstart;
            AddLogItem(TotalDuration(ts));
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
            {
                await RunOnWorker(trionic5, trionic5.Cleanup);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                await RunOnWorker(trionic7, trionic7.Cleanup);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8 ||
                     cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96 ||
                     cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP ||
                     cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG ||
                     cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
            {
                await RunOnWorker(trionic8, trionic8.Cleanup);
            }
            EnableUserInput(true);
            AddLogItem("Connection terminated");
        }

        public void UpdateLogManager()
        {
            if (AppSettings.EnableLogging)
            {
                LogManager.ResumeLogging();
            }
            else
            {
                LogManager.SuspendLogging();
            }
        }

        private void SetGenericOptions(ITrionic trionic)
        {
            trionic.OnlyPBus = AppSettings.OnlyPBus;
            trionic.bypassCANfilters = m_bypassCANfilters;

            trionic.LegionOptions.Faster = AppSettings.Faster;
            trionic.LegionOptions.InterframeDelay = AppSettings.InterframeDelay.Value;
            trionic.LegionOptions.UseLastMarker = AppSettings.UseLastMarker;

            m_bypassCANfilters = false;
            bool IsT8Class = true;

            switch (cbxEcuType.SelectedIndex)
            {
                case (int)ECU.TRIONIC5:
                    trionic.ECU = ECU.TRIONIC5;
                    IsT8Class = false;
                    break;
                case (int)ECU.TRIONIC7:
                    trionic.ECU = ECU.TRIONIC7;
                    IsT8Class = false;
                    break;
                case (int)ECU.TRIONIC8:
                    trionic.ECU = ECU.TRIONIC8;
                    break;
                case (int)ECU.TRIONIC8_MCP:
                    trionic.ECU = ECU.TRIONIC8_MCP;
                    break;
                case (int)ECU.Z22SEMain_LEG:
                    trionic.ECU = ECU.Z22SEMain_LEG;
                    break;
                case (int)ECU.Z22SEMCP_LEG:
                    trionic.ECU = ECU.Z22SEMCP_LEG;
                    break;
                case (int)ECU.MOTRONIC96:
                    trionic.ECU = ECU.MOTRONIC96;
                    break;
                default:
                    IsT8Class = false;
                    break;
            }

            switch (AppSettings.AdapterType.Index)
            {
                case (int)CANBusAdapter.JUST4TRIONIC:
                    trionic.ForcedBaudrate = 115200;
                    break;
                case (int)CANBusAdapter.SLCAN:
                    //set selected com speed
                    switch (AppSettings.Baudrate.Index)
                    {
                        case (int)ComSpeed.S3Mbit:
                            trionic.ForcedBaudrate = 3000000;
                            break;
                        case (int)ComSpeed.S2Mbit:
                            trionic.ForcedBaudrate = 2000000;
                            break;
                        case (int)ComSpeed.S1Mbit:
                            trionic.ForcedBaudrate = 1000000;
                            break;
                        case (int)ComSpeed.S230400:
                            trionic.ForcedBaudrate = 230400;
                            break;
                        case (int)ComSpeed.S115200:
                            trionic.ForcedBaudrate = 115200;
                            break;
                        default:
                            trionic.ForcedBaudrate = 0; //default , no speed will be changed
                            break;
                    }
                    break;
                case (int)CANBusAdapter.ELM327:
                    //set selected com speed
                    switch (AppSettings.Baudrate.Index)
                    {
                        case (int)ComSpeed.S2Mbit:
                            trionic.ForcedBaudrate = 2000000;
                            break;
                        case (int)ComSpeed.S1Mbit:
                            trionic.ForcedBaudrate = 1000000;
                            break;
                        case (int)ComSpeed.S230400:
                            trionic.ForcedBaudrate = 230400;
                            break;
                        case (int)ComSpeed.S115200:
                            trionic.ForcedBaudrate = 115200;
                            break;
                        default:
                            trionic.ForcedBaudrate = 0; //default , no speed will be changed
                            break;
                    }
                    break;
                default:
                    break;
            }

            trionic.setCANDevice((CANBusAdapter)AppSettings.AdapterType.Index);
            if (AppSettings.Adapter.Name != null)
            {
                trionic.SetSelectedAdapter(AppSettings.Adapter.Name);
            }

            // Christian: Ideally these would be better in ITrionic?
            if (IsT8Class)
            {
                trionic8.FormatBootPartition = AppSettings.UnlockBoot;
                trionic8.FormatSystemPartitions = AppSettings.UnlockSys;
            }
        }

        bool checkFileSize(string fileName)
        {
            FileInfo fi = new FileInfo(fileName);
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
            {
                if (!FileT5.VerifyFileSize(fi.Length))
                {
                    AddLogItem("Not a trionic 5 file");
                    return false;
                }
            }
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                if (!FileT7.VerifyFileSize(fi.Length))
                {
                    AddLogItem("Not a trionic 7 file");
                    return false;
                }
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8 || cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG)
            {
                if (!FileT8.VerifyFileSize(fi.Length))
                {
                    AddLogItem("Not a trionic 8 file");
                    return false;
                }
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
            {
                if (!FileME96.VerifyFileSize(fi.Length))
                {
                    AddLogItem("Not a Motronic ME9.6 file");
                    return false;
                }
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP || cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
            {
                if (!FileT8mcp.VerifyFileSize(fi.Length))
                {
                    AddLogItem("Not a trionic 8 mcp file");
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Runs an ECU operation off the UI thread, so the window keeps repainting and its log and progress bar
        /// follow the operation. Keep open, operation and Cleanup in one body: KWPHandler's Mutex is thread-affine
        /// and the libraries expect one thread per session. AboveNormal, like the UI thread these used to run on.
        /// A body that throws is logged and its connection cleaned up on that same thread.
        /// </summary>
        /// <returns>false if the body threw</returns>
        private Task<bool> RunOnWorker(ITrionic trionic, Action body)
        {
            var done = new TaskCompletionSource<bool>();
            var worker = new Thread(() =>
            {
                bool ok = true;
                try
                {
                    body();
                }
                catch (Exception ex)
                {
                    ok = false;
                    logger.Trace(ex);
                    AddLogItem(ex.Message);
                    try
                    {
                        trionic.Cleanup();
                    }
                    catch (Exception cleanupEx)
                    {
                        logger.Trace(cleanupEx);
                    }
                }
                // not the caller's continuation directly: it would run at the click handler's priority, ahead of
                // the log lines and progress this operation queued at Default
                Dispatcher.UIThread.Post(() => done.SetResult(ok));
            })
            {
                IsBackground = true,
                Name = "ECU operation",
            };
            try { worker.Priority = ThreadPriority.AboveNormal; } catch (Exception) { }
            worker.Start();
            return done.Task;
        }

        /// <summary>
        /// What a BackgroundWorker operation does before RunWorkerAsync, on a worker: open, the second these flows
        /// always waited, then startMessage. When it doesn't open: failMessage, Cleanup, and the input comes back.
        /// </summary>
        /// <returns>true: start the BackgroundWorker</returns>
        private async Task<bool> OpenOnWorker(ITrionic trionic, Func<bool> open, string startMessage, string failMessage)
        {
            bool opened = false;
            bool ok = await RunOnWorker(trionic, () =>
            {
                opened = open();
                if (opened)
                {
                    Thread.Sleep(1000);
                    dtstart = DateTime.Now;
                    AddLogItem(startMessage);
                }
                else
                {
                    AddLogItem(failMessage);
                    trionic.Cleanup();
                }
            });
            opened = opened && ok;
            if (!opened)
            {
                EnableUserInput(true);
                AddLogItem("Connection terminated");
            }
            return opened;
        }

        private void EnableUserInput(bool enable)
        {
            m_operationRunning = !enable;

            btnFlashECU.IsEnabled = enable;
            btnReadECU.IsEnabled = enable;
            btnGetECUInfo.IsEnabled = enable;
            btnReadSRAM.IsEnabled = enable;
            btnRecoverECU.IsEnabled = enable;
            btnReadDTC.IsEnabled = enable;
            cbxEcuType.IsEnabled = enable;

            btnEditParameters.IsEnabled = enable;
            btnReadECUcalibration.IsEnabled = enable;
            btnRestoreT8.IsEnabled = enable;
            btnLogData.IsEnabled = enable;
            btnSettings.IsEnabled = enable;
            btnWriteDID.IsEnabled = enable;
            btnResetECU.IsEnabled = enable;

            bool PreCheck = true;
            if (AppSettings.AdapterType.Index == (int)CANBusAdapter.ELM327 &&
                (AppSettings.Baudrate.Index < 0 || AppSettings.Adapter.Index < 0))
            {
                PreCheck = false;
            }
            else if ((AppSettings.AdapterType.Index == (int)CANBusAdapter.J2534  ||
                      AppSettings.AdapterType.Index == (int)CANBusAdapter.KVASER ||
                      AppSettings.AdapterType.Index == (int)CANBusAdapter.LAWICEL) &&
                      AppSettings.Adapter.Index < 0)
            {
                PreCheck = false;
            }

            if (AppSettings.AdapterType.Index >= 0 && cbxEcuType.SelectedIndex >= 0 && PreCheck)
            {
                if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
                {
                    btnReadECUcalibration.IsEnabled = false;
                    btnReadDTC.IsEnabled = false;
                    btnEditParameters.IsEnabled = false;
                    btnRecoverECU.IsEnabled = false;
                    btnRestoreT8.IsEnabled = false;
                    btnWriteDID.IsEnabled = false;
                }

                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
                {
                    btnRecoverECU.IsEnabled = false;
                    btnReadECUcalibration.IsEnabled = false;
                    btnRestoreT8.IsEnabled = false;
                    btnWriteDID.IsEnabled = false;
                }

                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
                {
                    btnReadECUcalibration.IsEnabled = false;
                    btnWriteDID.IsEnabled = false;
                }

                else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
                {
                    btnReadSRAM.IsEnabled = false;
                    btnRestoreT8.IsEnabled = false;
                    btnRecoverECU.IsEnabled = false;
                    // the T8 reset request is T8 firmware specific, unknown on the Bosch ECU
                    btnResetECU.IsEnabled = false;

                    // OBDLink cannot handle write on me9.6, and truncate some fields in getecuinfo
                    if (AppSettings.AdapterType.Index == (int)CANBusAdapter.ELM327)
                    {
                        btnFlashECU.IsEnabled = false;
                        btnReadECU.IsEnabled = false;
                        btnGetECUInfo.IsEnabled = false;
                        btnReadSRAM.IsEnabled = false;
                        btnRecoverECU.IsEnabled = false;
                        btnReadDTC.IsEnabled = false;

                        btnEditParameters.IsEnabled = false;
                        btnReadECUcalibration.IsEnabled = false;
                        btnRestoreT8.IsEnabled = false;
                        btnLogData.IsEnabled = false;
                        btnWriteDID.IsEnabled = false;
                    }
                }

                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP || cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG || cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
                {
                    btnReadDTC.IsEnabled = false;
                    btnReadECUcalibration.IsEnabled = false;

                    // Bootloader handles recovery, if at all possible, on MCP.
                    btnRecoverECU.IsEnabled = false;

                    btnRestoreT8.IsEnabled = false;
                    btnReadSRAM.IsEnabled = false;

                    btnGetECUInfo.IsEnabled = false;
                    btnEditParameters.IsEnabled = false;
                    btnWriteDID.IsEnabled = false;
                    // MCP and Z22SE: the T8 main's reset request isn't known to apply
                    btnResetECU.IsEnabled = false;
                }
            }

            // Disable everything except Settings; Still not fully configured
            else if (cbxEcuType.SelectedIndex >= 0)
            {
                btnFlashECU.IsEnabled = false;
                btnReadECU.IsEnabled = false;
                btnGetECUInfo.IsEnabled = false;
                btnReadSRAM.IsEnabled = false;
                btnRecoverECU.IsEnabled = false;
                btnReadDTC.IsEnabled = false;

                btnEditParameters.IsEnabled = false;
                btnReadECUcalibration.IsEnabled = false;
                btnRestoreT8.IsEnabled = false;
                btnLogData.IsEnabled = false;
                btnWriteDID.IsEnabled = false;
                btnResetECU.IsEnabled = false;
            }

            // Disable everything; Select ECU before poking around!
            else
            {
                btnFlashECU.IsEnabled = false;
                btnReadECU.IsEnabled = false;
                btnGetECUInfo.IsEnabled = false;
                btnReadSRAM.IsEnabled = false;
                btnRecoverECU.IsEnabled = false;
                btnReadDTC.IsEnabled = false;

                btnEditParameters.IsEnabled = false;
                btnReadECUcalibration.IsEnabled = false;
                btnRestoreT8.IsEnabled = false;
                btnLogData.IsEnabled = false;
                btnSettings.IsEnabled = false;
                btnWriteDID.IsEnabled = false;
                btnResetECU.IsEnabled = false;
            }
        }

        private static string SubString8(string value)
        {
            return value.Length < 8 ? value : value.Substring(0, 8);
        }

        private async void btnFlashEcu_Click(object sender, RoutedEventArgs e)
        {
            bool result = false;
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8 || cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96 ||
                cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP ||
                cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG || cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
                result = await Dialogs.OkCancel(this, "Attach a charger. Now turn key to ON to wakeup ECU.",
                "Critical Warning");
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                result = await Dialogs.OkCancel(this, "Attach a charger. Turn key to ON wait a few seconds, turn to LOCK. Wait 15 to 20 seconds then initiate the flash operation in car.",
                "Critical Warning");
            }
            // Trionic 5 is complex. Skip dialog until a sutitable one has been written
            if (!result && cbxEcuType.SelectedIndex != (int)ECU.TRIONIC5)
            {
                return;
            }

            string fileName = await Dialogs.OpenFile(this, "Bin files", "*.bin");
            if (fileName != null)
            {
                if (checkFileSize(fileName))
                {
                    if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
                    {
                        ChecksumResult checksumResult = ChecksumT5.VerifyChecksum(fileName, AppSettings.AutoChecksum, m_ShouldUpdateChecksum);
                        if (checksumResult != ChecksumResult.Ok && AppSettings.VerifyChecksum)
                        {
                            AddLogItem("Checksum check failed: " + checksumResult);
                            return;
                        }

                        SetGenericOptions(trionic5);
                        AddLogItem("Opening connection");
                        EnableUserInput(false);
                        bool opened = false;
                        WriteFlashResult flashed = WriteFlashResult.Failed;
                        await RunOnWorker(trionic5, () =>
                        {
                            opened = trionic5.openDevice();
                            if (opened)
                            {
                                Thread.Sleep(1000);
                                AddLogItem("Update FLASH content");
                                dtstart = DateTime.Now;
                                flashed = trionic5.WriteFlash(fileName);
                            }
                            else
                            {
                                AddLogItem("Unable to connect to Trionic 5 ECU");
                            }
                            // no reset after a failure: the retry that WriteFlash advises reuses the running bootloader
                            trionic5.Cleanup();
                        });
                        EnableUserInput(true);
                        if (opened)
                        {
                            // the library logged what went wrong and what to do; a collapsed window would only show its last line
                            if (flashed == WriteFlashResult.Done)
                            {
                                AddLogItem("Operation done");
                            }
                            else if (flashed != WriteFlashResult.Cancelled)
                            {
                                AddLogItem("Operation failed");
                                SetViewMode(false);
                            }
                            TimeSpan ts = DateTime.Now - dtstart;
                            AddLogItem(TotalDuration(ts));
                            if (flashed == WriteFlashResult.Cancelled)
                            {
                                // the user said No or the file doesn't fit the ECU: the FLASH was not touched. Last line, as the
                                // window stays collapsed: the duration under a full progress bar would look like a finished flash
                                AddLogItem("Operation cancelled");
                            }
                        }
                        else
                        {
                            AddLogItem("Connection terminated");
                        }
                    }
                    else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
                    {
                        ChecksumResult checksumResult = ChecksumT7.VerifyChecksum(fileName, AppSettings.AutoChecksum, ChecksumT7.DO_NOT_AUTOFIXFOOTER, m_ShouldUpdateChecksum); // TODO: mattias, add AutoFixFooter to settings?
                        if (checksumResult != ChecksumResult.Ok && AppSettings.VerifyChecksum)
                        {
                            AddLogItem("Checksum check failed: " + checksumResult);
                            return;
                        }

                        trionic7.UseFlasherOnDevice = AppSettings.OnlyPBus ? AppSettings.CombiFlasher : false;
                        SetGenericOptions(trionic7);

                        AddLogItem("Opening connection");
                        EnableUserInput(false);
                        // the flash continues on Trionic7's threads, updateStatusInBox ends it
                        m_t7FlashRunning = true;
                        bool opened = false;
                        bool ok = await RunOnWorker(trionic7, () =>
                        {
                            opened = trionic7.openDevice();
                            if (opened)
                            {
                                Thread.Sleep(1000);
                                AddLogItem("Update FLASH content");
                                dtstart = DateTime.Now;
                                trionic7.WriteFlash(fileName);
                            }
                            else
                            {
                                AddLogItem("Unable to connect to Trionic 7 ECU");
                                trionic7.Cleanup();
                            }
                        });
                        if (!opened || !ok)
                        {
                            m_t7FlashRunning = false;
                            EnableUserInput(true);
                            AddLogItem("Connection terminated");
                        }
                    }
                    else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
                    {
                        ChecksumResult checksumResult = ChecksumT8.VerifyChecksum(fileName, AppSettings.AutoChecksum, m_ShouldUpdateChecksum);
                        if (checksumResult != ChecksumResult.Ok && AppSettings.VerifyChecksum)
                        {
                            AddLogItem("Checksum check failed: " + checksumResult);
                            return;
                        }

                        SetGenericOptions(trionic8);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;

                        if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Update FLASH content", "Unable to connect to Trionic 8 ECU"))
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            if (AppSettings.UseLegion)
                            {
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.WriteFlashLegT8);
                            }
                            else
                            {
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.WriteFlash);
                            }
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(fileName);
                        }
                    }
                    else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP)
                    {
                        SetGenericOptions(trionic8);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;

                        trionic8.FormatSystemPartitions = true; // This is undefined in mcp.

                        if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Update FLASH content", "Unable to connect to Trionic 8 ECU"))
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            bgWorker.DoWork += new DoWorkEventHandler(trionic8.WriteFlashLegMCP);
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(fileName);
                        }
                    }
                    else if (cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG)
                    {
                        SetGenericOptions(trionic8);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;

                        trionic8.FormatSystemPartitions = true;
                        trionic8.FormatBootPartition    = true;

                        if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Update FLASH content", "Unable to connect to Z22SE ECU"))
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            bgWorker.DoWork += new DoWorkEventHandler(trionic8.WriteFlashLegZ22SE_Main);
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(fileName);
                        }
                    }
                    else if (cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
                    {
                        SetGenericOptions(trionic8);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;

                        trionic8.FormatSystemPartitions = true; // This is undefined in mcp.
                        trionic8.FormatBootPartition    = true;

                        if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Update FLASH content", "Unable to connect to Z22SE ECU"))
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            bgWorker.DoWork += new DoWorkEventHandler(trionic8.WriteFlashLegZ22SE_MCP);
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(fileName);
                        }
                    }
                    else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
                    {
                        SetGenericOptions(trionic8);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                        // the questions below block the worker until answered, the connection stays with it
                        FlashReadArguments args = null;
                        await RunOnWorker(trionic8, () =>
                        {
                            if (trionic8.openDevice(false))
                            {
                                string ecuCalibrationset = trionic8.GetCalibrationSet();
                                ecuCalibrationset = SubString8(ecuCalibrationset);
                                if (ecuCalibrationset == "")
                                {
                                    AddLogItem("ECU connection issue, check logs");
                                    // this used to end here with the input disabled for good; the window can't be
                                    // closed during an operation, so it cleans up and gives the input back like the others
                                    trionic8.Cleanup();
                                }
                                else
                                {
                                    string ecuMainOS = trionic8.RequestECUInfoAsString(0xC1);
                                    ecuMainOS = SubString8(ecuMainOS);
                                    string ecuEngineCalib = trionic8.RequestECUInfoAsString(0xC2);
                                    ecuEngineCalib = SubString8(ecuEngineCalib);
                                    string ecuSystemCalib = trionic8.RequestECUInfoAsString(0xC3);
                                    ecuSystemCalib = SubString8(ecuSystemCalib);
                                    string ecuSpeedoCalib = trionic8.RequestECUInfoAsString(0xC4);
                                    ecuSpeedoCalib = SubString8(ecuSpeedoCalib);
                                    string ecuSlaveOS = trionic8.RequestECUInfoAsString(0xC5);
                                    ecuSlaveOS = SubString8(ecuSlaveOS);

                                    bool flash = true;
                                    int flashStart = (int)FileME96.EngineCalibrationAddress;
                                    int flashEnd = (int)FileME96.EngineCalibrationAddressEnd;

                                    string fileMainOS = FileME96.getMainOSVersion(fileName);
                                    AddLogItem("Main OS version in file: " + fileMainOS);
                                    if (fileMainOS != string.Empty)
                                    {
                                        AddLogItem("Main OS version in ECU: " + ecuMainOS);

                                        // Certain vendors are known to poke around in OS and leave version number the same
                                        if (ecuMainOS == fileMainOS && AppSettings.PowerUser)
                                        {
                                            bool ask = Dialogs.Wait(() => Dialogs.YesNo(this, "Version numbers match. Do you still want to overwrite Main OS?\n" +
                                                "Click No to only write calibration", "Overwrite Main OS?"));

                                            if (ask)
                                            {
                                                AddLogItem("User has opted to overwrite Main OS");
                                                FileInfo fi = new FileInfo(fileName);
                                                flashStart = (int)FileME96.MainOSAddress;
                                                flashEnd = (int)fi.Length;
                                            }
                                            else
                                            {
                                                flash = FlashEngineCalibration(fileName, ecuEngineCalib);
                                            }
                                        }

                                        else if (ecuMainOS != fileMainOS)
                                        {
                                            AddLogItem("Main OS version differs between file and ECU");

                                            if (AppSettings.UnlockSys)
                                            {
                                                AddLogItem("User has selected option format system partitions");
                                                FileInfo fi = new FileInfo(fileName);
                                                flashStart = (int)FileME96.MainOSAddress;
                                                flashEnd = (int)fi.Length;
                                            }
                                            else
                                            {
                                                AddLogItem("Aborted flash, format system partitions is unchecked");
                                                flash = false;
                                            }
                                        }
                                        else
                                        {
                                            flash = FlashEngineCalibration(fileName, ecuEngineCalib);
                                        }
                                    }
                                    else
                                    {
                                        // Or just force the user to read the complete ecu instead.

                                        // Check that the basefile version is matched with beginning of calibrationset
                                        string basefileInfo = FileME96.getFileInfo(fileName);
                                        if (!basefileInfo.Contains(ecuCalibrationset.Substring(0, 4)))
                                        {
                                            AddLogItem("Basefile and file to write is not compatible " + basefileInfo + " and " + ecuCalibrationset);
                                            flash = false;
                                        }
                                        else
                                        {
                                            flash = FlashEngineCalibration(fileName, ecuEngineCalib);
                                        }
                                    }

                                    if (flash)
                                    {
                                        AddLogItem("Flash addresses start:" + flashStart.ToString("X") + " and end: " + flashEnd.ToString("X"));
                                        Thread.Sleep(1000);
                                        dtstart = DateTime.Now;
                                        AddLogItem("Update FLASH content");
                                        args = new FlashReadArguments() { FileName = fileName, start = flashStart, end = flashEnd };
                                    }
                                    else
                                    {
                                        AddLogItem("Flash operation aborted");
                                        trionic8.Cleanup();
                                    }
                                }
                            }
                            else
                            {
                                AddLogItem("Unable to connect to ME9.6 ECU");
                                trionic8.Cleanup();
                            }
                        });
                        if (args != null)
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            bgWorker.DoWork += new DoWorkEventHandler(trionic8.WriteFlashME96);
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(args);
                        }
                        else
                        {
                            EnableUserInput(true);
                            AddLogItem("Connection terminated");
                        }
                    }
                }
            }
            LogManager.Flush();
        }

        // on the ME9.6 flash worker, the question blocks it until answered
        private bool FlashEngineCalibration(string fileName, string ecuEngineCalib)
        {
            bool flash = true;

            string fileEngineCalib = FileME96.getEngineCalibrationVersion(fileName);
            AddLogItem("Engine Calibration version in file: " + fileEngineCalib);
            if (fileEngineCalib != string.Empty)
            {
                AddLogItem("Engine Calibration version in ECU: " + ecuEngineCalib);
                if (ecuEngineCalib != fileEngineCalib)
                {
                    AddLogItem("Aborted flash, Engine Calibration version differs between file and ECU");
                    flash = false;
                }
                else
                {
                    // Read the ecu here and compare with file.
                    // So we know if there is any point in writing?

                    bool ask = Dialogs.Wait(() => Dialogs.YesNo(this, "Do you want to overwrite calibration?",
                        "Calibration write"));
                    if (!ask)
                    {
                        flash = false;
                    }
                }
            }
            return flash;
        }

        private async void btnReadECU_Click(object sender, RoutedEventArgs e)
        {
            string fileName = await Dialogs.SaveFile(this, "Bin files", "bin");
            if (fileName != null)
            {
                if (fileName != string.Empty)
                {
                    if (Path.GetFileName(fileName) != string.Empty)
                    {
                        if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
                        {
                            SetGenericOptions(trionic5);

                            AddLogItem("Opening connection");
                            EnableUserInput(false);

                            bool opened = false;
                            await RunOnWorker(trionic5, () =>
                            {
                                opened = trionic5.openDevice();
                                if (opened)
                                {
                                    Thread.Sleep(1000);
                                    dtstart = DateTime.Now;
                                    AddLogItem("Acquiring FLASH content");
                                }
                                else
                                {
                                    AddLogItem("Unable to connect to Trionic 5 ECU");
                                    trionic5.Cleanup();
                                }
                            });
                            if (opened)
                            {
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();

                                bgWorker.DoWork += new DoWorkEventHandler(trionic5.DumpECU);

                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(fileName);
                            }
                            else
                            {
                                AddLogItem("Connection closed");
                                EnableUserInput(true);
                            }
                        }
                        else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
                        {
                            trionic7.UseFlasherOnDevice = AppSettings.OnlyPBus ? AppSettings.CombiFlasher : false;
                            SetGenericOptions(trionic7);

                            AddLogItem("Opening connection");
                            EnableUserInput(false);

                            // the read continues on Trionic7's threads, updateStatusInBox ends it
                            m_t7FlashRunning = true;
                            bool opened = false;
                            bool ok = await RunOnWorker(trionic7, () =>
                            {
                                opened = trionic7.openDevice();
                                if (opened)
                                {
                                    // check reading status periodically
                                    Thread.Sleep(1000);
                                    AddLogItem("Acquiring FLASH content");
                                    dtstart = DateTime.Now;
                                    trionic7.ReadFlash(fileName);
                                }
                                else
                                {
                                    AddLogItem("Unable to connect to Trionic 7 ECU");
                                    trionic7.Cleanup();
                                }
                            });
                            if (!opened || !ok)
                            {
                                m_t7FlashRunning = false;
                                AddLogItem("Connection closed");
                                EnableUserInput(true);
                            }
                        }
                        else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
                        {
                            SetGenericOptions(trionic8);

                            EnableUserInput(false);
                            AddLogItem("Opening connection");
                            trionic8.SecurityLevel = AccessLevel.AccessLevel01;

                            if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Acquiring FLASH content", "Unable to connect to Trionic 8 ECU"))
                            {
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();
                                if (AppSettings.UseLegion)
                                {
                                    bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlashLegT8);
                                }
                                else
                                {
                                    bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlash);
                                }
                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(fileName);
                            }
                        }
                        else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
                        {
                            SetGenericOptions(trionic8);

                            EnableUserInput(false);
                            AddLogItem("Opening connection");
                            trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                            bool opened = await OpenOnWorker(trionic8, () =>
                            {
                                if (!trionic8.openDevice(false))
                                {
                                    return false;
                                }
                                trionic8.SaveAllDID(fileName);
                                return true;
                            }, "Acquiring FLASH content", "Unable to connect to ME9.6 ECU");
                            if (opened)
                            {
                                FlashReadArguments args = new FlashReadArguments() { FileName = fileName, start = (int)FileME96.MainOSAddress, end = (int)FileME96.LengthComplete };
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlashME96);
                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(args);
                            }
                        }

                        else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP)
                        {
                            SetGenericOptions(trionic8);

                            EnableUserInput(false);
                            AddLogItem("Opening connection");
                            trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                            if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Acquiring FLASH content", "Unable to connect to Trionic 8 ECU"))
                            {
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlashLegMCP);
                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(fileName);
                            }
                        }
                        else if (cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG)
                        {
                            SetGenericOptions(trionic8);

                            EnableUserInput(false);
                            AddLogItem("Opening connection");
                            trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                            if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Acquiring FLASH content", "Unable to connect to Z22SE ECU"))
                            {
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlashLegZ22SE_Main);
                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(fileName);
                            }
                        }
                        else if (cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
                        {
                            SetGenericOptions(trionic8);

                            EnableUserInput(false);
                            AddLogItem("Opening connection");
                            trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                            if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Acquiring FLASH content", "Unable to connect to Z22SE ECU"))
                            {
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlashLegZ22SE_MCP);
                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(fileName);
                            }
                        }
                    }
                }
            }
            LogManager.Flush();
        }

        private async void btnGetEcuInfo_Click(object sender, RoutedEventArgs e)
        {
            SetViewMode(false);
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
            {
                SetGenericOptions(trionic5);

                AddLogItem("Opening connection");
                EnableUserInput(false);

                await RunOnWorker(trionic5, () =>
                {
                    if (trionic5.openDevice())
                    {
                        Thread.Sleep(1000);
                        AddLogItem("Acquiring ECU info");
                        trionic5.GetECUInfo(true);
                    }
                    else
                    {
                        AddLogItem("Unable to connect to Trionic 5 ECU");
                    }
                    trionic5.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                trionic7.UseFlasherOnDevice = false;
                SetGenericOptions(trionic7);

                AddLogItem("Opening connection");
                EnableUserInput(false);

                await RunOnWorker(trionic7, () =>
                {
                    if (trionic7.openDevice())
                    {
                        Thread.Sleep(1000);
                        AddLogItem("Acquiring ECU info");
                        trionic7.GetECUInfo();
                    }
                    else
                    {
                        AddLogItem("Unable to connect to Trionic 7 ECU");
                    }
                    trionic7.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
            {
                SetGenericOptions(trionic8);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                trionic8.SecurityLevel = AccessLevel.AccessLevelFD;
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(false))
                    {
                        // ELM devices cannot detect send failures until in the readMessage thread
                        // Added a connection check here to avoid confused users when all fields show blank!
                        string ecuhardware = trionic8.GetECUHardware();
                        if (ecuhardware == "")
                        {
                            AddLogItem("ECU connection issue, check logs");
                        }
                        else
                        {
                            AddLogItem("VINNumber                 : " + trionic8.GetVehicleVIN());            //0x90
                            AddLogItem("Calibration set           : " + trionic8.GetCalibrationSet());        //0x74
                            AddLogItem("Codefile version          : " + trionic8.GetCodefileVersion());       //0x73
                            AddLogItem("ECU description           : " + trionic8.GetECUDescription());        //0x72
                            AddLogItem("ECU hardware              : " + ecuhardware);                         //0x71
                            AddLogItem("ECU sw number             : " + trionic8.GetECUSWVersionNumber());    //0x95
                            AddLogItem("Programming date          : " + trionic8.GetProgrammingDate());       //0x99
                            AddLogItem("Build date                : " + trionic8.GetBuildDate());             //0x0A
                            AddLogItem("Serial number             : " + trionic8.GetSerialNumber());          //0xB4
                            AddLogItem("Software version          : " + trionic8.GetSoftwareVersion());       //0x08
                            AddLogItem("0F identifier             : " + trionic8.RequestECUInfoAsString(0x0F));
                            AddLogItem("SW identifier 1           : " + trionic8.RequestECUInfoAsString(0xC1));
                            AddLogItem("SW identifier 2           : " + trionic8.RequestECUInfoAsString(0xC2));
                            AddLogItem("SW identifier 3           : " + trionic8.RequestECUInfoAsString(0xC3));
                            AddLogItem("SW identifier 4           : " + trionic8.RequestECUInfoAsString(0xC4));
                            AddLogItem("SW identifier 5           : " + trionic8.RequestECUInfoAsString(0xC5));
                            AddLogItem("SW identifier 6           : " + trionic8.RequestECUInfoAsString(0xC6));
                            AddLogItem("Hardware type             : " + trionic8.RequestECUInfoAsString(0x97));
                            AddLogItem("75 identifier             : " + trionic8.RequestECUInfoAsString(0x75));
                            AddLogItem("Engine type               : " + trionic8.RequestECUInfoAsString(0x0C));
                            AddLogItem("Supplier ID               : " + trionic8.RequestECUInfoAsString(0x92));
                            AddLogItem("Speed limiter             : " + trionic8.GetTopSpeed() + " km/h");
                            AddLogItem("Oil quality               : " + trionic8.GetOilQuality().ToString("F2") + " %");
                            AddLogItem("SAAB partnumber           : " + trionic8.GetSaabPartnumber());
                            AddLogItem("Diagnostic Data Identifier: " + trionic8.GetDiagnosticDataIdentifier());
                            AddLogItem("End model partnumber      : " + trionic8.GetInt64FromIdAsString(0xCB));
                            AddLogItem("Base model partnumber     : " + trionic8.GetInt64FromIdAsString(0xCC));
                            AddLogItem("ManufacturersEnableCounter: " + trionic8.GetManufacturersEnableCounter());
                            AddLogItem("Tester Serial             : " + trionic8.RequestECUInfoAsString(0x98));
                            bool convertible, sai, highoutput, biopower, clutchStart;
                            TankType tankType;
                            DiagnosticType diagnosticType;
                            string rawPI01;
                            trionic8.GetPI01(out convertible, out sai, out highoutput, out biopower, out diagnosticType, out clutchStart, out tankType, out rawPI01);

                            logger.Debug("PI 0x01         : Cab:" + convertible + " SAI:" + sai + " HighOutput:" + highoutput + " Biopower:" + biopower + " DiagnosticType:" + diagnosticType + " ClutchStart:" + clutchStart + " TankType:" + tankType + " rawValues: " + rawPI01);
                            logger.Debug("PI 0x03         : " + trionic8.GetPI03());
                            logger.Debug("PI 0x04         : " + trionic8.GetPI04());
                            logger.Debug("PI 0x07         : " + trionic8.GetPI07());
                            logger.Debug("PI 0x2E         : " + trionic8.GetPI2E());
                            logger.Debug("PI 0xB9         : " + trionic8.GetPIB9());
                            logger.Debug("PI 0x24         : " + trionic8.GetPI24());
                            logger.Debug("PI 0xA0         : " + trionic8.GetPIA0());
                            logger.Debug("PI 0x96         : " + trionic8.GetPI96());

                            // On a non biopower bin this request seem to poison the session, do it last always!
                            AddLogItem("E85                       : " + trionic8.GetE85Percentage().ToString("F2") + " %");
                        }
                    }

                    trionic8.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
            {
                SetGenericOptions(trionic8);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(false)) // change to test securityaccess
                    {
                        // ELM devices cannot detect send failures until in the readMessage thread
                        // Added a connection check here to avoid confused users when all fields show blank!
                        string calibrationset = trionic8.GetCalibrationSet();
                        if (calibrationset == "")
                        {
                            AddLogItem("ECU connection issue, check logs");
                        }
                        else
                        {
                            string ecuMainOS = trionic8.RequestECUInfoAsString(0xC1);
                            ecuMainOS = SubString8(ecuMainOS);
                            string ecuEngineCalib = trionic8.RequestECUInfoAsString(0xC2);
                            ecuEngineCalib = SubString8(ecuEngineCalib);
                            string ecuSystemCalib = trionic8.RequestECUInfoAsString(0xC3);
                            ecuSystemCalib = SubString8(ecuSystemCalib);
                            string ecuSpeedoCalib = trionic8.RequestECUInfoAsString(0xC4);
                            ecuSpeedoCalib = SubString8(ecuSpeedoCalib);
                            string ecuSlaveOS = trionic8.RequestECUInfoAsString(0xC5);
                            ecuSlaveOS = SubString8(ecuSlaveOS);

                            AddLogItem("VINNumber                 : " + trionic8.GetVehicleVIN());           //0x90
                            AddLogItem("Calibration set           : " + trionic8.GetCalibrationSet());       //0x74
                            AddLogItem("Codefile version          : " + trionic8.GetCodefileVersion());      //0x73
                            AddLogItem("Diagnostic address        : " + trionic8.GetDiagnosticAddress());    //0xB0
                            AddLogItem("Serial number             : " + trionic8.GetSerialNumber());         //0xB4
                            AddLogItem("Programming date          : " + trionic8.GetProgrammingDateME96());  //0x99
                            AddLogItem("Main OS                   : " + ecuMainOS);
                            AddLogItem("Engine Calib              : " + ecuEngineCalib);
                            AddLogItem("System Calib              : " + ecuSystemCalib);
                            AddLogItem("Speedo Calib              : " + ecuSpeedoCalib);
                            AddLogItem("Slave OS                  : " + ecuSlaveOS);
                            AddLogItem("Hardware type             : " + trionic8.RequestECUInfoAsString(0x97));
                            AddLogItem("Supplier ID               : " + trionic8.RequestECUInfoAsString(0x92));
                            AddLogItem("Speed limiter             : " + trionic8.GetTopSpeed() + " km/h"); //0x02
                            AddLogItem("Radum                     : " + trionic8.GetRadum());              //0x24
                            AddLogItem("Pmc w                     : " + trionic8.GetPmcW());               //0x2E
                            AddLogItem("Diagnostic Data Identifier: " + trionic8.GetDiagnosticDataIdentifier());
                            AddLogItem("End model partnumber      : " + trionic8.GetInt64FromIdAsString(0xCB));
                            AddLogItem("Base model partnumber     : " + trionic8.GetInt64FromIdAsString(0xCC));
                            AddLogItem("ManufacturersEnableCounter: " + trionic8.GetManufacturersEnableCounter());
                            AddLogItem("Tester Serial             : " + trionic8.RequestECUInfoAsString(0x98));
                            AddLogItem("Bosch Enable Counter      : " + trionic8.GetBoschEnableCounter());

                            //string name = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ecuinfo");
                            //trionic8.SaveAllDID(name);
                        }
                    }

                    trionic8.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            LogManager.Flush();
        }

        // Manual only: nothing resets a T7 by itself (throttle body limp mode, see the question below)
        private async void btnResetECU_Click(object sender, RoutedEventArgs e)
        {
            string question;
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                question = "Resetting a Trionic 7 while the ignition is ON in a car puts the electronic throttle body into limp mode, " +
                           "and it then has to be reset mechanically.\n\n" +
                           "Typical use: after flashing on a bench, or to get the ECU out of the state it stays in after a flash.\n\n" +
                           "Reset the ECU now?";
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
            {
                question = "A bootloader is loaded into the ECU's RAM and restarts the ECU, as at the end of Get ECU info.\n\nReset the ECU now?";
            }
            else
            {
                question = "The ECU saves its settings and restarts. It refuses while the engine is running.\n\nReset the ECU now?";
            }
            if (!await Dialogs.YesNo(this, question, "Reset ECU"))
            {
                return;
            }

            SetViewMode(false);
            EnableUserInput(false);
            AddLogItem("Opening connection");
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
            {
                SetGenericOptions(trionic5);
                await RunOnWorker(trionic5, () =>
                {
                    if (trionic5.openDevice())
                    {
                        Thread.Sleep(1000); // as Get ECU info
                        trionic5.ResetECU();
                    }
                    else
                    {
                        AddLogItem("Unable to connect to Trionic 5 ECU");
                    }
                    trionic5.Cleanup();
                });
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                // before SetGenericOptions, which only builds the KWP stack without the Combi's own flasher
                trionic7.UseFlasherOnDevice = false;
                SetGenericOptions(trionic7);
                await RunOnWorker(trionic7, () =>
                {
                    if (trionic7.openDevice())
                    {
                        trionic7.ResetECU();
                    }
                    else
                    {
                        AddLogItem("Unable to connect to Trionic 7 ECU");
                    }
                    trionic7.Cleanup();
                });
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
            {
                SetGenericOptions(trionic8);
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(false))
                    {
                        trionic8.ResetECU();
                    }
                    trionic8.Cleanup();
                });
            }
            AddLogItem("Connection closed");
            EnableUserInput(true);
            LogManager.Flush();
        }

        private async void btnReadSRAM_Click(object sender, RoutedEventArgs e)
        {
            string fileName = await Dialogs.SaveFile(this, "SRAM snapshots", "RAM");
            if (fileName != null)
            {
                if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
                {
                    SetGenericOptions(trionic5);

                    AddLogItem("Opening connection");
                    EnableUserInput(false);

                    await RunOnWorker(trionic5, () =>
                    {
                        if (trionic5.openDevice())
                        {
                            Thread.Sleep(1000);
                            AddLogItem("Acquiring snapshot");
                            dtstart = DateTime.Now;
                            trionic5.GetSRAMSnapshot(fileName);
                        }
                        else
                        {
                            AddLogItem("Unable to connect to Trionic 5 ECU");
                        }
                        trionic5.Cleanup();
                    });
                    EnableUserInput(true);
                    AddLogItem("Connection terminated");
                }
                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
                {
                    trionic7.UseFlasherOnDevice = false;
                    SetGenericOptions(trionic7);

                    AddLogItem("Opening connection");
                    EnableUserInput(false);

                    await RunOnWorker(trionic7, () =>
                    {
                        if (trionic7.openDevice())
                        {
                            Thread.Sleep(1000);
                            AddLogItem("Acquiring snapshot");
                            dtstart = DateTime.Now;
                            trionic7.GetSRAMSnapshot(fileName);
                        }
                        else
                        {
                            AddLogItem("Unable to connect to Trionic 7 ECU");
                        }
                        trionic7.Cleanup();
                    });
                    EnableUserInput(true);
                    AddLogItem("Connection terminated");
                }
                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
                {
                    SetGenericOptions(trionic8);

                    EnableUserInput(false);
                    AddLogItem("Opening connection");
                    trionic8.SecurityLevel = AccessLevel.AccessLevelFD;
                    // a failed open used to end here with the input disabled for good; the window can't be closed
                    // during an operation, so it cleans up and gives the input back like the others
                    if (await OpenOnWorker(trionic8, () => trionic8.openDevice(true), "Acquiring snapshot", "Unable to connect to Trionic 8 ECU"))
                    {
                        BackgroundWorker bgWorker;
                        bgWorker = new BackgroundWorker();
                        bgWorker.DoWork += new DoWorkEventHandler(trionic8.GetSRAMSnapshot);
                        bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                        bgWorker.RunWorkerAsync(fileName);
                    }
                }
            }
            LogManager.Flush();
        }

        private async void btnRecoverECU_Click(object sender, RoutedEventArgs e)
        {
            string fileName = await Dialogs.OpenFile(this, "Binary files", "*.bin");
            if (fileName != null)
            {
                if (checkFileSize(fileName))
                {
                    if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
                    {
                        ChecksumResult checksum = ChecksumT8.VerifyChecksum(fileName, AppSettings.AutoChecksum, m_ShouldUpdateChecksum);
                        if (checksum != ChecksumResult.Ok && AppSettings.VerifyChecksum)
                        {
                            AddLogItem("Checksum check failed: " + checksum);
                            return;
                        }

                        SetGenericOptions(trionic8);
                        trionic8.SetCANFilterIds(Trionic8.FilterIdRecovery);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                        if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Recovering ECU", "Unable to connect to Trionic 8 ECU"))
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            if (AppSettings.UseLegion)
                            {
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.RecoverECU_Leg);
                            }
                            else
                            {
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.RecoverECU_Def);
                            }
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(fileName);
                        }
                    }
                }
            }
            LogManager.Flush();
        }

        private async void btnReadDTC_Click(object sender, RoutedEventArgs e)
        {
            SetViewMode(false);
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                trionic7.UseFlasherOnDevice = false;
                SetGenericOptions(trionic7);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                await RunOnWorker(trionic7, () =>
                {
                    if (trionic7.openDevice())
                    {
                        string[] codes = trionic7.ReadDTC();
                        foreach (string a in codes)
                        {
                            AddLogItem(a);
                        }
                    }

                    trionic7.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
            {
                SetGenericOptions(trionic8);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(false))
                    {
                        string[] codes = trionic8.ReadDTC();
                        foreach (string a in codes)
                        {
                            AddLogItem(a);
                        }
                    }

                    trionic8.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
            {
                SetGenericOptions(trionic8);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(false))
                    {
                        string[] codes = trionic8.ReadDTC();
                        foreach (string a in codes)
                        {
                            AddLogItem(a);
                        }
                    }

                    trionic8.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            LogManager.Flush();
        }

        private async void btnEditParameters_Click(object sender, RoutedEventArgs e)
        {
            if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
            {
                trionic7.UseFlasherOnDevice = false;
                SetGenericOptions(trionic7);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                await RunOnWorker(trionic7, () =>
                {
                    if (trionic7.openDevice())
                    {
                        float e85 = trionic7.GetE85Percentage();
                        EditedParameters pi = ShowEditParameters(ECU.TRIONIC7, dlg =>
                        {
                            dlg.E85 = e85;
                        });

                        if (pi != null)
                        {
                            if (!pi.E85.Equals(e85))
                            {
                                if(trionic7.SetE85Percentage((int)pi.E85))
                                {
                                    AddLogItem("Set fields successful, E85Percentage:" + pi.E85);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, E85Percentage:" + pi.E85);
                                }
                            }
                        }
                    }

                    trionic7.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
            {
                SetGenericOptions(trionic8);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                trionic8.SecurityLevel = AccessLevel.AccessLevelFD;
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(true))
                    {
                        float oil = trionic8.GetOilQuality();

                        string vin = trionic8.GetVehicleVIN();

                        bool convertible, sai, highoutput, biopower, clutchStart;
                        TankType tankType;
                        DiagnosticType diagnosticType;
                        string rawPI01;
                        trionic8.GetPI01(out convertible, out sai, out highoutput, out biopower, out diagnosticType, out clutchStart, out tankType, out rawPI01);

                        int topspeed = trionic8.GetTopSpeed();

                        // On a non biopower this call seem to poison the session, do it last!
                        float e85 = trionic8.GetE85Percentage();

                        EditedParameters pi = ShowEditParameters(ECU.TRIONIC8, dlg =>
                        {
                            dlg.Oil = oil;
                            dlg.VIN = vin;
                            dlg.Convertible = convertible;
                            dlg.SAI = sai;
                            dlg.Highoutput = highoutput;
                            dlg.Biopower = biopower;
                            dlg.DiagnosticType = diagnosticType;
                            dlg.TankType = tankType;
                            dlg.ClutchStart = clutchStart;
                            AddLogItem("Read fields");
                            AddLogItem("Convertible:" + dlg.Convertible + " SAI:" + dlg.SAI + " HighOutput:" + dlg.Highoutput + " Biopower:" + dlg.Biopower + " DiagnosticType:" + dlg.DiagnosticType + " ClutchStart:" + dlg.ClutchStart + " TankType:" + dlg.TankType);
                            dlg.TopSpeed = topspeed;
                            dlg.E85 = e85;
                        });

                        if (pi != null)
                        {
                            if (!pi.Convertible.Equals(convertible) || !pi.SAI.Equals(sai) || !pi.Highoutput.Equals(highoutput) || !pi.Biopower.Equals(biopower) || !pi.ClutchStart.Equals(clutchStart) || !pi.DiagnosticType.Equals(diagnosticType) || !pi.TankType.Equals(tankType))
                            {
                                AddLogItem("Detected changed values from user:" + pi.Convertible + " SAI:" + pi.SAI + " HighOutput:" + pi.Highoutput + " Biopower:" + pi.Biopower + " DiagnosticType:" + pi.DiagnosticType + " ClutchStart:" + pi.ClutchStart + " TankType:" + pi.TankType);

                                // Do a second read to make sure the first one was ok
                                bool convertible2, sai2, highoutput2, biopower2, clutchStart2;
                                TankType tankType2;
                                DiagnosticType diagnosticType2;
                                trionic8.GetPI01(out convertible2, out sai2, out highoutput2, out biopower2, out diagnosticType2, out clutchStart2, out tankType2, out rawPI01);
                                if (convertible2.Equals(convertible) && sai2.Equals(sai) && highoutput2.Equals(highoutput) && biopower2.Equals(biopower) && clutchStart2.Equals(clutchStart) && diagnosticType2.Equals(diagnosticType) && tankType2.Equals(tankType))
                                {
                                    if (trionic8.SetPI01(pi.Convertible, pi.SAI, pi.Highoutput, pi.Biopower, pi.DiagnosticType, pi.ClutchStart, pi.TankType))
                                    {
                                        AddLogItem("Set fields successful");
                                    }
                                    else
                                    {
                                        AddLogItem("Set fields failed");
                                    }
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, verification read does not match");
                                }
                            }

                            if (!pi.VIN.Equals(vin))
                            {
                                if(trionic8.SetVIN(pi.VIN))
                                {
                                    AddLogItem("Set fields successful, VIN:" + pi.VIN);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, VIN:" + pi.VIN);
                                }
                            }

                            if (!pi.TopSpeed.Equals(topspeed))
                            {
                                if(trionic8.SetTopSpeed(pi.TopSpeed))
                                {
                                    AddLogItem("Set fields successful, TopSpeed:" + pi.TopSpeed);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, TopSpeed:" + pi.TopSpeed);
                                }
                            }

                            if (!pi.E85.ToString("F2").Equals(e85.ToString("F2")))
                            {
                                if(trionic8.SetE85Percentage(pi.E85))
                                {
                                    AddLogItem("Set fields successful, E85Percentage:" + pi.E85);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, E85Percentage:" + pi.E85);
                                }
                            }

                            if (!pi.Oil.ToString("F2").Equals(oil.ToString("F2")))
                            {
                                if(trionic8.SetOilQuality(pi.Oil))
                                {
                                    AddLogItem("Set fields successful, OilQuality:" + pi.Oil);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, OilQuality:" + pi.Oil);
                                }
                            }
                        }
                    }
                    trionic8.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            else if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
            {
                SetGenericOptions(trionic8);

                EnableUserInput(false);
                AddLogItem("Opening connection");
                trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                await RunOnWorker(trionic8, () =>
                {
                    if (trionic8.openDevice(true))
                    {
                        int topspeed = trionic8.GetTopSpeed();

                        string vin = trionic8.GetVehicleVIN();

                        EditedParameters pi = ShowEditParameters(ECU.MOTRONIC96, dlg =>
                        {
                            dlg.TopSpeed = topspeed;
                            dlg.VIN = vin;
                        });

                        if (pi != null)
                        {
                            if (!pi.TopSpeed.Equals(topspeed))
                            {
                                if(trionic8.SetTopSpeed(pi.TopSpeed))
                                {
                                    AddLogItem("Set fields successful, TopSpeed:" + pi.TopSpeed);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, TopSpeed:" + pi.TopSpeed);
                                }
                            }

                            if (!pi.VIN.Equals(vin))
                            {
                                if (trionic8.ProgramVIN(pi.VIN))
                                {
                                    AddLogItem("Set fields successful, VIN:" + pi.VIN);
                                }
                                else
                                {
                                    AddLogItem("Set fields failed, VIN:" + pi.VIN);
                                }
                            }
                        }
                    }

                    trionic8.Cleanup();
                });
                AddLogItem("Connection closed");
                EnableUserInput(true);
            }
            LogManager.Flush();
        }

        // What EditParameters held when "Write Fields to ECU" closed it, for the worker: the dialog's controls
        // can only be read on the UI thread
        private class EditedParameters
        {
            public bool Convertible, SAI, Highoutput, Biopower, ClutchStart;
            public DiagnosticType DiagnosticType;
            public TankType TankType;
            public string VIN;
            public int TopSpeed;
            public float E85, Oil;
        }

        // From the worker: shows EditParameters, fill sets it up on the UI thread. null unless the user chose to write.
        private EditedParameters ShowEditParameters(ECU ecu, Action<EditParameters> fill)
        {
            return Dialogs.Wait(async () =>
            {
                EditParameters pi = new EditParameters();
                pi.setECU(ecu);
                fill(pi);
                if (!await pi.ShowDialog<bool>(this))
                {
                    return null;
                }
                return new EditedParameters
                {
                    Convertible = pi.Convertible,
                    SAI = pi.SAI,
                    Highoutput = pi.Highoutput,
                    Biopower = pi.Biopower,
                    ClutchStart = pi.ClutchStart,
                    DiagnosticType = pi.DiagnosticType,
                    TankType = pi.TankType,
                    VIN = pi.VIN,
                    TopSpeed = pi.TopSpeed,
                    E85 = pi.E85,
                    Oil = pi.Oil,
                };
            });
        }

        private async void btnReadECUcalibration_Click(object sender, RoutedEventArgs e)
        {
            string fileName = await Dialogs.SaveFile(this, "Bin files", "bin");
            if (fileName != null)
            {
                if (fileName != string.Empty)
                {
                    if (Path.GetFileName(fileName) != string.Empty)
                    {
                        if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
                        {
                            SetGenericOptions(trionic8);

                            EnableUserInput(false);
                            AddLogItem("Opening connection");
                            trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                            bool opened = false;
                            bool ok = await RunOnWorker(trionic8, () =>
                            {
                                opened = trionic8.openDevice(false);
                                if (opened)
                                {
                                    Thread.Sleep(1000);

                                    trionic8.SaveAllDID(fileName);

                                    dtstart = DateTime.Now;
                                    AddLogItem("Acquiring FLASH content");
                                }
                                else
                                {
                                    AddLogItem("Unable to connect to ME9.6 ECU");
                                    trionic8.Cleanup();
                                }
                            });
                            if (opened && ok)
                            {
                                var args = new FlashReadArguments() { FileName = fileName, start = (int)FileME96.EngineCalibrationAddress, end = (int)FileME96.EngineCalibrationAddressEnd };
                                BackgroundWorker bgWorker;
                                bgWorker = new BackgroundWorker();
                                bgWorker.DoWork += new DoWorkEventHandler(trionic8.ReadFlashME96);
                                bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                                bgWorker.RunWorkerAsync(args);
                            }
                            else
                            {
                                EnableUserInput(true);
                                AddLogItem("Connection terminated");
                            }
                        }
                    }
                }
            }
            LogManager.Flush();
        }

        private async void btnRestoreT8_Click(object sender, RoutedEventArgs e)
        {
            await Dialogs.Info(this, "Power on ECU", "Information");

            string fileName = await Dialogs.OpenFile(this, "Bin files", "*.bin");
            if (fileName != null)
            {
                if (checkFileSize(fileName))
                {
                    if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8)
                    {
                        ChecksumResult checksum = ChecksumT8.VerifyChecksum(fileName, AppSettings.AutoChecksum, m_ShouldUpdateChecksum);
                        if (checksum != ChecksumResult.Ok && AppSettings.VerifyChecksum)
                        {
                            AddLogItem("Checksum check failed: " + checksum);
                            return;
                        }

                        SetGenericOptions(trionic8);

                        EnableUserInput(false);
                        AddLogItem("Opening connection");
                        trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                        if (await OpenOnWorker(trionic8, () => trionic8.openDevice(false), "Update FLASH content", "Unable to connect to Trionic 8 ECU"))
                        {
                            BackgroundWorker bgWorker;
                            bgWorker = new BackgroundWorker();
                            bgWorker.DoWork += new DoWorkEventHandler(trionic8.RestoreT8);
                            bgWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bgWorker_RunWorkerCompleted);
                            bgWorker.RunWorkerAsync(fileName);

                        }
                    }
                }
            }
            LogManager.Flush();
        }

        private async void btnLogData_Click(object sender, RoutedEventArgs e)
        {
            if ((string)btnLogData.Content != "Stop" && (string)btnLogData.Content != "Busy..")
            {
                btnLogData.Content = "Busy..";

                // Force logging on
                LogManager.ResumeLogging();
                dtstart = DateTime.Now;
                if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC5)
                {
                    m_bypassCANfilters = true;
                    SetGenericOptions(trionic5);

                    EnableUserInput(false);
                    AddLogItem("Opening connection");
                    bool opened = false;
                    await RunOnWorker(trionic5, () => opened = trionic5.openDevice());
                    if (opened)
                    {
                        StartBGWorkerLog(trionic5);
                        btnLogData.Content = "Stop";
                        btnLogData.IsEnabled = true;
                    }
                    else
                    {
                        // Reset logging to setting
                        UpdateLogManager();
                        btnLogData.Content = "Log Data";
                        EnableUserInput(true);
                    }
                }
                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC7)
                {
                    m_bypassCANfilters = true;
                    trionic7.UseFlasherOnDevice = false;
                    SetGenericOptions(trionic7);

                    EnableUserInput(false);
                    AddLogItem("Opening connection");
                    bool opened = false;
                    await RunOnWorker(trionic7, () => opened = trionic7.openDevice());
                    if (opened)
                    {
                        StartBGWorkerLog(trionic7);
                        btnLogData.Content = "Stop";
                        btnLogData.IsEnabled = true;
                    }
                    else
                    {
                        // Reset logging to setting
                        UpdateLogManager();
                        btnLogData.Content = "Log Data";
                        EnableUserInput(true);
                    }
                }
                else if (cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8 ||
                    cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96    ||
                    cbxEcuType.SelectedIndex == (int)ECU.TRIONIC8_MCP  ||
                    cbxEcuType.SelectedIndex == (int)ECU.Z22SEMain_LEG ||
                    cbxEcuType.SelectedIndex == (int)ECU.Z22SEMCP_LEG)
                {
                    m_bypassCANfilters = true;
                    SetGenericOptions(trionic8);

                    EnableUserInput(false);
                    AddLogItem("Opening connection");
                    trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                    bool opened = false;
                    await RunOnWorker(trionic8, () => opened = trionic8.openDevice(false));
                    if (opened)
                    {
                        StartBGWorkerLog(trionic8);
                        btnLogData.Content = "Stop";
                        btnLogData.IsEnabled = true;
                    }
                    else
                    {
                        // Reset logging to setting
                        UpdateLogManager();
                        btnLogData.Content = "Log Data";
                        EnableUserInput(true);
                    }
                }
            }

            else if ((string)btnLogData.Content != "Busy..")
            {
                bgworkerLogCanData.CancelAsync();
                // Reset logging to setting
                UpdateLogManager();
                btnLogData.Content = "Log Data";
                // the logger stops within a second, then bgWorker_RunWorkerCompleted closes the connection and gives
                // the input back. A new operation started before that lost its connection to that Cleanup.
                // A logger that ended by itself (device gone) already gave the input back: the button stays usable.
                btnLogData.IsEnabled = !m_operationRunning;
            }
        }

        private void cbxEcuType_SelectedIndexChanged(object sender, SelectionChangedEventArgs e)
        {
            // Items.Clear() also lands here, don't forget the remembered ECU because of it
            if (cbxEcuType.SelectedItem == null)
            {
                return;
            }
            AppSettings.SelectedECU.Index = cbxEcuType.SelectedIndex;
            AppSettings.SelectedECU.Name = cbxEcuType.SelectedItem.ToString();
            EnableUserInput(true);
        }

        private async void btnSettings_Click(object sender, RoutedEventArgs e)
        {
            bool LastLoggingState = AppSettings.EnableLogging;

            await new frmSettings(AppSettings).ShowDialog(this);

            if (LastLoggingState != AppSettings.EnableLogging)
            {
                UpdateLogManager();
            }

            EnableUserInput(true);
        }

        // ChecksumDelegate callback, may come from a worker thread; blocks until the user answered
        private bool ShouldUpdateChecksum(string layer, string filechecksum, string realchecksum)
        {
            return Dialogs.Wait(() =>
            {
                AddLogItem(layer);
                AddLogItem("File Checksum: " + filechecksum);
                AddLogItem("Real Checksum: " + realchecksum);

                frmChecksum frm = new frmChecksum() { Layer = layer, FileChecksum = filechecksum, RealChecksum = realchecksum };
                return frm.ShowDialog<bool>(this);
            });
        }

        private void linkLabelLogging_LinkClicked(object sender, RoutedEventArgs e)
        {
            string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "MattiasC", "TrionicCANFlasher");
            Dialogs.OpenWithShell(path);
        }

        private void documentation_LinkClicked(object sender, RoutedEventArgs e)
        {
            Dialogs.OpenWithShell(Path.Combine(AppContext.BaseDirectory, "TrionicCanFlasher.pdf"));
        }

        private void RestoreView()
        {
            if (AppSettings.RememberDimensions)
            {
                Screen primary = Screens.Primary;
                double maxX = primary != null ? primary.Bounds.Width / primary.Scaling : double.MaxValue;
                double maxY = primary != null ? primary.Bounds.Height / primary.Scaling : double.MaxValue;

                // Check if any of the parameters are out of bounds.
                // If it is, use default from .axaml
                if (AppSettings.MainWidth < 374 || AppSettings.MainHeight < 360 ||
                    AppSettings.MainWidth > maxX || AppSettings.MainHeight > maxY)
                {
                    AppSettings.MainWidth = (int)this.Width;
                    AppSettings.MainHeight = (int)this.Height;
                }

                int X = AppSettings.MainWidth;
                int Y = AppSettings.MainHeight;

                // not shown yet, WindowStartupLocation.CenterScreen centers whatever size we end up with
                if (AppSettings.Fullscreen)
                {
                    this.Width = X;
                    this.Height = Y;
                    WindowState = WindowState.Maximized;
                }
                else
                {
                    if (AppSettings.Collapsed)
                    {
                        HandleDynItems(true);
                    }
                    else
                    {
                        this.Width = X;
                        this.Height = Y;
                    }
                }
            }
            else
            {
                AppSettings.MainWidth  = (int)this.Width;
                AppSettings.MainHeight = (int)this.Height;
            }
        }

        // The WinForms version moved the links around and pinned Min/MaximumSize;
        // here the bottom StackPanel makes room for the minilog and the window sizes to the button panel.
        private void HandleDynItems(bool Compact)
        {
            if (Compact)
            {
                textBoxLog.IsVisible = false;
                Minilog.IsVisible = true;

                MinWidth = CompactWidth;
                Width = CompactWidth;
                SizeToContent = SizeToContent.Height;
                CanResize = false;

                this.CanMaximize = false;
                btnCollapse.IsVisible = false;
                btnCollapse.IsEnabled = false;
                btnExpand.IsEnabled = true;
                btnExpand.IsVisible = true;
            }
            else
            {
                textBoxLog.IsVisible = true;
                Minilog.IsVisible = false;
                Minilog.Text = ""; // the WinForms "Mini log" placeholder was never visible

                SizeToContent = SizeToContent.Manual;
                CanResize = true;
                MinWidth = 600;

                btnCollapse.IsVisible = true;
                btnCollapse.IsEnabled = true;
                btnExpand.IsEnabled = false;
                btnExpand.IsVisible = false;
                this.CanMaximize = true;
            }
        }

        private void SetViewMode(bool Compact)
        {
            // keep the right edge (where the buttons are) in place, like the WinForms version did
            int right = Position.X + (int)(Width * DesktopScaling);

            if (Compact)
            {
                AppSettings.Collapsed = true;

                AppSettings.MainWidth  = (int)this.Width;
                AppSettings.MainHeight = (int)this.Height;

                HandleDynItems(true);
                Position = new PixelPoint(right - (int)(CompactWidth * DesktopScaling), Position.Y);
            }

            else if(AppSettings.Collapsed)
            {
                HandleDynItems(false);

                this.Width  = AppSettings.MainWidth;
                this.Height = AppSettings.MainHeight;
                Position = new PixelPoint(right - (int)(AppSettings.MainWidth * DesktopScaling), Position.Y);

                AppSettings.Collapsed = false;
            }
        }

        private void btnExpand_Click(object sender, RoutedEventArgs e)
        {
            SetViewMode(false);
        }

        private void btnCollapse_Click(object sender, RoutedEventArgs e)
        {
            SetViewMode(true);

            // Show last logged item in minilog. If available
            string selected = m_logLines.Count > 0 ? m_logLines[m_logLines.Count - 1] : "";
            if (m_logLines.Count > 0)
            {

                if (selected.Length > 0)
                {
                    string Lastmsg = selected;

                    // Strip time stamp from item
                    if (selected.Length > 15)
                    {
                        if (selected[2] == 0x3A &&
                            selected[5] == 0x3A &&
                            selected[8] == 0x2E)
                        {
                            Lastmsg = Lastmsg.Remove(0, 15);
                        }
                    }

                    // Truncate text that is longer than the progress bar
                    if (Lastmsg.Length > 57)
                    {
                        Lastmsg = Lastmsg.Remove(57, Lastmsg.Length - 57);
                    }

                    Minilog.Text = Lastmsg;
                    Minilog.IsVisible = true;
                }
            }
        }

        protected override void OnPropertyChanged(AvaloniaPropertyChangedEventArgs change)
        {
            base.OnPropertyChanged(change);
            if (change.Property == ClientSizeProperty || change.Property == WindowStateProperty)
            {
                // Win32 reports the maximized size before the state change (WinForms already saw Maximized),
                // look once both have landed or the maximized size gets remembered as the normal one
                Dispatcher.UIThread.Post(frmMainResized);
            }
        }

        // Hide btnCollapse when maximised (For simplicity's sake. Less parameters to keep track of)
        private void frmMainResized()
        {
            if (btnCollapse == null)
            {
                return; // base Window constructor, before InitializeComponent
            }

            AppSettings.Fullscreen = WindowState == WindowState.Maximized ? true : false;

            if (!AppSettings.Collapsed && WindowState != WindowState.Maximized)
            {
                AppSettings.MainWidth = (int)this.Width;
                AppSettings.MainHeight = (int)this.Height;
            }


            if (WindowState != LastWindowState)
            {

                if (WindowState == WindowState.Maximized)
                {
                    btnCollapse.IsVisible = false;
                    btnCollapse.IsEnabled = false;
                }

                else if (LastWindowState == WindowState.Maximized)
                {
                    btnCollapse.IsVisible = true;
                    btnCollapse.IsEnabled = true;
                }

                LastWindowState = WindowState;
            }
        }

        private async void btnWriteDID_Click(object sender, RoutedEventArgs e)
        {
            string fileName = await Dialogs.OpenFile(this, "Did files", "*.did");
            if (fileName != null)
            {
                if (cbxEcuType.SelectedIndex == (int)ECU.MOTRONIC96)
                {
                    SetGenericOptions(trionic8);

                    EnableUserInput(false);
                    AddLogItem("Opening connection");
                    trionic8.SecurityLevel = AccessLevel.AccessLevel01;
                    await RunOnWorker(trionic8, () =>
                    {
                        if (trionic8.openDevice(true))
                        {
                            trionic8.LoadAllDID(fileName);
                        }
                        else
                        {
                            AddLogItem("Unable to connect to ME9.6 ECU");
                        }

                        trionic8.Cleanup();
                    });
                    EnableUserInput(true);
                    AddLogItem("Connection terminated");
                }
            }
            LogManager.Flush();
        }
    }
}
