using System;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using TrionicCANLib.API;
using NLog;

namespace TrionicCANFlasher
{
    /// <summary>
    /// Settings dialog. Edits a FlasherSettings; only Save writes back into it.
    /// </summary>
    public partial class frmSettings : Window
    {
        private Logger logger = LogManager.GetCurrentClassLogger();
        private readonly FlasherSettings m_settings;

        // Hidden features
        private bool cbEnableSUFeatures = false; // This mode does not have a checkbox
        private int  m_hiddenclicks     = 5;     // Click this many times + 1 to enable su features

        // Used to lock out SettingsLogic while populating items
        private bool m_lockout = true;

        private void LoadItems()
        {
            // Lock out logics while populating items
            m_lockout = true;

            try
            {
                if (m_settings.AdapterType.Name != null)
                {
                    Select(cbxAdapterType, m_settings.AdapterType.Name);
                }

                if (m_settings.Adapter.Name != null)
                {
                    Select(cbxAdapterItem, m_settings.Adapter.Name);
                }

                if (m_settings.Baudrate.Name != null)
                {
                    Select(cbxComSpeed, m_settings.Baudrate.Name);
                }

                if (m_settings.InterframeDelay.Name != null)
                {
                    Select(cbxInterFrame, m_settings.InterframeDelay.Name);
                }
            }

            catch (Exception ex)
            {
                logger.Debug(ex.Message);
            }

            cbOnlyPBus.IsChecked = m_settings.OnlyPBus;
            cbEnableLogging.IsChecked = m_settings.EnableLogging;
            cbUseLegion.IsChecked = m_settings.UseLegion;
            cbOnboardFlasher.IsChecked = m_settings.CombiFlasher;

            cbPowerUser.IsChecked = m_settings.PowerUser;
            cbUnlockSys.IsChecked = m_settings.UnlockSys;
            cbUnlockBoot.IsChecked = m_settings.UnlockBoot;
            cbAutoChecksum.IsChecked = m_settings.AutoChecksum;

            // This is not a real checkbox.
            cbEnableSUFeatures = m_settings.SuperUser;

            cbUseLastPointer.IsChecked = m_settings.UseLastMarker;
            cbVerifyChecksum.IsChecked = m_settings.VerifyChecksum;
            cbFaster.IsChecked = m_settings.Faster;

            cbRemember.IsChecked = m_settings.RememberDimensions;

            m_lockout = false;
        }

        private void StoreItems()
        {
            try
            {
                if (cbxAdapterType.SelectedIndex >= 0)
                {
                    m_settings.AdapterType.Index = cbxAdapterType.SelectedIndex;
                    m_settings.AdapterType.Name = cbxAdapterType.SelectedItem.ToString();
                }

                if (cbxAdapterItem.SelectedIndex >= 0)
                {
                    m_settings.Adapter.Index = cbxAdapterItem.SelectedIndex;
                    m_settings.Adapter.Name = cbxAdapterItem.SelectedItem.ToString();
                }

                if (cbxComSpeed.SelectedIndex >= 0)
                {
                    m_settings.Baudrate.Index = cbxComSpeed.SelectedIndex;
                    m_settings.Baudrate.Name = cbxComSpeed.SelectedItem.ToString();
                }

                if (cbxInterFrame.SelectedIndex >= 0)
                {
                    m_settings.InterframeDelay.Index = cbxInterFrame.SelectedIndex;
                    m_settings.InterframeDelay.Name = cbxInterFrame.SelectedItem.ToString();
                }
            }

            catch (Exception ex)
            {
                logger.Debug(ex.Message);
            }

            m_settings.OnlyPBus = cbOnlyPBus.IsChecked == true;
            m_settings.EnableLogging = cbEnableLogging.IsChecked == true;
            m_settings.UseLegion = cbUseLegion.IsChecked == true;
            m_settings.CombiFlasher = cbOnboardFlasher.IsChecked == true;

            m_settings.PowerUser = cbPowerUser.IsChecked == true;
            m_settings.UnlockSys = cbUnlockSys.IsChecked == true;
            m_settings.UnlockBoot = cbUnlockBoot.IsChecked == true;
            m_settings.AutoChecksum = cbAutoChecksum.IsChecked == true;

            // This is not a real checkbox.
            m_settings.SuperUser = cbEnableSUFeatures;

            m_settings.UseLastMarker = cbUseLastPointer.IsChecked == true;
            m_settings.VerifyChecksum = cbVerifyChecksum.IsChecked == true;
            m_settings.Faster = cbFaster.IsChecked == true;

            m_settings.RememberDimensions = cbRemember.IsChecked == true;
        }

        /// <summary>
        /// WinForms' SelectedItem = name, which left the selection alone when name wasn't in the list
        /// </summary>
        private static void Select(ComboBox cbx, string name)
        {
            int index = cbx.Items.IndexOf(name);
            if (index >= 0)
            {
                cbx.SelectedIndex = index;
            }
        }

        private void GetAdapterInformation()
        {
            if (cbxAdapterType.SelectedIndex >= 0)
            {
                logger.Debug("ITrionic.GetAdapterNames selectedIndex=" + cbxAdapterType.SelectedIndex);
                string[] adapters = ITrionic.GetAdapterNames((CANBusAdapter)cbxAdapterType.SelectedIndex);
                cbxAdapterItem.ItemsSource = adapters;
                foreach (string adapter in adapters)
                {
                    logger.Debug("Adaptername=" + adapter);
                }

                if (adapters.Length > 0)
                {
                    cbxAdapterItem.SelectedIndex = 0;
                }
            }
        }

        /// <summary>
        /// This method determines what should be enabled / shown depending on what the user has selected
        /// </summary>
        private void SettingsLogic()
        {
            // Check if we're being populated or if the user did something
            if (!m_lockout)
            {

                int typeindex = cbxAdapterType.SelectedIndex;
                int ecuindex = m_settings.SelectedECU.Index;

                cbOnboardFlasher.IsEnabled = false;
                cbEnableLogging.IsEnabled = true;
                cbUseLegion.IsEnabled = false;
                cbOnlyPBus.IsEnabled = true;
                cbPowerUser.IsEnabled = true;
                cbUnlockSys.IsEnabled = false;
                cbUnlockBoot.IsEnabled = false;
                cbAutoChecksum.IsEnabled = false;

                if (cbEnableSUFeatures && cbPowerUser.IsChecked == true)
                {
                    cbPowerUser.Content = "I definitely know what I am doing";
                }
                else
                {
                    cbPowerUser.Content = "I know what I am doing";
                }

                cbxAdapterItem.IsEnabled = false;
                AdapterLabel.IsEnabled = false;

                switch (typeindex)
                {
                    case (int)CANBusAdapter.LAWICEL:
                    case (int)CANBusAdapter.KVASER:
                    case (int)CANBusAdapter.J2534:
                        cbxAdapterItem.IsEnabled = true;
                        AdapterLabel.IsEnabled = true;
                        break;
                    case (int)CANBusAdapter.ELM327:
                    case (int)CANBusAdapter.SLCAN:
                        cbxAdapterItem.IsEnabled = true;
                        AdapterLabel.IsEnabled = true;
                        ComBaudLabel.IsEnabled = true;
                        cbxComSpeed.IsEnabled = true;
                        break;
                    case (int)CANBusAdapter.JUST4TRIONIC:
                        // macOS ports carry no USB id, so the library falls back to the port picked here
                        cbxAdapterItem.IsEnabled = OperatingSystem.IsMacOS();
                        AdapterLabel.IsEnabled = OperatingSystem.IsMacOS();
                        ComBaudLabel.IsEnabled = false;
                        cbxComSpeed.IsEnabled = false;
                        break;
                    default:
                        ComBaudLabel.IsEnabled = false;
                        cbxComSpeed.IsEnabled = false;
                        break;
                }

                if (typeindex >= 0)
                {
                    if (ecuindex == (int)ECU.TRIONIC5)
                    {
                        cbOnlyPBus.IsEnabled = false;
                        cbAutoChecksum.IsEnabled = true;
                    }

                    else if (ecuindex == (int)ECU.TRIONIC7)
                    {
                        cbOnboardFlasher.IsEnabled = typeindex == (int)CANBusAdapter.COMBI ? cbOnlyPBus.IsChecked == true : false;
                        cbAutoChecksum.IsEnabled = true;
                    }

                    else if (ecuindex == (int)ECU.TRIONIC8)
                    {
                        cbUseLegion.IsEnabled = true;
                        cbUnlockSys.IsEnabled = cbUseLegion.IsChecked == true;
                        cbUnlockBoot.IsEnabled = (cbUnlockSys.IsChecked == true && cbUnlockSys.IsEnabled);

                        if (cbUnlockSys.IsChecked != true)
                        {
                            cbUnlockBoot.IsChecked = false;
                        }
                        cbAutoChecksum.IsEnabled = true;
                    }

                    else if (ecuindex == (int)ECU.TRIONIC8_MCP)
                    {
                        cbUseLegion.IsChecked = true;
                        cbUnlockBoot.IsEnabled = true;
                    }

                    else if (ecuindex == (int)ECU.MOTRONIC96)
                    {
                        cbUnlockSys.IsEnabled = true;
                    }

                    // Other ECUs do not have power user features
                    else
                    {
                        cbPowerUser.IsEnabled = cbEnableSUFeatures;
                    }

                    if (!cbPowerUser.IsEnabled)
                    {
                        cbUnlockSys.IsEnabled = false;
                        cbUnlockBoot.IsEnabled = false;
                        cbAutoChecksum.IsEnabled = false;
                    }

                    if (cbPowerUser.IsChecked != true)
                    {
                        cbUnlockSys.IsEnabled = false;
                        cbUnlockBoot.IsEnabled = false;
                        cbAutoChecksum.IsEnabled = false;
                        cbUnlockSys.IsChecked = false;
                        cbUnlockBoot.IsChecked = false;
                        cbAutoChecksum.IsChecked = false;

                        // Can not be super user without being a power user first
                        cbEnableSUFeatures = false;

                        // Restore safe settings
                        cbUnlockBoot.IsChecked = false;
                        cbUnlockSys.IsChecked = false;
                        cbAutoChecksum.IsChecked = false;
                    }

                    if (cbEnableSUFeatures)
                    {
                        bool precheck = ((ecuindex == (int)ECU.TRIONIC8 && cbUseLegion.IsChecked == true && cbUseLegion.IsEnabled) ||
                            ecuindex == (int)ECU.TRIONIC8_MCP || ecuindex == (int)ECU.Z22SEMain_LEG || ecuindex == (int)ECU.Z22SEMCP_LEG);

                        InterframeLabel.IsEnabled = precheck;
                        cbxInterFrame.IsEnabled = precheck;
                        cbUseLastPointer.IsEnabled = (ecuindex == (int)ECU.TRIONIC8 && cbUseLegion.IsChecked == true && cbUseLegion.IsEnabled);
                        cbFaster.IsEnabled = precheck;
                        cbVerifyChecksum.IsEnabled = (ecuindex == (int)ECU.TRIONIC8 || ecuindex == (int)ECU.TRIONIC7 || ecuindex == (int)ECU.TRIONIC5);
                    }
                    else
                    {
                        InterframeLabel.IsEnabled = false;
                        cbUseLastPointer.IsEnabled = false;
                        cbFaster.IsEnabled = false;
                        cbxInterFrame.IsEnabled = false;
                        cbVerifyChecksum.IsEnabled = false;

                        // Restore safe settings
                        cbFaster.IsChecked = false;
                        cbUseLastPointer.IsChecked = true;
                        cbVerifyChecksum.IsChecked = true;
                        cbxInterFrame.SelectedIndex = FlasherSettings.m_interframe.DefaultIndex;
                    }
                }

                // No adapter selected, blank everything!
                else
                {
                    cbPowerUser.IsEnabled = false;
                    cbEnableLogging.IsEnabled = false;
                    cbAutoChecksum.IsEnabled = false;
                    cbOnlyPBus.IsEnabled = false;
                }
            }
        }

        /// <summary>
        /// Dialog is about to be shown. Now populate it with current settings
        /// </summary>
        private void DialogShown()
        {
            // Reset click counter
            if (m_hiddenclicks > 0)
            {
                m_hiddenclicks = 5;
            }

            // Restore the regular label
            label2.Text = "Advanced features";

            LoadItems();
            SettingsLogic();
        }

        private void cbxAdapterType_SelectedIndexChanged(object sender, SelectionChangedEventArgs e)
        {
            if (cbxAdapterType.SelectedIndex == (int)CANBusAdapter.JUST4TRIONIC)
            {
                cbxComSpeed.SelectedIndex = (int)ComSpeed.S115200;
            }

            // Prevent checkboxes from popping in and out as the adapter name is loaded
            SettingsLogic();

            GetAdapterInformation();

            // Now perform a real check
            SettingsLogic();
        }

        private void cbOnlyPBus_Checkchanged(object sender, RoutedEventArgs e)
        {
            SettingsLogic();
        }

        private void cbUseLegion_CheckedChanged(object sender, RoutedEventArgs e)
        {
            SettingsLogic();
        }

        private void cbUnlockSys_CheckedChanged(object sender, RoutedEventArgs e)
        {
            SettingsLogic();
        }

        private void cbPowerUser_CheckedChanged(object sender, RoutedEventArgs e)
        {
            // Ignore changes made by "LoadItems"
            if (!m_lockout)
            {
                if (cbEnableSUFeatures && cbPowerUser.IsChecked != true)
                {
                    label2.Text = "Advanced features";
                    cbPowerUser.IsChecked = true;
                    cbEnableSUFeatures = false;
                    m_hiddenclicks = 5;
                }

                SettingsLogic();
            }
        }

        private void btnSave_Click(object sender, RoutedEventArgs e)
        {
            StoreItems();
            this.Close();
        }

        private void bntDiscard_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        /// <summary>
        /// Enable super user options by clicking "Advanced features" 6 times
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void label2_Click(object sender, PointerPressedEventArgs e)
        {
            if (!cbEnableSUFeatures && cbPowerUser.IsChecked == true)
            {
                if (m_hiddenclicks > 0)
                {
                    m_hiddenclicks--;
                }
                else
                {
                    label2.Text = "You are in deep water now..";
                    cbEnableSUFeatures = true;
                    m_hiddenclicks = 5;
                    SettingsLogic();
                }
            }

            // Feature is already enabled
            else if (cbPowerUser.IsChecked == true)
            {
                if (m_hiddenclicks > 0)
                {
                    m_hiddenclicks--;
                }
                else
                {
                    m_hiddenclicks = 5;
                    label2.Text = "Already a super user";
                }
            }
        }

        // For the XAML previewer / runtime loader only
        public frmSettings() : this(new FlasherSettings())
        {
        }

        public frmSettings(FlasherSettings settings)
        {
            InitializeComponent();
            m_settings = settings;
            cbxAdapterType.ItemsSource = FlasherSettings.AdapterTypeNames;
            cbxComSpeed.ItemsSource = FlasherSettings.ComSpeedNames;
            cbxInterFrame.ItemsSource = FlasherSettings.InterframeDelayNames;
            DialogShown();
        }
    }
}
