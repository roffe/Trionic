using Avalonia.Controls;
using Avalonia.Interactivity;
using TrionicCANLib.API;

namespace TrionicCANFlasher
{
    /// <summary>
    /// ShowDialog&lt;bool&gt; returns true for "Write Fields to ECU", false otherwise.
    /// </summary>
    public partial class EditParameters : Window
    {
        public EditParameters()
        {
            InitializeComponent();
        }

        public bool Convertible
        {
            get
            {
                return cbCab.IsChecked == true;
            }
            set
            {
                cbCab.IsChecked = value;
            }
        }

        public bool SAI
        {
            get
            {
                return cbSAI.IsChecked == true;
            }
            set
            {
                cbSAI.IsChecked = value;
            }
        }

        public bool Highoutput
        {
            get
            {
                return cbOutput.IsChecked == true;
            }
            set
            {
                cbOutput.IsChecked = value;
            }
        }

        public bool Biopower
        {
            get
            {
                return cbBiopower.IsChecked == true;
            }
            set
            {
                cbBiopower.IsChecked = value;
            }
        }

        public string VIN
        {
            get
            {
                // WinForms never returned null here and callers rely on that
                return tbVIN.Text ?? string.Empty;
            }
            set
            {
                tbVIN.Text = value;
            }
        }

        public int TopSpeed
        {
            get
            {
                int speed;
                int.TryParse(tbTopSpeed.Text, out speed);
                return speed;
            }
            set
            {
                tbTopSpeed.Text = value.ToString();
            }
        }

        public float E85
        {
            get
            {
                float e85;
                float.TryParse(tbE85.Text, out e85);
                return e85;
            }
            set
            {
                tbE85.Text = value.ToString();
            }
        }

        public float Oil
        {
            get
            {
                float oil;
                float.TryParse(tbOilQuality.Text, out oil);
                return oil;
            }
            set
            {
                tbOilQuality.Text = value.ToString();
            }
        }

        public bool ClutchStart
        {
            get
            {
                return cbClutchStart.IsChecked == true;
            }
            set
            {
                cbClutchStart.IsChecked = value;
            }
        }

        public DiagnosticType DiagnosticType
        {
            get
            {
                return (DiagnosticType)comboBoxDiag.SelectedIndex;
            }
            set
            {
                comboBoxDiag.SelectedIndex = (int)value;
            }
        }

        public TankType TankType
        {
            get
            {
                return (TankType)comboBoxTank.SelectedIndex;
            }
            set
            {
                comboBoxTank.SelectedIndex = (int)value;
            }
        }

        // The labels follow their field's IsVisible (bound in the axaml)
        public void setECU(ECU ecu)
        {
            if(ecu == ECU.TRIONIC8)
            {
                tbVIN.IsVisible = true;
                cbCab.IsVisible = true;
                cbSAI.IsVisible = true;
                cbOutput.IsVisible = true;
                tbTopSpeed.IsVisible = true;
                tbE85.IsVisible = true;
                tbOilQuality.IsVisible = true;
                cbBiopower.IsVisible = true;
                cbClutchStart.IsVisible = true;
                comboBoxDiag.IsVisible = true;
                comboBoxTank.IsVisible = true;
            }
            else if(ecu == ECU.MOTRONIC96)
            {
                tbVIN.IsVisible = true;
                cbCab.IsVisible = false;
                cbSAI.IsVisible = false;
                cbOutput.IsVisible = false;
                tbTopSpeed.IsVisible = true;
                tbE85.IsVisible = false;
                tbOilQuality.IsVisible = false;
                cbBiopower.IsVisible = false;
                cbClutchStart.IsVisible = false;
                comboBoxDiag.IsVisible = false;
                comboBoxTank.IsVisible = false;
            }
            else if(ecu == ECU.TRIONIC7)
            {
                tbVIN.IsVisible = false;
                cbCab.IsVisible = false;
                cbSAI.IsVisible = false;
                cbOutput.IsVisible = false;
                tbTopSpeed.IsVisible = false;
                tbE85.IsVisible = true;
                tbOilQuality.IsVisible = false;
                cbBiopower.IsVisible = false;
                cbClutchStart.IsVisible = false;
                comboBoxDiag.IsVisible = false;
                comboBoxTank.IsVisible = false;
            }
        }

        private void btnWriteToECU_Click(object sender, RoutedEventArgs e)
        {
            this.Close(true);
        }

        private void closeButton_Click(object sender, RoutedEventArgs e)
        {
            this.Close(false);
        }
    }
}
