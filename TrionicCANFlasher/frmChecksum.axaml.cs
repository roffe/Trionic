using Avalonia.Controls;
using Avalonia.Interactivity;

namespace TrionicCANFlasher
{
    /// <summary>
    /// ShowDialog&lt;bool&gt; returns true for "Update and close", false for Ignore or the title bar close.
    /// </summary>
    public partial class frmChecksum : Window
    {
        public frmChecksum()
        {
            InitializeComponent();
        }

        public string Layer
        {
            set
            {
                groupBox1.Header = value;
            }
        }

        public string FileChecksum
        {
            set
            {
                textBox1.Text = value;
            }
        }

        public string RealChecksum
        {
            set
            {
                textBox2.Text = value;
            }
        }

        private void btnUpdate_Click(object sender, RoutedEventArgs e)
        {
            this.Close(true);
        }

        private void btnIgnore_Click(object sender, RoutedEventArgs e)
        {
            this.Close(false);
        }
    }
}
