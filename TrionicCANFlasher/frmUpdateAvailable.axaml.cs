using System;
using Avalonia.Controls;
using Avalonia.Interactivity;

namespace TrionicCANFlasher
{
    /// <summary>ShowDialog&lt;bool&gt;: true = OK (update), false = Ignore or closed.</summary>
    public partial class frmUpdateAvailable : Window
    {
        public frmUpdateAvailable()
        {
            InitializeComponent();
        }

        public void SetVersionNumber(string version)
        {
            label1.Text = "Available version: " + version;
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            this.Close(true);
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            this.Close(false);
        }
    }
}
