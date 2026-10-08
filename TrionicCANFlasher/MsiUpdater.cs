using System;
using System.Diagnostics;
using System.Net;
using System.Net.Http;
using System.Text.Json;


namespace CommonSuite
{
    class MsiUpdater
    {
        private readonly Version m_currentversion;
        private string m_githubURL = "";
        private Version m_NewVersion;
        private bool m_blockauto_updates;

        public bool Blockauto_updates
        {
            get { return m_blockauto_updates; }
            set { m_blockauto_updates = value; }
        }

        public Version NewVersion
        {
            get { return m_NewVersion; }
            set { m_NewVersion = value; }
        }
        public delegate void DataPump(MSIUpdaterEventArgs e);
        public event MsiUpdater.DataPump onDataPump;

        public class MSIUpdaterEventArgs : System.EventArgs
        {
            private string _Data;
            private bool _UpdateAvailable;
            private bool _Version2High;
            private Version _Version;
            // private string _xmlFile;


            private string _msiFile;

            public string MSIFile
            {
                get
                {
                    return _msiFile;
                }
            }
            public string Data
            {
                get
                {
                    return _Data;
                }
            }
            public bool UpdateAvailable
            {
                get
                {
                    return _UpdateAvailable;
                }
            }
            public bool Version2High
            {
                get
                {
                    return _Version2High;
                }
            }
            public Version Version
            {
                get
                {
                    return _Version;
                }
            }
            public MSIUpdaterEventArgs(string Data, bool Update, bool mVersion2High, Version version, string msiFile)
            {
                _Data = Data;
                _UpdateAvailable = Update;
                _Version2High = mVersion2High;
                _Version = version;
                _msiFile = msiFile;
            }
        }

        public MsiUpdater(Version CurrentVersion)
        {
            m_currentversion = CurrentVersion;
            m_NewVersion = new Version("0.0.0.0");
        }

        public void CheckForUpdates(string githubUrl)
        {
            m_githubURL = githubUrl;
            if (!m_blockauto_updates)
            {
                System.Threading.Thread t = new System.Threading.Thread(updatecheck) { IsBackground = true };
                t.Start();
            }
        }

        public void ExecuteUpdate(string msiFile)
        {
            try
            {
                // a URL: the browser downloads the msi (Windows) or shows the release page
                Process.Start(new ProcessStartInfo(msiFile) { UseShellExecute = true });
            }
            catch (Exception E)
            {
                PumpString("Exception when checking new update(s): " + E.Message, false, false, new Version(), "");
            }
        }


        private void PumpString(string text, bool updateavailable, bool version2high, Version version, string msiFile)
        {
            onDataPump(new MSIUpdaterEventArgs(text, updateavailable, version2high, version, msiFile));
        }

        public string GetPageHTML(string pageUrl, int timeoutSeconds)
        {
            try
            {
                using (HttpClient client = new HttpClient(new HttpClientHandler() { DefaultProxyCredentials = CredentialCache.DefaultNetworkCredentials }))
                {
                    client.Timeout = TimeSpan.FromSeconds(timeoutSeconds);
                    client.DefaultRequestHeaders.UserAgent.ParseAdd("Mozilla/5.0");
                    return client.GetStringAsync(pageUrl).GetAwaiter().GetResult();
                }
            }
            catch (Exception ex)
            {
                // Error occured grabbing data, return empty string.
                PumpString("An error occurred while retrieving the HTML content. " + ex.Message, false, false, new Version(), "");

                return "";
            }
        }

        private void updatecheck()
        {
            string releaseInfo = "";
            bool m_updateavailable = false;
            bool m_version_toohigh = false;
            Version maxversion = new Version("0.0.0.0");
            string msiFile = "";

            try
            {
                releaseInfo = GetPageHTML(m_githubURL, 10);
                using JsonDocument doc = JsonDocument.Parse(releaseInfo);
                JsonElement release = doc.RootElement;

                string tag_name = release.GetProperty("tag_name").GetString(); // "TrionicCanFlasher_v0.1.72.0"
                int index = tag_name.IndexOf("_v", 0, tag_name.Length - 1, StringComparison.CurrentCulture);
                Version v = new Version(tag_name.Substring(index + 2));
                if (v > m_currentversion)
                {
                    if (v > maxversion) maxversion = v;
                    m_updateavailable = true;
                    PumpString("Available version: " + tag_name, false, false, v, "");
                }
                else if (v.Major < m_currentversion.Major || (v.Major == m_currentversion.Major && v.Minor < m_currentversion.Minor) || (v.Major == m_currentversion.Major && v.Minor == m_currentversion.Minor && v.Build < m_currentversion.Build))
                {
                    // mmm .. gebruiker draait een versie die hoger is dan dat is vrijgegeven... 
                    if (v > maxversion) maxversion = v;
                    m_updateavailable = false;
                    m_version_toohigh = true;
                }

                foreach (JsonElement asset in release.GetProperty("assets").EnumerateArray())
                {
                    string browser_download_url = asset.GetProperty("browser_download_url").GetString();
                    if (browser_download_url.Contains("msi"))
                    {
                        msiFile = browser_download_url;
                    }
                }

                // ponytail: only Windows has an installer, everyone else gets the release page
                if (msiFile == "" || !OperatingSystem.IsWindows())
                {
                    msiFile = release.GetProperty("html_url").GetString();
                }

                if (m_updateavailable)
                {
                    PumpString("A newer version is available: " + maxversion.ToString(), m_updateavailable, m_version_toohigh, v, msiFile);
                    m_NewVersion = maxversion;
                }
                else if (m_version_toohigh)
                {
                    PumpString("Versionnumber is too high: " + maxversion.ToString(), m_updateavailable, m_version_toohigh, v, msiFile);
                    m_NewVersion = maxversion;
                }
                else
                {
                    PumpString("No new version(s) found...", false, false, new Version(), "");
                }
            }
            catch (Exception tuE)
            {
                PumpString(tuE.Message, false, false, new Version(), "");
            }

        }
    }
}
