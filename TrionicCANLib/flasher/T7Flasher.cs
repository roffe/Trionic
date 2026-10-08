using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using TrionicCANLib.KWP;
using System.Threading;
using NLog;
using System.Security.Cryptography;
using System.Text;
using TrionicCANLib.Checksum;

namespace TrionicCANLib.Flasher
{
    //-----------------------------------------------------------------------------
    /// <summary>
    /// T7Flasher handles reading and writing of flash in Trionic 7 ECUs.
    /// 
    /// To use this class a KWPHandler must be set for the communication.
    /// </summary>
    public class T7Flasher : IFlasher
    {
        public override event IFlasher.StatusChanged onStatusChanged;

        private Logger logger = LogManager.GetCurrentClassLogger();

        /// <summary>
        /// This method returns the number of bytes that has been read or written so far.
        /// 0 is returned if there is no read or write session ongoing.
        /// </summary>
        /// <returns>Number of bytes that has been read or written.</returns>
        /// 
        public override int getNrOfBytesRead() { return m_nrOfBytesRead; }

        public static void setKWPHandler(KWPHandler a_kwpHandler)
        {
            m_kwpHandler = a_kwpHandler;
        }

        public ChecksumDelegate.ChecksumUpdate m_ShouldUpdateChecksum;

        // A read request that fails (no reply, adapter error) is resent until it has failed for
        // m_retryTimeMs and at least c_minAttempts times. One KWPHandler call already tries 3 times,
        // so on a silent ECU it fails after 3x the device's reply timeout: 3 s for KWPCANDevice at the
        // default latency (1 s per frame), 60 ms at low latency (20 ms), ~9 s for the ELM327 K-line
        // device (3 s serial read timeout). A live T7 answers in 1-2 ms and a lost frame costs one call.
        // 10 s without an answer is no hiccup: the ECU is halted or rebooted (and has no session left),
        // or the adapter stopped. The 3 attempts give a device whose failed call alone takes ~9 s two
        // retries. A read gives up after ~12 s (default latency), ~10 s (low), ~27 s (K-line).
        internal static int m_retryTimeMs = 10000; // tests shorten it
        private const int c_minAttempts = 3;

        /// <summary>
        /// Constructor for T7Flasher.
        /// </summary>
        /// <param name="a_kwpHandler">The KWPHandler to be used for the communication.</param>
        public T7Flasher()
        {
            m_thread = new Thread(run);
            m_thread.Name = "T7Flasher.m_thread";
            m_thread.Start();
        }

        public override void cleanup()
        {
            lock (m_synchObject)
            {
                m_endThread = true;
            }
            m_resetEvent.Set();
        }

        /// <summary>
        /// Destructor.
        /// </summary>
        ~T7Flasher()
        {
            cleanup();
        }

        /// <summary>
        /// This method starts a reading session.
        /// </summary>
        /// <param name="a_fileName">Name of the file where the flash contents is saved.</param>
        public override void readFlash(string a_fileName)
        {
            base.readFlash(a_fileName);
            lock (m_synchObject)
            {
                m_fileName = a_fileName;
            }
            m_resetEvent.Set();
        }

        /// <summary>
        /// This method starts a reading session for reading memory.
        /// </summary>
        /// <param name="a_fileName">Name of the file where the flash contents is saved.</param>
        /// <param name="a_offset">Starting address to read from.</param>
        /// <param name="a_length">Length to read.</param>
        public override void readMemory(string a_fileName, UInt32 a_offset, UInt32 a_length)
        {
            base.readMemory(a_fileName, a_offset, a_length);
            lock (m_synchObject)
            {
                m_fileName = a_fileName;
                m_offset = a_offset;
                m_length = a_length;
            }
            m_resetEvent.Set();
        }

        /// <summary>
        /// This method starts symbol map.
        /// </summary>
        /// <param name="a_fileName">Name of the file where the flash contents is saved.</param>
        /// <param name="a_offset">Starting address to read from.</param>
        /// <param name="a_length">Length to read.</param>
        public override void readSymbolMap(string a_fileName)
        {
            base.readSymbolMap(a_fileName);
            lock (m_synchObject)
            {
                m_fileName = a_fileName;
            }
            m_resetEvent.Set();
        }

        /// <summary>
        /// This method starts writing to flash.
        /// </summary>
        /// <param name="a_fileName">The name of the file from where to read the data from.</param>
        public override void writeFlash(string a_fileName)
        {
            base.writeFlash(a_fileName);
            lock (m_synchObject)
            {
                m_fileName = a_fileName;
            }
            m_resetEvent.Set();
        }

        bool gotSequrityAccess = false;

        private void SetFlashStatus(FlashStatus status)
        {
            m_flashStatus = status;
            
        }
        /// <summary>
        /// The run method handles writing and reading. It waits for a command to start read
        /// or write and handles this command until it's completed, stopped or until there is 
        /// a failure.
        /// </summary>
        void run()
        {
            while (true)
            {
                logger.Debug("Running T7Flasher");
                m_nrOfRetries = 0;
                m_nrOfBytesRead = 0;
                m_resetEvent.WaitOne(-1, true);
                gotSequrityAccess = false;
                lock (m_synchObject)
                {
                    if (m_endThread)
                    {
                        return;
                    }
                }
                FlashCommand command = m_command;
                // nothing above this thread catches: an exception here used to end the whole process
                try
                {
                    NotifyStatusChanged(this, new StatusEventArgs("Starting session..."));

                    m_kwpHandler.startSession();
                    logger.Debug("Session started");
                    NotifyStatusChanged(this, new StatusEventArgs("Session started, requesting security access to ECU"));
                    if (!gotSequrityAccess)
                    {
                        logger.Debug("No security access");

                        for (int nrOfSequrityTries = 0; nrOfSequrityTries < 5; nrOfSequrityTries++)
                        {

                            if (!KWPHandler.getInstance().requestSequrityAccess(true))
                            {
                                logger.Debug("No security access granted");

                            }
                            else
                            {
                                gotSequrityAccess = true;
                                logger.Debug("Security access granted");

                                break;
                            }
                        }
                    }
                    if (!gotSequrityAccess)
                    {
                        SetFlashStatus(FlashStatus.NoSequrityAccess);
                        logger.Debug("No security access granted after 5 retries");
                        NotifyStatusChanged(this, new StatusEventArgs("Failed to get security access after 5 retries"));
                    }
                    //Here it would make sense to stop if we didn't ge security access but
                    //let's try anyway. It could be that we don't get a possitive reply from the
                    //ECU if we alredy have security access (from a previous, interrupted, session).

                    // Christian: This causes hard crashes with ECUs that have modified key algorithms
                    // Modified code to abort. TODO: Read actual cause and determine if it actually is a failure

                    else if (m_command == FlashCommand.ReadCommand)
                    {
                        ReadCommand();
                    }
                    else if (m_command == FlashCommand.ReadMemoryCommand)
                    {
                        ReadMemoryCommand();
                    }
                    else if (m_command == FlashCommand.ReadSymbolMapCommand)
                    {
                        ReadSymbolMapCommand();
                    }
                    else if (m_command == FlashCommand.WriteCommand)
                    {
                        WriteCommand();
                    }
                }
                catch (Exception e)
                {
                    logger.Error(e, "T7Flasher " + command + " failed");
                    NotifyStatusChanged(this, new StatusEventArgs("Error: " + e.Message));
                    SetFlashStatus(command == FlashCommand.WriteCommand ? FlashStatus.WriteError : FlashStatus.ReadError);
                }

                if (m_endThread)
                    return;
                // Trionic7's read and write timers report a failure only if they see the error
                // status; overwriting it with Completed announced a write that stopped at a refused
                // requestDownload as "Finished FLASH session", and a failed read without a file as
                // "Finished download of FLASH"
                if (m_flashStatus == FlashStatus.WriteError || m_flashStatus == FlashStatus.ReadError ||
                    m_flashStatus == FlashStatus.NoSuchFile || m_flashStatus == FlashStatus.NoSequrityAccess ||
                    m_flashStatus == FlashStatus.EraseError)
                {
                    logger.Debug("T7Flasher failed: " + m_flashStatus);
                    continue;
                }
                NotifyStatusChanged(this, new StatusEventArgs("Flasing procedure completed"));
                logger.Debug("T7Flasher completed");
                SetFlashStatus(FlashStatus.Completed);
            }
        }


        private void ReadCommand()
        {
            const int nrOfBytes = 64;
            byte[] data;
            logger.Debug("Reading flash content to file: " + m_fileName);
            NotifyStatusChanged(this, new StatusEventArgs("Reading data from ECU..."));

            using (MD5 md5Hash = MD5.Create())
            {
                if (File.Exists(m_fileName))
                    File.Delete(m_fileName);
                FileStream fileStream = File.Create(m_fileName, 1024);
                logger.Debug("File created");
                SetFlashStatus(FlashStatus.Reading);
                logger.Debug("Flash status is reading");

                try
                {
                    for (int i = 0; i < 512 * 1024 / nrOfBytes; i++)
                    {
                        if (!readBlock(fileStream, (uint)(nrOfBytes * i), nrOfBytes, out data))
                            return;
                        fileStream.Write(data, 0, nrOfBytes);
                        md5Hash.TransformBlock(data, 0, nrOfBytes, data, 0);
                        m_nrOfBytesRead += nrOfBytes;
                    }
                }
                catch (Exception e)
                {
                    endFailedRead(fileStream, "read error: " + e.Message, false);
                    return;
                }
                fileStream.Close();
                logger.Debug("Closed file");
                // before Completed lets the read timer close the device
                m_kwpHandler.sendDataTransferExitRequest();

                // A complete read is kept whatever its checksums say: an ECU holding a hand-patched
                // image must still be backed up. The read data is never touched: VerifyChecksum's offer
                // to update a mismatch is declined, and a footer it can't parse (it throws) only means
                // the checksums can't be verified.
                string checksumProblem = null;
                try
                {
                    ChecksumResult checksumResult = ChecksumT7.VerifyChecksum(m_fileName, ChecksumT7.DO_NOT_AUTOCORRECT, ChecksumT7.DO_NOT_AUTOFIXFOOTER,
                        (layer, filechecksum, realchecksum) => false);
                    if (checksumResult != ChecksumResult.Ok)
                        checksumProblem = checksumResult.ToString();
                }
                catch (Exception e)
                {
                    checksumProblem = e.Message;
                }
                if (checksumProblem != null)
                {
                    logger.Debug("Checksum verification of the read failed: " + checksumProblem);
                    NotifyStatusChanged(this, new StatusEventArgs("Warning: checksum verification failed (" + checksumProblem +
                        "). The file is saved as read from the ECU; fix its checksums before flashing it"));
                }
                Md5Tools.WriteMd5Hash(md5Hash, m_fileName);
            }

            logger.Debug("Done reading");
        }

        /// <summary>
        /// Reads one block (2C F0 03 + 21 F0) of the read going to a_fileStream. When the read can't go
        /// on (stopped, no reply within the retry bound, device closed, a short reply) it ends it with
        /// endFailedRead.
        /// </summary>
        /// <returns>true with a_length bytes in r_data, false if the read has ended</returns>
        private bool readBlock(FileStream a_fileStream, uint a_address, int a_length, out byte[] r_data)
        {
            byte[] data = null;
            string failure = sendWithRetries(() => m_kwpHandler.sendReadRequest(a_address, (uint)a_length)) ??
                sendWithRetries(() => m_kwpHandler.sendRequestDataByOffset(out data));
            r_data = data;
            if (failure != null)
            {
                // the ECU stopped answering or the device is gone: a stop would only wait for its own
                // timeout, Cleanup's stopSession sends one (a single short try)
                endFailedRead(a_fileStream, failure + " reading 0x" + a_address.ToString("X6"), false);
                return false;
            }
            if (data.Length != a_length)
            {
                // a negative reply, e.g. 7F 21 12 from an ECU still in its EOL loop right after a flash,
                // carries one byte; writing a whole block of them threw on this thread and took the
                // process down
                endFailedRead(a_fileStream, "unexpected reply length " + data.Length + " reading 0x" + a_address.ToString("X6"), true);
                return false;
            }
            return true;
        }

        /// <summary>
        /// Sends a read request until it succeeds, within the retry bound (m_retryTimeMs).
        /// </summary>
        /// <returns>null on success, otherwise why it gave up</returns>
        private string sendWithRetries(Func<bool> a_request)
        {
            Stopwatch sw = Stopwatch.StartNew();
            for (int attempt = 1; ; attempt++)
            {
                lock (m_synchObject)
                {
                    if (m_endThread || m_command == FlashCommand.StopCommand)
                        return "stopped";
                }
                if (a_request())
                    return null;
                m_nrOfRetries++;
                // a closed device fails every request at once (DeviceNotConnected): don't spin on it
                if (!m_kwpHandler.isDeviceOpen())
                    return "device closed";
                if (attempt >= c_minAttempts && sw.ElapsedMilliseconds >= m_retryTimeMs)
                    return "no reply after " + attempt + " attempts in " + sw.ElapsedMilliseconds + " ms";
                // an adapter that fails every send instantly would otherwise spin a core for the whole bound
                Thread.Sleep(20);
            }
        }

        /// <summary>
        /// Ends a read that can't complete: the partial file is closed and deleted, and the read fails
        /// with ReadError, unless the flasher is being cleaned up. With a_endSession the read-end
        /// stopCommunication goes out first, to an ECU that still answers: once the read timer sees
        /// the error the GUI closes the device.
        /// </summary>
        private void endFailedRead(FileStream a_fileStream, string a_reason, bool a_endSession)
        {
            logger.Debug("Read failed: " + a_reason);
            a_fileStream.Close();
            if (File.Exists(m_fileName))
                File.Delete(m_fileName);
            lock (m_synchObject)
            {
                if (m_endThread)
                    return;
            }
            NotifyStatusChanged(this, new StatusEventArgs("Failed to read data from ECU: " + a_reason));
            if (a_endSession)
                m_kwpHandler.sendDataTransferExitRequest();
            SetFlashStatus(FlashStatus.ReadError);
        }

        private void ReadMemoryCommand()
        {
            int nrOfBytes = 64;
            byte[] data;
            NotifyStatusChanged(this, new StatusEventArgs("Reading data from ECU..."));

            if (File.Exists(m_fileName))
                File.Delete(m_fileName);
            FileStream fileStream = File.Create(m_fileName, 1024);
            int nrOfReads = (int)m_length / nrOfBytes;
            try
            {
                for (int i = 0; i < nrOfReads; i++)
                {
                    SetFlashStatus(FlashStatus.Reading);

                    if (i == nrOfReads - 1)
                        nrOfBytes = (int)m_length - nrOfBytes * i;
                    if (!readBlock(fileStream, (uint)m_offset + (uint)(nrOfBytes * i), nrOfBytes, out data))
                        return;
                    logger.Debug("Writing data to file: " + m_length + " bytes");
                    fileStream.Write(data, 0, nrOfBytes);
                    m_nrOfBytesRead += nrOfBytes;
                }
            }
            catch (Exception e)
            {
                endFailedRead(fileStream, "read error: " + e.Message, false);
                return;
            }
            fileStream.Close();
            logger.Debug("Done reading");
            m_kwpHandler.sendDataTransferExitRequest();
        }

        private void ReadSymbolMapCommand()
        {
            byte[] data;
            string swVersion = "";
            m_nrOfBytesRead = 0;
            NotifyStatusChanged(this, new StatusEventArgs("Reading symbol map from ECU..."));

            if (File.Exists(m_fileName))
                File.Delete(m_fileName);
            FileStream fileStream = File.Create(m_fileName, 1024);
            if (m_kwpHandler.sendUnknownRequest() != KWPResult.OK)
            {
                NotifyStatusChanged(this, new StatusEventArgs("Failed to read data from ECU..."));
                SetFlashStatus(FlashStatus.ReadError);
                return;
            }
            SetFlashStatus(FlashStatus.Reading);
            m_kwpHandler.getSwVersionFromDR51(out swVersion);

            if (m_kwpHandler.sendReadSymbolMapRequest() != KWPResult.OK)
            {
                NotifyStatusChanged(this, new StatusEventArgs("Failed to read data from ECU..."));

                SetFlashStatus(FlashStatus.ReadError);
                return;
            }
            m_kwpHandler.sendDataTransferRequest(out data);
            while (data.Length > 0x10)
            {
                fileStream.Write(data, 1, data.Length - 3);
                m_nrOfBytesRead += data.Length - 3;
                bool stop;
                lock (m_synchObject)
                {
                    stop = m_command == FlashCommand.StopCommand || m_endThread;
                }
                if (stop)
                {
                    // a continue here wrote the same block to the file again and again, without end
                    endFailedRead(fileStream, "stopped", false);
                    return;
                }
                m_kwpHandler.sendDataTransferRequest(out data);
            }
            fileStream.Flush();
            fileStream.Close();
        }

        private void WriteCommand()
        {
            logger.Debug("Write command seen");
            const int nrOfBytes = 128;
            int i = 0;
            byte[] data = new byte[nrOfBytes];
            if (!gotSequrityAccess)
            {
                // nothing was written: Completed announced it as "Finished FLASH session"
                SetFlashStatus(FlashStatus.NoSequrityAccess);
                return;
            }
            if (!File.Exists(m_fileName))
            {
                SetFlashStatus(FlashStatus.NoSuchFile);
                logger.Debug("No such file found: " + m_fileName);
                NotifyStatusChanged(this, new StatusEventArgs("Failed to find file to flash..."));

                return;
            }
            logger.Debug("Start erasing");
            NotifyStatusChanged(this, new StatusEventArgs("Erasing flash..."));

            SetFlashStatus(FlashStatus.Eraseing);
            if (m_kwpHandler.sendEraseRequest() != KWPResult.OK)
            {
                NotifyStatusChanged(this, new StatusEventArgs("Failed to erase flash..."));
                logger.Debug("Erase error occured");
                // the write used to go on here (a commented-out break) and send every block into a
                // flash that wasn't erased
                SetFlashStatus(FlashStatus.EraseError);
                return;
            }
            logger.Debug("Opening file for reading");

            using FileStream fs = new FileStream(m_fileName, FileMode.Open, FileAccess.Read);

            SetFlashStatus(FlashStatus.Writing);
            logger.Debug("Set flash status to writing");
            NotifyStatusChanged(this, new StatusEventArgs("Writing flash... 0x00000-0x7B000"));

            //Write 0x0-0x7B000
            logger.Debug("0x0-0x7B000");
            Thread.Sleep(100);
            if (m_kwpHandler.sendWriteRequest(0x0, 0x7B000) != KWPResult.OK)
            {
                NotifyStatusChanged(this, new StatusEventArgs("Failed to write data to flash..."));

                SetFlashStatus(FlashStatus.WriteError);
                logger.Debug("Write error occured");

                return;
            }
            for (i = 0; i < 0x7B000 / nrOfBytes; i++)
            {
                fs.ReadExactly(data, 0, nrOfBytes);
                m_nrOfBytesRead = i * nrOfBytes;
                logger.Debug("sendWriteDataRequest " + m_nrOfBytesRead);
                if (m_kwpHandler.sendWriteDataRequest(data) != KWPResult.OK)
                {
                    // stop here: going on would leave a 128-byte gap (sendWriteDataRequest has
                    // already resent the block for as long as the ECU said busy)
                    NotifyStatusChanged(this, new StatusEventArgs("Failed to write data to flash..."));
                    SetFlashStatus(FlashStatus.WriteError);
                    logger.Debug("Write error occured " + m_nrOfBytesRead);
                    fs.Close();
                    return;
                }
                if (writeStopped(fs))
                    return;
            }

            //Write 0x7FE00-0x7FFFF
            logger.Debug("Write 0x7FE00-0x7FFFF");
            NotifyStatusChanged(this, new StatusEventArgs("Writing flash... 0x7FE00-0x7FFFF"));

            if (m_kwpHandler.sendWriteRequest(0x7FE00, 0x200) != KWPResult.OK)
            {
                NotifyStatusChanged(this, new StatusEventArgs("Failed to write data to flash..."));
                SetFlashStatus(FlashStatus.WriteError);
                logger.Debug("Write error occured");
                return;
            }
            fs.Seek(0x7FE00, System.IO.SeekOrigin.Begin);
            for (i = 0x7FE00 / nrOfBytes; i < 0x80000 / nrOfBytes; i++)
            {
                fs.ReadExactly(data, 0, nrOfBytes);
                m_nrOfBytesRead = i * nrOfBytes;
                logger.Debug("sendWriteDataRequest " + m_nrOfBytesRead.ToString());

                if (m_kwpHandler.sendWriteDataRequest(data) != KWPResult.OK)
                {
                    NotifyStatusChanged(this, new StatusEventArgs("Failed to write data to flash..."));
                    SetFlashStatus(FlashStatus.WriteError);
                    logger.Debug("Write error occured " + m_nrOfBytesRead);
                    fs.Close();
                    return;
                }
                if (writeStopped(fs))
                    return;
            }
            fs.Close();
        }

        /// <summary>
        /// stopFlasher or cleanup during a write: no further block is sent (a continue here went on
        /// with the next block). The flash is left part written, a write error for the write timer.
        /// </summary>
        /// <returns>true if the write has to end</returns>
        private bool writeStopped(FileStream a_fs)
        {
            bool endThread;
            lock (m_synchObject)
            {
                endThread = m_endThread;
                if (!endThread && m_command != FlashCommand.StopCommand)
                    return false;
            }
            logger.Debug("Write stopped at " + m_nrOfBytesRead);
            a_fs.Close();
            if (!endThread)
            {
                NotifyStatusChanged(this, new StatusEventArgs("Write stopped, the flash is incomplete"));
                SetFlashStatus(FlashStatus.WriteError);
            }
            return true;
        }

        private void NotifyStatusChanged(T7Flasher t7Flasher, StatusEventArgs statusEventArgs)
        {
            if (onStatusChanged != null)
                onStatusChanged(t7Flasher, statusEventArgs);
        }


        private readonly Thread m_thread;
        private readonly AutoResetEvent m_resetEvent = new AutoResetEvent(false);
        private string m_fileName;
        private static KWPHandler m_kwpHandler;
        private int m_nrOfRetries;
        private int m_nrOfBytesRead;
        private bool m_endThread = false;
        private UInt32 m_offset;
        private UInt32 m_length;
    }
}
