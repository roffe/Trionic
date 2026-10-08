//-------------------------------------------------------------------------------------------------
//	CombiAdapter .NET library
//	(C) Janis Silins, 2010-2019
//
//	Managed port of combilib-net (C++/CLI, which cannot load on .NET Core) on LibUsbDotNet 3 and
//	libusb-1.0. Only the CAN side is kept: BDM, NVRAM, firmware update and the FTDI / USBBDM
//	adapters were never used by TrionicCANLib. USB transport details follow gocan's hardware-tested
//	adapters/combi, the flash operations follow the combilib-net C++ source (unchanged since the DLL
//	that shipped with Trionic CAN Flasher, apart from the default timeout).
//-------------------------------------------------------------------------------------------------
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Threading;
using LibUsbDotNet;
using LibUsbDotNet.LibUsb;
using LibUsbDotNet.Main;
using NLog;

namespace Combi
{

//-------------------------------------------------------------------------------------------------
/**
    Interface class for a LPC17xx based CAN/BDM adapter (USB VID 0xFFFF, PID 0x0005).
*/
public class caCombiAdapter
{
    // CAN frame storage structure
    public struct caCANFrame
    {
        public uint id;                 ///< ID
        public byte length;             ///< data block length, bytes
        public ulong data;              ///< data block
        public byte is_extended;        ///< ID format
        public byte is_remote;          ///< frame type
    }

    // command terminators
    public const byte term_ack = 0x0;   ///< command acknowledged
    public const byte term_nak = 0xff;  ///< command failed

    // commands
    const byte cmd_brd_fwversion =  0x20;   ///< firmware version
    const byte cmd_brd_adcfilter =  0x21;   ///< ADC filter settings
    const byte cmd_brd_adc =        0x22;   ///< ADC value
    const byte cmd_brd_egt =        0x23;   ///< EGT value
    const byte cmd_can_open =       0x80;   ///< open/close CAN channel
    const byte cmd_can_bitrate =    0x81;   ///< set bitrate
    const byte cmd_can_frame =      0x82;   ///< incoming frame
    const byte cmd_can_txframe =    0x83;   ///< outgoing frame
    const byte cmd_can_filter =     0x84;   ///< acceptance filter, firmware >= 1.2 (unused)
    const byte cmd_can_ecuconnect = 0x89;   ///< connect / disconnect ECU
    const byte cmd_can_readflash =  0x8a;   ///< read ECU flash
    const byte cmd_can_writeflash = 0x8b;   ///< write ECU flash

    // misc constants
    const uint packet_timeout = 1000;       ///< default comms timeout as shipped; the later source has 10000, too long for a lost TX ack mid KWP session
    const uint transfer_block_size = 256;   ///< transfer block size, bytes
    const uint adc_num_channels = 5;        ///< number of A/D channels
    const int vid = 0xffff;
    const int pid = 0x0005;
    const int usb_interface = 1;            ///< the second interface; the original took WinUSB's associated interface 0
    const byte ep_in = 0x82;
    const byte ep_out = 0x05;

    // flash size by ECU index: T5.2, T5.5 (28F010), T5.5 (29F010), T7, T8
    static readonly uint[] flash_size = { 0x20000, 0x40000, 0x40000, 0x80000, 0x100000 };

    static readonly Logger logger = LogManager.GetCurrentClassLogger();

    // USB connection
    readonly object usb_lock = new object();
    readonly object cmd_lock = new object();
    UsbContext usb_ctx;
    UsbDevice usb_dev;
    UsbEndpointReader ep_reader;
    UsbEndpointWriter ep_writer;
    Thread read_thread;
    volatile bool read_stop;

    // incoming data; responses are queued per command code so a late reply to a timed out
    // command can't be taken for the answer to the next one
    caPacketParser parser;
    readonly BlockingCollection<(byte[] data, byte term)>[] response_queue = new BlockingCollection<(byte[] data, byte term)>[256];
    readonly BlockingCollection<caCANFrame> frame_queue = new BlockingCollection<caCANFrame>();

    // ECU session and async operation
    int selected_ecu = -1;
    string file_name;
    byte flash_method;
    volatile Thread operation_thread;
    volatile uint operation_progress;
    volatile bool operation_succeeded;
    volatile Exception operation_exception;

    //---------------------------------------------------------------------------------------------
    /**
        Opens a new USB connection to adapter; the previous connection, if any, will be closed.
    */
    public void Open()
    {
        Close();

        try
        {
            int in_size;
            lock (usb_lock)
            {
                usb_ctx = new UsbContext();
                usb_dev = (UsbDevice)usb_ctx.Find(d => d.VendorId == vid && d.ProductId == pid);
                if (usb_dev == null)
                {
                    throw new Exception("No compatible adapters found");
                }

                usb_dev.Open();
                usb_dev.SetAutoDetachKernelDriver(true);
                usb_dev.ClaimInterface(usb_interface);

                // STM32 based clones have the OUT endpoint on EP2 instead of EP5; like gocan, keep EP5
                // when the descriptor can't be read (LIBUSB_ERROR_OTHER) and only switch when it's absent
                ep_writer = usb_dev.OpenEndpointWriter(usb_dev.GetMaxPacketSize(ep_out) != (int)Error.NotFound ? WriteEndpointID.Ep05 : WriteEndpointID.Ep02);
                ep_reader = usb_dev.OpenEndpointReader(ReadEndpointID.Ep02);
                in_size = usb_dev.GetMaxPacketSize(ep_in);
            }

            // known state: close CAN so the bus stops streaming, then drop stale input
            write_usb(BuildPacket(cmd_can_open, new byte[] { 0 }, term_ack), packet_timeout);
            ep_reader.ReadFlush();
            foreach (var q in response_queue)
            {
                drain(q);
            }
            drain(frame_queue);

            // fresh parser: the last session may have ended mid-packet
            parser = new caPacketParser(process_packet);
            read_stop = false;
            read_thread = new Thread(read_usb) { Name = "Combi.read_usb", IsBackground = true };
            read_thread.Start(in_size > 0 ? in_size : 64);

            CAN_Open(false);
        }

        catch (Exception e)
        {
            // keep the inner UsbException / DllNotFoundException, LPCCANDevice explains those
            close_usb();
            throw new Exception("Failed to connect to adapter:\n" + (e is DllNotFoundException ? "libusb-1.0 not found" : e.Message), e);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Closes the current USB connection to adapter.
    */
    public void Close()
    {
        if (!IsOpen())
        {
            return;
        }

        if (OperationRunning())
        {
            end_operation();
        }

        // best effort, the USB side is released regardless
        try
        {
            CAN_DisconnectECU(false);
            CAN_Open(false);
        }
        catch (Exception e)
        {
            logger.Debug("Close: " + e.Message);
        }

        close_usb();
    }

    public bool IsOpen()
    {
        UsbDevice dev = usb_dev;
        return dev != null && dev.IsOpen;
    }

    //---------------------------------------------------------------------------------------------
    /**
        Returns adapter's firmware version number (major << 8 | minor).
    */
    public ushort GetFirmwareVersion()
    {
        try
        {
            return BitConverter.ToUInt16(send_command(cmd_brd_fwversion, null, 2, packet_timeout), 0);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to get firmware version:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Checks if ADC low-pass filter is enabled.

        @param      channel     A/D channel number [0...4]
    */
    public bool GetADCFiltering(uint channel)
    {
        try
        {
            if (channel >= adc_num_channels)
            {
                throw new Exception("Unknown channel number");
            }

            return send_command(cmd_brd_adcfilter, new byte[] { (byte)channel }, 1, packet_timeout)[0] == 0x1;
        }
        catch (Exception e)
        {
            throw new Exception("Failed to get A/D filter settings:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Enables / disables low-pass filtering for an ADC channel and stores the setting in EEPROM.
    */
    public void SetADCFiltering(uint channel, bool enable)
    {
        try
        {
            if (channel >= adc_num_channels)
            {
                throw new Exception("Unknown channel number");
            }

            send_command(cmd_brd_adcfilter, new byte[] { (byte)channel, (byte)(enable ? 1 : 0) }, 0, packet_timeout);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to set A/D filter flag:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Returns momentary voltage from A/D converter, V; works in all modes.
    */
    public float GetADCValue(uint channel)
    {
        try
        {
            if (channel >= adc_num_channels)
            {
                throw new Exception("Unknown channel number");
            }

            return BitConverter.ToSingle(send_command(cmd_brd_adc, new byte[] { (byte)channel }, 4, packet_timeout), 0);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to read A/D value:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Returns current temperature from K-type thermocouple, DegC.
    */
    public float GetThermoValue()
    {
        try
        {
            return BitConverter.ToSingle(send_command(cmd_brd_egt, null, 5, packet_timeout), 1);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to read EGT value:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Establishes a new communications session with an ECU via CAN bus; if a session is already
        active, it will not be interrupted.

        @param      _selected_ecu       ECU index: 0 T5.2, 1 T5.5 28F010, 2 T5.5 29F010, 3 T7, 4 T8
    */
    public void CAN_ConnectECU(int _selected_ecu)
    {
        if (_selected_ecu < 0 || _selected_ecu >= flash_size.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(_selected_ecu));
        }

        if (selected_ecu == _selected_ecu)
        {
            // already connected
            return;
        }

        try
        {
            send_command(cmd_can_ecuconnect, new byte[] { 1, (byte)_selected_ecu }, 0, 10000);
            selected_ecu = _selected_ecu;
        }
        catch (Exception e)
        {
            throw new Exception("Failed to connect to ECU:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Disconnects from a previously established ECU communications session.

        @param      reset       reset ECU after disconnecting
    */
    public void CAN_DisconnectECU(bool reset)
    {
        if (selected_ecu < 0)
        {
            // already disconnected
            return;
        }

        try
        {
            send_command(cmd_can_ecuconnect, new byte[] { 0, (byte)(reset ? 1 : 0) }, 0, packet_timeout);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to disconnect from ECU:\n" + e.Message);
        }
        finally
        {
            selected_ecu = -1;
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Asynchronously reads contents of flash memory into file; connection to the ECU must be
        established first. Poll OperationRunning() / GetOperationProgress(), then OperationSucceeded().
    */
    public void CAN_ReadFlash(string _file_name)
    {
        try
        {
            if (selected_ecu < 0)
            {
                throw new Exception("Not connected to ECU");
            }

            if (OperationRunning())
            {
                throw new Exception("Operation already in progress");
            }

            file_name = _file_name;
            begin_operation(read_flash_can);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to read flash:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Asynchronously writes contents of file to flash memory; see CAN_ReadFlash().

        @param      method      flashing method; 1 also strips the T7 VIN and immo fields
    */
    public void CAN_WriteFlash(string _file_name, byte method)
    {
        try
        {
            if (selected_ecu < 0)
            {
                throw new Exception("Not connected to ECU");
            }

            if (OperationRunning())
            {
                throw new Exception("Operation already in progress");
            }

            file_name = _file_name;
            flash_method = method;
            begin_operation(write_flash_can);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to write flash:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Sets CAN bus bitrate (bps); available when channel is closed.
    */
    public void CAN_SetBitrate(uint bitrate)
    {
        try
        {
            send_command(cmd_can_bitrate, new byte[] { (byte)(bitrate >> 24), (byte)(bitrate >> 16), (byte)(bitrate >> 8), (byte)bitrate }, 0, packet_timeout);
        }
        catch (Exception e)
        {
            throw new Exception("Failed to set CAN bitrate:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Opens or closes the CAN channel; bitrate must be set first.
    */
    public void CAN_Open(bool open)
    {
        try
        {
            send_command(cmd_can_open, new byte[] { (byte)(open ? 1 : 0) }, 0, packet_timeout);
        }
        catch (Exception e)
        {
            throw new Exception((open ? "Failed to open CAN channel:\n" : "Failed to close CAN channel:\n") + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Reads and removes the oldest CAN frame from receive queue; fails if the queue is empty or
        timeout (ms, 0 = don't wait) expires.
    */
    public bool CAN_GetMessage(ref caCANFrame frame, uint timeout)
    {
        if (frame_queue.TryTake(out caCANFrame f, (int)timeout))
        {
            frame = f;
            return true;
        }

        return false;
    }

    //---------------------------------------------------------------------------------------------
    /**
        Sends a CAN data frame and waits for the adapter's ack.
    */
    public void CAN_SendMessage(ref caCANFrame frame)
    {
        try
        {
            byte[] pkt = BuildPacket(cmd_can_txframe, EncodeFrame(frame), term_ack);
            lock (cmd_lock)
            {
                // the firmware NAKs the frame (without queueing it) while its single TX buffer is
                // still busy: lost arbitration or a slow bus. Resend for up to 100 ms like gocan.
                Stopwatch sw = Stopwatch.StartNew();
                while (true)
                {
                    clear_responses(cmd_can_txframe);
                    write_usb(pkt, packet_timeout);
                    if (wait_response(cmd_can_txframe, packet_timeout).term == term_ack)
                    {
                        return;
                    }

                    if (sw.ElapsedMilliseconds > 100)
                    {
                        throw new Exception("Command failed");
                    }
                }
            }
        }
        catch (Exception e)
        {
            throw new Exception("Failed to send CAN message:\n" + e.Message);
        }
    }

    //---------------------------------------------------------------------------------------------
    // async operations

    public bool OperationRunning()
    {
        Thread t = operation_thread;
        return t != null && t.IsAlive;
    }

    public uint GetOperationProgress()
    {
        return operation_progress;
    }

    public bool OperationSucceeded()
    {
        return operation_succeeded;
    }

    public Exception GetOperationException()
    {
        return operation_exception;
    }

    void begin_operation(ThreadStart thread_start)
    {
        operation_progress = 0;
        operation_succeeded = false;
        operation_exception = null;

        Thread t = new Thread(thread_start) { Name = "Combi.operation" };
        operation_thread = t;
        t.Start();
    }

    void end_operation()
    {
        operation_thread = null;
    }

    // interrupts the operation running on the adapter
    void abort_operation(byte cmd_code, Exception e, string what)
    {
        operation_exception = e.Message.IndexOf('\n') != -1 ? e : new Exception(what + ":\n" + e.Message);
        logger.Debug(operation_exception.Message);

        try
        {
            write_usb(BuildPacket(cmd_code, null, term_nak), packet_timeout);
        }
        catch (Exception)
        {
            // adapter gone
        }

        Thread.Sleep(100);
        clear_responses(cmd_code);
    }

    //---------------------------------------------------------------------------------------------
    /**
        Reads contents of ECU flash memory into file via CAN.
    */
    void read_flash_can()
    {
        try
        {
            uint size = flash_size[selected_ecu];
            uint crc = 0xffffffff;

            using (FileStream fs = File.Open(file_name, FileMode.Create))
            {
                uint bytes_read = 0;
                byte[] data_in;

                while (bytes_read < size)
                {
                    // the adapter streams all blocks after a single request
                    data_in = bytes_read == 0 ?
                        send_command(cmd_can_readflash, null, transfer_block_size, packet_timeout) :
                        get_response(cmd_can_readflash, transfer_block_size, packet_timeout);

                    bytes_read += transfer_block_size;
                    fs.Write(data_in, 0, data_in.Length);
                    crc = AddCrc32(crc, data_in);
                    operation_progress = bytes_read;
                }

                // compare with the adapter's checksum
                data_in = get_response(cmd_can_readflash, 4, packet_timeout);
                if (~crc != BitConverter.ToUInt32(data_in, 0))
                {
                    throw new Exception("Checksums do not match");
                }
            }

            operation_succeeded = true;
        }
        catch (Exception e)
        {
            abort_operation(cmd_can_readflash, e, "Failed to read flash");
        }
        finally
        {
            end_operation();
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Writes file contents into ECU flash memory via CAN.
    */
    void write_flash_can()
    {
        try
        {
            int ecu = selected_ecu;
            uint size = flash_size[ecu];

            // short files are zero padded, as before
            byte[] bin_buf = new byte[size];
            using (FileStream fs = File.OpenRead(file_name))
            {
                fs.ReadAtLeast(bin_buf, bin_buf.Length, false);
            }

            // check data signature, prepare for flashing
            if (ecu == 3)
            {
                if (bin_buf[0] != 0xff || bin_buf[1] != 0xff || bin_buf[2] != 0xef || bin_buf[3] != 0xfc)
                {
                    throw new Exception("File is not a Trionic 7 binary!");
                }

                if (flash_method == 1)
                {
                    // remove VIN and immo fields
                    strip_header_t7(bin_buf);
                }
            }

            uint crc = 0xffffffff;

            // start writing
            send_command(cmd_can_writeflash, new byte[] { flash_method }, 0, 5000);

            byte[] cmd_data = new byte[transfer_block_size];
            uint bytes_written = 0;

            while (bytes_written < size)
            {
                // the first block also waits for the erase
                Array.Copy(bin_buf, bytes_written, cmd_data, 0, transfer_block_size);
                send_command(cmd_can_writeflash, cmd_data, 0, bytes_written > 0 ? packet_timeout : 20000);

                bytes_written += transfer_block_size;
                crc = AddCrc32(crc, cmd_data);
                operation_progress = bytes_written;
            }

            // compare with the adapter's checksum
            cmd_data = get_response(cmd_can_writeflash, 4, packet_timeout);
            if (~crc != BitConverter.ToUInt32(cmd_data, 0))
            {
                throw new Exception("Checksums do not match");
            }

            operation_succeeded = true;
        }
        catch (Exception e)
        {
            abort_operation(cmd_can_writeflash, e, "Failed to write flash");
        }
        finally
        {
            end_operation();
        }
    }

    //---------------------------------------------------------------------------------------------
    /**
        Strips header info from a T7 binary file.
    */
    static void strip_header_t7(byte[] bin_buf)
    {
        uint addr = 0x7ffff;

        while (addr > 0x7fd00)
        {
            // field length
            uint field_len = bin_buf[addr];
            if (field_len == 0x0 || field_len == 0xff)
            {
                break;
            }
            --addr;

            // field ID
            byte field_id = bin_buf[addr];
            --addr;

            if (field_id == 0x92)
            {
                // remove header
                addr -= field_len;
                while (addr > 0x7fd00)
                {
                    bin_buf[addr] = 0xff;
                    --addr;
                }

                return;
            }

            addr -= field_len;
        }
    }

    //---------------------------------------------------------------------------------------------
    // USB communications

    // sends a command and waits for its response; returns the response data or null
    byte[] send_command(byte cmd_code, byte[] cmd_data, uint reply_data_len, uint timeout)
    {
        lock (cmd_lock)
        {
            clear_responses(cmd_code);
            write_usb(BuildPacket(cmd_code, cmd_data, term_ack), timeout);
            return get_response(cmd_code, reply_data_len, timeout);
        }
    }

    byte[] get_response(byte cmd_code, uint reply_data_len, uint timeout)
    {
        var pack = wait_response(cmd_code, timeout);

        if (pack.term != term_ack)
        {
            // adapter reported command failure
            throw new Exception("Command failed");
        }

        if (pack.data.Length != reply_data_len)
        {
            throw new Exception("Unexpected response data length");
        }

        return reply_data_len > 0 ? pack.data : null;
    }

    (byte[] data, byte term) wait_response(byte cmd_code, uint timeout)
    {
        if (!responses(cmd_code).TryTake(out var pack, (int)timeout))
        {
            throw new Exception("Command timed out");
        }

        return pack;
    }

    BlockingCollection<(byte[] data, byte term)> responses(byte cmd_code)
    {
        return LazyInitializer.EnsureInitialized(ref response_queue[cmd_code]);
    }

    void clear_responses(byte cmd_code)
    {
        drain(responses(cmd_code));
    }

    static void drain<T>(BlockingCollection<T> q)
    {
        while (q != null && q.TryTake(out _))
        {
        }
    }

    // writes a packet to the adapter; silently does nothing when closed, like the original
    void write_usb(byte[] buf, uint timeout)
    {
        lock (usb_lock)
        {
            if (ep_writer == null)
            {
                return;
            }

            if (ep_writer.Write(buf, (int)timeout, out int bytes_written) != Error.Success || bytes_written != buf.Length)
            {
                throw new Exception("Failed to write to adapter");
            }
        }
    }

    void read_usb(object in_size)
    {
        // one packet per transfer: the firmware never sends a ZLP after a full packet, so a longer
        // read could sit on received data until the timeout
        UsbEndpointReader reader = ep_reader;
        byte[] buf = new byte[(int)in_size];

        while (!read_stop)
        {
            Error err = reader.Read(buf, 100, out int count);
            if (count > 0)
            {
                parser.Feed(buf, count);
            }

            if (err != Error.Success && err != Error.Timeout)
            {
                logger.Debug("USB read failed: " + err);
                return;
            }
        }
    }

    void process_packet(byte cmd_code, byte[] data, byte term)
    {
        if (cmd_code == cmd_can_frame)
        {
            if (data.Length == 15 && term == term_ack)
            {
                frame_queue.Add(DecodeFrame(data));
            }
            return;
        }

        responses(cmd_code).Add((data, term));
    }

    void close_usb()
    {
        read_stop = true;
        if (read_thread != null && read_thread != Thread.CurrentThread)
        {
            read_thread.Join();
        }
        read_thread = null;

        lock (usb_lock)
        {
            ep_reader = null;
            ep_writer = null;

            if (usb_dev != null)
            {
                try
                {
                    usb_dev.ReleaseInterface(usb_interface);
                }
                catch (Exception)
                {
                    // not claimed or not open
                }

                usb_dev.Dispose();
                usb_dev = null;
            }

            usb_ctx?.Dispose();
            usb_ctx = null;
        }
    }

    //---------------------------------------------------------------------------------------------
    // wire format, public for TrionicCANLibTest

    /** Builds a packet: command, data size (u16 BE), data, terminator. */
    public static byte[] BuildPacket(byte cmd_code, byte[] cmd_data, byte term)
    {
        int len = cmd_data != null ? cmd_data.Length : 0;
        byte[] pkt = new byte[len + 4];
        pkt[0] = cmd_code;
        pkt[1] = (byte)(len >> 8);
        pkt[2] = (byte)len;
        cmd_data?.CopyTo(pkt, 3);
        pkt[len + 3] = term;
        return pkt;
    }

    /** CAN frame packet data: ID (u32 LE), data (u64 LE), length, extended, remote. */
    public static byte[] EncodeFrame(caCANFrame frame)
    {
        byte[] data = new byte[15];
        BitConverter.GetBytes(frame.id).CopyTo(data, 0);
        BitConverter.GetBytes(frame.data).CopyTo(data, 4);
        data[12] = frame.length;
        data[13] = frame.is_extended;
        data[14] = frame.is_remote;
        return data;
    }

    public static caCANFrame DecodeFrame(byte[] data)
    {
        caCANFrame frame;
        frame.id = BitConverter.ToUInt32(data, 0);
        frame.data = BitConverter.ToUInt64(data, 4);
        frame.length = data[12];
        frame.is_extended = data[13];
        frame.is_remote = data[14];
        return frame;
    }

    /**
        CRC-32 (the zip one) as the adapter computes it over flash contents: start with 0xffffffff,
        invert the result. Same value as the original bit-reflected MSB-first loop.
    */
    public static uint AddCrc32(uint crc, byte[] data)
    {
        foreach (byte b in data)
        {
            crc ^= b;
            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
            }
        }

        return crc;
    }

    //---------------------------------------------------------------------------------------------
    /**
        Incremental parser for the adapter's USB stream; packets may span USB transfers.
    */
    public class caPacketParser
    {
        const int max_size = 1024;

        readonly Action<byte, byte[], byte> on_packet;     ///< (command, data, terminator)
        int state;
        byte cmd_code;
        int size;
        byte[] data;
        int pos;

        public caPacketParser(Action<byte, byte[], byte> _on_packet)
        {
            on_packet = _on_packet;
        }

        public void Feed(byte[] buf, int count)
        {
            for (int i = 0; i < count; ++i)
            {
                byte b = buf[i];
                switch (state)
                {
                    case 0:
                        // command; skip bytes that can't start a packet to resync (gocan)
                        if (is_command(b))
                        {
                            cmd_code = b;
                            state = 1;
                        }
                        break;

                    case 1:
                        size = b << 8;
                        state = 2;
                        break;

                    case 2:
                        size |= b;
                        if (size >= max_size)
                        {
                            state = 0;
                            break;
                        }
                        data = size > 0 ? new byte[size] : Array.Empty<byte>();
                        pos = 0;
                        state = size > 0 ? 3 : 4;
                        break;

                    case 3:
                        data[pos++] = b;
                        if (pos == size)
                        {
                            state = 4;
                        }
                        break;

                    case 4:
                        // terminator
                        state = 0;
                        on_packet(cmd_code, data, b);
                        break;
                }
            }
        }

        static bool is_command(byte b)
        {
            return (b >= cmd_brd_fwversion && b <= cmd_brd_egt) ||
                (b >= cmd_can_open && b <= cmd_can_filter) ||
                (b >= cmd_can_ecuconnect && b <= cmd_can_writeflash);
        }
    }
};

}   // end namespace
//-------------------------------------------------------------------------------------------------
//  EOF
//-------------------------------------------------------------------------------------------------
