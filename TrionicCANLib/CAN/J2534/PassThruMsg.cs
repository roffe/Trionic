using System;
using System.Runtime.InteropServices;

namespace TrionicCANLib.CAN.J2534
{
    // J2534-1 v04.04 values, every "unsigned long" is 32 bits (also in the Linux drivers, they use uint32_t)

    public enum J2534Err
    {
        STATUS_NOERROR = 0,
        ERR_NOT_SUPPORTED = 1,
        ERR_INVALID_CHANNEL_ID = 2,
        ERR_INVALID_PROTOCOL_ID = 3,
        ERR_NULL_PARAMETER = 4,
        ERR_INVALID_IOCTL_VALUE = 5,
        ERR_INVALID_FLAGS = 6,
        ERR_FAILED = 7,
        ERR_DEVICE_NOT_CONNECTED = 8,
        ERR_TIMEOUT = 9,
        ERR_INVALID_MSG = 10,
        ERR_INVALID_TIME_INTERVAL = 11,
        ERR_EXCEEDED_LIMIT = 12,
        ERR_INVALID_MSG_ID = 13,
        ERR_DEVICE_IN_USE = 14,
        ERR_INVALID_IOCTL_ID = 15,
        ERR_BUFFER_EMPTY = 16,
        ERR_BUFFER_FULL = 17,
        ERR_BUFFER_OVERFLOW = 18,
        ERR_PIN_INVALID = 19,
        ERR_CHANNEL_IN_USE = 20,
        ERR_MSG_PROTOCOL_ID = 21,
        ERR_INVALID_FILTER_ID = 22,
        ERR_NO_FLOW_CONTROL = 23,
        ERR_NOT_UNIQUE = 24,
        ERR_INVALID_BAUDRATE = 25,
        ERR_INVALID_DEVICE_ID = 26
    }

    public enum ProtocolID
    {
        J1850VPW = 1,
        J1850PWM,
        ISO9141,
        ISO14230,
        CAN,
        ISO15765,
        SCI_A_ENGINE,
        SCI_A_TRANS,
        SCI_B_ENGINE,
        SCI_B_TRANS
    }

    public enum BaudRate
    {
        ISO9141 = 10400,
        ISO9141_10400 = ISO9141,
        ISO9141_10000 = 10000,
        ISO14230 = ISO9141,
        ISO14230_10400 = ISO9141,
        ISO14230_10000 = ISO9141_10000,
        J1850PWM = 41600,
        J1850PWM_41600 = J1850PWM,
        J1850PWM_83300 = 83300,
        J1850VPW = ISO9141,
        J1850VPW_10400 = ISO9141,
        J1850VPW_41600 = J1850PWM,
        CAN = 500000,
        CAN_125000 = 125000,
        CAN_250000 = 250000,
        CAN_500000 = CAN,
        CAN_615000 = 615000,
        ISO15765 = CAN,
        ISO15765_125000 = CAN_125000,
        ISO15765_250000 = CAN_250000,
        ISO15765_500000 = CAN
    }

    [Flags]
    public enum ConnectFlag
    {
        NONE = 0,
        ISO9141_K_LINE_ONLY = 0x1000,
        CAN_ID_BOTH = 0x800,
        ISO9141_NO_CHECKSUM = 0x200,
        CAN_29BIT_ID = 0x100
    }

    public enum FilterType
    {
        PASS_FILTER = 1,
        BLOCK_FILTER,
        FLOW_CONTROL_FILTER
    }

    public enum Ioctl
    {
        GET_CONFIG = 1,
        SET_CONFIG = 2,
        READ_VBATT = 3,
        FIVE_BAUD_INIT = 4,
        FAST_INIT = 5,
        CLEAR_TX_BUFFER = 7,
        CLEAR_RX_BUFFER = 8,
        CLEAR_PERIODIC_MSGS = 9,
        CLEAR_MSG_FILTERS = 10,
        CLEAR_FUNCT_MSG_LOOKUP_TABLE = 11,
        ADD_TO_FUNCT_MSG_LOOKUP_TABLE = 12,
        DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE = 13,
        READ_PROG_VOLTAGE = 14
    }

    [Flags]
    public enum RxStatus
    {
        NONE = 0,
        TX_MSG_TYPE = 1,
        START_OF_MESSAGE = 2,
        RX_BREAK = 4,
        TX_INDICATION = 8,
        ISO15765_PADDING_ERROR = 0x10,
        ISO15765_ADDR_TYPE = 0x80,
        CAN_29BIT_ID = 0x100
    }

    [Flags]
    public enum TxFlag
    {
        NONE = 0,
        SCI_TX_VOLTAGE = 0x800000,
        SCI_MODE = 0x400000,
        WAIT_P3_MIN_ONLY = 0x200,
        CAN_29BIT_ID = 0x100,
        ISO15765_ADDR_TYPE = 0x80,
        ISO15765_FRAME_PAD = 0x40
    }

    /// <summary>
    /// PASSTHRU_MSG, six 32-bit fields and the data array: 4152 bytes on every platform.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct PassThruMsg
    {
        public const int DataCapacity = 4128;

        public ProtocolID ProtocolID;
        public RxStatus RxStatus;
        public TxFlag TxFlags;
        public uint Timestamp;
        public uint DataSize;
        public uint ExtraDataIndex;
        public fixed byte Data[DataCapacity];

        public PassThruMsg(ProtocolID myProtocolId, TxFlag myTxFlag, byte[] myByteArray)
        {
            ProtocolID = myProtocolId;
            TxFlags = myTxFlag;
            SetBytes(myByteArray);
        }

        public void SetBytes(byte[] myByteArray)
        {
            fixed (byte* data = Data)
            {
                myByteArray.CopyTo(new Span<byte>(data, DataCapacity));
            }
            // No extra data: the index equals the size, like gocan sends it
            DataSize = ExtraDataIndex = (uint)myByteArray.Length;
        }

        public byte[] GetBytes()
        {
            fixed (byte* data = Data)
            {
                return new ReadOnlySpan<byte>(data, (int)Math.Min(DataSize, DataCapacity)).ToArray();
            }
        }
    }

    public static class Utils
    {
        /// <summary>Copies the message to unmanaged memory, the caller frees it with Marshal.FreeHGlobal.</summary>
        public static IntPtr ToIntPtr(this PassThruMsg msg)
        {
            IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf<PassThruMsg>());
            Marshal.StructureToPtr(msg, ptr, false);
            return ptr;
        }

        public static T AsStruct<T>(this IntPtr ptr) where T : struct
        {
            return Marshal.PtrToStructure<T>(ptr);
        }
    }
}
