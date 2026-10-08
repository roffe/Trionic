using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace TrionicCANLib
{
    /// <summary>
    /// Maps the Windows DLL names used in [DllImport] to their per-platform equivalents.
    /// .NET allows a single resolver per assembly, so every native mapping for this assembly lives here.
    /// </summary>
    internal static class NativeLibs
    {
        // ponytail: probe list, first one that loads wins
        static string[] Candidates(string name)
        {
            switch (name)
            {
                case "canlib32": // Kvaser CANlib
                    if (OperatingSystem.IsLinux()) return new[] { "libcanlib.so.1", "libcanlib.so" };
                    if (OperatingSystem.IsWindows()) return new[] { "canlib32.dll" };
                    return Array.Empty<string>();
                case "canusbdrv": // Lawicel CANUSB, Windows only
                    if (OperatingSystem.IsWindows()) return new[] { Environment.Is64BitProcess ? "canusbdrv64.dll" : "canusbdrv.dll" };
                    return Array.Empty<string>();
            }
            return null;
        }

        static IntPtr Resolve(string name, Assembly assembly, DllImportSearchPath? searchPath)
        {
            var candidates = Candidates(name);
            if (candidates == null) return IntPtr.Zero; // not ours, default probing
            foreach (var c in candidates)
            {
                if (NativeLibrary.TryLoad(c, assembly, searchPath, out IntPtr handle)) return handle;
            }
            return IntPtr.Zero; // → DllNotFoundException at the call site, callers already treat that as "driver not installed"
        }

#pragma warning disable CA2255 // library on purpose: the resolver must be in place before the first P/Invoke from any class
        [ModuleInitializer]
#pragma warning restore CA2255
        internal static void Init()
        {
            NativeLibrary.SetDllImportResolver(typeof(NativeLibs).Assembly, Resolve);
            // LibUsbDotNet 3 registers its own resolver (libusb-1.0.dll / .so.0 / .0.dylib) and probes
            // NATIVE_DLL_SEARCH_DIRECTORIES first; add the Homebrew prefixes macOS doesn't search by default
            if (OperatingSystem.IsMacOS())
                AppContext.SetData("NATIVE_DLL_SEARCH_DIRECTORIES", AppContext.GetData("NATIVE_DLL_SEARCH_DIRECTORIES") + ":/opt/homebrew/lib:/usr/local/lib");
        }
    }

    /// <summary>
    /// Linux ftdi_sio defaults to a 16 ms latency timer, which cripples the request/response
    /// adapters (OBDLink SX, CANUSB VCP, ...). On Windows users are told to set 2 ms in Device
    /// Manager; here ASYNC_LOW_LATENCY makes the driver drop it to 1 ms, no root needed.
    /// </summary>
    public static class SerialLowLatency
    {
        const int O_RDWR = 2, O_NOCTTY = 0x100, O_NONBLOCK = 0x800;
        const uint TIOCGSERIAL = 0x541E, TIOCSSERIAL = 0x541F;
        const int ASYNC_LOW_LATENCY = 1 << 13;
        const int FlagsOffset = 16; // struct serial_struct { int type, line; unsigned port; int irq; int flags; ... }

        [DllImport("libc", SetLastError = true)] static extern int open(string path, int flags);
        [DllImport("libc", SetLastError = true)] static extern int ioctl(int fd, nuint request, byte[] arg);
        [DllImport("libc")] static extern int close(int fd);

        /// <summary>
        /// Call BEFORE SerialPort.Open(): .NET opens ttys with TIOCEXCL, so a second open() of an open port gets EBUSY.
        /// ftdi_sio keeps the flag on the port after this close. Best effort, silently does nothing off Linux or for
        /// ports that don't support it.
        /// </summary>
        public static void TryEnable(string portName)
        {
            if (!OperatingSystem.IsLinux() || string.IsNullOrEmpty(portName)) return;
            try
            {
                int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd < 0) return;
                try
                {
                    var ss = new byte[128]; // sizeof(serial_struct) is 72 on 64-bit, leave headroom
                    if (ioctl(fd, TIOCGSERIAL, ss) != 0) return;
                    int flags = BitConverter.ToInt32(ss, FlagsOffset);
                    if ((flags & ASYNC_LOW_LATENCY) != 0) return;
                    BitConverter.GetBytes(flags | ASYNC_LOW_LATENCY).CopyTo(ss, FlagsOffset);
                    ioctl(fd, TIOCSSERIAL, ss);
                }
                finally { close(fd); }
            }
            catch (Exception) { }
        }
    }
}
