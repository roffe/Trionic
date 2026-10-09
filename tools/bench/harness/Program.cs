using System;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using NLog;
using NLog.Config;
using NLog.Targets;
using TrionicCANLib.API;

// Bench harness: drives TrionicCANLib the way frmMain does, one GUI-like session per operation
// (settings, open, 1 s wait, operation, Cleanup). See ../README.md for the bench rules.
static class Bench
{
    static readonly Stopwatch clock = Stopwatch.StartNew();
    static int lastProgress = -1;

    static void Log(string s) => Console.WriteLine($"{DateTime.Now:HH:mm:ss.fff} +{clock.Elapsed.TotalSeconds,8:F3} {s}");

    static int Main(string[] args)
    {
        var argv = args.ToList();
        int t = argv.IndexOf("--trace");
        if (t >= 0 && t + 1 < argv.Count)
        {
            var cfg = new LoggingConfiguration();
            var file = new FileTarget("trace") { FileName = argv[t + 1], Layout = "${date:format=HH\\:mm\\:ss.ffff} ${threadid} ${logger:shortName=true} ${message}" };
            cfg.AddRule(NLog.LogLevel.Trace, NLog.LogLevel.Fatal, file);
            LogManager.Configuration = cfg;
            argv.RemoveRange(t, 2);
        }
        if (argv.Count < 3 || (argv[0] != "t5" && argv[0] != "t7"))
        {
            Console.Error.WriteLine("usage: bench [--trace FILE] t5|t7 canusb-vcp|combi|combi-ondevice|j2534-canusb OP...\n" +
                "  OP: info  dtc  read=FILE  flash=FILE  sram=FILE  reset  wait=SECONDS\n" +
                "  flash needs BENCH_FLASH_MD5=<md5>[,<md5>...] listing the file's md5");
            return 2;
        }
        string ecu = argv[0], path = argv[1];
        // the library asks the user these; on the bench the answer is always No (and logged)
        UserPrompt.YesNo = (text, caption) => { Log("[prompt] " + caption + ": " + text.Replace('\n', ' ') + " -> No"); return false; };
        UserPrompt.Notify = (text, caption) => Log("[notify] " + caption + ": " + text.Replace('\n', ' '));

        bool ok = true;
        foreach (string op in argv.Skip(2))
        {
            Log("==== " + op);
            bool r;
            try { r = ecu == "t5" ? RunT5(path, op) : RunT7(path, op); }
            catch (Exception e) { Log("EXCEPTION " + e); r = false; }
            Log("==== " + op + (r ? " ok" : " FAILED"));
            ok &= r;
            if (!r) break;
        }
        LogManager.Shutdown();
        Log("exit " + (ok ? 0 : 1));
        return ok ? 0 : 1;
    }

    // ---------------- settings, like frmMain.SetGenericOptions with OnlyPBus on ----------------

    static void Setup(ITrionic trionic, ECU ecu, string path)
    {
        trionic.OnlyPBus = true;
        trionic.ECU = ecu;
        CANBusAdapter type;
        string name;
        switch (path)
        {
            case "canusb-vcp":
                type = CANBusAdapter.LAWICEL;
                name = Env("BENCH_CANUSB") ?? ITrionic.GetAdapterNames(type).FirstOrDefault();
                break;
            case "combi":
            case "combi-ondevice":
                type = CANBusAdapter.COMBI;
                name = null;
                break;
            case "j2534-canusb":
                type = CANBusAdapter.J2534;
                name = Env("BENCH_J2534") ?? "LAWICEL CANUSB";
                break;
            default:
                throw new ArgumentException("unknown path " + path);
        }
        trionic.setCANDevice(type);
        if (name != null) trionic.SetSelectedAdapter(name);
        Log($"adapter {path}: {type} {name ?? "(default)"}");
        trionic.onCanInfo += (s, e) => Log($"[info:{e.Type}] {e.Info}");
        trionic.onWriteProgress += (s, e) => Progress("write", e.Percentage);
        trionic.onReadProgress += (s, e) => Progress("read", e.Percentage);
    }

    static void Progress(string what, int pct)
    {
        int step = pct / 10 * 10;
        if (step == lastProgress) return;
        lastProgress = step;
        Log($"[progress] {what} {step}%");
    }

    static bool Open(Func<bool> open)
    {
        var sw = Stopwatch.StartNew();
        bool r = open();
        Log($"openDevice={r} ({sw.ElapsedMilliseconds} ms)");
        if (r) Thread.Sleep(1000); // the GUI waits a second after opening
        return r;
    }

    // ---------------- Trionic 5 ----------------

    static bool RunT5(string path, string op)
    {
        if (op.StartsWith("wait=")) { Thread.Sleep(int.Parse(op[5..]) * 1000); return true; }
        if (op.StartsWith("flash=") && !FlashAllowed(op[6..])) return false;
        var t5 = new Trionic5();
        Setup(t5, ECU.TRIONIC5, path);
        try
        {
            if (!Open(t5.openDevice)) return false;
            var sw = Stopwatch.StartNew();
            if (op == "info")
            {
                t5.GetECUInfo(true);
                return true;
            }
            if (op == "reset")
            {
                bool r = t5.ResetECU();
                Log($"ResetECU={r} ({sw.ElapsedMilliseconds} ms)");
                return r;
            }
            if (op.StartsWith("read="))
            {
                string file = op[5..];
                var work = new DoWorkEventArgs(file);
                t5.DumpECU(null, work); // the GUI runs this in a BackgroundWorker
                return Result("read", file, sw, work.Result is bool b && b);
            }
            if (op.StartsWith("sram="))
            {
                string file = op[5..];
                t5.GetSRAMSnapshot(file);
                return Result("sram", file, sw, File.Exists(file));
            }
            if (op.StartsWith("flash="))
            {
                string file = op[6..];
                WriteFlashResult r = t5.WriteFlash(file);
                Log($"RESULT op=flash result={r} seconds={sw.Elapsed.TotalSeconds:F1}");
                return r == WriteFlashResult.Done;
            }
            throw new ArgumentException("unknown T5 op " + op);
        }
        finally
        {
            t5.Cleanup();
            Log("Cleanup done");
        }
    }

    // ---------------- Trionic 7 ----------------

    static bool RunT7(string path, string op)
    {
        if (op.StartsWith("wait=")) { Thread.Sleep(int.Parse(op[5..]) * 1000); return true; }
        if (op.StartsWith("flash=") && !FlashAllowed(op[6..])) return false;
        var t7 = new Trionic7();
        bool flashOp = op.StartsWith("read=") || op.StartsWith("flash=");
        // like the GUI: the on-device flasher only for read/flash, info/DTC/SRAM/reset always over KWP
        t7.UseFlasherOnDevice = path == "combi-ondevice" && flashOp;
        Setup(t7, ECU.TRIONIC7, path);
        var done = new ManualResetEventSlim();
        string finish = null;
        t7.onCanInfo += (s, e) =>
        {
            if (e.Type == ActivityType.FinishedFlashing || e.Type == ActivityType.FinishedDownloadingFlash) { finish = e.Info; done.Set(); }
        };
        try
        {
            if (!Open(t7.openDevice)) return false;
            var sw = Stopwatch.StartNew();
            if (op == "info") { t7.GetECUInfo(); return true; }
            if (op == "dtc")
            {
                string[] dtcs = t7.ReadDTC();
                Log($"ReadDTC returned {dtcs?.Length ?? 0} line(s) ({sw.ElapsedMilliseconds} ms)");
                foreach (string d in dtcs ?? Array.Empty<string>()) Log("  DTC: " + d);
                return dtcs != null;
            }
            if (op == "reset")
            {
                bool r = t7.ResetECU();
                Log($"ResetECU={r} ({sw.ElapsedMilliseconds} ms)");
                return r;
            }
            if (op.StartsWith("sram="))
            {
                string file = op[5..];
                return Result("sram", file, sw, t7.GetSRAMSnapshot(file));
            }
            if (op.StartsWith("read="))
            {
                string file = op[5..];
                t7.ReadFlash(file);
                bool finished = done.Wait(TimeSpan.FromMinutes(15));
                Log("READ FINISHED: " + (finished ? "'" + finish + "'" : "timed out after 15 min"));
                return Result("read", file, sw, finished && finish != null && finish.StartsWith("Finished download"));
            }
            if (op.StartsWith("flash="))
            {
                string file = op[6..];
                t7.WriteFlash(file);
                bool finished = done.Wait(TimeSpan.FromMinutes(15));
                Log($"RESULT op=flash finished={finished} info='{finish}' seconds={sw.Elapsed.TotalSeconds:F1}");
                return finished && finish != null && finish.StartsWith("Finished FLASH");
            }
            throw new ArgumentException("unknown T7 op " + op);
        }
        finally
        {
            t7.Cleanup();
            Log("Cleanup done");
        }
    }

    // ---------------- helpers ----------------

    static bool Result(string op, string file, Stopwatch sw, bool ok)
    {
        string md5 = File.Exists(file) ? Md5(file) : "-";
        long size = File.Exists(file) ? new FileInfo(file).Length : 0;
        Log($"RESULT op={op} ok={ok} file={file} size={size} md5={md5} seconds={sw.Elapsed.TotalSeconds:F1}");
        return ok;
    }

    // hard rule of the bench: only images listed in BENCH_FLASH_MD5 may be written
    static bool FlashAllowed(string file)
    {
        string md5 = Md5(file);
        string[] allowed = (Env("BENCH_FLASH_MD5") ?? "").Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        if (allowed.Contains(md5, StringComparer.OrdinalIgnoreCase)) return true;
        Log($"REFUSED: {file} md5 {md5} is not in BENCH_FLASH_MD5");
        return false;
    }

    static string Md5(string file) => Convert.ToHexString(MD5.HashData(File.ReadAllBytes(file))).ToLowerInvariant();

    static string Env(string name) { string v = Environment.GetEnvironmentVariable(name); return string.IsNullOrEmpty(v) ? null : v; }
}
