# Bench tooling

Hardware-in-the-loop testing of TrionicCANLib against a real ECU on the bench. Nothing here is
part of the shipped app. Everything the tools produce (build output, logs, dumps, the lock file)
goes to `tools/bench/out/`, which git ignores: **never commit ECU dumps.**

| | |
|---|---|
| `run.sh NAME t5\|t7 PATH OP...` | builds and runs the harness under the bench lock; log in `out/logs/NAME.log`, NLog trace in `NAME.nlog` |
| `harness/` | console app that drives the library like the GUI does: settings, open, 1 s wait, operation, Cleanup — one session per operation |
| `bdm.sh identify\|read FILE\|write FILE\|hold SECONDS` | BDM through the CombiAdapter using BDMTool's own Go code (`bdm/bench_hw_test.go`, injected with `go test -overlay`, nothing is written to the BDMTool checkout) |
| `out/progress.log` | one line per start/finish of every run, handy to `tail -f` |

PATH is the adapter: `canusb-vcp` (Lawicel CANUSB, the in-tree driver), `combi` (CombiAdapter over
KWP/CAN), `combi-ondevice` (T7 read/flash with the Combi's own flasher), `j2534-canusb` (J2534 from
`~/.passthru`, device name in `BENCH_J2534`, default `LAWICEL CANUSB`). `BENCH_CANUSB` picks the
CANUSB serial if several are attached.

OP: `info`, `dtc` (T7), `read=FILE`, `flash=FILE`, `sram=FILE`, `reset`, `wait=SECONDS`. The run
stops at the first failed operation; exit code 0 = all ok, 1 = an operation failed, 2 = usage.

Requirements: the .NET 10 SDK; for BDM also Go and a BDMTool checkout (`BDMTOOL`, default
`~/go/src/github.com/roffe/bdmtool`) and the CombiAdapter wired to the ECU's BDM header.

## Examples

```sh
export BENCH_FLASH_MD5=2e0ce9fea680e79a10ff915ac8ca6b09   # the reference image(s) that may be written

tools/bench/bdm.sh read tools/bench/out/baseline.bin       # what's on the ECU now (BDM, independent of our code)
tools/bench/run.sh t5-vcp t5 canusb-vcp info read=tools/bench/out/t5-vcp.bin sram=tools/bench/out/t5-vcp.RAM
tools/bench/run.sh t5-flash t5 combi flash=tools/bench/out/reference.bin read=tools/bench/out/verify.bin
tools/bench/run.sh t7-info t7 j2534-canusb info info dtc
nohup tools/bench/run.sh t7-read t7 combi-ondevice read=tools/bench/out/t7.bin > /dev/null 2>&1 &   # long runs: background, tail the log
tools/bench/bdm.sh write tools/bench/out/reference.bin    # restore: erase, program, read back, compare
```

## Rules (hard)

1. **One thing talks to the ECU at a time.** `run.sh` and `bdm.sh` take `out/bench.lock` (flock);
   anything else that touches an adapter must too. The GUI does **not** use the lock — close it, and
   check `fuser -v /dev/ttyUSB*` before a run.
2. **Only a known reference image is ever written** (CAN flash or BDM write). Both tools refuse any
   file whose md5 isn't listed in `BENCH_FLASH_MD5`. Take a BDM backup before the first write.
3. **Never leave the ECU erased or half written.** If a flash fails and the ECU doesn't come back:
   retry once over another CAN path, otherwise restore with `bdm.sh write` and confirm with
   `bdm.sh read` (md5). Prove `bdm.sh write` on the healthy ECU before any test that may break the FLASH.
4. **BDM always runs with `-count=1`** (`bdm.sh` does it). A cached `go test` result reports PASS
   without touching the ECU — a "restore" that never happened.
5. **Never auto-reset a T7 in a car**: with the ignition on, a reset puts the electronic throttle
   body in limp mode (mechanical reset needed). `reset` is for the bench only.
6. A T7 accepts one diagnostic session at a time and drops an idle one after ~6 s; a session left
   by a killed process is cleared by the library's stop-and-retry on the next open (~1.4 s).
7. A BDM read halts the CPU and ends with an MCU reset, which also gets a T7 out of its post-flash
   state.
