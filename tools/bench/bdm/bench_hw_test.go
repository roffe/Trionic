package bdm

// Bench BDM access through BDMTool's own code (github.com/roffe/bdmtool/bdm), independent of TrionicCANLib.
// Not part of BDMTool: tools/bench/bdm.sh injects it with `go test -count=1 -overlay ...` so nothing is
// written to the BDMTool checkout. The ECU comes from Identify() unless BENCH_ECU names a table entry
// (e.g. "Trionic 7"). Every mode ends with an MCU reset, so the ECU runs its own code again.
//
//   BENCH_MODE=identify
//   BENCH_MODE=read  BENCH_FILE=out.bin
//   BENCH_MODE=write BENCH_FILE=in.bin      erase, program, read back and compare
//   BENCH_MODE=hold  BENCH_HOLD=30          halt the CPU in BDM for N seconds, then reset

import (
	"bytes"
	"crypto/md5"
	"os"
	"strconv"
	"testing"
	"time"
)

func TestBenchBDM(t *testing.T) {
	mode, path := os.Getenv("BENCH_MODE"), os.Getenv("BENCH_FILE")
	if mode == "" {
		t.Skip("run through tools/bench/bdm.sh")
	}
	a, err := openCombi()
	if err != nil {
		t.Fatal(err)
	}
	defer a.Close()
	maj, min, _ := a.Version()
	t.Logf("CombiAdapter firmware %d.%d", maj, min)

	var e *ECU
	if name := os.Getenv("BENCH_ECU"); name != "" {
		if e = ecuByName(name); e == nil {
			t.Fatalf("unknown BENCH_ECU %q", name)
		}
		t.Logf("ECU forced: %s", e.Name)
	} else {
		id, ok := a.(Identifier)
		if !ok {
			t.Fatal("adapter cannot identify the ECU, set BENCH_ECU")
		}
		var desc string
		if e, desc, err = id.Identify(); err != nil {
			t.Fatalf("identify: %v", err)
		}
		t.Logf("identified: %s (%s)", e.Name, desc)
	}

	last := uint32(0)
	prog := func(done uint32) {
		if done-last >= e.FlashSize/8 || done == e.FlashSize {
			t.Logf("%d / %d", done, e.FlashSize)
			last = done
		}
	}
	read := func() []byte {
		var buf bytes.Buffer
		if err := a.ReadFlash(e, &buf, prog); err != nil {
			t.Fatalf("ReadFlash after %d bytes: %v", buf.Len(), err)
		}
		return buf.Bytes()
	}

	switch mode {
	case "identify":
	case "read":
		d := read()
		if err := os.WriteFile(path, d, 0o644); err != nil {
			t.Fatal(err)
		}
		t.Logf("read %d bytes md5 %x -> %s", len(d), md5.Sum(d), path)
	case "write":
		bin, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		if err := a.WriteFlash(e, bin, true, prog); err != nil {
			t.Fatalf("WriteFlash: %v", err)
		}
		last = 0
		if d := read(); !bytes.Equal(d, bin) {
			t.Fatalf("verify failed: read back md5 %x, file md5 %x", md5.Sum(d), md5.Sum(bin))
		}
		t.Logf("wrote and verified %s md5 %x", path, md5.Sum(bin))
	case "hold":
		secs, err := strconv.Atoi(os.Getenv("BENCH_HOLD"))
		if err != nil || secs <= 0 {
			t.Fatal("BENCH_HOLD must be a number of seconds")
		}
		// Identify (or the forced ECU's first access) left the CPU halted in BDM
		if os.Getenv("BENCH_ECU") != "" {
			if err := a.Stop(); err != nil {
				t.Fatalf("stop: %v", err)
			}
		}
		t.Logf("HOLDING the CPU in BDM for %d s", secs)
		time.Sleep(time.Duration(secs) * time.Second)
		t.Logf("RELEASED")
	default:
		t.Fatalf("unknown BENCH_MODE %q", mode)
	}
	if err := a.Reset(); err != nil {
		t.Logf("reset: %v", err)
	}
}
