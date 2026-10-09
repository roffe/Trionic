#!/bin/sh
# bdm.sh identify | read FILE | write FILE | hold SECONDS
# BDM through the CombiAdapter with BDMTool's own Go code (bdm/bench_hw_test.go, injected with
# go test -overlay), under the bench lock. Always -count=1: a cached go test result reports PASS
# without touching the ECU. write only accepts files listed in BENCH_FLASH_MD5.
# Optional: BDMTOOL (default ~/go/src/github.com/roffe/bdmtool), BENCH_ECU (e.g. "Trionic 7").
set -eu
here=$(cd "$(dirname "$0")" && pwd)
out=$here/out
mkdir -p "$out/logs"
BDMTOOL=${BDMTOOL:-$HOME/go/src/github.com/roffe/bdmtool}
[ -d "$BDMTOOL/bdm" ] || { echo "no BDMTool checkout at $BDMTOOL (set BDMTOOL)"; exit 2; }

mode=${1:-}
arg=${2:-}
abspath() { echo "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"; }
case $mode in
identify) ;;
read)
	[ -n "$arg" ] || { echo "usage: bdm.sh read FILE"; exit 2; }
	arg=$(abspath "$arg") ;;
write)
	[ -f "$arg" ] || { echo "usage: bdm.sh write FILE"; exit 2; }
	arg=$(abspath "$arg")
	md5=$(md5sum < "$arg" | cut -d' ' -f1)
	case ",${BENCH_FLASH_MD5:-}," in
	*",$md5,"*) ;;
	*) echo "REFUSED: $arg md5 $md5 is not in BENCH_FLASH_MD5"; exit 1 ;;
	esac ;;
hold)
	[ -n "$arg" ] || { echo "usage: bdm.sh hold SECONDS"; exit 2; } ;;
*)
	echo "usage: bdm.sh identify | read FILE | write FILE | hold SECONDS"; exit 2 ;;
esac

printf '{"Replace":{"%s/bdm/zz_bench_hw_test.go":"%s/bdm/bench_hw_test.go"}}\n' "$BDMTOOL" "$here" > "$out/bdm-overlay.json"
log=$out/logs/bdm-$mode-$(date +%H%M%S).log
echo "$(date +%H:%M) [bdm] $mode $arg" >> "$out/progress.log"
set +e
(cd "$BDMTOOL" && BENCH_MODE=$mode BENCH_FILE=$arg BENCH_HOLD=$arg \
	flock "$out/bench.lock" go test -count=1 -timeout 30m -overlay "$out/bdm-overlay.json" -run '^TestBenchBDM$' -v ./bdm) > "$log" 2>&1
rc=$?
set -e
grep -E 'firmware|identified|forced|read [0-9]+ bytes|wrote and verified|verify failed|HOLDING|RELEASED|FAIL|PASS|ok |Error|error' "$log" || true
echo "$(date +%H:%M) [bdm] $mode done rc=$rc ($log)" >> "$out/progress.log"
exit $rc
