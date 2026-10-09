#!/bin/sh
# run.sh NAME t5|t7 PATH OP... : build the harness, run it under the bench lock,
# log to out/logs/NAME.log (NLog trace in NAME.nlog), print the summary lines.
# Example: tools/bench/run.sh t5-vcp t5 canusb-vcp info read=tools/bench/out/t5-read.bin
set -eu
here=$(cd "$(dirname "$0")" && pwd)
out=$here/out
mkdir -p "$out/logs"
[ $# -ge 4 ] || { echo "usage: run.sh NAME t5|t7 PATH OP..."; exit 2; }
name=$1
shift
dotnet build "$here/harness/bench.csproj" --artifacts-path "$out/build" -nologo -v q > "$out/logs/$name.build" 2>&1 \
	|| { cat "$out/logs/$name.build"; exit 2; }
echo "$(date +%H:%M) [$name] start: $*" >> "$out/progress.log"
set +e
flock "$out/bench.lock" dotnet "$out/build/bin/bench/debug/bench.dll" --trace "$out/logs/$name.nlog" "$@" > "$out/logs/$name.log" 2>&1
rc=$?
set -e
grep -E '====|RESULT|REFUSED|EXCEPTION|ResetECU=|openDevice=|ReadDTC|DTC:|exit ' "$out/logs/$name.log" || true
echo "$(date +%H:%M) [$name] done rc=$rc" >> "$out/progress.log"
exit $rc
