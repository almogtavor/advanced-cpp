#!/usr/bin/env bash
#
# run.sh - portable end-to-end validator for any ex2 submission.
#
# It enforces ONLY behaviour that assignment2.md actually requires, through the
# standardised surface: the two CLI binaries (drone_mapper_simulation,
# maps_comparison), the mandated output names (simulation_output.yaml,
# output_results/) and the .npy output-map format. It does NOT assert
# unspecified properties (symmetry, specific in-between comparison values,
# monotonicity, determinism, exact output-map shape, exit codes on bad input) -
# the spec leaves those to the student. It includes no project header, so it
# runs against anyone's compiled binaries.
#
# Usage:  ./run.sh [<ex2_project_or_bin_dir>]     (default ../ex2)
#   Set VCPKG_ROOT first if the binaries must be built. Only python3 (stdlib)
#   is required at runtime.

set -uo pipefail

SUITE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-$SUITE_DIR/../ex2}"
FIX="$SUITE_DIR/fixtures"
NPY="$SUITE_DIR/tools/check_npy.py"

PASS=0; FAIL=0
green() { printf '\033[32m%s\033[0m' "$1"; }
red()   { printf '\033[31m%s\033[0m' "$1"; }
ok()  { PASS=$((PASS+1)); printf '  [%s] %s\n' "$(green PASS)" "$1"; }
bad() { FAIL=$((FAIL+1)); printf '  [%s] %s\n' "$(red FAIL)" "$1"; }
check() { if [[ "$2" -eq 0 ]]; then ok "$1"; else bad "$1"; fi; }
num_ok() { awk -v v="$1" "BEGIN{exit !($2)}" 2>/dev/null; }

find_bins() {
    local d
    for d in "$TARGET" "$TARGET/build"; do
        [[ -x "$d/drone_mapper_simulation" && -x "$d/maps_comparison" ]] && { BIN_DIR="$d"; return 0; }
    done
    if [[ -f "$TARGET/CMakeLists.txt" ]]; then
        [[ -z "${VCPKG_ROOT:-}" ]] && { echo "error: binaries missing and VCPKG_ROOT unset." >&2; exit 1; }
        echo ">> building binaries in $TARGET/build"
        cmake -S "$TARGET" -B "$TARGET/build" \
              -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
              -DCMAKE_BUILD_TYPE=Release >/dev/null || { echo "configure failed" >&2; exit 1; }
        cmake --build "$TARGET/build" --target drone_mapper_simulation maps_comparison \
              -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)" >/dev/null \
              || { echo "build failed" >&2; exit 1; }
        BIN_DIR="$TARGET/build"; return 0
    fi
    echo "error: could not find or build the binaries under '$TARGET'." >&2; exit 1
}
find_bins
BIN_DIR="$(cd "$BIN_DIR" && pwd)"
SIM="$BIN_DIR/drone_mapper_simulation"
CMP="$BIN_DIR/maps_comparison"
echo ">> using binaries in: $BIN_DIR"
ROOT_OUT="$(mktemp -d "${TMPDIR:-/tmp}/ex2_e2e.XXXXXX")"
trap 'rm -rf "$ROOT_OUT"' EXIT

cmp_val()  { "$CMP" "$@" 2>/dev/null | head -1 | awk '{print $1}'; }
npy_stat() { python3 "$NPY" "$1" 2>/dev/null | sed 's/^ok //'; }   # d0 d1 d2 vox inv mapped
run_scn()  { local out="$ROOT_OUT/$1${2:-}"; rm -rf "$out"; mkdir -p "$out"
             "$SIM" "$FIX/scenarios/$1/composition.yaml" "$out" >/dev/null 2>&1; local rc=$?; echo "$out"; return $rc; }
map_count() { find "$1/output_results" -name '*.npy' 2>/dev/null | wc -l | tr -d ' '; }
# spec leaves the error-log filename to the student: accept any non-empty,
# non-.npy file under output_results as "an error log was written".
has_error_log() { find "$1/output_results" -type f ! -name '*.npy' -size +0c 2>/dev/null | grep -q .; }

# per-map checks: valid 3-D .npy, only legal voxel codes, and (the core
# deliverable) that the drone actually mapped something.
check_maps() { # <outdir> <label> <require_mapped 0|1>
    local f i=0 st inv mp
    while read -r f; do
        st=$(npy_stat "$f")
        [[ -n "$st" ]]; check "$2 map $i is a valid 3-D .npy" "$?"
        inv=$(awk '{print $5}' <<<"$st"); mp=$(awk '{print $6}' <<<"$st")
        [[ "${inv:-1}" -eq 0 ]]; check "$2 map $i uses only legal voxel codes" "$?"
        [[ "$3" -eq 1 ]] && { [[ "${mp:-0}" -gt 0 ]]; check "$2 map $i has mapped voxels" "$?"; }
        i=$((i+1))
    done < <(find "$1/output_results" -name '*.npy' | sort)
}

# ===========================================================================
echo ""; echo "== maps_comparison (spec: identical=100, distinct~0, similar<100, error=-1) =="
# identical maps -> 100
for m in all_occupied all_empty half_occupied quarter_occupied base base_1diff base_2diff base_3diff base_4diff; do
    s=$(cmp_val "$FIX/cmp/$m.npy" "$FIX/cmp/$m.npy")
    num_ok "${s:-x}" 'v>=99.999'; check "identical: $m vs itself -> 100 (got ${s:-none})" "$?"
done
# very distinct maps -> close to 0 (both orderings are still "very distinct")
d1=$(cmp_val "$FIX/cmp/all_occupied.npy" "$FIX/cmp/all_empty.npy")
num_ok "${d1:-x}" 'v<20'; check "distinct occupied/empty -> near 0 (got ${d1:-none})" "$?"
d2=$(cmp_val "$FIX/cmp/all_empty.npy" "$FIX/cmp/all_occupied.npy")
num_ok "${d2:-x}" 'v<20'; check "distinct empty/occupied -> near 0 (got ${d2:-none})" "$?"
# very similar maps -> close to 100 but not 100
sims=""
for n in 1 2 3 4; do
    s=$(cmp_val "$FIX/cmp/base.npy" "$FIX/cmp/base_${n}diff.npy"); sims="$sims $s"
    num_ok "${s:-x}" 'v>50 && v<100'; check "similar: base vs base_${n}diff -> high but <100 (got ${s:-none})" "$?"
done
# combining the two endpoint guarantees: a very-similar pair scores above a
# very-distinct pair (follows directly from ~100 vs ~0)
far=$(cmp_val "$FIX/cmp/base.npy" "$FIX/cmp/all_empty.npy")
one=$(cmp_val "$FIX/cmp/base.npy" "$FIX/cmp/base_1diff.npy")
num_ok "${one:-0}" "v>${far:-101}"; check "a very-similar pair scores above a very-distinct pair" "$?"
# spec: the score is a float between 0 and 100 (or -1 on error)
b=0; for r in $d1 $d2 $sims $far; do num_ok "${r:-x}" '(v>=0 && v<=100)' || b=1; done
[[ "$b" -eq 0 ]]; check "all comparison scores are within [0, 100]" "$?"
# spec: on error print -1 to stdout
for c in "missing_a missing_b" "base missing_b" "missing_a base"; do
    set -- $c; s=$(cmp_val "$FIX/cmp/$1.npy" "$FIX/cmp/$2.npy")
    num_ok "${s:-x}" 'v==-1'; check "error ($1 / $2 unreadable) -> -1 (got ${s:-none})" "$?"
done
s=$("$CMP" "$FIX/cmp/base.npy" 2>/dev/null | head -1 | awk '{print $1}')
num_ok "${s:-x}" 'v==-1'; check "error (too few arguments) -> -1 (got ${s:-none})" "$?"
s=$("$CMP" 2>/dev/null | head -1 | awk '{print $1}')
num_ok "${s:-x}" 'v==-1'; check "error (no arguments) -> -1 (got ${s:-none})" "$?"

# ===========================================================================
echo ""; echo "== drone_mapper_simulation: basic =="
OUT=$(run_scn basic); rc=$?
check "a valid composition runs to completion" "$rc"
[[ -f "$OUT/simulation_output.yaml" ]]; check "writes simulation_output.yaml" "$?"
[[ -d "$OUT/output_results" ]]; check "creates output_results/" "$?"
[[ "$(map_count "$OUT")" -eq 1 ]]; check "produces exactly 1 output map" "$?"
check_maps "$OUT" "basic" 1

echo ""; echo "== cartesian product (spec: runs = missions x drones x lidars) =="
OUT=$(run_scn product); rc=$?
check "product composition runs" "$rc"
[[ "$(map_count "$OUT")" -eq 4 ]]; check "2 drones x 2 lidars -> 4 output maps (got $(map_count "$OUT"))" "$?"
check_maps "$OUT" "product" 1

echo ""; echo "== multi-mission =="
OUT=$(run_scn multimission); rc=$?
check "multi-mission composition runs" "$rc"
[[ "$(map_count "$OUT")" -eq 2 ]]; check "2 missions -> 2 output maps (got $(map_count "$OUT"))" "$?"
check_maps "$OUT" "multimission" 1

echo ""; echo "== resolution request (spec: output may be a different resolution) =="
OUT=$(run_scn resolution); rc=$?
check "resolution composition runs" "$rc"
[[ "$(map_count "$OUT")" -eq 2 ]]; check "factor1 + factor2 -> 2 output maps" "$?"
check_maps "$OUT" "resolution" 0
mapfiles=( $(find "$OUT/output_results" -name '*.npy'|sort) )
if [[ "${#mapfiles[@]}" -eq 2 ]]; then
    v0=$(npy_stat "${mapfiles[0]}"|awk '{print $4}'); v1=$(npy_stat "${mapfiles[1]}"|awk '{print $4}')
    hi=$v0; lo=$v1; [[ "$v1" -gt "$v0" ]] && { hi=$v1; lo=$v0; }
    num_ok "$hi" "v > $lo"; check "coarser (factor-2) map differs in size from factor-1 ($lo vs $hi)" "$?"
else bad "resolution factor changes output map voxel count"; fi

echo ""; echo "== error handling (spec: log immediately, score -1, continue) =="
OUT=$(run_scn error); rc=$?
check "bad map file: simulation continues to completion" "$rc"
[[ -f "$OUT/simulation_output.yaml" ]]; check "bad map file: report still written" "$?"
has_error_log "$OUT"; check "bad map file: an error log was written under output_results" "$?"
OUT=$(run_scn mixed); rc=$?
check "mixed good+bad: continues to completion" "$rc"
[[ "$(map_count "$OUT")" -ge 1 ]]; check "mixed: the good scenario produced a map" "$?"
has_error_log "$OUT"; check "mixed: the bad scenario was logged" "$?"
check_maps "$OUT" "mixed-good" 1

echo ""; echo "== max_steps & invalid bounds (spec: continue, don't crash) =="
OUT=$(run_scn maxsteps); rc=$?
check "tiny max_steps: runs to completion" "$rc"
[[ "$(map_count "$OUT")" -ge 1 ]]; check "tiny max_steps: still produced an output map" "$?"
check_maps "$OUT" "maxsteps" 0
OUT=$(run_scn badbounds); rc=$?
check "invalid mission bounds: does not crash the simulation" "$rc"
[[ -f "$OUT/simulation_output.yaml" ]]; check "invalid bounds: report still written" "$?"

echo ""; echo "== CLI argument handling (spec: default simulation.yaml, path resolution) =="
WD="$ROOT_OUT/cwd"; mkdir -p "$WD"
cp "$FIX/scenarios/basic"/*.yaml "$FIX/scenarios/basic"/*.npy "$WD"/ 2>/dev/null
mv "$WD/composition.yaml" "$WD/simulation.yaml"
( cd "$WD" && "$SIM" >/dev/null 2>&1 ); check "no args -> uses ./simulation.yaml" "$?"
[[ -f "$WD/simulation_output.yaml" ]]; check "no args -> writes report into the current directory" "$?"
[[ "$(map_count "$WD")" -ge 1 ]]; check "no args -> produces an output map" "$?"
RD="$ROOT_OUT/rel"; mkdir -p "$RD/out"; cp -R "$FIX/scenarios/basic/." "$RD/scn"
( cd "$RD" && "$SIM" scn/composition.yaml out >/dev/null 2>&1 ); check "relative composition path resolves" "$?"
[[ -f "$RD/out/simulation_output.yaml" ]]; check "relative path -> report written in the given output dir" "$?"

# ===========================================================================
echo ""; echo "==================================================="
printf "  %s passed, %s failed  (%s total)\n" \
    "$(green "$PASS")" "$([[ $FAIL -gt 0 ]] && red "$FAIL" || echo "$FAIL")" "$((PASS+FAIL))"
echo "==================================================="
[[ "$FAIL" -eq 0 ]]
