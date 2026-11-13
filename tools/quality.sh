#!/usr/bin/env bash
# R1.9 code-quality gates for the core (src/gdn_fusion). Each subcommand exits non-zero
# when its gate fails; CI runs them all (.github/workflows/ci.yml).
#
#   tools/quality.sh werror   [FILE...]  ROS-free sources build with -Werror under g++ and clang++
#   tools/quality.sh sanitize [FILE...]  ASan + UBSan builds: selftest and short MC runs (or FILEs)
#   tools/quality.sh cppcheck [FILE...]  cppcheck: warning, performance, portability
#   tools/quality.sh tidy     [FILE...]  clang-tidy with .clang-tidy (bug-finding + performance)
#   tools/quality.sh format   [BASE]     lines changed since BASE (default origin/main) follow .clang-format
#   tools/quality.sh selftest            every gate must fail on a planted defect, and pass clean code
#   tools/quality.sh all                 werror, cppcheck, tidy, sanitize, format
#
# tidy on the ROS nodes needs build/gdn_fusion/compile_commands.json, from
#   colcon build --packages-select gdn_fusion --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# Formatting is checked on changed lines only: the existing code is hand-aligned, and a
# one-off reformat would touch about 40% of it (decided in R1.9).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$ROOT/src/gdn_fusion"
EIGEN_INC="${EIGEN_INC:-$(pkg-config --cflags-only-I eigen3 2>/dev/null | sed 's/^-I//;s/ *$//')}"
EIGEN_INC="${EIGEN_INC:-/usr/include/eigen3}"
OUT="${TMPDIR:-/tmp}/gdn_quality"; mkdir -p "$OUT"
CORE=("$PKG/src/selftest.cpp" "$PKG/src/mc_campaign.cpp" "$PKG/src/imu_sim.cpp")    # ROS-free
NODES=("$PKG/src/eskf_node.cpp" "$PKG/src/map_viz_node.cpp" "$PKG/src/gdn_replay.cpp")   # need ROS
WARN=(-Wall -Wextra -Wpedantic -Werror)    # the same set as CMakeLists.txt with GDN_WERROR=ON
INC=(-std=c++17 -I "$PKG/include" -isystem "$EIGEN_INC")
export GDN_ROOT="${GDN_ROOT:-$ROOT}"

say() { printf '%s\n' "$*"; }

werror() {   # FILE...: compile each with -Werror under both compilers
    local files=("$@"); [ ${#files[@]} -gt 0 ] || files=("${CORE[@]}")
    local rc=0 cxx extra f
    for cxx in g++ clang++; do
        command -v "$cxx" >/dev/null || { say "werror: $cxx not installed, skipped"; continue; }
        extra=(); [ "$cxx" = g++ ] && extra=(-Wno-maybe-uninitialized)   # Eigen false positives (CMakeLists.txt)
        for f in "${files[@]}"; do
            if "$cxx" "${INC[@]}" -O2 "${WARN[@]}" "${extra[@]}" -c "$f" -o "$OUT/werror.o" 2>"$OUT/werror.log"; then
                say "werror: $cxx $(basename "$f") clean"
            else
                cat "$OUT/werror.log"; say "werror: $cxx $(basename "$f") FAILED"; rc=1
            fi
        done
    done
    return $rc
}

san_build() {   # SRC BIN
    g++ "${INC[@]}" -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=all \
        "$1" -o "$2"
}

sanitize() {   # FILE...: build and run each under ASan+UBSan; default: selftest + short MC runs
    # Leak checking is off: LeakSanitizer needs ptrace, which CI containers don't allow,
    # and a leak in a short-lived test tool isn't what this gate is for.
    export ASAN_OPTIONS="detect_leaks=0:abort_on_error=0" UBSAN_OPTIONS="print_stacktrace=1"
    local rc=0 f
    if [ $# -gt 0 ]; then
        for f in "$@"; do
            san_build "$f" "$OUT/san_file" && "$OUT/san_file" >"$OUT/san_file.log" 2>&1 \
                || { tail -20 "$OUT/san_file.log" 2>/dev/null; say "sanitize: $(basename "$f") FAILED"; rc=1; }
        done
        return $rc
    fi
    san_build "$PKG/src/selftest.cpp" "$OUT/selftest_san"
    san_build "$PKG/src/mc_campaign.cpp" "$OUT/mc_san"
    if "$OUT/selftest_san" >"$OUT/selftest_san.log" 2>&1; then say "sanitize: selftest clean"
    else grep -E "FAIL|runtime error|AddressSanitizer|SUMMARY" "$OUT/selftest_san.log" | head -20
         say "sanitize: selftest FAILED"; rc=1; fi
    # Short MC runs through each code path: the node start-up, GNSS loss with a model IMU,
    # a start error, and the recorded IMU (flight001, if fetched).
    local runs=(
        "2 data/flight_zermatt_csv data/maps/zermatt.bin --model=agl --startup=node"
        "1 data/flight_zermatt_long_csv data/maps/zermatt.bin --model=agl --startup=node --gnss-loss=150 --imu=config/imu/oakd_lite.yaml"
        "1 data/flight_zermatt_csv data/maps/zermatt.bin --model=agl --startup=node --start-error=150"
        "1 data/flight001_csv data/maps/terrain_db_sigma4.bin --model=agl --baro-drift=5 --gnss-loss=150"
    )
    local r
    for r in "${runs[@]}"; do
        local csv; csv=$(cut -d' ' -f2 <<<"$r")
        [ -d "$ROOT/$csv" ] || { say "sanitize: MC $csv: no data, skipped (tools/make_data.sh, fetch_data.sh)"; continue; }
        # shellcheck disable=SC2086
        if (cd "$ROOT" && "$OUT/mc_san" $r >"$OUT/mc_san.log" 2>&1); then say "sanitize: MC $r clean"
        else grep -E "runtime error|AddressSanitizer|SUMMARY" "$OUT/mc_san.log" | head -20
             say "sanitize: MC $r FAILED"; rc=1; fi
    done
    return $rc
}

cppcheck_gate() {   # FILE...
    local files=("$@"); [ ${#files[@]} -gt 0 ] || files=("$PKG/src" "$PKG/include")
    cppcheck --std=c++17 --language=c++ --enable=warning,performance,portability --inline-suppr \
        --suppress=missingIncludeSystem --error-exitcode=1 --quiet -I "$PKG/include" \
        --template='{file}:{line}: {severity} ({id}): {message}' "${files[@]}" \
        && say "cppcheck: clean" || { say "cppcheck: FAILED"; return 1; }
}

tidy() {   # FILE...: ROS-free files with explicit flags; the ROS nodes from compile_commands.json
    local files=("$@") rc=0 f
    [ ${#files[@]} -gt 0 ] || files=("${CORE[@]}" "${NODES[@]}")
    local db="$ROOT/build/gdn_fusion"
    for f in "${files[@]}"; do
        f=$(realpath "$f")
        local node=0 n; for n in "${NODES[@]}"; do [ "$f" = "$n" ] && node=1; done
        if [ $node = 1 ]; then
            [ -f "$db/compile_commands.json" ] || { say "tidy: $db/compile_commands.json missing (see the header)"; return 1; }
            clang-tidy --quiet -p "$db" --extra-arg=-Wno-unknown-warning-option "$f" >"$OUT/tidy.log" 2>&1 || { cat "$OUT/tidy.log"; rc=1; say "tidy: $(basename "$f") FAILED"; continue; }
        else
            clang-tidy --quiet --config-file="$ROOT/.clang-tidy" "$f" -- "${INC[@]}" >"$OUT/tidy.log" 2>&1 \
                || { cat "$OUT/tidy.log"; rc=1; say "tidy: $(basename "$f") FAILED"; continue; }
        fi
        say "tidy: $(basename "$f") clean"
    done
    return $rc
}

format() {   # BASE: lines changed since BASE must already be formatted
    local base="${1:-origin/main}"
    local d; d=$(git clang-format --diff --extensions cpp,hpp,h,cc "$base" -- 2>&1) || true
    if [ -z "$d" ] || grep -qE "^(no modified files to format|clang-format did not modify any files)" <<<"$d"; then
        say "format: lines changed since $base are formatted"; return 0
    fi
    printf '%s\n' "$d"
    say "format: FAILED. Fix the lines above with  git add -u && git clang-format --extensions cpp,hpp,h,cc $base  (only changed C++ lines are checked)."
    return 1
}

selftest() {   # each gate must fail on a planted defect and pass clean code
    local T; T=$(mktemp -d); local ok=0
    expect() {   # WANT(pass|fail) NAME CMD...
        local want=$1 name=$2; shift 2
        if "$@" >"$T/out.log" 2>&1; then got=pass; else got=fail; fi
        if [ "$got" = "$want" ]; then say "PASS  quality: $name ($got, as it should)"
        else say "FAIL  quality: $name: expected $want, got $got"; sed 's/^/      /' "$T/out.log" | tail -8; ok=1; fi
    }
    printf '#include <cstdio>\nint main() { std::printf("ok\\n"); return 0; }\n' >"$T/clean.cpp"
    printf 'int main() { int unused = 0; return 0; }\n' >"$T/unused.cpp"
    printf '#include <vector>\nint main() { std::vector<int> v(3); int* p = v.data(); return p[3]; }\n' >"$T/overflow.cpp"
    printf '#include <climits>\n#include <cstdio>\n#include <cstdlib>\nint main() { int x = INT_MAX - (std::rand() %% 2); x += 2; std::printf("%%d\\n", x); return 0; }\n' >"$T/ub.cpp"
    printf 'int f() { int a[3] = {1, 2, 3}; return a[5]; }\n' >"$T/bounds.cpp"
    printf '#include <string>\n#include <utility>\nstd::string f() { std::string s = "x"; std::string t = std::move(s); return s + t; }\n' >"$T/moved.cpp"
    expect pass "werror passes clean code" werror "$T/clean.cpp"
    expect fail "werror fails an unused variable" werror "$T/unused.cpp"
    expect pass "sanitize passes clean code" sanitize "$T/clean.cpp"
    expect fail "sanitize catches a heap overflow (ASan)" sanitize "$T/overflow.cpp"
    expect fail "sanitize catches signed overflow (UBSan)" sanitize "$T/ub.cpp"
    expect pass "cppcheck passes clean code" cppcheck_gate "$T/clean.cpp"
    expect fail "cppcheck catches an out-of-bounds index" cppcheck_gate "$T/bounds.cpp"
    expect pass "tidy passes clean code" tidy "$T/clean.cpp"
    expect fail "tidy catches use after move" tidy "$T/moved.cpp"
    # format: a scratch repo with our .clang-format, one commit, then a changed line
    ( cd "$T" && git init -q repo && cp "$ROOT/.clang-format" repo/ && cd repo \
        && printf 'int f() { return 1; }\n' >a.cpp && git add . \
        && git -c user.name=t -c user.email=t@t commit -qm base )
    printf 'int f() { return 1; }\nint g() { return 2; }\n' >"$T/repo/a.cpp"
    expect pass "format passes a formatted change" bash -c "cd '$T/repo' && '$ROOT/tools/quality.sh' format HEAD"
    printf 'int f() { return 1; }\nint    g( ) {return    2 ;}\n' >"$T/repo/a.cpp"
    expect fail "format fails a badly formatted change" bash -c "cd '$T/repo' && '$ROOT/tools/quality.sh' format HEAD"
    printf 'int    f( ) {return    1 ;}\n' >"$T/repo/a.cpp"; (cd "$T/repo" && git -c user.name=t -c user.email=t@t commit -qam old)
    printf 'int    f( ) {return    1 ;}\nint g() { return 2; }\n' >"$T/repo/a.cpp"
    expect pass "format leaves old, unchanged lines alone" bash -c "cd '$T/repo' && '$ROOT/tools/quality.sh' format HEAD"
    rm -rf "$T"
    [ $ok = 0 ] && say "QUALITY SELFTEST PASSED" || say "QUALITY SELFTEST FAILED"
    return $ok
}

cmd="${1:-}"; shift || true
case "$cmd" in
    werror) werror "$@" ;;
    sanitize) sanitize "$@" ;;
    cppcheck) cppcheck_gate "$@" ;;
    tidy) tidy "$@" ;;
    format) format "$@" ;;
    selftest) selftest ;;
    all) rc=0; werror || rc=1; cppcheck_gate || rc=1; tidy || rc=1; sanitize || rc=1; format "$@" || rc=1; exit $rc ;;
    *) sed -n '2,17p' "${BASH_SOURCE[0]}"; exit 2 ;;
esac
