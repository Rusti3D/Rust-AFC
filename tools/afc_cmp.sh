#!/bin/bash
# Rust-AFC parity: each tests/cases/*.test (or the given ones) runs with the
# Python AFC on Kalico and with kaliru-afc (kaliru + Rust-AFC), diffed by
# refdiff.py (serialqueue trace and exit identical, klippy.log identical after
# masking wall clock / thread timing). A case whose first line is
#   # fake_ace: <fake_ace.py options>
# gets a fresh fake ACE Pro (tools/fake_ace.py) on its serial for each side;
# a "# fake_moonraker" line, a fake Moonraker (tools/fake_moonraker.py) on
# 127.0.0.1:7199, for the AFC stats loads and unloads update. A case's
# <name>.gcode beside it is put on the virtual SD card (SDCARD_PRINT_FILE).
#
# Needs (WSL): Kalico at ~/rk/kalico (KALICO), the reference venv ~/rk/venv315
# (CPython 3.15 + Kalico's requirements + pyserial 3.5), the MCU dictionaries
# in /ci_build/dict, the AFC Python sources (AFC_SRC: its extras files and
# the AFC_ACE plugin directory; default the copy COSMOS ships, from the
# user's printer) and a release build of kaliru-afc.
# usage: afc_cmp.sh [test files...]

HERE=$(cd "$(dirname "$0")/.." && pwd)
KALICO=${KALICO:-$HOME/rk/kalico}
AFC_SRC=${AFC_SRC:-$(realpath "$HERE/../Originals_Base/addons")}
BIN=${KALIRU_AFC_BIN:-$HOME/rk/target-rafc/release/kaliru-afc}
STATE=/tmp/rust_afc_state
K=$KALICO/klippy
installed=()
for f in "$AFC_SRC"/afc_extras/*.py; do cp "$f" $K/extras/; installed+=("$K/extras/$(basename "$f")"); done
# (ACE_SRC: a cosmoace_afc checkout; the port follows its commit 09b9cc3)
cp -r "${ACE_SRC:-$AFC_SRC/plugins/AFC_ACE}" $K/plugins/AFC_ACE && installed+=("$K/plugins/AFC_ACE")
fake=
mr=
trap 'rm -rf "${installed[@]}" $STATE; kill $fake $mr 2>/dev/null' EXIT
[ $# -eq 0 ] && set -- "$HERE"/tests/cases/*.test
# a fresh state directory (AFC.var, the fake ACE's link) and fake ACE per run
fresh() {
    if [ -n "$fake" ]; then kill $fake 2>/dev/null; wait $fake 2>/dev/null || true; fi
    fake=
    if [ -n "$mr" ]; then kill $mr 2>/dev/null; wait $mr 2>/dev/null || true; fi
    mr=
    rm -rf $STATE && mkdir -p $STATE
    [ -f "${1%.test}.gcode" ] && cp "${1%.test}.gcode" $STATE/
    if grep -q '^# fake_moonraker' "$1"; then
        python3 "$HERE/tools/fake_moonraker.py" 7199 & mr=$!
    fi
    local opts
    opts=$(sed -n '1s/^# fake_ace: //p' "$1")
    if [ -n "$opts" ]; then
        # shellcheck disable=SC2086
        python3 "$HERE/tools/fake_ace.py" $STATE/ace_tty $opts & fake=$!
        sleep 0.5
    fi
}
same=0
for t in "$@"; do
    t=$(realpath "$t")
    name=$(basename "$t" .test)
    rm -rf ~/rk/rafc_py ~/rk/rafc_rs
    fresh "$t"
    python3 "$HERE/tools/refrun.py" --timeout "${TIMEOUT:-300}" --out ~/rk/rafc_py --cwd $KALICO \
        --klippy "$HOME/rk/venv315/bin/python -m klippy" "$t" < /dev/null > /dev/null
    fresh "$t"
    KALIRU_KLIPPY_DIR=$K \
    KALIRU_LIBPYTHON=${KALIRU_LIBPYTHON:-$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0} \
    PYTHONPATH=$HOME/rk/venv315/lib/python3.15/site-packages \
    python3 "$HERE/tools/refrun.py" --timeout "${TIMEOUT:-300}" --out ~/rk/rafc_rs --klippy "$BIN" "$t" < /dev/null > /dev/null
    if python3 "$HERE/tools/refdiff.py" ~/rk/rafc_py ~/rk/rafc_rs > ~/rk/rafc_$name.diff 2>&1; then
        echo "SAME $name (exit $(cat ~/rk/rafc_py/$name/0/exit))"
        same=$((same + 1))
    else
        echo "DIFF $name (~/rk/rafc_$name.diff)"
        head -n 30 ~/rk/rafc_$name.diff | sed 's/^/    /'
    fi
done
echo "$same/$# identical"
