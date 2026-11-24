#!/usr/bin/env bash
# Build (and optionally upload) envTwo for the Raspberry Pi Pico with arduino-cli.
#
#   ./build.sh                      compile only -> build/out/envTwo.ino.uf2
#   ./build.sh upload [PORT]        compile, then upload (PORT default /dev/ttyACM0)
#   ./build.sh setup                one-time install of the core + pinned libraries
#   ./build.sh schematic            ERC-check both KiCad schematics, then re-render
#                                   envTwo_schematic.png (kicad/envTwo.kicad_sch) and
#                                   envTwo_schematic_wired.png (kicad/wired/envTwo_wired.kicad_sch)
#                                   (needs KiCad + pdftoppm)
#
# arduino-cli requires the sketch folder name to match the .ino name, and this
# repo is checked out as "env-two", so we compile from build/envTwo/, a folder of
# symlinks to the real sources (a symlink to the whole repo dir gets resolved and
# fails the name check).
set -euo pipefail

FQBN="rp2040:rp2040:rpipico"
CORE_URL="https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json"
SETUP_NAME="Setup_envTwo_RP2040_ILI9341_touch.h"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$REPO/build/envTwo"
OUT="$REPO/build/out"

die() { echo "error: $*" >&2; exit 1; }

libdir() {
  local ud
  ud="$(arduino-cli config get directories.user 2>/dev/null || echo "$HOME/Arduino")"
  echo "$ud/libraries"
}

do_setup() {
  arduino-cli config add board_manager.additional_urls "$CORE_URL" 2>/dev/null || true
  arduino-cli core update-index
  arduino-cli core install rp2040:rp2040
  # MCP_DAC is pinned: 0.3.0+ removed the 3-arg constructor and setGPIOpins().
  arduino-cli lib install TFT_eSPI TFT_eWidget RotaryEncoder Bounce2 MCP_DAC@0.2.1
  install_tft_setup
}

# TFT_eSPI reads its pin config from inside the library folder; point it at ours.
install_tft_setup() {
  local L; L="$(libdir)/TFT_eSPI"
  [ -d "$L" ] || die "TFT_eSPI not installed"
  cp "$REPO/$SETUP_NAME" "$L/User_Setups/$SETUP_NAME"
  if ! grep -q "^#include <User_Setups/$SETUP_NAME>" "$L/User_Setup_Select.h"; then
    [ -f "$L/User_Setup_Select.h.orig" ] || cp "$L/User_Setup_Select.h" "$L/User_Setup_Select.h.orig"
    sed -i "s|^#include <User_Setup.h>|//#include <User_Setup.h>\n#include <User_Setups/$SETUP_NAME> // envTwo|" \
      "$L/User_Setup_Select.h"
  fi
  echo "TFT_eSPI configured with $SETUP_NAME"
}

check_env() {
  local L; L="$(libdir)"
  grep -q "^#include <User_Setups/$SETUP_NAME>" "$L/TFT_eSPI/User_Setup_Select.h" 2>/dev/null \
    || die "TFT_eSPI is not using $SETUP_NAME (library updated?) - run: ./build.sh setup"
  cmp -s "$REPO/$SETUP_NAME" "$L/TFT_eSPI/User_Setups/$SETUP_NAME" \
    || die "TFT_eSPI copy of $SETUP_NAME differs from the repo - run: ./build.sh setup"
  grep -q '^version=0\.2\.1$' "$L/MCP_DAC/library.properties" 2>/dev/null \
    || die "MCP_DAC must be 0.2.1 - run: arduino-cli lib install MCP_DAC@0.2.1"
}

do_compile() {
  check_env
  mkdir -p "$SKETCH" "$OUT"
  find "$SKETCH" -maxdepth 1 -type l -delete   # drop links to files that no longer exist
  local f
  for f in "$REPO"/*.ino "$REPO"/*.h "$REPO"/*.cpp; do
    [ -e "$f" ] || continue
    case "$(basename "$f")" in Setup_*) continue ;; esac   # TFT_eSPI config lives in the library
    ln -sfn "$f" "$SKETCH/$(basename "$f")"
  done
  arduino-cli compile -b "$FQBN" --warnings "${WARNINGS:-default}" --output-dir "$OUT" "$SKETCH"
  echo "UF2: $OUT/envTwo.ino.uf2"
}

need_arduino_cli() {
  command -v arduino-cli >/dev/null || die "arduino-cli not found (expected in ~/.local/bin)"
}

# Re-render the README schematic image from the KiCad project. Temp files go
# in build/ (inside the repo) because the KiCad snap cannot write to /tmp.
KCLI=""
find_kicad_cli() {
  KCLI="$(command -v kicad-cli || true)"
  [ -n "$KCLI" ] || { [ -x /snap/bin/kicad.kicad-cli ] && KCLI=/snap/bin/kicad.kicad-cli; }
  [ -n "$KCLI" ] || die "kicad-cli not found (install KiCad 9)"
  command -v pdftoppm >/dev/null || die "pdftoppm not found (apt package poppler-utils)"
}

# export_schematic SCHEMATIC PNG_NAME DPI: ERC-check one schematic, render it to $REPO/PNG_NAME
export_schematic() {
  local sch="$1" name="$2" dpi="$3" tmp="$REPO/build/schematic"
  [ -f "$sch" ] || die "missing $sch"
  mkdir -p "$tmp"
  # errors only: the 'library not in sym-lib-table' items are warnings, and the
  # symbols are embedded in the schematic anyway
  if ! "$KCLI" sch erc --severity-error --exit-code-violations -o "$tmp/erc.rpt" "$sch" >/dev/null; then
    cat "$tmp/erc.rpt" >&2
    die "ERC found errors in $(basename "$sch") - fix them before exporting (report: $tmp/erc.rpt)"
  fi
  "$KCLI" sch export pdf -o "$tmp/$name.pdf" "$sch" >/dev/null
  pdftoppm -png -r "$dpi" -singlefile "$tmp/$name.pdf" "$tmp/$name"
  mv "$tmp/$name.png" "$REPO/$name.png"
  echo "ERC clean; wrote $REPO/$name.png"
}

do_schematic() {
  find_kicad_cli
  # label-based schematic, A3 at 210 dpi (~3470x2460 px)
  export_schematic "$REPO/kicad/envTwo.kicad_sch" envTwo_schematic 210
  # wired redraw in the 2023 layout, A4 at 300 dpi (3508x2480 px, the 2023 image size)
  export_schematic "$REPO/kicad/wired/envTwo_wired.kicad_sch" envTwo_schematic_wired 300
}

case "${1:-compile}" in
  compile)   need_arduino_cli; do_compile ;;
  upload)    need_arduino_cli; do_compile; arduino-cli upload -b "$FQBN" -p "${2:-/dev/ttyACM0}" --input-dir "$OUT" "$SKETCH" ;;
  setup)     need_arduino_cli; do_setup ;;
  schematic) do_schematic ;;
  *)         die "usage: $0 [compile | upload [PORT] | setup | schematic]" ;;
esac
