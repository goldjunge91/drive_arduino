#!/usr/bin/env bash
# Thin wrapper to run the central copyright header insertion script
# Allows: cd src/mecabridge_hardware && ./add_headers.sh --dry-run

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Prefer repo scripts/ version if present
if [[ -f "$SCRIPT_DIR/../../scripts/add_copyright_headers.sh" ]]; then
  exec bash "$SCRIPT_DIR/../../scripts/add_copyright_headers.sh" --scope "src/mecabridge_hardware" "$@"
elif [[ -f "$SCRIPT_DIR/add_copyright_headers.sh" ]]; then
  exec bash "$SCRIPT_DIR/add_copyright_headers.sh" --scope "src/mecabridge_hardware" "$@"
else
  echo "ERROR: Could not locate main header script relative to $SCRIPT_DIR" >&2
  exit 1
fi