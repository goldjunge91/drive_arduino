#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# MecaBridge copyright header inserter
# Idempotently prepends Apache-2.0 headers where missing.
#
# Usage:
#   scripts/add_copyright_headers.sh --dry-run
#   scripts/add_copyright_headers.sh [--year YYYY] [--author "Name"]
#
# Notes:
#   * Preserves shebang (#!) and python encoding lines.
#   * Uses // for C/C++ and # for Python/CMake/other text.
#   * Skips files already containing the Apache license string.
# -----------------------------------------------------------------------------
set -euo pipefail
IFS=$'\n\t'

YEAR="$(date +%Y)"
AUTHOR="MecaBridge Project"
DRY_RUN=false
SCOPE_DIR="src/mecabridge_hardware"  # default scope relative to repo root if it exists

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true; shift ;;
    --year) YEAR="$2"; shift 2 ;;
    --author) AUTHOR="$2"; shift 2 ;;
  --scope) SCOPE_DIR="$2"; shift 2 ;;
  --all) SCOPE_DIR=""; shift ;; # future option to span repo
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Try to locate git repo root if available
if ROOT_FROM_GIT="$(git -C "$script_dir" rev-parse --show-toplevel 2>/dev/null || true)" && [[ -n "$ROOT_FROM_GIT" ]]; then
  ROOT_DIR="$ROOT_FROM_GIT"
else
  # Fallback heuristic based on known folder names
  case "$script_dir" in
    */src/mecabridge_hardware) ROOT_DIR="${script_dir%/src/mecabridge_hardware}" ;;
    */scripts) ROOT_DIR="${script_dir%/scripts}" ;;
    *) ROOT_DIR="${script_dir}" ;;
  esac
fi

# Normalize ROOT_DIR (remove any trailing /)
ROOT_DIR="${ROOT_DIR%/}"

# Adjust default scope if repo layout differs (avoid duplicating src/src)
if [[ ! -d "$ROOT_DIR/$SCOPE_DIR" && -d "$ROOT_DIR/mecabridge_hardware" ]]; then
  SCOPE_DIR="mecabridge_hardware"
fi

if [[ -n "$SCOPE_DIR" ]]; then
  TARGET_DIR="$ROOT_DIR/$SCOPE_DIR"
else
  TARGET_DIR="$ROOT_DIR"
fi

if [[ ! -d "$TARGET_DIR" ]]; then
  echo "ERROR: Target directory not found: $TARGET_DIR" >&2
  exit 1
fi

# Build candidate list (extensions we care about)
mapfile -t CANDIDATES < <(find "$TARGET_DIR" \
  -type f \( \
    -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' -o -name '*.c' -o \
    -name '*.py' -o -name '*.launch.py' -o -name 'CMakeLists.txt' -o -name '*.cmake' \
  \) \
  -not -path '*/build/*' -not -path '*/install/*' -not -path '*/log/*')

echo "[mecabridge-headers] ROOT_DIR=$ROOT_DIR" >&2
echo "[mecabridge-headers] TARGET_DIR=$TARGET_DIR" >&2
echo "[mecabridge-headers] CANDIDATES=${#CANDIDATES[@]}" >&2

needs_header() {
  local f="$1"
  if grep -q 'Licensed under the Apache License' "$f" 2>/dev/null; then
    return 1  # already has header
  else
    return 0  # needs header
  fi
}

header_for_style() {
  local style="$1"; local year="$2"; local author="$3"
  if [[ "$style" == "cpp" ]]; then
    cat <<EOF
// Copyright ${year} ${author}
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

EOF
  else
    cat <<EOF
# Copyright ${year} ${author}
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

EOF
  fi
}

detect_style() {
  case "$1" in
    *.c|*.cc|*.cxx|*.cpp|*.h|*.hpp) echo cpp ;;
    CMakeLists.txt|*/CMakeLists.txt|*.cmake) echo hash ;;
    *.launch.py) echo hash ;;
    *.py) echo hash ;;
    *) echo hash ;;
  esac
}

missing=()
skipped=0

for f in "${CANDIDATES[@]}"; do
  # Skip this script itself
  [[ "$f" == *add_copyright_headers.sh ]] && { ((skipped++)); continue; }
  # Skip generated markers
  if grep -q 'DO NOT EDIT' "$f" 2>/dev/null; then ((skipped++)); continue; fi
  if needs_header "$f"; then
    missing+=("$f")
  else
    ((skipped++))
  fi
done

echo "Files missing headers: ${#missing[@]}" >&2
echo "Files missing headers: ${#missing[@]}"  # duplicate to stdout for piping reliability
if $DRY_RUN; then
  if ((${#missing[@]}==0)); then
    echo "Nothing to do."
  else
    for f in "${missing[@]}"; do
      echo "${f#$ROOT_DIR/}"
    done
  fi
  exit 0
fi

if ((${#missing[@]}==0)); then
  echo "Nothing to do."
  exit 0
fi

processed=0

for f in "${missing[@]}"; do
  style=$(detect_style "$f")
  header=$(header_for_style "$style" "$YEAR" "$AUTHOR")
  tmp="${f}.tmp.lic"

  # Handle shebang / encoding for python
  if [[ "$style" == "hash" && "$f" == *.py ]]; then
    first_line="$(head -n1 "$f" || true)"
    second_line="$(sed -n '2p' "$f" || true)"
    insert_after=0
    {
      if [[ "$first_line" =~ ^#!/ ]]; then
        echo "$first_line"; insert_after=1; fi
      if (( insert_after == 1 )) && [[ "$second_line" =~ coding[:=] ]]; then
        echo "$second_line"; insert_after=2; fi
      printf '%s' "$header"
      if (( insert_after == 0 )); then
        tail -n +1 "$f"
      else
        tail -n +$((insert_after+1)) "$f"
      fi
    } > "$tmp"
  else
    { printf '%s' "$header"; cat "$f"; } > "$tmp"
  fi
  mv "$tmp" "$f"
  ((processed++))
  echo "Added: ${f#$ROOT_DIR/}"
done

echo "Added headers: $processed  Skipped existing: $skipped  Total scanned: ${#CANDIDATES[@]}"