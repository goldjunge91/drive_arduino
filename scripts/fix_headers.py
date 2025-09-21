#!/usr/bin/env python3
"""Bulk style maintenance helper for mecabridge_hardware.

Focus (non-invasive):
 1. Ensure trailing newline at EOF.
 2. Strip trailing whitespace.
 3. Normalize header guards for headers missing or malformed.
 4. Insert missing #include <memory> / <algorithm> when explicitly required by cpplint diagnostics
    (We perform heuristic: look for shared_ptr / unique_ptr / make_shared and absence of <memory>.)
 5. Remove duplicate self-include lines inside the same file (e.g., watchdog.hpp duplicate include).

Copyright headers: delegated to existing bash script. Run that first.

Guard naming scheme:
  <PATH_WITH_SLASHES_REPLACED_BY_DOUBLE_UNDERSCORES><FILENAME_UPPER>_<EXT>_
  Root prefix deduced: mecabridge_hardware/ or mecabridge_utils/ preserved per cpplint suggestions.

The script is idempotent.
"""
from __future__ import annotations
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET = ROOT / 'src' / 'mecabridge_hardware'
HEADER_EXTS = {'.h', '.hpp'}
SOURCE_EXTS = {'.c', '.cc', '.cxx', '.cpp'}

INCLUDE_GUARD_RE = re.compile(r'^#ifndef\s+([A-Z0-9_]+)\s*$')
ENDIF_GUARD_RE = re.compile(r'^#endif\s*(//.*)?$')
SHARED_PTR_RE = re.compile(r'\b(std::)?shared_ptr<')
UNIQUE_PTR_RE = re.compile(r'\b(std::)?unique_ptr<')
MAKE_SHARED_RE = re.compile(r'\bstd::make_shared<')
ALGO_COPY_RE = re.compile(r'\bstd::copy\b|\bcopy\s*\(')
ALGO_MINMAX_RE = re.compile(r'\bstd::(min|max)\b')

# Known files needing <algorithm> from cpplint output (copy, min usage) may be safer than broad heuristic
FORCE_ALGO = {
    'src/mecabridge_hardware/src/mecabridge_utils/serial/serial_backend.hpp',
    'src/mecabridge_hardware/test/mecabridge/testable_mecabridge_serial_protocol.cpp',
}
FORCE_MEMORY = set()  # cpplint explicitly mentioned one header lacking <memory>

# Map of specific file -> required additional includes (deduped later)
EXPLICIT_INCLUDES = {
    'src/mecabridge_hardware/include/mecabridge_hardware/mecabridge_serial_protocol.h': ['<memory>'],
}

def desired_guard(path: Path) -> str:
    """Compute header guard approximating cpplint suggestions.

    Rules distilled from cpplint output examples:
      * Drop leading 'src'.
      * Use the LAST occurrence of one of root tokens: mecabridge_utils, mecabridge_hardware, mecabridge.
      * Start guard from that token onward.
      * Skip directory markers: include, src, test.
      * Collapse consecutive duplicate tokens (e.g., mecabridge_hardware/mecabridge_hardware).
      * Separate path components with double underscore and append trailing underscore.
    """
    rel_parts = list(path.relative_to(ROOT).parts)
    # Drop leading 'src'
    if rel_parts and rel_parts[0] == 'src':
        rel_parts = rel_parts[1:]
    # Identify root token (prefer specific ordering)
    priority = ['mecabridge_hardware', 'mecabridge_utils', 'mecabridge']
    root_index = 0
    for i, part in enumerate(rel_parts):
        if part in priority:
            root_index = i
            break
    after = rel_parts[root_index:]
    # Filter out common structural dirs
    structural = {'include', 'src', 'test'}
    raw_tokens = [t for t in after[:-1] if t not in structural]
    path_tokens: list[str] = []
    for t in raw_tokens:
        if not path_tokens or path_tokens[-1] != t:
            path_tokens.append(t)
    filename = after[-1]
    if '.' in filename:
        stem, ext = filename.rsplit('.', 1)
        file_token = f"{stem}_{ext}".upper()
    else:
        file_token = filename.upper()
    # Compose: ROOT + subdirs + FILE_TOKEN
    root_token = after[0]
    components = [root_token] + path_tokens + [file_token]
    # Collapse duplicates
    collapsed: list[str] = []
    for c in components:
        up = c.upper()
        if not collapsed or collapsed[-1].upper() != up:
            collapsed.append(c)
    guard = '__'.join(c.upper().replace('.', '_') for c in collapsed) + '_'
    return guard


def process_header_guard(lines: list[str], path: Path) -> list[str]:
    # Detect existing guard pattern (#ifndef ... #define ... ... #endif ...)
    guard_name = desired_guard(path)
    # cpplint examples show double underscore between directory and file, maintain scheme above.
    has_ifndef = None
    has_define = None
    endif_index = None
    for i, line in enumerate(lines[:10]):  # usually guards at top
        if line.startswith('#ifndef '):
            has_ifndef = i
        if line.startswith('#define '):
            has_define = i
        if has_ifndef is not None and has_define is not None:
            break
    for j in range(len(lines)-1, -1, -1):
        if lines[j].startswith('#endif'):
            endif_index = j
            break
    # If any piece missing or names mismatched, rebuild guards.
    need_rewrite = False
    if has_ifndef is None or has_define is None or endif_index is None:
        need_rewrite = True
    else:
        current_ifndef = lines[has_ifndef].split()[1]
        if current_ifndef != guard_name:
            need_rewrite = True

    if not need_rewrite:
        # Ensure #endif comment style matches cpplint (only if endif_index detected)
        if endif_index is not None and not re.search(re.escape(guard_name), lines[endif_index]):
            lines[endif_index] = f'#endif  // {guard_name}\n'
        return lines

    # Remove any existing leading guard lines to avoid duplicates
    new_body_start = 0
    if has_ifndef is not None and has_define is not None and has_define == has_ifndef + 1:
        # Skip old pattern until after define
        new_body_start = has_define + 1
    new_lines = []
    new_lines.append(f'#ifndef {guard_name}\n')
    new_lines.append(f'#define {guard_name}\n\n')
    new_lines.extend(lines[new_body_start:])
    # Ensure single trailing newline
    if not new_lines[-1].endswith('\n'):
        new_lines[-1] += '\n'
    # Fix final endif
    if endif_index is not None:
        # remove existing endif if mismatched to append new
        if not re.search(re.escape(guard_name), new_lines[-1]):
            if new_lines[-1].startswith('#endif'):
                new_lines.pop()
        new_lines.append(f'#endif  // {guard_name}\n')
    else:
        new_lines.append(f'#endif  // {guard_name}\n')
    return new_lines


def needs_memory(lines: list[str]) -> bool:
    use = any(r.search(''.join(lines)) for r in (SHARED_PTR_RE, UNIQUE_PTR_RE, MAKE_SHARED_RE))
    if not use:
        return False
    for l in lines[:50]:
        if '#include <memory>' in l:
            return False
    return True


def needs_algorithm(lines: list[str], rel: str) -> bool:
    if rel in FORCE_ALGO:
        for l in lines[:80]:
            if '#include <algorithm>' in l:
                return False
        return True
    text = ''.join(lines)
    if ALGO_COPY_RE.search(text) or ALGO_MINMAX_RE.search(text):
        for l in lines[:80]:
            if '#include <algorithm>' in l:
                return False
        return True
    return False


def insert_includes(lines: list[str], rel: str) -> list[str]:
    add = []
    if rel in EXPLICIT_INCLUDES:
        add.extend([inc for inc in EXPLICIT_INCLUDES[rel] if not any(inc in l for l in lines)])
    if needs_memory(lines):
        add.append('<memory>')
    if needs_algorithm(lines, rel):
        add.append('<algorithm>')
    if not add:
        return lines
    # insert after first block of includes or after guard lines
    insertion_index = 0
    # Skip guard
    if lines and lines[0].startswith('#ifndef') and len(lines) > 2 and lines[1].startswith('#define'):
        insertion_index = 2
        while insertion_index < len(lines) and lines[insertion_index].strip() == '':
            insertion_index += 1
    # Find last consecutive include starting at top
    i = insertion_index
    while i < len(lines) and lines[i].startswith('#include'):
        i += 1
    insertion_index = i
    snippet = ''.join(f'#include {inc}\n' for inc in add)
    return lines[:insertion_index] + [snippet] + lines[insertion_index:]


def remove_duplicate_self_include(lines: list[str], path: Path) -> list[str]:
    # If a line includes its own header twice, remove duplicates
    filename = path.name
    # Match either just the filename or any relative path ending with the filename
    include_pattern_simple = f'#include "{filename}"'
    include_pattern_suffix = f'/{filename}"'
    seen = False
    new = []
    for l in lines:
        if include_pattern_simple in l or (include_pattern_suffix in l and '#include "' in l):
            if seen:
                continue
            seen = True
        new.append(l)
    return new


def strip_trailing_ws_and_ensure_newline(text: str) -> str:
    lines = text.splitlines()
    lines = [re.sub(r'[ \t]+$', '', l) for l in lines]
    return '\n'.join(lines) + '\n'


def transform_file(path: Path):
    rel = path.relative_to(ROOT).as_posix()
    original = path.read_text(encoding='utf-8', errors='ignore')
    new_text = original
    # Step 1 strip trailing whitespace + ensure final newline early
    new_text = strip_trailing_ws_and_ensure_newline(new_text)
    lines = new_text.splitlines(keepends=True)

    if path.suffix in HEADER_EXTS:
        lines = process_header_guard(lines, path)
    # Remove duplicate self-include (particular issue in watchdog.hpp)
    lines = remove_duplicate_self_include(lines, path)
    # Insert missing includes heuristically
    if path.suffix in HEADER_EXTS or path.suffix in SOURCE_EXTS:
        lines = insert_includes(lines, rel)

    final_text = ''.join(lines)
    if final_text != original:
        path.write_text(final_text, encoding='utf-8')
        return True
    return False


def main():
    changed = 0
    examined = 0
    for p in TARGET.rglob('*'):
        if not p.is_file():
            continue
        if p.suffix not in HEADER_EXTS | SOURCE_EXTS:
            continue
        examined += 1
        if transform_file(p):
            changed += 1
    print(f"Processed {examined} files. Modified {changed}.")

if __name__ == '__main__':
    main()
