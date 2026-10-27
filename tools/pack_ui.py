#!/usr/bin/env python3
"""Packt web/*.html als gzip-PROGMEM nach src/web/UiPagesGz.h."""
from __future__ import annotations

import gzip
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "src" / "web" / "UiPagesGz.h"
PAGES = (
    ("index.html", "PAGE_MAIN_GZ", "PAGE_MAIN_GZ_LEN"),
    ("ota.html", "PAGE_OTA_GZ", "PAGE_OTA_GZ_LEN"),
    ("login.html", "PAGE_LOGIN_GZ", "PAGE_LOGIN_GZ_LEN"),
)


def emit_array(name: str, len_name: str, gz: bytes) -> list[str]:
    lines = [
        f"static const size_t {len_name} = {len(gz)};",
        f"static const uint8_t {name}[] PROGMEM = {{",
    ]
    row: list[str] = []
    for b in gz:
        row.append(f"0x{b:02x}")
        if len(row) == 12:
            lines.append("  " + ", ".join(row) + ",")
            row = []
    if row:
        lines.append("  " + ", ".join(row) + ",")
    lines.append("};")
    lines.append("")
    return lines


def main() -> int:
    lines = [
        "#pragma once",
        "",
        "#include <Arduino.h>",
        "",
        "/** Autogeneriert von tools/pack_ui.py — nicht von Hand editieren. */",
        "",
    ]
    notes = []
    for filename, arr, ln in PAGES:
        src = ROOT / "web" / filename
        if not src.is_file():
            print(f"pack_ui: fehlt {src}", file=sys.stderr)
            return 1
        raw = src.read_bytes()
        gz = gzip.compress(raw, compresslevel=9)
        notes.append(f"{filename} {len(raw)}→{len(gz)}")
        lines.append(f"/* Quelle: web/{filename} ({len(raw)} B) → gzip ({len(gz)} B) */")
        lines.extend(emit_array(arr, ln, gz))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    text = "\n".join(lines) + "\n"
    if OUT.is_file() and OUT.read_text() == text:
        print("pack_ui: unveraendert (" + ", ".join(notes) + ")")
        return 0
    OUT.write_text(text)
    print("pack_ui: " + ", ".join(notes) + f" → {OUT.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
