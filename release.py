#!/usr/bin/env python3
"""Bump FW_VERSION in include/version.h, commit, tag and push. Usage: python release.py 1.4"""
import re
import subprocess
import sys
from pathlib import Path

VERSION_H = Path(__file__).parent / "include" / "version.h"


def run(args):
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"ERROR running {' '.join(args)}:\n{r.stderr.strip() or r.stdout.strip()}")
    return r.stdout.strip()


def main():
    if len(sys.argv) != 2 or not re.fullmatch(r"\d+\.\d+(\.\d+)?", sys.argv[1].lstrip("v")):
        sys.exit("Usage: python release.py <X.Y>   e.g.  python release.py 1.4")
    version = sys.argv[1].lstrip("v")
    if run(["git", "status", "--porcelain"]):
        sys.exit("ERROR: uncommitted changes. Commit or stash them first.")
    text, n = re.subn(r'(#define FW_VERSION\s+")[^"]*(")', rf"\g<1>{version}\g<2>",
                      VERSION_H.read_text(encoding="utf-8"))
    if n != 1:
        sys.exit("ERROR: no '#define FW_VERSION' in include/version.h")
    VERSION_H.write_text(text, encoding="utf-8")
    tag = f"v{version}"
    run(["git", "add", "include/version.h"])
    run(["git", "commit", "-m", f"Release {tag}"])
    run(["git", "tag", "-a", tag, "-m", tag])
    run(["git", "push", "origin"])
    run(["git", "push", "origin", tag])
    print(f"Released {tag}")


if __name__ == "__main__":
    main()
