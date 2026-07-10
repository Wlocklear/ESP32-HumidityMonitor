#!/usr/bin/env python3
"""
release.py — bump FIRMWARE_VERSION in src/main.cpp, commit, tag, and push.

Usage:
    python release.py 1.4
    python release.py v1.4   (leading 'v' is stripped automatically)
"""

import re
import subprocess
import sys
from pathlib import Path

MAIN_CPP = Path(__file__).parent / "src" / "main.cpp"


def run(args):
    result = subprocess.run(args, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"\nERROR running: {' '.join(str(a) for a in args)}")
        print(result.stderr.strip() or result.stdout.strip())
        sys.exit(1)
    return result.stdout.strip()


def check_clean():
    print("Checking working tree...")
    status = run(["git", "status", "--porcelain"])
    if status:
        print("ERROR: Working tree has uncommitted changes. Commit or stash them first.")
        print(status)
        sys.exit(1)
    print("  Working tree is clean.")


def bump_version(version):
    print(f"Updating {MAIN_CPP.relative_to(Path(__file__).parent)} to v{version}...")
    text = MAIN_CPP.read_text(encoding="utf-8")

    # #define FIRMWARE_VERSION  "vX.Y"
    new_text, n1 = re.subn(
        r'(#define FIRMWARE_VERSION\s+"v)\d+\.\d+(")',
        rf'\g<1>{version}\g<2>',
        text,
    )
    if n1 == 0:
        print("ERROR: Could not find '#define FIRMWARE_VERSION' line in src/main.cpp.")
        sys.exit(1)

    # * Humidity Controller Firmware vX.Y  (header comment block)
    new_text, n2 = re.subn(
        r'(Firmware v)\d+\.\d+',
        rf'\g<1>{version}',
        new_text,
    )
    if n2 == 0:
        print("ERROR: Could not find 'Firmware v' version string in header comment.")
        sys.exit(1)

    MAIN_CPP.write_text(new_text, encoding="utf-8")
    print(f"  #define FIRMWARE_VERSION -> \"v{version}\"")
    print(f"  Header comment -> Firmware v{version}")


def git_commit(version):
    msg = f"Bump FIRMWARE_VERSION to v{version}"
    print(f"Committing: {msg!r}")
    run(["git", "add", "src/main.cpp"])
    run(["git", "commit", "-m", msg])
    print("  Committed.")


def git_tag(version):
    tag = f"v{version}"
    print(f"Creating annotated tag {tag}...")
    run(["git", "tag", "-a", tag, "-m", tag])
    print(f"  Tagged {tag}.")


def git_push(version):
    print("Pushing commit to origin...")
    run(["git", "push", "origin"])
    print("  Pushed commit.")

    tag = f"v{version}"
    print(f"Pushing tag {tag} to origin...")
    run(["git", "push", "origin", tag])
    print(f"  Pushed {tag}.")


def main():
    if len(sys.argv) != 2:
        print("Usage: python release.py <version>   e.g.  python release.py 1.4")
        sys.exit(1)

    version = sys.argv[1].lstrip("v")

    if not re.fullmatch(r"\d+\.\d+", version):
        print(f"ERROR: Version must be X.Y (e.g. 1.4), got: {version!r}")
        sys.exit(1)

    check_clean()
    bump_version(version)
    git_commit(version)
    git_tag(version)
    git_push(version)
    print(f"\nRelease v{version} complete.")


if __name__ == "__main__":
    main()
