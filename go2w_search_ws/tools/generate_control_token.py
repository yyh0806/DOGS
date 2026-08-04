#!/usr/bin/env python3
"""Create a local Go2W control token without disclosing or overwriting it."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import secrets
import sys
from typing import Sequence


def create_control_token(path: str | Path) -> Path:
    """Create *path* exclusively with a 256-bit URL-safe token and mode 0600."""

    destination = Path(path)
    token = secrets.token_urlsafe(32)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    fd = os.open(destination, flags, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="ascii", newline="\n") as stream:
            stream.write(token)
            stream.write("\n")
        os.chmod(destination, 0o600)
    except BaseException:
        destination.unlink(missing_ok=True)
        raise
    return destination


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Create a private Go2W bearer-token file without printing it."
    )
    parser.add_argument("path", help="New token file; an existing path is never replaced")
    args = parser.parse_args(argv)
    try:
        destination = create_control_token(args.path)
    except FileExistsError:
        parser.error(f"refusing to overwrite existing token file: {args.path}")
    print(f"created private control-token file: {destination}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
