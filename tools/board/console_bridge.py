#!/usr/bin/env python3
"""Share a serial console with GTKTerm and retain received bytes in a log."""

import argparse
import os
from pathlib import Path
import pty
import select
import tty

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--link", default="/tmp/lab34-console")
    parser.add_argument("--log", default="/tmp/lab34-console.log")
    args = parser.parse_args()
    master, slave = pty.openpty()
    tty.setraw(slave)
    link = Path(args.link)
    if link.is_symlink():
        link.unlink()
    link.symlink_to(os.ttyname(slave))
    try:
        with serial.Serial(args.port, 115200, timeout=0, exclusive=True) as board:
            with open(args.log, "ab", buffering=0) as log:
                print(f"Console: {link}; receive log: {args.log}", flush=True)
                while True:
                    ready, _, _ = select.select([master, board.fileno()], [], [])
                    if board.fileno() in ready:
                        data = board.read(4096)
                        if data:
                            log.write(data)
                            os.write(master, data)
                    if master in ready:
                        board.write(os.read(master, 4096))
    finally:
        link.unlink(missing_ok=True)
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
