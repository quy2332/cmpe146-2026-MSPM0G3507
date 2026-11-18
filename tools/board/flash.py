#!/usr/bin/env python3
"""Back up, program, verify, and reset an MSPM0G3507 through its XDS110."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import struct

from pyocd.core.exceptions import FlashEraseFailure
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.probe.pydapaccess.interface import pyusb_backend

LABS = {"3": "lab3_spi", "4": "lab4_i2c", "5": "lab5_adc_uart_crc"}


def pin_usb_port(port_path):
    """Make pyocd open only the XDS110 at a USB port path such as 3-1.1.4.

    LaunchPads often ship with the same XDS110 serial number (e.g. MG350001),
    and pyocd opens probes by serial number, so with two boards attached it
    cannot tell them apart. The port path is the same one sysfs shows for the
    board's /dev/ttyACM* devices (udevadm info -q path -n /dev/ttyACM0).
    """
    bus, _, ports = port_path.partition("-")
    wanted = (int(bus), tuple(int(port) for port in ports.split(".")))
    original_find = pyusb_backend.usb_find

    def find_at_port(*args, **kwargs):
        find_all = kwargs.pop("find_all", False)
        devices = [
            device
            for device in original_find(*args, find_all=True, **kwargs)
            if (device.bus, tuple(device.port_numbers or ())) == wanted
        ]
        if find_all:
            return devices
        return devices[0] if devices else None

    pyusb_backend.usb_find = find_at_port


def open_session(args):
    options = {
        "cache.enable_memory": False,
        # Limit outstanding packets for this XDS110's CMSIS-DAP transport.
        "cmsis_dap.limit_packets": True,
    }
    settings = dict(
        target_override="mspm0g3507",
        pack=str(Path(args.pack).resolve()),
        frequency=100_000,
        connect_mode="halt",
        auto_unlock=False,
        # Leave the freshly programmed image running when the probe lets go.
        resume_on_disconnect=True,
        options=options,
    )
    if args.usb_port:
        pin_usb_port(args.usb_port)
        probes = ConnectHelper.get_all_connected_probes(blocking=False)
        if not probes:
            raise RuntimeError(f"No debug probe at USB port {args.usb_port}")
        return Session(probes[0], **settings)
    return ConnectHelper.session_with_chosen_probe(
        blocking=False, unique_id=args.uid, **settings
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lab", choices=sorted(LABS))
    parser.add_argument("--role", choices=["sensor", "display"],
                        help="Lab 5 firmware role")
    parser.add_argument("--pack", required=True, help="TI device-family .pack")
    probe = parser.add_mutually_exclusive_group(required=True)
    probe.add_argument("--uid", help="XDS110 serial number")
    probe.add_argument("--usb-port",
                       help="USB port path of the board, e.g. 3-1.1.4; use "
                            "this when several probes share one serial")
    args = parser.parse_args()
    if (args.lab == "5") != (args.role is not None):
        parser.error("--role is required for lab 5 and only valid for lab 5")
    repo = Path(__file__).resolve().parents[2]
    lab = LABS[args.lab]
    image_name = f"{lab}_{args.role}" if args.role else lab
    image_path = repo / lab / "Debug" / "uart" / f"{image_name}.bin"
    image = image_path.read_bytes()
    if not 8 <= len(image) <= 0x20000:
        raise RuntimeError("Image does not fit the 128 KiB main flash")
    stack, reset = struct.unpack_from("<II", image)
    if not (0x20200000 < stack <= 0x20208000 and reset & 1 and reset < len(image)):
        raise RuntimeError("Invalid initial stack or reset vector")

    session = open_session(args)
    if session is None:
        raise RuntimeError("Debug probe unavailable: check USB access and cable")
    with session:
        target = session.target
        device_id = target.read32(0x41C40004)
        if (device_id >> 12) & 0xFFFF != 0xBB88 or (device_id >> 1) & 0x7FF != 0x17:
            raise RuntimeError(f"Unexpected TI device ID: {device_id:#010x}")
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        backup = image_path.parent / f"before-flash-{timestamp}.bin"
        print(f"Connected to TI device {device_id:#010x}; backing up main flash", flush=True)
        previous = bytes(target.read_memory_block8(0, 0x20000))
        with backup.open("xb") as output:
            output.write(previous)
        print(f"Saved previous main flash: {backup}", flush=True)
        # Run the flash algorithm from a freshly reset core. With an
        # application still running, the first sector erase can report
        # failure (0xFFFFFFFF) even though it erased; retry once.
        target.reset_and_halt()
        for attempt in (1, 2):
            try:
                FileProgrammer(session, chip_erase="sector").program(
                    str(image_path), file_format="bin", base_address=0
                )
                break
            except FlashEraseFailure:
                if attempt == 2:
                    raise
                print("Sector erase reported failure; retrying once", flush=True)
                target.reset_and_halt()
        target.halt()
        target.flush()
        actual = bytes(target.read_memory_block8(0, len(image)))
        if actual != image:
            raise RuntimeError("Flash readback differs from the image; target halted")
        print(f"Verified all {len(image)} programmed bytes; resetting {image_name}", flush=True)
        target.reset()
        target.resume()


if __name__ == "__main__":
    main()
