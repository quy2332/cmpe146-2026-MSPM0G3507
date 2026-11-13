"""Lab 5 end-to-end test with one debugger connection per board.

Connecting resets a board, so both sessions are opened once up front, the
boards are allowed to boot, and every simulated button press happens inside
those sessions. Presses use the IOMUX INV bit (input inversion):
  S1 display = PA18 / PINCM40, S2 sensor = PB21 / PINCM49.
"""
import sys
import threading
import time

import serial
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.probe.pydapaccess.interface import pyusb_backend

pack, log_dir = sys.argv[1:3]
import glob
import os


def tty_for(port_path):
    """The console (interface 0) tty of the XDS110 at a USB port path."""
    for tty in glob.glob("/sys/class/tty/ttyACM*"):
        if os.path.realpath(tty + "/device").endswith(f"/{port_path}:1.0"):
            return "/dev/" + os.path.basename(tty)
    raise RuntimeError(f"no console tty for {port_path}")


SENSOR = ("3-1.1.4", tty_for("3-1.1.4"))
DISPLAY = ("3-1.1.2", tty_for("3-1.1.2"))
T0 = time.time()


def mark(text):
    print(f"[{time.time() - T0:6.2f}] {text}", flush=True)
IOMUX = 0x40428000
INV = 0x04000000
GPIOB_DOUT = 0x400A2000 + 0x1280
LED_BITS = {"red": 26, "green": 27, "blue": 22}
true_find = pyusb_backend.usb_find


def pincm(n):
    return IOMUX + 4 + (n - 1) * 4


def open_session(port_path):
    bus, _, ports = port_path.partition("-")
    wanted = (int(bus), tuple(int(p) for p in ports.split(".")))

    def find_at_port(*args, **kwargs):
        find_all = kwargs.pop("find_all", False)
        found = [d for d in true_find(*args, find_all=True, **kwargs)
                 if (d.bus, tuple(d.port_numbers or ())) == wanted]
        return found if find_all else (found[0] if found else None)

    pyusb_backend.usb_find = find_at_port
    probe = ConnectHelper.get_all_connected_probes(blocking=False)[0]
    session = Session(probe, target_override="mspm0g3507", pack=pack, frequency=100_000,
                      connect_mode="attach", auto_unlock=False, resume_on_disconnect=True,
                      options={"cache.enable_memory": False, "cmsis_dap.limit_packets": True})
    session.open()
    return session


logs = {}
stop = threading.Event()


def reader(name, port):
    lines = []
    pending = b""
    with serial.Serial(port, 115200, timeout=0.05) as s:
        s.reset_input_buffer()
        while not stop.is_set():
            pending += s.read(4096)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                lines.append(f"[{time.time() - T0:6.2f}] " + line.decode(errors="replace").rstrip())
    logs[name] = "\n".join(lines)


threads = [threading.Thread(target=reader, args=("sensor", SENSOR[1])),
           threading.Thread(target=reader, args=("display", DISPLAY[1]))]
for th in threads:
    th.start()
time.sleep(0.5)

mark('open sensor session')
sensor = open_session(SENSOR[0])
mark('open display session')
display = open_session(DISPLAY[0])
s, d = sensor.target, display.target
for t in (s, d):
    if t.get_state().name == "HALTED":
        t.resume()
mark('both resumed')
time.sleep(2.5)  # both boards boot and print their banners


def leds():
    dout = d.read32(GPIOB_DOUT)
    on = [c for c, bit in LED_BITS.items() if (dout >> bit) & 1]
    return "+".join(on) or "off"


def press_s1():
    mark('press S1 on display')
    address = pincm(40)
    value = d.read32(address)
    d.write32(address, value | INV)
    d.flush()  # pyocd queues writes until a read or flush
    time.sleep(0.15)
    d.write32(address, value)
    d.flush()
    time.sleep(1.2)


def hold_s2(held):
    mark(f'sensor S2 held={held}')
    address = pincm(49)
    value = s.read32(address)
    s.write32(address, (value | INV) if held else (value & ~INV))
    s.flush()


results = []
press_s1()
results.append(("press 1, normal", leds()))
hold_s2(True)
press_s1()
results.append(("press 2, sensor S2 held", leds()))
hold_s2(False)
press_s1()
results.append(("press 3, S2 released", leds()))
press_s1()
results.append(("press 4, normal", leds()))

time.sleep(0.5)
sensor.close()
display.close()
stop.set()
for th in threads:
    th.join()
for label, color in results:
    print(f"{label}: LED {color}")
for name in ("sensor", "display"):
    text = logs.get(name, "")
    with open(f"{log_dir}/{name}-e2e.log", "w") as f:
        f.write(text)
    print(f"===== {name} console =====")
    print(text)
