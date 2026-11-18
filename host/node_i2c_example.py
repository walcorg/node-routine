#!/usr/bin/env python3
"""
WALCORG TERRAFORMING CORP. - I2C master example (Raspberry Pi Zero W / shield)
for the walcorg_node_v45.ino firmware.

Requires:  pip install smbus2
Recommended on the Pi: software I2C (dtoverlay=i2c-gpio), because the BCM2835
hardware I2C does not handle clock stretching correctly.
"""
import struct
import time
from smbus2 import SMBus, i2c_msg

BUS = 1          # with dtoverlay=i2c-gpio this is usually another number (e.g. 3)
ADDR = 0x10      # 0x10..0x13 depending on the node's solder jumpers

STATES = {0: "IDLE", 1: "BUSY", 2: "UVC", 3: "PLAYING"}
ERRORS = {0: "none", 1: "unknown command", 2: "invalid argument",
          3: "busy", 4: "blocked by active UVC", 5: "aborted"}
FLAGS = ["DATA_READY", "DARK_ALERT", "XERR", "UV_ACTIVE",
         "SENSITIVE_RANGE", "INT_EXP", "WATCHDOG_RESET"]


class Node:
    def __init__(self, bus, addr=ADDR):
        self.bus, self.addr = bus, addr

    def _write(self, data):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, list(data)))

    def command(self, letter, *args):
        self._write([ord(letter), *args])

    def status(self):
        w = i2c_msg.write(self.addr, [ord("?")])
        r = i2c_msg.read(self.addr, 8)
        self.bus.i2c_rdwr(w, r)
        b = list(r)
        return {
            "state": STATES.get(b[0], b[0]),
            "flags": [n for i, n in enumerate(FLAGS) if b[1] & (1 << i)],
            "last_cmd": chr(b[2]) if b[2] else "",
            "error": ERRORS.get(b[3], b[3]),
            "fw": b[4],
            "uv_sec": b[5] | (b[6] << 8),
            "addr": hex(b[7]),
        }

    def wait(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.status()
            if s["state"] in ("IDLE", "UVC"):
                return s
            time.sleep(0.02)
        raise TimeoutError("the node did not finish in time")

    def page(self, n):
        """0 = last measurement; 1 = dark; 2..13 = LED 0..11; 14 = config."""
        w = i2c_msg.write(self.addr, [ord("R"), n])
        r = i2c_msg.read(self.addr, 32)
        self.bus.i2c_rdwr(w, r)
        return list(struct.unpack("<16H", bytes(list(r))))

    # ---- high-level operations ----
    def check_dark(self):
        self.command("G"); s = self.wait()
        return "DARK_ALERT" not in s["flags"], self.page(0)

    def measure_led(self, n):
        self.command("H", n); self.wait()
        return self.page(0)

    def encode_text(self, text):
        """Shows each character on the 8 data LEDs and measures the 16 sensors."""
        results = []
        for ch in text.encode("ascii"):
            self.command("K", ch); self.wait()
            results.append((chr(ch), self.page(0)))
        self.command("O")
        return results

    def measure_pattern(self, levels):
        """Laura Encoding challenge: 12 LED intensities (0-4095) -> 16 readings."""
        if len(levels) != 12 or any(not 0 <= v <= 4095 for v in levels):
            raise ValueError("levels must be 12 integers in 0..4095")
        args = []
        for v in levels:
            args += [v & 0xFF, v >> 8]
        self.command("X", *args); self.wait()
        return self.page(0)

    def calibrate(self):
        self.command("C"); self.wait(timeout=30)
        return {"dark": self.page(1),
                **{f"led{i}": self.page(2 + i) for i in range(12)}}

    def disinfect(self, seconds=300):
        self.command("E", seconds & 0xFF, seconds >> 8)

    def stop_uv(self):
        self.command("F")

    def safe_mode(self):
        self.command("D")

    def set_param(self, pid, value):
        self.command("P", pid, value & 0xFF, value >> 8)


if __name__ == "__main__":
    with SMBus(BUS) as bus:
        node = Node(bus)
        print("Status:", node.status())

        dark, readings = node.check_dark()
        print("Darkness OK" if dark else "ALERT: light is leaking in", readings)

        print("LED 3 ->", node.measure_led(3))

        for ch, readings in node.encode_text("WALCORG"):
            print(ch, readings[8:])        # SENS_T0..T7 (receiver wall)

        # 60 s test disinfection (only with an empty node and the lid closed)
        # node.disinfect(60)
        # while node.status()["state"] == "UVC":
        #     print("UV remaining:", node.status()["uv_sec"], "s"); time.sleep(5)
