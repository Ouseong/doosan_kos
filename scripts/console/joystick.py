#!/usr/bin/env python3
"""Dependency-free Linux joystick reader (/dev/input/jsN).

No pygame / evdev needed — reads raw ``js_event`` structs from the joydev
character device in a background thread and keeps the latest axis/button
state. Built for the Logitech Extreme 3D Pro used by the Task Space screen
joystick teleop, but works with any jsN device.

joydev event layout (struct js_event, 8 bytes):
    __u32 time      event timestamp (ms)
    __s16 value     axis value (-32767..32767) or button (0/1)
    __u8  type      0x01 button, 0x02 axis, 0x80 OR'd in for init events
    __u8  number    axis / button index

Extreme 3D Pro js-axis order (from its ABS bitmap ABS_X,Y,RZ,THROTTLE,HAT):
    axis 0 = stick X (left/right)
    axis 1 = stick Y (forward/back)   forward = negative
    axis 2 = twist  (RZ)
    axis 3 = throttle lever           forward = negative
    axis 4 = hat X,  axis 5 = hat Y
    button 0 = trigger
"""
import glob
import os
import select
import struct
import threading
import time

_JS_EVENT_FMT = "IhBB"                       # u32, s16, u8, u8
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FMT)   # 8 bytes
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80

# Preferred device patterns, most specific first. The by-id symlink survives
# re-plugging into a different USB port; js0 is the last-resort fallback.
# NOTE: the by-id "*-joystick" symlinks point at the joydev jsN node, but
# there is ALSO a "*-event-joystick" symlink pointing at the evdev eventNN
# node — which uses a different (input_event) struct. We must read the joydev
# node, so any path containing "event" is rejected below.
_DEVICE_PATTERNS = (
    "/dev/input/by-id/*Extreme_3D*-joystick",
    "/dev/input/by-id/*Extreme*3D*joystick",
    "/dev/input/by-id/*-joystick",
    "/dev/input/js*",
)


def find_joystick_device():
    """Return the path of the best-match joydev jsN device, or None.
    Rejects evdev (event*) nodes — those use a different event struct."""
    for pat in _DEVICE_PATTERNS:
        for path in sorted(glob.glob(pat)):
            if "event" in os.path.basename(path):
                continue
            return path
    return None


def deadzone(v, dz=0.12):
    """Apply a centred dead-zone to a -1..1 axis and rescale the remainder
    back to the full -1..1 range, so output is continuous past the dead-zone."""
    if abs(v) < dz:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)


class Joystick:
    """Background reader for a single jsN device. Thread-safe state access."""

    def __init__(self, path=None):
        self.path = path or find_joystick_device()
        self._fd = None
        self._axes = {}
        self._buttons = {}
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self.connected = False     # device file opened
        self.ready = False         # device confirmed live (sent init events)
        self.error = None

    # ── lifecycle ──
    def start(self, ready_timeout=0.4):
        """(Re)scan, open the device, spawn the reader thread, and confirm the
        joystick is actually live. Returns True if the device opened. Sets
        ``self.ready`` True once joydev init events arrive (every real device
        emits them immediately on open), so callers can distinguish a truly
        connected stick from a stale/empty node. Never raises."""
        if self._running:
            return self.connected
        # Re-detect each time so a stick plugged in (or moved to another port)
        # after construction is still found on ARM.
        self.path = self.path or find_joystick_device()
        self.ready = False
        if not self.path or not os.path.exists(self.path):
            self.error = "no joystick device found (is it plugged in?)"
            return False
        try:
            self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            self.error = f"open {self.path}: {e}"
            return False
        self.connected = True
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # Wait briefly for the joydev init-state burst that confirms a real,
        # connected device is talking.
        deadline = time.monotonic() + ready_timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._axes or self._buttons:
                    self.ready = True
                    break
            time.sleep(0.02)
        if not self.ready:
            self.error = "device opened but sent no data"
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        self.connected = False

    # ── reader thread ──
    def _loop(self):
        buf = b""
        while self._running:
            # select with timeout lets the thread notice _running going False
            try:
                r, _, _ = select.select([self._fd], [], [], 0.2)
            except (OSError, ValueError):
                break
            if not r:
                continue
            try:
                data = os.read(self._fd, _JS_EVENT_SIZE * 64)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError as e:
                self.error = f"read: {e}"
                self.connected = False
                break
            if not data:
                continue
            buf += data
            while len(buf) >= _JS_EVENT_SIZE:
                chunk, buf = buf[:_JS_EVENT_SIZE], buf[_JS_EVENT_SIZE:]
                _t, value, etype, number = struct.unpack(_JS_EVENT_FMT, chunk)
                etype &= ~_JS_EVENT_INIT          # treat init events as real
                with self._lock:
                    if etype == _JS_EVENT_AXIS:
                        self._axes[number] = max(-1.0, min(1.0, value / 32767.0))
                    elif etype == _JS_EVENT_BUTTON:
                        self._buttons[number] = int(value)

    # ── state access ──
    def axis(self, n, default=0.0):
        with self._lock:
            return self._axes.get(n, default)

    def button(self, n):
        with self._lock:
            return bool(self._buttons.get(n, 0))

    def snapshot(self):
        with self._lock:
            return dict(self._axes), dict(self._buttons)
