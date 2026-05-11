"""
serial_handler.py — Low-level serial protocol handler for Neato Botvac D6.

Communicates with the Neato over USB serial (/dev/ttyACM0).
All commands are ASCII text terminated by newline.
Responses end with Ctrl-Z (0x1A).

Based on jnugen/neato_robot and LoyVanBeek/neato_ros2 drivers.
"""

import logging
import threading
import time
from typing import Optional

import serial

logger = logging.getLogger(__name__)

# Neato D6 physical constants
BASE_WIDTH_MM = 241       # wheel-to-wheel distance in mm (measured center-to-center: 9.5 in = 241.3 mm)
MAX_SPEED_MM_S = 300      # max motor speed in mm/s
LIDAR_POINTS = 360        # number of LIDAR scan points per revolution


class NeatoSerial:
    """Thread-safe serial interface to a Neato Botvac."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self._port_name = port
        self._baudrate = baudrate
        self._serial: Optional[serial.Serial] = None

        # Response buffering (threaded reader)
        self._read_thread: Optional[threading.Thread] = None
        self._reading = False
        self._lock = threading.RLock()
        self._response_queue: list[list[str]] = []
        self._current_response: list[str] = []
        self._flush_event = threading.Event()

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the serial port and start the background reader thread."""
        logger.info("Opening serial port %s @ %d", self._port_name, self._baudrate)
        self._serial = serial.Serial(
            self._port_name, self._baudrate, timeout=0
        )
        if not self._serial.is_open:
            raise ConnectionError(f"Failed to open {self._port_name}")

        self._serial.flushInput()
        self._serial.flushOutput()

        # Start background reader
        self._reading = True
        self._read_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="neato-serial-reader"
        )
        self._read_thread.start()

        # Sync the serial line — wait for reader to start, then flush
        time.sleep(0.1)
        self._serial.write(b"\n\n\n")
        time.sleep(0.2)
        self.flush()

        logger.info("Serial port opened successfully")

    def disconnect(self) -> None:
        """Stop the reader thread and close the serial port."""
        self._reading = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("Serial port closed")

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # Command interface
    # ------------------------------------------------------------------

    def flush(self) -> None:
        """Drain any pending responses and the reader's internal buffer."""
        self._flush_event.set()
        time.sleep(0.05)  # give reader thread time to see the flag
        with self._lock:
            self._response_queue.clear()
            self._current_response = []

    def send_command(self, command: str) -> bool:
        """Send a command string to the Neato. Returns True on success."""
        if not self.is_connected:
            logger.error("Cannot send command — not connected")
            return False

        logger.debug("TX: %s", command)
        self._serial.write(f"{command}\n".encode("ascii"))
        return True

    def send_command_and_drain(self, command: str, timeout: float = 1.0) -> bool:
        """Send a command and wait for its response to arrive, discarding it.

        Use for commands like TestMode, SetLDSRotation where you don't
        need the response but must wait for it to clear the serial line.
        """
        self.flush()
        if not self.send_command(command):
            return False
        # Wait until we get a complete response (is_last=True) or timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line, is_last = self.get_response(timeout=deadline - time.monotonic())
            if not line:
                break
            logger.debug("drain(%s): %s%s", command, line,
                         " [last]" if is_last else "")
            if is_last:
                break
        return True

    def get_response(self, timeout: float = 1.0) -> tuple[str, bool]:
        """
        Get the next line from the current command response.

        Returns (line, is_last_line).
        If timeout expires with no data, returns ("", False).
        """
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            # If current_response is empty, try to pop a new batch
            if not self._current_response:
                with self._lock:
                    if self._response_queue:
                        self._current_response = self._response_queue.pop(0)
                if not self._current_response:
                    time.sleep(0.01)
                    continue

            if self._current_response:
                line = self._current_response.pop(0)
                is_last = len(self._current_response) == 0
                return line, is_last

        return "", False

    def read_until_tag(self, tag: str, timeout: float = 1.0) -> bool:
        """Read lines until we find one starting with `tag`. Returns False on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line, _ = self.get_response(timeout=deadline - time.monotonic())
            if not line:
                return False
            if line.startswith(tag):
                return True
        return False

    def send_and_collect(self, command: str, header_tag: str,
                         timeout: float = 1.0) -> list[str]:
        """
        Send a command, skip lines until `header_tag`, then collect
        all remaining response lines.

        Returns list of CSV data lines (excluding the header).
        """
        self.flush()
        self.send_command(command)
        if not self.read_until_tag(header_tag, timeout=timeout):
            logger.warning("Timeout waiting for header '%s' from '%s'", header_tag, command)
            return []

        lines = []
        while True:
            line, is_last = self.get_response(timeout=timeout)
            if not line:
                break
            lines.append(line)
            if is_last:
                break
        return lines

    # ------------------------------------------------------------------
    # Background reader thread
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        """
        Continuously reads bytes from serial. When a Ctrl-Z (0x1A) is seen,
        the accumulated lines are pushed as a complete response to the queue.
        """
        buf = bytearray()

        while self._reading:
            # Check if a flush was requested
            if self._flush_event.is_set():
                buf.clear()
                self._flush_event.clear()

            try:
                chunk = self._serial.read(4096)
            except (serial.SerialException, OSError) as exc:
                logger.error("Serial read error: %s", exc)
                chunk = b""

            if not chunk:
                time.sleep(0.005)
                continue

            buf.extend(chunk)

            # Process all complete responses (delimited by Ctrl-Z 0x1A)
            while 0x1A in buf:
                idx = buf.index(0x1A)
                raw = buf[:idx]
                buf = buf[idx + 1:]

                text = raw.decode("ascii", errors="replace")
                text = text.rstrip("\r\n")
                lines = [
                    ln.strip()
                    for ln in text.split("\n")
                    if ln.strip()
                ]
                if lines:
                    with self._lock:
                        self._response_queue.append(lines)

    # ------------------------------------------------------------------
    # Convenience: high-level Neato commands
    # ------------------------------------------------------------------

    def set_test_mode(self, on: bool) -> bool:
        return self.send_command_and_drain(f"TestMode {'On' if on else 'Off'}")

    def set_lds_rotation(self, on: bool) -> bool:
        return self.send_command_and_drain(f"SetLDSRotation {'On' if on else 'Off'}")

    def set_motors(self, left_dist_mm: int, right_dist_mm: int, speed_mm: int) -> bool:
        return self.send_command(
            f"SetMotor LWheelDist {left_dist_mm} RWheelDist {right_dist_mm} Speed {speed_mm}"
        )

    def set_backlight(self, on: bool) -> bool:
        return self.send_command_and_drain(f"SetLED {'BacklightOn' if on else 'BacklightOff'}")

    def play_sound(self, sound_id: int) -> bool:
        return self.send_command_and_drain(f"PlaySound {sound_id}")

    def set_system_mode(self, mode: str) -> bool:
        """mode: 'Shutdown', 'PowerCycle', 'Hibernate'"""
        self.set_test_mode(True)
        return self.send_command_and_drain(f"SetSystemMode {mode}")

    def send_event(self, event: str, skey: str) -> bool:
        """Send a SetEvent command using the robot's SKey."""
        return self.send_command(f"SetEvent event {event} SKey {skey}")

    def set_user_setting(self, setting: str, value: str) -> bool:
        """Change a user setting (e.g., 'EcoMode', 'ON')."""
        return self.send_command_and_drain(
            f"SetUserSettings {setting} {value}"
        )

    def set_navigation_mode(self, mode: str) -> bool:
        """Set navigation mode: Normal, Gentle, Deep, Quick."""
        return self.send_command_and_drain(f"SetNavigationMode {mode}")

    def set_vacuum(self, on: bool, speed_pct: int = 65) -> bool:
        """Turn vacuum motor on/off at a given speed (1-100%). Requires TestMode."""
        if on:
            speed_pct = max(1, min(100, speed_pct))
            return self.send_command_and_drain(
                f"SetMotor VacuumSpeed {speed_pct} VacuumOn"
            )
        return self.send_command_and_drain("SetMotor VacuumOff")

    def clear_errors(self) -> bool:
        """Clear all UI errors."""
        return self.send_command_and_drain("SetUIError clearall")

    # ------------------------------------------------------------------
    # SKey computation (RC4-based, from brainslug)
    # ------------------------------------------------------------------

    @staticmethod
    def compute_skey(serial_number: str) -> str:
        """
        Compute the SKey from the robot's serial number for SetEvent commands.

        The serial number format is 'OPSxxxxx,YYYYYYYYYYYY' where Y is a 12-char
        MAC-like identifier. The SKey is derived using RC4.
        """
        # Extract the 12-char MAC after the comma
        comma_pos = serial_number.find(",")
        if comma_pos < 0 or len(serial_number) < comma_pos + 13:
            raise ValueError(f"Invalid serial number format: {serial_number}")
        mac = serial_number[comma_pos + 1 : comma_pos + 13]

        # RC4 key (static, from Neato firmware)
        key = [0x68, 0x36, 0x43, 0x58, 0x09, 0x09, 0x3A, 0x3C, 0x2A, 0x7B, 0x59]

        # RC4 Key Scheduling Algorithm (KSA)
        s = list(range(256))
        j = 0
        for i in range(256):
            j = (j + s[i] + key[i % 11]) & 0xFF
            s[i], s[j] = s[j], s[i]

        # RC4 Pseudo-Random Generation Algorithm (PRGA) — 12 bytes
        keystream = []
        i = 0
        j = 0
        for _ in range(12):
            i = (i + 1) & 0xFF
            j = (j + s[i]) & 0xFF
            s[i], s[j] = s[j], s[i]
            keystream.append(s[(s[i] + s[j]) & 0xFF])

        # XOR MAC with keystream, hex-encode
        result = ""
        for k in range(12):
            result += f"{keystream[k] ^ ord(mac[k]):02x}"

        # Append char at position len/2 (brainslug compatibility)
        result += result[6]

        return result
