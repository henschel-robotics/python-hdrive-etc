"""
HDrive EtherCAT SDK — Main motor control class

Provides :class:`HDriveETC`, the high-level API for controlling
Henschel Robotics HDrive servo motors over EtherCAT (via PySOEM).
"""

import struct
import threading
import time

import pysoem

from .exceptions import ConfigurationError, ConnectionError, CommunicationError
from .protocol import (
    configure_pdo_mapping,
    decode_rx_pdo,
    encode_tx_pdo,
    cia402_state_from_status,
    next_control_word,
    error_message,
    ERROR_CODES,
)


class Mode:
    """CiA 402 operation-mode constants.

    Use these with :meth:`HDriveETC.set_mode`.
    """

    POSITION = 8
    """Cyclic synchronous position mode (CSP)."""

    VELOCITY = 2
    """Cyclic synchronous velocity / (CSV) mode."""

    STEPPER = -3
    """VELOCITY teppermotor mode."""

    TORQUE = 4
    """Cyclic synchronous torque mode (CST)."""

    PROFILE_POSITION = 1
    """Profile position mode."""

    PROFILE_VELOCITY = 3
    """Profile velocity mode."""

    CALIBRATION = -99
    """Manufacturer-specific calibration mode."""

    STOP = 0
    """Motor disabled / stopped."""


class Error:
    """HDrive error codes (mirrors firmware ``Error.h``)."""

    PROGRAM_ERROR_NULL_POINTER = 1
    POSITION_OVER_OR_UNDERFLOW = 15
    OVER_TEMPERATURE = 16
    UNDER_VOLTAGE = 17
    OVER_VOLTAGE = 18
    OVER_SPEED = 19
    POSITIVE_SOFTWARE_POSITION_LIMIT = 20
    NEGATIVE_SOFTWARE_POSITION_LIMIT = 21
    NEGATIVE_LIMIT_SWITCH_TRIGGERED = 22
    POSITIVE_LIMIT_SWITCH_TRIGGERED = 23
    LIMIT_SWITCH_TIMEOUT = 25
    POS_SENSOR_ERROR = 26
    POWER_STAGE_ERROR = 27
    WATCHDOG_TIMEOUT = 28
    SPI_POS_SENSOR_ERROR = 30
    CALIB_ERROR = 31
    WRONG_TICKET_FORMAT = 32
    CONFIGURATION_ERROR = 33
    IP_CONFIGURATION_ERROR = 34
    CONFIGURATION_FILE_WRONG_FORMAT = 35
    OBJECT_NOT_FOUND_IN_DICTIONARY = 36
    HARDWARE_NOT_COMPATIBLE = 37
    ETHERCAT_CONNECTION_INTERRUPTED = 38
    CAN_SPECIAL_COMMAND_NOT_FOUND = 40
    LIMIT_SWITCH_MIN_DISTANCE_TO_END_SWITCH = 50
    MOTOR_MODE_NOT_EXISTING = 51
    WRONG_ARGUMENT_COUNT_IN_TICKET = 52
    NULL_POINTER_ERROR = 53


class HDriveETC:
    """Control an HDrive servo motor over EtherCAT.

    Two-thread architecture for optimal real-time performance:

    1. **ProcessData thread** (5 ms cycle) — fast send/receive of raw
       EtherCAT frames.  Minimal latency, no complex processing.
    2. **PDO Update thread** (configurable *cycle_time_ms*) — decodes
       received PDOs, runs the CiA 402 state machine, encodes TX PDOs
       with user setpoints.

    A third background thread monitors slave health and attempts
    automatic recovery.

    Args:
        adapter_index: Network-adapter index (see :meth:`list_adapters`).
        slave_index: EtherCAT slave index (usually ``0`` for a single motor).
        cycle_time_ms: PDO update cycle time in milliseconds.

    Example::

        with HDriveETC(adapter_index=0) as motor:
            motor.set_mode(Mode.TORQUE)
            motor.set_torque(200)
            time.sleep(2)
            motor.stop()
    """

    def __init__(self, adapter_index=0, slave_index=0, cycle_time_ms=10):
        self.adapter_index = adapter_index
        self.slave_index = slave_index
        self.cycle_time = cycle_time_ms / 1000.0

        # EtherCAT receive timeout: 2x cycle time (microseconds)
        self.rx_timeout_us = int(cycle_time_ms * 2000)

        self.master = None
        self._processdata_thread = None
        self._pdo_thread = None
        self._check_thread = None
        self._pd_stop_event = None
        self._pdo_stop_event = None
        self._check_stop_event = None
        self._lock = threading.Lock()
        self._state = {}

        # Communication statistics
        self._comm_ok_count = 0
        self._comm_error_count = 0
        self._actual_wkc = 0
        self._last_wkc = 0

        # Cycle timing statistics
        self._cycle_time_min = float("inf")
        self._cycle_time_max = 0.0
        self._cycle_time_sum = 0.0
        self._last_cycle_time = None

        # Auto-reconnect
        self.auto_reconnect = True
        self._reconnecting = threading.Event()

        # Setpoints (thread-safe via lock)
        self._target_position = 0
        self._target_velocity = 0
        self._target_torque = 0
        self._target_mode = Mode.TORQUE
        self._target_debug_outputs = None
        self._manual_controlword = None

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    def __del__(self):
        try:
            if self.master:
                self.disconnect()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    @staticmethod
    def list_adapters():
        """Print and return available network adapters.

        Returns:
            list: Adapter objects from PySOEM.
        """
        adapters = pysoem.find_adapters()
        print("Available network adapters:")
        for i, adapter in enumerate(adapters):
            print(f"  {i}: {adapter.name} - {adapter.desc}")
        return adapters

    def connect(self):
        """Open the EtherCAT connection and bring the motor to OP state.

        Raises:
            ConnectionError: If no adapters/slaves are found or the state
                transition fails.
            ConfigurationError: If PDO mapping fails.
        """
        adapters = pysoem.find_adapters()
        if not adapters:
            raise ConnectionError("No network adapters found")

        if self.adapter_index >= len(adapters):
            raise ConnectionError(
                f"Adapter index {self.adapter_index} out of range "
                f"(0-{len(adapters) - 1})"
            )

        adapter = adapters[self.adapter_index]
        print(f"Connecting to: {adapter.name}")

        self.master = pysoem.Master()
        self.master.open(adapter.name)
        self.master.in_op = False
        self.master.do_check_state = False

        if self.master.config_init() <= 0:
            raise ConnectionError("No EtherCAT slaves found")

        print(f"Found {len(self.master.slaves)} EtherCAT slave(s)")

        for slave in self.master.slaves:
            slave.is_lost = False

        # Configure PDO mapping
        try:
            configure_pdo_mapping(self.master.slaves[self.slave_index])
        except Exception as exc:
            raise ConfigurationError(f"PDO mapping failed: {exc}") from exc

        self.master.config_map()

        if self.master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            raise ConnectionError("Failed to reach SAFE-OP state")
        print("Reached SAFE-OP state")

        # Start communication threads BEFORE requesting OP state
        print("Starting communication threads...")
        self._start_threads()

        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        print("Requested OP state transition...")

        if self.master.state_check(pysoem.OP_STATE, 50000) != pysoem.OP_STATE:
            self._pd_stop_event.set()
            self._pdo_stop_event.set()
            raise ConnectionError("Failed to reach OP state")

        self.master.in_op = True
        print("Reached OP state - motor ready")

    def disconnect(self):
        """Stop the motor and close the EtherCAT connection."""
        self._safe_stop()

        if self.master:
            self.master.in_op = False

        for evt in (self._pd_stop_event, self._pdo_stop_event, self._check_stop_event):
            if evt:
                evt.set()

        for thr in (self._processdata_thread, self._pdo_thread, self._check_thread):
            if thr:
                thr.join(timeout=2.0)

        self._processdata_thread = None
        self._pdo_thread = None
        self._check_thread = None

        if self.master:
            self.master.close()
            self.master = None
            print("Disconnected")

    # ------------------------------------------------------------------
    # Motor control API
    # ------------------------------------------------------------------

    def set_mode(self, mode):
        """Set the operation mode.

        Args:
            mode: One of :class:`Mode` constants (e.g. ``Mode.TORQUE``).
        """
        with self._lock:
            self._target_mode = mode

    def set_position(self, position):
        """Set target position in degrees.

        Args:
            position: Target position (float).
        """
        with self._lock:
            self._target_position = int(position * 10)

    def set_velocity(self, velocity):
        """Set target velocity.

        Args:
            velocity: Target velocity value.
        """
        with self._lock:
            self._target_velocity = int(velocity)

    def set_torque(self, torque):
        """Set target torque in milli-Newton-metres.

        Args:
            torque: Target torque (int, mNm).
        """
        with self._lock:
            self._target_torque = int(torque)

    def stop(self):
        """Stop the motor (sets mode to ``Mode.STOP`` and torque to 0).

        Returns:
            bool: Always ``True``.
        """
        self.set_mode(Mode.STOP)
        self.set_torque(0)
        return True

    def enable(self):
        """Enable the motor.

        The CiA 402 state machine handles the transition automatically.
        """
        pass

    def disable(self):
        """Disable the motor (zero torque and velocity setpoints)."""
        with self._lock:
            self._target_torque = 0
            self._target_velocity = 0

    # ------------------------------------------------------------------
    # Controlword override
    # ------------------------------------------------------------------

    def set_controlword(self, controlword):
        """Manually override the CiA 402 controlword.

        This bypasses the automatic state machine.  Call
        :meth:`clear_controlword` to return to automatic control.

        Args:
            controlword: 16-bit controlword value.
        """
        with self._lock:
            self._manual_controlword = int(controlword) & 0xFFFF

    def clear_controlword(self):
        """Clear the manual controlword override.

        The automatic CiA 402 state machine resumes.
        """
        with self._lock:
            self._manual_controlword = None

    def get_controlword(self):
        """Return the current manual controlword, or ``None`` if automatic."""
        with self._lock:
            return self._manual_controlword

    # ------------------------------------------------------------------
    # Telemetry / status
    # ------------------------------------------------------------------

    def get_position(self):
        """Return the actual position in encoder increments."""
        with self._lock:
            return self._state.get("position", 0)

    def get_velocity(self):
        """Return the actual velocity in RPM."""
        with self._lock:
            return self._state.get("velocity", 0) / 6

    def get_torque(self):
        """Return the actual torque."""
        with self._lock:
            return self._state.get("torque", 0)

    def get_status(self):
        """Return a snapshot of all motor status fields as a dict."""
        with self._lock:
            return dict(self._state)

    def get_state_name(self):
        """Return the current CiA 402 state name (e.g. ``"operation_enabled"``)."""
        with self._lock:
            status = self._state.get("status", 0)
            return cia402_state_from_status(status)

    def get_debug_values(self):
        """Return 16 REAL32 debug values from PDO 0x1A05, or ``None``."""
        return self._state.get("debug_values", None)

    def get_comm_stats(self):
        """Return communication and cycle-timing statistics.

        Returns:
            dict: Keys include ``ok_count``, ``error_count``,
            ``success_rate``, ``cycle_time_actual_ms``, etc.
        """
        with self._lock:
            total = self._comm_ok_count + self._comm_error_count
            success_rate = (self._comm_ok_count / total * 100) if total > 0 else 0.0

            cycle_avg = (
                (self._cycle_time_sum / self._comm_ok_count) if self._comm_ok_count > 0 else 0.0
            )
            jitter = (
                max(self._cycle_time_max - self.cycle_time, self.cycle_time - self._cycle_time_min)
                if self._comm_ok_count > 0
                else 0.0
            )

            return {
                "ok_count": self._comm_ok_count,
                "error_count": self._comm_error_count,
                "total_count": total,
                "success_rate": success_rate,
                "last_wkc": self._last_wkc,
                "cycle_time_target_ms": self.cycle_time * 1000,
                "cycle_time_actual_ms": (
                    self._last_cycle_time * 1000 if self._last_cycle_time else 0.0
                ),
                "cycle_time_min_ms": (
                    self._cycle_time_min * 1000 if self._cycle_time_min != float("inf") else 0.0
                ),
                "cycle_time_max_ms": self._cycle_time_max * 1000,
                "cycle_time_avg_ms": cycle_avg * 1000,
                "cycle_time_jitter_ms": jitter * 1000,
            }

    # ------------------------------------------------------------------
    # SDO read / write
    # ------------------------------------------------------------------

    def read_sdo(self, index, subindex, data_type=None):
        """Read an SDO object from the drive.

        Args:
            index: Object index (e.g. ``0x6660``).
            subindex: Object subindex.
            data_type: Optional :mod:`struct` format character
                (``'I'`` = UINT32, ``'H'`` = UINT16, ``'i'`` = INT32, …).
                When *None* (default) the size is auto-detected from the
                slave response and unpacked as an unsigned integer.

        Returns:
            The unpacked value, or ``None`` on error.
        """
        if not self.master:
            raise CommunicationError("Not connected")
        try:
            slave = self.master.slaves[self.slave_index]
            if data_type is not None:
                size = struct.calcsize(f"<{data_type}")
                data_bytes = slave.sdo_read(index, subindex, size)
                if data_bytes and len(data_bytes) >= size:
                    return struct.unpack(f"<{data_type}", data_bytes[:size])[0]
                return None
            data_bytes = slave.sdo_read(index, subindex)
            if not data_bytes:
                return None
            n = len(data_bytes)
            if n >= 4:
                return struct.unpack("<I", data_bytes[:4])[0]
            if n >= 2:
                return struct.unpack("<H", data_bytes[:2])[0]
            return data_bytes[0]
        except Exception as exc:
            print(f"Error reading SDO 0x{index:04X}:0x{subindex:02X}: {exc}")
            return None

    def write_sdo(self, index, subindex, value, data_type=None):
        """Write a value to an SDO object.

        The data size is determined automatically: it first tries 4 bytes
        (UINT32), then 2 bytes (UINT16), then 1 byte (UINT8).  Pass
        *data_type* explicitly to force a specific packing format.

        Args:
            index: Object index.
            subindex: Object subindex.
            value: Integer value to write.
            data_type: Optional :mod:`struct` format character.
        """
        if not self.master:
            raise CommunicationError("Not connected")
        slave = self.master.slaves[self.slave_index]
        val = int(value)
        if data_type is not None:
            slave.sdo_write(index, subindex, struct.pack(f"<{data_type}", val))
            return
        formats = ("i", "h", "b") if val < 0 else ("I", "H", "B")
        for fmt in formats:
            try:
                slave.sdo_write(index, subindex, struct.pack(f"<{fmt}", val))
                return
            except Exception:
                continue
        raise CommunicationError(
            f"Failed to write SDO 0x{index:04X}:0x{subindex:02X}"
        )

    # ------------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------------

    def get_error_code(self):
        """Read the CiA 402 error code (0x603F).

        Returns:
            int: Error code (0 = no error), or ``None`` on failure.
        """
        if not self.master:
            return None
        try:
            slave = self.master.slaves[self.slave_index]
            raw = slave.sdo_read(0x603F, 0x00, 2)
            if raw and len(raw) >= 2:
                return struct.unpack("<H", raw)[0]
            return None
        except Exception as exc:
            print(f"Error reading error code: {exc}")
            return None

    def get_error_message(self, error_code=None):
        """Return a human-readable message for *error_code*.

        If *error_code* is ``None`` the current error code is read
        from the drive automatically.
        """
        if error_code is None:
            error_code = self.get_error_code()
        return error_message(error_code)

    def clear_error(self):
        """Clear the drive error by writing to 0x6637.

        Returns:
            bool: ``True`` on success.
        """
        if not self.master:
            return False
        try:
            slave = self.master.slaves[self.slave_index]
            slave.sdo_write(0x6637, 0x00, struct.pack("<b", 100))
            return True
        except Exception as exc:
            print(f"Error clearing error code: {exc}")
            return False

    # ------------------------------------------------------------------
    # Debug outputs
    # ------------------------------------------------------------------

    def set_debug_outputs(self, values):
        """Set 8 INT32 debug output values (PDO 0x1605).

        Args:
            values: List/tuple of 8 integers, or ``None`` to disable.
        """
        if values is not None:
            if len(values) != 8:
                raise ValueError("values must be a list/tuple of 8 INT32 values")
            values = tuple(int(v) for v in values)
        with self._lock:
            self._target_debug_outputs = values

    # ------------------------------------------------------------------
    # Control tuning helpers
    # ------------------------------------------------------------------

    def set_rotor_inertia(self, inertia):
        """Write rotor inertia (0x6633)."""
        self.write_sdo(0x6633, 0x00, inertia)

    def set_damping(self, damping):
        """Write damping coefficient (0x6634)."""
        self.write_sdo(0x6634, 0x00, damping)

    def set_control_bandwidth(self, torque_bw, velocity_bw=None, position_bw=None):
        """Write control-loop bandwidth parameters (0x6640).

        Args:
            torque_bw: Torque bandwidth (subindex 1, required).
            velocity_bw: Velocity bandwidth (subindex 2, optional).
            position_bw: Position bandwidth (subindex 3, optional).
        """
        self.write_sdo(0x6640, 0x01, int(torque_bw))
        if velocity_bw is not None:
            self.write_sdo(0x6640, 0x02, int(velocity_bw))
        if position_bw is not None:
            self.write_sdo(0x6640, 0x03, int(position_bw))

    def trigger_parameter_calculation(self):
        """Trigger firmware parameter recalculation (0x6637 = 1)."""
        if not self.master:
            raise CommunicationError("Not connected")
        slave = self.master.slaves[self.slave_index]
        slave.sdo_write(0x6637, 0x00, struct.pack("<B", 1))

    def configure_control_parameters(
        self, rotor_inertia, damping, torque_bw, velocity_bw=None, position_bw=None
    ):
        """Set all control parameters and trigger recalculation.

        Args:
            rotor_inertia: Rotor inertia value.
            damping: Damping coefficient.
            torque_bw: Torque control bandwidth.
            velocity_bw: Velocity bandwidth (optional).
            position_bw: Position bandwidth (optional).
        """
        self.set_rotor_inertia(rotor_inertia)
        self.set_damping(damping)
        self.set_control_bandwidth(torque_bw, velocity_bw, position_bw)
        self.trigger_parameter_calculation()
        print("Control parameters configured and calculation triggered")

    # ------------------------------------------------------------------
    # Internal: threads
    # ------------------------------------------------------------------

    def _attempt_reconnect(self):
        """Tear down the master and rebuild the connection from scratch.

        Called by ``_check_loop`` when sustained communication loss is
        detected.  The ``_reconnecting`` event pauses the processdata
        and PDO threads while the master is being rebuilt.
        """
        self._reconnecting.set()
        self.master.in_op = False
        print("[RECONNECT] Connection lost — attempting reconnect ...")

        # Give processdata / PDO loops time to see the flag and pause
        time.sleep(0.1)

        try:
            self.master.close()
        except Exception:
            pass

        backoff = 1.0
        while not self._check_stop_event.is_set():
            try:
                adapters = pysoem.find_adapters()
                if self.adapter_index >= len(adapters):
                    raise CommunicationError("Adapter not found")

                adapter = adapters[self.adapter_index]
                self.master = pysoem.Master()
                self.master.open(adapter.name)
                self.master.in_op = False
                self.master.do_check_state = False

                if self.master.config_init() <= 0:
                    raise CommunicationError("No EtherCAT slaves found")

                for slave in self.master.slaves:
                    slave.is_lost = False

                configure_pdo_mapping(self.master.slaves[self.slave_index])
                self.master.config_map()

                if self.master.state_check(
                    pysoem.SAFEOP_STATE, 50000
                ) != pysoem.SAFEOP_STATE:
                    raise CommunicationError("Failed to reach SAFE-OP")

                tx_init = encode_tx_pdo(0, 0, 0, Mode.TORQUE, 0x0006, None)
                self.master.slaves[self.slave_index].output = tx_init

                self.master.state = pysoem.OP_STATE
                self.master.write_state()

                # Pump processdata while waiting for OP transition
                deadline = time.time() + 5.0
                reached_op = False
                while time.time() < deadline:
                    self.master.send_processdata()
                    self.master.receive_processdata(self.rx_timeout_us)
                    if self.master.state_check(
                        pysoem.OP_STATE, 1000
                    ) == pysoem.OP_STATE:
                        reached_op = True
                        break
                    time.sleep(0.005)

                if not reached_op:
                    raise CommunicationError("Failed to reach OP")

                self.master.in_op = True
                self._comm_error_count = 0
                with self._lock:
                    self._target_mode = Mode.STOP
                    self._target_position = 0
                    self._target_velocity = 0
                    self._target_torque = 0
                    self._manual_controlword = 0x0006
                self._reconnecting.clear()
                print("[RECONNECT] Successfully reconnected — motor held in STOP")
                return

            except Exception as exc:
                print(f"[RECONNECT] Attempt failed: {exc}  — retrying in {backoff:.0f}s")
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None
                self._check_stop_event.wait(backoff)
                backoff = min(backoff * 2, 10.0)

    def _safe_stop(self):
        """Zero setpoints and wait for the state machine to leave OP."""
        try:
            if hasattr(self, "_lock") and self._lock:
                with self._lock:
                    self._target_torque = 0
                    self._target_velocity = 0
                    self._target_mode = Mode.STOP

            threads_alive = any(
                t and t.is_alive()
                for t in (self._processdata_thread, self._pdo_thread)
            )

            if threads_alive:
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    try:
                        if self.get_state_name() != "operation_enabled":
                            return
                    except Exception:
                        break
                    time.sleep(self.cycle_time if hasattr(self, "cycle_time") else 0.01)

                if hasattr(self, "cycle_time"):
                    time.sleep(self.cycle_time * 100)
        except Exception:
            pass

    def _start_threads(self):
        """Launch processdata, PDO-update, and state-check threads."""
        # Seed output buffer
        tx_init = encode_tx_pdo(0, 0, 0, Mode.TORQUE, 0x0006, None)
        self.master.slaves[self.slave_index].output = tx_init
        print(f"Initialized TX buffer: {len(tx_init)} bytes")

        # Thread 1 — fast processdata (5 ms)
        self._pd_stop_event = threading.Event()
        self._processdata_thread = threading.Thread(
            target=self._processdata_loop, name="HDrive-ProcessData", daemon=False
        )
        self._processdata_thread.start()
        print("ProcessData thread started (5 ms cycle)")

        # Thread 2 — PDO update (cycle_time)
        self._pdo_stop_event = threading.Event()
        self._pdo_thread = threading.Thread(
            target=self._pdo_update_loop, name="HDrive-PDOUpdate", daemon=False
        )
        self._pdo_thread.start()
        print(f"PDO Update thread started ({self.cycle_time * 1000:.1f} ms cycle)")

        # Thread 3 — state check (10 ms)
        self._check_stop_event = threading.Event()
        self._check_thread = threading.Thread(
            target=self._check_loop, name="HDrive-StateCheck", daemon=False
        )
        self._check_thread.start()
        print("State check thread started (10 ms cycle)")

    def _processdata_loop(self):
        """Fast send/receive — 5 ms cycle.  No locks, no processing."""
        while not self._pd_stop_event.is_set():
            if self._reconnecting.is_set():
                time.sleep(0.05)
                continue
            try:
                self.master.send_processdata()
                self._actual_wkc = self.master.receive_processdata(10000)

                if self._actual_wkc != self.master.expected_wkc:
                    self._comm_error_count += 1
                    if self.master.in_op:
                        self.master.do_check_state = True
                else:
                    self._comm_ok_count += 1
            except Exception:
                self._comm_error_count += 1

            time.sleep(0.001)

    def _pdo_update_loop(self):
        """Decode RX, run state machine, encode TX — cycle_time."""
        control_word = 0x0006
        pdo_cycle_count = 0
        _last_reported_errors = 0

        while not self._pdo_stop_event.is_set():
            if self._reconnecting.is_set():
                time.sleep(0.05)
                continue
            try:
                pdo_cycle_count += 1

                if pdo_cycle_count % 20 == 0 and self._comm_error_count > _last_reported_errors:
                    _last_reported_errors = self._comm_error_count
                    print(f"[PDO {pdo_cycle_count}] Comm errors: {self._comm_error_count}")

                rx_raw = bytes(self.master.slaves[self.slave_index].input)
                rx = decode_rx_pdo(rx_raw)
                if rx:
                    state = cia402_state_from_status(rx["status"])

                    if self._manual_controlword is not None:
                        control_word = self._manual_controlword
                    else:
                        control_word = next_control_word(state)

                    self._state.update(rx)
                    self._state["state_name"] = state
                    self._last_wkc = self._comm_ok_count

                target_position = self._target_position
                target_velocity = self._target_velocity
                target_torque = self._target_torque
                mode = self._target_mode
                debug_outputs = self._target_debug_outputs

                tx_raw = encode_tx_pdo(
                    target_position, target_velocity, target_torque,
                    mode, control_word, debug_outputs,
                )
                self.master.slaves[self.slave_index].output = tx_raw
            except Exception:
                pass

            time.sleep(self.cycle_time)

    def _check_loop(self):
        """Monitor slave health and attempt recovery — 10 ms cycle."""
        _consecutive_lost = 0
        _RECONNECT_THRESHOLD = 200  # ~2 seconds of sustained failure

        while not self._check_stop_event.is_set():
            if self._reconnecting.is_set():
                _consecutive_lost = 0
                time.sleep(0.1)
                continue

            try:
                if self.master and self.master.in_op and (
                    (self._actual_wkc < self.master.expected_wkc)
                    or self.master.do_check_state
                ):
                    self.master.do_check_state = False
                    self.master.read_state()

                    all_ok = True
                    for i, slave in enumerate(self.master.slaves):
                        if slave.state != pysoem.OP_STATE:
                            all_ok = False
                            self.master.do_check_state = True
                            self._recover_slave(slave, i)

                    if not self.master.do_check_state:
                        _consecutive_lost = 0
                        print("[STATE CHECK] All slaves resumed OPERATIONAL")
                    elif not all_ok:
                        _consecutive_lost += 1
                else:
                    _consecutive_lost = 0

            except Exception:
                _consecutive_lost += 1

            if (
                _consecutive_lost >= _RECONNECT_THRESHOLD
                and self.auto_reconnect
                and not self._reconnecting.is_set()
            ):
                print(f"[STATE CHECK] Lost contact for "
                      f"{_consecutive_lost * 0.01:.1f}s — triggering reconnect")
                _consecutive_lost = 0
                self._attempt_reconnect()

            time.sleep(0.01)

    @staticmethod
    def _recover_slave(slave, pos):
        """Attempt to recover a slave that left OP state."""
        if slave.state == (pysoem.SAFEOP_STATE + pysoem.STATE_ERROR):
            print(f"[STATE CHECK] Slave {pos} is in SAFE_OP + ERROR, attempting ack...")
            slave.state = pysoem.SAFEOP_STATE + pysoem.STATE_ACK
            slave.write_state()
        elif slave.state == pysoem.SAFEOP_STATE:
            print(f"[STATE CHECK] Slave {pos} is in SAFE_OP, changing to OPERATIONAL...")
            slave.state = pysoem.OP_STATE
            slave.write_state()
        elif slave.state > pysoem.NONE_STATE:
            if slave.reconfig():
                slave.is_lost = False
                print(f"[STATE CHECK] Slave {pos} reconfigured")
        elif not slave.is_lost:
            slave.state_check(pysoem.OP_STATE)
            if slave.state == pysoem.NONE_STATE:
                slave.is_lost = True
                print(f"[STATE CHECK] ERROR: Slave {pos} lost!")

        if slave.is_lost:
            if slave.state == pysoem.NONE_STATE:
                if slave.recover():
                    slave.is_lost = False
                    print(f"[STATE CHECK] Slave {pos} recovered!")
            else:
                slave.is_lost = False
                print(f"[STATE CHECK] Slave {pos} found")
