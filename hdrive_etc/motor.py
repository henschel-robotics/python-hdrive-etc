"""
HDrive EtherCAT SDK — Main motor control class

Provides :class:`HDriveETC`, the high-level API for controlling
Henschel Robotics HDrive servo motors over EtherCAT (via PySOEM).

Can be used standalone (creates its own bus internally) or attached
to a shared :class:`EtherCATBus` for multi-slave setups.
"""

import struct
import threading

import pysoem

from .bus import EtherCATBus
from .exceptions import ConnectionError, CommunicationError
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

    **Standalone mode**: pass ``adapter`` (name/UID string) and
    ``cycle_time_ms`` — a private :class:`EtherCATBus` is created
    automatically.

    **Shared-bus mode**: pass a pre-created :class:`EtherCATBus` via the
    *bus* parameter.  The motor registers itself with the bus and
    participates in the shared PDO cycle.

    Args:
        adapter: Network-adapter name/UID string (standalone mode).
        slave_index: EtherCAT slave index (usually ``0``).
        cycle_time_ms: PDO update cycle time in milliseconds (standalone).
        bus: Optional :class:`EtherCATBus` to share with other slaves.

    Example (standalone)::

        with HDriveETC(adapter=r"\\Device\\NPF_{...}") as motor:
            motor.set_mode(Mode.TORQUE)
            motor.set_torque(200)
            time.sleep(2)
            motor.stop()

    Example (shared bus)::

        bus = EtherCATBus(adapter=r"\\Device\\NPF_{...}", cycle_time_ms=1)
        motor0 = HDriveETC(slave_index=0, bus=bus)
        motor1 = HDriveETC(slave_index=1, bus=bus)
        bus.open()
        # ... control motor0 and motor1 ...
        bus.close()
    """

    def __init__(self, adapter=None, slave_index=0, cycle_time_ms=10,
                 bus=None, pdo_config_path=None):
        self.adapter = adapter
        self.slave_index = slave_index
        self.cycle_time = cycle_time_ms / 1000.0
        self._pdo_config_path = pdo_config_path

        # EtherCAT receive timeout: 2x cycle time (microseconds)
        self.rx_timeout_us = int(cycle_time_ms * 2000)

        # Bus reference
        self._bus = bus
        self._owns_bus = bus is None

        # Legacy attributes for backward compat
        self.master = None

        self._lock = threading.Lock()
        self._state = {}

        # Communication statistics (used when bus is shared)
        self._comm_ok_count = 0
        self._comm_error_count = 0
        self._actual_wkc = 0
        self._last_wkc = 0

        # Cycle timing statistics
        self._cycle_time_min = float("inf")
        self._cycle_time_max = 0.0
        self._cycle_time_sum = 0.0
        self._last_cycle_time = None

        # Auto-reconnect (only meaningful in standalone mode)
        self.auto_reconnect = True
        self._reconnecting = threading.Event()

        # Setpoints (thread-safe via lock)
        self._target_position = 0
        self._target_velocity = 0
        self._target_torque = 0
        self._target_mode = Mode.TORQUE
        self._target_debug_outputs = None
        self._manual_controlword = None

        # PDO loop internal state
        self._control_word = 0x0006

        if bus is not None:
            bus.register_slave(self)

    # ------------------------------------------------------------------
    # Slave-handle interface (called by EtherCATBus)
    # ------------------------------------------------------------------

    def configure(self, pysoem_slave, rx_pdo=None, tx_pdo=None):
        """Called by the bus during config_init to set up PDO mapping."""
        configure_pdo_mapping(pysoem_slave, rx_pdo=rx_pdo, tx_pdo=tx_pdo)

    def seed_tx(self, pysoem_slave):
        """Seed the TX output buffer with safe defaults."""
        tx_init = encode_tx_pdo(0, 0, 0, Mode.TORQUE, 0x0006, None)
        pysoem_slave.output = tx_init

    def pdo_update(self, master, reconnecting):
        """Called every PDO cycle by the bus to decode RX and encode TX."""
        if reconnecting.is_set():
            return

        slave = master.slaves[self.slave_index]

        rx_raw = bytes(slave.input)
        rx = decode_rx_pdo(rx_raw)
        if rx:
            state = cia402_state_from_status(rx["status"])

            if self._manual_controlword is not None:
                self._control_word = self._manual_controlword
            else:
                self._control_word = next_control_word(state)

            self._state.update(rx)
            self._state["state_name"] = state

        target_position = self._target_position
        target_velocity = self._target_velocity
        target_torque = self._target_torque
        mode = self._target_mode
        debug_outputs = self._target_debug_outputs

        tx_raw = encode_tx_pdo(
            target_position, target_velocity, target_torque,
            mode, self._control_word, debug_outputs,
        )
        slave.output = tx_raw

    def on_reconnect(self, master):
        """Called by the bus after a successful reconnect."""
        with self._lock:
            self._target_mode = Mode.STOP
            self._target_position = 0
            self._target_velocity = 0
            self._target_torque = 0
            self._manual_controlword = 0x0006
        print(f"[MOTOR {self.slave_index}] Reconnected — held in STOP")

    def safe_stop(self):
        """Zero setpoints; called by the bus before closing."""
        with self._lock:
            self._target_torque = 0
            self._target_velocity = 0
            self._target_mode = Mode.STOP

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
            if self._owns_bus and self._bus:
                self.disconnect()
            elif self.master:
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

        In standalone mode, creates a private :class:`EtherCATBus` and
        opens it.  In shared-bus mode this is a no-op (call
        ``bus.open()`` instead).

        Raises:
            ConnectionError: If no adapters/slaves are found or the state
                transition fails.
            ConfigurationError: If PDO mapping fails.
        """
        if not self._owns_bus:
            return

        self._bus = EtherCATBus(
            adapter=self.adapter,
            cycle_time_ms=self.cycle_time * 1000,
            pdo_config_path=self._pdo_config_path,
        )
        self._bus.auto_reconnect = self.auto_reconnect
        self._bus.register_slave(self)
        self._bus.open()

        # Expose master for backward compat (SDO access, etc.)
        self.master = self._bus.master

    def disconnect(self):
        """Stop the motor and close the EtherCAT connection.

        In shared-bus mode, only unregisters this slave from the bus.
        """
        if self._owns_bus and self._bus:
            self._bus.close()
            self._bus = None
            self.master = None
            print("Disconnected")
        elif self._bus:
            self.safe_stop()
            self._bus.unregister_slave(self)
            self.master = None

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

        In shared-bus mode the stats come from the bus.

        Returns:
            dict: Keys include ``ok_count``, ``error_count``,
            ``success_rate``, ``cycle_time_actual_ms``, etc.
        """
        bus = self._bus
        if bus and not self._owns_bus:
            ok = bus._comm_ok_count
            err = bus._comm_error_count
        else:
            ok = bus._comm_ok_count if bus else self._comm_ok_count
            err = bus._comm_error_count if bus else self._comm_error_count

        total = ok + err
        success_rate = (ok / total * 100) if total > 0 else 0.0

        cycle_avg = (
            (self._cycle_time_sum / ok) if ok > 0 else 0.0
        )
        jitter = (
            max(self._cycle_time_max - self.cycle_time, self.cycle_time - self._cycle_time_min)
            if ok > 0
            else 0.0
        )

        return {
            "ok_count": ok,
            "error_count": err,
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

    def _get_master(self):
        """Return the active pysoem master, preferring the bus."""
        if self._bus and self._bus.master:
            return self._bus.master
        return self.master

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
        master = self._get_master()
        if not master:
            raise CommunicationError("Not connected")
        try:
            slave = master.slaves[self.slave_index]
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
        master = self._get_master()
        if not master:
            raise CommunicationError("Not connected")
        slave = master.slaves[self.slave_index]
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
        master = self._get_master()
        if not master:
            return None
        try:
            slave = master.slaves[self.slave_index]
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
        master = self._get_master()
        if not master:
            return False
        try:
            slave = master.slaves[self.slave_index]
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
        master = self._get_master()
        if not master:
            raise CommunicationError("Not connected")
        slave = master.slaves[self.slave_index]
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
