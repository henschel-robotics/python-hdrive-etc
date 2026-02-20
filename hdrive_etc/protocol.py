"""
HDrive EtherCAT SDK — CiA 402 protocol and PDO encoding/decoding

This module contains all low-level EtherCAT / CiA 402 protocol logic:
- PDO mapping configuration
- TX/RX PDO encoding and decoding
- CiA 402 state machine
- Error code definitions
"""

import struct


# ---------------------------------------------------------------------------
# HDrive error codes (from firmware Error.h)
# ---------------------------------------------------------------------------

ERROR_CODES = {
    0: "No error",
    1: "Program error - Null pointer",
    15: "Position overflow/underflow",
    16: "Over temperature",
    17: "Under voltage",
    18: "Over voltage",
    19: "Over speed",
    20: "Positive software position limit",
    21: "Negative software position limit",
    22: "Negative limit switch triggered",
    23: "Positive limit switch triggered",
    25: "Limit switch timeout",
    26: "Position sensor error",
    27: "Power stage error",
    28: "Watchdog timeout",
    30: "SPI position sensor error",
    31: "Calibration error",
    32: "Wrong ticket format",
    33: "Configuration error",
    34: "IP configuration error",
    35: "Configuration file wrong format",
    36: "Object not found in dictionary",
    37: "Hardware not compatible",
    38: "EtherCAT connection interrupted",
    40: "CAN special command not found",
    50: "Limit switch minimum distance to end switch",
    51: "Motor mode not existing",
    52: "Wrong argument count in ticket",
    53: "Null pointer error",
}


def error_message(code):
    """Return human-readable error message for an error code.

    Args:
        code: Error code integer (0 = no error), or ``None``.

    Returns:
        str: Descriptive message.
    """
    if code is None:
        return "Cannot read error code"
    return ERROR_CODES.get(code, f"Unknown error code: {code}")


# ---------------------------------------------------------------------------
# PDO mapping
# ---------------------------------------------------------------------------

def configure_pdo_mapping(slave):
    """Write SDO objects to configure PDO mapping on *slave*.

    Mapping layout:
        - 0x1600 (RxPDO): Motor control  — 16 bytes
        - 0x1605 (RxPDO): Debug outputs   — 32 bytes  (8x INT32, 0x6710-0x6717)
        - 0x1A00 (TxPDO): Motor status    — 18 bytes
        - 0x1A05 (TxPDO): Debug values    — 64 bytes  (16x REAL32, 0x6700-0x670F)
    """

    def _write_u8(index, subindex, value):
        slave.sdo_write(index, subindex, struct.pack("<B", value))

    def _write_u16(index, subindex, value):
        slave.sdo_write(index, subindex, struct.pack("<H", value))

    # Clear PDO assignments
    _write_u8(0x1C12, 0x00, 0)   # clear SM2 (RxPDO)
    _write_u8(0x1C13, 0x00, 0)   # clear SM3 (TxPDO)

    # Configure RxPDO (master -> slave)
    _write_u16(0x1C12, 0x01, 0x1600)  # Motor control
    _write_u16(0x1C12, 0x02, 0x1605)  # Debug outputs
    _write_u8(0x1C12, 0x00, 2)        # Enable 2 PDOs

    # Configure TxPDO (slave -> master)
    _write_u16(0x1C13, 0x01, 0x1A00)  # Motor status
    _write_u16(0x1C13, 0x02, 0x1A05)  # Debug values
    _write_u8(0x1C13, 0x00, 2)        # Enable 2 PDOs


# ---------------------------------------------------------------------------
# PDO encode / decode
# ---------------------------------------------------------------------------

def encode_tx_pdo(target_position, target_velocity, target_torque,
                  mode, control_word, debug_outputs=None):
    """Encode the outgoing (master -> slave) PDO frame.

    Layout:
        0x1600 — 16 bytes: position(i32), velocity(i32), torque(i16),
                           mode(i8), controlword(u16), pad(u8), pad(u16)
        0x1605 — 32 bytes: 8x INT32 debug outputs

    Returns:
        bytes: 48-byte PDO frame.
    """
    pdo_1600 = struct.pack(
        "<ii h b H B H",
        target_position,
        target_velocity,
        target_torque,
        mode,
        control_word,
        0,  # padding
        0,  # padding
    )

    if debug_outputs and len(debug_outputs) == 8:
        pdo_1605 = struct.pack("<8i", *debug_outputs)
    else:
        pdo_1605 = struct.pack("<8i", 0, 0, 0, 0, 0, 0, 0, 0)

    return pdo_1600 + pdo_1605


def decode_rx_pdo(data):
    """Decode the incoming (slave -> master) PDO frame.

    Layout:
        0x1A00 — 18 bytes: position(i32), velocity(i32), torque(i16),
                           statusword(u16), mode_display(i8), …
        0x1A05 — 64 bytes: 16x REAL32 debug values

    Returns:
        dict or None: Decoded fields, or ``None`` if *data* is too short.
    """
    fmt_motor = "<ii hH b B B B H"
    if len(data) < struct.calcsize(fmt_motor):
        return None

    (position, velocity, torque, status, mode_display,
     _obj_6691, _obj_6692, _pad8, _pad16) = struct.unpack_from(fmt_motor, data, 0)

    result = {
        "position": position * 0.1,
        "velocity": velocity,
        "torque": torque,
        "status": status,
        "mode_display": mode_display,
    }

    # Debug values (optional 64-byte block)
    offset = struct.calcsize(fmt_motor)
    if len(data) >= offset + 64:
        result["debug_values"] = struct.unpack_from("<16f", data, offset)

    return result


# ---------------------------------------------------------------------------
# CiA 402 state machine
# ---------------------------------------------------------------------------

def cia402_state_from_status(status_word):
    """Decode the CiA 402 state name from a statusword value.

    Returns:
        str: One of ``"not_ready"``, ``"switch_on_disabled"``,
        ``"ready_to_switch_on"``, ``"switched_on"``,
        ``"operation_enabled"``, ``"quick_stop_active"``,
        ``"fault_reaction"``, ``"fault"``, or ``"unknown"``.
    """
    masked = status_word & 0x006F
    states = {
        0x0000: "not_ready",
        0x0040: "switch_on_disabled",
        0x0021: "ready_to_switch_on",
        0x0023: "switched_on",
        0x0027: "operation_enabled",
        0x0007: "quick_stop_active",
        0x000F: "fault_reaction",
        0x0008: "fault",
    }
    return states.get(masked, "unknown")


def next_control_word(state):
    """Return the next controlword to advance the CiA 402 state machine.

    Sequence: ``0x0006`` (shutdown) -> ``0x0007`` (switch on) ->
    ``0x000F`` (enable operation).
    """
    if state in ("not_ready", "switch_on_disabled"):
        return 0x0006
    if state == "ready_to_switch_on":
        return 0x0007
    if state in ("switched_on", "operation_enabled"):
        return 0x000F
    return 0x0006
