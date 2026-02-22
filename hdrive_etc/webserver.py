"""
HDrive EtherCAT Web Server
~~~~~~~~~~~~~~~~~~~~~~~~~~

Web interface for the HDrive servo motor over EtherCAT.

Usage::

    hdrive-web
    hdrive-web --port 8081
    hdrive-web --adapter "\\Device\\NPF_{...}" --slave 0

Then open http://localhost:8081 in your browser.
"""

import argparse
import json
import os
import re
import socket
import struct
import sys
import threading
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import numpy as np

try:
    from . import EtherCATBus, HDriveETC, Mode
    from .protocol import ERROR_CODES, error_message
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from hdrive_etc import EtherCATBus, HDriveETC, Mode
    from hdrive_etc.protocol import ERROR_CODES, error_message

# ---------------------------------------------------------------------------
# Paths (relative to this file, which lives in hdrive_etc/)
# ---------------------------------------------------------------------------

_PKG_DIR = Path(__file__).parent.resolve()
_WEBGUI_DIR = _PKG_DIR / "webgui"
_TEST_SCRIPTS_DIR = _PKG_DIR / "test_scripts"
_RESULTS_DIR = _WEBGUI_DIR / "results"

_RESULTS_DIR.mkdir(exist_ok=True)

# ---------------------------------------------------------------------------
# Network config persistence (stored in pdo_mapping.json → "network" key)
# ---------------------------------------------------------------------------


def _find_pdo_config():
    candidates = [
        _PKG_DIR / "pdo_mapping.json",
        _PKG_DIR.parent / "pdo_mapping.json",
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return None


def _load_net_config(pdo_config_path: str | None) -> dict:
    defaults = {"adapter": None, "slave": 0, "cycle_ms": 0.2}
    try:
        if pdo_config_path and Path(pdo_config_path).exists():
            raw = json.loads(Path(pdo_config_path).read_text(encoding="utf-8"))
            net = raw.get("network", {})
            if "adapter" in net:
                defaults["adapter"] = net["adapter"]
            if "slave" in net:
                defaults["slave"] = int(net["slave"])
            if "cycle_ms" in net:
                defaults["cycle_ms"] = float(net["cycle_ms"])
    except Exception:
        pass
    return defaults


def _save_net_config(pdo_config_path: str | None, adapter: str | None,
                     slave: int, cycle_ms: float):
    if not pdo_config_path:
        return
    try:
        p = Path(pdo_config_path)
        existing = {}
        if p.exists():
            existing = json.loads(p.read_text(encoding="utf-8"))
        existing["network"] = {"adapter": adapter, "slave": slave, "cycle_ms": cycle_ms}
        p.write_text(json.dumps(existing, indent=2) + "\n", encoding="utf-8")
    except Exception as exc:
        print(f"[WARN] Could not save net config: {exc}")


# ---------------------------------------------------------------------------
# Motor Bridge
# ---------------------------------------------------------------------------


class MotorBridge:
    """Thread-safe bridge between the web server and the HDriveETC motor."""

    def __init__(self, adapter=None, slave_index=0, cycle_time_ms=10,
                 pdo_config_path=None):
        self._adapter = adapter
        self._slave_index = slave_index
        self._cycle_time_ms = cycle_time_ms
        self._pdo_config_path = pdo_config_path
        self.motor = None
        self._lock = threading.Lock()
        self._object_cache = {}
        self._connected = False
        self._calibrating = False
        self._max_torque = 1000
        self._device_name = ""
        self._hw_version = ""
        self._fw_version = ""

    def connect(self, adapter=None, slave_index=None, cycle_time_ms=None):
        if self._connected:
            self.disconnect()
        if adapter is not None:
            self._adapter = adapter
        if slave_index is not None:
            self._slave_index = slave_index
        if cycle_time_ms is not None:
            self._cycle_time_ms = cycle_time_ms
        self.motor = HDriveETC(
            adapter=self._adapter,
            slave_index=self._slave_index,
            cycle_time_ms=self._cycle_time_ms,
            pdo_config_path=self._pdo_config_path,
        )
        self.motor.connect()
        self._connected = True
        self.motor.set_debug_outputs([2, 0, 0, 0, 0, 0, 0, 0])
        self.motor.stop()
        self.motor.set_controlword(0x0006)
        max_trq = self.motor.read_sdo(0x6072, 0x00)
        self._max_torque = max_trq if max_trq else 1000
        self._read_device_identity()
        _save_net_config(self._pdo_config_path, self._adapter,
                        self._slave_index, self._cycle_time_ms)
        print(f"[MotorBridge] Connected — {self._device_name} "
              f"hw={self._hw_version} fw={self._fw_version} "
              f"max_torque={self._max_torque}")

    def _read_device_identity(self):
        slave = self.motor.master.slaves[self.motor.slave_index]
        self._device_name = ""
        self._hw_version = ""
        self._fw_version = ""
        for attr, idx in [("_device_name", 0x1008),
                          ("_hw_version", 0x1009),
                          ("_fw_version", 0x100A)]:
            best = ""
            for sz in (64, 32, 16, None):
                try:
                    raw = (slave.sdo_read(idx, 0) if sz is None
                           else slave.sdo_read(idx, 0, sz))
                    if raw:
                        s = raw.decode("utf-8", errors="replace").rstrip("\x00").strip()
                        if len(s) > len(best):
                            best = s
                except Exception:
                    continue
            setattr(self, attr, best)

    def disconnect(self):
        if self._connected and self.motor:
            try:
                self.motor.stop()
                self.motor.set_controlword(0x0006)
            except Exception:
                pass
            self.motor.disconnect()
            self._connected = False
            self._object_cache.clear()
            print("[MotorBridge] Disconnected")

    @property
    def connected(self):
        return self._connected

    # --- Binary telemetry ---

    def get_binary_telemetry(self) -> bytes:
        """Build a 20-float (80-byte) REAL32 telemetry packet."""
        status = self.motor.get_status()
        statusword = float(status.get("status", 0))
        dbg = status.get("debug_values") or (0.0,) * 16

        values = [
            dbg[0], dbg[1], dbg[2], dbg[3], dbg[4],
            dbg[6],
            dbg[10] * 0.1,
            dbg[9], dbg[14], dbg[11], dbg[12],
            dbg[8], dbg[7], dbg[15], dbg[5],
            statusword,
            0.0, 0.0, 0.0, 0.0,
        ]

        if self._calibrating and dbg[11] != -99.0:
            self._calibrating = False
            self.motor.stop()
            self.motor.set_controlword(0x0006)
            print("[MotorBridge] Calibration finished — motor set to stop")

        return struct.pack("<20f", *values)

    # --- Object read/write ---

    def read_object(self, sdo_index, sdo_subindex):
        if sdo_index == 0x99:
            return 0
        try:
            val = self.motor.read_sdo(sdo_index, sdo_subindex)
            return val if val is not None else 0
        except Exception:
            return 0

    def write_object(self, sdo_index, sdo_subindex, value):
        if sdo_index == 0x99:
            self._handle_mode_switch(sdo_subindex, value)
            return

        pdo_demands = {
            (0x6071, 0x00): self.motor.set_torque,
            (0x607A, 0x00): self.motor.set_position,
            (0x6081, 0x00): self.motor.set_velocity,
        }
        key = (sdo_index, sdo_subindex)
        if key in pdo_demands:
            pdo_demands[key](int(value))
            return

        self.motor.write_sdo(sdo_index, sdo_subindex, int(value))
        if sdo_index == 0x6072:
            self.motor.trigger_parameter_calculation()

    def _handle_mode_switch(self, sub_key, value):
        value = int(value)
        if value == 0:
            self.motor.stop()
            self.motor.set_controlword(0x0006)
        elif value == 128:
            self.motor.set_mode(Mode.TORQUE)
            self.motor.clear_controlword()
        elif value == 130:
            self.motor.set_mode(Mode.VELOCITY)
            self.motor.clear_controlword()
        elif value == 133 or value == 129:
            self.motor.set_mode(Mode.POSITION)
            self.motor.clear_controlword()
        elif value == 134:
            self.motor.set_mode(Mode.STEPPER)
            self.motor.clear_controlword()
        elif value == 8:
            self.motor.set_mode(Mode.POSITION)
            self.motor.clear_controlword()
        else:
            self.motor.set_mode(Mode.STOP)
            self.motor.set_controlword(0x0006)

    def read_all_objects(self) -> str:
        """Read all known SDO objects and return as ``"idx,sub,val;..."``."""
        entries = []

        def _sdo(idx, sub):
            try:
                v = self.motor.read_sdo(idx, sub)
                return v if v is not None else 0
            except Exception:
                return 0

        entries.append(f"{0x1008},{0x00},{self._device_name}")
        entries.append(f"{0x1009},{0x00},{self._hw_version}")
        entries.append(f"{0x100A},{0x00},{self._fw_version}")

        for idx, sub in [
            (0x6041, 0x00), (0x6061, 0x00), (0x6064, 0x00), (0x606C, 0x00),
            (0x6077, 0x00), (0x603F, 0x00), (0x6079, 0x00),
            (0x6062, 0x00), (0x606B, 0x00), (0x6071, 0x00),
            (0x607F, 0x00), (0x6072, 0x00), (0x6083, 0x00), (0x6084, 0x00),
            (0x6691, 0x00), (0x6692, 0x00),
            (0x6630, 0x00), (0x6631, 0x00), (0x6632, 0x00),
            (0x6633, 0x00), (0x6634, 0x00), (0x6635, 0x00),
            (0x6640, 0x01), (0x6640, 0x02), (0x6640, 0x03),
            (0x6620, 0x01), (0x6620, 0x02), (0x6620, 0x03),
            (0x6645, 0x01), (0x6645, 0x02), (0x6645, 0x03),
            (0x6645, 0x04), (0x6645, 0x05),
            (0x6660, 0x01), (0x6660, 0x02), (0x6660, 0x03),
            (0x6660, 0x04), (0x6660, 0x05), (0x6660, 0x06),
        ]:
            entries.append(f"{idx},{sub},{_sdo(idx, sub)}")

        return ";".join(entries)


# ---------------------------------------------------------------------------
# Test script runners
# ---------------------------------------------------------------------------

_test_lock = threading.Lock()
_test_abort = threading.Event()


_INSTALL_HINT = (
    "Install the optional test dependencies:  "
    "pip install hdrive-etc[tests]   "
    "(requires matplotlib and scipy)"
)


def _import_test(module_name):
    """Import a test module from the test_scripts/ directory."""
    if str(_TEST_SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(_TEST_SCRIPTS_DIR))
    return __import__(module_name)


def run_step_response(motor, params: dict) -> dict:
    try:
        mod = _import_test("test_step_response")
    except ImportError as e:
        return {"error": f"Missing dependency: {e}\n\n{_INSTALL_HINT}"}

    step_size = int(params.get("step_size", 180))
    record_time = float(params.get("record_time", 2000)) / 1000.0
    torque_limit = int(params.get("torque_limit", 300))
    inertia = int(params.get("inertia", 100))
    damping = int(params.get("damping", 50))
    bw_current = int(params.get("bw_current", 800))
    bw_speed = int(params.get("bw_speed", 0))
    bw_position = int(params.get("bw_position", 20))

    _test_abort.clear()
    with _test_lock:
        try:
            test = mod.StepResponseTest(motor, output_file="")
            test.STEP_SIZE = step_size
            test.CAPTURE_DURATION = record_time
            test.abort_event = _test_abort

            motor.set_mode(Mode.POSITION)
            motor.clear_controlword()
            motor.write_sdo(0x6072, 0x00, torque_limit)
            motor.set_control_bandwidth(bw_current, bw_speed, bw_position)
            motor.set_damping(damping)
            motor.set_rotor_inertia(inertia)
            motor.trigger_parameter_calculation()
            time.sleep(0.2)

            test.capture_step_response()

            if _test_abort.is_set():
                motor.stop()
                motor.set_controlword(0x0006)
                return {"error": "Test aborted"}

            if len(test.actual_positions) == 0:
                return {"error": "No position data captured"}

            metrics = test.analyze_step_response()
            motor.stop()
            motor.set_controlword(0x0006)

            def _safe(v):
                if v is None:
                    return None
                f = float(v)
                return None if (np.isnan(f) or np.isinf(f)) else f

            return {
                "timestamps": [round(t, 6) for t in test.timestamps],
                "target_positions": [round(p, 3) for p in test.target_positions],
                "actual_positions": [round(p, 3) for p in test.actual_positions],
                "velocities": [round(v, 3) for v in test.velocities],
                "metrics": {k: _safe(v) for k, v in (metrics or {}).items()},
            }
        except Exception as exc:
            motor.stop()
            motor.set_controlword(0x0006)
            import traceback
            return {"error": f"Test failed: {exc}\n{traceback.format_exc()[-500:]}"}


def run_bode_plot(motor, params: dict) -> dict:
    try:
        mod = _import_test("test_frequency_response")
    except ImportError as e:
        return {"error": f"Missing dependency: {e}\n\n{_INSTALL_HINT}"}

    start_freq = int(params.get("start_freq", 5))
    stop_freq = int(params.get("stop_freq", 30))
    amplitude = int(params.get("amplitude", 100))
    inertia = int(params.get("inertia", 100))
    damping = int(params.get("damping", 50))
    bw_current = int(params.get("bw_current", 1000))
    bw_speed = int(params.get("bw_speed", 0))
    bw_position = int(params.get("bw_position", 30))

    _test_abort.clear()
    with _test_lock:
        try:
            test = mod.FrequencyResponseTest(motor, output_file="")
            test._start_freq = start_freq
            test._stop_freq = stop_freq
            test.AMPLITUDE = amplitude
            test.abort_event = _test_abort
            test.FREQUENCIES = np.logspace(
                np.log10(start_freq), np.log10(stop_freq), 20
            )

            motor.set_mode(Mode.POSITION)
            motor.clear_controlword()
            motor.write_sdo(0x6072, 0x00, amplitude)
            motor.set_control_bandwidth(bw_current, bw_speed, bw_position)
            motor.set_damping(damping)
            motor.set_rotor_inertia(inertia)
            motor.trigger_parameter_calculation()
            time.sleep(0.2)

            test.run_frequency_sweep()

            if _test_abort.is_set():
                motor.stop()
                motor.set_controlword(0x0006)
                return {"error": "Test aborted"}

            if len(test.results) == 0:
                return {"error": "No frequency data captured"}

            analysis = test.analyze_results()
            motor.stop()
            motor.set_controlword(0x0006)

            def _safe(v):
                if v is None:
                    return None
                f = float(v)
                return None if (np.isnan(f) or np.isinf(f)) else f

            return {
                "frequencies": [round(r["frequency"], 4) for r in test.results],
                "gains_db": [round(r["gain_db"], 3) for r in test.results],
                "phases_deg": [round(r["phase_deg"], 3) for r in test.results],
                "analysis": {k: _safe(v) for k, v in (analysis or {}).items()},
            }
        except Exception as exc:
            motor.stop()
            motor.set_controlword(0x0006)
            import traceback
            return {"error": f"Test failed: {exc}\n{traceback.format_exc()[-500:]}"}


def run_inertia_test(motor, params: dict) -> dict:
    try:
        mod = _import_test("test_inertia_identification")
    except ImportError as e:
        return {"error": f"Missing dependency: {e}\n\n{_INSTALL_HINT}"}

    torque_mNm = int(params.get("torque_mNm", 200))
    duration_ms = float(params.get("duration_ms", 500))
    torque_limit = int(params.get("torque_limit", 600))

    _test_abort.clear()
    with _test_lock:
        try:
            test = mod.InertiaIdentificationTest(motor, output_file="")
            test.TORQUE_MNM = torque_mNm
            test.CAPTURE_DURATION = duration_ms / 1000.0
            test.abort_event = _test_abort

            motor.set_mode(Mode.TORQUE)
            motor.clear_controlword()
            motor.write_sdo(0x6072, 0x00, torque_limit)
            motor.trigger_parameter_calculation()
            time.sleep(0.2)

            test.capture_acceleration()

            if _test_abort.is_set():
                motor.stop()
                motor.set_controlword(0x0006)
                return {"error": "Test aborted"}

            if len(test.timestamps) < 20:
                motor.stop()
                motor.set_controlword(0x0006)
                return {"error": "Insufficient data captured"}

            metrics = test.analyze_inertia()
            motor.stop()
            motor.set_controlword(0x0006)

            if metrics is None:
                return {"error": "Inertia analysis failed (motor may not have accelerated)"}

            return {
                "timestamps": [round(t, 6) for t in test.timestamps],
                "velocities": [round(v, 3) for v in test.velocities],
                "metrics": metrics,
            }
        except Exception as exc:
            motor.stop()
            motor.set_controlword(0x0006)
            import traceback
            return {"error": f"Test failed: {exc}\n{traceback.format_exc()[-500:]}"}


def run_network_test(motor, params: dict) -> dict:
    try:
        mod = _import_test("test_network_latency")
    except ImportError as e:
        return {"error": f"Missing dependency: {e}\n\n{_INSTALL_HINT}"}

    num_samples = int(params.get("num_samples", 200))

    _test_abort.clear()
    with _test_lock:
        try:
            test = mod.NetworkLatencyTest(motor)
            test.NUM_SAMPLES = num_samples
            test.abort_event = _test_abort

            test.run_measurement()

            if _test_abort.is_set():
                return {"error": "Test aborted"}

            if len(test.latencies_ms) < 10:
                return {"error": "Insufficient data captured"}

            metrics = test.analyze()
            if metrics is None:
                return {"error": "Analysis failed"}

            return metrics
        except Exception as exc:
            import traceback
            return {"error": f"Test failed: {exc}\n{traceback.format_exc()[-500:]}"}


# ---------------------------------------------------------------------------
# HTTP Request Handler
# ---------------------------------------------------------------------------

motor_bridge: MotorBridge = None

_SKIP_ADAPTERS = (
    "wi-fi", "wifi", "bluetooth", "loopback",
    "wi-fi direct", "wan miniport", "hyper-v",
)

_SKIP_LINUX_IFACES = ("lo", "wlan", "wlp", "docker", "br-", "veth", "virbr")


class HDriveRequestHandler(SimpleHTTPRequestHandler):
    """HTTP handler that serves the web GUI and proxies API calls to the motor."""

    def setup(self):
        super().setup()
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(_WEBGUI_DIR), **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        raw_query = parsed.query
        query = parse_qs(parsed.query)

        if path == "/api/adapters":
            try:
                adapters = HDriveETC.list_adapters()
                result = []
                for a in adapters:
                    name = a.name.decode("utf-8", errors="replace") if isinstance(a.name, bytes) else str(a.name)
                    desc = a.desc.decode("utf-8", errors="replace") if isinstance(a.desc, bytes) else str(a.desc)
                    if any(s in desc.lower() for s in _SKIP_ADAPTERS):
                        continue
                    if any(name.startswith(p) for p in _SKIP_LINUX_IFACES):
                        continue
                    result.append({"name": name, "desc": desc})
                self._send_json({
                    "adapters": result,
                    "current": motor_bridge._adapter,
                    "connected": motor_bridge.connected,
                })
            except Exception as exc:
                self._send_json({"error": str(exc)})
            return

        if path == "/api/connect":
            if motor_bridge.connected:
                self._send_json({"error": "Already connected. Disconnect first."})
                return
            try:
                adapter = query.get("adapter", [motor_bridge._adapter or ""])[0]
                slave = int(query.get("slave", [str(motor_bridge._slave_index)])[0])
                cycle = float(query.get("cycle", [str(motor_bridge._cycle_time_ms)])[0])

                try:
                    found = EtherCATBus.discover(
                        adapter=adapter or None,
                        pdo_config_path=motor_bridge._pdo_config_path,
                    )
                    target = next((s for s in found if s["index"] == slave), None)
                    if target and "hdrive" not in (target.get("device_name") or target.get("name") or "").lower():
                        self._send_json({"error": f"Slave {slave} ({target.get('device_name') or target.get('name')}) is not an HDrive motor. Only HDrive devices are supported."})
                        return
                except Exception:
                    pass

                motor_bridge.connect(adapter=adapter or None,
                                     slave_index=slave, cycle_time_ms=cycle)
                self._send_json({
                    "ok": True,
                    "device": motor_bridge._device_name,
                    "hw": motor_bridge._hw_version,
                    "fw": motor_bridge._fw_version,
                })
            except Exception as exc:
                self._send_json({"error": str(exc)})
            return

        if path == "/api/disconnect":
            motor_bridge.disconnect()
            self._send_json({"ok": True})
            return

        if path == "/api/connection":
            self._send_json({
                "connected": motor_bridge.connected,
                "adapter": motor_bridge._adapter,
                "slave": motor_bridge._slave_index,
                "cycle": motor_bridge._cycle_time_ms,
                "device": motor_bridge._device_name if motor_bridge.connected else "",
                "hw": motor_bridge._hw_version if motor_bridge.connected else "",
                "fw": motor_bridge._fw_version if motor_bridge.connected else "",
            })
            return

        if path == "/api/discover":
            adapter = query.get("adapter", [motor_bridge._adapter or ""])[0]
            if motor_bridge.connected:
                self._send_json({"error": "Disconnect first — discovery needs exclusive adapter access."})
                return
            try:
                slaves = EtherCATBus.discover(
                    adapter=adapter or None,
                    pdo_config_path=motor_bridge._pdo_config_path,
                )
                for s in slaves:
                    s["is_hdrive"] = "hdrive" in (s.get("device_name") or s.get("name") or "").lower()
                self._send_json({"slaves": slaves, "adapter": adapter})
            except Exception as exc:
                self._send_json({"error": str(exc)})
            return

        if path == "/api/pdo_config":
            cfg_path = motor_bridge._pdo_config_path
            if cfg_path and Path(cfg_path).exists():
                try:
                    raw = json.loads(Path(cfg_path).read_text(encoding="utf-8"))
                    self._send_json(raw)
                except Exception as exc:
                    self._send_json({"error": str(exc)})
            else:
                self._send_json({"default": {}, "slaves": {}})
            return

        # Guard: motor-dependent endpoints need a connection
        if not motor_bridge.connected:
            if path.startswith("/api/"):
                self._send_json({"error": "Not connected. Use Network Config to connect first."})
                return
            if path == "/getData.cgi":
                if raw_query == "bin":
                    self._send_binary(b'\x00' * 80)
                else:
                    self._send_text("not connected")
                return

        if path == "/getData.cgi" and raw_query == "bin":
            self._send_binary(motor_bridge.get_binary_telemetry())
            return

        if path == "/api/telemetry-stream":
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    self.wfile.write(motor_bridge.get_binary_telemetry())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError, OSError):
                pass
            return

        if path == "/getData.cgi" and "obj" in query:
            try:
                index = int(query["obj"][0], 16)
                subindex = int(query.get("sub", ["0"])[0], 16)
                value = motor_bridge.read_object(index, subindex)
                self._send_text(str(value))
            except Exception as exc:
                self._send_text(f"error: {exc}", 500)
            return

        if path == "/getData.cgi" and raw_query == "objL":
            try:
                data = motor_bridge.read_all_objects()
                self._send_text(data)
            except Exception as exc:
                self._send_text(f"error: {exc}", 500)
            return

        if path == "/getData.cgi" and raw_query == "nam":
            self._send_text("HDrive EtherCAT")
            return

        if path == "/api/status":
            self._send_json(motor_bridge.motor.get_status())
            return

        if path == "/api/comm_stats":
            self._send_json(motor_bridge.motor.get_comm_stats())
            return

        if path == "/api/calibrate":
            motor_bridge._calibrating = True
            motor_bridge.motor.set_mode(Mode.CALIBRATION)
            motor_bridge.motor.clear_controlword()
            self._send_json({"ok": True})
            return

        if path == "/api/abort_test":
            _test_abort.set()
            motor_bridge.motor.stop()
            self._send_json({"ok": True})
            return

        if path == "/api/run_step_response":
            try:
                params = {k: v[0] for k, v in query.items()}
                self._send_json(run_step_response(motor_bridge.motor, params))
            except Exception as exc:
                self._send_json({"error": f"Server error: {exc}"})
            return

        if path == "/api/run_bode_plot":
            try:
                params = {k: v[0] for k, v in query.items()}
                self._send_json(run_bode_plot(motor_bridge.motor, params))
            except Exception as exc:
                self._send_json({"error": f"Server error: {exc}"})
            return

        if path == "/api/run_inertia_test":
            try:
                params = {k: v[0] for k, v in query.items()}
                self._send_json(run_inertia_test(motor_bridge.motor, params))
            except Exception as exc:
                self._send_json({"error": f"Server error: {exc}"})
            return

        if path == "/api/run_network_test":
            try:
                params = {k: v[0] for k, v in query.items()}
                self._send_json(run_network_test(motor_bridge.motor, params))
            except Exception as exc:
                self._send_json({"error": f"Server error: {exc}"})
            return

        if path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/pdo_config":
            cfg_path = motor_bridge._pdo_config_path
            if not cfg_path:
                self._send_json({"error": "No PDO config path configured"})
                return
            try:
                content_length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(content_length).decode("utf-8")
                new_data = json.loads(body)

                existing = {}
                p = Path(cfg_path)
                if p.exists():
                    existing = json.loads(p.read_text(encoding="utf-8"))

                if "slaves" not in existing:
                    existing["slaves"] = {}
                for k, v in new_data.get("slaves", {}).items():
                    existing["slaves"][k] = v

                p.write_text(
                    json.dumps(existing, indent=2) + "\n", encoding="utf-8"
                )
                self._send_json({"ok": True})
            except Exception as exc:
                self._send_json({"error": str(exc)})
            return

        if not motor_bridge.connected:
            self._send_json({"error": "Not connected. Use Network Config to connect first."})
            return

        if path == "/writeTicket":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length).decode("utf-8", errors="replace")

            if "<objWrite" in body:
                self._handle_obj_write(body)
                return
            if "<system" in body:
                self._handle_system_command(body)
                return
            self._send_text("ok")
            return

        self._send_text("not found", 404)

    def _handle_obj_write(self, body):
        match = re.search(
            r'a="([\da-fA-F]+)"\s*b="([\da-fA-F]+)"\s*c="(-?[\d.]+)"', body
        )
        if match:
            index = int(match.group(1), 16)
            subindex = int(match.group(2), 16)
            value = int(float(match.group(3)))
            try:
                motor_bridge.write_object(index, subindex, value)
                self._send_text("ok")
            except Exception as exc:
                self._send_text(f"error: {exc}", 500)
        else:
            self._send_text("parse error", 400)

    def _handle_system_command(self, body):
        match = re.search(r'mode="(\d+)"', body)
        if match:
            mode = int(match.group(1))
            if mode == 4:
                try:
                    motor_bridge.motor.write_sdo(0x6637, 0x00, 2)
                    self._send_text("ok")
                except Exception as exc:
                    self._send_text(f"error: {exc}", 500)
            else:
                self._send_text("ok")
        else:
            self._send_text("ok")

    # --- Response helpers ---

    def _send_text(self, text, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(text.encode("utf-8"))

    def _send_binary(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(obj).encode("utf-8"))

    def log_message(self, format, *args):
        msg = str(args)
        if "/getData.cgi" in msg or "/api/telemetry-stream" in msg:
            return
        super().log_message(format, *args)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global motor_bridge

    parser = argparse.ArgumentParser(
        description="HDrive EtherCAT Web Server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Open http://localhost:<port> in your browser after starting.",
    )
    parser.add_argument("--adapter", type=str, default=None,
                        help="EtherCAT network adapter name/UID")
    parser.add_argument("--slave", type=int, default=None,
                        help="EtherCAT slave index")
    parser.add_argument("--cycle", type=float, default=None,
                        help="PDO cycle time in ms")
    parser.add_argument("--port", type=int, default=8081,
                        help="HTTP server port (default: 8081)")
    parser.add_argument("--pdo-config", type=str, default=None,
                        help="Path to pdo_mapping.json (auto-detected if not set)")
    parser.add_argument("--list-adapters", action="store_true",
                        help="List available network adapters and exit")
    args = parser.parse_args()

    if args.list_adapters:
        print("Available network adapters:")
        for adapter in HDriveETC.list_adapters():
            name = adapter.name.decode("utf-8", errors="replace") if isinstance(adapter.name, bytes) else str(adapter.name)
            desc = adapter.desc.decode("utf-8", errors="replace") if isinstance(adapter.desc, bytes) else str(adapter.desc)
            print(f"  {desc}  ->  {name}")
        return

    pdo_path = args.pdo_config or _find_pdo_config()
    if not pdo_path:
        pdo_path = str(_PKG_DIR.parent / "pdo_mapping.json")
        Path(pdo_path).write_text(
            '{"network": {}, "default": {}, "slaves": {}}\n',
            encoding="utf-8",
        )
        print(f"Created default PDO config: {pdo_path}")
    else:
        print(f"PDO config: {pdo_path}")

    saved = _load_net_config(pdo_path)
    adapter = args.adapter if args.adapter is not None else saved["adapter"]
    slave = args.slave if args.slave is not None else saved["slave"]
    cycle = args.cycle if args.cycle is not None else saved["cycle_ms"]

    motor_bridge = MotorBridge(
        adapter=adapter,
        slave_index=slave,
        cycle_time_ms=cycle,
        pdo_config_path=pdo_path,
    )

    print(f"Connecting to HDrive on adapter {adapter}, "
          f"slave {slave}, cycle {cycle} ms ...")
    try:
        motor_bridge.connect()
    except Exception as exc:
        print(f"[WARN] Initial connection failed: {exc}")
        print("  -> The web GUI will start anyway. Use the Network Config tab to connect.\n")
        try:
            print("Available adapters:")
            for i, a in enumerate(HDriveETC.list_adapters()):
                print(f"  [{i}] {a.name} — {a.desc}")
        except Exception:
            pass

    server = ThreadingHTTPServer(("0.0.0.0", args.port), HDriveRequestHandler)
    print(f"\nHDrive Web GUI running at http://localhost:{args.port}")
    print("Press Ctrl+C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down ...")
    finally:
        motor_bridge.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
