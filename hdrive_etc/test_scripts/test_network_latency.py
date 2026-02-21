"""
EtherCAT Network Latency Test

Measures the round-trip SDO read latency between the host and the
motor drive over the EtherCAT bus.  Performs many consecutive SDO
reads and records timing for each, then computes statistics (min,
max, mean, median, p95, p99, jitter) and returns the raw samples
for histogram plotting.

Run as administrator: ADAPTER=\\Device\\NPF_{...} python test_network_latency.py
"""

import os
import sys
import time
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from hdrive_etc import HDriveETC, Mode


class NetworkLatencyTest:
    """Measure EtherCAT SDO round-trip latency."""

    NUM_SAMPLES = 200
    SDO_INDEX = 0x6041      # statusword — always readable
    SDO_SUBINDEX = 0x00

    def __init__(self, motor_or_adapter):
        if isinstance(motor_or_adapter, str):
            self.adapter = motor_or_adapter
            self.motor = None
            self._external_motor = False
        elif isinstance(motor_or_adapter, HDriveETC):
            self.adapter = None
            self.motor = motor_or_adapter
            self._external_motor = True
        else:
            self.adapter = None
            self.motor = None
            self._external_motor = False

        self.latencies_ms = []
        self.abort_event = None

    def setup(self) -> bool:
        if self._external_motor:
            return True
        try:
            self.motor = HDriveETC(
                adapter=self.adapter,
                slave_index=0,
                cycle_time_ms=0.02,
            )
            self.motor.connect()
            time.sleep(0.5)
            return True
        except Exception as exc:
            print(f"[ERROR] Setup failed: {exc}")
            return False

    def teardown(self):
        if self.motor and not self._external_motor:
            try:
                self.motor.disconnect()
            except Exception:
                pass

    def run_measurement(self):
        """Perform NUM_SAMPLES SDO reads and record round-trip time."""
        self.latencies_ms = []
        errors = 0

        for i in range(self.NUM_SAMPLES):
            if self.abort_event and self.abort_event.is_set():
                print("[ABORT] Network test aborted")
                return

            t0 = time.perf_counter()
            try:
                self.motor.read_sdo(self.SDO_INDEX, self.SDO_SUBINDEX)
            except Exception:
                errors += 1
                continue
            t1 = time.perf_counter()
            self.latencies_ms.append((t1 - t0) * 1000.0)

        if errors:
            print(f"[WARN] {errors}/{self.NUM_SAMPLES} reads failed")

    def analyze(self) -> dict | None:
        if len(self.latencies_ms) < 10:
            return None

        arr = np.array(self.latencies_ms)

        return {
            "samples": [round(v, 3) for v in self.latencies_ms],
            "count": len(arr),
            "errors": self.NUM_SAMPLES - len(arr),
            "min_ms": float(round(np.min(arr), 3)),
            "max_ms": float(round(np.max(arr), 3)),
            "mean_ms": float(round(np.mean(arr), 3)),
            "median_ms": float(round(np.median(arr), 3)),
            "std_ms": float(round(np.std(arr), 3)),
            "p95_ms": float(round(np.percentile(arr, 95), 3)),
            "p99_ms": float(round(np.percentile(arr, 99), 3)),
        }

    def run_all_tests(self) -> bool:
        if not self.setup():
            return False
        try:
            self.run_measurement()
            if not self.latencies_ms:
                print("[ERROR] No data captured")
                return False
            metrics = self.analyze()
            if metrics is None:
                print("[ERROR] Analysis failed")
                return False
            print(f"Samples:  {metrics['count']}")
            print(f"Mean:     {metrics['mean_ms']:.3f} ms")
            print(f"Median:   {metrics['median_ms']:.3f} ms")
            print(f"Min/Max:  {metrics['min_ms']:.3f} / {metrics['max_ms']:.3f} ms")
            print(f"Std dev:  {metrics['std_ms']:.3f} ms")
            print(f"P95:      {metrics['p95_ms']:.3f} ms")
            print(f"P99:      {metrics['p99_ms']:.3f} ms")
            return True
        except Exception as exc:
            print(f"[ERROR] Test failed: {exc}")
            return False
        finally:
            self.teardown()


if __name__ == "__main__":
    adapter = os.environ.get("ADAPTER", None)
    tester = NetworkLatencyTest(adapter)
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)
