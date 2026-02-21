"""
Motor Step Response Test

Tests motor position control step response:
- Sends step position commands in position mode
- Captures demanded (target) and actual position over time
- Calculates position error and control metrics
- Analyzes settling time, overshoot, rise time
- Performs FFT analysis on step response
- Plots time-domain response and frequency spectrum
- Saves comprehensive analysis plots as PNG

Run as administrator: ADAPTER=\\Device\\NPF_{...} python test_step_response.py
"""

import os
import sys
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from hdrive_etc import HDriveETC, Mode

class StepResponseTest:
    """Test motor position control step response"""
    
    # Test configuration — overridable via environment variables
    STEP_SIZE = int(os.environ.get("STEP_SIZE", 10000))
    CAPTURE_DURATION = float(os.environ.get("CAPTURE_DURATION", 1.0))
    SAMPLE_INTERVAL = 0.001  # seconds - time between samples (1000 Hz)
    SETTLING_THRESHOLD = 50  # deg - position error threshold for settling
    
    def __init__(self, motor_or_adapter, output_file: str = None):
        """Initialize test with either an existing motor or an adapter name.
        
        Args:
            motor_or_adapter: Either an HDriveETC instance or an adapter name string.
            output_file: Path to save the plot image.
        """
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
        
        self.output_file = output_file
        self.timestamps = []
        self.target_positions = []
        self.actual_positions = []
        self.position_errors = []
        self.velocities = []
        
    def setup(self) -> bool:
        """Initialize motor connection (if not using external motor)"""
        print("\n" + "="*60)
        print("Motor Step Response Test")
        print("="*60)
        
        # Skip connection if using external motor
        if self._external_motor:
            print("  Using existing motor connection")
            print(f"  Step size: {self.STEP_SIZE} deg")
            return True
        
        try:
            self.motor = HDriveETC(adapter=self.adapter, slave_index=0, cycle_time_ms=0.02)
            print("  Connecting to motor...")
            self.motor.connect()
            time.sleep(0.5)
            
            # Set to position mode
            print("  Setting motor to POSITION mode")
            self.motor.set_mode(Mode.POSITION)
            bw_current  = int(os.environ.get("BW_CURRENT", 800))
            bw_speed    = int(os.environ.get("BW_SPEED", 0))
            bw_position = int(os.environ.get("BW_POSITION", 20))
            self.motor.set_control_bandwidth(bw_current, bw_speed, bw_position)
            print(f"  Control bandwidth: current={bw_current} Hz, speed={bw_speed} Hz, position={bw_position} Hz")
            self.motor.set_damping(int(os.environ.get("MOTOR_DAMPING", 200)))
            self.motor.set_rotor_inertia(int(os.environ.get("ROTOR_INERTIA", 200)))
            self.motor.trigger_parameter_calculation()

            time.sleep(0.5)
            
            # Get initial position
            initial_pos = self.motor.get_position()
            print(f"  Initial position: {initial_pos} deg")
            print(f"  Step size: {self.STEP_SIZE} deg")
            
            print("  [OK] Motor ready in POSITION mode")
            return True

        except Exception as e:
            print(f"  [ERROR] Setup failed: {e}")
            return False
    
    def teardown(self):
        """Clean up motor connection (only if we created it)"""
        if self.motor and not self._external_motor:
            try:
                print("\n  Stopping motor...")
                self.motor.set_mode(Mode.STOP)
                time.sleep(0.1)
                self.motor.disconnect()
                print("  [OK] Motor connection closed")
            except Exception as e:
                print(f"  [ERROR] Teardown error: {e}")
    
    def capture_step_response(self):
        """Capture step response data"""
        print("\n" + "-"*60)
        print(f"Capturing Step Response ({self.CAPTURE_DURATION}s)")
        print("-"*60)
        
        # Clear data arrays
        self.timestamps = []
        self.target_positions = []
        self.actual_positions = []
        self.position_errors = []
        
        # Get initial position
        initial_pos = self.motor.get_position()
        target_pos = initial_pos + self.STEP_SIZE
        
        print(f"  Initial: {initial_pos}")
        print(f"  Target:  {target_pos}")
        print(f"  Step:    {self.STEP_SIZE}")
        print()
        
        # Send step command
        print("  Sending step command...")
        self.motor.set_position(target_pos)
        
        # Start capturing
        start_time = time.time()
        sample_count = 0
        
        print("  Progress: ", end="", flush=True)
        last_progress = -1
        
        while (time.time() - start_time) < self.CAPTURE_DURATION:
            if getattr(self, 'abort_event', None) and self.abort_event.is_set():
                print("\n  [ABORT] Test aborted by user")
                return
            
            elapsed = time.time() - start_time
            
            actual_pos = self.motor.get_position()
            error = target_pos - actual_pos
            
            # Store data
            self.timestamps.append(elapsed)
            self.target_positions.append(target_pos)
            self.actual_positions.append(actual_pos)
            self.position_errors.append(error)
            self.velocities.append(self.motor.get_velocity())
            
            sample_count += 1
            
            # Update progress bar
            progress = int((elapsed / self.CAPTURE_DURATION) * 100)
            if progress != last_progress and progress % 10 == 0:
                print(f"{progress}% ", end="", flush=True)
                last_progress = progress
            
            # Wait for next sample
            time.sleep(self.SAMPLE_INTERVAL)
        
        print("100% [OK]")
        print(f"  Captured {sample_count} samples")
        print(f"  Sample rate: {sample_count / self.CAPTURE_DURATION:.1f} Hz")
    
    def analyze_step_response(self):
        """Analyze step response characteristics"""
        print("\n" + "-"*60)
        print("Step Response Analysis")
        print("-"*60)
        
        if not self.actual_positions or len(self.actual_positions) < 10:
            print("  [ERROR] Insufficient data")
            return None
        
        target = self.target_positions[0]
        initial = self.actual_positions[0]
        actual_array = np.array(self.actual_positions)
        error_array = np.array(self.position_errors)
        time_array = np.array(self.timestamps)
        
        # Calculate step size
        step_size = target - initial
        
        # Rise time (10% to 90% of step)
        pos_10 = initial + 0.1 * step_size
        pos_90 = initial + 0.9 * step_size
        
        idx_10 = np.where(actual_array >= pos_10)[0]
        idx_90 = np.where(actual_array >= pos_90)[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            rise_time = time_array[idx_90[0]] - time_array[idx_10[0]]
        else:
            rise_time = None
        
        # Settling time (within threshold)
        settled_mask = np.abs(error_array) <= self.SETTLING_THRESHOLD
        if np.any(settled_mask):
            # Find first index where it stays settled
            for i in range(len(settled_mask) - 10):
                if np.all(settled_mask[i:i+10]):  # Settled for at least 10 samples
                    settling_time = time_array[i]
                    break
            else:
                settling_time = None
        else:
            settling_time = None
        
        # Overshoot
        if step_size > 0:
            max_pos = np.max(actual_array)
            overshoot = max_pos - target
            overshoot_percent = (overshoot / step_size * 100) if step_size != 0 else 0
        else:
            min_pos = np.min(actual_array)
            overshoot = target - min_pos
            overshoot_percent = (overshoot / abs(step_size) * 100) if step_size != 0 else 0
        
        # Final position and error
        final_pos = actual_array[-1]
        final_error = error_array[-1]
        steady_state_error = abs(final_error)
        
        # RMS error
        rms_error = np.sqrt(np.mean(error_array**2))
        
        metrics = {
            'initial': initial,
            'target': target,
            'final': final_pos,
            'step_size': step_size,
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot': overshoot,
            'overshoot_percent': overshoot_percent,
            'steady_state_error': steady_state_error,
            'rms_error': rms_error
        }
        
        return metrics
    
    def print_metrics(self, metrics: dict):
        """Print step response metrics"""
        if not metrics:
            return
        
        print(f"\nPosition:")
        print(f"  Initial:      {metrics['initial']:8.0f} deg")
        print(f"  Target:       {metrics['target']:8.0f} deg")
        print(f"  Final:        {metrics['final']:8.0f} deg")
        print(f"  Step Size:    {metrics['step_size']:8.0f} deg")
        
        print(f"\nTiming:")
        if metrics['rise_time'] is not None:
            print(f"  Rise Time (10-90%):  {metrics['rise_time']*1000:.1f} ms")
        else:
            print(f"  Rise Time (10-90%):  N/A (not reached 90%)")
        
        if metrics['settling_time'] is not None:
            print(f"  Settling Time (±{self.SETTLING_THRESHOLD}): {metrics['settling_time']*1000:.1f} ms")
        else:
            print(f"  Settling Time (±{self.SETTLING_THRESHOLD}): N/A (not settled)")
        
        print(f"\nError:")
        print(f"  Overshoot:           {metrics['overshoot']:8.0f} deg ({metrics['overshoot_percent']:.1f}%)")
        print(f"  Steady-State Error:  {metrics['steady_state_error']:8.0f} deg")
        print(f"  RMS Error:           {metrics['rms_error']:8.1f} deg")
       
    def plot_step_response(self, metrics: dict) -> str:
        """Plot step response with position and velocity (time domain)"""
        print("\n" + "-"*60)
        print("Generating Step Response Plots")
        print("-"*60)

        fig = plt.figure(figsize=(14, 8))
        gs = fig.add_gridspec(2, 1, hspace=0.3)

        ax1 = fig.add_subplot(gs[0])
        ax2 = fig.add_subplot(gs[1])

        fig.suptitle(
            f'Motor Step Response Test - Step: {self.STEP_SIZE} deg',
            fontsize=14,
            fontweight='bold'
        )

        # --- Plot 1: Position vs Time ---
        ax1.plot(
            self.timestamps,
            self.target_positions,
            'k--',
            linewidth=1.5,
            label='Target Position'
        )
        ax1.plot(
            self.timestamps,
            self.actual_positions,
            'b-',
            linewidth=1,
            label='Actual Position'
        )

        ax1.set_ylabel('Position (deg)')
        ax1.set_xlim(0, self.CAPTURE_DURATION)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='right')

        # Settling band
        target = metrics['target']
        ax1.axhline(target + self.SETTLING_THRESHOLD, color='g', linestyle=':', alpha=0.5)
        ax1.axhline(target - self.SETTLING_THRESHOLD, color='g', linestyle=':', alpha=0.5)
        ax1.fill_between(
            self.timestamps,
            target - self.SETTLING_THRESHOLD,
            target + self.SETTLING_THRESHOLD,
            alpha=0.1,
            color='green'
        )

        if metrics['rise_time'] and metrics['settling_time']:
            ax1.set_title(
                f'Position Response (Rise: {metrics["rise_time"]*1000:.1f} ms, '
                f'Settling: {metrics["settling_time"]*1000:.1f} ms)'
            )
        else:
            ax1.set_title('Position Step Response')

        # --- Plot 2: Velocity vs Time ---
        ax2.plot(
            self.timestamps,
            self.velocities,
            'r-',
            linewidth=1,
            label='Velocity'
        )

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (deg/s)')
        ax2.set_xlim(0, self.CAPTURE_DURATION)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right')
        ax2.set_title('Velocity Response')

        # Save plot
        if self.output_file:
            filepath = Path(self.output_file)
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"step_response_test_{timestamp}.png"
            filepath = Path(__file__).parent / filename

        filepath.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close()

        print(f"  [OK] Plot saved: {filepath.name}")
        return str(filepath)

    def run_all_tests(self) -> bool:
        """Run step response test"""
        if not self.setup():
            return False
        
        try:
            # Capture step response
            self.capture_step_response()
            
            # Check if we have valid data
            if len(self.actual_positions) == 0:
                print("\n[ERROR] No position data captured!")
                return False
            
            # Analyze step response
            metrics = self.analyze_step_response()
            
            # Print results
            self.print_metrics(metrics)
            
            # Plot response
            plot_file = self.plot_step_response(metrics)
            
            # Summary
            print("\n" + "="*60)
            print("Test Summary")
            print("="*60)
            print(f"  Duration:       {self.CAPTURE_DURATION}s")
            print(f"  Samples:        {len(self.actual_positions)}")
            print(f"  Step Size:      {self.STEP_SIZE} deg")
            print(f"  Plot saved:     {Path(plot_file).name}")
            print("="*60)
            print("\n[OK] Step response test completed successfully")
            
            return True
            
        except Exception as e:
            print(f"\n[ERROR] Test failed: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.teardown()


if __name__ == "__main__":
    adapter = os.environ.get("ADAPTER", None)
    output_file = os.environ.get("OUTPUT_FILE", None)
    tester = StepResponseTest(adapter, output_file=output_file)
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)
