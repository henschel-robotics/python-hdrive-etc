"""
Motor Frequency Response Test (Bode Plot)

Tests motor position control frequency response:
- Sends sinusoidal position commands at various frequencies
- Measures amplitude ratio (gain) and phase shift at each frequency
- Calculates closed-loop bandwidth and phase margin
- Generates Bode plot (magnitude and phase vs frequency)
- Useful for control loop tuning and stability analysis

Run as administrator: ADAPTER=\\Device\\NPF_{...} python test_frequency_response.py
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
from scipy import signal

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from hdrive_etc import HDriveETC, Mode


class FrequencyResponseTest:
    """Test motor position control frequency response"""
    
    # Test configuration — overridable via environment variables
    _start_freq = float(os.environ.get("START_FREQ", 2.0))
    _stop_freq = float(os.environ.get("STOP_FREQ", 30.0))
    AMPLITUDE = int(os.environ.get("AMPLITUDE", 1000))
    CYCLES_PER_FREQ = 5
    SAMPLE_INTERVAL = 0.001
    SETTLE_TIME = 0.3

    # Build frequency list: logarithmically spaced between start and stop
    FREQUENCIES = sorted(set(
        [round(f, 1) for f in np.logspace(
            np.log10(max(_start_freq, 0.5)),
            np.log10(max(_stop_freq, 1.0)),
            10
        )]
    ))
    
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
        self.results = []  # List of (freq, gain_db, phase_deg) tuples
        
    def setup(self) -> bool:
        """Initialize motor connection (if not using external motor)"""
        print("\n" + "="*60)
        print("Motor Frequency Response Test (Bode Plot)")
        print("="*60)
        
        # Skip connection if using external motor
        if self._external_motor:
            print("  Using existing motor connection")
            print(f"  Sine amplitude: {self.AMPLITUDE} deg")
            print(f"  Frequency range: {self.FREQUENCIES[0]} - {self.FREQUENCIES[-1]} Hz")
            return True
        
        try:
            self.motor = HDriveETC(adapter=self.adapter, slave_index=0, cycle_time_ms=0.2)
            print("  Connecting to motor...")
            self.motor.connect()
            time.sleep(0.5)
            
            # Set to position mode
            print("  Setting motor to POSITION mode")
            self.motor.set_mode(Mode.POSITION)
            bw_current  = int(os.environ.get("BW_CURRENT", 1000))
            bw_speed    = int(os.environ.get("BW_SPEED", 0))
            bw_position = int(os.environ.get("BW_POSITION", 30))
            self.motor.set_control_bandwidth(bw_current, bw_speed, bw_position)
            print(f"  Control bandwidth: current={bw_current} Hz, speed={bw_speed} Hz, position={bw_position} Hz")
            self.motor.set_damping(int(os.environ.get("MOTOR_DAMPING", 200)))
            self.motor.set_rotor_inertia(int(os.environ.get("ROTOR_INERTIA", 800)))
            self.motor.trigger_parameter_calculation()
            time.sleep(0.5)
            
            # Get initial position
            initial_pos = self.motor.get_position()
            print(f"  Initial position: {initial_pos:.0f} deg")
            print(f"  Sine amplitude: {self.AMPLITUDE} deg")
            print(f"  Frequency range: {self.FREQUENCIES[0]} - {self.FREQUENCIES[-1]} Hz")
            
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
                time.sleep(0.5)
                self.motor.disconnect()
                print("  [OK] Motor connection closed")
            except Exception as e:
                print(f"  [ERROR] Teardown error: {e}")
    
    def test_frequency(self, frequency: float, center_position: float) -> dict:
        """
        Test response at a single frequency
        
        Args:
            frequency: Frequency to test in Hz
            center_position: Center position for sine wave
        
        Returns:
            dict with frequency, gain_db, phase_deg, and raw data
        """
        # Calculate test duration (at least N cycles)
        period = 1.0 / frequency
        duration = self.CYCLES_PER_FREQ * period + self.SETTLE_TIME
        
        # Debug: ensure minimum duration
        if duration < 0.5:
            duration = 0.5  # At least 500ms total
        
        print(f"\n  Testing {frequency:.1f} Hz (dur: {duration:.2f}s)...", end=" ", flush=True)
        
        # Generate sine wave reference
        total_samples = int(duration / self.SAMPLE_INTERVAL)
        time_array = np.arange(total_samples) * self.SAMPLE_INTERVAL
        
        # Sine wave: position = center + amplitude * sin(2*pi*f*t)
        target_positions = center_position + self.AMPLITUDE * np.sin(2 * np.pi * frequency * time_array)
        
        # Storage for actual positions
        actual_positions = []
        timestamps = []
        
        # Send sine wave and capture response
        start_time = time.time()
        sample_idx = 0
        
        while (time.time() - start_time) < duration:
            if getattr(self, 'abort_event', None) and self.abort_event.is_set():
                break
            elapsed = time.time() - start_time
            
            if sample_idx < len(target_positions):
                self.motor.set_position(int(target_positions[sample_idx]))
                sample_idx += 1
            
            # Read actual position
            actual_pos = self.motor.get_position()
            actual_positions.append(actual_pos)
            timestamps.append(elapsed)
            
            # Wait for next sample
            time.sleep(self.SAMPLE_INTERVAL)
        
        # Convert to numpy arrays
        timestamps = np.array(timestamps)
        actual_positions = np.array(actual_positions)
        
        # Check if we have enough data
        if len(actual_positions) < 10:
            print(f"ERROR: Only captured {len(actual_positions)} samples")
            return {
                'frequency': frequency,
                'gain_db': -100,
                'phase_deg': 0,
                'timestamps': timestamps,
                'target': target_positions[:len(actual_positions)],
                'actual': actual_positions
            }
        
        # Remove settling time data for analysis
        settle_samples = int(self.SETTLE_TIME / self.SAMPLE_INTERVAL)
        if len(timestamps) > settle_samples + 50:  # Ensure at least 50 samples remain
            timestamps = timestamps[settle_samples:] - timestamps[settle_samples]
            actual_positions = actual_positions[settle_samples:]
            target_positions_analysis = target_positions[settle_samples:settle_samples + len(actual_positions)]
        else:
            # Not enough samples, use all data
            target_positions_analysis = target_positions[:len(actual_positions)]
        
        # Final safety check
        if len(actual_positions) == 0 or len(target_positions_analysis) == 0:
            print(f"ERROR: Empty data after processing")
            return {
                'frequency': frequency,
                'gain_db': -100,
                'phase_deg': 0,
                'timestamps': np.array([0]),
                'target': np.array([center_position]),
                'actual': np.array([center_position])
            }
        
        # Calculate gain and phase using FFT
        gain_db, phase_deg = self.calculate_gain_phase(target_positions_analysis, actual_positions, frequency)
        
        print(f"Gain: {gain_db:+6.2f} dB, Phase: {phase_deg:+7.1f}°")
        
        return {
            'frequency': frequency,
            'gain_db': gain_db,
            'phase_deg': phase_deg,
            'timestamps': timestamps,
            'target': target_positions,
            'actual': actual_positions
        }
    
    def calculate_gain_phase(self, target: np.ndarray, actual: np.ndarray, frequency: float):
        """
        Calculate gain and phase at the test frequency using FFT
        
        Args:
            target: Target position array
            actual: Actual position array
            frequency: Test frequency in Hz
        
        Returns:
            gain_db: Gain in dB (20*log10(output_amp/input_amp))
            phase_deg: Phase shift in degrees (positive = lead, negative = lag)
        """
        sample_rate = 1.0 / self.SAMPLE_INTERVAL
        
        # Make sure arrays are same length
        min_len = min(len(target), len(actual))
        target = target[:min_len]
        actual = actual[:min_len]
        
        if min_len < 10:
            # Not enough data
            return -100, 0
        
        n = min_len
        
        # Apply Hanning window to reduce spectral leakage
        window = np.hanning(n)
        target_windowed = (target - np.mean(target)) * window
        actual_windowed = (actual - np.mean(actual)) * window
        
        # Compute FFT
        target_fft = np.fft.rfft(target_windowed)
        actual_fft = np.fft.rfft(actual_windowed)
        freqs = np.fft.rfftfreq(n, 1/sample_rate)
        
        # Find the bin closest to the test frequency
        freq_idx = np.argmin(np.abs(freqs - frequency))
        
        # Get complex values at the test frequency
        target_complex = target_fft[freq_idx]
        actual_complex = actual_fft[freq_idx]
        
        # Calculate amplitude ratio (gain)
        target_amplitude = np.abs(target_complex)
        actual_amplitude = np.abs(actual_complex)
        
        if target_amplitude > 0:
            gain_linear = actual_amplitude / target_amplitude
            gain_db = 20 * np.log10(gain_linear) if gain_linear > 0 else -100
        else:
            gain_db = -100
        
        # Calculate phase shift
        target_phase = np.angle(target_complex, deg=True)
        actual_phase = np.angle(actual_complex, deg=True)
        phase_deg = actual_phase - target_phase
        
        # Wrap phase to [-180, 180]
        while phase_deg > 180:
            phase_deg -= 360
        while phase_deg < -180:
            phase_deg += 360
        
        return gain_db, phase_deg
    
    def run_frequency_sweep(self):
        """Run frequency sweep test"""
        print("\n" + "-"*60)
        print("Running Frequency Sweep")
        print("-"*60)
        print(f"Frequencies: {len(self.FREQUENCIES)}")
        print(f"Amplitude: {self.AMPLITUDE} deg")
        print(f"Cycles per frequency: {self.CYCLES_PER_FREQ}")
        
        # Get center position
        center_pos = self.motor.get_position()
        
        self.results = []
        for freq in self.FREQUENCIES:
            if getattr(self, 'abort_event', None) and self.abort_event.is_set():
                print("\n  [ABORT] Test aborted by user")
                return
            result = self.test_frequency(freq, center_pos)
            self.results.append(result)
        
        print(f"\n  [OK] Completed {len(self.results)} frequency tests")
    
    def analyze_results(self):
        """Analyze frequency response results"""
        print("\n" + "-"*60)
        print("Frequency Response Analysis")
        print("-"*60)
        
        # Extract data
        frequencies = [r['frequency'] for r in self.results]
        gains_db = [r['gain_db'] for r in self.results]
        phases_deg = [r['phase_deg'] for r in self.results]
        
        # Find -3dB bandwidth (cutoff frequency)
        gains_array = np.array(gains_db)
        dc_gain = gains_db[0]  # Gain at lowest frequency
        cutoff_gain = dc_gain - 3.0  # -3dB point
        
        # Find frequency where gain drops below -3dB
        bandwidth_idx = np.where(gains_array < cutoff_gain)[0]
        if len(bandwidth_idx) > 0:
            # Interpolate for more accurate bandwidth
            idx = bandwidth_idx[0]
            if idx > 0:
                f1, g1 = frequencies[idx-1], gains_db[idx-1]
                f2, g2 = frequencies[idx], gains_db[idx]
                bandwidth = f1 + (cutoff_gain - g1) * (f2 - f1) / (g2 - g1)
            else:
                bandwidth = frequencies[0]
        else:
            bandwidth = frequencies[-1]  # Above test range
        
        # Find phase margin at gain crossover (0 dB)
        phase_margin = None
        for i in range(len(gains_db)-1):
            if gains_db[i] >= 0 and gains_db[i+1] < 0:
                # Interpolate frequency at 0 dB
                f1, g1, p1 = frequencies[i], gains_db[i], phases_deg[i]
                f2, g2, p2 = frequencies[i+1], gains_db[i+1], phases_deg[i+1]
                
                # Interpolate phase at 0 dB crossing
                phase_at_0db = p1 + (0 - g1) * (p2 - p1) / (g2 - g1)
                phase_margin = 180 + phase_at_0db  # Phase margin is 180° + phase
                break
        
        print(f"\nSystem Characteristics:")
        print(f"  DC Gain:          {dc_gain:+6.2f} dB")
        print(f"  Bandwidth (-3dB): {bandwidth:6.2f} Hz")
        if phase_margin is not None:
            print(f"  Phase Margin:     {phase_margin:6.1f}° ")
            if phase_margin > 45:
                print(f"                    ([OK] Good stability)")
            elif phase_margin > 30:
                print(f"                    ([WARNING] Marginal stability)")
            else:
                print(f"                    ([WARNING] Poor stability)")
        else:
            print(f"  Phase Margin:     N/A (no 0 dB crossing)")
        
        return {
            'dc_gain': dc_gain,
            'bandwidth': bandwidth,
            'phase_margin': phase_margin
        }
    
    def plot_bode(self, analysis: dict) -> str:
        """Generate Bode plot"""
        print("\n" + "-"*60)
        print("Generating Bode Plot")
        print("-"*60)
        
        # Extract data
        frequencies = [r['frequency'] for r in self.results]
        gains_db = [r['gain_db'] for r in self.results]
        phases_deg = [r['phase_deg'] for r in self.results]
        
        # Create figure with 2 subplots (magnitude and phase)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        fig.suptitle(f'Position Control Frequency Response (Bode Plot)\nAmplitude: {self.AMPLITUDE} deg', 
                     fontsize=14, fontweight='bold')
        
        # Plot 1: Magnitude (Gain in dB)
        ax1.semilogx(frequencies, gains_db, 'b-o', linewidth=2, markersize=6, label='Measured')
        ax1.axhline(analysis['dc_gain'] - 3, color='r', linestyle='--', linewidth=1, 
                   label=f'-3dB Line ({analysis["dc_gain"]-3:.1f} dB)')
        ax1.axvline(analysis['bandwidth'], color='g', linestyle='--', linewidth=1,
                   label=f'Bandwidth: {analysis["bandwidth"]:.1f} Hz')
        ax1.axhline(0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
        ax1.set_ylabel('Magnitude (dB)', fontsize=11)
        ax1.set_title(f'Magnitude Response (DC Gain: {analysis["dc_gain"]:.1f} dB, BW: {analysis["bandwidth"]:.1f} Hz)', 
                     fontsize=12)
        ax1.grid(True, which='both', alpha=0.3)
        ax1.legend(loc='upper right')
        ax1.set_xlim(frequencies[0], frequencies[-1])
        
        # Plot 2: Phase
        ax2.semilogx(frequencies, phases_deg, 'r-o', linewidth=2, markersize=6, label='Measured')
        ax2.axhline(-180, color='k', linestyle='--', linewidth=1, alpha=0.5, label='-180°')
        ax2.axhline(0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
        if analysis['phase_margin'] is not None:
            # Find frequency at 0 dB for phase margin annotation
            for i in range(len(gains_db)-1):
                if gains_db[i] >= 0 and gains_db[i+1] < 0:
                    f_crossover = frequencies[i]
                    ax2.axvline(f_crossover, color='g', linestyle='--', linewidth=1, alpha=0.5)
                    break
        ax2.set_xlabel('Frequency (Hz)', fontsize=11)
        ax2.set_ylabel('Phase (deg)', fontsize=11)
        if analysis['phase_margin'] is not None:
            ax2.set_title(f'Phase Response (Phase Margin: {analysis["phase_margin"]:.1f}°)', fontsize=12)
        else:
            ax2.set_title('Phase Response', fontsize=12)
        ax2.grid(True, which='both', alpha=0.3)
        ax2.legend(loc='lower left')
        ax2.set_xlim(frequencies[0], frequencies[-1])
        ax2.set_ylim(-200, 50)
        
        plt.tight_layout()
        
        # Save plot
        if self.output_file:
            filepath = Path(self.output_file)
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"frequency_response_test_{timestamp}.png"
            filepath = Path(__file__).parent / filename
        
        filepath.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"  [OK] Bode plot saved: {filepath.name}")
        
        # Close plot to free memory
        plt.close()
        
        return str(filepath)

    def run_all_tests(self) -> bool:
        """Run frequency response test"""
        if not self.setup():
            return False
        
        try:
            # Run frequency sweep
            self.run_frequency_sweep()
            
            # Analyze results
            analysis = self.analyze_results()
            
            # Generate Bode plot
            plot_file = self.plot_bode(analysis)
            
            # Summary
            print("\n" + "="*60)
            print("Test Summary")
            print("="*60)
            print(f"  Frequencies tested: {len(self.FREQUENCIES)}")
            print(f"  Amplitude:          {self.AMPLITUDE} deg")
            print(f"  Bandwidth:          {analysis['bandwidth']:.2f} Hz")
            if analysis['phase_margin']:
                print(f"  Phase Margin:       {analysis['phase_margin']:.1f}°")
            print(f"  Bode plot saved:    {Path(plot_file).name}")
            print("="*60)
            print("\n[OK] Frequency response test completed successfully")
            
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
    tester = FrequencyResponseTest(adapter, output_file=output_file)
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)
