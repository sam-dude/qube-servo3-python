"""Experimental Measurement of Pendulum Moment of Inertia.

Holds the motor arm rigid (zero-voltage — relies on the virtual twin's
"Lock Servo base" setting, or physical clamping on hardware), then records
free-swinging pendulum oscillations triggered automatically when the pendulum
is perturbed beyond a 5° threshold.

From the recorded oscillation data the script:
  1. Detects positive-peak times via a local-maxima scan.
  2. Counts cycles and computes the natural frequency f (Hz).
  3. Derives J_p from the small-angle pendulum formula:
       J_p = m_p·g·l / ω_n²     (ω_n = 2π·f)
  4. Compares the result against the analytical value from inertia_analytical.py.

Virtual twin setup:
  In Quanser Interactive Labs, open the QUBE-Servo 3 Pendulum Workspace,
  go to Settings and enable "Lock Servo base" before clicking "Lift Pendulum".

Reference:
  Quanser sp5 Application Guide — Interfacing and Inertia
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/1_interfacing_and_inertia/
"""

from __future__ import annotations

import sys
import time
import numpy as np
from threading import Thread
from pathlib import Path

try:
    from pal.products.qube import QubeServo3
    from pal.utilities.scope import MultiScope
except ModuleNotFoundError:
    print("[-] Error: Quanser PAL libraries not found.")
    print("    Activate the project virtual environment before running:")
    print("      .venv\\Scripts\\Activate.ps1")
    print("    See README.md > 'Installation' for setup instructions.")
    sys.exit(1)

# Physical constants — import from shared module, fall back to defaults
_repo_root = str(Path(__file__).resolve().parents[3])
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
try:
    from modeling.constants import m_p as PEND_MASS, L_p as PEND_LENGTH, g as GRAVITY
    PEND_COM = PEND_LENGTH / 2.0
except ImportError:
    PEND_MASS   = 0.024
    PEND_LENGTH = 0.129
    PEND_COM    = PEND_LENGTH / 2.0
    GRAVITY     = 9.81


def wrap_to_pi(angle: float) -> float:
    """Wrap angle in radians to [-π, π]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def find_peaks(
    times: list[float],
    angles: list[float],
    min_height_deg: float = 3.0,
    min_dist_s: float = 0.25,
) -> list[tuple[float, float]]:
    """Detect local maxima (positive peaks) in the angle data.

    Args:
        times:          Sample timestamps in seconds.
        angles:         Pendulum angle samples in degrees.
        min_height_deg: Minimum peak height to count (filters small noise).
        min_dist_s:     Minimum time between consecutive peaks (de-bouncing).

    Returns:
        List of (time_s, angle_deg) tuples, one per detected peak.
    """
    peaks: list[tuple[float, float]] = []
    n = len(angles)
    if n < 3:
        return peaks

    last_peak_time = -999.0
    for i in range(1, n - 1):
        t = times[i]
        val = angles[i]
        if val > angles[i - 1] and val > angles[i + 1] and val > min_height_deg:
            if t - last_peak_time >= min_dist_s:
                peaks.append((t, val))
                last_peak_time = t
    return peaks


def main() -> None:
    print("==================================================")
    print("  sp5.1b: Pendulum Inertia — Experimental        ")
    print("==================================================")
    print("Virtual twin setup:")
    print("  1. Open QUBE-Servo 3 Pendulum Workspace in Q-Labs.")
    print("  2. Settings → enable 'Lock Servo base'.")
    print("  3. Run this script.")
    print("  4. Click 'Lift Pendulum' in Q-Labs.")
    print("  5. The script auto-triggers on perturbation and records for 5 s.")
    print("--------------------------------------------------")
    input("Press Enter to connect to QUBE-Servo 3...")

    multiScope = MultiScope(rows=2, cols=1, title="sp5.1b: Pendulum Free Oscillations")
    multiScope.addAxis(row=0, col=0, timeWindow=6)
    multiScope.axes[0].attachSignal(name='Pendulum Angle (deg)')
    multiScope.addAxis(row=1, col=0, timeWindow=6)
    multiScope.axes[1].attachSignal(name='Arm Angle (deg)')

    global running, recording, recorded_times, recorded_angles
    running = True
    recording = False
    recorded_times: list[float] = []
    recorded_angles: list[float] = []

    def control_loop() -> None:
        global running, recording, recorded_times, recorded_angles

        with QubeServo3(hardware=0, pendulum=1) as myQube:
            start_time = time.time()
            trigger_threshold = np.deg2rad(5.0)  # 5° perturbation to start recording
            record_duration = 5.0                 # s

            count = 0
            count_max = 500 / 50                  # 50 Hz scope update
            record_start_time = 0.0

            print("\n[+] Connected. Waiting for perturbation (>5°)...")

            while running:
                myQube.read_outputs()

                theta     = float(myQube.motorPosition.item()    if hasattr(myQube.motorPosition,    'item') else myQube.motorPosition)
                alpha_raw = float(myQube.pendulumPosition.item() if hasattr(myQube.pendulumPosition, 'item') else myQube.pendulumPosition)

                # Zero voltage: let the simulator/hardware handle base-locking
                myQube.write_voltage(0.0)

                alpha_deg = np.rad2deg(alpha_raw)
                theta_deg = np.rad2deg(theta)
                current_time = time.time() - start_time

                count += 1
                if count >= count_max:
                    multiScope.axes[0].sample(current_time, [alpha_deg])
                    multiScope.axes[1].sample(current_time, [theta_deg])
                    count = 0

                # Auto-trigger on perturbation
                if not recording and abs(alpha_raw) > trigger_threshold:
                    recording = True
                    record_start_time = current_time
                    print("\n[+] Perturbation detected. Recording 5 s of oscillations...")

                if recording:
                    elapsed = current_time - record_start_time
                    recorded_times.append(elapsed)
                    recorded_angles.append(alpha_deg)
                    if elapsed >= record_duration:
                        print("\n[+] Recording complete. Analysing...")
                        running = False

                time.sleep(0.002)

    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    try:
        while running and thread.is_alive():
            multiScope.refresh()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user.")
        running = False

    thread.join(timeout=2.0)

    # Analysis
    if len(recorded_angles) < 10:
        print("\n[-] No oscillation data recorded. Check that 'Lock Servo base' is enabled.")
        print("Press Enter to exit.")
        input()
        return

    print("\n--------------------------------------------------")
    print("  Analysis Results")
    print("--------------------------------------------------")

    peaks = find_peaks(recorded_times, recorded_angles, min_height_deg=3.0, min_dist_s=0.25)

    if len(peaks) < 2:
        print("[-] Too few peaks detected. Increase perturbation amplitude and retry.")
        print("Press Enter to exit.")
        input()
        return

    print(f"Detected {len(peaks)} peaks:")
    for i, (t_peak, val_peak) in enumerate(peaks):
        print(f"  Peak {i + 1:2d}: t = {t_peak:.3f} s, α = {val_peak:+.2f}°")

    t_start = peaks[0][0]
    t_end   = peaks[-1][0]
    n_cyc   = len(peaks) - 1
    delta_t = t_end - t_start

    f       = n_cyc / delta_t
    omega_n = 2.0 * np.pi * f

    mgl     = PEND_MASS * GRAVITY * PEND_COM
    J_p_exp = mgl / (omega_n ** 2)

    J_p_theory  = (1.0 / 3.0) * PEND_MASS * (PEND_LENGTH ** 2)
    error_pct   = abs(J_p_exp - J_p_theory) / J_p_theory * 100.0

    print("\nFrequency:")
    print(f"  Cycles (n)     : {n_cyc}")
    print(f"  Duration (Δt)  : {delta_t:.3f} s")
    print(f"  f              : {f:.3f} Hz")
    print(f"  ω_n            : {omega_n:.3f} rad/s")
    print(f"  T = 1/f        : {1.0 / f:.3f} s")

    print("\nMoment of inertia J_p:")
    print(f"  m_p·g·l        : {mgl:.6f} N·m")
    print(f"  Experimental   : {J_p_exp:.8f} kg·m²")
    print(f"  Analytical     : {J_p_theory:.8f} kg·m²")
    print(f"  Error          : {error_pct:.2f}%")

    if error_pct <= 10.0:
        print("  [+] Within 10% — consistent with uniform-rod model.")
    else:
        print("  [!] >10% error. Check that the base was rigidly locked")
        print("      and the pendulum oscillated freely without hitting the stops.")

    print("\nPress Enter to exit.")
    input()


if __name__ == "__main__":
    main()
