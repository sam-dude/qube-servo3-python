"""Pendulum Interfacing: Developing the QUBE-Servo 3 Pendulum Conventions.

This script interfaces with the QUBE-Servo 3 (virtual or physical) to:
1. Stream raw encoder counts and converted angles (radians & degrees).
2. Allow real-time voltage adjustments limited to ±1.0 V using the keyboard.
3. Toggle a "Modulus & Bias" wrapping function to align the pendulum angle (alpha = 0 upright).

Angle conventions established here:
  theta : arm angle from horizontal, positive CCW when viewed from above.
  alpha : pendulum angle from upright equilibrium (= 0 when rod is straight up).
          Raw driver output has alpha = 0 at the downward-hanging rest position.
          With wrapping enabled: alpha_rad = wrap_to_pi(alpha_rad_raw - pi)
          so that upright is 0, hanging is ±pi.

Encoder resolution: 2048 counts per revolution (as shipped).

Controls:
  [w] or [Up Arrow]    : Increase motor voltage by 0.1 V (limit ±1.0 V)
  [s] or [Down Arrow]  : Decrease motor voltage by 0.1 V (limit ±1.0 V)
  [r]                  : Reset motor voltage to 0.0 V
  [m]                  : Toggle Modulus & Bias wrapping (alpha = 0 upright)
  [q] or [Esc]         : Quit

Reference:
  Quanser sp5 Application Guide — Interfacing and Inertia
  https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/6_teaching/1_Controls/Qube_Servo_3/sp5_pendulum_modeling/1_interfacing_and_inertia/
"""

from __future__ import annotations

import sys
import time
import msvcrt
import numpy as np
from threading import Thread

try:
    from pal.products.qube import QubeServo3
    from pal.utilities.scope import MultiScope
except ModuleNotFoundError:
    print("[-] Error: Quanser PAL libraries not found.")
    print("    Activate the project virtual environment before running:")
    print("      .venv\\Scripts\\Activate.ps1")
    print("    See README.md > 'Installation' for setup instructions.")
    sys.exit(1)

# Encoder resolution: 2048 counts per revolution
ENCODER_CPR = 2048
RAD_PER_COUNT = 2.0 * np.pi / ENCODER_CPR


def wrap_to_pi(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def main():
    print("==================================================")
    print("  Pendulum Interfacing: QUBE-Servo 3 Conventions  ")
    print("==================================================")
    print("[*] Target Hardware: 0 (Virtual Twin)")
    print("[*] Encoders Resolution: 2048 counts/revolution")
    print("--------------------------------------------------")
    print("Controls:")
    print("  [w] or [Up Arrow]   : Increase voltage (+0.1V, limit ±1.0V)")
    print("  [s] or [Down Arrow] : Decrease voltage (-0.1V, limit ±1.0V)")
    print("  [r]                 : Reset voltage to 0.0V")
    print("  [m]                 : Toggle Modulus & Bias wrapping")
    print("  [q] or [Esc]        : Quit")
    print("--------------------------------------------------")

    hardware = 0  # Default to virtual twin

    # Initialize MultiScope
    multiScope = MultiScope(
        rows=2,
        cols=2,
        title="Pendulum Interfacing: QUBE-Servo 3 Conventions"
    )

    multiScope.addAxis(row=0, col=0, timeWindow=5)
    multiScope.axes[0].attachSignal(name='Arm Angle (deg)')
    multiScope.axes[0].attachSignal(name='Arm Count (x0.1)')  # Scaled for visibility

    multiScope.addAxis(row=0, col=1, timeWindow=5)
    multiScope.axes[1].attachSignal(name='Pendulum Angle (deg)')
    multiScope.axes[1].attachSignal(name='Pendulum Count (x0.1)')  # Scaled for visibility

    multiScope.addAxis(row=1, col=0, timeWindow=5)
    multiScope.axes[2].attachSignal(name='Applied Voltage (V)')

    multiScope.addAxis(row=1, col=1, timeWindow=5)
    multiScope.axes[3].attachSignal(name='Arm Speed (deg/s)')
    multiScope.axes[3].attachSignal(name='Pendulum Speed (deg/s)')

    # Shared variables between threads
    global running, voltage, wrap_enabled
    running = True
    voltage = 0.0
    wrap_enabled = False

    def control_loop():
        global running, voltage, wrap_enabled
        count = 0
        count_max = 500 / 50  # 50 Hz scope update

        # Connect to virtual/physical QUBE-Servo 3
        with QubeServo3(hardware=hardware, pendulum=1) as myQube:
            start_time = time.time()
            time_last_print = 0.0

            while running:
                # Read outputs from device
                myQube.read_outputs()

                # Retrieve raw encoder counts and positions
                arm_counts = myQube.motorEncoderCounts
                pend_counts = myQube.pendulumEncoderCounts

                # Position from driver (radians)
                theta_rad = myQube.motorPosition
                alpha_rad_raw = myQube.pendulumPosition

                # Apply Modulus & Bias wrapping if enabled:
                # Raw driver: alpha = 0 at hanging (downward) rest position.
                # Convention: alpha = 0 at upright equilibrium.
                # Transform: alpha = wrap_to_pi(alpha_raw - pi)
                if wrap_enabled:
                    alpha_rad = wrap_to_pi(alpha_rad_raw - np.pi)
                else:
                    alpha_rad = alpha_rad_raw

                # Convert to degrees for scope display
                theta_deg = np.rad2deg(theta_rad)
                alpha_deg = np.rad2deg(alpha_rad)

                # Speeds
                theta_dot_deg = np.rad2deg(myQube.motorSpeed)
                alpha_dot_deg = np.rad2deg(myQube.pendulumSpeed)

                # Write voltage to motor (clamped to ±1.0 V for this lab)
                clamped_voltage = np.clip(voltage, -1.0, 1.0)
                myQube.write_voltage(clamped_voltage)

                current_time = time.time() - start_time

                # Print telemetry to console at 5 Hz
                if current_time - time_last_print >= 0.2:
                    sys.stdout.write(
                        f"\r[Ctrl] Volts: {clamped_voltage:+.2f}V | "
                        f"[Arm] Pos: {theta_deg:+.1f}° (Counts: {arm_counts:5d}) | "
                        f"[Pend] Pos: {alpha_deg:+.1f}° (Counts: {pend_counts:5d}, Wrap: {'ON' if wrap_enabled else 'OFF'})"
                    )
                    sys.stdout.flush()
                    time_last_print = current_time

                # Sample into Scope
                count += 1
                if count >= count_max:
                    multiScope.axes[0].sample(current_time, [theta_deg, arm_counts * 0.1])
                    multiScope.axes[1].sample(current_time, [alpha_deg, pend_counts * 0.1])
                    multiScope.axes[2].sample(current_time, [clamped_voltage])
                    multiScope.axes[3].sample(current_time, [theta_dot_deg, alpha_dot_deg])
                    count = 0

                # Sleep to maintain ~500 Hz loop rate
                time.sleep(0.002)

    # Start control thread
    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    # Keyboard input handler (main thread)
    try:
        while running and thread.is_alive():
            # Refresh Scope (must run in main thread)
            multiScope.refresh()

            if msvcrt.kbhit():
                ch = msvcrt.getch()
                # Handle special keys/arrows on Windows
                if ch == b'\x00' or ch == b'\xe0':
                    ch2 = msvcrt.getch()
                    if ch2 == b'H':   # Up Arrow
                        voltage = min(1.0, voltage + 0.1)
                    elif ch2 == b'P':  # Down Arrow
                        voltage = max(-1.0, voltage - 0.1)
                else:
                    ch = ch.lower()
                    if ch == b'w':
                        voltage = min(1.0, voltage + 0.1)
                    elif ch == b's':
                        voltage = max(-1.0, voltage - 0.1)
                    elif ch == b'r':
                        voltage = 0.0
                    elif ch == b'm':
                        wrap_enabled = not wrap_enabled
                    elif ch in (b'q', b'\x1b'):  # q or Esc
                        running = False

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by user.")
    finally:
        running = False
        thread.join(timeout=1.0)
        print("\n[+] Pendulum interfacing shut down cleanly. Voltage reset to 0.0 V.")


if __name__ == "__main__":
    main()
