"""LQR Balance Control & Setpoint Tracking.

This script implements an LQR/LQI controller to balance the pendulum in the upright position.
It supports real-time setpoint tracking (arm angle command) and allows interactive
tuning of LQR weighting matrices Q and R via the keyboard.

Controls:
  [w] / [s]   : Tune arm weight q_theta (+10% / -10%)
  [e] / [d]   : Tune pendulum weight q_alpha (+10% / -10%)
  [x] / [c]   : Tune input weight r (+10% / -10%)
  [u] / [j]   : Tune integral weight q_integral (+10% / -10%)
  [t]         : Toggle setpoint tracking (Arm tracking square wave)
  [g]         : Toggle Gain Mode (Factory Tuned vs. Custom LQI)
  [r]         : Reset weights to defaults
  [q] or [Esc]: Quit
"""

from __future__ import annotations

import sys
import time
import msvcrt
import math
import signal
import numpy as np
from threading import Thread

# Point python to Quanser library path
QUANSER_LIB_PATH = r"C:\Users\USER\Documents\Quanser\0_libraries\python"
if QUANSER_LIB_PATH not in sys.path:
    sys.path.append(QUANSER_LIB_PATH)

try:
    from pal.products.qube import QubeServo3
    from pal.utilities.math import ddt_filter
    from pal.utilities.scope import MultiScope
except ModuleNotFoundError:
    print(f"[-] Error: Quanser PAL libraries not found at '{QUANSER_LIB_PATH}'")
    print("    Please verify the library path in this script.")
    sys.exit(1)

# Import state space and LQR design modules
try:
    sys.path.append(r"C:\Users\USER\Downloads\test_qube.py")
    from modeling.lqr_design import design_lqr, design_lqi
except ImportError as e:
    print(f"[-] Error: Could not import modeling modules: {e}")
    sys.exit(1)

def main():
    print("==================================================")
    print("      LQR Balance Setpoint Tracking        ")
    print("==================================================")
    print("[*] Catch range: |alpha| < 10° (upright)")
    print("--------------------------------------------------")
    
    # Prompt user for Virtual vs. Physical QUBE-Servo 3 hardware
    hw_input = input("Select device: [0] Virtual Twin (Default), [1] Physical QUBE-Servo 3: ").strip()
    target_hardware = 1 if hw_input == "1" else 0
    
    print("--------------------------------------------------")
    print("Instructions:")
    if target_hardware == 1:
        print("[*] Target: PHYSICAL QUBE-Servo 3 hardware.")
        print("1. Set up the physical QUBE-Servo 3.")
        print("2. Run this script.")
        print("3. Manually lift the pendulum by hand to the upright position.")
    else:
        print("[*] Target: VIRTUAL TWIN.")
        print("1. Launch Quanser Interactive Labs (QUBE-Servo 3 - Pendulum Workspace).")
        print("2. Run this script.")
        print("3. Click 'Lift Pendulum' in Q-Labs to swing it up.")
        
    print("4. The LQI/LQR controller will automatically catch and balance the pendulum.")
    print("5. Press [t] to toggle tracking, and tune Q/R/Integral weights on the fly.")
    print("6. Press [g] to toggle between Factory Tuned gains and Custom LQI Tuned gains.")
    print("--------------------------------------------------")
    print("Controls:")
    print("  [w] / [s]   : Tune arm weight q_theta (+10% / -10%)")
    print("  [e] / [d]   : Tune pendulum weight q_alpha (+10% / -10%)")
    print("  [x] / [c]   : Tune input weight r (+10% / -10%)")
    print("  [u] / [j]   : Tune integral weight q_integral (+10% / -10%)")
    print("  [t]         : Toggle setpoint tracking (Arm tracking square wave)")
    print("  [g]         : Toggle Gain Mode (Factory Tuned vs. Custom LQI)")
    print("  [r]         : Reset weights to defaults")
    print("  [q] or [Esc]: Quit")
    print("--------------------------------------------------")
    input("Press Enter to begin the LQR balance loop...")

    # Initialize Scope
    multiScope = MultiScope(
        rows=2,
        cols=2,
        title="LQR Balance Control"
    )
    
    multiScope.addAxis(row=0, col=0, timeWindow=5)
    multiScope.axes[0].attachSignal(name='Arm Angle (deg)')
    multiScope.axes[0].attachSignal(name='Arm Target (deg)')
    
    multiScope.addAxis(row=0, col=1, timeWindow=5)
    multiScope.axes[1].attachSignal(name='Pendulum Angle (deg)')
    
    multiScope.addAxis(row=1, col=0, timeWindow=5)
    multiScope.axes[2].attachSignal(name='Applied Voltage (V)')
    
    multiScope.addAxis(row=1, col=1, timeWindow=5)
    multiScope.axes[3].attachSignal(name='Arm Gain k_th (x1)')
    multiScope.axes[3].attachSignal(name='Pend Gain k_al (x0.1)')

    # Shared variables
    global running, q_theta, q_alpha, r_weight, q_integral, tracking_enabled, gain_mode, log_buffer
    running = True
    q_theta = 2.0
    q_alpha = 35.0
    r_weight = 1.0
    q_integral = 0.5
    tracking_enabled = False
    gain_mode = 0  # 0 for Factory Tuned, 1 for Custom LQI Tuned
    log_buffer = []
    
    # Setup signal handler to exit control loop cleanly on keyboard interrupts
    def sig_handler(*args):
        global running
        running = False
    signal.signal(signal.SIGINT, sig_handler)

    def control_loop(hw_type: int):
        global running, q_theta, q_alpha, r_weight, q_integral, tracking_enabled, gain_mode, log_buffer
        
        frequency = 500.0 # Hz
        state_theta_dot = np.array([0, 0], dtype=np.float64)
        state_alpha_dot = np.array([0, 0], dtype=np.float64)
        
        theta_integral = 0.0
        theta_target_offset = 0.0
        count = 0
        count_max = frequency / 50.0 # 50 Hz scope update
        
        # Track previous weights to only design LQI on changes
        last_q_theta, last_q_alpha, last_r, last_q_integral = -1.0, -1.0, -1.0, -1.0
        K_custom = np.zeros(5)
        K_raw = np.zeros(5)
        
        with QubeServo3(hardware=hw_type, pendulum=1, frequency=int(frequency)) as myQube:
            start_time = time.perf_counter()
            time_last_print = 0.0
            
            while running:
                myQube.read_outputs()
                
                # Dynamic LQI gain calculation when weights change
                if (q_theta != last_q_theta or q_alpha != last_q_alpha or 
                    r_weight != last_r or q_integral != last_q_integral):
                    try:
                        K_raw, _ = design_lqi(
                            q_theta=q_theta,
                            q_alpha=q_alpha,
                            q_theta_dot=0.1,
                            q_alpha_dot=0.1,
                            q_integral=q_integral,
                            r_val=r_weight
                        )
                        K_custom = np.array([K_raw[0], -K_raw[1], K_raw[2], -K_raw[3], K_raw[4]])
                        last_q_theta, last_q_alpha, last_r, last_q_integral = q_theta, q_alpha, r_weight, q_integral
                    except Exception:
                        pass
                
                # Choose active gain vector
                if gain_mode == 0:
                    K_active = np.array([-1.2247, 24.9044, -0.6877, 3.1321])
                    mode_str = "Factory"
                else:
                    K_active = K_custom
                    mode_str = "Custom "
                
                current_time = time.perf_counter() - start_time
                
                # Read sensor positions (matching Quanser example exactly)
                theta = float(myQube.motorPosition.item() if hasattr(myQube.motorPosition, "item") else myQube.motorPosition) * -1.0
                alpha_f = float(myQube.pendulumPosition.item() if hasattr(myQube.pendulumPosition, "item") else myQube.pendulumPosition)
                alpha = np.mod(alpha_f, 2.0 * np.pi) - np.pi
                alpha_degrees = abs(math.degrees(alpha))
                
                # Calculate filtered velocities (using continuous alpha_f to prevent wrapping spikes)
                theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1.0 / frequency)
                alpha_dot, state_alpha_dot = ddt_filter(alpha_f, state_alpha_dot, 100, 1.0 / frequency)
                
                # Setpoint command (degrees, converted inline)
                if tracking_enabled:
                    command_deg = 20.0 if (current_time % 6.0) < 3.0 else -20.0
                else:
                    command_deg = 0.0
                
                # LQI/LQR catch logic: only apply voltage when pendulum is near upright
                if alpha_degrees > 10.0:
                    voltage = 0.0
                    theta_integral = 0.0
                    theta_target_offset = theta
                    theta_ref = theta
                else:
                    # Slowly decay the offset to 0 to return the arm to absolute center
                    theta_target_offset *= 0.999
                    theta_ref = (command_deg * np.pi / 180.0) + theta_target_offset
                    
                    # Accumulate arm position error
                    theta_error = theta_ref - theta
                    theta_integral += theta_error * (1.0 / frequency)
                    theta_integral = np.clip(theta_integral, -2.0, 2.0)
                
                # State vector: target - measured
                states = np.array([theta_ref - theta, -alpha, -theta_dot, -alpha_dot])
                
                if alpha_degrees > 10.0:
                    voltage = 0.0
                else:
                    if len(K_active) >= 5:
                        K_int = K_active[4]
                        voltage = -1.0 * np.dot(K_active[:4], states) + K_int * theta_integral
                    else:
                        K_int = 0.5  # Fallback ad-hoc integral gain for Factory LQR
                        voltage = -1.0 * np.dot(K_active, states) + K_int * theta_integral
                
                # Output clamp
                final_voltage = float(np.clip(voltage, -6.0, 6.0))
                myQube.write_voltage(final_voltage)
                
                # Buffer telemetry data for logging (only when controller is active)
                if alpha_degrees <= 10.0:
                    log_buffer.append((
                        current_time,
                        theta,
                        alpha,
                        theta_dot,
                        alpha_dot,
                        final_voltage,
                        gain_mode
                    ))
                
                # Telemetry printing
                if current_time - time_last_print >= 0.2:
                    k_i_val = K_active[4] if len(K_active) >= 5 else 0.5
                    sys.stdout.write(
                        f"\r[{mode_str}] q_th:{q_theta:5.2f} | q_al:{q_alpha:5.2f} | q_in:{q_integral:5.2f} | r:{r_weight:5.2f} | "
                        f"K:[{K_active[0]:+.2f}, {K_active[1]:+.2f}, {K_active[2]:+.2f}, {K_active[3]:+.2f}, {k_i_val:+.2f}] | "
                        f"V:{final_voltage:+.2f}V"
                    )
                    sys.stdout.flush()
                    time_last_print = current_time

                # Sample into Scope at 50 Hz
                count += 1
                if count >= count_max:
                    multiScope.axes[0].sample(current_time, [float(np.rad2deg(-theta)), float(command_deg)])
                    multiScope.axes[1].sample(current_time, [float(np.rad2deg(alpha))])
                    multiScope.axes[2].sample(current_time, [final_voltage])
                    # Plot raw active weights/gains scaled down for visibility
                    multiScope.axes[3].sample(current_time, [float(K_active[0]), float(K_active[1] * 0.1)])
                    count = 0

    # Start loop
    thread = Thread(target=control_loop, args=(target_hardware,))
    thread.daemon = True
    thread.start()

    # Scope refresh and key readings
    try:
        last_refresh = time.perf_counter()
        while running and thread.is_alive():
            # Limit scope refresh to ~30 Hz (every 33ms) to avoid GIL contention
            now = time.perf_counter()
            if now - last_refresh >= 0.033:
                multiScope.refresh()
                last_refresh = now
            
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                ch = ch.lower()
                if ch == b'w':
                    q_theta = min(100.0, q_theta * 1.1)
                elif ch == b's':
                    q_theta = max(0.1, q_theta * 0.9)
                elif ch == b'e':
                    q_alpha = min(1000.0, q_alpha * 1.1)
                elif ch == b'd':
                    q_alpha = max(1.0, q_alpha * 0.9)
                elif ch == b'x':
                    r_weight = min(10.0, r_weight * 1.1)
                elif ch == b'c':
                    r_weight = max(0.05, r_weight * 0.9)
                elif ch == b'u':
                    q_integral = min(50.0, q_integral * 1.1)
                elif ch == b'j':
                    q_integral = max(0.01, q_integral * 0.9)
                elif ch == b't':
                    tracking_enabled = not tracking_enabled
                    print(f"\n[Toggle] Setpoint tracking: {'ENABLED' if tracking_enabled else 'DISABLED'}")
                elif ch == b'g':
                    gain_mode = 1 - gain_mode
                    mode_name = "Custom LQI Tuned" if gain_mode == 1 else "Factory Tuned"
                    print(f"\n[Toggle] Gain Mode: {mode_name}")
                elif ch == b'r':
                    q_theta = 2.0
                    q_alpha = 35.0
                    r_weight = 1.0
                    q_integral = 0.5
                    print("\n[Reset] LQI/LQR weights reset to defaults.")
                elif ch in (b'q', b'\x1b'):
                    running = False
            
            # Yield CPU to the control loop thread
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by user.")
    finally:
        running = False
        thread.join(timeout=1.0)
        print("\n[+] Lab 6 shut down cleanly. Voltage reset to 0.0 V.")
        
        if log_buffer:
            import csv
            log_path = "lqr_run_log.csv"
            print(f"[*] Saving {len(log_buffer)} lines of telemetry to '{log_path}'...")
            try:
                with open(log_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time_s', 'theta_rad', 'alpha_rad', 'theta_dot_rads', 'alpha_dot_rads', 'voltage_V', 'gain_mode'])
                    writer.writerows(log_buffer)
                print("[+] Telemetry saved successfully!")
            except Exception as e:
                print(f"[-] Error saving telemetry: {e}")

if __name__ == "__main__":
    main()
