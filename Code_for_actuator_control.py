import odrive
import time
import math

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_VEL_RAMP = 2


def find_odrive():
    print("Connecting to ODrive...")
    odrv = odrive.find_any()
    print(f"✓ Connected: {odrv.serial_number}\n")
    return odrv


def test_velocity_smoothness(axis, ramp_rate, vel_gain, duration=10):
    """
    Test velocity control smoothness with given parameters
    """
    # Configure
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.vel_ramp_rate = ramp_rate
    axis.controller.config.vel_gain = vel_gain
    axis.controller.config.vel_integrator_gain = vel_gain * 5

    # Enter closed loop
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    if not axis.is_armed:
        print("Failed to arm")
        return None

    print(f"\n{'=' * 60}")
    print(f"Testing: Ramp Rate={ramp_rate} rev/s², Vel Gain={vel_gain}")
    print(f"{'=' * 60}")

    # Run sine wave test
    start_time = time.time()
    base_rpm = 60
    amplitude_rpm = 40
    frequency_hz = 0.3

    max_jerk = 0
    avg_error = 0
    samples = 0

    print("\nTime | Target | Actual | Error | Max Jerk")
    print("-" * 60)

    last_vel = axis.vel_estimate
    last_accel = 0

    while (time.time() - start_time) < duration:
        elapsed = time.time() - start_time

        # Sine wave command
        target_rpm = base_rpm + amplitude_rpm * math.sin(2 * math.pi * frequency_hz * elapsed)
        target_rps = target_rpm / 60.0
        axis.controller.input_vel = target_rps

        # Measure
        actual_rpm = axis.vel_estimate * 60
        error = abs(target_rpm - actual_rpm)

        # Calculate jerk (derivative of acceleration)
        accel = (axis.vel_estimate - last_vel) / 0.2
        jerk = abs(accel - last_accel) / 0.2

        max_jerk = max(max_jerk, jerk)
        avg_error += error
        samples += 1

        if samples % 5 == 0:  # Print every 1 second
            print(f"{elapsed:4.1f}s | {target_rpm:6.1f} | {actual_rpm:6.1f} | {error:5.1f} | {max_jerk:5.2f}")

        last_vel = axis.vel_estimate
        last_accel = accel
        time.sleep(0.2)

    avg_error = avg_error / samples

    # Stop
    axis.controller.input_vel = 0
    time.sleep(1)
    axis.requested_state = AXIS_STATE_IDLE

    print(f"\n{'=' * 60}")
    print(f"RESULTS:")
    print(f"  Average Error: {avg_error:.1f} RPM")
    print(f"  Maximum Jerk: {max_jerk:.2f} rev/s³")
    print(f"  Smoothness Score: {100 - min(max_jerk * 10, 100):.0f}/100")
    print(f"{'=' * 60}\n")

    return {
        'ramp_rate': ramp_rate,
        'vel_gain': vel_gain,
        'avg_error': avg_error,
        'max_jerk': max_jerk,
        'score': 100 - min(max_jerk * 10, 100)
    }


def auto_tune_smoothness(axis):
    """
    Automatically find optimal parameters for smooth motion
    """
    print("\n" + "=" * 60)
    print("AUTO-TUNING FOR MAXIMUM SMOOTHNESS")
    print("=" * 60)
    print("\nThis will test multiple parameter combinations...")
    print("Looking for lowest jerk and good tracking accuracy\n")

    response = input("Start break-in? (yes/no): ")
    if response.lower() != 'yes':
        return

    # Configure velocity control
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.vel_ramp_rate = 5.0  # turns/s² (gentle ramp)
    axis.controller.config.vel_limit = max(20.0, velocity_rps * 1.5)  # Set higher than commanded

    print(f"\nVel limit set to: {axis.controller.config.vel_limit} turns/s")
    print(f"Commanded velocity: {velocity_rps} turns/s ({velocity_rps * 60:.1f} RPM)\n")

    # Parameter ranges to test
    ramp_rates = [5.0, 10.0, 15.0, 20.0]  # rev/s²
    vel_gains = [0.005, 0.01, 0.02, 0.03]

    results = []
    best_result = None

    print(f"\nTesting {len(ramp_rates) * len(vel_gains)} combinations...\n")

    for ramp_rate in ramp_rates:
        for vel_gain in vel_gains:
            print(f"\n[Test {len(results) + 1}/{len(ramp_rates) * len(vel_gains)}]")
            result = test_velocity_smoothness(axis, ramp_rate, vel_gain, duration=8)

            if result:
                results.append(result)

                # Best = lowest jerk with error < 15 RPM
                if result['avg_error'] < 15:
                    if best_result is None or result['max_jerk'] < best_result['max_jerk']:
                        best_result = result

            time.sleep(2)  # Cool down between tests

    # Display results
    print("\n" + "=" * 60)
    print("AUTO-TUNE RESULTS")
    print("=" * 60)

    print("\nAll tested combinations:")
    print(f"{'Ramp Rate':<12} {'Vel Gain':<10} {'Avg Error':<12} {'Max Jerk':<12} {'Score':<8}")
    print("-" * 60)
    for r in sorted(results, key=lambda x: x['score'], reverse=True):
        print(
            f"{r['ramp_rate']:<12.1f} {r['vel_gain']:<10.3f} {r['avg_error']:<12.1f} {r['max_jerk']:<12.2f} {r['score']:<8.0f}")

    if best_result:
        print("\n" + "=" * 60)
        print("OPTIMAL SETTINGS FOR SMOOTHEST MOTION:")
        print("=" * 60)
        print(f"  Velocity Ramp Rate: {best_result['ramp_rate']:.1f} rev/s²")
        print(f"  Velocity Gain: {best_result['vel_gain']:.3f}")
        print(f"  Velocity Integrator Gain: {best_result['vel_gain'] * 5:.3f}")
        print(f"\n  Expected Performance:")
        print(f"    - Average tracking error: {best_result['avg_error']:.1f} RPM")
        print(f"    - Maximum jerk: {best_result['max_jerk']:.2f} rev/s³")
        print(f"    - Smoothness score: {best_result['score']:.0f}/100")
        print("=" * 60)

        # Apply settings
        response = input("\nApply these optimal settings? (yes/no): ")
        if response.lower() == 'yes':
            axis.controller.config.vel_ramp_rate = best_result['ramp_rate']
            axis.controller.config.vel_gain = best_result['vel_gain']
            axis.controller.config.vel_integrator_gain = best_result['vel_gain'] * 5
            print("✓ Settings applied!")

            response = input("Save to ODrive permanently? (yes/no): ")
            if response.lower() == 'yes':
                try:
                    odrv = axis._dev
                    odrv.save_configuration()
                    print("✓ Saved! (ODrive rebooting...)")
                except:
                    print("✓ ODrive rebooted (saved)")
    else:
        print("\n⚠ Could not find optimal settings - all tests had high error")


def manual_tune(axis):
    """
    Manually adjust parameters and test
    """
    print("\n" + "=" * 60)
    print("MANUAL TUNING")
    print("=" * 60)

    print("\nCurrent settings:")
    print(f"  Velocity ramp rate: {axis.controller.config.vel_ramp_rate} rev/s²")
    print(f"  Velocity gain: {axis.controller.config.vel_gain}")
    print(f"  Velocity integrator gain: {axis.controller.config.vel_integrator_gain}")

    print("\nRecommendations:")
    print("  For maximum smoothness (lower jerk):")
    print("    - Lower ramp rate (5-10 rev/s²)")
    print("    - Lower vel_gain (0.005-0.01)")
    print("\n  For better tracking (lower error):")
    print("    - Higher ramp rate (15-25 rev/s²)")
    print("    - Higher vel_gain (0.02-0.04)")

    ramp_rate = float(input("\nEnter velocity ramp rate (rev/s²) [10]: ") or "10")
    vel_gain = float(input("Enter velocity gain [0.01]: ") or "0.01")

    test_velocity_smoothness(axis, ramp_rate, vel_gain, duration=12)


def demonstrate_ultra_smooth(axis):
    """
    Demonstrate ultra-smooth motion with optimized settings
    """
    print("\n" + "=" * 60)
    print("ULTRA-SMOOTH SINE WAVE DEMONSTRATION")
    print("=" * 60)

    # Apply ultra-smooth settings
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.vel_ramp_rate = 8.0  # Very gentle
    axis.controller.config.vel_gain = 0.008
    axis.controller.config.vel_integrator_gain = 0.04
    axis.controller.config.vel_limit = 15.0

    print("\nApplied ultra-smooth settings:")
    print("  Ramp rate: 8 rev/s² (very gentle acceleration)")
    print("  Vel gain: 0.008 (smooth response)")
    print("  Low frequency sine wave for fluid motion")

    response = input("\nRun demo? (yes/no): ")
    if response.lower() != 'yes':
        return

    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    if not axis.is_armed:
        print("Failed to arm")
        return

    print("\n✓ Motor armed - watch for butter-smooth motion!\n")

    start_time = time.time()
    duration = 20
    base_rpm = 50
    amplitude_rpm = 30
    frequency_hz = 0.15  # Very slow for maximum smoothness

    print("Time | Target RPM | Actual RPM | Smoothness")
    print("-" * 60)

    last_vel = axis.vel_estimate

    while (time.time() - start_time) < duration:
        elapsed = time.time() - start_time

        target_rpm = base_rpm + amplitude_rpm * math.sin(2 * math.pi * frequency_hz * elapsed)
        target_rps = target_rpm / 60.0
        axis.controller.input_vel = target_rps

        actual_rpm = axis.vel_estimate * 60
        accel = abs(axis.vel_estimate - last_vel) / 0.5

        smoothness = "Excellent" if accel < 1.0 else "Good" if accel < 2.0 else "Jerky"

        print(f"{elapsed:4.1f}s | {target_rpm:6.1f} RPM | {actual_rpm:6.1f} RPM | {smoothness}")

        last_vel = axis.vel_estimate
        time.sleep(0.5)

    axis.controller.input_vel = 0
    time.sleep(2)
    axis.requested_state = AXIS_STATE_IDLE

    print("\n✓ Ultra-smooth demo complete!")


def measure_backlash_bidirectional(axis, amplitude_deg=10, wavelength_deg=180, duration_per_direction=10):
    """
    Run bidirectional sine wave motion in position mode to measure backlash
    Records torque vs position for hysteresis curve

    amplitude_deg: amplitude of sine wave oscillations (degrees)
    wavelength_deg: wavelength of sine wave pattern (degrees)
    duration_per_direction: time for one direction (seconds)
    """
    import matplotlib.pyplot as plt
    import numpy as np

    # Convert to ODrive units (turns)
    amplitude = amplitude_deg / 360.0
    wavelength = wavelength_deg / 360.0

    print(f"\n{'=' * 60}")
    print(f"BACKLASH MEASUREMENT - BIDIRECTIONAL SINE WAVE")
    print(f"{'=' * 60}")
    print(f"Amplitude: {amplitude_deg}°")
    print(f"Wavelength: {wavelength_deg}°")
    print(f"Duration per direction: {duration_per_direction}s")
    print(f"{'=' * 60}\n")

    # Configure position control
    axis.controller.config.control_mode = 3  # POSITION_CONTROL
    axis.controller.config.input_mode = 5  # TRAP_TRAJ
    axis.trap_traj.config.vel_limit = 2.0  # turns/s
    axis.trap_traj.config.accel_limit = 5.0  # turns/s²

    # Enter closed loop
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    if not axis.is_armed:
        print("Failed to arm")
        return

    # Zero position
    axis.controller.input_pos = 0
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    axis.pos_estimate = 0
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)

    print("Starting bidirectional motion...\n")

    # Data storage
    forward_positions = []
    forward_torques = []
    backward_positions = []
    backward_torques = []

    # FORWARD: 0 to 360 degrees (0 to 1 turn)
    print("Phase 1: Moving forward (0° → 360°)")
    start_time = time.time()

    while (time.time() - start_time) < duration_per_direction:
        t = (time.time() - start_time) / duration_per_direction

        # Linear progression with sine wave overlay
        base_position = t  # 0 to 1 turn
        sine_overlay = amplitude * math.sin(2 * math.pi * t / wavelength)
        target_position = base_position + sine_overlay

        axis.controller.input_pos = target_position

        # Record data
        forward_positions.append(axis.pos_estimate * 360)  # Convert to degrees
        forward_torques.append(axis.torque_estimate)

        time.sleep(0.02)  # 50Hz sampling

    print("Phase 1 complete\n")
    time.sleep(0.5)

    # BACKWARD: 360 to 0 degrees (1 to 0 turn)
    print("Phase 2: Moving backward (360° → 0°)")
    start_time = time.time()

    while (time.time() - start_time) < duration_per_direction:
        t = (time.time() - start_time) / duration_per_direction

        # Linear progression with sine wave overlay
        base_position = 1 - t  # 1 to 0 turn
        sine_overlay = amplitude * math.sin(2 * math.pi * t / wavelength)
        target_position = base_position + sine_overlay

        axis.controller.input_pos = target_position

        # Record data
        backward_positions.append(axis.pos_estimate * 360)  # Convert to degrees
        backward_torques.append(axis.torque_estimate)

        time.sleep(0.02)  # 50Hz sampling

    print("Phase 2 complete\n")

    # Stop
    axis.controller.input_pos = 0
    time.sleep(1)
    axis.requested_state = AXIS_STATE_IDLE

    # Plot backlash hysteresis curve
    plt.figure(figsize=(10, 8))
    plt.plot(forward_positions, forward_torques, 'b-', label='Forward', linewidth=2)
    plt.plot(backward_positions, backward_torques, 'r-', label='Backward', linewidth=2)
    plt.xlabel('Position (degrees)', fontsize=12)
    plt.ylabel('Torque (Nm)', fontsize=12)
    plt.title('Backlash Hysteresis Curve', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=11)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='--', alpha=0.3)

    # Calculate backlash metrics
    max_torque = max(max(forward_torques), max(backward_torques))
    min_torque = min(min(forward_torques), min(backward_torques))

    print(f"{'=' * 60}")
    print(f"BACKLASH ANALYSIS")
    print(f"{'=' * 60}")
    print(f"Torque range: {min_torque:.3f} to {max_torque:.3f} Nm")
    print(f"Position range: {min(forward_positions):.2f}° to {max(forward_positions):.2f}°")
    print(f"Data points collected: {len(forward_positions) + len(backward_positions)}")
    print(f"{'=' * 60}\n")

    plt.tight_layout()
    plt.show()

    return {
        'forward_positions': forward_positions,
        'forward_torques': forward_torques,
        'backward_positions': backward_positions,
        'backward_torques': backward_torques
    }


def break_in_actuator(axis, num_cycles=20, velocity_rps=0.3, time_per_direction=10):
    """
    Run repeated bidirectional cycles at constant velocity to break in actuator

    num_cycles: number of complete back-and-forth cycles
    velocity_rps: constant velocity in turns/s (revolutions per second)
    time_per_direction: time spent moving in each direction (seconds)
    """
    print(f"\n{'=' * 60}")
    print(f"ACTUATOR BREAK-IN PROCEDURE - VELOCITY MODE")
    print(f"{'=' * 60}")
    print(f"Cycles: {num_cycles}")
    print(f"Velocity: {velocity_rps} turns/s ({velocity_rps * 60:.1f} RPM)")
    print(f"Time per direction: {time_per_direction}s")
    print(f"Cycle duration: {time_per_direction * 2}s")
    print(f"Total time: {num_cycles * time_per_direction * 2 / 60:.1f} minutes")
    print(f"{'=' * 60}\n")

    response = input("Start break-in? (yes/no): ")
    if response.lower() != 'yes':
        return

    # Configure velocity control
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.vel_ramp_rate = 5.0  # turns/s² (gentle ramp)
    axis.controller.config.vel_limit = max(20.0, velocity_rps * 1.5)  # Set higher than commanded

    print(f"\nVel limit set to: {axis.controller.config.vel_limit} turns/s")
    print(f"Commanded velocity: {velocity_rps} turns/s ({velocity_rps * 60:.1f} RPM)\n")

    # Enter closed loop
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    if not axis.is_armed:
        print("Failed to arm")
        return

    print("Starting break-in cycles...\n")
    print(f"{'Cycle':<8} {'Direction':<12} {'Cmd (RPM)':<12} {'Act (RPM)':<12} {'Time Left'}")
    print("-" * 70)

    total_start = time.time()
    sample_count = 0

    for cycle in range(num_cycles):
        # FORWARD
        axis.controller.input_vel = velocity_rps
        time.sleep(1.0)  # Allow ramp up

        start_time = time.time()

        while (time.time() - start_time) < time_per_direction:
            actual_vel = axis.vel_estimate
            cmd_vel = axis.controller.input_vel
            remaining_total = (num_cycles - cycle) * time_per_direction * 2 - (time.time() - start_time)

            if sample_count % 2 == 0:  # Print every second
                print(
                    f"{cycle + 1:<8} {'Forward':<12} {cmd_vel * 60:<12.1f} {actual_vel * 60:<12.1f} {remaining_total / 60:<.1f} min")

            sample_count += 1
            time.sleep(0.5)

        # BACKWARD
        axis.controller.input_vel = -velocity_rps
        time.sleep(1.0)  # Allow direction change

        start_time = time.time()

        while (time.time() - start_time) < time_per_direction:
            actual_vel = axis.vel_estimate
            cmd_vel = axis.controller.input_vel
            remaining_total = (num_cycles - cycle - 1) * time_per_direction * 2 + (
                        time_per_direction - (time.time() - start_time))

            if sample_count % 2 == 0:
                print(
                    f"{cycle + 1:<8} {'Backward':<12} {cmd_vel * 60:<12.1f} {actual_vel * 60:<12.1f} {remaining_total / 60:<.1f} min")

            sample_count += 1
            time.sleep(0.5)

    # Stop
    axis.controller.input_vel = 0
    time.sleep(2)
    axis.requested_state = AXIS_STATE_IDLE

    total_time = time.time() - total_start

    print(f"\n{'=' * 60}")
    print(f"BREAK-IN COMPLETE")
    print(f"{'=' * 60}")
    print(f"Total cycles completed: {num_cycles}")
    print(f"Total runtime: {total_time / 60:.1f} minutes")
    print(f"Actuator should now be settled and ready for characterization")
    print(f"{'=' * 60}\n")

def main():
    try:
        odrv = find_odrive()
        axis = odrv.axis0

        if not axis.commutation_mapper.config.offset_valid:
            print("Encoder not calibrated!")
            return

        # Calibrate if needed
        if axis.current_state == AXIS_STATE_IDLE:
            print("Running motor calibration...")
            axis.requested_state = 4
            while axis.current_state == 4:
                time.sleep(0.1)
            print("✓ Calibration complete\n")

        print("=" * 60)
        print("ULTRA-SMOOTH MOTION TUNING TOOL")
        print("=" * 60)
        print("\n1. Auto-tune (find optimal settings)")
        print("2. Manual tune (test custom settings)")
        print("3. Ultra-smooth demo (pre-optimized)")
        print("4. Backlash measurement (bidirectional sine wave)")
        print("5. Break-in procedure (repeated cycles)")
        print("=" * 60)

        choice = input("\nEnter choice (1-5): ")

        if choice == '1':
            auto_tune_smoothness(axis)
        elif choice == '2':
            manual_tune(axis)
        elif choice == '3':
            demonstrate_ultra_smooth(axis)
        elif choice == '4':
            amp = float(input("Amplitude (degrees) [10]: ") or "10")
            wl = float(input("Wavelength (degrees) [180]: ") or "180")
            dur = float(input("Duration per direction (seconds) [10]: ") or "10")
            measure_backlash_bidirectional(axis, amp, wl, dur)
        elif choice == '5':
            cycles = 150
            velocity = 15
            time_dir = 5
            break_in_actuator(axis, cycles, velocity, time_dir)
        else:
            print("Invalid choice")

    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted!")
        if 'axis' in locals():
            try:
                axis.controller.input_vel = 0
                axis.requested_state = AXIS_STATE_IDLE
            except:
                pass
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()