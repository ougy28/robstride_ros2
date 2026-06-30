#!/usr/bin/env python3
"""
pp_control_node.py
------------------
Profile-Position (PP, Mode 1) motor control — no ROS topics.
The motor uses its internal motion profiler to move smoothly between
positions; the node sends target positions and displays live feedback.

Edit the constants below, then run:

    ros2 run rob_py pp_control_node
"""

import signal
import threading
import time

import rclpy

from rob_py.can_setup import setup_can_interface
from robstride_dynamics import RobstrideBus, Motor

# ── Configuration ─────────────────────────────────────────────────────────────
CAN_CHANNEL          = 'can0'
BITRATE              = 1_000_000   # bps
MOTOR_ID             = 3
MOTOR_MODEL          = 'rs-06'
PP_VELOCITY_MAX      = 10.0        # rad/s  — profile speed limit
PP_ACCELERATION      = 5.0        # rad/s² — profile acceleration
TORQUE_LIMIT         = 2.0         # Nm     — max torque during travel
FEEDBACK_RATE_HZ     = 50          # status-poll frequency
# Move the joint to its mechanical centre, read the RAW encoder value,
# and enter it here. This shifts logical zero to that encoder position.
# Tip: Set this to ~π (3.14) to center your ±1.57 rad range and avoid wrapping.
HOMING_OFFSET        = 3.14        # rad — raw encoder reading at joint zero
# Soft limits in logical (post-offset) space. Commands are clamped here.
MIN_POS              = -3.14       # rad  (~-180°)
MAX_POS              =  3.14       # rad  (~ 180°)
# ──────────────────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)

    if not setup_can_interface(CAN_CHANNEL, BITRATE):
        print(f"[ERROR] Could not bring up CAN interface '{CAN_CHANNEL}'. Exiting.")
        rclpy.shutdown()
        return

    motor_name  = f'motor_{MOTOR_ID}'
    motors      = {motor_name: Motor(id=MOTOR_ID, model=MOTOR_MODEL)}
    calibration = {motor_name: {'direction': 1, 'homing_offset': HOMING_OFFSET}}

    bus = RobstrideBus(CAN_CHANNEL, motors, calibration)
    bus.connect(handshake=True)

    # Ensure motor is disabled before mode switch
    try:
        bus.disable(motor_name)
    except Exception:
        pass
    time.sleep(0.3)

    bus.set_pp_mode(
        motor_name,
        vel_max=PP_VELOCITY_MAX,
        acceleration=PP_ACCELERATION,
        torque_limit=TORQUE_LIMIT,
    )

    print(f"[INFO] PP mode active — motor {MOTOR_ID} on {CAN_CHANNEL}")
    print(f"       vel_max={PP_VELOCITY_MAX} rad/s  acc={PP_ACCELERATION} rad/s²  "
          f"torque_limit={TORQUE_LIMIT} Nm")

    # Read current position to avoid sudden movement on startup
    try:
        import math
        pos, vel, trq, temp = bus.control_pp(motor_name, 0.0)  # dummy read
        
        # Calculate the multi-turn offset: how many full rotations the motor thinks it did
        # We'll add this offset to all user commands to keep them in the motor's frame
        TWO_PI = 2 * math.pi
        multiturn_offset = TWO_PI * round((pos) / TWO_PI)
        
        # Display unwrapped position to user — normalised to [-π, π]
        display_pos = (pos - multiturn_offset + math.pi) % (2 * math.pi) - math.pi
        
        # Hold current position (in motor's coordinate frame)
        target_position = pos
        
        print(f"[INFO] Starting position: {display_pos:.4f} rad — holding until commanded.")
        if abs(multiturn_offset) > 0.01:
            print(f"       (multi-turn offset: {multiturn_offset/TWO_PI:.1f} revolutions)")
    except Exception as exc:
        print(f"[WARN] Could not read initial position: {exc}")
        target_position = 0.0   # fallback
        multiturn_offset = 0.0

    running = True

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    def _input_thread():
        nonlocal target_position, running
        print("Enter target position in rad (or 'q' to quit):")
        while running:
            try:
                line = input("> ").strip()
                if line.lower() == 'q':
                    running = False
                    break
                raw = float(line)
                clamped = max(MIN_POS, min(MAX_POS, raw))
                if clamped != raw:
                    print(f"  [WARN] {raw:.4f} rad out of limits [{MIN_POS:.4f}, {MAX_POS:.4f}] — clamped to {clamped:.4f} rad")
                # Add multi-turn offset so command is in motor's coordinate frame
                target_position = clamped + multiturn_offset
                print(f"  → target set to {clamped:.4f} rad")
            except ValueError:
                print("  Invalid input. Enter a number in radians.")
            except EOFError:
                running = False
                break

    input_t = threading.Thread(target=_input_thread, daemon=True)
    input_t.start()

    dt = 1.0 / FEEDBACK_RATE_HZ

    while running:
        try:
            # Continuously write the position target; the motor holds position if
            # the target is unchanged, and the write always returns a status frame.
            pos, vel, trq, temp = bus.control_pp(motor_name, target_position)
            # Display unwrapped position to user — normalised to [-π, π]
            display_pos = (pos - multiturn_offset + math.pi) % (2 * math.pi) - math.pi
            print(
                f"\r  pos={display_pos:+.4f} rad  vel={vel:+.4f} rad/s  "
                f"trq={trq:+.4f} Nm  temp={temp:.1f}°C   ",
                end='', flush=True,
            )
        except Exception as exc:
            print(f"\n[WARN] Loop error: {exc}")

        time.sleep(dt)

    print("\n[INFO] Current position:")
    try:
        pos, vel, trq, temp = bus.control_pp(motor_name, target_position)
        print(f"  pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  trq={trq:+.4f} Nm  temp={temp:.1f}°C")
    except Exception as exc:
        print(f"  Could not read position — {exc}")

    try:
        confirm = input("\nReturn motor to zero position? [y/N]: ").strip().lower()
    except EOFError:
        confirm = 'n'

    try:
        if confirm == 'y':
            print("[INFO] Returning to home position ...")
            bus.control_pp(motor_name, 0.0)
            time.sleep(0.8)
        else:
            print("[INFO] Skipping home — motor left at current position.")
        bus.disable(motor_name)
        bus.disconnect()
    except Exception:
        pass

    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == '__main__':
    main()
