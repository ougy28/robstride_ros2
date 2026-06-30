#!/usr/bin/env python3
"""
7_dof_v1.py
-----------
Profile-Position (PP, Mode 1) control for RS-04 motors (IDs 1, 2) and RS-06
motor (ID 3). Commands all motors independently; each uses its own multi-turn
offset so displayed positions stay in [-1.57, 1.57] rad.

Run:
    ros2 run rob_py 7_dof_v1
"""

import signal
import threading
import time
import math

import rclpy

from rob_py.can_setup import setup_can_interface
from robstride_dynamics import RobstrideBus, Motor

# ── Configuration ─────────────────────────────────────────────────────────────
CAN_CHANNEL     = 'can0'
BITRATE         = 1_000_000   # bps

# Per-motor settings — extend the dicts to add more motors
MOTOR_IDS       = [1, 2, 3, 7]
MOTOR_MODELS    = {
    1: 'rs-04',
    2: 'rs-04',
    3: 'rs-06',
    7: 'rs-05',
}
HOMING_OFFSETS  = {
    1: 3.14,   # raw encoder reading at joint zero for motor 1
    2: 3.14,   # raw encoder reading at joint zero for motor 2
    3: 3.14,   # raw encoder reading at joint zero for motor 3
    7: 3.14,   # raw encoder reading at joint zero for motor 7
}
PP_VELOCITY_MAX = {
    1: 10.0,   # rad/s
    2: 10.0,
    3: 10.0,
    7: 20.0,
}
PP_ACCELERATION = {
    1:  5.0,   # rad/s²
    2:  5.0,
    3:  5.0,
    7: 10.0,
}
TORQUE_LIMIT = {
    1: 2.0,   # Nm
    2: 2.0,
    3: 2.0,
    7: 2.0,
}
FEEDBACK_RATE_HZ = 50

MIN_POS = -3.14   # rad  (~-180°)
MAX_POS  =  3.14  # rad  (~ 180°)
# ──────────────────────────────────────────────────────────────────────────────

TWO_PI = 2 * math.pi


def main(args=None):
    rclpy.init(args=args)

    if not setup_can_interface(CAN_CHANNEL, BITRATE):
        print(f"[ERROR] Could not bring up CAN interface '{CAN_CHANNEL}'. Exiting.")
        rclpy.shutdown()
        return

    motors = {
        f'motor_{mid}': Motor(id=mid, model=MOTOR_MODELS[mid])
        for mid in MOTOR_IDS
    }
    calibration = {
        f'motor_{mid}': {'direction': 1, 'homing_offset': HOMING_OFFSETS[mid]}
        for mid in MOTOR_IDS
    }

    bus = RobstrideBus(CAN_CHANNEL, motors, calibration)
    bus.connect(handshake=True)

    # Disable all motors before mode switch
    for mid in MOTOR_IDS:
        try:
            bus.disable(f'motor_{mid}')
        except Exception:
            pass
    time.sleep(0.3)

    active_motors = []
    for mid in MOTOR_IDS:
        try:
            bus.set_pp_mode(
                f'motor_{mid}',
                vel_max=PP_VELOCITY_MAX[mid],
                acceleration=PP_ACCELERATION[mid],
                torque_limit=TORQUE_LIMIT[mid],
            )
            active_motors.append(mid)
            print(f"[INFO] PP mode active — motor {mid} ({MOTOR_MODELS[mid]}) on {CAN_CHANNEL}  "
                  f"vel_max={PP_VELOCITY_MAX[mid]} rad/s  acc={PP_ACCELERATION[mid]} rad/s²  "
                  f"torque_limit={TORQUE_LIMIT[mid]} Nm")
        except Exception as exc:
            print(f"[WARN] Motor {mid} not available, skipping: {exc}")

    # Read initial positions and compute multi-turn offsets
    target_positions   = {}
    multiturn_offsets  = {}

    for mid in active_motors:
        name = f'motor_{mid}'
        try:
            pos, vel, trq, temp = bus.control_pp(name, 0.0)
            offset = TWO_PI * round(pos / TWO_PI)
            multiturn_offsets[mid]  = offset
            target_positions[mid]   = pos          # hold current position
            display_pos = pos - offset
            display_pos = (display_pos + math.pi) % (2 * math.pi) - math.pi
            print(f"[INFO] Motor {mid} starting position: {display_pos:.4f} rad — holding.")
            if abs(offset) > 0.01:
                print(f"       (multi-turn offset: {offset / TWO_PI:.1f} revolutions)")
        except Exception as exc:
            print(f"[WARN] Could not read initial position for motor {mid}: {exc}")
            multiturn_offsets[mid]  = 0.0
            target_positions[mid]   = 0.0

    running = True

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    def _input_thread():
        nonlocal running
        print(f"\nEnter commands as:  <motor_id> <position_rad>  (e.g. '1 0.5')")
        print(f"Active motors: {active_motors}  |  'q' to quit.\n")
        while running:
            try:
                line = input("> ").strip()
                if line.lower() == 'q':
                    running = False
                    break
                parts = line.split()
                if len(parts) != 2:
                    print("  Usage: <motor_id> <position_rad>   e.g.  1 0.5")
                    continue
                mid  = int(parts[0])
                raw  = float(parts[1])
                if mid not in active_motors:
                    print(f"  Unknown motor id {mid}. Valid ids: {MOTOR_IDS}")
                    continue
                clamped = max(MIN_POS, min(MAX_POS, raw))
                if clamped != raw:
                    print(f"  [WARN] {raw:.4f} rad clamped to [{MIN_POS:.4f}, {MAX_POS:.4f}]")
                target_positions[mid] = clamped + multiturn_offsets[mid]
                print(f"  → motor {mid} target set to {clamped:.4f} rad")
            except ValueError:
                print("  Invalid input. Enter: <int motor_id> <float position_rad>")
            except EOFError:
                running = False
                break

    input_t = threading.Thread(target=_input_thread, daemon=True)
    input_t.start()

    dt = 1.0 / FEEDBACK_RATE_HZ

    while running:
        parts = []
        for mid in active_motors:
            name = f'motor_{mid}'
            try:
                pos, vel, trq, temp = bus.control_pp(name, target_positions[mid])
                display_pos = pos - multiturn_offsets[mid]
                display_pos = (display_pos + math.pi) % (2 * math.pi) - math.pi
                parts.append(
                    f"M{mid}: pos={display_pos:+.4f} vel={vel:+.4f} trq={trq:+.4f} {temp:.1f}°C"
                )
            except Exception as exc:
                parts.append(f"M{mid}: ERROR({exc})")
        print(f"\r  {'   |   '.join(parts)}   ", end='', flush=True)
        time.sleep(dt)

    print("\n[INFO] Final positions:")
    for mid in active_motors:
        name = f'motor_{mid}'
        try:
            pos, vel, trq, temp = bus.control_pp(name, target_positions[mid])
            display_pos = pos - multiturn_offsets[mid]
            display_pos = (display_pos + math.pi) % (2 * math.pi) - math.pi
            print(f"  Motor {mid}: pos={display_pos:+.4f} rad  vel={vel:+.4f}  trq={trq:+.4f}  temp={temp:.1f}°C")
        except Exception as exc:
            print(f"  Motor {mid}: Could not read — {exc}")

    try:
        confirm = input("\nReturn all motors to zero position? [y/N]: ").strip().lower()
    except EOFError:
        confirm = 'n'

    for mid in active_motors:
        name = f'motor_{mid}'
        try:
            if confirm == 'y':
                bus.control_pp(name, multiturn_offsets[mid])   # logical zero
            bus.disable(name)
        except Exception:
            pass

    if confirm == 'y':
        print("[INFO] Returning to home — waiting 0.8 s ...")
        time.sleep(0.8)

    try:
        bus.disconnect()
    except Exception:
        pass

    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == '__main__':
    main()
