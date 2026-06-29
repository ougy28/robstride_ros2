#!/usr/bin/env python3
"""
7_dof_v1.py
-----------
Profile-Position (PP, Mode 1) control for a 7-DOF arm — no ROS topics.
All motors are initialised on the same CAN bus and set to PP mode.
Type 'm <id>' to switch which motor you are commanding, then enter a
target position in radians.

Run:
    ros2 run rob_py 7_dof_v1
"""

import signal
import threading
import time

import rclpy

from rob_py.can_setup import setup_can_interface
from robstride_dynamics import RobstrideBus, Motor

# ── CAN bus ───────────────────────────────────────────────────────────────────
CAN_CHANNEL      = 'can0'
BITRATE          = 1_000_000   # bps
FEEDBACK_RATE_HZ = 50

# ── Motor table ──────────────────────────────────────────────────────────────
# id → (model, vel_max rad/s, accel rad/s², torque_limit Nm,
#        homing_offset rad,   min_pos rad,  max_pos rad)
#
# homing_offset: raw encoder reading when the joint is at its physical zero.
#   Move each joint to its mechanical centre, read pos from the feedback line,
#   and enter that value here. This shifts the 0/2π wrap-point away from
#   the joint's travel range.
#
# min_pos / max_pos: soft limits in logical (post-offset) space.
#   Commands outside this range are clamped with a warning.
# ─────────────────────────────────────────────────────────────────────────────
MOTOR_TABLE = {
    #  id   model    vel    acc  torque  offset  min     max
    1: ('rs-04', 20.0, 10.0, 2.0,  0.0, -1.57,  1.57),
    2: ('rs-04', 20.0, 10.0, 2.0,  0.0, -1.57,  1.57),
    3: ('rs-06', 20.0, 10.0, 4.0,  0.0, -1.57,  1.57),
    4: ('rs-06', 20.0, 10.0, 4.0,  0.0, -1.57,  1.57),
    5: ('rs-02', 20.0, 10.0, 1.0,  0.0, -1.57,  1.57),
    6: ('rs-00', 20.0, 10.0, 1.0,  0.0, -1.57,  1.57),
    7: ('rs-05', 20.0, 10.0, 3.0,  0.0, -1.57,  1.57),
}
# ──────────────────────────────────────────────────────────────────────────────


def motor_name(mid: int) -> str:
    return f'motor_{mid}'


def main(args=None):
    rclpy.init(args=args)

    if not setup_can_interface(CAN_CHANNEL, BITRATE):
        print(f"[ERROR] Could not bring up CAN interface '{CAN_CHANNEL}'. Exiting.")
        rclpy.shutdown()
        return

    motors      = {motor_name(mid): Motor(id=mid, model=cfg[0])
                   for mid, cfg in MOTOR_TABLE.items()}
    calibration = {motor_name(mid): {'direction': 1, 'homing_offset': cfg[4]}
                   for mid, cfg in MOTOR_TABLE.items()}

    bus = RobstrideBus(CAN_CHANNEL, motors, calibration)
    bus.connect(handshake=True)

    # Disable all motors, then switch each to PP mode
    for mid, (model, vel_max, accel, torque_lim, offset, min_pos, max_pos) in MOTOR_TABLE.items():
        name = motor_name(mid)
        try:
            bus.disable(name)
        except Exception:
            pass
    time.sleep(0.3)

    active_motors = []
    for mid, (model, vel_max, accel, torque_lim, offset, min_pos, max_pos) in MOTOR_TABLE.items():
        name = motor_name(mid)
        try:
            bus.set_pp_mode(name, vel_max=vel_max, acceleration=accel, torque_limit=torque_lim)
            active_motors.append(mid)
            print(f"[INFO] Motor {mid} ({model}) — PP mode  "
                  f"vel_max={vel_max} rad/s  acc={accel} rad/s²  torque_limit={torque_lim} Nm")
        except Exception as exc:
            print(f"[WARN] Motor {mid} ({model}) skipped — {exc}")

    if not active_motors:
        print("[ERROR] No motors responded. Exiting.")
        bus.disconnect()
        rclpy.shutdown()
        return

    print(f"[INFO] Active motors: {active_motors}")

    running          = True
    active_id        = active_motors[0]
    target_positions = {mid: 0.0 for mid in active_motors}

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    def _input_thread():
        nonlocal active_id, running
        print("\nCommands:")
        print("  m <id>   — switch active motor (e.g. 'm 3')")
        print("  <rad>    — set target position for active motor")
        print("  q        — quit\n")
        while running:
            try:
                line = input(f"[motor_{active_id}]> ").strip()
                if not line:
                    continue
                if line.lower() == 'q':
                    running = False
                    break
                if line.lower().startswith('m '):
                    try:
                        new_id = int(line.split()[1])
                        if new_id in active_motors:
                            active_id = new_id
                            print(f"  → active motor switched to {active_id} "
                                  f"({MOTOR_TABLE[active_id][0]})")
                        else:
                            print(f"  Unknown motor id. Active motors: {active_motors}")
                    except (IndexError, ValueError):
                        print("  Usage: m <id>   e.g. m 3")
                else:
                    raw = float(line)
                    cfg = MOTOR_TABLE[active_id]
                    min_pos, max_pos = cfg[5], cfg[6]
                    clamped = max(min_pos, min(max_pos, raw))
                    if clamped != raw:
                        print(f"  [WARN] {raw:.4f} rad out of limits [{min_pos:.4f}, {max_pos:.4f}] — clamped to {clamped:.4f} rad")
                    target_positions[active_id] = clamped
                    print(f"  → motor_{active_id} target = {clamped:.4f} rad")
            except ValueError:
                print("  Invalid input. Enter a number in radians, or 'm <id>' to switch motor.")
            except EOFError:
                running = False
                break

    input_t = threading.Thread(target=_input_thread, daemon=True)
    input_t.start()

    dt = 1.0 / FEEDBACK_RATE_HZ

    while running:
        name = motor_name(active_id)
        try:
            pos, vel, trq, temp = bus.control_pp(name, target_positions[active_id])
            print(
                f"\r[motor_{active_id}]  pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  "
                f"trq={trq:+.4f} Nm  temp={temp:.1f}°C   ",
                end='', flush=True,
            )
        except Exception as exc:
            print(f"\n[WARN] Loop error: {exc}")

        time.sleep(dt)

    print("\n[INFO] Current motor positions:")
    for mid in active_motors:
        name = motor_name(mid)
        try:
            pos, vel, trq, temp = bus.control_pp(name, target_positions[mid])
            print(f"  motor_{mid} ({MOTOR_TABLE[mid][0]}):  pos={pos:+.4f} rad  "
                  f"vel={vel:+.4f} rad/s  trq={trq:+.4f} Nm  temp={temp:.1f}°C")
        except Exception as exc:
            print(f"  motor_{mid}: could not read — {exc}")

    try:
        confirm = input("\nReturn all motors to zero position? [y/N]: ").strip().lower()
    except EOFError:
        confirm = 'n'

    if confirm == 'y':
        print("[INFO] Returning all motors to home position ...")
        for mid in active_motors:
            name = motor_name(mid)
            try:
                bus.control_pp(name, 0.0)
            except Exception:
                pass
    else:
        print("[INFO] Skipping home — motors left at current positions.")
    time.sleep(0.8)
    for mid in active_motors:
        name = motor_name(mid)
        try:
            bus.disable(name)
        except Exception:
            pass
    try:
        bus.disconnect()
    except Exception:
        pass

    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == '__main__':
    main()
