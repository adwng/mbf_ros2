"""Verify the GIM6010-8 (GDS68) torque convention over CAN.

Goal: confirm whether the firmware's torque path is rotor-referenced (so the
hardware interface is correct to divide commanded torque by reduction_ratio=8)
and whether torque_constant is set consistently.

What it does, for one node id, in torque-control closed loop:
  1. Commands a series of known Input_Torque values (Set_Input_Torque, 0x00E).
  2. Reads back Iq via Get_Iq (0x014) and torque via Get_Torques (0x01C).
  3. Prints expected vs measured Iq, so you can see:
        - implied torque_constant = Input_Torque / Iq_setpoint
          (should match what you flashed, e.g. 0.0588)
        - Iq_measured vs Iq_setpoint (if measured << setpoint you are
          voltage/current limited -> battery/headroom problem)
  4. (Optional) If you physically measure the OUTPUT-shaft torque for the last
     commanded value (lever arm x scale), it computes the true output Kt and
     tells you whether Input_Torque is rotor- or output-referenced.

SAFETY:
  * Clamp/hold the output shaft. In torque mode a free shaft will spin up.
  * Start with small torques. Defaults are intentionally low.
  * The motor must already be calibrated (run odrive_calibrate.py first).

Requires python-can:  pip install python-can
Bring up the bus first, e.g.:
    sudo ip link set can0 up type can bitrate 1000000
"""

import argparse
import struct
import time

import can


# CAN-Simple command ids (see robot_hardware_interface / manual section 4.1.2)
CMD_SET_AXIS_STATE = 0x007
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_TORQUE = 0x00E
CMD_GET_IQ = 0x014
CMD_GET_BUS_VI = 0x017
CMD_CLEAR_ERRORS = 0x018
CMD_GET_TORQUES = 0x01C

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE_TORQUE = 1
INPUT_MODE_PASSTHROUGH = 1


def arb_id(node_id: int, cmd_id: int) -> int:
    return (node_id << 5) | cmd_id


def send(bus, node_id, cmd_id, data=b""):
    bus.send(can.Message(arbitration_id=arb_id(node_id, cmd_id),
                         is_extended_id=False, data=data))


def request(bus, node_id, cmd_id):
    """Send a Remote Transmission Request for a motor->host message."""
    bus.send(can.Message(arbitration_id=arb_id(node_id, cmd_id),
                         is_extended_id=False, is_remote_frame=True, dlc=8))


def collect(bus, node_id, want_cmds, window_s=0.25):
    """Drain frames for window_s seconds, return latest payload per cmd_id."""
    out = {}
    end = time.time() + window_s
    while time.time() < end:
        msg = bus.recv(timeout=window_s)
        if msg is None or msg.is_remote_frame:
            continue
        if (msg.arbitration_id >> 5) != node_id:
            continue
        cmd = msg.arbitration_id & 0x1F
        if cmd in want_cmds:
            out[cmd] = bytes(msg.data)
    return out


def read_iq_torque_vbus(bus, node_id):
    request(bus, node_id, CMD_GET_IQ)
    request(bus, node_id, CMD_GET_TORQUES)
    request(bus, node_id, CMD_GET_BUS_VI)
    frames = collect(bus, node_id,
                     {CMD_GET_IQ, CMD_GET_TORQUES, CMD_GET_BUS_VI})
    res = {}
    if CMD_GET_IQ in frames:
        iq_sp, iq_meas = struct.unpack("<ff", frames[CMD_GET_IQ][:8])
        res["iq_setpoint"], res["iq_measured"] = iq_sp, iq_meas
    if CMD_GET_TORQUES in frames:
        t_sp, t = struct.unpack("<ff", frames[CMD_GET_TORQUES][:8])
        res["torque_setpoint"], res["torque"] = t_sp, t
    if CMD_GET_BUS_VI in frames:
        vbus, ibus = struct.unpack("<ff", frames[CMD_GET_BUS_VI][:8])
        res["vbus"], res["ibus"] = vbus, ibus
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--node-id", type=int, required=True)
    ap.add_argument("--torque-constant", type=float, default=0.0588,
                    help="value you flashed (rotor Kt). default 0.47/8")
    ap.add_argument("--torques", default="0.1,0.2,0.3",
                    help="comma list of Input_Torque values to command "
                         "(rotor Nm, as sent on the wire)")
    ap.add_argument("--reduction", type=float, default=8.0,
                    help="gearbox ratio used by the hardware interface")
    ap.add_argument("--hold-time", type=float, default=1.0,
                    help="seconds to hold each torque before sampling")
    ap.add_argument("--measured-output-torque", type=float, default=None,
                    help="OPTIONAL: physically measured OUTPUT-shaft torque "
                         "[Nm] for the LAST commanded value (lever x scale). "
                         "Resolves rotor- vs output-referenced.")
    args = ap.parse_args()

    torques = [float(x) for x in args.torques.split(",")]

    print(f"Opening {args.interface} for node {args.node_id} ...")
    bus = can.interface.Bus(channel=args.interface, interface="socketcan")
    try:
        print("Clearing errors, setting torque control + closed loop...")
        send(bus, args.node_id, CMD_CLEAR_ERRORS)
        send(bus, args.node_id, CMD_SET_CONTROLLER_MODE,
             struct.pack("<II", CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH))
        send(bus, args.node_id, CMD_SET_AXIS_STATE,
             struct.pack("<I", AXIS_STATE_CLOSED_LOOP))
        time.sleep(0.5)

        print("\n  cmd_T[Nm]   Iq_sp[A]  Iq_meas[A]  implied_Kt   T_rep[Nm]  "
              "vbus[V]  ibus[A]")
        print("  " + "-" * 76)

        last = {}
        for t_cmd in torques:
            send(bus, args.node_id, CMD_SET_INPUT_TORQUE,
                 struct.pack("<f", t_cmd))
            time.sleep(args.hold_time)
            r = read_iq_torque_vbus(bus, args.node_id)
            last = dict(r, t_cmd=t_cmd)

            iq_sp = r.get("iq_setpoint", float("nan"))
            iq_meas = r.get("iq_measured", float("nan"))
            implied_kt = (t_cmd / iq_sp) if iq_sp else float("nan")
            print(f"  {t_cmd:8.3f}  {iq_sp:9.3f} {iq_meas:11.3f}  "
                  f"{implied_kt:10.4f}  {r.get('torque', float('nan')):9.3f}  "
                  f"{r.get('vbus', float('nan')):7.2f}  "
                  f"{r.get('ibus', float('nan')):6.2f}")
    finally:
        send(bus, args.node_id, CMD_SET_INPUT_TORQUE, struct.pack("<f", 0.0))
        send(bus, args.node_id, CMD_SET_AXIS_STATE,
             struct.pack("<I", AXIS_STATE_IDLE))
        print("\nMotor set back to IDLE.")

    print("\n--- Interpretation ---")
    print("* 'implied_Kt' should match --torque-constant; that just confirms "
          "the firmware computes Iq = Input_Torque / torque_constant and that "
          "your flashed value is active.")
    print("* If Iq_meas is much smaller than Iq_sp, the current loop is "
          "saturating -> voltage/current headroom problem (battery).")

    if args.measured_output_torque is not None and last.get("iq_measured"):
        iq = last["iq_measured"]
        t_cmd = last["t_cmd"]
        true_output_kt = args.measured_output_torque / iq if iq else float("nan")
        ratio = args.measured_output_torque / t_cmd if t_cmd else float("nan")
        print(f"\n* Physical check (last cmd T={t_cmd:.3f} Nm, "
              f"Iq_meas={iq:.3f} A, measured output torque="
              f"{args.measured_output_torque:.3f} Nm):")
        print(f"    true OUTPUT torque constant ~= {true_output_kt:.4f} Nm/A "
              f"(datasheet 0.47)")
        print(f"    measured_output / commanded = {ratio:.2f}")
        if abs(ratio - args.reduction) < args.reduction * 0.35:
            print(f"    => ratio ~= reduction ({args.reduction:g}). "
                  "Input_Torque is ROTOR-referenced: dividing the joint "
                  "torque by reduction_ratio in the C++ is CORRECT, and "
                  "torque_constant should be the rotor value "
                  "(~0.47/8 = 0.0588).")
        elif abs(ratio - 1.0) < 0.35:
            print("    => ratio ~= 1. Input_Torque is OUTPUT-referenced: do "
                  "NOT divide by reduction_ratio in the C++; keep "
                  "torque_constant = 0.47.")
        else:
            print("    => inconclusive; re-check the lever arm / scale and "
                  "make sure the shaft was fully stalled.")


if __name__ == "__main__":
    main()
