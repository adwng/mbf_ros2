import odrive
import time
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors


# ==============================
# Connect
# ==============================

print("Connecting to ODrive...")
odrv0 = odrive.find_any()
axis = odrv0.axis0

dump_errors(odrv0)
odrv0.clear_errors()

axis.requested_state = AxisState.IDLE
print("Axis set to IDLE.\n")

current_mode = None
position_offset = 0.0


# ==============================
# Mode Switching
# ==============================

def set_mode(mode):
    global current_mode

    if mode == "p":
        axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
        axis.controller.config.input_mode = InputMode.POS_FILTER
        current_mode = "position"
        print("Switched to POSITION control\n")

    elif mode == "v":
        axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        axis.controller.config.input_mode = InputMode.PASSTHROUGH
        current_mode = "velocity"
        print("Switched to VELOCITY control\n")

    elif mode == "t":
        axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        axis.controller.config.input_mode = InputMode.PASSTHROUGH
        current_mode = "torque"
        print("Switched to TORQUE control\n")

    else:
        print("Invalid mode\n")


# ==============================
# Command Help
# ==============================

print("Commands:")
print("  arm            -> enter closed loop")
print("  idle           -> go to idle")
print("  mode p/v/t     -> select control mode")
print("  number         -> send command (depends on mode)")
print("  o +/-x         -> change position offset")
print("  r              -> read state")
print("  q              -> quit\n")


# ==============================
# Main Loop
# ==============================

while True:
    cmd = input(">> ")

    if cmd == "q":
        break

    if cmd == "arm":
        axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.5)
        print("Entered CLOSED_LOOP_CONTROL\n")
        continue

    if cmd == "idle":
        axis.requested_state = AxisState.IDLE
        print("Axis set to IDLE\n")
        continue

    if cmd.startswith("mode"):
        try:
            mode = cmd.split()[1]
            set_mode(mode)
        except:
            print("Usage: mode p/v/t\n")
        continue

    if cmd == "r":
        raw_pos = axis.encoder.pos_estimate
        raw_vel = axis.encoder.vel_estimate
        iq = axis.motor.current_control.Iq_measured

        print(f"State: {axis.current_state}")
        print(f"Position (raw)     : {raw_pos:.4f} turns")
        print(f"Position (corrected): {raw_pos - position_offset:.4f} turns")
        print(f"Velocity           : {raw_vel:.4f} turns/s")
        print(f"Iq measured        : {iq:.4f} A\n")
        continue

    if cmd.startswith("o"):
        try:
            delta = float(cmd.split()[1])
            position_offset += delta
            print(f"New position offset: {position_offset:.4f}\n")
        except:
            print("Invalid offset command\n")
        continue

    # Numeric command input
    try:
        value = float(cmd)

        if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
            print("Motor not armed. Use 'arm' first.\n")
            continue

        if current_mode == "position":
            axis.controller.input_pos = value + position_offset
            print(f"Position command sent: {value}\n")

        elif current_mode == "velocity":
            axis.controller.input_vel = value
            print(f"Velocity command sent: {value} turns/s\n")

        elif current_mode == "torque":
            axis.controller.input_torque = value
            print(f"Torque command sent: {value} Nm\n")

        else:
            print("Set control mode first using: mode p/v/t\n")

    except:
        print("Invalid command\n")


# ==============================
# Shutdown
# ==============================

axis.requested_state = AxisState.IDLE
print("Exited safely.")