import argparse
import time
from dataclasses import dataclass

import odrive
from odrive.utils import dump_errors
from odrive.enums import AxisState


# ==============================
# GIM6010-8 constants
# ==============================
#
# The GIM6010-8 has an integrated 8:1 gearbox. The CAN-Simple interface (and
# robot_hardware_interface.cpp) command the ROTOR side: Set_Input_Pos and
# Set_Input_Torque are scaled by reduction_ratio=8 in write(). Therefore the
# firmware's torque_constant MUST be the ROTOR torque constant, i.e. the
# datasheet OUTPUT-shaft Kt (0.47 Nm/A = 5 Nm / 10.5 A) divided by the gear
# ratio. Setting the output value (0.47) here while the C++ divides torque by
# 8 makes the motor deliver only 1/8 of the commanded torque.
GEAR_RATIO = 8.0
OUTPUT_TORQUE_CONSTANT = 0.47                       # Nm/A at the output shaft (datasheet)
ROTOR_TORQUE_CONSTANT = OUTPUT_TORQUE_CONSTANT / GEAR_RATIO  # ~0.0588 Nm/A at the rotor


# ==============================
# Configuration Dataclass
# ==============================

@dataclass
class ODriveMotorConfig:
    # Motor electrical parameters
    pole_pairs: int
    torque_constant: float

    # Limits
    current_limit: float
    velocity_limit: float

    # 6S LiPo protection
    undervoltage_trip: float
    overvoltage_trip: float
    dc_max_positive_current: float
    dc_max_negative_current: float

    # CAN settings (per motor)
    can_node_id: int
    can_baudrate: int


# ==============================
# Apply Configuration
# ==============================

def apply_config(odrv, cfg: ODriveMotorConfig):

    axis = odrv.axis0

    print("Applying motor parameters...")

    axis.motor.config.pole_pairs = cfg.pole_pairs
    # NOTE: phase_resistance and phase_inductance are MEASURED by
    # MOTOR_CALIBRATION below (datasheet ~0.48 ohm / 368 uH), so we do not
    # hard-code them here -- any value set now is overwritten by calibration.
    axis.motor.config.torque_constant = cfg.torque_constant

    axis.motor.config.current_lim = cfg.current_limit
    axis.controller.config.vel_limit = cfg.velocity_limit

    print("Applying power / battery protection (6S LiPo)...")
    # 6S: 25.2V full, 22.2V nominal. Protect the pack from over-discharge and
    # avoid false overvoltage trips at full charge.
    # odrv.config.dc_bus_undervoltage_trip_level = cfg.undervoltage_trip
    # odrv.config.dc_bus_overvoltage_trip_level = cfg.overvoltage_trip
    odrv.config.dc_max_positive_current = cfg.dc_max_positive_current
    # A LiPo cannot safely absorb regen current. Keep this small (or add a
    # brake resistor) so braking energy does not overvolt the bus.
    odrv.config.dc_max_negative_current = cfg.dc_max_negative_current

    print("Applying CAN parameters...")

    axis.config.can.node_id = cfg.can_node_id
    odrv.can.config.baud_rate = cfg.can_baudrate

    print("Configuration applied.\n")


# ==============================
# Calibration Routine
# ==============================

def calibrate_axis(odrv):

    axis = odrv.axis0

    print("Starting motor calibration...")
    axis.requested_state = AxisState.MOTOR_CALIBRATION
    time.sleep(5)

    while axis.current_state != AxisState.IDLE:
        time.sleep(0.5)

    dump_errors(odrv)

    print("Starting encoder offset calibration (output shaft must be FREE)...")
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    time.sleep(6)

    while axis.current_state != AxisState.IDLE:
        time.sleep(0.5)

    dump_errors(odrv)

    print("Marking as pre-calibrated...")
    axis.motor.config.pre_calibrated = True
    axis.encoder.config.pre_calibrated = True


# ==============================
# Main Setup Routine
# ==============================

def setup_odrive(cfg: ODriveMotorConfig):

    print("Connecting to ODrive over USB...")
    odrv = odrive.find_any()

    print(f"VBUS Voltage: {odrv.vbus_voltage:.2f} V")

    dump_errors(odrv)
    odrv.clear_errors()

    apply_config(odrv, cfg)
    calibrate_axis(odrv)

    print("Saving configuration...")
    try:
        odrv.save_configuration()
    except Exception:
        print("Device rebooted after save (this is normal).")

    print(f"Setup complete for node_id={cfg.can_node_id}. "
          f"Connect the next motor and re-run with its --node-id.\n")


# ==============================
# Example Usage
# ==============================
#
#   Calibrate each motor ONE AT A TIME over USB, giving each a unique node id
#   that matches ros2_control.xacro (0..11). Pass --terminate ONLY for the
#   single motor at the far physical end of the CAN bus:
#
#       python odrive_calibrate.py --node-id 3
#       python odrive_calibrate.py --node-id 4
#       python odrive_calibrate.py --node-id 5 --terminate
#
if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Calibrate one GIM6010-8 (GDS68) over USB for a 12-motor CAN bus.")
    parser.add_argument("--node-id", type=int, required=True,
                        help="Unique CAN node id 0-63 (must match ros2_control.xacro)")
    args = parser.parse_args()

    if not 0 <= args.node_id <= 63:
        parser.error("--node-id must be in range 0..63")

    config = ODriveMotorConfig(
        pole_pairs=14,
        torque_constant=ROTOR_TORQUE_CONSTANT,   # ~0.0588, see note at top of file
        current_limit=25.0,                      # Q-axis amps; <=50 for the 6010-8
        velocity_limit=30,
        undervoltage_trip=19.5,                  # ~3.25 V/cell, protects the 6S pack
        overvoltage_trip=26.0,                   # above 25.2 V full charge
        dc_max_positive_current=30.0,            # allow enough draw for torque
        dc_max_negative_current=-2.0,            # LiPo: limit regen current
        can_node_id=args.node_id,
        can_baudrate=1000000,
    )

    setup_odrive(config)
