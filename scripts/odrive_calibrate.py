import odrive
import time
from dataclasses import dataclass
from odrive.utils import dump_errors
from odrive.enums import AxisState


# ==============================
# Configuration Dataclass
# ==============================

@dataclass
class ODriveMotorConfig:
    # Motor electrical parameters
    pole_pairs: int
    phase_resistance: float
    phase_inductance: float
    torque_constant: float

    # Limits
    current_limit: float
    velocity_limit: float

    # CAN settings
    can_node_id: int
    can_baudrate: int
    enable_termination: bool


# ==============================
# Apply Configuration
# ==============================

def apply_config(odrv, cfg: ODriveMotorConfig):

    axis = odrv.axis0

    print("Applying motor parameters...")

    axis.motor.config.pole_pairs = cfg.pole_pairs
    axis.motor.config.phase_resistance = cfg.phase_resistance
    axis.motor.config.phase_inductance = cfg.phase_inductance
    axis.motor.config.torque_constant = cfg.torque_constant

    axis.motor.config.current_lim = cfg.current_limit
    axis.controller.config.vel_limit = cfg.velocity_limit

    print("Applying CAN parameters...")

    axis.config.can.node_id = cfg.can_node_id
    odrv.can.config.baud_rate = cfg.can_baudrate
    odrv.can.config.enable_r120 = cfg.enable_termination

    print("Configuration applied.\n")


# ==============================
# Calibration Routine
# ==============================

def calibrate_axis(axis):

    print("Starting motor calibration...")
    axis.requested_state = AxisState.MOTOR_CALIBRATION
    time.sleep(5)

    while axis.current_state != AxisState.IDLE:
        time.sleep(0.5)

    dump_errors(axis)

    print("Starting encoder offset calibration...")
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    time.sleep(6)

    while axis.current_state != AxisState.IDLE:
        time.sleep(0.5)

    dump_errors(axis)

    print("Marking as pre-calibrated...")
    axis.motor.config.pre_calibrated = True
    axis.encoder.config.pre_calibrated = True


# ==============================
# Main Setup Routine
# ==============================

def setup_odrive(cfg: ODriveMotorConfig):

    print("Connecting to ODrive...")
    odrv = odrive.find_any()

    print(f"VBUS Voltage: {odrv.vbus_voltage:.2f} V")

    dump_errors(odrv)
    odrv.clear_errors()

    apply_config(odrv, cfg)
    calibrate_axis(odrv.axis0)

    print("Saving configuration...")
    try:
        odrv.save_configuration()
    except Exception:
        print("Device rebooted after save (this is normal).")

    print("Setup complete. Rebooting recommended.")


# ==============================
# Example Usage
# ==============================

if __name__ == "__main__":

    config = ODriveMotorConfig(
        pole_pairs=14,
        phase_resistance=0.44,
        phase_inductance=0.23,
        torque_constant=0.47,
        current_limit=23.4,
        velocity_limit=30,
        can_node_id=0,
        can_baudrate=1000000,
        enable_termination=True
    )

    setup_odrive(config)
