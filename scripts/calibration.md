# Odrive Calibratioin Scripts

## Installation
```
cd scripts
python3 -m venv venv
source venv/bin/activate
pip install odrive
```

## Usage
For calibrating odrives, connect power supply to motor and type-c cable to computer, change node ID in [here](odrive_calibrate.py) and run the script

For interactive experience, try [this](odrive_interactive.py)

> [!IMPORTANT]
> Make sure to deactivate the virtual environment if not in use.

## Commands

### Constants
- Current Limit: 23.4A
- Phase Resistance: 0.44
- Phase Inductance: 0.23
- Speed Constant: 11.54
- Torque Constant: 0.47
- Num Pole Pairs: 14

Get VBUS Voltage
```
odrv0.vbus_voltage
```

Configure Current Limit
```
odrv0.axis0.motor.config.current_lim = 30
```

Configure Velocity Limit
```
odrv0.axis0.controller.config.vel_limit = 30
```

Configure pole pairs
```
odrv0.axis0.motor.config.pole_pairs
```

Configure Torque Constant
```
odrv0.axis0.motor.config.torque_constant
```

Calibration
```
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dump_errors(odrv0)
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dump_errors(odrv0)
odrv0.axis0.motor.config.pre_calibrated = 1
odrv0.axis0.encoder.config.pre_calibrated = 1
odrv0.save_configuration()
```

Check Phase Resistance and Inductance
```
odrv0.axis0.motor.config.phase_resistance
odrv0.axis0.motor.config.phase_inductance
```

Configuring Node ID
```
odrv0.axis0.config.can.node_id = xxx
```

Configure Termination Resistor
```
odrv0.can.config.r120_5
odrv0.can.config.enable_r120 = False
```

Configure CAN Baudrate
```
odrv0.can.config.baud_rate
```

Save Configuration
```
odrv0.save_configuration()
```
