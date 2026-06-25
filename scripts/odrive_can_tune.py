#!/usr/bin/env python3
"""Configure ODrive CAN cyclic rates to reduce bus load."""

import argparse
import odrive
from odrive.utils import dump_errors


def tune(odrv, control_hz: int, heartbeat_hz: float):
    axis = odrv.axis0
    encoder_ms = max(1, int(round(1000 / control_hz)))
    heartbeat_ms = max(1, int(round(1000 / heartbeat_hz)))

    axis.config.can.encoder_rate_ms = encoder_ms
    axis.config.can.heartbeat_rate_ms = heartbeat_ms
    # axis.config.can.torques_msg_rate_ms = 0
    # axis.config.can.iq_msg_rate_ms = 0
    # axis.config.can.bus_voltage_msg_rate_ms = 0
    # axis.config.can.temperature_msg_rate_ms = 0
    # axis.config.can.error_msg_rate_ms = 0

    print(f"encoder_msg_rate_ms = {encoder_ms} ({control_hz} Hz)")
    print(f"heartbeat_msg_rate_ms = {heartbeat_ms} ({heartbeat_hz} Hz)")
    # print("disabled: torques, iq, bus_voltage, temperature, error")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-hz", type=int, default=200,
                        help="Match ros2_control update_rate (default: 200)")
    parser.add_argument("--heartbeat-hz", type=float, default=20.0,
                        help="Heartbeat rate (default: 2 Hz)")
    parser.add_argument("--save", action="store_true",
                        help="Call save_configuration() after tuning")
    args = parser.parse_args()

    print("Connecting to ODrive...")
    odrv = odrive.find_any()
    dump_errors(odrv)
    odrv.clear_errors()

    tune(odrv, args.control_hz, args.heartbeat_hz)

    if args.save:
        print("Saving configuration...")
        odrv.save_configuration()
        print("Done. Reboot or power-cycle the ODrive.")
    else:
        print("Dry run — pass --save to persist.")


if __name__ == "__main__":
    main()