from zaber_motion import Units
from zaber_motion.ascii import Connection
from zaber_motion.ascii.warning_flags import WarningFlags

import click

with Connection.open_serial_port("/dev/tty.usbmodem1122811") as connection:
    connection.enable_alerts()

    device_list = connection.detect_devices()
    print("Found {} devices".format(len(device_list)))

    device = device_list[0]

    axis = device.get_axis(3)
    print(axis.warnings.get_flags())

    if WarningFlags.NO_REFERENCE_POSITION in axis.warnings.get_flags():
        if click.confirm(
            f"Axis {axis.axis_number} does not have reference position, home it?"
        ):
            axis.home()

        # # Move to 10mm
        # axis.move_absolute(10, Units.LENGTH_MILLIMETRES)

        # # Move by an additional 5mm
        # axis.move_relative(5, Units.LENGTH_MILLIMETRES)
