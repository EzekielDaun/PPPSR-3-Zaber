import time
from dataclasses import dataclass
from typing import Optional

import click
import numpy as np
from pppsr_3 import DIMENSION, RDOF0, Array3, PPPSRDimension, Rotation
from zaber_motion import Units
from zaber_motion.ascii import Axis, Connection, Device, Lockstep
from zaber_motion.ascii.warning_flags import WarningFlags
from zaber_motion.dto.ascii import (
    PvtAxisDefinition,
    PvtAxisType,
    PvtMode,
    StreamAxisDefinition,
    StreamAxisType,
    StreamMode,
)
from zaber_motion.dto.measurement import Measurement

from pppsr_3_zaber.utils.pppsr_zaber_config import DEFAULT_TOML_NAME, PPPSRZaberConfig


@dataclass(frozen=True)
class PPPSRZaberLeg:
    device: Device
    axes: tuple[Axis | Lockstep, Axis | Lockstep, Axis | Lockstep]
    origin: Array3
    inverted: tuple[bool, bool, bool]

    def move_absolute_local(self, p_i: Array3, **kwargs):
        p = self.origin + np.where(self.inverted, -p_i, p_i)
        self.axes[0].move_absolute(p[0], **kwargs)
        self.axes[1].move_absolute(p[1], **kwargs)
        self.axes[2].move_absolute(p[2], **kwargs)

    def move_absolute_pvt_live(self, p_i: Array3, unit: Units, time_s: float):
        driver_coordinate = self.origin + np.where(self.inverted, -p_i, p_i)
        pvt_sequence = self.device.pvt.get_sequence(1)

        if pvt_sequence.check_disabled():
            pvt_sequence.setup_live_composite(
                *[
                    (
                        PvtAxisDefinition(axis.axis_number, PvtAxisType.PHYSICAL)
                        if isinstance(axis, Axis)
                        else PvtAxisDefinition(
                            axis.lockstep_group_id, PvtAxisType.LOCKSTEP
                        )
                    )
                    for axis in self.axes
                ],
            )
        pvt_sequence.point(
            [Measurement(p, unit) for p in driver_coordinate],
            [None for p in driver_coordinate],
            Measurement(time_s, Units.TIME_SECONDS),
        )

    def move_absolute_stream_live(self, p_i: Array3, unit: Units):
        driver_coordinate = self.origin + np.where(self.inverted, -p_i, p_i)

        stream = self.device.streams.get_stream(1)
        # if stream.check_disabled():
        #     stream.setup_live_composite(
        #         *[
        #             (
        #                 StreamAxisDefinition(axis.axis_number, StreamAxisType.PHYSICAL)
        #                 if isinstance(axis, Axis)
        #                 else StreamAxisDefinition(
        #                     axis.lockstep_group_id, StreamAxisType.LOCKSTEP
        #                 )
        #             )
        #             for axis in self.axes
        #         ],
        #     )
        stream.line_absolute(*[Measurement(p, unit) for p in driver_coordinate])


class PPPSRZaberRobot:
    def __init__(
        self,
        config: PPPSRZaberConfig,
        dimension: PPPSRDimension,
        ask_for_home: bool = True,
    ):
        self.__config = config
        self.__dimension = dimension
        self.__ask_for_home = ask_for_home
        self.__connection: Optional[Connection] = None
        self.legs: list[PPPSRZaberLeg] = []

    def __enter__(self):
        self.__connection = Connection.open_serial_port(self.__config.comport)
        self.__connection.disable_alerts()
        # self.__connection.enable_alerts()
        # self.__connection.alert.subscribe(
        # lambda alert: print(f"Alert from device: {alert}")
        # )
        self.__connection.detect_devices()
        self.__check_axes_setup()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.__connection:
            self.__connection.close()

    def __check_axes_setup(self):
        if self.__connection is not None:
            for leg in (self.__config.leg1, self.__config.leg2, self.__config.leg3):
                device = self.__connection.get_device(leg.device_address)

                # Home X and Y axes if necessary
                for axis_config in [leg.x_axis, leg.y_axis]:
                    axis = device.get_axis(axis_config.axis_address)
                    if WarningFlags.NO_REFERENCE_POSITION in axis.warnings.get_flags():
                        if not self.__ask_for_home:
                            raise RuntimeError(
                                f"Device {device.device_address} Axis {axis.axis_number} does not have reference position"
                            )
                        elif click.confirm(
                            f"Device {device.device_address} Axis {axis.axis_number} does not have reference position, home it?"
                        ):
                            axis.home(wait_until_idle=False)

                # Check Z-Axis LockStep and Home if necessary
                lockstep = list(
                    map(
                        lambda n: device.get_lockstep(n),
                        filter(
                            lambda n: device.get_lockstep(n).is_enabled(),
                            range(
                                1, 1 + int(device.settings.get("lockstep.numgroups"))
                            ),
                        ),
                    )
                )
                assert len(lockstep) == 1, "Only one lockstep group should be enabled"
                lockstep = lockstep[0]

                if all(
                    [
                        axis_config.axis_address in lockstep.get_axis_numbers()
                        for axis_config in leg.z_axis
                    ]
                ):
                    print(
                        f"Device {device.device_address} Z-Axis LockStep has been set up correctly"
                    )
                else:
                    raise RuntimeError("Z-Axis LockStep is not set up correctly")

                if any(
                    [
                        WarningFlags.NO_REFERENCE_POSITION
                        in device.get_axis(n).warnings.get_flags()
                        for n in lockstep.get_axis_numbers()
                    ]
                ):
                    if not self.__ask_for_home:
                        raise RuntimeError(
                            f"Device {device.device_address} LockStep group {lockstep.lockstep_group_id} does not have reference position"
                        )
                    elif click.confirm(
                        f"Device {device.device_address} LockStep group {lockstep.lockstep_group_id} does not have reference position, home it?"
                    ):
                        lockstep.home(wait_until_idle=False)

                # construct the Zaber leg object
                self.legs.append(
                    PPPSRZaberLeg(
                        device,
                        (
                            device.get_axis(leg.x_axis.axis_address),
                            device.get_axis(leg.y_axis.axis_address),
                            lockstep,
                        ),
                        np.array(
                            (leg.x_axis.origin, leg.y_axis.origin, leg.z_axis[0].origin)
                        ),
                        (
                            leg.x_axis.inverted,
                            leg.y_axis.inverted,
                            leg.z_axis[0].inverted,
                        ),
                    ),
                )

            while any(
                [
                    self.__connection.get_device(leg.device_address).all_axes.is_busy()
                    for leg in [
                        self.__config.leg1,
                        self.__config.leg2,
                        self.__config.leg3,
                    ]
                ]
            ):
                # print("Waiting for all axes to be idle")
                pass

            print("All axes are set up correctly")

    def move_to(
        self,
        p: Array3,
        R: Rotation,
        redundant_angle: Array3,
        wait_until_idle: bool = True,
    ):
        if self.__connection is None:
            raise RuntimeError("Connection is not open")

        for leg, local_coords in zip(
            self.legs,
            self.__dimension.p_i_local(p, R, redundant_angle),
        ):
            leg.move_absolute_local(
                local_coords,
                unit=Units.LENGTH_MILLIMETRES,
                wait_until_idle=False,
            )
        if wait_until_idle:
            while any(
                [
                    self.__connection.get_device(leg.device_address).all_axes.is_busy()
                    for leg in [
                        self.__config.leg1,
                        self.__config.leg2,
                        self.__config.leg3,
                    ]
                ]
            ):
                pass

    def move_to_pvt_live(
        self, p: Array3, R: Rotation, redundant_angle: Array3, time_s: float
    ):
        if self.__connection is None:
            raise RuntimeError("Connection is not open")

        for leg, local_coords in zip(
            self.legs,
            self.__dimension.p_i_local(p, R, redundant_angle),
        ):
            leg.move_absolute_pvt_live(
                local_coords,
                Units.LENGTH_MILLIMETRES,
                time_s,
            )

    def move_to_stream_live(self, p: Array3, R: Rotation, redundant_angle: Array3):
        if self.__connection is None:
            raise RuntimeError("Connection is not open")

        for leg, local_coords in zip(
            self.legs,
            self.__dimension.p_i_local(p, R, redundant_angle),
        ):
            leg.move_absolute_stream_live(
                local_coords,
                Units.LENGTH_MILLIMETRES,
            )


if __name__ == "__main__":
    config = PPPSRZaberConfig.from_toml(DEFAULT_TOML_NAME)

    with PPPSRZaberRobot(config, DIMENSION) as robot:
        robot.move_to(
            np.array([0, 0, 0]),
            Rotation.from_euler("x", 0, degrees=True),
            RDOF0,
        )

        for leg in robot.legs:
            stream = leg.device.streams.get_stream(1)
            if stream.check_disabled():
                stream.setup_live_composite(
                    *[
                        (
                            StreamAxisDefinition(
                                axis.axis_number, StreamAxisType.PHYSICAL
                            )
                            if isinstance(axis, Axis)
                            else StreamAxisDefinition(
                                axis.lockstep_group_id, StreamAxisType.LOCKSTEP
                            )
                        )
                        for axis in leg.axes
                    ],
                )

        # dt = 1 / 24
        dt = 1 / 30
        for t in np.arange(0, 5, dt):
            # print(t)
            robot.move_to_stream_live(
                np.array([-30 * t, 0, 5 * t]),
                Rotation.from_euler("x", 0, degrees=True),
                RDOF0,
            )
        for t in np.arange(0, 5, dt):
            # print(t)
            t = 5 - t
            robot.move_to_stream_live(
                np.array([-30 * t, 0, 5 * t]),
                Rotation.from_euler("x", 0, degrees=True),
                RDOF0,
            )
        print("Done")
