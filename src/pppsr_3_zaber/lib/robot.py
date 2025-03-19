import asyncio
from dataclasses import dataclass
from typing import Optional

import click
import numpy as np
from pppsr_3 import Array3, PPPSRDimension, Rotation
from zaber_motion import Units
from zaber_motion.ascii import Axis, Connection, Device, Lockstep
from zaber_motion.ascii.warning_flags import WarningFlags
from zaber_motion.dto.ascii import StreamAxisDefinition, StreamAxisType
from zaber_motion.dto.measurement import Measurement

from pppsr_3_zaber.utils.pppsr_zaber_config import PPPSRZaberConfig


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

    async def move_absolute_local_async(self, p_i: Array3, **kwargs):
        p = self.origin + np.where(self.inverted, -p_i, p_i)
        return await asyncio.gather(
            self.axes[0].move_absolute_async(p[0], **kwargs),
            self.axes[1].move_absolute_async(p[1], **kwargs),
            self.axes[2].move_absolute_async(p[2], **kwargs),
        )

    def move_absolute_stream_live(self, p_i: Array3, unit: Units):
        driver_coordinate = self.origin + np.where(self.inverted, -p_i, p_i)
        stream = self.device.streams.get_stream(1)
        stream.line_absolute(*[Measurement(p, unit) for p in driver_coordinate])

    async def move_absolute_stream_live_async(self, p_i: Array3, unit: Units):
        driver_coordinate = self.origin + np.where(self.inverted, -p_i, p_i)
        stream = self.device.streams.get_stream(1)
        return await stream.line_absolute_async(
            *[Measurement(p, unit) for p in driver_coordinate]
        )


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
        # self.__connection.disable_alerts()
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

    async def move_to_async(
        self,
        p: Array3,
        R: Rotation,
        redundant_angle: Array3,
        wait_until_idle: bool = True,
    ):
        return await asyncio.gather(
            *[
                leg.move_absolute_local_async(
                    local_coords,
                    unit=Units.LENGTH_MILLIMETRES,
                    wait_until_idle=wait_until_idle,
                )
                for leg, local_coords in zip(
                    self.legs,
                    self.__dimension.p_i_local(p, R, redundant_angle),
                )
            ]
        )

    def enable_stream_live(self):
        for leg in self.legs:
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

    async def enable_stream_live_async(self):
        return await asyncio.gather(
            *[
                disabled_leg.device.streams.get_stream(1).setup_live_composite_async(
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
                        for axis in disabled_leg.axes
                    ],
                )
                for disabled_leg in filter(
                    lambda leg: leg.device.streams.get_stream(1).check_disabled(),
                    self.legs,
                )
            ]
        )

    def move_to_stream_live(self, p: Array3, R: Rotation, redundant_angle: Array3):
        for leg, local_coords in zip(
            self.legs,
            self.__dimension.p_i_local(p, R, redundant_angle),
        ):
            leg.move_absolute_stream_live(
                local_coords,
                Units.LENGTH_MILLIMETRES,
            )

    async def move_to_stream_live_async(
        self, p: Array3, R: Rotation, redundant_angle: Array3
    ):
        return await asyncio.gather(
            *[
                leg.move_absolute_stream_live_async(
                    local_coords,
                    Units.LENGTH_MILLIMETRES,
                )
                for leg, local_coords in zip(
                    self.legs,
                    self.__dimension.p_i_local(p, R, redundant_angle),
                )
            ]
        )

    def wait_until_idle(self):
        if self.__connection is None:
            raise RuntimeError("Connection is not open")
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

    async def wait_until_idle_async(self):
        return await asyncio.gather(
            *[leg.device.all_axes.wait_until_idle_async() for leg in self.legs]
        )

    async def move_to_target_by_step_stream_async(
        self,
        p: Array3,
        R: Rotation,
        rdof: Array3,
        p_target: Array3,
        R_target: Rotation,
        rdof_target: Array3,
        max_translation_step_mm: float,
        max_rotation_step_deg: float,
        max_rdof_step_deg: float,
    ):
        dp = p_target - p
        dR = R_target * R.inv()
        dR_rotvec = dR.as_rotvec()
        dR_ang_deg = np.rad2deg(np.linalg.norm(dR_rotvec))

        ratio = np.max(
            list(np.abs(dp) / max_translation_step_mm)
            + [np.abs(dR_ang_deg) / max_rotation_step_deg]
            + list(np.abs(rdof_target - rdof) / max_rdof_step_deg)
            + [1],
        )

        p_next = p + dp / ratio
        R_next = R * Rotation.from_rotvec(dR_rotvec / ratio)
        rdof_next = rdof + (rdof_target - rdof) / ratio

        await self.move_to_stream_live_async(p_next, R_next, rdof_next)
        return p_next, R_next, rdof_next
