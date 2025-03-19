import asyncio
from typing import Callable

import numpy as np
from arl_utils_py.trajectory_planning.task_space_trajectory_planning import (
    QuinticTrajectoryPlanner,
    Range,
    TaskSpaceTrajectory,
    TaskSpaceTrajectorySegment,
)
from numpy.typing import NDArray
from pppsr_3 import PPPSRDimension, Rotation

from pppsr_3_zaber.lib.robot import PPPSRZaberRobot
from pppsr_3_zaber.utils.pppsr_zaber_config import PPPSRZaberConfig

P0 = np.array([0, 0, 0])
R0 = Rotation.from_euler("z", 0, degrees=True)
RDOF0 = np.array(
    [
        np.rad2deg(np.atan2(-12.396, 84.09)),
        np.rad2deg(np.atan2(79.02, -31.31)),
        np.rad2deg(np.atan2(-66.63, -52.78)),
    ]
)


def trajectory_p_R_func_raise_tilt_torsion_down_1_10_40_10_1(
    tilt_angle_deg: float,
) -> tuple[Callable[[float], tuple[NDArray, Rotation]], Range]:
    trajectory_f = TaskSpaceTrajectory(
        [
            TaskSpaceTrajectorySegment(
                Range(0, 1),
                lambda u: P0 + u * np.array([0, 0, 0.1]),
                lambda u: R0,
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(1, 11),
                lambda u: P0 + np.array([0, 0, 0.1]),
                lambda u: Rotation.from_euler("y", u * tilt_angle_deg, degrees=True)
                * R0,
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(11, 51),
                lambda u: P0 + np.array([0, 0, 0.1]),
                lambda u: Rotation.from_rotvec(
                    Rotation.from_euler("z", u * 2 * np.pi).apply(np.array([0, 1, 0]))
                    * tilt_angle_deg,
                    degrees=True,
                )
                * R0,
                # TrapezoidalTrajectoryPlanner(v_max_unsigned=1 / (10 - 2) * 1.2),
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(51, 61),
                lambda u: P0 + np.array([0, 0, 0.1]),
                lambda u: Rotation.from_euler(
                    "y", (1 - u) * tilt_angle_deg, degrees=True
                )
                * R0,
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(61, 62, include_end=True),
                lambda u: P0 + (1 - u) * np.array([0, 0, 0.1]),
                lambda u: Rotation.from_euler("y", 0) * R0,
                QuinticTrajectoryPlanner(),
            ),
        ]
    )
    return trajectory_f, trajectory_f.range


async def main():
    trajectory_f, t_range = trajectory_p_R_func_raise_tilt_torsion_down_1_10_40_10_1(90)

    with PPPSRZaberRobot(
        PPPSRZaberConfig.from_toml("pppsr-zaber-config.toml"),
        PPPSRDimension.load_from_toml("pppsr-zaber-dimension.toml"),
    ) as robot:
        await robot.move_to_async(P0, R0, RDOF0)

        await robot.enable_stream_live_async()

        dt = 1 / 60
        for t in np.arange(t_range.start, t_range.end, dt):
            p, R = trajectory_f(t)  # type: ignore
            await robot.move_to_stream_live_async(p, R, RDOF0)
        print("Command Writing Done!")

        await robot.wait_until_idle_async()
        print("Done!")


if __name__ == "__main__":
    asyncio.run(main())
