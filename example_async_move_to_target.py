import asyncio

import numpy as np
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


async def main():
    with PPPSRZaberRobot(
        PPPSRZaberConfig.from_toml("pppsr-zaber-config.toml"),
        PPPSRDimension.load_from_toml("pppsr-zaber-dimension.toml"),
    ) as robot:
        await robot.move_to_async(
            P0,
            R0,
            RDOF0,
        )

        await robot.enable_stream_live_async()

        p_now = P0
        r_now = R0
        rdof_now = RDOF0

        dt = 1 / 24
        for t in np.arange(0, 15, dt):
            p_now, r_now, rdof_now = await robot.move_to_target_by_step_stream_async(
                p_now,
                r_now,
                rdof_now,
                p_target=P0 + np.array([10, 0, 0]),
                R_target=R0,
                rdof_target=RDOF0,
                max_translation_step_mm=1 * dt,
                max_rotation_step_deg=1 * dt,
            )
            await asyncio.sleep(dt)
            print(t)
            print(p_now)
            print()


if __name__ == "__main__":
    asyncio.run(main())
