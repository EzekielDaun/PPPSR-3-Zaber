import numpy as np
from pppsr_3 import PPPSRDimension, Rotation

from pppsr_3_zaber.lib.robot import PPPSRZaberRobot
from pppsr_3_zaber.utils.pppsr_zaber_config import PPPSRZaberConfig

RDOF0 = np.array(
    [
        np.rad2deg(np.atan2(-12.396, 84.09)),
        np.rad2deg(np.atan2(79.02, -31.31)),
        np.rad2deg(np.atan2(-66.63, -52.78)),
    ]
)

if __name__ == "__main__":
    with PPPSRZaberRobot(
        PPPSRZaberConfig.from_toml("pppsr-zaber-config.toml"),
        PPPSRDimension.load_from_toml("pppsr-zaber-dimension.toml"),
    ) as robot:
        robot.move_to(
            np.array([0, 0, 0]),
            Rotation.from_euler("x", 0, degrees=True),
            RDOF0,
        )

        robot.enable_stream_live()
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
