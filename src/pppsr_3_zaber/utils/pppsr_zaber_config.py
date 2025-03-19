import tomllib

from pathlib import Path
import tomli_w
from pydantic import Field
from pydantic.dataclasses import dataclass


DEFAULT_TOML_NAME = "pppsr-zaber-config.toml"


# Define the dataclasses with pydantic validation
@dataclass(frozen=True)
class PPPSRAxisConfig:
    inverted: bool  # True if the axis is inverted
    axis_address: int = Field(..., ge=1, le=4)  # Must be between 1 and 4
    origin: float = Field(..., ge=0)  # Must be greater or equal to 0


@dataclass(frozen=True)
class PPPSRDeviceConfig:
    x_axis: PPPSRAxisConfig
    y_axis: PPPSRAxisConfig
    z_axis: tuple[PPPSRAxisConfig, PPPSRAxisConfig]  # lockstep of 2 axes
    device_address: int = Field(..., ge=1, le=3)  # Must be 1, 2, or 3

    def __post_init__(self):
        # Ensure that the Z-Axis lockstep is a tuple of 2 axes
        if len(self.z_axis) != 2:
            raise ValueError("Z-Axis lockstep must be a tuple of 2 axes")
        # Ensure that the Z-Axis lockstep axes are unique
        if self.z_axis[0].axis_address == self.z_axis[1].axis_address:
            raise ValueError("Z-Axis lockstep axes must be unique")
        # Ensure that the Z-Axis lockstep axes are not the same as the X or Y axes
        if {
            axis_config.axis_address
            for axis_config in [self.x_axis, self.y_axis, *self.z_axis]
        } != {1, 2, 3, 4}:
            raise ValueError("Z-Axis lockstep axes must be different from X and Y axes")
        # Ensure that the 2 Z-Axis have the same inverted value
        if self.z_axis[0].inverted != self.z_axis[1].inverted:
            raise ValueError("Z-Axis lockstep axes must have the same inverted value")


@dataclass(frozen=True)
class PPPSRZaberConfig:
    comport: str
    leg1: PPPSRDeviceConfig
    leg2: PPPSRDeviceConfig
    leg3: PPPSRDeviceConfig

    # Function to serialize Python object to TOML
    def to_toml(self, filepath: str):
        # Convert the dataclass to a dict using the `dataclass_to_dict` recursive function

        # Recursive function to serialize dataclass to dictionary
        def __dataclass_to_dict(obj):
            """
            Recursively converts a dataclass instance into a dictionary.
            """
            if isinstance(obj, tuple) or isinstance(obj, list):
                return [__dataclass_to_dict(o) for o in obj]
            elif isinstance(obj, dict):
                return {k: __dataclass_to_dict(v) for k, v in obj.items()}
            elif hasattr(obj, "__dict__"):
                return {k: __dataclass_to_dict(v) for k, v in obj.__dict__.items()}
            else:
                return obj

        data_dict = __dataclass_to_dict(self)
        if Path(filepath).exists():
            Path(filepath).unlink()
        with open(filepath, "wb") as f:
            tomli_w.dump(data_dict, f)  # type: ignore

    # Function to deserialize TOML file back into Python object
    @classmethod
    def from_toml(cls, filepath: str):
        with open(filepath, "rb") as f:
            toml_data = tomllib.load(f)

        # Convert the TOML data back into the dataclass
        # Deserialization needs to manually map the data back into a instance

        leg1_device_data = toml_data["leg1"]
        leg2_device_data = toml_data["leg2"]
        leg3_device_data = toml_data["leg3"]

        leg1 = PPPSRDeviceConfig(
            x_axis=PPPSRAxisConfig(**leg1_device_data["x_axis"]),
            y_axis=PPPSRAxisConfig(**leg1_device_data["y_axis"]),
            z_axis=(
                PPPSRAxisConfig(**leg1_device_data["z_axis"][0]),
                PPPSRAxisConfig(**leg1_device_data["z_axis"][1]),
            ),
            device_address=leg1_device_data["device_address"],
        )

        leg2 = PPPSRDeviceConfig(
            x_axis=PPPSRAxisConfig(**leg2_device_data["x_axis"]),
            y_axis=PPPSRAxisConfig(**leg2_device_data["y_axis"]),
            z_axis=(
                PPPSRAxisConfig(**leg2_device_data["z_axis"][0]),
                PPPSRAxisConfig(**leg2_device_data["z_axis"][1]),
            ),
            device_address=leg2_device_data["device_address"],
        )

        leg3 = PPPSRDeviceConfig(
            x_axis=PPPSRAxisConfig(**leg3_device_data["x_axis"]),
            y_axis=PPPSRAxisConfig(**leg3_device_data["y_axis"]),
            z_axis=(
                PPPSRAxisConfig(**leg3_device_data["z_axis"][0]),
                PPPSRAxisConfig(**leg3_device_data["z_axis"][1]),
            ),
            device_address=leg3_device_data["device_address"],
        )

        return PPPSRZaberConfig(
            comport=toml_data["comport"], leg1=leg1, leg2=leg2, leg3=leg3
        )


# Example Usage:
if __name__ == "__main__":
    # Create a sample object
    pppsr_zaber = PPPSRZaberConfig(
        comport="/dev/tty.usbmodem1122811",
        leg1=PPPSRDeviceConfig(
            device_address=1,
            x_axis=PPPSRAxisConfig(axis_address=3, origin=250.0, inverted=True),
            y_axis=PPPSRAxisConfig(axis_address=4, origin=250.0, inverted=False),
            z_axis=(
                PPPSRAxisConfig(axis_address=1, origin=250.0, inverted=True),
                PPPSRAxisConfig(axis_address=2, origin=250.0, inverted=True),
            ),
        ),
        leg2=PPPSRDeviceConfig(
            device_address=2,
            x_axis=PPPSRAxisConfig(axis_address=3, origin=250.0, inverted=True),
            y_axis=PPPSRAxisConfig(axis_address=4, origin=250.0, inverted=False),
            z_axis=(
                PPPSRAxisConfig(axis_address=1, origin=250.0, inverted=True),
                PPPSRAxisConfig(axis_address=2, origin=250.0, inverted=True),
            ),
        ),
        leg3=PPPSRDeviceConfig(
            device_address=3,
            x_axis=PPPSRAxisConfig(axis_address=3, origin=250.0, inverted=True),
            y_axis=PPPSRAxisConfig(axis_address=4, origin=250.0, inverted=False),
            z_axis=(
                PPPSRAxisConfig(axis_address=1, origin=250.0, inverted=True),
                PPPSRAxisConfig(axis_address=2, origin=250.0, inverted=True),
            ),
        ),
    )

    # Serialize the object to a TOML file
    pppsr_zaber.to_toml(DEFAULT_TOML_NAME)

    # Deserialize from the TOML file
    deserialized_config = PPPSRZaberConfig.from_toml(DEFAULT_TOML_NAME)

    print(deserialized_config)
