from pathlib import Path
from typing import List, Optional
from pydantic import BaseModel, root_validator, validator

from ament_index_python.packages import get_package_share_directory

USB_CAM_DIR = get_package_share_directory('usb_cam')


class CameraConfig(BaseModel):
    name: str = "camera1"
    param_path: Path = Path(USB_CAM_DIR, "config", "params_1.yaml")
    remappings: Optional[List]
    namespace: Optional[str]

    @validator("param_path")
    def validate_param_path(cls, value):
        if value and not value.exists():
            raise FileNotFoundError(f"Could not find parameter file: {value}")
        return value

    @root_validator
    def validate_root(cls, values):
        name = values.get("name")
        remappings = values.get("remappings")
        if name and not remappings:
            # Automatically set remappings if name is set
            remappings = [
                ("image_raw", f"{name}/image_raw"),
                ("image_raw/compressed", f"{name}/image_compressed"),
                ("image_raw/compressedDepth", f"{name}/compressedDepth"),
                ("image_raw/theora", f"{name}/image_raw/theora"),
                ("camera_info", f"{name}/camera_info"),
            ]
        values["remappings"] = remappings
        return values
