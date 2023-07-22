# Copyright 2023 usb_cam Authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the usb_cam Authors nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from pathlib import Path
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from pydantic import BaseModel, root_validator, validator

USB_CAM_DIR = get_package_share_directory('usb_cam')


class CameraConfig(BaseModel):
    name: str = 'camera1'
    param_path: Path = Path(USB_CAM_DIR, 'config', 'params_1.yaml')
    remappings: Optional[List]
    namespace: Optional[str]

    @validator('param_path')
    def validate_param_path(cls, value):
        if value and not value.exists():
            raise FileNotFoundError(f'Could not find parameter file: {value}')
        return value

    @root_validator
    def validate_root(cls, values):
        name = values.get('name')
        remappings = values.get('remappings')
        if name and not remappings:
            # Automatically set remappings if name is set
            remappings = [
                ('image_raw', f'{name}/image_raw'),
                ('image_raw/compressed', f'{name}/image_compressed'),
                ('image_raw/compressedDepth', f'{name}/compressedDepth'),
                ('image_raw/theora', f'{name}/image_raw/theora'),
                ('camera_info', f'{name}/camera_info'),
            ]
        values['remappings'] = remappings
        return values
