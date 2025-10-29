# Copyright 2024 VoltBro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""Вспомогательные action-компоненты TurtleBro."""

from .examples.move_client import MoveClient
from .examples.rotate_client import RotateClient
from .examples.servo_client import ServoClient
from .examples.video_client import VideoClient
from .servers.audio_service import RecordAudioService
from .servers.move_server import MoveServer
from .servers.photo_service import PhotoService
from .servers.rotate_server import RotateServer

__all__ = [
    "RecordAudioService",
    "MoveClient",
    "MoveServer",
    "PhotoService",
    "RotateClient",
    "RotateServer",
    "ServoClient",
    "VideoClient",
]
