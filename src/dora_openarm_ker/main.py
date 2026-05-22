# Copyright 2026 Enactic, Inc.
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

"""dora-rs node for leader OpenArm KER."""

import argparse
import dora
import openarm_ker
import pathlib
import pyarrow as pa
import numpy as np


def main():
    """Act OpenArm KER as a leader of OpenArm."""
    parser = argparse.ArgumentParser(
        description="Act OpenArm KER as a leader of OpenArm"
    )
    parser.add_argument(
        "--config",
        required=True,
        help="The configuration file how to map leader position to follower position",
        type=pathlib.Path,
    )
    parser.add_argument(
        "--device",
        default="/dev/ttyACM0",
        help="The serial port device path (e.g. /dev/ttyACM0)",
        type=str,
    )
    parser.add_argument(
        "--mode",
        default="binary",
        help="The mode of the KER (binary or json)",
        type=str,
        choices=["binary", "json"],
    )
    args = parser.parse_args()

    m5_port = openarm_ker.m5_port.M5Port(args.device, num_sensors=16, mode=args.mode)
    right_leader_joint_names = [f"right_arm_joint{i}" for i in range(1, 9)]
    mapper_right = openarm_ker.mapper.Mapper(
        leader_joint_names=right_leader_joint_names,
        mapping_key="right_arm_mappings",
        mappingyaml_path=args.config,
    )
    left_leader_joint_names = [f"left_arm_joint{i}" for i in range(1, 9)]
    mapper_left = openarm_ker.mapper.Mapper(
        leader_joint_names=left_leader_joint_names,
        mapping_key="left_arm_mappings",
        mappingyaml_path=args.config,
    )

    node = dora.Node()
    for event in node:
        if event["type"] != "INPUT":
            continue

        # Main process
        m5_port.fetch_present_status_bulk()
        position_right = m5_port.present_position[:8]
        position_left = m5_port.present_position[8:16]

        radian_right = np.deg2rad(position_right)
        follower_position_right = mapper_right.map(radian_right)
        radian_left = np.deg2rad(position_left)
        follower_position_left = mapper_left.map(radian_left)

        joystick_x = m5_port.get_joystick_x()
        joystick_y = m5_port.get_joystick_y()
        joystick_button = m5_port.get_joystick_button()

        node.send_output("position_right", pa.array(position_right, type=pa.float32()))
        node.send_output("position_left", pa.array(position_left, type=pa.float32()))

        node.send_output(
            "follower_position_right",
            pa.array(follower_position_right, type=pa.float32()),
        )
        node.send_output(
            "follower_position_left",
            pa.array(follower_position_left, type=pa.float32()),
        )

        node.send_output("joystick_x", pa.array([joystick_x], type=pa.float32()))
        node.send_output("joystick_y", pa.array([joystick_y], type=pa.float32()))
        node.send_output(
            "joystick_button", pa.array([joystick_button], type=pa.int32())
        )

    m5_port.cleanup()


if __name__ == "__main__":
    main()
