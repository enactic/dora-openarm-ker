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
    args = parser.parse_args()

    m5_port = openarm_ker.m5_port.M5Port(args.device, num_sensors=16, mode="binary")
    right_leader_joint_names = [f"right_arm_joint{i}" for i in range(1, 9)]
    right_mapper = openarm_ker.mapper.Mapper(
        leader_joint_names=right_leader_joint_names,
        mapping_key="right_arm_mappings",
        mappingyaml_path=args.config,
    )
    left_leader_joint_names = [f"left_arm_joint{i}" for i in range(1, 9)]
    left_mapper = openarm_ker.mapper.Mapper(
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
        right_position = m5_port.present_positions[:8]
        left_position = m5_port.present_positions[8:16]

        right_radian = np.deg2rad(right_position)
        right_follower_position = right_mapper.map(right_radian)
        left_radian = np.deg2rad(left_position)
        left_follower_position = left_mapper.map(left_radian)

        right_chain_encoders = m5_port.get_chain_encoder()
        right_chain_buttons = m5_port.get_chain_encoder_button()

        joystick_x = m5_port.get_joystick_x()
        joystick_y = m5_port.get_joystick_y()
        joystick_button = m5_port.get_joystick_button()

        node.send_output("right_position", pa.array(right_position, type=pa.float32()))
        node.send_output("left_position", pa.array(left_position, type=pa.float32()))

        node.send_output(
            "right_follower_position",
            pa.array(right_follower_position, type=pa.float32()),
        )
        node.send_output(
            "left_follower_position",
            pa.array(left_follower_position, type=pa.float32()),
        )

        node.send_output(
            "chain_encoders", pa.array([right_chain_encoders], type=pa.int32())
        )

        node.send_output(
            "chain_buttons", pa.array([right_chain_buttons], type=pa.int32())
        )

        node.send_output("joystick_x", pa.array([joystick_x], type=pa.float32()))
        node.send_output("joystick_y", pa.array([joystick_y], type=pa.float32()))
        node.send_output(
            "joystick_button", pa.array([joystick_button], type=pa.int32())
        )

    m5_port.cleanup()


if __name__ == "__main__":
    main()
