# dora-openarm-ker

A [dora-rs](https://dora-rs.ai/) node for leader OpenArm KER (Kinematic Equivalent Replica).

This node reads joint angles from OpenArm KER via USB and outputs them as target positions for follower OpenArm arms.

---

## Quick Start

### 1. Install system dependencies

```bash
sudo apt install libusb-1.0-0-dev
```

### 2. Set up udev rules (run once)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-m5stack.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Connect M5Stack and verify

Plug the M5Stack CoreS3 into your PC via USB and verify the device is recognized:

```bash
lsusb | grep 303a
# 303a:4002 should appear
```

### 4. Use in a dataflow

Add this node to your dataflow YAML. This node reads a received packet from KER whenever it receives an input, so connect a timer to `tick`. Its outputs can be used as inputs of follower [dora-openarm](https://github.com/enactic/dora-openarm) nodes:

```yaml
nodes:
  - id: leader
    build: pip install dora-openarm-ker
    path: dora-openarm-ker
    # args: --hampel --one-euro
    inputs:
      # 250Hz
      tick: dora/timer/millis/4
    outputs:
      - follower_position_left
      - follower_position_right
      - metadata

  - id: follower-right
    build: pip install dora-openarm
    path: dora-openarm
    args: "--side right --start-on-startup"
    inputs:
      request_state: leader/follower_position_right
      move_position: leader/follower_position_right
    outputs:
      - state
      - status

  - id: follower-left
    build: pip install dora-openarm
    path: dora-openarm
    args: "--side left --start-on-startup"
    inputs:
      request_state: leader/follower_position_left
      move_position: leader/follower_position_left
    outputs:
      - state
      - status
```

Then build and run your dataflow:

```bash
dora build dataflow.yaml --uv
dora run dataflow.yaml --uv
```

See [`dataflow-ker.yaml` in dora-openarm-data-collection](https://github.com/enactic/dora-openarm-data-collection/blob/main/dataflow-ker.yaml) for a complete data collection example with cameras and a recorder.

---

## Options

| Option               | Description                                                                                                                  |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `--hampel`           | Enable a Hampel filter that suppresses spikes in encoder values.                                                             |
| `--one-euro`         | Enable a [1 Euro filter](https://gery.casiez.net/1euro/) that smooths arm joint jitter. Grippers are not filtered.           |
| `--min-cutoff <Hz>`  | 1 Euro cutoff at rest. Higher means less lag and more jitter. Default: `5.0`.                                                |
| `--beta <Hz/(rad/s)>`| How much the cutoff rises with joint speed. Higher means less lag while moving. Default: `5.0`.                              |
| `--d-cutoff <Hz>`    | 1 Euro cutoff for the velocity estimate. Default: `5.0`.                                                                     |
| `--ker-rate-hz <Hz>` | KER streaming rate, used to turn the device frame counter into sample time for the 1 Euro filter. Default: `1000`.          |

The 1 Euro options take effect only with `--one-euro`. The Hampel filter runs first, so spikes are removed before smoothing.

The 1 Euro filter delays arm joints by roughly `1 / (2π · cutoff)`: about 30 ms at rest with the defaults, about 10 ms at 2 rad/s. Grippers are not delayed, so in recorded data they lead the arm by that amount.

To tune, start with `--beta 0` and lower `--min-cutoff` until jitter at rest is gone, then raise `--beta` until fast motion no longer lags.

## Inputs

| ID                | Description                                                                                                                                  |
| ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| any (e.g. `tick`) | Each input triggers reading a received packet from KER. Its value is ignored. If no new packet has been received, nothing is output. |

## Outputs

| ID                        | Type                                    | Description                                                                                       |
| ------------------------- | --------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `metadata`                | `string` (JSON)                         | KER device metadata such as `hw`, `fw` and `updated`. Sent only once on startup.                  |
| `follower_position_right` | `struct<qpos: list<float32>>`           | Target positions for the right follower arm: 7 joints and 1 gripper (8 values) in radians.       |
| `follower_position_left`  | `struct<qpos: list<float32>>`           | Target positions for the left follower arm: 7 joints and 1 gripper (8 values) in radians.        |

Each output has a `timestamp` metadata in nanoseconds.

---

## Development

See [dev/README.md](dev/README.md).

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
