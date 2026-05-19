# data_collection

Minimal tools for Mirte Master data collection and arm teleoperation.

## What it does

- Arm teleop with keyboard increments (joint trajectory commands).
- Data capture with spacebar (saves RGB, depth, and IR images).

## Usage

In separate terminals:

```bash
# Base teleop (existing)
ros2 launch mirte_teleop teleop_key.launch.py

# Arm teleop
ros2 run data_collection arm_teleop

# Data collection
ros2 run data_collection data_collector
```

## Key bindings

Arm teleop:
- q/a: shoulder pan +/ -
- w/s: shoulder lift +/ -
- e/d: elbow +/ -
- r/f: wrist +/ -
- t/g: gripper close/open (not working yet, only opens)
- Ctrl+C: exit

Data collector:
- space: save images
- Ctrl+C: exit

## Topics (compressed)

- RGB cameras: `/camera/color/image_raw/compressed`, `/gripper_camera/image_raw/compressed`
- Depth: `/camera/depth/image_raw/compressedDepth`
- IR: `/camera/ir/image_raw/compressed`

Images are saved to `./data/data_collection/<timestamp>/`.
Depth images also get a colorized preview with `_vis` appended.

Known bug: Sometimes not all image streams are working and you get warnings that one some/no images were saved. Running
```bash
sudo systemctl restart mirte-ros
```
on the Mirte (via SSH) should fix this (for a while).
