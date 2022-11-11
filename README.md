# ROS2 Static Tree Merger
The purpose of this package is to link a subordinate TF tree to the main robot TF tree in the case in which only the transformation between two non-root links is known.

## Example usage
Assume you have a robot and a rgb-d camera. Their TF trees might look like the ones below:

![Alt text](assets/known_tree.png?raw=true "Known transformations")

In these situations, hand-eye calibration is often used to estimate the rigid transformation between the camera `rgb_link` and the `camera_mount` on the robot. However, ROS does not allow linking of TF trees like shown above (red arrow). The static_tree_merger allows users to link the two trees in a ROS-compliant way by dynamically querying the `rbg_link -> camera_base` transform, and using it to publish `camera_base -> camera_mount`. In practice:
```bash
ros2 launch static_tree_merger tree_merger.launch.py parent_link:=camera_mount child_link:=rgb_link child_link_root:=camera_base child_to_parent_tf:="0 0 0 0 0 0 1"
```
where `child_to_parent_tf` is the known transform `rgb_link -> camera_mount` (translation xyz, rotation quaternion xyzw). The result is the TF tree below:

<br></br>
![Alt text](assets/final_tf.png?raw=true "Final TF tree")

## Limitations
The node assumes that the `child_link -> child_link_root` and `child_link -> parent_link` transforms are static.

## Requirements
Tested on ROS2 Foxy and ROS2 Humble.
<!-- ```bash
# example
ros2 launch static_tree_merger tree_merger.launch.py parent_link:=yumi_base_link child_link:=rgb_camera_link child_link_root:=camera_base child_to_parent_tf:="0.220237, -0.03886157, 0.86828830, -0.68238, 0.69057, -0.17517, 0.163618"
``` -->
