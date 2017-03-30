# planning

This is the package to control the CPR Mover4 arm for our purposes.

There are Launch-Files of importance
- `cpr_move_group_python_interface\ .launch`
  - This launches a CLI interface to control the arm's final position. NOTE: this is not the same as the end effector position. Also use caution when doing this as it may not account for potential collisions with un modeled components.
- `melle_arm_interface_node.launch`
  - This is launches the node to communicate with the rest of the Mell-E ecosystem

The `melle_arm_interface_node` interacts with the rest of the Mell-E ecosystem with the following messages
- This node subscribes to the `down_cam_msg` and requires an x and y coordinate in the image to plan to a particular position.
- This node publishes to `arm_state` which is just a standard string message
  - "in_progress" indicates that the arm is in the process of picking up a litter item.
  - "pickup_done" idicates that the arm has finished picking up a piece of litter.
