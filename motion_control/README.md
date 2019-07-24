# Gait
The main purpose of his project is to study quadruped gait. In the `include/motion_controller` folder there are two implementation of gaits: `walk_controller.h` and `discontinuous_gait_controller.h`.<br/>
The walk gait looks more natural, while the discontiunous gait is more stable.

Dynamic gaits like pace gait or trot gait, in which case only two foot are in contact with the ground, are harder to maintain stability. I still not able to implement these gaits. **Please let me know if you have any ideas or reference guide :)**

To implement a different gait. you can derive from `GaitController`. subscribe to `/cmd_vel` and `/cmd_pose` topics, then update the `lf, rf, lr, rr, base_link, base_footprint, pose_changed` variables.

`lf` represent the coordinate position of **l**eft **f**ront foot, and `rr` represent **r**ight **r**are foot, etc.<br/>
If anyone of the variables is changed, set `*pose_changed=true` to notify the controller_manager that a IK recalculation is required in order the change the joints' position.

### Bugs
* When the robot is spawned into gazebo simulation, it's not at the origin, despite I have set the start position in the launch file. I dont know why it happens. You'll need to manually set the robot dog to initial position before you send `/cmd_pose` or `/cmd_vel` msg, by clicking in the left panel: Models -> spotmicro -> pose, and set z=0.2, and set all the others to 0.