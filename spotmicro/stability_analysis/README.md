This is a simple test to verify if the gait control is stable.
```
roslaunch stability_analysis static_stability.launch
```
Then in RViz, a green polygon will appear to indicate the support plane, which is formed by the supporing feet. a green dot is the projection of the center of mass onto the ground plane.<br/>
If the dot is within the support plane, the robot is statically stable; It turns red if the dot goes outside the support plane.<br/>
Stability margin is the shortest distance from the dot to the edge of the support plane, greater the number, more statically stable the robot is.

If you don't see those markers in RViz, you need to click "Add" and select "Marker", then set Marker Topic to "stability_marker".

---
Note:

* Center of mass is simplifed to the center of /base_link, so it's inaccurate.
* Static tability does not take dynamic factors(like inertial) into consideration. When the robot is moving fast, dynamic stability should be considered, but it's out of my knowledge.
