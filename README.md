# SpotMicro
Most computation runs on PC or laptop, and send command to the robot through wifi.

## Install
```
mkdir -p spotmicro_ws/src
cd spotmicro/src
git clone https://github.com/hyansuper/spotmicro.git
cd ..
catkin_make
```

## Run
```
roslaunch spotmicro bringup.launch mode:=[rviz/sim/pi]
```

### Input topic:
* /cmd_vel: control angular and linear velocity just like the turtlesim program.
* /cmd_pose: control the pose(RPY and body heigt) of the body.

You can run the following to control the robot with a Logitech F710 gamepad. The gamepad node will publish msg to the above topics.
```
roslaunch spotmicro_teleop gamepad.launch
```

Also, some parameters about the gait can be dynamically reconfigured by running
```
rosrun rqt_reconfigure rqt_reconfigure
```

### Output topic:
* /servo_pwm: the Int32MultiArray msg has a data field, specifying the board, channel and PWM signal for each servo. you can subcribe to this topic and send PWM signal to each servo using ESP32 or Raspberry Pi as controller. The default update rate is 30 hz.<br/>
The following is the **pseudo code** for the controller:
```
count = msg.data.size()/3
for(i=0; i<count; i+=3):
	board = msg.data[i]
	channel = msg.data[i+1]
	pwm_value = msg.data[i+2]
	pwm.set_address(board)
	pwm.set_pwm(channel, 0, pwm_value)
```
The servo_driver pkg is meant to be run on a Raspberry Pi as a controller board on the real robot. To compile the pkg on Raspberry Pi, change servo_driver/CMakeLists.txt.pi to servo_driver/CMakeLists.txt.<br/>
This pkg is not needed if you only want to try RViz animation or gazebo simulation.

All the other pkg will run on PC.
