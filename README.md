# SpotMicro
Most computation runs on PC or laptop, and send command to the robot through wifi.

## dependency:
I use ROS melodic and gazebo-9 on Ubuntu 18.04. I think it's compatible with ROS kinetic.

## Install

### On PC
```
cd catkin_ws/src
git clone https://github.com/hyansuper/spotmicro.git
cd ..
catkin_make
```

### On Raspberry Pi (optional, only if you have built the robot, otherwise you can just test the robot in simulation)
```
sudo apt-get install libi2c-dev -y
cd catkin_ws/src
git clone --branch rpi https://github.com/hyansuper/spotmicro.git
cd ..
catkin_make
chmod +x src/spotmicro/pi_ros_remote.sh 
```

## Run
```
roslaunch spotmicro bringup.launch mode:=[rviz/sim/pi] gait:=[walk/discontinuous]
rostopic pub -r 3 /cmd_vel [tab][tab]
... x: 0.01 ...
```
to launch it in RViz animation(the default), gazebo simulation or bring up the real robot.<br/>
To bring up the real robot, you may need to customize `spotmicro/pi_ros_remote.sh` and `spotmicro/spotmicro/launch/machine.launch` files

I implemented two gaits. The walk gait is more natrual, but discontinuous gait is more stable.

### Input topic:
* `/cmd_vel`: control angular and linear velocity just like the turtlesim program.
* `/cmd_pose`: control the pose(RPY and body heigt) of the body.

You can run the following to control the robot with a Logitech F710 gamepad. The gamepad node will publish msg to the above topics.
```
roslaunch spotmicro_teleop gamepad.launch
```

Also, some parameters about the gait can be dynamically reconfigured
```
rosrun rqt_reconfigure rqt_reconfigure
```

### Output topic:
* `/servo_pwm`: the Int32MultiArray msg has a data field, specifying the board, channel and PWM signal for each servo. you can subcribe to this topic and send PWM signal to each servo using ESP32 or Raspberry Pi as controller. The default update rate is 30 hz.<br/>
The following is the **pseudo code** for the controller:
```
for(i=0; i<msg.data.size(); i+=3):
	board = msg.data[i]
	channel = msg.data[i+1]
	pwm_value = msg.data[i+2]
	pwm.set_address(board)
	pwm.set_pwm(channel, 0, pwm_value)
```
