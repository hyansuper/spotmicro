#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <signal.h>
#include "servo_driver/i2c_pwm.h"

servo_driver::I2CPWM pwm;
int num_boards;

void relax_all_servos(){
	for(int i=0;i<num_boards;i++) {
		pwm.set_active_board(i);
		pwm.set_pwm_interval_all(0,0);
	}
}

void cb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
	ROS_INFO("GET MSG");
	for(std::vector<int>::const_iterator d=msg->data.begin();d!=msg->data.end();d+=3){
		pwm.set_active_board(*d);
		pwm.set_pwm_interval(*(d+1),0,*(d+2));
	}
}

void mySigintHandler(int sig)
{
	relax_all_servos();
	pwm.close_i2c();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_driver", ros::init_options::NoSigintHandler);
	ros::NodeHandle n("~");
	std::string i2c_dev;
	n.param<std::string>("i2c_dev", i2c_dev, "/dev/i2c-1");
	int freq;
	n.param<int>("freq", freq, 60);
	if(!n.getParam("num_boards", num_boards)){
		ROS_ERROR("num_boards param is not set.");
		ros::shutdown();
		return -1;
	}

	if(pwm.open_i2c(i2c_dev.c_str())) {
		ROS_INFO("I2C bus opened");
	} else {
		ROS_ERROR("Failed to open %s", i2c_dev.c_str());
		ros::shutdown();
		return -1;
	}
	for(int i=0;i<num_boards;i++){
		pwm.set_active_board(i);
		pwm.init_board();
		pwm.set_pwm_freq(freq);
	}
	ros::Subscriber sub = ros::NodeHandle().subscribe("servo_pwm", 10, cb);
	ros::spin();
	return 0;
}
