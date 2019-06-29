#ifndef __SM_GAMEPAD__
#define __SM_GAMEPAD__
#include "spotmicro_teleop/spotmicro_teleop.h"
#include <sensor_msgs/Joy.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
namespace spotmicro_teleop {
class Gamepad: public SpotmicroTeleop {
public:
	Gamepad(ros::NodeHandle *n, ros::NodeHandle *param_n): SpotmicroTeleop(n, param_n),
		new_cmd(false), walking(false), dz(0),
		joy_sub(n->subscribe<sensor_msgs::Joy>("joy", 2, &Gamepad::joy_cb, this)) {}
	void update() {
		if(dz!=0.0 || new_cmd){
			pose_msg.position.z += dz*scale.pose_dz;		
			scale.limit_z(pose_msg.position.z);
			pose_pub.publish(pose_msg);
			new_cmd = false;
		}
		if(walking) {
			vel_pub.publish(vel_msg);
		}
	}
protected:
	void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg) {
		tf2::Vector3 vel_scaled(joy_msg->axes[3]*scale.vel_x, joy_msg->axes[2]*scale.vel_y, 0);
		scale.limit_vel(vel_scaled);
		vel_msg.linear.x = vel_scaled.getX();
		vel_msg.linear.y = vel_scaled.getY();
		vel_msg.angular.z = joy_msg->axes[4]*scale.vel_angular;
		dz = joy_msg->axes[5];
		q.setRPY(0, joy_msg->axes[1]*scale.pose_p, joy_msg->axes[0]*scale.pose_y);
		pose_msg.orientation = tf2::toMsg(q);
		if(joy_msg->buttons[9]) {
			walking = !walking;		
		}
		new_cmd = true;
	}
	bool new_cmd, walking;
	double dz;
	ros::Subscriber joy_sub;
	tf2::Quaternion q;
};
}
#endif