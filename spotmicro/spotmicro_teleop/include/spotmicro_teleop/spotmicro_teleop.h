#ifndef __SM_TELEOP__
#define __SM_TELEOP__
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace spotmicro_teleop {
class SpotmicroTeleop {
public:
	SpotmicroTeleop(ros::NodeHandle *n, ros::NodeHandle *param_n): n(n), param_n(param_n),
		pose_pub(n->advertise<geometry_msgs::Pose>("cmd_pose", 1)),
		vel_pub(n->advertise<geometry_msgs::Twist>("cmd_vel", 1))
	{
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener listener(tfBuffer);
		// try {
			// tfBuffer.waitForTransform("bask_footprint", "base_link", ros::Duration(10));
    		geometry_msgs::TransformStamped base_link = tfBuffer.lookupTransform("base_footprint", "base_link", ros::Time(0), ros::Duration(5));
    		pose_msg.position.z = base_link.transform.translation.z;
    		pose_msg.orientation = base_link.transform.rotation;
	    // } catch (tf2::TransformException &ex) {
	    // 	ROS_ERROR("%s",ex.what());
	    // 	ros::shutdown();
	    // }
	    ros::NodeHandle lim_nh(*param_n, "limits");
    	lim_nh.param("vel_x", scale.vel_x, 0.035);
    	lim_nh.param("vel_y", scale.vel_y, 0.025);
    	lim_nh.param("vel_angular", scale.vel_angular, 0.2);
    	lim_nh.param("pose_dz", scale.pose_dz, 0.001);
    	lim_nh.param("pose_z_min", scale.pose_z_min, 0.1);
    	lim_nh.param("pose_z_max", scale.pose_z_max, 0.19);
    	lim_nh.param("pose_r", scale.pose_r, 0.2);
    	lim_nh.param("pose_p", scale.pose_p, 0.2);
    	lim_nh.param("pose_y", scale.pose_y, 0.3);
	}
	virtual void update() = 0;
struct Scale {
	double vel_x, vel_y, vel_angular, pose_dz, pose_r, pose_p, pose_y, pose_z_max, pose_z_min;
	void limit_vel(tf2::Vector3 &v) {
		double ang = atan2(v.getY(), v.getX());
		tf2::Vector3 limit(vel_x*cos(ang), vel_y*sin(ang), 0);
		if(v.length2() > limit.length2())
			v = limit;
	}
	void limit_z(double &z) {
		if(z > pose_z_max) z=pose_z_max;
		if(z < pose_z_min) z=pose_z_min;
	}
};
protected:	
	Scale scale;
	ros::NodeHandle *n, *param_n;
	geometry_msgs::Pose pose_msg;
	geometry_msgs::Twist vel_msg;
	ros::Publisher vel_pub, pose_pub;
};
}
#endif