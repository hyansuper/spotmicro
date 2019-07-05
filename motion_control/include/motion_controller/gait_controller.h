#ifndef __SM_GAIT_CONTROLLER__
#define __SM_GAIT_CONTROLLER__
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "motion_control/motion_hardware_interface.h"

namespace motion_control {
class GaitController: public controller_interface::MultiInterfaceController
						<Vector3CommandInterface,
						TransformCommandInterface,
						BoolInterface>
{
public:
	GaitController(): nh("~") {}
	bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n) {
		Vector3CommandInterface* vec_cmd_if = robot_hw->get<Vector3CommandInterface>();
		if(!vec_cmd_if) return false;
		TransformCommandInterface* tf_cmd_if = robot_hw->get<TransformCommandInterface>();
		if(!tf_cmd_if) return false;
		lf = vec_cmd_if->getHandle("lf_cmd").data;
		rf = vec_cmd_if->getHandle("rf_cmd").data;
		rr = vec_cmd_if->getHandle("rr_cmd").data;
		lr = vec_cmd_if->getHandle("lr_cmd").data;
		base_footprint = tf_cmd_if->getHandle("base_footprint_cmd").data;
		base_link = tf_cmd_if->getHandle("base_link_cmd").data;
		BoolInterface* b_if = robot_hw->get<BoolInterface>();
		if(!b_if) return false;
		pose_changed = b_if->getHandle("gait_pose_changed").data;
		return true;
	}
	virtual void update(const ros::Time& time, const ros::Duration& period)=0;
	void starting(const ros::Time& time) {
		pose_sub = nh.subscribe("cmd_pose", 1, &GaitController::pose_callback, this);
		vel_sub = nh.subscribe("cmd_vel", 1, &GaitController::vel_callback, this);
	}
	void stopping(const ros::Time& time) {
		vel_sub.shutdown();
		pose_sub.shutdown();
	}
protected:
	virtual void pose_callback(const geometry_msgs::Pose::ConstPtr& msg)=0;
  	virtual void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)=0;
	
	tf2::Vector3 *lr, *lf, *rr, *rf;
	tf2::Transform *base_link, *base_footprint;
	bool *pose_changed;
	ros::Subscriber pose_sub, vel_sub;
	ros::NodeHandle nh;
};
}


#endif