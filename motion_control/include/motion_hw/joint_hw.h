#ifndef __JOINT_HW__
#define __JOINT_HW__
#include <string>
#include <vector>
#include <ros/ros.h>
#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>

namespace motion_control
{

class JointHW: public hardware_interface::RobotHW
{
public:

	bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {

		if(joint_name.size()==0 && !robot_hw_nh.getParam("joints", joint_name)) {
			ROS_ERROR("No joints param found.");
			return false;
		}
		int num_joints = joint_name.size();
	    pos.resize(num_joints);
	    eff.resize(num_joints);
	    vel.resize(num_joints);
	    cmd.resize(num_joints);
	    lim.resize(num_joints);
	 
	 	urdf::Model* model = new urdf::Model();
	    if(!model->initParam("robot_description")) {
	        ROS_ERROR("Could not find robot model for JointHW.");
	        return false;
	    }
	    for(int i=0; i<num_joints; i++){
	        hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &pos[i], &vel[i], &eff[i]);
	        jnt_state_if.registerHandle(jointStateHandle);
	        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &cmd[i]);
	        jnt_cmd_if.registerHandle(jointPositionHandle);
	        urdf::JointConstSharedPtr urdf_joint = model->getJoint(joint_name[i]);	        
	        if(!joint_limits_interface::getJointLimits(urdf_joint, lim[i])) {
	            ROS_ERROR("Could not find joint_limits");
	            return false;
	        }
	        jnt_lim_if.registerHandle(joint_limits_interface::PositionJointSaturationHandle(jointPositionHandle, lim[i]));
	    }
	    delete model;
	    registerInterface(&jnt_state_if);
	    registerInterface(&jnt_cmd_if);
	    return true;
	}
	void read(const ros::Time& time, const ros::Duration& period) {
    // Read actuator state from hardware...
    // Propagate current actuator state to joints...
		pos.assign(cmd.begin(), cmd.end());
	}
	void write(const ros::Time& time, const ros::Duration& period) {
	    // enforce limits, check constraint and collision...
	    jnt_lim_if.enforceLimits(period);        
	}
protected:
	hardware_interface::JointStateInterface jnt_state_if;
  	hardware_interface::PositionJointInterface jnt_cmd_if;
  	joint_limits_interface::PositionJointSaturationInterface jnt_lim_if;
  	std::vector<double> cmd;
  	std::vector<double> pos;
  	std::vector<double> eff;
  	std::vector<double> vel;
  	std::vector<joint_limits_interface::JointLimits> lim;
  	std::vector<std::string> joint_name;

};
}
#endif