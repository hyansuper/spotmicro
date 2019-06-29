#ifndef __SM_GAIT_CONTROLLER__
#define __SM_GAIT_CONTROLLER__
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "motion_control/leg_ik.h"

namespace motion_control {
class GaitController: public controller_interface::MultiInterfaceController
						<hardware_interface::PositionJointInterface>
{
public:	
	GaitController(): pose_changed(true) {}
	bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n) {
		hardware_interface::PositionJointInterface* pos_cmd_if = robot_hw->get<hardware_interface::PositionJointInterface>();
		if(!pos_cmd_if) return false;
		
		double init_height;
	    n.param("init_body_height", init_height, 0.17);
	    base_link.frame_id_ = "base_footprint";
	    base_link.setIdentity();
	    base_link.getOrigin().setZ(init_height);
	    base_footprint.frame_id_ = "odom";    
	    base_footprint.setIdentity();

		urdf::Model* model = new urdf::Model();
	    if(!model->initParam("robot_description")) {
	        ROS_ERROR("Could not find robot model for motion controller.");
	        return false;
	    }
		std::vector<std::string> leg_name={"lf","rf", "lr","rr"};
    	for(int i=0;i<4;i++) {
    		hardware_interface::JointHandle jnt_handles[3];
    		for(int j=0;j<3;j++) {
    			jnt_handles[j] = pos_cmd_if->getHandle(leg_name[i]+std::to_string(j));
    		}
    		ik[i]=LegIK(leg_name[i], model, &feet[i], jnt_handles);
	        urdf::Pose j0 = model->getJoint(leg_name[i]+"0")->parent_to_joint_origin_transform;
	        urdf::Pose j1 = model->getJoint(leg_name[i]+"1")->parent_to_joint_origin_transform;
	        feet[i].setValue(j0.position.x, j0.position.y+j1.position.y, 0);        	
    	}
    	lf = &feet[0]; rf = &feet[1]; lr = &feet[2]; rr = &feet[3];
    	delete model;
		nh = n;
		return true;
	}
	void update(const ros::Time& time, const ros::Duration& period) {
		if(pose_changed) {
	        tf2::Transform base_link_inv = base_link.inverse();
	        tf2::Transform base_footprint_inv = base_footprint.inverse();
	        for(int i=0;i<4;i++)
	            ik[i].calc_ik(base_footprint_inv, base_link_inv);
	        pose_changed = false;
	    }
	    for(int i=0;i<4;i++){
	        ik[i].setCommand();
	    }
		base_footprint.stamp_ = time;
	    base_link.stamp_ = time;
	    geometry_msgs::TransformStamped base_link_msg = tf2::toMsg(base_link);
	    base_link_msg.child_frame_id = "base_link";
	    geometry_msgs::TransformStamped base_footprint_msg = tf2::toMsg(base_footprint);
	    base_footprint_msg.child_frame_id = "base_footprint";
	    base_footprint_msg.header.seq ++;
	    base_link_msg.header.seq ++;
	    br.sendTransform(base_footprint_msg);
	    br.sendTransform(base_link_msg);
	}
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
	
	tf2::Stamped<tf2::Transform> base_footprint, /* odom -> base_footprint */ 
                                base_link;/* base_footprint -> base_link */
    tf2_ros::TransformBroadcaster br;
    LegIK ik[4];
    tf2::Vector3 feet[4], *lf, *rf, *lr, *rr;
	bool pose_changed;
	ros::Subscriber pose_sub, vel_sub;
	ros::NodeHandle nh;
};
}


#endif