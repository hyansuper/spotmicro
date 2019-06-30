#ifndef __SM_DISCONTINUOUS_GAIT_CONTROLLER__
#define __SM_DISCONTINUOUS_GAIT_CONTROLLER__
#include "motion_control/gait_datatype.h"
#include <urdf/model.h>
#include <math.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "motion_control/DiscontinuousGaitConfig.h"
#include <dynamic_reconfigure/server.h>
namespace motion_control {
class DiscontinuousGaitController: public GaitController {
public:
	DiscontinuousGaitController():GaitController(), server(), cmd_vel_timeout(3.0), state(STAND), new_cfg(true){
		dynamic_reconfigure::Server<DiscontinuousGaitConfig>::CallbackType f;
		f = boost::bind(&DiscontinuousGaitController::dyn_cb, this, _1, _2);
		server.setCallback(f);
	}
	bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n) {
		GaitController::init(robot_hw, n);
		urdf::Model* model = new urdf::Model();
	    if(!model->initParam("robot_description")) {
	        ROS_ERROR("Could not find robot model for gait controller.");
	        return false;
	    }
	    std::string name[4]={"lf","rr","rf","lr"};
		legseq[0].foot = lf;
		legseq[1].foot = rr;
		legseq[2].foot = rf;
		legseq[3].foot = lr;
		for(int i=0;i<4;i++){
			legseq[i].next=&legseq[(i+1)%4];
			legseq[i].prev=&legseq[(i-1)%4];
			urdf::Pose j0 = model->getJoint(name[i]+"0")->parent_to_joint_origin_transform;
			urdf::Pose j1 = model->getJoint(name[i]+"1")->parent_to_joint_origin_transform;
			legseq[i].foot_traj.shoulder_proj = tf2::Vector3(j0.position.x,j0.position.y+j1.position.y,0);
		}
		delete model;
		active_legseq=&legseq[0];

		com = base_link.getOrigin();
		com.setZ(0);
		com_ori = com;
		for(int i=0; i<4; i++) {
			if(&legseq[i]!=active_legseq) {
				com_dir += *(legseq[i].foot);
			}			
		}
		com_dir = (com_dir/3 - com_ori)/2;

	    return true;
	}
	void update(const ros::Time& time, const ros::Duration& period) {
		if(state==WALK) {
			base_footprint *= vel_d;
			ros::Duration elapse = time-cycle_start;
			if(elapse < tt) {
				double phase = elapse.toSec()/tt.toSec()*M_PI;
				*(active_legseq->foot) = active_legseq->foot_traj.transfer_ori - active_legseq->foot_traj.transfer_dir * cos(phase);
				active_legseq->foot->setZ(sin(phase)*step_height);
			} else if(elapse > tc_025) {
				active_legseq = active_legseq->next;
				state = STAND;
			} else {
				active_legseq->foot->setZ(0);
			}

			if(elapse > tt) {
				com = com_ori - com_dir * cos((elapse-tt).toSec()*ts_PI);
			}
			tf2::Vector3 com_to_base = base_footprint.inverse()(com);
			base_link.getOrigin().setX(com_to_base.getX());
			base_link.getOrigin().setY(com_to_base.getY());

			pose_changed = true;
		}
		if(state==STAND && time-vel_prev_time<cmd_vel_timeout) {
			if(new_cfg) {
				// set config
				cmd_vel_timeout=ros::Duration(config.cmd_vel_timeout);
				duty_factor = config.duty_factor;
				tc = ros::Duration(config.leg_cycle);
				ts = ros::Duration(duty_factor*config.leg_cycle);
				tt = tc-ts;
				step_height = config.step_height;
				legseq[0].foot_traj.offset_from_shoulder_proj.setValue(config.stand_offset_x, config.stand_offset_y, 0);
				legseq[3].foot_traj.offset_from_shoulder_proj.setValue(-config.stand_offset_x, config.stand_offset_y, 0);
				legseq[2].foot_traj.offset_from_shoulder_proj.setValue(config.stand_offset_x, -config.stand_offset_y, 0);
				legseq[1].foot_traj.offset_from_shoulder_proj.setValue(-config.stand_offset_x, -config.stand_offset_y, 0);
				// tc_0125 = tc*0.125;
				tc_025 = tc*0.25;
				// tc_025_PI = tc_025.toSec()/M_PI;
				ts_PI = M_PI/(tc_025-tt).toSec();
				new_cfg = false;
			}

			// start a new leg cycle
			double dur = (ts*0.5+tt).toSec();
			tf2::Vector3 axis(0, 0, 1);
			tf2::Transform base_pl(tf2::Quaternion(axis, vel_r*dur), vel*dur);
			if(vel_r) {
				base_pl.getOrigin() = vel.rotate(axis, vel_r*dur*0.5)*sin(dur*vel_r*0.5)/vel_r;
			}
			base_pl = base_footprint*base_pl;
			tf2::Vector3 foot_pl = base_pl*(active_legseq->foot_traj.shoulder_proj + active_legseq->foot_traj.offset_from_shoulder_proj);
			active_legseq->foot_traj.transfer_ori = (foot_pl + *(active_legseq->foot))*0.5;
			active_legseq->foot_traj.transfer_dir = (foot_pl - *(active_legseq->foot))*0.5;

			double sec = period.toSec();
			vel_d.getOrigin() = vel*sec;
			vel_d.getBasis().setRPY(0, 0, vel_r*sec);
			
			// com_dir = com_dir2;
			// com_ori = com_ori2;
			// com_dir2 = active_legseq->foot_traj.transfer_ori + active_legseq->foot_traj.transfer_dir;
			com_dir = active_legseq->foot_traj.transfer_ori + active_legseq->foot_traj.transfer_dir;
			for(int i=0; i<4; i++) {
				if(&legseq[i]!=active_legseq && &legseq[i]!=active_legseq->next) {
					com_dir += *(legseq[i].foot);
				}			
			}
			com_dir /= 3;
			com_ori = (com_dir + com)/2;
			com_dir = (com_dir - com)/2;
			// tf2::Vector3 com_end = com_dir + com_ori;
			// com_ori2 = (com_dir2/3 + com_end)/2;
			// com_dir2 = (com_dir2/3 - com_end)/2;
			
			state=WALK;
			cycle_start=time;
		}
		GaitController::update(time, period);
	}
protected:
	LegSequence legseq[4];
	LegSequence* active_legseq;
	GaitState state;
	ros::Time vel_prev_time;
	tf2::Vector3 vel;
	double vel_r;
	tf2::Transform vel_d;
	bool pose_cmd;
	ros::Time cycle_start;

	// com: center of mass in odom frame,
	// simplified to the center of base_link, disregarding the mass of legs
	tf2::Vector3 com_dir, com_ori,/* com_ori2, com_dir2, */com; 

    ros::Duration tc, tt, ts, tc_025;//, tc_0125;
    double ts_PI;

	DiscontinuousGaitConfig config;
    bool new_cfg;
    double step_height, duty_factor;
	ros::Duration cmd_vel_timeout;

	dynamic_reconfigure::Server<DiscontinuousGaitConfig> server;
	
	void pose_callback(const geometry_msgs::Pose::ConstPtr& msg) {
		tf2::Quaternion q;
		tf2::convert(msg->orientation,q);
		base_link.setRotation(q);
		base_link.getOrigin().setZ(msg->position.z);
		pose_changed = true;
	}
	void vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
		tf2::convert(msg->linear,vel);
		vel.setZ(0);
		vel_r = msg->angular.z;
		vel_prev_time = ros::Time::now();
	}

	void dyn_cb(DiscontinuousGaitConfig &config, uint32_t level) {
		this->config = config;
		new_cfg = true;
	}
};
}
#endif