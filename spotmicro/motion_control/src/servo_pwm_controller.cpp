#include <controller_interface/multi_interface_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits.h>
#include <hardware_interface/joint_state_interface.h>
#include "motion_control/servo.h"
#include <std_msgs/Int32MultiArray.h>
#include "motion_control/EnableServo.h"
#include "motion_control/ConfigServo.h"

#define STREAMER(astr, ...) {std::stringstream ss; ss __VA_ARGS__; astr=ss.str();}

namespace motion_control {
class ServoPWMController: public controller_interface::MultiInterfaceController
						<hardware_interface::JointStateInterface/*, BoolInterface*/>
{
public:
	bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n) {
		
		if(!n.hasParam("servo_config")) {
			ROS_ERROR("No servo configurations found.");
			return false;
		}
		ros::NodeHandle config_nh(n, "servo_config");
		hardware_interface::JointStateInterface *jnt_state_if = robot_hw->get<hardware_interface::JointStateInterface>();
		const std::vector<std::string>& jname = jnt_state_if->getNames();
		pwm_msg.data.resize(jname.size()*3);
		pwm_msg.layout.dim.resize(2);
		pwm_msg.layout.dim[0].stride=pwm_msg.data.size();
		pwm_msg.layout.dim[0].size=jname.size();
		pwm_msg.layout.dim[0].label="pwm_array";
		pwm_msg.layout.dim[1].stride=3;
		pwm_msg.layout.dim[1].size=3;
		pwm_msg.layout.dim[1].label="board/channel/pwm";
		
		bool enable_servo;
		n.param("enable_servo", enable_servo, true);

		urdf::Model* model = new urdf::Model();
	    if(!model->initParam("robot_description")) {
	        ROS_ERROR("Could not find robot model for ServoPWMController.");
	        return false;
	    }

		for(int i=0;i<jname.size();i++) {
			
			if(!config_nh.hasParam(jname[i])) {
				ROS_ERROR_STREAM("Could not find servo config for "<< jname[i]);
				return false;
			}
			ros::NodeHandle servo_nh(config_nh, jname[i]);
			Servo s(enable_servo);
			s.board = &pwm_msg.data[i*3];
			s.channel = &pwm_msg.data[i*3+1];
			s.pwm = &pwm_msg.data[i*3+2];
			s.js = jnt_state_if->getHandle(jname[i]);
			if(!(servo_nh.getParam("pwm_start", s.pwm_start)
				&& servo_nh.getParam("pwm_end", s.pwm_end)
				&& servo_nh.getParam("board", *s.board)
				&& servo_nh.getParam("channel", *s.channel))) {
				ROS_ERROR_STREAM("Error in servo config params for "<< jname[i]);
				return false;
			}
			joint_limits_interface::JointLimits lim;
			urdf::JointConstSharedPtr urdf_joint = model->getJoint(jname[i]);
			if(!getJointLimits(urdf_joint, lim)) {
                ROS_ERROR_STREAM("Could not find joint limits for "<< jname[i]);
                return false;
            }
            s.max_position = lim.max_position;
            s.min_position = lim.min_position;
            s.calc_kb();
			servo.insert(std::pair<std::string,Servo>(jname[i], s));		
		}		
		// jnt_changed = robot_hw->get<BoolInterface>()->getHandle("joint_changed").data;
		delete model;
		nh = n;
		return true;
	}
	void update(const ros::Time& time, const ros::Duration& period) {
		// if(*jnt_changed) {
		bool state_changed = false;
		for(std::map<std::string, Servo>::iterator s=servo.begin(); s!=servo.end(); s++) 
			if(s->second.state_changed()) {
				state_changed = true;
				break;
			}
		if(state_changed)
			pub_pwm();			
			// pwm_pub.publish(pwm_msg);
			// *jnt_changed = false;
		// }
	}
	void starting(const ros::Time& time) {
		pwm_pub = nh.advertise<std_msgs::Int32MultiArray>("servo_pwm", 1);
		config_servo_srv = nh.advertiseService("config_servo", &ServoPWMController::config_servo_cb, this);
    	enable_servo_srv = nh.advertiseService("enable_servo", &ServoPWMController::enable_servo_cb, this);
	}
	void stopping(const ros::Time& time) {
		pwm_pub.shutdown();
		config_servo_srv.shutdown();
		enable_servo_srv.shutdown();
	}

protected:
	std::map<std::string, Servo> servo;
	ros::Publisher pwm_pub;
	ros::ServiceServer enable_servo_srv, config_servo_srv;
	ros::NodeHandle nh;	
	// bool* jnt_changed;
	std_msgs::Int32MultiArray pwm_msg;

	void pub_pwm() {
		for(std::map<std::string, Servo>::iterator s=servo.begin(); s!=servo.end(); s++) 
			s->second.calc_pwm();
		pwm_pub.publish(pwm_msg);
	}
	
	bool set_servo_config_param(const ros::NodeHandle& nh, ServoConfig& sc) {		
		const std::string config_namespace = "servo_config/" + sc.joint;
		ros::NodeHandle config_nh(nh, config_namespace);
		config_nh.setParam("pwm_start", sc.pwm_start);
		config_nh.setParam("pwm_end", sc.pwm_end);
		config_nh.setParam("board", sc.board);
		config_nh.setParam("channel", sc.channel);
		return true;
	}

	bool config_servo_cb(ConfigServo::Request &req, ConfigServo::Response &res)
	{	
		if(servo.find(req.config.joint) != servo.end()) {
	        servo.find(req.config.joint)->second.reconfig(req.config);
	        // *jnt_changed = true;
	        pub_pwm();
	        set_servo_config_param(nh, req.config);
	        res.success = true;
	    } else {
	    	res.success = false;
	        STREAMER(res.extra_info, <<"Could not find servo named " << req.config.joint);	        
	    }
	    return true;
	}

	bool enable_servo_cb(EnableServo::Request &req, EnableServo::Response &res)
	{
	    if(req.joint=="ALL" || req.joint=="all") {
	    	for(std::map<std::string, Servo>::iterator s=servo.begin(); s!=servo.end(); s++)
	    		s->second.enabled = req.enable;
	        res.success = true;
	        // *jnt_changed = true;
	        pub_pwm();
	    } else if(servo.find(req.joint) != servo.end()) {
	    	servo.find(req.joint)->second.enabled = req.enable;
	        res.success = true;
	        // *jnt_changed = true;
	        pub_pwm();
	    } else {
	    	STREAMER(res.extra_info, << "Could not find servo named " << req.joint);
	        res.success = false;
	    }
	    return true;
	}


};
}

PLUGINLIB_EXPORT_CLASS(motion_control::ServoPWMController, controller_interface::ControllerBase);

