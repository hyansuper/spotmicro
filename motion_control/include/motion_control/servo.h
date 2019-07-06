#ifndef __SM_SERVO__
#define __SM_SERVO__
#include <hardware_interface/joint_state_interface.h>
#include "motion_control/ServoConfig.h"
namespace motion_control {
struct Servo{

	Servo(bool en): enabled(en) {}
	Servo(){}
	double max_position;
	double min_position;
	bool enabled;
	double k,b;
	int *pwm;
	hardware_interface::JointStateHandle js;
	int *board;
	int *channel;
	double pwm_end;
	double pwm_start;
	double prev_angle;
	bool state_changed() {
		return prev_angle!=js.getPosition();
	}
	void calc_kb() {
		k=(pwm_end-pwm_start)/(max_position-min_position);
		b=pwm_start-k*min_position;
	}
	void calc_pwm(){
		prev_angle = js.getPosition();
		*pwm = enabled? (k* prev_angle +b) :0;
	}
	void reconfig(const ServoConfig& c){
		pwm_start = c.pwm_start;
		pwm_end = c.pwm_end;
		*board = c.board;
		*channel = c.channel;
		calc_kb();
	}


};
}
#endif