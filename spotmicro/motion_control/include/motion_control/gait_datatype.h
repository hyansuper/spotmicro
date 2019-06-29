#ifndef __SM_GAIT_DATATYPE__
#define __SM_GAIT_DATATYPE__
#include <tf2/LinearMath/Vector3.h>

namespace motion_control {
	enum GaitState {STAND, WALK};

	struct FootTrajectory { // in odom frame
		tf2::Vector3 shoulder_proj, offset_from_shoulder_proj,
			transfer_dir, transfer_ori;
		ros::Time transfer_start;
	};
	struct LegSequence {		
		tf2::Vector3 *foot;
		FootTrajectory foot_traj;
		LegSequence* next;
		LegSequence* prev;
	};
}
#endif