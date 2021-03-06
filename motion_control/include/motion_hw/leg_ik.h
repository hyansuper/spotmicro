#ifndef __SM_LEG_IK__
#define __SM_LEG_IK__
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <urdf/model.h>
#include <math.h>

namespace motion_control {
class LegIK{
public:
	LegIK(){}
	LegIK(const std::string& name, const urdf::Model* model, tf2::Transform* base_link, tf2::Vector3* foot, double* js):
	name(name), base_link(base_link), foot(foot), js(js){
		
		urdf::Pose j0 = model->getJoint(name+"0")->parent_to_joint_origin_transform;
		urdf::Pose j1 = model->getJoint(name+"1")->parent_to_joint_origin_transform;
		urdf::Pose j2 = model->getJoint(name+"2")->parent_to_joint_origin_transform;
		urdf::Pose j3 = model->getJoint(name+"3")->parent_to_joint_origin_transform;
    
    	a=-j1.position.z;
    	b=j1.position.y;
    	double c=j2.position.y;
    	double d=-j2.position.z;
    	double e=j3.position.y;
    	f=sqrt(c*c+d*d);
    	offset_x=j0.position.x;
    	offset_y=j0.position.y;
    	offset_z=j0.position.z;
		a2_offset=atan2(d,c);		
		b2=b*b;
		e2f2=e*e+f*f;
		_2f=2*f;
		_2ef=2*e*f;
		f2_e2=f*f-e*e;
	}

	void calc_ik(tf2::Transform& base_footprint_inv, tf2::Transform& base_link_inv) {
		tf2::Vector3 relative = base_link_inv(base_footprint_inv(*foot));
		double x = relative.getX()-offset_x;
		double y = relative.getY()-offset_y;
		double z = relative.getZ()-offset_z;
		double y2z2 = pow(z, 2)+pow(y, 2);
		pose_cmd[0] = atan2(y, -z) - asin(b/sqrt(y2z2));
		double g2 = pow(sqrt(y2z2-b2)-a, 2) + pow(x, 2);
		double g = sqrt(g2);
		pose_cmd[2] = acos((e2f2-g2)/_2ef) - a2_offset;
		pose_cmd[1] = acos((f2_e2+g2)/(_2f*g)) - asin(x/g) + a2_offset;
	}

	void setCommand() {
		for(int i=0;i<3;i++)
			js[i]=pose_cmd[i];
	}
protected:
	std::string name;
	tf2::Transform* base_link;
	tf2::Vector3* foot;
	double* js;// 3 element array
	double pose_cmd[3];

	double a, b, f, offset_x, offset_y, offset_z, a2_offset;
	double b2, e2f2, _2ef, _2f, f2_e2;
};
}

#endif