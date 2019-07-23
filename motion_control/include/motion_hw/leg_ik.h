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
		urdf::Pose j4 = model->getJoint(name+"4")->parent_to_joint_origin_transform;
		urdf::Pose jt = model->getJoint(name+"t")->parent_to_joint_origin_transform;
		
    	offset_x=j0.position.x+j1.position.x;
    	offset_y=j0.position.y;
    	offset_z=j0.position.z;
    	a=-j1.position.z;
		b=j1.position.y+j2.position.x;
		b2=b*b;

    	double c=j2.position.y;
    	double e=jt.position.y;
    	f=c;
		e2f2=e*e+f*f;
		_2f=2*f;
		_2ef=2*e*f;
		f2_e2=f*f-e*e;
		j2k2=pow(0.024,2)+pow(0.055,2);
		_2jk=2*0.024*0.055;
		j2_k2=pow(0.024,2)-pow(0.055,2);
		_2j=0.024*2;
	}

	void calc_ik(tf2::Transform& base_footprint_inv, tf2::Transform& base_link_inv) {
		tf2::Vector3 relative = base_link_inv(base_footprint_inv(*foot));
		double x = relative.getX()-offset_x;
		double y = relative.getY()-offset_y;
		double z = relative.getZ()-offset_z;
		double y2z2 = pow(z, 2)+pow(y, 2);
		pos_cmd[0] = atan2(y, -z) - asin(b/sqrt(y2z2));
		double h=sqrt(y2z2-b2);
		double g2 = pow(h-a, 2) + pow(x, 2);
		double g = sqrt(g2);
		pos_cmd[2] = acos((e2f2-g2)/_2ef);
		pos_cmd[1] = acos((f2_e2+g2)/(_2f*g)) - asin(x/g);

		double p=pos_cmd[1]+pos_cmd[2]-M_PI*.5;
		double xx=0.08*cos(p)-x;
		double zz=h-0.08*sin(p)+a;
		double i2=pow(xx, 2) + pow(zz, 2);
		double i=sqrt(i2);
		pos_cmd[3] = atan2(xx,zz)+acos((i2+j2_k2)/(_2j*i))-M_PI*.5;
		pos_cmd[4] = acos((j2k2-i2)/_2jk);
	}

	void setCommand() {
		for(int i=0;i<5;i++)
			js[i]=pos_cmd[i];
	}
protected:
	std::string name;
	tf2::Transform* base_link;
	tf2::Vector3* foot;
	double* js;// 3 element array
	double pos_cmd[5];

	double a, b, f, offset_x, offset_y, offset_z;
	double b2, e2f2, _2ef, _2f, f2_e2, j2k2, _2jk, _2j, j2_k2;
};
}

#endif