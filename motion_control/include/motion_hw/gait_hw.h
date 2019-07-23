#include "motion_hw/joint_hw.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "motion_hw/leg_ik.h"
#include "motion_control/motion_hardware_interface.h"

namespace motion_control
{

class GaitHW: public JointHW
{
public:
GaitHW(): JointHW(), pose_changed(true) {}
void read(const ros::Time& time, const ros::Duration& period) {
    // Read actuator state from hardware...
    // Propagate current actuator state to joints...
    JointHW::read(time, period);
}
void write(const ros::Time& time, const ros::Duration& period) {
    // enforce limits, check constraint and collision...
    // calc ik and publish tf
    if(pose_changed) {
        tf2::Transform base_link_inv = base_link.inverse();
        tf2::Transform base_footprint_inv = base_footprint.inverse();
        for(int i=0;i<4;i++)
            ik[i].calc_ik(base_footprint_inv, base_link_inv);
        pose_changed = false;
    }
    for(int i=0;i<4;i++)
        ik[i].setCommand();
    JointHW::write(time, period);// enforce limits
            
    //publish transform
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

bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    std::vector<std::string> leg_name={"lf","rf", "lr","rr"};
    for(int i=0;i<4;i++) 
        for(int j=0;j<5;j++)
            joint_name.push_back(leg_name[i]+std::to_string(j));
    if(!JointHW::init(root_nh, robot_hw_nh))
        return false;

    urdf::Model* model = new urdf::Model();
    if(!model->initParam("robot_description")) {
        ROS_ERROR("Could not find robot model for GaitHW.");
        return false;
    }
    for(int i=0;i<4;i++) {
        vec_if.registerHandle(Vector3Handle(leg_name[i], &feet[i]));
        vec_cmd_if.registerHandle(Vector3Handle(leg_name[i]+"_cmd", &feet[i]));
        ik[i]=LegIK(leg_name[i], model, &base_link, &feet[i], &cmd[i*5]);
        urdf::Pose j0 = model->getJoint(leg_name[i]+"0")->parent_to_joint_origin_transform;
        urdf::Pose j1 = model->getJoint(leg_name[i]+"1")->parent_to_joint_origin_transform;
        urdf::Pose j2 = model->getJoint(leg_name[i]+"2")->parent_to_joint_origin_transform;
        feet[i].setValue(j0.position.x+j1.position.x, j0.position.y+j1.position.y+j2.position.x, 0);
    }
    delete model;

    tf_if.registerHandle(TransformHandle("base_link", &base_link));
    tf_if.registerHandle(TransformHandle("base_footprint", &base_footprint));
    tf_cmd_if.registerHandle(TransformHandle("base_link_cmd", &base_link));
    tf_cmd_if.registerHandle(TransformHandle("base_footprint_cmd", &base_footprint));
    bool_if.registerHandle(BoolHandle("gait_pose_changed", &pose_changed));

    registerInterface(&tf_cmd_if);    
    registerInterface(&tf_if);
    registerInterface(&vec_if);
    registerInterface(&vec_cmd_if);
    registerInterface(&bool_if);

    double init_height;
    robot_hw_nh.param("init_body_height", init_height, 0.08);
    base_link.frame_id_ = "base_footprint";
    base_link.setIdentity();
    base_link.getOrigin().setZ(init_height);
    base_footprint.frame_id_ = "odom";    
    base_footprint.setIdentity();

    return true;
}

protected:

    tf2::Vector3 feet[4]; // in odom frame
    Vector3Interface vec_if;
    Vector3CommandInterface vec_cmd_if;
    tf2::Stamped<tf2::Transform> base_footprint, /* odom -> base_footprint */ 
                                base_link;/* base_footprint -> base_link */
    tf2_ros::TransformBroadcaster br;
    TransformInterface tf_if;
    TransformCommandInterface tf_cmd_if;    

    LegIK ik[4];

    bool pose_changed;
    BoolInterface bool_if;
};
}

