#include <iostream>
#include <ros/ros.h>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "combo_control_node");
    ros::NodeHandle nh;

    int num_threads;
    nh.param("num_threads", num_threads, 2);
    ros::AsyncSpinner spinner(num_threads);    
    spinner.start();
    
    combined_robot_hw::CombinedRobotHW hw;
    bool init_success = hw.init(nh,nh);

    controller_manager::ControllerManager cm(&hw,nh);

    double r;
    nh.param("update_rate", r, 30.0);
    ros::Duration period(1.0/r);

    ROS_INFO("control manager started");
    while(ros::ok()){
        hw.read(ros::Time::now(), period);
        cm.update(ros::Time::now(), period);
        hw.write(ros::Time::now(), period);
        period.sleep();
    }

    spinner.stop();
    return 0;
}