#include "motion_hw/joint_hw.h"
#include "motion_hw/gait_hw.h"
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(motion_control::JointHW, hardware_interface::RobotHW);
PLUGINLIB_EXPORT_CLASS(motion_control::GaitHW, hardware_interface::RobotHW);