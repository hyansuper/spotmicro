#include <pluginlib/class_list_macros.hpp>
#include "motion_controller/walk_controller.h"
#include "motion_controller/discontinuous_gait_controller.h"
#include "motion_controller/servo_pwm_controller.h"
PLUGINLIB_EXPORT_CLASS(motion_control::ServoPWMController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(motion_control::WalkController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(motion_control::DiscontinuousGaitController, controller_interface::ControllerBase);