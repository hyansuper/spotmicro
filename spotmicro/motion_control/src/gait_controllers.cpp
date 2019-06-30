#include <pluginlib/class_list_macros.hpp>
#include "motion_control/walk_controller.h"
#include "motion_control/discontinuous_gait_controller.h"
PLUGINLIB_EXPORT_CLASS(motion_control::WalkController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(motion_control::DiscontinuousGaitController, controller_interface::ControllerBase);