#ifndef __SM_MOTION_HARDWARE_INTERFACE__
#define __SM_MOTION_HARDWARE_INTERFACE__

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include "motion_control/simple_interface.h"
namespace motion_control{

typedef SimpleHandle<bool> BoolHandle;
typedef SimpleHandle<double> DoubleHandle;
typedef SimpleHandle<tf2::Vector3> Vector3Handle;
typedef SimpleHandle<tf2::Quaternion> QuaternionHandle;
typedef SimpleHandle<tf2::Transform> TransformHandle;

typedef SimpleInterface<bool> BoolInterface;
typedef SimpleInterface<tf2::Vector3> Vector3Interface;
typedef SimpleInterface<tf2::Quaternion> QuaternionInterface;
typedef SimpleInterface<double> DoubleInterface;
typedef SimpleInterface<tf2::Transform> TransformInterface;

typedef SimpleClaimedInterface<tf2::Vector3> Vector3CommandInterface;
typedef SimpleClaimedInterface<tf2::Quaternion> QuaternionCommandInterface;
typedef SimpleClaimedInterface<tf2::Transform> TransformCommandInterface;

}

#endif