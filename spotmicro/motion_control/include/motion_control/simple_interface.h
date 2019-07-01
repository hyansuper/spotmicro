#ifndef __SM_SIMPLE_INTERFACE__
#define __SM_SIMPLE_INTERFACE__
#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace motion_control{
template <class T>
class SimpleHandle {
public:
SimpleHandle(const std::string& name, T* data): name(name), data(data) {}
SimpleHandle():name(), data(NULL){}
std::string getName()const {return name;}
	std::string name;
	T* data;
};
template <class T>
class SimpleInterface: public hardware_interface::HardwareResourceManager<SimpleHandle<T>> {};
template <class T>
class SimpleClaimedInterface: public hardware_interface::HardwareResourceManager<SimpleHandle<T>, hardware_interface::ClaimResources> {};
}

#endif