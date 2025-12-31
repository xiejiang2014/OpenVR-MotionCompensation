#include "common.h"

#include "../logging.h"
#include "IVRDriverContextHooks.h"
#include "IVRServerDriverHost004Hooks.h"
#include "IVRServerDriverHost005Hooks.h"
#include "IVRServerDriverHost006Hooks.h"
#include "ITrackedDeviceServerDriver005Hooks.h"


namespace vrmotioncompensation
{
	namespace driver
	{
		ServerDriver* InterfaceHooks::serverDriver = nullptr;

		std::shared_ptr<InterfaceHooks> InterfaceHooks::hookInterface(void* interfaceRef, std::string interfaceVersion)
		{
			std::shared_ptr<InterfaceHooks> retval;
			if (interfaceVersion.compare("IVRDriverContext") == 0)
			{
				LOG(INFO) << "创建 IVRDriverContext 的钩子 ";
				retval = IVRDriverContextHooks::createHooks(interfaceRef);
			}
			else if (interfaceVersion.compare("IVRServerDriverHost_004") == 0)
			{
				LOG(INFO) << "创建 IVRServerDriverHost_004 的钩子 ";
				retval = IVRServerDriverHost004Hooks::createHooks(interfaceRef);
			}
			else if (interfaceVersion.compare("IVRServerDriverHost_005") == 0)
			{
				LOG(INFO) << "创建 IVRServerDriverHost_005 的钩子 ";
				retval = IVRServerDriverHost005Hooks::createHooks(interfaceRef);
			}
			else if (interfaceVersion.compare("IVRServerDriverHost_006") == 0)
			{
				LOG(INFO) << "创建 IVRServerDriverHost_006 的钩子 ";
				retval = IVRServerDriverHost006Hooks::createHooks(interfaceRef);
			}
			else if (interfaceVersion.compare("ITrackedDeviceServerDriver_005") == 0)
			{
				LOG(INFO) << "创建 ITrackedDeviceServerDriver_005 的钩子 ";
				retval = ITrackedDeviceServerDriver005Hooks::createHooks(interfaceRef);
			}
			else
			{
				LOG(INFO) << "无法创建 " << interfaceVersion <<" 的钩子 ";
			}
			return retval;
		}
	}
}