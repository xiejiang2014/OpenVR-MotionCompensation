#include "ITrackedDeviceServerDriver005Hooks.h"

#include "../driver/ServerDriver.h"


namespace vrmotioncompensation
{
	namespace driver
	{
		std::map<void*, ITrackedDeviceServerDriver005Hooks::_hookedAdressMapEntry<ITrackedDeviceServerDriver005Hooks::activate_t>> ITrackedDeviceServerDriver005Hooks::_hookedActivateAdressMap;

		ITrackedDeviceServerDriver005Hooks::ITrackedDeviceServerDriver005Hooks(void* iptr)
		{
			LOG(TRACE) << "ITrackedDeviceServerDriver005Hooks::ctr(" << iptr << ")";
			auto vtable = (*((void***)iptr));
			activateAddress = vtable[0];

			//查表：检查这个函数地址以前是否被 Hook 过.
			auto it = _hookedActivateAdressMap.find(activateAddress);
			if (it == _hookedActivateAdressMap.end())// 没 Hook 过
			{
				//Hook 硬件启动过程，捕获分配到的 设备 ID
				CREATE_MH_HOOK(activateHook, _activate, "ITrackedDeviceServerDriver005::Activate", iptr, 0);
				_hookedActivateAdressMap[activateAddress].useCount = 1;
				_hookedActivateAdressMap[activateAddress].hookData = activateHook;
			}
			else// 已经 Hook 过了
			{
				activateHook = it->second.hookData;
				it->second.useCount += 1;
			}
		}

		std::shared_ptr<InterfaceHooks> ITrackedDeviceServerDriver005Hooks::createHooks(void* iptr)
		{
			std::shared_ptr<InterfaceHooks> retval = std::shared_ptr<InterfaceHooks>(new ITrackedDeviceServerDriver005Hooks(iptr));
			return retval;
		}

		ITrackedDeviceServerDriver005Hooks::~ITrackedDeviceServerDriver005Hooks()
		{
			auto it = _hookedActivateAdressMap.find(activateAddress);
			if (it != _hookedActivateAdressMap.end())
			{
				if (it->second.useCount <= 1)
				{
					REMOVE_MH_HOOK(activateHook);
					_hookedActivateAdressMap.erase(it);
				}
				else
				{
					it->second.useCount -= 1;
				}
			}
		}

		vr::EVRInitError ITrackedDeviceServerDriver005Hooks::_activate(void* _this, uint32_t unObjectId)
		{
			LOG(TRACE) << "ITrackedDeviceServerDriver005Hooks::_activate(" << _this << ", " << unObjectId << ")";
			auto vtable = (*((void***)_this));
			auto activateAddress = vtable[0];
			auto it = _hookedActivateAdressMap.find(activateAddress);
			if (it != _hookedActivateAdressMap.end())
			{
				serverDriver->hooksTrackedDeviceActivated(_this, 5, unObjectId);
				return it->second.hookData.origFunc(_this, unObjectId);
			}
			else
			{
				LOG(ERROR) << "this pointer not in ITrackedDeviceServerDriver005Hooks::_hookedActivateAdressMap.";
				return vr::VRInitError_Unknown;
			}
		}
	}
}