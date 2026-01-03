#include "IVRDriverContextHooks.h"


namespace vrmotioncompensation
{
	namespace driver
	{
		HookData<IVRDriverContextHooks::getGenericInterface_t> IVRDriverContextHooks::getGenericInterfaceHook;
		std::map<std::string, std::shared_ptr<InterfaceHooks>> IVRDriverContextHooks::_hookedInterfaces;

		IVRDriverContextHooks::IVRDriverContextHooks(void* iptr)
		{
			if (!_isHooked)
			{
				CREATE_MH_HOOK(getGenericInterfaceHook, _getGenericInterface, "IVRDriverContext::GetGenericInterface", iptr, 0);
				_isHooked = true;
			}
		}

		IVRDriverContextHooks::~IVRDriverContextHooks()
		{
			if (_isHooked)
			{
				REMOVE_MH_HOOK(getGenericInterfaceHook);
				_isHooked = false;
			}
		}

		std::shared_ptr<InterfaceHooks> IVRDriverContextHooks::createHooks(void* iptr)
		{
			std::shared_ptr<InterfaceHooks> retval = std::shared_ptr<InterfaceHooks>(new IVRDriverContextHooks(iptr));
			return retval;
		}

		std::shared_ptr<InterfaceHooks> IVRDriverContextHooks::getInterfaceHook(std::string interfaceVersion)
		{
			auto it = _hookedInterfaces.find(interfaceVersion);
			if (it != _hookedInterfaces.end())
			{
				return it->second;
			}
			return nullptr;
		}



		void* IVRDriverContextHooks::_getGenericInterface(vr::IVRDriverContext* _this, const char* pchInterfaceVersion, vr::EVRInitError* peError)
		{
			//实测这里被调用了很多次,  pchInterfaceVersion 有以下这些值, 且有的出现了多次
			//  IVRServerDriverHost_006
			//	IVRSettings_003
			//	IVRProperties_001
			//	IVRDriverLog_001
			//	IVRDriverManager_001
			//	IVRResources_001
			//	IVRRenderModels_006
			//	IVRRenderModelsInternal_XXX
			//	IVRSettingsInternal_001
			//	IVRPaths_002
			//	IVRPathsInternal_XXX
			//	IVRPropertiesInternal_001
			//	IVRServerInternal_XXX
			//	IVRDriverInputInternal_XXX
			//	IVRSystemLayerInternal_XXX
			//	IVRClientInternal_XXX
			//	LocalizationManager
			//	IVRCompositorSystemInternal_XXX
			//	IVRInput_010
			//	IVRInputInternal_002
			//	IVRChaperone_004
			//	IVRChaperoneSetup_006
			//	IVRChaperoneInternal_XXX
			//	IVRChaperoneServerDriver_XXX
			//	IVRApplications_007
			//	IVRSystem_023
			//	IVRMailbox_002
			//	IVRDebug_001
			//	IVRBlockQueue_005
			//	IVRControlPanel_006
			//	IVRApplicationsInternal_XXX
			//	IVRNotificationsInternal_001
			//	IVROverlay_028
			//	IVROverlayInternal_XXX
			//	IVRExtendedDisplay_001
			//	IVRTrackedCameraInternal_XXX
			//	IVRCameraPassthroughInternal_001
			//	IVRDriverDirectInternal_XXX
			//	IVRIOBuffer_002
			//	IVRServerDriverHost_005
			//	IVRSettings_002
			//	IVRDriverInput_003
			//	IVRDriverInput_004

			
			
			auto retval = getGenericInterfaceHook.origFunc(_this, pchInterfaceVersion, peError);
			if (_hookedInterfaces.find(pchInterfaceVersion) == _hookedInterfaces.end())
			{
				LOG(INFO) << "IVRDriverContextHooks::_getGenericInterface 中创建钩子对象: " << pchInterfaceVersion;



				auto hooks = InterfaceHooks::hookInterface(retval, pchInterfaceVersion);
				if (hooks != nullptr)
				{
					_hookedInterfaces.insert({ std::string(pchInterfaceVersion), hooks });
				}
			}
			LOG(TRACE) << "IVRDriverContextHooks::_getGenericInterface(" << _this << ", " << pchInterfaceVersion << ") = " << retval;
			return retval;
		}
	}
}