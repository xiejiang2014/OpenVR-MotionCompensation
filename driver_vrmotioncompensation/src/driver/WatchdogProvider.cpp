#include "WatchdogProvider.h"

#include "../logging.h"


// driver namespace
namespace vrmotioncompensation
{
	namespace driver
	{
		vr::EVRInitError WatchdogProvider::Init(vr::IVRDriverContext* pDriverContext)
		{
			LOG(INFO) << "WatchdogProvider::Init()";
			VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
			return vr::VRInitError_None;
		}

		void WatchdogProvider::Cleanup()
		{
			LOG(INFO) << "WatchdogProvider::Cleanup()";
			VR_CLEANUP_WATCHDOG_DRIVER_CONTEXT();
		}
	}
}