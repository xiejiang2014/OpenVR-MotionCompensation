#include "IVRServerDriverHost006Hooks.h"

#include "../driver/ServerDriver.h"


namespace vrmotioncompensation
{
	namespace driver
	{
		/// <summary>
		/// 原始函数的地址
		/// 处理完自己的逻辑后，要通过这个指针调用 SteamVR 原来的函数，否则 SteamVR 就永远不知道有设备插入了。
		/// </summary>
		HookData<IVRServerDriverHost006Hooks::trackedDeviceAdded_t> IVRServerDriverHost006Hooks::trackedDeviceAddedHook; 

		HookData<IVRServerDriverHost006Hooks::trackedDevicePoseUpdated_t> IVRServerDriverHost006Hooks::trackedDevicePoseUpdatedHook;


		IVRServerDriverHost006Hooks::IVRServerDriverHost006Hooks(void* iptr)
		{
			if (!_isHooked)
			{
				CREATE_MH_HOOK(
					trackedDeviceAddedHook,  //原始函数
					_trackedDeviceAdded,	 //输入的函数
					"IVRServerDriverHost006::TrackedDeviceAdded", //调试用的名字字符串
					iptr,					 //目标对象（SteamVR 的接口指针）
					0						 //0 表示 IVRServerDriverHost 接口中的第 1 个虚函数。   查阅 OpenVR 头文件可知，第 1 个函数正是 TrackedDeviceAdded。
				);


				CREATE_MH_HOOK(
					trackedDevicePoseUpdatedHook, 
					_trackedDevicePoseUpdated, 
					"IVRServerDriverHost006::TrackedDevicePoseUpdated", 
					iptr, 
					1
				);

				_isHooked = true;
			}
		}

		IVRServerDriverHost006Hooks::~IVRServerDriverHost006Hooks()
		{
			if (_isHooked)
			{
				REMOVE_MH_HOOK(trackedDeviceAddedHook);
				REMOVE_MH_HOOK(trackedDevicePoseUpdatedHook);
				_isHooked = false;
			}
		}

		/// <summary>
		/// 工厂方法
		/// </summary>
		/// <param name="iptr"></param>
		/// <returns></returns>
		std::shared_ptr<InterfaceHooks> IVRServerDriverHost006Hooks::createHooks(void* iptr)
		{
			std::shared_ptr<InterfaceHooks> retval = std::shared_ptr<InterfaceHooks>(new IVRServerDriverHost006Hooks(iptr));
			return retval;
		}

		void IVRServerDriverHost006Hooks::trackedDevicePoseUpdatedOrig(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t unPoseStructSize)
		{
			trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		}

		bool IVRServerDriverHost006Hooks::_trackedDeviceAdded(void* _this, const char* pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, void* pDriver)
		{
			LOG(TRACE) << "IVRServerDriverHost006Hooks::_trackedDeviceAdded(" << _this << ", " << pchDeviceSerialNumber << ", " << eDeviceClass << ", " << pDriver << ")";
			

			serverDriver->hooksTrackedDeviceAdded(_this, 6, pchDeviceSerialNumber, eDeviceClass, pDriver);
			
			//（调用原函数）
			// 把消息传递给 SteamVR，让 SteamVR 完成正常的设备添加流程
			auto retval = trackedDeviceAddedHook.origFunc(_this, pchDeviceSerialNumber, eDeviceClass, pDriver);
			return retval;
		}

		void IVRServerDriverHost006Hooks::_trackedDevicePoseUpdated(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t unPoseStructSize)
		{
			// Call rates:
			//
			// Vive HMD: 1120 calls/s
			// Vive Controller: 369 calls/s each
			//
			// Time is key. If we assume 1 HMD and 13 controllers, we have a total of  ~6000 calls/s. That's about 166 microseconds per call at 100% load.
			auto poseCopy = newPose;


			// 篡改数据 (中间人攻击)
			// 调用 serverDriver 的钩子函数。
			// 这一步会进入 MotionCompensationManager::applyMotionCompensation
			// 也就是我们之前费劲修改的那个函数！
			// 此时，poseCopy 里的坐标会被加上运动补偿，速度会被减去平台速度。
			if (serverDriver->hooksTrackedDevicePoseUpdated(_this, 6, unWhichDevice, poseCopy, unPoseStructSize))
			{
				// 提交篡改后的数据
				// 注意：这里传给 origFunc 的是 poseCopy (改过的)，而不是 newPose (原始的)
				// SteamVR 以为这是硬件发来的真实数据，实际上是被我们修正过的。
				trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
			}
		}

	}
}