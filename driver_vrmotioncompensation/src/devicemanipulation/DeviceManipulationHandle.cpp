#include "DeviceManipulationHandle.h"

#include "../driver/ServerDriver.h"
#include "../hooks/IVRServerDriverHost004Hooks.h"
#include "../hooks/IVRServerDriverHost005Hooks.h"

#undef WIN32_LEAN_AND_MEAN
#undef NOSOUND
#include <Windows.h>
// According to windows documentation mmsystem.h should be automatically included with Windows.h when WIN32_LEAN_AND_MEAN and NOSOUND are not defined
// But it doesn't work so I have to include it manually
#include <mmsystem.h>


namespace vrmotioncompensation
{
	namespace driver
	{
		DeviceManipulationHandle::DeviceManipulationHandle(const char* serial, vr::ETrackedDeviceClass eDeviceClass)
			: m_isValid(true), m_parent(ServerDriver::getInstance()), m_motionCompensationManager(m_parent->motionCompensation()), m_eDeviceClass(eDeviceClass), m_serialNumber(serial)
		{
		}

		void DeviceManipulationHandle::setValid(bool isValid)
		{
			m_isValid = isValid;
		}

		bool DeviceManipulationHandle::handlePoseUpdate(uint32_t& unWhichDevice, vr::DriverPose_t& newPose, uint32_t unPoseStructSize)
		{
			//实测在用手柄进行补偿时, 以下两个分支的代码都会被调用, 大概是 ReferenceTracker 每调用一次, MotionCompensated会调用3-6次
			if (m_deviceMode == MotionCompensationDeviceMode::ReferenceTracker)
			{ 
				//return true;
				//LOG(INFO) << "handlePoseUpdate() m_deviceMode= ReferenceTracker";


				//Check if the pose is valid to prevent unwanted jitter and movement
				if (newPose.poseIsValid && newPose.result == vr::TrackingResult_Running_OK) //参考跟踪器
				{
					//Set the Zero-Point for the reference tracker if not done yet
					if (!m_motionCompensationManager.isZeroPoseValid())
					{						
						m_motionCompensationManager.setZeroPose(newPose);
					}
					else
					{
						//Update reference tracker position
						m_motionCompensationManager.updateRefPose(newPose);
					}
				}
			}
			else if (m_deviceMode == MotionCompensationDeviceMode::MotionCompensated) //运动补偿
			{

				//LOG(INFO) << "handlePoseUpdate() m_deviceMode= MotionCompensated";

				//Check if the pose is valid to prevent unwanted jitter and movement
				if (newPose.poseIsValid && newPose.result == vr::TrackingResult_Running_OK)
				{
					m_motionCompensationManager.applyMotionCompensation(newPose);
				}
			}

			return true;
		}

		//ui层发送 DeviceManipulation_MotionCompensationMode 消息后最终会到这里进行处理
		void DeviceManipulationHandle::setMotionCompensationDeviceMode(MotionCompensationDeviceMode DeviceMode)
		{
			m_deviceMode = DeviceMode;
		}
	} // end namespace driver
} // end namespace vrmotioncompensation