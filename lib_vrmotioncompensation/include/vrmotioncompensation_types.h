#pragma once

#include <stdint.h>


namespace vrmotioncompensation
{
	enum class MotionCompensationMode : uint32_t
	{
		Disabled = 0,
		ReferenceTracker = 1,
	};

	enum class MotionCompensationDeviceMode : uint32_t
	{
		Default = 0,
		ReferenceTracker = 1,
		MotionCompensated = 2,
	};

	struct DeviceInfo
	{
		uint32_t OpenVRId;
		vr::ETrackedDeviceClass deviceClass;
		MotionCompensationDeviceMode deviceMode;
	};

	struct MMFstruct_OVRMC_v1
	{
		uint32_t dataIndex;				//数据索引
		vr::HmdVector3d_t Translation;	//3个double 位置
		vr::HmdVector3d_t Rotation;		//3个double 旋转角度

		double Reserved_double1[4];		//保留

		//以下为回传上位机
		double HeaderQw;
		double HeaderQx;
		double HeaderQy;
		double HeaderQz;

		double Reserved_double2[6];
		int Reserved_int[10];

		MMFstruct_OVRMC_v1()
		{
			Translation = { 0, 0, 0 };
			Rotation = { 0, 0, 0 };
		}
	};

} // end namespace vrmotioncompensation