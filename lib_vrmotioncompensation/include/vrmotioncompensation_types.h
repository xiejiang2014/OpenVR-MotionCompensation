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
		vr::HmdVector3d_t Translation;	//3个double
		vr::HmdVector3d_t Rotation;		//3个double
		vr::HmdQuaternion_t QRotation;	//保留
		uint32_t Flags_1;
		uint32_t Flags_2;

		double HeaderQw;
		double HeaderQx;
		double HeaderQy;
		double HeaderQz;

		double Reserved_double[6];

		uint32_t dataIndex;
		int Reserved_int[9];

		MMFstruct_OVRMC_v1()
		{
			Translation = { 0, 0, 0 };
			Rotation = { 0, 0, 0 };
			QRotation = { 0, 0, 0, 0 };
			Flags_1 = 0;
			Flags_2 = 0;
		}
	};

} // end namespace vrmotioncompensation