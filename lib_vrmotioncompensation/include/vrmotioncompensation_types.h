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

	struct MMFstruct_H2VR
	{
		vr::HmdVector3d_t Translation;	//3个double
		vr::HmdVector3d_t Rotation;		//3个double
		vr::HmdQuaternion_t QRotation;	//保留
		uint32_t Flags_1;
		uint32_t Flags_2;

		//头显实时姿态
		double HeaderQw;
		double HeaderQx;
		double HeaderQy;
		double HeaderQz;

		//参考起始姿态
		double ZeroRotW;
		double ZeroRotX;
		double ZeroRotY;
		double ZeroRotZ;

		//补偿姿态
		double RefRotQw;
		double RefRotQx;
		double RefRotQy;
		double RefRotQz;

		int Reserved_int0;
		int Reserved_int1;
		int Reserved_int2;
		int Reserved_int3;
		int Reserved_int4;

		uint32_t dataIndex;

		MMFstruct_H2VR()
		{
			Translation = { 0, 0, 0 };
			Rotation = { 0, 0, 0 };
			QRotation = { 0, 0, 0, 0 };
			Flags_1 = 0;
			Flags_2 = 0;
		}
	};



	struct MMFstruct_VR2H
	{
		//头显实时姿态
		double HeaderRotw;
		double HeaderRotx;
		double HeaderRoty;
		double HeaderRotz;


		double HeaderPosX;
		double HeaderPosY;
		double HeaderPosZ;

		//参考起始姿态
		double ZeroRotW;
		double ZeroRotX;
		double ZeroRotY;
		double ZeroRotZ;

		double ZeroPosX;
		double ZeroPosY;
		double ZeroPosZ;

		//追踪器实时姿态
		double TrackerRotW;
		double TrackerRotX;
		double TrackerRotY;
		double TrackerRotZ;

		double TrackerPosX;
		double TrackerPosY;
		double TrackerPosZ;

		//补偿姿态
		double RefRotQw;
		double RefRotQx;
		double RefRotQy;
		double RefRotQz;

		double RefPosX;
		double RefPosY;
		double RefPosZ;

		//未补偿的位移值
		double UncompensatedPosX;
		double UncompensatedPosY;
		double UncompensatedPosZ;
		
		//位移补偿
		double CompensatedPoseWorldPosX;
		double CompensatedPoseWorldPosY;
		double CompensatedPoseWorldPosZ;

		//补偿后的位移结果
		double CompensatedPosX;
		double CompensatedPosY;
		double CompensatedPosZ;


		double Reserved10;
		double Reserved11;
		double Reserved12;
		double Reserved13;

		MMFstruct_VR2H()
		{
		
		}
	};

} // end namespace vrmotioncompensation