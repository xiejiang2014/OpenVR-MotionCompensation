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
		vr::HmdVector3d_t Rotation;		//平台旋转量 角度 3个double
		vr::HmdVector3d_t Translation;	//平台平移量 毫米 3个double
		vr::HmdVector3d_t EyePos;		//眼点相对于平台原点平移量 毫米 3个double

		//保留
		double		Reserved01;
		double		Reserved02;
		double		Reserved03;
		double		Reserved04;
		double		Reserved05;
		double		Reserved06;
		double		Reserved07;
		double		Reserved08;
		double		Reserved09;
		double		Reserved10;
		double		Reserved11;
		double		Reserved12;
		double		Reserved13;
		double		Reserved14;
		double		Reserved15;
		double		Reserved16;

		int			Enable;
		uint32_t	DataIndex;

		MMFstruct_H2VR()
		{
			Translation = { 0, 0, 0 };
			Rotation = { 0, 0, 0 };
			EyePos = { 0, 0, 0 };
		}
	};


	/// <summary>
	/// 从 vr 传到 host
	/// </summary>
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

		uint32_t DataIndex;
		uint32_t Reserved13;

		MMFstruct_VR2H()
		{
		
		}
	};

} // end namespace vrmotioncompensation