#include "MotionCompensationManager.h"

#include "DeviceManipulationHandle.h"
#include "../driver/ServerDriver.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <chrono>

// driver namespace
namespace vrmotioncompensation
{
	namespace driver
	{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
		const float DegToRad = M_PI / 180.0f;
		const float RadToDeg = 180.0f / M_PI;


		MotionCompensationManager::MotionCompensationManager(ServerDriver* parent) : m_parent(parent)
		{
			LOG(INFO) << "MotionCompensationManager 构造    ";

			try
			{
				// create shared memory  创建内存映射
				_shdmemH2VR = { boost::interprocess::open_or_create, "OVRMC_Darren_H2VR", boost::interprocess::read_write, 4096 };
				//获取这个映射的元素内存
				_regionH2VR = { _shdmemH2VR, boost::interprocess::read_write };

				// get pointer address and fill it with data
				// 强行以 MMFstruct_H2VR 为类型来操作映射的内存,并将指针放到 _PH2VR 里面
				_PH2VR = static_cast<MMFstruct_H2VR*>(_regionH2VR.get_address());
				*_PH2VR = _H2VR;  //向指针填充数据
				LOG(INFO) << "Shared memory OVRMC_Darren_H2VR created";

				_msgH2VR = "Shared memory OVRMC_Darren_H2VR created";
			}
			catch (boost::interprocess::interprocess_exception& e)
			{
				LOG(INFO) << "Could not create or open shared memory OVRMC_Darren_H2VR. Error code " + e.get_error_code();
				_msgH2VR = "Could not create or open shared memory OVRMC_Darren_H2VR. Error code " + e.get_error_code();
			}


			try
			{
				// create shared memory  创建内存映射
				_shdmemVR2H = { boost::interprocess::open_or_create, "OVRMC_Darren_VR2H", boost::interprocess::read_write, 4096 };
				//获取这个映射的元素内存
				_regionVR2H = { _shdmemVR2H, boost::interprocess::read_write };

				// get pointer address and fill it with data
				// 强行以 MMFstruct_H2VR 为类型来操作映射的内存,并将指针放到 _PVR2H 里面
				_PVR2H = static_cast<MMFstruct_VR2H*>(_regionVR2H.get_address());
				*_PVR2H = _VR2H;  //向指针填充数据
				LOG(INFO) << "Shared memory OVRMC_Darren_VR2H created";
				_msgVR2H= "Shared memory OVRMC_Darren_VR2H created";
			}
			catch (boost::interprocess::interprocess_exception& e)
			{
				LOG(INFO) << "Could not create or open shared memory OVRMC_Darren_VR2H. Error code " + e.get_error_code();

				_msgVR2H = "Could not create or open shared memory OVRMC_Darren_VR2H. Error code " + e.get_error_code();
			}
		}

		bool MotionCompensationManager::setMotionCompensationMode(MotionCompensationMode Mode, int McDevice, int RtDevice)
		{
			if (Mode == MotionCompensationMode::ReferenceTracker)
			{
				_RefPoseValid = false;
				_RefPoseValidCounter = 0;
				_ZeroPoseValid = false;

				setAlpha(_Samples);
			}
			else
			{
				_zeroMotionRotValid = false;
			}

			_McDeviceID = McDevice;
			_RtDeviceID = RtDevice;
			_Mode = Mode;

			return true;
		}

		void MotionCompensationManager::setNewMotionCompensatedDevice(int MCdevice)
		{
			_McDeviceID = MCdevice;
		}

		void MotionCompensationManager::setNewReferenceTracker(int RTdevice)
		{
			_RtDeviceID = RTdevice;
			_RefPoseValid = false;
			_ZeroPoseValid = false;
		}

		void MotionCompensationManager::setAlpha(uint32_t samples)
		{
			_Samples = samples;
			_Alpha = 2.0 / (1.0 + (double)samples);
		}

		void MotionCompensationManager::setZeroMode(bool setZero)
		{
			_SetZeroMode = setZero;

			_zeroVec(_RefVel);
			_zeroVec(_RefRotVel);
			_zeroVec(_RefAcc);
			_zeroVec(_RefRotAcc);
		}

		void MotionCompensationManager::setOffsets(MMFstruct_H2VR offsets)
		{
			_H2VR.Translation = offsets.Translation;
			_H2VR.Rotation = offsets.Rotation;
			_H2VR = offsets;
			*_PH2VR = _H2VR;
		}

		bool MotionCompensationManager::isZeroPoseValid()
		{
			return _ZeroPoseValid;
		}

		/// <summary>
		/// 应用动作补偿,
		/// </summary>
		/// <param name="pose">应该是头盔的姿态,注意本函数会修改这个姿态达到补偿的目的</param>
		/// <returns></returns>
		bool MotionCompensationManager::applyMotionCompensation(vr::DriverPose_t& pose)
		{
			//没有启用则不进行补偿逻辑
			if (!_PH2VR || !_PH2VR->Enable)
			{
				return true;
			}

			//---------------------------------------------------提前计算好旋转的逆,方便后面将数据从世界坐标系转换回驱动坐标系
			// pose.qWorldFromDriverRotation 这个四元数意思是从驱动坐标到世界坐标的旋转量.
			// 通过 quaternionConjugate 函数 得到了这个旋转量的逆,即 tmpConj
			// 之后用 tmpConj * 任何世界坐标系下的旋转量,就可以把这个旋转量转换回驱动坐标系
			vr::HmdQuaternion_t tmpConj = vrmath::quaternionConjugate(pose.qWorldFromDriverRotation);


			//---------------------------------------------------计算出头显当前在世界坐标系下的位置和旋转

			//将头显在硬件坐标系下的原始位置转换到世界坐标系下 (Driver Space -> App Space)
			//分为两个部分
			//1 旋转向量(位置)
			//		想象你的定位基站是歪着挂在墙角的（向下倾斜 45 度）。
			//		头显在你前方 1 米处。但在基站看来，头显是在它“上方” 1 米处。
			//		这一步就是把这个“相对于基站的歪坐标”，旋转修正为“相对于地面的平坐标”。
			//		结果：此时得到的坐标，方向已经和世界坐标一致了（比如 Y 轴垂直向上），但原点还在基站上。
			//2 平移/位移
			//		加上这个偏移量，把原点从驱动坐标转到世界坐标。
			vr::HmdVector3d_t poseWorldPos = vrmath::quaternionRotateVector(
				pose.qWorldFromDriverRotation,	//旋转
				tmpConj,						//逆旋转
				pose.vecPosition,				//头显在硬件坐标系下的原始位置
				false) 
				+ pose.vecWorldFromDriverTranslation//(平移/位移)  
				;

			// 将头显旋转从驱动坐标系转换到世界坐标系 (Driver Space -> App Space):  水平时 pitch 和 roll 角度为0   正对b通道探头时 yaw 角度为0
			vr::HmdQuaternion_t poseWorldRot = pose.qWorldFromDriverRotation * pose.qRotation;


			//---------------------------------------------------记录初始旋转和位移
			vr::HmdVector3d_t headerQ =QuaternionToEulerOpenVR(poseWorldRot.w, poseWorldRot.x, poseWorldRot.y, poseWorldRot.z);
			if (!_ZeroPoseValid)
			{
				_ZeroRotYaw = headerQ.v[2];
				//重点: 这里假设开启补偿的时候头显的正前方表示运动平台的正前方.
				//在openvr中,b传感器所在的方向才是正前方,所以把头显此时的yaw记录下载,作为初始旋转角度.
				//平台的pitch和roll是绝对的, 只有yaw是相对的,所以ZeroRot仅记录yaw
				_ZeroRot = vrmath::quaternionFromYawPitchRoll(_ZeroRotYaw,0,0);
				_ZeroRotInv = vrmath::quaternionConjugate(_ZeroRot);


				_ZeroPos = vrmath::quaternionRotateVector(_ZeroRot, _ZeroRotInv,{0,0,0}, false) + pose.vecWorldFromDriverTranslation;

				_ZeroPoseValid = true;
			}


			try
			{
				updatePoseFromPlatform();
			}
			catch (std::exception& e)
			{
				LOG(ERROR) << "updatePoseFromPlatform error  " << e.what();
			}

			//---------------------------------------------------计算补偿
			_RefLock.lock();

			_ZeroLock.lock();
			// 计算位移补偿
			// 使用平台的位移数据时, _RefPos 本来就已经在世界中, 不需要再去反向旋转_RefRot  所以直接用减法计算即可
			vr::HmdVector3d_t compensatedPoseWorldPos = poseWorldPos - _RefPos;
			_ZeroLock.unlock();


			//计算旋转补偿 
			// 直接把座椅旋转的逆 (_RefRotInv) 乘到头显旋转上。这实现了“去耦合”。
			vr::HmdQuaternion_t compensatedPoseWorldRot = _RefRotInv *  poseWorldRot;

			_RefLock.unlock();

			////////////////// Translate the motion ref Velocity / Acceleration values into driver space and directly subtract them
			//////////////////处理速度和加速度:
			////////////////if (_SetZeroMode)
			////////////////{
			////////////////	_zeroVec(pose.vecVelocity);
			////////////////	_zeroVec(pose.vecAcceleration);
			////////////////	_zeroVec(pose.vecAngularVelocity);
			////////////////	_zeroVec(pose.vecAngularAcceleration);
			////////////////}
			////////////////else
			////////////////{
			////////////////	// Translate the motion ref Velocity / Acceleration values into driver space and directly subtract them
			////////////////	_RefVelLock.lock();
			////////////////	vr::HmdVector3d_t tmpPosVel = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefVel, true);
			////////////////	// 直接减去参考追踪器的速度
			////////////////	pose.vecVelocity[0] -= tmpPosVel.v[0];
			////////////////	pose.vecVelocity[1] -= tmpPosVel.v[1];
			////////////////	pose.vecVelocity[2] -= tmpPosVel.v[2];

			////////////////	vr::HmdVector3d_t tmpRotVel = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefRotVel, true);
			////////////////	pose.vecAngularVelocity[0] -= tmpRotVel.v[0];
			////////////////	pose.vecAngularVelocity[1] -= tmpRotVel.v[1];
			////////////////	pose.vecAngularVelocity[2] -= tmpRotVel.v[2];

			////////////////	vr::HmdVector3d_t tmpPosAcc = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefAcc, true);
			////////////////	pose.vecAcceleration[0] -= tmpPosAcc.v[0];
			////////////////	pose.vecAcceleration[1] -= tmpPosAcc.v[1];
			////////////////	pose.vecAcceleration[2] -= tmpPosAcc.v[2];

			////////////////	vr::HmdVector3d_t tmpRotAcc = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefRotAcc, true);
			////////////////	pose.vecAngularAcceleration[0] -= tmpRotAcc.v[0];
			////////////////	pose.vecAngularAcceleration[1] -= tmpRotAcc.v[1];
			////////////////	pose.vecAngularAcceleration[2] -= tmpRotAcc.v[2];
			////////////////	_RefVelLock.unlock();


			////////////////	//这部分非常重要！
			////////////////	//	如果只改位置不改速度，SteamVR 的预测算法会发疯，导致画面抖动。
			////////////////	//	代码计算了参考追踪器（座椅）的速度，并将其转换到 Driver Space，然后直接从头显的速度中减去。
			////////////////	//	结果 : 告诉 SteamVR “虽然我的传感器说我在动，但实际上我在虚拟世界里没动（或者动得没那么快）”。
			////////////////}

			//---------------------------------------------------应用补偿 : 这里直接修改了参数 pose 的成员变量。
			//将世界坐标系下的补偿量 compensatedPoseWorldRot 转回到了驱动坐标系,并应用到pose中使其生效
			pose.qRotation = tmpConj * compensatedPoseWorldRot;




			// convert back to driver space
			// 转换回驱动坐标系 (App Space -> Driver Space):
			// SteamVR 只要 Driver Space 的数据，所以算完还得转回去。
			vr::HmdVector3d_t adjPoseDriverPos = vrmath::quaternionRotateVector(
				pose.qWorldFromDriverRotation, 
				tmpConj, 
				compensatedPoseWorldPos - pose.vecWorldFromDriverTranslation, 
			true);
			_copyVec(pose.vecPosition, adjPoseDriverPos.v);


			//回传头显实时姿态
			if (_PVR2H)
			{
				_PVR2H->DataIndex++;
				//--------------
				_PVR2H->HeaderRotw = poseWorldRot.w;
				_PVR2H->HeaderRotx = poseWorldRot.x;
				_PVR2H->HeaderRoty = poseWorldRot.y;
				_PVR2H->HeaderRotz = poseWorldRot.z;

				_PVR2H->HeaderPosX = poseWorldPos.v[0];
				_PVR2H->HeaderPosY = poseWorldPos.v[1];
				_PVR2H->HeaderPosZ = poseWorldPos.v[2];

				//--------------

				_PVR2H->ZeroRotW = _ZeroRot.w;
				_PVR2H->ZeroRotX = _ZeroRot.x;
				_PVR2H->ZeroRotY = _ZeroRot.y;
				_PVR2H->ZeroRotZ = _ZeroRot.z;

				_PVR2H->ZeroPosX = _ZeroPos.v[0];
				_PVR2H->ZeroPosY = _ZeroPos.v[1];
				_PVR2H->ZeroPosZ = _ZeroPos.v[2];

				//--------------

				_PVR2H->TrackerRotW = _trackWorldRot.w;
				_PVR2H->TrackerRotX = _trackWorldRot.x;
				_PVR2H->TrackerRotY = _trackWorldRot.y;
				_PVR2H->TrackerRotZ = _trackWorldRot.z;


				//--------------

				_PVR2H->RefRotQw = _RefRot.w;
				_PVR2H->RefRotQx = _RefRot.x;
				_PVR2H->RefRotQy = _RefRot.y;
				_PVR2H->RefRotQz = _RefRot.z;

				_PVR2H->RefPosX = _RefPos.v[0];
				_PVR2H->RefPosY = _RefPos.v[1];
				_PVR2H->RefPosZ = _RefPos.v[2];

				//--------------
				_PVR2H->UncompensatedPosX = pose.vecPosition[0];
				_PVR2H->UncompensatedPosY = pose.vecPosition[1];
				_PVR2H->UncompensatedPosZ = pose.vecPosition[2];

				_PVR2H->CompensatedPoseWorldPosX = compensatedPoseWorldPos.v[0];
				_PVR2H->CompensatedPoseWorldPosY = compensatedPoseWorldPos.v[1];
				_PVR2H->CompensatedPoseWorldPosZ = compensatedPoseWorldPos.v[2];

				_PVR2H->CompensatedPosX = adjPoseDriverPos.v[0];
				_PVR2H->CompensatedPosY = adjPoseDriverPos.v[1];
				_PVR2H->CompensatedPosZ = adjPoseDriverPos.v[2];

			}

			return true;
		}


		void MotionCompensationManager::updatePoseFromPlatform()
		{
			//return;

			// 没有开启内存共享通道则不反应
			if (!_PH2VR) return;

			// [快照] 读取共享内存
			MMFstruct_H2VR localData = *_PH2VR;

			// 数据无效
			if (localData.DataIndex <= 0) return;

			// 计算 Index 差值
			long long indexDiff = (long long)localData.DataIndex - _lastDataIndex;

			// 1. 如果 indexDiff == 0: 数据没变，维持上一帧状态，直接返回
			if (indexDiff == 0 && _lastDataIndex != 0)
			{
				_RefPoseValid = true;
				return;
			}

			//传过来的原始数据调整方向
			double roll  = -localData.Rotation.v[0];			//角度
			double pitch = -localData.Rotation.v[1];			//角度
			double yaw   =  localData.Rotation.v[2];			//角度
			double surge =  localData.Translation.v[0]	/ 1000;	//毫米->米
			double sway  =  localData.Translation.v[1]	/ 1000;	//毫米->米
			double heave =  localData.Translation.v[2]	/ 1000;	//毫米->米

			double eyeTx =  localData.EyePos.v[1] / 1000;		//毫米->米 vr中 X轴指向右方
			double eyeTy =  localData.EyePos.v[2] / 1000;		//毫米->米 vr中 Y轴指向上方
			double eyeTz =  localData.EyePos.v[0] / 1000;		//毫米->米 vr中 Z轴指向前方



			//-------------------------------------------------------计算相对旋转
			//将旋转值转为4元数形式
			vr::HmdQuaternion_t qRotation = vrmath::quaternionFromYawPitchRoll(
				yaw * DegToRad+ _ZeroRotYaw,  //以初始yaw方向为yaw的0点
				pitch * DegToRad, 
				roll * DegToRad
			);

			_RefLock.lock();
			_ZeroLock.lock();

			//计算“相对偏移”
			_RefRot = qRotation * _ZeroRotInv;
			_RefRotInv = vrmath::quaternionConjugate(_RefRot);

			_ZeroLock.unlock();
			_RefLock.unlock();

			//-------------------------------------------------------计算相对位移
			//平台自身的平移量
			vr::HmdVector3d_t motionPos = { sway ,heave ,surge };

			//因旋转造成的眼点平移
			vr::HmdVector3d_t eyePos = { eyeTx ,eyeTy ,eyeTz };
			vr::HmdVector3d_t eyeTranslationByRotation = GetEyeWorldPosition(roll, pitch, yaw, eyePos);

			//总偏移量
			vr::HmdVector3d_t totalTranslation = motionPos + eyeTranslationByRotation;

			_RefLock.lock();
			_RefPos = vrmath::quaternionRotateVector(_ZeroRot, _ZeroRotInv, totalTranslation, false);
			_RefLock.unlock();


			//////////////////----------------------------------------旋转量

			//////////////////得到旋转后的旋转四元数
			//////////////////vr::HmdQuaternion_t  qRotation = vrmath::quaternionFromYawPitchRoll(yawInRRP, pitchInRRP, rollInRRP);

			//////////////////if (_PlatformPoseIndex % 10 == 0)
			//////////////////{
			//////////////////	vr::HmdVector3d_t eulerAngles= QuaternionToEulerOpenVR(qRotation.w, qRotation.x, qRotation.y, qRotation.z);
			//////////////////	LOG(INFO) << "四元数转换测试	|" << yawInRRP << "|" << pitchInRRP << "|" << rollInRRP << "|"  << eulerAngles.v[2] << "|" << eulerAngles.v[1] << "|" << eulerAngles.v[1];
			//////////////////}
			//////////////////_PlatformPoseIndex++;
			////////////////

			////////////////// -----------2提取并清洗数据 (无论是否复位，都需要提取位置和旋转)

			////////////////vr::HmdQuaternion_t rawRot;

			////////////////if (qRotation.w == 0 && qRotation.x == 0 && qRotation.y == 0 && qRotation.z == 0)
			////////////////{
			////////////////	rawRot = { 1.0, 0.0, 0.0, 0.0 };
			////////////////}
			////////////////else
			////////////////{
			////////////////	rawRot.w = qRotation.w;
			////////////////	rawRot.x = qRotation.x;
			////////////////	rawRot.y = qRotation.y;
			////////////////	rawRot.z = qRotation.z;

			////////////////	// [保持] 归一化四元数
			////////////////	double mag = sqrt(rawRot.w * rawRot.w + rawRot.x * rawRot.x + rawRot.y * rawRot.y + rawRot.z * rawRot.z);
			////////////////	if (mag > 0.00001) {
			////////////////		rawRot.w /= mag; rawRot.x /= mag; rawRot.y /= mag; rawRot.z /= mag;
			////////////////	}
			////////////////}

			////////////////// [保持] 四元数连续性检查
			////////////////// 如果是复位情况(indexDiff < 0)，其实不需要做连续性检查，
			////////////////// 但为了代码简洁，且 rawRot 翻转不影响单帧姿态，保留此处也无妨。
			////////////////if (_lastDataIndex != 0)
			////////////////{
			////////////////	double dot = rawRot.w * _lastPlatformRot.w + rawRot.x * _lastPlatformRot.x +
			////////////////		rawRot.y * _lastPlatformRot.y + rawRot.z * _lastPlatformRot.z;

			////////////////	if (dot < 0)
			////////////////	{
			////////////////		rawRot.w = -rawRot.w;
			////////////////		rawRot.x = -rawRot.x;
			////////////////		rawRot.y = -rawRot.y;
			////////////////		rawRot.z = -rawRot.z;
			////////////////	}
			////////////////}

			////////////////// -----------3计算物理属性 (速度 & 加速度)	初始化为 0。如果发生复位(indexDiff < 0)，则不会进入下面的计算块，保持为 0。
			////////////////vr::HmdVector3d_t currVel = { 0, 0, 0 };
			////////////////vr::HmdVector3d_t currAcc = { 0, 0, 0 };
			////////////////vr::HmdVector3d_t currAngVel = { 0, 0, 0 };
			////////////////vr::HmdVector3d_t currAngAcc = { 0, 0, 0 };

			////////////////// 仅当 数据是递增的 (indexDiff > 0) 且 不是第一帧 (_lastDataIndex != 0) 时计算物理属性
			////////////////if (indexDiff > 0 && _lastDataIndex != 0)
			////////////////{
			////////////////	// 使用 DataIndex 计算精确时间差 (100Hz = 0.01秒)
			////////////////	double tdiff = (double)indexDiff * 0.01;
			////////////////	// 防御性检查
			////////////////	if (tdiff < 0.00001) tdiff = 0.01;

			////////////////	// [保持] 位置死区 (消除静止抖动)
			////////////////	double distSq =
			////////////////		pow(rawPos.v[0] - _lastPlatformPos.v[0], 2) +
			////////////////		pow(rawPos.v[1] - _lastPlatformPos.v[1], 2) +
			////////////////		pow(rawPos.v[2] - _lastPlatformPos.v[2], 2);

			////////////////	// 死区阈值：0.0001米 (0.1mm)
			////////////////	if (distSq > 1.0e-8)
			////////////////	{
			////////////////		currVel = { 0, 0, 0 }; // <--- 强制为 0
			////////////////		currAcc = { 0, 0, 0 };

			////////////////		//currVel.v[0] = (rawPos.v[0] - _lastPlatformPos.v[0]) / tdiff;
			////////////////		//currVel.v[1] = (rawPos.v[1] - _lastPlatformPos.v[1]) / tdiff;
			////////////////		//currVel.v[2] = (rawPos.v[2] - _lastPlatformPos.v[2]) / tdiff;

			////////////////		//currAcc.v[0] = (currVel.v[0] - _lastPlatformVel.v[0]) / tdiff;
			////////////////		//currAcc.v[1] = (currVel.v[1] - _lastPlatformVel.v[1]) / tdiff;
			////////////////		//currAcc.v[2] = (currVel.v[2] - _lastPlatformVel.v[2]) / tdiff;
			////////////////	}

			////////////////	// [保持] 旋转死区
			////////////////	double rotDiffSq =
			////////////////		pow(rawRot.w - _lastPlatformRot.w, 2) +
			////////////////		pow(rawRot.x - _lastPlatformRot.x, 2) +
			////////////////		pow(rawRot.y - _lastPlatformRot.y, 2) +
			////////////////		pow(rawRot.z - _lastPlatformRot.z, 2);

			////////////////	if (rotDiffSq > 1.0e-8)
			////////////////	{
			////////////////		vr::HmdVector3d_t eulerNow = toEulerAngles(rawRot);
			////////////////		vr::HmdVector3d_t eulerOld = toEulerAngles(_lastPlatformRot);

			////////////////		currAngVel = { 0, 0, 0 }; // <--- 强制为 0
			////////////////		currAngAcc = { 0, 0, 0 };

			////////////////		//currAngVel.v[0] = rotVelocity(tdiff, eulerNow.v[0], eulerOld.v[0]);
			////////////////		//currAngVel.v[1] = rotVelocity(tdiff, eulerNow.v[1], eulerOld.v[1]);
			////////////////		//currAngVel.v[2] = rotVelocity(tdiff, eulerNow.v[2], eulerOld.v[2]);

			////////////////		//currAngAcc.v[0] = (currAngVel.v[0] - _lastPlatformAngVel.v[0]) / tdiff;
			////////////////		//currAngAcc.v[1] = (currAngVel.v[1] - _lastPlatformAngVel.v[1]) / tdiff;
			////////////////		//currAngAcc.v[2] = (currAngVel.v[2] - _lastPlatformAngVel.v[2]) / tdiff;
			////////////////	}
			////////////////}
			////////////////// else { // 这里隐含处理了 indexDiff < 0 的情况：
			////////////////	 // 变量 currVel 等保持初始化时的 {0,0,0}，
			////////////////	 // 实现了“复位时不计算疯狂的速度”这一目标。
			////////////////// }

			////////////////// -----------4. 更新 _Ref 变量 (线程安全)
			////////////////_RefLock.lock();
			////////////////_ZeroLock.lock();
			////////////////_RefVelLock.lock();

			////////////////// 设置参考数据 (无论是否复位，当前位置都是准确的)
			////////////////_RefPos = rawPos;
			//////////////////_RefRot = rawRot * vrmath::quaternionConjugate(_ZeroRot);
			//////////////////_RefRotInv = vrmath::quaternionConjugate(_RefRot);

			////////////////// 设置物理数据 (如果是复位，这里全是 0，符合预期)
			////////////////_RefVel = currVel;
			////////////////_RefAcc = currAcc;
			////////////////_RefRotVel = currAngVel;
			////////////////_RefRotAcc = currAngAcc;

			////////////////_RefVelLock.unlock();
			////////////////_ZeroLock.unlock();
			////////////////_RefLock.unlock();

			////////////////// -----------5. 更新缓存
			////////////////_lastPlatformPos = rawPos;
			////////////////_lastPlatformVel = currVel;
			////////////////_lastPlatformRot = rawRot;
			////////////////_lastPlatformAngVel = currAngVel;

			_RefPoseValid = true;

			////////////////if (_PlatformPoseIndex % 10 == 0)
			////////////////{
			////////////////	vr::HmdVector3d_t q = QuaternionToEulerOpenVR(_RefRot.w, _RefRot.x, _RefRot.y, _RefRot.z);

			////////////////	//LOG(INFO) << "平台补偿结果	|" << q.v[0] * RadToDeg << "|" << q.v[1] * RadToDeg << "|" << q.v[2] * RadToDeg << "|" << pitch << "|" << roll << "|" << yaw << "|" << _rrp[2] << "|" << POSInRRP[0] << "|" << POSInRRP[1] << "|" << POSInRRP[2];
			////////////////	//LOG(INFO) << "PlatformPose _RefRot	|" << pitch << "|" << roll << "|" << yaw << "|" << 0-_zeroMotionYaw * RadToDeg << "|" << poseInRRP[0] << "|" << poseInRRP[1] << "|" << poseInRRP[2];
			////////////////
			////////////////


			////////////////	//LOG(INFO) << "正四元|" << _RefRot.w << "|" << _RefRot.x << "|" << _RefRot.y << "|" << _RefRot.z ;
			////////////////	//LOG(INFO) << "逆四元|" << _RefRotInv.w << "|" << _RefRotInv.x << "|" << _RefRotInv.y << "|" << _RefRotInv.z;
			////////////////	//vr::HmdVector3d_t q2 = QuaternionToEulerOpenVR(_RefRotInv.w, _RefRotInv.x, _RefRotInv.y, _RefRotInv.z);
			////////////////	//LOG(INFO) << "正逆检查(弧度)|" << q.v[0]  << "|" << q.v[1]  << "|" << q.v[2]  << "|" << q2.v[0]  << "|" << q2.v[1]  << "|" << q2.v[2] ;
			////////////////	//LOG(INFO) << "正逆检查(角度)|" << q.v[0] * RadToDeg << "|" << q.v[1] * RadToDeg << "|" << q.v[2] * RadToDeg << "|" << q2.v[0] * RadToDeg << "|" << q2.v[1] * RadToDeg << "|" << q2.v[2] * RadToDeg;

			////////////////}
			////////////////_PlatformPoseIndex++;
			_lastDataIndex = localData.DataIndex;
		}

		void MotionCompensationManager::runFrame()
		{
			/*if (_H2VR.Flags_1 & (1 << FLAG_ENABLE_MC) && _Mode == MotionCompensationMode::Disabled)
			{

			}
			else if (!(_H2VR.Flags_1 & (1 << FLAG_ENABLE_MC)) && _Mode == MotionCompensationMode::ReferenceTracker)
			{
				setMotionCompensationMode(MotionCompensationMode::ReferenceTracker, -1, -1);
			}*/
		}

		double MotionCompensationManager::vecVelocity(double time, const double vecPosition, const double Old_vecPosition)
		{
			double NewVelocity = 0.0;

			if (time != (double)0.0)
			{
				NewVelocity = (vecPosition - Old_vecPosition) / time;
			}

			return NewVelocity;
		}

		double MotionCompensationManager::vecAcceleration(double time, const double vecVelocity, const double Old_vecVelocity)
		{
			double NewAcceleration = 0.0;

			if (time != (double)0.0)
			{
				NewAcceleration = (vecVelocity - Old_vecVelocity) / time;
			}

			return NewAcceleration;
		}

		double MotionCompensationManager::rotVelocity(double time, const double vecAngle, const double Old_vecAngle)
		{
			double NewVelocity = 0.0;

			if (time != (double)0.0)
			{
				NewVelocity = (1 - angleDifference(vecAngle, Old_vecAngle)) / time;
			}

			return NewVelocity;
		}

		// Low Pass Filter for 3d Vectors
		double MotionCompensationManager::DEMA(const double RawData, int Axis)
		{
			_Filter_vecPosition[0].v[Axis] += _Alpha * (RawData - _Filter_vecPosition[1].v[Axis]);
			_Filter_vecPosition[1].v[Axis] += _Alpha * (_Filter_vecPosition[0].v[Axis] - _Filter_vecPosition[1].v[Axis]);
			return 2 * _Filter_vecPosition[0].v[Axis] - _Filter_vecPosition[1].v[Axis];
		}

		// Low Pass Filter for 3d Vectors
		vr::HmdVector3d_t MotionCompensationManager::LPF(const double RawData[3], vr::HmdVector3d_t SmoothData)
		{
			vr::HmdVector3d_t RetVal;

			RetVal.v[0] = SmoothData.v[0] - (_LpfBeta * (SmoothData.v[0] - RawData[0]));
			RetVal.v[1] = SmoothData.v[1] - (_LpfBeta * (SmoothData.v[1] - RawData[1]));
			RetVal.v[2] = SmoothData.v[2] - (_LpfBeta * (SmoothData.v[2] - RawData[2]));

			return RetVal;
		}

		// Low Pass Filter for 3d Vectors
		vr::HmdVector3d_t MotionCompensationManager::LPF(vr::HmdVector3d_t RawData, vr::HmdVector3d_t SmoothData)
		{
			vr::HmdVector3d_t RetVal;

			RetVal.v[0] = SmoothData.v[0] - (_LpfBeta * (SmoothData.v[0] - RawData.v[0]));
			RetVal.v[1] = SmoothData.v[1] - (_LpfBeta * (SmoothData.v[1] - RawData.v[1]));
			RetVal.v[2] = SmoothData.v[2] - (_LpfBeta * (SmoothData.v[2] - RawData.v[2]));

			return RetVal;
		}

		// Low Pass Filter for quaternion
		vr::HmdQuaternion_t MotionCompensationManager::lowPassFilterQuaternion(vr::HmdQuaternion_t RawData, vr::HmdQuaternion_t SmoothData)
		{
			return slerp(SmoothData, RawData, _LpfBeta);
		}

		// Spherical Linear Interpolation for Quaternions
		// 四元数的球面线性插值
		vr::HmdQuaternion_t MotionCompensationManager::slerp(vr::HmdQuaternion_t q1, vr::HmdQuaternion_t q2, double lambda)
		{
			vr::HmdQuaternion_t qr;

			double dotproduct = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

			// if q1 and q2 are the same, we can return either of the values
			if (dotproduct >= 1.0 || dotproduct <= -1.0)
			{
				return q1;
			}

			double theta, st, sut, sout, coeff1, coeff2;

			// algorithm adapted from Shoemake's paper
			lambda = lambda / 2.0;

			theta = (double)acos(dotproduct);
			if (theta < 0.0) theta = -theta;

			st = (double)sin(theta);
			sut = (double)sin(lambda * theta);
			sout = (double)sin((1 - lambda) * theta);
			coeff1 = sout / st;
			coeff2 = sut / st;

			qr.x = coeff1 * q1.x + coeff2 * q2.x;
			qr.y = coeff1 * q1.y + coeff2 * q2.y;
			qr.z = coeff1 * q1.z + coeff2 * q2.z;
			qr.w = coeff1 * q1.w + coeff2 * q2.w;


			//Normalize
			double norm = sqrt(qr.x * qr.x + qr.y * qr.y + qr.z * qr.z + qr.w * qr.w);
			qr.x /= norm;
			qr.y /= norm;
			qr.z /= norm;

			return qr;
		}

		// Convert Quaternion to Euler Angles in Radians
		// 将四元数转换为以弧度表示的欧拉角
		vr::HmdVector3d_t MotionCompensationManager::toEulerAngles(vr::HmdQuaternion_t q)
		{
			vr::HmdVector3d_t angles;

			// roll (x-axis rotation)
			double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
			double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
			angles.v[0] = std::atan2(sinr_cosp, cosr_cosp);

			// pitch (y-axis rotation)
			double sinp = 2 * (q.w * q.y - q.z * q.x);

			if (std::abs(sinp) >= 1)
			{
				angles.v[1] = std::copysign(boost::math::constants::pi<double>() / 2, sinp); // use 90 degrees if out of range
			}
			else
			{
				angles.v[1] = std::asin(sinp);
			}

			// yaw (z-axis rotation)
			double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
			double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
			angles.v[2] = std::atan2(siny_cosp, cosy_cosp);

			return angles;
		}


		// 将四元数传唤为欧拉角(弧度),返回数据顺序为  pitch roll yaw
		// 专门针对 OpenVR/SteamVR 的 (Qy * Qx * Qz) 顺序进行逆运算
		// 实测有效  
		vr::HmdVector3d_t MotionCompensationManager::QuaternionToEulerOpenVR(double w, double x, double y, double z) {
			vr::HmdVector3d_t angles;

			// 1. 计算 Pitch (X-axis) - 这是 Y-X-Z 顺序下的中间轴
			// 公式核心: 2(w*x - y*z)
			double sinp = 2.0 * (w * x - y * z);

			// 防止数值误差导致 asin 越界 (NaN)
			if (std::abs(sinp) >= 1.0)
				angles.v[0] = std::copysign(M_PI / 2.0, sinp); // 90度锁定
			else
				angles.v[0] = std::asin(sinp);

			// 2. 计算 Yaw (Y-axis)
			// 公式核心: atan2(2(w*y + x*z), 1 - 2(x^2 + y^2))
			double siny_cosp = 2.0 * (w * y + x * z);
			double cosy_cosp = 1.0 - 2.0 * (x * x + y * y);
			angles.v[2] = std::atan2(siny_cosp, cosy_cosp);

			// 3. 计算 Roll (Z-axis)
			// 公式核心: atan2(2(w*z + x*y), 1 - 2(x^2 + z^2))
			double sinr_cosp = 2.0 * (w * z + x * y);
			double cosr_cosp = 1.0 - 2.0 * (x * x + z * z);
			angles.v[1] = std::atan2(sinr_cosp, cosr_cosp);

			return angles;
		}

		// Returns the shortest difference between to angles
		// 返回两个角度之间的最短差值。
		const double MotionCompensationManager::angleDifference(double Raw, double New)
		{
			double diff = fmod((New - Raw + (double)180), (double)360) - (double)180;
			return diff < -(double)180 ? diff + (double)360 : diff;
		}

		vr::HmdVector3d_t MotionCompensationManager::transform(vr::HmdVector3d_t VecRotation, vr::HmdVector3d_t VecPosition, vr::HmdVector3d_t point)
		{
			// point is the user-input offset to the controller
			// VecRotation and VecPosition is the current reference-pose (Controller or input from Mover)
			vr::HmdQuaternion_t quat = vrmath::quaternionFromYawPitchRoll(VecRotation.v[0], VecRotation.v[1], VecRotation.v[2]);
			return transform(quat, VecPosition, point);
		}

		// Calculates the new coordinates of 'point', moved and rotated by VecRotation and VecPosition
		// 计算点“point”经过向量旋转和向量平移后的新坐标。
		vr::HmdVector3d_t MotionCompensationManager::transform(vr::HmdQuaternion_t quat, vr::HmdVector3d_t VecPosition, vr::HmdVector3d_t point)
		{
			vr::HmdVector3d_t translation = vrmath::quaternionRotateVector(quat, VecPosition);

			return vrmath::quaternionRotateVector(quat, point) + translation;
		}

		// 
		vr::HmdVector3d_t MotionCompensationManager::transform(vr::HmdVector3d_t VecRotation, vr::HmdVector3d_t VecPosition, vr::HmdVector3d_t centerOfRotation, vr::HmdVector3d_t point)
		{
			// point is the user-input offset to the controller
			// VecRotation and VecPosition is the current rig-pose

			vr::HmdQuaternion_t quat = vrmath::quaternionFromYawPitchRoll(VecRotation.v[0], VecRotation.v[1], VecRotation.v[2]);

			double n1 = quat.x * 2.f;
			double n2 = quat.y * 2.f;
			double n3 = quat.z * 2.f;

			double _n4 = quat.x * n1;
			double _n5 = quat.y * n2;
			double _n6 = quat.z * n3;
			double _n7 = quat.x * n2;
			double _n8 = quat.x * n3;
			double _n9 = quat.y * n3;
			double _n10 = quat.w * n1;
			double _n11 = quat.w * n2;
			double _n12 = quat.w * n3;

			vr::HmdVector3d_t translation = {
				(1 - (_n5 + _n6)) * (VecPosition.v[0]) + (_n7 - _n12) * (VecPosition.v[1]) + (_n8 + _n11) * (VecPosition.v[2]),
				(_n7 + _n12) * (VecPosition.v[0]) + (1 - (_n4 + _n6)) * (VecPosition.v[1]) + (_n9 - _n10) * (VecPosition.v[2]),
				(_n8 - _n11) * (VecPosition.v[0]) + (_n9 + _n10) * (VecPosition.v[1]) + (1 - (_n4 + _n5)) * (VecPosition.v[2])
			};

			return {
				(1.0 - (_n5 + _n6)) * (point.v[0] - centerOfRotation.v[0]) + (_n7 - _n12) * (point.v[1] - centerOfRotation.v[1]) + (_n8 + _n11) * (point.v[2] - centerOfRotation.v[2]) + centerOfRotation.v[0] + translation.v[0],
				(_n7 + _n12) * (point.v[0] - centerOfRotation.v[0]) + (1.0 - (_n4 + _n6)) * (point.v[1] - centerOfRotation.v[1]) + (_n9 - _n10) * (point.v[2] - centerOfRotation.v[2]) + centerOfRotation.v[1] + translation.v[1],
				(_n8 - _n11) * (point.v[0] - centerOfRotation.v[0]) + (_n9 + _n10) * (point.v[1] - centerOfRotation.v[1]) + (1.0 - (_n4 + _n5)) * (point.v[2] - centerOfRotation.v[2]) + centerOfRotation.v[2] + translation.v[2]
			};
		}


		//-------------------复刻c#中的矩阵旋转
		// 矩阵乘法 (A * B)
		vr::HmdMatrix44_t MatrixMultiply(const vr::HmdMatrix44_t& A, const vr::HmdMatrix44_t& B)
		{
			vr::HmdMatrix44_t R;
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					R.m[i][j] = 
						A.m[i][0] * B.m[0][j] +
						A.m[i][1] * B.m[1][j] +
						A.m[i][2] * B.m[2][j] +
						A.m[i][3] * B.m[3][j];
				}
			}
			return R;
		}


		/// <summary>
		/// 刚体变换矩阵求逆 (优化算法)
		/// </summary>
		/// <param name="In"></param>
		/// <returns></returns>
		vr::HmdMatrix44_t MatrixInvertRigidBody(const vr::HmdMatrix44_t& In)
		{
			vr::HmdMatrix44_t Out;

			// 1. 转置旋转部分 (R^T)
			// Out_ij = In_ji
			Out.m[0][0] = In.m[0][0]; Out.m[0][1] = In.m[1][0]; Out.m[0][2] = In.m[2][0];
			Out.m[1][0] = In.m[0][1]; Out.m[1][1] = In.m[1][1]; Out.m[1][2] = In.m[2][1];
			Out.m[2][0] = In.m[0][2]; Out.m[2][1] = In.m[1][2]; Out.m[2][2] = In.m[2][2];

			// 2. 计算新的平移部分 (-R^T * t)
			// 公式: T_new = - (Transpose(R) * T_old)
			// 这相当于点积：T_new.x = -(R_Col0 . T_old)

			float tx = In.m[0][3];
			float ty = In.m[1][3];
			float tz = In.m[2][3];

			// 修正后的计算逻辑：
			// X分量 = -( In.m[0][0]*tx + In.m[1][0]*ty + In.m[2][0]*tz )
			Out.m[0][3] = -(In.m[0][0] * tx + In.m[1][0] * ty + In.m[2][0] * tz);

			// Y分量 = -( In.m[0][1]*tx + In.m[1][1]*ty + In.m[2][1]*tz )
			Out.m[1][3] = -(In.m[0][1] * tx + In.m[1][1] * ty + In.m[2][1] * tz);

			// Z分量 = -( In.m[0][2]*tx + In.m[1][2]*ty + In.m[2][2]*tz )
			Out.m[2][3] = -(In.m[0][2] * tx + In.m[1][2] * ty + In.m[2][2] * tz);

			// 3. 填充最后一行
			Out.m[3][0] = 0.0f;
			Out.m[3][1] = 0.0f;
			Out.m[3][2] = 0.0f;
			Out.m[3][3] = 1.0f;

			return Out;
		}


		// 根据欧拉角(弧度)和平移创建变换矩阵
		// 对应 C# 中的 CreateTransformMatrix
		vr::HmdMatrix44_t CreateTransformMatrix(float txRad, float tyRad, float tzRad, float px, float py, float pz)
		{
			vr::HmdMatrix44_t mat;

			float cosTZ = std::cos(tzRad), sinTZ = std::sin(tzRad);
			float cosTY = std::cos(tyRad), sinTY = std::sin(tyRad);
			float cosTX = std::cos(txRad), sinTX = std::sin(txRad);

			// Row 0
			mat.m[0][0] = cosTZ * cosTY;
			mat.m[0][1] = cosTZ * sinTY * sinTX - sinTZ * cosTX;
			mat.m[0][2] = cosTZ * sinTY * cosTX + sinTZ * sinTX;
			mat.m[0][3] = px; // Translation X

			// Row 1
			mat.m[1][0] = sinTZ * cosTY;
			mat.m[1][1] = sinTZ * sinTY * sinTX + cosTZ * cosTX;
			mat.m[1][2] = sinTZ * sinTY * cosTX - cosTZ * sinTX;
			mat.m[1][3] = py; // Translation Y

			// Row 2
			mat.m[2][0] = -sinTY;
			mat.m[2][1] = cosTY * sinTX;
			mat.m[2][2] = cosTY * cosTX;
			mat.m[2][3] = pz; // Translation Z

			// Row 3
			mat.m[3][0] = 0.0f;
			mat.m[3][1] = 0.0f;
			mat.m[3][2] = 0.0f;
			mat.m[3][3] = 1.0f;

			return mat;
		}


		/**
		 * 计算 RRP 参考坐标系下姿态值
		 * @param pos_original 原始位姿数组 [Rx, Ry, Rz, Px, Py, Pz]
		 * @param RRP 参考坐标系数组 [Rx, Ry, Rz, Px, Py, Pz]
		 * @return 变换后的位姿数组
		 */
		std::vector<float> MotionCompensationManager:: CoordinateTransform(const std::vector<float>& pos_original, const std::vector<float>& RRP)
		{
			// 1. 构建原始位姿矩阵
			vr::HmdMatrix44_t T_original = CreateTransformMatrix(
				pos_original[0] * DegToRad,
				pos_original[1] * DegToRad,
				pos_original[2] * DegToRad,
				pos_original[3],
				pos_original[4],
				pos_original[5]
			);

			// 2. 构建参考坐标系矩阵
			vr::HmdMatrix44_t T_rrp = CreateTransformMatrix(
				RRP[0] * DegToRad,
				RRP[1] * DegToRad,
				RRP[2] * DegToRad,
				RRP[3],
				RRP[4],
				RRP[5]
			);

			// 3. 计算 T_rrp 的逆矩阵
			vr::HmdMatrix44_t T_rrp_inv = MatrixInvertRigidBody(T_rrp);

			// 4. 计算相对变换: T = T_rrp_inv * T_original * T_rrp
			// 注意乘法顺序 (A*B)*C
			vr::HmdMatrix44_t Temp = MatrixMultiply(T_rrp_inv, T_original);
			vr::HmdMatrix44_t T = MatrixMultiply(Temp, T_rrp);

			// 5. 提取新欧拉角 (ZYX 顺序)
			// C# T.M32 -> C++ m[2][1]
			// C# T.M33 -> C++ m[2][2]
			float rx = std::atan2(T.m[2][1], T.m[2][2]); // Pitch

			// Y旋转 (Roll): atan2(-M31, sqrt(M32^2 + M33^2))
			float ry = std::atan2(-T.m[2][0], std::sqrt(T.m[2][1] * T.m[2][1] + T.m[2][2] * T.m[2][2]));

			// Z旋转 (Yaw): atan2(M21, M11)
			float rz = std::atan2(T.m[1][0], T.m[0][0]);

			// 6. 返回结果
			return {
				rx * RadToDeg,
				ry * RadToDeg,
				rz * RadToDeg,
				T.m[0][3], // Px
				T.m[1][3], // Py
				T.m[2][3]  // Pz
			};
		}
		//-------------------



		/**
		 * @brief 旋转向量投影 (ProjectRotationVector)
		 * 将 pos_original 的旋转值视为矢量，投影到 RRP 坐标系下。
		 *
		 * @param pos_original 原始姿态及位置 [rx, ry, rz, x, y, z] (角度/或弧度都可以)
		 * @param rrp 参考坐标系姿态及位置 [rx, ry, rz, x, y, z] (角度)
		 * @return std::vector<float> 投影后的分量 [rx', ry', rz', x, y, z]
		 */
		std::vector<float>MotionCompensationManager::ProjectRotationVector(const std::vector<float>& pos_original, const std::vector<float>& rrp)
		{
			// 创建返回结果，大小为6
			std::vector<float> result(6);
			
			// 1. 预计算三角函数值
			// RRP 的旋转角度转弧度
			float rrpX = rrp[0] * DegToRad;
			float rrpY = rrp[1] * DegToRad;
			float rrpZ = rrp[2] * DegToRad;

			float cx = std::cos(rrpX);
			float sx = std::sin(rrpX);
			float cy = std::cos(rrpY);
			float sy = std::sin(rrpY);
			float cz = std::cos(rrpZ);
			float sz = std::sin(rrpZ);

			// 2. 构建旋转矩阵 R (Matrix 3x3)
			// 对应 Z-Y-X 顺规： R = Rz * Ry * Rx
			// m00 m01 m02
			// m10 m11 m12
			// m20 m21 m22

			float m00 = cz * cy;
			float m01 = cz * sy * sx - sz * cx;
			float m02 = cz * sy * cx + sz * sx;

			float m10 = sz * cy;
			float m11 = sz * sy * sx + cz * cx;
			float m12 = sz * sy * cx - cz * sx;

			float m20 = -sy;
			float m21 = cy * sx;
			float m22 = cy * cx;

			// 3. 计算投影
			// 输入矢量 V (将前三个角度视为矢量)
			float vx = pos_original[0];
			float vy = pos_original[1];
			float vz = pos_original[2];

			// 公式：V_local = R_inverse * V_global
			// 对于旋转矩阵，逆矩阵等于转置矩阵 (R^-1 = R^T)
			// 所以我们用 R 的转置来乘以 V
			//
			// | m00 m10 m20 |   | vx |
			// | m01 m11 m21 | x | vy |
			// | m02 m12 m22 |   | vz |

			result[0] = m00 * vx + m10 * vy + m20 * vz; // New Pitch (rx)
			result[1] = m01 * vx + m11 * vy + m21 * vz; // New Roll (ry)
			result[2] = m02 * vx + m12 * vy + m22 * vz; // New Yaw (rz)

			// 4. 位置部分直接透传
			result[3] = pos_original[3];
			result[4] = pos_original[4];
			result[5] = pos_original[5];

			return result;
		}




		/// <summary>
		/// 计算眼睛在世界坐标系中的位置 (C++ 版 - 使用 HmdVector3d_t)
		/// </summary>
		/// <param name="rx">绕X轴旋转角度 (度) -> 对应 C# 传入的 Roll</param>
		/// <param name="ry">绕Y轴旋转角度 (度) -> 对应 C# 传入的 Pitch</param>
		/// <param name="rz">绕Z轴旋转角度 (度) -> 对应 C# 传入的 Yaw</param>
		/// <param name="eyePos">眼睛相对于平台中心的初始位置</param>
		/// <param name="platformPos">平台中心在世界坐标系中的平移位置</param>
		/// <returns>计算后的世界坐标</returns>
		vr::HmdVector3d_t MotionCompensationManager::GetEyeWorldPosition(double roll, double pitch, double yaw, vr::HmdVector3d_t eyePos)
		{
			// 1. 将角度转换为弧度
			// 根据 C# 代码逻辑: Matrix4x4.CreateFromYawPitchRoll(radZ, radY, radX)
			// 参数1 (Yaw/Y轴) = rz
			// 参数2 (Pitch/X轴) = ry
			// 参数3 (Roll/Z轴) = rx
			double radYaw = yaw * DegToRad;
			double radPitch = pitch * DegToRad;
			double radRoll = roll * DegToRad;

			// 2. 预计算三角函数
			double cYaw = std::cos(radYaw);
			double sYaw = std::sin(radYaw);

			double cPtch = std::cos(radPitch);
			double sPtch = std::sin(radPitch);

			double cRoll = std::cos(radRoll);
			double sRoll = std::sin(radRoll);

			// 3. 构建旋转矩阵元素 (展开 Matrix4x4.CreateFromYawPitchRoll 公式)
			// 旋转顺序: Roll(Z) -> Pitch(X) -> Yaw(Y)

			// X轴基向量 (M11, M12, M13)
			double m11 = cYaw * cRoll + sYaw * sPtch * sRoll;
			double m12 = cPtch * sRoll;
			double m13 = -sYaw * cRoll + cYaw * sPtch * sRoll;

			// Y轴基向量 (M21, M22, M23)
			double m21 = -cYaw * sRoll + sYaw * sPtch * cRoll;
			double m22 = cPtch * cRoll;
			double m23 = sYaw * sRoll + cYaw * sPtch * cRoll;

			// Z轴基向量 (M31, M32, M33)
			double m31 = sYaw * cPtch;
			double m32 = -sPtch;
			double m33 = cYaw * cPtch;

			// 4. 执行旋转变换 (Vector3.Transform 逻辑)
			// v' = v.x * X_Axis + v.y * Y_Axis + v.z * Z_Axis
			vr::HmdVector3d_t finalPos;


			finalPos.v[0] = eyePos.v[0] * m11 + eyePos.v[1] * m21 + eyePos.v[2] * m31;
			finalPos.v[1] = eyePos.v[0] * m12 + eyePos.v[1] * m22 + eyePos.v[2] * m32;
			finalPos.v[2] = eyePos.v[0] * m13 + eyePos.v[1] * m23 + eyePos.v[2] * m33;

			return finalPos;
		}
	}
}