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
		MotionCompensationManager::MotionCompensationManager(ServerDriver* parent) : m_parent(parent)
		{
			try
			{
				// create shared memory  创建内存映射
				_shdmem = { boost::interprocess::open_or_create, "OVRMC_MMFv1", boost::interprocess::read_write, 4096 };
				//获取这个映射的元素内存
				_region = { _shdmem, boost::interprocess::read_write };

				// get pointer address and fill it with data
				// 强行以 MMFstruct_OVRMC_v1 为类型来操作映射的内存,并将指针放到 _Poffset 里面
				_Poffset = static_cast<MMFstruct_OVRMC_v1*>(_region.get_address());
				*_Poffset = _Offset;  //向指针填充数据
				LOG(INFO) << "Shared memory OVRMC_MMFv1 created";
			}
			catch (boost::interprocess::interprocess_exception& e)
			{
				LOG(ERROR) << "Could not create or open shared memory. Error code " << e.get_error_code();
			}
		}

		bool MotionCompensationManager::setMotionCompensationMode(MotionCompensationMode Mode, int McDevice, int RtDevice)
		{
			if (Mode == MotionCompensationMode::ReferenceTracker)
			{
				_RefPoseValid = false;
				_RefPoseValidCounter = 0;
				_ZeroPoseValid = false;

				LOG(INFO) << "setMotionCompensationMode	_ZeroPoseValid->false";
				_Enabled = true;

				setAlpha(_Samples);
			}
			else
			{
				_Enabled = false;
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
			LOG(INFO) << "setNewReferenceTracker	_ZeroPoseValid->false";
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

		void MotionCompensationManager::setOffsets(MMFstruct_OVRMC_v1 offsets)
		{
			//_Offset.Translation = offsets.Translation;
			//_Offset.Rotation = offsets.Rotation;
			_Offset = offsets;
			*_Poffset = _Offset;
		}

		bool MotionCompensationManager::isZeroPoseValid()
		{
			return _ZeroPoseValid;
		}

		void MotionCompensationManager::resetZeroPose()
		{
			_ZeroPoseValid = false;
			LOG(INFO) << "resetZeroPose	_ZeroPoseValid->false";
		}

		void MotionCompensationManager::setZeroPose(const vr::DriverPose_t& pose)
		{
			// convert pose from driver space to app space
			vr::HmdQuaternion_t tmpConj = vrmath::quaternionConjugate(pose.qWorldFromDriverRotation);


			// Save zero points
			_ZeroLock.lock();
			_ZeroPos = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, pose.vecPosition, false) + pose.vecWorldFromDriverTranslation;
			_ZeroRot = pose.qWorldFromDriverRotation * pose.qRotation;
			_ZeroPoseValid = true;


			LOG(INFO) << "setZeroPose	_ZeroPoseValid->true";
			LOG(INFO) << "setZeroPose	Pos:	"<< _ZeroPos.v[0] << "	" << _ZeroPos.v[1] << "	" << _ZeroPos.v[2];
			LOG(INFO) << "setZeroPose	Rot:	" << _ZeroRot.w << "	" << _ZeroRot.x << "	" << _ZeroRot.y << "	" << _ZeroRot.z;

			_ZeroLock.unlock();
		}

		//下面的两个函数  
		// updateRefPose 用于计算tracker的姿态, 
		// applyMotionCompensation 将 updateRefPose 计算出的姿态作为参考进行姿态补偿
		//以下是 applyMotionCompensation 中用到的所有来自 updateRefPose 计算的结果列表，按数据类型分类：
		//	1. 状态标志
		//		_RefPoseValid
		//			用途 : 用于判断条件 if (... && _RefPoseValid)。
		//			来源 : updateRefPose 结尾处。只有当参考追踪器连续稳定运行超过 100 帧后，该值才会被设为 true，防止在数据尚未稳定时应用补偿。
		//	2. 核心姿态数据(位置与旋转)
		//		_RefPos(参考位置)
		//			用途 : 在计算 compensatedPoseWorldPos 时使用：poseWorldPos - _RefPos。
		//			逻辑 : 计算头显相对于动感座椅当前位置的偏移量。
		//		_RefRot(参考旋转差)
		//			用途 : 在 vrmath::quaternionRotateVector(_RefRot, ...) 中作为参数。
		//			逻辑 : 用于辅助计算旋转后的向量偏移。
		//		_RefRotInv(参考旋转的逆)
		//			用途 :
		//			计算位置补偿时：vrmath::quaternionRotateVector(..., _RefRotInv, ...)。
		//			计算旋转补偿时：compensatedPoseWorldRot = _RefRotInv * poseWorldRot。
		//			逻辑 : 这是最关键的变量。它代表了座椅旋转的“反方向”。把它应用到头显上，就能抵消掉座椅的旋转运动。
		//	3. 物理属性数据(用于修正预测)
		//		这些变量在 else 分支中（即非归零模式下）被使用，用于直接从头显的速度 / 加速度中减去座椅的运动分量，防止 SteamVR 的预测算法失效。
		//		_RefVel(参考线速度)
		//			用途: pose.vecVelocity[...] -= tmpPosVel.v[...](经过坐标转换后的 _RefVel)。
		//		_RefRotVel(参考角速度)
		//			用途 : pose.vecAngularVelocity[...] -= tmpRotVel.v[...]。
		//		_RefAcc(参考线加速度)
		//			用途 : pose.vecAcceleration[...] -= tmpPosAcc.v[...]。
		//		_RefRotAcc(参考角加速度)
		//			用途 : pose.vecAngularAcceleration[...] -= tmpRotAcc.v[...]。
		//	
		//	总结对照表
		//	变量名			在 updateRefPose 中(生产者)			在 applyMotionCompensation 中(消费者)
		//	_RefPoseValid	被赋值为 true (当计数器 > 100)			用于 if 判断是否执行补偿
		//	_RefPos			计算并更新(世界坐标系)					被头显位置减去(计算相对位置)
		//	_RefRot			计算(当前旋转 * 归零旋转的逆)			用于向量旋转计算
		//	_RefRotInv		计算(_RefRot 的共轭 / 逆)				乘到头显旋转上(核心抵消操作)
		//	_RefVel			计算(位置微分 + 坐标转换)				从头显线速度中减去
		//	_RefRotVel		计算(角度微分 + 坐标转换)				从头显角速度中减去
		//	_RefAcc			计算(速度微分 + 坐标转换)				从头显线加速度中减去
		//	_RefRotAcc		计算(角速微分 + 坐标转换)				从头显角加速度中减去




		/// <summary>
		/// 计算追踪器的姿态,注意参数中的 pose 应该就是从openvr得到的原始姿态值, 在这个函数中是不会去修改它的.
		/// 本函数会更新 _RefPos, _RefRotInv, _RefVel  的值 留给 applyMotionCompensation 中使用
		/// </summary>
		/// <param name="pose"></param>
		void MotionCompensationManager::updateRefPose(const vr::DriverPose_t& pose)
		{
			// From https://github.com/ValveSoftware/driver_hydra/blob/master/drivers/driver_hydra/driver_hydra.cpp Line 835:
			// "True acceleration is highly volatile, so it's not really reasonable to
			// extrapolate much from it anyway.  Passing it as 0 from any driver should
			// be fine."

			// Line 832:
			// "The trade-off here is that setting a valid velocity causes the controllers
			// to jitter, but the controllers feel much more "alive" and lighter.
			// The jitter while stationary is more annoying than the laggy feeling caused
			// by disabling velocity (which effectively disables prediction for rendering)."
			// That means that we have to calculate the velocity to not interfere with the prediction for rendering

			// Oculus devices do use acceleration. It also seems that the HMD uses theses values for render-prediction


			// "真实的加速度数据波动性很强，因此无论如何都不宜基于它进行过多推测。
			// 任何驱动程序将其传递为0值都是可以接受的。"
			// 第832行：
			// "此处的权衡在于：设置有效速度会导致控制器产生抖动，
			// 但控制器的反馈会显得更'鲜活'和轻盈。
			// 静止状态下的抖动感，比关闭速度数据（这实际会禁用渲染预测）
			// 所造成的延迟感更令人困扰。"
			// 这意味着我们必须计算速度数据，以避免干扰渲染预测
			// Oculus设备确实会使用加速度数据。似乎头戴设备也会将这些数值用于渲染预测。




			//这段代码是 VR 运动补偿系统的核心“信号处理”模块。
			//	它的作用是：处理参考追踪器（Reference Tracker）的原始数据。
			//	你可以把这个函数想象成一个** “净化工厂”** 。
			//	原料：参考追踪器（绑在动感座椅上的那个）传来的原始位置和旋转数据（可能带有抖动和噪音）。
			//	加工：滤波（平滑去噪）、计算速度和加速度、坐标系转换。
			//	产品：一个平滑的、干净的、带有物理属性的“基准姿态”，供后续计算头显补偿时使用。


			vr::HmdVector3d_t Filter_vecPosition = { 0, 0, 0 };
			vr::HmdVector3d_t Filter_vecVelocity = { 0, 0, 0 };
			vr::HmdVector3d_t Filter_vecAcceleration = { 0, 0, 0 };
			vr::HmdVector3d_t Filter_vecAngularVelocity = { 0, 0, 0 };
			vr::HmdVector3d_t Filter_vecAngularAcceleration = { 0, 0, 0 };
			vr::HmdVector3d_t RotEulerFilter = { 0, 0, 0 };

			vr::HmdQuaternion_t tmpConj = vrmath::quaternionConjugate(pose.qWorldFromDriverRotation);

			// Get current time in microseconds and convert it to seconds
			// 获取微秒级当前时间
			long long now = std::chrono::duration_cast <std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

			//当前帧和上一帧的时间差
			double tdiff = (double)(now - _RefTrackerLastTime) / 1.0E6 + (pose.poseTimeOffset - _RefTrackerLastPose.poseTimeOffset);

			// Position
			// Add a exponential median average filter
			// 位置滤波
			if (_Samples >= 2)
			{
				// ----------------------------------------------------------------------------------------------- //
				// ----------------------------------------------------------------------------------------------- //
				// Position
				// DEMA(Double Exponential Moving Average) : 双重指数移动平均滤波。
				// 作用：去噪。动感座椅震动时，追踪器的数据会有微小的乱跳。这个滤波器把这些高频噪音滤掉，让位置变化更平滑。
				Filter_vecPosition.v[0] = DEMA(pose.vecPosition[0], 0);
				Filter_vecPosition.v[1] = DEMA(pose.vecPosition[1], 1);
				Filter_vecPosition.v[2] = DEMA(pose.vecPosition[2], 2);

				// ----------------------------------------------------------------------------------------------- //
				// ----------------------------------------------------------------------------------------------- //
				// Velocity and acceleration
				// 既然硬件的原始速度太抖，我们就算“数学速度”。
				// 速度 = (当前滤波位置 - 上一帧位置) / 时间差。
				// 加速度 = (当前速度 - 上一帧速度) / 时间差。
				if (!_SetZeroMode)
				{
					Filter_vecVelocity.v[0] = vecVelocity(tdiff, Filter_vecPosition.v[0], _RefTrackerLastPose.vecPosition[0]);
					Filter_vecVelocity.v[1] = vecVelocity(tdiff, Filter_vecPosition.v[1], _RefTrackerLastPose.vecPosition[1]);
					Filter_vecVelocity.v[2] = vecVelocity(tdiff, Filter_vecPosition.v[2], _RefTrackerLastPose.vecPosition[2]);

					Filter_vecAcceleration.v[0] = vecAcceleration(tdiff, Filter_vecVelocity.v[0], _RefTrackerLastPose.vecVelocity[0]);
					Filter_vecAcceleration.v[1] = vecAcceleration(tdiff, Filter_vecVelocity.v[1], _RefTrackerLastPose.vecVelocity[1]);
					Filter_vecAcceleration.v[2] = vecAcceleration(tdiff, Filter_vecVelocity.v[2], _RefTrackerLastPose.vecVelocity[2]);
				}
			}
			else
			{
				_copyVec(Filter_vecPosition, pose.vecPosition);
				_copyVec(Filter_vecVelocity, pose.vecVelocity);
			}

			// convert pose from driver space to app space
			// 坐标系转换 (Driver Space -> World Space)
			_RefLock.lock();
			_RefPos = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, Filter_vecPosition, false) + pose.vecWorldFromDriverTranslation;
			_RefLock.unlock();

			// ----------------------------------------------------------------------------------------------- //
			// ----------------------------------------------------------------------------------------------- //
			// Rotation
			// 旋转滤波
			if (_LpfBeta <= 0.9999)
			{
				// 两级低通滤波器

				// 1st stage
				_Filter_rotPosition[0] = lowPassFilterQuaternion(pose.qRotation, _Filter_rotPosition[0]);

				// 2nd stage
				_Filter_rotPosition[1] = lowPassFilterQuaternion(_Filter_rotPosition[0], _Filter_rotPosition[1]);


				vr::HmdVector3d_t RotEulerFilter = toEulerAngles(_Filter_rotPosition[1]);

				if (!_SetZeroMode)
				{
					Filter_vecAngularVelocity.v[0] = rotVelocity(tdiff, RotEulerFilter.v[0], _RotEulerFilterOld.v[0]);
					Filter_vecAngularVelocity.v[1] = rotVelocity(tdiff, RotEulerFilter.v[1], _RotEulerFilterOld.v[1]);
					Filter_vecAngularVelocity.v[2] = rotVelocity(tdiff, RotEulerFilter.v[2], _RotEulerFilterOld.v[2]);

					Filter_vecAngularAcceleration.v[0] = vecAcceleration(tdiff, Filter_vecAngularVelocity.v[0], _RefTrackerLastPose.vecAngularVelocity[0]);
					Filter_vecAngularAcceleration.v[1] = vecAcceleration(tdiff, Filter_vecAngularVelocity.v[1], _RefTrackerLastPose.vecAngularVelocity[1]);
					Filter_vecAngularAcceleration.v[2] = vecAcceleration(tdiff, Filter_vecAngularVelocity.v[2], _RefTrackerLastPose.vecAngularVelocity[2]);
				}
			}
			else
			{
				_Filter_rotPosition[1] = pose.qRotation;

				_copyVec(Filter_vecAngularVelocity, pose.vecAngularVelocity);
				_copyVec(Filter_vecAngularAcceleration, pose.vecAngularAcceleration);
			}

			// calculate orientation difference and its inverse
		// _ZeroRot: 动感座椅静止归零时的旋转角度（基准）。
		//	poseWorldRot : 动感座椅现在的旋转角度。
		//	_RefRot : 现在的角度相对于基准角度转了多少。
		//	_RefRotInv : _RefRot 的逆（Inverse）。
		//	意义：如果座椅向左转了 10 度，_RefRotInv 就是“向右转 10 度”。
		//	后续：这个逆旋转将被应用到头显上，从而抵消掉座椅的运动。

			vr::HmdQuaternion_t poseWorldRot = pose.qWorldFromDriverRotation * _Filter_rotPosition[1];
			_RefLock.lock();
			_ZeroLock.lock();
			//核心算法：计算“相对偏移”(The Magic)
			_RefRot = poseWorldRot * vrmath::quaternionConjugate(_ZeroRot);
			_RefRotInv = vrmath::quaternionConjugate(_RefRot);
			_ZeroLock.unlock();
			_RefLock.unlock();

			if (!_SetZeroMode)
			{
				// Convert velocity and acceleration values into app space
				_RefVelLock.lock();
				_RefVel = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, Filter_vecVelocity, false);
				_RefRotVel = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, Filter_vecAngularVelocity, false);

				_RefAcc = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, Filter_vecAcceleration, false);
				_RefRotAcc = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, Filter_vecAngularAcceleration, false);
				_RefVelLock.unlock();
			}

			// ----------------------------------------------------------------------------------------------- //
			// ----------------------------------------------------------------------------------------------- //
			// Wait 100 frames before setting reference pose to valid
			// 预热 滤波器（DEMA, LPF）需要一定的数据量才能稳定（收敛）。
			// 刚启动的前 100 帧（约 0.1 秒），数据可能不准，所以标记为“无效”，防止画面在一开始乱飞。
			if (_RefPoseValidCounter > 100)
			{
				_RefPoseValid = true;
			}
			else
			{
				_RefPoseValidCounter++;
			}

			// Save last rotation and pose
			_RotEulerFilterOld = RotEulerFilter;
			_RefTrackerLastPose = pose;
		}

		/// <summary>
		/// 应用动作补偿,
		/// </summary>
		/// <param name="pose">应该是头盔的姿态,注意本函数会修改这个姿态达到补偿的目的</param>
		/// <returns></returns>
		bool MotionCompensationManager::applyMotionCompensation(vr::DriverPose_t& pose)
		{
			if (_Enabled && _ZeroPoseValid && _RefPoseValid)//只有在功能开启、归零点有效、参考数据有效（前100帧热身完毕）时才工作。否则直接返回 true（不做任何修改）。
			{
				// All filter calculations are done within the function for the reference tracker, because the HMD position is updated 3x more often.
				// Convert pose from driver space to app space
				// 所有滤波器计算都在参考跟踪器的函数中完成，因为头显位置的更新频率是其他部分的3倍。
				// 将姿态从驱动程序坐标系转换到应用程序坐标系
				vr::HmdQuaternion_t tmpConj = vrmath::quaternionConjugate(pose.qWorldFromDriverRotation);

				//转换到世界坐标系 (Driver Space -> App Space):
				vr::HmdVector3d_t poseWorldPos = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, pose.vecPosition, false) + pose.vecWorldFromDriverTranslation;

				// Do motion compensation
				vr::HmdQuaternion_t poseWorldRot = pose.qWorldFromDriverRotation * pose.qRotation;
				_RefLock.lock();
				_ZeroLock.lock();

				//这是一行极其复杂的向量数学运算：
				vr::HmdVector3d_t compensatedPoseWorldPos = _ZeroPos + //把结果加回到基准位置
					vrmath::quaternionRotateVector(
					_RefRot, 
					_RefRotInv,             //应用座椅旋转的逆。如果不动，座椅转了 10 度，头显也会跟着转 10 度。这里我们让头显反向转 10 度，这样在视觉上头显就“不动”了。
					poseWorldPos - _RefPos, //计算头显相对于参考追踪器（座椅）的位置。
					true
				);

				_ZeroLock.unlock();

				//应用旋转补偿 直接把座椅旋转的逆 (_RefRotInv) 乘到头显旋转上。这实现了“去耦合”。
				vr::HmdQuaternion_t compensatedPoseWorldRot = _RefRotInv * poseWorldRot;
				_RefLock.unlock();

				// Translate the motion ref Velocity / Acceleration values into driver space and directly subtract them
				//处理速度和加速度:
				if (_SetZeroMode)
				{
					_zeroVec(pose.vecVelocity);
					_zeroVec(pose.vecAcceleration);
					_zeroVec(pose.vecAngularVelocity);
					_zeroVec(pose.vecAngularAcceleration);
				}
				else
				{
					// Translate the motion ref Velocity / Acceleration values into driver space and directly subtract them
					_RefVelLock.lock();
					vr::HmdVector3d_t tmpPosVel = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefVel, true);
					// 直接减去参考追踪器的速度
					pose.vecVelocity[0] -= tmpPosVel.v[0];
					pose.vecVelocity[1] -= tmpPosVel.v[1];
					pose.vecVelocity[2] -= tmpPosVel.v[2];

					vr::HmdVector3d_t tmpRotVel = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefRotVel, true);
					pose.vecAngularVelocity[0] -= tmpRotVel.v[0];
					pose.vecAngularVelocity[1] -= tmpRotVel.v[1];
					pose.vecAngularVelocity[2] -= tmpRotVel.v[2];

					vr::HmdVector3d_t tmpPosAcc = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefAcc, true);
					pose.vecAcceleration[0] -= tmpPosAcc.v[0];
					pose.vecAcceleration[1] -= tmpPosAcc.v[1];
					pose.vecAcceleration[2] -= tmpPosAcc.v[2];

					vr::HmdVector3d_t tmpRotAcc = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, _RefRotAcc, true);
					pose.vecAngularAcceleration[0] -= tmpRotAcc.v[0];
					pose.vecAngularAcceleration[1] -= tmpRotAcc.v[1];
					pose.vecAngularAcceleration[2] -= tmpRotAcc.v[2];
					_RefVelLock.unlock();


					//这部分非常重要！
					//	如果只改位置不改速度，SteamVR 的预测算法会发疯，导致画面抖动。
					//	代码计算了参考追踪器（座椅）的速度，并将其转换到 Driver Space，然后直接从头显的速度中减去。
					//	结果 : 告诉 SteamVR “虽然我的传感器说我在动，但实际上我在虚拟世界里没动（或者动得没那么快）”。
				}


				// convert back to driver space
				// 转换回驱动坐标系 (App Space -> Driver Space):
				// SteamVR 只要 Driver Space 的数据，所以算完还得转回去。
				//	关键点 : 这里直接修改了参数 pose 的成员变量。
				pose.qRotation = tmpConj * compensatedPoseWorldRot;
				vr::HmdVector3d_t adjPoseDriverPos = vrmath::quaternionRotateVector(pose.qWorldFromDriverRotation, tmpConj, compensatedPoseWorldPos - pose.vecWorldFromDriverTranslation, true);
				_copyVec(pose.vecPosition, adjPoseDriverPos.v);
			}
			return true;
		}

		void MotionCompensationManager::runFrame()
		{
			/*if (_Offset.Flags_1 & (1 << FLAG_ENABLE_MC) && _Mode == MotionCompensationMode::Disabled)
			{

			}
			else if (!(_Offset.Flags_1 & (1 << FLAG_ENABLE_MC)) && _Mode == MotionCompensationMode::ReferenceTracker)
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
	}
}