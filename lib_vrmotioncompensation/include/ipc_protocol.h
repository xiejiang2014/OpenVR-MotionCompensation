#pragma once

#include "vrmotioncompensation_types.h"
#include <utility>
#include <chrono>

//确保通信双方使用相同的协议版本，避免数据解析错误。
#define IPC_PROTOCOL_VERSION 3

namespace vrmotioncompensation
{
	namespace ipc
	{
		// 消息类型
		enum class RequestType : uint32_t
		{
			None,

			// IPC connection handling 基础的连接管理和心跳检测。
			IPC_ClientConnect,
			IPC_ClientDisconnect,
			IPC_Ping,

			// 获取设备信息、设置运动补偿模式、设置滤波参数、重置零位、设置偏移量等。
			DeviceManipulation_GetDeviceInfo,
			DeviceManipulation_MotionCompensationMode,
			DeviceManipulation_SetMotionCompensationProperties,
			DeviceManipulation_ResetRefZeroPose,
			DeviceManipulation_SetOffsets,

			// 控制调试日志。
			DebugLogger_Settings,
		};

		// 定义服务端给客户端的回应类型。
		enum class ReplyType : uint32_t
		{
			None,
			IPC_ClientConnect,
			IPC_Ping,
			GenericReply,
			DeviceManipulation_GetDeviceInfo
		};

		// 状态码
		enum class ReplyStatus : uint32_t
		{
			None,
			Ok,
			UnknownError,
			InvalidId,				//ID无效
			AlreadyInUse,
			InvalidType,
			NotFound,
			SharedMemoryError,		//共享内存错误
			InvalidVersion,
			MissingProperty,
			InvalidOperation,
			NotTracking				//未追踪
		};

		// 握手请求，包含协议版本和队列名称
		struct Request_IPC_ClientConnect
		{
			uint32_t messageId;
			uint32_t ipcProcotolVersion;
			char queueName[128];
		};

		struct Request_IPC_ClientDisconnect
		{
			uint32_t clientId;
			uint32_t messageId;
		};

		struct Request_IPC_Ping
		{
			uint32_t clientId;
			uint32_t messageId;
			uint64_t nonce;
		};

		struct Request_OpenVR_GenericClientMessage
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
		};

		struct Request_OpenVR_GenericDeviceIdMessage
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
			uint32_t OpenVRId;
		};


		struct Request_DeviceManipulation_MotionCompensationMode
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
			uint32_t MCdeviceId;		// Motion compensated device ID  哪个设备需要被补偿（通常是头显）。
			uint32_t RTdeviceId;		// Reference tracker device ID   哪个设备是参考追踪器（固定在动感座椅上的追踪器）
			MotionCompensationMode CompensationMode;			//使用哪种补偿算法。
		};

		struct Request_DeviceManipulation_SetMotionCompensationProperties
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
			double LPFBeta;				//低通滤波器参数（用于平滑数据，减少抖动）。
			uint32_t samples;
			bool setZero;				//是否归零
			//MMFstruct_v1 offsets;
		};

		struct Request_DeviceManipulation_ResetRefZeroPose
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
		};

		// 设置物理偏移量（比如追踪器安装位置和实际座椅中心的距离）。
		struct Request_DeviceManipulation_SetOffsets
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
			MMFstruct_H2VR offsets;
		};

		struct Request_DebugLogger_Settings
		{
			uint32_t clientId;
			uint32_t messageId;			// Used to associate with Reply
			uint32_t MaxDebugPoints;
			bool enabled;
		};

		struct Request
		{
			Request()
			{
			}
			Request(RequestType type) : type(type)
			{
				timestamp = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			}
			Request(RequestType type, uint64_t timestamp) : type(type), timestamp(timestamp)
			{
			}

			void refreshTimestamp()
			{
				timestamp = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			}

			RequestType type = RequestType::None;
			int64_t timestamp = 0; // milliseconds since epoch  时间戳
			union MsgUnion  //这是一个共用体，意味着一段内存空间可以根据 type 被解释为上述任何一种 Request_... 结构体。
			{
				Request_IPC_ClientConnect ipc_ClientConnect;
				Request_IPC_ClientDisconnect ipc_ClientDisconnect;
				Request_IPC_Ping ipc_Ping;
				Request_OpenVR_GenericClientMessage ovr_GenericClientMessage;
				Request_OpenVR_GenericDeviceIdMessage ovr_GenericDeviceIdMessage;
				Request_DeviceManipulation_MotionCompensationMode dm_MotionCompensationMode;
				Request_DeviceManipulation_SetMotionCompensationProperties dm_SetMotionCompensationProperties;
				Request_DeviceManipulation_ResetRefZeroPose dm_ResetRefZeroPose;
				Request_DeviceManipulation_SetOffsets dm_SetOffsets;
				Request_DebugLogger_Settings dl_Settings;
				MsgUnion()
				{
				}
			} msg;
		};

		struct Reply_IPC_ClientConnect
		{
			uint32_t clientId;
			uint32_t ipcProcotolVersion;
		};

		struct Reply_IPC_Ping
		{
			uint64_t nonce;
		};

		struct Reply_DeviceManipulation_GetDeviceInfo
		{
			uint32_t OpenVRId;
			vr::ETrackedDeviceClass deviceClass;
			MotionCompensationDeviceMode deviceMode;
		};

		struct Reply
		{
			Reply()
			{
			}
			Reply(ReplyType type) : type(type)
			{
				timestamp = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			}
			Reply(ReplyType type, uint64_t timestamp) : type(type), timestamp(timestamp)
			{
			}

			ReplyType type = ReplyType::None;
			uint64_t timestamp = 0; // milliseconds since epoch
			uint32_t messageId;  //用于将回复和请求对应起来
			ReplyStatus status; // 成功 / 失败）
			union MsgUnion  //存放具体的回复数据
			{
				Reply_IPC_ClientConnect ipc_ClientConnect;
				Reply_IPC_Ping ipc_Ping;
				Reply_DeviceManipulation_GetDeviceInfo dm_deviceInfo;
				MsgUnion()
				{
				}
			} msg;
		};

	} // end namespace ipc
} // end namespace vrmotioncompensation
