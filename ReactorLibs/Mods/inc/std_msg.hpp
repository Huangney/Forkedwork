#ifndef STD_MSG_HPP
#define STD_MSG_HPP

#include "stm32f4xx_hal.h"
#include "std_math.hpp"

typedef enum
{
    MANA_TYPE_UART,
    MANA_TYPE_CAN,
    MANA_TYPE_NONE,
}ManagerType;

typedef enum
{
    MSG_TYPE_Pose2D,
    MSG_TYPE_Kinematic2D,
    MSG_TYPE_MoveCmd2D,
    MSG_TYPE_JoystickCmd,
    MSG_TYPE_PortCmd,
    MSG_TYPE_RobotLog,
    MSG_TYPE_None
}MessageType, MsgType;

typedef enum
{
    MSG_PRIOR_REALTIME,
    MSG_PRIOR_HIGH,
    MSG_PRIOR_NORMAL,
    MSG_PRIOR_LOW,
    MSG_PRIOR_NONE,
}MessagePrior, MsgPrior;


/**
 * @brief 消息管理器
 * @note  如果需要发送、解码格式化的信息，可以使用该类。类的方法与实际的句柄是解码的
 */
class MsgManager
{
    public:
        MsgManager(){};
        ~MsgManager(){};

        ManagerType mana_type;
        
        void Init(ManagerType type);
        void Encode(MsgType type, void* in_data, void* out_data);
        void Decode(MsgType type, void* in_data, void* out_data);
};














/************************       消息类型定义        ***********************/
namespace StdMsgDef
{

/// @brief 2D位姿消息
typedef struct
{
    float x;
    float y;
    float theta;
}Pose2D;

/// @brief 2D运动学消息
typedef struct
{
    Pose2D pos;
    Vec3 vel;
}Kinematic2D;

/// @brief 移动指令
typedef struct
{
    MsgPrior priority;
    Vec3 vel;
}MoveCmd2D;

/**
 * @brief 摇杆（遥控器）指令
 * @details 实际在传输的数据： 四路摇杆数据、十六个按键状态（uint16_t的bit）、四个拨杆
 * @attention 遥控器指令默认优先级是Normal
 */
typedef struct
{
    Vec2 lef_stick;   // 左摇杆
    Vec2 rig_stick;  // 右摇杆
    uint16_t key_state; // 按键状态
    uint16_t dial;      // 拨杆状态（一个拨杆占4bit）
}JoystickCmd;



typedef struct
{
    uint8_t port_id;    // 端口号
    uint8_t action;     // 端口动作
}PortCmd;



/**
 * @brief 机器人的运行情况日志
 * @warning 请注意，该结构体不用于CAN通信，因为其非常长。
 * 同时其中的 `bool变量` 不会真的发送到总线上，而是会在Encode中被编码
 */
typedef struct
{
    uint32_t RuntimeTick;
    uint8_t RobotState;             // 机器人状态
    uint8_t RobotSubState;          // 机器人子状态
    bool ChassisMotorOnline[4];     // 底盘在线
    bool OdomOnline;                // 里程计在线
    bool HostCompOnline;            // 工控机在线
    bool SubBoardOnline[2];         // 子板在线
    bool ActionMotorOnline[16];     // 执行操作的电机在线
    bool Periphs_0_Online[8];       // 其他外设在线
    bool Periphs_1_Online[8];       // 其他外设在线    
    bool Periphs_2_Online[8];       // 其他外设在线          
}RobotLog;




}
#endif