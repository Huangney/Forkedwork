#include "std_msg.hpp"

static void Encode_Pose2D(void* in_data, void* out_data);




/**
 * @brief 初始化
 */
void MsgManager::Init(ManagerType type)
{
    mana_type = type;
}

/**
 * @brief 解码方法
 */
void MsgManager::Encode(MsgType type, void* in_data, void* out_data)
{
    // 根据消息类型进行编码
    switch(type)
    {
        case MSG_TYPE_Pose2D:
        {
            Encode_Pose2D(in_data, out_data);
        }
            // 编码Pose2D消息
            break;
        case MSG_TYPE_Kinematic2D:
            // 编码Kinematic消息
            break;
        case MSG_TYPE_MoveCmd2D:
            // 编码Command消息
            break;
        default:break;
    }
}

static void Encode_Pose2D(void* in_data, void* out_data)
{
    
}
