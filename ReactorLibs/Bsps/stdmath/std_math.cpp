#include "std_math.hpp"
#include "string.h"
#include "arm_math.h"


///////////////////////////////////////////     VEC3        /////////////////////////////////////////////////////
/*********      Byte_Vec3编码解码      **********/
void Vec3::ToBytes(uint8_t* buffer)
{
    memcpy(buffer, &(this->x), sizeof(float));
    memcpy(buffer + sizeof(float), &(this->y), sizeof(float));
    memcpy(buffer + 2 * sizeof(float), &(this->z), sizeof(float));
}
void Vec3::FromBytes(const uint8_t* buffer)
{
    memcpy(&(this->x), buffer, sizeof(float));
    memcpy(&(this->y), buffer + sizeof(float), sizeof(float));
    memcpy(&(this->z), buffer + 2 * sizeof(float), sizeof(float));
}


/*********      Vec3归一化      **********/
Vec3 Vec3::Norm() 
{
    Vec3 v = *this;
    // 计算向量的模
    float magnitude = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

    // 检查模是否为零，如果为零则返回零向量
    if (magnitude == 0) {
        Vec3 zero_vector = {0, 0, 0};
        return zero_vector;
    }

    // 归一化向量
    Vec3 normalized_vector;
    normalized_vector.x = v.x / magnitude;
    normalized_vector.y = v.y / magnitude;
    normalized_vector.z = v.z / magnitude;
    return normalized_vector;
}

/*********      Vec3长度      **********/
float Vec3::Length() 
{
    // 计算向量的模
    return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

/**********      Vec3转2      *********/
Vec2 Vec3::ToVec2()
{
    Vec2 v(this->x, this->y);
    return v;
}



///////////////////////////////////////////          VEC2        /////////////////////////////////////////////////////
/*********      Byte_Vec2编码解码      **********/
void Vec2::ToBytes(uint8_t* buffer)
{
    memcpy(buffer, &(this->x), sizeof(float));
    memcpy(buffer + sizeof(float), &(this->y), sizeof(float));
}
void Vec2::FromBytes(const uint8_t* buffer)
{
    memcpy(&(this->x), buffer, sizeof(float));
    memcpy(&(this->y), buffer + sizeof(float), sizeof(float));
}

/*********      Vec2    归一化      **********/
Vec2 Vec2::Norm() 
{
    Vec2 v = *this;
    // 计算向量的模
    float magnitude = sqrt(v.x * v.x + v.y * v.y);
    // 检查模是否为零，如果为零则返回零向量
    if (magnitude == 0) {
        Vec2 zero_vector = {0, 0};
        return zero_vector;
    }
    // 归一化向量
    Vec2 normed_vec;
    normed_vec.x = v.x / magnitude;
    normed_vec.y = v.y / magnitude;

    return normed_vec;
}

/*********      Vec2长度      **********/
float Vec2::Length() 
{
    // 计算向量的模
    return sqrt(this->x * this->x + this->y * this->y);
}

/*********      Vec2角度      **********/
///@brief 计算弧度制（RAD）的角度值
float Vec2::Angle() 
{
    // 使用atan2函数计算向量的角度，atan2返回值范围是[-π, π]
    return atan2(this->y, this->x);
}

/*********      Vec2 旋转      **********/
///@brief 计算弧度制（RAD）的角度值
Vec2 Vec2::Rotate(float thetaRad)
{
    Vec2 result;
    float cosTheta = cosf(thetaRad);
    float sinTheta = sinf(thetaRad);
    
    // 旋转公式：x' = x*cosθ - y*sinθ, y' = x*sinθ + y*cosθ
    result.x = this->x * cosTheta - this->y * sinTheta;
    result.y = this->x * sinTheta + this->y * cosTheta;

    return result;
}

/*********      Vec2转3      **********/
Vec3 Vec2::ToVec3()
{
    Vec3 v(this->x, this->y, 0);
    return v;
}
///////////////////////////////////////////          COLOR        /////////////////////////////////////////////////////
Color Color::Red = Color(255.0f, 0.0f, 0.0f);   // 红色（r=1, g=0, b=0）
Color Color::Green = Color(0.0f, 255.0f, 0.0f); // 绿色（r=0, g=1, b=0）
Color Color::Blue = Color(0.0f, 0.0f, 255.0f);  // 蓝色（r=0, g=0, b=1）
Color Color::White = Color(255.0f, 255.0f, 255.0f);  // 白色（r=1, g=1, b=1）

/*********      限幅     **********/

float Limit_ABS(float targ_num, float limit_mx)
{
    if (targ_num > limit_mx)
    {
        return limit_mx;
    }
    if (targ_num < -limit_mx)
    {
        return -limit_mx;
    }
    return targ_num;
}


float StdMath::RpmToRadS(float rpm)
{
    return rpm * (2.0f * 3.1415926f) / 60.0f;
}

float StdMath::RadSToRpm(float rad_s)
{
    return rad_s * 60.0f / (2.0f * 3.1415926f);
}

float StdMath::fclamp(float val, float limit)
{
    if (limit <= 0.0f)  return val;         // 0代表不限制
    if (val > limit)    return limit;
    if (val < -limit)   return -limit;
    return val;
}

int StdMath::signf(float val)
{
    if (val > 0)        return 1;
    else if (val < 0)   return -1;
    else return 0;
}
