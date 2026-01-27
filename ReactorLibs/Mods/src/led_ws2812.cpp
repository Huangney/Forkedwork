#include "led_ws2812.hpp"
#include "arm_math.h"
#include "bsp_dwt.h"

static LedWs2812* targ_led;


void LedWs2812::Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t LedNums)
{
    // 记录所用的PWM通道，灯数
    this->htim = htim;
    this->Channel = Channel;
    this->LedNums = LedNums;

    for(int i = 0; i < MaxLedNums ; i++)
    {
        SetColor(i, 0, 0, 0);
    }

    // 初始化PWM
    PwmMaxValue = __HAL_TIM_GET_AUTORELOAD(this->htim);
    // HIGH_WS2812 = PwmMaxValue * 0.67f;    // PWM高电平数值
    // LOW_WS2812 = PwmMaxValue * 0.33f;           // PWM低电平数值   
    HIGH_WS2812 = PwmMaxValue * 0.67f;    // PWM高电平数值
    LOW_WS2812 = PwmMaxValue * 0.33f;           // PWM低电平数值   

    // 注册
    targ_led = this;
}


/**
 * @brief 设置某个LED的颜色
 * @warning 输入的RGB为0~255范围
 */
void LedWs2812::SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    // 不允许 Led_id 大于 实际拥有的灯数
    while(Led_id > (LedNums - 1))   Led_id -= LedNums;
    while(Led_id < 0)   Led_id += LedNums;


    // 向 数组 覆写颜色
	int i = 0;
	for(i=0;i<8;i++) WS2812buf2send[Led_id][i]   = ( G & (1 << (7 -i))? (HIGH_WS2812):LOW_WS2812 ); 
	for(i=8;i<16;i++) WS2812buf2send[Led_id][i]  = ( R & (1 << (15-i))? (HIGH_WS2812):LOW_WS2812 ); 
	for(i=16;i<24;i++) WS2812buf2send[Led_id][i] = ( B & (1 << (23-i))? (HIGH_WS2812):LOW_WS2812 ); 
}

/**
 * @brief 两个颜色的流动渐变模式（改进版：每个LED独立循环并保持固定相位差）
 * @param color_0 颜色1
 * @param color_1 颜色2
 * @param resolu 渐变流动的分辨率（值越大流动越平滑）
 * @param period 流动周期，单位秒
 */
void LedWs2812::GradientFlow(Color color_0, Color color_1, float period)
{
    color_0 = color_0 * BiasFactor;
    color_1 = color_1 * BiasFactor;

    // 记录已运行时间
    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = fmod(RuntimeCnt, period);
    }

    // 计算每个LED之间的相位差，确保整体效果连续
    float phaseStep = (float)period / LedNums;
    
    
    for (int i = 0; i < LedNums; i++)
    {
        // 计算当前LED的相位（带偏移）
        float phase = fmod(RuntimeCnt + (i * phaseStep), period);
        
        // 将相位转换为0-1之间的比例（0表示颜色1，1表示颜色2）
        // 使用正弦曲线让过渡更平滑，也可以改用线性过渡：float ratio = (float)phase / period;
        float ratio = 0.5f * (1.0f + sinf(2 * 3.1415 * phase / period - 1.5708));
        
        // 根据比例计算当前位置的颜色（在两种颜色间插值）
        Color color_targ = (color_0 + (color_1 - color_0) * ratio);

        // 设置当前位置的颜色（亮度调整）
        SetColor(i, color_targ.r, color_targ.g, color_targ.b);
    }
}

/**
 * @brief 单色呼吸灯
 * @details 呼吸灯的亮灭不是线性的哦，而是遵循正弦曲线变化的
 */
void LedWs2812::Breath(Color color_0, float period)
{
    color_0 = color_0 * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = fmod(RuntimeCnt, period);
    }

    // 计算亮度比例（0-1之间），使用正弦函数实现平滑过渡
    float ratio = 0.5f * (1.0f + sinf((2*PI) * RuntimeCnt / period - (PI/2)));

    // 计算并填充当前颜色
    Color color_targ = color_0 * ratio;
    Fill(color_targ.r, color_targ.g, color_targ.b);
}


/**
 * @brief 单色跑马灯
 * @details 颜色在灯带上循环移动，不在亮带上的灯熄灭
 * @param width 跑马灯的宽度（0~1），表示占整个灯带的比例
 */
void LedWs2812::Running(Color color, float width, float period)
{
    color = color * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = fmod(RuntimeCnt, period);
    }

    // 计算当前跑马灯的起始位置（0~LedNums范围内）
    float position = (RuntimeCnt / period) * LedNums;
    int width_in_leds = width * LedNums;

    for (int i = 0; i < LedNums; i++)
    {
        // 计算当前LED与跑马灯位置的距离
        float distance = fabsf(i - position);
        
        // 判断是否在跑马灯的宽度范围内（考虑循环）
        if (distance <= (width_in_leds / 2) || distance >= (LedNums - width_in_leds / 2))
        {
            // 在跑马灯范围内，设置颜色
            SetColor(i, color.r, color.g, color.b);
        }
        else
        {
            // 不在范围内，熄灭
            SetColor(i, 0, 0, 0);
        }
    }
}


/**
 * @brief 直接填充灯带颜色，每0.5s更新一次
 */
void LedWs2812::Lit(Color color)
{
    RuntimeCnt += DWT_GetDeltaTime(&dwt_tick);
    if (RuntimeCnt > 0.5)
    {
        color = color * BiasFactor;
        Fill(color.r, color.g, color.b);
        RuntimeCnt = 0;
    }
}

void LedWs2812::Expand(Color color_0, float period)
{
    // color_0 = color_0 * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);

    if (RuntimeCnt < period)
    {
        RuntimeCnt += delta_t;

        // 计算填充比例
        float ratio = RuntimeCnt / period;

        // 计算中心位置
        float center = LedNums / 2.0f;
        
        for (int i = 0; i < LedNums; i++)
        {
            float litness = 0;
            if (i < center)     litness = (i / center) + 2 * ratio - 1;
            else                litness = 2 * ratio - ((float)i - center) / center;
            

            if (litness < 0) litness = 0;
            if (litness > 1) litness = 1;
            Color color_targ = color_0 * litness;
            SetColor(i, color_targ.r, color_targ.g, color_targ.b);
        }
    }
}



/// @brief 直接填充灯带颜色
/// @param R 
/// @param G 
/// @param B 
void LedWs2812::Fill(uint8_t R, uint8_t G, uint8_t B)
{
    for(int i = 0; i < LedNums ; i++)
    {
        SetColor(i, R, G, B);
    }
}


/**
 * @brief 发送数据，更新LED的颜色（使用PWM + DMA）
 * @details 对于某一个LED的RGB信息，共24bit，每个bit发送：{0 ：取占空比33%，1：取占空比66%}
 * @warning PWM DMA被配置为byte了
 */
void LedWs2812::Upload()
{
    htim->Instance->CNT = htim->Instance->ARR - 1;
    HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)WS2812buf2send, LedNums * 24 + 1);
}


/**
 * @brief 覆写PWM + DMA发送完成中断
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == targ_led->htim)
    {
        HAL_TIM_PWM_Stop_DMA(targ_led->htim, targ_led->Channel);
    }
}