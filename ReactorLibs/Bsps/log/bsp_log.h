#pragma once
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif
typedef void (*cmd_handler_t)(int argc, char **argv);


typedef struct {
    const char *RESET;
    const char *RED;
    const char *GREEN;
    const char *YELLOW;
    const char *BLUE;
} _RTT_Color_t;

extern const _RTT_Color_t RTT_Color; // 外部声明

extern void BspLog_Init(void);
extern void BspLog_RecvCMD(void);


extern void BspLog_LogInfo(const char* fmt, ...);
extern void BspLog_LogWarning(const char* fmt, ...);
extern void BspLog_LogError(const char* fmt, ...);
extern void BspLog_LogOK(const char* fmt, ...);
extern void BspLog_LogSpec(const char* fmt, ...);
extern void BspLog_LogRespond(const char* fmt, ...);

extern void BspLog_RegistCMD(const char* name, cmd_handler_t handler, const char* help);

#ifdef __cplusplus
}
#endif