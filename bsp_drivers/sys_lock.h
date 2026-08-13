#ifndef __SYS_LOCK_H
#define __SYS_LOCK_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "stdio.h"

/* 全局 printf 互斥锁 (freertos.c 中创建) */
extern SemaphoreHandle_t g_xPrintfMutex;

/* 线程安全的 printf 宏
 * 因为 fputc 使用阻塞 HAL_UART_Transmit(HAL_MAX_DELAY),
 * 两个任务同时调用 printf 会导致 HAL 状态机死锁 */
#define SAFE_PRINTF(...) \
    do { \
        xSemaphoreTake(g_xPrintfMutex, portMAX_DELAY); \
        printf(__VA_ARGS__); \
        xSemaphoreGive(g_xPrintfMutex); \
    } while(0)

#endif /* __SYS_LOCK_H */
