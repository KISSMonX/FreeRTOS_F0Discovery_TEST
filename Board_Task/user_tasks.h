#ifndef __USER_TASKS_H
#define __USER_TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f0_discovery.h"

/*-----------------------------------------------------------*/

static void prvLED3BlinkTask(void *pvParameters);//LED3闪烁任务
static void prvButtonCheckTask(void *pvParameters);//按键检测任务
static void prvLCDTask(void *pvParameters);//LCD任务
static void prvUsart1Task(void *pvParameters);//串行口1通讯任务
void prvUserTasks(void);//任务初始化函数

void  USART1PutString(const char *const pcString, uint32_t u32StringLength);//USART发送字符串
uint32_t Usart1GetChar(char *ch);//USART接收字符
uint32_t Usart1PutChar(char ch);//USART发送字符

/*-----------------------------------------------------------*/

extern xQueueHandle RxQueue, TxQueue;

/*-----------------------------------------------------------*/
#endif
