#ifndef __USER_TASKS_H
#define __USER_TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f0_discovery.h"

/*-----------------------------------------------------------*/

static void prvLED3BlinkTask(void *pvParameters);//LED3��˸����
static void prvButtonCheckTask(void *pvParameters);//�����������
static void prvLCDTask(void *pvParameters);//LCD����
static void prvUsart1Task(void *pvParameters);//���п�1ͨѶ����
void prvUserTasks(void);//�����ʼ������

void  USART1PutString(const char *const pcString, uint32_t u32StringLength);//USART�����ַ���
uint32_t Usart1GetChar(char *ch);//USART�����ַ�
uint32_t Usart1PutChar(char ch);//USART�����ַ�

/*-----------------------------------------------------------*/

extern xQueueHandle RxQueue, TxQueue;

/*-----------------------------------------------------------*/
#endif
