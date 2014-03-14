/*-----------------------------------------------------------
 *创建5个任务：
 *1. LED3闪烁任务prvLED3BlinkTask
 *2. 按键检测任务prvButtonCheckTask
 *3. USART1通讯任务prvUsart1Task
 *4. LCD5110显示任务prvLCDTask
 *5. 增加 IO 口反转速度检测
 *-----------------------------------------------------------*/

/* 标准头文件 */	
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "user_tasks.h"	
#include "lcd.h"

/*-----------------------------------------------------------*/

#define	mainLED3_BLINK_TASK_PRIORITY	(tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY		(tskIDLE_PRIORITY + 2)
#define mainBUTTON_CHECK_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)
#define mainUSART1_TASK_PRIORITY	(tskIDLE_PRIORITY + 4)	 
#define mainPORTst_TASK_PRIORITY	(tskIDLE_PRIORITY + 5)	

#define DEBOUNCECOUNTS 			10
#define serPUT_STRING_CHAR_DELAY        (5 / portTICK_RATE_MS)
#define lcdPUT_DATA_DELAY		(1 / portTICK_RATE_MS)


/*-----------------------------------------------------------*/

//const char *const pcUsartTaskStartMsg = "USART task started.\r\n";
char * pcUsartTaskStartMsg = "USART task started.\r\n";
static xSemaphoreHandle xButtonSpeedUpSemaphore;	// 按键信号
xQueueHandle RxQueue, TxQueue;				// 串行口发送/接收队列 
uint8_t u8LCDFrameBuffer[LCD_X*LCD_Y/8];		// LCD 显示缓冲区

//=========================================================================================================
/**
 * @brief  LED3闪烁任务：每隔500ms切换LED3的显示状态.
 * @param  pvParameters:任务默认参数.
 * @retval 无
 */
 //=========================================================================================================
static void prvLED3BlinkTask(void *pvParameters)
{
        portTickType xNextWakeTime;
        const portTickType xFrequency = 100;
        xNextWakeTime = xTaskGetTickCount();
        for(;;)
        {				
                GPIOC->ODR ^= GPIO_Pin_9;
                vTaskDelayUntil(&xNextWakeTime, xFrequency);
        }
}


//=========================================================================================================
/**
 * @brief  测试 IO 口翻转速度与 CPU 工作频率
 * @param  pvParameters:任务默认参数.
 * @retval 无
 */
//=========================================================================================================
static void prvPB13_ToggleTask(void *pvParameters)
{
	for (;;) {
		GPIOB->ODR ^= GPIO_Pin_14;
		vTaskDelay(1);
	}
}	
	
	
//=========================================================================================================
/**
 * @brief  按键检测任务：每隔20ms检测按键状态。如果有按键事件则释放信号.
 * @param  pvParameters:任务默认参数.
 * @retval 无
 */
//=========================================================================================================
static void prvButtonCheckTask(void *pvParameters)
{	
        static uint8_t bounce_count;
        portTickType xNextWakeTime;
        const portTickType xFrequency = 2;
        xNextWakeTime = xTaskGetTickCount();

        /* 创建信号 */
        vSemaphoreCreateBinary(xButtonSpeedUpSemaphore);	

        /* 如果创建信号成功，将信号初始化为0 */	
        if (xButtonSpeedUpSemaphore != NULL) {
                xSemaphoreTake(xButtonSpeedUpSemaphore, (portTickType)0);
        }	

        for(;;) {
                /* 读取按键状态 */
                if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == pdTRUE)//如果检测到按键按下
                {
                        bounce_count++;
                        if (bounce_count == DEBOUNCECOUNTS)//按键防抖
                        {	
				GPIOC->ODR ^= GPIO_Pin_8;
                                xSemaphoreGive(xButtonSpeedUpSemaphore);//释放按键信号 
                        }
                }
                /* 每隔Nms检测一次 */ 
                vTaskDelayUntil(&xNextWakeTime,xFrequency);
        }
}


//=========================================================================================================
/**
 * @brief  USART1通讯任务：发送字符串并将PC发送来的字符返回PC.
 * @param  pvParameters:任务默认参数.
 * @retval 无
 */
//=========================================================================================================
static void prvUsart1Task(void *pvParameters)
{
        char ch;
        portTickType xLastWakeTime;   
        const portTickType xFrequency = 50;   
        xLastWakeTime = xTaskGetTickCount();   

        /* 创建能容纳128个字符的队列 */
        RxQueue = xQueueCreate(configCOM1_RX_BUFFER_LENGTH, sizeof(portCHAR));   
        TxQueue = xQueueCreate(configCOM1_TX_BUFFER_LENGTH, sizeof(portCHAR));

        USART1PutString(pcUsartTaskStartMsg, strlen(pcUsartTaskStartMsg));

        for( ;; )   
        {             
                if (Usart1GetChar(&ch)) {           
                        Usart1PutChar(ch);         
                }       
                vTaskDelayUntil(&xLastWakeTime,xFrequency);   
        }  	
}


//=========================================================================================================
/**
 * @brief  向队列发送字符.
 * @param  ch:待发送的字符.
 * @retval 无
 */
//=========================================================================================================
uint32_t Usart1PutChar(char ch)
{
        if(xQueueSend(TxQueue, &ch, 10) == pdPASS)//发送字符成功
        {
                USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//使能发送中断，当发送寄存器为空时触发该中断
                return pdTRUE;
        }
        else {
                return pdFAIL;
        }
}


//=========================================================================================================
/**
 * @brief  从队列读取字符.
 * @param  ch:从队列读取的字符.
 * @retval 无
 */
//=========================================================================================================
uint32_t Usart1GetChar(char *ch)
{
        if(xQueueReceive(RxQueue, ch, 0) == pdPASS)//读取字符成功
        {
                return pdTRUE;
        }
        return pdFALSE;
}


//=========================================================================================================
/**
 * @brief  向队列发送字符串数组.
 * @param  pcString: 待写入的字符串数组指针.
 * @param  u32StringLength: 待写入字符串数组长度.
 * @retval 无
 */
//=========================================================================================================
void  USART1PutString(const char *const pcString, uint32_t u32StringLength)
{
        uint32_t i;
        for(i = 0; i < u32StringLength; i++)		
        {
                if(xQueueSend(TxQueue, &(pcString[i]), serPUT_STRING_CHAR_DELAY) != pdPASS)
                {					
                        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//队列已装满，使能发送中断发送数据.
                        vTaskDelay(serPUT_STRING_CHAR_DELAY);						
                        continue;
                }
        }	
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//使能发送中断
}


//=========================================================================================================
/**
  * @brief  LCD任务：每隔500ms清屏并显示固定字符串“Hello World！”.
  * @param  pvParameters:任务默认参数.
  * @retval 无
  */
//=========================================================================================================
static void prvLCDTask(void *pvParameters)
{
	uint8_t lcd_row = 0;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 500;
	xLastWakeTime=xTaskGetTickCount();
	/* 初始化LCD */
	LCD_Init();
	/* 清除显示缓冲区 */
	LCD_Clear();
	/* 将数据写入显示缓冲区 */
	LCD_WriteString(0, 0, "Hello World!");
	/* 刷新LCD */
	for(;;)
	{
		if (DMA_GetCurrDataCounter(DMA1_Channel5) == 0) {
			LCD_Send((uint32_t)(u8LCDFrameBuffer + lcd_row * LCD_X), LCD_X);
//			vTaskDelay(lcdPUT_DATA_DELAY);
			if(lcd_row == LCD_Y/8 - 1)
				lcd_row = 0;
			else
				lcd_row += 1;
		}
		
		if (lcd_row == 0) {
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
	}		
}


//=========================================================================================================
/**
 * @brief  创建所有任务.
 * @param  无
 * @retval 无
 */
//=========================================================================================================
void prvUserTasks(void)
{
        /* 创建任务 */
        xTaskCreate(prvLED3BlinkTask,			//任务函数
                        ( char *) "LED3 BLINK",   	//任务名称
                        configMINIMAL_STACK_SIZE, 	//任务堆栈
                        NULL, 				//任务参数
                        mainLED3_BLINK_TASK_PRIORITY,	//任务优先级 
                        NULL);				//任务句柄
			
        xTaskCreate(prvButtonCheckTask,
                        ( char *) "BTN CHECK",
                        configMINIMAL_STACK_SIZE,
                        NULL,
                        mainBUTTON_CHECK_TASK_PRIORITY,	
                        NULL);

        xTaskCreate(prvUsart1Task,
                        ( char *) "USART1",
                        configMINIMAL_STACK_SIZE,
                        NULL,
                        mainUSART1_TASK_PRIORITY,
                        NULL);

	xTaskCreate(prvLCDTask,
			( char *) "LCD",
			configMINIMAL_STACK_SIZE,
			NULL,
			mainLCD_TASK_PRIORITY,
			NULL);		

	xTaskCreate(prvPB13_ToggleTask,
                        ( char *) "Port Test",
                        configMINIMAL_STACK_SIZE,
                        NULL,
                        mainPORTst_TASK_PRIORITY,	
                        NULL);
			
        /* 启动任务调度器 */
        vTaskStartScheduler();

        /* 在调度器正常运行时，程序不会执行到此处。如果程序运行到这里，则说明
           FreeRTOS分配给空闲任务和定时器任务的堆空间不足 */
        for( ;; );
}
