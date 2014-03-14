/*-----------------------------------------------------------
 *����5������
 *1. LED3��˸����prvLED3BlinkTask
 *2. �����������prvButtonCheckTask
 *3. USART1ͨѶ����prvUsart1Task
 *4. LCD5110��ʾ����prvLCDTask
 *5. ���� IO �ڷ�ת�ٶȼ��
 *-----------------------------------------------------------*/

/* ��׼ͷ�ļ� */	
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
static xSemaphoreHandle xButtonSpeedUpSemaphore;	// �����ź�
xQueueHandle RxQueue, TxQueue;				// ���пڷ���/���ն��� 
uint8_t u8LCDFrameBuffer[LCD_X*LCD_Y/8];		// LCD ��ʾ������

//=========================================================================================================
/**
 * @brief  LED3��˸����ÿ��500ms�л�LED3����ʾ״̬.
 * @param  pvParameters:����Ĭ�ϲ���.
 * @retval ��
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
 * @brief  ���� IO �ڷ�ת�ٶ��� CPU ����Ƶ��
 * @param  pvParameters:����Ĭ�ϲ���.
 * @retval ��
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
 * @brief  �����������ÿ��20ms��ⰴ��״̬������а����¼����ͷ��ź�.
 * @param  pvParameters:����Ĭ�ϲ���.
 * @retval ��
 */
//=========================================================================================================
static void prvButtonCheckTask(void *pvParameters)
{	
        static uint8_t bounce_count;
        portTickType xNextWakeTime;
        const portTickType xFrequency = 2;
        xNextWakeTime = xTaskGetTickCount();

        /* �����ź� */
        vSemaphoreCreateBinary(xButtonSpeedUpSemaphore);	

        /* ��������źųɹ������źų�ʼ��Ϊ0 */	
        if (xButtonSpeedUpSemaphore != NULL) {
                xSemaphoreTake(xButtonSpeedUpSemaphore, (portTickType)0);
        }	

        for(;;) {
                /* ��ȡ����״̬ */
                if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == pdTRUE)//�����⵽��������
                {
                        bounce_count++;
                        if (bounce_count == DEBOUNCECOUNTS)//��������
                        {	
				GPIOC->ODR ^= GPIO_Pin_8;
                                xSemaphoreGive(xButtonSpeedUpSemaphore);//�ͷŰ����ź� 
                        }
                }
                /* ÿ��Nms���һ�� */ 
                vTaskDelayUntil(&xNextWakeTime,xFrequency);
        }
}


//=========================================================================================================
/**
 * @brief  USART1ͨѶ���񣺷����ַ�������PC���������ַ�����PC.
 * @param  pvParameters:����Ĭ�ϲ���.
 * @retval ��
 */
//=========================================================================================================
static void prvUsart1Task(void *pvParameters)
{
        char ch;
        portTickType xLastWakeTime;   
        const portTickType xFrequency = 50;   
        xLastWakeTime = xTaskGetTickCount();   

        /* ����������128���ַ��Ķ��� */
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
 * @brief  ����з����ַ�.
 * @param  ch:�����͵��ַ�.
 * @retval ��
 */
//=========================================================================================================
uint32_t Usart1PutChar(char ch)
{
        if(xQueueSend(TxQueue, &ch, 10) == pdPASS)//�����ַ��ɹ�
        {
                USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//ʹ�ܷ����жϣ������ͼĴ���Ϊ��ʱ�������ж�
                return pdTRUE;
        }
        else {
                return pdFAIL;
        }
}


//=========================================================================================================
/**
 * @brief  �Ӷ��ж�ȡ�ַ�.
 * @param  ch:�Ӷ��ж�ȡ���ַ�.
 * @retval ��
 */
//=========================================================================================================
uint32_t Usart1GetChar(char *ch)
{
        if(xQueueReceive(RxQueue, ch, 0) == pdPASS)//��ȡ�ַ��ɹ�
        {
                return pdTRUE;
        }
        return pdFALSE;
}


//=========================================================================================================
/**
 * @brief  ����з����ַ�������.
 * @param  pcString: ��д����ַ�������ָ��.
 * @param  u32StringLength: ��д���ַ������鳤��.
 * @retval ��
 */
//=========================================================================================================
void  USART1PutString(const char *const pcString, uint32_t u32StringLength)
{
        uint32_t i;
        for(i = 0; i < u32StringLength; i++)		
        {
                if(xQueueSend(TxQueue, &(pcString[i]), serPUT_STRING_CHAR_DELAY) != pdPASS)
                {					
                        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//������װ����ʹ�ܷ����жϷ�������.
                        vTaskDelay(serPUT_STRING_CHAR_DELAY);						
                        continue;
                }
        }	
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//ʹ�ܷ����ж�
}


//=========================================================================================================
/**
  * @brief  LCD����ÿ��500ms��������ʾ�̶��ַ�����Hello World����.
  * @param  pvParameters:����Ĭ�ϲ���.
  * @retval ��
  */
//=========================================================================================================
static void prvLCDTask(void *pvParameters)
{
	uint8_t lcd_row = 0;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 500;
	xLastWakeTime=xTaskGetTickCount();
	/* ��ʼ��LCD */
	LCD_Init();
	/* �����ʾ������ */
	LCD_Clear();
	/* ������д����ʾ������ */
	LCD_WriteString(0, 0, "Hello World!");
	/* ˢ��LCD */
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
 * @brief  ������������.
 * @param  ��
 * @retval ��
 */
//=========================================================================================================
void prvUserTasks(void)
{
        /* �������� */
        xTaskCreate(prvLED3BlinkTask,			//������
                        ( char *) "LED3 BLINK",   	//��������
                        configMINIMAL_STACK_SIZE, 	//�����ջ
                        NULL, 				//�������
                        mainLED3_BLINK_TASK_PRIORITY,	//�������ȼ� 
                        NULL);				//������
			
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
			
        /* ������������� */
        vTaskStartScheduler();

        /* �ڵ�������������ʱ�����򲻻�ִ�е��˴�������������е������˵��
           FreeRTOS�������������Ͷ�ʱ������Ķѿռ䲻�� */
        for( ;; );
}
