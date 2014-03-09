#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f0xx.h"                  // Device header


static void prvSetupHardware(void);//Ӳ����ʼ������
extern void prvUserTasks(void);//LED��˸�����ļ�

/*-----------------------------------------------------------*/

int main(void)
{
	/* ��ʼ��STM32F0XX DiscoveryӲ��ϵͳ */
	prvSetupHardware();
	/* �����������û����� */
	prvUserTasks();
	return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	GPIO_InitTypeDef  	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	SPI_InitTypeDef 	SPI_InitStructure;
	DMA_InitTypeDef 	DMA_InitStructure;	
	
	/*
	 * ϵͳʱ�ӳ�ʼ�� 
	 * ϵͳ��ʹ�õ�ģ�飺
	 * GPIOA
	 * GPIOB
	 * GPIOC
	 * USART1
	 * SPI2
	 * DMA1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	//SPI2ģ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //USART1ģ��ʱ��	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | 		//DMA1ģ��ʱ��
				RCC_AHBPeriph_GPIOA |		//GPIOAģ��ʱ��
				RCC_AHBPeriph_GPIOB | 		//GPIOBģ��ʱ��
				RCC_AHBPeriph_GPIOC, ENABLE);	//GPIOCģ��ʱ��											

	/*
	 * LED GPIO��ʼ��
	 * GPIOC��PC9����LED3
	 */	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//PC9����LED����	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* 
	 * ������ʼ��
	 * GPIOA��PA0���ڿ��ư���
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0���ڰ���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	/* 
	 * USART1��ʼ����USART1+DMA���䷽ʽ
	 */
	 
	/* GPIO��ʼ����GPIOA��PA8����USART1_Tx��PA9����USART1_Rx */	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//PA9-USART1_Tx 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//PA10-USART1_Rx 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//PA9�˿�ΪUSART1_Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//ʹ�ñ��ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�˿��ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO	
	
	/* USART1ģ���ʼ�� */
	USART_InitStructure.USART_BaudRate = 9600;//������9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//���÷��������
	USART_Init(USART1, &USART_InitStructure);//��ʼ��USART1ģ��
	USART_Cmd(USART1, ENABLE);//ʹ��USART1ģ��
	USART_DMACmd(USART1, (USART_DMAReq_Tx | USART_DMAReq_Rx), ENABLE);//ʹ��DMA����
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//����USART1�����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ʹ��USART1�����ж�	
	
	/*
	 * LCD5110��ʼ����LCD5110����SPI+DMA���䷽ʽ
	 */
	 
	/* SPI2ģ��GPIO��ʼ����GPIOB��PB3����SCLK��PB15����MOSI */	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);//PB13-SPI SCLK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);//PB15-SPI MOSI
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;//PB13��PB15����SPIͨѶ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//ѡ�ö˿ڵı���(SPI Port)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�˿��ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//������������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	/* ����SPI2ģ�� */
	SPI_I2S_DeInit(SPI2);//�ָ�SPI2ģ���Ĭ������		
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//SPI2ģ��Ϊ���豸
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//ֻ��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//���ݿ��Ϊ8λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//����״̬ʱ��SCK ���ֵ͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//��һ��ʱ���ض�׼��һλ����
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//�ȷ���MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC �������ʽ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//����������豸����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//������ΪfPCLK/8	
	SPI_Init(SPI2, &SPI_InitStructure);//��ʼ��SPI2ģ��

	/* ����DMA1ģ��ͨ��5����SPI���� */
	DMA_StructInit(&DMA_InitStructure);//��Ĭ�����ó�ʼ��DMAģ��
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;//�洢����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR);//�����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//�Ӵ洢����lcd_buffer -> SPI2->DR
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//����Ŀ�Ĺ̶�ΪSPI2->DR
	DMA_InitStructure.DMA_BufferSize = 0;//DMA�����ݳߴ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݿ��8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�ڴ����ݿ��8bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//��ͨģʽ����ѭ����
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;//�����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//�ر��ڴ浽�ڴ�ģʽ
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); //��ʼ��DMA������

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);//ʹ��SPI+DMA����
																								 //����DMA������ȱʡ��СΪ0�����Ը���䲢���ᴥ�����ݴ���
																								 //���ڳ������޸Ļ�������Сʱ���ᴥ�����ݴ���	
	SPI_Cmd(SPI2, ENABLE);//ʹ��SPIģ��	
	DMA_Cmd(DMA1_Channel5, ENABLE);//ʹ��DMAģ��

	/* ��ʼ��LCD���ƶ˿ڡ�PC0 -> BL��PC1 -> DC��PC2 -> CE��PC3 -> RST��*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;//ѡ��PC0~3�˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//�˿�����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�ر���������		
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO	
	
//	/* LCD��ʼ�� */
//	LCD_Init();
	
	/* 
	 * ϵͳ�жϳ�ʼ�� 
	 * ϵͳ�����жϣ�
	 * USART1�ж�
	 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* 
	 �ڴ����ʧ�ܹ��Ӻ���
	 FreeRTOS�ں��ڴ������񡢶��С���ʱ�������ź�ʱ�����pvPortMalloc() ������
	 һ��pvPortMalloc() ��������ʧ�ܣ��������ļ�FreeRTOSConfig.h�У�
	 configUSE_MALLOC_FAILED_HOOK����Ϊ1ʱ����øú�����
	 �ú���Ҳ������Ӧ�ó�����á�
	 ���Ӧ�ó����а������ļ�heap_1.c ���� heap_2.c��pvPortMalloc() ��ʹ�õĶѴ�С
	 ���ļ�FreeRTOSConfig.h�е�configTOTAL_HEAP_SIZE���塣
	 ����API����xPortGetFreeHeapSize()���Է��ؿ��еĶѿռ��С��
	*/
	taskDISABLE_INTERRUPTS();//�������п����ε��ж�
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* 
	 ���������Ӻ���
	 ���ļ�FreeRTOSConfig.h�� configUSE_IDLE_HOOK ����Ϊ1ʱ��ϵͳ���������
	 ÿ��ѭ���л���øú������ú����в���������Ӧ�ó���ִ�еĴ��롣
	 ���Ӧ�ó������vTaskDelete() API����ʱ���ù��Ӻ���Ӧ�ܹ����ص��ú�����
	 ��ʱ��Ϊ��������һ�����������ں˷������ɾ������Ĵ洢�ռ䡣
	*/
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* 
	 Ӧ�ó����ջ������Ӻ���
	 ���ļ�FreeRTOSConfig.h�� configCHECK_FOR_STACK_OVERFLOW����Ϊ1��2ʱ��ִ��
	 ����ʱ��ջ�����顣����⵽��ջ���ʱ����øù��Ӻ���R 
	*/ 
	taskDISABLE_INTERRUPTS();//�������п������ж�
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* 
	 Ӧ�ó���tick���Ӻ���
	���ļ�FreeRTOSConfig.h�� configUSE_TICK_HOOK����Ϊ1ʱ���ú�������tick�ж��б����á�
	�ù��Ӻ����п�������û����򣬵������ڸú������жϷ�������е��ã��û����벻����������
	ִ�У�����ֻ��ʹ���жϰ�ȫ��API������
	*/
}
/*-----------------------------------------------------------*/

#ifdef JUST_AN_EXAMPLE_ISR

void Dummy_IRQHandler(void)
{
long lHigherPriorityTaskWoken = pdFALSE;

	/* Clear the interrupt if necessary. */
	Dummy_ClearITPendingBit();

	/* This interrupt does nothing more than demonstrate how to synchronise a
	task with an interrupt.  A semaphore is used for this purpose.  Note
	lHigherPriorityTaskWoken is initialised to zero. Only FreeRTOS API functions
	that end in "FromISR" can be called from an ISR. */
	xSemaphoreGiveFromISR( xTestSemaphore, &lHigherPriorityTaskWoken );

	/* If there was a task that was blocked on the semaphore, and giving the
	semaphore caused the task to unblock, and the unblocked task has a priority
	higher than the current Running state task (the task that this interrupt
	interrupted), then lHigherPriorityTaskWoken will have been set to pdTRUE
	internally within xSemaphoreGiveFromISR().  Passing pdTRUE into the
	portEND_SWITCHING_ISR() macro will result in a context switch being pended to
	ensure this interrupt returns directly to the unblocked, higher priority,
	task.  Passing pdFALSE into portEND_SWITCHING_ISR() has no effect. */
	portEND_SWITCHING_ISR( lHigherPriorityTaskWoken );
}

#endif /* JUST_AN_EXAMPLE_ISR */




