#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f0xx.h"                  // Device header


static void prvSetupHardware(void);//硬件初始化函数
extern void prvUserTasks(void);//LED闪烁任务文件

/*-----------------------------------------------------------*/

int main(void)
{
	/* 初始化STM32F0XX Discovery硬件系统 */
	prvSetupHardware();
	/* 创建并运行用户任务 */
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
	 * 系统时钟初始化 
	 * 系统所使用的模块：
	 * GPIOA
	 * GPIOB
	 * GPIOC
	 * USART1
	 * SPI2
	 * DMA1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	//SPI2模块时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //USART1模块时钟	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | 		//DMA1模块时钟
				RCC_AHBPeriph_GPIOA |		//GPIOA模块时钟
				RCC_AHBPeriph_GPIOB | 		//GPIOB模块时钟
				RCC_AHBPeriph_GPIOC, ENABLE);	//GPIOC模块时钟											

	/*
	 * LED GPIO初始化
	 * GPIOC：PC9控制LED3
	 */	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//PC9用于LED控制	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* 
	 * 按键初始化
	 * GPIOA：PA0用于控制按键
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0用于按键
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	/* 
	 * USART1初始化：USART1+DMA传输方式
	 */
	 
	/* GPIO初始化：GPIOA：PA8用于USART1_Tx；PA9用于USART1_Rx */	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//PA9-USART1_Tx 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//PA10-USART1_Rx 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//PA9端口为USART1_Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//使用备用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//端口速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//开启上拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO	
	
	/* USART1模块初始化 */
	USART_InitStructure.USART_BaudRate = 9600;//波特率9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//启用发送与接收
	USART_Init(USART1, &USART_InitStructure);//初始化USART1模块
	USART_Cmd(USART1, ENABLE);//使能USART1模块
	USART_DMACmd(USART1, (USART_DMAReq_Tx | USART_DMAReq_Rx), ENABLE);//使能DMA传输
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//禁用USART1发送中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能USART1接收中断	
	
	/*
	 * LCD5110初始化：LCD5110采用SPI+DMA传输方式
	 */
	 
	/* SPI2模块GPIO初始化：GPIOB：PB3用于SCLK；PB15用于MOSI */	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);//PB13-SPI SCLK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);//PB15-SPI MOSI
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;//PB13和PB15用于SPI通讯
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//选用端口的备用(SPI Port)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//端口速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//开启上拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	/* 配置SPI2模块 */
	SPI_I2S_DeInit(SPI2);//恢复SPI2模块的默认设置		
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//SPI2模块为主设备
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//只发模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//数据宽度为8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//空闲状态时，SCK 保持低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//第一个时钟沿对准第一位数据
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//先发送MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC 计算多项式
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//启用软件从设备管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//波特率为fPCLK/8	
	SPI_Init(SPI2, &SPI_InitStructure);//初始化SPI2模块

	/* 配置DMA1模块通道5用于SPI发送 */
	DMA_StructInit(&DMA_InitStructure);//以默认设置初始化DMA模块
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;//存储器地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR);//外设地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//从存储器读lcd_buffer -> SPI2->DR
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//数据目的固定为SPI2->DR
	DMA_InitStructure.DMA_BufferSize = 0;//DMA的数据尺寸
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据宽度8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据宽度8bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//普通模式（非循环）
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;//低优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//关闭内存到内存模式
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); //初始化DMA控制器

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);//使能SPI+DMA传输
																								 //由于DMA缓冲区缺省大小为0，所以该语句并不会触发数据传输
																								 //当在程序中修改缓冲区大小时将会触发数据传输	
	SPI_Cmd(SPI2, ENABLE);//使能SPI模块	
	DMA_Cmd(DMA1_Channel5, ENABLE);//使能DMA模块

	/* 初始化LCD控制端口。PC0 -> BL；PC1 -> DC；PC2 -> CE；PC3 -> RST；*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;//选用PC0~3端口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//端口设置为输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//关闭上拉电阻		
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO	
	
//	/* LCD初始化 */
//	LCD_Init();
	
	/* 
	 * 系统中断初始化 
	 * 系统所用中断：
	 * USART1中断
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
	 内存分配失败钩子函数
	 FreeRTOS内核在创建任务、队列、定时器或者信号时会调用pvPortMalloc() 函数。
	 一旦pvPortMalloc() 函数调用失败，并且在文件FreeRTOSConfig.h中，
	 configUSE_MALLOC_FAILED_HOOK设置为1时会调用该函数。
	 该函数也可以由应用程序调用。
	 如果应用程序中包含了文件heap_1.c 或者 heap_2.c，pvPortMalloc() 可使用的堆大小
	 由文件FreeRTOSConfig.h中的configTOTAL_HEAP_SIZE定义。
	 利用API函数xPortGetFreeHeapSize()可以返回空闲的堆空间大小。
	*/
	taskDISABLE_INTERRUPTS();//禁用所有可屏蔽的中断
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* 
	 空闲任务钩子函数
	 当文件FreeRTOSConfig.h中 configUSE_IDLE_HOOK 设置为1时，系统空闲任务的
	 每次循环中会调用该函数。该函数中不能有阻塞应用程序执行的代码。
	 如果应用程序调用vTaskDelete() API函数时，该钩子函数应能够返回调用函数。
	 这时因为空闲任务一般用来清理内核分配给已删除任务的存储空间。
	*/
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* 
	 应用程序堆栈溢出钩子函数
	 当文件FreeRTOSConfig.h中 configCHECK_FOR_STACK_OVERFLOW设置为1或2时将执行
	 运行时堆栈溢出检查。当检测到堆栈溢出时会调用该钩子函数R 
	*/ 
	taskDISABLE_INTERRUPTS();//禁用所有可屏蔽中断
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* 
	 应用程序tick钩子函数
	当文件FreeRTOSConfig.h中 configUSE_TICK_HOOK设置为1时，该函数将在tick中断中被调用。
	该钩子函数中可以添加用户程序，但是由于该函数在中断服务程序中调用，用户代码不能阻塞程序
	执行，而且只能使用中断安全的API函数。
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




