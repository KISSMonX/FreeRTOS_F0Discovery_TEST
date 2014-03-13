//=========================================================================================================
// 
//=========================================================================================================

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f0xx.h"              
#include "lcd.h"

//=========================================================================================================
static void prvSetupHardware(void);	// Ó²¼ş³õÊ¼»¯º¯Êı
extern void prvUserTasks(void);		

//=========================================================================================================
// Ö÷º¯Êı
//=========================================================================================================
int main(void)
{
	// ³õÊ¼»¯STM32F0XX Discovery ËùĞèÒªµÄÓ²¼şÏµÍ³
	prvSetupHardware();		

	// ´´½¨²¢ÔËĞĞÓÃ»§ÈÎÎñ 
	prvUserTasks();			
	
	return 0;
}

//=========================================================================================================
// Ó²¼ş³õÊ¼»¯
//=========================================================================================================
static void prvSetupHardware( void )
{
	// ÄÚ²¿ÍâÉè½á¹¹Ìå±äÁ¿
	GPIO_InitTypeDef  	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	SPI_InitTypeDef 	SPI_InitStructure;
	DMA_InitTypeDef 	DMA_InitStructure;	
	

	//ÍâÉèÊ±ÖÓÅäÖÃ---ENABLE
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	// SPI2Ä£¿éÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // USART1Ä£¿éÊ±ÖÓ	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | 		// DMA1Ä£¿éÊ±ÖÓ
				RCC_AHBPeriph_GPIOA |		// GPIOAÄ£¿éÊ±ÖÓ
				RCC_AHBPeriph_GPIOB | 		// GPIOBÄ£¿éÊ±ÖÓ
				RCC_AHBPeriph_GPIOC, ENABLE);	// GPIOCÄ£¿éÊ±ÖÓ											

	/*
	 * LED GPIO³õÊ¼»¯
	 * GPIOC£ºPC9 ¿ØÖÆLED3, PC8 ¿ØÖÆ LED4
	 */	 
	GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_8 | GPIO_Pin_9);	 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;		// Êä³öÄ£Ê½
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	// ½ûÖ¹ÉÏÏÂÀ­
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// IO ¿ÚËÙ¶È
	GPIO_Init(GPIOC, &GPIO_InitStructure);			// Ğ´ÈëÅäÖÃ
	
	/*
	 * ²âÊÔÓÃ IO ¿Ú³õÊ¼»¯
	 * GPIOB.13 ÓÃÓÚ²âÊÔ IO ¿ÚËÙ¶ÈºÍ CPU ËÙ¶È
	 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	 
	 
	/* 
	 * °´¼ü³õÊ¼»¯
	 * GPIOA£ºPA0ÓÃÓÚ¿ØÖÆ°´¼ü
	 */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	/* 
	 * USART1³õÊ¼»¯£ºUSART1+DMA´«Êä·½Ê½
	 */
	 
	/* GPIO³õÊ¼»¯£ºGPIOA£ºPA8ÓÃÓÚUSART1_Tx£»PA9ÓÃÓÚUSART1_Rx */	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);		// PA9-USART1_Tx 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);		// PA10-USART1_Rx 

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;	// PA9 ¶Ë¿ÚÎªUSART1_Tx
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;			// Ê¹ÓÃ±¸ÓÃ¹¦ÄÜ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// ¶Ë¿ÚËÙ¶È50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;			// ¿ªÆôÉÏÀ­µç×è
	GPIO_Init(GPIOA, &GPIO_InitStructure);				// ³õÊ¼»¯GPIO	
	
	/* USART1Ä£¿é³õÊ¼»¯ */
	USART_InitStructure.USART_BaudRate   = 9600;			// ²¨ÌØÂÊ9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	// 8Î»Êı¾İ
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;	// 1Î»Í£Ö¹Î»
	USART_InitStructure.USART_Parity     = USART_Parity_No;		// ÎŞĞ£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎŞÓ²¼şÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;//ÆôÓÃ·¢ËÍÓë½ÓÊÕ
	USART_Init(USART1, &USART_InitStructure);			// ³õÊ¼»¯USART1Ä£¿é
	
	USART_Cmd(USART1, ENABLE);					// Ê¹ÄÜUSART1Ä£¿é
	USART_DMACmd(USART1, (USART_DMAReq_Tx | USART_DMAReq_Rx), ENABLE);// Ê¹ÄÜDMA´«Êä
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);			// ½ûÓÃUSART1·¢ËÍÖĞ¶Ï
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			// Ê¹ÄÜUSART1½ÓÊÕÖĞ¶Ï	
	
	/*
	 * LCD5110³õÊ¼»¯£ºLCD5110²ÉÓÃSPI+DMA´«Êä·½Ê½
	 */
	 
	/* SPI2Ä£¿éGPIO³õÊ¼»¯£ºGPIOB£ºPB3ÓÃÓÚSCLK£»PB15ÓÃÓÚMOSI */	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);	// PB13-SPI SCLK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);	// PB15-SPI MOSI
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_15;// PB13ºÍPB15ÓÃÓÚSPIÍ¨Ñ¶
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		// Ñ¡ÓÃ¶Ë¿ÚµÄ±¸ÓÃ(SPI Port)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// ¶Ë¿ÚËÙ¶È50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;		// ¿ªÆôÉÏÀ­µç×è
	GPIO_Init(GPIOB, &GPIO_InitStructure);			// Ğ´Èë¶Ë¿Ú B ÅäÖÃĞÅÏ¢
	
	/* ÅäÖÃSPI2Ä£¿é */
	SPI_I2S_DeInit(SPI2);					// »Ö¸´SPI2Ä£¿éµÄÄ¬ÈÏÉèÖÃ		
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		// SPI2Ä£¿éÎªÖ÷Éè±¸
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;// Ö»·¢Ä£Ê½
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	// Êı¾İ¿í¶ÈÎª8Î»
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		// ¿ÕÏĞ×´Ì¬Ê±£¬SCK ±£³ÖµÍµçÆ½
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		// µÚÒ»¸öÊ±ÖÓÑØ¶Ô×¼µÚÒ»Î»Êı¾İ
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	// ÏÈ·¢ËÍMSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;		// CRC ¼ÆËã¶àÏîÊ½
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		// ÆôÓÃÈí¼ş´ÓÉè±¸¹ÜÀí
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;// ²¨ÌØÂÊÎªfPCLK/8	
	SPI_Init(SPI2, &SPI_InitStructure);			// ³õÊ¼»¯SPI2Ä£¿é

	/* ÅäÖÃ DMA1 Ä£¿éÍ¨µÀ 5 ÓÃÓÚ SPI ·¢ËÍ */
	DMA_StructInit(&DMA_InitStructure);			// ÒÔÄ¬ÈÏÉèÖÃ³õÊ¼»¯DMAÄ£¿é
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;	// ´æ´¢Æ÷µØÖ·
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR);//ÍâÉèµØÖ·
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	// ´Ó´æ´¢Æ÷¶Álcd_buffer -> SPI2->DR
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	// ÄÚ´æµØÖ·µİÔö
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//Êı¾İÄ¿µÄ¹Ì¶¨ÎªSPI2->DR
	DMA_InitStructure.DMA_BufferSize = 0;			// DMAµÄÊı¾İ³ß´ç
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//ÍâÉèÊı¾İ¿í¶È8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//ÄÚ´æÊı¾İ¿í¶È8bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		// ÆÕÍ¨Ä£Ê½£¨·ÇÑ­»·£©
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;	// µÍÓÅÏÈ¼¶
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		// ¹Ø±ÕÄÚ´æµ½ÄÚ´æÄ£Ê½
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 		// ³õÊ¼»¯DMA¿ØÖÆÆ÷

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);	// Ê¹ÄÜSPI+DMA´«Êä
																								 //ÓÉÓÚDMA»º³åÇøÈ±Ê¡´óĞ¡Îª0£¬ËùÒÔ¸ÃÓï¾ä²¢²»»á´¥·¢Êı¾İ´«Êä
																								 //µ±ÔÚ³ÌĞòÖĞĞŞ¸Ä»º³åÇø´óĞ¡Ê±½«»á´¥·¢Êı¾İ´«Êä	
	SPI_Cmd(SPI2, ENABLE);		// Ê¹ÄÜSPIÄ£¿é	
	DMA_Cmd(DMA1_Channel5, ENABLE);	// Ê¹ÄÜDMAÄ£¿é

	/* ³õÊ¼»¯LCD¿ØÖÆ¶Ë¿Ú¡£PC0 -> BL£»PC1 -> DC£»PC2 -> CE£»PC3 -> RST£»*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;//Ñ¡ÓÃPC0~3¶Ë¿Ú
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// ¶Ë¿ÚÉèÖÃÎªÊä³öÄ£Ê½
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// ÍÆÍìÊä³öÄ£Ê½
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// ¹Ø±ÕÉÏÀ­µç×è		
	GPIO_Init(GPIOC, &GPIO_InitStructure);			// ³õÊ¼»¯GPIO	
	
	// LCD³õÊ¼»¯
	LCD_Init();
	
	// ÏµÍ³ÖĞ¶Ï³õÊ¼»¯ -> ÏµÍ³ËùÓÃÖĞ¶Ï -> USART1ÖĞ¶Ï
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}


//=========================================================================================================
// 
//=========================================================================================================
void vApplicationMallocFailedHook(void)
{
	/* 
	 ÄÚ´æ·ÖÅäÊ§°Ü¹³×Óº¯Êı
	 FreeRTOSÄÚºËÔÚ´´½¨ÈÎÎñ¡¢¶ÓÁĞ¡¢¶¨Ê±Æ÷»òÕßĞÅºÅÊ±»áµ÷ÓÃpvPortMalloc() º¯Êı¡£
	 Ò»µ©pvPortMalloc() º¯Êıµ÷ÓÃÊ§°Ü£¬²¢ÇÒÔÚÎÄ¼şFreeRTOSConfig.hÖĞ£¬
	 configUSE_MALLOC_FAILED_HOOKÉèÖÃÎª1Ê±»áµ÷ÓÃ¸Ãº¯Êı¡£
	 ¸Ãº¯ÊıÒ²¿ÉÒÔÓÉÓ¦ÓÃ³ÌĞòµ÷ÓÃ¡£
	 Èç¹ûÓ¦ÓÃ³ÌĞòÖĞ°üº¬ÁËÎÄ¼şheap_1.c »òÕß heap_2.c£¬pvPortMalloc() ¿ÉÊ¹ÓÃµÄ¶Ñ´óĞ¡
	 ÓÉÎÄ¼şFreeRTOSConfig.hÖĞµÄconfigTOTAL_HEAP_SIZE¶¨Òå¡£
	 ÀûÓÃAPIº¯ÊıxPortGetFreeHeapSize()¿ÉÒÔ·µ»Ø¿ÕÏĞµÄ¶Ñ¿Õ¼ä´óĞ¡¡£
	*/
	taskDISABLE_INTERRUPTS();//½ûÓÃËùÓĞ¿ÉÆÁ±ÎµÄÖĞ¶Ï
	for( ;; );
}

//=========================================================================================================
//=========================================================================================================
void vApplicationIdleHook(void)
{
	/* 
	 ¿ÕÏĞÈÎÎñ¹³×Óº¯Êı
	 µ±ÎÄ¼şFreeRTOSConfig.hÖĞ configUSE_IDLE_HOOK ÉèÖÃÎª1Ê±£¬ÏµÍ³¿ÕÏĞÈÎÎñµÄ
	 Ã¿´ÎÑ­»·ÖĞ»áµ÷ÓÃ¸Ãº¯Êı¡£¸Ãº¯ÊıÖĞ²»ÄÜÓĞ×èÈûÓ¦ÓÃ³ÌĞòÖ´ĞĞµÄ´úÂë¡£
	 Èç¹ûÓ¦ÓÃ³ÌĞòµ÷ÓÃvTaskDelete() APIº¯ÊıÊ±£¬¸Ã¹³×Óº¯ÊıÓ¦ÄÜ¹»·µ»Øµ÷ÓÃº¯Êı¡£
	 ÕâÊ±ÒòÎª¿ÕÏĞÈÎÎñÒ»°ãÓÃÀ´ÇåÀíÄÚºË·ÖÅä¸øÒÑÉ¾³ıÈÎÎñµÄ´æ´¢¿Õ¼ä¡£
	*/
}

//=========================================================================================================
//=========================================================================================================
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* 
	 Ó¦ÓÃ³ÌĞò¶ÑÕ»Òç³ö¹³×Óº¯Êı
	 µ±ÎÄ¼şFreeRTOSConfig.hÖĞ configCHECK_FOR_STACK_OVERFLOWÉèÖÃÎª1»ò2Ê±½«Ö´ĞĞ
	 ÔËĞĞÊ±¶ÑÕ»Òç³ö¼ì²é¡£µ±¼ì²âµ½¶ÑÕ»Òç³öÊ±»áµ÷ÓÃ¸Ã¹³×Óº¯ÊıR 
	*/ 
	taskDISABLE_INTERRUPTS();//½ûÓÃËùÓĞ¿ÉÆÁ±ÎÖĞ¶Ï
	for( ;; );
}


//=========================================================================================================
// This function will be called by each tick interrupt if
// configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
// added here, but the tick hook is called from an interrupt context, so
// code must not attempt to block, and only the interrupt safe FreeRTOS API
// functions can be used (those that end in FromISR()).
//=========================================================================================================
void vApplicationTickHook(void)
{
	/* 
	 Ó¦ÓÃ³ÌĞòtick¹³×Óº¯Êıµ±ÎÄ¼şFreeRTOSConfig.hÖĞ configUSE_TICK_HOOKÉèÖÃÎª1Ê±£¬¸Ãº¯Êı½
	ÔÚtickÖĞ¶ÏÖĞ±»µ÷ÓÃ¡£¸Ã¹³×Óº¯ÊıÖĞ¿ÉÒÔÌí¼ÓÓÃ»§³ÌĞò£¬µ«ÊÇÓÉÓÚ¸Ãº¯ÊıÔÚÖĞ¶Ï·şÎñ³ÌĞòÖĞµ÷ÓÃ£
	ÓÃ»§´úÂë²»ÄÜ×èÈû³ÌĞòÖ´ĞĞ£¬¶øÇÒÖ»ÄÜÊ¹ÓÃÖĞ¶Ï°²È«µÄAPIº¯Êı¡£
	*/
}


//=========================================================================================================
//=========================================================================================================
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




