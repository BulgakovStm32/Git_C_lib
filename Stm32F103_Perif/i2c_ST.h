/*
 * i2c_ST.h
 *
 *  Created on: 20 дек. 2020 г.
 *      Author: Беляев А.А.
 *
 *   Изменения: 13 марта 2023 года
 */

#ifndef I2C_ST_H
#define I2C_ST_H
//*******************************************************************************************
//*******************************************************************************************

#include "stm32f103xb.h"

//*******************************************************************************************
//*******************************************************************************************
//Константы для настройки скорости работы I2C
#define APB1_CLK			36000000U 				//Частота шины APB1 в Гц
#define I2C_FREQ	    	(APB1_CLK / 1000000U)	//Peripheral clock frequency (MHz)

//Sm mode or SMBus:
//TPCLK1 = 27,7777 ns
//CCR    = 1000nS/ (2 * TPCLK1)
//TRISE  = (1000nS/TPCLK1)
#define I2C_SM_CCR			180 //(10000U / (2 * TPCLK1))
#define I2C_SM_TRISE		36  //(1000U  / TPCLK1)

//Fm mode:
//TPCLK1 = 27,7777 ns
//CCR    = 2500nS/ (3 * TPCLK1)
//TRISE  = (300nS/TPCLK1)
#define I2C_FM_CCR			30 //(2500U / (3 * TPCLK1))
#define I2C_FM_TRISE		12 //(300U  / TPCLK1)
//--------------------------
//Таймаут ожидания сброса флага регистра SR1.
//Меньше 70 не делать, т.е. пауза будет меньше чем время передачи байта адреса
#define I2C_WAIT_TIMEOUT_SR1	100//1000//5000//50000U

//Таймаут ожидания сброса флага регистра CR1.
#define I2C_WAIT_TIMEOUT_CR1	2000
//--------------------------
#define I2C_MODE_MASTER		0	//режим Master.
#define I2C_MODE_SLAVE		1	//режим Slave

#define I2C_GPIO_NOREMAP	0	//
#define I2C_GPIO_REMAP		1	//Ремап выводов для I2C1, для I2C2 ремапа нет.
//--------------------------
#define I2C_READ  			1	//
#define I2C_WRITE			0	//
#define I2C_ADDRESS(addr, mode) ((addr<<1) | mode)
//--------------------------
//Состояние I2C при работе в блокирующем режиме (поллинг)
typedef enum{
	I2C_OK = 0,
	I2C_ERR_START,	//Ошибка при фоормировании Старт-последовательности
	I2C_ERR_NAC,	//Отсутствие Slave на шине
	//I2C_BUSY,		//Шина I2C занята (передача/прием данных)
	//I2C_ERR_TX_BYTE,//Вышел таймаут передачи байта.
	//I2C_ERR_RX_BYTE,//Вышел таймаут приема байта.
	//I2C_ERR_BTF	  //Вышел таймаут Byte transfer finished
}I2C_State_t;
//*******************************************************************************************
//*******************************************************************************************
//Общие функции.
I2C_State_t I2C_StartAndSendDeviceAddr(I2C_TypeDef *i2c, uint8_t deviceAddr);
void		I2C_Stop(I2C_TypeDef *i2c);
I2C_State_t I2C_ReadData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len);
I2C_State_t I2C_SendByte(I2C_TypeDef *i2c, uint8_t byte);
I2C_State_t I2C_SendDataWithStop(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len);
I2C_State_t I2C_SendDataWithoutStop(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len);

//************************************
//Функции для работы в режиме Master
void 		I2C_Master_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t speed);
uint32_t 	I2C_Master_GetNacCount(I2C_TypeDef *i2c);
I2C_State_t I2C_Master_CheckSlave(I2C_TypeDef *i2c, uint8_t deviceAddr);
I2C_State_t I2C_Master_Write(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint32_t len);
I2C_State_t I2C_Master_Read (I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint32_t len);

//************************************
//Функции для работы в режиме Slave
void I2C_Slave_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t slaveAddr, uint32_t speed);

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//***********************************Работа по прерываниям.**********************************
//Состояние I2C при работе по прерываниям
typedef enum{
  I2C_IT_STATE_RESET = 0,	//I2C не инициализован
  I2C_IT_STATE_READY,		//I2C инициализован и готов к использованию
  I2C_IT_STATE_ADDR_WR,		//Принят адрес+Wr - мастер записывает данные
  I2C_IT_STATE_ADDR_RD,		//Принят адрес+Rd - мастер читает данные
  I2C_IT_STATE_BUSY_TX,		//I2C занят передачей данных
  I2C_IT_STATE_BUSY_RX,		//I2C занят приемом данных
  I2C_IT_STATE_STOP,		//от Мастера принят STOP - признак завершения записи байтов.
  I2C_IT_STATE_NAC,			//от Мастера принят NACK - признак завершения чтения байтов.
}I2C_IT_State_t;
//****************************************************
//События для обработки прерываний. Подросбности в документации на контроллер.
/* --EV1 */
/* 1) Case of One Single Address managed by the slave */
#define  I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED          ((uint32_t)0x00020002) /* BUSY and ADDR flags */
#define  I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED       ((uint32_t)0x00060082) /* TRA, BUSY, TXE and ADDR flags */

/* Slave RECEIVER mode --------------------------*/
/* --EV2 */
#define  I2C_EVENT_SLAVE_BYTE_RECEIVED                     ((uint32_t)0x00020040)  /* BUSY and RXNE flags */
/* --EV4  */
#define  I2C_EVENT_SLAVE_STOP_DETECTED                     ((uint32_t)0x00000010)  /* STOPF flag */

/* Slave TRANSMITTER mode -----------------------*/
/* --EV3 */
#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTED                  ((uint32_t)0x00060084)  /* TRA, BUSY, TXE and BTF flags */
/* --EV3_1 */
#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTING                 ((uint32_t)0x00060080)  /* TRA, BUSY and TXE flags */
/* --EV3_2 */
#define  I2C_EVENT_SLAVE_ACK_FAILURE                       ((uint32_t)0x00000400)  /* AF flag */
//****************************************************
//Структура контекста для работы с портом I2C по прерываниям.
//#pragma pack(push, 1)//размер выравнивания в 1 байт
typedef struct{
	//Конфигурация I2C
	I2C_TypeDef 	*i2c;			//Порт I2C
	uint32_t 	 	i2cMode	 : 1;	//Master или Slave
	uint32_t 		gpioRemap: 1;	//Ремап выводов для I2C1, для I2C2 ремапа нет.
	uint32_t 	 	i2cSpeed : 30;	//Скорость работы I2C
	uint8_t			slaveAddr;		// В режиме Master - адрес Slave-устройства к которому идет обращение,
									// В режиме Slave  - адрес устройста на шине.
	uint8_t 		slaveRegAddr;	// В режиме Master - адрес регистра Slave-устройства куда хотим писать/читать данные.
									// В режиме Slave  - не используется
	//Переменные для работы в прерываниях
	__IO I2C_IT_State_t state;		//Состояние I2C (работа по прерываниям)
	uint32_t			timeOut;	//Таймаут между запросами. Нужно для периициализации I2C в случае зависания.
	uint32_t    		resetCount;	//Счетчик переинициализации i2c. Нужен для отладки.
	//Рабочий буфер приема/передачи
	uint8_t 	 *pBuf;				//Указатель на рабочий буфер приема/передачи
	__IO uint32_t bufCount;			//Счетчик принятых/переданных байтов.
	__IO uint32_t bufSize;			//Размер данных на прием/передачу
}I2C_IT_t;
//#pragma pack(pop)//вернули предыдущую настройку.
//*******************************************************************************************
//*******************************************************************************************
void 	 		I2C_IT_Init(I2C_IT_t *i2c);
void 	 		I2C_IT_DeInit(I2C_IT_t *i2cIt);
void 	 		I2C_IT_SetTxSize(I2C_IT_t *i2cIt, uint32_t size);
void 	 		I2C_IT_SetpBuf(I2C_IT_t *i2cIt, uint8_t *pBuf);
void 	 		I2C_IT_SetDataSize(I2C_IT_t *i2cIt, uint32_t size);
uint32_t 		I2C_IT_GetDataCount(I2C_IT_t *i2cIt);
I2C_IT_State_t 	I2C_IT_GetState(I2C_IT_t *i2cIt);

//Функции обратного вызова
void I2C_IT_SlaveTxCpltCallback(I2C_IT_t *i2cIt);
void I2C_IT_SlaveRxCpltCallback(I2C_IT_t *i2cIt);

//Обработчики прерывания
void I2C_IT_EV_Handler(I2C_TypeDef *i2c);
void I2C_IT_ER_Handler(I2C_TypeDef *i2c);
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************





















//#define I2C_IT_RX_BUF_LEN_MAX		64
//#define I2C_IT_TX_BUF_LEN_MAX		64
//
//#define I2C_IT_RX_BUF_SIZE_DEFAULT	32
//#define I2C_IT_TX_BUF_SIZE_DEFAULT	32
//
//////Константы из STM32F10x_StdPeriph_Driver, файл stm32f10x_i2c.h V3.5.0 11-March-2011
//////I2C Slave Events (Events grouped in order of communication)
/////* --EV1  (all the events below are variants of EV1) */
/////* 1) Case of One Single Address managed by the slave */
////#define  I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED          ((uint32_t)0x00020002) /* BUSY and ADDR flags */
////#define  I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED       ((uint32_t)0x00060082) /* TRA, BUSY, TXE and ADDR flags */
////
/////* 2) Case of Dual address managed by the slave */
////#define  I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED    ((uint32_t)0x00820000)  /* DUALF and BUSY flags */
////#define  I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED ((uint32_t)0x00860080)  /* DUALF, TRA, BUSY and TXE flags */
////
/////* 3) Case of General Call enabled for the slave */
////#define  I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED        ((uint32_t)0x00120000)  /* GENCALL and BUSY flags */
////
/////* Slave RECEIVER mode --------------------------*/
/////* --EV2 */
//////#define  I2C_EVENT_SLAVE_BYTE_RECEIVED                     ((uint32_t)0x00020040)  /* BUSY and RXNE flags */
/////* --EV4  */
////#define  I2C_EVENT_SLAVE_STOP_DETECTED                     ((uint32_t)0x00000010)  /* STOPF flag */
////
/////* Slave TRANSMITTER mode -----------------------*/
/////* --EV3 */
////#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTED                  ((uint32_t)0x00060084)  /* TRA, BUSY, TXE and BTF flags */
////#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTING                 ((uint32_t)0x00060080)  /* TRA, BUSY and TXE flags */
/////* --EV3_2 */
////#define  I2C_EVENT_SLAVE_ACK_FAILURE                       ((uint32_t)0x00000400)  /* AF flag */
////
/////* I2C FLAG mask */
////#define I2C_EVENT_FLAG_Mask	((uint32_t)0x00FFFFFF)
////
//////Мои константы.
////#define  I2C_EVENT_SLAVE_BYTE_RECEIVED		((uint32_t)0x00020044)  /* BUSY, RXNE and BTF flags */
////#define  I2C_EVENT_SLAVE_BYTE_RECEIVING		((uint32_t)0x00020040)  /* BUSY and RXNE flags */
////#define  I2C_EVENT_SLAVE_UNCKNOW			((uint32_t)0x000600c2)  /* ADDR, TXE, RXNE, BUSY and TRA flags */
////#define  I2C_EVENT_SLAVE_UNCKNOW_1			((uint32_t)0x00060004)  /* TRA, BUSY and BTF flags */
////--------------------------
//typedef enum{
//  I2C_IT_STATE_RESET = 0,   	/*!< Peripheral is not yet Initialized         */
//  I2C_IT_STATE_READY,  			/*!< Peripheral Initialized and ready for use  */
//  I2C_IT_STATE_ADDR_MATCH,
//  I2C_IT_STATE_BUSY_TX,   		/*!< Data Transmission process is ongoing      */
//  I2C_IT_STATE_BUSY_RX,   		/*!< Data Reception process is ongoing         */
//  I2C_IT_STATE_STOP,
//}I2C_IT_State_t;
////****************************************************
////Структура контекста для работы с портом I2C по прерываниям.
////#pragma pack(push, 1)//размер выравнивания в 1 байт
//typedef struct{
//	I2C_TypeDef *i2c;
//	uint32_t 	 i2cMode;		// Master или Slave
//	uint32_t 	 gpioRemap;		// Ремап выводов для I2C1, для I2C2 ремапа нет.
//	uint32_t 	 i2cSpeed;
//
//	I2C_IT_State_t 	ITState;	//
//	uint32_t 		DMAState;	//
//	uint32_t	timeOut;		//таймаут между запросами. Нужно для периициализации I2C в случае зависания.
//	uint32_t    resetCount;		//Счетчик переинициализация i2c. Нужен для отладки.
//
//	uint32_t 	slaveAddr;		// В режиме Master - адрес Slave-устройства к которому идет обращение,
//								// в режиме Slave  - адрес устройста на шине.
//
//	uint32_t 	slaveRegAddr;	// В режиме Master - адрес регистра Slave-устройства куда хотим писать/читать данные.
//								// в режиме Slave  - ???
//
//	//uint8_t 	*pTxBuf;		// указатель на буфер передачи.
//	uint8_t 	pTxBuf[I2C_IT_TX_BUF_LEN_MAX]; // буфер передачи.
//	uint32_t 	txBufSize;		// размер буфера передачи
//	uint32_t	txBufIndex;		// индекс буфера передачи.
//
//	//uint8_t 	*pRxBuf;		// указатель на буфер приема.
//	uint8_t 	pRxBuf[I2C_IT_RX_BUF_LEN_MAX]; // буфер приема.
//	uint32_t 	rxBufSize;		// размер буфера приема.
//	uint32_t	rxBufIndex;		// индекс буфера приема.
//
//	void(*i2cSlaveRxCpltCallback)(void); 	//
//	void(*i2cSlaveTxCpltCallback)(void); 	//
//
//}I2C_IT_t;
////#pragma pack(pop)//вернули предыдущую настройку.
////*******************************************************************************************
//void 	 I2C_IT_Init(I2C_IT_t *i2c);
//void 	 I2C_IT_SetTxSize(I2C_IT_t *i2cIt, uint32_t size);
//uint8_t* I2C_IT_GetpTxBuf(I2C_IT_t *i2cIt);
//uint8_t* I2C_IT_GetpRxBuf(I2C_IT_t *i2cIt);
//uint32_t I2C_IT_GetLastEvent(I2C_TypeDef* I2Cx);
//
////Обработчики прерывания
//void I2C_IT_EV_Handler(I2C_TypeDef *i2c);
//void I2C_IT_ER_Handler(I2C_TypeDef *i2c);
////*******************************************************************************************
////*******************************************************************************************
////Работа чере DMA.
//#define I2C1_TX_DMAChannel	DMA1_Channel6
//#define I2C1_RX_DMAChannel	DMA1_Channel7
//
//#define I2C2_TX_DMAChannel	DMA1_Channel4
//#define I2C2_RX_DMAChannel	DMA1_Channel5
////--------------------------
//typedef enum{
//	I2C_DMA_READY = 0,	//I2C и DMA готовы к передаче данных.
//	I2C_DMA_NOT_INIT,	//I2C и DMA не инициализированны.
//	I2C_DMA_NAC,		//Slave не ответил на свой адрес.
//	I2C_DMA_BUSY,		//I2C и DMA заняты, идет передача/прием данных.
//	I2C_DMA_ERR			//Ошибка DMA.
//}I2C_DMA_State_t;
////*******************************************************************************************
//void 	 I2C_DMA_Init(I2C_IT_t *i2cIt);
//uint32_t I2C_DMA_State(I2C_IT_t *i2cIt);
//
//uint32_t I2C_DMA_Write(I2C_IT_t *i2cIt);
//uint32_t I2C_DMA_Read (I2C_IT_t *i2cIt);
//
////Обработчики прерывания
////I2C1_TX -> DMA1_Ch6
////I2C1_RX -> DMA1_Ch7
//void I2C1_IT_DMA_TX_Handler(void);
//void I2C1_IT_DMA_RX_Handler(void);
//
////I2C2_TX -> DMA1_Ch4
////I2C2_RX -> DMA1_Ch5
//void I2C2_IT_DMA_TX_Handler(void);
//void I2C2_IT_DMA_RX_Handler(void);
//*******************************************************************************************
//*******************************************************************************************
#endif /* I2C_ST_H_ */






















