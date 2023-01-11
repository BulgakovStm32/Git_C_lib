/*
 * i2c_ST.c
 *
 *  Created on: 20 дек. 2020 г.
 *      Author:
 */
//*******************************************************************************************
//*******************************************************************************************

#include "i2c_ST.h"

//*******************************************************************************************
//*******************************************************************************************
static uint32_t I2C1NacCount = 0;
static uint32_t I2C2NacCount = 0;
//*******************************************************************************************
//*******************************************************************************************
static uint32_t _i2c_LongWait(I2C_TypeDef *i2c, uint32_t flag){

	uint32_t count = 0;
	//---------------------
	while(!(i2c->SR1 & flag))//Ждем отпускания флага.
	{
		if(++count >= I2C_WAIT_TIMEOUT) return 1;
	}
	return 0;
}
//**********************************************************
static void _i2c_GPIO_Init(I2C_TypeDef *i2c, uint32_t remap){

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;//Включаем тактирование GPIOB
	//Тактирование I2C_1
	if(i2c == I2C1)
	{
		//Ремап:
		//I2C1_SCL - PB8,
		//I2C1_SDA - PB9.
		if(remap == I2C_GPIO_REMAP)
		{
			AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
			GPIOB->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1 |
						  GPIO_CRH_CNF8    | GPIO_CRH_CNF9;
		}
		//I2C1_SCL - PB6,
		//I2C1_SDA - PB7.
		else
		{
			GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1 |
						  GPIO_CRL_CNF6    | GPIO_CRL_CNF7;
		}
	}
	//Тактирование I2C_2
	else if(i2c == I2C2)
	{
		//I2C2_SCL - PB10,
		//I2C2_SDA - PB11.
		GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_MODE11_1 |
					  GPIO_CRH_CNF10    | GPIO_CRH_CNF11;
	}
}
//**********************************************************
static void _i2c_ClockEnable(I2C_TypeDef *i2c){

	if(i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	else		    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
}
//**********************************************************
static void _i2c_ModeInit(I2C_TypeDef *i2c, uint32_t mode){

	//Инициализация I2C.
	i2c->CR1 &= ~I2C_CR1_PE;    //Откл. модуля I2C.
	i2c->CR1 |=  I2C_CR1_SWRST; //Программный сброс модуля I2C
	i2c->CR1 &= ~I2C_CR1_SWRST; //Это нужно для востановления работоспособноси после КЗ на линии.
	i2c->CR1 |=  I2C_CR1_PE;    //Включение модуля I2C1.
	i2c->CR1 &= ~I2C_CR1_SMBUS; //модуль работает в режиме I2C
	i2c->SR1  = 0; 			    //Сброс флагов ошибок.

	if(mode == I2C_MODE_MASTER) i2c->SR2 |=  I2C_SR2_MSL;//режим Master.
	else						i2c->SR2 &= ~I2C_SR2_MSL;//режим Slave
}
//**********************************************************
static void _i2c_SetSlaveAddress(I2C_TypeDef *i2c, uint8_t slaveAddr){

	i2c->OAR1 = slaveAddr << 1; //адрес устройства на шине.
	i2c->CR1 |= I2C_CR1_ACK;	//разрешаем отправлять ACK/NACK после приема байта адреса.
}
//**********************************************************
static void _i2c_SetSpeed(I2C_TypeDef *i2c, uint32_t speed){

	i2c->CR2  = 0;
	i2c->CR2 |= (I2C_FREQ << I2C_CR2_FREQ_Pos);//APB1 = 36MHz
	i2c->CCR  = 0;
	//FastMode(400kHz)
	if(speed >= 400000)
	{
		i2c->CCR   |=  I2C_CCR_FS; //1 - режим FastMode(400kHz).
		i2c->CCR   |= (I2C_FM_CCR << I2C_CCR_CCR_Pos);
		i2c->TRISE |= (I2C_FM_TRISE << I2C_TRISE_TRISE_Pos);
	}
	//StandartMode(100kHz).
	else
	{
		i2c->CCR   &= ~I2C_CCR_FS; //0 - режим STANDART(100kHz).
		i2c->CCR   |= (I2C_SM_CCR << I2C_CCR_CCR_Pos);
		i2c->TRISE |= (I2C_SM_TRISE << I2C_TRISE_TRISE_Pos);
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**************************Общие функции для работы с I2C***********************************
I2C_State_t I2C_StartAndSendDeviceAddr(I2C_TypeDef *i2c, uint8_t deviceAddr){

	//Формирование Start condition.
	i2c->CR1 |= I2C_CR1_START;
	if(_i2c_LongWait(i2c, I2C_SR1_SB)) return I2C_ERR_START;//Ожидание формирования Start condition.
	(void)i2c->SR1;			   		//Для сброса флага SB необходимо прочитать SR1

	//Передаем адрес.
	i2c->DR = deviceAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_ADDR))//Ожидаем окончания передачи адреса
	{
		i2c->SR1 &= ~I2C_SR1_AF;  //Сброс флагов ошибок.
		if(i2c == I2C1) I2C1NacCount++;
		else			I2C2NacCount++;

		return I2C_ERR_ADDR;
	}
	(void)i2c->SR1;	//сбрасываем бит ADDR (чтением SR1 и SR2):
	(void)i2c->SR2;	//
	return I2C_OK;
}
//**********************************************************
I2C_State_t I2C_SendByte(I2C_TypeDef *i2c, uint8_t byte){

	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
	i2c->DR = byte;
	return I2C_OK;
}
//**********************************************************
I2C_State_t I2C_ReadData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){

	//receiving 1 byte
	if(len == 1)
	{
		i2c->CR1 &= ~I2C_CR1_ACK;				   //Фомирование NACK.
		i2c->CR1 |= I2C_CR1_STOP;				   //Формируем Stop.
		if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;//ожидаем окончания приема байта
		*(pBuf + 0) = i2c->DR;				       //считали принятый байт.
		i2c->CR1 |= I2C_CR1_ACK;				   //to be ready for another reception
	}
	//receiving 2 bytes
	else if(len == 2)
	{
		i2c->CR1 |=  I2C_CR1_POS;//
		i2c->CR1 &= ~I2C_CR1_ACK;//Фомирование NACK.
		if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
		i2c->CR1 |= I2C_CR1_STOP;//Формируем Stop.
		*(pBuf + 0) = i2c->DR;	 //считали принятый байт.
		*(pBuf + 1) = i2c->DR;	 //считали принятый байт.
		i2c->CR1 &= ~I2C_CR1_POS;//
		i2c->CR1 |= I2C_CR1_ACK; //to be ready for another reception
	}
	//receiving more than 2 bytes
	else
	{
		uint32_t i;
		for(i=0; i<(len-3); i++)
		{
			if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;
			*(pBuf + i) = i2c->DR;//Read DataN
		}
		//Вычитываем оставшиеся 3 байта.
		if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
		i2c->CR1 &= ~I2C_CR1_ACK; //Фомирование NACK
		*(pBuf + i + 0) = i2c->DR;//Read DataN-2
		i2c->CR1 |= I2C_CR1_STOP; //Формируем Stop
		*(pBuf + i + 1) = i2c->DR;//Read DataN-1
		if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;
		*(pBuf + i + 2) = i2c->DR;//Read DataN
		i2c->CR1 |= I2C_CR1_ACK;
	}
	return I2C_OK;
}
//**********************************************************
void I2C_Stop(I2C_TypeDef *i2c){

	if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return;
//	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return;
	i2c->CR1 |= I2C_CR1_STOP;		 //Формируем Stop
}
//**********************************************************
I2C_State_t I2C_SendDataWithStop(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){

	for(uint32_t i = 0; i < len; i++)
	{
		if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
		i2c->DR = *(pBuf + i);
	}
	I2C_Stop(i2c);	 //Формируем Stop
	return I2C_OK;
}
//**********************************************************
I2C_State_t I2C_SendDataWithoutStop(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){

	for(uint32_t i = 0; i < len; i++)
	{
		if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
		i2c->DR = *(pBuf + i);
	}
	//if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
	return I2C_OK;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**************************Функции для работы в режиме Master*******************************
void I2C_Master_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t speed){

	_i2c_GPIO_Init(i2c, remap);		    //Инициализация портов
	_i2c_ClockEnable(i2c);			    //Вкл. тактирования модуля I2C
	_i2c_ModeInit(i2c, I2C_MODE_MASTER);//Инициализация I2C в режиме Master.
	_i2c_SetSpeed(i2c, speed);			//Скорость работы.

	//Включение модуля I2C.
	i2c->CR1 |= I2C_CR1_PE;
}
//**********************************************************
uint32_t I2C_Master_GetNacCount(I2C_TypeDef *i2c){

	if(i2c == I2C1) return I2C1NacCount;
					return I2C2NacCount;
}
//**********************************************************
uint32_t I2C_Master_CheckSlave(I2C_TypeDef *i2c, uint8_t deviceAddr){

	uint32_t err = I2C_StartAndSendDeviceAddr(i2c, deviceAddr);
	i2c->CR1 |= I2C_CR1_STOP; //Формируем Stop

	if(err == I2C_OK) return 1;
					  return 0;
}
//**********************************************************
/*  Ф-ия передачи массива данных в Slave-устройство.
 *  Ф-ия блокирует работу основной программы пока не будет передан весь массив данных.
 *  Вход: *i2c 		 - используемый порт i2c,
 *  	  deviceAddr - адресс Slave-устройства,
 *  	  regAddr    - адрес регистра Slave-устройства куда хотим записать массив,
 *  	  *pBuf      - указать на буфер передачи,
 *  	  len		 - размер буфера передачи.
 *
 *  Выход: статус передачи I2C_State_t.
 */
I2C_State_t I2C_Master_Write(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint32_t len){

	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_WRITE) != I2C_OK) return I2C_ERR_ADDR;

	//Передача адреса в который хотим записать.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;

	//передача данных на запись.
	return(I2C_SendDataWithStop(i2c, pBuf, len));
}
//**********************************************************
/*  Ф-ия чтения массива данных из Slave-устройство.
 *  Ф-ия блокирует работу основной программы пока не будет вычитан весь буфер.
 *  Вход: *i2c 		 - используемый порт i2c,
 *  	  deviceAddr - адресс Slave-устройства,
 *  	  regAddr    - адрес регистра Slave-устройства откуда хотим читать данные,
 *  	  *pBuf      - указать на буфер приема,
 *  	  len		 - размер буфера приема.
 *
 *  Выход: статус передачи I2C_State_t.
 */
I2C_State_t I2C_Master_Read(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint32_t len){

	//if(I2C_DMA_State() != I2C_DMA_READY) return I2C_BUSY;

	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_WRITE) != I2C_OK) return I2C_ERR_ADDR;

	//Передача адреса с которого начинаем чтение.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;
	//---------------------
	//Формирование reStart condition.
	i2c->CR1 |= I2C_CR1_STOP; //Это команда нужна для работы с DS2782. Без нее не работает

	//Формирование Start + AddrSlave|Read.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_READ) != I2C_OK) return I2C_ERR_ADDR;

	//прием даннных
	return(I2C_ReadData(i2c, pBuf, len));
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void I2C_Slave_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t slaveAddr, uint32_t speed){

	_i2c_GPIO_Init(i2c, remap);		    		  //Инициализация портов
	_i2c_ClockEnable(i2c);			    		  //Вкл. тактирования модуля I2C
	_i2c_ModeInit(i2c, I2C_MODE_SLAVE); 		  //I2C в режиме Slave.
	_i2c_SetSlaveAddress(i2c, (uint8_t)slaveAddr);//адрес устройства на шине.
	_i2c_SetSpeed(i2c, speed);					  //Скорость работы.



//	//Инициализация портов.
//	_i2c_GPIO_Init(i2c, remap);
//	//Включение тактирования.
//	if(i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
//	else		    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
//	//Инициализация I2C в режиме Slave.
//	i2c->CR1 &= ~I2C_CR1_PE;   //Откл. модуля I2C.
//	i2c->CR1 |=  I2C_CR1_SWRST;//Программный сброс модуля I2C
//	i2c->CR1 &= ~I2C_CR1_SWRST;//Это нужно для востановления работоспособноси после КЗ на линии.
//	i2c->CR1 |=  I2C_CR1_PE;    //Включение модуля I2C1.
//	i2c->CR1 &= ~I2C_CR1_SMBUS; //модуль работает в режиме I2C
//	i2c->SR2 &= ~I2C_SR2_MSL;   //режим Slave.
//	i2c->SR1  = 0; 			    //Сброс флагов ошибок.
//
//	i2c->OAR1 = slaveAddr << 1; //адрес устройства на шине.
//	i2c->CR1 |= I2C_CR1_ACK;	//разрешаем отправлять ACK/NACK после приема байта адреса.
//
//	//Скорость работы.
//	i2c->CR2  = 0;
//	i2c->CR2 |= (I2C_FREQ << I2C_CR2_FREQ_Pos);//APB1 = 36MHz
//	i2c->CCR  = 0;
//
//	//FastMode(400kHz)
//	if(speed >= 400000)
//	{
//		i2c->CCR   |=  I2C_CCR_FS; //1 - режим FastMode(400kHz).
//		i2c->CCR   |= (I2C_FM_CCR << I2C_CCR_CCR_Pos);
//		i2c->TRISE |= (I2C_FM_TRISE << I2C_TRISE_TRISE_Pos);
//	}
//	//StandartMode(100kHz).
//	else
//	{
//		i2c->CCR   &= ~I2C_CCR_FS; //0 - режим STANDART(100kHz).
//		i2c->CCR   |= (I2C_SM_CCR << I2C_CCR_CCR_Pos);
//		i2c->TRISE |= (I2C_SM_TRISE << I2C_TRISE_TRISE_Pos);
//	}
//	//StandartMode(100kHz).
////	i2c->CCR   &= ~I2C_CCR_FS; //1 - режим FastMode(400kHz), 0 - режим STANDART(100kHz).
////	i2c->CCR   |= (I2C_SM_CCR << I2C_CCR_CCR_Pos);
////	i2c->TRISE |= (I2C_SM_TRISE << I2C_TRISE_TRISE_Pos);

	//Включение модуля I2C.
//	i2c->CR1 |= I2C_CR1_PE;
}
//**********************************************************

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//****************************Работа I2C по прерываниям.*************************************
/*
 * Указатели на контекст шины I2C. Они нужны для работы обработчиков прерываний.
 */
static I2C_IT_t	*i2c1_IT_Define;
static I2C_IT_t	*i2c2_IT_Define;

//static uint8_t crc = 0xff;
//*******************************************************************************************
//*******************************************************************************************
void I2C_IT_Init(I2C_IT_t *i2cIt){

		 if(i2cIt->i2c == I2C1)  i2c1_IT_Define = i2cIt;
	else if(i2cIt->i2c == I2C2)  i2c2_IT_Define = i2cIt;
	else return;
	//Инициализация I2C
	if(i2cIt->i2cMode == I2C_MODE_MASTER) I2C_Master_Init(i2cIt->i2c,
														  i2cIt->gpioRemap,
														  i2cIt->i2cSpeed);
	else								  I2C_Slave_Init (i2cIt->i2c,
														  i2cIt->gpioRemap,
														  i2cIt->slaveAddr,
														  i2cIt->i2cSpeed);
	//Инициализация прерывания.
	i2cIt->i2c->CR2 |= I2C_CR2_ITEVTEN | //Разрешение прерывания по событию.
					   I2C_CR2_ITERREN;  //Разрешение прерывания по ошибкам.
	if(i2cIt->i2c == I2C1)
	{
		//Приоритет прерывания.
		NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
		NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
		//Разрешаем прерывание.
		NVIC_EnableIRQ(I2C1_EV_IRQn);
		NVIC_EnableIRQ(I2C1_ER_IRQn);
	}
	else
	{

//		/* 2 bits for pre-emption priority and 2 bits for subpriority */
//		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//
//		/* Set USART1 interrupt preemption priority to 1 */
//		NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
//
//		/* Set SysTick interrupt preemption priority to 3 */
//		NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));

//		//Приоритет прерывания.
//		NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
//		NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
//		//Разрешаем прерывание.
//		NVIC_EnableIRQ(I2C2_EV_IRQn);
//		NVIC_EnableIRQ(I2C2_ER_IRQn);
	}
	i2cIt->ITState = I2C_IT_STATE_READY;
	//Включение модуля I2C.
	i2cIt->i2c->CR1 |= I2C_CR1_PE;
}
//**********************************************************
uint8_t* I2C_IT_GetpTxBuf(I2C_IT_t *i2cIt){

	return i2cIt->pTxBuf;
}
//**********************************************************
void I2C_IT_SetTxSize(I2C_IT_t *i2cIt, uint32_t size){

	i2cIt->txBufSize = size;
}
//**********************************************************
uint8_t* I2C_IT_GetpRxBuf(I2C_IT_t *i2cIt){

	return i2cIt->pRxBuf;
}
//**********************************************************
//Ф-я из STM32F10x_StdPeriph_Driver, файл stm32f10x_i2c.c V3.5.0 11-March-2011
/**
  * @brief Returns the last I2Cx Event.
  * @param I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @note: For detailed description of Events, please refer to section
  *    	   I2C_Events in stm32f10x_i2c.h file.
  * @retval The last event
  */
//uint32_t I2C_IT_GetLastEvent(I2C_TypeDef* I2Cx){
//
//  uint32_t lastevent = 0;
//  uint32_t flag1 = 0, flag2 = 0;
//  //---------------------
//  /* Read the I2Cx status register */
//  flag1 = I2Cx->SR1;
//  flag2 = I2Cx->SR2;
//  flag2 = flag2 << 16;
//
//  /* Get the last event value from I2C status register */
//  lastevent = (flag1 | flag2) & I2C_EVENT_FLAG_Mask;
//
//  /* Return status */
//  return lastevent;
//}
//*******************************************************************************************
//*******************************************************************************************
static void I2C_IT_ReadByteToBuffer(I2C_IT_t *i2cIt){

	//static uint8_t crc  = 0xff;
	//	   uint8_t data = 0;
	//---------------------
	i2cIt->ITState = I2C_IT_STATE_BUSY_RX;
	//Если есть что принимать то принимаем.
	if(i2cIt->rxBufIndex < i2cIt->rxBufSize)
	{
		//Складываем принятый байт в приемный буфер.
		*(i2cIt->pRxBuf + i2cIt->rxBufIndex) = (uint8_t)i2cIt->i2c->DR;
		i2cIt->rxBufIndex++;
		//if(i2cIt->rxBufIndex < I2C_IT_RX_BUF_LEN_MAX) i2cIt->rxBufIndex++;
		//смотрим сколько байт нужно принять.
		//второй байт пакета - это размер принимаемого пакета. подробности в описании протокола.
		if(i2cIt->rxBufIndex == 2 &&
		   i2cIt->pRxBuf[1]	 != 0) i2cIt->rxBufSize = i2cIt->pRxBuf[1] + 2;
		if(i2cIt->rxBufSize > I2C_IT_RX_BUF_LEN_MAX) i2cIt->rxBufSize = I2C_IT_RX_BUF_LEN_MAX;
		//приняли нужное кол-во байтов.
		if(i2cIt->rxBufIndex >= i2cIt->rxBufSize)
		{
			i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Откл. прерывание I2C_IT_BUF.
			i2cIt->i2cSlaveRxCpltCallback();    //Разбор принятого пакета.
		}


//		data = (uint8_t)i2cIt->i2c->DR;
//		//Складываем принятый байт в приемный буфер.
//		*(i2cIt->pRxBuf + i2cIt->rxBufIndex) = data;
//		i2cIt->rxBufIndex++;
//		//if(i2cIt->rxBufIndex < I2C_IT_RX_BUF_LEN_MAX) i2cIt->rxBufIndex++;
//		//смотрим сколько байт нужно принять.
//		//второй байт пакета - это размер принимаемого пакета. подробности в описании протокола.
//		if(i2cIt->rxBufIndex == 2 &&
//		   i2cIt->pRxBuf[1]	 != 0) i2cIt->rxBufSize = i2cIt->pRxBuf[1] + 2;
//		if(i2cIt->rxBufSize > I2C_IT_RX_BUF_LEN_MAX) i2cIt->rxBufSize = I2C_IT_RX_BUF_LEN_MAX;
//		//Считаем CRC при приеме каждого байта.
//		crc = CRC8_ForEachByte(data, crc);
//		//приняли нужное кол-во байтов.
//		if(i2cIt->rxBufIndex >= i2cIt->rxBufSize)
//		{
//			i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Откл. прерывание I2C_IT_BUF.
//
//			if(crc == *(i2cIt->pRxBuf + i2cIt->rxBufIndex - 1))
//			{
//				i2cIt->i2cSlaveRxCpltCallback();    //Разбор принятого пакета.
//			}
//			crc = 0xff;
//		}
	}
}
//**********************************************************
static void I2C_IT_WriteByteFromBuffer(I2C_IT_t *i2cIt){

	i2cIt->ITState = I2C_IT_STATE_BUSY_TX;
	//Еcли есть что передавать то передаем
	if(i2cIt->txBufSize > 0)
	{
		//Передаем очередной байт
		i2cIt->i2c->DR = (uint8_t)*(i2cIt->pTxBuf + i2cIt->txBufIndex);
		//Уменьшили кол-во передаваемых байтов на 1.
		i2cIt->txBufSize--;
		//Инкремент указателя буфера.
		i2cIt->txBufIndex++;
		//Вызов обработчика события передачи пакета.
		if(i2cIt->txBufSize == 0) i2cIt->i2cSlaveTxCpltCallback();
	}
}
//*******************************************************************************************
//***************************Обработчики прерывания******************************************
static void I2C_IT_Master(I2C_IT_t *i2cIt){


}
//*******************************************************************************************
//*******************************************************************************************
static void I2C_IT_Slave(I2C_IT_t *i2cIt){

	I2C_TypeDef *i2c = i2cIt->i2c;
	//---------------------
	(void)i2c->SR1;//рекомендация из даташита
	/*------------------------------------------------------------------------*/
	/* ADDR set --------------------------------------------------------------*/
	if((i2c->SR1 & I2C_SR1_ADDR) && (i2c->CR2 & I2C_CR2_ITEVTEN))
	{
		(void)i2c->SR2;             //сбрасываем I2C_IT_ADDR чтением SR1 и SR2
		i2c->CR2 |= I2C_CR2_ITBUFEN;//включаем прерывание I2C_IT_BUF - вначале было отключено

		//Это нужно для работы через DMA
		//if(!(i2c->SR2 & I2C_SR2_TRA)) i2c->CR2 |=  I2C_CR2_ITBUFEN;//вкл. прерывание I2C_IT_BUF
		//else						  i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Откл. прерывание I2C_IT_BUF.

		i2cIt->rxBufIndex = 0;		//Сброс счетчиков байтов RX
		i2cIt->txBufIndex = 0;		//и TX.
		i2cIt->ITState 	  = I2C_IT_STATE_ADDR_MATCH;
	}
	/*------------------------------------------------------------------------*/
	/* STOPF set -------------------------------------------------------------*/
	else if((i2c->SR1 & I2C_SR1_STOPF) && (i2c->CR2 & I2C_CR2_ITEVTEN))
	{
		i2c->CR1 &= ~I2C_CR1_STOP;   //и записью в CR1 (из примера от ST)
		i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Откл. прерывание I2C_IT_BUF.
		i2cIt->ITState = I2C_IT_STATE_STOP;
	}
	/*------------------------------------------------------------------------*/
	/* I2C in mode Transmitter -----------------------------------------------*/
	else if(i2c->SR2 & I2C_SR2_TRA)//От Мастера пришла команда на чтение - SLA+Rd
	{
		/* TXE set and BTF reset --------------------*/
		if((i2c->SR1 & I2C_SR1_TXE)     &&
		   (i2c->CR2 & I2C_CR2_ITBUFEN) &&
		  !(i2c->SR1 & I2C_SR1_BTF) )
		{
			I2C_IT_WriteByteFromBuffer(i2cIt);//Передаем очередной байт
		}
		/* BTF set ----------------------------------*/
		else if((i2c->SR1 & I2C_SR1_BTF) && (i2c->CR2 & I2C_CR2_ITEVTEN))
		{
			I2C_IT_WriteByteFromBuffer(i2cIt);//Передаем очередной байт
		}
		else{ /* Do nothing */ }
	}
	/*------------------------------------------------------------------------*/
	/* I2C in mode Receiver --------------------------------------------------*/
	else
	{
		/* RXNE set and BTF reset -------------*/
		if((i2c->SR1 & I2C_SR1_RXNE)    &&
		   (i2c->CR2 & I2C_CR2_ITBUFEN) &&
		  !(i2c->SR1 & I2C_SR1_BTF))
		{
			I2C_IT_ReadByteToBuffer(i2cIt);//Складываем принятый байт в приемный буфер.
		}
		/* BTF set ---------------------------*/
		else if((i2c->SR1 & I2C_SR1_BTF) && (i2c->CR2 & I2C_CR2_ITEVTEN))
		{
			I2C_IT_ReadByteToBuffer(i2cIt);//Складываем принятый байт в приемный буфер.
		}
		else{ /* Do nothing */ }
	}
	/*------------------------------------------------------------------------*/
	/*------------------------------------------------------------------------*/


//	uint32_t event;
//	// Reading last event
//	event = I2C_IT_GetLastEvent(i2cIt->i2c);
//	/*------------------------------------------------------------------------*/
//	/* BUSY and ADDR flags ---------------------------------------------------*/
//	if(event == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)
//	{
//		(void)i2cIt->i2c->SR2;	//сбрасываем I2C_IT_ADDR чтением SR1 и SR2
//		(void)i2cIt->i2c->SR2;
//
//		i2cIt->i2c->CR2  |= I2C_CR2_ITBUFEN;//включаем прерывание I2C_IT_BUF - вначале было отключено
//		i2cIt->rxBufIndex = 0;			    //Сброс счетчиков байтов RX
//		i2cIt->txBufIndex = 0;			    //и TX.
//		i2cIt->ITState 	  = I2C_IT_STATE_ADDR_MATCH;
//	}
//	/*------------------------------------------------------------------------*/
//	/* I2C in mode Receiver --------------------------------------------------*/
//	/* BUSY and RXNE flags */
//	if(event == I2C_EVENT_SLAVE_BYTE_RECEIVING)
//	{
//		I2C_IT_ReadByteToBuffer(i2cIt);//Складываем принятый байт в приемный буфер.
//	}
//	/*-----------------------------------------*/
//	/* BUSY, RXNE and BTF flags */
//	if(event == I2C_EVENT_SLAVE_BYTE_RECEIVED)
//	{
//		I2C_IT_ReadByteToBuffer(i2cIt);//Складываем принятый байт в приемный буфер.
//	}
//	/*-----------------------------------------*/
//	/* TRA, BUSY, TXE, ADDR and RXNE  flags */
////	if(event == I2C_EVENT_SLAVE_UNCKNOW)
////	{
////		(void)i2cIt->i2c->DR; 			    //Очистка флага RXNE
////		i2cIt->i2c->CR2  |= I2C_CR2_ITBUFEN;//включаем прерывание I2C_IT_BUF - вначале было отключено
////		i2cIt->rxBufIndex = 0;			    //Сброс счетчиков байтов RX
////		i2cIt->txBufIndex = 0;			    //и TX.
////
////		I2C_IT_WriteByteFromBuffer(i2cIt);  //Передаем очередной байт
////		event = I2C_IT_GetLastEvent(i2cIt->i2c);
////	}
//	/*------------------------------------------------------------------------*/
//	/* I2C in mode Transmitter -----------------------------------------------*/
//	/* TRA, BUSY, TXE and ADDR flags */
//	if(event == I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED)
//	{
//		I2C_IT_WriteByteFromBuffer(i2cIt);  //Передаем очередной байт
//	}
//	/*-----------------------------------------*/
//	/* TRA, BUSY and TXE flags */
//	if(event == I2C_EVENT_SLAVE_BYTE_TRANSMITTING)
//	{
//		I2C_IT_WriteByteFromBuffer(i2cIt);//Передаем очередной байт
//	}
//	/*-----------------------------------------*/
//	/* TRA, BUSY, TXE and BTF flags */
//	if(event == I2C_EVENT_SLAVE_BYTE_TRANSMITTED)
//	{
//		I2C_IT_WriteByteFromBuffer(i2cIt);//Передаем очередной байт
//	}
//	/*-----------------------------------------*/
//	/* TRA, BUSY and BTF flags */
////	if(event == I2C_EVENT_SLAVE_UNCKNOW_1)
////	{
////		I2C_IT_WriteByteFromBuffer(i2cIt);//Передаем очередной байт
////	}
//	/*------------------------------------------------------------------------*/
//	/*------------------------------------------------------------------------*/
//	/* STOPF flag */
//	if(event == I2C_EVENT_SLAVE_STOP_DETECTED)
//	{
//		i2cIt->i2c->CR1 &= ~I2C_CR1_STOP;   //и записью в CR1 (из примера от ST)
//		i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Откл. прерывание I2C_IT_BUF.
//		i2cIt->ITState   = I2C_IT_STATE_STOP;
//	}
//	/*------------------------------------------------------------------------*/
//	/*------------------------------------------------------------------------*/
}
//*******************************************************************************************
//*******************************************************************************************
static void _i2c_ClearErrFlagAndStop(I2C_TypeDef *i2c, uint32_t flag){

	i2c->SR1 &= ~(flag); 	   //Сброс флага ошибки.
	i2c->CR1 |=	I2C_CR1_ACK;   //to be ready for another reception
	//i2c->CR1 |= I2C_CR1_STOP; //Формируем Stop
}

//Обработчик прерывания ошибок I2C
static void I2C_IT_Error(I2C_IT_t *i2cIt){

	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	//NACK - Acknowledge failure
	if(i2c->SR1 & I2C_SR1_AF)
	{
		_i2c_ClearErrFlagAndStop(i2c, I2C_SR1_AF);//Сброс флага ошибки.
		i2c->CR2 &= ~I2C_CR2_ITBUFEN;			  //Откл. прерывание I2C_IT_BUF.
		i2cIt->ITState = I2C_IT_STATE_STOP;
	}
	//------------------------------
	//Bus error
	if(i2c->SR1 & I2C_SR1_BERR)
	{
		//i2c->SR1 &= ~I2C_SR1_BERR; //Сброс BERR.
		_i2c_ClearErrFlagAndStop(i2c, I2C_SR1_BERR);
	}
	//------------------------------
	//Arbitration loss (Master)
	if(i2c->SR1 & I2C_SR1_ARLO)
	{
		//i2c->SR1 &= ~I2C_SR1_ARLO; //Сброс ARLO.
		_i2c_ClearErrFlagAndStop(i2c, I2C_SR1_ARLO);
	}
	//------------------------------
	//Overrun/Underrun
	if(i2c->SR1 & I2C_SR1_OVR)
	{
		//i2c->SR1 &= ~I2C_SR1_OVR; //Сброс OVR.
		_i2c_ClearErrFlagAndStop(i2c, I2C_SR1_OVR);
	}
	//------------------------------
	//PEC error
	if(i2c->SR1 & I2C_SR1_PECERR)
	{
		i2c->SR1 &= ~I2C_SR1_PECERR; //Сброс PECERR.
	}
	//------------------------------
	//Timeout/Tlow error
	if(i2c->SR1 & I2C_SR1_TIMEOUT)
	{
		i2c->SR1 &= ~I2C_SR1_TIMEOUT; //Сброс TIMEOUT.
	}
	//------------------------------
	//SMBus Alert
	if(i2c->SR1 & I2C_SR1_SMBALERT)
	{
		i2c->SR1 &= ~I2C_SR1_SMBALERT; //Сброс SMBALERT.
	}
	//------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//Обработчик прерывания событий I2C
void I2C_IT_EV_Handler(I2C_TypeDef *i2c){

	I2C_IT_t *i2cIt;
	//---------------------
	//Определение для какого I2C прерывание.
		 if(i2c == I2C1) i2cIt = i2c1_IT_Define;
	else if(i2c == I2C2) i2cIt = i2c2_IT_Define;
	else return;
	//Мастер или Слейв
	if(i2cIt->i2cMode == I2C_MODE_MASTER) I2C_IT_Master(i2cIt);
	else								  I2C_IT_Slave(i2cIt);
}
//**********************************************************
//Обработчик прерывания ошибок I2C
void I2C_IT_ER_Handler(I2C_TypeDef *i2c){

	if(i2c == I2C1) I2C_IT_Error(i2c1_IT_Define);
	else			I2C_IT_Error(i2c2_IT_Define);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**********************************Работа с I2C через DMA.**********************************
//I2C1_TX -> DMA1_Ch6
//I2C1_RX -> DMA1_Ch7

//I2C2_TX -> DMA1_Ch4
//I2C2_RX -> DMA1_Ch5

//static volatile I2C_DMA_State_t I2cDmaStateReg = I2C_DMA_NOT_INIT;
//*******************************************************************************************
void I2C_DMA_Init(I2C_IT_t *i2cIt){

	I2C_IT_Init(i2cIt);
	RCC->AHBENR    |= RCC_AHBENR_DMA1EN;//Enable the peripheral clock DMA1
	i2cIt->DMAState = I2C_DMA_READY;
}
//**********************************************************
uint32_t I2C_DMA_State(I2C_IT_t *i2cIt){

	return i2cIt->DMAState;
}
//**********************************************************
//I2C_DMA_State_t I2C_DMA_Write(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t size){
uint32_t I2C_DMA_Write(I2C_IT_t *i2cIt){

	DMA_Channel_TypeDef *dma;
	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	if(i2cIt->DMAState != I2C_DMA_READY) return I2C_DMA_BUSY;

	__disable_irq();
	//DMA1Channel config
	if(i2c == I2C1) dma = DMA1_Channel6;//I2C1_TX -> DMA1_Ch6
	else			dma = DMA1_Channel4;//I2C2_TX -> DMA1_Ch4

	dma->CCR  &= ~DMA_CCR_EN;		  		//Channel disable
	dma->CPAR  = (uint32_t)&(i2c->DR);		//Peripheral address.
	dma->CMAR  = (uint32_t)i2cIt->pTxBuf;	//Memory address.
	dma->CNDTR = i2cIt->txBufSize;	  		//Data size.
	dma->CCR   = (3 << DMA_CCR_PL_Pos)   | 	//PL[1:0]: Channel priority level - 11: Very high.
			     (0 << DMA_CCR_PSIZE_Pos)| 	//PSIZE[1:0]: Peripheral size - 00: 8-bits.
				 (0 << DMA_CCR_MSIZE_Pos)| 	//MSIZE[1:0]: Memory size     - 00: 8-bits.
			     DMA_CCR_MINC |			 	//MINC: Memory increment mode - Memory increment mode enabled.
				 DMA_CCR_DIR  |           	//DIR:  Data transfer direction: 1 - Read from memory.
				 //DMA_CCR_CIRC | 		 	//CIRC: Circular mode
				 //DMA_CCR_TEIE | 		 	//TEIE: Transfer error interrupt enable
				 //DMA_CCR_HTIE | 		 	//HTIE: Half transfer interrupt enable
				 DMA_CCR_TCIE;// | 		 	//TCIE: Transfer complete interrupt enable
				 //DMA_CCR_EN;			 	//EN: Channel enable

	if(i2c == I2C1)
	{
		NVIC_SetPriority(DMA1_Channel6_IRQn, 4);//Set priority
		NVIC_EnableIRQ(DMA1_Channel6_IRQn);     //Enable DMA1_Channel6_IRQn
	}
	else
	{
		NVIC_SetPriority(DMA1_Channel4_IRQn, 4);//Set priority
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);     //Enable DMA1_Channel6_IRQn
	}

//	i2c->CR2 |= I2C_CR2_DMAEN;//DMAEN(DMA requests enable)
//	//Формирование Start + AddrSlave|Write.
//	if(I2C_StartAndSendDeviceAddr(i2c, (i2cIt->slaveAddr<<1)|I2C_MODE_WRITE) != I2C_OK)
//	{
//		__enable_irq();
//		return I2C_DMA_NAC;
//	}
//	//Передача адреса в который хотим записать.
//	i2c->DR = i2cIt->slaveRegAddr;

	i2c->CR2 |= I2C_CR2_DMAEN; //DMAEN(DMA requests enable)
	//i2c->CR1 |= I2C_CR1_ACK;   //to be ready for another reception
	dma->CCR |= DMA_CCR_EN;    //DMA Channel enable
	i2cIt->DMAState = I2C_DMA_BUSY;
	__enable_irq();
	return I2C_DMA_BUSY;
}
//**********************************************************
uint32_t I2C_DMA_Read(I2C_IT_t *i2cIt){

	DMA_Channel_TypeDef *dma;
	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	if(i2cIt->DMAState != I2C_DMA_READY) return I2C_DMA_BUSY;

	__disable_irq();
	//DMA1Channel config
	if(i2c == I2C1) dma = DMA1_Channel7;//I2C1_RX -> DMA1_Ch7
	else			dma = DMA1_Channel5;//I2C2_RX -> DMA1_Ch5

	dma->CCR  &= ~DMA_CCR_EN;		  	 	//Channel disable
	dma->CPAR  = (uint32_t)&(i2c->DR);	 	//Peripheral address.
	dma->CMAR  = (uint32_t)i2cIt->pRxBuf; 	//Memory address.
	dma->CNDTR = i2cIt->rxBufSize;	  	 	//Data size.
	dma->CCR   = (3 << DMA_CCR_PL_Pos)   | 	//PL[1:0]: Channel priority level - 11: Very high.
			     (0 << DMA_CCR_PSIZE_Pos)| 	//PSIZE[1:0]: Peripheral size - 00: 8-bits.
				 (0 << DMA_CCR_MSIZE_Pos)| 	//MSIZE[1:0]: Memory size     - 00: 8-bits.
				 DMA_CCR_MINC |			 	//MINC: Memory increment mode - Memory increment mode enabled.
				 //DMA_CCR_DIR  |           //DIR:  Data transfer direction: 1 - Read from memory.
				 //DMA_CCR_CIRC | 		    //CIRC: Circular mode
				 //DMA_CCR_TEIE | 		    //TEIE: Transfer error interrupt enable
				 //DMA_CCR_HTIE | 		    //HTIE: Half transfer interrupt enable
				 DMA_CCR_TCIE;// | 		    //TCIE: Transfer complete interrupt enable
				//DMA_CCR_EN;			    //EN: Channel enable
	NVIC_SetPriority(DMA1_Channel7_IRQn, 0);//Set priority
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);     //Enable DMA1_Channel6_IRQn

	i2c->CR2 |= I2C_CR2_DMAEN | //DMA Requests Enable.
				I2C_CR2_LAST;	//DMA Last Transfer.
	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, (i2cIt->slaveAddr<<1)|I2C_MODE_READ) != I2C_OK)
	{
		__enable_irq();
		return I2C_DMA_NAC;
	}
	i2c->CR1 |= I2C_CR1_ACK;//to be ready for another reception

	dma->CCR |= DMA_CCR_EN;//DMA Channel enable
	i2cIt->DMAState = I2C_DMA_BUSY;
	__enable_irq();
	return I2C_DMA_BUSY;
}
//*******************************************************************************************
//*******************************************************************************************
//Прерываение от DMA1.
//I2C1_TX -> DMA1_Ch6
//I2C1_RX -> DMA1_Ch7

//I2C2_TX -> DMA1_Ch4
//I2C2_RX -> DMA1_Ch5

//**********************************************************
static void DMA_ChDisableAndITFlagClear(DMA_Channel_TypeDef *dma, uint32_t flag){

	DMA1->IFCR |=  flag;      //сбросить флаг окончания обмена.
	dma->CCR   &= ~DMA_CCR_EN;//отключение канала DMA.
}
//**********************************************************
static void I2C_DMA_TX_Handler(I2C_IT_t *i2cIt){

	//-------------------------
	//Обмен завершен.
	if(DMA1->ISR & DMA_ISR_TCIF6)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel6, DMA_IFCR_CTCIF6);
		//Ожидаем окончания передачи последних байтов.
//		while(!(i2cIt->i2c->SR1 & I2C_SR1_BTF)){};//Ждем отпускания флага.
//		i2cIt->i2c->CR1 |= I2C_CR1_STOP | //Формируем Stop
//					       I2C_CR1_ACK;
		i2cIt->DMAState = I2C_DMA_READY;
	}
	//-------------------------
	//Передана половина буфера
	if(DMA1->ISR & DMA_ISR_HTIF6)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF6;//сбросить флаг.
	}
	//-------------------------
	//Произошла ошибка при обмене
	if(DMA1->ISR & DMA_ISR_TEIF6)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel6 ,DMA_IFCR_CTEIF6);
		i2cIt->DMAState = I2C_DMA_READY;
	}
	//-------------------------

}
//**********************************************************
static void I2C_DMA_RX_Handler(I2C_IT_t *i2cIt){

	//-------------------------
	//Обмен завершен.
	if(DMA1->ISR & DMA_ISR_TCIF7)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel7, DMA_IFCR_CTCIF7);

//		//Ожидаем окончания приема последних байтов.
//		if(I2C_LongWait(I2C1, I2C_SR1_BTF) != 0)
//		{
//			I2C1->CR1 |= I2C_CR1_STOP | //Формируем Stop
//					     I2C_CR1_ACK;
//		}

		i2cIt->i2c->CR1 |= I2C_CR1_STOP | //Формируем Stop
				     I2C_CR1_ACK;
		i2cIt->DMAState = I2C_DMA_READY;
	}
	//-------------------------
	//Передана половина буфера
	if(DMA1->ISR & DMA_ISR_HTIF7)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF7;//сбросить флаг.
	}
	//-------------------------
	//Произошла ошибка при обмене
	if(DMA1->ISR & DMA_ISR_TEIF7)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel7, DMA_IFCR_CTEIF7);
		i2cIt->DMAState = I2C_DMA_READY;
	}
	//-------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//Прерываение от DMA1.
//I2C1_TX -> DMA1_Ch6
void I2C1_IT_DMA_TX_Handler(void){

	I2C_DMA_TX_Handler(i2c1_IT_Define);
}
//**********************************************************
//I2C1_RX -> DMA1_Ch7
void I2C1_IT_DMA_RX_Handler(void){

	I2C_DMA_RX_Handler(i2c1_IT_Define);
}
//*******************************************************************************************
//*******************************************************************************************
//I2C2_TX -> DMA1_Ch4
void I2C2_IT_DMA_TX_Handler(void){

//	I2C_DMA_TX_Handler(i2c2_IT_Define);

	//-------------------------
	//Обмен завершен.
	if(DMA1->ISR & DMA_ISR_TCIF4)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel4, DMA_IFCR_CTCIF4);
		//Ожидаем окончания передачи последних байтов.
//		while(!(i2cIt->i2c->SR1 & I2C_SR1_BTF)){};//Ждем отпускания флага.
//		i2cIt->i2c->CR1 |= I2C_CR1_STOP | //Формируем Stop
//					       I2C_CR1_ACK;
//		i2cIt->DMAState = I2C_DMA_READY;
	}
	//-------------------------
	//Передана половина буфера
	if(DMA1->ISR & DMA_ISR_HTIF4)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF4;//сбросить флаг.
	}
	//-------------------------
	//Произошла ошибка при обмене
	if(DMA1->ISR & DMA_ISR_TEIF4)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel4 ,DMA_IFCR_CTEIF4);
//		i2cIt->DMAState = I2C_DMA_READY;
	}
	//-------------------------
}
//**********************************************************
//I2C2_RX -> DMA1_Ch5
void I2C2_IT_DMA_RX_Handler(void){

	I2C_DMA_RX_Handler(i2c2_IT_Define);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************















