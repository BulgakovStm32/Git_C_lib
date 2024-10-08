/*
 *  AHT10.c
 *
 *  Created on: 25 янв. 2021 г.
 *      Author: belyaev
 */
//AHT10 - датчик температуры и влажности.
//*******************************************************************************************
//*******************************************************************************************

#include "AHT10.h"

//*******************************************************************************************
//*******************************************************************************************
<<<<<<< HEAD
static uint8_t rxBuf[6] = {0};
=======
static uint8_t rxBuf[7] = {0};
static uint8_t status = 0;
>>>>>>> 3166ede0e13cc88e01e30a6755f6e1241489497c

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
<<<<<<< HEAD
static uint8_t aht10_read(uint8_t addr, uint8_t *data, uint8_t size){

	I2C_Master_Read(AHT10_I2C, AHT10_ADDR, addr, data, size);
    return 0;
=======
static void _readMeasurement(void){

	/* send measurement command */
	rxBuf[0] = AHTXX_START_MEASUREMENT_REG;			//send measurement command, strat measurement
	rxBuf[1] = AHTXX_START_MEASUREMENT_CTRL;		//send measurement control
	rxBuf[2] = AHTXX_START_MEASUREMENT_CTRL_NOP;	//send measurement NOP control

	if(I2C_StartAndSendDeviceAddr(AHT10_I2C, AHT10_ADDR|I2C_MODE_WRITE) != I2C_OK)
	{
		status = AHTXX_ACK_ERROR;
		return;
	}
	I2C_SendDataWithStop(AHT10_I2C, rxBuf, 3);

	//TODO ... /* check busy bit */

	//DELAY_milliS(AHTXX_MEASUREMENT_DELAY);

	/* read data from sensor */
	if(I2C_StartAndSendDeviceAddr(AHT10_I2C, AHT10_ADDR|I2C_MODE_READ) != I2C_OK)
	{
		status = AHTXX_ACK_ERROR;
		return;
	}
	I2C_ReadData(AHT10_I2C, rxBuf, 6);

	status = AHTXX_NO_ERROR;
>>>>>>> 3166ede0e13cc88e01e30a6755f6e1241489497c
}
//************************************************************
//static uint8_t aht10_write(uint8_t addr, uint8_t *data, uint8_t size){
//
//	I2C_Master_Write(AHT10_I2C, AHT10_ADDR, addr, data, size);
//    return 0;
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void AHT10_Init(void){

}
//************************************************************
void AHT10_SoftReset(void){

	uint8_t cmd = 0xBA;//soft reset command
	//-------------------
	I2C_StartAndSendDeviceAddr(AHT10_I2C, AHT10_ADDR);
	//I2C_SendData(I2C2, &cmd, 1);
	I2C_SendDataWithStop(AHT10_I2C, &cmd, 1);

//	aht10_write(0xBA, uint8_t *data, uint8_t size);//0xBA - soft reset command
	msDelay(50);
}
//************************************************************
void AHT10_ReadData(void){

	//Чтение данных
<<<<<<< HEAD
	aht10_read(0xAC, rxBuf, 6); //0xAC - start measurment command
=======
	_readMeasurement();
>>>>>>> 3166ede0e13cc88e01e30a6755f6e1241489497c
}
//**********************************************************
int32_t AHT10_GetTemperature(void){

	int32_t temp;
	//-------------------
	//Расчет температуры.
	//Выражение для преобразования температуры, °C: T = AHT10_ADC_Raw * 200 / 2^20 - 50
	temp  = ((rxBuf[3] & 0x0F) << 16) | (rxBuf[4] << 8) | rxBuf[5];//собираем значение температуры
	temp *= 19; // 19 - это 200 / 2^20 * 100000
	temp  = temp - (50 * 100000);
	return ((temp + 1000/2) / 1000);
}
//**********************************************************
uint32_t  AHT10_GetHumidity(void){

	uint32_t temp;
	const uint32_t div = 1<<20;//это 2^20
	//-------------------
	//Расчет влажности
	//Выражение для преобразования относительной влажности, %: H = AHT10_ADC_Raw * 100 / 2^20
	temp   = (rxBuf[1] << 16) | (rxBuf[2] << 8) | (rxBuf[3] & 0xF0);//собираем значение влажности
	temp >>= 4;
	return ((temp * 100 + div/2) / div);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************



<<<<<<< HEAD
=======






>>>>>>> 3166ede0e13cc88e01e30a6755f6e1241489497c
