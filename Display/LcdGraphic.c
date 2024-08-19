/*
 * LcdGraphic.c
 *
 *  Created on: 
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "LcdGraphic.h"

//*******************************************************************************************
//*******************************************************************************************
static uint8_t  lcdTextBuf[LCD_TEXT_BUFFER_SIZE];	   //буфер для вывода текста
static uint8_t  lcdVideoBuffer[LCD_VIDEO_BUFFER_SIZE]; //Cache buffer in SRAM 128*64 bits or 1024 bytes
static uint16_t lcdVideoBufferIndex = 0;               //Cache index
//*****************************
extern const unsigned char Ascii_Tab_12864[];//Рабочая.
#define TabAscii	       Ascii_Tab_12864   //Рабочая.
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Очистка текстового буфера.
static void ClearTextBuf(void){

	for(uint8_t i=0; i<LCD_TEXT_BUFFER_SIZE; i++) lcdTextBuf[i] = 0;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void Lcd_Init(void){

//	Lm6063LcdInit();
	SSD1306_Init(SSD1306_128x64);
}
//*****************************************************************************
//Вывод буфера на дисплей.
void Lcd_Update(void){

//	Lm6063LcdUpdate(lcdVideoBuffer);
//	Lcd_TIC32_SendData(lcdVideoBuffer);
	SSD1306_UpdateScreen(lcdVideoBuffer, LCD_VIDEO_BUFFER_SIZE);
}
//*****************************************************************************
uint8_t* Lcd_pVideoBuffer(void){

	return lcdVideoBuffer;
}
//*****************************************************************************
void Lcd_Filling(uint8_t byte){

	for(uint16_t i=0; i < LCD_VIDEO_BUFFER_SIZE; i++) lcdVideoBuffer[i] = byte;
}
//*****************************************************************************
//Clears the display
void Lcd_ClearVideoBuffer(void){
  
	//забиваем всю память 0
	//for(uint16_t i = 0; i < LCD_VIDEO_BUFFER_SIZE; i++) lcdVideoBuffer[i] = 0;
	Lcd_Filling(0);
}
//*****************************************************************************
//управление пикселем с координатами x,y. mode -> Off, On or Xor.
void Lcd_Pixel(uint8_t x, uint8_t y, uint8_t mode){
  
	uint16_t index;
	uint16_t offset, data;
	//--------------------
	//если передали в функцию некорректные данные - выходим.
	if((x > LCD_X_RES) || (y > LCD_Y_RES)) return;
	//--------------------
	index  = (((int)(y)/8)*128)+x; //считаем номер байта в массиве памяти дисплея
	offset = y-((y/8)*8);          //считаем номер бита в этом байте
	data   = lcdVideoBuffer[index];//берем байт по найденному индексу.
	//редактируем бит в этом байте
	     if( mode == PIXEL_OFF ) data &= ~(0x01 << offset);
	else if( mode == PIXEL_ON  ) data |=  (0x01 << offset);
	else if( mode == PIXEL_XOR ) data ^=  (0x01 << offset);
	//--------------------
	lcdVideoBuffer[index] = data;		//загружаем байт назад
}
//*****************************************************************************
//Draws a line between two points on the display - по Брезенхейму
void Lcd_Line(int x1, int y1, int x2, int y2, uint8_t mode){
  
	signed int dy       = 0;
	signed int dx       = 0;
	signed int stepx    = 0;
	signed int stepy    = 0;
	signed int fraction = 0;
	//--------------------
	//если передали в функцию некорректные данные - выходим.
	if(x1>LCD_X_RES || x2>LCD_X_RES || y1>LCD_Y_RES || y2>LCD_Y_RES) return;
	//--------------------
	//Перемещение начала координат в нижний левый угол экрана.
	y1 = 63 - y1;
	y2 = 63 - y2;
	//--------------------

	dy = y2 - y1;
	dx = x2 - x1;

	if(dy < 0)
	{
		dy    = -dy;
		stepy = -1;
	}
	else stepy = 1;

	if(dx < 0)
	{
		dx    = -dx;
		stepx = -1;
	}
	else stepx = 1;

	dy <<= 1;
	dx <<= 1;
	Lcd_Pixel(x1,y1,mode);

	if(dx > dy)
	{
	  fraction = dy - (dx >> 1);
	  while(x1 != x2)
		{
		  if(fraction >= 0)
			{
			  y1 += stepy;
			  fraction -= dx;
			}
		  x1 += stepx;
		  fraction += dy;
		  Lcd_Pixel(x1,y1,mode);
		}
	}
	else
	{
	  fraction = dx - (dy >> 1);
	  while(y1 != y2)
		{
		  if(fraction >= 0)
			{
			  x1 += stepx;
			  fraction -= dy;
			}
		  y1 += stepy;
		  fraction += dx;
		  Lcd_Pixel(x1,y1,mode);
		}
	}
}
//*****************************************************************************
//рисуем круг по координатам с радиусом - по Брезенхейму
void Lcd_Circle(uint8_t center_x, uint8_t center_y, uint8_t radius, uint8_t mode){
  
	signed char xc = 0;
	signed char yc = 0;
	signed char p  = 0;
	//--------------------
	if(center_x > LCD_X_RES || center_y > LCD_Y_RES) return;
	//--------------------
	yc = radius;
	p  = 3 - (radius<<1);
	while(xc <= yc)
	{
		Lcd_Pixel(center_x + xc, center_y + yc, mode);
		Lcd_Pixel(center_x + xc, center_y - yc, mode);
		Lcd_Pixel(center_x - xc, center_y + yc, mode);
		Lcd_Pixel(center_x - xc, center_y - yc, mode);
		Lcd_Pixel(center_x + yc, center_y + xc, mode);
		Lcd_Pixel(center_x + yc, center_y - xc, mode);
		Lcd_Pixel(center_x - yc, center_y + xc, mode);
		Lcd_Pixel(center_x - yc, center_y - xc, mode);
		if(p < 0) p +=  (xc++ << 2) + 6;
		else      p += ((xc++ - yc--)<<2) + 10;
	}
}
//*****************************************************************************
//рисуем батарейку с заполнением в %
void Lcd_Bar(int x1, int y1, int x2, int y2, uint8_t persent){
  
	unsigned char horizon_line,horizon_line2,i;
	//--------------------
	if(persent>100)return;

	Lcd_Line(x1,y2,x2,y2,1);  //down
	Lcd_Line(x2,y1,x2,y2,1);  //right
	Lcd_Line(x1,y1,x1,y2,1);  //left
	Lcd_Line(x1,y1,x2,y1,1);  //up
	Lcd_Line(x1+7,y1-1,x2-7,y1-1,1);
	Lcd_Line(x1+7,y1-2,x2-7,y1-2,1);

	horizon_line = persent*(y2-y1-3)/100;
	for(i=0;i<horizon_line;i++) Lcd_Line(x1+2,y2-2-i,x2-2,y2-2-i,1);

	horizon_line2 = (y2-y1-3);
	for(i=horizon_line2;i>horizon_line;i--) Lcd_Line(x1+2,y2-2-i,x2-2,y2-2-i,0);
}
//*****************************************************************************
//вывод изображения.
void Lcd_Image(const uint8_t *imageData){

	for(uint32_t i=0; i < LCD_VIDEO_BUFFER_SIZE; i++)
	{
		lcdVideoBuffer[i] = imageData[1023 - i];	//грузим данные
		//lcdVideoBuffer[i] = imageData[i];	//грузим данные
	}
}
//*****************************************************************************
//Установка курсора в положение Х,У. Диапазон значений Х,У: 1,1 .. 14,8.
void Lcd_GotoXYFont(uint8_t x, uint8_t y){
  
	if((x > 24) || (y > 8)) return;
	lcdVideoBufferIndex = (uint16_t)((y-1) * 128) + (uint16_t)((x-1) * 6);
}
//*****************************************************************************
//Утсановка курсора.
void Lcd_SetCursor(uint8_t x, uint8_t y){
  
	if(x > 22) x = 22;
	if(y > 8)  y = 8;
	//--------------------
	Lcd_GotoXYFont(x, y);

	for(uint8_t i=0; i < (22-x); i++)
	{
		if(lcdTextBuf[i]) Lcd_Chr(lcdTextBuf[i]);
	}
}
//*****************************************************************************
//Displays a character at current cursor location and increment cursor location
void Lcd_Chr(char ch){

	for(uint8_t i=0; i<5; i++)
	{
		//выделяем байт-столбик из символа и грузим в массив - 5 раз
		lcdVideoBuffer[lcdVideoBufferIndex++] = TabAscii[ch*5+i];
	}
	lcdVideoBuffer[lcdVideoBufferIndex++] = 0x00; //добавляем пробел между символами
}
//*****************************************************************************
//Displays a bold character at current cursor location and increment cursor location
void Lcd_ChrBold(char ch){
  
	uint8_t a = 0, b = 0, c = 0;
	//--------------------
	for(uint8_t i = 0; i < 5; i++)
	{
		c = TabAscii[(ch*5+i)];//выделяем столбец из символа

		b  = (c & 0x01) * 3;   //"растягиваем" столбец на два байта
		b |= (c & 0x02) * 6;
		b |= (c & 0x04) * 12;
		b |= (c & 0x08) * 24;

		c >>= 4;
		a  = (c & 0x01) * 3;
		a |= (c & 0x02) * 6;
		a |= (c & 0x04) * 12;
		a |= (c & 0x08) * 24;

		lcdVideoBuffer[lcdVideoBufferIndex]     = b;//копируем байты в экранный буфер
		lcdVideoBuffer[lcdVideoBufferIndex+1]   = b;//дублируем для получения жирного шрифта
		lcdVideoBuffer[lcdVideoBufferIndex+128] = a;
		lcdVideoBuffer[lcdVideoBufferIndex+129] = a;
		lcdVideoBufferIndex = lcdVideoBufferIndex+2;
	}
	lcdVideoBuffer[lcdVideoBufferIndex++] = 0x00;	//для пробела между символами
	lcdVideoBuffer[lcdVideoBufferIndex++] = 0x00;
}
//*****************************************************************************	
//Displays a character at current cursor location and increment cursor location
void Lcd_ChrBig(char ch){
  
	uint8_t a = 0, b = 0, c = 0;
	//--------------------
	for(uint8_t i = 0; i < 5; i++)
	{
		c = TabAscii[(ch*5+i)];		//выделяем столбец из символа

		b  = (c & 0x01) * 3;        //"растягиваем" столбец на два байта
		b |= (c & 0x02) * 6;
		b |= (c & 0x04) * 12;
		b |= (c & 0x08) * 24;

		c >>= 4;
		a  = (c & 0x01) * 3;
		a |= (c & 0x02) * 6;
		a |= (c & 0x04) * 12;
		a |= (c & 0x08) * 24;

		lcdVideoBuffer[lcdVideoBufferIndex]     = b;
		lcdVideoBuffer[lcdVideoBufferIndex+128] = a;
		lcdVideoBufferIndex = lcdVideoBufferIndex+1;
	}
	lcdVideoBuffer[lcdVideoBufferIndex++] = 0x00;
}
//*****************************************************************************
uint8_t Lcd_Print(char *txt){
  
	uint8_t i = 0;
	//--------------------
	ClearTextBuf();
	while(*txt)
	{
		Lcd_Chr(*txt++);
		i++;
	}
	return i;
}
//*****************************************************************************
uint8_t Lcd_PrintBold(char *txt){

	uint8_t i = 0;
	//--------------------
	ClearTextBuf();
	while(*txt)
	{
		Lcd_ChrBold(*txt++);
		i++;
	}
	return i;
}
//*****************************************************************************
uint8_t Lcd_PrintBig(char *txt){

	uint8_t i = 0;
	//--------------------
	ClearTextBuf();
	while(*txt)
	{
		Lcd_ChrBig(*txt++);
		i++;
	}
	return i;
}
//*****************************************************************************
uint32_t Lcd_BinToDec(uint32_t var, uint32_t num, uint32_t charSize){

	#define DEC_ARR_SIZE 10

	uint8_t  decArray[DEC_ARR_SIZE];
	uint32_t div = 1000000000;
	//uint8_t	 temp;
	void(*func)(char);
	//--------------------
	//Выбираем функцию для выводв символа
	 	 if(charSize == LCD_CHAR_SIZE_BIG) func = Lcd_ChrBig;
	else if(charSize == LCD_CHAR_SIZE_BOLD)func = Lcd_ChrBold;
	else								   func = Lcd_Chr;
	//Преобразование числа в строку.
	for(uint32_t i = DEC_ARR_SIZE; i > 0; i--)
	{
		decArray[i-1] = (uint8_t)(var/div);
		var %= div;
		div /= 10;
	}
	//Вывод десятичных разрядов числа.
	for(uint32_t i = 0; i < num; i++)
	{
		var = 0x30 + decArray[(num - 1) - i];
		(*func)((uint8_t)var);
	}
	//Возвращаем кол-во выведенных символов.
	return num+1;
}
//*****************************************************************************
uint32_t Lcd_BinToDecWithSign(int32_t var, uint32_t num, uint32_t charSize){

	void(*func)(char);
	//--------------------
	//Выбираем функцию для выводв символа
	 	 if(charSize == LCD_CHAR_SIZE_BIG) func = Lcd_ChrBig;
	else if(charSize == LCD_CHAR_SIZE_BOLD)func = Lcd_ChrBold;
	else					     		   func = Lcd_Chr;
	//Вывод знака
	if(var < 0)
	{
		var  = -var;
		(*func)('-');
	}
	else (*func)(' ');
	//Вывод десятичных разрядов числа. Возвращаем кол-во выведенных символов.
	return Lcd_BinToDec(var, num, charSize) + 1;
}
//*****************************************************************************
void Lcd_u8ToHex(uint8_t var){

	uint8_t ch;
	//--------------------
	ch = (var >> 4) & 0x0F;
	if(ch <= 9) ch += '0';
	else		ch += ('A' - 10);
	Lcd_Chr(ch);

	ch = var & 0x0F;
	if(ch <= 9) ch += '0';
	else		ch += ('A' - 10);
	Lcd_Chr(ch);
}
//*****************************************************************************
void Lcd_u32ToHex(uint32_t var){

	Lcd_Print("0x");
	Lcd_u8ToHex((uint8_t)((var & 0xFF000000) >> 24));
	Lcd_u8ToHex((uint8_t)((var & 0x00FF0000) >> 16));
	Lcd_u8ToHex((uint8_t)((var & 0x0000FF00) >> 8));
	Lcd_u8ToHex((uint8_t)( var & 0x000000FF));
}
//*****************************************************************************
void Lcd_PrintStringAndNumber(uint8_t cursor_x, uint8_t cursor_y, char *str, uint32_t number, uint32_t numDigit){

	Lcd_SetCursor(cursor_x, cursor_y);
	if(*str != '\0')  Lcd_Print(str);
	if(numDigit != 0) Lcd_BinToDec(number, numDigit, LCD_CHAR_SIZE_NORM);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************










