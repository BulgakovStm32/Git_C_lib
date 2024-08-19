/*
 * LcdGraphic.h
 *
 *  Created on: 
 *      Author: belyaev
 */
#ifndef _LcdGraphic_H
#define _LcdGraphic_H
//*******************************************************************************************
//*******************************************************************************************
//#include "lcd12864.h"
#include "LM6063D.h"
//#include "Lcd_TIC32.h"
//#include "ssd1306.h"
//*******************************************************************************************
//*******************************************************************************************
#define LCD_X_RES 			   128	 //разрешение экрана
#define LCD_Y_RES 			   64
#define LCD_VIDEO_BUFFER_SIZE  ((LCD_X_RES * LCD_Y_RES) / 8)  //
#define LCD_TEXT_BUFFER_SIZE   22
//
#define	X0					0xb8
#define	Y0					0x40
//режимы отображения пикселя - используются в графических функциях
#define PIXEL_OFF	        0
#define PIXEL_ON	        1
#define PIXEL_XOR	        2
//режимы отображения символов - используются в функциях вывода текстов.
#define LCD_CHAR_SIZE_NORM	0
#define LCD_CHAR_SIZE_BIG	1
#define LCD_CHAR_SIZE_BOLD	2
//*******************************************************************************************
//*******************************************************************************************
void     Lcd_Init		     (void);
void     Lcd_Update          (void);
uint8_t* Lcd_pVideoBuffer    (void);

void 	Lcd_Filling(uint8_t byte);
void    Lcd_ClearVideoBuffer(void);

void 	Lcd_Pixel (uint8_t x, uint8_t y, uint8_t mode);
void 	Lcd_Line  (int x1, int y1, int x2, int y2, uint8_t mode);
void 	Lcd_Circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t mode);
void 	Lcd_Bar   (int x1, int y1, int x2, int y2, uint8_t persent);
void 	Lcd_Image (const uint8_t *imageData);

void 	Lcd_GotoXYFont(uint8_t x, uint8_t y);
void 	Lcd_SetCursor (uint8_t x, uint8_t y);
void 	Lcd_Chr       (char ch);
void 	Lcd_ChrBold   (char ch);
void 	Lcd_ChrBig    (char ch);
uint8_t Lcd_Print    (char *txt);
uint8_t Lcd_PrintBold(char *txt);
uint8_t Lcd_PrintBig (char *txt);

uint32_t Lcd_BinToDec        (uint32_t var, uint32_t num, uint32_t charSize);
uint32_t Lcd_BinToDecWithSign( int32_t var, uint32_t num, uint32_t charSize);

void     Lcd_u8ToHex (uint8_t hexChar);
void     Lcd_u32ToHex(uint32_t varHex);
void 	 Lcd_PrintStringAndNumber(uint8_t cursor_x, uint8_t cursor_y, char *str, uint32_t number, uint32_t numDigit);

//*******************************************************************************************
//*******************************************************************************************
#endif /*_LcdGraphic_H*/



