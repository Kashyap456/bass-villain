/*
 * LCD_GFX.c
 *
 * Created: 9/20/2021 6:54:25 PM
 *  Author: You
 */ 

#include "LCD_GFX.h"
#include "ST7735.h"
#define _USE_MATH_DEFINES
#include <math.h>

/******************************************************************************
* Local Functions
******************************************************************************/



/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn			uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
* @brief		Convert RGB888 value to RGB565 16-bit color data
* @note
*****************************************************************************/
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
	return ((((31*(red+4))/255)<<11) | (((63*(green+2))/255)<<5) | ((31*(blue+4))/255));
}

/**************************************************************************//**
* @fn			void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color)
* @brief		Draw a single pixel of 16-bit rgb565 color to the x & y coordinate
* @note
*****************************************************************************/
void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color) {
	LCD_setAddr(x,y,x,y);
	SPI_ControllerTx_16bit(color);
}

/**************************************************************************//**
* @fn			void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor)
* @brief		Draw a character starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor){
	uint16_t row = character - 0x20;		//Determine row of ASCII table starting at space
	int i, j;
	if ((LCD_WIDTH-x>7)&&(LCD_HEIGHT-y>7)){
		for(i=0;i<5;i++){
			uint8_t pixels = ASCII[row][i]; //Go through the list of pixels
			for(j=0;j<8;j++){
				if ((pixels>>j)&1 == 1){
					LCD_drawPixel(x+i,y+j,fColor);
				}
				else {
					LCD_drawPixel(x+i,y+j,bColor);
				}
			}
		}
	}
}


/******************************************************************************
* LAB 4 TO DO. COMPLETE THE FUNCTIONS BELOW.
* You are free to create and add any additional files, libraries, and/or
*  helper function. All code must be authentically yours.
******************************************************************************/

/**************************************************************************//**
* @fn			void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
* @brief		Draw a colored circle of set radius at coordinates
* @note
*****************************************************************************/
void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
{
    int r2 = radius*radius;
    for (int yl = radius; yl >= (radius*-1); yl--) {
        int offset = 0;
        int diff = r2 - yl*yl;
        int x = 0;
        while (x*x <= diff) {
            offset += 1;
            x++;
        }
        LCD_setAddr(x0-offset+1,y0+yl,x0+offset-1,y0+yl);
        for (int i = 0; i < 2*radius+4; i++) {
            SPI_ControllerTx_16bit(color);
       }
    }
//    int x;
//    int y;
//    for (double th = 0; th < 2*M_PI; th+=0.3) {
//        x = (int) (radius * cos(th));
//        y = (int) (radius * sin(th));
//        if (x0+x < x0) {
//            LCD_setAddr(x0+x,y+y0,x0-x,y+y0);
//        } else {
//            LCD_setAddr(x0-x,y+y0,x0+x,y+y0);
//        }
//        for (int i = 0; i < 2*radius; i++) {
//            SPI_ControllerTx_16bit(color);
//        }
//    }
	// Fill this out
}


/**************************************************************************//**
* @fn			void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
* @brief		Draw a line from and to a point with a color
* @note
*****************************************************************************/
void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
{
	// Fill this out
    if (fabs(x1-x0) < fabs(y1-y0)) {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int diff = 2 * dx - dy;
        int x = x0;
        for (int y = y0; y <= y1; y++) {
            LCD_drawPixel(x, y, c);
            if (diff > 0) {
                x += (dx < 0) ? -1 : 1;
                diff -= 2 * dy;
            }
            diff += 2 * dx;
        }
    } else {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int diff = 2 * dy - dx;
        int y = y0;
        for (int x = x0; x <= x1; x++) {
            LCD_drawPixel(x, y, c);
            if (diff > 0) {
                y += (dy < 0) ? -1 : 1;
                diff -= 2 * dx;
            }
            diff += 2 * dy;
        }
    }
}



/**************************************************************************//**
* @fn			void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
* @brief		Draw a colored block at coordinates
* @note
*****************************************************************************/
void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
{
	// Fill this out
    LCD_setAddr(x0,y0,x1,y1);
    for (int x = 0; x < LCD_WIDTH; x++) {
        for (int y = 0; y < LCD_HEIGHT; y++) {
            SPI_ControllerTx_16bit(color);
        }
    }
}

/**************************************************************************//**
* @fn			void LCD_setScreen(uint16_t color)
* @brief		Draw the entire screen to a color
* @note
*****************************************************************************/
void LCD_setScreen(uint16_t color) 
{
    LCD_setAddr(0,0,LCD_WIDTH,LCD_HEIGHT);
    for (int x = 0; x < LCD_WIDTH; x++) {
        for (int y = 0; y < LCD_HEIGHT; y++) {
            SPI_ControllerTx_16bit(color);
        }
    }
	// Fill this out
}

/**************************************************************************//**
* @fn			void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
* @brief		Draw a string starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
{
    while (*str != '\0') {
        LCD_drawChar(x, y, *(str), fg, bg);
        str++;
        x+=5;
    }
	// Fill this out
}