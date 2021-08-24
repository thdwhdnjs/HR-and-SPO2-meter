#ifndef ssd1306_H_
#define ssd1306_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define ssd1306_ADDR 0x3C<<1
#define font_width 12

extern I2C_HandleTypeDef hi2c1;
extern void Error_Handler(void);



void ssd1306_Init(void);
void ssd1306_W_Command(uint8_t cmd);
void ssd1306_W_Data(uint8_t* data, uint16_t buf_size);
void ssd1306_Clear(void);
void ssd1306_Set_Coord(uint8_t page, uint8_t col);
void ssd1306_W_Char(uint8_t Character, uint8_t page, uint16_t col);
void ssd1306_W_String(char *str, uint8_t page, uint8_t col);
void ssd1306_Horizontal_Scroll(void);
void ssd1306_W_On_Heart(uint8_t page, uint16_t col);
void ssd1306_W_Off_Heart(uint8_t page, uint16_t col);
#endif