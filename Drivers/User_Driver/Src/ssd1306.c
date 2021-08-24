#include "ssd1306.h"
#include "font.h"

////////--------------command table-----------------/////////////
////////Fundamental Command Table
uint8_t ssd1306_SCC=  0x81;
uint8_t ssd1306_Entire_Off= 0xA4;
uint8_t ssd1306_Entire_On= 0xA5;
uint8_t ssd1306_Set_Normal= 0xA6;
uint8_t ssd1306_Set_Inverse= 0xA7;
uint8_t ssd1306_Set_Display_Off= 0xAE;
uint8_t ssd1306_Set_Display_ON= 0xAF;

////////Scrolling Command Table
uint8_t ssd1306_Horizontal_Scroll_Right= 0x26;
uint8_t ssd1306_Horizontal_Scroll_Left= 0x27;
//A[7:0]:0x00(dummy byte)
//B[2:0]:Define start page address
//C[2:0]:Set time interval between each scroll step in terms of frame frequency
//D[2:0]:Define end page address
//E[7:0]:0x00(dummy byte)
//F[7:0]:0xFF(dummy byte)

uint8_t ssd1306_V_H_Scroll_Right= 0x29; //vertical and horizontal scroll
uint8_t ssd1306_V_H_Scroll_Left= 0x2A;
//A[7:0]:0x00(dummy byte)
//B[2:0]:Define start page address
//C[2:0]:Set time interval between each scroll step in terms of frame frequency
//D[2:0]:Define end page address
//E[5:0]:Vertical scrolling offset

uint8_t ssd1306_Deactivate_scroll= 0x2E;

uint8_t ssd1306_Activate_scroll= 0x2F;

uint8_t ssd1306_Vertical_Scroll_Area= 0xA3; 
//A[5:0]:Set Number of rows in top fixed area
//B[6:0]:Set Number of rows in scroll area

////////Addressing Setting Command Table
uint8_t ssd1306_Set_Memory_ADDR_Mode= 0x20;
//A[1:0]:0x00->Horizontal, 0x01->Vertical, 0x10->Page, 0x11->Invalid

uint8_t ssd1306_Set_Page_START_ADDR= 0xB0;
//B0~B7(PAGE0~PAGE7)변경가능

////////Hardware Configuration Command Table
uint8_t ssd1306_Set_Display_Start_Line= 0x40;

uint8_t ssd1306_Set_Segment_Remap_Start= 0xA0;
uint8_t ssd1306_Set_Segment_Remap_End= 0xA1; //좌우반전

uint8_t ssd1306_Set_COM_Output_Direction_normal= 0xC0;
uint8_t ssd1306_Set_COM_Output_Direction_remap= 0xC8; //상하반전

uint8_t ssd1306_Set_Display_Offset= 0xD3;

uint8_t ssd1306_Set_Multiplex_Ratio= 0xA8;
//+A[5:0]

uint8_t ssd1306_Set_COM_Pins= 0xDA;
//+A[5:4]

/////////Timing & Driving Scheme Setting Command Table
uint8_t ssd1306_Set_Clock_Divide= 0xD5;

/////////Charge Pump Command Table
uint8_t ssd1306_Charge_Pump_Setting= 0x8D;

void ssd1306_W_Command(uint8_t cmd)
{
  uint8_t buf[2]={0};
  buf[0]=(0<<7)|(0<<6);
  buf[1]=cmd;

  if(HAL_I2C_Master_Transmit_DMA(&hi2c1,(uint16_t)ssd1306_ADDR,(uint8_t*)buf,2)!= HAL_OK)
  {
    Error_Handler();
  }
  while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READY);
}

void ssd1306_W_Data(uint8_t* data, uint16_t buf_size)
{
  if(HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)(ssd1306_ADDR),0x40,1,data,buf_size)!=HAL_OK)
  {
    Error_Handler();
  }
  while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READY);
}

void ssd1306_Horizontal_Scroll(void)
{
  uint8_t buf[8]={0};
  
  buf[0]=(0<<7)|(0<<6);
  buf[1]=ssd1306_Horizontal_Scroll_Right;
  buf[2]=0;                                                        //Dummy data:0x0
  buf[3]=0;                                                        //Start page address: PAGE0
  buf[4]=0;                                                        //Frame: 5Frames
  buf[5]=1;                                                        //End page address: PAGE1
  buf[6]=0;                                                        //Dummy data:0x0
  buf[7]=0xFF;                                                   //Dummy data:0xFF
  
  
   if(HAL_I2C_Master_Transmit_DMA(&hi2c1,(uint16_t)ssd1306_ADDR,(uint8_t*)buf,8)!= HAL_OK)
  {
    Error_Handler();
  }
  while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READY);
  ssd1306_W_Command(ssd1306_Activate_scroll);
}

void ssd1306_Init(void)
{
  ssd1306_W_Command(ssd1306_Set_Multiplex_Ratio);
  ssd1306_W_Command(0x3F);
  
  ssd1306_W_Command(ssd1306_Set_Display_Offset);
  
  ssd1306_W_Command(ssd1306_Set_Display_Start_Line);
  ssd1306_W_Command(ssd1306_Set_Segment_Remap_End);                   //좌우반전
  ssd1306_W_Command(ssd1306_Set_COM_Output_Direction_remap);        //상하반전
  
  ssd1306_W_Command(ssd1306_Set_COM_Pins);
  ssd1306_W_Command(0x12);
  
  ssd1306_W_Command(ssd1306_SCC);
  ssd1306_W_Command(0x7F);
  
  ssd1306_W_Command(ssd1306_Set_Normal);
  
  ssd1306_W_Command(ssd1306_Set_Clock_Divide);
  ssd1306_W_Command(0x80);
  
  ssd1306_W_Command(ssd1306_Charge_Pump_Setting);
  ssd1306_W_Command(0x14);
  
  ssd1306_W_Command(ssd1306_Set_Display_ON);
}

void ssd1306_Clear(void)
{
  uint8_t buf[128]={0};
  
  ssd1306_W_Command(0x00);
  ssd1306_W_Command(0x10);      //데이터 입력으로 변경
  
  for(uint8_t i = 0; i<8; i++)
  {
    ssd1306_W_Command(ssd1306_Set_Page_START_ADDR+i);
    ssd1306_W_Data(buf,128);
  }
}

void ssd1306_Set_Coord(uint8_t page, uint8_t col)
{
  uint8_t col_low=0x0F,col_high=0x1F;
  col_low=(col&0x0F);
  col_high=0x10|((col>>4)&0x0F);
  ssd1306_W_Command(ssd1306_Set_Page_START_ADDR+page);
  ssd1306_W_Command(col_low);
  ssd1306_W_Command(col_high);
}

//uint8_t font_width=12;
void ssd1306_W_Char(uint8_t Character, uint8_t page, uint16_t col)
{
  
  uint8_t char_Buf[font_width*2];
  
  for(uint8_t i=0;i<font_width*2;i++)
  {
    char_Buf[i]=ssd1306_Fonts[(Character-32)*(font_width*2)+i];
  }
  
  for(uint8_t i=0;i<2;i++)
  {
    ssd1306_Set_Coord(page+i,col);
    ssd1306_W_Data(&char_Buf[i*font_width],font_width);
  }
}

void ssd1306_W_On_Heart(uint8_t page, uint16_t col)
{
  
  uint8_t char_Buf[17*4];
  
  for(uint8_t i=0;i<17*4;i++)
  {
    char_Buf[i]=heartBitmaps[i];
  }
  
  for(uint8_t i=0;i<4;i++)
  {
    ssd1306_Set_Coord(page+i,col);
    ssd1306_W_Data(&char_Buf[i*17],17);
  }
}

void ssd1306_W_Off_Heart(uint8_t page, uint16_t col)
{
  
  uint8_t char_Buf[17*4];
  
  for(uint8_t i=0;i<17*4;i++)
  {
    char_Buf[i]=heartBitNomaps[i];
  }
  
  for(uint8_t i=0;i<4;i++)
  {
    ssd1306_Set_Coord(page+i,col);
    ssd1306_W_Data(&char_Buf[i*17],17);
  }
}

void ssd1306_W_String(char *str, uint8_t page, uint8_t col)
{
        
	while(*str)
	{
		if((col+font_width)>127)
		{
			if(page==6)
			{
				break;
			}
			page+=2;
			col=0;
		}
		ssd1306_W_Char(*str,page,col);

		col+=font_width;
		str++;
	}
}