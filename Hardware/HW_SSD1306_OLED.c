/*-----------------------------------------------------------------------
*@file     HW_SSD1306_OLED.c
*@brief    SSD1306为主控芯片的0.96"OLED显示器驱动程序
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2019/05/08
-----------------------------------------------------------------------*/
#include "Hardware.h"

#define OLED_WR_DATA	1
#define OLED_WR_CMD		0
#define OLED_IIC_ADDR	0x78//不同型号OLED模组的地址不同
/*-----------------------------------------------------------------------
*@func	   OLED_WriteCommand
*@brief    向OLED写入指令
*@param	   IIC_Command 要写入的指令码
*@return   无
-----------------------------------------------------------------------*/
static void OLED_WriteCommand(uint8_t IIC_Command)
{
	IIC_Start();
   	IIC_SendOneByte(OLED_IIC_ADDR);	//SSD1306的设备地址
	IIC_WaitAck();	
   	IIC_SendOneByte(0x00);	//写指令
	IIC_WaitAck();	
   	IIC_SendOneByte(IIC_Command); 
	IIC_WaitAck();	
  	IIC_Stop();
}
/*-----------------------------------------------------------------------
*@func	   OLED_WriteData
*@brief    向OLED写入数据
*@param	   IIC_Data 要写入的数据
*@return   无
-----------------------------------------------------------------------*/
static void OLED_WriteData(uint8_t IIC_Data)
{
	IIC_Start();
   	IIC_SendOneByte(OLED_IIC_ADDR);	//SSD1306的设备地址
	IIC_WaitAck();	
   	IIC_SendOneByte(0x40);	//写数据
	IIC_WaitAck();	
   	IIC_SendOneByte(IIC_Data); 
	IIC_WaitAck();	
  	IIC_Stop();
}
/*-----------------------------------------------------------------------
*@func	   OLED_WriteByte
*@brief    向OLED写入数据或指令
*@param	   Data 要写入的数据或指令
*@param    DC 决定写入数据还是指令
*  @val    1 写入数据
*  @val	   0 写入指令 
*@return   无
-----------------------------------------------------------------------*/
static void OLED_WriteByte(uint8_t Data,uint8_t DC)
{
	if(DC)
		OLED_WriteData(Data);
	else
		OLED_WriteCommand(Data);
}
/*-----------------------------------------------------------------------
*@func	   OLED_SetPos
*@brief    设置OLED显示游标位置
*@param	   x 横向坐标位置，范围0-127 
*@param	   y 纵向坐标位置，即页面编号，范围0-7
*@return   无
-----------------------------------------------------------------------*/
static void OLED_SetPos(uint8_t x,uint8_t y)
{
	OLED_WriteByte(0xb0+y,OLED_WR_CMD);
	OLED_WriteByte(((x&0xf0)>>4)|0x10,OLED_WR_CMD);
	OLED_WriteByte((x&0x0f),OLED_WR_CMD); 
}
/*-----------------------------------------------------------------------
*@func	   OLED_DispToggle
*@brief    打开或关闭显示屏
*@param	   Stat 切换状态
*  @val	   1 打开显示
*  @val    0 关闭显示
*@return   无
-----------------------------------------------------------------------*/
void OLED_DispToggle(uint8_t Stat)
{
	OLED_WriteByte(0X8D,OLED_WR_CMD);  //SET DCDC命令
	if(Stat)
	{
		OLED_WriteByte(0X14,OLED_WR_CMD);  //DCDC ON
		OLED_WriteByte(0XAF,OLED_WR_CMD);  //DISPLAY ON	
	}
	else
	{
		OLED_WriteByte(0X10,OLED_WR_CMD);  //DCDC OFF
		OLED_WriteByte(0XAE,OLED_WR_CMD);  //DISPLAY OFF			
	}
}
/*-----------------------------------------------------------------------
*@func	   OLED_Clear
*@brief    清屏
*@return   无
-----------------------------------------------------------------------*/
void OLED_Clear(void)
{
	uint8_t  i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WriteByte(0xb0+i,OLED_WR_CMD);    //设置页地址（0~7）
		OLED_WriteByte(0x00,OLED_WR_CMD);      //设置显示位置―列低地址
		OLED_WriteByte(0x10,OLED_WR_CMD);      //设置显示位置―列高地址   
		for(n=0;n<128;n++)
			OLED_WriteByte(0,OLED_WR_DATA);	 //更新显示 
	}
}
/*-----------------------------------------------------------------------
*@func	   OLED_ShowOneChar
*@brief    在指定位置显示一个ASCII字符
*@param	   x 横向坐标0-127
*@param    y 纵向坐标0-7
*@param    ch 要显示的ASCII字符
*@param	   FontSize 使用16/12点字模
*@return   无
-----------------------------------------------------------------------*/
void OLED_ShowOneChar(uint8_t x,uint8_t y,char ch,uint8_t FontSize)
{
	char c = ch - 0x20;	//点阵字模内的数据是从空格符开始的
	uint8_t i;

	x = x % 128;	//确保x和y坐标不超出范围
	y = y % 8;

	if(FontSize == 16)	//如果选用16点字模(横8点，纵16点)
	{
		OLED_SetPos(x,y);
		for(i = 0;i < 8;i++)
			OLED_WriteByte(Font8x16[c*16+i],OLED_WR_DATA);//先写入上半个字从左到右
		OLED_SetPos(x,y+1);
		for(i = 0;i < 8;i++)
			OLED_WriteByte(Font8x16[c*16+8+i],OLED_WR_DATA);//再写入下半个字从左到右		
	}
	else if(FontSize == 12)	//如果选用12点字模(横6点，纵8点)
	{
		OLED_SetPos(x,y);
		for(i = 0;i < 6;i++)
			OLED_WriteByte(Font6x8[c][i],OLED_WR_DATA);
	}
}
/*-----------------------------------------------------------------------
*@func	   OLED_ShowString
*@brief    在指定位置显示一个ASCII字符串
*@param	   x 横向坐标0-127
*@param    y 纵向坐标0-7
*@param    *str 指向字符串的指针
*@param	   FontSize 使用16/12点字模
*@return   无
-----------------------------------------------------------------------*/
void OLED_ShowString(uint8_t x,uint8_t y,char *str,uint8_t FontSize)
{
	while(*str)
	{
		OLED_ShowOneChar(x,y,*str,FontSize);
		x += 8;
		if(x>120)
		{
			x = 0;
			y += 2;
		}
		str++;
	}
}
/*-----------------------------------------------------------------------
*@func	   OLED_ShowPattern16x16
*@brief    在指定位置显示一个16x16的字模，例如汉字或自定义的特殊符号
*@param	   x 横向坐标0-127
*@param    y 纵向坐标0-7
*@param    index 要显示的字模在字模数组中的索引号
*@return   无
-----------------------------------------------------------------------*/
void OLED_ShowPattern16x16(uint8_t x,uint8_t y,uint8_t index)
{
	uint8_t i;
	OLED_SetPos(x,y);
	for(i = 0;i< 16;i++)
		OLED_WriteByte(HZK16x16[2*index][i],OLED_WR_DATA);	//先显示上半个字模，从左到右
	OLED_SetPos(x,y+1);
	for(i = 0;i < 16;i++)
		OLED_WriteByte(HZK16x16[2*index+1][i],OLED_WR_DATA);	//后显示下半个字模，从左到右	
}
/*-----------------------------------------------------------------------
*@func	   OLED_ShowBMP128x64
*@brief    全屏幕显示一个128x64的位图，取模方式：列行式，高位在前
*@param    *BMP 指向位图数据的指针
*@return   无
-----------------------------------------------------------------------*/
void OLED_ShowBMP128x64(const unsigned char *BMP)
{
	uint8_t x,y;
	for(y = 0;y < 8;y++)
	{
		OLED_SetPos(0,y);
		for(x = 0;x < 128;x++)
			OLED_WriteByte(*BMP++,OLED_WR_DATA);
	}	
}
/*-----------------------------------------------------------------------
*@func	   OLED_Init
*@brief    OLED初始化
*@return   无
-----------------------------------------------------------------------*/
void OLED_Init(void)
{
	OLED_WriteByte(0xAE,OLED_WR_CMD);//--display off
	OLED_WriteByte(0x00,OLED_WR_CMD);//---set low column address
	OLED_WriteByte(0x10,OLED_WR_CMD);//---set high column address
	OLED_WriteByte(0x40,OLED_WR_CMD);//--set start line address  
	OLED_WriteByte(0xB0,OLED_WR_CMD);//--set page address
	OLED_WriteByte(0x81,OLED_WR_CMD); // contract control
	OLED_WriteByte(0xFF,OLED_WR_CMD);//--128   
	OLED_WriteByte(0xA0,OLED_WR_CMD);//set segment remap (使用白色屏幕时由A1改成A0)
	OLED_WriteByte(0xA6,OLED_WR_CMD);//--normal / reverse
	OLED_WriteByte(0xA8,OLED_WR_CMD);//--set multiplex ratio(1 to 64)
	OLED_WriteByte(0x3F,OLED_WR_CMD);//--1/32 duty
	OLED_WriteByte(0xC0,OLED_WR_CMD);//Com scan direction（使用白色屏幕时由C8改成C0）
	OLED_WriteByte(0xD3,OLED_WR_CMD);//-set display offset
	OLED_WriteByte(0x00,OLED_WR_CMD);//
	
	OLED_WriteByte(0xD5,OLED_WR_CMD);//set osc division
	OLED_WriteByte(0xF0,OLED_WR_CMD);//由0x80改为0xF0,提高刷新频率
	
	OLED_WriteByte(0xD8,OLED_WR_CMD);//set area color mode off
	OLED_WriteByte(0x05,OLED_WR_CMD);//
	
	OLED_WriteByte(0xD9,OLED_WR_CMD);//Set Pre-Charge Period
	OLED_WriteByte(0xF1,OLED_WR_CMD);//
	
	OLED_WriteByte(0xDA,OLED_WR_CMD);//set com pin configuartion
	OLED_WriteByte(0x12,OLED_WR_CMD);//
	
	OLED_WriteByte(0xDB,OLED_WR_CMD);//set Vcomh
	OLED_WriteByte(0x30,OLED_WR_CMD);//
	
	OLED_WriteByte(0x8D,OLED_WR_CMD);//set charge pump enable
	OLED_WriteByte(0x14,OLED_WR_CMD);//
	
	OLED_WriteByte(0xAF,OLED_WR_CMD);//--turn on oled panel
	OLED_Clear();	//初始化后清屏
}
