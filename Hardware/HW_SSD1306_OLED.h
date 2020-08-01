/*-----------------------------------------------------------------------
*@file     HW_SSD1306_OLED.h
*@brief    SSD1306为主控芯片的0.96"OLED显示器驱动程序头文件
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2019/05/08
-----------------------------------------------------------------------*/
#ifndef SSD1306_OLED_H
#define SSD1306_OLED_H

void OLED_DispToggle(uint8_t Stat);	//打开或关闭屏幕显示
void OLED_Clear(void);				//清屏
void OLED_ShowOneChar(uint8_t x,uint8_t y,char ch,uint8_t FontSize);
									//在指定位置显示一个ASCII字符
void OLED_ShowString(uint8_t x,uint8_t y,char *str,uint8_t FontSize);
									//在指定位置显示一个ASCII字符串
void OLED_ShowPattern16x16(uint8_t x,uint8_t y,uint8_t index);
									//在指定位置显示一个16x16的字模
void OLED_ShowBMP128x64(const unsigned char *BMP);
									//全屏幕显示一个128*64的位图
void OLED_Init(void);				//OLED初始化

extern const unsigned char Font8x16[];
extern const unsigned char Font6x8[][6];
extern const unsigned char HZK16x16[][16];
extern const unsigned char nBitmapDot[];

#define OLED_DISP_ON	1
#define OLED_DISP_OFF	0
#endif
