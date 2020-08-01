/*-----------------------------------------------------------------------
*@file     system_utils.h
*@brief    系统层实用功能
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/7/27
-----------------------------------------------------------------------*/
#ifndef SYSTEM_UTILS_H
#define SYSTEM_UTILS_H

void ParseSerialCmdLine(char *Rxbuff);	//解析串口命令行
void CC1101_Initialize(void);	//检测CC1101并给出报告
void ShowBuildInfo(void);	//串口打印版本信息
void ShowSettings(void);	//串口打印设置项目
void ShowMessageLBJ(POCSAG_RESULT* POCSAG_Msg,float rssi,uint8_t lqi);//OLED屏幕上显示LBJ解码信息
void ShowSplashScreen(void); //OLED显示开机画面和版本信息
void ShowFixedPattern(void); //OLED上显示固定字符(“车次”、“速度”等汉字)
void Rx_Callback(void);	//CC1101数据包接收完成时的回调函数
void RxDataFeedProc(void);	//读取已接收的数据并处理和显示

extern volatile bool bDataArrivalFlag;	//标识数据到来标志

#endif
