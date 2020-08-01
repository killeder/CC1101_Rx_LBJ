/*-----------------------------------------------------------------------
*@file     Hardware.h
*@brief    底层硬件的总体头文件,在需要包含硬件头文件时包含此文件即可。
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/07/19
-----------------------------------------------------------------------*/
#ifndef HARDWARE_H
#define HARDWARE_H
/*-------------芯片级支持库-------------*/
#include "stm32f10x.h"
/*-------------C语言标准库--------------*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
/*---------引用自外部的其他模块---------*/
#include "delay.h"	//基于Systick的公用延时功能，在App/Utils层中定义口
/*------各个底层硬件驱动程序头文件集合----*/
#include "HW_USART.h"	//串行口
#include "HW_TIM.h"	//定时器
#include "HW_GPIO.h"	//指示灯和蜂鸣器
#include "HW_RADIO_CC1101.h"	//CC1101无线芯片驱动程序
#include "HW_IIC_SoftSimulate.h"	//通过GPIO口模拟IIC总线程序
#include "HW_SSD1306_OLED.h"	//OLED显示器驱动程序
/*-----------宏定义和一些参数------------*/
#define CRIS_ENTER()	__set_PRIMASK(1)	//进入临界区，关闭总中断
#define CRIS_EXIT()		__set_PRIMASK(0)	//退出临界区，开启总中断
#endif
