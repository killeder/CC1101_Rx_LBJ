/*-----------------------------------------------------------------------
*@file     POCSAG_ParseLBJ.h
*@brief    POCSAG寻呼码编码的列车接近预警信息解析程序
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/07/27
-----------------------------------------------------------------------*/
#ifndef POCSAG_PARSE_H
#define POCSAG_PARSE_H

//TB/T3504-2018《列车接近预警地面设备》中定义的表示上下行的功能码
#define FUNC_UNDEFINED		-1	//功能码未定义
#define FUNC_XIAXING		 1	//功能码1:下行，蜂鸣器响一声
#define FUNC_SHANGXING		 3	//功能码3:上行，蜂鸣器响两声
//TB/T3504-2018《列车接近预警地面设备》中定义的不同功能地址信息
#define LBJ_MESSAGE_ADDR	1234000	//列车接近预警在地址1234000发送
#define LBJ_TIMESYNC_ADDR	1234008 //机车CIR时间同步信息在地址1234008发送

typedef struct{
	uint32_t Address;	//地址
	int8_t FuncCode;	//功能码
	char txtMsg[41];	//解码出的文本消息，最长40个字符
}POCSAG_RESULT;	//保存解析结果

//定义若干错误码
#define POCSAG_ERR_NONE					0
#define POCSAG_ERR_ADDR_NOTFOUND		-1
#define POCSAG_ERR_FUNC_NOTFOUND		-2
#define POCSAG_ERR_PARITY_NOTMATCH		-3

int8_t POCSAG_ParseCodeWordsLBJ(POCSAG_RESULT* Parse_Result, uint8_t* Batch_data,
							 uint8_t Batch_len, bool Invert);//POCSAG编码的列车接近预警消息解析子程序

#ifdef POCSAG_DEBUG_MSG_ON
	    #define POCSAG_DEBUG_MSG(...) printf(__VA_ARGS__)
#else
		#define POCSAG_DEBUG_MSG(...)
#endif

#endif
