/*-----------------------------------------------------------------------
*@file     POCSAG_ParseLBJ.c
*@brief    POCSAG寻呼码编码的列车接近预警信息解析程序
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/07/27
-----------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include "POCSAG_ParseLBJ.h"
/*-----------------------------------------------------------------------
*@brief		翻转BCD位顺序
*@param		bcd - 要转换的BCD码
*@retval	转换结果
-----------------------------------------------------------------------*/
static uint8_t FlipBCDbitOrder(uint8_t bcd)
{
	uint8_t result = 0;

	for(uint8_t i = 4;i > 0;i--)//i=4->1,因此i-1=3->0
	{
		if(bcd & (1<<(i-1)))	//bit3->bit0遍历
			result |= 1<<(3-(i-1));//bit0-bit3判断是否写1
	}
	return result;
}
/*-----------------------------------------------------------------------
*@brief		BCD转字符
*@param		bcd - 要转换的BCD码
*@retval	转换结果
-----------------------------------------------------------------------*/
static char BCD2Char(uint8_t bcd)
{
	char ch = 0;
	const char punc[6] = {'*','U','\x20','-','(',')'};//与0x0A-0x0F对应

	if(bcd <= 9)
		ch = bcd + 0x30;
	else if(bcd >= 0x0A && bcd <= 0x0F)
		ch = punc[bcd - 0x0A];
	return(ch);
}
/*-----------------------------------------------------------------------
*@brief		检查偶校验结果
*@param		n - 待检查的数据
*@retval	偶校验检查成功与否：true:成功，false:检查未通过
-----------------------------------------------------------------------*/
static bool CheckEvenParity(uint8_t n)
{
	uint8_t parity = 0;	//“1”的个数计数

	for(uint8_t bit = 1;bit <= 32;bit++,n<<=1)
	{
		if(n & 0x80000000)
			parity++;
	}
	if(parity%2 == 0)
		return true;	//如果总共有偶数个1则校验通过
	else
		return false;	//否则校验不通过
}
/*-----------------------------------------------------------------------
*@brief		POCSAG编码的列车接近预警信息码字解析子程序
*@param		Parse_Result - 指向存储解析结果结构体的指针
*           Batch_data - 指向码组数据的指针，码组数据包含若干码字，
*           每个码字4字节
*           Batch_len - 码组数据的长度，为组内码字数*4
*           Invert - 是否将原始码组数据反相后解析
*@retval	错误码，在头文件中定义，无错误时返回POCSAG_ERR_NONE(0)
-----------------------------------------------------------------------*/
int8_t POCSAG_ParseCodeWordsLBJ(POCSAG_RESULT* Parse_Result, uint8_t* Batch_data,
							 uint8_t Batch_len, bool Invert)
{
	int8_t State_code = POCSAG_ERR_NONE;	//储存每一步的错误码
	uint8_t Bytecount_In = 0;	//已读入原始数据字节计数
	char DecodedText[41] = {0};	//暂存已解码的数据
	uint8_t Decoded_len = 0;	//已解码的数据长度
	uint32_t CodeWord_Item = 0;//储存每一个合成的码字
	uint32_t AddrCode = 0;	//从码字中读取的地址码
	int8_t FuncCode = FUNC_UNDEFINED;//从码字中读取的功能码，初始化为"未定义"

	if(Invert)	//如果要求先反相再解析
	{
		for(uint8_t i = 0;i < Batch_len;i++)
			Batch_data[i] ^= 0xFF;	//则先将原始数据每一位颠倒
		POCSAG_DEBUG_MSG("Bit of POCSAG batch "
						 "is inverted to normal.\r\n");
	}
	POCSAG_DEBUG_MSG("Processing %u received codewords...\r\n",
					  Batch_len/4);		
	//原始Batch数据每4个字节捏成一组，作为一个码字
	for(Bytecount_In = 0;Bytecount_In < Batch_len;Bytecount_In += 4)
	{
		CodeWord_Item = (uint32_t)((Batch_data[Bytecount_In]<<24)|
								  (Batch_data[Bytecount_In+1]<<16)|
								  (Batch_data[Bytecount_In+2]<<8)|
								  (Batch_data[Bytecount_In+3]));
		if((CodeWord_Item == 0)||(CodeWord_Item == 0x7A89C197))
		{
			POCSAG_DEBUG_MSG("Found IDLE codeword at batch[%u].\r\n",
							  Bytecount_In/4);
			continue;	//如果是空码字或空闲码字，就舍弃并合成下一个码字
		}
		if(!CheckEvenParity(CodeWord_Item))//如果偶校验失败
		{
			State_code |= POCSAG_ERR_PARITY_NOTMATCH;//标记“偶校验失败”
			POCSAG_DEBUG_MSG("Codeword parity check failed at batch[%u].\r\n",
								Bytecount_In/4);
			continue;	//舍弃并合成下一个码字
		}
		if(!(CodeWord_Item&0x80000000))
		{				//如果如果第31位为0,解析地址码字
			AddrCode = CodeWord_Item >> 11;	//右移动11位，以去掉BCH和偶校验位
			FuncCode = AddrCode & 0x3;	//AddrCode低2位代表功能码
			AddrCode >>= 2;	//右移2位去掉功能码，保留高18位地址码
			AddrCode <<= 3;	//左移3位，在低位补0，为地址码代表起始帧的低3位留地方
			AddrCode |= (Bytecount_In>>1)&0x7;	//添加地址低3位
			AddrCode &= 0x1FFFFF;	//限定地址码为21位
			POCSAG_DEBUG_MSG("Found address codeword at batch[%u]:",
							  Bytecount_In/4);
			POCSAG_DEBUG_MSG("Addr:%u,Func:%hhd.\r\n",AddrCode,FuncCode);
		}
		else	//如果第31位为1,是消息码字
		{
			POCSAG_DEBUG_MSG("Found message codeword at batch[%u]:",Bytecount_In/4);
			//列车接近预警在地址1234000发送,时间同步信息在地址1234008发送
			if((AddrCode == LBJ_MESSAGE_ADDR)||(AddrCode == LBJ_TIMESYNC_ADDR))
			{
				POCSAG_DEBUG_MSG("Type:BCD,Data:");
				CodeWord_Item = (CodeWord_Item>>11)&0xFFFFF;
									//右移11位去掉BCH和偶校验位，然后只保留低20位
				for(uint8_t i = 5; i > 0;i--)//i=5->1
				{
					uint32_t bcd_decode = CodeWord_Item & (0x0000F<<(i-1)*4);
					uint8_t bcd_target;

					bcd_decode >>= (i-1)*4;//i-1=4->0
					//此时“bcd_decode”低4位的BCD码和我们想要的BCD码在位顺序上正好相反。
					//因此调用"FlipBCDbitOrder()"解决此问题
					bcd_target = FlipBCDbitOrder((uint8_t)bcd_decode);	//翻转位顺序
					if(Decoded_len < 40)//防止下标越界，并给字符串末尾的\0留地方
						DecodedText[Decoded_len++] = BCD2Char(bcd_target);
					POCSAG_DEBUG_MSG("%hhX",bcd_target);
				}	
			}
			else
				POCSAG_DEBUG_MSG("Addr code mismatch!");
			POCSAG_DEBUG_MSG("\r\n");
		}
	}

	if(AddrCode == 0)	//如果至今没发现地址码
		State_code |= POCSAG_ERR_ADDR_NOTFOUND;//标记“地址码未发现”
	if(FuncCode == FUNC_UNDEFINED)	//如果至今没发现功能码
		State_code |= POCSAG_ERR_FUNC_NOTFOUND;//标记“功能码未发现”

	if(State_code == POCSAG_ERR_NONE)	//如果以上执行步骤没发生错误
	{
		Parse_Result->Address = AddrCode;	//保存地址码
		Parse_Result->FuncCode = FuncCode;	//保存功能码
		DecodedText[Decoded_len++] = '\0';	//字符串末尾填\0
		memcpy(Parse_Result->txtMsg,DecodedText,Decoded_len);//拷贝解码结果
	}
	return(State_code);//返回错误码
}
