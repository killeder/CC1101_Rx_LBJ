/*-----------------------------------------------------------------------
*@file     system_utils.c
*@brief    系统层实用功能
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/7/27
-----------------------------------------------------------------------*/
#include "CC1101_Rx_LBJ.h"

float Rf_Freq = 821.2375f;		//接收频率821.2375MHz
volatile bool bDataArrivalFlag = false;	//标识数据到来标志
/*-----------------------------------------------------------------------
*@brief		解析串口命令行
*@param		指向串行口数据缓冲区
*@retval	无
-----------------------------------------------------------------------*/
void ParseSerialCmdLine(char *Rxbuff)
{
	char *pos;

	if(Rxbuff[0] == '$')
	{
		if(Rxbuff[1] == '\0')	//$显示帮助信息
		{
			MSG("$ (View this help tips again)\r\n"
				"$$ (List current settings)\r\n"
				"$V (View version info)\r\n"
				"$F=xxx.xxxx (Setting frenquency to xxx.xxxx MHz)\r\n");
		}
		else if(Rxbuff[1] == '$')//$$打印设置项目
		{
			ShowSettings();			//串口打印设置项目
		}
		else if(Rxbuff[1] == 'V')//$V显示版本信息
		{
			ShowBuildInfo();		//串口打印版本信息	
		}		
		else if(Rxbuff[1] == 'F')//$F=xxx.xxxx设置接收频率
		{
			pos = strchr(Rxbuff,'=') + 1;	//确定等号后的第一个字符的位置
			if(sscanf(pos,"%f",&Rf_Freq)!=1)	//如果输入有误
			{	
				MSG("Wrong RF frenquency formate.\r\n");
			}
			else
			{
				MSG("RF freq was set to %f MHz.\r\n",Rf_Freq);
				CC1101_Initialize();	//重新初始化设置CC1101
				CC1101_StartReceive(Rx_Callback);	//重新开始接收
			}
		}
		else
			MSG("Unsupported command type.\r\n");
		BeeperMode = BEEP_ONCE;	//处理完命令后响一声
	}
	else
		MSG("Wrong Command Format! Type '$' for help.\r\n");
}
/*-----------------------------------------------------------------------
*@brief		检测CC1101并给出报告
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void CC1101_Initialize(void)
{
	int8_t cc1101_state;	//设置CC1101时返回的状态码
	uint8_t delay_count = 0;	//延时计数

	MSG("CC1101 Initializing...\r\n");
	//1200bps 2FSK频偏4.5khz 接收机带宽58.0kHz 前导码16字节
	//固定包长度64字节，不允许同步字有位错误，启用载波检测，关闭CRC过滤
	//同步字0x15D8（标准POCSAG的低16位）
	cc1101_state = CC1101_Setup(Rf_Freq,1.2f,4.5f,58.0f,0,16);
	MSG("CC1101 initialize ");
	if(cc1101_state == RADIO_ERR_NONE)	//若找到器件，设置成功
	{
		MSG("OK!\r\n");
		BeeperMode = BEEP_ONCE;//响一声
		StatusBlinkMode = BLINK_SLOW;//慢闪指示进入接收待命状态
		ShowFixedPattern();	//OLED上显示固定字符(“车次”、“速度”等汉字)
	}
	else
	{
		MSG("failed! StateCode:%d\r\n",cc1101_state);
		StatusBlinkMode = BLINK_FAST;//快闪指示进入异常状态
		MSG("System halt!\r\n");
		OLED_ShowString(0*8,0,"   Attention!   ",16);
		OLED_ShowString(0*8,2," CC1101 Invalid!",16);
		OLED_ShowString(0*8,4,"  Please Check! ",16);
		OLED_ShowString(0*8,6,"System Halting..",16);
		while(true)
		{
			Delay_ms(10);
			if(++delay_count == 100)
			{
				delay_count = 0;
				BeeperMode = DBL_BEEP;	//每1秒响2次
			}
		}
	}
}
/*-----------------------------------------------------------------------
*@brief		在OLED屏幕上显示LBJ解码信息
*@param		LBJ_Msg - 指向已解码POCSAG消息的指针
*           rssi,lqi - 从CCC1101读到的本次解码时RSSI和LQI水平
*@retval	无
-----------------------------------------------------------------------*/
void ShowMessageLBJ(POCSAG_RESULT* POCSAG_Msg,float rssi,uint8_t lqi)
{
	char LBJ_Info[3][7] = {{0},{0},{0}};//存储车次、速度、公里标信息，每条预留6字符
	char Link_Info[2][6] = {{0},{0}};	//RSSI/LQI连接质量信息

	for(uint8_t i = 0;i < 3;i++)
	{
		strncpy(LBJ_Info[i],POCSAG_Msg->txtMsg+i*5,5);//txtMsg每5个字符一组分别存储
	}
	//LBJ_Info[0]车次：12345，LBJ_Info[1]速度：C100C，LBJ_Info[2]公里标：23456
	//"C"代表空格，车次数和公里标不够5位时在高位补C,位无效时显示"-"
	LBJ_Info[2][5] = LBJ_Info[2][4];	//将公里标最后位和倒数2位间加入小数点"."
	LBJ_Info[2][4] = '.';
	sprintf(Link_Info[0],"%.1f",rssi);	//连接质量指示
	sprintf(Link_Info[1],"%hhu",lqi);

	OLED_ShowString(6*8,0,LBJ_Info[0],16);	//1.1显示车次
	if(POCSAG_Msg->FuncCode == FUNC_SHANGXING)	//1.2显示上下行
		OLED_ShowPattern16x16(6*16,0,7); //上
	else if(POCSAG_Msg->FuncCode == FUNC_XIAXING)
		OLED_ShowPattern16x16(6*16,0,8); //下
	OLED_ShowOneChar(6*8,2,LBJ_Info[1][1],16);	//2.1显示速度-百位
	OLED_ShowOneChar(7*8,2,LBJ_Info[1][2],16); //2.2显示速度-十位
	OLED_ShowOneChar(8*8,2,LBJ_Info[1][3],16); //2.3显示速度-个位
	OLED_ShowString(7*8,4,LBJ_Info[2],16);	//3.显示公里标
	OLED_ShowString(5*8,6,Link_Info[0],16);	//4.1显示RSSI
	OLED_ShowString(14*8,6,Link_Info[1],16);//4.2显示LQI

}
/*-----------------------------------------------------------------------
*@brief		OLED上显示固定字符(“车次”、“速度”等汉字)
**@detail   车次: ----- 上行	汉字大小16*16 英文16x8
*           速度: --- km/h   汉字大小16*16 英文16x8
*           公里标: ---.- km 汉字大小16*16 英文16x8
*           RSSI:---.-LQI:-- 英文16x8
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void ShowFixedPattern(void)
{
	//第一行，行号0-1
	OLED_ShowPattern16x16(0*16,0,0); //车
	OLED_ShowPattern16x16(1*16,0,1); //次
	OLED_ShowPattern16x16(6*16,0,7); //上
	OLED_ShowPattern16x16(7*16,0,9); //行
	OLED_ShowString(4*8,0,": ----- ",16);
	//第二行,行号2-3
	OLED_ShowPattern16x16(0*16,2,2); //速
	OLED_ShowPattern16x16(1*16,2,3); //度
	OLED_ShowString(4*8,2,": --- km/h",16);
	//第三行，行号4-5
	OLED_ShowPattern16x16(0*16,4,4); //公
	OLED_ShowPattern16x16(1*16,4,5); //里
	OLED_ShowPattern16x16(2*16,4,6); //标
	OLED_ShowString(6*8,4,": ---.- km",16);
	//第四行，行号6-7
	OLED_ShowString(0*8,6,"RSSI:---.- LQI--",16);	
}
/*-----------------------------------------------------------------------
*@brief		显示版本信息通过串口打印
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void ShowBuildInfo(void)
{
	MSG("%s %s (Build %s %s) [Type '$' for help.]\r\n",APP_NAME_STR,
		      VERTION_STR,BUILD_DATE_STR,BUILD_TIME_STR);//串口打印版本信息
	MSG("Xie Yingnan Works.<xieyingnan1994@163.com>\r\n");
}

/*-----------------------------------------------------------------------
*@brief		OLED显示开机画面和版本信息
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void ShowSplashScreen(void)
{
	char buf[17] = {0};	//每行最多16字符
	OLED_ShowBMP128x64(nBitmapDot);
	//nBitmapDot是在HW_OLED_Font.c中定义的开机画面位图
	Delay_ms(1500);
	OLED_Clear();//清屏
	sprintf(buf,"<%s>",APP_NAME_STR);
	OLED_ShowString(0,0,buf,12);//8x6字体，列宽6占用8，行高8
	sprintf(buf,"<Version:%s>",VERTION_STR);
	OLED_ShowString(0,1,buf,12);
	OLED_ShowString(0,2,"  <Build Date>  ",12);
	sprintf(buf,"Date:%s",BUILD_DATE_STR);
	OLED_ShowString(0,3,buf,12);
	sprintf(buf,"Time:%s",BUILD_TIME_STR);
	OLED_ShowString(0,4,buf,12);
	OLED_ShowString(0,5,"----------------",12);
	OLED_ShowString(0,6,">Author:Xie Y.N.",12);
	OLED_ShowString(0,7,">CallSign:BH2RPH",12);
	Delay_ms(1500);
	Delay_ms(1200);
	OLED_Clear();//清屏
}
/*-----------------------------------------------------------------------
*@brief		串口打印设置项目
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void ShowSettings(void)
{
	MSG("Settings:\r\n");
	MSG("Rf frequency:%.4fMHz\r\n",Rf_Freq);
}
/*-----------------------------------------------------------------------
*@brief		CC1101数据包接收完成时的回调函数
*@param		按照初始化时的寄存器设置，接收完成时CC1101处于IDLE态，
*        	数据保存在FIFO
*@retval	无
-----------------------------------------------------------------------*/
void Rx_Callback(void)
{
	if(!bDataArrivalFlag)
		bDataArrivalFlag = true;	//置数据到达标志位
}
/*-----------------------------------------------------------------------
*@brief		读取已接收的数据并处理和显示
*@param		按照初始化时的寄存器设置，接收完成时CC1101处于IDLE态，
*        	数据保存在FIFO中。处理后，需要再次发送RX命令，方可接收
*@retval	无
-----------------------------------------------------------------------*/
void RxDataFeedProc(void)
{
	uint8_t* batch_buff = NULL;	//存放码字原始数据的缓冲区
	uint32_t batch_len = CC1101_GetPacketLength(false);
			//获取已设置的包长度,在本例中已在初始化中设置为FIFO的大小64字节
	uint32_t actual_len;//实际读到的原始数据长度，定长模式时和batch_len相同
	POCSAG_RESULT PocsagMsg;//保存POCSAG解码结果的结构体

	if((batch_buff=(uint8_t*)malloc(batch_len*sizeof(uint8_t))) != NULL)
	{
		memset(batch_buff,0,batch_len);	//清空batch缓存

		CC1101_ReadDataFIFO(batch_buff,&actual_len);//从FIFO读入原始数据
		float rssi = CC1101_GetRSSI();//由于接收完成后处于IDLE态
		uint8_t lqi = CC1101_GetLQI();//这里的RSSI和LQI冻结不变与本次数据包相对应

		MSG("!!Received %u bytes of raw data.\r\n",actual_len);
		MSG("RSSI:%.1f LQI:%hhu\r\n",rssi,lqi);
		MSG("Raw data:\r\n");
		for(uint32_t i=0;i < actual_len;i++)
		{
			MSG("%02Xh ",batch_buff[i]);//打印原始数据
			if((i+1)%16 == 0)
				MSG("\r\n");	//每行16个
		}
		//解析LBJ信息（地址已过滤）
		int8_t state = POCSAG_ParseCodeWordsLBJ(&PocsagMsg,batch_buff,
												 actual_len,true);							     		 
		if(state == POCSAG_ERR_NONE)
		{										
			MSG("Address:%u,Function:%hhd.\r\n",PocsagMsg.Address,PocsagMsg.FuncCode);
												//显示地址码，功能码
			MSG("LBJ Message:%s.\r\n",PocsagMsg.txtMsg);//显示文本消息
			ShowMessageLBJ(&PocsagMsg,rssi,lqi);	//在OLED屏幕上显示LBJ解码信息
			switch(PocsagMsg.FuncCode)	//蜂鸣器根据功能码鸣响对应次数
			{
				case FUNC_XIAXING:
					BeeperMode = BEEP_ONCE;//响一次
					InfoBlinkMode = BLINK_ONCE;//闪烁一次
					break;
				case FUNC_SHANGXING:
					InfoBlinkMode = DBL_BLINK;//闪烁两次
					BeeperMode = DBL_BEEP;//响两次
					break;
				default: BeeperMode = DBL_BEEP; break;
			}
		}
		else
		{
			MSG("POCSAG parse failed! Errorcode:%d\r\n",state);
			BeeperMode = DBL_BEEP;
		}
		free(batch_buff);
	}
	CC1101_StartReceive(Rx_Callback);	//继续接收
}
/*-----------------------------------------------------------------------
*@brief		定时器中断服务函数用于提供时基
*@detail 	定时周期在TIM初始化函数中确定，本程序为10mS
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void INT_TIMER_IRQHandler(void)
{
	static uint8_t cnt_beep = 0,cnt_beeptimes = 0;
	static uint8_t cnt_blink = 0;
	static uint8_t cnt_infoblink = 0,cnt_infoblinktimes = 0;

	if(TIM_GetITStatus(INT_TIMER,TIM_IT_Update)!=RESET)
	{
		switch(BeeperMode)	//提示蜂鸣器的控制
		{
		case BEEP_ONCE:							//响一次
			BUZZER_ON();
			if(++cnt_beep >= 10)
			{ BUZZER_OFF(); BeeperMode = BEEP_OFF; }
			break;
		case DBL_BEEP: 
			if(cnt_beeptimes < 2)
			{
				if(cnt_beep <= 8) {BUZZER_ON();}	//响80ms
				else {BUZZER_OFF();}				//停80ms
				if(++cnt_beep >= 16)				//周期160ms
				{cnt_beep = 0; cnt_beeptimes++;}
			}
			else
				BeeperMode = BEEP_OFF;	//响2次后停
			break;
		default:
			BUZZER_OFF();	//默认关闭蜂鸣器
			cnt_beep = 0;	//清空beep计时变量
			cnt_beeptimes = 0; //清空beep记次变量
			break;
		}

		switch(StatusBlinkMode)	//状态LED
		{
		case BLINK_FAST:
			if(++cnt_blink >= 10)	//周期200ms,50%duty
			{ STATUS_LED_TOGGLE(); cnt_blink = 0; }
			break;
		case BLINK_SLOW:
			if(cnt_blink <= 20) {STATUS_LED_ON();}	//亮200ms
			else {STATUS_LED_OFF();}			//灭800ms
			if(++cnt_blink >= 100)				//周期1000ms
				cnt_blink = 0;
			break;
		default:
			STATUS_LED_OFF();
			cnt_blink = 0;
			break;
		}

		switch(InfoBlinkMode)
		{
		case BLINK_ONCE:
			INFO_LED_ON();
			if(++cnt_infoblink >= 15)
			{ 
				INFO_LED_OFF();
				InfoBlinkMode = BLINK_OFF;
			}
			break;
		case DBL_BLINK:
			if(cnt_infoblinktimes < 2)
			{
				if(cnt_infoblink <= 10) {INFO_LED_ON();}	//亮100ms
				else {INFO_LED_OFF();}				//灭100ms
				if(++cnt_infoblink >= 20)				//周期200ms
				{cnt_infoblink = 0; cnt_infoblinktimes++;}
			}
			else
				InfoBlinkMode = BLINK_OFF;	//响2次后停
			break;
		default:
			INFO_LED_OFF();
			cnt_infoblink = 0;
			cnt_infoblinktimes = 0;
			break;
		}
	}
	TIM_ClearITPendingBit(INT_TIMER,TIM_IT_Update);
}
