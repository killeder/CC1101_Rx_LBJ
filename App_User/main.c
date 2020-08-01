/*-----------------------------------------------------------------------
*@file     main.c
*@brief    CC1101_Rx_LBJ工程主文件
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/07/27
-----------------------------------------------------------------------*/
#include "CC1101_Rx_LBJ.h"
/*-----------------------------------------------------------------------
*@brief		各个底层硬件初始化
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void HW_Base_Init(void)
{
	Delay_init();		//初始化Systick延时
	HW_USART1_Init(115200);	//初始化串行口
	IIC_GPIOConfig();	//初始化软件模拟IIC总线用到的GPIO口
	HW_GPIO_Init_Out(STATUS_LED_CLOCK,STATUS_LED_PORT,
									STATUS_LED_PIN,GPIO_Mode_Out_PP);
	STATUS_LED_OFF();	//关闭状态指示灯
	HW_GPIO_Init_Out(INFO_LED_CLOCK,INFO_LED_PORT,
									INFO_LED_PIN,GPIO_Mode_Out_PP);
	INFO_LED_OFF();	//关闭消息指示灯
	HW_GPIO_Init_Out(BUZZER_CLOCK,BUZZER_PORT,BUZZER_PIN,GPIO_Mode_Out_PP);
	BUZZER_OFF();	//关闭蜂鸣器
	HW_TIM_Interrupt_Init(INT_TIM_PERIOD,INT_TIM_PRESCALER);//初始化用于提供时基的定时器								
	HW_TIM_Interrupt_Enable();	//打开时基定时器中断
	OLED_Init();	//SSD1306 OLED屏幕初始化
}
/*-----------------------------------------------------------------------
*@brief		主函数：程序入口
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//打开复用功能时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//关闭JTAG,只保留SWD
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//初始化中断向量组为组2
	
	HW_Base_Init();			//各个底层硬件初始化
	ShowBuildInfo();		//串口打印版本信息
	ShowSettings();			//串口打印设置项目
	ShowSplashScreen();		//OLED显示开机画面和版本信息
	CC1101_Initialize();	//检测CC1101并初始化设置
	CC1101_StartReceive(Rx_Callback);	//开始接收

	while(true)
	{
		if(bDataArrivalFlag)
		{
			RxDataFeedProc();	//数据收妥后开始读取数据并处理显示
			bDataArrivalFlag = false;//处理后清空标志位
		}
		//响应串口收妥标志位、解析串口传来的命令字符串				
		if(bit_IsTrue(USART1_RxState,USART1_RXCOMPLETE_FLAG))
		{
			ParseSerialCmdLine((char*)USART1_RxBuffer);//若串口收妥一行数据则开始解析
			USART1_RxState = 0;//清除数据收妥的标志位和数据计数
		}
	}
}
