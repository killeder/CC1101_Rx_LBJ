/*-----------------------------------------------------------------------
*@file     HW_RADIO_CC1101.c
*@brief    德州仪器CC1101无线芯片驱动程序
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/07/13
-----------------------------------------------------------------------*/
#include "Hardware.h"

#define CC1101_SPIx_CLOCK 	RCC_APB2Periph_SPI1	//CC1101所使用硬件SPI时钟
#define CC1101_SPIx 	  	SPI1  				//CC1101所使用的硬件SPI号

#define CC1101_SCK_CLOCK	RCC_APB2Periph_GPIOA	//SPI时钟引脚PA5
#define CC1101_SCK_PORT		GPIOA
#define CC1101_SCK_PIN		GPIO_Pin_5

#define CC1101_MISO_CLOCK	RCC_APB2Periph_GPIOA	//SPI的MISO引脚PA6
#define CC1101_MISO_PORT	GPIOA
#define CC1101_MISO_PIN		GPIO_Pin_6

#define CC1101_MOSI_CLOCK	RCC_APB2Periph_GPIOA	//SPI的MOSI引脚PA7
#define CC1101_MOSI_PORT	GPIOA
#define CC1101_MOSI_PIN		GPIO_Pin_7

#define CC1101_CS_CLOCK 	RCC_APB2Periph_GPIOA	//SPI的片选引脚PA1
#define CC1101_CS_PORT 		GPIOA
#define CC1101_CS_PIN 		GPIO_Pin_1
#define	CC1101_CS_High()	CC1101_CS_PORT->BSRR = CC1101_CS_PIN//拉高片选（失能）
#define	CC1101_CS_Low() 	CC1101_CS_PORT->BRR = CC1101_CS_PIN	//拉低片选（使能）

#define CC1101_IRQ_PORT 	GPIOC 					//CC1101中断引脚PC0(连接到GDO2)
#define CC1101_IRQ_CLOCK 	RCC_APB2Periph_GPIOC
#define CC1101_IRQ_PIN		GPIO_Pin_0
#define CC1101_IRQ_PORTSOURCE 	GPIO_PortSourceGPIOC	//外部中断端口源
#define CC1101_IRQ_PINSOURCE 	GPIO_PinSource0		//外部中断引脚源
#define CC1101_IRQ_LINE		EXTI_Line0				//外部中断线
#define CC1101_IRQ_IRQn		EXTI0_IRQn			//外部中断标号(位于stm32f10x.h头文件)
#define CC1101_IRQ_Handler  EXTI0_IRQHandler	//外部中断响应程序入口(位于s启动文件中)

static void (*fpGDO2_IRQ_Callback)(void) = NULL;	//GDO2外部中断响应函数中要调用的回调函数指针
static uint8_t _packetLength = CC1101_FIFO_SIZE;	//内部变量，记录包长度或最大长度
static uint8_t _packetLengthMode = CC1101_LENGTH_CONFIG_FIXED;	//内部变量，保存数据包长度模式
static bool _packetLengthQueried = false;	//内部变量，标记包长度是否已查询
static bool _CRC_On = false;//内部变量，保存CRC校验功能是否启用
static float _freq = 433.0f;//内部变量，用于保存工作频率供设置发射功率时使用
static int8_t _power = -30;	//内部变量，用于保存设置好的发射功率，在更改频率后更新发射功率时用
/*-----------------------------------------------------------------------
*@brief		CC1101所使用的硬件SPI和引脚初始化
*@detail 	使用的硬件SPI编号、SPI引脚、片选引脚在头文件定义为宏
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
static void HwSPI_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	SPI_InitTypeDef		SPI_InitStructure;
	//1.初始化SPIx使用到的四个引脚
	RCC_APB2PeriphClockCmd(CC1101_SCK_CLOCK|CC1101_MISO_CLOCK|
							CC1101_MOSI_CLOCK|CC1101_CS_CLOCK,ENABLE);
							//打开SPI涉及的4个引脚的时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;//频率10MHz
	GPIO_InitStructure.GPIO_Pin = CC1101_SCK_PIN;
	GPIO_Init(CC1101_SCK_PORT,&GPIO_InitStructure);//设置SCK引脚

	GPIO_InitStructure.GPIO_Pin = CC1101_MISO_PIN;
	GPIO_Init(CC1101_MISO_PORT,&GPIO_InitStructure);//设置MISO引脚

	GPIO_InitStructure.GPIO_Pin = CC1101_MOSI_PIN;
	GPIO_Init(CC1101_MOSI_PORT,&GPIO_InitStructure);//设置MOSI引脚

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin = CC1101_CS_PIN;
	GPIO_Init(CC1101_CS_PORT,&GPIO_InitStructure);//设置CS引脚
	CC1101_CS_High();	//拉高片选，使CC1101芯片失能
	//2.初始化硬件SPI
	SPI_I2S_DeInit(CC1101_SPIx);			//复位SPI
	if(CC1101_SPIx == SPI1)
		RCC_APB2PeriphClockCmd(CC1101_SPIx_CLOCK,ENABLE);//SPI1位于APB2
	else
		RCC_APB1PeriphClockCmd(CC1101_SPIx_CLOCK,ENABLE);//SPI2,3位于APB1
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//空闲时低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//在上升沿采样，MODE0
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;// 72/8=9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(CC1101_SPIx,&SPI_InitStructure);
	SPI_Cmd(CC1101_SPIx,ENABLE);	//使能SPI1
}
/*-----------------------------------------------------------------------
*@brief		SPI发送一字节数据
*@detail 	
*@param		data - 要发送的数据
*@retval	无
-----------------------------------------------------------------------*/
static void SPISendByte(uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(CC1101_SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
	}	//等待发送缓存空
	SPI_I2S_SendData(CC1101_SPIx, data);	//发送一字节数据
	while(SPI_I2S_GetFlagStatus(CC1101_SPIx, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}	//等待接收缓冲区空
	SPI_I2S_ReceiveData(CC1101_SPIx);	//收一字节数据（状态字）并舍弃	
}
/*-----------------------------------------------------------------------
*@brief		SPI接收一字节数据
*@detail 	
*@param		无
*@retval	读到的数据
-----------------------------------------------------------------------*/
static uint8_t SPIReadByte(void)
{
	uint8_t retval = 0;	//读取到的数据

	while(SPI_I2S_GetFlagStatus(CC1101_SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
	}	//等待发送缓存空
	SPI_I2S_SendData(CC1101_SPIx, 0xFF);	//发送一字节Dummy数据
	while(SPI_I2S_GetFlagStatus(CC1101_SPIx, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}	//等待接收缓冲区空
	retval = SPI_I2S_ReceiveData(CC1101_SPIx);	//收一字节数据

	return retval;
}
/*-----------------------------------------------------------------------
*@brief		SPI底层数据传输
*@detail 	
*@param		rw - 数据传输方向/reg - 数据传输对象/dataOut - 要输出的数据存放地址
*           dataIn - 读取到的数据存放位置/numByte - 要传输的数据个数
*@retval	无
-----------------------------------------------------------------------*/
static void SPItransfer(uint8_t rw, uint8_t reg, uint8_t* dataOut, uint8_t* dataIn, uint32_t numBytes)
{
	RADIO_VERBOSE_MSG("[CC1101]SPI Transfer:");
	RADIO_VERBOSE_MSG("%s",(rw==CC1101_CMD_WRITE)?"Write":"Read");
	RADIO_VERBOSE_MSG(" Register %02Xh -> ",reg);

	CC1101_CS_Low();	//芯片使能
	SPISendByte(rw|reg);	//发送读写状态位和寄存器地址
	for(uint32_t n = 0;n < numBytes; n++)
	{
		if(rw == CC1101_CMD_WRITE)
		{
			SPISendByte(dataOut[n]);	//发送一字节数据
			RADIO_VERBOSE_MSG("%02Xh ",dataOut[n]);
		}
		else if(rw == CC1101_CMD_READ)
		{
			dataIn[n] = SPIReadByte();	//接收一字节数据
			RADIO_VERBOSE_MSG("%02Xh ",dataIn[n]);
		}
	}
	CC1101_CS_High();	//芯片除能

	RADIO_VERBOSE_MSG("\r\n");
}
/*-----------------------------------------------------------------------
*@brief		写入单个寄存器
*@detail 	
*@param		reg- 要写入的寄存器地址/data - 要写入的数据
*@retval	无
-----------------------------------------------------------------------*/
static void SPIwriteRegister(uint8_t reg, uint8_t data)
{
	if(reg > CC1101_REG_TEST0)
		reg |= CC1101_CMD_ACCESS_STATUS_REG;
	SPItransfer(CC1101_CMD_WRITE,reg,&data,NULL,1);
}
/*-----------------------------------------------------------------------
*@brief		批量写入寄存器
*@detail 	
*@param		reg- 要写入的寄存器地址/dataOut - 要写入的数据的地址
*           len - 要写入数据的长度
*@retval	无
-----------------------------------------------------------------------*/
static void SPIwriteRegisterBurst(uint8_t reg, uint8_t* dataOut, uint32_t len)
{
	reg |= CC1101_CMD_BURST;	//加入批量访问标志位
	SPItransfer(CC1101_CMD_WRITE,reg,dataOut,NULL,len);
}
/*-----------------------------------------------------------------------
*@brief		读取单个寄存器
*@detail 	
*@param		reg- 要写入的寄存器地址
*@retval	读到的数据
-----------------------------------------------------------------------*/
static uint8_t SPIreadRegister(uint8_t reg)
{
	uint8_t retval = 0;

	if(reg > CC1101_REG_TEST0)
		reg |= CC1101_CMD_ACCESS_STATUS_REG;
	SPItransfer(CC1101_CMD_READ,reg,NULL,&retval,1);

	return retval;
}
/*-----------------------------------------------------------------------
*@brief		批量读取寄存器
*@detail 	
*@param		reg- 要读取的寄存器地址 dataIn - 读取到的数据保存地址
*           len - 要读取的数据长度
*@retval	无
-----------------------------------------------------------------------*/
static void SPIreadRegisterBurst(uint8_t reg, uint8_t* dataIn, uint32_t len)
{
	reg |= CC1101_CMD_BURST;	//加入批量访问标志位
	SPItransfer(CC1101_CMD_READ,reg,NULL,dataIn,len);
}
/*-----------------------------------------------------------------------
*@brief		将寄存器写为掩码运算后的值（只改变某些位，其余位保持不变）
*@detail 	
*@param		reg- 要写入的寄存器地址/value - 要写入的值（掩码前）/msb - 高位掩码位置
*           lsb - 低位掩码位置
*@retval	错误码
-----------------------------------------------------------------------*/
static int8_t SPIsetMaskedRegValue(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb)
{
	if((msb > 7) || (lsb > 7) || (lsb > msb))
    	return(RADIO_ERR_INVALID_BIT_RANGE);

	uint8_t readvalue = 0;	//用于在写入后尝试读取校验写入是否成功
	uint8_t currentValue = SPIreadRegister(reg);//取得写入前目标寄存器的值
	uint8_t mask = ~((0xFF << (msb + 1)) | (0xFF >> (8 - lsb)));//生成掩码
	uint8_t newValue = (currentValue & ~mask) | (value & mask);//掩码运算得新值
	SPIwriteRegister(reg, newValue);//写入新值，只改变指定位，其他位保持不变

	for(uint8_t count = 0;count < 200;count++)
	{
		Delay_us(10);	//延时10微秒
		readvalue = SPIreadRegister(reg);
		if(readvalue == newValue)
			return(RADIO_ERR_NONE);//如果读取到的值和已写入的值一致，则返回“无错误”
	}

	RADIO_DEBUG_MSG("[CC1101]SPI set masked value failed!\r\n");
	RADIO_DEBUG_MSG("REG=%02Xh,PreviousValue=%02Xh,Mask=%02Xh,\r\n"
					"NewValue=%02Xh,ReadValue=%02Xh.\r\n",
					 reg,currentValue,mask,newValue,readvalue);
	return(RADIO_ERR_SPI_WRITE_FAILED);//超2ms仍无响应，则返回“写入错误”
}
/*-----------------------------------------------------------------------
*@brief		取得位掩码运算后的寄存器值
*@detail 	
*@param		reg- 要写入的寄存器地址/msb - 高位掩码位置
*           lsb - 低位掩码位置
*@retval	掩码后的寄存器值
-----------------------------------------------------------------------*/
static uint8_t SPIgetMaskedRegValue(uint8_t reg, uint8_t msb, uint8_t lsb)
{
	uint8_t rawValue = SPIreadRegister(reg);
	uint8_t maskedValue = rawValue & ((0xFF << lsb) & (0xFF >> (7 - msb)));
	return maskedValue;
}
/*-----------------------------------------------------------------------
*@brief		SPI发送单字节命令0x30-0x3D
*@detail 	
*@param		cmd - 要发送的命令
*@retval	无
-----------------------------------------------------------------------*/
static void SPISendCommand(uint8_t cmd)
{
	CC1101_CS_Low();	//芯片使能
	SPISendByte(cmd);	//发送单字节命令
	CC1101_CS_High();	//芯片除能
	RADIO_VERBOSE_MSG("[CC1101]Send command strobe:%02Xh.\r\n",cmd);
}
/*-----------------------------------------------------------------------
*@brief		逼近下面公式的值浮点数target，取得式中指数exp和尾数mant
*@detail 	target = ((mantOffset+mant)*2^exp)/2^divExp*Fosc*10^6
*@param		如以上公式所示
*@retval	无
-----------------------------------------------------------------------*/
static void getExpMant(float target, uint16_t mantOffset, int8_t divExp,
						int8_t expMax, uint8_t* exp, uint8_t* mant)
{
 	//取得逼近起始点(exp = 0, mant = 0)
  	float origin = (mantOffset * CC1101_CRYSTAL_FREQ * 1000000.0)/((uint32_t)1 << divExp);

  	//迭代，遍历所有指数
  	for(int8_t e = expMax; e >= 0; e--)
  	{
	 	 float intervalStart = ((uint32_t)1 << e) * origin;
	  	if(target >= intervalStart)
	  	{
      		*exp = e;//逼近完指数后保存指数，接下来逼近尾数
	    	float stepSize = intervalStart/(float)mantOffset;//为(2^exp/2^divExp)*Fosc*10^6
	   		*mant = ((target - intervalStart) / stepSize);
	    	return;//逼近一次后即为所求，跳出
	  	}
	}
}
/*-----------------------------------------------------------------------
*@brief		先软复位芯片，后直接访问寄存器进行多种初始化设置
*@detail 	本函数主要完成后续API无法完成的初始化寄存器设置项目，任何需要直接
*         	访问寄存器进行的初始化操作都在此进行。
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
static int8_t MiscRegisterConfigs(void)
{
	int8_t state;

	SPISendCommand(CC1101_CMD_IDLE);	//使CC1101进入空闲模式
	state = SPIsetMaskedRegValue(CC1101_REG_MCSM0,
								 CC1101_FS_AUTOCAL_IDLE_TO_RXTX,5,4);
					//设置自动频率合成器校准模式为“在IDLE切换为TX/RX模式时”
	state |= SPIsetMaskedRegValue(CC1101_REG_MCSM1,
								 CC1101_RXOFF_IDLE|CC1101_TXOFF_IDLE,3,0);
					//(重点!)设置发送后模式为“IDLE”，接收完成后模式为“IDLE”
	state |= SPIsetMaskedRegValue(CC1101_REG_IOCFG0,
							CC1101_GDOX_CHANNEL_CLEAR,5,0);
					//设置GDO0引脚为“空信道检测”，信道电平低于阈值后置有效电平
	state |= SPIsetMaskedRegValue(CC1101_REG_IOCFG2,
							CC1101_GDOX_SYNC_WORD_SENT_OR_RECEIVED,5,0);
					//设置GDO2引脚为“同步字收到或发出后有效”，作为发送完成和
					//接收完成的外部中断触发源
	state |= SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0,
					CC1101_WHITE_DATA_OFF|CC1101_PKT_FORMAT_NORMAL,6,4);
					//关闭数据白化，使用数据包模式
	state |= SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0,
					CC1101_CRC_OFF|_packetLengthMode,2,0);
					//关闭CRC校验，设置为定长模式
	state |= SPIsetMaskedRegValue(CC1101_REG_PKTCTRL1,
					CC1101_CRC_AUTOFLUSH_OFF|CC1101_APPEND_STATUS_OFF|
					CC1101_ADR_CHK_NONE,3,0);
					//关闭CRC校验后的FIFO自动清除，取消数据后附加RSSI等状态信息
					//取消地址过滤
	RADIO_ASSERT(state);
	return (state);
}
/*-----------------------------------------------------------------------
*@brief		GDO2引脚所接外部中断初始化
*@detail 	中断设置为下降沿触发，即在发送完成和接收完成后触发
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
static void GDO2_ExtIRQ_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	//1.初始化GDO2连接到的引脚
	RCC_APB2PeriphClockCmd(CC1101_IRQ_CLOCK,ENABLE);//打开外部中断引脚的时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;//频率10MHz
	GPIO_InitStructure.GPIO_Pin = CC1101_IRQ_PIN;
	GPIO_Init(CC1101_IRQ_PORT,&GPIO_InitStructure);//设置外部中断引脚	
	//2.初始化EXTI中断
	GPIO_EXTILineConfig(CC1101_IRQ_PORTSOURCE,CC1101_IRQ_PINSOURCE);	//选择EXTI的信号源	
	EXTI_InitStructure.EXTI_Line = CC1101_IRQ_LINE; //选择相应中断线
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//选择为中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能中断线
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CC1101_IRQ_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//次优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}
/*-----------------------------------------------------------------------
*@brief		GDO2引脚外部中断响应函数
*@detail 	在此函数中调用fpGDO2_IRQ_Callback指针指向的函数，为实际回调函数
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void CC1101_IRQ_Handler(void)
{
	if(EXTI_GetITStatus(CC1101_IRQ_LINE) != RESET)
	{
		if(fpGDO2_IRQ_Callback != NULL)
		{
			(*fpGDO2_IRQ_Callback)();	//调用回调函数
			//当发送状态，fpGDO2_IRQ_Callback实际调用的为发送完成回调函数
			//当接收状态，fpGDO2_IRQ_Callback实际调用的为接收完成回调函数
		}
		EXTI_ClearITPendingBit(CC1101_IRQ_LINE);//清除中断标志位
	}
}
/*-----------------------------------------------------------------------
*@brief		使CC1101进入待命IDLE模式
*@detail 	直接向SPI总线发送命令
*@param		无
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_GoIdle(void)
{
	SPISendCommand(CC1101_CMD_IDLE);
	return(RADIO_ERR_NONE);
}
/*-----------------------------------------------------------------------
*@brief		设置包长度模式为固定长度模式/可变长度模式
*@detail 	
*@param		mode - 取CC1101_LENGTH_CONFIG_FIXED为定长模式；
*             	   取CC1101_LENGTH_CONFIG_VARIABLE时为变长模式；
*           len -  定长模式时为包固定长度，接收时大于该长度的数据会舍弃；
*           	   变长模式时为包最大长度，大于此长度的部分会被舍弃
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetPacketLengthMode(uint8_t mode, uint8_t len)
{
	uint8_t len_limit;
	int8_t state;
	
	if(mode == CC1101_LENGTH_CONFIG_FIXED)
		len_limit = CC1101_FIFO_SIZE;//固定长度时为FIFO大小
	else if(mode == CC1101_LENGTH_CONFIG_VARIABLE)
		len_limit = CC1101_FIFO_SIZE - 1;//可变长度时，为FIFO大小-1
								   //因为第一个字节表征包长度，不记入包总长度
	if(len > len_limit)
		return(RADIO_ERR_PACKET_TOO_LONG);	//包长度超范围则返回“包过长”错误

	state = SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0,mode,1,0);
	RADIO_ASSERT(state);//写入包长模式
	state = SPIsetMaskedRegValue(CC1101_REG_PKTLEN,len,7,0);
	RADIO_ASSERT(state);//写入包长或包长最大值

	_packetLength = len;//更新包长度和包模式内部变量记录
	_packetLengthMode = mode;//作为后续包长度和模式查询时的缓存数据

	return (state);
}
/*-----------------------------------------------------------------------
*@brief		设置编码模式：曼彻斯特编码、数据白化功能是否启用
*@detail 	
*@param		encoding - 0 关闭曼彻斯特编码和数据白化
*                 	   1 只启用曼彻斯特编码
*                 	   2 只启用数据白化
*                 	   3 同时启动曼彻斯特编码和数据白化
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetEncoding(uint8_t encoding)
{
  int8_t state = CC1101_GoIdle();	//先确保进入IDLE模式再进行设置
  RADIO_ASSERT(state);

  switch(encoding)	//设置编码模式
  {	
    case 0:
      state = SPIsetMaskedRegValue(CC1101_REG_MDMCFG2, CC1101_MANCHESTER_EN_OFF, 3, 3);
      RADIO_ASSERT(state);
      return(SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0, CC1101_WHITE_DATA_OFF, 6, 6));
    case 1:
      state = SPIsetMaskedRegValue(CC1101_REG_MDMCFG2, CC1101_MANCHESTER_EN_ON, 3, 3);
      RADIO_ASSERT(state);
      return(SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0, CC1101_WHITE_DATA_OFF, 6, 6));
    case 2:
      state = SPIsetMaskedRegValue(CC1101_REG_MDMCFG2, CC1101_MANCHESTER_EN_OFF, 3, 3);
      RADIO_ASSERT(state);
      return(SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0, CC1101_WHITE_DATA_ON, 6, 6));
    default:
      return(RADIO_ERR_INVALID_ENCODING);
  }
}
/*-----------------------------------------------------------------------
*@brief		设置调制方式：2FSK或GFSK调制方式，
*@detail 	
*@param		sh - 0 		使用2FSK调制方式
*                0.5 	使用高斯滤波后的2FSK即GFSK调制方式
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetDataShaping(float sh)
{
  int8_t state = CC1101_GoIdle();	//先确保进入IDLE模式再进行设置
  RADIO_ASSERT(state);

  sh *= 10.0;	//设置数据调制模式
  if(abs(sh - 0.0) <= 0.001) {
    state = SPIsetMaskedRegValue(CC1101_REG_MDMCFG2, CC1101_MOD_FORMAT_2_FSK, 6, 4);
  } else if(abs(sh - 5.0) <= 0.001) {
    state = SPIsetMaskedRegValue(CC1101_REG_MDMCFG2, CC1101_MOD_FORMAT_GFSK, 6, 4);
  } else {
    return(RADIO_ERR_INVALID_DATA_SHAPING);
  }
  return(state);
}
/*-----------------------------------------------------------------------
*@brief		设置CRC校验功能开启/关闭
*@detail 	
*@param		crcon - true 开启; false 关闭
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetCrcFiltering(bool crcOn)
{
	_CRC_On = crcOn;	//保存CRC开启/关闭状态，
						//用于接收数据时决定是否判断CRC校验状态
  if (crcOn == true) {
    return(SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0,CC1101_CRC_ON,2,2));
  } else {
    return(SPIsetMaskedRegValue(CC1101_REG_PKTCTRL0,CC1101_CRC_OFF,2,2));
  }
}
/*-----------------------------------------------------------------------
*@brief		关闭同步字过滤功能并决定是保留载波检测功能
*@detail 	
*@param		requireCarrierSense - 关闭同步字过滤后是否保留载波检测
*                            	true 保留载波检测; false 关闭载波检测
*           启用载波检测后，需要同步码正确接收指定的位数，同时检测到载波，
*           方可继续接收后续。
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_DisableSyncWordFiltering(bool requireCarrierSense)
{
  return(SPIsetMaskedRegValue(CC1101_REG_MDMCFG2,
    requireCarrierSense ? CC1101_SYNC_MODE_NONE_THR : CC1101_SYNC_MODE_NONE,2,0));
}
/*-----------------------------------------------------------------------
*@brief		启用同步字过滤功能并决定是启用载波检测功能
*@detail 	
*@param		maxErrBits - 最大允许同步字错误的位数，取0或1
*			requireCarrierSense - 是否同时启用载波检测,true或false
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_EnableSyncWordFiltering(uint8_t maxErrBits, bool requireCarrierSense)
{
  switch (maxErrBits){
    case 0:
      // in 16 bit sync word, expect all 16 bits
      return (SPIsetMaskedRegValue(CC1101_REG_MDMCFG2,
        requireCarrierSense ? CC1101_SYNC_MODE_16_16_THR : CC1101_SYNC_MODE_16_16,2,0));
    case 1:
      // in 16 bit sync word, expect at least 15 bits
      return (SPIsetMaskedRegValue(CC1101_REG_MDMCFG2,
        requireCarrierSense ? CC1101_SYNC_MODE_15_16_THR : CC1101_SYNC_MODE_15_16,2,0));
    default:
      return (RADIO_ERR_INVALID_SYNC_WORD);
  }
}
/*-----------------------------------------------------------------------
*@brief		取消地址过滤功能
*@detail 	
*@param		无
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_DisableAddressFiltering(void)
{
  int8_t state = SPIsetMaskedRegValue(CC1101_REG_PKTCTRL1,
  									   CC1101_ADR_CHK_NONE,1,0);
  									//取消地址过滤功能
  RADIO_ASSERT(state);
  //设置地址为默认地址0x00
  return(SPIsetMaskedRegValue(CC1101_REG_ADDR,0x00,7,0));
}
/*-----------------------------------------------------------------------
*@brief		启用节点地址过滤功能并设置地址和广播地址模式
*@detail 	启用地址过滤功能后，只接收包含以下地址的数据包
*         	1.与本机CC1101_REG_ADDR内设置的地址相同的
*         	2.广播包(0x00作为广播地址或0x00与0xFF同时作为广播地址)
*@param		nodeAddr - 本机地址，保存在CC1101_REG_ADDR寄存器，当收到数据包地址
*                 	   与此地址相同时继续接收，否则舍弃。
*           numBroadcastAddrs - 广播地址的数量：0,无广播地址;1,0x00为广播地址;
*           		   2,0x00和0xFF同时为广播地址，当收到数据包地址与广播地址
*           		   相同时，也会继续接收。
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_EnableAddressFiltering(uint8_t nodeAddr, uint8_t numBroadcastAddrs)
{
	if(numBroadcastAddrs > 2)
		return(RADIO_ERR_INVALID_NUM_BROAD_ADDRS);	//检查广播地址个数的范围，应为0~2
	//启动地址过滤功能
	int8_t state = SPIsetMaskedRegValue(CC1101_REG_PKTCTRL1, numBroadcastAddrs+1,1,0);
	RADIO_ASSERT(state);
	//设置本机地址到寄存器
	return(SPIsetMaskedRegValue(CC1101_REG_ADDR,nodeAddr,7,0));
}

/*-----------------------------------------------------------------------
*@brief		读取数据包长度
*@detail 	变长模式从FIFO读取第一字节，定长模式读取包长度寄存器
*         	update为true且_packetLengthQueried为false时，按以上方式读取更新
*         	否则返回已缓存的包长度信息
*@param		update - 是否更新，读取从FIFO或寄存器更新后的信息
*@retval	包长度
-----------------------------------------------------------------------*/
uint32_t CC1101_GetPacketLength(bool update)
{
  if(!_packetLengthQueried && update) {//判断是否返回已缓存的包长度信息
    if (_packetLengthMode == CC1101_LENGTH_CONFIG_VARIABLE) {
      _packetLength = SPIreadRegister(CC1101_REG_FIFO);//变长模式从FIFO读取第一字节
    } else {
      _packetLength = SPIreadRegister(CC1101_REG_PKTLEN);//定长模式读取包长度寄存器
    }

    _packetLengthQueried = true;	//标记“包长度信息已读取”
  }

  return(_packetLength);
}
/*-----------------------------------------------------------------------
*@brief		取得链接质量指数
*@detail 	该指数表明解码的难易程度，其数值越高表示越容易解码，链接质量越高
*@param		无
*@retval	LQI
-----------------------------------------------------------------------*/
uint8_t CC1101_GetLQI(void)
{
	uint8_t LQI = SPIgetMaskedRegValue(CC1101_REG_LQI,6,0);
					//寄存器内原始数据最高位为CRC_OK标志，将其去掉取得低6位作为LQI
	return(LQI);	//返回LQI，范围0-64，数值越大表示解码越容易
}
/*-----------------------------------------------------------------------
*@brief		取得RSSI(接收信号强度指示)
*@detail 	RSSI为当前信道接收信号功率值的估计，取决于LNA的增益设置
*@param		无
*@retval	RSSI
-----------------------------------------------------------------------*/
float CC1101_GetRSSI(void)
{
  uint8_t rawRSSI = SPIreadRegister(CC1101_REG_RSSI);
  float rssi;
  if(rawRSSI >= 128) {
    rssi = (((float)rawRSSI - 256.0)/2.0) - 74.0;
  } else {
    rssi = (((float)rawRSSI)/2.0) - 74.0;
  }
  return(rssi);	
}
/*-----------------------------------------------------------------------
*@brief		设置前导码长度
*@detail 	
*@param		preambleLength - 前导码长度（字节数）
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetPreambleLength(uint8_t preambleLength)
{
  uint8_t value;
  switch(preambleLength){
    case 2:
      value = CC1101_NUM_PREAMBLE_2;
      break;
    case 3:
      value = CC1101_NUM_PREAMBLE_3;
      break;
    case 4:
      value = CC1101_NUM_PREAMBLE_4;
      break;
    case 6:
      value = CC1101_NUM_PREAMBLE_6;
      break;
    case 8:
      value = CC1101_NUM_PREAMBLE_8;
      break;
    case 12:
      value = CC1101_NUM_PREAMBLE_12;
      break;
    case 16:
      value = CC1101_NUM_PREAMBLE_16;
      break;
    case 24:
      value = CC1101_NUM_PREAMBLE_24;
      break;
    default:
      return(RADIO_ERR_INVALID_PREAMBLE_LENGTH);
  }
  return SPIsetMaskedRegValue(CC1101_REG_MDMCFG1, value, 6, 4);
}
/*-----------------------------------------------------------------------
*@brief		设置同步字及启用同步字过滤功能
*@detail 	先设置16位同步字，后启用同步字过滤功能
*@param		SyncH - 同步码的高8位；SyncL - 同步码的低8位；
*           maxErrBits - 同步码过滤时最大允许错误位数，0或1；
*           requireCarrierSense - 同步码过滤时是否同时启用载波检测
*           启用载波检测后，需要同步码正确接收指定的位数，同时检测到载波，
*           方可继续接收后续。
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetSyncWord(uint8_t SyncH, uint8_t SyncL, uint8_t maxErrBits,
							bool requireCarrierSense)
{
	if((maxErrBits>1)||(SyncH==0)||(SyncL==0))
		return(RADIO_ERR_INVALID_SYNC_WORD);//同步字不能为0，允许错误位数不能大于1
	//启用同步字过滤功能
	int8_t state = CC1101_EnableSyncWordFiltering(maxErrBits,
												   requireCarrierSense);
	RADIO_ASSERT(state);

	//写入同步字寄存器
	state = SPIsetMaskedRegValue(CC1101_REG_SYNC1,SyncH,7,0);
	state |= SPIsetMaskedRegValue(CC1101_REG_SYNC0,SyncL,7,0);
	return(state);
}
/*-----------------------------------------------------------------------
*@brief		设置射频发射功率
*@detail 	
*@param		power - 发射功率，单位dBm
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetOutputPower(int8_t power)
{
	uint8_t f;	//频率范围，作为查表的列
	uint8_t powerRaw;	//功率参数，要写入寄存器的原始数据
	//功率查表，以功率档位为行，以频率范围为列
	uint8_t paTable[8][4] = {{0x12, 0x12, 0x03, 0x03},
							{0x0D, 0x0E, 0x0F, 0x0E},
							{0x1C, 0x1D, 0x1E, 0x1E},
							{0x34, 0x34, 0x27, 0x27},
							{0x51, 0x60, 0x50, 0x8E},
							{0x85, 0x84, 0x81, 0xCD},
							{0xCB, 0xC8, 0xCB, 0xC7},
							{0xC2, 0xC0, 0xC2, 0xC0}};

	if(_freq < 374.0)	//将已保存的频率划分范围，便于稍后查表
		f = 0;    // 315 MHz
	else if(_freq < 650.5)
		f = 1;    // 434 MHz
	else if(_freq < 891.5)
		f = 2;    // 868 MHz
	else
		f = 3;    // 915 MHz
		
	switch(power)
	{
		case -30: powerRaw = paTable[0][f]; break;
		case -20: powerRaw = paTable[1][f]; break;
		case -15: powerRaw = paTable[2][f]; break;
		case -10: powerRaw = paTable[3][f]; break;
		case 0: powerRaw = paTable[4][f]; break;
		case 5: powerRaw = paTable[5][f]; break;
		case 7: powerRaw = paTable[6][f]; break;
		case 10: powerRaw = paTable[7][f]; break;
		default:
		  return(RADIO_ERR_INVALID_OUTPUT_POWER);
	}

	_power = power;	//保存功率值，便于重新设置工作频率后更新发射功率
    // FSK模式只使用PA_TABLE[0]设置的功率作为发射功率
    return(SPIsetMaskedRegValue(CC1101_REG_PATABLE, powerRaw,7,0));
}
/*-----------------------------------------------------------------------
*@brief		设置射频工作频率
*@detail 	
*@param		freq - 射频工作频率,范围300~348,387~464,779~928MHz
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetFrequency(float freq)
{
	if(!(((freq > 300.0) && (freq < 348.0)) ||	//检查许用频率范围
	   ((freq > 387.0) && (freq < 464.0)) ||
	   ((freq > 779.0) && (freq < 928.0))))
	{
		return(RADIO_ERR_INVALID_FREQUENCY);
	}
	CC1101_GoIdle();	//使CC1101进入空闲模式

	uint32_t base = 1;
	uint32_t FRF = (freq * (base << 16)) / 26.0;
	int8_t state = SPIsetMaskedRegValue(CC1101_REG_FREQ2,
										 (FRF & 0xFF0000) >> 16, 7, 0);
	state |= SPIsetMaskedRegValue(CC1101_REG_FREQ1,
								  (FRF & 0x00FF00) >> 8, 7, 0);
	state |= SPIsetMaskedRegValue(CC1101_REG_FREQ0,
								  FRF & 0x0000FF, 7, 0);

	if(state == RADIO_ERR_NONE)
	{
		_freq = freq;	//保存频率，方便此后更改发射功率时候查表使用
	}
  	// 根据新频率更新发射功率 (PA数值与已选择的发射频率有关)
  	return(CC1101_SetOutputPower(_power));//初始默认发射功率-30dBm,如需更改
  					//可在后续调用CC1101_SetOutputPower进行更改
}
/*-----------------------------------------------------------------------
*@brief		设置码率,单位kbps
*@detail 	
*@param		br - 码率，单位kbps 范围0.025~600
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetBitRate(float br)
{
	RADIO_CHECK_RANGE(br, 0.025, 600.0, RADIO_ERR_INVALID_BIT_RATE);

	CC1101_GoIdle();	//使CC1101进入空闲模式

	uint8_t e = 0;
	uint8_t m = 0;
	getExpMant(br * 1000.0, 256, 28, 14, &e, &m);//计算br对应的指数和尾数
	//将指数和尾数写入寄存器
	int8_t state = SPIsetMaskedRegValue(CC1101_REG_MDMCFG4, e, 3, 0);
	state |= SPIsetMaskedRegValue(CC1101_REG_MDMCFG3, m, 7, 0);
	return(state);
}
/*-----------------------------------------------------------------------
*@brief		设置接收机滤波器带宽
*@detail 	
*@param		rxBw - 滤波器带宽，单位kHz，范围58.0~812.0kHz
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetRxBandwidth(float rxBw)
{
	RADIO_CHECK_RANGE(rxBw, 58.0, 812.0, RADIO_ERR_INVALID_RX_BANDWIDTH);

	CC1101_GoIdle();	//使CC1101进入空闲模式
	//计算br对应的指数和尾数
	for(int8_t e = 3; e >= 0; e--)
	{
		for(int8_t m = 3; m >= 0; m --)
		{
	  		float point = (CC1101_CRYSTAL_FREQ * 1000000.0)/
	  						(8 * (m + 4) * ((uint32_t)1 << e));
	  		if(abs((rxBw * 1000.0) - point) <= 1000)
	  		{
	    		//设置接收机滤波器带宽
	    		return(SPIsetMaskedRegValue(CC1101_REG_MDMCFG4,
	    									(e << 6) | (m << 4), 7, 4));
	  		}
		}
	}
  	return(RADIO_ERR_INVALID_RX_BANDWIDTH);
}
/*-----------------------------------------------------------------------
*@brief		设置FSK频偏
*@detail 	
*@param		freqDev - 频偏，单位kHz，范围1.587~380.8kHz
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_SetFrequencyDeviation(float freqDev)
{
	//如果参数freqDev=0,则设置频偏为可用的最小频偏，可供RTTY等模式使用
	if(freqDev == 0.0)
	{
		int8_t state = SPIsetMaskedRegValue(CC1101_REG_DEVIATN, 0, 6, 4);
		state |= SPIsetMaskedRegValue(CC1101_REG_DEVIATN, 0, 2, 0);
		return(state);
	}
	RADIO_CHECK_RANGE(freqDev, 1.587, 380.8,
						RADIO_ERR_INVALID_FREQUENCY_DEVIATION);
	CC1101_GoIdle();	//使CC1101进入空闲模式
	//计算freqDev对应的指数和尾数
	uint8_t e = 0;
	uint8_t m = 0;
	getExpMant(freqDev * 1000.0, 8, 17, 7, &e, &m);
	//设置FSK频偏数值
	int8_t state = SPIsetMaskedRegValue(CC1101_REG_DEVIATN, (e << 4), 6, 4);
	state |= SPIsetMaskedRegValue(CC1101_REG_DEVIATN, m, 2, 0);
	return(state);
}
/*-----------------------------------------------------------------------
*@brief		从接收FIFO中读取收到的数据包数据
*@detail 	按照MiscRegisterConfigs函数中对CC1101状态机的配置，收妥数据包后
*         	CC1101会进入IDLE状态。因此，本函数的执行是在CC1101位于IDLE态时进行，
*         	收妥并处理完数据后，需要由上层程序再次调用StartReceive()方可再次进入
*         	接收模式。
*@param		data - 收到数据包数据的保存地址;
*           actual_len - 返回实际读到的数据长度
*@retval	错误码
-----------------------------------------------------------------------*/
int8_t CC1101_ReadDataFIFO(uint8_t* data, uint32_t* actual_len)
{
	uint32_t packet_len = CC1101_GetPacketLength(true);//获取更新后的包长度
	//定长模式：从包长寄存器里取值；变长模式：读取接收FIFO的第一个字节
	//这里包长度定义为：有用数据+地址字节（若有）的总长，为有用数据长度或
	//有用数据长度+1（有地址字节时）。具体参考手册第33页左半幅的描述。
	//判断：是否已启用地址过滤功能？
	uint8_t addr_filter = SPIgetMaskedRegValue(CC1101_REG_PKTCTRL1,1,0);
	if(addr_filter != CC1101_ADR_CHK_NONE)	//如果已启用地址过滤功能
	{
		SPIreadRegister(CC1101_REG_FIFO);//那么紧接着的数据是无用的地址数据
										 //从接收FIFO中读取出并舍弃
		packet_len -= 1;	//剔除地址后，包长度也要-1，剩下的都是有用数据
	}
	//开始批量读取FIFO内包数据
	SPIreadRegisterBurst(CC1101_REG_FIFO, data, packet_len);
	*actual_len = packet_len;//返回实际读到的数据长度

	SPISendCommand(CC1101_CMD_FLUSH_RX);//清空接收FIFO
	//清除“包长已查询”标记以便CC1101_GetPacketLength再次调用时
	//可返回更新后的包长度
	_packetLengthQueried = false;
	//检查CRC是否匹配
	if (_CRC_On && (SPIgetMaskedRegValue(CC1101_REG_LQI,7,7)!=0x80))
	{
		return (RADIO_ERR_CRC_MISMATCH);
	}
	return(RADIO_ERR_NONE);
}
/*-----------------------------------------------------------------------
*@brief		进入接收模式，准备接收数据包
*@detail 	
*@param		RxCallback - 接收完成后的回调函数
*@retval	无
-----------------------------------------------------------------------*/
void CC1101_StartReceive(void (*RxCallback)(void))
{
	CC1101_GoIdle();	//使CC1101进入空闲模式
	SPISendCommand(CC1101_CMD_FLUSH_RX);//清空接收FIFO
	fpGDO2_IRQ_Callback = RxCallback;//指定接收完成时的回调函数
	//在GDO2外部中断响应函数中调用
  	SPISendCommand(CC1101_CMD_RX);//使CC1101进入接收模式
  	RADIO_DEBUG_MSG(">Start Receiving...\r\n");
}
/*-----------------------------------------------------------------------
*@brief		开始发送数据包，同时发送地址
*@detail 	
*@param		data - 要发送的数据所在地址
*           len - 有用数据的长度
*           addr - 目的地址
*           TxCallback - 发送完成后的回调函数
*@retval	无
-----------------------------------------------------------------------*/
int8_t CC1101_TransmitWithAddress(uint8_t* data, uint32_t len,
									uint8_t addr,void (*TxCallback)(void))
{
	if(len+1 > _packetLength)	//检查数据长度，因为地址占用1字节，所以可用长度
	{								//比先前设置的_packetLength小1字节
		return(RADIO_ERR_PACKET_TOO_LONG);
	}
	CC1101_GoIdle();	//使CC1101进入空闲模式
	SPISendCommand(CC1101_CMD_FLUSH_TX);//清空发送FIFO
	fpGDO2_IRQ_Callback = TxCallback;//指定发送完成时的回调函数

	//1.如果为变长数据模式，先发送包长度，为有用数据长度+1（地址字节）
	if (_packetLengthMode == CC1101_LENGTH_CONFIG_VARIABLE)
		SPIwriteRegister(CC1101_REG_FIFO,len+1);
	//2.判断：是否已启用地址过滤功能？
	uint8_t addr_filter = SPIgetMaskedRegValue(CC1101_REG_PKTCTRL1,1,0);
	if(addr_filter != CC1101_ADR_CHK_NONE)
		SPIwriteRegister(CC1101_REG_FIFO, addr);//写入地址字节
	//3.写入后续有用数据到FIFO
	SPIwriteRegisterBurst(CC1101_REG_FIFO, data, len);
	SPISendCommand(CC1101_CMD_TX);//使CC1101进入发射模式
	return(RADIO_ERR_NONE);
}
/*-----------------------------------------------------------------------
*@brief		开始发送数据包，不发送地址
*@detail 	
*@param		data - 要发送的数据所在地址
*           len - 有用数据的长度
*           TxCallback - 发送完成后的回调函数
*@retval	无
-----------------------------------------------------------------------*/
int8_t CC1101_Transmit(uint8_t* data, uint32_t len, void (*TxCallback)(void))
{
	if(len > _packetLength)	//检查数据长度,不得大于已设置的包长度
	{
		return(RADIO_ERR_PACKET_TOO_LONG);
	}
	CC1101_GoIdle();	//使CC1101进入空闲模式
	SPISendCommand(CC1101_CMD_FLUSH_TX);//清空发送FIFO
	fpGDO2_IRQ_Callback = TxCallback;//指定发送完成时的回调函数

	//1.如果为变长数据模式，先发送包长度，为有用数据长度
	if (_packetLengthMode == CC1101_LENGTH_CONFIG_VARIABLE)
		SPIwriteRegister(CC1101_REG_FIFO,len);
	//2.写入后续有用数据到FIFO
	SPIwriteRegisterBurst(CC1101_REG_FIFO, data, len);
	SPISendCommand(CC1101_CMD_TX);//使CC1101进入发射模式
	return(RADIO_ERR_NONE);
}
/*-----------------------------------------------------------------------
*@brief		CC1101初始化配置
*@detail 	
*@param		RxCallback - 接收完成后的回调函数
*@retval	无
-----------------------------------------------------------------------*/
int8_t CC1101_Setup(float freq, float br, float freqDev, float rxBw,
					 int8_t power, uint8_t preambleLength)
{
	HwSPI_Init();	//硬件SPI+4线端口初始化
	GDO2_ExtIRQ_Init();//外部中断及中断端口初始化

	//尝试识别总线上的CC1101器件
	uint8_t i = 0;
	bool flagFound = false;
	while((i < 10) && !flagFound)
	{
		uint8_t version = SPIreadRegister(CC1101_REG_VERSION);//读寄存器版本号
		if(version == 0x14)
			flagFound = true;
		else
		{
		    RADIO_DEBUG_MSG("CC1101 not found!");
		    RADIO_DEBUG_MSG(" (%d of 10 tries)\r\n",i+1);
		    RADIO_DEBUG_MSG("CC1101_REG_VERSION = 0x%02X",version);
		    RADIO_DEBUG_MSG(", expected 0x14.\r\n");
				Delay_ms(10);
		  	i++;
		}
	}

	if(!flagFound)
	{
		RADIO_DEBUG_MSG("No CC1101 found!\r\n");
		return(RADIO_ERR_CHIP_NOT_FOUND);
	}
	else
		RADIO_DEBUG_MSG("Found CC1101! (matched by CC1101_REG_VERSION == 0x14)\r\n");

	//0.先设置为IDLE态，后直接访问寄存器进行多种初始化设置，尤其是API无法完成的配置设置(重点!)
	int8_t state = MiscRegisterConfigs();
	RADIO_ASSERT(state);
	//1.设置射频工作频率
	state = CC1101_SetFrequency(freq);
	RADIO_ASSERT(state);
	//2.设置发射功率
	state = CC1101_SetOutputPower(power);
	RADIO_ASSERT(state);
	//3.设置码率
	state = CC1101_SetBitRate(br);
	RADIO_ASSERT(state);
	//4.设置接收机带宽
	state = CC1101_SetRxBandwidth(rxBw);
	RADIO_ASSERT(state);
	//5.设置FSK频偏
	state = CC1101_SetFrequencyDeviation(freqDev);
	RADIO_ASSERT(state);
	//6.设置前导码长度
	state = CC1101_SetPreambleLength(preambleLength);
	RADIO_ASSERT(state);
	//7.设置同步字及同步字过滤功能（不允许位错误，启用载波检测）
	state = CC1101_SetSyncWord(0x15,0xD8,0,true);
	RADIO_ASSERT(state);
	//8.地址过滤功能：取消
	state = CC1101_DisableAddressFiltering();
	RADIO_ASSERT(state);
	//9.设置包长模式及包长度(重点!) ：固定长度，64字节
	state = CC1101_SetPacketLengthMode(CC1101_LENGTH_CONFIG_FIXED,
										CC1101_FIFO_SIZE);
	RADIO_ASSERT(state);
	//10.设置数据调制制式：2-FSK
	state = CC1101_SetDataShaping(0);
	RADIO_ASSERT(state);
	//11.设置编码：关闭曼彻斯特编码和数据白化
	state = CC1101_SetEncoding(0);
	RADIO_ASSERT(state);
	//12.CRC校验：关闭
	state = CC1101_SetCrcFiltering(false);
	RADIO_ASSERT(state);
	//清空FISO
	SPISendCommand(CC1101_CMD_FLUSH_RX);
	SPISendCommand(CC1101_CMD_FLUSH_TX);

	return(state);
}
