/*********************************************************************************************
文件名：IIC_SoftSimulate.c - 使用GPIO，软件模拟IIC总线协议　 
编写人：谢英男(E-mail:xieyingnan1994@163.com)　　 　
编写时间：2018年10月15日　　　　  
修改日志：　　
　　NO.1-								
**********************************************************************************************
说明：
**********************************************************************************************/
#include "Hardware.h"
/*---------------本模块私有的宏、类型、函数声明、变量----------------------*/
#define IIC_GPIO_PORT	GPIOB	//IIC总线GPIO端口
#define IIC_GPIO_CLOCK	RCC_APB2Periph_GPIOB	//GPIO端口时钟
#define IIC_SCL_PIN		GPIO_Pin_6	//SCL时钟线对应的GPIO引脚
#define IIC_SDA_PIN		GPIO_Pin_7	//SDA数据线对应的GPIO引脚

#define IIC_SDA_1()		IIC_GPIO_PORT->BSRR = IIC_SDA_PIN	//SDA置位
#define IIC_SDA_0()		IIC_GPIO_PORT->BRR = IIC_SDA_PIN	//SDA复位
#define IIC_SCL_1()		IIC_GPIO_PORT->BSRR = IIC_SCL_PIN	//SCL置位
#define IIC_SCL_0()		IIC_GPIO_PORT->BRR = IIC_SCL_PIN	//SCL复位
#define IIC_SDA_Read()	((uint16_t)(IIC_GPIO_PORT->IDR & IIC_SDA_PIN))	//读取SDA线状态

static void IIC_Delay(void);	//IIC专用的延时程序，用于产生总线时钟
/*---------------------------------------------------------------------------
函数名：IIC_GPIOConfig
功能：配置用于模拟IIC的GPIO口
输入参数：无
返回值：无
---------------------------------------------------------------------------*/
void IIC_GPIOConfig(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(IIC_GPIO_CLOCK,ENABLE);//打开时钟
	
	GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN|IIC_SDA_PIN;//选择引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//开漏输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设定速率
	GPIO_Init(IIC_GPIO_PORT,&GPIO_InitStructure);//初始化GPIO
}
/*---------------------------------------------------------------------------
函数名：IIC_Delay
功能：IIC专用延时程序用于产生总线时钟
输入参数：无
返回值：无
---------------------------------------------------------------------------*/
static void IIC_Delay(void)
{
	uint8_t	i;
	for(i = 0;i < 8;i++);
	/*
	循环10次, SCL频率 = 205kHz;循环7次,SCL频率 = 347kHz;
	循环5次，SCL频率 = 421kHz;
	*/
}
/*---------------------------------------------------------------------------
函数名：IIC_Start
功能：发送起始信号
输入参数：无
返回值：无
---------------------------------------------------------------------------*/
void IIC_Start(void)
{
	IIC_SDA_1();
	IIC_SCL_1();
	
	IIC_Delay();
	IIC_SDA_0();	//在SCL为高期间SDA出现下降沿,主机产生起始信号
	IIC_Delay();
	
	IIC_SCL_0();
	IIC_Delay();
}
/*---------------------------------------------------------------------------
函数名：IIC_Stop
功能：发送终止信号
输入参数：无
返回值：无
---------------------------------------------------------------------------*/
void IIC_Stop(void)
{	
	IIC_SCL_0();
	IIC_SDA_0();
	IIC_Delay();
	IIC_SCL_1();
	IIC_Delay();	//SCL为高时，SDA出现上升沿，主机发送停止信号
	IIC_SDA_1();
	IIC_Delay();
}
/*---------------------------------------------------------------------------
函数名：IIC_SendAck
功能：主机给从机发送响应信号
输入参数：无
返回值：无
---------------------------------------------------------------------------*/
void IIC_SendAck(void)
{
	IIC_SDA_0();//驱动SDA=0
	IIC_Delay();
	
	IIC_SCL_1();//产生一个时钟周期
	IIC_Delay();
	IIC_SCL_0();
	IIC_Delay();
	
	IIC_SDA_1();//释放SDA总线
}
/*---------------------------------------------------------------------------
函数名：IIC_SendNAck
功能：主机给从机发送非响应信号
输入参数：无
返回值：无
---------------------------------------------------------------------------*/
void IIC_SendNAck(void)
{
	IIC_SDA_1();//驱动SDA=1
	IIC_Delay();
	
	IIC_SCL_1();//产生一个时钟周期
	IIC_Delay();
	IIC_SCL_0();
	IIC_Delay();
}
/*---------------------------------------------------------------------------
函数名：IIC_WaitAck
功能：主机等待从机响应
输入参数：无
返回值：0 - 器件正确应答，1 - 无器件响应
---------------------------------------------------------------------------*/
uint8_t IIC_WaitAck(void)
{
	uint8_t Response;
	
	IIC_SDA_1();//释放SDA线
	IIC_Delay();
	IIC_SCL_1();
	IIC_Delay();	
	
	if(IIC_SDA_Read()){//读取响应
		Response = 1;
	}//NACK
	else{
		Response = 0;
	}//ACK	

	IIC_SCL_0();
	IIC_Delay();
	
	return Response;
}
/*---------------------------------------------------------------------------
函数名：IIC_SendOneByte
功能：发送1字节数据
输入参数：uint8_t ByteData - 要发送的数据
返回值：无
---------------------------------------------------------------------------*/
void IIC_SendOneByte(uint8_t ByteData)
{
	uint8_t i;
	
	for(i=0;i < 8;i++)
	{
		if(ByteData & 0x80){//先从MSB发送
			IIC_SDA_1();
		}
		else{
			IIC_SDA_0();
		}
		IIC_Delay();
		IIC_SCL_1();
		IIC_Delay();
		IIC_SCL_0();
		if(i == 7){//发送完了LSB，释放SDA总线
			IIC_SDA_1();
		}
		ByteData <<= 1;//左移一个bit
		IIC_Delay();
	}
}
/*---------------------------------------------------------------------------
函数名：IIC_ReadOneByte
功能：发送1字节数据
输入参数：无
返回值：读取到的1字节数据
---------------------------------------------------------------------------*/
uint8_t IIC_ReadOneByte(void)
{
	uint8_t i;
	uint8_t value = 0;
	
	for(i=0;i < 8;i++)
	{
		value <<= 1;	//左移一位腾出地方，接收顺序为MSB->LSB
		IIC_SCL_1();
		IIC_Delay();
		if(IIC_SDA_Read()){//在一个SCL时钟周期内读取SDA状态
			value++;
		}
		IIC_SCL_0();
		IIC_Delay();
	}
	return value;
}
