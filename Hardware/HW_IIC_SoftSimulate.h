/*********************************************************************************************
文件名：IIC_SoftSimulate.h - 使用GPIO，软件模拟IIC总线协议　 
编写人：谢英男(E-mail:xieyingnan1994@163.com)　　 　
编写时间：2018年10月15日　　　　  
修改日志：　　
　　NO.1-								
**********************************************************************************************
说明：
**********************************************************************************************/
#ifndef HW_IIC_SOFTSIM_H
#define HW_IIC_SOFTSIM_H

void IIC_GPIOConfig(void);	//配置用于模拟IIC的GPIO口
void IIC_Start(void);	//发送起始信号
void IIC_Stop(void);	//发送终止信号
uint8_t IIC_WaitAck(void);//主机等待从机响应
void IIC_SendAck(void);	//主机给从机发送响应信号
void IIC_SendNAck(void);	//主机给从机发送非响应信号
void IIC_SendOneByte(uint8_t ByteData);	//发送1字节数据
uint8_t IIC_ReadOneByte(void);	//读取1字节数据

#endif
