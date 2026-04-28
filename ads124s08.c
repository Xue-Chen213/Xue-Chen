#include "gpio.h"
#include "ADS124S08.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
uint8_t Change_Flag=0;
uint8_t STATUS_Commands[3]=  {0x41,0x00,0x00};
uint8_t INPMUX_Commands0[3]= {0x42,0x00,0x01};  //AIN0  AIN1  应变
uint8_t INPMUX_Commands1[3]= {0x42,0x00,0x5C};  //AIN5  AINCOM  应变
uint8_t INPMUX_Commands2[3]= {0x42,0x00,0xAB};  //AIN10 AIN11  应变
uint8_t INPMUX_Commands3[3]= {0x42,0x00,0x23};  //AIN2  AIN3  温度
uint8_t PGA_Commands[3]=     {0x43,0x00,0x00};             //08 1倍；09 2倍；0a 4倍；0b 8倍；......0f 128倍；
uint8_t DATARATE_Commands[3]={0x44,0x00,0x3C};        //2000sps    7=100  8=200 9=400 a=800 b=1000 c=2000 d=4000 
uint8_t REF_Commands[3]=     {0x45,0x00,0x19};             //内部参考，掉电关闭
uint8_t IDACMAG_Commands[3]= {0x46,0x00,0x04};   //7--1mA
uint8_t IDACMUX_Commands[3]= {0x47,0x00,0xf4};
uint8_t VBIAS_Commands[3]=   {0x48,0x00,0x00};
uint8_t SYS_Commands[3]=     {0x49,0x00,0x00};
uint8_t GPIODAT_Commands[3]= {0x50,0x00,0x11};         //GPIO高电平输入
uint8_t GPIOCON_Commands[3]= {0x51,0x00,0x01};         //配置为GPIO

uint8_t RREG[2]={0x20,0x11};
uint8_t RDATA[2]={0x12};


int32_t ADS124S08Buff = 0;//储存数据
uint8_t ADS124S08_RREGdata[18]={0};
uint16_t ADS124S08Str1_flag=0;     //应变1数据存储标志,原来是8位，导致丢数据
uint16_t ADS124S08Str2_flag=0;	   //应变2数据存储标志,原来是8位，导致丢数据
uint16_t ADS124S08Str3_flag=0;	   //应变3数据存储标志,原来是8位，导致丢数据
uint16_t ADS124S08Temp_flag=0;     //温度数据存储标志
uint16_t Transmit_flag=0;//数据包发送标志
uint16_t INPMUX_Flag;
uint16_t check=0;

//测试用变量
uint16_t Str1_flag=0; 
uint16_t Str2_flag=0; 
uint16_t Str3_flag=0; 
uint16_t Temp_flag=0;

uint8_t group1[4]={0x01,'Z',0x07,0xD0};  //振动
uint8_t group2[4]={0x02,'Y',0x00,0xF0};  //应变
uint8_t group3[4]={0x03,'T',0x00,0x03};  //温度


uint8_t Str1_Data[5]={0xab,0,0,0,0xcd};
uint8_t Str2_Data[5]={0xac,0,0,0,0xbd};
uint8_t Str3_Data[5]={0x6A,0,0,0,0xa3};
uint8_t Temp_Data[5]={0x9B,0,0,0,0xB5};

uint8_t start1[4]={0xab};
uint8_t end1[4]={0xcd};
uint8_t start2[4]={0xac};
uint8_t end2[4]={0xbd};
uint8_t start3[4]={0xCD};
uint8_t end3[4]={0xA3};
uint8_t start4[5]={0xA9,0xBB,0xCC,0x50,0x14};
uint8_t end4[4]={0xDD,0xEE};
uint32_t zero[2]={0x0000};
uint8_t finaldata[4];
uint8_t vibdata[2];
uint16_t Channel_Flag=0;
double temp=0;
uint8_t Stopdata=0;
uint32_t Temp_data[2]={0xabcd1234};
float vib;
int convflag=0;
uint8_t start5[4]={0x40};

volatile uint8_t Packet_Ready = 0;   // 初始为0

Bearing_Data_Struct Bearing_Data={
	.header={0x5A, 0xA5,           //帧头
			 0xB5,0x0A,            //版长度识别符
			 0x02,                 //ID号
			 0xDD,                 //功能
			 0x80,0x02,            //振动个数，2000
			 0xA0,0x00,	          //应变个数，160
			 0x02,0x00             //温度个数，1	
	},
	.adcx={0},
	.ADS124S08Str1_Data={0},
	.ADS124S08Str2_Data={0},
	.ADS124S08Str3_Data={0},
	.ADS124S08Temp_Data={0},
	.pri=0,
	.end={0xDD,0xEE},
};


//void ADS124S08_Reset(void)
//{
//	//977ns  4tclk
//	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
//	for(int i = 1000; i > 0; i--){
//        __NOP();//1000/480 ns = 2.08ns
//    }
//	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
//	//4096 tclk=1ms
//	HAL_Delay(2);
//}


void ADS124S08_Start(void)
{
	HAL_GPIO_WritePin(ADS124S08_START_GPIO_Port, ADS124S08_START_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
}


void ADS124S08_Stop_it(void)
{
	HAL_GPIO_WritePin(ADS124S08_START_GPIO_Port, ADS124S08_START_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
}

void ADS124S08REG_Init(void)   //寄存器初始化
{
	//HAL_GPIO_WritePin(START_GPIO_Port, START_Pin, GPIO_PIN_SET);
	//HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)PGA_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DATARATE_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)REF_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)IDACMAG_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)IDACMUX_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)GPIOCON_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)GPIODAT_Commands,3,1000);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)INPMUX_Commands0,3,1000);
	//HAL_GPIO_WritePin(START_GPIO_Port, START_Pin, GPIO_PIN_RESET);
	//HAL_Delay(10);
	
//  //写寄存器和回读寄存器
//  for(int i=0;i<18;i++)
//  {
//	  printf("%x  ",ADS124S08_RREGdata[i]);
//  }
//  ADS124S08_ReadReg((unsigned char)0x20,(unsigned char *)ADS124S08_RREGdata,18);  

//  for(int i=0;i<18;i++)
//  {
//	  printf("%x  ",ADS124S08_RREGdata[i]);
//  }
//  printf("寄存器初始化成功\n");
}

//通道转换
void ADS124S08INPMUX_Cov(uint16_t INPMUX_Flag)
{
    // 1. 拉低CS，选中芯片（必须！原函数缺少这步，指令发不进去）
    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);        // 满足CS拉低后≥20ns的时序要求
    
	switch (INPMUX_Flag)
	{
		case 0:
			HAL_SPI_Transmit(&hspi1,(uint8_t *)INPMUX_Commands0,3,10000);
			break;
		
		case 1:
			HAL_SPI_Transmit(&hspi1,(uint8_t *)INPMUX_Commands1,3,10000);
			break;
				
		case 2:
			HAL_SPI_Transmit(&hspi1,(uint8_t *)INPMUX_Commands2,3,10000);
			break;
						
		case 3:
			HAL_SPI_Transmit(&hspi1,(uint8_t *)INPMUX_Commands3,3,10000);
			break;
		
		default:
			break;	
	}
        // 3. 等待指令写入完成（手册要求：写寄存器后需等待至少4个内部时钟周期）
    HAL_Delay(1);

    // 4. 拉高CS，释放芯片（必须！原函数缺少这步）
    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1); // 满足CS拉高前≥20ns的时序要求

//    // 5. 可选：写后立即读取一次，验证配置是否生效（调试用）
//    uint8_t verify_data = 0;
//    ADS124S08_ReadReg(0x22, &verify_data, 1);
//    if (verify_data != INPMUX_Commands0[2] && INPMUX_Flag == 0) 
//    {
//        // 调试断点：若仍不匹配，检查指令值或SPI通信
//    }
}

void ADS124S08_ReadData(void)
{
	ADS124S08_ReadReg((unsigned char)0x22,(unsigned char *)ADS124S08_RREGdata,1);  
		  
		if(ADS124S08_RREGdata[0]==0x01)
		{				
			Str1_flag++;	
//            printf("通道: 0x%02X\r\n", ADS124S08_RREGdata[0]);
            
			ADS124S08Buff=ADS124S08_Read();
			finaldata[0]=(ADS124S08Buff>>16)&0xff;
			finaldata[1]=(ADS124S08Buff>>8)&0xff;
			finaldata[2]=(ADS124S08Buff)&0xff;
//			Str1_Data[1]=(ADS124S08Buff>>16)&0xff;
//			Str1_Data[2]=(ADS124S08Buff>>8)&0xff;
//			Str1_Data[3]=(ADS124S08Buff)&0xff;
			Bearing_Data.ADS124S08Str1_Data[ADS124S08Str1_flag++]=finaldata[0];	
			Bearing_Data.ADS124S08Str1_Data[ADS124S08Str1_flag++]=finaldata[1];			
			Bearing_Data.ADS124S08Str1_Data[ADS124S08Str1_flag++]=finaldata[2];
			//printf("应1%d\r\n",ADS124S08Buff);
            INPMUX_Flag=1;
			ADS124S08INPMUX_Cov(INPMUX_Flag);      //通道转换
//            HAL_Delay(5);
            ADS124S08_Stop_it();
//			单通道输出测试用			
//			string1_temp=ADS124S08ConvValue(ADS124S08Buff);
			//if(ADS124S08Buff < 2500000 && ADS124S08Buff > -3000000){
//            HAL_UART_Transmit_DMA(&huart1,Str1_Data,sizeof(Str1_Data));
//            HAL_UART_Transmit(&huart1,Str1_Data,sizeof(Str1_Data),10000);
			//}
			
//			HAL_UART_Transmit(&huart1,(uint8_t *)start1,1,1000);
//			HAL_UART_Transmit(&huart1,(uint8_t *)finaldata,3,1000);
//			HAL_UART_Transmit(&huart1,(uint8_t *)end1,1,1000);
//			printf("%f\r\n",string1_temp);
//			HAL_UART_Transmit(&huart3,(uint8_t *)start,1,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)finaldata,3,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)end1,1,1000);
		}
		
		else if(ADS124S08_RREGdata[0]==0x5C)
		{
			Str2_flag++;
			//printf("应2%d\r\n",Str2_flag); 
			//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Str2_flag,sizeof(Str2_flag));
//            printf("通道: 0x%02X\r\n", ADS124S08_RREGdata[0]);
			ADS124S08Buff=ADS124S08_Read();
			finaldata[0]=(ADS124S08Buff>>16)&0xff;
			finaldata[1]=(ADS124S08Buff>>8)&0xff;
			finaldata[2]=(ADS124S08Buff)&0xff;
//			Str2_Data[1]=(ADS124S08Buff>>16)&0xff;
//			Str2_Data[2]=(ADS124S08Buff>>8)&0xff;
//			Str2_Data[3]=(ADS124S08Buff)&0xff;
			Bearing_Data.ADS124S08Str2_Data[ADS124S08Str2_flag++]=finaldata[0];	
			Bearing_Data.ADS124S08Str2_Data[ADS124S08Str2_flag++]=finaldata[1];			
			Bearing_Data.ADS124S08Str2_Data[ADS124S08Str2_flag++]=finaldata[2];
            INPMUX_Flag=2;
			ADS124S08INPMUX_Cov(INPMUX_Flag);      //通道转换
//            HAL_Delay(5);
            ADS124S08_Stop_it();
//            HAL_UART_Transmit_DMA(&huart1,Str2_Data,sizeof(Str2_Data));
//            HAL_UART_Transmit(&huart1,Str2_Data,sizeof(Str1_Data),10000);
//			单通道输出测试用				
////			string2_temp=ADS124S08ConvValue(ADS124S08Buff);
//			HAL_UART_Transmit(&huart1,(uint8_t *)start2,1,1000);
//			HAL_UART_Transmit(&huart1,(uint8_t *)finaldata,3,1000);
//			HAL_UART_Transmit(&huart1,(uint8_t *)end2,1,1000);
			//printf("%f\r\n",string2_temp);
//			HAL_UART_Transmit(&huart3,(uint8_t *)start2,1,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)finaldata,3,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)end2,1,1000);
		}
		
		else if(ADS124S08_RREGdata[0]==0xAB)
		{	
			Str3_flag++;
			//printf("应3%d\r\n",Str3_flag); 
			//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Str3_flag,sizeof(Str3_flag));
			//HAL_UART_Transmit(&huart1,(uint8_t *)start,1,1000);
            
//            printf("通道: 0x%02X\r\n", ADS124S08_RREGdata[0]);
            
			ADS124S08Buff=ADS124S08_Read();
			finaldata[0]=(ADS124S08Buff>>16)&0xff;
			finaldata[1]=(ADS124S08Buff>>8)&0xff;
			finaldata[2]=(ADS124S08Buff)&0xff;
//			Str3_Data[1]=(ADS124S08Buff>>16)&0xff;
//			Str3_Data[2]=(ADS124S08Buff>>8)&0xff;
//			Str3_Data[3]=(ADS124S08Buff)&0xff;
			Bearing_Data.ADS124S08Str3_Data[ADS124S08Str3_flag++]=finaldata[0];	
			Bearing_Data.ADS124S08Str3_Data[ADS124S08Str3_flag++]=finaldata[1];			
			Bearing_Data.ADS124S08Str3_Data[ADS124S08Str3_flag++]=finaldata[2];
			if(ADS124S08Str3_flag==480)
			{
				INPMUX_Flag=3;
				ADS124S08INPMUX_Cov(INPMUX_Flag);      //通道转换
//                HAL_Delay(5);
//				ADS124S08_Start();
			}
			else INPMUX_Flag=0;
			ADS124S08INPMUX_Cov(INPMUX_Flag);      //通道转换
//            HAL_Delay(5);
			//if(ADS124S08Buff < 3500000 && ADS124S08Buff > -4000000){
//				HAL_UART_Transmit_DMA(&huart1,Str3_Data,sizeof(Str3_Data));
			//}
            ADS124S08_Stop_it();
//			HAL_UART_Transmit_DMA(&huart1,Str3_Data,sizeof(Str3_Data));
//			ADS124S08_Start();
////			单通道输出测试用				
//			HAL_UART_Transmit(&huart1,(uint8_t *)start3,1,1000);
//			HAL_UART_Transmit(&huart1,(uint8_t *)finaldata,3,1000);
//			HAL_UART_Transmit(&huart1,(uint8_t *)end3,1,1000);
//			//printf("%d\r\n",ADS124S08Str3_Data[ADS124S08Str3_flag-1]);
			
//	 		HAL_UART_Transmit(&huart3,(uint8_t *)start3,1,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)finaldata,3,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)end3,1,1000);
		}
		
		else if(ADS124S08_RREGdata[0]==0x23)   
		{	
//				Temp_flag++;
				//printf("温%d\r\n",Temp_flag); 
				//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Temp_flag,sizeof(Temp_flag));   
//                printf("通道: 0x%02X\r\n", ADS124S08_RREGdata[0]);
            
				ADS124S08Buff=ADS124S08_Read();
				finaldata[0]=(ADS124S08Buff>>16)&0xff;
				finaldata[1]=(ADS124S08Buff>>8)&0xff;
				finaldata[2]=(ADS124S08Buff)&0xff;
			
//				Temp_dataArr[FILTER4_N]=ADS124S08Buff;				
//				ADS124S08Buff=Tempdata_filter();
//				finaldata[0]=(ADS124S08Buff>>16)&0xff;
//				finaldata[1]=(ADS124S08Buff>>8)&0xff;
//				finaldata[2]=(ADS124S08Buff)&0xff;
//				Temp_Data[1]=(ADS124S08Buff>>16)&0xff;
//				Temp_Data[2]=(ADS124S08Buff>>8)&0xff;
//				Temp_Data[3]=(ADS124S08Buff)&0xff;
				Bearing_Data.ADS124S08Temp_Data[ADS124S08Temp_flag++]=finaldata[0];	
				Bearing_Data.ADS124S08Temp_Data[ADS124S08Temp_flag++]=finaldata[1];			
				Bearing_Data.ADS124S08Temp_Data[ADS124S08Temp_flag++]=finaldata[2];
				//if(ADS124S08Buff > 0 && ADS124S08Buff < 8000000){
//				HAL_UART_Transmit_DMA(&huart1,Temp_Data,sizeof(Temp_Data));
				//}
				//HAL_UART_Transmit_DMA(&huart1,Temp_Data,sizeof(Temp_Data));
//				单通道输出测试用					
//				data_temp=ADS124S08ConvValue(ADS124S08Buff);
//				printf("%f\r\n",data_temp);
//				HAL_UART_Transmit(&huart1,(uint8_t *)start4,3,1000);
//				HAL_UART_Transmit(&huart1,(uint8_t *)finaldata,3,1000);
//				HAL_UART_Transmit(&huart1,(uint8_t *)end4,2,1000);			
//				HAL_UART_Transmit(&huart2,(uint8_t *)start4,5,1000);
//				HAL_UART_Transmit(&huart2,(uint8_t *)finaldata,3,1000);
//				HAL_UART_Transmit(&huart2,(uint8_t *)end4,1,1000);
                INPMUX_Flag=0;
				ADS124S08INPMUX_Cov(INPMUX_Flag);  	
//                HAL_Delay(5);
                ADS124S08_Stop_it();
                
//                Packet_Ready = 1;                    // 告诉主程序：可以发了！
                
				ADS124S08Temp_flag=0;	
				ADS124S08Str1_flag=0;
				ADS124S08Str2_flag=0;
				ADS124S08Str3_flag=0;
				Str1_flag=0;
				Str2_flag=0;
				Str3_flag=0;
				Temp_flag=0;	
				//Tdata_Process();
                
		}
//				HAL_UART_Transmit(&huart1,(uint8_t *)start4,3,1000);
//				HAL_UART_Transmit(&huart1,(uint8_t *)finaldata,3,1000);
//				HAL_UART_Transmit(&huart1,(uint8_t *)end4,2,1000);	
	
}

void Tdata_Process(void)
{
	//校验码计算
	for(int i=0;i<2560;i++)
	{
		check+=(Bearing_Data.adcx[i]>>8);
		check+=(Bearing_Data.adcx[i]&0xFF);
	}
	for(int i=0;i<480;i++)
	{
		check+=Bearing_Data.ADS124S08Str1_Data[i];
		check+=Bearing_Data.ADS124S08Str2_Data[i];
		check+=Bearing_Data.ADS124S08Str3_Data[i];
	}
	check+=Bearing_Data.ADS124S08Temp_Data[0];
	check+=Bearing_Data.ADS124S08Temp_Data[1];
	check+=Bearing_Data.ADS124S08Temp_Data[2];
	for(int i=0;i<12;i++)
	{
		check+=Bearing_Data.header[i];
	}
	Bearing_Data.pri=(check&0xFF);
	
//	HAL_UART_Transmit(&huart1,(uint8_t *)Bearing_Data.header,12,1000);//数据包头
//	HAL_UART_Transmit(&huart1,(uint8_t *)Bearing_Data.adcx,4000,1000);//振动数据
////	HAL_UART_Transmit(&huart2,(uint8_t *)start4,5,1000);
////	HAL_UART_Transmit(&huart2,(uint8_t *)ADS124S08Str1_Data,160,1000);//应变1数据
////	HAL_UART_Transmit(&huart1,(uint8_t *)start4,5,1000);
////	HAL_UART_Transmit(&huart1,(uint8_t *)ADS124S08Str1_Data,160,1000);//应变1数据
//	for(int i=0;i<160;i++)
//	{
//		HAL_UART_Transmit(&huart1,(uint8_t *)&Bearing_Data.ADS124S08Str1_Data[3*i],3,1000);//应变1数据
//		HAL_UART_Transmit(&huart1,(uint8_t *)&Bearing_Data.ADS124S08Str2_Data[3*i],3,1000);//应变2数据
//		HAL_UART_Transmit(&huart1,(uint8_t *)&Bearing_Data.ADS124S08Str3_Data[3*i],3,1000);//应变3数据
//	}

//	HAL_UART_Transmit(&huart1,(uint8_t *)Bearing_Data.ADS124S08Temp_Data,13,1000);//温度数据及预留数据
//	
//	HAL_UART_Transmit(&huart1,(uint8_t *)&Bearing_Data.pri,1,1000);//校验码
//	HAL_UART_Transmit(&huart1,(uint8_t *)Bearing_Data.end,2,1000);//帧尾

}
/*********************************参考****************************************************************/

////读寄存器
void ADS124S08_ReadReg(unsigned char RegAddr,unsigned char *Buffer,unsigned char Length)  
{  
    unsigned char Cmd[2];  
		
    AD_nCS_LOW;
	HAL_Delay(1);
	
//		AD_START_HIGH;		//在写寄存器时吗，需要将START拉高(不让其进入睡眠模式)
	//HAL_Delay(10);	
	 
    Cmd[0]=ADC_CMD_RREG|RegAddr;  
    Cmd[1]=Length-1;  
	
	HAL_SPI_Transmit(&hspi1,Cmd,2,HAL_MAX_DELAY);		//发送命令		
	
	HAL_SPI_Receive(&hspi1, Buffer, Length, HAL_MAX_DELAY);		//接收寄存器数据
	
	Cmd[0]=ADC_CMD_NOP;  
    HAL_SPI_Transmit(&hspi1, Cmd,1,HAL_MAX_DELAY); 	//最后在发送一个NOP，强制拉高DOUT
	
    HAL_Delay(1);
	AD_nCS_HIGH;
}


////写寄存器
void ADS124S08_WriteReg(unsigned char RegAddr,unsigned char *Buffer,unsigned char Length)  
{  
    unsigned char Cmd[2];  
		
    AD_nCS_LOW;
	HAL_Delay(5);		
		AD_START_HIGH;		//在写寄存器时吗，需要将START拉高(不让其进入睡眠模式)
      HAL_Delay(10);			//硬件延迟
	
    Cmd[0]=ADC_CMD_WREG|RegAddr;  
    Cmd[1]=Length-1; 
	
	  HAL_SPI_Transmit(&hspi1, Cmd, 2,HAL_MAX_DELAY); 	//指定向指定寄存器写入指定字节数据
	  HAL_SPI_Transmit(&hspi1, Buffer, Length,HAL_MAX_DELAY); 	//发送数据字节
		
		 HAL_Delay(10);			//硬件延迟
		
    AD_nCS_HIGH; 
		AD_START_LOW;		
		
}

//读取ADS124S08中的转换数据
int32_t ADS124S08_Read()
{
        unsigned char  Cmd[5]={ADC_CMD_RDATA,ADC_CMD_NOP,ADC_CMD_NOP,ADC_CMD_NOP,ADC_CMD_NOP};  //最后一个字节是为了强制拉高nDRDY
        unsigned char  Buf[5];
        int32_t  Data = 0;  
	  
        AD_nCS_LOW;
        HAL_Delay(1);
    
		HAL_SPI_TransmitReceive(&hspi1,Cmd,Buf,5,HAL_MAX_DELAY);

        HAL_Delay(1);
        AD_nCS_HIGH;

    	Data=Buf[1];
		Data=Data*256+Buf[2];
		Data=Data*256+Buf[3];

		Data = Data*256;				//先乘再除是为了保留正负号
		return (Data/256); 
}

//写命令
//static void ADS124S08_WriteCmd(unsigned char Cmd)  
//{   
//	
//        AD_nCS_LOW;		//拉低片选线，使能SPI通信
//				
//		HAL_SPI_Transmit(&hspi1, &Cmd, 1,HAL_MAX_DELAY);
//    
//		AD_nCS_HIGH;		//通信结束，拉高片选
//    
//} 

//void ADS124S08_Chg_Channel(unsigned char channel)
//{
//    unsigned char Cmd;
//    switch (channel)
//        {
//        case 0:
//        Cmd = (POSI_AIN(0)|NEGA_AIN(1));												
//		ADS124S08_WriteReg(ADC_REG_INPMUX,&Cmd,1);
//            break;
//        case 1:
//        Cmd = (POSI_AIN(2)|NEGA_AIN(3));												
//		ADS124S08_WriteReg(ADC_REG_INPMUX,&Cmd,1);
//            break;
//        case 2:
//        Cmd = (POSI_AIN(4)|NEGA_AIN(5));												
//		ADS124S08_WriteReg(ADC_REG_INPMUX,&Cmd,1);
//            break;
//        case 3:
//        Cmd = (POSI_AIN(6)|NEGA_AIN(7));												
//		ADS124S08_WriteReg(ADC_REG_INPMUX,&Cmd,1);
//            break; 
//        default:break;
//        }
//}

//判断忙状态
 unsigned char ADS124S08_WaitBusy(unsigned int Timeout)  
{  
    unsigned int i = 0; 
		AD_nCS_LOW;
		while(nAD_DRDY_STATE > 0)
			{
				HAL_Delay(1);
					i++; 
				if(i>Timeout)
					return 1;   

			}
	  
		AD_nCS_HIGH;
					
		return 0;
	
} 

//ADS124S08系统校准  校准顺序为：自偏移校准->偏移校准->增益校准 .
//校准命令不能在设备处于待机模式时使用(当START/SYNC引脚低，或当停止命令发出时)。
static unsigned char ADS124S08_Calibrate(unsigned char Gain)  
{  
		unsigned char R=0;
    unsigned char Cmd;  
//    ADS124S08_WriteReg(ADC_REG_PGA,&Gain,1);      // 设置增益值、ADC输出数据率
	
		Cmd=0x18;  	//采样4次
		ADS124S08_WriteReg(ADC_REG_SYS,&Cmd,1);       // 设置系统监测为自偏移测量 
//		ADS124S08_WriteCmd(ADC_CMD_SELFOCAL);          // 自偏移校准  
		R |= ADS124S08_WaitBusy(500);    							 // 等待校准完成 
						
			
//		Cmd=0x21;  //0010 0001
//		ADS124S08_WriteReg(ADC_REG_MUX1,&Cmd,1);       // 设置系统监测为偏移测量  
//		ADS124S08_WriteCmd(ADC_CMD_SYSOCAL);           // 系统偏移校准  
//		R |= ADS124S08_WaitBusy(500);   						  // 等待校准完成 
//			
//		
//		Cmd=0x22;  
//		ADS124S08_WriteReg(ADC_REG_MUX1,&Cmd,1);       // 设置系统监测为增益测量  
//		ADS124S08_WriteCmd(ADC_CMD_SYSGCAL);           // 系统增益校准  
//		R |= ADS124S08_WaitBusy(500);    							 // 等待校准完成  
		
		return R;
}


////ADS124S08初始化
//void ADS124S08_Init(void)  
//{  
//		unsigned char Cmd;
//		unsigned char Gain;
//		
//		ADS124S08_Reset();  																	 //系统复位
//		
//		HAL_Delay(20);	
//		
//		Gain = 11;//ADC_GAIN_EN;
//	//uint8_t INPMUX_Commands[3]={0x42,0x00,0x00};

//		//初始化MUX0多路复用控制寄存器
//		Cmd = (POSI_AIN(0)|NEGA_AIN(1));				//00 010 111,Bit7-6:传感器电流源检测不使用，Bit5-3:正输入为AIN2，Bit2-0:负输入为AIN7		
//		ADS124S08_WriteReg(ADC_REG_INPMUX,&Cmd,1); 

//		ADS124S08_WriteReg(ADC_REG_PGA,&Gain,1);           					 // 设置增益值、ADC输出数据率  8倍增益
//		
//		Cmd=0x20|ADC_SPS_50 ;//关闭斩波模式,单次转换,滤波,4k采样,50/60hz陷波要60sps以下.
//		ADS124S08_WriteReg(ADC_REG_DATARATE,&Cmd,1);            	// 将MUX1置，(内部晶振时钟源，启动内部参考电压，选择REF0作为参考电平，普通操作)																												
//                                                                  // 校准时MUX1将被重新赋值，因此这里可以不用对其进行赋值，校准之后再配置内部参考电压
//        //当选择内部基准进行测量时，建议禁用基准缓冲区。0x30	

//        Cmd = 2<<2 | 2<<0 |0x30;
//        ADS124S08_WriteReg(ADC_REG_REF,&Cmd,1);

////        Cmd=0x07 ;//0000 0111																		// 设置极大恒流源电流值1500uA（1.5mA）
////		ADS124S08_WriteReg(ADC_REG_IDACMAG,&Cmd,1);   				
////		
////		Cmd=0x17 ;//0010 0111																	// 选择第一个恒流源输出引脚 (AIN2) 选择第二个电流源输出引脚（AIN7）
////		ADS124S08_WriteReg(ADC_REG_IDACMUX,&Cmd,1);   					
//	
//		Cmd=ADS124S08_Calibrate(Gain);                        // 通道校准.配置转换参数  

////		Cmd = 0x03;
////		ADS124S08_WriteReg(ADC_REG_GPIODAT,&Cmd,1);
////		
////	  Cmd = 0x0f;
////		ADS124S08_WriteReg(ADC_REG_GPIOCON,&Cmd,1);

//		//重新配置MUX1
////		Cmd=0x20|ADC_SPS_4000;  //0011 0000
////		ADS124S08_WriteReg(ADC_REG_DATARATE,&Cmd,1);							// 启用内部参考电压总是开启

//		AD_START_LOW;

//} 








