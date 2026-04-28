#ifndef      __KX134_H__
#define	     __KX134_H__

#include "main.h"

/* ===================== 硬件接口定义 ===================== */
// 片选信号（SPI用）——拉低选中KX134
#define KX134_CS_Pin                GPIO_PIN_1
#define KX134_CS_GPIO_Port          GPIOB

// 中断引脚（KX134 → MCU）——数据就绪/事件触发
#define KX134_INT_Pin               GPIO_PIN_0
#define KX134_INT_GPIO_Port         GPIOB

/* ===================== 寄存器地址定义 ===================== /
/
说明：
这些宏定义的是 KX134 内部寄存器地址
通过 SPI 读写这些地址可以完成：
配置工作模式、读取三轴数据、设置中断、FIFO/缓存控制 等
*/
// ----------- 基本信息寄存器 -----------
#define  MAN_ID  		0x00
#define  PART_ID  		0x01

// ----------- 原始ADC数据（未处理）-----------
#define  XADP_L  		0x02
#define  XADP_H  		0x03
#define  YADP_L  		0x04
#define  YADP_H  		0x05
#define  ZADP_L  		0x06
#define  ZADP_H  		0x07

// ----------- 三轴加速度输出（最重要）-----------
#define  XOUT_L  		0x08
#define  XOUT_H  		0x09
#define  YOUT_L  		0x0A
#define  YOUT_H  		0x0B
#define  ZOUT_L  		0x0C
#define  ZOUT_H  		0x0D
#define  COTR  			0x12

// ----------- 芯片状态/识别 -----------
#define  WHO_AM_I  		0x13        // 固定ID（用于检测通信是否正常）

// ----------- 中断状态寄存器 -----------
#define  TSCP  			0x14
#define  TSPP  			0x15
#define  INS1  			0x16
#define  INS2  			0x17        // 常用于判断Data Ready（你现在就在用）
#define  INS3  			0x18
#define  STATUS_REG  	0x19
#define  INT_REL  		0x1A        // 中断释放寄存器（读它可清中断）

// ----------- 控制寄存器（核心配置）-----------
#define  CNTL1  		0x1B        // 工作模式（开关、量程等）
#define  CNTL2  		0x1C
#define  CNTL3  		0x1D
#define  CNTL4  		0x1E
#define  CNTL5  		0x1F
#define  CNTL6  		0x20

// ----------- 输出数据率（ODR）-----------
#define  ODCNTL  		0x21        // 设置采样频率（50Hz~25600Hz）

// ----------- 中断控制 -----------
#define  INC1  			0x22        // 中断使能
#define  INC2  			0x23
#define  INC3  			0x24
#define  INC4  			0x25        // 中断路由（映射到INT引脚）
#define  INC5  			0x26
#define  INC6  			0x27

// ----------- 倾斜/方向检测 -----------
#define  TILT_TIMER  	0x29
#define  TDTRC  		0x2A
#define  TDTC  			0x2B
#define  TTH  			0x2C
#define  TTL  			0x2D
#define  FTD  			0x2E
#define  STD  			0x2F
#define  TLT  			0x30
#define  TWS  			0x31

// ----------- 自由落体检测 -----------
#define  FFTH  			0x32
#define  FFC  			0x33
#define  FFCNTL  		0x34

// ----------- 倾角角度设置 -----------
#define  TILT_ANGLE_LL  0x37
#define  TILT_ANGLE_HL  0x38
#define  HYST_SET  		0x39

// ----------- 低功耗设置 -----------
#define  LP_CNTL1 		0x3A
#define  LP_CNTL2  		0x3B

// ----------- 唤醒功能 -----------
#define  WUFTH  		0x49
#define  BTSWUFTH  		0x4A
#define  BTSTH  		0x4B
#define  BTSC  			0x4C
#define  WUFC  			0x4D

// ----------- 自测试 -----------
#define  SELF_TEST  	0x5D

// ----------- FIFO缓存（进阶功能）-----------
#define  BUF_CNTL1  	0x5E
#define  BUF_CNTL2  	0x5F
#define  BUF_STATUS_1  	0x60
#define  BUF_STATUS_2  	0x61
#define  BUF_CLEAR  	0x62
#define  BUF_READ  		0x63

// ----------- 高级滤波/算法（一般不用）-----------
#define  ADP_CNTL1  	0x64
#define  ADP_CNTL2  	0x65
#define  ADP_CNTL3  	0x66
#define  ADP_CNTL4  	0x67
#define  ADP_CNTL5  	0x68
#define  ADP_CNTL6  	0x69
#define  ADP_CNTL7  	0x6A
#define  ADP_CNTL8  	0x6B
#define  ADP_CNTL9  	0x6C
#define  ADP_CNTL10  	0x6D
#define  ADP_CNTL11  	0x6E
#define  ADP_CNTL12  	0x6F
#define  ADP_CNTL13  	0x70
#define  ADP_CNTL14  	0x71
#define  ADP_CNTL15  	0x72
#define  ADP_CNTL16  	0x73
#define  ADP_CNTL17  	0x74
#define  ADP_CNTL18  	0x75
#define  ADP_CNTL19  	0x76
#define  INTERNAL_0X7F  0x7F        // 内部寄存器（一般不用）


/* ===================== 函数接口 ===================== /
/
这些是对外提供的驱动接口
在 main.c 中直接调用即可
*/
void KX134_Init(void);                                                      // 初始化KX134（配置寄存器）
void KX134_WriteReg(unsigned char RegAddr,uint8_t Buffer);                  // 写寄存器（SPI）
void KX134_ReadReg(unsigned char RegAddr,uint8_t *Buffer,uint8_t Length);   // 读寄存器（SPI）
void KX134_Start(void);                                                     // 启动测量（进入工作模式）
void KX134_ReadData(void);                                                  // 读取三轴数据（核心函数）
	

#endif



