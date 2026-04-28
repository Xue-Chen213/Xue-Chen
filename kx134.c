#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "kx134.h"
#include "stm32f1xx_hal.h"
#include "ADS124S08.h"

uint8_t KX134_testBuf[10] = {0};
uint8_t KX134_CNTL1 = 0xAA;

uint8_t KX134_Buf[10] = {0};
uint8_t KX134_XOUT[2] = {0};
uint8_t KX134_YOUT[2] = {0};
uint8_t KX134_ZOUT[2] = {0};
uint16_t KX134_xOUT = 0;
uint16_t KX134_yOUT = 0;
uint16_t KX134_zOUT = 0;
float KX134_ConvValue[3];
uint8_t KX134_RD_flag = 0;
uint16_t KX134_Cnt_flag = 0;
uint8_t KX134_Data[9] = {0x6A, 0xBD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C};
uint8_t KX134_flag = 0;

extern Bearing_Data_Struct Bearing_Data;
extern uint16_t Temp_flag;

void KX134_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = KX134_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KX134_CS_GPIO_Port, &GPIO_InitStruct);
}

void KX134_WriteReg(unsigned char RegAddr, uint8_t Buffer)
{
    RegAddr &= 0x7F;
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RegAddr, 1, 1000);
    HAL_SPI_Transmit(&hspi2, &Buffer, 1, 1000);
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
}

void KX134_ReadReg(unsigned char RegAddr, uint8_t *Buffer, uint8_t Length)
{
    RegAddr |= 0x80;
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RegAddr, 1, 1000);
    HAL_SPI_Receive(&hspi2, Buffer, Length, 1000);
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
}

void KX134_Init(void)
{
    KX134_GPIO_Init();
    HAL_Delay(20);
    KX134_WriteReg(CNTL1, 0x00);
    HAL_Delay(1);
    KX134_WriteReg(INC1, 0x30);
    HAL_Delay(1);
    KX134_WriteReg(INC4, 0x10);
    HAL_Delay(1);
    KX134_WriteReg(ODCNTL, 0x4B);
}

void KX134_Start(void)
{
    KX134_WriteReg(CNTL1, 0xE8);
}

void KX134_ReadData(void)
{
    KX134_ReadReg(INS2, KX134_Buf, 1);

    if(KX134_Buf[0] & 0x10U)
    {
        KX134_RD_flag = 0;
        KX134_ReadReg(XOUT_L, KX134_Buf, 6);

        KX134_Data[2] = KX134_Buf[1];
        KX134_Data[3] = KX134_Buf[0];
        KX134_Data[4] = KX134_Buf[3];
        KX134_Data[5] = KX134_Buf[2];
        KX134_Data[6] = KX134_Buf[5];
        KX134_Data[7] = KX134_Buf[4];

        KX134_xOUT = (uint16_t)((KX134_Buf[1] << 8) | KX134_Buf[0]);
        KX134_yOUT = (uint16_t)((KX134_Buf[3] << 8) | KX134_Buf[2]);
        KX134_zOUT = (uint16_t)((KX134_Buf[5] << 8) | KX134_Buf[4]);

        Bearing_Data.adcx[KX134_Cnt_flag] = KX134_zOUT;
        KX134_Cnt_flag++;

        if(KX134_Cnt_flag == 640U)
        {
            KX134_Cnt_flag = 0U;
            KX134_flag = 1U;
        }
    }
}
