/********************************** (C) COPYRIGHT *******************************
* File Name          :Motor_SPI_MonitorData.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/09/01
* Description        : SPI MonitorData
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 * @Note
 * The purpose of this description and routine is to provide customers with the application framework of electric fan cartridge
 * permanent magnet synchronous motor based on WCH MCU,including MCU peripherals, FOC control, observer, loop control, protection,
 * and other basic solutions and modules, to assist customers in shortening the product development cycle.
*/
/* Includes -----------------------------------------------------------------*/
#include "Motor_SPI_MonitorData.h"
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Variables ----------------------------------------------------------------*/
t_sendUsbData sendUsbData = {0,0,0,0,0,0,0};
const u_int8_t usbFrameByteNum[12] = {0, 32, 32, 28, 28, 24, 28, 32, 20, 22, 24, 26};
const u_int8_t needSendLen[12] = {0, USB_NEEDSEND_LEN1, USB_NEEDSEND_LEN2, USB_NEEDSEND_LEN3, USB_NEEDSEND_LEN4, USB_NEEDSEND_LEN5,
        USB_NEEDSEND_LEN6, USB_NEEDSEND_LEN7, USB_NEEDSEND_LEN8, USB_NEEDSEND_LEN9, USB_NEEDSEND_LEN10, USB_NEEDSEND_LEN11};
/*********************************************************************
 * @fn      SPI_Deal_sendMonitorData
 *
 * @brief   sendMonitorData
 *
 * @return  none
 */
__attribute__((section(".highcode")))
void SPI_Deal_sendMonitorData(void)
{
    u_int16_t data=0;
    u_int8_t i=0, frameByteNum=0;
    u_int8_t reDataLen;
    sendUsbData.sendDataLen=6;
    if(sendUsbData.sendDataLen)
    {
        if(sendUsbData.head >= sendUsbData.end)
        {
            reDataLen = sendUsbData.head - sendUsbData.end;
        }
        else
        {
            reDataLen = (u_int16_t)USB_SENDBUF_LEN - sendUsbData.end + sendUsbData.head;
        }
        if(reDataLen >= needSendLen[sendUsbData.sendDataLen])
        {
            GPIOC->BCR = GPIO_Pin_4;
            __asm volatile ("nop");
            while((SPI1->STATR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);

            SPI1->DATAR = 0x55AA;
            while((SPI1->STATR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);

            sendUsbData.heartBeat++;
            data=sendUsbData.heartBeat<<8;
            data|=usbFrameByteNum[sendUsbData.sendDataLen];
            SPI1->DATAR = data;
            while((SPI1->STATR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);

            frameByteNum = 4;
            while(frameByteNum < usbFrameByteNum[sendUsbData.sendDataLen])
            {
                for(i = 0; i < sendUsbData.sendDataLen; i++)
                {
                    SPI1->DATAR=sendUsbData.sendDataBuf[sendUsbData.end+i];
                    while((SPI1->STATR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
                    frameByteNum+=2;
                }
                sendUsbData.end += MaxUsbDataLen;
            }

            while( (SPI1->STATR & SPI_I2S_FLAG_BSY) != (uint16_t)RESET);
            __asm volatile ("nop");
            GPIOC->BSHR = GPIO_Pin_4;
        }
    }
}
/*********************************************************************
 * @fn      SPI_FullDuplex_Init
 *
 * @brief   SPI configuration
 *
 * @return  none
 */
void SPI_FullDuplex_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    SPI_InitTypeDef SPI_InitStructure={0};

    RCC_PB2PeriphClockCmd( RCC_PB2Periph_GPIOA|RCC_PB2Periph_GPIOC |RCC_PB2Periph_SPI1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//SCK

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );//MOSI

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC,GPIO_Pin_4);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    SPI_Init( SPI1, &SPI_InitStructure );

    GPIO_ResetBits(GPIOC,GPIO_Pin_4);
    SPI_Cmd( SPI1, ENABLE );
}

