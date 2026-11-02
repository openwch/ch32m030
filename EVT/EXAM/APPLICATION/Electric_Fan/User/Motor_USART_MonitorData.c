/* Includes -----------------------------------------------------------------*/
#include "Motor_USART_MonitorData.h"
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Variables ----------------------------------------------------------------*/
t_sendUsbData sendUsbData = {0,0,0,0,0,0,0};
const uint8_t usbFrameByteNum[12] = {0, 32, 32, 28, 28, 24, 28, 32, 20, 22, 24, 26};
const uint8_t needSendLen[12] = {0, USB_NEEDSEND_LEN1, USB_NEEDSEND_LEN2, USB_NEEDSEND_LEN3, USB_NEEDSEND_LEN4, USB_NEEDSEND_LEN5,
        USB_NEEDSEND_LEN6, USB_NEEDSEND_LEN7, USB_NEEDSEND_LEN8, USB_NEEDSEND_LEN9, USB_NEEDSEND_LEN10, USB_NEEDSEND_LEN11};

/******************************************************************************
* Function Name  : getUsbBufLen
* Description    : return the head-tail pointer length
* Input          : u_int8_t data
* Output         : None
* Return         : None
******************************************************************************/
u_int8_t getUsbBufLen(void)
{
    u_int8_t reDataLen;
    if(sendUsbData.head >= sendUsbData.end)
    {
        reDataLen = sendUsbData.head - sendUsbData.end;
    }
    else
    {
        reDataLen = (u_int16_t)USB_SENDBUF_LEN - sendUsbData.end + sendUsbData.head;
    }
    return reDataLen;
}
/******************************************************************************
* Function Name  : Usart_Deal_sendMonitorData
* Description    : Usart send
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
__attribute__((section(".highcode")))
void Usart_Deal_sendMonitorData(void)
{
    u_int8_t i=0, frameByteNum=0;
    u_int8_t reDataLen;
    sendUsbData.sendDataLen=4;
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
        if(reDataLen >= needSendLen[sendUsbData.sendDataLen])//head minus tail >= 14 and the buffer is empty
        {
            __asm volatile ("nop");                                         //wait for the data to pass through the optocoupler
            while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);     //wait for the data to be transferred to the shift register
            USART1->DATAR=0xAA;
            while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register
            USART1->DATAR=0x55;
            while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register

            USART1->DATAR=usbFrameByteNum[sendUsbData.sendDataLen];
            while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register
            sendUsbData.heartBeat++;
            USART1->DATAR=sendUsbData.heartBeat;
            while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register

            frameByteNum = 4;
            while(frameByteNum < usbFrameByteNum[sendUsbData.sendDataLen])//{0, 32, 32, 28, 28, 24, 28, 32, 20, 22, 24, 26};
            {
                for(i = 0; i < sendUsbData.sendDataLen; i++)
                {
                    USART1->DATAR = (u_int8_t)((sendUsbData.sendDataBuf[sendUsbData.end+i])&0x00FF);
                    frameByteNum++;
                    while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register
                    USART1->DATAR = (u_int8_t)((sendUsbData.sendDataBuf[sendUsbData.end+i])>>8);
                    frameByteNum++;
                    while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register
                }
                sendUsbData.end += MaxUsbDataLen;
            }
            while((USART1->STATR & USART_FLAG_TXE) == (uint16_t)RESET);      //wait for the data to be transferred to the shift register
            __asm volatile ("nop");                                         //wait for the data to pass through the optocoupler
        }
    }
}
