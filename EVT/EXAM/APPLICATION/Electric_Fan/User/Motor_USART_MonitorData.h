#ifndef __MOTOR_USART_MONITORDATA_H_
#define __MOTOR_USART_MONITORDATA_H_
/* Includes -----------------------------------------------------------------*/
#include "M0_Control_Library.h"
/* Exported constants -------------------------------------------------------*/
#define MaxUsbDataLen       8
#define USB_SENDBUF_LEN       (MaxUsbDataLen<<5)
#define USB_NEEDSEND_LEN1       (MaxUsbDataLen*14)
#define USB_NEEDSEND_LEN2       (MaxUsbDataLen*7)
#define USB_NEEDSEND_LEN3       (MaxUsbDataLen*4)
#define USB_NEEDSEND_LEN4       (MaxUsbDataLen*3)
#define USB_NEEDSEND_LEN5       (MaxUsbDataLen*2)
#define USB_NEEDSEND_LEN6       (MaxUsbDataLen*2)
#define USB_NEEDSEND_LEN7       (MaxUsbDataLen*2)
#define USB_NEEDSEND_LEN8       (MaxUsbDataLen)
#define USB_NEEDSEND_LEN9       (MaxUsbDataLen)
#define USB_NEEDSEND_LEN10       (MaxUsbDataLen)
#define USB_NEEDSEND_LEN11       (MaxUsbDataLen)

/* Exported types -----------------------------------------------------------*/
typedef struct
{
    uint16_t sendDataBuf[USB_SENDBUF_LEN];    //---send data buffer---
    uint8_t heartBeat;        //---heartbeat packet---
//    uint8_t delayNum;         //---delayed transmission---
    uint8_t volatile sendDataLen;       //----data length---
    //uint8_t frameByteNum;         //---send packet byte sequence---
//    uint8_t curWriteFrame;         //----current write data frame---
    uint8_t recDataLen;          //----length of data received from the host---
    uint8_t end;
    uint8_t volatile head;
//    u_int8_t recDataBuf[16];          //---receive data buffer---
//    u_int8_t sendDataType[MaxUsbDataLen];    //---send data type---
}t_sendUsbData;

/* Exported variables -------------------------------------------------------*/
extern t_sendUsbData sendUsbData;

/* Exported functions--------------------------------------------------------*/
extern void Usart_Init(uint32_t Baud);
extern void Usart_Deal_sendMonitorData(void);
#endif
