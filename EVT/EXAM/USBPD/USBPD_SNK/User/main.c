/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.2
 * Date               : 2025/09/19
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
/*
 *@Note
 *
 * PD SNK Sample code
 *
 * This sample code may have compatibility issues and is for learning purposes only.

 * Make sure that the board is not powered on before use.
 * Be sure to pay attention to the voltage when changing the request
 * to prevent burning the board.
 *
 * There is no integrated 5.1K pull-down inside CC in CH32M030C8T and CH32M030G8R
 * Make sure the hardware circuit CC pin is externally connected to 5.1 K pull-down resistance
 * CC_PD is only for status differentiation,
 * bit write 1 means SNK mode, write 0 means SCR mode
 *
 * Modify "PDO_Request( PDO_INDEX_1 )" to modify the request voltage.
 *
 *Select either USBPD0 or USBPD1 to modify the configuration in PD_Process.h
 *Chip CH32M030G8R only has USBPD1
 *
 * According to the usage scenario of PD SNK, whether
 * it is removed or not should be determined by detecting
 * the Vbus voltage, this code only shows the detection
 * and the subsequent communication flow.
 */
#include "debug.h"
#include "PD_Process.h"

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

volatile UINT8  Tim_Ms_Cnt = 0x00;
UINT8 PD_TX_BUF[30];
/*********************************************************************
 * @fn      TIM1_Init
 *
 * @brief   Initialize TIM1
 *
 * @return  none
 */
void TIM1_Init( u16 arr, u16 psc )
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_PB2PeriphClockCmd( RCC_PB2Periph_TIM1, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );

    NVIC_SetPriority(TIM1_UP_IRQn, 3);
    NVIC_EnableIRQ(TIM1_UP_IRQn);


    TIM_ITConfig( TIM1, TIM_IT_Update , ENABLE );

    TIM_Cmd( TIM1, ENABLE );
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(921600);

    printf( "SystemClk:%d\r\n", SystemCoreClock );
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf( "PD SNK TEST\r\n" );

#if(PD_SEL==PD0)
#elif(PD_SEL==PD1)
    Delay_Ms(3);
    RCC_PB2PeriphClockCmd( RCC_PB2Periph_AFIO, ENABLE );
    GPIO_PinRemapConfig(GPIO_Remap_SDI_Disable,ENABLE);
#endif
    PD_Init( );
    TIM1_Init( 999, 48-1);
    while(1)
    {
        /* Get the calculated timing interval value */
        TIM_ITConfig( TIM1, TIM_IT_Update , DISABLE );
        Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
        Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
        TIM_ITConfig( TIM1, TIM_IT_Update , ENABLE );

        PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
        if( PD_Ctl.Det_Timer > 4 )
        {
            PD_Ctl.Det_Timer = 0;
            PD_Det_Proc( );
        }
        PD_Main_Proc( );
    }
}

/*********************************************************************
 * @fn      TIM1_UP_IRQHandler
 *
 * @brief   This function handles TIM1 interrupt.
 *
 * @return  none
 */
void TIM1_UP_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM1, TIM_IT_Update ) != RESET )
    {

        Tim_Ms_Cnt++;
        TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
    }
}
