/* Includes -----------------------------------------------------------------*/
#include "M0_Control_Library.h"

/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_BRK_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void Waveform_Display (void);//waveform display variable setup

/* Private variables --------------------------------------------------------*/
/* Variables ----------------------------------------------------------------*/
/******************************************************************************
* Function Name  : Interrupt_Configuration
* Description    : interrupt priority configuration
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void Interrupt_Configuration(void)
{
    PFIC->IPRIOR[SysTick_IRQn]|=((0XE0));
    NVIC_EnableIRQ(SysTick_IRQn);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
    NVIC_EnableIRQ(TIM1_BRK_IRQn);

    NVIC_EnableIRQ(ADC_IRQn);
}

/******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : timer 1 underflow interrupt, handles waveform data output
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
//__attribute__((section(".highcode")))
void TIM1_UP_IRQHandler(void)
{
    if( RunningStatus_M ==PRESTART)//motor start processing
    {
        TIM1->CH4CVR = 10;
        ADC_Start(ENABLE);
        RunningStatus_M=DIRCHECK;
        if(RunningStatus_M==DIRCHECK)
        {
           //observed d-axis current low-pass filter parameter initialization
            MStruc_M.IdLPF1stH.Input_Coef  = IDQLPF_HIGH_INPUT_COEF;
            MStruc_M.IdLPF1stH.Output_Coef = IDQLPF_HIGH_OUTPUT_COEF;

           //observed q-axis current low-pass filter parameter initialization
            MStruc_M.IqLPF1stH.Input_Coef  = IDQLPF_HIGH_INPUT_COEF;
            MStruc_M.IqLPF1stH.Output_Coef = IDQLPF_HIGH_OUTPUT_COEF;

            //current loop PI gain increase
            Mpid_M.dCurr.Kp=Mpid_M.dCurr.Kp<<FLYSTART_CURRLOOP_GAIN_M;
            Mpid_M.dCurr.Ki=Mpid_M.dCurr.Ki<<FLYSTART_CURRLOOP_GAIN_M;
            Mpid_M.qCurr.Kp=Mpid_M.qCurr.Kp<<FLYSTART_CURRLOOP_GAIN_M;
            Mpid_M.qCurr.Ki=Mpid_M.qCurr.Ki<<FLYSTART_CURRLOOP_GAIN_M;
        }
    }

    if((RunningStatus_M!=RUN)&&(RunningStatus_M!=DIRCHECK))
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    DCBUS_Volt_Cal(&ADC_M,ADC1);

    if((Flystart_M.Process_mark==TRUE)&&(RunningStatus_M == DIRCHECK))
     {
         Flystart_Switch(&Flystart_M,&Ramp_M);
     }
    Waveform_Display();
    TIM1->INTFR = (uint16_t)~TIM_FLAG_Update;
}

/******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : brake interrupt, sets the fault
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
    Protection_SetFault(DC_OVER_CURR_HARD_M);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
}

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{

}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  while (1)
  {
      //NVIC_SystemReset();
  }
}
