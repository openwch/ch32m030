/* Includes -----------------------------------------------------------------*/
#include "M0_Control_Library.h"
#include "ch32m030_conf.h"
#include "IQmath_RV32.h"
#include "debug.h"
//extern void Waveform_Display();
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Variables ----------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
    //Delay initialization
    Delay_Init();

    //Delay100ms
    Delay_Ms(100);

    //key and LED IO configuration
    KeyandLED_IOInit();
    KeyandLED_Parameter_Define(&Key_M);

    //Usart initialization, for debugging
    USART_Printf_Init(4000000);

    //global status initialization
    Global_Status_Initial();

    //operation parameter definition
    Operation_Parameter_Define(&Operation_M);

    //motor parameter definition
    Motor_Prameter_Define(&MBase_M);

    //motor phase filter parameter initialization
    Motor_FilerPara_Cal_M(&MStruc_M);

    //base-value parameter calculation
    Motor_BaseCal_M(&MBase_M);

    //PID parameter definition
    PID_Prameter_Define(&Mpid_M);

    //d-axis current processing parameter definition
    DCurrProcess_Parameter_Define(&DCurrProcess_M);

    //PID base-value parameter calculation
    PID_ParaBase_Cal_M(&MBase_M,&Mpid_M);

    PowerCal_Prameter_Define(&Power_Cal);

    //protection parameter configuration
    Protection_and_Emergency_Define();

    //speed operation parameter configuration
    rampctr_Parameter_Define(&Ramp_M);

    //ADC calculation parameter configuration
    ADC_Parameter_Init(&ADC_M,&MBase_M);

    //ADC and OPA configuration
    ADC_OPA_Configure(&ADC_M);

    //SVPWM calculation parameter configuration
    SVPWM_2R_Paramter_Init(&SVPWM_Cal_M ,&MBase_M);

    //SVPWM timer and GPIO configuration
    SVPWM_2R_Timer_Configure();

    SMO_Parameter_Define(&SMO_M,&SMO_Status_M);

    SMO_Paramter_Cal(&MBase_M,&SMO_M);

    //fly start parameter initialization
    Flystart_Prameter_Define(&Flystart_M);

    //fly start parameter calculation
    Flystart_Paramter_Cal(&Flystart_M,&MBase_M);

#ifdef  POSITION_EN
    //pulse positioning parameter configuration
   HFVQI_Parameter_Define(&HFVQI_StartStruc_M,&SVPWM_Cal_M);
#endif
    //power on/off control parameter configuration
    Poweroperation_Initial(&Powerctr_M);

    //speed loop configuration
    Systick_Init();

    //interrupt configuration
    Interrupt_Configuration();

    //timer enable
    TIM_Cmd(TIM1, ENABLE);
    SYS_TIM_Cmd(ENABLE);

    while(1)
    {
        Motor_Operation_Process();    //motor operation
        Usart_Deal_sendMonitorData(); //virtual oscilloscope, for debugging
     }
}
