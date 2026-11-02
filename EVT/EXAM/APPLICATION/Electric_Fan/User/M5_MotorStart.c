/* Includes -----------------------------------------------------------------*/
#include "M0_Control_Library.h"

/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Variables ----------------------------------------------------------------*/
/******************************************************************************
* Function Name  : Motor_Start_Init
* Description    : initialization before motor start
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void Motor_Start_Init_M(void)
{
    rampctr_Initial(&Ramp_M);//ramp initialization, placed first, contains the direction definition

    KeyandLED_Status_Init(&Key_M);//key and LED status initialization

    Operation_Status_Init(&Operation_M);//operation status initialization

    Protection_and_Emergency_Init();//protection parameter and status initialization

    PID_ParaBase_Cal_M(&MBase_M,&Mpid_M);//recalculate PI parameters
    PID_Output_Init(&Mpid_M,Ramp_M.Direction);//PID regulator output initialization

#if (CLOSE_LOOP==POWER_LOOP)
    PowerCal_Init(&Power_Cal);//power loop initialization
#endif

    Motor_FilerPara_Cal_M(&MStruc_M);    //filter initialization

    Motor_Status_Initial_M(&MStruc_M);//motor state initialization

    Dcurr_Process_Init(&DCurrProcess_M,1);//D-axis current processing initialization

    SVPWM_2R_Status_Init_M(&SVPWM_Cal_M);//SVPWM calculation state initialization

    SMO_Status_Init(&SMO_M,&SMO_Status_M,Ramp_M.Direction);//SMO observer initialization

    Buffer_Clear(&(SMO_SpeedBuffer_M[0]),&(SMO_M.SpeedBuffPara));//SMO observed speed array initialization

#ifdef  POSITION_EN
    HFVQI_Status_Init_M(&HFVQI_StartStruc_M);//HFVQI positioning initialization
#endif

    ADC_Offset_Reading(ADC1,&ADC_M,1);//ADC1 sampling offset reading

    Flystart_Status_Init(&Flystart_M,Ramp_M.Direction); //back-EMF detection

    SVPWM_Zerovolt_Output(&SVPWM_Cal_M);//zero voltage output

    TIM_CtrlPWMOutputs(TIM1,ENABLE);//enable PWM output
}
