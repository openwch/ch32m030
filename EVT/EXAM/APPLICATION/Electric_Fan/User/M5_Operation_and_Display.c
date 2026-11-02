/* Includes -----------------------------------------------------------------*/
#include "M0_Control_Library.h"
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private Functions --------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Variables ----------------------------------------------------------------*/
static volatile uint16_t Timewait = 0;//time used for delay or wait, timebase is the interrupt of the processing count

Rampctr_Type   Ramp_M;                  //ramp control structure
Operation_Type      Operation_M;        //system operation structure
Powercontrol_Type   Powerctr_M;         //power on/off control structure
Key_Manager_Type    Key_M;              //KEY management structure
/******************************************************************************
* Function Name  : Motor_Operation_Process
* Description    : motor operation processing
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void Motor_Operation_Process(void)
{
    //motor running state machine processing
    switch (RunningStatus_M)
    {
        case IDLE:
            if (Powerctr_M.Poweron == TRUE)
            {
                Ramp_Set(&Ramp_M);    //speed setting
            }
//            RunningStatus_M = INIT;
            break;

        case INIT:
            Motor_Start_Init_M();       //motor start initialization
            RunningStatus_M = PRESTART;    //switch the motor running state to pre-start
            break;

        case PRESTART://wait for the timer underflow interrupt to formally start
            break;

        case POSITION://positioning; during positioning, switch to the start state
            break;

        case START://startup; during startup, switch to the run state
            break;

        case RUN://run

            break;

        case STOP://stop
            TIM_CtrlPWMOutputs(TIM1, DISABLE);  //PWM output disable
            ADC_Start(DISABLE);                 //ADC conversion disable
            RunningStatus_M = WAIT;             //switch the motor running state machine to the WAIT state
            Time_Delay_Set(Operation_M.StopwaitSet,&Operation_M.StopwaitCnt);//stop wait delay setting
            Operation_M.Checkmode = 0;
		   break;

        case WAIT://wait
            //stop wait time elapsed, set the IDLE state
            if (Time_Delay_Elapse(Operation_M.StopwaitCnt) == TRUE)
                RunningStatus_M=IDLE;

            break;
        case FAULT://fault
            TIM_CtrlPWMOutputs(TIM1, DISABLE);  //PWM output disable
            ADC_Start(DISABLE);                 //ADC conversion disable
            break;
        default:
            break;
    }
}
/******************************************************************************
* Function Name  : Poweroperation_Initial
* Description    : power on/off parameter and state initialization
* Input          : power on/off control structure pointer
* Output         : None
* Return         : None
*****************************************************************************/
void Poweroperation_Initial(Powercontrol_Type *PowerOper)
{
    PowerOper->Poweron_CntThresh = (uint16_t)(POWERVERY_FREQ*POWERON_TIME);
    PowerOper->Poweron = FALSE;
    PowerOper->Poweron_Cnt = 0;
}

/******************************************************************************
* Function Name  : Operation_Parameter_Define
* Description    : operation parameter definition
* Input          : operation control structure pointer
* Output         : None
* Return         : None
*****************************************************************************/
void Operation_Parameter_Define(Operation_Type *Operation)
{
    Operation->StopwaitSet = (uint16_t)(STOP_WAITING_TIME * OPERATION_CAL_FREQ);
    Operation->FaultHoldingSet = (uint16_t)(FAULT_HOLDING_TIME * OPERATION_CAL_FREQ);
    Operation->Startmark = FALSE;
    Operation->Checkmode = 0;
    Operation->Precharge_Set = (uint16_t)(PRECHAGE_TIME*SPEED_RAMP_FREQ_M);
}

/******************************************************************************
* Function Name  : Operation_Status_Init
* Description    : operation status initialization
* Input          : operation control structure pointer
* Output         : None
* Return         : None
*****************************************************************************/
void Operation_Status_Init(Operation_Type *Operation)
{
    Operation->StopwaitCnt = 0;
    Operation->FaultHoldingCnt = 0;
}

/******************************************************************************
* Function Name  : rampctr_Parameter_Define
* Description    : ramp control parameter definition
* Input          : ramp speed control structure pointer
* Output         : None
* Return         : None
*****************************************************************************/
void rampctr_Parameter_Define(Rampctr_Type *Ramp_M)
{
    Ramp_M->Direction = STARTUP_DIRECTION;
    Ramp_M->Point_adncnt = POINT_DEFAULT;

    Ramp_M->AccStep = _IQ(1.0)/(SPEED_LOOP_CAL_FREQ*ACCE_TIME_M);
    Ramp_M->DecStep = _IQ(1.0)/(SPEED_LOOP_CAL_FREQ*DECE_TIME_M);
}

/******************************************************************************
* Function Name  : rampctr_Initial
* Description    : ramp control status initialization
* Input          : ramp control structure pointer, loop
* Output         : None
* Return         : None
*****************************************************************************/
void rampctr_Initial(Rampctr_Type *Ramp_M)
{
    Ramp_Set(Ramp_M);

#if(CLOSE_LOOP==SPEED_LOOP)
    Ramp_M->Aftramp = SPEEDPOINT1*Ramp_M->Direction;
#elif (CLOSE_LOOP==POWER_LOOP)
    Ramp_M->Aftramp =  POWERPOINT1;
#endif
    Ramp_M->Startfinish =  FALSE;
    Ramp_M->Decemark =FALSE;

    System_Status_Global = SPEED_CONTROL_M;
}

/******************************************************************************
* Function Name  : Ramp_Set
* Description    : ramp setting
* Input          : acceleration/deceleration control structure pointer
* Output         : None
* Return         : None
*****************************************************************************/
void Ramp_Set(Rampctr_Type *Ramp_M)
{
#if     (CLOSE_LOOP==SPEED_LOOP)
    switch(Ramp_M->Point_adncnt)
    {
        case 1://speed gear 1
           Ramp_M->Preramp =  SPEEDPOINT1*Ramp_M->Direction;
           break;
       case 2://speed gear 2
           Ramp_M->Preramp =  SPEEDPOINT2*Ramp_M->Direction;
           break;
       case 3://speed gear 3
           Ramp_M->Preramp =  SPEEDPOINT3*Ramp_M->Direction;
           break;
       default:
           break;
    }
#elif   (CLOSE_LOOP==POWER_LOOP)
    switch(Ramp_M->Point_adncnt)
    {
        case 1://speed gear 1
           Ramp_M->Preramp =  POWERPOINT1;
           break;
       case 2://speed gear 2
           Ramp_M->Preramp =  POWERPOINT2;
           break;
       case 3://speed gear 3
           Ramp_M->Preramp =  POWERPOINT3;
           break;
       default:
           break;
    }
#endif
}
/******************************************************************************
* Function Name  : Speed_Ramp_Process
* Description    : speed ramp processing, executed in the loop; the acceleration/deceleration calculation frequency must match the interrupt frequency;
* Input          : speed ramp structure pointer
* Output         : speed after ramp processing
* Return         : None
******************************************************************************/
void Speed_Ramp_Process(Rampctr_Type *Speed_Ramp)
{
    _iq24 Speed_Ref_Output;
    Speed_Ref_Output = Speed_Ramp->Aftramp;

    //the new speed and the current speed are both >= 0
    if((Speed_Ref_Output >=_IQ24(0.0)) && (Speed_Ramp->Preramp >=_IQ24(0.0)))
    {
        if(Speed_Ref_Output < Speed_Ramp->Preramp)//acceleration processing
        {
            Speed_Ref_Output += Speed_Ramp->AccStep;
            if(Speed_Ref_Output > Speed_Ramp->Preramp)//final value processing
            {
               Speed_Ref_Output = Speed_Ramp->Preramp;
            }
            else;
        }
        else if(Speed_Ref_Output > Speed_Ramp->Preramp)//deceleration processing
        {
            Speed_Ref_Output -= Speed_Ramp->DecStep;
            if(Speed_Ref_Output < Speed_Ramp->Preramp)//final value processing
            {
               Speed_Ref_Output = Speed_Ramp->Preramp;
            }
            else;
        }
        else;
    }

    //new speed < 0, current speed > 0
    else if((Speed_Ref_Output >=_IQ24(0.0)) && (Speed_Ramp->Preramp < _IQ24(0.0)))
    {
        Speed_Ref_Output -= Speed_Ramp->DecStep;
    }

    //the new speed and the current speed are both < 0
    else if((Speed_Ref_Output < _IQ24(0.0)) && (Speed_Ramp->Preramp < _IQ24(0.0)))
     {
         if(Speed_Ref_Output < Speed_Ramp->Preramp)//deceleration processing
         {
            Speed_Ref_Output += Speed_Ramp->DecStep;
            if(Speed_Ref_Output > Speed_Ramp->Preramp)
            {
                Speed_Ref_Output = Speed_Ramp->Preramp;//final value processing
            }
            else;
         }
         else if(Speed_Ref_Output > Speed_Ramp->Preramp)//acceleration processing
         {
            Speed_Ref_Output -= Speed_Ramp->AccStep;
            if(Speed_Ref_Output < Speed_Ramp->Preramp)//final value processing
            {
                Speed_Ref_Output = Speed_Ramp->Preramp;
            }
            else;
         }
         else;
     }

    //new speed >= 0, current speed < 0
    else if((Speed_Ref_Output < _IQ24(0.0)) && (Speed_Ramp->Preramp >= _IQ24(0.0)))
    {
        Speed_Ref_Output += Speed_Ramp->DecStep;
    }

    else;
    Speed_Ramp->Aftramp = Speed_Ref_Output;
}
/******************************************************************************
* Function Name  : Power_Ramp_Process
* Description    : power ramp processing, executed in the loop; the acceleration/deceleration calculation frequency must match the interrupt frequency;
* Input          : ramp structure pointer
* Output         : power after ramp processing
* Return         : None
******************************************************************************/
void Power_Ramp_Process(Rampctr_Type *Power_Ramp)
{
    _iq24 Power_Ref_Output;
    Power_Ref_Output = Power_Ramp->Aftramp;
        if(Power_Ref_Output < Power_Ramp->Preramp)//ramp-up processing
        {
            Power_Ref_Output += Power_Ramp->AccStep;
            if(Power_Ref_Output > Power_Ramp->Preramp)//final value processing
            {
                Power_Ref_Output = Power_Ramp->Preramp;
            }
            else;
        }
        else if(Power_Ref_Output > Power_Ramp->Preramp)//ramp-down processing
        {
            Power_Ref_Output -= Power_Ramp->DecStep;
            if(Power_Ref_Output < Power_Ramp->Preramp)//final value processing
            {
                Power_Ref_Output = Power_Ramp->Preramp;
            }
            else;
        }
        Power_Ramp->Aftramp = Power_Ref_Output;
}
/******************************************************************************
* Function Name : Poweroperation_Process
* Description   : power on/off control logic
* Input         : acceleration/deceleration control structure pointer, ADC sampling structure pointer, power on/off control structure pointer, running status, speed PID controller pointer, actual speed
* Output        : None
* Return        : None
******************************************************************************/
void Poweroperation_Process(Rampctr_Type *SpeedRamp,ADCStruc_Type *ADCStruc,Powercontrol_Type *PowerOper,\
                    Runningstatus_Type *Runningstatus,_iq24 Speedact)
{
//startup end determination
    if((_IQabs(SpeedRamp->Aftramp) >= START_FINISH_SPEED)&&(_IQabs(Speedact) >= START_FINISH_SPEED))
        SpeedRamp->Startfinish = TRUE;

//power-off logic, detected when powered on
//    if( (PowerOper->DCVoltAver <= PowerOper->Poweroff_Volt)||(PD_DEVICE.ConnectStat==0))//stop directly to prevent low-voltage hardware instability
    if( (ADC_M.DCVoltAver  <= POWEROFF_VOLT))//stop directly to prevent low-voltage hardware instability
    {
        switch(*Runningstatus)
        {
            case INIT:
            case DIRCHECK:
            case PRESTART:
            case POSITION:
            case START:
            case RUN:
                if(*Runningstatus != FAULT)
                *Runningstatus = STOP;
                break;
            case IDLE:
            case STOP:
            case BRAKE:
            case WAIT:
            case FAULT: break;
            }

        PowerOper->Poweron = FALSE;
    }


 //power-on logic, detected when there is no power
//    if((PowerOper->Poweron == FALSE)&&(PD_DEVICE.ConnectStat==1))
    if((PowerOper->Poweron == FALSE))
    {
        if( (ADC_M.DCVoltAver > POWERON_VOLT))
        {
            PowerOper->Poweron_Cnt++;
            if(PowerOper->Poweron_Cnt >= PowerOper->Poweron_CntThresh)
            {
                PowerOper->Poweron = TRUE;
                PowerOper->Poweron_Cnt = 0;
            }
        }
    }
}

/******************************************************************************
* Function Name  : Time_Delay_Process
* Description    : program used for variable decrement processing based on the interrupt time (0.5ms)
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void Time_Delay_Process(void)
{
    if (Timewait != 0)
        Timewait --;

    if (Operation_M.StopwaitCnt != 0)
        Operation_M.StopwaitCnt --;

    if (Operation_M.FaultHoldingCnt != 0)
        Operation_M.FaultHoldingCnt--;

    if (Operation_M.Precharge_cnt != 0)
        Operation_M.Precharge_cnt --;

    if ((Phaseloss_M.DetectphaseCnt!=0)&&(RunningStatus_M==RUN))
    {
        Phaseloss_M.DetectphaseCnt--;
        if(Phaseloss_M.DetectphaseCnt==0)
        {
            Phaseloss_M.Detectphase=TRUE;
        }
    }
}

/******************************************************************************
* Function Name  : Time_Wait
* Description    : The function wait for a delay to be over.
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void Time_Wait(uint16_t time)
{
    Timewait = time;
    while (Timewait != 0)
    {;}
}

/******************************************************************************
* Function Name  : Time_Delay_Set
* Description    : timer time setting function
* Input          : timer time, timer count variable pointer
* Output         : None
* Return         : None
******************************************************************************/
void Time_Delay_Set(uint16_t Delayset, uint16_t *Delaycounter)
{
    *Delaycounter = Delayset;
}

/******************************************************************************
* Function Name  : Time_Delay_Elapse
* Description    : timer elapsed determination
* Input          : timer count variable
* Output         : determination result, elapsed: TRUE, not elapsed: FALSE
* Return         : determination result
******************************************************************************/
Truth_Verify_Type Time_Delay_Elapse(uint16_t Delaycounter)
{
     if (Delaycounter == 0)
       return (TRUE);
     else
       return (FALSE);
}
