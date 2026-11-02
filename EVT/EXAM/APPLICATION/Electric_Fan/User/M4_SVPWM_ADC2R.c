
/* Includes------------------------------------------------------------------*/
#include "M0_Control_Library.h"
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* variables ----------------------------------------------------------------*/
SVPWM_2R_Type    SVPWM_Cal_M;    //SVPWM calculation structure
ADCStruc_Type    ADC_M;          //ADC sampling structure
/******************************************************************************
* Function Name  : SVPWM_2R_Paramter_Init
* Description    : SVPWM parameter initialization
* Input          : SVPWM calculation structure, motor base-value parameter structure pointer
* Output         : None
* Return         : None
******************************************************************************/
void SVPWM_2R_Paramter_Init(SVPWM_2R_Type *SVPWM_Cal,Motor_Base_Type *Mbase)
{
    SVPWM_Cal->T = ((uint16_t)(TIMER_CLOCK_FREQ_SVPWM/(uint32_t)(FREQ_SVPWM)));
    SVPWM_Cal->HALF_T = SVPWM_Cal->T/2;
    SVPWM_Cal->Quard_T = SVPWM_Cal->T/4;

    //vector action time calculation coefficient
    SVPWM_Cal->coef1 = _IQ16mpy(_IQ16div((_IQ16(SVPWM_Cal->T))/2,(_IQ16div(DCBUS_RATED_VOLT,Mbase->Vbase))),Three_Q16);
    SVPWM_Cal->coef2 = _IQ16mpy(_IQ16div((_IQ16(SVPWM_Cal->T))/2,(_IQ16div(DCBUS_RATED_VOLT,Mbase->Vbase))),SQRT3_Q16);

    SVPWM_Cal->Maxmodulesqure = MAX_MODULATIONSQUARE;
    SVPWM_Cal->Maxmodule = MAX_MODULATION;
}

/******************************************************************************
* Function Name  : SVPWM_Status_Initial_M
* Description    : SVPWM state initialization
* Input          : SVPWM calculation structure
* Output         : None
* Return         : None
******************************************************************************/
void SVPWM_2R_Status_Init_M(SVPWM_2R_Type *SVPWM_Cal)
{
    TIM1->CH1CVR = SVPWM_Cal->Quard_T;
    TIM1->CH2CVR = SVPWM_Cal->Quard_T;
    TIM1->CH3CVR = SVPWM_Cal->Quard_T;

    SVPWM_Cal->cc1Load = SVPWM_Cal->Quard_T;
    SVPWM_Cal->cc2Load = SVPWM_Cal->Quard_T;
    SVPWM_Cal->cc3Load = SVPWM_Cal->Quard_T;

    SVPWM_Cal->sector = 1;
    SVPWM_Cal->VoltLimMark = 0;
}

/******************************************************************************
* Function Name  : ADC_Parameter_Init
* Description    : ADC parameter calculation
* Input          : ADC calculation structure pointer, motor base-value parameter structure pointer
* Output         : None
* Return         : None
******************************************************************************/
void ADC_Parameter_Init (ADCStruc_Type *ADCStruc,Motor_Base_Type *Mbase)
{
    //calculate the sampling value corresponding to rated current
    ADCStruc->RatedCurr_ConvInv = _IQ16mpyIQX(_IQ16div(_IQ16(128),_IQ16(31774)),\
                            16,_IQ16div(CURR_SAMPLE_RANGE_ADC,Mbase->Ibase),16);

    //calculate the reciprocal of the sampling value corresponding to rated voltage
     ADCStruc->DCVoltRatedADCInv  =_IQ16div_FAST(_IQ16(128.0),(_IQ16div_FAST(_IQ16mpy(DCVOLT_CONV_RATIO,DCBUS_RATED_VOLT),_IQ16(3.3))*4096));

    //bus voltage average calculation coefficients
    ADCStruc->DCVoltAverSizeInv1 = _IQ(1.0)/DCVOLT_AVER_SIZE;
    ADCStruc->DCVoltAverSizeInv2 = _IQ(1.0) - ADCStruc->DCVoltAverSizeInv1;

}
/******************************************************************************
* Function Name  : SVPWM_2R_Timer_Configure
* Description    : SVPWM port and timer initialization
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void SVPWM_2R_Timer_Configure(void)
{
    // configuration register structure definition
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};

    //clock configuration
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB\
                          | RCC_PB2Periph_AFIO | RCC_PB2Periph_TIM1, ENABLE);

    //IO configuration for PWM
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    FLASH->CTLR2 |= 0x60;//pre-driver output configured to 10V

    //timer 1 configuration
    TIM_DeInit(TIM1);//reset timer registers

    //initialization of the structure used to initialize the timer
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    //prescaler configuration
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PRESCALER_SVPWM;

    //up/down counting mode configuration
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;

    //period configuration
    TIM_TimeBaseStructure.TIM_Period = SVPWM_Cal_M.HALF_T-1;

    //digital filter and dead-time generator division ratio: divide by 1
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    //event update rate
    TIM_TimeBaseStructure.TIM_RepetitionCounter = TIMER_UPDATE_RATE_SVPWM;

    //apply the configuration register
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //initialize the structure used to initialize the compare channels
    TIM_OCStructInit(&TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;

    //forward channel compare output enable
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

    //complementary channel compare output enable
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

    //dummy value
    TIM_OCInitStructure.TIM_Pulse = SVPWM_Cal_M.HALF_T/2;

    //forward channel compare output active high
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    //complementary channel compare output active low
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //positive logic

    //forward channel idle state low
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

    //complementary channel idle state high
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;   //positive logic

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);//capture/compare channel 1 initialization
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);//capture/compare channel 2 initialization
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);//capture/compare channel 3 initialization

    //channel 4 configuration, used to trigger the ADC
    TIM_OCStructInit(&TIM_OCInitStructure);

    //valid when greater than the compare register value, otherwise invalid
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse = 2;

    //forward channel compare output enable
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

    //complementary channel compare output disable
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;

    //positive channel compare output active high
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    //complementary channel compare output active low
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;

    //complementary channel idle state low
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

    //complementary channel idle state low
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC4Init(TIM1, &TIM_OCInitStructure);         //apply the configuration register

    //enable the preload function of the four compare outputs of timer1
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//enable capture/compare channel 1 preload
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);//enable capture/compare channel 2 preload
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);//enable capture/compare channel 3 preload
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//enable capture/compare channel 4 preload

    //brake and dead-time related configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;//OSSR enable
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;//OSSI enable

    //register configuration lock level 1
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;

    //set dead time
    TIM_BDTRInitStructure.TIM_DeadTime = DEADTIME_CONFIGURE;
//
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;//brake enable

  //  brake signal polarity configured as active high
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;

    //automatic output disable
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

    //apply the configuration register
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    //interrupt configuration
    TIM_SetCounter(TIM1,0);                    //clear the timer counter
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);      //clear the update event interrupt flag
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //enable the update event interrupt
    TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);    //disable the capture/compare 4 interrupt

}

/******************************************************************************
* Function Name  : ADC_OPA_Configure
* Description    : ADC and op-amp configuration
* Input          : ADC calculation structure pointer, ADC register pointer
* Output         : None
* Return         : None
******************************************************************************/
void ADC_OPA_Configure(ADCStruc_Type *ADCStruc)
{
    ADC_InitTypeDef ADC_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    OPA_ISP_InitTypeDef OPA_ISP1_InitStruct= {0};
    OPA_ISP_InitTypeDef OPA_ISP2_InitStruct= {0};
    OPA2_InitTypeDef OPA2_InitStruct= {0};
    CMP2_InitTypeDef CMP2_InitStruct= {0};


    RCC_HBPeriphClockCmd(RCC_HBPeriph_OPCM, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR,ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA|RCC_PB2Periph_GPIOB|RCC_PB2Periph_GPIOC\
                         |RCC_PB2Periph_AFIO | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_HB_Div4);

    PWR->CTLR&=~(0X3<<12);
    PWR->CTLR|=(0X1<<12);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    OPA_ISP_StructInit(&OPA_ISP1_InitStruct);
    OPA_ISP1_InitStruct.OPA_ISP_GAIN=OPA_ISP_GAIN_4;
    OPA_ISP1_InitStruct.OPA_ISP_VFBIAS=OPA_ISP_VFBIAS_1_6V;
    OPA_ISP1_InitStruct.OPA_ISP_NESL=OPA_ISP_NESL_ISN;
    OPA_ISP1_InitStruct.OPA_ISP_SEL_IO=OPA_ISP_SEL_TO_ADC_ON;
    OPA_ISP_Init(OPA3_ISP1,&OPA_ISP1_InitStruct);

    OPA_ISP_StructInit(&OPA_ISP2_InitStruct);
    OPA_ISP2_InitStruct.OPA_ISP_GAIN=OPA_ISP_GAIN_4;
    OPA_ISP2_InitStruct.OPA_ISP_VFBIAS=OPA_ISP_VFBIAS_1_6V;
    OPA_ISP2_InitStruct.OPA_ISP_NESL=OPA_ISP_NESL_ISN;
    OPA_ISP2_InitStruct.OPA_ISP_SEL_IO=OPA_ISP_SEL_TO_ADC_ON;
    OPA_ISP_Init(OPA4_ISP2,&OPA_ISP2_InitStruct);

    OPA_ISP_Cmd(OPA3_ISP1,ENABLE);
    OPA_ISP_Cmd(OPA4_ISP2,ENABLE);

    OPA2_StructInit(&OPA2_InitStruct);
    OPA2_InitStruct.QII2_AVSEL=OPA2_QII2_AVSEL_5;
    OPA2_InitStruct.QII2_SESEL=OPA2_QII2_SESEL_MODE_Amplification;
    OPA2_Init(&OPA2_InitStruct);
    QII2_OPA2_Cmd(ENABLE);

    CMP2_StructInit(&CMP2_InitStruct);
    CMP2_InitStruct.CMP2_BK_EN=CMP2_BK_EN_TIM1BKIN_ON;
    CMP2_InitStruct.CMP2_FILT_CFG=3;
    CMP2_InitStruct.CMP2_FILT_EN=CMP2_FILT_EN_ON;
    CMP2_InitStruct.CMP2_QII2_DAC=8;
    CMP2_InitStruct.CMP2_QII2_DACEN=CMP2_QII2_DACEN_ON;
    CMP2_InitStruct.CMP2_PT_IO = CMP2_PT_IO_PA13_OFF;
    CMP2_InitStruct.QII2_CHNSEL=CMP2_QII2_CHNSEL_DAC1;
    CMP2_InitStruct.QII2_CHPSEL=CMP2_QII2_CHPSEL_OPA2_Output;

    CMP2_Init(&CMP2_InitStruct);
    QII2_AE_Cmd(ENABLE);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1,ADC_Channel_17, 1, ADC_SampleTime_11Cycles5);

    ADC_Cmd(ADC1, ENABLE);
}
/******************************************************************************
* Function Name  : DCBUS_Volt_Cal
* Description    : bus voltage calculation
* Input          : ADC calculation structure pointer, ADC register pointer
* Output         : None
* Return         : None
******************************************************************************/
__attribute__((section(".highcode")))
void DCBUS_Volt_Cal(ADCStruc_Type *ADCStruc,ADC_TypeDef *ADCx)
{
    if(((ADCx->STATR & ADC_FLAG_EOC) == ADC_FLAG_EOC)&&((ADCx->STATR & ADC_FLAG_JEOC) != ADC_FLAG_JEOC))
   {
        //get the current bus voltage
        ADCStruc->DCVoltPres=ADCStruc->DCVoltRatedADCInv * (ADCx->RDATAR>>3);

        //average calculation
        ADCStruc->DCVoltAver = _IQMPY_MACRO(ADCStruc->DCVoltAver,ADCStruc->DCVoltAverSizeInv2)\
                                + _IQMPY_MACRO(ADCStruc->DCVoltPres,ADCStruc->DCVoltAverSizeInv1);
                                
        ADCStruc->DCVoltcali= (_IQdiv(_IQ(1.0),ADCStruc->DCVoltAver));

   }
}

/******************************************************************************
* Function Name  : ADC_Offset_Reading
* Description    : AD current sampling zero reading
* Input          : ADC register pointer, ADC calculation structure pointer, ADC number
* Output         : None
* Return         : None
******************************************************************************/
void ADC_Offset_Reading(ADC_TypeDef *ADCx, ADCStruc_Type *ADCStruc,u_int8_t ADC_num)
{
    uint16_t i;

    ADCStruc->OffsetValue1 = 0;//current channel offset initialization
    ADCStruc->OffsetValue2 = 0;

    //disable automatic injected channel group conversion, disable injected interrupt
    ADC_ITConfig(ADCx, ADC_IT_JEOC, DISABLE);

    //injected conversion external trigger source is software trigger
    ADC_ExternalTrigInjectedConvConfig(ADCx, ADC_ExternalTrigInjecConv_None);

    //enable external event to start injected conversion
    ADC_ExternalTrigInjectedConvCmd(ADCx,ENABLE);

    //injected sequence conversion length is 1
    ADC_InjectedSequencerLengthConfig(ADCx,2);

    //current sampling rank is 1, sampling time is SAMPLING_TIME_CK
    ADC_InjectedChannelConfig(ADCx, CURR_CHANNEL_PHASEA_M,1,CURR_SAMPLING_TIMER_CLOCK);
    ADC_InjectedChannelConfig(ADCx, CURR_CHANNEL_PHASEB_M,2,CURR_SAMPLING_TIMER_CLOCK);

    ADC_ClearFlag(ADCx, ADC_FLAG_JEOC);//clear the injected conversion end flag

    ADC_SoftwareStartInjectedConvCmd(ADCx,ENABLE);//software trigger injected conversion

    //accumulate 16 times, i.e. shift the sampling result left by 4 bits, to achieve left alignment of the injected conversion (consistent with regular sampling)
    for(i=0; i <CURR_OFFSET_READ_NUM_ADC; i++)
    {
         while(!ADC_GetFlagStatus(ADCx,ADC_FLAG_JEOC));//wait for the injected conversion to end
        //phase A offset sampling result shifted right by 3 bits accumulated
         ADCStruc->OffsetValue1 += (ADC_GetInjectedConversionValue(ADCx,ADC_InjectedChannel_1)>>3);
         ADCStruc->OffsetValue2 += (ADC_GetInjectedConversionValue(ADCx,ADC_InjectedChannel_2)>>3);
         ADC_ClearFlag(ADCx, ADC_FLAG_JEOC);//clear the injected conversion end flag
         ADC_SoftwareStartInjectedConvCmd(ADCx,ENABLE);//software trigger again
    }

    while(!ADC_GetFlagStatus(ADCx,ADC_FLAG_JEOC));//wait for the injected conversion to end
   //phase A offset sampling result shifted right by 3 bits accumulated
    ADC_ClearFlag(ADCx, ADC_FLAG_JEOC);//clear the injected conversion end flag

}

/******************************************************************************
* Function Name  : ADC_Start
* Description    : ADC conversion start
* Input          : enable command, ADC register pointer
* Output         : None
* Return         : None
******************************************************************************/
void ADC_Start (FunctionalState cmd)
{
    if (cmd == ENABLE)
    {
        //the external trigger source of ADC1 is T1 CC4
        ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);

        //clear the ADC conversion complete flag
        ADC1->STATR = ~(uint32_t)ADC_FLAG_JEOC;

        //enable the ADC injected conversion interrupt
        ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

    }
    else
    {
        //cancel the ADC external trigger source
        ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);

        //clear the ADC conversion complete flag
        ADC1->STATR = ~(uint32_t)ADC_FLAG_JEOC;

        //disable the ADC injected conversion interrupt
        ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);

        //disable the ADC external-triggered injected conversion
        ADC_ExternalTrigInjectedConvCmd(ADC1,DISABLE);

        TIM_CtrlPWMOutputs(TIM1, DISABLE);
    }
}

/*******************************************************************************
* Function Name  : SVPWM_Lowside_Effective
* Description    : all low-side switches on
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_Lowside_Effective(SVPWM_2R_Type *SVPWM_Cal)
{
    SVPWM_Cal->cc1Load = SVPWM_Cal->HALF_T+1;
    SVPWM_Cal->cc2Load = SVPWM_Cal->HALF_T+1;
    SVPWM_Cal->cc3Load = SVPWM_Cal->HALF_T+1;
}

/*******************************************************************************
* Function Name  : SVPWM_Zerovolt_Output
* Description    : 0 voltage output
* Input          : SVPWM calculation structure pointer
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_Zerovolt_Output(SVPWM_2R_Type *SVPWM_Cal)
{
    SVPWM_Cal->cc1Load = SVPWM_Cal->Quard_T;
    SVPWM_Cal->cc2Load = SVPWM_Cal->Quard_T;
    SVPWM_Cal->cc3Load = SVPWM_Cal->Quard_T;
}

/******************************************************************************
* Function Name  : SVPWM_Update
* Description    : timer counter compare value update
* Input          : SVPWM calculation structure pointer
* Output         : None
* Return         : None
******************************************************************************/
void SVPWM_Update (SVPWM_2R_Type *SVPWM_Cal)
{
    TIM1->CH1CVR = SVPWM_Cal->cc1Load;
    TIM1->CH2CVR = SVPWM_Cal->cc2Load;
    TIM1->CH3CVR = SVPWM_Cal->cc3Load;
}

/*******************************************************************************
* Function Name  : SVPWM_ALL_OFF
* Description    : SVPWM output all disabled
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_ALL_OFF(void)
{
    TIM1->CCER&=~((1<<0)|(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10));
}

/*******************************************************************************
* Function Name  : SVPWM_ALL_ON
* Description    : SVPWM output all enabled
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_ALL_ON(void)
{
    TIM1->CCER|=((1<<0)|(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10));
}

/*******************************************************************************
* Function Name  : SVPWM_UP_ON
* Description    : SVPWM output upper switches all enabled
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_UP_ON(void)
{
    TIM1->CCER|=((1<<0)|(1<<4)|(1<<8));
}

/*******************************************************************************
* Function Name  : SVPWM_DOWN_ON
* Description    : SVPWM output lower switches all enabled
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_DOWN_ON(void)
{
    TIM1->CCER|=((1<<2)|(1<<6)|(1<<10));
}

/*******************************************************************************
* Function Name  : SVPWM_UP_OFF
* Description    : SVPWM output upper switches all disabled
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_UP_OFF(void)
{
    TIM1->CCER&=~((1<<0)|(1<<4)|(1<<8));
}

/*******************************************************************************
* Function Name  : SVPWM_DOWN_OFF
* Description    : SVPWM output lower switches all disabled
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_DOWN_OFF(void)
{
    TIM1->CCER&=~((1<<2)|(1<<6)|(1<<10));
}

/*******************************************************************************
* Function Name  : SVPWM_Calculation
* Description    : SVPWM calculation
* Input          : SVPWM calculation structure pointer, two-phase stationary coordinate voltage pointer
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((section(".highcode")))
void SVPWM_Calculation(SVPWM_2R_Type *SVPWM_Cal, Component_2S_Type *VoltRef)
{
    _iq16 Ualphacoef1=0;   //SVPWM calculation coefficient 1
    _iq16 Ubetacoef2=0;    //SVPWM calculation coefficient 2
    _iq16 hx=0;            //intermediate variable for sector and conduction time determination
    _iq16 hy=0;            //intermediate variable for sector and conduction time determination
    _iq16 hz=0;            //intermediate variable for sector and conduction time determination
    _iq16 ht1=0;           //scaling value of voltage vector 1 action time
    _iq16 ht2=0;           //scaling value of voltage vector 2 action time

    Ualphacoef1 = _IQ16MPY_MACRO(SVPWM_Cal->coef1,  VoltRef->Alpha>>8);
    Ubetacoef2 = _IQ16MPY_MACRO(SVPWM_Cal->coef2, VoltRef->Beta>>8);

    hx = Ubetacoef2 * 2;
    hy = Ualphacoef1 + Ubetacoef2;
    hz = Ubetacoef2 - Ualphacoef1;

    //sector determination
    if(hx > 0)
    {
        if((-hz) > 0)
        {
            SVPWM_Cal->sector = 1;
            ht1 = -hz;//t1=-z
            ht2 = hx;//t2 =x
            SVPWM_Cal->cc3Load = SVPWM_Cal->Quard_T + (uint16_t)(ht1>>18) + (uint16_t)(ht2>>18);
            SVPWM_Cal->cc2Load = SVPWM_Cal->cc3Load-(uint16_t)(ht2>>17);
            SVPWM_Cal->cc1Load = SVPWM_Cal->cc2Load-(uint16_t)(ht1>>17);
        }
        else
        {
            if((-hy)>= 0)
            {
                SVPWM_Cal->sector = 3;
                ht1 = hx;
                ht2 = -hy;
                SVPWM_Cal->cc1Load = SVPWM_Cal->Quard_T + (uint16_t)(ht1>>18) + (uint16_t)(ht2>>18);
                SVPWM_Cal->cc3Load = SVPWM_Cal->cc1Load-(uint16_t)(ht2>>17);
                SVPWM_Cal->cc2Load = SVPWM_Cal->cc3Load-(uint16_t)(ht1>>17);
            }
            else
            {
                SVPWM_Cal->sector = 2;
                ht1 = hz;
                ht2 = hy;
                SVPWM_Cal->cc3Load = SVPWM_Cal->Quard_T + (uint16_t)(ht1>>18) + (uint16_t)(ht2>>18);
                SVPWM_Cal->cc1Load = SVPWM_Cal->cc3Load-(uint16_t)(ht2>>17);
                SVPWM_Cal->cc2Load = SVPWM_Cal->cc1Load-(uint16_t)(ht1>>17);
            }
        }
    }
    else
    {
        if((-hz) <= 0)
        {
            SVPWM_Cal->sector = 4;
            ht1 = -hx;
            ht2 = hz;
            SVPWM_Cal->cc1Load = SVPWM_Cal->Quard_T + (uint16_t)(ht1>>18) + (uint16_t)(ht2>>18);
            SVPWM_Cal->cc2Load = SVPWM_Cal->cc1Load-(uint16_t)(ht2>>17);
            SVPWM_Cal->cc3Load = SVPWM_Cal->cc2Load-(uint16_t)(ht1>>17);
        }
        else
        {
            if((-hy)>= 0)
            {
                SVPWM_Cal->sector= 5;
                ht1 = -hy;
                ht2 = -hz;
                SVPWM_Cal->cc2Load = SVPWM_Cal->Quard_T + (uint16_t)(ht1>>18) + (uint16_t)(ht2>>18);
                SVPWM_Cal->cc1Load = SVPWM_Cal->cc2Load-(uint16_t)(ht2>>17);
                SVPWM_Cal->cc3Load = SVPWM_Cal->cc1Load-(uint16_t)(ht1>>17);
            }
            else
            {
                SVPWM_Cal->sector = 6;
                ht1 = hy;
                ht2 = -hx;
                SVPWM_Cal->cc2Load = SVPWM_Cal->Quard_T + (uint16_t)(ht1>>18) + (uint16_t)(ht2>>18);
                SVPWM_Cal->cc3Load = SVPWM_Cal->cc2Load-(uint16_t)(ht2>>17);
                SVPWM_Cal->cc1Load = SVPWM_Cal->cc3Load-(uint16_t)(ht1>>17);
            }
        }
    }
}
