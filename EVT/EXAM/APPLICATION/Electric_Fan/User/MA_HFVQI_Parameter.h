/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef _MA_HFVQI_OBSERVER_H
#define _MA_HFVQI_OBSERVER_H

/* Includes -----------------------------------------------------------------*/
/* Exported types -----------------------------------------------------------*/
/* Exported constants -------------------------------------------------------*/
#define HFVQI_POSI_VMAG_M               _IQ(0.4) //pulse positioning voltage amplitude (unit-V_per)
#define HFVQI_POSI_VMAGMAX_M            _IQ(0.60) //pulse positioning voltage amplitude upper limit (unit-V_per)
#define HFVQI_POSI_VMAGMIN_M            _IQ(0.10) //pulse positioning voltage amplitude lower limit (unit-V_per)

#if defined  TWELVE_SECTOR_POSITION
#define HFVQI_POSI_SECTOR_M             12      //number of pulse positioning positions
#endif

#if defined SIX_SECTOR_POSITION
#define HFVQI_POSI_SECTOR_M             6       //number of pulse positioning positions
#endif

#define HFVQI_POSI_ANGLE_INIT_M         _IQ(-0.083333)//pulse positioning initial angle_IQ(-1.0)/12

#define HFVQI_POSI_STEP_M               30       //pulse positioning switching step, based on the calculation interrupt time  10
#define HFVQI_POSI_PULSE_EFFECTIVE_M    10       //pulse voltage action time; the actual effective pulse time is the defined value minus 1  3


/* Exported variables -------------------------------------------------------*/
extern HFVQI_Start_Type HFVQI_StartStruc_M;     //HFVQI observer startup control structure
extern BuffPara_Type    HFVQIPosi_CBuffPara_M;  //injected voltage pulse positioning observed current array parameters
extern _iq24    HFVQIPosi_CBuff_M[];            //injected voltage pulse positioning observed current array

/* Exported macro -----------------------------------------------------------*/
/* Exported functions -------------------------------------------------------*/
//HFVQI initial positioning parameter initialization
void HFVQI_Parameter_Define (HFVQI_Start_Type *HFVQI_Start,SVPWM_2R_Type *SVPWM_Cal);

//HFVQI initial positioning status initialization
void HFVQI_Status_Init_M (HFVQI_Start_Type *HFVQI_Start);

//HFVQI initial positioning sector array initialization
void HFVQI_Buffer_Clear(void);

//HFVQI initial positioning execution function
void HFVQI_PosiSearch_Process(HFVQI_Start_Type *HFVQI_Start,SVPWM_2R_Type *SVPWM_Cal,_iq24 DCVoltAver , int8_t Direction);
#endif
