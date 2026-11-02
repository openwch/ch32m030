/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __M8_PROTECTION_H
#define __M8_PROTECTION_H

/* Includes -----------------------------------------------------------------*/
/* Exported types -----------------------------------------------------------*/

//instantaneous overcurrent protection structure prototype
typedef struct
{
    _iq24 CurrThreshold;      //overcurrent threshold (unit-A_per)
    uint16_t CntThreshold;    //condition-satisfied count threshold
    uint16_t Cnt;             //condition-satisfied count value
} InstPhaseOC_Type;

//bus voltage protection structure prototype
typedef struct
{
    _iq24 OverThresh;               //overvoltage threshold (unit-V_per)
    _iq24 UnderThresh;              //undervoltage threshold (unit-V_per)

    uint16_t UnderCntThresh;        //undervoltage protection count threshold
    uint16_t UnderCnt;              //undervoltage protection count value
    uint16_t OverCntThresh;         //overvoltage protection count threshold
    uint16_t OverCnt;               //overvoltage protection count value

    Truth_Verify_Type Undervolt;    //undervoltage flag
    Truth_Verify_Type Overvolt;     //overvoltage flag
} DCbusProtection_Type;

//overload protection structure prototype
typedef struct
{
    _iq24 CurrCal;         //overload calculation current (unit-A_per)
    _iq24 CurrThresh;      //overload threshold (unit-A_per)
    _iq24 CalCoef;         //calculation coefficient
    uint16_t CntThresh;    //condition-satisfied count threshold
    uint16_t Cnt;          //condition-satisfied count value
} OverLoad_Type;

//phase loss protection structure prototype
typedef struct
{
    _iq24 CurrThresh;                 //phase loss current threshold (unit-A_per)
    _iq24 CurrAabs;                   //phase A current absolute value (unit-A_per)
    _iq24 CurrBabs;                   //phase B current absolute value (unit-A_per)
    _iq24 CurrCabs;                   //phase C current absolute value (unit-A_per)
    BuffPara_Type CurrBuffPara;       //current average calculation parameter

    uint16_t CntThresh;               //phase loss condition count threshold
    uint16_t SiglephaseCntA;          //U-phase loss condition-satisfied count value
    uint16_t SiglephaseCntB;          //V-phase loss condition-satisfied count value
    uint16_t SiglephaseCntC;          //W-phase loss condition-satisfied count value
    uint16_t MultiphaseCnt;           //multi-phase loss condition-satisfied count value

    uint16_t DetectphaseSet;          //phase loss detection count setpoint
    uint16_t DetectphaseCnt;          //phase loss detection count value
    Truth_Verify_Type Detectphase;    //phase loss detection flag
} Phaseloss_Type;
/* Exported constants -------------------------------------------------------*/
//overcurrent protection parameter definition
#define INST_PHASEOC_THRE_M       _IQ(1.2)      //AC instantaneous overcurrent software protection threshold (unit-A_per)
#define INST_PHASEOC_CNT_THRE_M   (uint16_t)2   //AC instantaneous overcurrent software protection determination count threshold

//bus voltage protection parameter definition
#define DCBUS_OVER_VOLT_THRE    _IQ(1.2)        //bus overvoltage threshold (unit-V_per)90V
#define DCBUS_UNDER_VOLT_THRE   _IQ(0.8)        //bus undervoltage threshold (unit-V_per)37.8V
#define DCBUS_OVER_VOLT_CNT     (uint16_t)500   //bus overvoltage protection determination count threshold
#define DCBUS_UNDER_VOLT_CNT    (uint16_t)500   //bus undervoltage protection determination count threshold

//phase loss protection parameter definition
#define PHASELOSS_DETECT_TIME    0.5            //phase loss detection start (unit-s)
#define PHASELOSS_CURR_THRE      _IQ(0.05)      //phase loss current threshold (unit-A_per)
#define PHASELOSS_TIME_THRE      0.2            //phase loss time threshold (unit-s)
#define PHASELOSS_CAL_FREQ       FREQ_SVPWM     //phase loss calculation frequency (unit-Hz)
#define PHASELOSS_CAL_BUFFSIZE   16             //phase loss calculation buffer depth
#define PHASELOSS_CNT_THRE       3              //phase loss calculation count

/* Exported variables -------------------------------------------------------*/
extern InstPhaseOC_Type  InstPhaseOC_M;        //AC instantaneous overcurrent software protection
extern DCbusProtection_Type DCBusProtecion_M;  //bus voltage protection
extern Phaseloss_Type Phaseloss_M;             //phase loss protection structure
extern DCurrProcess_Type    DCurrProcess_M;    //D-axis current processing structure

/* Exported macro -----------------------------------------------------------*/
/* Exported functions -------------------------------------------------------*/
//fault and protection related parameter definition
void Protection_and_Emergency_Define (void);

//fault and protection related initialization
void Protection_and_Emergency_Init (void);

//fault action definition
void Protection_SetFault(uint16_t Fault_type);

//fault clearing, triggered by the reset key
void Protction_Fault_Reset_M(void);

//software AC instantaneous overcurrent determination
Truth_Verify_Type Phase_Ins_OverCurrent_Verify (Component_3S_Type *PhaseABC, InstPhaseOC_Type * InstOC);

//bus overvoltage protection
void DCOvervolt_Protection(_iq24 DCVolt, DCbusProtection_Type *DCbusProtec);

//bus undervoltage protection
void DCUndervolt_Protection(_iq24 DCVolt, DCbusProtection_Type *DCbusProtec);

//speed reliability determination
Truth_Verify_Type Speed_Reliable_Verify(BuffPara_Type *Buffpara,_iq24 *Buffer,_iq24 Averspeed,_iq24 Threshold);

//speed stability determination
void Speed_Stable_Verify(Observer_Status_Type *Status,_iq24 Speedact);

//stall determination
void Motor_Stall_Verify(_iq24 Speedref, _iq24 EalphaNew ,_iq24 EbetaNew,_iq24 Iqref, Observer_Status_Type *Status, _iq24 Angspeed);

//overspeed and underspeed protection
void Speed_Outrange_Verify(_iq24 Speedact, Observer_Status_Type *Status);

//phase loss determination
Truth_Verify_Type Phaseloss_Verify2R3R (Phaseloss_Type *Phaseloss,Motor_Sructure_Type *MStruc);

#endif
