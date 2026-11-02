/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __M6_SPEED_POWER_LOOP_H
#define __M6_SPEED_POWER_LOOP_H

/* Includes -----------------------------------------------------------------*/
/* Exported types -----------------------------------------------------------*/

/* Exported constants -------------------------------------------------------*/
//timer clock frequency used by the speed loop (unit-Hz)
#define TIMER_CLOCK_FREQ_SPEED      72000000uL

//speed loop timer interrupt frequency (unit-Hz)
#define TIMER_FREQ_SPEED_LOOP       2000

//SYS timer period value
#define TIMER_PERIDO_SYSTICK     TIMER_CLOCK_FREQ_SPEED / (u32)(TIMER_FREQ_SPEED_LOOP*8)

//speed loop calculation frequency (unit-Hz)
#define SPEED_LOOP_CAL_FREQ         2000

//speed loop calculation period (unit-s)
#define SPEED_LOOP_CAL_PERIOD      _IQ(1.0)/SPEED_LOOP_CAL_FREQ

//speed loop PID regulator calculation period (x+1)*500usec
#define SPEED_SAMPLE_PID_PERIOD     TIMER_FREQ_SPEED_LOOP/SPEED_LOOP_CAL_FREQ

/* Exported variables -------------------------------------------------------*/

/* Exported macro -----------------------------------------------------------*/

/* Exported functions -------------------------------------------------------*/
//speed processing timer initialization
void Systick_Init(void);

//speed processing timer enable
void SYS_TIM_Cmd(FunctionalState NewState);

#endif
