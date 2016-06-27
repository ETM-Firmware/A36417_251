/* 
 * File:   A36417.h
 * Author: cosorio
 *
 * Created on January 5, 2015, 3:28 PM
 */

#ifndef __A36417_H
#define	__A36417_H

#include <p30f6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include <xc.h>
#include <timer.h>
#include "P1395_CAN_SLAVE.h"
#include "ETM.h"

#define FCY_CLK     10000000

/*

 * Ion Pump Board Assembly
 *
 * Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN
  Timer2(will change to 4) - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such)
  Timer3(will change to 5) - Used/Configured by ETM CAN - Used for detecting error on can bus

  I2C    - Used/Configured by EEPROM Module

  Timer4(will change to 2) - Used for looking at time between pulses
  Timer5(will change to 3) - Used for 10msTicToc

  ADC Module - See Below For Specifics

  
 */

typedef struct{
    AnalogInput analog_input_ion_pump_current;
    AnalogInput analog_input_ion_pump_voltage;
    AnalogInput analog_input_target_current;
    AnalogInput analog_input_5V_monitor;
    AnalogInput analog_input_15V_monitor;
    AnalogInput analog_input_minus_5V_monitor;
    AnalogOutput analog_output_emco_control;

    unsigned int accumulator_counter;
    unsigned int control_state;
    unsigned int target_current_high;
    unsigned int target_current_low;
    unsigned int current_below_limit;             // Is current below threshold where over current fault can be cleared?

    unsigned int trigger_recieved;
    unsigned int sample_level;
    unsigned int pulse_id;
    unsigned int EMCO_control_setpoint;
    unsigned int EMCO_enable;
    unsigned int self_test_count;
    unsigned int reset_active;
    
}IonPumpControlData;


typedef struct{
    double dState;
    double iState;
    double iMax, iMin;

    double iGain;
    double pGain;
    double dGain;

}SPid;


double UpdatePID(SPid* pid, double error, double reading);
extern IonPumpControlData global_data_A36417_000;
// ------------------------ CONFIGURE ADC MODULE ------------------- //


// -------- Analog Input Pins ----------//

#define PIN_A_IN_ION_PUMP_VOLTAGE               _RB3
#define PIN_A_IN_ION_PUMP_CURRENT               _RB4
#define PIN_A_IN_5V_MONITOR                     _RB5
#define PIN_A_IN_15V_MONITOR                    _RB6
#define PIN_A_IN_MINUS_5V_MONITOR               _RB7

#define PIN_A_IN_TARGET_CURRENT                 _RB10
#define PIN_A_IN_SPARE_1                        _RB11
#define PIN_A_IN_SPARE_2                        _RB12

/*
  This sets up the ADC to work as following
  AUTO Sampling
  VDD / GND as reference

  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 4 ADC Clock so total sample time is 9uS
  Conversion rate of 111KHz (27.8 Khz per Channel), 277 Samples per 10mS interrupt

  8 Samples per Interrupt, use alternating buffers
*/
#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_26Tcy)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN2 & ADC_CH0_NEG_SAMPLEB_VREFN) 
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA & ENABLE_AN12_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 &SKIP_SCAN_AN15) //Modified to skip unused inputs


/*
   TMR3 Configuration
   Timer3 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT)
#define PR3_PERIOD_US                  10000   // 10mS
#define PR3_VALUE_10_MILLISECONDS      12500   //(FCY_CLK_MHZ*PR3_PERIOD_US/8)


// -------------------- A36417_000 FAULTS/WARNINGS CONFIGURATION-------------------- //

#define _FAULT_CAN_COMMUNICATION                 _FAULT_0
#define _FAULT_ION_PUMP_OVER_CURRENT             _FAULT_1
#define _FAULT_ION_PUMP_OVER_VOLTAGE             _FAULT_2
#define _FAULT_ION_PUMP_UNDER_VOLTAGE            _FAULT_3

#define _STATUS_ION_PUMP_HAS_OVER_CURRENT        _WARNING_0

//#define _FAULT_5V_OV                             _FAULT_4
//#define _FAULT_5V_UV                             _FAULT_5
//#define _FAULT_15V_OV                            _FAULT_6
//#define _FAULT_15V_UV                            _FAULT_7
//#define _FAULT_MINUS_5V_OV                       _FAULT_8
//#define _FAULT_MINUS_5V_UV                       _FAULT_9


/*

-----------Digital Input Pins------------

RA14 - Digital Input - Sample Target Current
*/

#define PIN_D_IN_SAMPLE_TARGET_CURRENT  _RA14

/*

-----------Digital Output Pins-------------

RA7 - Digital Output - LED Operational
RA8 - Digital Output - Test Point E
RA12 - Digital Output - Test Point C
RA13 - Digital Output - Test Point D

RB2 - Digital Output - Test Point F
RB8 - Digital Output - Test Point G
RB9 - Digital Output - Test Point H

RG12 - Digital Output - Led Test Point A
RG13 - Digital Output - Led Test Point B
RG14 - Digital Output - Reset detect
*/

#define OLL_LED_ON                  0

#define PIN_LED_OPERATIONAL_GREEN   _LATA7  //Moved to Can Module
#define PIN_LED_A_RED               _LATG12 //Moved to Can Module
#define PIN_LED_B_GREEN             _LATG13 //Moved to Can Module

#define PIN_D_OUT_TEST_POINT_E      _LATA6

#define PIN_D_OUT_TEST_POINT_C      _LATA12
#define PIN_D_OUT_TEST_POINT_D      _LATA13
#define PIN_D_OUT_TEST_POINT_F      _LATB2
#define PIN_D_OUT_TEST_POINT_G      _LATB8
#define PIN_D_OUT_TEST_POINT_H      _LATB9

#define PIN_D_OUT_RESET_DETECT      _LATG14

/*
-------------Analog Inputs-------------

RB3 - Analog Input - Ion pump voltage
RB4 - Analog Input - Ion pump current
RB5 - Analog Input - 5V monitor
RB6 - Analog Input - -5V monitor
RB7 - Analog Input - -15v monitor

RB10 - Analog Input - Target Current
*/


/*
-----------------Not used---------------
RB11
RB12

--------- Pins that are overidden by a hardware module and should be left as inputs during port configuration ----
RB0 - PROGRAM
RB1 - PROGRAM

RF0 - CAN 1
RF1 - CAN 1
RF6 - SPI 1
RF8 - SPI 1

-------- Pins that are configured by other software modules and should be left as inputs during port configuration -----------
RF2 (DAC CS/LS)
RF3  (DAC LDAC)

*/

#define A36417_TRISA_VALUE 0b1100000000000000
#define A36417_TRISB_VALUE 0b0000010011111011
#define A36417_TRISC_VALUE 0b0000000000000000
#define A36417_TRISD_VALUE 0b0000000000000000
#define A36417_TRISF_VALUE 0b0000000101001111
#define A36417_TRISG_VALUE 0b0000000000000000



#define SELF_TEST_TIME                      100    // 1sec

#define EMCO_SETPOINT                       3000   // 3V --> 3kV
#define DAC_SETPOINT_CAP                    0xFFF   // 2.5V

#define PID_DGAIN                           .06
#define PID_IGAIN                           .05
#define PID_PGAIN                           .05
#define PID_IMAX                            100000
#define PID_IMIN                            0

#endif	/* A36417_H */

