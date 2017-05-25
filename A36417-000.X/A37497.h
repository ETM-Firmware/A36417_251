/* 
 * File:   A37497.h
 * Author: cosorio, dparker
 *
 * Created on January 5, 2015, 3:28 PM
 */

#ifndef __A37497_H
#define	__A37497_H

#include <xc.h>
#include <adc12.h>
#include <timer.h>
#include <libpic30.h>
#include "P1395_CAN_SLAVE.h"
#include "ETM.h"

#define FCY_CLK     10000000
#define FCY_CLK_MHZ 10


/*

 * Ion Pump Board Assembly
 *
 * Hardware Module Resource Usage

 CAN1   - Used/Configured by ETM CAN
 Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such)
 Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

 I2C    - Used/Configured by EEPROM Module
 SPI1   - Used by DAC that controls EMCO supply 

 Timer3 - Used for 10msTicToc

 ADC Module - See Below For Specifics

 PIN_G13, PIN_A7, PIN_G12 - Used/Configured by ETM CAN
 
*/



/*
  -----------Digital Output Pins-------------

  RA6  - PIC 15V Supply Enable
  RA7  - Digital Output - LED Operational (Configured by CAN Module)
  RA12 - Digital Output - Test Point C
  RA13 - Digital Output - Test Point D

  RB9  - Digital Output - Test Point H

  RC14 - Done DRV A
  RC15 - Done DRV B

  RF4  - Digital Output - Led Test Point B (Configured by CAN Module)
  RF5  - Digital Output - Led Test Point A (Configured by CAN Module)

  RG14 - Digital Output - Reset detect

*/

#define PIN_D_OUT_TEST_POINT_A      _LATF5
#define PIN_D_OUT_TEST_POINT_B      _LATF4
#define PIN_D_OUT_TEST_POINT_C      _LATA12
#define PIN_D_OUT_TEST_POINT_D      _LATA13
#define PIN_D_OUT_TEST_POINT_H      _LATB9
#define PIN_D_OUT_RESET_DETECT      _LATG14


#define PIN_D_OUT_15V_SUPPLY_ENABLE _LATA6
#define PIN_D_OUT_DONE_DRV_A        _LATC14
#define PIN_D_OUT_DONE_DRV_B        _LATC15

#define OLL_NOT_DONE                1
#define OLL_15V_SUPPLY_ENABLE       0

#define A37497_TRISA_VALUE 0b1100111100111111
#define A37497_TRISB_VALUE 0b1111110111111111
#define A37497_TRISC_VALUE 0b0011111111111111
#define A37497_TRISD_VALUE 0b1111111111111111
#define A37497_TRISF_VALUE 0b1111111111001111
#define A37497_TRISG_VALUE 0b1011111111111111


// ------------------------ CONFIGURE ADC MODULE ------------------- //




/*
  -------------Analog Inputs-------------

  RB2 - Analog Input - Ion Pump Current High Resolution
  RB3 - Analog Input - Ion pump voltage
  RB4 - Analog Input - Ion pump current
  RB5 - Analog Input - 5V monitor
  RB6 - Analog Input - +15V monitor
  RB7 - Analog Input - -15v monitor

  RB9 - Analog Input - Spare Test Point (unused as analog at this time)
*/


/*
  This sets up the ADC to work as following
  AUTO Sampling
  VDD / GND as reference

  6 Samples per Interrupt, use alternating buffers
  With 10MHz System Clock, ADC Clock is 800ns, Sample Time is 10 ADC Clock
  Total Conversion time is 24 TAD = 19.2uS
  Conversion rate of 52KHz (8.68 Khz per Channel), 86 Samples per channel per 10mS interrupt

*/
#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_6 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_10 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_8Tcy)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN2 & ADC_CH0_NEG_SAMPLEB_VREFN) 
#define ADPCFG_SETTING          (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 &SKIP_SCAN_AN15)

// DPARKER - CHANGE THE ADC to use Reference and redo all the scalings


typedef struct{
  AnalogInput analog_input_ion_pump_current;
  AnalogInput analog_input_ion_pump_current_high_resolution;  
  AnalogInput analog_input_ion_pump_voltage;
  AnalogInput analog_input_5V_monitor;
  AnalogInput analog_input_15V_monitor;
  AnalogInput analog_input_minus_15V_divider_point;
  AnalogInput analog_input_minus_15V_monitor;

  AnalogOutput analog_output_emco_control;

  unsigned int accumulator_counter;
  unsigned int control_state;

  unsigned int EMCO_control_setpoint;
  unsigned int power_up_count;
    
} IonPumpControlData;



/*
  TMR3 Configuration
  Timer3 - Used for 10msTicToc
  Period should be set to 10mS
  With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT)
#define PR3_PERIOD_US                  10000   // 10mS
#define PR3_VALUE_10_MILLISECONDS      12500   //(FCY_CLK_MHZ*PR3_PERIOD_US/8)



// -------------------- A37497 FAULTS/WARNINGS CONFIGURATION-------------------- //

#define _FAULT_CAN_COMMUNICATION                 _FAULT_0
#define _FAULT_ION_PUMP_OVER_CURRENT             _FAULT_1
//#define _FAULT_ION_PUMP_OVER_VOLTAGE             _FAULT_2 // - THIS FAULT IS NOT USED
#define _FAULT_ION_PUMP_UNDER_VOLTAGE            _FAULT_3
#define _FAULT_POWER_RAIL_FAILURE                _FAULT_4

#define _STATUS_ION_PUMP_HAS_OVER_CURRENT        _WARNING_0  // This is used by the ECB
#define _STATUS_KEEP_HEATER_OFF                  _WARNING_1  // This will be used by the ECB Someday
#define _STATUS_ION_PUMP_CURRENT_VERY_LOW        _WARNING_2  // This is set when the Ion Pump Current is less than 1uA



#define STATE_STARTUP                0x10
#define STATE_POWER_UP               0x20
#define STATE_OPERATE                0x30
#define STATE_FAULT_ION_PUMP_ON      0x40
#define STATE_FAULT_ION_PUMP_OFF     0x50



#endif	/* A37497_H */

