#include "A37497.h"
#include "A37497_SETTINGS.h"
#include "FIRMWARE_VERSION.h"

// DPARKER - switch to library rev 3


void FlashLeds(void);
unsigned int CheckFaultIonPumpOn(void);
unsigned int CheckFaultIonPumpOff(void);
unsigned int ETMMath16Add(unsigned int value_1, unsigned int value_2);
unsigned int ETMMath16Sub(unsigned int value_1, unsigned int value_2);







_FOSC(ECIO & CSW_FSCM_OFF);
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);

void DoStateMachine(void);
void InitializeA37497(void);
void DoA37497(void);
//unsigned int Check_Supplies(void);


MCP4822 U11_MCP4822;

SPid emco_pid;
IonPumpControlData global_data_A37497;

int main (void){
  global_data_A37497.control_state = STATE_STARTUP;
  while(1){
    DoStateMachine();
  }
}

void DoStateMachine(void){

  switch(global_data_A37497.control_state){
    
  case STATE_STARTUP:
    _CONTROL_NOT_READY = 1;
    _STATUS_KEEP_HEATER_OFF = 1;
    InitializeA37497();
    global_data_A37497.control_state = STATE_POWER_UP;
    break;

  case STATE_POWER_UP:
    _CONTROL_NOT_READY = 1;
    _STATUS_KEEP_HEATER_OFF = 1;
    global_data_A37497.power_up_count = 0;
    while (global_data_A37497.control_state == STATE_POWER_UP) {
      DoA37497();
      FlashLeds();
      ClrWdt();  // There will be no can communication when battery powered so the WDT must be manually cleared
      if (global_data_A37497.power_up_count >= MAX_BATTERY_POWERED_STARTUP_TIME) {
	global_data_A37497.control_state = STATE_OPERATE;
      }
      if ((global_data_A37497.power_up_count > MIN_BATTERY_POWERED_STARTUP_TIME) && _STATUS_ION_PUMP_CURRENT_VERY_LOW) {
	global_data_A37497.control_state = STATE_OPERATE;
      }
    }
    break;

  case STATE_OPERATE:
    PIN_D_OUT_DONE_DRV_A = !OLL_NOT_DONE;
    PIN_D_OUT_DONE_DRV_B = !OLL_NOT_DONE;
    _CONTROL_NOT_READY = 0;
    _STATUS_KEEP_HEATER_OFF = 0;
    while (global_data_A37497.control_state == STATE_OPERATE) {
      DoA37497();
      if (CheckFaultIonPumpOn()) {
	global_data_A37497.control_state = STATE_FAULT_ION_PUMP_ON;
      }
      if (CheckFaultIonPumpOff()) {
	global_data_A37497.control_state = STATE_FAULT_ION_PUMP_OFF;
      }
    }
    break;

  case STATE_FAULT_ION_PUMP_ON:
    PIN_D_OUT_DONE_DRV_A = !OLL_NOT_DONE;
    PIN_D_OUT_DONE_DRV_B = !OLL_NOT_DONE;
    _CONTROL_NOT_READY = 1;
    _STATUS_KEEP_HEATER_OFF = 0;
    while (global_data_A37497.control_state == STATE_FAULT_ION_PUMP_ON) {
      DoA37497();
      if (!CheckFaultIonPumpOn()) {
	global_data_A37497.control_state = STATE_OPERATE;
      }
      if (CheckFaultIonPumpOff()) {
	global_data_A37497.control_state = STATE_FAULT_ION_PUMP_OFF;
      }
    }            
    break;

  case STATE_FAULT_ION_PUMP_OFF:
    PIN_D_OUT_DONE_DRV_A = !OLL_NOT_DONE;
    PIN_D_OUT_DONE_DRV_B = !OLL_NOT_DONE;
    _CONTROL_NOT_READY = 1;
    _STATUS_KEEP_HEATER_OFF = 1;
    while (global_data_A37497.control_state == STATE_FAULT_ION_PUMP_OFF) {
      DoA37497();
      if (!CheckFaultIonPumpOff()) {
	global_data_A37497.control_state = STATE_POWER_UP;
      }
    }            
    break;

  default:
    global_data_A37497.control_state = STATE_STARTUP;
    break;
  }
}


void DoA37497(void){
  unsigned int ion_pump_voltage;
  unsigned int power_supply_fault;

  ETMCanSlaveDoCan();

  if (ETMCanSlaveGetComFaultStatus()) {
    _FAULT_CAN_COMMUNICATION = 1;
  } else if (ETMCanSlaveGetSyncMsgResetEnable()) {
    _FAULT_CAN_COMMUNICATION = 0;
  }
  
  if(_T3IF){
    _T3IF=0;

    global_data_A37497.power_up_count++; 

    ETMAnalogScaleCalibrateADCReading(&global_data_A37497.analog_input_ion_pump_current_high_resolution);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37497.analog_input_ion_pump_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37497.analog_input_ion_pump_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37497.analog_input_5V_monitor);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37497.analog_input_15V_monitor);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37497.analog_input_minus_15V_monitor);
    
    // Data to be set to ECB
    slave_board_data.log_data[0] = global_data_A37497.analog_input_ion_pump_current_high_resolution.reading_scaled_and_calibrated;
    slave_board_data.log_data[1] = global_data_A37497.analog_input_ion_pump_current.reading_scaled_and_calibrated;
    slave_board_data.log_data[2] = global_data_A37497.analog_input_ion_pump_voltage.reading_scaled_and_calibrated;
    slave_board_data.log_data[3] = global_data_A37497.analog_input_ion_pump_current.reading_scaled_and_calibrated;
    if (global_data_A37497.analog_input_ion_pump_current.reading_scaled_and_calibrated < 1000) {
      // If the reading is less than 10uA, use the high resolution reading
      slave_board_data.log_data[3] = global_data_A37497.analog_input_ion_pump_current_high_resolution.reading_scaled_and_calibrated;
    }
    
    // Run the PID on the Ion Pump Voltage
    ion_pump_voltage = global_data_A37497.analog_input_ion_pump_voltage.reading_scaled_and_calibrated;
    global_data_A37497.EMCO_control_setpoint = (unsigned int)(UpdatePID(&emco_pid,(EMCO_SETPOINT-(double)ion_pump_voltage), (double) ion_pump_voltage));

    if(global_data_A37497.EMCO_control_setpoint > DAC_SETPOINT_CAP){
      global_data_A37497.EMCO_control_setpoint = DAC_SETPOINT_CAP;
    }
    if (global_data_A37497.control_state == STATE_FAULT_ION_PUMP_OFF) {
      global_data_A37497.EMCO_control_setpoint = 0;
    }

    /*
      DPARKER - Test this
      NEW EMCO regulation LOOP
      global_data_A37497.EMCO_control_setpoint = UpdateHVControl(global_data_A37497.analog_input_ion_pump_voltage.reading_scaled_and_calibrated, global_data_A37497.EMCO_control_setpoint)
    */


    WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A37497.EMCO_control_setpoint);



    // -------- Debug Logging Information ----------- //
    ETMCanSlaveSetDebugRegister(0, global_data_A37497.control_state);
    ETMCanSlaveSetDebugRegister(1, _STATUS_ION_PUMP_HAS_OVER_CURRENT);
    ETMCanSlaveSetDebugRegister(2, global_data_A37497.EMCO_control_setpoint);
    ETMCanSlaveSetDebugRegister(3, global_data_A37497.analog_input_5V_monitor.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(4, global_data_A37497.analog_input_15V_monitor.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(5, global_data_A37497.analog_input_minus_15V_monitor.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(6, global_data_A37497.analog_input_ion_pump_current.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(7, global_data_A37497.analog_input_ion_pump_current_high_resolution.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(8, global_data_A37497.analog_input_ion_pump_voltage.reading_scaled_and_calibrated);
    // DEBUG A,B,C,D are reserved for pid data local to UpdataePID    
    // DEBUG F is reserved for battery_startup_counter local to initialization function


    // -------------------- CHECK FOR FAULTS ------------------- //

    if (ETMAnalogCheckUnderAbsolute(&global_data_A37497.analog_input_ion_pump_voltage)) {
      _FAULT_ION_PUMP_UNDER_VOLTAGE = 1;
    } else if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_ION_PUMP_UNDER_VOLTAGE = 0;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A37497.analog_input_ion_pump_current)) {
      _STATUS_ION_PUMP_HAS_OVER_CURRENT = 0;
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_ION_PUMP_OVER_CURRENT = 0;
      }
    }
        
    if (ETMAnalogCheckOverAbsolute(&global_data_A37497.analog_input_ion_pump_current)) {
      _STATUS_ION_PUMP_HAS_OVER_CURRENT = 1;
      _FAULT_ION_PUMP_OVER_CURRENT = 1;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A37497.analog_input_ion_pump_current_high_resolution)) {
      _STATUS_ION_PUMP_CURRENT_VERY_LOW = 1;
    } else {
      _STATUS_ION_PUMP_CURRENT_VERY_LOW = 0;
    }

    

    power_supply_fault = 0;

    // Update power supply faults
    if (ETMAnalogCheckOverAbsolute(&global_data_A37497.analog_input_15V_monitor)) {
      power_supply_fault = 1;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A37497.analog_input_15V_monitor)) {
      power_supply_fault = 1;
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A37497.analog_input_minus_15V_monitor)) {
      power_supply_fault = 1;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A37497.analog_input_minus_15V_monitor)) {
      power_supply_fault = 1;
    }

    if (power_supply_fault) {
      _FAULT_POWER_RAIL_FAILURE = 1;
    }  else if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_POWER_RAIL_FAILURE = 0;
    }
  }  
}

unsigned int CheckFaultIonPumpOn(void) {
  if (_FAULT_CAN_COMMUNICATION) {
    return 1;
  }
  
  if (_FAULT_ION_PUMP_OVER_CURRENT) {
    return 1;
  }

  if (_FAULT_ION_PUMP_UNDER_VOLTAGE) {
    return 1;
  }
  
  return 0;
}


unsigned int CheckFaultIonPumpOff(void) {
  if (_FAULT_POWER_RAIL_FAILURE) {
    return 1;
  }
  return 0;
}


unsigned int ETMMath16Add(unsigned int value_1, unsigned int value_2) {
  if ((0xFFFF - value_1) <= value_2) {
    return 0xFFFF;
  } else {
    return (value_1 + value_2);
  }
}

unsigned int ETMMath16Sub(unsigned int value_1, unsigned int value_2) {
  if (value_2 > value_1) {
    return 0;
  } else {
    return (value_1 - value_2);
  }
}

#define DAC_ADJUSTMENT_LARGE   100
#define DAC_ADJUSTMENT_MEDIUM  10
#define DAC_ADJUSTMENT_SMALL   1
#define MAX_DAC_SETPOINT       0x0FFF

unsigned int UpdateHVControl(unsigned int current_reading, unsigned int current_dac_setting) {
  unsigned int voltage_error;
  unsigned int new_dac_setting;

  if (current_reading > EMCO_SETPOINT) {
    voltage_error = ETMMath16Sub(current_reading,EMCO_SETPOINT);
    if (voltage_error > 500) {
      new_dac_setting = 0;
    } else if (voltage_error > 100) {
      new_dac_setting = ETMMath16Sub(current_dac_setting, DAC_ADJUSTMENT_LARGE);
    } else if (voltage_error > 20) {
      new_dac_setting = ETMMath16Sub(current_dac_setting, DAC_ADJUSTMENT_MEDIUM);
    } else {
      new_dac_setting = ETMMath16Sub(current_dac_setting, DAC_ADJUSTMENT_SMALL);
    }
  } else {
    voltage_error = ETMMath16Sub(EMCO_SETPOINT, current_reading);
    if (voltage_error > 500) {
      new_dac_setting = ETMMath16Add(current_dac_setting, DAC_ADJUSTMENT_LARGE);
    } else if (voltage_error > 100) {
      new_dac_setting = ETMMath16Add(current_dac_setting, DAC_ADJUSTMENT_MEDIUM);
    } else if (voltage_error > 10) {
      new_dac_setting = ETMMath16Add(current_dac_setting, DAC_ADJUSTMENT_SMALL);
    } else {
      // the voltage is close enough, do not adjust the DAC setting
      new_dac_setting = current_dac_setting;
    }
    if (new_dac_setting > MAX_DAC_SETPOINT) {
      new_dac_setting = MAX_DAC_SETPOINT;
    }
  }
  return new_dac_setting;
}


double UpdatePID(SPid* pid, double error, double reading) {
  double pTerm, dTerm, iTerm;
  pTerm = pid->pGain * error;

  pid->iState += error;
  if (pid->iState > pid->iMax){
    pid->iState = pid->iMax;
  }

  else if (pid->iState < pid->iMin){
    pid->iState = pid->iMin;
  }

  iTerm = pid->iGain * pid->iState;  // calculate the integral term
  dTerm = pid->dGain * (reading - pid->dState);
  pid->dState = reading;

  ETMCanSlaveSetDebugRegister(0xA, (unsigned int)error);
  ETMCanSlaveSetDebugRegister(0xB, (unsigned int)pTerm);
  ETMCanSlaveSetDebugRegister(0xC, (unsigned int)iTerm);
  ETMCanSlaveSetDebugRegister(0xD, (unsigned int)dTerm);

  if(pTerm + iTerm - dTerm < 0)
    return 1;
  else
    return pTerm + iTerm - dTerm;

}


#define EEPROM_BATTERY_STARTUP_COUNTER_REGISTER 0x300

void InitializeA37497(void) {
  unsigned int battery_startup_counter;

  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;

  // Configure ADC Interrupt
  _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)

  PIN_D_OUT_DONE_DRV_A = OLL_NOT_DONE;
  PIN_D_OUT_DONE_DRV_B = OLL_NOT_DONE;

  // Initialize all I/O Registers
  TRISA = A37497_TRISA_VALUE;
  TRISB = A37497_TRISB_VALUE;
  TRISC = A37497_TRISC_VALUE;
  TRISD = A37497_TRISD_VALUE;
  TRISF = A37497_TRISF_VALUE;
  TRISG = A37497_TRISG_VALUE;
  
  // Initialize the DAC
  U11_MCP4822.pin_chip_select_not = _PIN_RF2;
  U11_MCP4822.pin_load_dac_not = _PIN_RF3;
  U11_MCP4822.spi_port = ETM_SPI_PORT_1;
  U11_MCP4822.spi_con1_value = MCP4822_SPI_CON_VALUE;
  U11_MCP4822.spi_con2_value = MCP4822_SPI_CON2_VALUE;
  U11_MCP4822.spi_stat_value = MCP4822_SPI_STAT_VALUE;
  U11_MCP4822.spi_bit_rate = MCP4822_SPI_1_M_BIT;
  U11_MCP4822.fcy_clk = FCY_CLK;

  SetupMCP4822(&U11_MCP4822);


  // Initialize the External EEprom
  ETMEEPromUseExternal();
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);


  //  if (ETMEEPromCheckOK()) { DPARKER add this library
  if (1) {
    // We only want to run the Ion Pump once every BATTERY_STARTUP_REPEATS to save power
    battery_startup_counter = ETMEEPromReadWord(EEPROM_BATTERY_STARTUP_COUNTER_REGISTER);
    battery_startup_counter++;
    if (battery_startup_counter >= BATTERY_STARTUP_REPEATS) {
      battery_startup_counter = 0;
    } else {
      PIN_D_OUT_DONE_DRV_A = !OLL_NOT_DONE;
      PIN_D_OUT_DONE_DRV_B = !OLL_NOT_DONE;
    }
    ETMEEPromWriteWord(EEPROM_BATTERY_STARTUP_COUNTER_REGISTER, battery_startup_counter);
  } else {
    // THE EEPROM is not working in this application
    // DPARKER - IS THERE ANYTHING WE NEED TO DO
  }
  
  ETMCanSlaveSetDebugRegister(0xF, battery_startup_counter);

 
  // Initialize TMR3
  T3CON = T3CON_VALUE;
  TMR3  = 0;
  _T3IF = 0;
  PR3   = PR3_VALUE_10_MILLISECONDS;

  // Initialize integral ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  //Initialize PID control loop variables
  emco_pid.dGain=PID_DGAIN;
  emco_pid.dState=0;
  emco_pid.iState=0;
  emco_pid.iGain=PID_IGAIN;
  emco_pid.pGain=PID_PGAIN;
  emco_pid.iMax=PID_IMAX;
  emco_pid.iMin=PID_IMIN;

  global_data_A37497.EMCO_control_setpoint=0;
  WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A37497.EMCO_control_setpoint);


  // Initialize the CAN module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_ION_PUMP_BOARD, _PIN_RG13, 4, _PIN_RA7, _PIN_RG12);
  ETMCanSlaveLoadConfiguration(37497, 252, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);


  //Initialize analog input/output scaling

  ETMAnalogInitializeInput(&global_data_A37497.analog_input_ion_pump_voltage,
                           MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_VOLTAGE_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           ION_PUMP_VOLTAGE_OVER_TRIP_POINT,
            	           ION_PUMP_VOLTAGE_UNDER_TRIP_POINT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_RELATIVE_COUNTER,
                           ION_PUMP_VOLTAGE_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37497.analog_input_ion_pump_current_high_resolution,
                           MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_CURRENT_HR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           ION_PUMP_CURRENT_HR_OVER_TRIP_POINT,
            	           ION_PUMP_CURRENT_HR_UNDER_TRIP_POINT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_RELATIVE_COUNTER,
                           ION_PUMP_CURRENT_HR_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37497.analog_input_ion_pump_current,
                           MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_CURRENT_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           ION_PUMP_CURRENT_OVER_TRIP_POINT,
            	           ION_PUMP_CURRENT_UNDER_TRIP_POINT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_RELATIVE_COUNTER,
                           ION_PUMP_CURRENT_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37497.analog_input_5V_monitor,
                           MACRO_DEC_TO_SCALE_FACTOR_16(_5V_MONITOR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           _5V_MONITOR_OVER_TRIP_POINT,
            	           _5V_MONITOR_UNDER_TRIP_POINT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           _5V_MONITOR_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37497.analog_input_15V_monitor,
                           MACRO_DEC_TO_SCALE_FACTOR_16(_15V_MONITOR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           _15V_MONITOR_OVER_TRIP_POINT,
            	           _15V_MONITOR_UNDER_TRIP_POINT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           _15V_MONITOR_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37497.analog_input_minus_15V_monitor,
                           MACRO_DEC_TO_SCALE_FACTOR_16(MINUS_15V_MONITOR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           MINUS_15V_MONITOR_OVER_TRIP_POINT,
            	           MINUS_15V_MONITOR_UNDER_TRIP_POINT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           MINUS_15V_MONITOR_ABSOLUTE_TRIP_TIME);

}




void FlashLeds(void) {
  // Startup LEDs
  switch (((global_data_A37497.power_up_count >> 4) & 0b11)) {
    
  case 0:
    _LATA7 = 1;
    _LATF4 = 1;
    _LATF5 = 1;
    break;
    
  case 1:
    _LATA7 = 0;
    _LATF4 = 1;
    _LATF5 = 1;
    break;
    
  case 2:
    _LATA7 = 0;
    _LATF4 = 0;
    _LATF5 = 1;
    break;
    
  case 3:
    _LATA7 = 0;
    _LATF4 = 0;
    _LATF5 = 0;
    break;
  }
}


void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;

  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A37497.analog_input_ion_pump_current_high_resolution.adc_accumulator  += ADCBUF0;
    global_data_A37497.analog_input_ion_pump_voltage.adc_accumulator                  += ADCBUF1;
    global_data_A37497.analog_input_ion_pump_current.adc_accumulator                  += ADCBUF2;
    global_data_A37497.analog_input_5V_monitor.adc_accumulator                        += ADCBUF3;
    global_data_A37497.analog_input_15V_monitor.adc_accumulator                       += ADCBUF4;
    global_data_A37497.analog_input_minus_15V_monitor.adc_accumulator                 += ADCBUF5;

  } else {
    // read ADCBUF 8-15
    global_data_A37497.analog_input_ion_pump_current_high_resolution.adc_accumulator  += ADCBUF8;
    global_data_A37497.analog_input_ion_pump_voltage.adc_accumulator                  += ADCBUF9;
    global_data_A37497.analog_input_ion_pump_current.adc_accumulator                  += ADCBUFA;
    global_data_A37497.analog_input_5V_monitor.adc_accumulator                        += ADCBUFB;
    global_data_A37497.analog_input_15V_monitor.adc_accumulator                       += ADCBUFC;
    global_data_A37497.analog_input_minus_15V_monitor.adc_accumulator                 += ADCBUFD;
  }

  global_data_A37497.accumulator_counter += 1;

  if (global_data_A37497.accumulator_counter >= 64) {
    
    global_data_A37497.analog_input_ion_pump_current_high_resolution.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 64 samples
    global_data_A37497.analog_input_ion_pump_current_high_resolution.filtered_adc_reading = global_data_A37497.analog_input_ion_pump_current_high_resolution.adc_accumulator;
    global_data_A37497.analog_input_ion_pump_current_high_resolution.adc_accumulator = 0;

    global_data_A37497.analog_input_ion_pump_current.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 64 samples
    global_data_A37497.analog_input_ion_pump_current.filtered_adc_reading = global_data_A37497.analog_input_ion_pump_current.adc_accumulator;
    global_data_A37497.analog_input_ion_pump_current.adc_accumulator = 0;
    
    global_data_A37497.analog_input_ion_pump_voltage.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 64 samples
    global_data_A37497.analog_input_ion_pump_voltage.filtered_adc_reading = global_data_A37497.analog_input_ion_pump_voltage.adc_accumulator;
    global_data_A37497.analog_input_ion_pump_voltage.adc_accumulator = 0;
    
    global_data_A37497.analog_input_5V_monitor.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 64 samples
    global_data_A37497.analog_input_5V_monitor.filtered_adc_reading = global_data_A37497.analog_input_5V_monitor.adc_accumulator;
    global_data_A37497.analog_input_5V_monitor.adc_accumulator = 0;
    
    global_data_A37497.analog_input_15V_monitor.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 64 samples
    global_data_A37497.analog_input_15V_monitor.filtered_adc_reading = global_data_A37497.analog_input_15V_monitor.adc_accumulator;
    global_data_A37497.analog_input_15V_monitor.adc_accumulator = 0;
    
    global_data_A37497.analog_input_minus_15V_monitor.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 64 samples
    global_data_A37497.analog_input_minus_15V_monitor.filtered_adc_reading = global_data_A37497.analog_input_minus_15V_monitor.adc_accumulator;
    global_data_A37497.analog_input_minus_15V_monitor.adc_accumulator = 0;

    global_data_A37497.accumulator_counter = 0;
  }
}




void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
 
  index_word = message_ptr->word3;
  switch (index_word)
    {
      /*
	Place all board specific commands here
      */
     
    default:
      //local_can_errors.invalid_index++;
      break;
    }
}
