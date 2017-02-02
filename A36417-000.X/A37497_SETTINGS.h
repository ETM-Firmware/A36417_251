/* 
 * File:   A37497_SETTINGS.h
 * Author: hwanetick, dparker
 *
 * Created on December 21, 2015, 10:18 AM
 */

#ifndef A37497_SETTINGS_H
#define	A37497_SETTINGS_H



#define MAX_BATTERY_POWERED_STARTUP_TIME   12000  // 120 seconds
#define MIN_BATTERY_POWERED_STARTUP_TIME    1000  // 10 seconds
#define BATTERY_STARTUP_REPEATS               12  // The ion pump will run 1 out of every 12 starts (2 hours each)





// DPARKER - Fix all the scale factors


#define ION_PUMP_VOLTAGE_SCALE_FACTOR            .0763              // 1V per 1kV
#define ION_PUMP_VOLTAGE_OVER_TRIP_POINT         4300               // 4.3kV
#define ION_PUMP_VOLTAGE_UNDER_TRIP_POINT        2500               // 2.5kV
#define ION_PUMP_VOLTAGE_ABSOLUTE_TRIP_TIME      5000               // 5 Seconds  This is in 10ms Units

#define ION_PUMP_CURRENT_SCALE_FACTOR            .50863            
#define ION_PUMP_CURRENT_OVER_TRIP_POINT         4000               // 40uA this is in 10nA units
#define ION_PUMP_CURRENT_UNDER_TRIP_POINT        2000               // 20uA this is in 10nA Units - The current must drop below this to clear overcurrent condition
#define ION_PUMP_CURRENT_ABSOLUTE_TRIP_TIME      200                // 2 Seconds This is in 10ms Units

#define ION_PUMP_CURRENT_HR_SCALE_FACTOR         .50863
#define ION_PUMP_CURRENT_HR_OVER_TRIP_POINT      100                // 1uA this is in 10nA units
#define ION_PUMP_CURRENT_HR_UNDER_TRIP_POINT     100                // 1uA this is in 10nA Units - The current must drop below this to clear overcurrent condition
#define ION_PUMP_CURRENT_HR_ABSOLUTE_TRIP_TIME   100                // 1 Seconds This is in 10ms Units


#define _5V_MONITOR_SCALE_FACTOR                 0.0763
#define _5V_MONITOR_OVER_TRIP_POINT              6000
#define _5V_MONITOR_UNDER_TRIP_POINT             4000
#define _5V_MONITOR_ABSOLUTE_TRIP_TIME           200

#define _15V_MONITOR_SCALE_FACTOR                0.0763
#define _15V_MONITOR_OVER_TRIP_POINT             16500
#define _15V_MONITOR_UNDER_TRIP_POINT            11000
#define _15V_MONITOR_ABSOLUTE_TRIP_TIME          200

#define MINUS_15V_MONITOR_SCALE_FACTOR           0.0763
#define MINUS_15V_MONITOR_OVER_TRIP_POINT        16500
#define MINUS_15V_MONITOR_UNDER_TRIP_POINT       11000
#define MINUS_15V_MONITOR_ABSOLUTE_TRIP_TIME     200

#endif	/* A37497_SETTINGS_H */

