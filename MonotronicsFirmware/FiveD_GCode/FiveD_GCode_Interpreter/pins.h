#ifndef PINS_H
#define PINS_H



#ifndef __AVR_ATmega644P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif

#define DEBUG_PIN 0
//Motor axiss, as connected in RepRap Monolithic electronics by TechZoneCom.com
#define X_STEP_PIN (byte)15
#define X_DIR_PIN (byte)18
#define X_MIN_PIN (byte)20
#define X_ENABLE_PIN (byte)24 //actually uses Y_enable_pin 
#define Y_STEP_PIN (byte)23
#define Y_DIR_PIN (byte)22
#define Y_MIN_PIN (byte)25
#define Y_ENABLE_PIN (byte)24 //shared with X_enable_pin 
#define Z_STEP_PIN (byte)27
#define Z_DIR_PIN (byte)28
#define Z_MIN_PIN (byte)30
#define Z_ENABLE_PIN (byte)29

//extruder pins as connected in RepRap Monolithic electronics by TechZoneCom.com
#define EXTRUDER_0_STEP_PIN (byte)12 //this is the step pin
#define EXTRUDER_0_DIR_PIN (byte)17

#define tipManageData (byte)13
#define tipManageClock (byte)14
#define bedManageData (byte)10
#define bedManageClock (byte)11

#define EXTRUDER_0_HEATER_PIN (byte)11
#define EXTRUDER_0_FAN_PIN (byte)16 //Used as general system fan
#define EXTRUDER_0_TEMPERATURE_PIN (byte)0
#define EXTRUDER_0_STEP_ENABLE_PIN (byte)3
#define EXTRUDER_0_ENABLE_PIN (byte)3
//Auxilary device (heated bed or second extruder) as connected in RepRap Monolithick electronics by TechZoneCom.com

#define EXTRUDER_1_HEATER_PIN (byte)11
#define EXTRUDER_1_TEMPERATURE_PIN (byte)10
#define BED_HEATER_PIN (byte)11
#define BED_TEMPERATURE_PIN (byte)10
//Breakout Pins as connected in RepRap Monolithic electronics by TechZoneCom.com
#define breakout1_PIN (byte)27
#define breakout1_PIN_A (byte)5
#define breakout2_PIN (byte)1
#define breakout3_PIN (byte)24  //can also be used for Aref
#define breakout4_PIN (byte)4
#define breakout5_PIN (byte)32  // AnalogSenseBreakout PIN (byte) D32/A0 this is set up with a pullup resistor and capacitor for thermister or other to use for digital you must remove the capacitor

#endif
