# Notes

## In the Sanguino core (obsolete, already committed fixes>)
- Edit boards.txt baudrate to 38400 (bootloader in the atmega644 was built with that)

## In the Monotronics Firmware (obsolete, already committed fixes>)
- Edit FiveD_GCode_Interpreter.pde move #include "WProgram.h" up above #include <HardwareSerial.h>
- Edit configuration.h, look for the section MOTHERBOARD == 3, look for the #define Z_STEPS_PER_MM define, it should be 1000 

## Setup before building
- Copy the Sanguino core dir into hardware/Sanguino of the IDE dir. You should have a dir structure like this:
```
arduino-0021
|-hardware
  |-Sanguino
  |-arduino
  |-tools
```

## In the Arduino IDE
- Open arduino-0021/arduino.exe
- Set serial monitor baud to 19200
- Open MonotronicsFirmware/FiveD_GCode/FiveD_GCode_Interpreter/FiveD_GCode_Interpreter.pde
- Click Upload, should see "Uploading to I/O Board..." message. Don't wait for this to be done, continue with...
- Press and hold the reset button on the electronics
- Wait for the "Binary sketch size: blah bytes" message, then immediately release the reset button
- Should see "Done uploading" message


## Helpful GCode
Startup:
```
G21 ; metric
G90 ; absolute positioning
G28 ; home all axis by looking for the low limit switch
M106 ; fan on (no fan connected, but still)
```

Shutdown:
```
M107 ; fan off (no fan connected, but still)
G91 ; relative positioning, precaution in case of wrong command
M112 ; shutdown
```

Example commands:
```
M105 ; get both tip T and bed B temperatures
M111 6 ; turn on some debugging msgs (maybe higher number?)
G1 X90 F1000 ; move X to absolute position 90mm at speed 1000
G1 Y90 F1000 ; move Y to absolute position 90mm at speed 1000
G1 E500 F1000 ; advance filament 500mm at speed 1000
G1 E500 F20 ; repeat previous command, but change feedrate, preparation for lower feedrate for Z, otherwise it goes crazy
G1 Z60 F20 ; move Z to absolute position 60mm at speed 20
```

## Other Notes
- configuration.h has most of the firmware params for calibration
- Extruder won't move unless tip temp is above some threshold
- X/Y FAST feedrate is 3000 mm/min
- X/Y SLOW feedrate is 1000 mm/min
- Z FAST is 50 mm/min
- Z SLOW is 20 mm/min
- Z axis is 1 turn of the screw/mm, cog ratio is 8/20, stepper is 400steps/rev => 400*20/8 = 1000steps/mm (value in the last firmware of 800+ was wrong)
- Power supply needs to be at least 12V 10A with bed and tip

