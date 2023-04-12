# D35 MPH Monitor

## Note on conversions and calculating speed
| Conversion Formula | Formula |
|--------------------|---------|
| Convert Knots to MPH | knots * 1.15078 = mph |
| Convert Knots to KPH | knots * 1.852 = kph |
| Convert MPH to KPH | mph * 1.60934 = kph |
| Speed Calculation | Software counts the pulses on an interrupt for 500ms and then calculates speed |


 mph = pulses detected in one second / (pulses output in a mile/3600)  note pulses output in a mile is the output of the speed device
pulses inlimi300 ft times 17.6 = pulses in a mile


(set software version on line 182)

# Changelog

This is a record of changes made to the software.

## Version 10
- Added field calibration.

## Version 11
- Added distance counter.
- Added reset button for distance.

## Version 22
- Added averaging routine to speed calculation.
- Do not display decimal on speeds over 30.
- Removed 'if' code from pulse interrupt routine that was causing pulses to reset to zero sometimes.

## Version 23TP
- For tractor puller application.
- Deleted the distance measuring mode.
- Show .1 mph increments for all speeds under 50mph.

## Version 24TP
- Changed the algorithm that captures pulses from the speed device (old system was not accurate enough).
- Went to positive edge on interrupt trigger to prevent false trips we were getting on negative edge trips of speed input line.
- Changed to 250ms speed updates for quicker response time.
- Made speed average an option on the option screen.
- Added speed average checkbox to option screen.

## Version 25TP
- Changed 250ms interrupt clock to reset to zero on start of first pulse.
- Added software version number in upper left corner of "Option Screen".

## Version 1.26
- Fixed problem of instruction text staying on screen when leaving field calibration mode.

## Version 1.27
- Added checkboxes to option screen for radar or gps input.
- Added code to read GPS on serial port "Serial2" //Serial2.begin(19200,SERIAL_8N1,25,22) 25-rx 22-tx.
- Moved radar input from pin 33 to pin 25 that is shared with rs232 input.

## Version 1.28
- Added pulsing alarm light that flashes faster as you near target.
- Created a 100ms timer with interrupt routine to flash alarm light.


## GPS Sensor

To output the RMC string from the GPS sensor, send the following code: 

```
$ /P /G /R /M /O /' /G /P /R /M /C /' /1/cr/lf
24/50/47/52/4D/4F/2C/47/50/52/4D/43/2C/31/0D/0A
```

Below is an example of the GPRMC string that the software decodes:
```
GPRMC,154030.4,A,3935.76900,N,08610.28625,W,000.02,240.3,171020,004.5,W*0.05C
```

- 0: Log Header
- 1: UTC of position
- 2: A=data valid; V=data invalid
- 3: Latitude (DDmm.mm)
- 4: Latitude direction (N=North, S=South)
- 5: Longitude (DDDmm.mm)
- 6: Longitude direction (E=East, W=West)
- 7: Speed in knots
- 8: Track made good (degrees true)
- 9: Date (dd/mm/yy)
- 10: Magnetic variation
- 11: Magnetic variation direction
- 12: Positioning system mode indicator
- 13: Checksum
- 14: [CR][LF] Sentence terminator

## Hardware

The breakout is built around the MTK3339 chipset, a high-quality GPS module that can track up to 22 satellites on 66 channels, has an excellent high-sensitivity receiver (-165 dB tracking!), and a built-in antenna. It can do up to 10 location updates a second for high-speed, high-sensitivity logging or tracking. Power usage is incredibly low, only 20 mA during navigation.

### Display Header Pinout

The 320 x 480 red board with a 14-pin header has a resistive touch with a touch controller built on board.

- 1: T_IRQ (N/C)
- 2: T_DO (19)
- 3: T_DIN (23)
- 4: T_CS (21)
- 5: T_CLK (18)
- 6: MISO (N/C) (Do not use; the display does not tristate this line when CS is deactivated)
- 7: BACKLIGHT (5V)
- 8: SCK (18)
- 9: MOSI (23)
- 10: DC/RS (15)
- 11: RESET (32)
- 12: CS (5)
- 13: GND
- 14: VCC 3.3V

## Fonts

Custom fonts used in the project should be defined in `lv_conf.h` (line 274).
```c++
#define LV_FONT_CUSTOM_DECLARE  LV_FONT_DECLARE(Bebasneue)
```
Fonts must be built using LittlevGL font converter program. The `Bebasneue` font was converted with LittlevGL font converter and used as the large speed font. Fonts must be saved in the LittlevGL "font" directory.

## User_Setup.h

Declare IO pins, fonts, and driver chips being used in `User_Setup.h`.

## User_Setup_Select.h

Uncomment the line `#include<User_Setup.h>` in `User_Setup_Select.h`.
