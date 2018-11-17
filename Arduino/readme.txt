HSLR Code
Readme_v1_5.txt
- HW_CONFIG:  Arduino Feather ARM Cortex M0 (SAMD21) BLE + PURE breakout board (for the 5V PS, LEDs)

- TO DO:
x- directly address PORT i/o registers
- dark subtraction (analog) or filter / subtract dark signal in post processing??
- dynamic integration-time setting
- external 1MSPS 16bit ADC (Analog devices)
- binary SD writing
- add RTC and date-time filenames to SD print
x- add voltage battery monitor
x- add temperature monitor
- add RTC and date-time SD filename
- IMU sensor unit
- GPS
- Bigger battery(!)

//
// hslr_v1_5:
- Direct port manipulation on samd21 to reduce integration time 
(went from 450 usec minimum to under 90 usecs;  76usecs in data sheet)
- Converted (sw) to 12bit samd21 (internal) ADC
changed line of code in fastAnalogRead to get 10bit -> 12bit
- Added voltage divider to Hamamatsu sensor video out signal to lower sensor output
from 5V max to 0 -> 3.3V;  lowered noise floor and using all of analog signal range
- Added LiPo battery voltage read-out if connected
printing values to serial and SD fileout
- Added Analog Devices TMP36 sensor / analog data to A2;  
ADC output noisy due to impedance (low-power) of temp sensor;  using delays and 
multiple AnalogRead samples, since data is noisy

//
// processing_hslr_v1_5:
- Changed ADCres setting to 12bit (re-scales y-axis)
- Lowered position of Sample Rate and Integration Time text on plot, since noise floor 
is now lower with voltage divider
- Set ignore counter to 7 data pts;  corrected two lines of code depending on this to be 
accurate in read data from Arduino code
- Added difference spectrum toggle (press key 's' to toggle, which records last spectrum
and plots remaining spectra as a difference plot);  difference spectrum turned off if 
'c', 't', '[' or ']' options selected
