HSLR Code
v1.6.1VIIRS
Readme_v1_6.txt
- HW_CONFIG:  Arduino Feather ARM Cortex M0 (SAMD21) BLE or Adalogger
w or wo/ PURE breakout board (for the 5V PS, LEDs) and either RTC option
RTC_PCF8523 and RTC_DS3231. Start/Stop button supported

//**************************************************************************************//
TO DO:
x- directly address PORT i/o registers
x- dynamic integration-time setting
x(partial)- dark current measurement (analog) => subtract dark signal in post-processing
x- Add start-stop switch, Start acquisition by default 
x- add RTC (two versions included -- settle on one and retire the other?)
x- add 3.3 to 5V step-up DC (boost) DC converter and add level-shifting circuitry
x- (proto) add external BLUE Laser and WHITE LED
x- add voltage battery monitor
x- add temperature monitor
- *** binary writing to SD
- IMU sensor unit
- External 1MSPS 16bit ADC (Analog devices)
- write to USB Flash drive instead of SD via i2c or UART (speed??)!?
- add shutter and characterize dark-measurement temperature response
- Power / operational management -->  optimize storage and battery
- add control of sample-averaging / storage --> and improve sample-rates w/ fewer writes!!!
- External comms (GSM celluar, LoRa, or irridium)
- add depth sensor input for profiling -- what to do?
- RTC RTC_DS3231 has a temp sensor -- +/-3 C, could be useful to include this read as well
- GPS

DEMO improvements (visualization and wireless):
- Revise serial interface for demos to matlab or python?
- BLE laptop and / or phone interface (visualize + storage)

//**************************************************************************************//
// hslr_v1_6:
- Added pin-mode assignment for LASER_404 and WHITE_LED pins using register manipulation
(updated v1_7 also)
- Added dynamic integration function dynInteg() which drives output of 12bit data 
to target value > 90% of ADCrange and < ADCrange -1;  easy to turn-off
Set limit on max integration time to a max_integ_time variable -- set normally to 2 or 
3seconds (need to verify optimum for low-light conditions, otherwise sample rate becomes
very low).
- Improved integration adjustment response-time when saturation occurs 
by  forcing a fast drop in int_time to 1/8 of current int_time value
- Added RTC clock readout and storing / printing to file;  the stored string is in 
YYYYMMDDHHMMSS format -- this changes the list of outputs for serial print and SD print 
(and binaries TBD);  added support for both RTC_PCF8523 and RTC_DS3231 in #define
- Added line to support external SD card on Adafruit PFC8523 SD Featherwing;  
only difference is chipSelect=10 instead of chipSelect=4 on M0 Feather Datalogger
- Added commented placeholder values to print functions for additional variables to be 
added in SD data write only: depth (x.xx meters), GPS LAT/LON (x.xxxxxx) and 10DOF IMU 
(roll/pitch/yaw/heading/alt) (x.xx) for future testing only:  
do not need in serial print -- will be stored on SD (when binary write developed)
- Added pseudo-dark measurement;  can do either by taking consecutive analog reads when
ADC signal is ready but without advancing clock (not used), or remove ST signal toggle to
start integration (the 2nd is currently used) -- not sure either approach is right.  
Integration time set to minimum during pseudo-dark measurement and (timings[3]) parameter 
set to 0 to identify dark measurement in output data
- Cleaned up void setup(){} by wrapping SD and BLE initialization in separate functions
- Cleaned up timings[] array;  integration time stored in timings[3] not [2]
- Added SSTstate manual push-button interrupt service request on pin 12 with 
ceramic capacitor 0.1uF and pull-down resistor 100kOhm -- seems to work well
- Added RED led (pin 13) ON-OFF to indicate SSTstate 
(NOTE:  it is OFF when running; ON when stopped in order to minimize light inside housing) 
- Refined GREEN led for SD card (pin 8);  ON for SD init. ready & flicker for SD writes
(NOTE:  should remove all internal LED blinking in future release;  
enable only for debugging)
- Changed spec_counter data type to uint32_t from uint16_t to reduce overloading for long 
runs
- Fixed filename increment so second digit increments correctly when > H099.DAT


- WIP:  started working on writeSerialBinary() function

//***************************************************************************************
// processing_hslr_v1_6:
- Added spectrum counter increment display to plot above wavelength label
- Moved Laser ON / White LED ON box / labels 5pts to right for visual
- Modified 1_6 to accomodate RTC entry in serial print data (placeholders);  note 
processing_hslr_v1_6 will not work with earlier versions of Arduino code without changes
to data input format
