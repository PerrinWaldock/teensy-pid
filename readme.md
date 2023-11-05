# Teensy-based Intensity Feedback Controller Guide

This repository contains code and PCBs for creating Teensy microcontroller-based feedback controllers. They are used to control the laser intensity on Prometheus because (a) they are relatively inexpensive and (b) they provide faster rise times than similarly-priced analog feedback controllers.

## PCB-free Feedback Controller

The code in the `multistate` branch is currently installed in the Prometheus apparatus. It is running on a Teensy 3.2 because that microcontroller has 5V-tolerant digital inputs, and an inbuilt DAC. It uses two digital signals from the QDGBus digital outputs to select one of four setpoints which are configured over USB. It controls the voltage into a XM-B6H4-0404C-01.

However, this setup has some flaws:
* the inbuilt ADC and DAC are noisy, with only ~8-10 usable bits. This limits the feedback stability shown in the above posts.
* there is no input anti-aliasing filter
* the connections are soldered together on stripboard. The digital input controlling the LSB of the state is currently loose, so the feedback controller occasionally malfunctions when the digital input is not connected to the microcontroller
* the Teensy 3.2 is currently EOL so it is challenging to find a replacement.

A replacement PCB was built that provides the following features:
* input anti-aliasing filtering
* input and output buffering
* input over-voltage protection for both digital and analog inputs
* 16-bit ADCs and DACs
* a faster (600 MHz) microcontroller
* a separate +5V power supply for all analog electronics
* space on the board to mount an XM-B6H4-0404C-01.

The new PCB and its corresponding control code is in the `master` branch of the repository. Below is a guide to the PCB and control code:

## PCB Important Components
The KiCAD design files and schematics are in the schematics folder of this repository. Below is a guide to some components that can be populated or depopulated to change the performance characteristics of the board:
* R1: Populate this to give this board a 50-ohm terminated input. Useful when interfacing with a [PDA100A2](https://www.thorlabs.com/thorproduct.cfm?partnumber=PDA100A2) because they produce a 0-5V output when terminated with a 50 ohm load.
  * One can also modify the values of R1, R2 to remove a DC offset from the input signal, or modify the values of R7 and R9 to amplify the input. Useful if the input is a small signal.
* R3: Populate this to bypass the input op amp. Do not do this without first disconnecting the op amp from by depopulating R2 and R9. Often used when R15 and R17 are populated to use the internal ADC instead of the external ADC (not recommended).
* C1 and R7: Change these values to change the input op amp filter (currently not populated).
* R16 and R18. ONLY POPULATE ONE OF THESE AT A TIME. Used to select 5V or 3.3V reference. If 5V reference is selected (R18), make sure to depopulate R6 so that the input voltage range is the full 0V-5V.
* R34: Populate to use external 5V instead of USB 5V power. DO NOT DO THIS WITHOUT FIRST COVERING THE PIN THAT SUPPLIES POWER ON THE USB CABLE: https://community.octoprint.org/t/put-tape-on-the-5v-pin-why-and-how/13574
* R40: Populate to bypass the output buffer. Do not do this without first disconnecting the op amp by depopulating R41 and FB2.


## Software Important Preprocessor Directives
You can access the following directives in the arduinopid.h file:
* OUTPUT_SETTLE_DELAY_US should be set to the response time of the voltage variable attenuator. It is a delay that kicks in immediately after the set point changes from off to on.
  * When the output changes state from off (below the minimum set point) to on (between the minimum and maximum setpoint), the feedback calculation is skipped for the first output, and only the feedforward value is used. This allows the feedback controller to rapidly approach the desired value. However, it takes a finite amount of time for the system to reach the desired value. This delay stops the next feedback measurement from being taken before the system settles to its new state.
* PIN_REFERENCE0 and PIN_REFERENCE1 are used to select which preprogrammed set voltage the feedback controller will try to attain.
* PIN_REFERENCE can be used as a traditional analog setpoint (NOTE: The PCB is not currently set up to support this)
* INPUT_MODE sets the type of input that the feedback controller uses to select its state. DIGITAL_INPUT selects preprogrammed set voltage number PIN_REFERENCE1*2 + PIN_REFERENCE0, ANALOG_INPUT uses the voltage on PIN_REFERENCE, and SOFTWARE_INPUT only accepts software changes to the set voltage. Note that the ANALOG_INPUT mode is not currently well-tested.
  * It may be advantageous to reduce ANALOGREFERENCERESOLUTION to 8 or lower to speed up the loop frequency.
* FEEDFORWARD enables feedforward control. One can characterize the transfer function of the system via a serial command, then perform corrections to the feedforward function using PID. It generally produces faster rise times than standard PID feedback control.
* NEGATIVE_OUTPUT should be set to true if a larger output voltage should produce a smaller input voltage and vice versa
* LIMITED_SETPOINT set to true means that PID control is only active within a certain setpoint range. Set to true to avoid integral wind-up when the output is set to zero or maximum. These
* SAVE_DATA will give the option to save PID constants, loop frequency, and feedforward data and load them on startup.

## Serial Commands
Connect the feedback controller using Putty or the Arduino IDE to tune the feedback parameters, adjust the setpoint, loop frequency, and other parameters.

Note that the FeedbackController class in the Prometheus repository uses some of the following serial commands in order to set the laser power to a certain level at specific points in a recipe.

Commands:
* h to print the help menu
* kp=x to set the proportional constant. kp? to print the proportional constant. Similar for ki and kd.
* sv0=0.5 to set the 0th preprogammed set voltage to 0.5 volts. similar for sv1, sv2, and sv3. use sv? to view all for set voltages. Use sp to set the value as an integer instead of as a voltage.
* ls=0.05 to set the minimum setpoint to 0.05V. Set to some value close to the minimum attainable feedback value. Used to prevent integral wind-up if the system cannot reach the set value.
* ls=4.5 to set the maximum setpoint to 4.5V. Set to some value close to the maximum attainable feedback value. Used to prevent integral wind-up if the system cannot reach the set value.
* lf=2000 to set the loop frequency to 2000 Hz. lf? to view the loop frequency. Make sure that the loop period is not less than the time the controller takes to perform a pid calculation.
* ra=2 to set the number of read averages to four. The formula is <read averages>=2^x where x is an integer -- this ensures that the number of read averages is always a multiple of two, which speeds up the averaging process. Use ra? to view the number of read averages
* pa=1 to enable feedback control, pa=0 to disable it
* po=1 to print sample feedback control values. po=0 to disable the printed sample feedback control values. A sample string would be `s:6553	 f:6553	 ff:13490	 o:12850	 t:16`, where s refers to the set voltage, f is the feedback voltage, ff is the output voltage the feedforward term recommends, o is the output voltage of the controller, and t is the time the controller took to perform the feedback calculation. All voltages are written as integers, and the time is in microseconds. Write po=2 to display the values as voltages instead.
* cf=1 to create the feedforward function. Write cd? to print out the calibration data, and ce? to print out the calibration extrema (minimum and maximum set voltage).
* ew=1 to write the feedback constants, loop rate, and calibration data to EEPROM so they are persistent after startup. ew=0 to read all data from EEPROM. Useful when tuning to restore the feedback controller to a working state.
* cl=1 to clear the integral term -- helpful if the integral term has spooled up.

Typical usage:
Via the USB serial connection, enter the following comnmands:
* `cf=1` (to calibrate the feedforward function)
* `ce?` (to print the minimum and maximum values that can theoretically be set)
* `sv0=1` (to set the set voltage to 1V. Replace the 1 with a value close to the maximum set voltage found above)
* `po=2` (monitor the output, ensure that it is stable)
* `po0` (turn off the output)
* set the tuning parameters and loop frequency with `ki=x`,`kp=x`,`kd=0`, `lf=100000`. Ensure that the calculation time does not exceed the loop period. Adjust parameters until performance is satisfactory.

Some of these functions are implemented automatically in FeedbackController class in hardware/FeedbackController in the [Prometheus repository](https://qdg-code.phas.ubc.ca:2633/Perrin/PrometheusPython). This class automatically uses USB commands to assign different set voltages to different set voltage states during the recipe's compile time, then uses digital outputs to select those states while the recipe is executing. It automatically calibrates the feedforward function during compile time.

## Future Work To Do
### Speeding things up
Currently, the microcontroller is set to use a 600 MHz clock. Attaching a heatsink to the processor should allow high clock rates.

Higher clock rates may disrupt communication with the ADC and DAC. To improve the ADC and DAC throughput:
* connect an oscilloscope to the data lines as well as the analog output
* change "#define dacdigwr(p, s) digitalWrite(p, s)" in dac.ino to "#define dacdigwr(p, s) digitalWriteFast(p, s)"
* adjust WRITE_DELAY_NS such that the DAC still consistantly writes the correct value
* similarly, adjust WAIT_TIME_NS and CLOCK_DELAY_NS in adc.ino.
* R21, R22, R23, R24, R25, R26, R30, R31, and R32 may need to be adjusted to a smaller value to accommodate higher clock rates. Choose values that are as large as possible while still allowing the clock to remain high for the specified time in the ADC and DAC datasheets.

### Testing
The latest version of the PCB (V3.0) has not been tested as a feedback controller for a laser intensity lock.

### New Algorithms
This controller should be usable for a gradient-descent saturated adsorption lock algorithm.
