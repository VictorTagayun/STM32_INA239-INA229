# User Guide for Digital Constant Current SMPS 

You probably clicked the user guide, so u got here, as shown below.

https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01.png

This is the user guide for controllimg the Digital Constant Cureent SMPS.
This is a POC (Proof Of Concept) design stage.

 ## Hardware setup   
__Vin__ = 12V   
__Rload__ = 0.5 ohms   
__MCU__ = ST Micro   
__Mofset driver__ = IR2110 https://www.infineon.com/dgdl/Infineon-IR2110-DataSheet-v01_00-EN.pdf?fileId=5546d462533600a4015355c80333167e   
__Current Sensor__ = INA229EVM https://www.ti.com/tool/INA229_239EVM   
__Fsw_ = 100kHz or 10uS period


## Key fratures and limitation   
** 1A max = heat dissipation limitation   
** 55mA min = co trol algo still a work in progress   
** 10-20secs response time = need to improve control algorithm, but not needed in this POC (proof of concept) as the control engineers are "more" responsible about it

__*POC is to show or prove if the hardware can meet minimum requrements.*__

## User Guide

To control the digital smsps, u need to click the three lines (red box) so the tabs menu will show (green box), then click on "Advanced" tab, as shown below.

https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01.png

__Basic Tab__ - just to view the parameter and You cannot control anything   
__Advanced Tab__ - you will be able to:   
a. Close and Open the loop   
b. Program and change the pwm ton and   
c. Program and change the desired output constant current  

### Overview of Advanced tab

https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01.png

### Start/Stop Aquisition

This will start and stop the aquisition but will not stop the operation of the digital smps

https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01.png

### Write

You can write values to variables in the Fiwmware simultaneously or individually whenever those variables are checked or unchecked. After entering the number click on the "WRITE" button

__close_loop__ = accepts values 0 or on-zero number. O means open loop. Non-zero means close loop, generally write 1
__PWM_184nS__ = 1 count is 182ps of 10us period, maximum count is 54000, but it limited to 20000 at the moment in the firmware
__TargetCurrent__ = desired output current in mA

https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01.png

### Graphs

__Left Graph__ = mainly to view the output current in mA. But other variables are also shown. You can add or remove remove other variables by clicking the name of the riables

https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01.png

__Right Graph__ = other variables for monitoring and troubleshooting

## Simple Operation

Operation is mostly confined with:  
1. Changing the PWM Ton (open loop) and 
2. Changing the load current (close loop)   

It is advisable to un-check all variables first as shown below. IT is not neccesarry to click the "WRITE" button at this time.

You may change the PWM Ton, by enreting values for __PWM_184nS__, values should be within 700 - 4000, if incorrect entry is enterred, i.e. alphabet, the browser will show "x" next to the number and if numerically outside 700 to 4000 is entered, the Firmware will take care. This operation should be open loop and is the default state. After entering the number, click "WRITE" button.

To close the loop (__PWM_184nS__, will no longer take effect), enter 1 to __close_loop__ and click "WRITE" button. You may enter the target current (mA) in __TargetCurrent__. After entering the number, click "WRITE" button.

Note: You may enter __close_loop__ and __TargetCurrent__ simultaneously, then click "WRITE" button.

Further Improvements in the GUI datalogger and Firmware. But these improvements are not important at the moment as these does not impact the POC.
1. Show only output current in the Left Graph
2. Show Current status of loop control, either close or open loop
3. Improve the response time if possible
4. Program current to below 55mA

# THANKS and give feedback if possible

more interesting things at https://victortagayun.github.io

disclaimer




