# User Guide for Digital Constant Current SMPS 

You probably clicked the Simple User Manual, so you got here, just like what is shown below.

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01-01.png)

This is the user manual/guide for controllimg the Digital Constant Current SMPS.   
This is a __*POC*__ (Proof Of Concept) design stage.

 ## Hardware setup   
__Vin__ = 12V   
__Rload__ = 0.5 ohms   
__MCU__ = ST Micro   
__Mofset driver__ = IR2110 https://www.infineon.com/dgdl/Infineon-IR2110-DataSheet-v01_00-EN.pdf?fileId=5546d462533600a4015355c80333167e   
__Current Sensor__ = INA229EVM https://www.ti.com/tool/INA229_239EVM, connected at the __*low*__ side   
__Fsw__ = 100kHz or 10uS period   
__Topology__ = Syncronous Buck Conveter (non-isolated)   


## Key features and limitation   
__10A max__ = due to heat dissipation limitation, using small heatsink   
__40mA min__ = control algorithm still a work in progress   
__30-50secs response time__ = need to improve control algorithm, but not needed in this __POC__ (proof of concept) as the control engineers are *"more"* responsible about it

__*POC is to show or prove if the hardware can meet minimum requrements. It does not include the firmware and control feedback algorithm.*__

## User Manual/Guide

To control the Digital SMPS, you need to click the three lines (red box) so the tabs menu will show (green box), then click on "Advanced" tab, as shown below.

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/01-02.png)

__Basic Tab__ = just to view the Firmware variables/parameter and You cannot control anything   
__Advance Tab__ = you will be able to:   
a. Close and Open the loop   
b. Program and change the pwm Ton count (1 count = 184nS) and   
c. Program and change the desired output constant current  


### Overview of Advance tab

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/02.png)


### Start/Stop Aquisition

This will start and stop the aquisition but will not stop the operation of the digital smps

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/02-StartStop.png)


### Write

You can write values to variables in the Firmware simultaneously or individually whenever those variables are checked or unchecked. After entering the number click on the "WRITE" button

__close_loop__ = accepts values 0 or on-zero number. 0 means open loop. Non-zero means close loop, generally write 1   
__PWM_184nS__ = 1 count is 182ps of 10us period, maximum count is 54000, but it limited to 20000 at the moment in the firmware   
__TargetCurrent__ = desired or target output current in mA   

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/02-Write.png)


### Graphs

__Left Graph__ = mainly to view the output current in mA. But other variables are also shown. You can add or remove other variables by clicking the name of the variables below the graphs as shown below

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/02-LeftGraph.png)

__Right Graph__ = other variables for "my" monitoring and troubleshooting

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/02-RightGraph.png)


## Simple Operation

Operation is mostly confined with:  
1. Changing the PWM Ton (open loop) and 
2. Changing the load current (close loop)   

It is advisable to un-check all variables first as shown below. IT is not neccesarry to click the __"WRITE"__ button at this time.

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/03-Uncheck.png)

It is best and safe to open the loop first if the current state is not in open loop or unknown. Check the __open_loop__ check box and enter 0.
You may change the PWM Ton, by checking its left checkbox too and entering values for __PWM_184nS__, values should be within __900 - 10000__, if incorrect entry is enterred, i.e. alphabet, the browser will show __"x"__ next to the number and if numerically outside of __32 to 50000__ is entered, the Firmware will take care and limit its value. At this moment it is advisable to keep value of __PWM_184nS__ to within __900 - 10000__ for demonstration purposes. Exceeding 10000, will generate heat to the MOSFETS. After entering the numbers (__open_loop__ and __PWM_184nS__), click __"WRITE"__ button.

To close the loop (__PWM_184nS__, will no longer take effect), enter 1 to __close_loop__ and click __"WRITE"__ button. You may enter the target current (mA) in __TargetCurrent__. After entering the number, click __"WRITE"__ button.

![](https://raw.githubusercontent.com/VictorTagayun/STM32_INA239-INA229/main/Simple_UserManual/pixx/04-Closeloop.png)

__*Note*__ : You may enter __close_loop__ and __TargetCurrent__ simultaneously or individually, then click __"WRITE"__ button  on every change to take effect.

Further Improvements in the GUI datalogger and Firmware. But these improvements are not important at the moment as these does not impact the POC.
1. Show only output current in the Left Graph. At the moment, some variables need to turn off.
2. Show Current status of loop control, either close or open loop. At the moment, it cannot be determined if the loop is open or close.
3. Improve the response time if possible.
4. Program current to below 40mA.
5. Increase current to 15A - 20A.


# THANKS and give feedback if possible


more interesting things at [https://victortagayun.github.io](https://victortagayun.github.io)

*Disclaimer:*
[See Updated "Global Disclaimer" in case disclamer below is outdated](https://github.com/VictorTagayun/GlobalDisclaimer)

*The projects posted here are for my Personal reference, learning and educational purposes only.*
*The purpose of a certain project may be for testing a module and may be just a part of a whole project.*
*It should not be used in a production or commercial environment.*
*Any cause of injury and/or death is the sole responsibility of the user.*
