# STM32 Cube IDE to ROS2 motor pilot and monitoring
## Driving 4 stepper motors from ROS2


## Harware requirements

- 2 x stacked [XnucleoIHM02A1](https://www.st.com/en/ecosystems/x-nucleo-ihm02a1.html)
- 1  x [Nucleo-F446RE](https://os.mbed.com/platforms/ST-Nucleo-F446RE/) developpement board 
- 4 x 2 phase stepper motor such as [Sanyo Denki SF2423-10B41](https://products.sanyodenki.com/en/sanmotion/stepping/f2/SF2423-10B41/)
- External power supply matching stepper motor nominal voltage (24V here)
- Male to male Arduino Jumper
- Heating sink for high motor speeds
- USB Mini-B to USB uploading cable

## Features ( from ROS2 node)

> Those are the supported interactions with ros2 node
> Other motor control parameters are to be set directly in the Nucleo-F446RE Firmware
> (see Features of motor driver)

- Drive motors with rad/s speeds
- Get measured speeds by driver

## Features Of Motor driver

> You can decide to discard the communication part with ROS2
> in order to keep only the motor driving part

- Drive 4 motors with rad/s speeds or normalized speeds
- Drive 4 motors with a desired number of step
- Set microstepping mode of driver
- Several Motor stop routines 
- Set maximum acceleration, maximum decelleration of motors
- Set maximum absolute speed of motors

## Hardware Configuration
### Resolving Pin conflict
From brand new motor driver, several steps are to be taken.
On one of the shield, that will be reffered as the **top shield** :

**Move SB23 solder bridge from SB23 to SB7 using a solder iron.**
| ℹ️ Note                         | 
|------------------------------------------|
| Since the MCU communicates with the driver using SPI protol, that step is to be taken in order to change the hardware configuration of one of the 2 shield so that the selection pin of the SPI protol is not the same for both shields (it would create a conflict otherwise)|

The motor shields have the same reset pin, but contrary to SPI selection pin, there is no solder bridge to change that configuration. That is why on top shield you should :
**Bend the pin ont the top shield that would go on the PB5=D4  pin of the MCU (obviously the one on the Arduino CM9 morpho header)**

With those changes, the pin used by the 2 drivers are :
| Function | Bottom shield | Top Shield |
| ----------- | ----------- | ----------- |
| spi SDI = MOSI | PA7 = D11 | PA7 = D11 |
| spi SDO = MISO | PA6 = D12 | PA6 = D12 | 
| spi SCK | PB3 = D3 | PB3 = D3 |
| spi NCS | PA4 = A2 | PA10 = D2 | 
| Reset pin | PB5 = D4 | PB4 = D5 | 

| ⚠️ Warning                               | 
|------------------------------------------|
| Connect the arduino jumper between D4 and D5 so that top shield reset pin is connected to D5 (as D4 pin was bended previously to prevent a conflict.    |

There are also pin used by the driver but not implemented in the code. If you plan on using those pin trough the arduino connector you may encounter unexpected behavior. So it is wise to use the external connector (CN7 and CN10)

| Function | St Pin format | Arduino Pin format |
| ----------- | ----------- | ----------- |
| Busy Pin | PC0 | A5 |
| Flag Pin | PC1 | A4 | 
| Good Pin | PB0 | A3 |
### Connecting the motor

In the case of the sanyo denki stepper motor, the wiring is the following. You can replicate it for other stepper motors by comparing the datasheet of this stepper motor and the one you want to add.

| Terminal | Motor Color |
| ----------- | ----------- | 
| 1A | orange |
| 2A | blue |  
| 1B | red | 
| 2B | yellow | 

## API
Motor functions are explained in BlocMoteurs.cpp

## Motor Profiling
For better performances, it may be useful to perform a profiling of the motor.

More info at [Profiling Motor](/doc/MotorProfiling.pdf)

## Connecting to Ros2

See : [Ros 2 node repo](https://github.com/Adrien9o2/stm32_to_ros_rasp)

## MCU Configuration

See : [MCU Configuration report](doc/stm32_to_ros.pdf)
Beware that modifying clock configuration and timer 2 Counter Period may result in unwanted behavior.

## License

MIT

**Free Software, Hell Yeah!**
