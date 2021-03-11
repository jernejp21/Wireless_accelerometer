# Wireless accelerometer

Wireless accelerometer is a device, made of 2 pieces. One piece is transmitter (which also measures accelerations), second part is receiver, which receives data from multiple devices and sends it to PC.

## Transmitter - STM32L010

- Transmitter is a Nucleo32 board powered by SMT32L010RB.
- Accelerometer is LSM6DSL
- Wireless communication device is nRF24L01P

Device has data output rate of maximum 6663 Hz.

## Receiver - STM32L476

- Receiver is a Nucle3o2 board powered by SMT32L476RG.
- It uses the same wireless communication device as sender.
- It can read data from 3 sensors (3 sensors; each sensor sends X, Y and Z acceleration).
- All the data is transmitted to the PC via USB cable and UART serial protocol.

## PC software

PC software is still work in progress. First try was using python and PyQt, but I will probably use Unity for graphs and data plotting.

# Notes

In previous iteration, MCU on transmitter was AVR atmega328p. I have switched to STM32L010 because Cortex devices are easier to debug with the tools I have. I have also changed accelerometer (MEMS sensor), because STM provides shield for Nucleo boards with multiple MEMS sensors.

Previous project can be found [here](https://github.com/jernejp21/Accelerometer), but it will be removed when this project is up to date with previous iteration.