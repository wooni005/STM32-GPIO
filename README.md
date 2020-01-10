# STM32_GPIO

This project is using the STM32F103C8T6 aka Blue Pill for 24 GPIO pins and support for the BMP085 barometer/temperature sensor.
All I/O is done via the uUSB cable, which is shown in the OS as a serial interface.

## Programming the board

I'm using the Arduino IDE with STM32duino in the board manager. No need to flash the bootloader, I'm using the original bootloader with STLink V2, no need to use the boot jumpers.
You can find here more information about the setup here: 
https://alselectro.wordpress.com/2018/11/18/stm32f103-bluepill-getting-started-with-arduino-core/

## Usage

When the board is programmed, connect the micro-USB cable to the machine and startup a serial terminal program.
Press 'h' to see the available commands:

```
Available commands:
a          - get all I/O values
<nn>i      - get input <nn>
<on>,<nn>o - set output <nn> <on> (nn=pinNr 1..16, <on>=0-off/1-on)
<nn>n      - set node ID (0..7)
<n>l       - set activity led on/off (0: off, 1: on)
p          - get temperature and barometer
<n>d       - switch on pulldown inputs (0: INPUT, 1: INPUT_PULLDOWN)
<n>u       - switch on pullup inputs (0: INPUT, 1: INPUT_PULLUP)
t          - for hardware testing check if pin is bouncing
v          - display board name and board id
h          - this help
```

### Inputs

Default all pins are programmed as input with pullup. You will receive this message:

```
OK I <pinIndex> <pinName> <status>
```

For example:

```
OK 17 PA10 0
```

When sending the 'a' command, you will receive all input and output statuses at once in the same format.

### Output

When sending an output command, this pin will be programmed at that time in an output pin. Switch an output pin like this:

```
<status>,<pinIndex>o
```

For example to switch pinIndex 17 to high:

```
1,17o
```

**Remark:** to reprogram all outputs back to input, use the 'u' or 'd' command.

### Onboard led

To switch the onboard led:

```
<status>l
```

For example to switch the led on:

```
1l
```

### BMP085

It's possible to connect a BMP085 to the I2C bus. The bus will be initialized when sending the 'p '. If the I2C bus is not used, these pins can be used for I/O.

I2C_SDA_PIN   PB7
I2C_SCL_PIN   PB6

```
OK P <pressure> <temp>
```

For example:

```
OK P 95100 21,20
```

### Node ID

It's possible to program a node ID for the board, so several boards can be used on one machine and will be identified by the node ID. The node id is limited from 0 till 7, but this limit can be undone in the code.

You can find the node ID under the help when pressing 'h' and also when pressing 'v'. 'v' is used to identify the board name and board/node ID. For example for nodeId 2:

```
v
[STM32-GPIO.2]
```

For example to set the node ID to 3, use the 'n' command:

```
3n
```

### pinIndex

```
PA0,  //pinIndex: 00
PA1,  //pinIndex: 01
PA2,  //pinIndex: 02
PA3,  //pinIndex: 03
PA4,  //pinIndex: 04
PA5,  //pinIndex: 05
PA6,  //pinIndex: 06
PA7,  //pinIndex: 07
PB0,  //pinIndex: 08
PB1,  //pinIndex: 09
PB10, //pinIndex: 10
PB11, //pinIndex: 11

PB9,  //pinIndex: 12
PB8,  //pinIndex: 13
PB7,  //pinIndex: 14: Also used for i2c-SDA
PB6,  //pinIndex: 15: Also used for i2c-SCL
PB5,  //pinIndex: 16
PA10, //pinIndex: 17
PA9,  //pinIndex: 18
PA8,  //pinIndex: 19
PB15, //pinIndex: 20
PB14, //pinIndex: 21
PB13, //pinIndex: 22
PB12  //pinIndex: 23
```

