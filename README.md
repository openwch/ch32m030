# Industrial Motor Microcontroller CH32M030

EN | [中文](README_zh.md)

### Overview

CH32M030 is based on barley RISC-V3B core design of industrial-grade motor microcontroller.CH32M030 built-in OPA and CMP, support the combination of 2 groups of AC small-signal amplification decoder QII and 2 groups of differential input current sampling ISP.Built-in USB PHY and PD PHY, support for USB Host and USB Device device functions, PDUSB, Type-C fast charging function, BC1.2 and DCP/CDP and other high-voltage charging protocols; built-in 4 pairs of N-type power tubes gate pre-driven to provide high-voltage I/O; built-in programmable current-flooding module; to provide DMA controller, 12-bit analog-to-digital converter ADC, multi-group timer, UART serial port, I2C, SPI, and other peripheral resources, to provide over-voltage protection and over-temperature protection. Over-voltage protection and over-temperature protection are provided.

### System Block Diagram
![[frame.png]]

### Features

- Barley 32-bit RISC-V3B cor
- Supports RV32IMAC instruction set and self-expanding instructions
- High-speed interrupt response mechanism
- Maximum 72M system frequency
- 12KB SRAM, 64KBCode Flash
- 512B user-defined information storage area
- Built-in high-voltage LDO, VHV supports rated 5-28V system power supply.
- Pre-driven I/O power supply VDD8 Rated voltage: 5~10V
- General I/0 and ADC power supply VDD33 Rated voltage: 3.3V
- Low power consumption modes: sleep, stop, standby
- 4 dual N-type MOSFET half-bridge drivers
- Built-in low dropout bootstrap diode, only external capacitor required
- Can be combined as a three-phase motor gate drive, can be combined as two independent full-bridge drive
- Built-in factory-tuned 8MHz RC oscillator
- Built-in factory-tuned 8MHz RC oscillator 340kHz RC oscillator, supports external 4-25MHz crystal.
- 7-channel general-purpose DMA controller, support TIMx/ADC/UART/I2C/SPI
- 12-bit analog-to-digital ADC, analog input range: GND ~ VDD33
- 20 external signal channels, support external delay trigger, support ADC sliding average function
- OTP over-temperature protection and OVP over-voltage protection and under-voltage reset
- Multi-pin mapped UART serial interface
- I2C interface, SPI interface
- 2 sets of Type-C and USB PD controllers and PHYs to support DRP, Sink and Source applications
- 4 high voltage tolerant CC pins
- Full-speed USB 2.0 controller and PHYs
- Supports PDUSB, Host and Device device modes
- Supports BC1.2 and multiple HV DCP/CDP charging protocols
- Built-in 6-bit DAC, programmable voltage output and pull-up/down
- One 16-bit advanced timer for deadband control and emergency brake, and complementary PWM outputs for motor control.
- 1 16-bit general purpose timer
- 1 16-bit Thin General Purpose Timer
- 1 Window Watchdog Timer
- System time base timer: 32-bit counter
- 4 op-amps OPA and 3 analog voltage comparators CMP can be combined to form 2 sets of AC small signal amplifier decoders QII1 and QII2 and 2 sets of differential input current samples ISP1 and ISP2
- OPA1 and OPA2 support self-biased PGAs.
- OPA3 and OPA4 support single-ended and differential inputs, PGA multi-step gain selection, and provide internal self-biasing voltage.
- CMP1 supports digital filtering
- CMP2 and CMP3 support N-terminal bias selectable, digital filtering
- CMP3 has multiple input channels, output to I/O or internal
- 2 sets of 10-bit programmable sink current modules
- 2 sets of source current modules
- 37 I/O ports mapping 16 external interrupts
- 8 MV pre-drive pins, 2 HV high voltage pins
- Security features: 64-bit chip unique ID
- Debug Mode: Supports both single-wire and dual-wire debug modes.
- Package: LQFP, QFN, QSOP