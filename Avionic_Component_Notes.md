## **3 AXIS ACCELEROMETER ADXL375**

Measurement

* 3 axis accelerometer measuring static/dynamic acceleration
* Digital output, 16 bit 2's complement
* 3 int16\_t outputs for x, y, z axis
* code will convert 16 bit data to g's

Connection

* SPI or I2C
* SPI makes more sense for faster data rate

SPI Pins

* Chip select: ADXL CS ->SPI CS
* Clock: ADXL SCLK -> SPI SCK
* MOSI: ADXL SDI -> SPI MOSI
* MISO: ADXL SDO -> SPI MISO
* Interrupt output: ADXL INT1 and INT2 -> GPIO
* Max speed = 5MHz

Power Requirements

* VS (Main Supply): 2.0V-3.6V
* VDD I/O: 1.7V-VS
* Measurement current: 35-145 micro amps
* standby: 1 micro amp

Integration Features

* 32 level FIFO Buffer
* 4 Modes: Bypass, FIFO, Stream, Trigger

Interrupts

Both pins, INT1 and INT2 can trigger on:

* Data ready
* Shock detection
* Activity
* Inactivity
* FIFO watermark
* Overrun

These are enabled/disabled via registers

Data Rate

* 0.1Hz -> 3200 Hz

## **3 AXIS MAGNETOMETER LIS2MDL**

Measurement

* Magnetic field in x, y, z
* Measured in milligauss \[mG]

Data

* 16 bit 2's complement
* Separate registers for OUT(A)\_L and OUT(A)\_H, where (A) could be X, Y or Z
* STM reads int16\_t for X,Y,Z

Connection

* SPI or I2C

SPI Connections

* Chip select: LIS CS ->SPI CS
* Clock: LIS SCLK -> SPI SCK
* MOSI: LIS SDI -> SPI MOSI
* MISO: LIS SDO -> SPI MISO
* Interrupt output (optional): GPIO
* MAX speed 10MHz

I2C Connections

* SDA -> Data
* SCL -> Clock

I2C Address:

* 0x1E (if SDO = GND)
* 0x1c (if SDO = VDD)

Power Requirements

* VDD = 1.71V - 3.6V
* VDD\_IO = 1.71V - 3.6V
* Current (continuous) ~100 micro amps

Data Rate

* &nbsp;4 data rate options \[Hz]: 10, 20, 50, 100

Interrupt

* 1 Interrupt Pin, triggered on:
* Data ready
* Threshold event

## **6 AXIS ACCEL GYRO ICM-40609-D**

Measurement 

All 16 bit 2's complement, all int16\_t

* accel X, Y, Z
* Gyro X, Y, Z
* Temperature sensor

Communication

* SPI or I2C
* SPI optimal, up to 24MHz

SPI Connections

ICM pin -> STM pin

* Serial Clock: SCLK -> SPI SCK
* Serial Data In: SDI -> SPI MOSI
* Serial Data out: SDO -> SPI MISO
* Chip Select: CS -> GPIO
* Interrupt: INT1/INT2 -> GPIO

I2C Connections

* SDA -> I2C Data
* SCL -> I2C Clock

Power

* VDD = 1.71-3.6V
* VDDIO = 1.71-3.6V
* Current (typical) ~1mA

Interrupts

* INT1 and INT2 connect to STM GPIO
* Can Signal when
* Data ready
* FIFO watermark
* Motion detection
* Significant motion
* Wake-on-motion

2KB FIFO Buffer

## **6 AXIS ACCEL GYRO LSM6DSV16X**

Measurement

* 3 axis digital accelerometer+gyroscrop
* both 16 bit 2's complement
* Also embedded temperature and Qvar sensor

Connection

* SPI, I2C and I3C
* 10MHz SPI, 1MHz I2C

SPI Pins

* Chip Select: CS -> SPI CS
* Clock: SCL -> SPI SCK
* MOSI: SDA/SDI -> SPI MOSI
* MISO: SDO/SA0 -> SPI MOSI

I2C Connections

* SDA -> Data
* SCL -> Clock
* I2C Address:
* 0x6A if SA0 = GND
* 0x6B if SA0 = VDD\_IO

Power

* VDD = 1.71-3.6V
* VDD\_IO = 1.08-3.6V
* current (typical) 0.65 mA
* Current (accelerometer) 190 micro amps

Integration

* Smart FIFO 4.5kB with data compression
* Machine learning support
* Sensor hub: mode 2 allows 4 external sensor connections via master I2C

Interrupts

* 2 programmable interrupts INT1 and INT2
* can be triggered by: Free fall, wakeup, 6D/4D orientation, single/double click, activity/inactivity, significant motion detection, tilt, pedometer step detection, interrupts triggered by finite state machine/machine learning

Data Rate

* Accelerometer up to 6400Hz
* Pedometer algorithm at 30 Hz, FSM processes up to 960 Hz

## MAGNETIC BUZZER FUET-8530-3.6V

Function

* 2700Hz oscillation frequency
* Sound output: minimum  85 dB (10cm away at rated voltage)
* Works by applying a 2700Hz square wave from STM32

Connection

* Drive signal: requires 1/2 duty square wave (pulse width modulation)
* Drive frequency: 2700 Hz optimal (resonant frequency)
* Positive terminal connected to driving circuit
* negative terminal connected to ground

Driving circuit

* Requires an external NPN transistor to handle current
* Flyback diode should be connected in parallel with the buzzer to protect from voltage spike

Power 

* Rated voltage: 3.6V peak to peak
* Operating voltage = 2.5-4.5V
* Current consumption: Max 100 mA
* Coil resistance 16 +/- 3 ohm

\*\*\*\*\*NOTE: 100mA IS MUCH LARGER THAN OTHER COMPONENTS, ENSURE POWER RAILS CAN HANDLE WHEN BUZZER IS ACTIVE\*\*\*\*\*\*

## BAROMETRIC PRESSURE SENSOR MS5607-02BA03

Measurement

* Pressure range 10-2000 mbar
* temperature -40 -> 85 C
* 24 bit digital output for pressure and temperature
* Accuracy @ 25 C: +/- 1.5 mbar

Connection

* SPI (20MHz) and I2C
* Protocol selection: SPI Mode PS -> GND, I2C Mode PS-> VDD

SPI Pins

* Chip Select: CSB -> GPIO (Active Low)
* Clock: SCLK -> SPI SCK
* MOSI: SDI -> SPI MOSI
* MISO: SDO -> SPI MISO

I2C Pins

* SDA -> Data
* SCLK -> Clock
* I2C Address = 111011C, where C is the complement of the logic level on the Chip select pin (CSB)

Power

* VDD = 1.8V - 3.6V (3V recommended)
* Current consumption: Standby < 0.15 microamp, typical peak = 1.4mA, average current 0.9-12.5 micro amps
* Decoupling: requires 100 nf ceramic capacitor placed as close as possible to VDD pin

Interrupts

* No interrupt pins
* in I2C, master monitors for acknowledge signal
* in SPI, user wait for specified conversion time

Data rate

* Conversion time: Programmable from 0.5-8.22 ms
* Maximum ODR: limited by communication interface chosen (OSR 256 allows > 1000 Hz)

## **U-BLOX CAM-M8 CONCURRENT GNSS MODULE**

Measurement

* 72 channel GNSS engine: receieves and tracks up to 3 concurrent GNS systems
* Supported systems: GPS/QZSS, GLONASS, Galileo, Beidou
* Outputs position, velocity, time, and raw satellite signal measurements

Connection

* Interfaces: UART, SPI and DDC (I2C application)
* Interface selection, controlled by D\_SEL pin
* D\_SEL = Open: UART and DDC enabled
* D\_SEL = GND: SPI interface enabled
* UART pins: TXD transmit data output, RXD receive data input

SPI pins (Slave only)

* MISO shared with UART TXD
* MOSI Shared with DDC SDA
* SCK Shared with DDC SCL
* Chip select (CS\_N) shared with UART RXD
* max speed 5.5 MHz

DDC (I2C) pins

* SDA Data
* SCL clock
* Max speed 400kb/s

Power

* Main supply: CAM-M8Q 2.7-3.6V, CAM-M8C 1.65-3.6V
* Backup supply (V\_BCKP) 1.4-3.6V for RAM and RTC backup

Interrupts

* EXTINT (external interrupt)
* TIMEPULSE, configurable pulse output (1 Hz def.) up to 10MHz
* SAFEBOOT\_N: pin used for entering safe boot mode 

Data Rate

* GPS and GLONASS: up to 10Hz
* Single GNSS (GPS): Up to 18 Hz

## **LoRa SMD MODULE E22-900MM22S**

Data

* 850MHz-930MHz operating frequency
* LoRa Mode - 0.018-62.5 kbps
* FSK mode up to 300 kbps
* FIFO: 256 byte data caching

Connection

* SPI
* 0-10 mbps

SPI pins

* NSS (Pin 14) -> Start SPI communication
* SCK (pin 15) -> SPI clock input
* MOSI (Pin 13) -> SPI data input
* MISO (Pin 12) -> SPI data output

Other Pins

* NRST Pin3 active low chip reset
* BUSY Pin11 Status indication output
* TXEN/RXEN Pins9/10 RF switch controls for transmit/receive

Power

* VCC 1.8-3.7V 3.3V optimal
* Communication level: 3.3V
* Current consumption: 119 mA emission current, 6.8 mA receiving current, 2 micro amp sleep current

Interrupts

* 3 pins, DIO1 (Pin 20), DIO2 (Pin 19), DIO3 (Pin 18)
* Many possible interrupt calls
