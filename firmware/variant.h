/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#pragma once

#ifndef _VARIANT_OPENTRACKER_
#define _VARIANT_OPENTRACKER_

#ifndef _VARIANT_ARDUINO_DUE_X_
#define _VARIANT_ARDUINO_DUE_X_ // for compatibility (required?)
#endif

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			84000000

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (41u)
#define NUM_DIGITAL_PINS     (41u)
#define NUM_ANALOG_INPUTS    (5u)

#define digitalPinToPort(P)        ( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->PIO_ODSR) )
#define portInputRegister(port)    ( &(port->PIO_PDSR) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * portModeRegister(..) should return a register to set pin mode
 * INPUT or OUTPUT by setting the corresponding bit to 0 or 1.
 * Unfortunately on SAM architecture the PIO_OSR register is
 * read-only and can be set only through the enable/disable registers
 * pair PIO_OER/PIO_ODR.
 */
// #define portModeRegister(port)   ( &(port->PIO_OSR) )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAM
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(p)  ((p) < NUM_DIGITAL_PINS ? (p) : -1)

// LEDs
#define PIN_LED_13           (13u)
#define PIN_LED              PIN_LED_13
#define PIN_POWER_LED          13
#define LED_BUILTIN          13

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE        SPI0
#define SPI_INTERFACE_ID     ID_SPI0
#define SPI_CHANNELS_NUM 3
#define PIN_SPI_SS0          (37u)
#define PIN_SPI_SS1          (47u)
#define PIN_SPI_SS2          (46u)
#define PIN_SPI_SS3          (38u)
#define PIN_SPI_MOSI         (35u)
#define PIN_SPI_MISO         (34u)
#define PIN_SPI_SCK          (36u)
#define BOARD_SPI_SS0        PIN_SPI_SS0
#define BOARD_SPI_SS1        PIN_SPI_SS1
#define BOARD_SPI_SS2        PIN_SPI_SS2
#define BOARD_SPI_SS3        PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS0

#define BOARD_PIN_TO_SPI_PIN(x) \
	(x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : \
	(x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
  (x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3)))
#define BOARD_PIN_TO_SPI_CHANNEL(x) \
	(x==BOARD_SPI_SS0 ? 0 : \
	(x==BOARD_SPI_SS1 ? 1 : \
  (x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
static const uint8_t SS1  = BOARD_SPI_SS1;
static const uint8_t SS2  = BOARD_SPI_SS2;
static const uint8_t SS3  = BOARD_SPI_SS3;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (20u)
#define PIN_WIRE_SCL         (21u)
#define WIRE_INTERFACE       TWI1
#define WIRE_INTERFACE_ID    ID_TWI1
#define WIRE_ISR_HANDLER     TWI1_Handler
#define WIRE_ISR_ID          TWI1_IRQn

/*
 * UART/USART Interfaces
 */
// Serial
#define PINS_UART            (41u)
// Serial1
#define PINS_USART0          (42u)
// Serial2
#define PINS_USART1          (43u)
// Serial3
#define PINS_USART2          (44u)

/*
 * USB Interfaces
 */
#define PINS_USB             (45u)

/*
 * Analog pins
 */
static const uint8_t A2  = 31;
static const uint8_t A3  = 30;
static const uint8_t A5  = 28;
static const uint8_t A6  = 29;
static const uint8_t A8  = 49;
static const uint8_t DAC1 = 48;
static const uint8_t CANRX = 32;
static const uint8_t CANTX = 33;
#define ADC_RESOLUTION		12

// CAN0
#define PINS_CAN0            (50u)
// no enable pin (due_can compatibility)
#define CAN0_EN	255

/*
 * DACC
 */
#define DACC_INTERFACE		DACC
#define DACC_INTERFACE_ID	ID_DACC
#define DACC_RESOLUTION		12
#define DACC_ISR_HANDLER    DACC_Handler
#define DACC_ISR_ID         DACC_IRQn

/*
 * PWM
 */
#define PWM_INTERFACE		PWM
#define PWM_INTERFACE_ID	ID_PWM
#define PWM_FREQUENCY		1000
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0
#define PWM_RESOLUTION		8

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        1000
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION		8

#ifdef __cplusplus
}
#endif

/* OpenTracker PINS */

#define PIN_EXT_RX      0
#define PIN_EXT_TX      1
#define PIN_S_PPS_GPS   2
#define PIN_S_DETECT    3
#define PIN_C_KILL_GSM  5
#define PIN_EXT_PA21    6
#define PIN_EXT_PA20    7
#define PIN_S_INLEVEL   8
#define PIN_CAN_RS      9
#define PIN_C_OUT_1     10
#define PIN_C_OUT_2     11
#define PIN_STATUS_GSM  12
#define PIN_POWER_LED   13
#define PIN_EXT_TXD2    14
#define PIN_EXT_RXD2    15
#define PIN_TX1_GSM     16
#define PIN_RX1_GSM     17
#define PIN_TX0_GPS     18
#define PIN_RX0_GPS     19
#define PIN_EXT_SDA     20
#define PIN_EXT_SCL     21
#define PIN_WAKE_GSM    23
#define PIN_C_PWR_GSM   25
#define PIN_RESET_GPS   26
#define PIN_STANDBY_GPS 27
#define PIN_EXT_IN1     28
#define PIN_EXT_IN2     29
#define PIN_EXT_PA22    30
#define PIN_EXT_PA23    31
#define PIN_CANRX       32
#define PIN_CANTX       33
#define PIN_EXT_MISO    34
#define PIN_EXT_MOSI    35
#define PIN_EXT_SCK     36
#define PIN_EXT_NS0     37
#if !defined(OPENTRACKER_HW_REV) || (OPENTRACKER_HW_REV < 0x0240)
// HW Revision 2.3 (and earlier)
#define PIN_C_REBOOT    4
#define PIN_RING_GSM    22
#define PIN_GSM_VDD_EXT 24
#define ANALOG_VREF     3.40
#define MODEM_VBAT      4.13
#define MODEM_M95       1
#else
// HW Revision 2.4 (and later)
// dropped "REBOOT" pin and "GSM VDD" (unused)
// moved "RING" to wakeup capable pins
// added "INx_PD" pull-down enable pins, to change analog input range
#define PIN_RING_GSM    24
#define PIN_C_IN1_PD    38
#define PIN_C_IN2_PD    39
#define ANALOG_VREF     3.31
#if (OPENTRACKER_HW_REV == 0x024A)
#define MODEM_VBAT      4.10
#define MODEM_UG96      1
#else
#define MODEM_VBAT      4.36
#define MODEM_M95       1
#endif
#endif

#ifndef MODEM_UG96
#define MODEM_UG96      0
#endif
#ifndef MODEM_M95
#define MODEM_M95       0
#endif

#define AIN_S_INLEVEL   49
#define AIN_EXT_IN1     28
#define AIN_EXT_IN2     29
#define AIN_EXT_PA22    30
#define AIN_EXT_PA23    31

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern UARTClass Serial;
extern USARTClass Serial1;
extern USARTClass Serial2;
extern USARTClass Serial3;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial
#define SERIAL_PORT_HARDWARE_OPEN1  Serial3
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3

#endif /* _VARIANT_OPENTRACKER_ */

