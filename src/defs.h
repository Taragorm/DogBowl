/*
 * defs.h
 *
 * Created: 25/05/2025 14:59:56
 *  Author: Simon
 */

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATtiny1614 / ARDUINO
//                          _____
//                  VDD   1|*    |14  GND
// (nSS)  (AIN4) PA4  0~  2|     |13  10~ PA3 (AIN3)(SCK)(EXTCLK)
//        (AIN5) PA5  1~  3|     |12  9   PA2 (AIN2)(MISO)
// (DAC)  (AIN6) PA6  2   4|     |11  8   PA1 (AIN1)(MOSI)
//        (AIN7) PA7  3   5|     |10  11  PA0 (nRESET/UPDI)
// (RXD) (TOSC1) PB3  4   6|     |9   7~  PB0 (AIN11)(SCL)
// (TXD) (TOSC2) PB2  5~  7|_____|8   6~  PB1 (AIN10)(SDA)
//                    ^^              ^^
//                     |--------------|----- These

/*
  PIN#   DESC         Pin Name  Other/Sp  ADC0      ADC1      PTC       AC0       AC1       AC2       DAC0      USART0    SPI0      TWI0      TCA(PWM)  TCBn      TCD0      CCL
  0      A0 or SS     PA4                 AIN4      AIN0      X0/Y0                                             XDIR      SS                  WO4                 WOA       LUT0-OUT
  1      A1           PA5       VREFA     AIN5      AIN1      X1/Y1     OUT       AINN0                                                       WO5       TCB0 WO   WOB
  2      A2 or DAC    PA6                 AIN6      AIN2      X2/Y2     AINN0     AINP1     AINP0     OUT
  3      A3           PA7                 AIN7      AIN3      X3/Y3     AINP0     AINP0     AINN0                                                                           LUT1-OUT
  4      RX           PB3       TOSC1                                             OUT                           RxD                           *WO0
  5      TX           PB2       TOSC2 /                                                     OUT                 TxD                           WO2
                                EVOUT1
  6      SDA          PB1                 AIN10               X4/Y4     AINP2                                   XCK                 SDA       WO1
  7      SCL          PB0                 AIN11               X5/Y5               AINP2     AINP1               XDIR                SCL       WO0
  8      MOSI         PA1                 AIN1                                                                  *TxD      MOSI      *SDA                                    LUT0-IN1
  9      MISO         PA2       EVOUT0    AIN2                                                                  *RxD      MISO      *SCL                                    LUT0-IN2
  10     SCK          PA3       EXTCLK    AIN3                                                                  *XCK      SCK                 WO3       TCB1 WO
  11     UPDI         PA0       RESET/    AIN0                                                                                                                              LUT1-IN0
                              UPDI
    alternative pin locations
*/

#ifndef DEFS_H_
#define DEFS_H_
#include <Arduino.h>
#include <stdint.h>

namespace Pins
{
    // PA0 is UDPI
    const uint8_t R             = PIN_PA1;
    const uint8_t B             = PIN_PA2;
    const uint8_t G             = PIN_PA3;
    const uint8_t TARE          = PIN_PA4;
    const uint8_t VBATT         = PIN_PA5;
    const uint8_t SOUND_EN      = PIN_PA6;
    const uint8_t SOUND         = PIN_PA7;
    const uint8_t MX711_DOUT    = PIN_PB0;
    const uint8_t MX711_SCK     = PIN_PB1;
    const uint8_t TX            = PIN_PB2;
    const uint8_t RX            = PIN_PB3;
}


#endif /* DEFS_H_ */