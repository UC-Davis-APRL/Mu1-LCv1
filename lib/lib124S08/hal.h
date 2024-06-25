/**
 * @file hal.h
 *
 * @brief Hardware abstraction layer for Teensy 4.1 with Arduino
 *
 * Original code source from TI
 * @copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef HAL_H_
#define HAL_H_


//****************************************************************************
//
// Standard libraries
//
//****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "ads124s08.h"


//****************************************************************************
//
// Insert processor specific header file(s) here
//
//****************************************************************************

/*  --- INSERT YOUR CODE HERE --- */

//*****************************************************************************
//
// Pin definitions (Teensy 4.1 Test)
//
//*****************************************************************************
// #define nDRDY_PORT          (GPIO_PORTK_BASE)
#define nDRDY_PIN           9
// #define nDRDY_INT           (INT_GPIOK)
// #define nCS_PORT            (GPIO_PORTQ_BASE)
#define nCS_PIN             10
// #define nRESET_PORT         (GPIO_PORTM_BASE)
#define nRESET_PIN          8
// #define START_PORT          (GPIO_PORTH_BASE)
#define START_PIN           7

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
void delay_ms(const uint32_t delay_time_ms);
void delay_us(const uint32_t delay_time_us);
bool InitADCPeripherals( void );
void InitGPIO( void );
void InitSPI( void );
void setSTART (bool state );
bool getSTART( void );
void setCS( bool state );
bool getCS( void );
void setRESET( bool state );
bool getRESET( void );
void toggleSTART( bool direction );
void toggleRESET( void );
void sendSTART( void );
void sendSTOP( void );
void sendRESET( void );
void sendPowerDown( void );
void sendWakeup( void );
void spiSendReceiveArrays( uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength );
uint8_t spiSendReceiveByte( uint8_t dataTx );
bool waitForDRDYHtoL( uint32_t timeout_ms );
bool getDRDYinterruptStatus(void);
void setDRDYinterruptStatus(const bool value);
void enableDRDYinterrupt(const bool intEnable);

//*****************************************************************************
//
// Macros
//
//*****************************************************************************



#endif /* HAL_H_ */
