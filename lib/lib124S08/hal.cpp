/**
 * @file hal.c
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

#include <Arduino.h>
#include <SPI.h>
#include "hal.h"
#include "ads124s08.h"

// SPI configuration
#define SPI_SPEED 2000000
#define SPI_WORD_SIZE 8



//****************************************************************************
//
// Internal variables
//
//****************************************************************************
// Flag to indicate if a /DRDY interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;

//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitSPI(void);
void GPIO_DRDY_IRQHandler(void);

//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************

/************************************************************************************//**
 * @brief getDRDYinterruptStatus()
 *          Gets the current status of nDRDY interrupt flag.
 *
 * @ return boolean status of flag_nDRDY_INTERRUPT
 */
bool getDRDYinterruptStatus(void)
{
   return flag_nDRDY_INTERRUPT;
}

/************************************************************************************//**
 * @brief setDRDYinterruptStatus()
 *          Sets the value of the nDRDY interrupt flag.
 *
 * @param[in] value where status is set with true; false clears the status.
 *
 * @return None
 */
void setDRDYinterruptStatus(const bool value)
{
    flag_nDRDY_INTERRUPT = value;
}
/************************************************************************************//**
 *
 * @brief enableDRDYinterrupt()
 *          Enables or disables the nDRDY interrupt.
 *
 * @param[in] intEnable Where interrupt is enabled with true; false disables the interrupt.
 *
 * @return None
 */
void enableDRDYinterrupt(const bool intEnable)
{
    
    if(intEnable)
    {
        flag_nDRDY_INTERRUPT = false;
        attachInterrupt(digitalPinToInterrupt(nDRDY_PIN), GPIO_DRDY_IRQHandler, FALLING);
    }
    else detachInterrupt(digitalPinToInterrupt(2));

}

//****************************************************************************
//
// Timing functions
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief delay_ms()
 *          Provides a timing delay with 'ms' resolution.
 *
 * @param[in] delay_time_ms Is the number of milliseconds to delay.
 *
 * @return none
 */
void delay_ms(const uint32_t delay_time_ms)
{
    delay(delay_time_ms);
}

/************************************************************************************//**
 *
 * @brief delay_us()
 *          Provides a timing delay with 'us' resolution.
 *
 * @param[in] delay_time_us Is the number of microseconds to delay.
 *
 * @return none
 */
void delay_us(const uint32_t delay_time_us)
{
    delayMicroseconds(delay_time_us);
}

//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief InitGPIO()
 *          Configures the MCU's GPIO pins that interface with the ADC.
 *
 * @return none
 *
 */
void InitGPIO(void)
{
    pinMode(nDRDY_PIN, INPUT);

    pinMode(nCS_PIN, OUTPUT);
    pinMode(nRESET_PIN, OUTPUT);
    pinMode(START_PIN, OUTPUT);
}

//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************

/************************************************************************************//**
 *
 * @brief GPIO_DRDY_IRQHandler()
 *          Interrupt handler for nDRDY falling edge interrupt.
 *
 * @param[in] index Position of the interrupt for callback.
 *
 * @return None
 */
void GPIO_DRDY_IRQHandler( void )
{
    /* --- INSERT YOUR CODE HERE --- */

    //NOTE: You many need to rename or register this interrupt function for your processor

    /* Interrupt action: Set a flag */
    flag_nDRDY_INTERRUPT = true;

}


//****************************************************************************
//
// SPI Communication
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief InitSPI()
 *          Configures the MCU's SPI peripheral, for interfacing with the ADC.
 *
 * @return None
 */
void InitSPI(void)
{
    SPI.begin();

    return;
}

/************************************************************************************//**
 *
 * @brief InitADCPeripherals()
 *          Initialize MCU peripherals and pins to interface with ADC
 *
 * @param[in]   *adcChars  ADC characteristics
 * @param[in]   *spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 *
 * @code
 *     if ( !InitADCPeripherals( &spiHdl ) ) {
 *        // Error initializing MCU SPI Interface
 *        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
 *        return( false );
 *     }
 *     // MCU initialized ADC successfully
 *     return( true );
 * @endcode
 */
bool InitADCPeripherals( void )
{
    bool status;
    SPI.begin();

    // Start up the ADC
    status = adcStartupRoutine();

    return( status );
}


/************************************************************************************//**
 *
 * @brief getRESET()
 *          Returns the state of the MCU's ADC_RESET GPIO pin
 *
 * @return boolean level of /RESET pin (false = low, true = high)
 */
bool getRESET( void )
{
    return (bool) digitalRead(nRESET_PIN);
}

/************************************************************************************//**
 *
 * @brief setRESET()
 *            Sets the state of the MCU ADC_RESET GPIO pin
 *
 * @param[in]   state   level of /RESET pin (false = low, true = high)
 *
 * @return      None
 */

void setRESET( bool state )
{
    digitalWrite( nRESET_PIN, (uint8_t) state );
    return;
}

/************************************************************************************//**
 *
 * @brief toggleRESET()
 *            Pulses the /RESET GPIO pin low
 *
 * @return      None
 */
void toggleRESET( void )
{
    digitalWrite( nRESET_PIN, LOW );

    // Minimum nRESET width: 4 tCLKs = 4 * 1/4.096MHz =
    delay_us( DELAY_4TCLK );

    digitalWrite( nRESET_PIN, HIGH );
    return;

}

/************************************************************************************//**
 *
 * @brief getSTART()
 *          Returns the state of the MCU's ADC_START GPIO pin
 *
 * @return boolean level of START pin (false = low, true = high)
 */
bool getSTART( void )
{
    return (bool) digitalRead( START_PIN );
}

/************************************************************************************//**
 *
 * @brief setSTART()
 *            Sets the state of the MCU START GPIO pin
 *
 * @param[in]   state   level of START pin (false = low, true = high)
 *
 * @return      None
 */
void setSTART( bool state )
{
    digitalWrite( START_PIN, (uint8_t) state );

    // Minimum START width: 4 tCLKs
    delay_us( DELAY_4TCLK );

    return;
}

/************************************************************************************//**
 *
 * @brief toggleSTART()
 *            Pulses the START GPIO pin low
 * param[in]    direction sets the toggle direction base on initial START pin configuration
 *
 * @return      None
 */
void toggleSTART( bool direction )
{
    if ( direction )
    {
        digitalWrite( START_PIN, LOW );

        // Minimum START width: 4 tCLKs
        delay_us( DELAY_4TCLK );

        digitalWrite( START_PIN, HIGH );
    }
    else
    {
        digitalWrite( START_PIN, HIGH );

        // Minimum START width: 4 tCLKs
        delay_us( DELAY_4TCLK );

        digitalWrite( START_PIN, LOW );
    }
    return;
}

/************************************************************************************//**
 *
 * @brief sendSTART()
 *            Sends START Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendSTART( void )
{
    uint8_t dataTx = OPCODE_START;

    // Send START Command
    sendCommand( dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendSTOP()
 *            Sends STOP Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendSTOP( void )
{
    uint8_t dataTx = OPCODE_STOP;

    // Send STOP Command
    sendCommand( dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendRESET()
 *            Sends RESET Command through SPI, then waits 4096 tCLKs
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendRESET( void )
{
    uint8_t dataTx = OPCODE_RESET;

    // Send RESET command
    sendCommand( dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendWakeup()
 *            Sends WAKEUP command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendWakeup( void )
{
    uint8_t dataTx = OPCODE_WAKEUP;

    // Wakeup device
    sendCommand( dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendPowerdown()
 *            Sends POWERDOWN command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendPowerdown( void )
{
    uint8_t dataTx = OPCODE_POWERDOWN;

    // Power down device
    sendCommand( dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief setCS()
 *            Sets the state of the "/CS" GPIO pin
 *
 * @param[in]   level   Sets the state of the "/CS" pin
 *
 * @return      None
 */
void setCS( bool state )
{
    digitalWrite( nCS_PIN, (uint8_t) state );
    return;
}

/************************************************************************************//**
 *
 * @brief getCS()
 *          Returns the state of the MCU's ADC_CS GPIO pin
 *
 * @return boolean level of CS pin (false = low, true = high)
 */
bool getCS( void )
{
    return (bool) digitalRead( nCS_PIN );
}

/************************************************************************************//**
 *
 * @brief waitForDRDYHtoL()
 *            Waits for a nDRDY GPIO to go from High to Low or until a timeout condition occurs
 *            The DRDY output line is used as a status signal to indicate
 *            when a conversion has been completed. DRDY goes low
 *            when new data is available.
 *
 * @param[in]   timeout_ms number of milliseconds to allow until a timeout
 *
 * @return      Returns true if nDRDY interrupt occurred before the timeout
 *
 * @code
 *      // Read next conversion result
 *      if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
 *          adcValue = readConvertedData( spiHdl, &status, COMMAND );
 *      } else {
 *          // Error reading conversion result
 *          Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
 *          return( false );
 *      }
 * @endcode
 */
bool waitForDRDYHtoL( uint32_t timeout_ms )
{
    uint32_t timeoutCounter = timeout_ms * 8000;   // convert to # of loop iterations;

    do {
    } while ( !(flag_nDRDY_INTERRUPT) && (--timeoutCounter) );

    if ( !timeoutCounter ) {
        return false;
    } else {
        flag_nDRDY_INTERRUPT = false; // Reset flag
        return true;
    }
}

/************************************************************************************//**
 *
 * @brief spiSendReceiveArrays()
 *             Sends SPI commands to ADC and returns a response in array format
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]   *DataTx     array of SPI data to send on MOSI pin
 * @param[in]   *DataRx     array of SPI data that will be received from MISO pin
 * @param[in]   byteLength  number of bytes to send/receive on the SPI
 *
 * @return     None
 */
void spiSendReceiveArrays( uint8_t DataTx[], uint8_t *DataRx, uint8_t byteLength )
{
    /*
     *  This function sends and receives multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     *
     */

    
    setCS( LOW );

    /* Send or Receive Data */
    if ( byteLength > 0 ) {
        SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
        for (uint8_t i = 0; i < byteLength; i++)
        {
            DataRx[i] = SPI.transfer(DataTx[i]);
        }
        SPI.endTransaction(); 
    }

   setCS( HIGH );
   return;
}


/************************************************************************************//**
 *
 * @brief spiSendReceiveByte()
 *             Sends a single byte to ADC and returns a response
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]   dataTx      byte to send on DIN pin
 *
 * @return     SPI response byte
 */
uint8_t spiSendReceiveByte( uint8_t dataTx )
{
    uint8_t         dataRx = 0;

    setCS( LOW );

    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
    SPI.transfer(dataTx);
    SPI.endTransaction();

    setCS( HIGH );

    return( dataRx );
}
