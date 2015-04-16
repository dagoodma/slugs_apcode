/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

 */

// ==============================================================
// ipcScheduler.c
// This is code implements a fully DMA driven UART writer to
// be used in the UCSC Autopilot project. It makes use of the 
// circular buffer data structure circBuffer.c. It has been 
// written to be implemented in Simulink. It configures UART 2
// at a predefined baud rate, then initializes a circular buffer,
// configures the DMA and starts the service. 
// The main function logData writes data to UART2 in the predefined
// comm protocol and returns data in the comm protocol to be sent
// via SPI to the second MCU. 
// 
// This file also contains Sensor MCU support functions
//
// Code by: Mariano I. Lizarraga
// First Revision: Aug 26 2008 @ 21:15
// =========================================================
#include <stdio.h>
#include <stdbool.h>
#include "ipcScheduler.h"


struct CircBuffer com2Buffer;
CBRef logBuffer;

struct CircBuffer com2BufferIn;
CBRef uartBufferIn;

unsigned int BufferA[MAXSEND] __attribute__((space(dma))) = {0};

void schedulerInit(void) {
    // initialize the circular buffers
    logBuffer = (struct CircBuffer*) &com2Buffer;
    newCircBuffer(logBuffer);

    uartBufferIn = (struct CircBuffer*) &com2BufferIn;
    newCircBuffer(uartBufferIn);

    // DMA0REQ Register
    // ================
    DMA0REQ = 0x001F;

    // DMA0PAD Register
    // ================
    DMA0PAD = (volatile unsigned int) &U2TXREG;

    // DMA0CON Register
    // ================
    DMA0CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA0CONbits.MODE = 1; // One-shot, No Ping-Pong Mode	
    DMA0CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA0CONbits.SIZE = 0; // Word Data Transfer

    // DMA0CNT Register
    // ==============
    DMA0CNT = MAXSEND - 1;

    // DMA0STA Register
    // ================
    DMA0STA = __builtin_dmaoffset(BufferA);

    // Enable DMA0 TX interrupts
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt

    // Configure and open the port;
    // U2MODE Register
    // ==============
    U2MODEbits.ABAUD = 0; // Disable autobaud
    U2MODEbits.PDSEL = 0; // No parity 8 bit
    U2MODEbits.STSEL = 0; // 1 stop bit
    U2MODEbits.BRGH = 0; // Low speed mode

    // U2STA Register
    // ==============
    U2STAbits.UTXISEL0 = 0; // generate interrupt on every char
    U2STAbits.UTXISEL1 = 0; // for the DMA
    U2STAbits.URXISEL = 0; // RX interrupt with every char
    U2STAbits.OERR = 0; // clear overun error

    // U2BRG Register
    // ==============
    U2BRG = RADIO_BAUDRATE_GENERATOR; // Set the baud rate for data logger

    // Initialize the Interrupt  
    // ========================
    IPC7bits.U2RXIP = 5; // Interrupt priority 5  
    IFS1bits.U2RXIF = 0; // Clear the interrupt flag
    IEC1bits.U2RXIE = 1; // Enable interrupts

    // Enable the port;
    U2MODEbits.UARTEN = 1; // Enable the port	
    U2STAbits.UTXEN = 1; // Enable TX

    IEC4bits.U2EIE = 0;
}

void copyBufferToDMA(unsigned char size) {
    unsigned char i;
    for (i = 0; i < size; i += 1) {
        BufferA[i] = (unsigned int) readFront(logBuffer);
    }
}

/*
    logData is the implementation of the outgoing Comunications 		
    protocol it is geared to prepare the data for logging and
    for the SPI inter processor communication. It uses a 
    static variable to monitor at which samples the outgoing messages 
    are queued in the circular buffer for DMA transmision.
 */

/*
    TODO: This function needs to be merged with prepareTelemetry
    in groundStationDriver.c and moved to something like
    apUtils.c
	
 */

void scheduleData(unsigned char hilOn, unsigned char* dataOut) {
    // HIL mode enabled message
    static bool sentHILMessage = false;

    // Buffer to send data to HIL Sim
    uint8_t hilBuf[MAXSEND];

    // Cycles from 1 to 10 to decide which message's turn is to be sent (round robin)
    static uint8_t samplePeriod = 1;

    // Contains the total bytes to send via the serial port
    uint8_t bytes2Send = 0;

    switch (samplePeriod) {
        case 1: // Servo output (HIL only), Sensor diagnostic
            // HIL mode (should be: hilOn == 1)
            if (PORTDbits.RD2 == 1) {
                // == Servo Output ==
                hilBuf[0] = _prepareServoOutputMavlink(SLUGS_SENSOR_COMPID, SLUGS_HIL_CHANNEL, hilBuf + 1);
                send2DebugPort(hilBuf, 1);

                // == Status text ==
                if (!sentHILMessage) {
                    sprintf(mlStatustext.text,"Sensor DSC in HIL Mode.");
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    bytes2Send += _prepareStatusTextMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);
                    sentHILMessage = true;
                }
            }
            else if (sentHILMessage) { sentHILMessage = false; } // clear flag if set from last time

            // == Sensor Diagnostic ==
            bytes2Send += _prepareSensorDiagnosticMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;
        case 2: // Cpu Load, GPS Raw
            // == Cpu Load ==
            bytes2Send += _prepareCpuLoadMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            // == GPS Raw ==
            bytes2Send += _prepareGpsMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;
        case 3: // Raw IMU
            // == Raw IMU ==
            bytes2Send += _prepareRawImuMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);
            
            break;
        case 4: // Reboot (if required)		
            // == Reboot ==
            if (mlBoot.version == 1) {
                bytes2Send += _prepareBootMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);
                mlBoot.version = 0;
            }

            break;

        case 5: // Sensor bias
            // == Sensor bias ==
            bytes2Send += _prepareSensorBiasMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;
        case 6: // GPS Status
            // == GPS Status ==
            bytes2Send += _prepareGpsStatusMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;

        case 7: // Pilot Console Data
            // == RC Channels Raw ==
            bytes2Send += _prepareRcChannelsMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;

        case 8: // Sensor Data in meaningful units
            // == Scaled IMU ==
            bytes2Send += _prepareScaledImuMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;

        case 9: // GPS Origin Info, Raw Pressure
            // == Status text ==
            if (sendGpsOriginMessage) {
                sprintf(mlStatustext.text,"Sensor origin: %.2f %.2f %.1f",
                    mlGSLocationFloat.lat, mlGSLocationFloat.lon, mlGSLocationFloat.alt);
                mlStatustext.severity = MAV_SEVERITY_INFO;
                bytes2Send += _prepareStatusTextMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);
                sendGpsOriginMessage = false;
            }
            
            // == Raw Pressure ==
            bytes2Send += _prepareRawPressureMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;

        case 10: // Command ack, GPS date time
            // == Command Ack ==
            if (sendCommandAcknowledgement) {
                bytes2Send += _prepareCommandAckMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);
                sendCommandAcknowledgement = false;
            }

            // == GPS Date & Time ==
            bytes2Send += _prepareGpsTimeMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

            break;
        default:
            *dataOut = 0;

            break;
    }

    // == Scaled Pressure ==
    bytes2Send += _prepareScaledPressureMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);
    
    // == Attitude ==
    bytes2Send += _prepareAttitudeMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);

    // == Local Position NED ==
    bytes2Send += _prepareLocalPositionMavlink(SLUGS_SENSOR_COMPID, SLUGS_SPI_CHANNEL, dataOut + 1 + bytes2Send);


    // Put the length of the message in the first byte of the outgoing array
    *dataOut = bytes2Send;

    // increment/overflow the samplePeriod counter
    // configured for 10 Hz in non vital messages
    samplePeriod = (samplePeriod >= 10) ? 1 : samplePeriod + 1;

    //send2DebugPort(dataOut,1);

}

void send2DebugPort(unsigned char* protData, unsigned char hilOn) {
    unsigned int bufLen, i;

    // add the data to the circular buffer
    for (i = 1; i <= protData[0]; i += 1) {
        writeBack(logBuffer, protData[i]);
    }

    // get the Length of the logBuffer
    bufLen = getLength(logBuffer);

    // if the interrupt catched up with the circularBuffer
    //  then turn on the DMA
    if (!(DMA0CONbits.CHEN) && bufLen > 0) {
        // Configure the bytes to send
        DMA0CNT = bufLen <= (MAXSEND - 1) ? bufLen - 1 : MAXSEND - 1;
        // copy the buffer to the DMA channel outgoing buffer	
        copyBufferToDMA((unsigned char) DMA0CNT + 1);
        // Enable the DMA
        DMA0CONbits.CHEN = 1;
        // Init the transmission
        DMA0REQbits.FORCE = 1;
    }
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {

    // Clear the DMA0 Interrupt Flag;
    IFS0bits.DMA0IF = 0;

}

// Interrupt service routine for U2 HIL protocol port

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {

    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U2STAbits.URXDA == 1) {
        writeBack(uartBufferIn, (unsigned char) U2RXREG);
    }

    // If there was an overun error clear it and continue
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
    IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
}






