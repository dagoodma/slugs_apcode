/*
The MIT License

Copyright (c) 2015 UCSC Autonomous Systems Lab

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

#include "apDefinitions.h"
#include "ipcScheduler.h"
#include "pressure.h"

volatile unsigned char i2c1State;		// State Machine State

bool pressureInitialized; // do we need this?

tShortToChar currentPressure;		// placeholder for current pressure reading
tShortToChar currentTemperature;	// placeholder for current temp reading

void pressureDebugUartConfig(void){
	// Configure and open the port;
	// U2MODE Register
	// ==============
	U2MODEbits.ABAUD	= 0;		// Disable autobaud
	U2MODEbits.PDSEL	= 0;		// No parity 8 bit
	U2MODEbits.STSEL	= 0;		// 1 stop bit
	U2MODEbits.BRGH 	= 0;		// Low speed mode
	
	// U2STA Register
	// ==============
	U2STAbits.UTXISEL0	= 0;		// generate interrupt on every char
	U2STAbits.UTXISEL1	= 0;		// for the DMA
	U2STAbits.URXISEL	= 0;		// RX interrupt with every char
	U2STAbits.OERR		= 0;		// clear overun error

	// U2BRG Register
	// ==============
	U2BRG = RADIO_BAUDRATE_GENERATOR;				// Set the baud rate for data logger

	// Initialize the Interrupt  
  	// ========================
	IPC7bits.U2RXIP   = 5;    		// Interrupt priority 6  
  	IFS1bits.U2RXIF   = 0;    		// Clear the interrupt flag
  	IEC1bits.U2RXIE   = 0;    		// Disable interrupts
  		
	// Enable the port;
	U2MODEbits.UARTEN	= 1;		// Enable the port	
	U2STAbits.UTXEN		= 1;		// Enable TX
		
	IEC4bits.U2EIE 		= 0;	
}

void pressureInit (void){
        //pressureDebugUartConfig();
	// Configure the I2C bus
	I2C1CONbits.A10M = 0;		// 7 bit address
	I2C1BRG =	363;			// I2CBRG = 363. 100Khz for a 40 Mhz clock
	
	// Configure the Interrupts
	IFS1bits.MI2C1IF = 0;		// Clear the master interrupt
	IFS1bits.SI2C1IF = 0;		// Clear the slave interrupt

	IEC1bits.MI2C1IE = 1;		// Enable the interrupts
	IPC4bits.MI2C1IP = 7;		// Highest Prority
	
	// turn on the I2C Module
	I2C1CONbits.I2CEN = 1;
	
	// Initialize the state machine
	i2c1State = READ_IDLE;
		
	// Wait ~10 mS for device power on reset
        LED_SENS_BUSY_SET(ON); // busy light on for init
	dummyDelay();

        // Start reading pressure
        printToUart2("Starting Pressure I2C... ");
        pressureInitialized = false;
        i2c1Start();

        while (i2c1State != READ_IDLE) {
            // wait for initialization
        }
        printToUart2("Initialized.\r\n");
        pressureInitialized = true;
        LED_SENS_BUSY_SET(OFF);

        printToUart2("Got first data Press=x%X,Temp=x%X\n",
                currentPressure.shData, currentTemperature.shData);

        // Start the next read REMOVE
        //startPressureRead();
}

void startPressureRead (void) {
    if (pressureInitialized && i2c1State == READ_IDLE) {
        i2c1Start();
        LED_SENS_STATUS_SET(ON); // status light goes on when started, off when done
    }
}

void saveData (void){
    mlRawPressureData.press_diff1 = currentPressure.shData;
    // Temperature is only 11 bits, so shift the last 5 bits off.
    mlRawPressureData.temperature = (currentTemperature.shData) >> 5;
}

void getPressure (int16_t* pressureVals){
    //printToUart2("Read pressure!\n\r");
    
    pressureVals[0] =  mlRawPressureData.press_diff1;
    pressureVals[1] =  mlRawPressureData.temperature;
	
    startPressureRead();
}

// I2C Primitives
// ==============

void i2c1Start(void){
	i2c1State = READ_START;
	// Start the bus (set start enable bit)
        // This will pull the SDA line low (clock is high), then start the clock
	I2C1CONbits.SEN =1;  
}

void i2c1Stop(void){
	// Change the mode
	i2c1State = READ_STOP;
	// Stop the bus
	I2C1CONbits.PEN =1;  
}

void i2c1Write(unsigned char byte2write){
	// Send the data
	I2C1TRN = byte2write;
}

// Interrupt Service Routine (State Machine Implementation)
// =========================

void __attribute__((__interrupt__, no_auto_psv)) _MI2C1Interrupt(void){
    switch (i2c1State) {
        case READ_START:
            if (!I2C1CONbits.SEN) {
                // Start enable bit is automatically cleared on completion.
                // Now write the 8-bit read address to the bus
                i2c1Write(PRESSURE_READ);
                i2c1State = READ_WAIT;
                //printToUart2("here1\n\r");
            }
            break;
        case READ_WAIT:
            if (!I2C1STATbits.ACKSTAT) {
                    I2C1CONbits.RCEN = 1;
                    i2c1State = READ_BRDATA_HI;
            }
            break;
        case READ_BRDATA_HI:
            if (I2C1STATbits.RBF) {
                //printToUart2("...");
                // Read Bridge Data [13:8], bits 6 and 7 are status
                uint8_t readByte = (uint8_t)I2C1RCV;
                // TODO handle statusValue
                uint8_t statusValue = (readByte & 0xC0) >> 6;
                readByte &= 0x3F;
                currentPressure.chData[1] = readByte;


                //printToUart2("Got i2c BRHI=0x%X, STAT=0x%X\n",readByte,statusValue);

                // Generate an ack
                I2C1CONbits.ACKDT = 0;
                I2C1CONbits.ACKEN = 1;
            }
            else if (!I2C1CONbits.ACKEN) {
                I2C1CONbits.RCEN = 1;
                i2c1State = READ_BRDATA_LO;
            }
            break;
        case READ_BRDATA_LO:
            if (I2C1STATbits.RBF) {
                //printToUart2("...");
                // Read Bridge Data [7:0]
                uint8_t readByte = (uint8_t)I2C1RCV;
                currentPressure.chData[0] = readByte;

                //printToUart2("Got i2c BRLO=0x%X\n",readByte);

                // Generate an ack
                I2C1CONbits.ACKDT = 0;
                I2C1CONbits.ACKEN = 1;
            }
            else if (!I2C1CONbits.ACKEN) {
                I2C1CONbits.RCEN = 1;
                i2c1State = READ_TEMPDATA_HI;
            }
            break;
        case READ_TEMPDATA_HI:
            if (I2C1STATbits.RBF) {
                //printToUart2("...");
                // Read temperature data [10:3]
                uint8_t readByte = (uint8_t)I2C1RCV;
                currentTemperature.chData[1] = readByte;

                // Generate an ack
                I2C1CONbits.ACKDT = 0;
                I2C1CONbits.ACKEN = 1;
            }
            else if (!I2C1CONbits.ACKEN) {
                I2C1CONbits.RCEN = 1;
                i2c1State = READ_TEMPDATA_LO;
            }
            break;
        case READ_TEMPDATA_LO:
            if (I2C1STATbits.RBF) {
                // Read temperature data [2:0], last 5 bits are undetermined
                uint8_t readByte = (uint8_t)I2C1RCV;
                currentTemperature.chData[0] = readByte;
                saveData(); // shifts last 5 bits of temp off

                // Generate an nack
                I2C1CONbits.ACKDT = 1;
                I2C1CONbits.ACKEN = 1;

                i2c1State = READ_DONE;

                // DEBUG
                //printToUart2("P=x%X,T=x%X\n",
                //    currentPressure.shData, currentTemperature.shData);
            }
            break;
        case READ_DONE:
            if (!I2C1CONbits.ACKEN) {
                i2c1Stop();
            }
            break;
        case READ_STOP:
            if (!I2C1CONbits.PEN){
                i2c1State = READ_IDLE;
                LED_SENS_STATUS_SET(OFF);
            }
            break;
        default:
            break;
    }
    IFS1bits.MI2C1IF = 0;		// Clear the master interrupt
    I2C1STATbits.IWCOL = 0;		// Clear the collision flag just in case
}

void dummyDelay (void) {
    unsigned int i, j;
    // Put some huge delays to wait for pressure sensor power-up
    // without the need of a timer
    // 5 milliseconds are expected for power-up
    // @ 40Mhz requires aprox 200,000 nops
    for( i = 0; i < 40; i += 1 ){
            for( j = 0; j < 32700; j += 1 )
            {
                    Nop();
            }
    }
}

// TODO consolodate all of these types of functions
/*
void printToUart2 (const char *fmt, ...){
	va_list ap;
	char buf [300];

	va_start(ap, fmt);
	vsprintf(buf, fmt, ap);
	va_end (ap);
	putsUART2((unsigned int*)buf);
         while (BusyUART2());
}
 * */

