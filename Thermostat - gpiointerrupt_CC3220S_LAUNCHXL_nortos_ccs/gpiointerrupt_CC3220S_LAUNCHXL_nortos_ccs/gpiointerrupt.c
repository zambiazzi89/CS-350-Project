/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

/*
 * ========== UART Driver ==========
 */

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

/*
 * ========== I2C Driver ==========
 */

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
    DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
    DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

/*
 * ========== Timer Driver ==========
 */

// Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

// Temperature variables
int16_t setpoint;
int16_t temperature;
int8_t heat;

/*
 * ========== State Machine's States ==========
 */

enum BF_STATES {
    BF_Init,
    BF_Low,     // Waits for flag to be raised
    BF_Raised,  // Increases and/or decreases setpoint
} BF_STATE;

/*
 *  Button press variables
 *  Is set to 1 by the button callbacks if a button was pressed
 *  Button 0 Decreases setpoint
 *  Button 1 Increases setpoint
 */
unsigned char Button0_Pressed;
unsigned char Button1_Pressed;

void Tick_ButtonFlags() {
    switch(BF_STATE) { // Transitions
        case BF_Init:
            /* Initialize buttons to 0 (not pressed) */
            Button0_Pressed = 0;
            Button1_Pressed = 0;
            BF_STATE = BF_Low;
            break;
        case BF_Low:
            // Waits for button to be pressed
            if (Button0_Pressed || Button1_Pressed) {
                BF_STATE = BF_Raised;
            }
            break;
        case BF_Raised:
            // If no button flag is raised go to BF_Low
            if (!(Button0_Pressed || Button1_Pressed)){
                BF_STATE = BF_Low;
            }
            break;
        default:
            BF_STATE = BF_Init;
            break;
    }

    switch(BF_STATE) { // State actions
        case BF_Init:
            break;
        case BF_Low:
            break;
        case BF_Raised:
            // Updates setpoint value
            if (Button0_Pressed) {
                setpoint -= 1;          // Decreases setpoint by 1° C
                Button0_Pressed = 0;    // Lowers Button 0 flag
            }
            if (Button1_Pressed) {
                setpoint += 1;          // Increases setpoint by 1° C
                Button1_Pressed = 0;    // Lowers Button 1 flag
            }
            break;
        default:
            break;
    }
}


enum TL_STATES {
    TL_Init,
    TL_Off,
    TL_On,
} TL_STATE;

void Tick_TempLED() {
    switch(TL_STATE) { // Transitions
        case TL_Init:
            heat = 0;
            TL_STATE = TL_Off;
            break;
        case TL_Off:
            // If temperature is lower than setpoint goes to TL_On
            if (temperature < setpoint) {
                TL_STATE = TL_On;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                heat = 1;
            }
            break;
        case TL_On:
            // If temperature is not lower than setpoint goes to TL_Off
            if (!(temperature < setpoint)){
                TL_STATE = TL_Off;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                heat = 0;
            }
            break;
        default:
            TL_STATE = TL_Init;
            break;
    }

    switch(TL_STATE) { // State actions
        case TL_Init:
            break;
        case TL_Off:
            break;
        case TL_On:
            break;
        default:
            break;
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Tell program that button 0 was pressed */
    Button0_Pressed = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Tell program that button 1 was pressed */
    Button1_Pressed = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    initUART();
    initI2C();
    initTimer();
    GPIO_init();

    // Set period to 100,000 us (100 ms)
    const unsigned int timerPeriod = 100000;
    Timer_setPeriod(timer0, Timer_PERIOD_US, timerPeriod);

    // Button check period: 200,000 us (200 ms)
    unsigned long BT_elapsedTime = 200000;

    // Temperature check and LED period: 500,000 us (500 ms)
    unsigned long TL_elapsedTime = 500000;

    // UART Output: 1,000,000 us (1,000 ms = 1 second)
    unsigned long Output_elapsedTime = 1000000;

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    /* Turn LED off */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Initialize temperature and setpoint as current temperature */
    temperature = readTemp();
    setpoint = readTemp();

    /* Set initial state */
    BF_STATE = BF_Init;
    TL_STATE = TL_Init;

    /* Define and initialize variable seconds to 0 */
    int32_t seconds = 0;

    while(1){
        // Every  200ms: Check button flags
        if (BT_elapsedTime >= 200000) {
            Tick_ButtonFlags();
            BT_elapsedTime = 0;
        }

        // Every  500ms: Read the temperature and update the LED
        if (TL_elapsedTime >= 500000) {
            temperature = readTemp();
            Tick_TempLED();
            TL_elapsedTime = 0;
        }

        /*
         * Every 1000ms: Output to the UART formatted as <AA,BB,S,CCCC> broken down as follows:
         * AA = ASCII decimal value of room temperature (00 - 99) degrees Celsius
         * BB = ASCII decimal value of setpoint temperature (00-99) degrees Celsius
         * S = ‘0’ if heat is off, ‘1’ if heat is on
         * CCCC = decimal count of seconds since board has been reset
         * <%02d,%02d,%d,%04d> = temperature, setpoint, heat, seconds
         */
        if (Output_elapsedTime >= 1000000) {
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))
            seconds += 1;
            Output_elapsedTime = 0;
        }

        // Wait for timer period
        while(!TimerFlag){}

        // Lower flag
        TimerFlag = 0;

        // Increment elapsed time by period
        BT_elapsedTime += timerPeriod;
        TL_elapsedTime += timerPeriod;
        Output_elapsedTime += timerPeriod;
    }
}
