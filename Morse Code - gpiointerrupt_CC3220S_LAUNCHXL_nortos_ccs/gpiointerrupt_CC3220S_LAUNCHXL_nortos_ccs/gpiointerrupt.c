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

/* Driver configuration */
#include "ti_drivers_config.h"

volatile unsigned char TimerFlag = 0;

/*
 *  ======== timerCallback ========
 *  Callback function for the timer
 *  Whenever it is called, it raises the timer flag,
 *  which ends the while loop in the main thread.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    /* Raise timer flag */
    TimerFlag = 1;
}

/*
 *  ======== initTimer ========
 *  Initializes the timer and sets parameters.
 */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000; // 500 ms = 500000 us
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
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

/* State Machine's States */
enum SM_STATES {
    SM_Start,   /* Sets i to 0 and goes to first S                                                      */
    SM_S1,      /* First S LEDs stay off for 1500ms at the end                                          */
    SM_O,       /* Letter O LEDs stay off for 1500ms at the end, the it goes to S2 or K                 */
    SM_S2,      /* Second S does turn LEDs off at the end, instead it goes to SM_Pause                  */
    SM_K,       /* Letter K also does not turn the LEDs off at the end and goes to SM_Pause             */
    SM_Pause    /* At the end of the pause, SM_Pause checks if a button was pressed
                   If so, it switches the morse word and then it goes to either S1 if SOS or O if OK    */
} SM_STATE;

/*
 *  ButtonClick variable
 *  Is set to 1 by the button callback if a button was pressed
 */
unsigned char ButtonClick = 0;

/*
 *  Morse words:
 *  0: SOS
 *  1: OK
 */
unsigned char MorseWord = 0;


void Tick_Morse() {
    unsigned char i;
    switch(SM_STATE) { // Transitions
        case SM_Start:
            i = 0;
            SM_STATE = SM_S1;
            break;
        case SM_S1:
            /* Changes state after 8 iterations */
            if (!(i < 8)) {
                i = 0;
                SM_STATE = SM_O;
            }
            break;
        case SM_O:
            /* Changes state after 14 iterations */
            if (!(i < 14)) {
                i = 0;
                /* Goes to K if MorseWord is 1, else goes to S2 */
                if (MorseWord) {
                    SM_STATE = SM_K;
                } else {
                    SM_STATE = SM_S2;
                }
            }
            break;
        case SM_S2:
            /* Changes state after 5 iterations */
            if (!(i < 5)) {
                i = 0;
                SM_STATE = SM_Pause;
            }
            break;
        case SM_K:
            /* Changes state after 9 iterations */
            if (!(i < 9)) {
                i = 0;
                SM_STATE = SM_Pause;
            }
            break;
        case SM_Pause:
            /* Changes state after 7 iterations */
            if (!(i < 7)) {
                i = 0;
                /* Goes to O if MorseWord is 1, else goes to S1 */
                if (MorseWord) {
                    SM_STATE = SM_O;
                } else {
                    SM_STATE = SM_S1;
                }
            }
            break;
        default:
            SM_STATE = SM_Start;
            break;
    }

    switch(SM_STATE) { // State actions
        case SM_Start:
            break;
        case SM_S1:
            /*
             *  i    :   0   1   2   3   4   5   6   7
             *  LED0 :   ON  OFF ON  OFF ON  OFF OFF OFF
             *  Morse:   .       .       .
             */
            if (i == 0 || i == 2 || i == 4) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            i++;
            break;
        case SM_O:
            /*
             *  i    :   0   1   2   3   4   5   6   7   8   9   10  11  12  13
             *  LED1 :   ON  ON  ON  OFF ON  ON  ON  OFF ON  ON  ON  OFF OFF OFF
             *  Morse:   __________      __________      __________
             */
            if ((i < 3) || (i > 3 && i < 7) || (i > 7 && i < 11)) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            }
            i++;
            break;
        case SM_S2:
            /*
             *  i    :   0   1   2   3   4
             *  LED0 :   ON  OFF ON  OFF ON
             *  Morse:   .       .       .
             */
            if (i == 0 || i == 2 || i == 4) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            i++;
            break;
        case SM_K:
            /*
             *  i    :   0   1   2   3   4   5   6   7   8
             *  LED0 :   OFF OFF OFF OFF ON  OFF OFF OFF OFF
             *  LED1 :   ON  ON  ON  OFF OFF OFF ON  ON  ON
             *  Morse:   __________      .       __________
             */
            if (i < 3 || i > 5) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            }
            if (i == 4) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            i++;
            break;
        case SM_Pause:
            if (i == 0) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            } else if (i == 6) {
                /* If a button was pressed, switch Morse word and reset ButtonClick */
                if (ButtonClick) {
                    MorseWord = !MorseWord;
                    ButtonClick = 0;
                }
            }
            i++;
            break;
        default:
            break;
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0 and CONFIG_GPIO_BUTTON_1.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn(uint_least8_t index)
{
    /* Tell program that a button was pressed */
    ButtonClick = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    /* Turn LEDs off */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

    /* Set initial state */
    SM_STATE = SM_Start;

    /* Initialize timer */
    initTimer();

    while(1){
        Tick_Morse();           // Execute one tick
        while(!TimerFlag){}     // Wait for timer period
        TimerFlag = 0;          // Lower flag
    }
}
