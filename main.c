#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <string.h>
#include <stdio.h>

int echo_pulse_duration = 0;
int distance_cm[3] = {0};
int distance_cmb[3] = {0};

int mode = 0;
int ForB = 0;

int digone = 0;
int digtwo = 0;
int digthree = 0;
int entzero = 0;
int entone = 0;
int enttwo = 0;
int entthree = 0;
int entfour = 0;

int FT_ONE = 50;
int FT_TWO = 25;
int BT_ONE = 75;
int BT_TWO = 50;
int BT_THREE = 25;

void main(void) {
    __disable_interrupt();
    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);
    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_Clock();   //Sets up the necessary system clocks
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_SW_IRQ();  //Setup push button interrupts
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
    __enable_interrupt();

    Setup:
    {
        mode = 0;
        digone = 0;
        digtwo = 0;
        digthree = 0;
        entzero = 0;
        entone = 0;
        enttwo = 0;
        entthree = 0;
        entfour = 0;

        displayScrollText("SETUP MODE");

        showChar('D', pos1);
        showChar('S', pos6);

        while (1) {
            if (entzero == 1) {
                FT_ONE = 50;
                FT_TWO = 25;
                BT_ONE = 75;
                BT_TWO = 50;
                BT_THREE = 25;
                goto User;
            } else if (entzero == -1) {
                goto Start;
            }
        }
    }

    Start:
    {
        while (!mode) {
            if (entone == 0) {
                displayScrollText("SET FT1");
                entone = 1;
            } else if (enttwo == 0) {
                showChar(digone+48, pos6);
            } else if (entthree == 0) {
                showChar(digtwo+48, pos5);
            } else if (entfour == 0) {
                showChar(digthree+48, pos4);
            }
        }

        FT_ONE = digone + digtwo*10 + digthree*100;

        if (FT_ONE < 3) {
            FT_ONE = 3;
        } else if (FT_ONE > 400) {
            FT_ONE = 400;
        }

        mode = 0;
        digone = 0;
        digtwo = 0;
        digthree = 0;
        entone = 0;
        enttwo = 0;
        entthree = 0;
        entfour = 0;

        while (!mode) {
            if (entone == 0) {
                displayScrollText("SET FT2");
                entone = 1;
            } else if (enttwo == 0) {
                showChar(digone+48, pos6);
            } else if (entthree == 0) {
                showChar(digtwo+48, pos5);
            } else if (entfour == 0) {
                showChar(digthree+48, pos4);
            }
        }

        FT_TWO = digone + digtwo*10 + digthree*100;

        if (FT_TWO < 3) {
            FT_TWO = 3;
        } else if (FT_TWO > 400) {
            FT_TWO = 400;
        }

        mode = 0;
        digone = 0;
        digtwo = 0;
        digthree = 0;
        entone = 0;
        enttwo = 0;
        entthree = 0;
        entfour = 0;

        while (!mode) {
            if (entone == 0) {
                displayScrollText("SET BT1");
                entone = 1;
            } else if (enttwo == 0) {
                showChar(digone+48, pos6);
            } else if (entthree == 0) {
                showChar(digtwo+48, pos5);
            } else if (entfour == 0) {
                showChar(digthree+48, pos4);
            }
        }

        BT_ONE = digone + digtwo*10 + digthree*100;

        if (BT_ONE < 3) {
            BT_ONE = 3;
        } else if (BT_ONE > 400) {
            BT_ONE = 400;
        }

        mode = 0;
        digone = 0;
        digtwo = 0;
        digthree = 0;
        entone = 0;
        enttwo = 0;
        entthree = 0;
        entfour = 0;

        while (!mode) {
            if (entone == 0) {
                displayScrollText("SET BT2");
                entone = 1;
            } else if (enttwo == 0) {
                showChar(digone+48, pos6);
            } else if (entthree == 0) {
                showChar(digtwo+48, pos5);
            } else if (entfour == 0) {
                showChar(digthree+48, pos4);
            }
        }

        BT_TWO = digone + digtwo*10 + digthree*100;

        if (BT_TWO < 3) {
            BT_TWO = 3;
        } else if (BT_TWO > 400) {
            BT_TWO = 400;
        }

        mode = 0;
        digone = 0;
        digtwo = 0;
        digthree = 0;
        entone = 0;
        enttwo = 0;
        entthree = 0;
        entfour = 0;

        while (!mode) {
            if (entone == 0) {
                displayScrollText("SET BT3");
                entone = 1;
            } else if (enttwo == 0) {
                showChar(digone+48, pos6);
            } else if (entthree == 0) {
                showChar(digtwo+48, pos5);
            } else if (entfour == 0) {
                showChar(digthree+48, pos4);
            }
        }

        BT_THREE = digone + digtwo*10 + digthree*100;

        if (BT_THREE < 3) {
            BT_THREE = 3;
        } else if (BT_THREE > 400) {
            BT_THREE = 400;
        }

        if (FT_TWO >= FT_ONE) {
            FT_ONE = 50;
            FT_TWO = 25;
        }

        if (BT_THREE >= BT_TWO || BT_TWO >= BT_ONE || BT_THREE >= BT_ONE) {
            BT_ONE = 75;
            BT_TWO = 50;
            BT_ONE = 25;
        }
    }

    User:
    {
        displayScrollText("USER MODE");

        param2.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
        param2.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
        param2.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
        param2.timerClear = TIMER_A_SKIP_CLEAR;
        param2.startTimer = true;

        param3.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
        param3.captureMode = TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE;
        param3.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
        param3.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
        param3.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
        param3.captureOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;

        int bcount = 0;
        int fcount = 0;
        int count = 0;
        int number = 400;
        int average = 400;
        int i = 0;

        char x[3] = {0};
        mode = 1;
        int no_beep = 1;
        int double_beep = 0;

        Timer_A_initContinuousMode(TIMER_A1_BASE, &param2);

        while(1) {
            if (!mode) {
                goto Setup;
            }
            Timer_A_clear(TIMER_A1_BASE);
    //
    //        //send a trigger pulse of 10 us
            GPIO_setOutputHighOnPin(DSO_PORT, DSO_PIN);
    //        Timer_A_outputPWM(TIMER_A0_BASE, &param);

            __delay_cycles(1); //want 10 microsecond

            GPIO_setOutputLowOnPin(DSO_PORT, DSO_PIN);

            while(GPIO_getInputPinValue(DSI_PORT, DSI_PIN) == 0){}

            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

            while(GPIO_getInputPinValue(DSI_PORT, DSI_PIN) == 1){}

            Timer_A_stop(TIMER_A1_BASE);

            echo_pulse_duration = Timer_A_getCounterValue(TIMER_A1_BASE);

            distance_cm[fcount] = echo_pulse_duration/58;

            if (distance_cm[fcount] < 3) {
                distance_cm[fcount] = 3;
            } else if (distance_cm[fcount] > 400) {
                distance_cm[fcount] = 400;
            }

            if (count == 3) {
                i = 0;
                number = (distance_cm[0]+(4*distance_cm[1])+distance_cm[2])/6;
                average = number;
                while (number != 0) {
                    x[i] = number % 10;
                    number /= 10;
                    i++;
                }
            }

            if(ForB == 0) {
                showChar('F', pos1);
                if (average < 10) {
                    showChar(x[0]+48, pos6);
                    showChar('0', pos5);
                    showChar('0', pos4);
                } else if (average < 100) {
                    showChar(x[0]+48, pos6);
                    showChar(x[1]+48, pos5);
                    showChar('0', pos4);
                } else {
                    showChar(x[0]+48, pos6);
                    showChar(x[1]+48, pos5);
                    showChar(x[2]+48, pos4);
                }
            }

            if (average <= FT_TWO && (no_beep || double_beep)) { //QUAD BEEP FOR CLOSEST THRESHOLD
                no_beep = 0;
                double_beep = 0;

                Init_PWM2(200, 100); //Set period and high count

                Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                _delay_cycles(100000);
                Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

                _delay_cycles(100000);
                Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                _delay_cycles(100000);
                Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

                _delay_cycles(100000);
                Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                _delay_cycles(100000);
                Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

                _delay_cycles(100000);
                Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                _delay_cycles(100000);
                Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
            } else if (average <= FT_ONE && no_beep) { //DOUBLE BEEP MIDDLE THRESHOLD
                double_beep = 1;
                no_beep = 0;

                Init_PWM2(300, 150); //Set period and high count

                Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                _delay_cycles(200000);
                Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

                _delay_cycles(200000);
                Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                _delay_cycles(200000);
                Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
            } else if (average > FT_ONE) {
                no_beep = 1;
                double_beep = 0;
            } else if (average > FT_TWO) {
                double_beep = 1;
            }

            __delay_cycles(170000);

            fcount++;
            if (fcount > 2) {
                fcount = 0;
            }

            Timer_A_clear(TIMER_A1_BASE);

            //send a trigger pulse of 10 us
            GPIO_setOutputHighOnPin(DSOB_PORT, DSOB_PIN);

            __delay_cycles(1); //want 10 microsecond

            GPIO_setOutputLowOnPin(DSOB_PORT, DSOB_PIN);

            while(GPIO_getInputPinValue(DSIB_PORT, DSIB_PIN) == 0){}

            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

            while(GPIO_getInputPinValue(DSIB_PORT, DSIB_PIN) == 1){}

            Timer_A_stop(TIMER_A1_BASE);

            echo_pulse_duration = Timer_A_getCounterValue(TIMER_A1_BASE);

            distance_cmb[bcount] = echo_pulse_duration/58;

            if (distance_cmb[bcount] < 3) {
                distance_cmb[bcount] = 3;
            } else if (distance_cmb[bcount] > 400) {
                distance_cmb[bcount] = 400;
            }

            if (count == 3) {
                i = 0;
                number = (distance_cmb[0]+(4*distance_cmb[1])+distance_cmb[2])/6;
                average = number;
                while (number != 0) {
                    x[i] = number % 10;
                    number /= 10;
                    i++;
                }
            }

            if(ForB == 1) {
                showChar('B', pos1);
                if (average < 10) {
                    showChar(x[0]+48, pos6);
                    showChar('0', pos5);
                    showChar('0', pos4);
                } else if (average < 100) {
                    showChar(x[0]+48, pos6);
                    showChar(x[1]+48, pos5);
                    showChar('0', pos4);
                } else {
                    showChar(x[0]+48, pos6);
                    showChar(x[1]+48, pos5);
                    showChar(x[2]+48, pos4);
                }
            }

            if (average <= BT_THREE) {
                GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
                GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
                GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
                GPIO_setOutputHighOnPin(RED_PORT, RED_PIN);
            } else if (average <= BT_TWO) {
                GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
                GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
                GPIO_setOutputHighOnPin(ORANGE_PORT, ORANGE_PIN);
                GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
            } else if (average <= BT_ONE) {
                GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
                GPIO_setOutputHighOnPin(YELLOW_PORT, YELLOW_PIN);
                GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
                GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
            } else {
                GPIO_setOutputHighOnPin(GREEN_PORT, GREEN_PIN);
                GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
                GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
                GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
            }

            bcount++;
            if (bcount > 2) {
                bcount = 0;
            }

            if (count < 3) {
                count++;
            }
        }
    }
}

void Init_SW_IRQ(void) {
    //Initialize SW1 Interrupt
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_enableInterrupt(SW1_PORT, SW1_PIN);
    GPIO_selectInterruptEdge(SW1_PORT, SW1_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);

    //Initialize SW2 Interrupt
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    GPIO_enableInterrupt(SW2_PORT, SW2_PIN);
    GPIO_selectInterruptEdge(SW2_PORT, SW2_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(SW2_PORT, SW2_PIN);
}

void Init_GPIO(void) {
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Speaker 1
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);

    //Sensor 1
    GPIO_setAsInputPin(DSI_PORT, DSI_PIN);
    GPIO_setAsOutputPin(DSO_PORT, DSO_PIN);

    //Sensor 2
    GPIO_setAsInputPin(DSIB_PORT, DSIB_PIN);
    GPIO_setAsOutputPin(DSOB_PORT, DSOB_PIN);

    //Green LED
    GPIO_setAsOutputPin(GREEN_PORT, GREEN_PIN);

    //Orange LED
    GPIO_setAsOutputPin(ORANGE_PORT, ORANGE_PIN);

    //Yellow LED
    GPIO_setAsOutputPin(YELLOW_PORT, YELLOW_PIN);

    //Red LED
    GPIO_setAsOutputPin(RED_PORT, RED_PIN);
}

/* Clock System Initialization */
void Init_Clock(void) {
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* PWM Initialization */
void Init_PWM2(int period, int hc) {
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = period; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = hc; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

/* PWM Initialization */
void Init_PWM(void) {
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

#pragma vector=PORT1_VECTOR
__interrupt
void SW1(void) {
    __disable_interrupt();

    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);

    if (!mode) {
        //displayScrollText("SW1");
        if (entzero == 0) {
            entzero = 1;
        } else if (entone == 0) {

        } else if (enttwo == 0) {
            digone++;
            if (digone > 9) {
                digone = 0;
            }
        } else if (entthree == 0) {
            digtwo++;
            if (digtwo > 9) {
                digtwo = 0;
            }
        } else if (entfour == 0) {
            digthree++;
            if (digthree > 9) {
                digthree = 0;
            }
        }
    } else {
        if (ForB == 0) {
            ForB = 1;
        } else {
            ForB = 0;
        }
    }

    _delay_cycles(200000);
    __enable_interrupt();
}

#pragma vector=PORT2_VECTOR
__interrupt
void SW2(void) {
    __disable_interrupt();

    GPIO_clearInterrupt(SW2_PORT, SW2_PIN);

    if (!mode) {
        //displayScrollText("SW2");
        if (entzero == 0) {
            entzero = -1;
        } else if (entone == 0) {
            entone = 1;
        } else if (enttwo == 0) {
            enttwo = 1;
        } else if (entthree == 0) {
            entthree = 1;
        } else if (entfour == 0) {
            entfour = 1;
            mode = 1;
        }
    } else {
        mode = 0;
        digone = 0;
        digtwo = 0;
        digthree = 0;
        entone = 0;
        enttwo = 0;
        entthree = 0;
        entfour = 0;
    }

    _delay_cycles(200000);
    __enable_interrupt();
}
