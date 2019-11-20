#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <string.h>
#include <stdio.h>

/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int echo_pulse_duration = 0;
int distance_cm = 0;
int distance_cmb = 0;

int mode = 0;
int ForB = 0;

int digone = 0;
int digtwo = 0;
int digthree = 0;
int entone = 0;
int enttwo = 0;
int entthree = 0;
int entfour = 0;

int FT_ONE = 200;
int FT_TWO = 100;
int FT_THREE = 50;
int BT_ONE = 100;
int BT_TWO = 50;

void main(void) {
    __disable_interrupt();
    //char buttonState = 1; //Current button press state (to allow edge detection)

     /* Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_SW_IRQ();  //Setup push button interrupts

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    __enable_interrupt();

    Setup:
//    {
//        displayScrollText("SETUP MODE");
//
//        while (!mode) {
//            if (entone == 0) {
//                displayScrollText("SET FT1");
//            } else if (enttwo == 0) {
//                showChar(digone+48, pos6);
//            } else if (entthree == 0) {
//                showChar(digtwo+48, pos5);
//            } else if (entfour == 0) {
//                showChar(digthree+48, pos4);
//            }
//        }
//
//        FT_ONE = digone + digtwo*10 + digthree*100;
//
//        mode = 0;
//        digone = 0;
//        digtwo = 0;
//        digthree = 0;
//        entone = 0;
//        enttwo = 0;
//        entthree = 0;
//        entfour = 0;
//
//        while (!mode) {
//            if (entone == 0) {
//                displayScrollText("SET FT2");
//            } else if (enttwo == 0) {
//                showChar(digone+48, pos6);
//            } else if (entthree == 0) {
//                showChar(digtwo+48, pos5);
//            } else if (entfour == 0) {
//                showChar(digthree+48, pos4);
//            }
//        }
//
//        FT_TWO = digone + digtwo*10 + digthree*100;
//
//        mode = 0;
//        digone = 0;
//        digtwo = 0;
//        digthree = 0;
//        entone = 0;
//        enttwo = 0;
//        entthree = 0;
//        entfour = 0;
//
//        while (!mode) {
//            if (entone == 0) {
//                displayScrollText("SET FT3");
//            } else if (enttwo == 0) {
//                showChar(digone+48, pos6);
//            } else if (entthree == 0) {
//                showChar(digtwo+48, pos5);
//            } else if (entfour == 0) {
//                showChar(digthree+48, pos4);
//            }
//        }
//
//        FT_THREE = digone + digtwo*10 + digthree*100;
//
//        mode = 0;
//        digone = 0;
//        digtwo = 0;
//        digthree = 0;
//        entone = 0;
//        enttwo = 0;
//        entthree = 0;
//        entfour = 0;
//
//        while (!mode) {
//            if (entone == 0) {
//                displayScrollText("SET BT1");
//            } else if (enttwo == 0) {
//                showChar(digone+48, pos6);
//            } else if (entthree == 0) {
//                showChar(digtwo+48, pos5);
//            } else if (entfour == 0) {
//                showChar(digthree+48, pos4);
//            }
//        }
//
//        BT_ONE = digone + digtwo*10 + digthree*100;
//
//        mode = 0;
//        digone = 0;
//        digtwo = 0;
//        digthree = 0;
//        entone = 0;
//        enttwo = 0;
//        entthree = 0;
//        entfour = 0;
//
//        while (!mode) {
//            if (entone == 0) {
//                displayScrollText("SET BT2");
//            } else if (enttwo == 0) {
//                showChar(digone+48, pos6);
//            } else if (entthree == 0) {
//                showChar(digtwo+48, pos5);
//            } else if (entfour == 0) {
//                showChar(digthree+48, pos4);
//            }
//        }
//
//        BT_TWO = digone + digtwo*10 + digthree*100;
//    }

    displayScrollText("SENSOR");

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

    Timer_A_initContinuousMode (TIMER_A1_BASE, &param2);
    mode = 1;
    while(1) {
//        displayScrollText("START");
//        if (!mode) {
//            displayScrollText("LEAVE");
//            goto Setup;
//        }
        Timer_A_clear(TIMER_A1_BASE);
//
//        //send a trigger pulse of 10 us
        GPIO_setOutputHighOnPin(DSO_PORT, DSO_PIN);
//        Timer_A_outputPWM(TIMER_A0_BASE, &param);

        __delay_cycles(1); //want 10 microsecond

        GPIO_setOutputLowOnPin(DSO_PORT, DSO_PIN);

//        showChar('1', pos1);

        while(GPIO_getInputPinValue(DSI_PORT, DSI_PIN) == 0){}

        showChar('1', pos2);

        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

        while(GPIO_getInputPinValue(DSI_PORT, DSI_PIN) == 1){}

        Timer_A_stop(TIMER_A1_BASE);

        echo_pulse_duration = Timer_A_getCounterValue(TIMER_A1_BASE);

        distance_cm = echo_pulse_duration/58;

        char x[15] = {0};

        if (distance_cm < 3) {
            distance_cm = 3;
        } else if (distance_cm > 400) {
            distance_cm = 400;
        }

        sprintf(x, "%d", distance_cm);

        if(ForB == 0) {
            showChar('F', pos1);

            if (distance_cm < 10) {
                showChar(x[0]+48, pos6);
            } else if (distance_cm < 100) {
                showChar(x[0]+48, pos5);
                showChar(x[1]+48, pos6);
            } else {
                showChar(x[0]+48, pos4);
                showChar(x[1]+48, pos5);
                showChar(x[2]+48, pos6);
            }
        }

        if (distance_cm < FT_THREE) {
            GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
            GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
            GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
            GPIO_setOutputHighOnPin(RED_PORT, RED_PIN);
        } else if (distance_cm < FT_TWO) {
            GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
            GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
            GPIO_setOutputHighOnPin(YELLOW_PORT, YELLOW_PIN);
            GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
        } else if (distance_cm < FT_ONE) {
            GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
            GPIO_setOutputHighOnPin(ORANGE_PORT, ORANGE_PIN);
            GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
            GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
        } else {
            GPIO_setOutputHighOnPin(GREEN_PORT, GREEN_PIN);
            GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
            GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
            GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
        }
        __delay_cycles(170000);

        Timer_A_clear(TIMER_A1_BASE);

        //send a trigger pulse of 10 us
        GPIO_setOutputHighOnPin(DSOB_PORT, DSOB_PIN);

        __delay_cycles(1); //want 10 microsecond

        GPIO_setOutputLowOnPin(DSOB_PORT, DSOB_PIN);

//        showChar('3', pos3);

        while(GPIO_getInputPinValue(DSIB_PORT, DSIB_PIN) == 0){}

        showChar('2', pos3);

        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

        while(GPIO_getInputPinValue(DSIB_PORT, DSIB_PIN) == 1){}

        Timer_A_stop(TIMER_A1_BASE);

        echo_pulse_duration = Timer_A_getCounterValue(TIMER_A1_BASE);

        distance_cmb = echo_pulse_duration/58;

        if (distance_cmb < 3) {
            distance_cmb = 3;
        } else if (distance_cmb > 400) {
            distance_cmb = 400;
        }

        sprintf(x, "%d", distance_cmb);

        if(ForB == 1) {
            showChar('B', pos1);

            if (distance_cmb < 10) {
                showChar(x[0]+48, pos6);
            } else if (distance_cmb < 100) {
                showChar(x[0]+48, pos5);
                showChar(x[1]+48, pos6);
            } else {
                showChar(x[0]+48, pos4);
                showChar(x[1]+48, pos5);
                showChar(x[2]+48, pos6);
            }
        }

        if (distance_cmb < BT_TWO) {
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
        } else if (distance_cmb < BT_ONE) {
            Init_PWM2(300, 150); //Set period and high count

            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
            _delay_cycles(200000);
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

            _delay_cycles(200000);
            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
            _delay_cycles(200000);
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
        }

        __delay_cycles(170000);
        showChar('0', pos1);
        showChar('0', pos2);
        showChar('0', pos3);
        showChar('0', pos4);
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
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
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

/* UART Initialization */
void Init_UART(void) {
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param)) {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void) {
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus) {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}


/* PWM Initialization */
void Init_PWM2(int period, int hc) {
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
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
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
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

void Init_ADC(void) {
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void) {
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

#pragma vector=PORT1_VECTOR
__interrupt
void SW1(void) {
    __disable_interrupt();

    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);

    if (!mode) {
        //displayScrollText("SW1");
        if (enttwo == 0) {
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
        if (entone == 0) {
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
