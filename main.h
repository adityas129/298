#ifndef MAIN_H_
#define MAIN_H_

#include "driverlib/driverlib.h"

#define TIMER_A_PERIOD  250 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      125 //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//Output pin to buzzer
#define PWM_PORT        GPIO_PORT_P1
#define PWM_PIN         GPIO_PIN7

//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2

//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6

//Input to ADC - in this case input A9 maps to pin P8.1
#define ADC_IN_PORT     GPIO_PORT_P8
#define ADC_IN_PIN      GPIO_PIN1
#define ADC_IN_CHANNEL  ADC_INPUT_A9

//Sensor 1
#define DSO_PORT         GPIO_PORT_P1
#define DSO_PIN          GPIO_PIN5
#define DSI_PORT         GPIO_PORT_P8
#define DSI_PIN          GPIO_PIN2

//Sensor 2
#define DSOB_PORT         GPIO_PORT_P1
#define DSOB_PIN          GPIO_PIN4
#define DSIB_PORT         GPIO_PORT_P8
#define DSIB_PIN          GPIO_PIN3

//Green LED
#define GREEN_PORT         GPIO_PORT_P1
#define GREEN_PIN          GPIO_PIN6

//Orange LED
#define ORANGE_PORT         GPIO_PORT_P5
#define ORANGE_PIN          GPIO_PIN0

//Yellow LED
#define YELLOW_PORT         GPIO_PORT_P5
#define YELLOW_PIN          GPIO_PIN3

//Red LED
#define RED_PORT         GPIO_PORT_P1
#define RED_PIN          GPIO_PIN3

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_SW_IRQ(void);

void Init_PWM(void);
void Init_PWM2(int period, int hc);
void Init_ADC(void);

Timer_A_outputPWMParam param; //Timer configuration data structure for PWM
Timer_A_initContinuousModeParam param2; //Timer configuration data structure for continuous mode
Timer_A_initCaptureModeParam param3; //Timer configuration data structure for continuous mode

//Timer_A_initCaptureMode param3;

Timer_A_outputPWMParam param; //Timer configuration data structure for PWM

#endif /* MAIN_H_ */
