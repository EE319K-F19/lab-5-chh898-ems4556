// StepperMotorController.c starter file EE319K Lab 5
// Runs on TM4C123
// Finite state machine to operate a stepper motor.  
// Jonathan Valvano
// September 2, 2019

// Hardware connections (External: two input buttons and four outputs to stepper motor)
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)

#include "SysTick.h"
#include "TExaS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
//#include <unistd.h>         //This is for the sleep function (delay)

void EnableInterrupts(void);
// edit the following only if you need to move pins from PA4, PE3-0      
// logic analyzer on the real board
#define PA4                    (*((volatile unsigned long *)0x40004040))
#define PE50                    (*((volatile unsigned long *)0x400240FC))
//#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
//#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))

//#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400043FC))
//#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400))
//#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C))

//#define GPIO_PORTB_DATA_R       (*((volatile uint32_t *)0x400053FC))
//#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400))
//#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C))

//#define GPIO_PORTE_DATA_R       (*((volatile uint32_t *)0x400243FC))
//#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400))
//#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C))

//#define GPIO_PORTF_DATA_R       (*((volatile uint32_t *)0x400253FC))
//#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
//#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
//#define GPIO_PORTF_LOCK_R       (*((volatile uint32_t *)0x40025520))
//#define GPIO_LOCK_KEY           (*((volatile uint32_t *)0x4C4F434B))

struct State {

    uint8_t Pin_Out;
    uint8_t LED_Out;
    uint32_t delay;
    uint16_t Next_State[4];     //4 because 4 possible next outcomes
};

typedef const struct State State_Type;
#define Home 0
#define WiperC1 1
#define WiperC2 2
#define WiperC3 3
#define WiperC4 4
#define WiperC5 5
#define WiperC6 6
#define WiperC7 7
#define WiperC8 8
#define WiperC9 9
#define WiperC10 10
#define WiperCC11 11
#define WiperCC12 12
#define WiperCC13 13
#define WiperCC14 14
#define WiperCC15 15
#define WiperCC16 16
#define WiperCC17 17
#define WiperCC18 18
#define WiperCC19 19
//#define WiperCC20 20
#define CleanC1 20
#define CleanC2 21
#define CleanC3 22
#define CleanC4 23
#define CleanC5 24
#define CleanC6 25
#define CleanC7 26
#define CleanC8 27
#define CleanC9 28
#define CleanC10 29
#define CleanCC11 30
#define CleanCC12 31
#define CleanCC13 32
#define CleanCC14 33
#define CleanCC15 34
#define CleanCC16 35
#define CleanCC17 36
#define CleanCC18 37
#define CleanCC19 38
//#define CleanCC20 39

State_Type FSM[39] = {
				//										00			01				10				11
/*Home*/			{0x00, 0, 5, {Home,	CleanC1,	WiperC1,	CleanC1}},
/*WiperC1*/		{0x01, 0, 5, {WiperC2,	CleanC2,	WiperC2,	CleanC2}},
/*WiperC2*/		{0x02, 0, 5, {WiperC3,	CleanC3,	WiperC3,	CleanC3}},
/*WiperC3*/		{0x04, 0, 5, {WiperC4,	CleanC4,	WiperC4,	CleanC4}},
/*WiperC4*/		{0x08, 0, 5, {WiperC5,	CleanC5,	WiperC5,	CleanC5}},
/*WiperC5*/		{0x10, 0, 5, {WiperC6,	CleanC6,	WiperC6,	CleanC6}},
/*WiperC6*/		{0x01, 0, 5, {WiperC7,	CleanC7,	WiperC7,	CleanC7}},
/*WiperC7*/		{0x02, 0, 5, {WiperC8,	CleanC8,	WiperC8,	CleanC8}},
/*WiperC8*/		{0x04, 0, 5, {WiperC9,	CleanC9,	WiperC9,	CleanC9}},
/*WiperC9*/		{0x08, 0, 5, {WiperC10,	CleanC10,	WiperC10,	CleanC10}},
/*WiperC10*/	{0x10, 0, 5, {WiperCC11,	CleanCC11,	WiperCC11,	CleanCC11}},
/*WiperCC11*/	{0x08, 0, 5, {WiperCC12,	CleanCC12,	WiperCC12,	CleanCC12}},
/*WiperCC12*/	{0x04, 0, 5, {WiperCC13,	CleanCC13,	WiperCC13,	CleanCC13}},
/*WiperCC13*/	{0x02, 0, 5, {WiperCC14,	CleanCC14,	WiperCC14,	CleanCC14}},
/*WiperCC14*/	{0x01, 0, 5, {WiperCC15,	CleanCC15,	WiperCC15,	CleanCC15}},	
/*WiperCC15*/ {0x10, 0, 5, {WiperCC16,	CleanCC16,	WiperCC16,	CleanCC16}},
/*WiperCC16*/ {0x08, 0, 5, {WiperCC17,	CleanCC17,	WiperCC17,	CleanCC17}},
/*WiperCC17*/ {0x04, 0, 5, {WiperCC18,	CleanCC18,	WiperCC18,	CleanCC18}},
/*WiperCC18*/ {0x02, 0, 5, {WiperCC19,	CleanCC19,	WiperCC19,	CleanCC19}},
/*WiperCC19*/ {0x01, 0, 5, {Home,	CleanC1,	WiperC1,	CleanC1}},

				//										00			01				10				11
/*CleanC1*/ {0x01, 1, 5, {CleanC2,	CleanC2,	WiperC2,	CleanC2}},
/*CleanC2*/ {0x02, 0, 5, {CleanC3,	CleanC3,	WiperC3,	CleanC3}},
/*CleanC3*/ {0x04, 1, 5, {CleanC4,	CleanC4,	WiperC4,	CleanC4}},
/*CleanC4*/ {0x08, 0, 5, {CleanC5,	CleanC5,	WiperC5,	CleanC5}},
/*CleanC5*/ {0x10, 1, 5, {CleanC6,	CleanC6,	WiperC6,	CleanC6}},
/*CleanC6*/ {0x01, 0, 5, {CleanC7,	CleanC7,	WiperC7,	CleanC7}},
/*CleanC7*/ {0x02, 1, 5, {CleanC8,	CleanC8,	WiperC8,	CleanC8}},
/*CleanC8*/ {0x04, 0, 5, {CleanC9,	CleanC9,	WiperC9,	CleanC9}},
/*CleanC9*/ {0x08, 1, 5, {CleanC10,	CleanC10,	WiperC10,	CleanC10}},
/*CleanC10*/	{0x10, 0, 5, {CleanCC11,	CleanCC11,	WiperCC11,	CleanCC11}},
/*CleanCC11*/ {0x08, 1, 5, {CleanCC12,	CleanCC12,	WiperCC12,	CleanCC12}},
/*CleanCC12*/ {0x04, 0, 5, {CleanCC13,	CleanCC13,	WiperCC13,	CleanCC13}},
/*CleanCC13*/ {0x02, 1, 5, {CleanCC14,	CleanCC14,	WiperCC14,	CleanCC14}},
/*CleanCC14*/ {0x01, 0, 5, {CleanCC15,	CleanCC15,	WiperCC15,	CleanCC15}},
/*CleanCC15*/ {0x10, 1, 5, {CleanCC16,	CleanCC16,	WiperCC16,	CleanCC16}},
/*CleanCC16*/ {0x08, 0, 5, {CleanCC17,	CleanCC17,	WiperCC17,	CleanCC17}},
/*CleanCC17*/ {0x04, 1, 5, {CleanCC18,	CleanCC18,	WiperCC18,	CleanCC18}},
/*CleanCC18*/ {0x02, 0, 5, {CleanCC19,	CleanCC19,	WiperCC19,	CleanCC19}},
/*CleanCC19*/ {0x01, 1, 5, {Home,	CleanC1,	WiperC1,	CleanC1}}
        };

uint16_t currentState = Home;
uint32_t Input = 0;
void SendDataToLogicAnalyzer(void){
    UART0_DR_R = 0x80|(PA4<<2)|PE50;
}

int main(void){
    TExaS_Init(&SendDataToLogicAnalyzer);    // activate logic analyzer and set system clock to 80 MHz
    SysTick_Init();

// you initialize your system here
//PA 5-6 are inputs
    SYSCTL_RCGCGPIO_R |= 0x033;         //Clock in Pins A/B/E/F
		SysTick_Wait10ms(10);

    GPIO_PORTA_DEN_R |= 0x030;          //Enable PA 4-5
    GPIO_PORTB_DEN_R |= 0x020;          //Enable PB 6
    GPIO_PORTE_DEN_R |= 0x03F;          //Enable PE 0-5
    GPIO_PORTF_DEN_R |= 0x00E;          //Enable PF 1-3

    GPIO_PORTA_DIR_R &=~ 0x030;         //PA 4-5 are button inputs
    GPIO_PORTB_DIR_R |= 0x020;          //PB 6 is LED output
    GPIO_PORTE_DIR_R |= 0x03F;          //PE 0-5 are stepper motor outputs
    GPIO_PORTF_DIR_R |= 0x00E;          //PF 1-3 are control LED output color

    GPIO_PORTA_DATA_R &=~ 0x030;        //Clear all inputs and outputs
    GPIO_PORTB_DATA_R &=~ 0x020;
    GPIO_PORTE_DATA_R &=~ 0x01F;
    GPIO_PORTF_DATA_R &=~ 0x00E;

    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;  //Unlock port F                                    Not sure about this!**************************
    //GP
	
    EnableInterrupts();

	
    while(1) {
				int temp = (FSM[currentState].LED_Out<< 5) | (FSM[currentState].Pin_Out);
        GPIO_PORTE_DATA_R = temp; // output
        SysTick_Wait10ms(FSM[currentState].delay); // wait
        Input = (GPIO_PORTA_DATA_R & 0X30) >> 4; //input
        currentState = FSM[currentState].Next_State[Input];
    // next
    }
}


