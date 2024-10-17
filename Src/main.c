/*
 * Author: Josh Benner
 * Date: 10/11/24
 * Class: EECE 344
 * Project: Lab 2
 * TODO done
 *
 * Summary of Program:
 * This program initializes GPIO pins and toggles LEDs Based on
 * how long the button is pressed.
 */

#include "stm32l476xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Define constants for pin modes
#define INPUT  0
#define OUTPUT 1
#define ALTERNATE_FUNCTION 2
#define ANALOG 3

// Define constants for output types
#define PUSH_PULL 0
#define OPEN_DRAIN 1

// Define  pin states
#define HIGH 1
#define LOW  0

#define LONG_PRESS 200 //threshold for turning OFF LEDS (ms)

// Redirect C printf to this function to print to debug window
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}



//Count how long the button is pressed and return the result
//GPIO corresponds to the port that button is connected to
// Pin corresponds to the pin the button is connected to
//return = result which is the number of times the loop
// is executed
uint32_t time_pressed(GPIO_TypeDef *GPIO, unsigned int pin) {
    uint32_t result = 0;
    while ((GPIO->IDR & (1 << pin)) == 0) {
        for (int i = 0; i < 4000; i++) {
            __NOP();
        }
        result++;
    }
    return result;
}

//Set the pin mode
// GPIO is the port to config
// pin is the pin number to config
//mode uses the defined variables above
// labeled as "Define constants for pin modes"
void set_pinMode(GPIO_TypeDef *GPIO, unsigned int pin, unsigned int mode) {
    GPIO->MODER &= ~(0x3 << (pin * 2)); // clear bits
    GPIO->MODER |= (mode << (pin * 2));  // Set pin to input/output/other modes
}

// set the output type for pin
//GPIO is the port to config
// pin is the pin number to config
// type corresonds to the defined variables above labeled:
// "Define constants for output types"
void set_output_type(GPIO_TypeDef *GPIO, unsigned int pin, unsigned int type) {
    GPIO->OTYPER &= ~(0x1 << pin); //clear bits
    if (type == OPEN_DRAIN) {
        GPIO->OTYPER |= (0x1 << pin);
    }
}

//set pin state
// GPIO is the port to config
// pin is the pin number to config
//state is one of the defined variables above labeled as:
//"Define  pin states"
void set_pin_state(GPIO_TypeDef *GPIO, unsigned int pin, unsigned int state) {
    if (state == HIGH) {
        GPIO->BSRR |= (0x1 << pin);// set pin to high
    } else {
        GPIO->BSRR |= (0x1 << (pin + 16)); // reset pin by writing to upper 16 bits
    }
}
//Enable the clocks for GPIO A, B, C
void GPIO_Clock_Enable(void) {
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
}

//Intialize clock, pin mode, output type, pin state
void GPIO_Init(void) {
    GPIO_Clock_Enable();

    set_pinMode(GPIOA, 0, OUTPUT);        // Configure PA0 as output
    set_output_type(GPIOA, 0, PUSH_PULL); // Configure PA0 to push-pull
    set_pin_state(GPIOA, 0, HIGH);        // Set PA0 state to high

    set_pinMode(GPIOB, 10, OUTPUT);       // Configure PB10 as output
    set_output_type(GPIOB, 10, PUSH_PULL);// Configure PB10 to push-pull
    set_pin_state(GPIOB, 10, LOW);        // Set PB10 state to low

    set_pinMode(GPIOC, 13, INPUT);        // Configure PC13 as input
}

//delay the executions time by running through loop
// s is the delay parameter
void delay(volatile uint32_t s) {
    uint32_t i;
    while (s-- > 0) {
        for (i = 0; i < 16000; i++) {
            __NOP();  // 'nop' means "no operation", used to burn CPU cycles
        }
    }
}
//return the state of pin
// GPIO is the port
// Pin is the Pin number
// return HIGH if true else LOW
uint16_t get_pin_state(GPIO_TypeDef *GPIO, uint16_t pin) {
    return (GPIO->IDR & (1 << pin)) ? HIGH : LOW;
}

//change the state of led based on current state
// GPIO is the port
// pin is the pin number
void toggle_LEDs(GPIO_TypeDef *GPIO, uint16_t pin){
	if(get_pin_state(GPIO, pin) == LOW) { // if pin state is low
	set_pin_state(GPIO, pin, HIGH); // turn led HIGH
	}else{
		set_pin_state(GPIO, pin, LOW); // ELSE Pin is High so turn LOW
	}
}

/*
 * Main program
 */
int main(void) {
    GPIO_Init();  // Initialize GPIOs


    while (1) {

    		uint16_t pin_state = get_pin_state(GPIOC, 13);//get pin state of button

    	    if( pin_state == 0){//when button is pressed enter
    	    	delay(15); //debounce

    	    	pin_state = get_pin_state(GPIOC, 13); //check pin state again

    	    	//if button was pressed once execute part A
    	    	if(pin_state ==1){
    	    		toggle_LEDs(GPIOA, 0);
    	    		toggle_LEDs(GPIOB, 10);
    	    		delay(20);

    	    	//else execute PART B
    	    	}else{
    	    		uint32_t time_Pressed = time_pressed(GPIOC,13); // how long button is pressed for

    	    		if(time_Pressed >= LONG_PRESS){// if button pressed was longer than threshold
    	    			set_pin_state(GPIOA , 0, LOW);//turn off led
    	    			set_pin_state(GPIOB , 10, LOW);//turn off led
    	    			delay(20); //delay to show leds off
    	    		}else{// else button was pressed for less than threshold
    	    			set_pin_state(GPIOA , 0, HIGH); //turn on led
    	    			set_pin_state(GPIOB , 10, HIGH);//turn on led
    	    			delay(20);// delay to show leds on
    	    		}
    	    }


    	    }
    				// reset to intial state
    	    	   set_pin_state(GPIOB, 10, LOW);
    	    	   set_pin_state(GPIOA, 0, HIGH);
    }
    return 0;
}



