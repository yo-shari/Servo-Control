/*Authors:Aravind, Neal, and Sarahi 
Date:Nov 20th 2015
uses PWM to turn the servo back and forth */ 

#include <stdlib.h>
#include <avr/eeprom.h>  //presistant memory so if you remove power the info will still be there (not used)
#include <avr/interrupt.h> //interrupts not used
#include <avr/io.h>  //avr standard pin macros for teesny
#include <util/delay.h> // allows you to delay
#include <string.h>
#include <stdlib.h>  //standard library tools
#include <assert.h>  //preprocessor macro: so when it fails it prints out a message insteada core dumping (meaningful debug tool)
#include <avr/pgmspace.h> //program space: allows you to directly access the teensy memory (not used)
#include <stdio.h>   //c standard in out
#include "Arduino.h"
#include "WProgram.h"

#define F_CPU           8000000UL //central processing unit, sets time for cpu clock fastest. Max speed posssible
#define ADC_PRESCALER   128        //ADC reads voltages from pins, scales (slowers clock) to the amount of samples you want read
//#define BAUD_RATE       38400
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))  //used to set frequency, redundant way that you set the CPU to less that the max possible 

int angleA;
int angleB;
int min16 = 544/16;
int max16 = 2400/16;    //?

void pwm_init() {                                               //
        TCCR1A = (1<<WGM11);                                    //9 bit phase corect PWM
        TCCR1A = (TCCR1A & ~(1<<COM1A0)) |  (1<<COM1A1);        //set OC1A on (mode) compare match
        TCCR1A = (TCCR1A & ~(1<<COM1B0)) |  (1<<COM1B1);        //**************************************LOOK UP OC1B
        TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);           //this clears the timer for compare match| cs11 sets the clock to an (1/8) of clock speed
        //TCCR1A = (1 << COM1A1) | (1 << WGM11);
        //TCCR1B = (1 << WGM12) | (1 << WGM13) | (1<<CS00);
        OCR1A = 3000;           //sets the duty cycle registers
        OCR1B = 3000;           //same
        ICR1 = 20000;           //input capture register: used to define counter top value, set the clear on compare match
        DDRB |= (1<<PB6);
        DDRB |= (1<<PB5);       //passes a 1 to port B5 on the teensy, enables it as an output
}

void uart_init(void) {
        UBRR1 = 25;             //Set baud rate. 25 is a hex value (0010 0101) pins 2,5,7 set to high
        UCSR1A = (1<<U2X1);     //doubles uart transmission speed U2x1 is one bit in UCSR1A 
        UCSR1B = ((1<<RXEN1) | (1<<TXEN1));     //enables receive and transmit
        UCSR1C = ((1<<UCSZ11) | (1<<UCSZ10));   //set 8 bit character size
}

//The function below sends a character over serial
void uart_putchar(char data) {
                        
        while(!(UCSR1A & (1 << UDRE1)));
                UDR1 = data;
}

//The function below sends the whole string
void uart_putstring(char* data) {

        int i = 0;
        while (data[i] != '\0') {
                while(!(UCSR1A & (1 << UDRE1)));
                UDR1 = data[i];
                i++;
        }
}
//This will power the motor until it reaches the desired angle
void writeAngle(int angleArg,int * angle, char letter) {
        /*if (angleArg > 0){
            uart_putstring("angle output by servo\n\0");
            uart_putchar((unit_8)angleArg);
        }*/
        uint16_t p;
        if (angleArg < 0) angleArg = 0;
        if (angleArg > 180) angleArg = 180;
        *angle = angleArg;
        
        p = (min16*16L*8 + (max16-min16)*(16L*8)*(angleArg)/180L)/8L;        //calculates the duty cycle to reach the desired angle 
        switch(letter)
        {
                case 'A':
                        uart_putstring("Setting servo A to: ");
                        uart_putstring(*angle);
                        OCR1A = p;
                        break;
                case 'B':
                        uart_putstring("Setting servo B to: ");
                        uart_putstring(*angle);
                        OCR1B = p;
                        break;
                default:
                        break;                                              //the register you assign the duty cycle too
        }
}

void writeAngleB(int angleArg) {
        uint16_t p;
        
        if (angleArg < 0) angleArg = 0;
        if (angleArg > 180) angleArg = 180;
        angleB = angleArg;

        p = (min16*16L*8 + (max16-min16)*(16L*8)*angleB/180L)/8L;

        OCR1B = p;
        uart_putstring("Setting servo to:");
        uart_putstring(angleArg);
}

//reads the angle
uint8_t readAngleA() {
        return angleA;
}

uint8_t readAngleB() {
        return angleB;
}

uint8_t uart_hw_getc_timeout(char *data, uint16_t timeout){
	while (!(UCSR1A & (1<<RXC1))){
		if (timeout == 0) {//Has Timeout elapsed?
			return (1); // If so return and error
		}
		_delay_ms(1); // Wait a tick
		timeout --; // Decrement the Timeout number
	}
	*data = UDR1;
	return (0);
}

char uart_hw_getc(){
        int timer = 0;
	while (!(UCSR1A & (1<<RXC1))){ // Wait for byte to arrive
                timer++;
                if (timer == 100000) {
                        return -1;
                } 
        }
	return UDR1;
}

int main() {
        unsigned char unicorn_string[31];

        uart_init();            //calls the uart initialize function
        pwm_init();             //calls the pwm initialize function
        unsigned char temp;
        CPU_PRESCALE(0x01);
        /*while (1) {
                _delay_ms(40);
                uart_putchar('a');
                uart_putstring("hello");
                while(UCSR1A & (1 << RXC1));
                temp = UDR1;
        }*/       

//turns it while printing an angle
       /* while(1) {
                for (int pos = 0; pos<90; pos++) {
                        writeAngle(pos);
                        _delay_ms(15);
                        itoa(readAngle(), temp_string, 10);
                        uart_putstring(temp_string);
                        uart_putchar('\n');
                }
                for (int pos = 90; pos>=1; pos--) {
                        writeAngle(pos);
                        _delay_ms(15);
                        itoa(readAngle(), temp_string, 10);
                        uart_putstring(temp_string);
                        uart_putchar('\n');
                }
        }*/
        int i = 0; 
        int angle = 0;
        unsigned char a;
        
        uart_putstring("Enter an angle: "); 
        while(1){
                a = uart_hw_getc();
                if (a != '\n') {
                    unicorn_string[i] = a;
                    i++;
                }       
                else {
                    unicorn_string[i] = '\0';
 
                        //uart_putstring(unicorn_string);        //
                    angle = atoi(unicorn_string);
                    writeAngle(angle, &angleA, 'A');
                    writeAngle(angle, &angleB, 'B');
                    memset(unicorn_string, NULL, sizeof(unicorn_string));
              
                    uart_putstring("Enter an angle: "); 
                }     
                                //itoa(((int)unicorn), unicorn_string, 10); 
                //uart_putstring(unicorn_string);        //
                 //uart_hw_getc_timeout();

        } 
}

