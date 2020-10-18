/** @file   main.c
 *  @brief  Program that reads values from the potentiometer on the 16/32 Explorer Board
 *          and prints them on the LCD Display. The LEDs on the board light up linearly
 *          with the potentiometer reading values. The larger the potentiometer reading,
 *          the more the number of LEDs that light up.
 *          Analog values are read from the potentiometer and converted into digital form.
 *          Values are sampled at a rate of 200k readings per second - a delay time of 5µs.
 *          CPU Clock is configured to run at 80 MHz to speed the process.
 *  @author Mustafa Siddiqui
 *  @date   10/11/2020
*/

/* Include necessary header files */
#include <xc.h>
#include <stdlib.h>     // itoa()
#include <string.h>     // strlen()
#include "lcd_display_driver.h"

/*  Configure CPU and peripheral bus clock to run at 80MHz
 *  f_crystal = 8MHz
 *  f_CPU = (f_crystal * FPLLMUL) / (FPLLIDIV * FPLLODIV)
 *  f_peripheral = f_CPU / FPBDIV
 */
#pragma config POSCMOD = HS         // primary oscillator mode: high speed crystal
#pragma config FNOSC = PRIPLL       // oscillator selection: primary with PLL
#pragma config FPLLMUL = MUL_20     // PLL multiplier - multiply by 20
#pragma config FPLLIDIV = DIV_2     // PLL input divider - divide by 2
#pragma config FPLLODIV = DIV_1     // PLL output divider - divide by 1
#pragma config FPBDIV = DIV_1       // peripheral bus clock - divide by 1

/* Sample and convert the value on the given ADC pin */
unsigned int adc_sample_convert(int pin) {
    unsigned int wait_time = 0;
    
    AD1CON3bits.ADCS = 2;                   // since running at 80MHz
    AD1CHSbits.CH0SA = pin;                 // connect pin
    AD1CON1bits.SAMP = 1;                   // start sampling
    
    // core timer increments once every 2 cycles of SYSCLK
    // 1 cycle = 12.5 ns -> 2 cycles = 25 ns
    wait_time = _CP0_GET_COUNT() + 200;      // set wait_time to 5000 ns
    while (_CP0_GET_COUNT() < wait_time) {} // wait for 5000 ns
    
    AD1CON1bits.SAMP = 0;                   // stop sampling & start converting
    while (!AD1CON1bits.DONE) {}            // wait for conversion process to finish
    
    return ADC1BUF0;                        // return the buffer with result
}

/* Main function of the program */
int main(void) {
    // disable JTAG
    DDPCONbits.JTAGEN = 0;
    
    // set tri-state registers for LEDs D3-D10
    TRISA = 0xFF00;
    
    // set tri-state registers for LCD Display
    TRISE = 0xFF00;         // set RE to be output: b7-b0 = 0
    TRISDbits.TRISD4 = 0;   // enable
    TRISDbits.TRISD5 = 0;   // RW
    TRISBbits.TRISB15 = 0;  // RS
    
    // set registers for analog to digital conversion
    AD1PCFGbits.PCFG2 = 0;  // AN2 is an ADC pin
    AD1CON1bits.ADON = 1;   // turn on the ADC
    
    // initialize LCD and clear display
    lcd_display_driver_initialize();
    lcd_display_driver_clear();
    
    /* Read potentiometer value, turn on LEDs linearly, & print reading on LCD
     * Display as a string
     * p_value = 127.875 * num_leds
     *     - p_value goes from 0 to 1023
     *     - num_leds goes from 0 to 8
     * LED D10 turns on first, then LED D9 and so on
     */
    LATA = 0x0;
    unsigned int prev_analog_RValue = 0;      // to account for first value to be printed
    while (1) {
        // read potentiometer reading
        unsigned int analog_RValue = adc_sample_convert(2);
        
        // typecasting to 'int' discards decimal part (truncation)
        unsigned int numLEDs = (unsigned int)(analog_RValue / 127.875);

        // light LEDs according to the linear scale
        LATA = (numLEDs == 0) ? 0x00 : LATA;
        LATA = (numLEDs == 1) ? 0x80 : LATA;
        LATA = (numLEDs == 2) ? 0xC0 : LATA;
        LATA = (numLEDs == 3) ? 0xE0 : LATA;
        LATA = (numLEDs == 4) ? 0xF0 : LATA;
        LATA = (numLEDs == 5) ? 0xF8 : LATA;
        LATA = (numLEDs == 6) ? 0xFC : LATA;
        LATA = (numLEDs == 7) ? 0xFE : LATA;
        LATA = (numLEDs == 8) ? 0xFF : LATA;
        
        // print only if current value differs from previous value
        if (prev_analog_RValue != analog_RValue) {
            char buffer[5];
            itoa(buffer, analog_RValue, 10);
            lcd_display_driver_write(buffer, strlen(buffer));
        }
        
        // assign current value to previous value
        prev_analog_RValue = analog_RValue;
    }
    
    return 0;
}