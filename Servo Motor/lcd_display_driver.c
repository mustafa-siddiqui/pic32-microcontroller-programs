/** @file   lcd_display_driver.c
 *  @brief  This file contains the implementation of the functions defined in
 *          lcd_display_driver.h
 *  @author Mustafa Siddiqui
 *  @date   10/26/2020
 */

/* Include necessary header file */
#include <xc.h>                     // _CP0_GET_COUNT()
#include "lcd_display_driver.h"

// delay function1
void delay_300ns(unsigned long ns) {
    while (ns > 0) {
        // delay 300 ns (for a 80MHz clock)
        // 12 = 300 ns if 1 = 25 ns
        unsigned int delay_time = _CP0_GET_COUNT() + 12;
        while (_CP0_GET_COUNT() < delay_time) {}
        ns--;
    }
}

// delay function2
void delay_1us(unsigned long us) {
    while (us > 0) {
        // delay for 1 us (for a 80Mhz clock)
        // 40 = 1 microsec if 1 = 25 ns
        unsigned int delay_time = _CP0_GET_COUNT() + 40;
        while (_CP0_GET_COUNT() < delay_time) {}
        us--;
    }
}

// set tri-state registers for LCD Display
void lcd_display_driver_set_tristate(void) {
    TRISE = 0xFF00;         // set RE to be output: b7-b0 = 0
    TRISDbits.TRISD4 = 0;   // enable
    TRISDbits.TRISD5 = 0;   // RW
    TRISBbits.TRISB15 = 0;  // RS
}

// toggle enable bit on the LCD controller
void lcd_display_driver_enable(void) {
    // set the enable bit RD4s & delay for 300 ns
    LATDbits.LATD4 = 0x1;
    delay_300ns(1);
    
    // set the enable bit RD4 back to low & delay
    LATDbits.LATD4 = 0x0;
    delay_300ns(1);
}

// clear display of all characters
void lcd_display_driver_clear(void) {
    // set RS = 0 for instruction mode
    RS_lcd = 0x0;
    
    // DB0 = 1 to clear display
    LATE = 0x1;
    
    // toggle enable bit
    lcd_display_driver_enable();
    
    // delay for 1.64 ms
    delay_1us(1640);
}

// initialize to:
// - operate in a mode with 5x8 dots
// - dual line
// - no cursor
// - no blinking
void lcd_display_driver_initialize(void) {
    // set RS to low for instruction mode
    RS_lcd = 0x0;
    RW_lcd = 0x0;
    
    // function set: dual mode, 5x8 dots
    LATE = 0x38;
    lcd_display_driver_enable();
    delay_1us(50);
    
    // display on/off control: display on, cursor off, blink off
    LATE = 0xC;
    lcd_display_driver_enable();
    delay_1us(50);
    
    // clear display
    lcd_display_driver_clear();
    
    // entry mode set: display on, display shift off, increment on
    LATE = 0x6;
    lcd_display_driver_enable();
    delay_1us(50);
}

// write the string of some length to the display
// Note: display is not cleared in this function 
void lcd_display_driver_write(char* data, int length) {
    // set RS to high for data mode
    RS_lcd = 0x1;
    RW_lcd = 0x0;
    
    // write data
    int i;
    for (i = 0; i < length; i++) {
        if (*data != '\0') {
            // post-increment of pointers
            LATE = *(data++);
            lcd_display_driver_enable();
            delay_1us(40);
        }
    }
}

// configure display driver to use the first line
void display_driver_use_first_line(void) {
    RS_lcd = 0;
    RW_lcd = 0;
    LATE = 0x80;
    lcd_display_driver_enable();
}
