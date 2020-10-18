/** @file   lcd_display_driver.c
 *  @brief  This file contains the implementation of the functions defined in
 *          lcd_display_driver.h
 *  @author Mustafa Siddiqui
 *  @date   09/20/2020
 */

/* Include necessary header file */
#include "lcd_display_driver.h"

// delay function1
void delay_300ns(unsigned long ns) {
    unsigned long i;    /* apparently, loop initial declarations are not allowed */
    while (ns > 0) {
        // delay 300 ns (for a 80MHz clock)
        for (i = 0; i < 24; i++) {}
        ns--;
    }
}

// delay function2
void delay_1us(unsigned long us) {
    unsigned long i;
    while (us > 0) {
        // delay for 1 us (for a 80Mhz clock)
        for (i = 0; i < 80; i++) {}
        us--;
    }
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
    
    // delay for >1.64 ms
    delay_1us(1650);
}

// initialize to:
// - operate in a mode with 5x10 dots
// - dual line
// - no cursor
// - no blinking
void lcd_display_driver_initialize(void) {
    // set RS to low for instruction mode
    RS_lcd = 0x0;
    RW_lcd = 0x0;
    
    // function set: dual mode, 5x10 dots
    LATE = 0x3C;
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
void lcd_display_driver_write(char* data, int length) {
    // clear display before write operation
    lcd_display_driver_clear();
    
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