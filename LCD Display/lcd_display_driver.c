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
    
    // entry mode set: display on, display shift off, increment off
    LATE = 0x4;
    lcd_display_driver_enable();
    delay_1us(50);
}

// write the string of some length to the display
void lcd_display_driver_write(char* data) {
    // set RS to high for data mode
    RS_lcd = 0x1;
    RW_lcd = 0x0;
    
    // increment character
    *data += 1;
    
    // loop through A-Z for character options
    if (*data > 'Z') {
        *data = 'A';
    }
    
    // write data
    LATE = *data;
    lcd_display_driver_enable();
    delay_1us(40);
}

/* Move the position of the cursor */
void lcd_display_driver_move_cursor(int moveWhere) {
    // set RS to low for instruction mode
    RS_lcd = 0x0;
    RW_lcd = 0x0;
    
    /*
    // move right
    if (moveWhere == 1) {
        // display shift off, move cursor to right
        LATE = 0x14;
    }
    // move left
    else if (moveWhere == 0) {
        // display shift off, move cursor to left
        LATE = 0x10;
    }
    */
    
    // move right if true, else move left
    LATE = (moveWhere == 1) ? 0x14 : 0x10;
    lcd_display_driver_enable();
    delay_1us(40);
} 