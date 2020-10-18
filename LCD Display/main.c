/** @file   main.c
 *  @brief  This program configures Novatek's Single-Chip 16C X 2L Dot Matrix
 *          LCD Display Controller using the PIC32MX795F512L microcontroller
 *          such that it displays a string denoting a student's NetID.
 *  @author Mustafa Siddiqui
 *  @date   09/20/2020
 */

/* Include necessary header files */
#include <xc.h>
#include "lcd_display_driver.h"

/** @fn     int main()
    @brief  Main function of the program. This uses the functions defined in the
 *          header file to perform the required operations. It sets the appropriate
 *          registers to turn on the LEDs. It initializes the LCD Display Driver,
 *          sets the cursor to blink, and then writes the string containing the
 *          NetID to the Dot Matric LCD Display.
    @return An integer when run successfully.
*/
int main(void) {
    
    // set TRISA
    TRISA = 0xFF00;
    
    // set DDPCON
    DDPCON = 0x0;
    
    // clear LED 3,4,5,6 and set LED 7,8,9,10
    LATA &= 0x0;
    LATA |= 0xF0;
    
    // set RE to be output: b7-b0 = 0
    TRISE = !0xFF;
    
    // initialize the LCD display
    lcd_display_driver_initialize();
    
    // make cursor blink
    LATE = 0xF;
    delay_1us(50);
    lcd_display_driver_enable();
    
    // store net id in string
    char netID[] = "msiddiq7";
    char* pnetID = &netID[0];
    
    // calculate length of string
    int length;
    for (length = 0; *pnetID++; length++) {}
    
    // write string to LCD
    lcd_display_driver_write(netID, length);
    
    return 0;
}
