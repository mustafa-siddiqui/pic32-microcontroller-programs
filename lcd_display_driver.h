/** @file   lcd_display_driver.h
 *  @brief  Header file for LCD driver.
 *  @author Mustafa Siddiqui
 *  @date   10/26/2020
 */

#ifndef LCD_DISPLAY_DRIVER_H
#define	LCD_DISPLAY_DRIVER_H

/* include the header file */
#include <xc.h>

/* MACROS for readability */
#define RS_lcd LATBbits.LATB15
#define RW_lcd LATDbits.LATD5

/** @fn     delay_1ms
 *  @brief  Function to cause a x times 300 ns delay where x is the argument
 *          passed to the function.
 */
void delay_300ns(unsigned long ns);

/** @fn     delay_1us
 *  @brief  Function to cause x times 1 us delay where x is the argument
 *          passed to the function.
 */
void delay_1us(unsigned long us);

/** @fn     lcd_display_driver_set_tristate()
 *  @brief  Sets the appropriate tri-state registers for the LCD Display. These
 *          include the E7-E0, enable bit, RS, and RW.
 */
void lcd_display_driver_set_tristate(void);

/** @fn     lcd_display_driver_enable()
 *  @brief  Sets the enable bit on the LCD Controller/Driver long enough to
 *          perform a display operation.
 */
void lcd_display_driver_enable(void);

/** @fn     lcd_display_driver_clear()
 *  @brief  Clears the display of all characters.
 */
void lcd_display_driver_clear(void);

/** @fn     lcd_display_driver_initialize()
 *  @brief  Initialize to bring up the display such that it operates in a mode
 *          with 5x8 dots, dual line, no cursor, and no blinking.
 */
void lcd_display_driver_initialize(void);

/** @fn     lcd_display_driver_write()
 *  @brief  Write the entire string defined by 'data'of length 'length' to the
 *          display.
 */
void lcd_display_driver_write(char* data, int length);

/** @fn     display_driver_use_first_line
 *  @brief  Configure the display driver to use the first line on the display
 *          for write operations.
*/
void display_driver_use_first_line(void);

/** @fn     display_driver_use_second_line
 *  @brief  Configure the display driver to use the second line on the display
 *          for write operations.
 */
void display_driver_use_second_line(void);

#endif	/* LCD_DISPLAY_DRIVER_H */

