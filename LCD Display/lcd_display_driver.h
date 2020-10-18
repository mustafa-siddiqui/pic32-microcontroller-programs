/** @file   lcd_display_driver.h
 *  @brief  Header file for LCD driver.
 *  @author Mustafa Siddiqui
 *  @date   09/12/2020
 */

#ifndef LCD_DISPLAY_DRIVER_H
#define	LCD_DISPLAY_DRIVER_H

/* include the header file */
#include <xc.h>

/* MACROS for readability */
#define RS_lcd LATBbits.LATB15
#define RW_lcd LATDbits.LATD5
//--//
#define RIGHT 1
#define LEFT 0

/** @globalVar  CUR_POS
 *  @brief      Global variable to store the current position of the cursor
 *              so that it can be accessed by all the ISRs.
 */
static int CUR_POS = 1;

/** @globalVar  FIRSTPOS_DISPLAY
 *              SECONDPOS_DISPLAY
 *              THIRDPOS_DISPLAY
 *  @brief      Global variables to store characters to be displayed on the
 *              respective positions of the LCD Display.
 */
static char FIRSTPOS_DISPLAY = 'A' - 1;
static char SECONDPOS_DISPLAY = 'A' - 1;
static char THIRDPOS_DISPLAY = 'A' - 1;

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
 *          with 5x10 dots, dual line, no cursor, and no blinking.
 */
void lcd_display_driver_initialize(void);

/** @fn     lcd_display_driver_write()
 *  @brief  Write one character to the LCD display.
 *          Global variables passed into the function can be changed to keep 
 *          the characters in the range A-Z. The variable is passed by reference
 *          so that the function can change its value to keep in the specified
 *          range.
 */
void lcd_display_driver_write(char* data);

/** @fn     lcd_display_driver_move_cursor() 
 *  @brief  Move cursor left or right on the LCD Display.
 */
void lcd_display_driver_move_cursor(int moveWhere);

#endif	/* LCD_DISPLAY_DRIVER_H */

