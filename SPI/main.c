/** @file   main.c
 *  @brief  Code for Lab 12.
 *  @author Mustafa Siddiqui
 *  @date   12/05/2020
 */

#include <xc.h>
#include <sys/attribs.h>
#include <string.h>         // strtok()
#include <stdlib.h>         // atof()
#include <stdio.h>          // sprintf()
#include <math.h>           // fabs()

#include "lcd_display_driver.h"

/*  Configure CPU and peripheral bus clock to run at 80MHz
    f_crystal = 8MHz
    f_CPU = (f_crystal * FPLLMUL) / (FPLLIDIV * FPLLODIV)
    f_peripheral = f_CPU / FPBDIV
 */
#pragma config POSCMOD = HS         // primary oscillator mode: high speed crystal
#pragma config FNOSC = PRIPLL       // oscillator selection: primary with PLL
#pragma config FPLLMUL = MUL_20     // PLL multiplier - multiply by 20
#pragma config FPLLIDIV = DIV_2     // PLL input divider - divide by 2
#pragma config FPLLODIV = DIV_1     // PLL output divider - divide by 1
#pragma config FPBDIV = DIV_1       // peripheral bus clock - divide by 1

#define MAX_SHORT 32767

/*  Global Variables */
volatile float gamepad_x = 0.0;
volatile float gamepad_y = 0.0;

/*  Read a character array sent to slave SPI4 from master SPI.
    Character Array is passed by reference; the function doesn't return anything.
 */
void readSPI4(char* msg, unsigned int length) {
    char data;
    unsigned int complete = 0;
    unsigned int numBytes = 0;

    while (!complete) {
        // if data is available
        if (SPI4STATbits.SPIRBF) {
            data = SPI4BUF;
            if (data == '\n') {
                complete = 1;
            }
            else {
                msg[numBytes] = data;
                numBytes++;

                // rollover if char array not big enough
                if (numBytes > length) {
                    numBytes = 0;
                }

                // clear overflow bit to enable receival
                SPI4STATbits.SPIROV = 0;
            }
        }
    }
    msg[numBytes] = '\0';

    // clear status bits (receive buffer full & overflow)
    SPI4STATbits.SPIRBF = 0;
    SPI4STATbits.SPIROV = 0;
}

/*  Initialize register for output compare 4 to be used with timer 3. Duty 
    cycle is 100% and the period is set to be of 200 us. Output compare is used
    to produce Pulse Width Modulated (PWM) signals to control the motor.
 */
void OC4_init(void) {
    __builtin_disable_interrupts();

    OC4CONbits.OCM = 0b110;         // PWM mode with no fault pin
    OC4CONbits.OCTSEL = 1;          // Use Timer 3
    
    // period = (PR3+1)*N*12.5ns = 200 us, N = 8 (pre-scalar)
    PR3 = 1022;                     // PWM duty cycle between 0 & 1023
    TMR3 = 0;                       // initial timer count = 0
    T3CONbits.TCKPS = 0b011;        // 1:8 pre-scalar timer
    
    OC4R = 0;                       // initialize before turning OC4 on
    OC4RS = 0;                      // duty cycle = OC4RS / (PR3 + 1)
    
    OC4CONbits.ON = 1;              // turn on output compare 4
    T3CONbits.ON = 1;               // turn on timer 3

    __builtin_enable_interrupts();
}

/*  Initialize SPI4 as slave in 8 bit mode.
    Initialize SPI interrupt upon receival.

    Configure bits:
    SDI4 -> p49 (RF4)
    SDO4 -> p50 (RF5)
    SCK4 -> p39 (RF13)
 */
void SPI4_slaveInit(void) {
    // SDO4 as output, rest are input by default
    TRISFbits.TRISF5 = 0;
    
    __builtin_disable_interrupts();

    // clear the rx buffer and overflow
    SPI4BUF;
    SPI4STATbits.SPIROV = 0;
    
    // clock polarity and clock edge
    SPI4CONbits.CKP = 0;
    SPI4CONbits.CKE = 0;
    
    // 8 bit mode
    SPI4CONbits.MODE32 = 0;
    SPI4CONbits.MODE16 = 0;
    
    // set up SPI interrupt
    INTCONbits.MVEC = 1;            // enable multi-vector interrupts
    IFS1bits.SPI4RXIF = 0;          // clear receive status bit
    IEC1bits.SPI4RXIE = 1;          // enable interrupt on receive
    IPC8bits.SPI4IP = 6;            // priority = 6
    IPC8bits.SPI4IS = 3;            // sub-priority = 3
    
    // turn on SPI module in slave mode
    SPI4CONbits.MSTEN = 0;
    SPI4CONbits.ON = 1;
    
    __builtin_enable_interrupts();
}

/*  SPI4 ISR: Receive string in the form GL:XXXXXX\n where L = X or Y
    signifying an x-coordinate or a y-coordinate. Parses the string and
    converts the string to a short value, normalizes it such max and min
    values are 1.0 and -1.0 and stores it in the appropriate global variables.
 */
void __ISR(_SPI_4_VECTOR, IPL6SOFT) SPI4_ISR(void) {
    // check if receival
    if (IFS1bits.SPI4RXIF) {
        // received in the format: GL:XXXXXX\n, L = X or Y
        char receiveMsg[10];
        readSPI4(receiveMsg, 10);

        // split string and get numeric value
        char* token = strtok(receiveMsg, ":");
        short coordinate = (short)atof(strtok(NULL, ":"));
        if (*(token + 1) == 'X') {
            gamepad_x = coordinate / MAX_SHORT;
        }
        else if (*(token + 1) == 'Y') {
            gamepad_y = coordinate / MAX_SHORT;
        }

        // clear receive status bit
        IFS1bits.SPI4RXIF = 0;
    }
}

/*  Initialize Timer 2 interrupt to trigger at 15 Hz to update LCD Display 
 */
void TMR2_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector interrupts
    __builtin_disable_interrupts();
    
    // Timer 2 interrupt is configured to trigger at a frequency of 15 Hz
    // PR2 = ((1/15)/(12.5 ns)) / 256 = 20834
    PR2 = 20834;
    TMR2 = 0;                   // start at 0
    T2CONbits.ON = 1;           // enable timer 2
    T2CONbits.TCKPS = 0b111;    // or 0x7; pre-scalar set to 256
    IPC2bits.T2IP = 2;          // priority = 2
    IPC2bits.T2IS = 0;          // sub-priority = 0
    IFS0bits.T2IF = 0;          // clear status flag
    IEC0bits.T2IE = 1;          // enable interrupt
    
    __builtin_enable_interrupts();
}

/*  TMR2 ISR: Converts the x and y coordinates into a string and
    displays them on the first line of the LCD display.
 */
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_ISR(void) {
    // convert coordinates into string
    char coordDisplay[20] = {};
    int length = sprintf(coordDisplay, " X:%5.2f Y:%5.2f", gamepad_x, gamepad_y);

    // display on lcd display
    lcd_display_driver_clear();
    delay_1us(5000);            // 5 ms delay
    display_driver_use_first_line();

    lcd_display_driver_write(coordDisplay, length);

    // clear status flag
    IFS0bits.T2IF = 0;
}

/*  Main function: Initialize the LCD, Output Compare 4, SPI 4 as slave, 
    and Timer 2. Controls the direction and speed of the motor according
    to the y-coordinate value received by the joystick.
 */
int main(void) {
    // disable debugging through JTAG
    DDPCONbits.JTAGEN = 0;

    // set pins as digital outputs
    TRISFbits.TRISF1 = 0;       // p88
    TRISFbits.TRISF0 = 0;       // p87

    // set up LCD Display
    lcd_display_driver_set_tristate();
    lcd_display_driver_initialize();

    // initialization
    OC4_init();
    SPI4_slaveInit();
    TMR2_init();

    // infinite loop
    while (1) {
        // move motor according to y-movement of joystick
        if (gamepad_y < -0.1) {
            // move counter-clockwise
            LATFbits.LATF0 = 1;
            LATFbits.LATF1 = 0;

        }
        else if (gamepad_y > 0.1) {
            // move clockwise
            LATFbits.LATF0 = 0;
            LATFbits.LATF1 = 1;
        }
        else {
            // no rotation
            LATFbits.LATF0 = 0;
            LATFbits.LATF1 = 0;
        }

        // set duty cycle of motor according to y-coordinate
        // duty cycle = 1023 * y-coordinate
        // y = 0, pwm = 0; y = 1, pwm = 1023
        float pwm = fabs(1023.0 * gamepad_y);
        OC4RS = (int)pwm;
    }

    return 0;
}