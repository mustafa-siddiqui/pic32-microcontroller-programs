/** @file   main.c
 *  @brief  This program displays the counts of an encoder and the angle in
 *          degrees of the wheel attached to the output shaft of the DC
 *          gear motor. Updates the LCD Display at a frequency of 15 Hz.
 *  @author Mustafa Siddiqui
 *  @date   10/27/2020 
 */

/* Include the necessary libraries */
#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>      // sprintf()
//-//
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

/* Global Variables */
/*  Variable to store the number of counts for a net rotation of the encoder
    Can be positive for a net positive rotation and negative for a net negative
    rotation
 */
static volatile int NumCount = 0;

/*  Structure to hold data from encoder 
    Holds the current and previous values of encoder A output and the current
    value of the encoder B output 
*/
struct encoderPosition {
    volatile unsigned int prev_A;
    volatile unsigned int curr_A;
    volatile unsigned int curr_B;
} typedef position;

/* declare and initialize global struct */
position encoderPos = {0,0,0};

/*  Function: CN_init()
    Initializes change notification interrupts for CN8 and CN9 which correspond
    to the pins connected to the outputs of the encoder
    Sets the priority of CN interrupts to 5 which is higher than the priority of
    the timer 2 interrupt
*/
void CN_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector mode
    __builtin_disable_interrupts();
    
    // enable internal pull-up resistors
    CNPUEbits.CNPUE8 = 1;      // CN8
    CNPUEbits.CNPUE9 = 1;      // CN9
    
    IFS1bits.CNIF = 0;          // clear interrupt status flag
    IEC1bits.CNIE = 1;          // set interrupt enable flag
    IPC6bits.CNIP = 5;          // set priority = 5
    IPC6bits.CNIS = 2;          // set sub-priority = 2
    
    CNCONbits.ON = 1;           // enable change notification interrupts
    CNENbits.CNEN8 = 1;        // CN8 - encoder A output
    CNENbits.CNEN9 = 1;        // CN9 - encoder B output
    
    __builtin_enable_interrupts();
}

/*  ISR to implement the finite state machine
    State machine such that a clockwise rotation of the encoder will result in 
    the counts being negative and an anti-clockwise rotation will result in the
    counts being positive. Each state change results in a change of 1 count.
    ISR gets triggered when a change in value is noticed at either CN8 or CN9.
 */
void __ISR(_CHANGE_NOTICE_VECTOR, IPL5SOFT) CN_ISR(void) {
    
    // get current value at pins
    encoderPos.curr_A = PORTGbits.RG6;
    encoderPos.curr_B = PORTGbits.RG7;
    
    // Since the FSM is implemented by the ISR of the change notification 
    // interrupts, there is no need to explicitly account for no state change
    // like 00 to 00 since the interrupt is only triggered when there is a
    // change in encoder A or B output
    if (encoderPos.prev_A == encoderPos.curr_A) {
        // 01 to 00 & 10 to 11
        if (encoderPos.curr_B == encoderPos.curr_A) {
            NumCount--;
        }
        // 00 to 01 & 11 to 10
        else {
            NumCount++;
        }
    }
    else {
        // 10 to 00 & 01 to 11
        if (encoderPos.curr_B == encoderPos.curr_A) {
            NumCount++;
        } 
        // 11 to 01 & 00 to 10
        else {
            NumCount--;
        }
    }
    encoderPos.prev_A = encoderPos.curr_A;
    
    // clear interrupt status flag
    IFS1bits.CNIF = 0;
}

/*  Function: TMR2_init()
    Initializes the timer 2 interrupt such that the ISR is triggered at a 
    frequency of 15 Hz (so after every 1/15 seconds). Sets the pre-scalar and 
    PR2 values. Sets the priority of the timer to 3 - which is lower than that
    of the change notification interrupt so as to not to miss any counts.
*/
void TMR2_init(void) {
    INTCONbits.MVEC = 1;    // enable multi-vector interrupts
    __builtin_disable_interrupts();
    
    // Timer 2 interrupt is configured to trigger at a frequency of 15 Hz
    // PR2 = ((1/15)/(12.5 ns)) / 256 = 20834
    PR2 = 20834;
    TMR2 = 0;                   // start at 0
    T2CONbits.ON = 1;           // enable timer 2
    T2CONbits.TCKPS = 0b111;   // or 0x7; pre-scalar set to 256
    IPC2bits.T2IP = 3;          // priority = 3
    IPC2bits.T2IS = 1;          // sub-priority = 1
    IFS0bits.T2IF = 0;          // clear status flag
    IEC0bits.T2IE = 1;          // enable interrupt
    
    __builtin_enable_interrupts();
} 

/*  ISR for Timer 2 interrupts 
    Calculates the angle of the wheel attached to the output shaft of the DC 
    gear motor using the number of counts, gear-train, and CPR of the encoder.
    Prints the number of counts of the encoder and the angle (upto 2 decimal 
    places) on the LCD Display.
*/
void __ISR(_TIMER_2_VECTOR, IPL3SOFT) TMR2_ISR(void) {
    
    // calculate the angle using the number of counts and certain relations
    // encoder has a count per revolution (CPR) of 48
    // shaft is connected to the motor and encoder with a gear-train of 99:1
    // angle = ((# of counts / 48) * 360) / 99 
    // angleDecimal stores the decimal part of the string (upto 2 d.p.)
    float angle = (float)((NumCount / 48) * (360 / 98.78));
    int angleDecimal = abs((int)(100 * (angle - (int)angle)));
    
    // convert data into a string to be printed
    char stringCount[20];
    int stringCount_length = sprintf(stringCount, " Count: %d", NumCount);
    char stringAngle[20];
    int stringAngle_length = sprintf(stringAngle, " Angle: %d.%d%c", (int)angle, angleDecimal, 0xDF);
    
    // clear display
    lcd_display_driver_clear();
    
    // write number of counts on the first line of the display
    display_driver_use_first_line();
    lcd_display_driver_write(stringCount, stringCount_length);
    
    // write the angle on the second line of the display
    display_driver_use_second_line();
    lcd_display_driver_write(stringAngle, stringAngle_length);
    
    // clear status flag
    IFS0bits.T2IF = 0;
}

/* Main function of the program */
int main(void) {
    
    // disable JTAG debugging
    DDPCONbits.JTAGEN = 0;
    
    // set tri-state registers for LCD Display
    TRISE = 0xFF00;         // set RE to be output: b7-b0 = 0
    TRISDbits.TRISD4 = 0;   // enable
    TRISDbits.TRISD5 = 0;   // RW
    TRISBbits.TRISB15 = 0;  // RS
    
    // set tri-state registers for encoder input (explicitly)
    // default is already 1
    TRISGbits.TRISG6 = 1;   // set as input for encoder A output
    TRISGbits.TRISG7 = 1;   // set as input for encoder B output
    
    // initialize LCD Display
    lcd_display_driver_initialize();
    
    // initialize Change Notification & Timer interrupts
    CN_init();
    TMR2_init();
    
    // assign encoder outputs to struct
    encoderPos.curr_A = PORTGbits.RG6;
    encoderPos.curr_B = PORTGbits.RG7;
    
    // infinite loop
    while (1) {}
    
    return 0;
}
