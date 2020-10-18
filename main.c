/** @file   main.c
 *  @brief  Code for Lab3.
 *          Emulates a digital combination lock by using the push buttons on
 *          the 16/32 Explorer Board to increment letters on the display. This
 *          process happens with the use of interrupts. If the string on the 
 *          display matches 'MUS', the four LEDs are turned on.
 *  @author Mustafa Siddiqui
 *  @date   09/27/2020
 */

/* Include header files */
#include <xc.h>
#include <sys/attribs.h>
//--//
#include "lcd_display_driver.h"

/* Global variable for storing the string to be printed */
char string[4];

/** @fn     changeNotifications_init()
 *  @brief  Carries out the initialization process for the change notification
 *          interrupts used for push buttons S3, S4, & S6.
 */
void changeNotifications_init(void);

/** @fn     changeNotif_ISR()
 *  @brief  Interrupt Service Routine for change notification interrupts.
 */
void __ISR(26, IPL3SOFT) changeNotif_ISR(void);

/** @fn     INT2_init()
 *  @brief  Carries out the initialization process for the external interrupt 2
 *          used for push button S5 - which is originally connected to PIN92
 *          but is connected to PIN19 using a jumper so that it can be used with
 *          an interrupt.
 */
void INT2_init(void);

/** @fn     INT2_ISR()
 *  @brief  Interrupt Service Routine for external interrupt 2.
 */
void __ISR(11, IPL3SOFT) INT2_ISR(void);

/*  @fn     main.c
 *  @brief  Main function of the program.
 *          Sets the respective peripherals as inputs or outputs, and 
 *          initializes the interrupts. Checks to see if the string is equal to
 *          'MUS' - the first three letters of my name - and lights up the four
 *          LEDs if true.
 *  @return An integer when successfully run.
 */
int main(void) {
    
    DDPCONbits.JTAGEN = 0;
    
    // set LEDs D3-D6 as output
    TRISA = !0xF;
    
    // set the push buttons as inputs
    TRISDbits.TRISD7 = 0x1;     // S6
    TRISDbits.TRISD6 = 0x1;     // S3
    TRISAbits.TRISA7 = 0x1;     // S5
    TRISDbits.TRISD13 = 0x1;    // S4
     
    // for LCD Display
    TRISE = !0xFF;          // set RE to be output: b7-b0 = 0
    TRISDbits.TRISD4 = 0;   // enable
    TRISDbits.TRISD5 = 0;   // RW
    TRISBbits.TRISB15 = 0;  // RS
    
    // initialize LCD Display
    lcd_display_driver_initialize();
    
    // clear display
    lcd_display_driver_clear();
    
    // initialize interrupts
    changeNotifications_init();
    INT2_init();
    
    // initialize string to 'AAA' and print
    int i;
    for (i = 0; i < 3; i++) {
        string[i] = 'A';
    }
    lcd_display_driver_write(string, 3);
    
    // infinite loop
    while(1) {
        if (*string == 'M'&& 
            *(string + 1) == 'U' && 
            *(string + 2) == 'S') {
            // illuminate the LEDs configured as output
            LATA = 0xF;
        }
        else {
            // turn off the LEDs
            LATA = 0x0;
        }
    }
    
    return 0;
}

/* initialize change notifications */
void changeNotifications_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector mode
    __builtin_disable_interrupts();
    
    // enable internal pull-up resistors
    CNPUEbits.CNPUE15 = 1;      // S3
    CNPUEbits.CNPUE16 = 1;      // S6
    CNPUEbits.CNPUE19 = 1;      // S4
    
    IFS1bits.CNIF = 0;          // clear interrupt status flag
    IEC1bits.CNIE = 1;          // set interrupt enable flag
    IPC6bits.CNIP = 3;          // set priority = 3
    IPC6bits.CNIS = 2;          // set sub-priority = 2
    
    CNCONbits.ON = 1;           // enable change notification interrupts
    CNENbits.CNEN15 = 1;        // CN15 - S3
    CNENbits.CNEN19 = 1;        // CN19 - S4
    CNENbits.CNEN16 = 1;        // CN16 - S6
    
    __builtin_enable_interrupts();
}

/* ISR for change notification interrupts */
void __ISR(26, IPL3SOFT) changeNotif_ISR(void) {
    // S6 is pressed
    // increment first character of the string
    if (PORTDbits.RD7 == 0) {
        string[0] += 1;
        string[0] = (string[0] > 'Z') ? 'A' : string[0];
        lcd_display_driver_write(string, 3);
    }
    // S4 is pressed
    // increment third character of the string
    else if (PORTDbits.RD13 == 0) {
        string[2] += 1;
        string[2] = (string[2] > 'Z') ? 'A' : string[2];
        lcd_display_driver_write(string, 3);
    }
    // S3 is pressed
    // reset the display to 'AAA'
    else if (PORTDbits.RD6 == 0) {
        int i;
        for (i = 0; i < 3; i++) {
            string[i] = 'A';
        }
        lcd_display_driver_write(string, 3);
    }
    
    IFS1bits.CNIF = 0;          // clear status bit
}

/* initialize external interrupt #2 */
void INT2_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector mode
    __builtin_disable_interrupts();
    INTCONbits.INT2EP = 1;      // rising edge trigger
    IPC2bits.INT2IP = 3;        // priority = 3
    IPC2bits.INT2IS = 2;        // sub-priority = 2
    IFS0bits.INT2IF = 0;        // clear status flag
    IEC0bits.INT2IE = 1;        // enable interrupt
    __builtin_enable_interrupts();
}

/* ISR for interrupt 2 */
void __ISR(11, IPL3SOFT) INT2_ISR(void) {
    // when switch S5 is pressed
    // increment second character of the string & write to display
    string[1] += 1;
    string[1] = (string[1] > 'Z') ? 'A' : string[1];
    lcd_display_driver_write(string, 3);
    
    IFS0bits.INT2IF = 0;        // clear status bits
}