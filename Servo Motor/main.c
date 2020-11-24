/** @file   main.c
 *  @brief  Program to interact with a Pololu Power HD High-Torque Servo 1501 MG
 *          using a Pololu Maestro 6-Channel USB Servo Controller. The pulse 
 *          width of the PWM signal is calculated with the potentiometer. The 
 *          potentiometer position being all the way to the left results in the 
 *          pulse width being 1000 microseconds while all the way to the right
 *          results in the pulse width being 2000 microseconds. This results in
 *          a total revolution of about 45 degrees of the servo motor. The 
 *          Pololu Protocol sequence is displayed on the LCD Display on the 
 *          16/32 Explorer Board in hexadecimal.
 *  @author Mustafa Siddiqui
 *  @date   11/23/2020
 */

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>          // sprintf()

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

#define F_PB 80000000               // 80 MHz
#define BAUD_DESIRED 9600

/*  Global: Store Pololu protocol. 
    Pololu protocol: 0xAA, device #, 0x04, channel #, target low bits, target 
    high bits
 */
volatile unsigned int serialBytes[6] = {0};

/*  Sample and convert the value on the given ADC pin.
    Reading can vary from 0 to 1023.
 */
unsigned short adcSampleConvert(int pin) {
    unsigned int wait_time = 0;
    
    AD1CON3bits.ADCS = 2;                   // since running at 80MHz
    AD1CHSbits.CH0SA = pin;                 // connect pin
    AD1CON1bits.SAMP = 1;                   // start sampling
    
    // core timer increments once every 2 cycles of SYSCLK
    // 1 cycle = 12.5 ns -> 2 cycles = 25 ns
    wait_time = _CP0_GET_COUNT() + 200;     // set wait_time to 5000 ns
    while (_CP0_GET_COUNT() < wait_time) {} // wait for 5000 ns
    
    AD1CON1bits.SAMP = 0;                   // stop sampling & start converting
    while (!AD1CON1bits.DONE) {}            // wait for conversion process to finish
    
    unsigned short reading = ADC1BUF0;
    
    return reading;                         // return the buffer with result
}

/*  Publish a character (8 bits or 1 byte) with UART2.
 */
void writeUART(unsigned char data) {
    // wait until TX buffer is not full
    while (U2STAbits.UTXBF) {}
    U2TXREG = data;
}

/*  Initialize UART2 to run such as: 
    - 8 bits, no parity
    - 1 stop bit
    - baud rate: 9600
 */
void UART2_init(void) {
    // no hardware flow control, use pins U2TX and U2RX
    U2MODEbits.UEN = 0b00;
    
    // 8 bits, no parity
    U2MODEbits.PDSEL = 0b00;
    
    // 1 stop bit
    U2MODEbits.STSEL = 0;
    
    // UxBRG = (F_pb / (M * BaudRate)) - 1
    // M is 4 or 16 if BRGH is 1 or 0
    U2MODEbits.BRGH = 0;
    U2BRG = ((F_PB / BAUD_DESIRED) / 16) - 1;
    
    // configure TX, RX as output, input pins
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
    
    // enable UART
    U2MODEbits.ON = 1;
}

/*  Initialize Timer 5 interrupt to trigger at 10 Hz to publish PWM through UART.
 */
void TMR5_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector interrupts
    __builtin_disable_interrupts();
    
    // Timer 5 interrupt is configured to trigger at a frequency of 10 Hz
    // PR5 = ((1/10)/(12.5 ns)) / 256 = 31250
    PR5 = 31250;
    TMR5 = 0;                   // start at 0
    T5CONbits.ON = 1;           // enable timer 5
    T5CONbits.TCKPS = 0b111;    // or 0x7; pre-scalar set to 256
    IPC5bits.T5IP = 3;          // priority = 3
    IPC5bits.T5IS = 1;          // sub-priority = 1
    IFS0bits.T5IF = 0;          // clear status flag
    IEC0bits.T5IE = 1;          // enable interrupt
    
    __builtin_enable_interrupts();
}

/*  Publish PWM to servo motor controller using UART2.
    PWM is controlled by the potentiometer such that a potentiometer reading of
    0 gives a PWM of 1000 and a reading of 1023 gives a PWM of 2000:
    PWM = 1000/1023 * pReading + 1000
 
    Pololu protocol contains the bytes below in that order:
    0xAA, device #, 0x04, channel #, target low bits, target high bits
    Channel 0 is used.
 */
void __ISR(_TIMER_5_VECTOR, IPL3SOFT) TMR5_ISR(void) {
    // get reading from potentiometer
    unsigned short pReading = adcSampleConvert(2);
    
    // y = mx + c <=> pwm = (1000/1023) * pReading + 1000
    // PWM can vary from 1000 to 2000 microseconds
    unsigned short pulseWidth = (unsigned short)(pReading * (float)(1000.0/1023.0) + 1000);
    
    // convert PWM to target high and low bits
    pulseWidth *= 4;
    serialBytes[0] = 0xAA;                      // first command byte
    serialBytes[1] = 0x0C;                      // device number
    serialBytes[2] = 0x04;                      // second command byte
    serialBytes[3] = 0x00;                      // channel number
    serialBytes[4] = pulseWidth & 0x7F;         // target low bits (0-6)
    serialBytes[5] = (pulseWidth >> 7) & 0x7F;  // target high bits (7-13)
    
    // publish with UART2
    int i;
    for (i = 0; i < 6; i++) {
        writeUART((unsigned char)serialBytes[i]);
        delay_1us(50);
    }
    
    // clear status flag
    IFS0bits.T5IF = 0;
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

/*  Update LCD Display with hexadecimal representation of the Pololu protocol 
 */
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_ISR(void) {
    char string[20];
    int length = sprintf(string, " %X %X %X %X %X %X",  serialBytes[0], 
                                                        serialBytes[1], 
                                                        serialBytes[2], 
                                                        serialBytes[3], 
                                                        serialBytes[4], 
                                                        serialBytes[5]);
    
    lcd_display_driver_clear();
    delay_1us(5000);            // 5 ms delay
    display_driver_use_first_line();
    
    lcd_display_driver_write(string, length);
    
    // clear status flag
    IFS0bits.T2IF = 0;
}

/* Main function of the program */
int main(void) {
    // disable debugging through JTAG
    DDPCONbits.JTAGEN = 0;
    
    // set up LCD Display
    lcd_display_driver_set_tristate();
    lcd_display_driver_initialize();
    
    // initialize timers and UART
    TMR2_init();
    TMR5_init();
    UART2_init();
    
    // set registers for analog to digital conversion
    AD1PCFGbits.PCFG2 = 0;  // AN2 is an ADC pin
    AD1CON1bits.ADON = 1;   // turn on the ADC
    
    // infinite loop
    while (1) {}
    
    return 0;
}
