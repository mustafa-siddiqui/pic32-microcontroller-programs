/** @file   main.c
 *  @brief  Program to control the direction of the motor using a simple
 *          mechanism such that the current angle of the motor shaft is close
 *          to the target angle (within 7.5 degrees) while the target is set by
 *          the user by pressing the push buttons SW4 and SW5. The duty cycle 
 *          of the motor is set by the position of the potentiometer on the
 *          16/32 Explorer Board.
 *  @author Mustafa Siddiqui
 *  @date   10/30/2020
 */

/* Include necessary libraries */
#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>                  // sprintf()
//--//
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

/*  Reference input that the motor will close a loop on */
static volatile int TargetCount = 0;

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

/*  Function that causes a delay of about 50 ms to debounce the switch. The
    time is a reasonable estimate for this application (user pressing the 
    buttons)
 */
void debounceSwitch(void) {
    // delay for 50 ms
    delay_1us(50000);
}

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
    IPC6bits.CNIP = 6;          // set priority = 6
    IPC6bits.CNIS = 3;          // set sub-priority = 3
    
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
void __ISR(_CHANGE_NOTICE_VECTOR, IPL6SOFT) CN_ISR(void) {
    // get current values of encoder outputs A and B
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
    INTCONbits.MVEC = 1;        // enable multi-vector interrupts
    __builtin_disable_interrupts();
    
    // Timer 2 interrupt is configured to trigger at a frequency of 15 Hz
    // PR2 = ((1/15)/(12.5 ns)) / 256 = 20834
    PR2 = 20834;
    TMR2 = 0;                   // start at 0
    T2CONbits.ON = 1;           // enable timer 2
    T2CONbits.TCKPS = 0b111;    // or 0x7; pre-scalar set to 256
    IPC2bits.T2IP = 3;          // priority = 3
    IPC2bits.T2IS = 0;          // sub-priority = 0
    IFS0bits.T2IF = 0;          // clear status flag
    IEC0bits.T2IE = 1;          // enable interrupt
    
    __builtin_enable_interrupts();
} 

/*  ISR for Timer 2 interrupts. Prints the current count and the target count 
    equivalent angles on the LCD Display upto 2 decimal places.
    If the angle difference between current and target count is within 7.5 
    degrees, the motor is stopped. Otherwise, move the motor such that the 
    difference between target and count angle is reduced. 
    It also reads the potentiometer and sets OC4RS to the reading value. Hence,
    the motor speed is controlled by the movement of the potentiometer. This
    results in a 100% duty cycle if the potentiometer is all the way to the
    right and 0% if it is moved all the way to the left. This is because the 
    duty cycle is defined as OCxRS/(PRy+1) which translates to the amount of
    time under load over total time of the cycle.
 */
void __ISR(_TIMER_2_VECTOR, IPL3SOFT) TMR2_ISR(void) {
    // update the display at a frequency of 15 Hz
    // write the current angle on the first line of the display and the target
    // angle on the second line of the display
    float angleCount = (NumCount / 48.00) * (360 / 98.78);
    float angleTarget = (TargetCount / 48.00) * (360 / 98.78);
        
    char stringCount[20] = "";
    int stringCount_len = sprintf(stringCount, " Current: %.2f%c", angleCount, 0xDF);
        
    char stringTarget[20] = "";
    int stringTarget_len = sprintf(stringTarget, " Target: %.2f%c", angleTarget, 0xDF);
        
    lcd_display_driver_clear();
        
    display_driver_use_first_line();
    lcd_display_driver_write(stringCount, stringCount_len);
        
    display_driver_use_second_line();
    lcd_display_driver_write(stringTarget, stringTarget_len);
        
    // if number of counts between target count and numCount is within the
    // angle of 7.5 degrees, stop the motor
    // angle of 7.5 degrees translates to 99 counts
    // numCount = (angle * 48 * 99) / 360
    if (abs(TargetCount - NumCount) <= 99) {
        // stop the motor
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 0;
    }
    // set the motor to move such that the difference between actual number of
    // counts and the target count decreases
    else if (NumCount < TargetCount) {
        // increase number of counts; move clockwise
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 1;
    }
    else if (NumCount > TargetCount) {
        // decrease number of counts; move anti-clockwise
        LATFbits.LATF0 = 1;
        LATFbits.LATF1 = 0;
    }
    
    // read the potentiometer value and assign to OC4RS
    // this sets the duty cycle accordingly
    // the higher the reading, the more % of the duty cycle
    int pReading = read_potentiometer();
    OC4RS = pReading;
    
    // clear status flag
    IFS0bits.T2IF = 0;
}

/*  Function to initialize registers for external interrupts 1 and 2. These
    interrupts are to respond to pushes of buttons SW4 and SW5 by the user. Pins
    18 and 19 of the interrupts are connected with jumper wires to pins 80 and 
    92 which correspond to SW4 and SW5 respectively.
 */
void ExtInt_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector mode
    __builtin_disable_interrupts();
    INTCONbits.INT2EP = 1;      // rising edge trigger INT2
    INTCONbits.INT1EP = 1;      // rising edge trigger INT1 
    
    IPC2bits.INT2IP = 4;        // priority = 4 INT2
    IPC2bits.INT2IS = 2;        // sub-priority = 2
    
    IPC1bits.INT1IP = 5;        // priority = 5 INT1
    IPC1bits.INT1IS = 1;        // sub-priority = 1
    
    IFS0bits.INT2IF = 0;        // clear status flag for interrupt 2
    IEC0bits.INT2IE = 1;        // enable interrupt 2
    
    IFS0bits.INT1IF = 0;        // clear status flag for interrupt 1
    IEC0bits.INT1IE = 1;        // enable interrupt 1
    __builtin_enable_interrupts();
}

/*  ISR for external interrupt 2 (SW5). Calls debounce() to wait for the switch
    to debounce. The target count is increased by an equivalent of 90 degrees 
    when SW5 is pressed.
 */
void __ISR(_EXTERNAL_2_VECTOR, IPL4SOFT) INT2_ISR(void) {
    debounceSwitch();
    
    // increase target count by an equivalent of 90 degrees
    TargetCount += 1188;
    
    // clear status flag
    IFS0bits.INT2IF = 0;
}

/*  ISR for external interrupt 1 (SW4). Calls debounce() to wait for the switch
    to debounce. The target is decreased by an equivalent of 90 degrees when
    SW4 is pressed.
 */
void __ISR(_EXTERNAL_1_VECTOR, IPL5SOFT) INT1_ISR(void) {
    debounceSwitch();
    
    // decrease target count by an equivalent of 90 degrees
    TargetCount -= 1188;
    
    // clear status flag
    IFS0bits.INT1IF = 0;
}

/*  Function to initialize register for output compare 4 to be used with 
    timer 3. Duty cycle is 100% and the period is set to be of 200 us. Output 
    compare is used to produce Pulse Width Modulated (PWM) signals to control 
    the motor.
 */
void OC4_init(void) {
    __builtin_disable_interrupts();

    OC4CONbits.OCM = 0b110;         // PWM mode with no fault pin
    OC4CONbits.OCTSEL = 1;          // Use Timer 3
    
    // period = (PR3+1)*N*12.5ns = 200 us, N = 8 (pre-scalar)
    PR3 = 1022;                     // PWM duty cycle between 0 & 1023
    TMR3 = 0;                       // initial timer count = 0
    T3CONbits.TCKPS = 0b011;        // 1:8 pre-scalar timer
    
    OC4R = 1023;                    // initialize before turning OC4 on
    OC4RS = 1023;                   // duty cycle = OC4RS / (PR3 + 1) = 100%
    
    OC4CONbits.ON = 1;              // turn on output compare 4
    T3CONbits.ON = 1;               // turn on timer 3

    __builtin_enable_interrupts();
}

/*  Function to read analog readings of the potentiometer on the Explorer Board.
    Analog to Digital Conversion is done through successive approximation.
 */
int read_potentiometer(void) {
    unsigned int wait_time = 0;
    
    AD1CON3bits.ADCS = 2;                   // since running at 80MHz
    AD1CHSbits.CH0SA = 2;                   // connect pin AN2 for potentiometer
    AD1CON1bits.SAMP = 1;                   // start sampling
    
    // core timer increments once every 2 cycles of SYSCLK
    // 1 cycle = 12.5 ns -> 2 cycles = 25 ns
    wait_time = _CP0_GET_COUNT() + 200;     // set wait_time to 5000 ns
    while (_CP0_GET_COUNT() < wait_time) {} // wait for 5000 ns
    
    AD1CON1bits.SAMP = 0;                   // stop sampling & start converting
    while (!AD1CON1bits.DONE) {}            // wait for conversion process to finish
    
    return ADC1BUF0;                        // return the buffer with result
}

/* Main function of the program */
int main(void) {
    // disable debugging through JTAG
    DDPCONbits.JTAGEN = 0;
    
    // set F0 and F1 as outputs to control motor direction
    TRISFbits.TRISF0 = 0;
    TRISFbits.TRISF1 = 0;
    
    // set push buttons S5 and S4 as inputs explicitly
    TRISAbits.TRISA7 = 0x1;     // S5
    TRISDbits.TRISD13 = 0x1;    // S4
    
    // set encoder outputs A and B as inputs explicitly
    TRISGbits.TRISG6 = 1;
    TRISGbits.TRISG7 = 1;
    
    // set tri-state registers for LCD Display
    lcd_display_driver_set_tristate();
    
    // initialize LCD Display
    lcd_display_driver_initialize();
    
    // initialize interrupts
    CN_init();
    TMR2_init();
    ExtInt_init();
    
    // initialize output compare and timer 3
    OC4_init();
    
    // set registers for analog to digital conversion for potentiometer
    AD1PCFGbits.PCFG2 = 0;  // AN2 is an ADC pin
    AD1CON1bits.ADON = 1;   // turn on the ADC
    
    // initialize struct to current values of encoder outputs A and B
    encoderPos.curr_A = PORTGbits.RG6;
    encoderPos.curr_B = PORTGbits.RG7;

    // initialize motor direction (no rotation)
    LATFbits.LATF0 = 0;
    LATFbits.LATF1 = 0;
    
    // infinite loop
    while (1) {
        ;
    }
    
    return 0;
}
