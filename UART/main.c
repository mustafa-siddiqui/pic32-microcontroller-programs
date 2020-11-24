/** @file   main.c
 *  @brief  Program that implements a PI controller to control a DC motor. Target Angle is set
 *          by receiving a decimal with UART. The target angle is sent by a MATLAB script.
 *          The program also records the transient response of the motor and then transmits the
 *          data to MATLAB which is then plotted.
 *  @author Mustafa Siddiqui
 *  @date   11/14/2020
 */

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>          // sprintf()
#include <stdlib.h>         // atof()
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

#define F_PB 80000000
#define BAUD_DESIRED 230400

/*  Global Variables */

static volatile int NumCount = 0;       // number of counts for a net rotation of the encoder
volatile float AngleCurrent = 0.0;      // current angle equivalent of current count

static volatile int TargetCount = 0;    // reference input that the motor will close a loop on
volatile float AngleTarget = 0.0;       // target angle equivalent of target count

float Error = 0;                        // error calculated for PI control
unsigned int PrevTime = 0;              // previous time for the PI control
float ErrorIntegral = 0.0;              // integral of the error used for PI control

char ReceivedTarget[10];                // receive target angle from MATLAB script
unsigned int Time = 0.0;                // time of transmission of data
unsigned int PrevTime_Trans = 0.0;      // previous time for transmission of data

unsigned int RECEIVED = 0;              // check if data is received before transmission
int TransmitCount = 0;                  // keep track of number of transmissions made

/*  Struct holds the current and previous values of encoder A output and the 
    current value of the encoder B output 
 */
struct encoderPosition {
    volatile unsigned int prev_A;
    volatile unsigned int curr_A;
    volatile unsigned int curr_B;
} typedef position;

/*  Declare and initialize global struct */
position encoderPos = {0,0,0};

/*  Initialize change notification interrupts for CN8 and CN9 which 
    correspond to the pins connected to the outputs of the encoder.
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
    CNENbits.CNEN8 = 1;         // CN8 - encoder A output
    CNENbits.CNEN9 = 1;         // CN9 - encoder B output
    
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
    
    // no need to explicitly account for no state change like 00 to 00 since 
    // the interrupt is only triggered when there is a change in encoder 
    // A or B output
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
    
    OC4R = 1023;                    // initialize before turning OC4 on
    OC4RS = 1023;                   // duty cycle = OC4RS / (PR3 + 1) = 100%
    
    OC4CONbits.ON = 1;              // turn on output compare 4
    T3CONbits.ON = 1;               // turn on timer 3

    __builtin_enable_interrupts();
}

/*  Initialize Timer 4 interrupt to trigger at 150 Hz. Sets the pre-scalar 
    and PR4 values.
 */
void TMR4_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector interrupts
    __builtin_disable_interrupts();
    
    // Timer 4 interrupt is configured to trigger at a frequency of 150 Hz
    // PR2 = ((1/150)/(12.5 ns)) / 256 = 2083.33
    PR4 = 2084;
    TMR4 = 0;                   // start at 0
    T4CONbits.ON = 1;           // enable timer 4
    T4CONbits.TCKPS = 0b111;    // or 0x7; pre-scalar set to 256
    IPC4bits.T4IP = 5;          // priority = 5
    IPC4bits.T4IS = 2;          // sub-priority = 2
    IFS0bits.T4IF = 0;          // clear status flag
    IEC0bits.T4IE = 1;          // enable interrupt
    
    __builtin_enable_interrupts();
} 

/*  ISR for Timer 4 interrupt
    Implement proportional integral control to adjust the the rotation and speed
    of the motor.
 */
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) TMR4_ISR(void) {
    // angle from the respective counts
    AngleCurrent = (NumCount / 48.00) * (360 / 98.78);
    AngleTarget = (TargetCount / 48.00) * (360 / 98.78);
    
    // error 
    Error = AngleTarget - AngleCurrent;
    
    // get time in seconds (multiply by 25*10^(-9): 25 ns)
    float time = _CP0_GET_COUNT() / 40000000;
    float elapsedTime = time - PrevTime;
    PrevTime = time;
    
    // integral = integral + error * dt
    ErrorIntegral = ErrorIntegral + (Error * (time - elapsedTime)); 
    
    // control variable = Kp * error + Ki * integral
    float pwm = (10 * Error) + (1 * ErrorIntegral);
    
    // limit pwm to +-1023
    if (pwm > 1023) {
        pwm = 1023;
    }
    else if (pwm < -1023) {
        pwm = -1023;
    }
    
    // control the rotation of the motor
    if (pwm > 0) {
        // move clockwise
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 1;
    }
    else if (pwm < 0) {
        // move counter-clockwise
        LATFbits.LATF0 = 1;
        LATFbits.LATF1 = 0;
    }
    else {
        // control variable = 0 so stop the motor
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 0;
    }
    
    // set duty cycle equal to absolute value of control variable
    OC4RS = fabs(pwm);
    
    // clear status flag
    IFS0bits.IC4IF = 0;
}

/*  Initialize timer 2 interrupt such that the ISR is triggered at 15 Hz 
    (so after every 1/15 seconds). Sets the pre-scalar and PR2 values. 
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

/*  ISR for Timer 2 interrupts. Prints the current count and the target count 
    equivalent angles on the LCD Display upto 2 decimal places at a frequency
    of 15 Hz.
 */
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_ISR(void) {
    float angleCount = (NumCount / 48.00) * (360 / 98.78);
    float angleTarget = (TargetCount / 48.00) * (360 / 98.78);
        
    char stringCount[20] = "";
    int stringCount_len = sprintf(stringCount, " Current: %.2f%c", angleCount, 0xDF);
        
    char stringTarget[20] = "";
    int stringTarget_len = sprintf(stringTarget, " Target: %.2f%c", angleTarget, 0xDF);
        
    lcd_display_driver_clear();
   
    // write the target angle on the first line of the display
    display_driver_use_first_line();
    lcd_display_driver_write(stringTarget, stringTarget_len);
    
    // write the current angle on the second line of the display
    display_driver_use_second_line();
    lcd_display_driver_write(stringCount, stringCount_len);
}

/*  Initialize UART2 to run in 8-bit mode, no parity, 1 stop bit, with baud 
    rate of 230400. UART is configured to trigger interrupt if receive buffer
    is not empty.
 */
void UART2_init(void) {
    __builtin_disable_interrupts();
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
    
    // configure to interrupt when receive buffer not empty
    U2STAbits.URXISEL = 0x0;
    IFS1bits.U2RXIF = 0;        // clear status flag
    IPC8bits.U2IP = 3;          // priority = 3
    IPC8bits.U2IS = 1;          // sub-priority = 1
    IEC1bits.U2RXIE = 1;        // enable RX interrupt
    
    // enable UART
    U2MODEbits.ON = 1;
    __builtin_enable_interrupts();
}

/*  ISR to receive data received at the U2RX pin through UART.
    Block other functions until data is completely received
 */
void __ISR(_UART_2_VECTOR, IPL4SOFT) UART2_RX_ISR(void) {
    // if interrupt caused by RX event
    if (IFS1bits.U2RXIF) {
        int complete = 0;
        char data;
        int index = 0;
        
        while (!complete) {
            // if data is available
            if (U2STAbits.URXDA) {
                data = U2RXREG;
                if (data == '\n' || data == '\r') {
                    complete = 1;
                }
                else {
                    ReceivedTarget[index] = data;
                    index++;
                }
            }
            
            // wrap around if index is greater than max length of char array
            if (index > 10) {
                index = 0;
            }
        }
        ReceivedTarget[index] = '\0';
        
        // convert string to integer and set as target count
        float recTargetAngle = atof(ReceivedTarget);
        TargetCount = (int)(recTargetAngle * ((48 * 98.78) / 360.00));
        
        // clear status flag
        IFS1bits.U2RXIF = 0;
        
        // set RECEIVED
        RECEIVED = 1;
    }
}

/*  Initialize Timer 5 interrupts which are configured to trigger at 50 Hz. 
    Timer 5 interrupts are used to transmit data of transient response of the 
    DC motor. 
 */
void TMR5_init(void) {
    INTCONbits.MVEC = 1;        // enable multi-vector interrupts
    __builtin_disable_interrupts();
    
    // Timer 5 interrupt is configured to trigger at a frequency of 50 Hz
    // PR5 = ((1/50)/(12.5 ns)) / 256 = 6250
    PR5 = 6250;
    TMR5 = 0;                   // start at 0
    T5CONbits.ON = 1;           // enable timer 5
    T5CONbits.TCKPS = 0b111;    // or 0x7; pre-scalar set to 256
    IPC5bits.T5IP = 3;          // priority = 3
    IPC5bits.T5IS = 1;          // sub-priority = 1
    IFS0bits.T5IF = 0;          // clear status flag
    IEC0bits.T5IE = 1;          // enable interrupt
    
    __builtin_enable_interrupts();
}

/*  ISR to transmit 100 samples of data to MATLAB to plot the transient 
    response of the DC motor. Data is sent in the following sequence:
        - time
        - target angle
        - current angle
        - error
        - integral error
 */
void __ISR(_TIMER_5_VECTOR, IPL3SOFT) TMR5_ISR(void) {
    // get time in seconds
    Time = _CP0_GET_COUNT() / 40000000;
    unsigned int elapsedTime = Time - PrevTime_Trans;
    PrevTime_Trans = Time;
    
    // only transmit after data is received
    if (RECEIVED && TransmitCount < 100) {
        // convert data into string for transmission
        char timeTransmit[10] = "";
        sprintf(timeTransmit, " %.4f\r\n\0", (float)elapsedTime);
        
        char targetAngleTransmit[10] = "";
        sprintf(targetAngleTransmit, " %.4f\r\n\0", AngleTarget);
        
        char currentAngleTransmit[10] = "";
        sprintf(currentAngleTransmit, " %.4f\r\n\0", AngleCurrent);
        
        char errorTransmit[10] = "";
        sprintf(errorTransmit, " %.4f\r\n\0", Error);
        
        char integralErrorTransmit[10] = "";
        sprintf(integralErrorTransmit, " %.4f\r\n\0", ErrorIntegral);
        
        // transmit time value
        int i = 0;
        while (timeTransmit[i] != '\0') {
            // wait until TX buffer is not full
            while (U2STAbits.UTXBF) {}
            U2TXREG = timeTransmit[i];
            i++;
        }
        
        i = 0;
        // transmit target angle value
        while (targetAngleTransmit[i] != '\0') {
            // wait until TX buffer is not full
            while (U2STAbits.UTXBF) {}
            U2TXREG = targetAngleTransmit[i];
            i++;
        }
        
        i = 0;
        // transmit current angle value
        while (currentAngleTransmit[i] != '\0') {
            // wait until TX buffer is not full
            while (U2STAbits.UTXBF) {}
            U2TXREG = currentAngleTransmit[i];
            i++;
        }
        
        i = 0;
        // transmit error value
        while (errorTransmit[i] != '\0') {
            // wait until TX buffer is not full
            while (U2STAbits.UTXBF) {}
            U2TXREG = errorTransmit[i];
            i++;
        }
        
        i = 0;
        // transmit integral of error value
        while (integralErrorTransmit[i] != '\0') {
            // wait until TX buffer is not full
            while (U2STAbits.UTXBF) {}
            U2TXREG = integralErrorTransmit[i];
            i++;
        }
        
        TransmitCount += 1;
    }
    
    // clear status flag
    IFS0bits.T5IF = 0;
}
    
/* Main function of the program */
int main(void) {
    // disable debugging through JTAG
    DDPCONbits.JTAGEN = 0;
    
    // set encoder outputs A and B as inputs explicitly
    TRISGbits.TRISG6 = 1;
    TRISGbits.TRISG7 = 1;
    
    // set F0 and F1 as outputs to control motor direction
    TRISFbits.TRISF0 = 0;
    TRISFbits.TRISF1 = 0;
    
    // initialize LCD Display
    lcd_display_driver_set_tristate();
    lcd_display_driver_initialize();
    
    // initialize interrupts and output compare
    CN_init();
    TMR4_init();
    TMR2_init();
    OC4_init();
    UART2_init();
    TMR5_init();
    
    // initialize struct to current values of encoder outputs A and B
    encoderPos.curr_A = PORTGbits.RG6;
    encoderPos.curr_B = PORTGbits.RG7;
    
    // initialize motor direction (no rotation)
    LATFbits.LATF0 = 0;
    LATFbits.LATF1 = 0;
    
    // set timer counter to 0
    _CP0_SET_COUNT(0);
    
    // infinite loop
    while (1) {}
    
    return 0;
}
