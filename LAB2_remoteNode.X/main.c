/* 
 * Remote Node
 * File:   main.c
 * Author: ee475
 *
 * Created on January 19, 2017, 5:38 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p18f25k22.h>
#include <delays.h>



#define DEBUGL               PORTAbits.RA0

#define PWM_O                PORTCbits.RC2

#define SCL_2                PORTBbits.RB1
#define SDA_2                PORTBbits.RB2

#define SCL_1                PORTCbits.RC3
#define SDA_1                PORTCbits.RC4

#define SLAVE_ADD1           0x50
#define SLAVE_ADD2           0x40     

#define WE                   PORTCbits.RC1 // WE (WRITE ENABLE) LOW TRUE
#define DATA_EN              PORTAbits.RA1 // DATA_ENABLE 
#define OE                   PORTAbits.RA3 // OE (OUTPUT ENABLE) LOW TRUE 

#define SR_CLEAR             PORTCbits.RC0

#define addr_SR_DATA         PORTBbits.RB0
#define addr_SR_CLK          PORTBbits.RB3


#define data_SR_DATA_O       PORTAbits.RA5
#define data_SR_CLK          PORTBbits.RB5

#define data_SR_INH_CLK      PORTAbits.RA4
#define data_SR_DATA_I       PORTAbits.RA2
#define data_SR_LD           PORTAbits.RA6

/* -- test define -- */
#define TX                   PORTCbits.RC6
#define RX                   PORTCbits.RC7

#define MASK_1               0xFF
#define MASK_2               0x03

#define MOTOR_ENABLE    0xFF
#define MOTOR_DISABLE   0xFE
#define MOTOR_SET_POINT 0xFD
#define MOTOR_INCR      0xFC
#define MOTOR_DECR      0xFB

void __init(void);
void PWM_setPoint(double);
void PWM_changeSpeed(char, double);
void PWM_increment(void);
void PWM_decrement(void);
void I2C_config(void);
void SRAM_write_ctrl(void);
void SRAM_read_ctrl(void);
void SRAM_disable(void);
void SRAM_write(unsigned char, unsigned char);
unsigned char SRAM_read(unsigned char);
void addr_SR_tx(unsigned char);
void data_SR_tx(unsigned char);
unsigned char data_SR_rx(void);
void ADC_config(void);
void high_isr(void);
void low_isr(void);
void execute(unsigned char);

/* -- test functions -- */
void UART_config(void);
unsigned char mGetc(void);
void mPutc(unsigned char);





#pragma config FOSC = INTIO7, PLLCFG = OFF, PRICLKEN = ON, FCMEN = OFF, DEBUG = OFF
#pragma config IESO = OFF, PWRTEN = OFF, BOREN = OFF, BORV = 250, WDTEN = OFF
//#pragma config WDTPS = 1/*, CCP2MX = PORTB3*/, PBADEN = OFF/*, CCP3MX = PORTB5*/
//#pragma config HFOFST = OFF, T3CMX = PORTC0, P2BMX = PORTB5, MCLRE = EXTMCLR
#pragma config STVREN = OFF, LVP = OFF, XINST = OFF, CP0 = OFF, CP1 = OFF
#pragma config CP2 = OFF, CP3 = OFF, CPB = OFF, CPD = OFF, WRT0 = OFF, WRT1 = OFF
#pragma config WRT2 = OFF, WRT3 = OFF, WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTRB = OFF

#pragma interrupt high_isr
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
  _asm goto high_isr _endasm
}
#pragma interruptlow low_isr
#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
  _asm goto low_isr _endasm
} 
#pragma code

unsigned char MOTOR_EN, SET_POINT_FLAG;
int curDuty, error, sum;

void high_isr (void) {
    // SSP1CON2bits.ACKEN set to 1 to enable the slave acknowledge sequence
    unsigned char tmp, cmd, err = 0;
    int dum = 0;
    if(PIR1bits.SSP1IF) {
        SSP1CON1bits.CKP = 0;
        PIR1bits.SSP1IF = 0;
        if((SSP1CON1bits.SSPOV) || (SSP1CON1bits.WCOL)) {
            tmp = SSP1BUF;
            SSP1CON1bits.SSPOV = 0;
            SSP1CON1bits.WCOL = 0;
            SSP1CON1bits.CKP = 1;
        }
        
        if(!SSP1STATbits.D_NOT_A && !SSP1STATbits.R_NOT_W) { // master write
            tmp = SSP1BUF;
            while(SSP1STATbits.BF);
            SSP1CON1bits.CKP = 1; // release clock
            while(!PIR1bits.SSP1IF);
            PIR1bits.SSP1IF = 0;
            cmd = SSP1BUF;
            execute(cmd);
            SSP1CON1bits.CKP = 1; // release clock
        } else if(!SSP1STATbits.D_NOT_A && SSP1STATbits.R_NOT_W) { // master read
            tmp = SSP1BUF;
            dum = error;
            
            do {
                err = 0;
                err |= (dum & 0xFF);
                SSP1BUF = err;
                SSP1CON1bits.CKP = 1; // release clock, start to send data
                while(!PIR1bits.SSP1IF);
                PIR1bits.SSP1IF = 0;
                dum = dum >> 8;
            } while(!SSP1CON2bits.ACKSTAT);
        }
    }
}

void low_isr(void) {}



void main(void) {
    char i;
    int result;
    __init();
    UART_config();
    I2C_config();
    
    PWM_setPoint(0);
    ADC_config();
    MOTOR_EN = 0;
    SET_POINT_FLAG = 0;
    error = 0;
    while(1) {
        result = 0;
        ADCON0bits.GO_NOT_DONE = 1;
        while(ADCON0bits.GO_NOT_DONE);
        result |= ADRESH;
        result = result << 8;
        result |= ADRESL;
        error = result - (curDuty * 64/10);
        Delay10TCYx(0xFF);
    }
}

void execute(unsigned char cmd) {
    if(SET_POINT_FLAG) {
     PWM_setPoint((double) cmd);
     SET_POINT_FLAG = 0;
    }    
    else{
        switch(cmd) {
            case MOTOR_ENABLE :
                MOTOR_EN = 1;
                break;
            case MOTOR_DISABLE :
                MOTOR_EN = 0;
                PWM_setPoint(0);
                break;
            case MOTOR_SET_POINT : // Warning: Potentially problematic!!
                if(MOTOR_EN) {
                    SET_POINT_FLAG = 1;
                }
                break;
            case MOTOR_INCR :
                PWM_increment();
                break;
            case MOTOR_DECR :
                PWM_decrement();
                break;
            default : break;
        }
    }
}

void I2C_config(void) {
    OSCCONbits.IRCF = 0b111;
    
    ANSELCbits.ANSC3 = 0;
    ANSELCbits.ANSC4 = 0;
    
    TRISCbits.RC3 = 1;
    TRISCbits.RC4 = 1;
    
    SSP1STAT = 0x80; // MODE needs to be verified
    SSP1ADD = SLAVE_ADD1; 
    SSP1CON1 = 0x36; // clock enabled    
    SSP1CON2 = 0x01; 
   
        
    RCONbits.IPEN = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;    
    IPR1bits.SSP1IP = 0;
    //IPR2bits.BCL1IP = 1;
    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1;
}

void ADC_config(void) {
    ANSELAbits.ANSA0 = 1;
    TRISAbits.TRISA0 = 1;
    
    ADCON2bits.ADCS = 0b111;
    ADCON0bits.CHS = 0b00000;
    ADCON2bits.ADFM = 1;
    ADCON2bits.ACQT = 0b111;
    ADCON0bits.ADON = 1;
}

void SRAM_write_ctrl(void) {
    /* -- WRITE OPERATION -- */
    
    // ENABLE WRITE CONTROL SIGNALS
    OE = 0;
    WE = 0;
    DATA_EN = 0;
}

void SRAM_read_ctrl(void) {
    WE = 1;
    OE = 0;
    DATA_EN = 1;
}

void SRAM_disable(void) {
    // DISABLE OE WE CONTROL 
    OE = 1;
    WE = 1;
    DATA_EN = 1;
}

unsigned char SRAM_read(unsigned char addr) {
    int i;
    char rx;
    SRAM_disable();
    
    SR_CLEAR = 0;
    SR_CLEAR = 1;
    WE = 1;
    DATA_EN = 1;
    
    addr_SR_tx(addr);
    
    OE = 0;
    rx = data_SR_rx();
    OE = 1;
    SRAM_disable();
    return rx;
}

void SRAM_write(unsigned char data, unsigned char addr) {
    SRAM_disable();
    data_SR_LD = 1; //
    SR_CLEAR = 0;
    SR_CLEAR = 1;
    OE = 1;
    
    
    addr_SR_tx(addr);
    data_SR_tx(data);
    
    DATA_EN = 0;
    WE = 0;    
    
    WE = 1;
    SRAM_disable();
}

unsigned char data_SR_rx(void) {
    unsigned char rx;
    int i, j;
    data_SR_LD = 0;
    data_SR_INH_CLK = 1;
    data_SR_LD = 1;
    data_SR_INH_CLK = 0;
    
    data_SR_CLK = 0;    
    for(i = 0; i < 8; i++) {
        rx |= data_SR_DATA_I;
        rx = rx << 1;
        data_SR_CLK = 1;
        for(j = 0; j < 4; j++);
        data_SR_CLK = 0;
    }
    return rx;
}

void data_SR_tx(unsigned char rxData) {
    int i,j;
    /* -- Initialize SR -- */
    //SR_CLEAR = 0;
    //SR_CLEAR = 1;
    data_SR_CLK = 0;
    
    for(i = 7; i >= 0; i--) {
        data_SR_DATA_O = rxData & 1;
        data_SR_CLK = 1;
        data_SR_CLK = 0;
        rxData = rxData >> 1;
    }
}

void addr_SR_tx(unsigned char rxData) {
    int i, j;
    unsigned char tx;
    /* -- Initialize SR -- */
    addr_SR_CLK = 0;
    
    for(i = 7; i >= 0; i--) {
        tx = rxData & 1;
        addr_SR_DATA = tx;
        addr_SR_CLK = 1;
        addr_SR_CLK = 0;
        rxData = rxData >> 1;
    }
}

void __init(void) {
    ANSELA = 0x00; // digital I/O mode
    TRISA = 0x14; // digital Output mode
    //PORTA = 0x1A;
    
    ANSELB = 0x00;
    TRISB = 0x00;
    
    ANSELC = 0x00;
    TRISC = 0x00;
    
    data_SR_LD = 1;
}



void PWM_setPoint(double pt) {
    int duty;
    PR2 = 0b00100111 ;
    T2CON = 0b00000101 ;
    
    TRISCbits.TRISC2 = 1;
    ANSELCbits.ANSC2 = 0;
    OSCCONbits.IRCF = 0b111;
    duty = pt * (4 * (PR2 + 1)) / 100;
    CCPR1L = (duty >> 2) & MASK_1;
    CCP1CON = ((duty & MASK_2) << 4) + 0b1100;
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;
    curDuty = 0;
    curDuty |= (CCPR1L & 0xFF);  
    curDuty = curDuty << 2;
    curDuty |= (CCP1CONbits.DC1B & 0x03);
}

void PWM_changeSpeed(char inDe, float pre) {
    PR2 = 0b00100111 ;
    T2CON = 0b00000101 ;
    TRISCbits.TRISC2 = 1;
    ANSELCbits.ANSC2 = 0;
    OSCCONbits.IRCF = 0b111;
    if(inDe) {
        if(CCP1CONbits.DC1B == 0x03) {
            CCP1CONbits.DC1B = 0;
            CCPR1L = CCPR1L + 1;
        } else {
            CCP1CONbits.DC1B = CCP1CONbits.DC1B + 1;
        }
    } else {
        if(CCP1CONbits.DC1B == 0x00) {
            CCP1CONbits.DC1B = 0x03;
            CCPR1L = CCPR1L - 1;
        } else {
            CCP1CONbits.DC1B = CCP1CONbits.DC1B - 1;
        }
    }
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;
    curDuty = 0;
    curDuty |= (CCPR1L & 0xFF);  
    curDuty = curDuty << 2;
    curDuty |= (CCP1CONbits.DC1B & 0x03);
}



void PWM_increment(void) {
    PWM_changeSpeed(1, curDuty);
}

void PWM_decrement(void) {
    PWM_changeSpeed(0, curDuty);
}

/* -- test functions -- */
void UART_config(void) { 
    TRISCbits.RC7 = 1;
    TRISCbits.RC6 = 1;
    
    OSCCONbits.IRCF = 7; // 16MHz Oscillator
    
    TXSTA1bits.SYNC = 0; // Async mode
    TXSTA1bits.TXEN = 0; // TX enable
    TXSTA1bits.BRGH = 1; // High Baud Rate Select bit (1 = High speed)
    
    RCSTA1bits.SPEN = 1; // Serial Port Enable
    RCSTA1bits.RX9 = 0;  // disable 9 bit mode
    RCSTA1bits.CREN = 1; // Continuous Receive Enable
    
    BAUDCON1bits.BRG16 = 1; // High baud rate mode
    BAUDCON1bits.DTRXP = 1; 
    BAUDCON1bits.CKTXP = 1; 
    
    /* -- BAUD RATE: 9600 -- */
    SPBRG1 = 416;
    SPBRGH1 = 1;
    
    /* -- interrupt configuration (DISABLED) -- */
    RCONbits.IPEN = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;   
    
    
    PIE1bits.RC1IE = 1;
    IPR1bits.RC1IP = 0; // Receive Interrupt Priority bit
    PIE1bits.TX1IE = 0;
    
    TXSTA1bits.TXEN = 1; // TX enable
}

void mPutc(unsigned char txData) {
    while(!TXSTA1bits.TRMT); // wait for previous transmission finish
    TXREG1 = txData;    // load new data
}

unsigned char mGetc(void) {
    while(!PIR1bits.RC1IF);
    return RCREG1;
}

