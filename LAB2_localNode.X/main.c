/* 
 * Local Node
 * File:   main.c
 * Author: Abigail Santos, Yunie Yi, Yuran Wu
 *
 * Created on January 19, 2017, 4:03 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <p18f25k22.h>
#include <delays.h>

#pragma config FOSC = INTIO7, PLLCFG = OFF, PRICLKEN = ON, FCMEN = OFF, DEBUG = OFF
#pragma config IESO = OFF, PWRTEN = OFF, BOREN = OFF, BORV = 250, WDTEN = OFF
//#pragma config WDTPS = 1, CCP2MX = PORTB3, PBADEN = OFF, CCP3MX = PORTB5
//#pragma config HFOFST = OFF, T3CMX = PORTC0, P2BMX = PORTB5, MCLRE = EXTMCLR
#pragma config STVREN = OFF, LVP = OFF, XINST = OFF, CP0 = OFF, CP1 = OFF
#pragma config CP2 = OFF, CP3 = OFF, CPB = OFF, CPD = OFF, WRT0 = OFF, WRT1 = OFF
#pragma config WRT2 = OFF, WRT3 = OFF, WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTRB = OFF

#define DEBUG           PORTAbits.RA0
#define TX              PORTCbits.RC6
#define RX              PORTCbits.RC7
#define SR_DATA         PORTAbits.RA5
#define SR_CLK          PORTCbits.RC3
#define SR_CLEAR        PORTCbits.RC0
#define SCL             PORTBbits.RB1
#define SDA             PORTBbits.RB2

#define COMMAND         " is received\n"

#define MOTOR_ENABLE    0xFF
#define MOTOR_DISABLE   0xFE
#define MOTOR_SET_POINT 0xFD
#define MOTOR_INCR      0xFC
#define MOTOR_DECR      0xFB
#define SHOW_ERROR      0xFA

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

void __init(void);
void UART_config(void);
void I2C_config(void);
void SR_tx(char);
unsigned char mGetc(void);
void mPutc(char);
int mStrlen(static const rom char *);
void mPuts(static const rom char *, int);
void ISR(void);
void USER_MANUAL(void);

void I2C_config(void);
void I2C_waitForIdle(void);
void I2C_txStart(void);
void I2C_repTxStart(void);
unsigned char I2C_read(unsigned char);
void I2C_write(unsigned char);
void I2C_stop(void);
void I2C_tx(unsigned char);

char range(int, char);
void execute(unsigned char);
void printError(int);

void SRAM_write_ctrl(void);
void SRAM_read_ctrl(void);
void SRAM_disable(void);
void SRAM_write(unsigned char, unsigned char);
unsigned char SRAM_read(unsigned char);
void addr_SR_tx(unsigned char);
void data_SR_tx(unsigned char);
unsigned char data_SR_rx(void);

void high_isr(void);
void low_isr(void);

unsigned char RX_FLAG, CMD, SET_POINT_FLAG, MOTOR_START_FLAG;
char cur, show_count;
char level[5] = {'0', '0', '0', '0', '0'};
int index, ERROR;

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

 

void high_isr (void) {
    if(PIR1bits.RCIF) {
        unsigned char cmd = RCREG1;
        RX_FLAG = 1;
        if(SET_POINT_FLAG) {
            CMD = cmd;
        } else {
            switch(cmd) {
                case '1': 
                    /* -- start motor -- */
                    CMD = MOTOR_ENABLE;
                    SRAM_write(MOTOR_ENABLE, 0xAA);
                    SRAM_read(0xAA);
                    break;

                case '2':
                    /* -- stop motor -- */
                    CMD = MOTOR_SET_POINT;
                    SRAM_write(MOTOR_SET_POINT, 0xAA);
                    SRAM_read(0xAA);
                    break;

                case '3':
                    /* -- set point -- */
                    CMD = MOTOR_DISABLE;
                    SRAM_write(MOTOR_DISABLE, 0xAA);
                    SRAM_read(0xAA);
                    break;

                case '4':
                    // refresh
                    CMD = SHOW_ERROR;
                    SRAM_write(SHOW_ERROR, 0xAA);
                    SRAM_read(0xAA);
                    break;

                case 'w':
                    /* -- increment -- */
                    CMD = MOTOR_INCR;
                    SRAM_write(MOTOR_INCR, 0xAA);
                    SRAM_read(0xAA);
                    break;

                case 's':
                    /* -- decrement -- */
                    CMD = MOTOR_DECR;
                    SRAM_write(MOTOR_DECR, 0xAA);
                    SRAM_read(0xAA);
                    break;

                default:
                    mPuts("\nNo such Command!\n", 18);
                    break;
            }
        }
    }
}

void low_isr(void) {}


void main(void) {
    int err, i;
    int tmp;
    CMD = 0;
    RX_FLAG = 0;
    SET_POINT_FLAG = 0;
    ERROR = 0;
    cur = 0;
    show_count = 0;
    index = 0;
    
    
    
    __init();
    UART_config();
    I2C_config();
    
    /*----------*/
    USER_MANUAL();
    
    /*----------*/
    
    while(1) {
        // main control loop
        err = 0;
        I2C_txStart();
        I2C_write(0x51);
        Delay10TCYx(40);
        
        for(i = 0; i < 4; i++) {
            tmp = 0;
            if(i < 3) {
                tmp |= (I2C_read(0) & 0xFF);
            } else {
                tmp |= (I2C_read(1) & 0xFF);
            }
            err |= (tmp << (8 * i));
        }
        
        
        cur = range(err, cur);
        if(RX_FLAG) {
            RX_FLAG = 0;
            execute(CMD);
        }
    }
}

void execute(unsigned char cmd) {
    if(SET_POINT_FLAG) {
        int num = 0, i, mul;
        if(cmd != 0x0D) {
            level[index] = cmd;
            index++;
        } else {
            mul = 1;
            for(i = index - 1; i >= 0; i--) {
                if(level[i] > '9' || level[i] < '0') {
                    mPuts("\nError: invalid syntax.\n", 24);
                    mPuts("Set point : ", 12);
                    index = 0;
                    return ;
                }
                num += mul * (level[i] - '0');
                mul *= 10;
            }
            index = 0;
            if(num > 100) {
                mPuts("\nThe speed cannot be greater than 100.", 39);
                mPuts("Set point : ", 12);
            } else {
                I2C_tx(num & 0xFF);
                SET_POINT_FLAG = 0;
            }
        }
    } else {
        switch(cmd) {
            case MOTOR_ENABLE: 
                /* -- start motor -- */
                mPuts("\nMotor starts\n", 14);
                MOTOR_START_FLAG = 1;
                I2C_tx(MOTOR_ENABLE);
                break;

            case MOTOR_SET_POINT:
                /* -- set point -- */
                if(MOTOR_START_FLAG) {
                    mPuts("\nSet point : ", 13);
                    I2C_tx(MOTOR_SET_POINT);
                    SET_POINT_FLAG = 1;
                } else {
                    mPuts("\nMotor is OFF\n", 14);
                }
                break;
                
            case MOTOR_DISABLE:
                /* -- stop motor -- */
                mPuts("\nMotor stops\n", 13);
                MOTOR_START_FLAG = 0;
                I2C_tx(MOTOR_DISABLE);
                break;

            case SHOW_ERROR:
                // refresh
                printError(ERROR);
                break;

            case MOTOR_INCR:
                /* -- increment -- */
                I2C_tx(MOTOR_INCR);
                break;

            case MOTOR_DECR:
                /* -- decrement -- */
                I2C_tx(MOTOR_DECR);
                break;

            default:
                mPuts("\nNo such Command!\n", 18);
                break;
        }
    }
}

char range(int err, char pre) {
    int cur = 0;
    ERROR = err;
    if(err > 0) {
        if(err <= 10) { // LEVEL 2
            cur = 1;
        } else if(err <= 20) { // LEVEL 1
            cur = 2;
        } else if(err <= 50) { // LEVEL 0
            cur = 3;
        } else {    // VERY BAD
            cur = 100;
        }
    } else if(err < 0) {
        if(err >= -10) { // LEVEL 2
            cur = -1;
        } else if(err >= -20) { // LEVEL 1
            cur = -2;
        } else if(err >= -50) { // LEVEL 0
            cur = -3;
        } else {    // VERY BAD
            cur = 100;
        }
    }
    if(!SET_POINT_FLAG) {
        if(cur != pre & show_count == 0xFF) {
            show_count = 0x00;
            mPuts("\nWarning: ", 10);
            printError(err);
            mPuts("\nAlarm LEVEL ", 13);

            if(cur == 1 | cur == -1) mPuts("2 - OF CONCERN\n", 15);
            else if(cur == 2 | cur == -2) mPuts("1 - MODERATE\n", 13);
            else if(cur == 3 | cur == -3) mPuts("0 - SEVERE\n", 11);
            else if(cur == 100) mPuts("-1 - DANGER!\n", 13);
            else {
                if(MOTOR_START_FLAG) {
                    mPuts("NULL - MOTOR IS STABLE\n", 23);
                } else {
                    mPuts("NULL - MOTOR IS DISABLED\n", 25);
                }
            }
        } else {
            show_count++;
        }
    }
    return cur;
}

void printError(int err) {
        char sign = '+';
        char tens = '0';
        char ones = '0';
        char tenth = '0';
        char hunth = '0';
        
        if(err < 0) {
            sign = '-';
            err = -err;
        }
        hunth = '0'; 
        tenth = ((char) (err % 10)) + '0';
        ones = ((char) ((err / 10) % 10)) + '0';
        tens = ((char) ((err / 100) % 10)) + '0';
        mPuts("\nThe Error is : ", 16);
        mPutc(sign);
        mPutc(tens);
        mPutc(ones);
        mPutc('.');
        mPutc(tenth);
        mPutc(hunth);
        mPutc('%');
}

void I2C_tx(unsigned char cmd) {
    // Write
    I2C_config();
    I2C_txStart();
    Delay10TCYx(40);
    I2C_write(0x50);
    I2C_waitForIdle();
    I2C_write(cmd);
    //I2C_stop();
}

void I2C_waitForIdle(void) {
    while((SSP1CON2 & 0x1F) | SSP1STAT & 0x04);
}

void I2C_txStart(void) {
    SSP1CON2bits.SEN = 1;
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
}

void I2C_repTxStart(void) { 
    I2C_waitForIdle();
    SSP1CON2bits.RSEN = 1;
}

void I2C_stop(void) {
    I2C_waitForIdle();
    SSP1CON2bits.PEN = 1;
}

unsigned char I2C_read(unsigned char ack) {
    unsigned char READData;
    I2C_waitForIdle();
    
    SSP1CON2bits.RCEN = 1;
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    while(!SSP1STATbits.BF);
    SSP1STATbits.BF = 0;

    READData = SSP1BUF;

    if(!ack) {
        SSP1CON2bits.ACKDT = 0;
    } else {
        SSP1CON2bits.ACKDT = 1;
    }
    
    SSP1CON2bits.ACKEN = 1;
    
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    
    return READData;
}

void I2C_write(unsigned char I2CWriteData) {
    SSP1BUF = I2CWriteData;
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
}

void I2C_config(void) {
    OSCCONbits.IRCF = 0b111;
    ANSELC = 0;
    TRISCbits.RC3 = 1;
    TRISCbits.RC4 = 1;
    SSP1CON1 = 0b00101000;
    SSP1CON2 = 0;
    SSP1ADD = 199;
    SSP1STAT = 0;
}


void USER_MANUAL(void) {
    mPuts("\nUser Manual:\n", 14);
    mPuts(" - Type 1 to start motor\n", 25);
    mPuts(" - Type 2 to set point\n", 23);
    mPuts(" - Type 3 to stop motor\n", 24);
    mPuts(" - Type 4 to check error\n", 25);
    mPuts(" - Type w to increment 0.5%\n", 28);
    mPuts(" - Type s to decrement 0.5%\n", 28);
}



void mPuts(static const rom char * cmd, int len) {
    int i;
    for(i = 0; i < len; i++) {
        mPutc(cmd[i]);
    }
}

/* NOTE: UART transmission starts when the data is loaded to TXREGx */
void mPutc(char txData) {
    while(!TXSTA1bits.TRMT); // wait for previous transmission finish
    TXREG1 = txData;    // load new data
}

void SR_tx(char rxData) {
    int i;
    /* -- Initialize SR -- */
    SR_CLEAR = 0;
    SR_CLEAR = 1;
    SR_CLK = 0;
    
    for(i = 7; i >= 0; i--) {
        SR_DATA = rxData & 1;
        SR_CLK = 1;
        SR_CLK = 0;
        rxData = rxData >> 1;
    }
}


unsigned char mGetc(void) {
    while(!PIR1bits.RC1IF);
    return RCREG1;
}

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

void __init(void) {
    ANSELA = 0x00; // digital I/O mode
    TRISA = 0x10; // digital Output mode
    
    ANSELC = 0x00;
    TRISC &= 0x00;
    
    TRISCbits.RC7 = 1;
    TRISCbits.RC6 = 1;
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