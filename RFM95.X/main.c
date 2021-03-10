#include "elen116header3.h"
#define FCY 8000000 
#include <libpic30.h>
#include "RFM95_header.h"
#include "SHT20_header.h"


#define sendBTN_DIR TRISDbits.TRISD6
#define ANALOG_DIR TRISAbits.TRISA6

#define sendBTN PORTDbits.RD6

#define I2C_ADDR_WRITE 0b1000000  //LSB is 1 for read, 0 for write
#define I2C_ADDR_READ  0b10000001

#define I2C_CMD_TRIGGER_T_MEASURMENT 0b11100011

#define ANALOG_WATER 780
#define ANALOG_AIR 1570

uint8_t rx_data = 0;
uint16_t tx_data = 0b0000000100000000;
float ADC_out = 0;

void rfm_init(void);
void analog_init(void);
void i2c_init(void);

//HI HARPER
//HI DANE

int main()
{
    UART2_Initialize();
    
    spi_init();
    rfm_init();
    
    /*i2c_init();
    
    while(1){
        while(I2C1STATbits.TRSTAT);                 //wait for transmission to finish
        I2C1CONLbits.SEN = 1;                       //init start condition
        I2C1TRN = I2C_ADDR_WRITE;                   //send address and write bit
        while(I2C1STATbits.ACKSTAT);                //wait to receive ack
        I2C1TRN = I2C_CMD_TRIGGER_T_MEASURMENT;     //send temp measurment command
        while(I2C1STATbits.ACKSTAT);                //wait to receive ack
        I2C1TRN = I2C_ADDR_READ;                   //send address and read bit
        while(I2C1STATbits.ACKSTAT);                //wait to receive ack
        I2C1CONLbits.RCEN = 1;                      //enable receive mode
        while(I2C1STATbits.RBF);                    //wait till receive reg full
        I2C1CONLbits.PEN = 1;
        printf("RECEIVED: %d\n", I2C1RCV);
        __delay_us(1000000);
    }*/
    
    //analog_init();
    
    while(1);
    
    return 0;
}

//*************************************************************************
//Soil Moisture
//*************************************************************************

void analog_init(void)
{   
    ANSAbits.ANSA6 = 1; //set to analog
    ANALOG_DIR = 1;     //set RA6 as input = AN23

    //select V source
    AD1CON2bits.NVCFG0=0;
    AD1CON2bits.PCVFG0=0;
    AD1CON2bits.PCVFG1=0;
    
    //set channel positive and negative input
    AD1CHSbits.CH0NA=0;
    AD1CHSbits.CH0SA=23; //AN23
    
    //select conversion clock
    AD1CON3bits.ADRC=1;
    AD1CON3bits.ADCS=0x00FF; //TAD=256*Tcyc
    
    //select the appropriate sample/conversion sequence
    AD1CON1bits.SSRC=7; //auto convert mode
    AD1CON3bits.SAMC=0x1F; //auto samp time = 31*TAD
    
    //for channel A scanning operations, select the positive cannels to be included
    //AD1CSSH and AD1CSSL
    AD1CSSH = 0; // no scanning
    AD1CSSL = 0;
    
    //select how conversion results are presented to buffer
    AD1CON1bits.MODE12=1; //select 12 bit mode
    AD1CON1bits.FORM=0; //00 = absolute decimal result, unsigned, right
    AD1CON5bits.ASEN=0; //disable auto scan

    //select interrupt rate
    AD1CON2bits.SMPI=0; //interrupt completion of every sample
    
    
    //turn on A/D module
    AD1CON1bits.ADON=1;
    
    IFS0bits.AD1IF=0; //clear ADC flag
    IEC0bits.AD1IE=1;//enable ADC interrupt
    IPC3bits.AD1IP=4; //set priority to 4
    INTCON2bits.GIE=1; //enable global interrupts  
    AD1CON2bits.BUFREGEN=1; //AN5 result stored in ADC1BUF5
    AD1CON1bits.ASAM=1; //start sampling
}

// ADC Interrupt Service Routine
int val;
float per;

void __attribute__ ((__interrupt__)) _ADC1Interrupt(void) {
    IFS0bits.AD1IF = 0; // Clear Interrupt Flag
    val = ADC1BUF23;
    per = (val-1570)/(-7.9);
    printf("ADC reading = %f %\n", per);
}