/*
 * File:   lab1.c
 * Author: Nadir Ould-Khessal
 * Device:PIC24FJ1024GB610
 *
 * Created on November 13, 2019, 2:18 PM
 */

// PIC24FJ1024GB610 Configuration Bit Settings

// 'C' source line config statements

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PLLMODE = DISABLED       // PLL Mode Selection (No PLL used; PLLEN bit is not available)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFCN = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config SOSCSEL = ON             // SOSC Power Selection Configuration bits (SOSC is used in crystal (SOSCI/SOSCO) mode)
#pragma config PLLSS = PLL_PRI          // PLL Secondary Selection Configuration bit (PLL is fed by the Primary oscillator)
#pragma config IOL1WAY = OFF             // Peripheral pin select configuration bit (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler bits (1:32,768)
#pragma config FWPSA = PR128            // Watchdog Timer Prescaler bit (1:128)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable bits (WDT Enabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTCMX = WDTCLK          // WDT MUX Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)

// FWDT
/*#pragma config WDTPS = PS4096           // Watchdog Timer Postscaler bits (1:4,096)
#pragma config FWPSA = PR128            // Watchdog Timer Prescaler bit (1:128)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bits (WDT Enabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTCMX = LPRC            // WDT MUX Source Select bits (WDT always uses LPRC as its clock source)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)*/

// FPOR
#pragma config BOREN = ON               // Brown Out Enable bit (Brown Out Enable Bit)
#pragma config LPCFG = OFF              // Low power regulator control (No Retention Sleep)
#pragma config DNVPEN = ENABLE          // Downside Voltage Protection Enable bit (Downside protection enabled using ZPBOR when BOR is inactive)

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config BTSWP = OFF              // BOOTSWP Disable (BOOTSWP instruction disabled)

// FDEVOPT1
#pragma config ALTCMPI = DISABLE        // Alternate Comparator Input Enable bit (C1INC, C2INC, and C3INC are on their standard pin locations)
#pragma config TMPRPIN = OFF            // Tamper Pin Enable bit (TMPRN pin function is disabled)
#pragma config SOSCHP = ON              // SOSC High Power Enable bit (valid only when SOSCSEL = 1 (Enable SOSC high power mode (default))
#pragma config ALTVREF = ALTREFEN       // Alternate Voltage Reference Location Enable bit (VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9)

// #pragma config statements should precede project file includes.

/************************* DO NOT Alter any of the above code *******/
/************************* START of Your Program *********************/

            /***** Directives and Define AREA************/
#include <xc.h>
#include <stdio.h>
           

void UART2_Initialize(void)
{
    
    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    RPOR8bits.RP17R = 0x0005;    //RF5->UART2:U2TX
    RPINR19bits.U2RXR = 0x000A;    //RF4->UART2:U2RX

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    
/**    
     Set the UART2 module to the options selected in the user interface.
     Make sure to set LAT bit corresponding to TxPin as high before UART initialization
*/
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U2MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; URXEN disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U2STA = 0x00;
    // BaudRate = 9600; Frequency = 8000000 Hz; U2BRG 0x67; 
    U2BRG = 0x67;//0x33;
    // BaudRate = 57600; Frequency = 8000000 Hz; U2BRG 0x16; 
    //U2BRG = 0x16;//0x33;
    // BaudRate = 115200; Frequency = 8000000 Hz; U2BRG 0x16; 
    //U2BRG = 0x07;//0x33;
    
    // ADMADDR 0; ADMMASK 0; 
    U2ADMD = 0x00;
    
    UART2_Enable();  // enabling UARTEN bit
}

uint8_t UART2_Read(void)
{
    while(!(U2STAbits.URXDA == 1))
    {
        
    }

    if ((U2STAbits.OERR == 1))
    {
        U2STAbits.OERR = 0;
    }
    
    return U2RXREG;
}

void UART2_Write(uint8_t txData)
{
    while(U2STAbits.UTXBF == 1)
    {
        
    }

    U2TXREG = txData;    // Write the data byte to the USART.
}

uint16_t UART2_StatusGet (void)
{
    return U2STA;
}

void UART2_Enable(void)
{
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;
}

void UART2_Disable(void)
{
    U2MODEbits.UARTEN = 0;
    U2STAbits.UTXEN = 0;
}

//helper function for std output functions

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) {
    int i;
    //UART2_Initialize();
    while(U2STAbits.TRMT == 0);  
    for (i = len; i; --i)
    {
        while(U2STAbits.TRMT == 0);
        U2TXREG = *(char*)buffer++;        
    }
    return(len);
}
//helper function for std intput functions
int __attribute__((__section__(".libc.read"))) read(int handle, void *buffer, unsigned int len) {
    int i;
    //UART2_Initialize();
    while(U2STAbits.URXDA == 0);  
    for (i = len; i; --i)
    {
        while(U2STAbits.URXDA == 0);
        *(char*)buffer++=U2RXREG;        
    }
    return(len);
}
