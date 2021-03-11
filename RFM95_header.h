#define SPI_DIR_SDO TRISBbits.TRISB5
#define SPI_DIR_CLK TRISBbits.TRISB1
#define SPI_DIR_SDI TRISBbits.TRISB2
#define SPI_DIR_CS TRISBbits.TRISB3
#define SPI_DIR_RESET TRISBbits.TRISB12

#define SPI_CS LATBbits.LATB3
#define SPI_RESET LATBbits.LATB12

#define RFM_CMD_SLEEP           0b1000000100000000
#define RFM_CMD_STBY            0b1000000100000001
#define RFM_CMD_LORAMODE        0b1000000110000001
#define RFM_CMD_READREGOPMODE   0b0000000100000000
#define RFM_CMD_LOADTXREG_A     0b1000000010101010
#define RFM_CMD_TX              0b1000000100000011
#define RFM_CMD_SF7             0b1001111001110000  //CRC off

void spi_init(void);
uint8_t spi_write(uint16_t);
void rfm_init(void);
void lora_tx(void);

//code that was in main:

/*spi_init();
    rfm_init();
    
    ANSD = 0;
    ODCD = 0;
    sendBTN_DIR = 1;
    
    while(1){
        while(sendBTN);
        printf("response: %d\n", spi_write(tx_data));
    }*/

//initializes all needed PORTS and maps SPI to peripheral pins
void spi_init(void)
{
    printf("SPI init... ");
    
    //disable SPI module
    SPI1CON1Lbits.SPIEN = 0;
    
    //SPI pins mapped to port B
    //set port B to digital
    ANSD = 0x00;
    ODCD = 0x00;    //not open drain
    ANSB = 0;
    ODCB = 0;
    
    __builtin_write_OSCCONL(OSCCON & 0xbf);     //unlock PPS
    //set SDI1 to RP13, pin23, RB2
    RPINR20bits.SDI1R = 13;
    //set SDO1 to RP18, pin20, RB5
    RPOR9bits.RP18R = 7;
    //set SCK1 to RP, pin, RB
    RPOR0bits.RP1R = 8;
    __builtin_write_OSCCONL(OSCCON | 0xbf);     //lock PPS
    
    //set inputs and outputs
    SPI_DIR_SDO = 0;
    SPI_DIR_SDI = 1;
    SPI_DIR_CLK = 0;
    SPI_DIR_CS = 0;
    SPI_DIR_RESET = 0;
    
    //set CS to high (idle)
    SPI_CS = 1;
    //set reset to high (idle)
    SPI_RESET = 1;
    
    //clear interrupt flags
    IFS3bits.SPI1RXIF = 0;
    IFS0bits.SPI1IF = 0;
    //disable SPI interrupts
    IEC3bits.SPI1RXIE = 0;
    IEC0bits.SPI1IE = 0;
    
    //enable master mode
    SPI1CON1Lbits.MSTEN = 1;
    //16 bit mode
    SPI1CON1Lbits.MODE = 1;
    //use MODE bits in CON1 for word length
    SPI1CON2Lbits.WLENGTH = 0;
    //write baud rate register
    SPI1BRGLbits.BRG = 255; //16kHz
    //enable SPI with SPIEN bit 
    SPI1CON1Lbits.SPIEN = 1;        
    
    //enable SPI interrupts and set priority to 1 and 0
    INTCON2bits.INT1EP = 0;   //int on positive edge
    IEC3bits.SPI1RXIE = 1;
    IEC0bits.SPI1IE = 1;
    IPC14bits.SPI1RXIP = 1;
    IPC2bits.SPI1IP = 0;
    
    //clear receive buffer
    //SPI1BUFL = 0;
    //reset overflow status bit
    SPI1STATLbits.SPIROV = 0;
    printf("SPI initiated.\n");
}

//sets initial settings on the RFM95 module that are needed for transmission
void rfm_init(void)
{
    printf("RFM init... \n");
    
    //start with read or write bit, 1=w 0=r
    //7 bit reg address
    //8 bit data
    
    spi_write(RFM_CMD_STBY);
    spi_write(RFM_CMD_LORAMODE);  //lora mode, stby
    printf("op mode reg = %d\n", spi_write(RFM_CMD_READREGOPMODE));    //read address op reg
    spi_write(RFM_CMD_SF7);
    
    //set sync code 0x28 is FSK registers
    
    printf("RFM initiated.\n");
    printf("\n");
}

//writes a 16-bit value to the SPI bus and saves the received value
uint8_t spi_write(uint16_t data_in)
{
    uint8_t data_out = 0;
    
    while(SPI1STATLbits.SPIBUSY);
    //SPI1BUFL = 0x00;
    //write date to Tx to the SPI1BUFL and SPI1BUFH registers
    //wnr bit is 0 for read, 1 for write
    SPI_CS = 0;
    SPI1BUFL = data_in;
    while(!SPI1STATLbits.SRMT); //while Current or pending transactions
    data_out = SPI1BUFL;
    SPI_CS = 1;
    printf(" Response: %d\n", data_out);
    
    return data_out;
}

void lora_tx(void){
    spi_write(RFM_CMD_LOADTXREG_A); //put 10101010 into TX FIFO reg
    spi_write(RFM_CMD_TX); //transmit
}