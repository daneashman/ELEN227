#define SPI_DIR_SDO TRISBbits.TRISB5
#define SPI_DIR_CLK TRISBbits.TRISB1
#define SPI_DIR_SDI TRISBbits.TRISB2
#define SPI_DIR_CS TRISBbits.TRISB3
#define SPI_DIR_RESET TRISBbits.TRISB12

#define SPI_CS LATBbits.LATB3
#define SPI_RESET LATBbits.LATB12

//all addresses contain a write bit
#define REG_OPMODE          0x81
#define REG_TXBASE          0x8E
#define REG_SYNCWORD_CON    0xA7
#define REG_SYNCWORD        0xA8
#define REG_FRFMSB          0x86
#define REG_FRFMID          0x87
#define REG_FRFLSB          0x88
#define REG_SF              0x9E
#define REG_LNA             0x0C
#define REG_MODEMCON        0x26
#define REG_TXFIFO          0x00
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_PAYLOAD_LENGTH  0x22

void spi_init(void);
uint8_t spi_write(uint16_t);
void rfm_init(void);
void lora_tx(void);
uint8_t rfm_write(uint8_t,uint8_t);

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
    printf("RFM init... ");
    
    //start with read or write bit, 1=w 0=r
    //7 bit reg address
    //8 bit data
    
    //sleep mode
    rfm_write(REG_OPMODE, 0b00000000);
    
    //frequency setup 915MHz
    rfm_write(REG_FRFMSB, 0b11100100);
    rfm_write(REG_FRFMID, 0b11000000);
    rfm_write(REG_FRFLSB, 0b00000000);
    
    //config and set sync word to 0x34
    rfm_write(REG_SYNCWORD_CON, 0b00010000);
    rfm_write(REG_SYNCWORD, 0x34);
    
    //lora mode, sleep
    rfm_write(REG_OPMODE, 0b10000000);
    
    //set tx base addresss to 0
    rfm_write(REG_TXBASE, 0);
    
    //set LNA to 0x03
    rfm_write(REG_LNA, 0x03);
    
    //auto AGC
    rfm_write(REG_MODEMCON, 0x04);
    
    //SF = 7, normal mode, CRC on,
    rfm_write(REG_SF, 0b01110100);
    
    //Standby mode
    rfm_write(REG_OPMODE, 0b10000001);
    
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
    //printf(" Response: %d\n", data_out);
    
    return data_out;
}

uint8_t rfm_write(uint8_t addr, uint8_t data){
    uint16_t to_send = (addr << 8) | data;
    return spi_write(to_send);
}

void lora_tx(void){
    //put in standby
    rfm_write(REG_OPMODE, 0b10000001);
    
    //check header mode (explixite or implicite)??
   
    //reset FIFO address and payload length
    rfm_write(REG_FIFO_ADDR_PTR, 0); 
    rfm_write(REG_PAYLOAD_LENGTH, 0); 
    
    //fill tx buffer
    rfm_write(REG_FIFO_ADDR_PTR, 0xAA); 
   
    //initiate transmission
    rfm_write(REG_OPMODE, 0b10000011);
    
    
}