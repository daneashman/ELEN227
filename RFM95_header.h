#define SPI_DIR_SDO TRISBbits.TRISB5
#define SPI_DIR_CLK TRISBbits.TRISB1
#define SPI_DIR_SDI TRISBbits.TRISB2
#define SPI_DIR_CS TRISBbits.TRISB3
#define SPI_DIR_RESET TRISBbits.TRISB12

#define SPI_CS LATBbits.LATB3
#define SPI_RESET LATBbits.LATB12

//all addresses contain a write bit
#define REG_TXFIFO          0x00
#define REG_OPMODE          0x01
#define REG_PACONFIG        0x09
#define REG_PARAMP          0x0A
#define REG_OCP             0x0B
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_TXBASE_ADDR     0x0E
#define REG_MODEMCON1       0x1D
#define REG_MODEMCON2       0x1E
#define REG_PREAM_LEN_MSB   0x20
#define REG_PREAM_LEN_LSB   0x21
#define REG_PAYLOAD_LEN     0x22
#define REG_PAYLOAD_MAXLEN  0x23
#define REG_FREQ_HOPPING_T  0x24
#define REG_MODEMCON3       0x26

//already set write bit on these:
#define REG_FRFMSB          0x86
#define REG_FRFMID          0x87
#define REG_FRFLSB          0x88

//Tomorrow: set pre-lora mode settings according to REG dump

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
    
    //config and set sync word to 0x34
    rfm_write(REG_SYNCWORD_CON, 0b00010000);
    rfm_write(REG_SYNCWORD, 0x34);
    
    //lora mode, sleep **************************************************
    rfm_write(REG_OPMODE, 0b10000000);
    
    //frequency setup 915MHz
    rfm_write(REG_FRFMSB, 0xA4);
    rfm_write(REG_FRFMID, 0xC0);
    rfm_write(REG_FRFLSB, 0x00);
    
    //configure power settings
    rfm_write(REG_PACONFIG, 0x8F);
    
    //rise fall time
    rfm_write(REG_PARAMP, 0x09);
    
    //current overload protection
    rfm_write(REG_OCP, 0x2B);
    
    //LNA gain config
    rfm_write(REG_LNA, 0x23);
    
    //FIFO pointers
    rfm_write(REG_FIFO_ADDR_PTR, 0x04);
    
    //set tx base addresss to 0
    rfm_write(REG_TXBASE_ADDR, 0x00);
    
    //set BW = 125kHz and coding rate to 4/5, explicite header mode
    rfm_write(REG_MODEMCON1, 0x72);
    
    //SF = 7, normal mode (single packet), CRC off
    rfm_write(REG_MODEMCON2, 0x70);
    
    //preamble length MSB = 0 + 4.25 symbols
    rfm_write(REG_PREAM_LEN_MSB, 0x00)
    
    //preamble length LSB = 8
    rfm_write(REG_PREAM_LEN_LSB, 0x08)
    
    //payload length = 3 bytes     
    rfm_write(REG_PAYLOAD_LEN, 0x03)
            
    //payload max length = 0xFF   
    rfm_write(REG_PAYLOAD_MAXLEN, 0xFF)  
            
    //set frequency hopping period to 0
    rfm_write(REG_FREQ_HOPPING_T, 0x00)
            
    // LNA gain set by the internal AGC loop, static node
    rfm_write(REG_MODEMCON3, 0x04)
    
    //Standby mode
    rfm_write(REG_OPMODE, 0b10000001);
    
    printf("RFM initiated.\n");
    printf("\n");
}

//writes a 16-bit value to the SPI bus and saves the received value
uint8_t spi_write(uint16_t data_in){
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
    printf("Sending...\n");
    
    //put in standby
    rfm_write(REG_OPMODE, 0b10000001);
    
    //check header mode (explixite or implicite)??
   
    //reset FIFO address and payload length
    rfm_write(REG_FIFO_ADDR_PTR, 0); 
    rfm_write(REG_PAYLOAD_LENGTH, 1); 
    
    //fill tx buffer
    rfm_write(REG_TXFIFO, 0xAA); 
   
    //initiate transmission
    rfm_write(REG_OPMODE, 0b10000011);
    
    //wait for tx-done flag to set
    int i;
    for(i=0;i<5;i++){
        uint8_t mask = 0b00001000;
        uint8_t flags = rfm_write(REG_IRQ_FLAGS, 0);
        flags = flags & mask;
        if(mask = 0b00001000){
            printf("tx complete\n");
            break;
        }
        __delay_us(1000);
    }
    printf("end\n");
}