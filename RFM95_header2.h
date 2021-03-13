#define SPI_DIR_SDO TRISBbits.TRISB5
#define SPI_DIR_CLK TRISBbits.TRISB1
#define SPI_DIR_SDI TRISBbits.TRISB2
#define SPI_DIR_CS TRISBbits.TRISB3
#define SPI_DIR_RESET TRISBbits.TRISB12

#define SPI_CS LATBbits.LATB3
#define SPI_RESET LATBbits.LATB12

//****************************************************************************
// definitions
//****************************************************************************

//registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

//****************************************************************************
//functions
//****************************************************************************

void spi_init(void);
uint8_t spi_write(uint16_t);
void rfm_init(void);
void lora_tx(void);
uint8_t rfm_write(uint8_t,uint8_t);
uint8_t rfm_read(uint8_t);
void rfm_beginPacket(void);

//****************************************************************************
//Main code
//****************************************************************************

//initializes all needed PORTS and maps SPI to peripheral pins
void spi_init(void){
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
void rfm_init(void){
    printf("RFM init... ");
    
    //put in lora mode, sleep mode
    rfm_write(REG_OP_MODE, 0x80);
    
    // **************this may be problematic
    //frequency setup 915MHz
    long freq = 915000000;
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    
    rfm_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    rfm_write(REG_FRF_MID, (uint8_t)(frf >> 8));
    rfm_write(REG_FRF_LSB, (uint8_t)(frf >> 0));
    
    // set base addresses
    rfm_write(REG_FIFO_TX_BASE_ADDR, 0);
    rfm_write(REG_FIFO_RX_BASE_ADDR, 0);
    
    // set LNA boost
    //the dump said this was 0x23 but code used a weird operation
    rfm_write(REG_LNA, 0x23);
    
    // set auto AGC
    rfm_write(REG_MODEM_CONFIG_3, 0x04);
    
    // set output power to 17 dBm
    //dump said set to 0x84
    rfm_write(REG_PA_DAC, 0x84);
    
    //turn on explicit header mode. 4/5 coding rate, 125 kHz BW
    //dump says 0x72
    rfm_write(REG_MODEM_CONFIG_1, 0x72);
    
    //standby mode
    rfm_write(REG_OP_MODE, 0x81);
    
    printf("RFM initiated.\n");
    printf("\n");
}

void rfm_beginPacket(void){
    // reset FIFO address and payload length
    rfm_write(REG_FIFO_ADDR_PTR, 0);
    rfm_write(REG_PAYLOAD_LENGTH, 0);
}

void rfm_endPacket(void){
    //put in  tx mode
    rfm_write(REG_OP_MODE, 0x83);
    
    //wait for tx to be done
    while(1){
        uint8_t mask = 0b00001000;
        uint8_t flags = rfm_write(REG_IRQ_FLAGS, 0);
        flags = flags & mask;
        if(mask = 0b00001000){
            printf("tx complete\n");
            break;
        }
        __delay_us(1000);
    }
    
    //claer IRQ flags
    rfm_write(REG_IRQ_FLAGS, 0);
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

//adds a write bit and puts an address and data together
uint8_t rfm_write(uint8_t addr, uint8_t data){
    
    uint16_t to_send = (addr << 8) | data;
    
    //as write bit
    to_send = to_send | 0b1000000000000000;
    
    return spi_write(to_send);
}

uint8_t rfm_read(uint8_t addr){
    uint8_t data = 0;
    uint16_t to_send = (addr << 8) | data;
    return spi_write(to_send);
}

void lora_tx(void){
    printf("Sending...\n");
    
    //put in standby
    rfm_write(REG_OP_MODE, 0x81);
    
    //fill tx buffer
    rfm_write(REG_FIFO, 'H');
    rfm_write(REG_FIFO, 'I');
    rfm_write(REG_FIFO, 'D');
    
    // set length to 1 byte
    rfm_write(REG_PAYLOAD_LENGTH, 0x03);
}