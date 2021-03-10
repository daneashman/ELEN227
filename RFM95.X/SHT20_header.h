#define I2C_DIR_SCL TRISAbits.TRISA14
#define I2C_DIR_SDA TRISAbits.TRISA15

void i2c_init(void)
{
    ANSA = 0;
    ODCA = 0xFFFF;
    I2C1CONLbits.I2CSIDL = 1;   //stops module in idle mode
    I2C1CONLbits.A10M = 0;      //7 bit slave address mode
    I2C1BRG = 0x12;             //set baudrate
    I2C1CONLbits.DISSLW = 0;    //disable slew rate
    I2C1CONLbits.I2CEN = 1;     //enable I2C module and config SDA/SCL pins
}