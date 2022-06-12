//###########################################################################
/*
 * This is a minimum working example which demonstrates usage of the SCI Module
 * for the MSP430F28379D
 *
 * Pinout
 *
 * I2C :    SDA (J1.10 / GPIO104)
 *          SCL (J1.9 / GPIO105)
 * SCIA :   GPIO43 / GPIO42 (on board)
 * HEARTBEAT : GPIO0 (J4.40)
 * POWER_SUPPLY_PIN : GPIO1 (J4.39)
*/
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "w2bw.h"

struct w2bw_meas{
    uint16_t Bx;
    uint16_t By;
    uint16_t Bz;
    uint16_t Temp;
};

void init_i2c(void){
    I2C_disableModule(I2CA_BASE);
    I2C_initMaster(I2CA_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_50);
    I2C_setConfig(I2CA_BASE, I2C_MASTER_SEND_MODE);
    I2C_setSlaveAddress(I2CA_BASE, 0);
    I2C_disableLoopback(I2CA_BASE);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_setDataCount(I2CA_BASE, 1);
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);
    I2C_enableFIFO(I2CA_BASE);
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TXEMPTY, I2C_FIFO_RXEMPTY);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);
    I2C_enableModule(I2CA_BASE);
}

void pinmux_init(void){
    // GPIO11 -> HEARTBEAT Pinmux
    GPIO_setPinConfig(GPIO_0_GPIO0);
    GPIO_setDirectionMode(HEARTBEAT_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(HEARTBEAT_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(HEARTBEAT_PIN, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(HEARTBEAT_PIN, GPIO_QUAL_SYNC);
    // SDA & SCL lines for I2C
    GPIO_setPinConfig(GPIO_105_SCLA);
    GPIO_setPadConfig(105, GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(105, GPIO_QUAL_ASYNC);
    GPIO_setPinConfig(GPIO_104_SDAA);
    GPIO_setPadConfig(104, GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(104, GPIO_QUAL_ASYNC);
    // Power Supply Pin
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setPadConfig(POWER_SUPPLY_PIN,GPIO_PIN_TYPE_STD);
}

void gpio_init(void){
    //HEARTBEAT initialization
    GPIO_setDirectionMode(HEARTBEAT_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(HEARTBEAT_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(HEARTBEAT_PIN, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(HEARTBEAT_PIN, GPIO_QUAL_SYNC);

    //W2BW_SUPPLY initialization
    GPIO_setDirectionMode(POWER_SUPPLY_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(POWER_SUPPLY_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(POWER_SUPPLY_PIN, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(POWER_SUPPLY_PIN, GPIO_QUAL_SYNC);
}

inline void w2bw_power_enable(void){
    GPIO_writePin(POWER_SUPPLY_PIN,0);
}

inline void w2bw_power_disable(void){
    GPIO_writePin(POWER_SUPPLY_PIN,1);
}

uint8_t i2c_write(uint8_t addr,const uint8_t* data,uint8_t count){
    //wait until the bus is not busy anymore (e.g. STOP condition has been sent)
    while(I2C_getStopConditionStatus(I2CA_BASE))
        ;

    //set the I2C slave address to the write address
    I2C_setSlaveAddress(I2CA_BASE,I2C_W2BW_ADDRESS);
    //set the number of bytes to be sent
    I2C_setDataCount(I2CA_BASE, count);
    //push the data to the I2C TX FIFO
    uint8_t i=0;
    for(i=0; i<count; i++){
        I2C_putData(I2CA_BASE,data[i]);
    }
    //set the I2C mode to master
    I2C_setConfig(I2CA_BASE, I2C_MASTER_SEND_MODE);
    //send the start condition
    I2C_sendStartCondition(I2CA_BASE);
    //send the stop condition
    I2C_sendStopCondition(I2CA_BASE);
    return 0;
}

uint8_t i2c_read(uint8_t addr,uint8_t* data,uint8_t count){
    //wait until the bus is not busy anymore (e.g. STOP condition has been sent)
    while(I2C_getStopConditionStatus(I2CA_BASE))
        ;

    uint8_t i=0;
    //set the I2C slave address to the write address
    I2C_setSlaveAddress(I2CA_BASE,I2C_W2BW_ADDRESS);
    //set the I2C mode to master & repeat mode (data is read until STOP condition)
    I2C_setConfig(I2CA_BASE, I2C_MASTER_RECEIVE_MODE|I2C_REPEAT_MODE);
    //send the start condition
    I2C_sendStartCondition(I2CA_BASE);


    uint8_t bytes_read=0;
    while((count-bytes_read)>16){
        //wait until 16 bytes have been read
        while(I2C_getRxFIFOStatus(I2CA_BASE)!=I2C_FIFO_RX16)
            ;
        //copy the 16 bytes to the data buffer
        for(i=0; i<16; i++)
            data[bytes_read+i]=I2C_getData(I2CA_BASE);
        bytes_read+=16;
    }
    uint8_t remaining_bytes=count-bytes_read;
    //wait for the remaining bytes to be read into the RX FIFO
    while(I2C_getRxFIFOStatus(I2CA_BASE)!=remaining_bytes)
        ;
    //copy the remaining bytes
    //retreive the latest bit (e.g. the bit that was received most recently)
    data[remaining_bytes-1]=I2C_getData(I2CA_BASE);
    //copy the remaining received bits from the FIFO
    for(i=0; i<remaining_bytes-1; i++)
        data[bytes_read+i]=I2C_getData(I2CA_BASE);

    //send the stop condition
    I2C_sendStopCondition(I2CA_BASE);
    return 0;
}

void parse_w2bw_meas(const uint8_t* bytes,struct w2bw_meas* meas){
    uint16_t Bx,By,Bz,Temp;
    Bx=bytes[0]<<4;
    By=bytes[1]<<4;
    Bz=bytes[2]<<4;
    Temp=bytes[3]<<4;
    Bx=Bx+((bytes[4]&0b11110000)>>4);
    By=By+(bytes[4]&0b00001111);
    Bz=Bz+(bytes[5]&0b00001111);
    Temp=Temp+((bytes[5]&0b11000000)>>6);
    meas->Bx=Bx;
    meas->By=By;
    meas->Bz=Bz;
    meas->Temp=Temp;
    return;
}

//definitions for w2bw comm
//----register addresses
const uint8_t ADDR_MOD=0x11;
const uint8_t ADDR_CONFIG=0x10;
//----trigger modes
const uint8_t TRIGGER_NONE=0x00;
const uint8_t TRIGGER_AFTER_WRITE_FRAME=0b00100000;
//----bit-fields within registers
//MOD1 register
const uint8_t MOD_REG_MASK_MODE_MASTER=0b00000001;
const uint8_t MOD_REG_MASK_INT_DISABLE=0b00000100;
const uint8_t MOD_REG_MASK_ONEBYTE_EN=0b00010000;
const uint8_t MOD_REG_MASK_ODD_PARITY_BIT=0b10000000;
//Config Register
const uint8_t CONFIG_REG_MASK_TRIG_AFTER5H=0b00100000;
const uint8_t CONFIG_REG_ODD_PARITY_BOT=0b00000001;

//global variables
struct w2bw_meas current_meas;

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();
    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();
    //
    // Initialize PIE and clear PIE registers. Disable CPU interrupts.
    //
    Interrupt_initModule();
    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //init the I2C interface
    pinmux_init();
    gpio_init();
    init_i2c();

    //power cycle the sensor
    //disable the I2C first
    I2C_disableModule(I2CA_BASE);
    w2bw_power_disable();
    DEVICE_DELAY_US(200000u);
    w2bw_power_enable();
    DEVICE_DELAY_US(2000000u);
    //enable I2C module again
    I2C_enableModule(I2CA_BASE);

    //set the mode to master controlled mode, enable interrupts (associated bit is zero), enable 1 byte read, write parity bit
    //Hint: Parity bit needs to be set correctly, otherwise sensor will not communicate
    // format { [Trigger-Bits,Register-Address] , [Register Contents] }
    const uint8_t WRITE_CONFIG_MOD_REG[]={ADDR_MOD|TRIGGER_AFTER_WRITE_FRAME,
                                          MOD_REG_MASK_ODD_PARITY_BIT|MOD_REG_MASK_MODE_MASTER|MOD_REG_MASK_ONEBYTE_EN};
    const uint8_t WRITE_CONFIG_CONFIG_REG[]={ADDR_CONFIG,
                                             CONFIG_REG_ODD_PARITY_BOT|CONFIG_REG_MASK_TRIG_AFTER5H};
    //configure the register MOD1 by writing to it
    i2c_write(0,WRITE_CONFIG_MOD_REG,2);
    //configure the register CONFIG by writing to it
    i2c_write(0,WRITE_CONFIG_CONFIG_REG,2);

    for(;;){
        DEVICE_DELAY_US(200000u);
        uint8_t read_buf[6];
        i2c_read(0,read_buf,6);
        parse_w2bw_meas(read_buf,&current_meas);
    }
}

//
// End of File
//
