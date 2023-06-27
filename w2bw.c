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

#define TIMER_PERIOD_US 500

uint8_t start_meas=0;
//test string for testing the serial comm
const char teststr[]="NNAABBCC\r\n";
uint16_t sendbuf[8];

struct w2bw_meas{
    int16_t Bx;
    int16_t By;
    int16_t Bz;
    uint16_t Temp;
};

__interrupt void cpuTimer0ISR(void){
    start_meas=1;
    GPIO_togglePin(HEARTBEAT_PIN);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void serialize_w2bw(uint16_t* dest, const struct w2bw_meas* meas){
    //first two bytes are for framing
    dest[0]=0x0055;
    dest[1]=0x0055;
    //serialized measurements
    dest[2]=(meas->Bx);
    dest[3]=(meas->Bx)>>8;
    dest[4]=(meas->By);
    dest[5]=(meas->By)>>8;
    dest[6]=(meas->Bz);
    dest[7]=(meas->Bz)>>8;
}

void init_i2c(void){
    I2C_disableModule(I2CA_BASE);
    I2C_initMaster(I2CA_BASE, DEVICE_SYSCLK_FREQ, 200000, I2C_DUTYCYCLE_50);
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
    // SCIA
    GPIO_setPinConfig(GPIO_43_SCIRXDA);
    GPIO_setPinConfig(GPIO_42_SCITXDA);
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

void timer_init(void){
    const uint32_t timer_perios_us=TIMER_PERIOD_US;
    CPUTimer_setPeriod(CPUTIMER0_BASE,DEVICE_SYSCLK_FREQ/1000000*timer_perios_us);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);
    //register & enable CPU Timer0 interrupt
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
}

void sci_comm(void){
    //
    // Configuration for the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // Configuration for the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    //
    // Initialize SCIA and its FIFO.
    //
    SCI_performSoftwareReset(SCIA_BASE);


    //
    // Configure SCIA for echoback.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 460800, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
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
    Bx=0;
    By=0;
    Bz=0;
    Temp=0;
    //read the 8 most significant bytes and fill up the leading 4 bits with either 1s or 0s
    Bx=bytes[0]<<4;
    if(bytes[0]&0b10000000)
        Bx=Bx|(0xF000);
    By=bytes[1]<<4;
    if(bytes[1]&0b10000000)
        By=By|(0xF000);
    Bz=bytes[2]<<4;
    if(bytes[2]&0b10000000)
        Bz=Bz|(0xF000);
    Temp=bytes[3]<<4;
    Bx=Bx+((bytes[4]&0b11110000)>>4);
    By=By+(bytes[4]&0b00001111);
    Bz=Bz+(bytes[5]&0b00001111);
    Temp=Temp+((bytes[5]&0b11000000)>>6);
    meas->Bx=(int16_t)Bx;
    meas->By=(int16_t)By;
    meas->Bz=(int16_t)Bz;
    meas->Temp=Temp;
    return;
}

//definitions for w2bw comm
//----register addresses
const uint8_t ADDR_CONFIG=0x10;
const uint8_t ADDR_MOD=0x11;
const uint8_t ADDR_CONFIG2=0x14;
//----trigger modes
const uint8_t TRIGGER_NONE=0x00;
const uint8_t TRIGGER_AFTER_WRITE_FRAME=0b00100000;
//----bit-fields within registers
//Config Register
const uint8_t CONFIG_REG_MASK_TRIG_AFTER5H=0b00100000;
const uint8_t CONFIG_REG_ODD_PARITY_BIT=0b00000001;
const uint8_t CONFIG_REG_MASK_X2_SENS=0b00001000;
const uint8_t CONFIG_REG_DISABLE_EMP=0b10000000;
//MOD1 register
const uint8_t MOD_REG_MASK_MODE_MASTER=0b00000001;
const uint8_t MOD_REG_MASK_INT_DISABLE=0b00000100;
const uint8_t MOD_REG_MASK_ONEBYTE_EN=0b00010000;
const uint8_t MOD_REG_MASK_ODD_PARITY_BIT=0b10000000;
const uint8_t MOD_REG_MASK_A1=0b00100000;
//MOD2 register
const uint8_t MOD2_REG_MASK_X4_SENS=0b00000001;

//global variables
struct w2bw_meas current_meas;

//global memores of last N measurements
#define N_MEM 1024
int16_t Bxmem[N_MEM]={0};
int16_t Bymem[N_MEM]={0};
int16_t Bzmem[N_MEM]={0};

//#define HIGH_RANGE_LOW_SENS

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

    #ifdef SENSOR_A1
        const uint8_t WRITE_CONFIG_CONFI    G_REG[]={ADDR_CONFIG,
                                                 CONFIG_REG_MASK_TRIG_AFTER5H|CONFIG_REG_MASK_X2_SENS|CONFIG_REG_ODD_PARITY_BIT};
        // TODO : potential_error
        const uint8_t WRITE_CONFIG_MOD1_REG[]={ADDR_MOD,
                                              MOD_REG_MASK_MODE_MASTER|MOD_REG_MASK_ONEBYTE_EN|MOD_REG_MASK_A1};

        const uint8_t WRITE_CONFIG_CONFIG2_REG[]={ADDR_CONFIG2,
                                               MOD2_REG_MASK_X4_SENS};
    #endif
    #ifdef SENSOR_A0
        #ifdef SHORT_RANGE
            const uint8_t WRITE_CONFIG_CONFIG_REG[]={ADDR_CONFIG,
                                                     CONFIG_REG_MASK_TRIG_AFTER5H|CONFIG_REG_MASK_X2_SENS|CONFIG_REG_ODD_PARITY_BIT};
            // TODO : potential_error
            const uint8_t WRITE_CONFIG_MOD1_REG[]={ADDR_MOD,
                                                   MOD_REG_MASK_MODE_MASTER|MOD_REG_MASK_ONEBYTE_EN|MOD_REG_MASK_ODD_PARITY_BIT};

            const uint8_t WRITE_CONFIG_CONFIG2_REG[]={ADDR_CONFIG2,
                                                   MOD2_REG_MASK_X4_SENS};
        #endif
        #ifdef LONG_RANGE
                const uint8_t WRITE_CONFIG_CONFIG_REG[]={ADDR_CONFIG,
                                                         CONFIG_REG_MASK_TRIG_AFTER5H|CONFIG_REG_ODD_PARITY_BIT};
                // TODO : potential_error
                const uint8_t WRITE_CONFIG_MOD1_REG[]={ADDR_MOD,
                                                       MOD_REG_MASK_MODE_MASTER|MOD_REG_MASK_ONEBYTE_EN|MOD_REG_MASK_ODD_PARITY_BIT};

                const uint8_t WRITE_CONFIG_CONFIG2_REG[]={ADDR_CONFIG2};
        #endif
    #endif


    //configure the register CONFIG by writing to it
    i2c_write(0,WRITE_CONFIG_CONFIG_REG,2);
    //configure the register MOD1 by writing to it
    i2c_write(0,WRITE_CONFIG_MOD1_REG,2);
    //configure the register MOD2 by writing to it
    i2c_write(0,WRITE_CONFIG_CONFIG2_REG,2);

    //initialize the SCI comm interface to the PC
    sci_comm();

    //set up timer & enable interrupts
    timer_init();
    Interrupt_enableMaster();
    EINT;
    ERTM;

    uint16_t loop_counter=0;
    for(;;){
        if(start_meas){
            uint8_t read_buf[7];
            //SCI_writeCharArray(SCIA_BASE,(uint16_t*)teststr,sizeof(teststr)/2); //test string for testing the serial comm

            //serialize last measurement and send over UART
            serialize_w2bw(sendbuf,&current_meas);
            SCI_writeCharArray(SCIA_BASE,sendbuf,sizeof(sendbuf));

            i2c_read(0,read_buf,7);
            parse_w2bw_meas(read_buf,&current_meas);
            Bxmem[loop_counter%N_MEM]=current_meas.Bx;
            Bymem[loop_counter%N_MEM]=current_meas.By;
            Bzmem[loop_counter%N_MEM]=current_meas.Bz;

            start_meas=0;
            loop_counter+=1;
        }
    }
}

//
// End of File
//
