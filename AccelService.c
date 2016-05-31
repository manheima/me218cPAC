/****************************************************************************
 Module
   AccelService.c

 Revision
   1.0.1

 Description
   Communication from TIVA to XBee

 Notes

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "AccelService.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"	// Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"  //allows us to use i2c macros
#include "CommService.h"
#include "PacketSender.h" //to get the pot read function

/*----------------------------- Module Defines ----------------------------*/
#define ACCEL_XOUT_H       0x3B   // R  
#define ACCEL_XOUT_L       0x3C   // R  
#define ACCEL_YOUT_H       0x3D   // R  
#define ACCEL_YOUT_L       0x3E   // R  
#define ACCEL_ZOUT_H       0x3F   // R  
#define ACCEL_ZOUT_L       0x40   // R
#define GYRO_XOUT_H        0x43   // R
#define GYRO_XOUT_L        0x44   // R
#define GYRO_YOUT_H        0x45   // R
#define GYRO_YOUT_L        0x46   // R
#define GYRO_ZOUT_H        0x47   // R
#define GYRO_ZOUT_L        0x48   // R
#define ACCEL_POS_CAP      18000
#define ACCEL_NEG_CAP      -15000

#define CONFIG             0x1A
#define ACCEL_CONFIG       0x1C   // R/W
#define PWR_MGMT_1         0x6B   // R/W !!!!!! Most important one to clear
#define RED                0x00  //Red for sending request packet
#define BLUE               0x01  //Blue team
#define LobyistNumMask     0xc0  //1100 0000
#define alpha              0.7    // Used for complementary filter for accel
#define ALL_BITS           0xff<<2
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void InitI2C( void);
static uint8_t GetLobyistNum(void);
static void Init4WaySwitch(void);
static void CalcXaccel( void);
static void CalcYaccel( void);
static void CalcZaccel( void);
static void CalcXgyro( void);
static void CalcYgyro( void);
static void CalcZgyro( void);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
static void InitLED(void);
static uint8_t GetTeamColor( void);
static bool RequestUnpair = false;
static bool RequestBrake = false;

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
//static AccelServiceState_t CurrentState;
static uint8_t MyPriority;
static AccelServiceState_t CurrentState;
static AccelServiceState_t InnerState;
static uint8_t ui8SlaveAddr = 0x68; //1101000;

static uint8_t XaccelDataHigh;
static uint8_t XaccelDataLow;
static uint8_t YaccelDataHigh;
static uint8_t YaccelDataLow;
static uint8_t ZaccelDataHigh;
static uint8_t ZaccelDataLow;
static uint8_t XgyroDataHigh;
static uint8_t XgyroDataLow;
static uint8_t YgyroDataHigh;
static uint8_t YgyroDataLow;
static uint8_t ZgyroDataHigh;
static uint8_t ZgyroDataLow;

static int16_t  Xaccel;
static int16_t  Yaccel;
static int16_t  Zaccel;
static int32_t  LastZaccel = 0;
static uint8_t  accelCounter = 0;
static int16_t  Xgyro;
static int16_t  Ygyro;
static int16_t  Zgyro;

static uint16_t PotVal;
static int8_t   Speed; //Control bit 1
static int8_t   Direction; //Control bit 2
static uint8_t LobyistNum = 6;
static uint8_t Color = RED;
static uint8_t RequestPacket[2];
static uint8_t ControlPacket[5];
bool  LastYDir = false; //used to send events; false = stopped; true = forward
bool  LastXDir = false;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitCommService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

****************************************************************************/
bool InitAccelService ( uint8_t Priority )
{
    MyPriority = Priority;
    InitI2C();
    InitLED();
    Init4WaySwitch();
    //Start the acceleration timer
    ES_Timer_InitTimer(AccelTimer,200);
    CurrentState = WaitForMeasure;
    InnerState = WaitForPositiveShake;
    //Set the first bytes of the Packets (they wont change)
    RequestPacket[0] = 0x00; //Header for request packet
    ControlPacket[0] = 0x02; // Header for Control packet
    
    
    
    return true;
}

/****************************************************************************
 Function
     PostCommService

 Parameters
     ES_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 
****************************************************************************/
bool PostAccelService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunCommService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.

****************************************************************************/
ES_Event RunAccelService( ES_Event ThisEvent )
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    
    switch (CurrentState){
        case WaitForMeasure:  
            //printf("\n\n\n\rX Accel: \t\t");
            //printf("Y Accel: \t\t");
            //printf("Z Accel: \r\n");
            CurrentState = MeasureAccel;
            //Start the acceleration timer
            ES_Timer_InitTimer(AccelTimer,200);
        break;
        case MeasureAccel: 
            if (ThisEvent.EventType == ES_TIMEOUT && ((ThisEvent.EventParam == AccelTimer)||(ThisEvent.EventParam ==ShakeTimer))){
                
                if (QueryPacketSenderState() == Unpaired_NoPair){
                    //Check what team we are on
                    Color = GetTeamColor();
                }
                
                //if we are in Unpaired_NoPair state in packet sender then check for a hook shake
                if ((QueryPacketSenderState() == Unpaired_NoPair) || (QueryPacketSenderState() == Paired_Ready2Send) || (QueryPacketSenderState() == Paired_WaitingForResponse)){   
                    //CalcXgyro();
                    CalcYgyro();
                   // printf("Y Gyro: %d \r\n", Ygyro);
                    switch(InnerState){
                        case WaitForPositiveShake:
                            if (Ygyro>16000){
                                //printf("FIRST SHAKE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \r\n");
                                //Start timer 
                                ES_Timer_InitTimer(ShakeTimer, 250);
                                //Go to state where you are waiting for Xgyro to be >-100000
                                InnerState = WaitForNegativeShake;
                            }
                        break;
                        case WaitForNegativeShake:
                            if (Ygyro < -12000){
                                //printf("SECOND SHAKE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \r\n");
                                //ReStart timer 
                                ES_Timer_InitTimer(ShakeTimer, 250);
                                //Go to state where you are waiting for Xgyro to be >-100000
                                InnerState = WaitForSecondPositiveShake;
                            }else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ShakeTimer){
                                //go back to wait for positive shake
                                InnerState = WaitForPositiveShake;
                            }
                        break;
                        case WaitForSecondPositiveShake:
                            if (Ygyro>12000){
                                if (QueryPacketSenderState() == Unpaired_NoPair){
                                   ThisEvent.EventType = ES_HookShake;
                                   PostPacketSender(ThisEvent);
                                   RequestUnpair = false;
                                   InnerState = WaitForPositiveShake;
                                }else{
                                    if (RequestBrake){
                                        RequestBrake = false;
                                    }else{
                                        RequestBrake = true;
                                    }
                                    InnerState = WaitForPositiveShake;
                                }
                            }else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ShakeTimer){
                                //go back to wait for positive shake
                                InnerState = WaitForPositiveShake;
                            }
                        break;
                    
                    }  
                }
                                  
               //CalcYgyro(); 
               //CalcZgyro();
                
                
           //     CalcXaccel();
           //     CalcYaccel();
                CalcZaccel();
                //printf("Original Speed: %d\r\n",Zaccel);
                PotVal = Pot_Read(); //0-> 4095
                //printf("\r\n PotVal: %d \r\n", PotVal);
                if (PotVal < 582){
                    Direction = -128;
                }
                else if ((PotVal >= 582)&&(PotVal < 653)){
                    Direction = -102;
                }
                else if ((PotVal >= 653)&&(PotVal < 718)){
                    Direction = -77;
                }
                else if ((PotVal >= 718)&&(PotVal < 759)){
                    Direction = -51;
                }
                else if ((PotVal >= 759)&&(PotVal < 829)){
                    Direction = -26;
                }
                else if ((PotVal >= 829)&&(PotVal < 935)){
                    Direction = -0;
                }
                else if ((PotVal >= 935)&&(PotVal < 1186)){
                    Direction = 25;
                }
                else if ((PotVal >= 1186)&&(PotVal < 1606)){
                    Direction = 51;
                }
                else if ((PotVal >= 1606)&&(PotVal < 2216)){
                    Direction = 76;
                }
                else if ((PotVal >= 2216)&&(PotVal < 3014)){
                    Direction = 102;
                }
                else{
                    Direction = 127;
                }
                //Direction = (PotVal - 2047)*127/2047;
                //Direction += 70; // calibration tweak
                
                //printf("PotVal = %d\t\t", PotVal);
                //printf("\r\n Direction: %d \r\n",Direction);
                //Speed = -Xaccel*127/16383;//byte 1 is speed
                if (Zaccel < 0) { 
                    Speed = Zaccel*127/ACCEL_NEG_CAP;
                } else {
                    Speed = -Zaccel*127/ACCEL_POS_CAP;
                }
                                
                //printf("Speed: %d \t\t", Speed);
                if (abs(Speed)<25){
                    Speed = 0;
                }
                
                //printf("Speed Dead Zone: %d \r\n", Speed);
                LobyistNum = GetLobyistNum();
                
                //printf("LobyistNum: %d \r\n",LobyistNum);
                //LobyistNum = 14;
                //LobyistNum = 0x06; //take this out when we want actual lobyist num
                //Color = GetColor();
                //Direction = -Xaccel*127/16383; //byte 2 is direction (IF we want to use accelerometer for direction
                //Speed = 0; //TODO: Take this out!!!
                //Direction =  0;
                //printf("Speed: %d \r\n", Speed);
                //printf("Direction: %d\r\n", Direction);
                
                
                // Update the bytes of the request packet
                if (Color == RED){
                    RequestPacket[1] = (RequestPacket[1]&BIT7LO);
                }else if (Color == BLUE){
                    RequestPacket[1] = (RequestPacket[1]|BIT7HI);
                }
                //Update Lobbyist number
                RequestPacket[1]=(RequestPacket[1]&LobyistNumMask)|LobyistNum;
                
                //Update the bytes of the Control Packet
                ControlPacket[1] = Speed; //byte 1 is speed
                //ControlPacket[1] = 0x20; // debug
                //printf("Speed: %d \r\n", Speed); 
                //ControlPacket[2] = PotVal; //byte 2 is direction
                ControlPacket[2] = Direction;
                //printf("Direction: %d \r\n", Direction);
                // special actions:
                RequestBrake = false; //turn off request brake
                if (RequestUnpair || (Color != GetTeamColor())){
                    ControlPacket[3] = 0x02;
                } else if (RequestBrake){
                    //printf("brake requested !!!!!!!!!!!\r\n");
                    ControlPacket[3] = 0x01;
                } else {
                    ControlPacket[3] = 0x00;
                }
                //Calculate checksum -actually just do this in CommService    
                //ControlPacket[4] = 0xff - (ControlPacket[0]+ControlPacket[1]+ControlPacket[2]+ControlPacket[3]);
                
                //Start the acceleration timer
                ES_Timer_InitTimer(AccelTimer,40);
            } else if (ThisEvent.EventType == ES_Unpair){
                RequestUnpair = true;
            }              
        break;     
    }
    
         
    return ReturnEvent;
}
/*********************************************************************************
GetterFunctions for getting Packet location
************************************************************************************/
uint8_t* GetRequestPacket( void){
    return RequestPacket;
}
uint8_t* GetControlPacket( void){
    return ControlPacket;
}






/***************************************************************************
 private functions
 ***************************************************************************/



static void InitI2C( void){
   //enable I2C module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
 
    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
     
    //enable GPIO peripheral that contains I2C 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
 
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
     
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
 
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
     
    //clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
    
    //clear the sleep bit in power management 1
    uint8_t num_of_args = 2;
    I2CSend(0x68, num_of_args, PWR_MGMT_1, 0x00);
    
    //set the low pass filter to in CONFIG DLPF_CFG to 6
    I2CSend(0x68, num_of_args, CONFIG, 0x06);
    
}


//Function found from: https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
 
    //specify register to be read
    I2CMasterDataPut(I2C1_BASE, reg);
 
    //send control byte and register address byte to slave device
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C1_BASE));
     
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
     
    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C1_BASE));
     
    //return data pulled from the specified register
    return I2CMasterDataGet(I2C1_BASE);
}


//Function found from: https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
     
    //stores list of variable number of arguments
    va_list vargs;
     
    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(I2C1_BASE, va_arg(vargs, uint32_t));
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C1_BASE));
         
        //"close" variable argument list
        va_end(vargs);
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C1_BASE));
         
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(uint8_t i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C1_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C1_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C1_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C1_BASE));
         
        //"close" variable args list
        va_end(vargs);
    }
}

static void CalcXaccel( void){
     XaccelDataLow = I2CReceive(ui8SlaveAddr, ACCEL_XOUT_L);
     XaccelDataHigh = I2CReceive(ui8SlaveAddr, ACCEL_XOUT_H);
     Xaccel = ~((XaccelDataHigh<<8)+XaccelDataLow)+1;
    //printf("\nXaccel: %d \t\t",Xaccel);
}
static void CalcYaccel( void){
     YaccelDataLow = I2CReceive(ui8SlaveAddr, ACCEL_YOUT_L);
     YaccelDataHigh = I2CReceive(ui8SlaveAddr, ACCEL_YOUT_H);
     Yaccel = ~((YaccelDataHigh<<8)+YaccelDataLow)+1;
     //printf("\nYaccel: %d \r\n\n",Yaccel);
}
static void CalcZaccel( void){
    static int16_t ZaccelTemp;
    ZaccelDataLow = I2CReceive(ui8SlaveAddr, ACCEL_ZOUT_L);
    ZaccelDataHigh = I2CReceive(ui8SlaveAddr, ACCEL_ZOUT_H);
    ZaccelTemp = (~((ZaccelDataHigh<<8)+ZaccelDataLow)+1);
    if (ZaccelTemp > ACCEL_POS_CAP) {
        Zaccel = ACCEL_POS_CAP;
    } else if (ZaccelTemp < ACCEL_NEG_CAP) {
        Zaccel = ACCEL_NEG_CAP;
    } else {
        Zaccel = ZaccelTemp;
    }
    
    //printf("Zaccel:  %d\r\n",Zaccel);
    //LastZaccel = (1-alpha)*LastZaccel + alpha*(~((ZaccelDataHigh<<8)+ZaccelDataLow)+1);
    //LastZaccel += (int32_t) (~((ZaccelDataHigh<<8)+ZaccelDataLow)+1);
    //Only update Zaccel when counter reaches 5
//    if (++accelCounter  == 4) {
//        Zaccel = (int16_t) (LastZaccel/5);
//        LastZaccel = 0;
//        accelCounter = 0;
//        //Zaccel = (~((ZaccelDataHigh<<8)+ZaccelDataLow)+1);
//        //printf("Zaccel:  %d\r\n",Zaccel);
//    }
}
static void CalcXgyro( void){
     XgyroDataLow = I2CReceive(ui8SlaveAddr, GYRO_XOUT_L);
     XgyroDataHigh = I2CReceive(ui8SlaveAddr, GYRO_XOUT_H);
     Xgyro = ~((XgyroDataHigh<<8)+XgyroDataLow)+1;
     //printf("\nXGYRO: %d \r\n",Xgyro);
}
static void CalcYgyro( void){
     YgyroDataLow = I2CReceive(ui8SlaveAddr, GYRO_YOUT_L);
     YgyroDataHigh = I2CReceive(ui8SlaveAddr, GYRO_YOUT_H);
     Ygyro = ~((YgyroDataHigh<<8)+YgyroDataLow)+1;
     //printf("\nYGYRO: %d \t",Ygyro);
}
static void CalcZgyro( void){
    ZgyroDataLow = I2CReceive(ui8SlaveAddr, GYRO_ZOUT_L);
    ZgyroDataHigh = I2CReceive(ui8SlaveAddr, GYRO_ZOUT_H);
    Zgyro = ~((ZgyroDataHigh<<8)+ZgyroDataLow)+1;
    //printf("ZGYRO: %d \r\n",Zgyro);
}

static uint8_t GetTeamColor( void){
    //if pin is high we are blue, otherwise we are red
    if((HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + (ALL_BITS)))&(GPIO_PIN_4))==GPIO_PIN_4){
        //turn on blue LED
        // Turn off all of the LEDs
        HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + (0xff<<2))) &= ~(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        // Turn on the new LEDs
        HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + (0xff<<2))) |= BIT2HI;
        //printf("BLUE\r\n");
        return BLUE;
    }else{
        //turn on red LED
        // Turn off all of the LEDs
        HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + (0xff<<2))) &= ~(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        // Turn on the new LEDs
        HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + (0xff<<2))) |= BIT1HI;
        //printf("RED\r\n");
        return RED;
    }
}

static void InitLED(void)
{
	volatile uint32_t Dummy;
	// enable the clock to Port F
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5;
	// kill a few cycles to let the peripheral clock get going
	Dummy = HWREG(SYSCTL_RCGCGPIO);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	// Enable pins for digital I/O
	HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	
	// make pins 1,2 & 3 on Port F into outputs
	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) |= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	//GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

static void Init4WaySwitch(void){
    // initialize PC4,5,6,7 as pulled-up digital inputs
    // enable the clock to Port F
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
	// kill a few cycles to let the peripheral clock get going
    volatile uint32_t Dummy;
	Dummy = HWREG(SYSCTL_RCGCGPIO);
    // Enable pins for digital I/O
	HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	// make pins 1,2 & 3 on Port F into inputs
	HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    // enable pull-ups for PC4,5,6,7
    HWREG(GPIO_PORTC_BASE+GPIO_O_PUR) |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}

static uint8_t GetLobyistNum(void){
    if((HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + (ALL_BITS)))&GPIO_PIN_4) == 0x00){
        return 0x01;
    } else if ((HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + (ALL_BITS)))&GPIO_PIN_5) == 0x00){
        return 0x00;
    } else if ((HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + (ALL_BITS)))&GPIO_PIN_6) == 0x00){
        return 0x03;
    } else if ((HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + (ALL_BITS)))&GPIO_PIN_7) == 0x00){
        return 0x02;
    } else{
        return 5;
    }
}
/*****************************   End   *********************************/
