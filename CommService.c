/****************************************************************************
 Module
   CommService.c

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
#include "CommService.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_uart.h"
#include "inc/hw_nvic.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"	// Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "inc/tm4c123gh6pm.h"
#include "PacketSender.h"
#include "AccelService.h"
/*----------------------------- Module Defines ----------------------------*/
#define PairMask 0x01
#define DecryptMask 0x02
#define ALL_BITS (0xff<<2) 

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void initUART( void);
static void MakePacket(uint8_t* DataPtr);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static CommServiceState_t CurrentState;
static uint8_t MyPriority;

static uint8_t ReceiveLength;  // length of array that we are receiving
static uint8_t ReceiveCounter; // index of array that we are currently writing
static uint8_t ReceiveArray[109]; // RecvArray: holds bytes from received packet
static uint8_t ReceiveCheckSum = 0; //keeps running total of data bytes received
static bool ControlFlag = false; //tells us if we are sending a control packet so we can insert the checksum
static uint8_t* EncryptionPointer; //stores the pointer of the encyption array
static uint8_t EncryptionCounter = 1;
static uint8_t EncryptedCheckSum;
static uint8_t data;
// List different data arrays here
#define TESTDATA1 0;
#define TESTDATA2 1;
#define DRIVE 2;
#define TURNRIGHT 3;
#define TURNLEFT 4;
#define STOPMOTORS 5;
#define LED_ON 6;
#define LED_OFF 7;
#define LIFT_OFF 8;
#define LIFT_ON 9;


// PacketArray to hold transmit data (set to hold max number of possible bytes)
static uint8_t PacketArray[109];
static uint8_t Checksum = 0;
static uint8_t DataLength = 0;
static uint8_t PacketLength = 0;
static uint8_t SourceAddressH;
static uint8_t SourceAddressL;

    
    
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

 Author
     Margaret and Aaron
****************************************************************************/
bool InitCommService ( uint8_t Priority )
{
    MyPriority = Priority;
    // put us into the Initial PseudoState
    CurrentState = WaitFor7E;
    initUART(); //Init UART module 1 (PB0 = Rx; PB1 = Tx)

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

 Author
     Aaron and Margaret
****************************************************************************/
bool PostCommService( ES_Event ThisEvent )
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
 Author
   Margaret and Aaron and Alex
****************************************************************************/
ES_Event RunCommService( ES_Event ThisEvent )
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    CommServiceState_t NextState = CurrentState;
    
    switch ( CurrentState )
    {
        case WaitFor7E:  
       printf("\r\nEvent: %d\r\n",ThisEvent.EventType);
                                    
            if ( ThisEvent.EventType == ES_ReceivedByte && ThisEvent.EventParam == 0x7E){  
                NextState = WaitForMSB;
                ES_Timer_InitTimer(ReceiveTimer, 200);
            }
        break;
        case WaitForMSB:  
            if ( ThisEvent.EventType == ES_ReceivedByte && ThisEvent.EventParam == 0x0){  
                NextState = WaitForLSB;
                ES_Timer_InitTimer(ReceiveTimer, 200);
             }
            else if ( ThisEvent.EventType == ES_ReceivedByte && ThisEvent.EventParam != 0x0){ 
                NextState = WaitFor7E;
             }
            else if ( ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ReceiveTimer){  
                NextState = WaitFor7E;
             }
        break;
        case WaitForLSB:
            if ( ThisEvent.EventType == ES_ReceivedByte){  
                ReceiveLength = ThisEvent.EventParam;
                ReceiveCounter = ReceiveLength;
                ReceiveCheckSum = 0;
                NextState = SuckUpPacket;
                ES_Timer_InitTimer(ReceiveTimer, 200);
            }
            else if ( ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ReceiveTimer){  
                NextState = WaitFor7E;
             }
         break;
         case SuckUpPacket:
            if ( ThisEvent.EventType == ES_ReceivedByte){  
                if(ReceiveCounter != 0){
                    ReceiveArray[ReceiveLength-ReceiveCounter]=ThisEvent.EventParam;
                    ReceiveCheckSum += ThisEvent.EventParam;
                    ReceiveCounter--;
                    ES_Timer_InitTimer(ReceiveTimer, 200);
                }
                else{
                    NextState = WaitFor7E;
                        if (ReceiveCheckSum + ThisEvent.EventParam != 0xFF){                    
                        //Raise a flag for bad checksum
                        printf("\n\nBad checksum!!!!!!!!!!\r\n\n\n");
                        printf("RecvArray Bad Checksum: \r\n");
                        for(int i=0; i<ReceiveLength; i++){
                            printf("   %x   \r\n", ReceiveArray[i]);
                        }
                        SendPacket(GetControlPacket(),true); //change this to the most likely packet
                    }
                    else{
                        //printf("RecvArray: \r\n");
                        for(int i=0; i<ReceiveLength; i++){
                            //printf("   %x   \r\n", ReceiveArray[i]);
                        }
                        //Check if the receive array is result of a transmit or an incoming packet
                        //If the recieve array is an incoming packet 
                        if (ReceiveArray[0] == 0x81){ 
                           // printf("Data was an Incoming Packet from PAC! \r\n");
                            //Store source address from ReceiveArray[1]: Note that Packet Sender uses this value
                            if ((QueryPacketSenderState() == Unpaired_NoPair)||(QueryPacketSenderState() == Unpaired_TryPair)){
                                SourceAddressH = ReceiveArray[1];
                                SourceAddressL = ReceiveArray[2]; 
                            }
                            //Data starts at index 4 of recieve array
                            //First make sure that the incoming packet is a Status packet
                            if (ReceiveArray[5] == 0x03){
                                //Tell the packet sender service that we got a response
                                ThisEvent.EventType = ES_StatusReceived;
                                PostPacketSender(ThisEvent);
                                //Check if there was a decryption error (bit 1 hi) of rcvarray[5]
                                //Check if the pair bit is set
                                if ((ReceiveArray[6]&DecryptMask) == DecryptMask){
                                    printf("Decryption Error! \r\n");
                                }
                                if ((ReceiveArray[6]&PairMask) == PairMask){
                                    ThisEvent.EventType = ES_PairBitSet;
                                    PostPacketSender(ThisEvent);
                                }else if ((ReceiveArray[6]&PairMask) != PairMask)
                                    ThisEvent.EventType = ES_PairBitClear;
                                    PostPacketSender(ThisEvent);
                                }                                    
                            
                        }else if (ReceiveArray[0] == 0x89){
                             if (ReceiveArray[2] == 1){
                                  printf("Nack (XBEE) \r\n\n");
                             }else {
                                  printf("Ack (XBEE) \r\n\n");
                             }
                            
                        }
                        
                    }
                }
            
            }
            else if ( ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ReceiveTimer){  
                NextState = WaitFor7E;
             }
            break;
         }
             
    CurrentState = NextState;                           
    return ReturnEvent;
}
/**************************************
Getter functions for PacketSender to get variables
**************************************/


uint8_t* GetReceivePointer( void){
    return ReceiveArray;
}
    



/***************************************************************************
 private functions
 ***************************************************************************/
//Init UART module 1 (PB0 = Rx; PB1 = Tx)
static void initUART( void){
    //Enable the clock to UART and Port B
    HWREG(SYSCTL_RCGCUART) |= (SYSCTL_RCGCUART_R1);  // enable UART module 1
    while((HWREG(SYSCTL_PRUART) & SYSCTL_PRUART_R1) != SYSCTL_PRUART_R1); // wait for clock to be ready
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; // enable the clock to Port B
	while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1); // wait for the GPIO port to be ready
    //Set PB0 as digital input and PB1 as digital output
	HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT0HI | BIT1HI| BIT2HI | BIT3HI | BIT4HI); // program the port lines for digital I/O
    HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) &= BIT0LO; // Rx input (PB0)
	HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= BIT1HI; // Tx output (PB1)
    HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= BIT2HI; // PB2 output (debug)
    HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= BIT3HI; // PB3 output (debug)
    HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) &= BIT4LO; // Team Color Toggle input (PB4)

    //set pb2 and pb3 low for debug PB3 is speaker input
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~ (GPIO_PIN_2);
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= (GPIO_PIN_3); //set pb3 high for speaker 
    //Set the alternate functions for PB0 and PB1
    HWREG(GPIO_PORTB_BASE+GPIO_O_AFSEL) |= (BIT0HI | BIT1HI);
    HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) =
		(HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & 0xffffff00) + 1 + (1<<4);
    //Set up UART baud rate for 9600 baud
    HWREG(UART1_BASE+UART_O_CTL) &= ~(UART_CTL_UARTEN); //Disable the UART
    HWREG(UART1_BASE+UART_O_IBRD) = 260; //Set the integer baud rate divisor
    HWREG(UART1_BASE+UART_O_FBRD) = 27; //Set the Fractional Baud-Rate
    HWREG(UART1_BASE+UART_O_LCRH) |= UART_LCRH_WLEN_8; //Set WLEN to be 8 bits 
    HWREG(UART1_BASE+UART_O_CTL) |= (UART_CTL_RXE | UART_CTL_TXE); //Enable recieve, transmit 
    

    //Now enable interrupts for UART
    HWREG(UART1_BASE+UART_O_IM) |= (UART_IM_RXIM | UART_IM_TXIM); //Enable interrupts for transmit and receive
    // enable the UART1 interrupt in the NVIC-it is interrupt number 6 
	HWREG(NVIC_EN0) |= BIT6HI;
    //Make sure interrupt is cleared
    HWREG(UART1_BASE+UART_O_ICR) = 0xff;
	// make sure interrupts are enabled globally
	__enable_irq();
    
    //Finally enable UART
    HWREG(UART1_BASE+UART_O_CTL) |= UART_CTL_UARTEN; 
}

void UART_InterruptResponse( void){
    //Check where the interrupt came from
    //if we get a transmit interrupt
    if ((HWREG(UART1_BASE+UART_O_MIS)&UART_MIS_TXMIS) == UART_MIS_TXMIS){
        HWREG(UART1_BASE+UART_O_ICR) |= UART_ICR_TXIC; //Then clear the interrupt (set bit to clear) //try to fix this later maybe
        // wait for data register to empty
        while((HWREG(UART1_BASE+UART_O_FR)&UART_FR_TXFE) != UART_FR_TXFE);
    //Else if we get a Receive interrupt
    }else if ((HWREG(UART1_BASE+UART_O_MIS)&UART_MIS_RXMIS) == UART_MIS_RXMIS){
        HWREG(UART1_BASE+UART_O_ICR) |= UART_ICR_RXIC; //Then clear the interrupt (set bit to clear)
        data = (HWREG(UART1_BASE+UART_O_DR)&0xff); 
        
        //Post the data to Comm service with paramter of data
        ES_Event ThisEvent;
        ThisEvent.EventType = ES_ReceivedByte;
        ThisEvent.EventParam = data;
        PostCommService(ThisEvent);
        
    }else{
        printf("Interrupt cause unknown \r\n");
    }
}



/*******************************    Added by Alex   ************************************/

/* 
SendPacket: transmits a full packet to Xbee via UART

input parameters: uint8_t WhichData (which data to use)

example function call: 
    SendPacket(TESTDATA1);
*/
void SendPacket(uint8_t* DataPtr, bool SendNew){
    // build the packet
    if (SendNew){
        MakePacket(DataPtr);
        //printf("\r\n  New Packet Sent   \r\n");    
    }
    
    // send each byte of the packet
    for(int i=0; i<PacketLength; i++){
        // load data register
        HWREG(UART1_BASE+UART_O_DR) = PacketArray[i];
        // wait for data register to empty
        while((HWREG(UART1_BASE+UART_O_FR)&UART_FR_TXFE) != UART_FR_TXFE);
         
    }
    
    ////////See if you are actually resending the old packet
//    for(int i=0; i<PacketLength; i++){
//        printf("Packet that was just sent: %x \r\n",PacketArray[i]);   
//    }
   ////////////////////////////////////////////////////////
}



static void MakePacket(uint8_t* DataPtr){
    // pull in length of data to transmit
    //if we are sending a request to pair
    //printf("Packet Type: %x \r\n", *DataPtr);
    if (*DataPtr == 0x00){
        DataLength = 2;
    }
    //else if we a sending an encryption packet
    else if (*DataPtr == 0x01){
        DataLength = 33;
        EncryptionPointer = DataPtr; //Store the encryption pointer
        //Set the encryption index back to 1
        EncryptionCounter =1;
    }
     //else if we a sending a control packet
    else if (*DataPtr == 0x02){
        DataLength = 5;
        ControlFlag = true;
    }

    // calculate total packet length (data length + 9)
    PacketLength = DataLength + 9;
    /********************   build PacketArray   *****************/
    // start delimiter = 0x7E
    PacketArray[0] = 0x7e;
    // MSB of length = 0x00
    PacketArray[1] = 0x00;
    // LSB of length = DataLength+5
    PacketArray[2] = DataLength+5;
    // API Identifier = 0x01
    PacketArray[3] = 0x01;
    // Frame ID = 0x01 (arbitrary)
    PacketArray[4] = 0x00;
    // Destination Address = 0x2182 or 0x2082 for our Xbees
//    PacketArray[5] = 0x21; // 0x20 //make it 0xffff if we for request to pair packet
//    PacketArray[6] = 0x82;
    if (*DataPtr == 0x00){
        PacketArray[5] = 0xff;
        PacketArray[6] = 0xff;
    }else{
        PacketArray[5] = SourceAddressH;
        PacketArray[6] = SourceAddressL;
    }
    // Options byte = 0x00
    PacketArray[7] = 0x00;
    // RF Data
    for(int i=0; i<DataLength; i++){
        PacketArray[8+i] = *(DataPtr+i);
    }
    //Re-write the packet checksum if this is a control packet
    if (ControlFlag == true){
        PacketArray[12] = PacketArray[11]+PacketArray[10]+PacketArray[9]+PacketArray[8];
        //PacketArray[12] = 0xff - (PacketArray[11]+PacketArray[10]+PacketArray[9]+PacketArray[8]);
        //printf("Control Checksum: %x \r\n", PacketArray[12]);
        ControlFlag = false; 
        //Now add the encryption here:
        for(int i=0; i<DataLength; i++){
            PacketArray[8+i] = PacketArray[8+i] ^ *(EncryptionPointer+EncryptionCounter);
           // printf("Encr Counter: %d \r\n",EncryptionCounter);
            EncryptionCounter++;
            //printf("Encr Counter: %d \t\t, Data Length: %d \r\n",EncryptionCounter, DataLength);
            // printf("EncryptedArray: %x \r\n", PacketArray[8+i]);
            //Restart the encyption counter back to the second byte of the encypt array
            if (EncryptionCounter == 33){
                EncryptionCounter = 1;
            }
            //Put in a way to decrement the encryption counter in case they did not receive the last Packet
        }
        //Get the encrypted checksum
        EncryptedCheckSum = PacketArray[12];
        //printf("Unique ID: %x \r\n",EncryptedCheckSum);
    }
    
    
    // Checksum (XBEE)
    Checksum = 0;
    for(int i=3; i<8+DataLength; i++){
        Checksum += PacketArray[i];
    }
    Checksum = 0xff - Checksum;
    //printf("\r\n    Checksum = %d    \r\n ",Checksum);
    PacketArray[8+DataLength] = Checksum;
    /****************   end: build PacketArray   ****************/
}

uint8_t getEncryptedCheckSum( void){
    return EncryptedCheckSum;
}

void startSpeaker( void){
    //turn pb3 low 
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~ (GPIO_PIN_3);
}

void stopSpeaker( void){
    //turn pb3 high 
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |=  (GPIO_PIN_3);
}

void startPairedLight( void){
    //turn pb2 high
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= GPIO_PIN_2;
}
void stopPairedLight( void){
    //turn pb2 low
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~ (GPIO_PIN_2);
}


/*****************************   End   *********************************/
