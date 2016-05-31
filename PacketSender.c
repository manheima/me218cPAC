/****************************************************************************
 Module
   PacketSender.c

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
#include "CommService.h"
#include "ADMulti.h" //The header for analog input
#include "PacketSender.h"
/*----------------------------- Module Defines ----------------------------*/
#define UPDATE_TIME 200
#define TRANSMIT_TIME 3000
#define OLD false
#define NEW true
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void ADC_Init(void);
static uint16_t ADC_Read( void);
static void GetEncryption( void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
//static AccelServiceState_t CurrentState;
static uint8_t MyPriority;
static PacketSenderState_t CurrentState;
static uint32_t results[2];
static uint8_t  Encrypt_Array[33];
static uint8_t Counter = 0;
static uint8_t* RequestPointer;
static uint8_t* ControlPointer;
static uint8_t* EncryptionPointer;
static uint8_t* ReceivePointer;
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
bool InitPacketSender ( uint8_t Priority )
{
    MyPriority = Priority;
    ADC_Init(); //may want to put this in main for hardware inits
    RequestPointer = GetRequestPacket();
    ControlPointer = GetControlPacket();
    Encrypt_Array[0] = 0x01;
    EncryptionPointer = Encrypt_Array; //set the pointer

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
bool PostPacketSender( ES_Event ThisEvent )
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
ES_Event RunPacketSender( ES_Event ThisEvent )
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    PacketSenderState_t NextState = CurrentState;
    
   
    
    switch ( CurrentState )
    {
        case Unpaired_NoPair: 
            stopSpeaker();
            if ( ThisEvent.EventType == ES_HookShake){  
                stopPairedLight();
                NextState = Unpaired_TryPair;
                Counter = 0; //set the counter to 0 for attempts to pair 
                printf("\r\n Sent Request Packet \r\n");
                SendPacket(RequestPointer,NEW); //try to pair
                ES_Timer_InitTimer(UpdateTimer,UPDATE_TIME); 
            }
        break;
        case Unpaired_TryPair:  
            if (ThisEvent.EventType == ES_PairBitSet){
                //play the speaker!
                startSpeaker();
                //turn on the light!
                startPairedLight();
                NextState = WaitForEncryptionResponse;
                GetEncryption(); //Makes a new encryption packet with pointer of EncryptionPointer
                SendPacket(EncryptionPointer,NEW);
                printf("\r\nSentEncryption Packet\r\n");
                //ES_Timer_InitTimer(UpdateTimer,UPDATE_TIME ); dont want this since we go to wait for encrypt response
                ES_Timer_InitTimer(TransmitTimer,TRANSMIT_TIME);
            }else if ( ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == UpdateTimer){  
                //Dont change states unless you already tried pairing 10 times
                if (Counter>10){
                    NextState = Unpaired_NoPair;
                }else{ //otherwise try pairing again
                    Counter++;
                    ES_Timer_InitTimer(UpdateTimer,UPDATE_TIME);
                    SendPacket(RequestPointer,NEW);
                }
            }
        break;
        case WaitForEncryptionResponse:
                if (ThisEvent.EventType == ES_PairBitSet){
                    NextState = Paired_Ready2Send;
                    ES_Timer_InitTimer(UpdateTimer,UPDATE_TIME );
                    ES_Timer_InitTimer(TransmitTimer,TRANSMIT_TIME);
                    printf("Got Encryption Response!");
                }else if (ThisEvent.EventType == ES_PairBitClear ){
                    NextState = Unpaired_NoPair;
                    //startSpeaker();
                    stopPairedLight(); 
                }
        
        break;
             
        case Paired_Ready2Send:  
            //if we are still paired with the Lobbyist
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == UpdateTimer){
                printf("\r\n Sent New Control Packet \r\n");
                stopSpeaker();
                SendPacket(ControlPointer,NEW);
                ES_Timer_InitTimer(UpdateTimer,200 ); //100 ms timer
                NextState = Paired_WaitingForResponse;
                ES_Timer_InitTimer(TransmitTimer,TRANSMIT_TIME); //1 sec timer           
            }else if (ThisEvent.EventType == ES_PairBitClear ){
                NextState = Unpaired_NoPair;
                //startSpeaker();
                stopPairedLight();
                if (ThisEvent.EventType == ES_PairBitClear){
                    printf("\r\n Pair Bit Clear! \r\n");
                }
            }
        break;
        case Paired_WaitingForResponse:
            //if we are get a timeout from 100ms timer
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == UpdateTimer){
                //Resend the old control packet
                printf("\r\n Sent OLD Control Packet \r\n");
                SendPacket(ControlPointer,OLD);
                //printf("\r\n DoingNothing but waiting for response .................. \r\n");
                ES_Timer_InitTimer(UpdateTimer,200 ); //restart the 100 ms timer to see if you need to send again
            }else if (ThisEvent.EventType == ES_PairBitClear || (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == TransmitTimer)){
                NextState = Unpaired_NoPair;
                //startSpeaker();
                stopPairedLight();
                if (ThisEvent.EventType == ES_PairBitClear){
                    printf("\r\n Pair Bit Clear! \r\n");
                }else if (ThisEvent.EventType == ES_TIMEOUT){
                    printf("Timeout! Pac took too long to respond \r\n");
                }
            }else if (ThisEvent.EventType == ES_StatusReceived){
                //Check the checksum byte!!!!!!!!!!
                ReceivePointer = GetReceivePointer();
                if (*(ReceivePointer+7) != getEncryptedCheckSum()){
                    printf("Unique ID is different than last sent! %x", getEncryptedCheckSum());
                    ES_Timer_InitTimer(TransmitTimer,TRANSMIT_TIME); //restart 1 sec timer           
                }else if (*(ReceivePointer+7) == getEncryptedCheckSum()){
                    NextState = Paired_Ready2Send;
                    ES_Timer_InitTimer(UpdateTimer,UPDATE_TIME);
                }

                ES_Timer_InitTimer(TransmitTimer,TRANSMIT_TIME); //1 sec timer   
            }
        break;
    
    
    }
    
    
    
    CurrentState = NextState;     
    return ReturnEvent;
}
/***************************************************************************
 Public  functions
 ***************************************************************************/
PacketSenderState_t QueryPacketSenderState ( void )
{
   return(CurrentState);
}
/***************************************************************************
 private functions
 ***************************************************************************/
// Funtion to initialize ADC
static void ADC_Init(void){
	
ADC_MultiInit(2);
	
}

// Function to get random number (PE0)
static uint16_t ADC_Read( void){
	ADC_MultiRead(results);
	return results[0];
}
//Function to read pot value (PE1)
uint16_t Pot_Read(void){
    ADC_MultiRead(results);
	return results[1];
}


static void GetEncryption( void){
    for (int i=0; i<32; i++){
        Encrypt_Array[i+1] = (ADC_Read()&0xff);
        //printf("Num: %d\tEncryption: %x \r\n", i,Encrypt_Array[i+1]);
    }
    
}


/*****************************   End   *********************************/
