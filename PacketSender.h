#ifndef PacketSender_H
#define PacketSender_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {Unpaired_NoPair,Unpaired_TryPair, WaitForEncryptionResponse,Paired_Ready2Send, Paired_WaitingForResponse} PacketSenderState_t ;


// Public Function Prototypes

bool InitPacketSender ( uint8_t Priority );
bool PostPacketSender( ES_Event ThisEvent );
ES_Event RunPacketSender( ES_Event ThisEvent );



uint16_t Pot_Read(void);//used by accel service
PacketSenderState_t QueryPacketSenderState ( void );//used by accel service to see if we should check for a hook shake



#endif /* PacketSender_H */


