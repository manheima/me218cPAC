

#ifndef CommService_H
#define CommService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum { WaitFor7E, WaitForMSB, WaitForLSB, 
               SuckUpPacket } CommServiceState_t ;


// Public Function Prototypes

bool InitCommService ( uint8_t Priority );
bool PostCommService( ES_Event ThisEvent );
ES_Event RunCommService( ES_Event ThisEvent );

//Testing Pointers//
uint8_t* getDataArrays( void);
void SendPacket(uint8_t* DataPtr, bool SendNew);
uint8_t* GetReceivePointer( void);
uint8_t getEncryptedCheckSum( void);
void startSpeaker( void);
void stopSpeaker( void);
void startPairedLight( void);
void stopPairedLight( void);





#endif /* CommService_H */

