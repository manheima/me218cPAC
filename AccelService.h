

#ifndef AccelService_H
#define AccelService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {WaitForMeasure, MeasureAccel, WaitForPositiveShake, WaitForNegativeShake, WaitForSecondPositiveShake} AccelServiceState_t ;


// Public Function Prototypes

bool InitAccelService ( uint8_t Priority );
bool PostAccelService( ES_Event ThisEvent );
ES_Event RunAccelService( ES_Event ThisEvent );

// Get Packet Array Pointers
uint8_t* GetRequestPacket( void);
uint8_t* GetControlPacket( void);



#endif /* AccelService_H */

