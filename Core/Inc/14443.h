//------------------------------------------------------//
//This file contains functions for testing the		//
//14443-A and 14443-B protocol for TRF796x reader chip.	//
//							//
//Transmission and reception is done through the FIFO.	//
//------------------------------------------------------//
#include <stdio.h>
#include "parallel.h"
#include "anticollision.h"
#include "main.h"

extern unsigned char completeUID[14];

char SelectCommand(unsigned char select, unsigned char *UID);
void AnticollisionLoopA(unsigned char select, unsigned char NVB, unsigned char *UID);
void AnticollisionSequenceA(unsigned char REQA);
