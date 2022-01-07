//------------------------------------------------------//
//This file contains functions for testing the		//
//14443-A and 14443-B protocol for TRF796x reader chip.	//
//							//
//Transmission and reception is done through the FIFO.	//
//------------------------------------------------------//
#include <stdio.h>
#include "parallel.h"
#include "main.h"
/*
#include "SPI.h"
#include "anticollision.h"
#include "globals.h"
*/

extern unsigned char completeUID[14];

char SelectCommand(unsigned char select, unsigned char *UID);
void AnticollisionLoopA(unsigned char select, unsigned char NVB, unsigned char *UID);
void AnticollisionSequenceA(unsigned char REQA);
unsigned char Request14443A(unsigned char *pbuf, unsigned char lenght, unsigned char BitRate);
void SlotMarkerCommand(unsigned char number);
void AnticollisionSequenceB(unsigned char command, unsigned char slots);
