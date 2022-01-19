//Global variables-------------------------------
//
//
//Can be used in all header and C files

#define BUF_LENGTH 300		//Number of characters in a frame
#define TMP_BUF_LENGTH 30
#define EnableInterrupts _EINT()

char rxdata;			//RS232 RX data byte
unsigned char buf[BUF_LENGTH];
unsigned char buf_tmp[TMP_BUF_LENGTH];
signed char RXTXstate;	//used for transmit recieve byte count
unsigned char flags;	//stores the mask value (used in anticollision)
unsigned char AFI;
unsigned char RXErrorFlag;
unsigned char RXflag;		//indicates that data is in buffer

unsigned char i_reg;	//interrupt register

unsigned char CollPoss;

unsigned char RXdone;
unsigned char POLLING;
unsigned char GUI;

//-----------------------------------------------
