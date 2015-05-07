#include "stdint.h"

#define CMD_LINE_LENGTH 20

extern uint16_t non_alpha_caught;

class Terminal
{
	public:
	char cmdline[8][20];
	uint8_t cmdline_ptr;
	char * cmd_entered_ptr;
	uint8_t cmdline_lptr;
	
	Terminal();
	void splash( void );
	void init( void );
	uint8_t getLine();

	
};
