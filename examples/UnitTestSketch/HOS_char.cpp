// This is free and unencumbered software released into the public domain.

// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.

// For more information, please refer to <http://unlicense.org/>

#include "HOS_char.h"

// This takes a char input and converts to int
//
// The output will be an int if the char
//  is a number
//  is A-F
//  is a-f
//
// Otherwise, the output is zero.
//
int char2hex(char charin)
{
  int hexout;
  if(charin >= 0x30)
  {
    if(charin <= 0x39)
    {
      hexout = charin - 0x30;
    }
  }

  if(charin >= 0x61)
  {
    if(charin <= 0x66)
    {
      hexout = charin - 0x61 + 10;
    }
  }
  if(charin >= 0x41)
  {
    if(charin <= 0x46)
    {
      hexout = charin - 0x41 + 10;
    }
  }
  return hexout;
}

// This takes a int input and converts to char
//
// The output will be an char if the int
//  is between zero and 0xF
//
// Otherwise, the output is $.
//
char hex2char(int hexin)
{
  int charout;
  charout = 0x24; // default $ state
  if(hexin >= 0x00)
  {
    if(hexin <= 0x09)
    {
      charout = hexin + 0x30;
    }
  }
  if(hexin >= 0x0A)
  {
    if(hexin <= 0x0F)
    {
      charout = hexin -10 + 0x41;
    }
  }  
  return charout;
}

//Returns 1 if the input char is in the hex range
int ishex(char charin)
{
  int answer;
  answer = 0;
  if(charin >= 0x30)
  {
    if(charin <= 0x39)
    {
      answer = 1;
    }
  }

  if(charin >= 0x61)
  {
    if(charin <= 0x66)
    {
      answer = 1;
    }
  }
  if(charin >= 0x41)
  {
    if(charin <= 0x46)
    {
      answer = 1;
    }
  }
  return answer;
}

//Returns a number of correct chars out of the goal string input, at input position
//See documentation
int stringcompare(char sights[], char goal[], int mStart, int mLength)
{
  int comp_ptr = 0;
  char return_value = 0;

  if( mLength ) //if the mLength is a number
  {
	  return_value = 1;  //set to zero if one case of difference is found
	  for(comp_ptr = mStart; comp_ptr < (mStart + mLength); comp_ptr++)
	  {
		  if(sights[comp_ptr] != goal[comp_ptr - mStart])
		  {
			  return_value = 0;
		  }
	  }
  }
  else if( !mStart && !mLength ) //if neither are numbers
  {
	  return_value = 1;  //set to zero if one case of difference is found
	  for(comp_ptr = mStart; goal[comp_ptr + 1] != 0x00; comp_ptr++) //seeks for the next char to be null
	  {
		  if( sights[comp_ptr] != goal[comp_ptr] )  //if not same, flag zero.
		  {
			  return_value = 0;
		  }
	  }
	  //At this point the return_value is a zero if all chars matched and the next that would be searched for is null.
	  //If so, and the next char is not a space or a null, allow the 1 to output
	  if(return_value)
	  {
		  comp_ptr++;
		  if( (sights[comp_ptr] != ' ')&&(sights[comp_ptr] != 0x00) ) //if not a space or null...
		  {
			  return_value = 0;  // ...return failure.
		  }
	  }
  }

  return return_value;
}

//Looks for a basic number entry in a string, reporting as int.
//
//  (int mStart) should be a pointer to the first number.
//
//Will stop looking on discovery of a line control char, null, space, or after 6 chars.
int search_int(char sights[], int mStart)
{
  int i;
  int number = 0;
  int last_char;
  int place = 1;

  for(i = 0; i <= 5; i++)
  {
    if((sights[mStart + i] == 0x20)||(sights[mStart + i] == 0x0A)||(sights[mStart + i] == 0x0D))
    {
      last_char = mStart + i - 1;  //point to last number
      i = 6;  //Bust on outta here!
    }
  }

  //now work backwards building the number
  for(i = last_char; i >= mStart; i--)
  {
	  number += char2hex(sights[i]) * place;
	  place = place * 10;
  }
  return number;
}

//***************************************************************************//
//   This function takes an signed long up to 32 bits and turns it into a
//   null-terminated string.  It takes an argument of the mStarting address to
//   the output string, as well as the int.
//   The string should be 11 bytes long, 1 (-) sign and 10 integers.
//   The function overwrites should truncate large numbers to 10 places.
//
//   "-2147483648\0"
//
//***************************************************************************//

void long2ascii( signed long input_long, char * target_string )
{
	//Vars
	long decumulator;
	//int dec_fact = 10;
	char out_buffer[10];
	char target_string_ptr = 0;
	int out_buffer_ptr;

	//take care of the pesky negative.
	if( input_long < 0 )
	{
		target_string[target_string_ptr] = '-';
		target_string_ptr++;
		decumulator = input_long ^ 0xFFFFFFFF;  //convert to positive
		decumulator++;
	}
	else
	{
		decumulator = input_long;  //Leave as positive
	}
	//Perform the modulus math
	for( out_buffer_ptr = 0; out_buffer_ptr < 10; out_buffer_ptr++ )
	{
		//perform the mod
		//buffer the base 10 value
		int temp =  ( decumulator % 10 );
		out_buffer[out_buffer_ptr] = temp;
		//decrement the decumulator-- get rid of the last remainder and divide by 10
		decumulator = (decumulator - out_buffer[out_buffer_ptr]) / 10;
		//multiply by 10
		//dec_fact = dec_fact * 10;
	}

	//Now convert the base 10 int array to the string
	//First seek first non-zero place
	for( out_buffer_ptr = 9; (out_buffer[out_buffer_ptr] == 0) && (out_buffer_ptr > 0) ; out_buffer_ptr-- );  //oh, look at that tricky shit!

	//roll out the out_buffer upwards till pointer = 0
	for( ; out_buffer_ptr >= 0; out_buffer_ptr-- ) //What is this?  Is it even valid?
	{
		target_string[target_string_ptr] = hex2char(out_buffer[out_buffer_ptr]);
		target_string_ptr++;
	}
	//slap a null on it
	target_string[target_string_ptr] = 0x00;

	return;
}
