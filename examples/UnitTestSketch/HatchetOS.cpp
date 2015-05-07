#include "HatchetOS.h"
#include "Arduino.h"

extern int led;

Terminal::Terminal( void )
{
}

void Terminal::splash( void )
{
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n--------------------------------------------------------------------------------\n");
  Serial.print("\nHatchet OS Shell\n");
  Serial.print("Carefully bludgeoned together by Marshall Taylor\n");
  Serial.print("\n");
  Serial.print("alpha - 2012\n");
  Serial.print("beta - July, 2014\n");
  Serial.print("0.1.0 - MSP430 port.  August, 2014\n");
  Serial.print("0.2.0 - Arduino port.  May 6th, 2015\n");
  Serial.print("\n--------------------------------------------------------------------------------\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");

}

void Terminal::init( void )
{
  //Call HOS boot routines here
  Serial.print("Writing test data to screen...");
  Serial.print("Done!\n");

  Serial.print("Clearing the command line buffer...");
  //Clears the command line 2D array
  for (cmdline_ptr = 0; cmdline_ptr < CMD_LINE_LENGTH; cmdline_ptr++)
  {
    for (cmdline_lptr = 0; cmdline_lptr < 8; cmdline_lptr++)
    {
      cmdline[cmdline_lptr][cmdline_ptr] = 0x00;
    }
  }
  cmdline_ptr = 0;
  cmdline_lptr = 0;
  Serial.print("Done!\n");
  Serial.print("LED test...");
  for ( int i = 0; i < 7; i++)
  {
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(80);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(80);               // wait for a second
  }
  Serial.print("Done!\n");
  Serial.print("\n");
  Serial.print("Boot successful!\n\n");
  Serial.print("Use 'help' for more information\n");
  Serial.print("\n");

}

uint8_t Terminal::getLine( void )
{
  cmdline_ptr = 0; // reset to left (enter has been pressed)

  //Give a prompt
  Serial.print(">");
  char lastchar = 0x00;
  //Clear the start of the current line
  cmdline[cmdline_lptr][cmdline_ptr] = 0x00;

  uint8_t parse_now = 0;  //reset.  Set 1 to leave the loop

  while ( !parse_now ) // This loop is a bit redundant...
  {
    //Wait for input_iterator
    while ( Serial.available() == 0 );
    lastchar = Serial.read();

    //loop until internally decided to leave
    switch ( lastchar ) {
      case 0x1B: //escape
        lastchar = 0x00;
        while ( Serial.available() == 0 ); //wait for next key
        //Save the keystroke
        lastchar = Serial.read();
        if ( lastchar == '[' ) //OK!  We got a cursor control indicator
        {
          lastchar = 0x00;
          while ( Serial.available() == 0 ); //wait for next key
          //Save the keystroke
          lastchar = Serial.read();

          switch ( lastchar ) {
            case 'A': //Up
              Serial.print("\x1B[2K"); // clear line
              Serial.print("\r"); // cr
              Serial.print(">"); // Give a prompt
              //decrement cmdline_lptr
              cmdline_lptr--;
              cmdline_lptr &= 0x07;
              //print out that line
              cmdline_ptr = Serial.print((char *)((int)&cmdline + (CMD_LINE_LENGTH * cmdline_lptr)));  //writes line, also reports char position
              break;
            case 'B': //Down
              Serial.print("\x1B[2K"); // clear line
              Serial.print("\r"); // cr
              Serial.print(">"); // Give a prompt
              //increment the cmdline_lptr
              cmdline_lptr++;
              cmdline_lptr &= 0x07;
              //print out that line
              cmdline_ptr = Serial.print((char *)((int)&cmdline + (CMD_LINE_LENGTH * cmdline_lptr)));  //writes line, also reports char position
              break;
            case 'C': //Right
              if ( cmdline_ptr < CMD_LINE_LENGTH ) //Move pointer in command line
              {
                cmdline_ptr++;
                Serial.print("\x1B[C"); // go right
              }
              break;
            case 'D': //Left
              if ( cmdline_ptr > 0 ) //Move pointer in command line
              {
                cmdline_ptr--;
                Serial.print("\x1B[D"); // go left
              }
              break;
            default:
              break;
          }
        }
        lastchar = 0x00;
        break;
      case 0x08: //backspacearoo
        lastchar = 0x00; //Clear the char
        if ( cmdline_ptr > 0 ) //Move pointer in command line
        {
          cmdline_ptr--;
          Serial.print("\x1B[D"); // Go left
          cmdline[cmdline_lptr][cmdline_ptr] = 0x00; //clear current char in the buffer
          Serial.print(" "); //clear current char on the screen
          Serial.print("\x1B[D"); // Go left
        }
        break;
      case 0x00:
        break;
      case 0x0D:
        parse_now = 1;
        break;
      case 0x0A:
        parse_now = 1;
        break;
      default:
        if (( lastchar > 0x1F ) && ( lastchar < 0x7F )) // only process valid characters into the command line
        {
          Serial.print(lastchar);
          cmdline[cmdline_lptr][cmdline_ptr] = lastchar;
          if ( cmdline_ptr < CMD_LINE_LENGTH )
          {
            cmdline_ptr++;
          }
          cmdline[cmdline_lptr][cmdline_ptr] = 0x00;
        }
        else
        {
          non_alpha_caught++;
        }
        lastchar = 0x00;
        break;;
    }

  }
  //Clear out the serial buffer
  while ( Serial.available() )
  {
    Serial.read();
  }

}

