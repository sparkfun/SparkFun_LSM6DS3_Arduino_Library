//HatchetOS

#include "HatchetOS.h"
#include "HOS_char.h"
#include "UnitTest.h"
#include "SparkFunIMU.h"
#include "Wire.h"
#include "SPI.h"

int led = 13;
Terminal tty;
uint16_t non_alpha_caught = 0;

//Unit test
DeviceUnderTest DUT;

void setup()
{
  pinMode(led, OUTPUT);
  // initialize serial:
  Serial.begin(57600);

  // initialize tty
  tty.splash();
  tty.init();

}

void loop()
{
  tty.getLine();

  //Give a return
  Serial.write(0x0A);
  Serial.write(0x0D);

//  //*******************************************************************//
//  //Use this section to output full char array
//
//  char mOutput = 0;
//  char* tempPtr = (char*)tty.cmdline;
//  for (int i = 0; i < 160; i++)
//  {
//    mOutput = tempPtr[i];
//    if ( mOutput == 0x00 )
//    {
//      Serial.print("*");
//    }
//    else
//    {
//      Serial.print(mOutput);
//    }
//  }
//  Serial.print("\n");
//  Serial.println(tty.cmdline_lptr);
//  //*******************************************************************//


  //Reference the crazy 2D array into a single char *

  tty.cmd_entered_ptr = (char *)((int)&tty.cmdline + (CMD_LINE_LENGTH * tty.cmdline_lptr));


  if (*tty.cmd_entered_ptr != 0x00)
  {
    tty.cmdline_lptr++; //Now that the line to operate on is buffered int cmd_char_ptr we can increment the 2D line pointer
    tty.cmdline_lptr &= 0x07;  //Roll at 8
  }



  //***Line to operate on now called cmd_entered_ptr, use it as an array***//



  //Start parcin'
  tty.cmdline_ptr = 0;  //Is this really necessary?
  char dec_string[11];  //used to convert ints to ascii

  //Example parcer usage is the "help" command.
  if (stringcompare(tty.cmd_entered_ptr, "help", 0, 0))
  {
    Serial.write("\n");
    Serial.write("\nCommands:");
    Serial.write("\n  help          shows this menu");
    Serial.write("\n  info          displays debug info");
    Serial.write("\n  blink         The ard. hello world sketch");
    Serial.write("\n  imu begin     Start the driver");
    Serial.write("\n  imu readall   Read all parameters");
    Serial.write("\n  imu peak      Hold the peak value");

    Serial.write("\n\n");
  }


  if (stringcompare(tty.cmd_entered_ptr, "info", 0, 0))
  {
    //Report debug info
    Serial.print("Non-alpha characters caught: ");
    Serial.print(non_alpha_caught);
    Serial.print("\n");

  }

  if (stringcompare(tty.cmd_entered_ptr, "imu", 0, 0))
  {
    //Now check for args
    if (stringcompare(tty.cmd_entered_ptr, "begin", 4, 5))
    {
      DUT.begin();
    }
    if (stringcompare(tty.cmd_entered_ptr, "readall", 4, 7))
    {
      DUT.readAll();
    }
    if (stringcompare(tty.cmd_entered_ptr, "peak", 4, 4))
    {
      DUT.readAllPeak();
    }
  }

  if (stringcompare(tty.cmd_entered_ptr, "blink", 0, 0))
  {
    while (1)
    {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);               // wait for a second
    }
  }
}



