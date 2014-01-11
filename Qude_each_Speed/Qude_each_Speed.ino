/*
*  This code is in the public domain.
*  (Do whatever you want with it.)
*/

// Need the Servo library 
#include <Servo.h>

// This is our motor.
Servo myMotor1;
Servo myMotor2;
Servo myMotor3;
Servo myMotor4;


// This is the final output
// written to the motor.
String incomingString;

// input MoterNumber 1~4
int moterNum;

// Set everything up
void setup()
{
  // the motor Arduino pin 
  myMotor1.attach(9);
  myMotor2.attach(10);
  myMotor3.attach(11);
  myMotor4.attach(12);
  // Required for I/O from Serial monitor
  Serial.begin(9600);
  // Print a startup message
  Serial.println("initializing");
   myMotor1.write(30);
   myMotor2.write(30);
   myMotor3.write(30);
   myMotor4.write(30);
   
   delay(5000);
   
  // myMotor1.write(81);
  // myMotor2.write(88);
  //myMotor3.write(86);
  //myMotor4.write(81);

  //81 76
  
}


void loop()
{
  
  // If there is incoming value
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
  
    /*
    *  If ch isn't a newline
    *  (linefeed) character,
    *  we will add the character
    *  to the incomingString
    */
    if (ch != 10){

      // Add the character to
      // the incomingString
      incomingString += ch;
    }
    // received a newline (linefeed) character
    // this means we are done making a string
    else
    {  
      // Convert the string to an integer
      int val = incomingString.toInt();
      //int val = 0;
      // print the integer
      
      switch(val)
      {
       case 1 :
       case 2 :
       case 3 :
       case 4 :
       case 12 :
       case 13 :
       case 14 :
       case 23 :
       case 24 :
       case 34 :
       case 1234 :
           Serial.print("Moter : "); Serial.println(val);
           moterNum = val;
           break;
           
       default :
         if (val > 40 && val < 181 ) {
            Serial.print("Speed : "); Serial.println(val);
            
            switch (moterNum){
             case 1 :
                myMotor1.write(val);
                break;
             case 2 :
                myMotor2.write(val);
                break;
             case 3 :
                myMotor3.write(val);
                break;
             case 4 :
                myMotor4.write(val);
                break;
             case 12 :
                 myMotor1.write(val);
                 myMotor2.write(val);
                 break;
             case 13 :
                myMotor1.write(val);
                myMotor3.write(val);
                break;
             case 23 :
                myMotor2.write(val);
                myMotor3.write(val);
                break;
             case 34 :
                myMotor3.write(val);
                myMotor4.write(val);
                break;
             case 24 :
                myMotor2.write(val);
                myMotor4.write(val);
                break;
             case 14 :
                myMotor1.write(val);
                myMotor4.write(val);
                break;
             
                
             case 1234 :
                myMotor1.write(val);
                myMotor2.write(val);
                myMotor3.write(val);
                myMotor4.write(val);
            }
          }
          
          else
         {
           Serial.println("\nEmergency stop\n");
           
           myMotor1.write(30);
           myMotor2.write(30);
           myMotor3.write(30);
           myMotor4.write(30);
         }
         
         
           
      }
      
    
     
     //Emergency Stop     
     
    
      // Reset the value of the incomingString
      incomingString = "";
    }
  }
}
