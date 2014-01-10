#include <Wire.h>
#include <SoftwareSerial.h>
#include "I2C.h"
#include "Kalman.h"

// Controle Servo
#include <Servo.h>
          
#define SWAP(x,y) swap = x; x = y; y = swap;

#define BluetoothSpeed 9600
#define SerialSpeed 9600

#define BLUE_FOR 13
#define BLUE_BACK 12
#define BLUE_LEFT 11
#define BLUE_RIGHT 10
#define GYRO_SENSOR 4

#define FORWARD 'f'
#define BACKWARD 'b'
#define LEFTWARD 'l'
#define RIGHTWARD 'r'
#define STOPWARD 's'
#define MOD_HEIGHT 'h'
#define TEMPERATURE_REQUEST 'T'

SoftwareSerial BTSerial(2,3);

Kalman KALX, KALY, KAYZ;

int16_t accX, accY, accZ;
int16_t tempRaw, temp;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle, accZangle;
double gyroXangle, gyroYangle, gyroZangle;
double compXangle, compYangle, compZangle;
double kalXangle, kalYangle, kalZangle;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
uint8_t c, swap;

// Servo Moter Set
Servo myMotor1;
Servo myMotor2;
Servo myMotor3;
Servo myMotor4;

int dM1 = 0, dM2 = 0, dM3 =81, dM4 = 76;
int M1 = 0, M2 = 0, M3 =0, M4 = 0;

boolean isStop = false;
double X_err[3] = {0,0,0}, X_err_sum = 0;	// Err, accumulation err
double X_angle_aim, X_angle;  			//aim angle, now state angle
double X_kp_ctr, X_ki_ctr, X_kd_ctr;  	//P out, I out, D out
double X_angle_a; 			//total X angle out
double X_kp = 0.22,X_ki = 0,X_kd = 0.0006;  	//p gain, i gain, d gain

void setup()
{ 
    
  Serial.begin(SerialSpeed);
  Wire.begin();
  BTSerial.begin(BluetoothSpeed);
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 
   while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
    }
    
    while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  KALX.setAngle(accXangle); // Set starting angle
  KALY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compXangle = accXangle;
  compYangle = accYangle;
  timer = micros();
  pinMode(BLUE_FOR, OUTPUT);
  pinMode(BLUE_BACK, OUTPUT);
  pinMode(BLUE_LEFT, OUTPUT);
  pinMode(BLUE_RIGHT, OUTPUT);
  pinMode(GYRO_SENSOR, INPUT);
  

  //Moter Setup---------------------------------------
  // the motor Arduino pin 
  myMotor1.attach(9);
  myMotor2.attach(10);
  myMotor3.attach(11);
  myMotor4.attach(12);
  // Print a startup message
  Serial.println("initializing");
   for(int i = 0; i<5; i++)
   {
     
   myMotor1.write(65);
   myMotor2.write(65);
   myMotor3.write(65);
   myMotor4.write(55);
   delay(1000);
   }
}


void loop()
{
    char a = (char)Serial.read();

      
    //Gyro  
      while(i2cRead(0x3B,i2cData,14));
      accX = ((i2cData[0] << 8) | i2cData[1]);
      accY = ((i2cData[2] << 8) | i2cData[3]);
      accZ = ((i2cData[4] << 8) | i2cData[5]);
      tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
      gyroX = ((i2cData[8] << 8) | i2cData[9]);
      gyroY = ((i2cData[10] << 8) | i2cData[11]);
      gyroZ = ((i2cData[12] << 8) | i2cData[13]);
      
      //Acculate
      accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
      accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
      
      //Angle (Not use, using kalangle)
      double gyroXrate = (double)gyroX/131.0;
      double gyroYrate = -((double)gyroY/131.0);
      gyroXangle += KALX.getRate()*((double)(micros()-timer)/1200000); // Calculate gyro angle using the unbiased rate
      gyroYangle += KALY.getRate()*((double)(micros()-timer)/1200000);
            
      //Angle
      kalXangle = KALX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
      kalYangle = KALY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
      timer = micros();
      //Tmeperture
      temp = ((double)tempRaw + 12412.0) / 340.0;
    
     // Moter controle  -------------------------------------------
 	//Serial.print(accXangle);Serial.print("\t");
  	//Serial.print(gyroXangle);Serial.print("\t");
  	//Serial.print(compXangle);Serial.print("\t");
  	//Serial.print("\t");
  	//Serial.print(accYangle);Serial.print("\t");
  	//Serial.print(gyroYangle);Serial.print("\t");
  	//Serial.print(compYangle); Serial.print("\t");
  	//Serial.print(temp);Serial.print("\t");
   
   // MOTER CONTROLE CORE
   
   X_angle_aim = 180;//map(180,0,360,0,2700);
   X_angle = kalXangle;//map(kalXangle,0,360,0,2700);
   
   X_err[2] = X_err[1];
   X_err[1] = X_err[0];
   X_err[0] = X_angle_aim - X_angle;
   
   X_kp_ctr = X_kp * X_err[0];  		// P ctr
   X_err_sum += X_err[0] / 1000;		// I sum
   X_ki_ctr = X_ki * X_err_sum;  		// I ctr
   X_kd_ctr = X_kd * (X_err[0]-X_err[1]) * 1000; // D ctr
   X_angle_a = X_kp_ctr + X_ki_ctr + X_kd_ctr;	// Total speed
   
   // set moter speed
   
   	
     M1 = dM1 + X_angle_a;
     M2 = dM2 - X_angle_a;
     M3 = dM3 - X_angle_a;
     M4 = dM4 + X_angle_a;    
   
   /*
    switch(a)
    {
       case 'p':
         Serial.println("Zero is right y;ou fucnking asshole");
         M1 = 40; M2 = 40; M3 = 40;M4 = 40;
         
         isStop != isStop;
         return;
         
       break;
       
    }
    
    */
    if(X_angle_a < 1500){
     myMotor1.write(M1);
     myMotor2.write(M2);
     myMotor3.write(M3);
     myMotor4.write(M4);
  }
      
     Serial.print(X_err[0]);Serial.print("\t");
     Serial.print(X_kp_ctr);Serial.print("\t");
     Serial.print(X_ki_ctr);Serial.print("\t");
     Serial.print(X_kd_ctr);Serial.print("\t");
     Serial.print(X_angle_a);Serial.print("\t\t");
     Serial.print(kalXangle);Serial.print("\t");
     Serial.print(kalYangle);Serial.print("\t\t");
   
     Serial.print(M1);Serial.print("\t");
     Serial.print(M2);Serial.print("\t");
     Serial.print(M3);Serial.print("\t");
     Serial.print(M4);Serial.print("\t");
     
     Serial.print(temp);
     Serial.print("\r\n");
}








void int2byte(int num, char* arr)
{
    for(int i = 0; i < 4; i++)
      arr[3 - i] = num >> (i * 8);
}

int byte2int(char *arr)
{
   int num = 0;
   for(int i = 0; i < 4; i++)
       num += (arr[3 - i] & 0xFF) << (8 * i); 
     
     return num;
}


uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress,&data,1,sendStop);
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

