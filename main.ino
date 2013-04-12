//#include <AHRS.h>
#include <Wire.h> //The I2C library
#include <Servo.h>
#include <math.h>
#include <HMC5883L.h>  //Compass Library

#define BMA180              0x40
#define ee_w_MASK           0x10
#define mode_config_MASK    0x03
#define bw_MASK             0xF0
#define range_MASK          0x0E
#define lat_int_MASK        0x01
#define lat_int             0x01

//////////////////////////////////////////////////////Variabel Declaration////////////////////////////////////////////////////////////////
// Store our compass as a variable.
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;
int gyroResult[3], accelResult[3];
int magX, magY, magZ;

int AccelX,AccelY,AccelZ,temp;

float timeStep = 0.02;          //20ms. Need a time step value for integration of gyro angle from angle/sec
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro;// = 0;
float pitchAccel;// = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro;// = 0;
float rollAccel;// = 0;
float rollPrediction = 0;  //Output of Kalman filter
float yawGyro;// = 0;
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
float headingDegrees;
float realheading;

unsigned long timer;
float pitch,roll,yaw;
float pitchAccelRad;
float rollAccelRad;
float Xmag;
float Ymag;
float Zmag;
float pitchGyrorad;
float rollGyrorad;
float yawGyrorad;

  
  
//////////////////////////////////////////////////////////////////Function Declaration on Setup Section//////////////////////////////////////////////////////////////
//writTo Function
void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

//readFrom Function
void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

//Gyro Init
void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1]; //Combine two bytes into one int
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

//Accelerometer Init
byte initializeBMA180()
{
  /*Set EEPROM image to write mode so we can change configuration*/
  delay(20);
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte ee_w = Wire.read();
  ee_w |= ee_w_MASK;
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  Wire.write(ee_w);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set mode configuration register to Mode 00*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte mode_config = Wire.read();
  mode_config &= ~(mode_config_MASK);
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  Wire.write(mode_config);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set bandwidth to 10Hz*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte bw = Wire.read();
  bw &= ~(bw_MASK);
  bw |= 0x00 << 4;
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  Wire.write(bw);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set acceleration range to 2g*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte range = Wire.read();
  range &= ~(range_MASK);
  range |= 0x00 << 1 ;
  /*    case B000: // 1g
        case B001: // 1.5g
        case B010/0x02: // 2g
        case B011: // 3g
        case B100: // 4g
        case B101: // 8g
        case B110: // 16g
        */
  
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  Wire.write(range);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set interrupt latch state to non latching*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte latch_int = Wire.read();
  latch_int &= ~(0x01);
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(latch_int);
  if(Wire.endTransmission()){return(1);}
  delay(20); 
  /*Set interrupt type to new data*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte int_type = Wire.read();
  int_type |= 0x02;
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(int_type);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  return(0);
}

void MagnetometerInit() {
   //*************************COMPASS Init****************************
  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.

  Serial.println("Constructing new HMC5883L");
  compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(8.1); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

void GyroSetting() {
   //Accelerometer and Gyro Setting
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
}

void AccelerometerGyroBias() {
  int i;
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  
  // Determine zero bias for all axes of both sensors by averaging 50 measurements
  delay(100); //wait for gyro to "spin" up
  for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);

    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += AccelX;
    totalAccelYValues += AccelY;
    totalAccelZValues += AccelZ;
    delay(50);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50); //Don't compensate gravity away! We would all (float)!
}




//########################################################################--Main Program--#############################################################
void setup() {
 
  Wire.begin();            //Open I2C communications as master
  Serial.begin(115200);    //Open serial communications to the PC to see what's happening

  MagnetometerInit();
  GyroSetting();
  initializeBMA180();
  AccelerometerGyroBias();
  
}

void loop() {
  
  timer = millis(); //get a start value to determine the time the loop takes
  
  readCompass();
  getGyroscopeReadings(gyroResult);
  readAccel();
  
  calculateSection();

  KalmanFilter();
  tiltCompCompass();
   
 
 
  //Print data Ouput (Any variable that i want print)
  
  //Serial.print(pitchGyro);
  //Serial.print("\t");
  //Serial.print(pitchAccel);
  //Serial.print("Pitch \t");
  //Serial.print(pitchPrediction);
  //Serial.print("pitch \t"); 
  //Serial.print(rollGyro);
  //Serial.print("\t");
  //Serial.print(rollAccel);
  //Serial.print("Roll \t");
  //Serial.print(rollPrediction);
  //Serial.print("roll \t");
  
  //Serial.print(magX);
  //Serial.print("magX\t");
  //Serial.print(magY);
  //Serial.print("magY\t");
  //Serial.print(magZ);
  //Serial.print("magZ \t"); 
     
   
   Serial.print(headingDegrees);
   Serial.print(" degree   \t");
   
   Serial.print(realheading);
   Serial.print(" real   \t");
   
   Serial.print("\n");
   
  //delay(100);
  timer = millis() - timer;          //how long did the loop take?
  timer = (timeStep * 1000) - timer; //how much time to add to the loop to make it last time step msec
  delay(timer);    //make one loop last time step msec                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
}

//###################################################--Main Program End--##################################################################






//**********************************************Function Declaration On Loop Section******************************************//

void readCompass() {
//*********************************Read Compass Begin*****************************************
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  magX=raw.XAxis;
  magY=raw.YAxis;
  magZ=raw.ZAxis;
  
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(magY, magX);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.55;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI; 
 
  // Output the data via the serial port.
  //Output(raw, scaled, heading, headingDegrees);

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);
  
  //*************************************************Read Compas End*********************************************
}

byte readAccel()
{
  Wire.beginTransmission(BMA180);
  Wire.write(0x02);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,7) != 7){return(2);}
  AccelX = Wire.read();
  AccelX |= Wire.read() << 8;
  AccelX >>= 2;
  //x = map(x, -4096, 4096, 0, 179);
  //pitch.write(x);
  
  AccelY = Wire.read();
  AccelY |= Wire.read() << 8;
  AccelY >>= 2;
  //y = map(y, -4096, 4096, 0, 179);
  //roll.write(y);
  
  AccelZ = Wire.read();
  AccelZ |= Wire.read() << 8;
  AccelZ >>= 2;
  temp = Wire.read();
}

void calculateSection() {
  pitchAccel = atan2((AccelY - biasAccelY) / 1024, (AccelZ - biasAccelZ) / 1024) * 360.0 / (2*PI); //calculate Accelerometer Pitch Degree
  pitchAccelRad = atan2((AccelY - biasAccelY) / 1024, (AccelZ - biasAccelZ) / 1024); //calculate Accelerometer Pitch Degree
  //GAccelX = (x - biasAccelX) / 8192;
  
  pitchGyro = pitchGyro + ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; //calculate Gyroscope Pitch Degree
  pitchGyrorad = (pitchGyro / 180) *PI;
  
  pitchPrediction = pitchPrediction + ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; //calculate Kalman Filter Pitch Degree
  
  rollAccel = atan2((AccelX - biasAccelX) / 1024, (AccelZ - biasAccelZ) / 1024) * 360.0 / (2*PI); //calculate Accelerometer Roll Degree
  rollAccelRad = atan2((AccelX - biasAccelX) / 1024, (AccelZ - biasAccelZ) / 1024);
  //GAccelY = (accelResult[1] - biasAccelY) / 1024;
  
  rollGyro = rollGyro - ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;  //calculate Ggyroscope Roll Degree
  rollGyrorad =(rollGyro / 180) *PI;;
  
  rollPrediction = rollPrediction - ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;  //calculate Kalman Filter Roll Degree
  
  yawGyro = yawGyro - ((gyroResult[2] - biasGyroZ) / 14.375) * timeStep; //calculate Gyroscope Yaw Degree
  yawGyrorad = (yawGyro / 180) *PI; //calculate Gyroscope Yaw Degree
  //GAccelZ = (accelResult[2] - biasAccelZ) / 1024;
}

void KalmanFilter() {
  //-------------Filter Time-----------------KALMAN Filter//
  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
  Pxv += timeStep * Pvv;
  Pxx += timeStep * giroVar;
  Pvv += timeStep * deltaGiroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;

  //-------------Filter End-----------------KALMAN Filter//  
}


void tiltCompCompass() {
  //tilt compensate the compass
  //Xmag = magX * cos(pitchAccelRad) + magY *sin(rollAccelRad * sin(pitchAccelRad)) - magZ * cos(rollAccelRad) * sin(pitchAccelRad);
  //Ymag = magY * cos(rollAccelRad) + magZ * sin(rollAccelRad);
  Xmag = magX * cos(pitchAccelRad) + magZ * sin(pitchAccelRad);
  Ymag = magY * sin(rollAccelRad) * sin(pitchAccelRad) + magY * cos(rollAccelRad) - magZ * sin(rollAccelRad) * cos(pitchAccelRad);
  
    realheading = atan2(Ymag, Xmag) * 360/(2*PI);
  
  /*
  MAG_X = magX*cos(pitchAccel) + magY * sin(rollAccel)*sin(pitchAccel) + magZ*cos(rollAccel)*sin(pitchAccel);
  // Tilt compensated Magnetic filed Y:
  MAG_Y = magY*cos(rollAccel) - magZ*sin(rollAccel);
  // Magnetic Heading
  realHeading = atan2(-MAG_Y,MAG_X) * (180/M_PI);
  */

  //if (Xmag >= 0 && Ymag >= 0) {realheading = 180 - realheading;}
  //if (Xmag >= 0 && Ymag < 0) {realheading = realheading + 180;}
  //if (Xmag < 0 && Ymag < 0) {realheading = 360 - realheading;}
 
}

