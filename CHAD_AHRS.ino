// This code controls the AHRS on the Controlled
// Heading Automation Device (CHAD) built by Andrew Kruger and
// students in the Physics 295 Independent Research course at
// Wright College
// Instructions and information found at http://physi.cz/chad

//Choose the time interval between data points in milliseconds:
unsigned long timePerData = 100;  //in long to match other variables

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// **********************  CONFIGURATION   **********************************
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here

// Pins ---ITG/MPU to SD Sheild --- : VCC(5V), SDA(A4), SCL(A5), int(Pin 2), GND(GND)
// Pins From --- HMC5883L to ITG/MPU--- : SCL(4700ohms-->3.3,XCL), SDA(XDA,4700oms), VCC(3.3,SD Sheild),GRD(GRD,SD Sheild)

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68); // Most common is 0x68


// ================================================================
// ===               HMC5883L              ===

// Variables used
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07
int16_t m_raw[3];     //To store magnetometer readings

float m_proc[3],m_HIO[3],m_avg[3];
float HardIronOffset[3] = {108.8207,-72.6777,83.3884};

// MAGNETOMETER HARD AND SOFT IRON CORRECTIONS <-------------------
// Needs to be changed for each CHAD, see http://physi.cz/chad
float ScaleMag[3] = {1.69383e-3,1.68302e-3,1.81764e-3};
float SoftIronXY = 3.51392e-5;
float SoftIronXZ = 4.41685e-5;
float SoftIronYZ = -9.21195e-5;


// ================================================================
// ===               MPU6050              ===

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int dmpInterrupt_pin=0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro[3];          // [x, y, z]            angular speed around axes


// ================================================================
// ===               DATA COLLECTION              ===

// To do data recording
unsigned long seconds, milliSinceLastSecond; 
boolean newdata;  //used for testing, may not be necessary
float decAngleAdj = 0.064984; //Not needed, but reminder that declination needs correction
bool blinkState = false;
//Magnetometer
unsigned long Ndata; //number of readings so far, note: needs to be long
int Ndata_int; //Number of magnetometer readings in a given magnetometer data point


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===           MAGNETOMETER CONFIGURATION ROUTINE             ===
// ================================================================

void configMagAxis(int chan) {
  mpu.setSlaveAddress(chan, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
  mpu.setSlaveEnabled(chan, true);
  mpu.setSlaveWordByteSwap(chan, false);
  mpu.setSlaveWriteMode(chan, false);
  mpu.setSlaveWordGroupOffset(chan, false);
  mpu.setSlaveDataLength(chan, 2);
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  // Flag for errors in setup
  int errorFlag = 0;

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  //Serial.println(TWBR);
  // **************************************************************
  // It is best to configure I2C to 400 kHz. 
  // If you are using an Arduino DUE, modify the variable TWI_CLOCK to 400000, defined in the file:
  // c:/Program Files/Arduino/hardware/arduino/sam/libraries/Wire/Wire.h
  // If you are using any other Arduino instead of the DUE, uncomment the following line:
 //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)  //This line should be commented if you are using Arduino DUE
  // **************************************************************
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial1.begin(38400);  //115200 is too fast for Arduino Mini Pro, 38400 is max it can handle
  Serial.begin(38400);
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer

  // initialize device
  mpu.initialize();

  // verify connection
  if (!mpu.testConnection()) { errorFlag = 1;}
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // If you don't know yours, you can find an automated sketch for this task from: http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
  // scaled using offset detection software AccelGyro_calibration.ino
  mpu.setXAccelOffset(-1860);  
  mpu.setYAccelOffset(-171);   
  mpu.setZAccelOffset(1386);    

  mpu.setXGyroOffset(67);
  mpu.setYGyroOffset(-65);
  mpu.setZGyroOffset(-11);

  // In case you want to change MPU sensors' measurements ranges, you should implement it here. This has not been tested with DMP.

  // Magnetometer configuration

  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x02); 
  Wire.write(0x00);  // Set continuous mode
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x00);
  Wire.write(B00011000);  // 75Hz
  Wire.endTransmission();
  delay(5);

  mpu.setI2CBypassEnabled(0);

  mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
  mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
  mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
  configMagAxis(0);
  configMagAxis(1);
  configMagAxis(2);
  
  mpu.setI2CMasterModeEnabled(1);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(dmpInterrupt_pin, dmpDataReady, FALLING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR! if:
    // devstatus=1 => initial memory load failed
    // devstatus=2 => DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    errorFlag = 2;
  }
  
  if (errorFlag != 0) {
    Serial.print("Error ");
    Serial.print(errorFlag);
  } else {
     Serial.print("Setup complete.");
  }
  
  Ndata =0;
  
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) { }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    // Keep reading out buffer until newest data is retreived
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      newdata = true;  //flag that new data is available.
    }

    if (millis() < Ndata*timePerData && newdata) {
      readCompass();
    }

    if (millis() > Ndata*timePerData && newdata) { 
      readCompass();
      calcDirection();
    } 
  }
}


// ================================================================
// ===                    SUB-ROUTINES                       ===
// ================================================================

// Read HMC5883L and MPU6050 data and calculate pitch, roll, heading

void calcDirection() {

  //Read mpu6050 and calculate heading
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
  mpu.dmpGetGyro(gyro,fifoBuffer);      
  
  // Gravity vectors gyroscope data and kalman filter to remove acceleration
  // due to shaking and spinning, more accurate than using just accelerometer.
  float Gx = -gravity.y;
  float Gy = -gravity.z;
  float Gz = -gravity.x;
  float roll = atan2(Gy,Gz);
  float pitch = atan2(-Gx,Gy*sin(roll)+Gz*cos(roll));
  
  // Axes depend on orientation of magnetometer, see http://physi.cz/chad 
  float Bx=-m_avg[1]/Ndata_int;
  float By=m_avg[2]/Ndata_int;
  float Bz=-m_avg[0]/Ndata_int;
  
  // Calculate yaw with horizontal magnetic field vectors
  float XhScaled = Bx * cos(pitch) + By * sin(pitch) * sin(roll) + Bz * sin(pitch) * cos(roll);
  float YhScaled = By * cos(roll) - Bz * sin(roll);      
  float yaw = -atan2(YhScaled, XhScaled);
  Serial1.print(gyro[0]);
  Serial1.print(",");
  Serial1.println(yaw*180/3.14159);
  Serial.print(gyro[0]);
  Serial.print(",");
  Serial.println(yaw*180/3.14159);
  
  //Prep for next data point
  blinkState = !blinkState;
  digitalWrite(13, blinkState);
  Ndata=Ndata+1;
  Ndata_int = 0;
  m_avg[0] = 0;
  m_avg[1] = 0;
  m_avg[2] = 0;
  
}
  

void readCompass() {
  
  m_raw[0]=mpu.getExternalSensorWord(0);
  m_raw[1]=mpu.getExternalSensorWord(2);
  m_raw[2]=mpu.getExternalSensorWord(4);
  
  m_HIO[0] = m_raw[0] - HardIronOffset[0];
  m_HIO[1] = m_raw[1] - HardIronOffset[1];
  m_HIO[2] = m_raw[2] - HardIronOffset[2];
  
  // Calibrate the raw magnetic field vectors, correcting for both
  // hard and soft iron effects
  m_proc[0] = m_HIO[0]*ScaleMag[0] + m_HIO[1]*SoftIronXY + m_HIO[2]*SoftIronXZ;
  m_proc[1] = m_HIO[0]*SoftIronXY + m_HIO[1]*ScaleMag[1] + m_HIO[2]*SoftIronYZ;
  m_proc[2] = m_HIO[0]*SoftIronXZ + m_HIO[1]*SoftIronYZ + m_HIO[2]*ScaleMag[2];
  
  m_avg[0]+=m_proc[0];
  m_avg[1]+=m_proc[1];
  m_avg[2]+=m_proc[2];
  
  Ndata_int+=1;
  newdata = false;
  
}
