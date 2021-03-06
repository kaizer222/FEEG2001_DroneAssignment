/* ===   Combined Logging and Gimbal Firmware   ===
 *
 * Pin  No.   |            Description          |
 * ==============================================
 * Pin  13    |   MPU status                    |
 * Pin  12    |   DMP Init Status               |
 * Pin  02    |   MPU Interrupt Pin             |
 * pin  03    |   Toggle Pin                    |
 * Pin  A4    |   MPU SDA                       |
 * Pin  A5    |   MPU SCL                       |
 * Pin  10    |   DataLogger Shield ChipSelect  |
 * Pin  11    |   Yaw Servo                     |
 * Pin  9     |   Pitch Servo                   |
 * Pin  8     |   Roll Servo                    |
 */

// ========================   MPU required libraries    ======================== //
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif


// ========================   Data logger required libraries  ======================== //
#include "RTClib.h"
#include "SD.h"
// ========================   Gimbal required libraries       ======================== //
#include <Servo.h>

// ========================   MPU initialisation              ======================== //
MPU6050 mpu (0x69);
#define OUTPUT_READABLE_YAWPITCHROLL      // to see the yaw/ pitch/roll angles (in degrees) calculated from the quaternions coming from the FIFO.
#define INTERRUPT_PIN 2                   // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13                        // MPU status now tied to Pin 13 LED
bool blinkState = false; 

// MPU control/status variables
bool dmpReady = false;                    // set true if DMP init was successful
uint8_t mpuIntStatus;                     // holds actual interrupt status byte from MPU
uint8_t devStatus;                        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                      // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                   // FIFO storage buffer

// Orientation/motion variables
Quaternion q;                             // [w, x, y, z]         quaternion container
VectorInt16 aa;                           // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                       // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                      // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                      // [x, y, z]            gravity vector
float euler[3];                           // [psi, theta, phi]    Euler angle container
float ypr[3];                             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Interrupt detection routine
volatile bool mpuInterrupt = false;       // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ======================== DataLogger Initialisation   ======================== //

//#define ECHO_TO_SERIAL                    // Echo data to serial port
#define WAIT_TO_START                     // wait for serial input in setup()
const int chipSelect = 10;                // for the data logging shield, use digital pin 10

DateTime now;
// For debugging use

#define DEBUG {\
    Serial.print("\nDEBUG: LINE ");\
    Serial.println(__LINE__);\
}

// Print error to serial monitor
#define cerr(x) {\
    Serial.print("\nERROR: ");\
    Serial.println(x);\
    Serial.println("Please reset/restart the program.");\
    while(true);\
}

File logfile;
RTC_DS1307 RTC;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 5000; 
// ========================   Servo Initialisation      ======================== //
Servo servo_y;
Servo servo_p;
Servo servo_r;
int j = 0;
float correct;

// ========================     Toggle Initialisation    ======================== //
volatile bool lock_flag;
volatile int pwm_value = 0;
volatile int prev_time = 0;

void rising() {
    attachInterrupt(1, falling, FALLING);
    prev_time = micros();
}
 
void falling() {
    attachInterrupt(1, rising, RISING);
    pwm_value = micros() - prev_time;
    //Serial.println(pwm_value);
    if (pwm_value < 900){
        lock_flag = true;
    }
    else if (pwm_value > 2000){
        lock_flag = false;
    }
}


void setup() {
// ========================       Serial Setup          ======================== //
    Serial.begin(115200);                   // Change to project baud rate
    //Serial.print(F("Current date: "));
    //Serial.println(__DATE__);
    //Serial.print(F("Current time: "));
    //Serial.println(__TIME__);
  
// ========================         MPU Setup           ========================//
// Join I2C bus
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(250000);
    
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

// Initialise MPU device
  //Serial.println(F("Initialising I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

// Verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// Load and configure DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

// ***** GYRO OFFSET HERE! ***** 
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);
  
// Verify MPU set up works
  if (devStatus == 0) {
// Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
    // mpu.PrintActiveOffsets();
    
// Turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

// Enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

// Set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

// get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();} 
  else {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")."));
  } 

// Configure LED for output
pinMode(LED_PIN, OUTPUT);

// ========================       DataLogger Setup    ========================//

// Initialise the SD card
  //Serial.println(F("Initialising SD card..."));                               // make sure that the default chip select pin is set to output, even if you don't use it:
  pinMode(10, OUTPUT);

// See if the card is present and can be initialised:
  if (!SD.begin(chipSelect)) {
      //cerr(F("Card failed or not present."));
  }
  //Serial.println(F("Card initialised."));

// Create a new file
  char filename[] = "LOGGER00.CSV";
  for (int i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
// Only open a new file if it doesn't exist
    if (!SD.exists(filename)) {
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop
      }
    }

  if (!logfile) {
      //cerr("Couldn't create file.");
  }

  //Serial.print(F("Logging to: "));
  //Serial.println(filename);

  // Connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
      logfile.println("RTC failed");
#ifdef ECHO_TO_SERIAL
      //Serial.println(F("RTC failed"));
#endif  //ECHO_TO_SERIAL
  }

  logfile.println("ms,Unix Timestamp,Yaw,Pitch,Roll,X,Y,Z");  // CSV format
#ifdef ECHO_TO_SERIAL
  //Serial.println(F("millis,unix_stamp,datetime,x,y,z"));
#endif  //ECHO_TO_SERIAL

  // If you want to set the AREF to something other than 5V
  // analogReference(EXTERNAL);

// ========================     Servo Set Up      ========================//
  servo_y.attach(11);
  servo_p.attach(9);
  servo_r.attach(8);

// ========================     Toggle Set Up      ========================//
attachInterrupt(1, rising, RISING);

// ========================     TIMER START      ========================//
startMillis = millis();
}

void loop() {
// ========================     MPU Main Loop     ========================//
// If programming failed, don't try to do anything
  if (!dmpReady) return;
// Read a packet from FIFO
// Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {                     // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }                                                                 //if you are really paranoid you can frequently test in between other stuff to see if mpuInterrupt is true, and if so, "break;" from the while() loop to immediately process the MPU data
    }

// Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

// Get current FIFO count
    fifoCount = mpu.getFIFOCount();

// Check for overflow
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) { 
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
// display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
// ========================     Gimbal Implementation Here     ========================// 
// Lock gimbal if toggle is high
          if (lock_flag) {
              servo_y.write(90);
              servo_p.write(90);
              servo_r.write(90);
          }
// Else continue with gimbal operation
          else {
            // Yaw, Pitch, Roll values - Radians to degrees
            ypr[0] = ypr[0] * 180 / M_PI;
            ypr[1] = ypr[1] * 180 / M_PI;
            ypr[2] = ypr[2] * 180 / M_PI;
            if (j <= 300){
// Skip 300 readings for calibration process
              correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
              j++;
            } else {
              byte servo0Value = map(ypr[0], -90, 90, 0, 180);
              Serial.println(ypr[1]);
              byte servo1Value = map(ypr[1], -90, 90, 0, 180);
              byte servo2Value = map(ypr[2], -90, 90, 180, 0);
              
// Control the servos according to the MPU6050 orientation
              servo_y.write(servo0Value);
              servo_p.write(servo1Value);
              servo_r.write(servo2Value); 
            }
          }

      #endif  // OUTPUT_READABLE_YAWPITCHROLL
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    
    logfile.print((uint8_t) ypr[0]);
    logfile.print(',');
    logfile.print((uint8_t) ypr[1]); 
    logfile.print(',');
    logfile.print((uint8_t) ypr[2]);
    logfile.print(',');
    logfile.print((uint8_t)aaReal.x);
    logfile.print(',');
    logfile.print((uint8_t)aaReal.y);
    logfile.print(',');
    logfile.print((uint8_t)aaReal.z);
    logfile.print(','); 
    logfile.print('\n');
    logfile.flush();     
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }else{
    //do nothing
  }

}
