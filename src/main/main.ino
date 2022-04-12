/*Combined Logging and Gimbal Firmware
 * Pin List:
 * 
 *            |            Description          |
 * Pin  13    |   MPU status                    |
 * pIN  12    |   DMP Init Status               |
 * Pin  02    |   MPU Interrupt Pin             |
 * pin  03    |   Toggle Pin                    |
 * Pin  A4    |   MPU SDA                       |
 * Pin  A5    |   MPU SCL                       |
 * Pin  10    |   DataLogger Shield ChipSelect  |
 * Pin  11    |   Yaw Servo                     |
 * Pin  9     |   Pitch Servo                   |
 * Pin  8     |   Roll Servo                    |
 */

// ========================   MPU required libraries          ======================== //
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
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
#define DMP_PIN 12                        // DMP init status pin
bool blinkState = false; 

//MPU control/status variables
bool dmpReady = false;                    // set true if DMP init was successful
uint8_t mpuIntStatus;                     // holds actual interrupt status byte from MPU
uint8_t devStatus;                        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                      // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                   // FIFO storage buffer

//orientation/motion variables
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
/* how many milliseconds between grabbing data and logging it */
#define LOG_INTERVAL 1000                 // how many milliseconds between grabbing data and logging it. mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 1000                // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;                    // time of last sync()
#define ECHO_TO_SERIAL                    // Echo data to serial port
#define WAIT_TO_START                     // wait for serial input in setup()
const int chipSelect = 10;                // for the data logging shield, use digital pin 10

#define cerr(x) {\
    Serial.print(F("ERROR: "));\
    Serial.println(x);\
    Serial.println(F("Please reset/restart the program."));\
    while(true);\
}

File logfile;
RTC_DS1307 RTC;

// ========================   Servo Initialisation      ======================== //
Servo servo_y;
Servo servo_p;
Servo servo_r;
int j=0;
float correct;

// ========================     Toggle Initialisation    ======================== //
volatile boolean lock_flag;
#define TOGGLE_PIN 3
void GIMBAL_LOCK(){
  if (digitalRead(TOGGLE_PIN)==HIGH){
    lock_flag=true;
  }else{
    lock_flag=false;
  }
}

void setup() {
// ========================       Serial Setup          ======================== //
  Serial.begin(115200);                   // Change to project baud rate
  while (!Serial);                        // wait for Leonardo enumeration, others continue immediately
  Serial.print(F("Current date: "));
  Serial.println(__DATE__);
  Serial.print(F("Current time: "));
  Serial.println(__TIME__);
  
// ========================         MPU Setup           ========================//
pinMode (DMP_PIN,OUTPUT);
digitalWrite(DMP_PIN,LOW);
// join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //Wire.setClock(40000);                //uncommented due to stability issues
    
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true) 
  #endif

//Initialise MPU device
  Serial.println(F("Initialising I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

//verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//empty buffer
  while (Serial.available() && Serial.read());

//load and configure DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

// ***** GYRO OFFSET HERE! ***** 
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);
  
// Verify MPU set up works
  if (devStatus == 0) {
// Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    
// turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

// enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

// get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();} 
  else {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")."));
    digitalWrite(DMP_PIN,HIGH);
  } 

// configure LED for output
pinMode(LED_PIN, OUTPUT);

// ========================       DataLogger Setup    ========================//

// Initialise the SD card
  Serial.println(F("Initialising SD card..."));                               // make sure that the default chip select pin is set to output, even if you don't use it:
  pinMode(10, OUTPUT);

// See if the card is present and can be initialised:
  if (!SD.begin(chipSelect)) {
      cerr(F("Card failed or not present."));
  }
  Serial.println(F("Card initialised."));

// Create a new file
  char filename[] = "LOGGER00.CSV";
  for (int i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
// only open a new file if it doesn't exist
    if (!SD.exists(filename)) {
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop
      }
    }

  if (!logfile) {
      cerr("Couldn't create file.");
  }

  Serial.print(F("Logging to: "));
  Serial.println(filename);

  // Connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
      logfile.println("RTC failed");
#ifdef ECHO_TO_SERIAL
      Serial.println(F("RTC failed"));
#endif  //ECHO_TO_SERIAL
  }

  logfile.println("millis,unix_stamp,datetime,x,y,z");  
#ifdef ECHO_TO_SERIAL
  Serial.println(F("millis,unix_stamp,datetime,x,y,z"));
#endif  //ECHO_TO_SERIAL

  // If you want to set the AREF to something other than 5V
  // analogReference(EXTERNAL);

// ========================     Servo Set Up      ========================//
  servo_y.attach(11);
  servo_p.attach(9);
  servo_r.attach(8);

// ========================     Toggle Set Up      ========================//
attachInterrupt(1,GIMBAL_LOCK , CHANGE);
}

void loop() {
// ========================     MPU Main Loop     ========================//
// if programming failed, don't try to do anything
  if (!dmpReady) return;
// read a packet from FIFO
// wait for MPU interrupt or extra packet(s) available
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
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      #ifdef OUTPUT_READABLE_EULER
// display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetEuler(euler, &q);
          Serial.print(F("euler\t"));
          Serial.print(euler[0] * 180/M_PI);
          Serial.print(F("\t"));
          Serial.print(euler[1] * 180/M_PI);
          Serial.print(F("\t"));
          Serial.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
// display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          Serial.print(F("ypr\t"));
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print(F("\t"));
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print(F("\t"));
          Serial.println(ypr[2] * 180/M_PI);
// ========================     Gimbal Implementation Here     ========================// 
// Lock gimbal if toggle is high
          if (lock_flag==true) {
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
            }else{
              ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
// Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
//** MAP NEED TO BE CHANGED ACCORDINGLY

              int servo0Value = map(ypr[0], -90, 90, 0, 180);
              int servo1Value = map(ypr[1], -90, 90, 0, 180);
              int servo2Value = map(ypr[2], -90, 90, 180, 0);
              
// Control the servos according to the MPU6050 orientation
              servo_y.write(servo0Value);
              servo_p.write(servo1Value);
              servo_r.write(servo2Value); 
            }
                     
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
// display real acceleration, adjusted to remove gravity
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.println(aaReal.z);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
// display initial world-frame acceleration, adjusted to remove gravity and rotated based on known orientation from quaternion
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          Serial.print("aworld\t");
          Serial.print(aaWorld.x);
          Serial.print("\t");
          Serial.print(aaWorld.y);
          Serial.print("\t");
          Serial.println(aaWorld.z);
      #endif
  
// blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }

    DateTime now;

    // Delay for the amount of time we want between readings
    delay((LOG_INTERVAL ) - (millis() % LOG_INTERVAL));

    // Log milliseconds since starting
    uint32_t m = millis();
    logfile.print(m);       // milliseconds since start
    logfile.print(", ");    
#ifdef ECHO_TO_SERIAL
    Serial.print(m);        // milliseconds since start
    Serial.print(", ");  
#endif  // ECHO_TO_SERIAL

    // Fetch the time
    now = RTC.now();
    // log time
    logfile.print(now.unixtime());  // seconds since 1/1/1970
    logfile.print(", ");
    logfile.print('"');
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print('"');
#ifdef ECHO_TO_SERIAL
    Serial.print(now.unixtime());  // seconds since 1/1/1970
    Serial.print(", ");
    Serial.print('"');
    Serial.print(now.year(), DEC);
    Serial.print("/");
    Serial.print(now.month(), DEC);
    Serial.print("/");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
    Serial.print('"');
#endif  // ECHO_TO_SERIAL

    logfile.print(", ");    
    logfile.print("ypr\t");
    logfile.print(ypr[0] * 180/M_PI);
    logfile.print("\t");
    logfile.print(ypr[1] * 180/M_PI);
    logfile.print("\t");
    logfile.println(ypr[2] * 180/M_PI);
#ifdef ECHO_TO_SERIAL
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
#endif // ECHO_TO_SERIAL

    logfile.println();
#ifdef ECHO_TO_SERIAL
    Serial.println();
#endif // ECHO_TO_SERIAL

    if ((millis() - syncTime) < SYNC_INTERVAL) return;
    syncTime = millis();
    logfile.flush();
  }
}
