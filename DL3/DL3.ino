/* ============================================
I2Cdev device library code is placed under the MIT License
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
============================================ */

#pragma once
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "MPU6050_6Axis_MotionApps20.h"

// For debug use
#define DEBUG {\
    Serial.print("DEBUG: LINE ");\
    Serial.println(__LINE__);\
}

// Print error to serial monitor
#define cerr(x) {\
    Serial.print("ERROR: ");\
    Serial.println(x);\
    Serial.println("Please reset/restart the program.");\
    while(true);\
}

// #define OUTPUT_READABLE_QUATERNION
// #define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL
// #define OUTPUT_TEAPOT

MPU6050 mpu;
File logfile;
RTC_DS1307 RTC;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno
bool blinkState = false;

// MPU control/status variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Packet structure for InvenSense teapot demonstration
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// how many milliseconds between grabbing data and logging it
#define LOG_INTERVAL 1000 // how many milliseconds between grabbing data and logging it. mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 1000  // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;  // time of last sync()
#define ECHO_TO_SERIAL  // Echo data to serial port
#define WAIT_TO_START   // wait for serial input in setup()
const int chipSelect = 10;  // for the data logging shield, use digital pin 10

// Interrupt detection routine
volatile bool mpuInterrupt = false;  // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// =======================================================================================================

void setup() {
    // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // 115200 baud is chosen because it is required for Teapot Demo output, but it
    // depends on your project.
    Serial.begin(115200);
    delay(500);
    DEBUG;
    Serial.print("Current date: ");
    Serial.println(__DATE__);
    Serial.print("Current time: ");
    Serial.println(__TIME__);

#ifdef WAIT_TO_START
    Serial.println("Type any character to start >>>");
    while (!Serial.available());
#endif  // WAIT_TO_START

    // Initialise device
    Serial.println(F("Initialising I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Wait for ready
    Serial.println(F("\nType any character to begin DMP programming and demo >>>"));
    while (Serial.available() && Serial.read());  // empty buffer
    while (!Serial.available());                  // wait for data
    while (Serial.available() && Serial.read());  // empty buffer again

    // Load and configure the DMP
    Serial.println(F("Initialising DMP..."));
    devStatus = mpu.dmpInitialize();

    // !!!! IMPORTANT !!!!
    // Supply your own gyro offset values here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // Error values:
        // 1 = Initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP initialisation failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")."));
    }

    // Configure LED for output
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialise the SD card
    Serial.println("Initialising SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);

    // See if the card is present and can be initialised:
    if (!SD.begin(chipSelect)) {
        cerr("Card failed or not present.");
    }
    Serial.println("Card initialised.");

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

    Serial.print("Logging to: ");
    Serial.println(filename);

    // Connect to RTC
    Wire.begin();  
    if (!RTC.begin()) {
        logfile.println("RTC failed");
#ifdef ECHO_TO_SERIAL
        Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
    }

    logfile.println("millis,unix_stamp,datetime,x,y,z");  
#ifdef ECHO_TO_SERIAL
    Serial.println("millis,unix_stamp,datetime,x,y,z");
#endif  //ECHO_TO_SERIAL

    // If you want to set the AREF to something other than 5V
    // analogReference(EXTERNAL);
}

// =======================================================================================================

void loop() {
    if (!dmpReady) return; // if programming failed, don't try to do anything

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        } 
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
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

#ifdef OUTPUT_READABLE_QUATERNION
        // Display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
#endif  // OUTPUT_READABLE_QUATERNION

#ifdef OUTPUT_READABLE_EULER
        // Display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180/M_PI);
#endif  // OUTPUT_READABLE_EULER

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // Display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif  // OUTPUT_READABLE_YAWPITCHROLL

#ifdef OUTPUT_READABLE_REALACCEL
        // Display real acceleration, adjusted to remove gravity
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
#endif  // OUTPUT_READABLE_REALACCEL

#ifdef OUTPUT_READABLE_WORLDACCEL
        // Display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
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
#endif  // OUTPUT_READABLE_WORLDACCEL

#ifdef OUTPUT_TEAPOT
        // Display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++;
#endif  // OUTPUT_TEAPOT

        // Blink built-in LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_BUILTIN, blinkState);
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
