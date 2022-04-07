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
    Serial.print("\nDEBUG: LINE ");\
    Serial.println(__LINE__);\
}

// Print error to serial monitor
#define cerr(x) {\
    Serial.print("ERROR: ");\
    Serial.println(x);\
    Serial.println("Please reset/restart the program.");\
    while(true);\
}

// MPU6050 mpu;
MPU6050 mpu(0x69);
File logfile;

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

// how many milliseconds between grabbing data and logging it
//#define LOG_INTERVAL 1000 // how many milliseconds between grabbing data and logging it. mills between entries (reduce to take more/faster data)
//#define SYNC_INTERVAL 1000  // mills between calls to flush() - to write data to the card
//uint32_t syncTime = 0;  // time of last sync()
uint32_t logMillis = millis();
#define ECHO_TO_SERIAL  // Echo data to serial port
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
    Serial.begin(38400);
    delay(1000);  // crucial for stability

    Serial.println("Send any character to start >>>");
    while (!Serial.available());

    // Initialise device
    Serial.println(F("Initialising I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful.") : F("MPU6050 connection failed."));

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

    // See if the card is present and can be initialised
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
    logfile.println("Program runtime (s), Yaw, Pitch, Roll");
}

// =======================================================================================================

void loop() {
    if (!dmpReady) {
        cerr("DMP not ready.");
    }

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
        // Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // yaw pitch roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        if (millis()  - logMillis >= 1000) {  // only write to file every one second
            logfile.print(millis() / 1000);  // seconds since start
            logfile.print("s,");
            logfile.print(ypr[0] * 180/M_PI);
            logfile.print(",");
            logfile.print(ypr[1] * 180/M_PI);
            logfile.print(",");
            logfile.print(ypr[2] * 180/M_PI);
            logfile.println(",,");
            logfile.flush();
            logMillis = millis();
        }
#ifdef ECHO_TO_SERIAL
        Serial.print(millis() / 1000);
        Serial.print("s\t");          
        Serial.print("yaw ");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\tpitch ");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\troll ");
        Serial.println(ypr[2] * 180/M_PI);
#endif  // ECHO_TO_SERIAL

        // Blink built-in LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_BUILTIN, blinkState);
    }

    // Delay for the amount of time we want between readings
//    delay((LOG_INTERVAL) - (millis() % LOG_INTERVAL));

//    if ((millis() - syncTime) < SYNC_INTERVAL) return;
//    syncTime = millis();
//    logfile.flush();
    delay(10);
}
