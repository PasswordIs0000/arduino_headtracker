// based on:
// https://github.com/ElectronicCats/mpu6050/blob/master/examples/MPU6050_DMP6/MPU6050_DMP6.ino
// https://sourceforge.net/projects/hatire/files/ARDUINO/MPU6050/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// hatire frame
typedef struct {
    int16_t Begin;  // 2  Debut
    uint16_t Cpt;   // 2  Compteur trame or Code
    float gyro[3];  // 12 [Y, P, R]    gyro
    float pos[3];   // 12 [x, y, z]    position
    int16_t End;    // 2  Fin
} hatFrame;
hatFrame hat;

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// general information about the tracking
bool doRecentre = true;
unsigned long lastRecentre = 0;

// automatic re-centre and drift correction for the gyro
float meanGyro[3];
float zeroGyro[3];

// helper
#define SET_ARRAY_ZERO(_arr) { (_arr)[0] = 0.0; (_arr)[1] = 0.0; (_arr)[2] = 0.0; }

// enable for human-readable serial communication, else encoded for hatire/opentrack
// #define HUMAN_READABLE_MODE

// hardware configuration
#define INTERRUPT_PIN 7

// control the zero estimation
#define ZERO_DECAY_FACTOR 0.9

// control the gyro
#define GYRO_DECAY_FACTOR 0.99
#define GYRO_WARMUP_MILLIS 500
#define GYRO_NEUTRAL_WINDOW_YAW 5.0
#define GYRO_NEUTRAL_WINDOW_PITCH 5.0
#define GYRO_NEUTRAL_WINDOW_ROLL 3.0

void setup() {
    // initialize the serial connection
    Serial.begin(115200);
    delay(1000);

// initialize the sensor
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // use https://forum.arduino.cc/index.php?action=dlattach;topic=397918.0;attach=206004 to find your offsets
    mpu.setXAccelOffset(-2137);
    mpu.setYAccelOffset(1454);
    mpu.setZAccelOffset(453);
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(-7);
    mpu.setZGyroOffset(-13);

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel();
        mpu.CalibrateGyro();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // initialize the hatire frame
    hat.Begin = 0xAAAA;
    hat.Cpt = 0;
    hat.End = 0x5555;
    SET_ARRAY_ZERO(hat.pos);

    // meta-data for the iterations
    doRecentre = true;

    // clear the serial input buffer
    while (Serial.available()) {
        Serial.read();
    }

    // turn off the integrated led
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // centre happened in opentrack?
    while (Serial.available()) {
        switch (Serial.read()) {
            case 'C':
                doRecentre = true;
                break;
            default:
                break;
        }
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) {
        // Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
        // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // read a packet from FIFO
        while (fifoCount >= packetSize) {  // Lets catch up to NOW, someone is using the dreaded delay()!
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
        }

        // yaw, pitch and roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // time measurement
        const unsigned long cur_time = millis();

        // fill the hatire frame
        // mapped and inverted the axes so it matches my hardware configuration
        if (doRecentre) {
            SET_ARRAY_ZERO(meanGyro);
            SET_ARRAY_ZERO(hat.gyro);
            SET_ARRAY_ZERO(hat.pos);
            zeroGyro[0] = ypr[0];
            zeroGyro[1] = ypr[1];
            zeroGyro[2] = ypr[2];
        } else {
            // gyro is in absolute -180 to +180 degree
            hat.gyro[0] = (+1.0 * (ypr[0]-zeroGyro[0]) * 180.0) / M_PI;
            hat.gyro[1] = (-1.0 * (ypr[2]-zeroGyro[2]) * 180.0) / M_PI;
            hat.gyro[2] = (-1.0 * (ypr[1]-zeroGyro[1]) * 180.0) / M_PI;
        }
        
        // meta-data for next step
        if (doRecentre) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            lastRecentre = cur_time;
        }

        // update the zero values
        if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS) {
            zeroGyro[0] = (ZERO_DECAY_FACTOR * zeroGyro[0]) + ((1.0-ZERO_DECAY_FACTOR) * ypr[0]);
            zeroGyro[1] = (ZERO_DECAY_FACTOR * zeroGyro[1]) + ((1.0-ZERO_DECAY_FACTOR) * ypr[1]);
            zeroGyro[2] = (ZERO_DECAY_FACTOR * zeroGyro[2]) + ((1.0-ZERO_DECAY_FACTOR) * ypr[2]);
        }

        // automatic re-centre and drift correction for the gyro
        if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS || fabs(meanGyro[0]-hat.gyro[0]) < GYRO_NEUTRAL_WINDOW_YAW) {
            meanGyro[0] = (GYRO_DECAY_FACTOR * meanGyro[0]) + ((1.0-GYRO_DECAY_FACTOR) * hat.gyro[0]);
        }
        if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS || fabs(meanGyro[1]-hat.gyro[1]) < GYRO_NEUTRAL_WINDOW_PITCH) {
            meanGyro[1] = (GYRO_DECAY_FACTOR * meanGyro[1]) + ((1.0-GYRO_DECAY_FACTOR) * hat.gyro[1]);
        }
        if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS || fabs(meanGyro[2]-hat.gyro[2]) < GYRO_NEUTRAL_WINDOW_ROLL) {
            meanGyro[2] = (GYRO_DECAY_FACTOR * meanGyro[2]) + ((1.0-GYRO_DECAY_FACTOR) * hat.gyro[2]);
        }
        if (doRecentre) {
            meanGyro[0] = hat.gyro[0];
            meanGyro[1] = hat.gyro[1];
            meanGyro[2] = hat.gyro[2];
        }
        hat.gyro[0] = hat.gyro[0] - meanGyro[0];
        hat.gyro[1] = hat.gyro[1] - meanGyro[1];
        hat.gyro[2] = hat.gyro[2] - meanGyro[2];

        // don't re-centre in the next step
        doRecentre = false;

#ifdef HUMAN_READABLE_MODE
        if (devStatus == 0) {
            Serial.print("Yaw, Pitch, Roll:\t");
            Serial.print(hat.gyro[0]);
            Serial.print("\t");
            Serial.print(hat.gyro[1]);
            Serial.print("\t");
            Serial.println(hat.gyro[2]);
        } else {
            Serial.println("Device not ready!!!");
        }
#else
        // send the hatire frame
        Serial.write((byte*)&hat, 30);

        // increase the frame counter
        hat.Cpt++;
        if (hat.Cpt > 999) {
            hat.Cpt = 0;
        }
#endif
    }
}
