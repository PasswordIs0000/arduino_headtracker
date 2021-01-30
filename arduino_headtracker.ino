// based on:
// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
// https://sourceforge.net/projects/hatire/files/ARDUINO/MPU6050/

#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// enable for human-readable serial communication, else encoded for hatire/opentrack
// #define HUMAN_READABLE_MODE

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// hatire frame
typedef struct {
    int16_t Begin;  // 2  Debut
    uint16_t Cpt;   // 2  Compteur trame or Code
    float gyro[3];  // 12 [Y, P, R]    gyro
    float pos[3];   // 12 [x, y, z]    position
    int16_t End;    // 2  Fin
} hatFrame;
hatFrame hat;

// general information about the tracking
bool doRecentre = true;
unsigned long lastRecentre = 0;
unsigned long lastMeasurement = 0;

// automatic re-centre and drift correction for the gyro
float meanGyro[3];
float zeroGyro[3];
float lookGyro[3];

// helpers
#define SET_ARRAY_ZERO(_arr) { (_arr)[0] = 0.0; (_arr)[1] = 0.0; (_arr)[2] = 0.0; }

// correct degrees to be always within -180 and +180
float degrees_in_180(float deg) {
    while (deg < -180.0) {
        deg += 360.0;
    }
    while (deg > +180.0) {
        deg -= 360.0;
    }
    return deg;
}

// control the zero estimation
#define ZERO_DECAY_FACTOR 0.9

// control the gyro
#define GYRO_DECAY_FACTOR 0.99
#define GYRO_WARMUP_MILLIS 500
#define GYRO_SPEED_THRESHOLD 250.0
#define GYRO_NEUTRAL_WINDOW_YAW 2.0
#define GYRO_NEUTRAL_WINDOW_PITCH 2.0
#define GYRO_NEUTRAL_WINDOW_ROLL 2.0

void setup() {
    // initialize the serial connection
    Serial.begin(115200);
    delay(1000);

    // initialize the sensor
    if (!bno.begin()) {
        Serial.println("ERROR! No BNO-055 detected!");
        while (true) {
            delay(1000);
        }
    }
    bno.setExtCrystalUse(true);

    // initialize the hatire frame
    hat.Begin = 0xAAAA;
    hat.Cpt = 0;
    hat.End = 0x5555;
    SET_ARRAY_ZERO(hat.gyro);
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
    // is the sensor calibrated?
    uint8_t cal_system = 0;
    uint8_t cal_gyro = 0;
    uint8_t cal_accel = 0;
    uint8_t cal_mag = 0;
    bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);
    if (cal_system == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
        return;
    }
    if (cal_gyro < 3) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
        return;
    }

    // read from the sensor
    sensors_event_t orientation_data, gyro_data;
    const unsigned long t_sensor_start = millis(); 
    bno.getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
    const unsigned long t_sensor_finish = millis();
    
    // time measurement
    const unsigned long cur_time = (t_sensor_start + t_sensor_finish) / 2;

    // 3-array from -180 to +180 degrees each
    const float orientation[3] = {
        degrees_in_180(orientation_data.orientation.x),
        degrees_in_180(orientation_data.orientation.y),
        degrees_in_180(orientation_data.orientation.z)
    };

    // 3-array from -180 to +180 deegrees/sec each
    const float angular_speed[3] = {
        (gyro_data.gyro.x * 180.0) / M_PI,
        (gyro_data.gyro.y * 180.0) / M_PI,
        (gyro_data.gyro.z * 180.0) / M_PI
    };

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

    // reset for re-centre
    if (doRecentre) {
        lastRecentre = cur_time;
        SET_ARRAY_ZERO(meanGyro);
        SET_ARRAY_ZERO(lookGyro);
        SET_ARRAY_ZERO(hat.gyro);
        SET_ARRAY_ZERO(hat.pos);
        zeroGyro[0] = orientation[0];
        zeroGyro[1] = orientation[1];
        zeroGyro[2] = orientation[2];
    }
    
    // update the gyro zero values
    if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS) {
        zeroGyro[0] = (ZERO_DECAY_FACTOR * zeroGyro[0]) + ((1.0-ZERO_DECAY_FACTOR) * orientation[0]);
        zeroGyro[1] = (ZERO_DECAY_FACTOR * zeroGyro[1]) + ((1.0-ZERO_DECAY_FACTOR) * orientation[1]);
        zeroGyro[2] = (ZERO_DECAY_FACTOR * zeroGyro[2]) + ((1.0-ZERO_DECAY_FACTOR) * orientation[2]);
    }

    // hatire gyro is in absolute -180 to +180 degree
    // mapped and inverted the axes so it matches my hardware configuration
    hat.gyro[0] = +1.0 * (orientation[0]-zeroGyro[0]);
    hat.gyro[1] = -1.0 * (orientation[1]-zeroGyro[1]);
    hat.gyro[2] = -1.0 * (orientation[2]-zeroGyro[2]);

    // automatic re-centre and drift correction for the gyro
    if (fabs(angular_speed[0]) > GYRO_SPEED_THRESHOLD || fabs(angular_speed[1]) > GYRO_SPEED_THRESHOLD || fabs(angular_speed[2]) > GYRO_SPEED_THRESHOLD) {
        lookGyro[0] = hat.gyro[0];
        lookGyro[1] = hat.gyro[1];
        lookGyro[2] = hat.gyro[2];
        meanGyro[0] = hat.gyro[0];
        meanGyro[1] = hat.gyro[1];
        meanGyro[2] = hat.gyro[2];
    }
    if (doRecentre) {
        meanGyro[0] = hat.gyro[0];
        meanGyro[1] = hat.gyro[1];
        meanGyro[2] = hat.gyro[2];
    }
    if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS || fabs(meanGyro[0]-hat.gyro[0]) < GYRO_NEUTRAL_WINDOW_YAW) {
        meanGyro[0] = (GYRO_DECAY_FACTOR * meanGyro[0]) + ((1.0-GYRO_DECAY_FACTOR) * hat.gyro[0]);
    }
    if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS || fabs(meanGyro[1]-hat.gyro[1]) < GYRO_NEUTRAL_WINDOW_PITCH) {
        meanGyro[1] = (GYRO_DECAY_FACTOR * meanGyro[1]) + ((1.0-GYRO_DECAY_FACTOR) * hat.gyro[1]);
    }
    if ((cur_time-lastRecentre) < GYRO_WARMUP_MILLIS || fabs(meanGyro[2]-hat.gyro[2]) < GYRO_NEUTRAL_WINDOW_ROLL) {
        meanGyro[2] = (GYRO_DECAY_FACTOR * meanGyro[2]) + ((1.0-GYRO_DECAY_FACTOR) * hat.gyro[2]);
    }
    hat.gyro[0] = hat.gyro[0] - meanGyro[0] + lookGyro[0];
    hat.gyro[1] = hat.gyro[1] - meanGyro[1] + lookGyro[1];
    hat.gyro[2] = hat.gyro[2] - meanGyro[2] + lookGyro[2];

    // save the measurement time
    lastMeasurement = cur_time;

    // don't re-centre in the next step
    if (doRecentre) {
        lastMeasurement = 0;
    }
    doRecentre = false;

#ifdef HUMAN_READABLE_MODE
    Serial.print("Time\t");
    Serial.print(cur_time);
    Serial.print("\t");
    Serial.print("Calibration\t");
    Serial.print(cal_system);
    Serial.print("\t");
    Serial.print(cal_gyro);
    Serial.print("\t");
    Serial.print(cal_accel);
    Serial.print("\t");
    Serial.print(cal_mag);
    Serial.print("\t");
    Serial.print("Abs. Orientation\t");
    Serial.print(orientation[0]);
    Serial.print("\t");
    Serial.print(orientation[1]);
    Serial.print("\t");
    Serial.print(orientation[2]);
    Serial.print("\t");
    Serial.print("Angular speed\t");
    Serial.print(angular_speed[0]);
    Serial.print("\t");
    Serial.print(angular_speed[1]);
    Serial.print("\t");
    Serial.print(angular_speed[2]);
    Serial.print("\t");
    Serial.print("Orientation\t");
    Serial.print(hat.gyro[0]);
    Serial.print("\t");
    Serial.print(hat.gyro[1]);
    Serial.print("\t");
    Serial.print(hat.gyro[2]);
    Serial.print("\t");
    Serial.print("Position\t");
    Serial.print(hat.pos[0]);
    Serial.print("\t");
    Serial.print(hat.pos[1]);
    Serial.print("\t");
    Serial.println(hat.pos[2]);
#else
    // send the hatire frame
    Serial.write((byte*)&hat, 30);

    // increase the frame counter
    hat.Cpt++;
    if (hat.Cpt > 999) {
        hat.Cpt = 0;
    }
#endif

    delay(10);
}
