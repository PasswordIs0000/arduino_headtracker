# Arduino Headtracker

## Overview

This is a headtracker that uses an Arduino Micro and a MPU-6050 6-axis acceleration+gyroscope sensor. The goal is to read the values from the sensor and send them in useful form over the serial connection. On the PC, [OpenTrack](https://github.com/opentrack/opentrack) can receive this data with the [Hatire](https://sourceforge.net/projects/hatire/) plugin. The software tries to compute an exponentially decaying mean of the gyroscope readings in order to automatically re-center if the head position is only slightly off and also to automatically remove gyroscope drift. In my OpenTrack configuration I'm sending a 'C' character on start. This will centre the gyroscope readings in the Arduino. I then have a keybind in OpenTrack to restart the tracking. This allows to re-centre in a neutral head position. Re-centre and drift correction is done on the Arduino, so OpenTrack should always receive 0 degrees as the neutral position.

## To-Do

Acceleration is still a to-do and doesn't seem to work correctly in the current state. Gyroscope for yaw, pitch and roll is usable.

## Calibration

The MPU needs calibration data for the setXAccelOffset, setYAccelOffset, setZAccelOffset, setXGyroOffset, setYGyroOffset  and setZGyroOffset. This may very well be different for every sensor and every mounting or specific application. Please use [this sketch](https://forum.arduino.cc/index.php?action=dlattach;topic=397918.0;attach=206004) to find the values for our specific case.

## Required Arduino Libraries

- [ElectronicCats MPU6050](https://github.com/ElectronicCats/mpu6050)

## Required PC Software

- [OpenTrack](https://github.com/opentrack/opentrack)