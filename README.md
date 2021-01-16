# Arduino Headtracker

## Overview

This is a headtracker that uses an Arduino Micro and a BNO-055 9-axis INU sensor. The goal is to read the values from the sensor and send them in useful form over the serial connection. On the PC, [OpenTrack](https://github.com/opentrack/opentrack) can receive this data with the [Hatire](https://sourceforge.net/projects/hatire/) plugin. The software tries to compute an exponentially decaying mean of the gyroscope readings in order to automatically re-center if the head position is only slightly off and also to automatically remove gyroscope drift. In my OpenTrack configuration I'm sending a 'C' character on start. This will centre the gyroscope readings in the Arduino. I then have a keybind in OpenTrack to restart the tracking. This allows to re-centre in a neutral head position. Re-centre and drift correction is done on the Arduino, so OpenTrack should always receive 0 degrees as the neutral position.

## Required Arduino Libraries

- [Adafruit BNO-055](https://github.com/adafruit/Adafruit_BNO055)

## Required PC Software

- [OpenTrack](https://github.com/opentrack/opentrack)