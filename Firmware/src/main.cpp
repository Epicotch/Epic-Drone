/**
 * 
 * Cardinal V1 controller firmware
 * Author: Bernard Jin (bljin@mit.edu)
 * Created: 3 January 2025
 * 
 */

// includes
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <MCF8329A.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <TeensyThreads.h>
#include <pins.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <EEPROM.h>

/**
 * 
 * Motor stuff:
 * - BASE_CURRENT = 30A
 * - CSA max is 983.04A
 * - BUS_VOLT is 50V
 * - rest is just figurin it out (https://www.ti.com/lit/ug/sllu374/sllu374.pdf)
 * 
 */

/*
TODO:

NOTE: RESET PIN IS ACTIVE LOW, YOU DOOF! MAKE SURE TO FIX THAT WHEN ACTUALLY SOLDERING UP THE BOARD YA EEJIT

To write:
- Motor driver config stuff
- Motor startup script
- GPS stuff
- IMU integrator
- Barometer integration
- Sensor fusion
- PID
- Motor control
- Translational and rotational control (possibly follower-esque? pure pursuit?)
- State machines!
    - Flight modes
- Return to home
- Go to waypoint
*/

/**
 * 
 * Various variables
 * 
 */

const int SD_CS = BUILTIN_SDCARD;

int motor1PWM = 0;
int motor2PWM = 0;
int motor3PWM = 0;
int motor4PWM = 0;

unsigned long currTime, prevTime;

bool armed = false;

// TODO: Read this stuff from the EEPROM
float Kp_translation;
float Ki_translation; 
float Kd_translation;

float Kp_roll;
float Ki_roll;
float Kd_roll;

float Kp_pitch;
float Ki_pitch;
float Kd_pitch;

float Kp_yaw;
float Ki_yaw;
float Kd_yaw;

float maxI;
float maxRoll;
float maxPitch;
float maxYawRate;

float gpsZero[3];
volatile float gpsDelta[3];
volatile float gpsPos[3];
volatile float imuDelta[3];
volatile float imuHeading[3];
volatile float imuAccel[3];
float pos[3];
float heading[3];

BNO08x imu;
TinyGPSPlus gps;

void driverSelect(uint8_t);
void getPos(float*, float*);

void setup() {

    // Initialize USB serial
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("Booting up!");

    // Initialize radio stuff
    Serial7.begin(57600);
    while(!Serial7) delay(10);

    Serial7.println("hello");

    pinMode(XBEE_RESET, OUTPUT);
    pinMode(RSSI, INPUT);

    // Initialize GPS serial
    Serial1.begin(57600); // TODO: Configure the GPS to use higher baud rates and refresh rates.

    while (Serial1.available() == 0) ;
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    // Get GPS zeroes
    gpsZero[0] = gps.location.lat();
    gpsZero[1] = gps.location.lng();
    gpsZero[2] = gps.altitude.meters();

    // Initialize I2C 0
    Wire.setSCL(19);
    Wire.setSDA(18);
    Wire.setClock(100000);
    Wire.begin();

    Serial.println("I2C 0 initialized");

    if (!imu.begin(0x4A, Wire, IMU_INT, IMU_RST)) {
        Serial.println("Oops! Could not find IMU.");
        while (1) ;
    }
    if (!imu.enableAccelerometer()) {
        Serial.println("Oops! Could not initialize accelerometer.");
        while (1) ;
    }
    if (!imu.enableRotationVector()) {
        Serial.println("Oops! Could not initialize gyroscope/magnetometer.");
        while (1) ;
    }

    Serial.println("IMU initialized");

    threads.addThread(updateSensors);

    Serial.println("Sensor thread started");

    // Initialize I2C 1
    Wire1.setSCL(16);
    Wire1.setSDA(17);
    Wire1.begin();

    Serial.println("I2C 1 initialized");

    // Initialize I2C 2
    Wire2.setSCL(24);
    Wire2.setSDA(25);
    Wire2.begin();

    Serial.write("I2C 2 initialized");

    // Initialize UART 1
    // Serial2.begin(9600);
    // while(!Serial2) delay(10);

    // Initialize UART 2
    // Serial8.begin(9600);
    // while(!Serial8) delay(10);

    // Initialize motor PWMs
    pinMode(MOTOR_1, OUTPUT);
    pinMode(MOTOR_2, OUTPUT);
    pinMode(MOTOR_3, OUTPUT);
    pinMode(MOTOR_4, OUTPUT);

    // Initialize other motor pins
    pinMode(ENA_1, OUTPUT);
    pinMode(ENA_2, OUTPUT);
    pinMode(ENA_3, OUTPUT);
    pinMode(ENA_4, OUTPUT);

    pinMode(S_OX_1, INPUT);
    pinMode(S_OX_2, INPUT);
    pinMode(S_OX_3, INPUT);
    pinMode(S_OX_4, INPUT);

    pinMode(NFAULT_1, INPUT);
    pinMode(NFAULT_2, INPUT);
    pinMode(NFAULT_3, INPUT);
    pinMode(NFAULT_4, INPUT);

}

void loop() {
    if (armed) {
        // stuff goes on in here
    }
    else {
        // stuff to do while we're paused
    }
}

// Use extended Kalman filter to update our estimated position and heading.
void updatePos(double* pos, double* heading) {

}

// Updates position sensors based on whatever's coming in
// TODO: ADD BAROMETER SUPPORT
void updateSensors() {
    while (1) {
        // TODO: Possibly update IMU to use absolute positioning if need be?
        if (imu.getSensorEvent()) {
            switch (imu.getSensorEventID()) {
                case SENSOR_REPORTID_LINEAR_ACCELERATION:
                    imuAccel[0] = imu.getLinAccelX(); // m/s^2
                    imuAccel[1] = imu.getLinAccelY();
                    imuAccel[2] = imu.getLinAccelZ();
                    break;
                case SENSOR_REPORTID_ROTATION_VECTOR:
                    imuHeading[0] = imu.getRoll() * 180.0 / PI;
                    imuHeading[1] = imu.getPitch() * 180.0 / PI;
                    imuHeading[2] = imu.getYaw() * 180.0 / PI;
                    break;
            }
            // TODO: Dot each absolute axis with the lin accels
        }

        // grab GPS stuff
        bool valid = false;
        double gpsHeading;
        while (Serial1.available() > 0) {
            if(gps.encode(Serial1.read())) {
                valid = true;
            }
        }
        if (valid) {
            gpsPos[0] = gps.location.lat();
            gpsPos[1] = gps.location.lng();
            gpsPos[2] = gps.altitude.meters();

            double avgLat = (gpsPos[0] + gpsZero[0]) / 2.0f;

            // TODO: Possibly use integrated versions?
            double mPerDegLat = 111132.92 - (559.82 * cos(2 * avgLat)) + (1.175 * cos(4 * avgLat)) - (0.0023 * cos(6 * avgLat));
            double mPerDegLong = (111412.84 * cos(2 * avgLat)) - (93.5 * cos(3 * avgLat)) + (0.118 * cos(5*avgLat));

            gpsDelta[0] = (gpsPos[0] - gpsZero[0]) * mPerDegLat;
            gpsDelta[1] = (gpsPos[1] - gpsZero[1]) * mPerDegLat;
            gpsDelta[2] = gpsPos[2] - gpsZero[2];

            gpsHeading = gps.course.deg();
        }
    }
}

// Helper functions

void driverSelect(uint8_t i) {
    // Note: i = 0 corresponds to driver 1.
    
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();

}