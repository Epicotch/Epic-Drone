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

enum NavMode {
    IDLE,
    WAYPOINT_SERIES,
    RETURN_HOME,
    MANUAL,
    // add more stuff if need be
};

enum FlightMode {
    FLIGHT_IDLE,
    LANDED,
    HOVER,
    WAYPOINT,
    VELOCITY,
    // add more stuff if need be
};

enum NavMode navMode = NavMode::IDLE;
enum FlightMode flightMode = FlightMode::IDLE;

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
volatile float pos[3];
volatile float heading[3];

BNO08x imu;
TinyGPSPlus gps;

// Function initializations
void driverSelect(uint8_t);
void flightState(enum FlightMode);
void updateSensors();
float latToM(float, float);
float lngToM(float, float, float);

void navigationStateMachine();
void flightStateMachine();

void setup() {

    // Initialize USB serial
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("Booting up!");

    // Initialize radio stuff
    Serial7.begin(57600);
    while(!Serial7) delay(10);

    Serial7.println("hello");

    if (!SD.begin(SD_CS)) {
        Serial.println("SD card attachment failed!");
        while (1);
    }

    pinMode(XBEE_RESET, OUTPUT);
    pinMode(RSSI, INPUT);

    // Initialize GPS serial
    Serial1.begin(57600); // TODO: Configure the GPS to use higher baud rates and refresh rates.

    delay(5000); // allow GPS module to get locks and calibrate itself a bit

    while (Serial1.available() == 0);
    while (!gps.encode(Serial1.read())) {
        while (Serial1.available() == 0);
    }

    // Get GPS zeroes
    gpsZero[0] = gps.location.lat();
    gpsZero[1] = gps.location.lng();
    gpsZero[2] = gps.altitude.meters();

    Serial.println("GPS Initialized");

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

    prevTime = micros();
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
        navigationStateMachine();
        flightStateMachine();
    }
    else {
        // stuff to do while we're paused
    }
}

void navigationStateMachine() {
    switch (navMode) {
        case IDLE:
            // do nothing - use if the drone is intended to stay still (flight state machine IDLE, LANDED, or HOVER)
            break;
        case WAYPOINT_SERIES:
            // fly a series of waypoints as read from the SD card, or defined via radio command
            break;
        case RETURN_HOME:
            // basically a waypoint series but specifically for returning to home
            break;
        case MANUAL:
            // control via radio controls - loss of radio connection results in defaulting to RETURN_HOME
            break;
    }
}

void flightStateMachine() {
    switch(flightMode) {
        case FLIGHT_IDLE:
            // do nothing
            break;
        case LANDED:
            // spun up, but on ground
            break;
        case HOVER:
            // hovering in place via PID control
            break;
        case WAYPOINT:
            // flying to waypoint via PID control
            break;
        case VELOCITY:
            // fly at target velocity
            break;
    }
}

// Updates position sensors based on whatever's coming in
// TODO: ADD BAROMETER SUPPORT
void updateSensors() {
    float gpsDelta[3] = {0, 0, 0};
    float gpsPos[3] = {gpsZero[0], gpsZero[1], gpsZero[2]};
    float gpsVel[3] = {0, 0, 0};
    float imuDelta[3] = {0, 0, 0};
    float imuQuat[4] = {0, 0, 0, 0}; // quaternion in i, j, k, real
    float rotatedImuAccel[3] = {0, 0, 0};
    float imuVel[3] = {0, 0, 0};
    int translationalAccuracy = 0;
    int rotationalAccuracy = 0;
    while (1) {
        // TODO: Possibly update IMU to use absolute positioning if need be?
        if (imu.getSensorEvent()) {
            switch (imu.getSensorEventID()) {
                case SENSOR_REPORTID_LINEAR_ACCELERATION:
                    float imuAccel[3] = {imu.getLinAccelX(), imu.getLinAccelY(), imu.getLinAccelZ()};
                    translationalAccuracy = imu.getAccelAccuracy();

                    // Quaternion stuff. Fun! (https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion_rotation_operations)
                    float a = imuQuat[3];
                    float b = -imuQuat[0];
                    float c = -imuQuat[1];
                    float d = -imuQuat[2];

                    float s = 2 / (a * a + b * b + c * c + d * d);
                    float bs = b * s;
                    float cs = c * s;
                    float ds = d * s;
                    float ab = a * bs;
                    float ac = a * cs;
                    float ad = a * ds;
                    float bb = b * bs;
                    float bc = b * cs;
                    float bd = b * ds;
                    float cc = c * cs;
                    float cd = c * ds;
                    float dd = d * ds;

                    rotatedImuAccel[0] = (1 - cc - dd) * imuAccel[0] + (bc - ad) * imuAccel[1] + (bd + ac) * imuAccel[2];
                    rotatedImuAccel[1] = (bc + ad) * imuAccel[0] + (1 - bb - dd) * imuAccel[1] + (cd - ab) * imuAccel[2];
                    rotatedImuAccel[2] = (bd - ac) * imuAccel[0] + (cd + ab) * imuAccel[1] + (1 - bb - cc) * imuAccel[2];
                    break;
                case SENSOR_REPORTID_ROTATION_VECTOR:
                    imuQuat[0] = imu.getQuatI();
                    imuQuat[1] = imu.getQuatJ();
                    imuQuat[2] = imu.getQuatK();
                    imuQuat[3] = imu.getQuatReal();
                    rotationalAccuracy = imu.getQuatAccuracy();
                    break;
            }

            currTime = micros();
            
            double deltaTime = (currTime - prevTime) / 1000000.0;

            imuVel[0] += rotatedImuAccel[0] * deltaTime;
            imuVel[1] += rotatedImuAccel[1] * deltaTime;
            imuVel[2] += rotatedImuAccel[2] * deltaTime;

            imuDelta[0] = pos[0] + imuVel[0] * deltaTime;
            imuDelta[1] = pos[0] + imuVel[1] * deltaTime;
            imuDelta[2] = pos[0] + imuVel[2] * deltaTime;

            prevTime = currTime;
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

            gpsDelta[0] = latToM(gpsZero[0], gpsPos[0]);
            gpsDelta[1] = lngToM(gpsZero[1], gpsPos[1], gpsPos[0]);
            gpsDelta[2] = gpsPos[2] - gpsZero[2];

            double gpsSpeed = gps.speed.mps();

            gpsHeading = gps.course.deg();

            gpsVel[0] = sin(gpsHeading) * gpsSpeed;
            gpsVel[1] = cos(gpsHeading) * gpsSpeed;
            gpsVel[2] = imuVel[2]; // idk probably not going to use these sensors for altitude anyway
        }

        // Kalman time babyyyyyyy

        

    }
}

// Helper functions

float latToM(float latStart, float latEnd) {
    float beg = 111132.92 - (559.82 * sin(2 * latEnd)) + (1.175 * sin(4 * latEnd)) - (0.0023 * sin(6 * latEnd));
    float end = 111132.92 - (559.82 * sin(2 * latStart)) + (1.175 * sin(4 * latStart)) - (0.0023 * sin(6 * latStart));
    return beg - end;
}

float lngToM(float lngStart, float lngEnd, float lat) {
    return (lngEnd - lngStart) * (111412.84 * cos(2 * lat)) - (93.5 * cos(3 * lat)) + (0.118 * cos(5*lat));
}

void driverSelect(uint8_t i) {
    // Note: i = 0 corresponds to driver 1.
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();
}