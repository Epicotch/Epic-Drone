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
- Translational and rotational control (possibly follower-esque?)
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

BNO08x imu;

void driverSelect(uint8_t);

void setup() {

    // Initialize USB serial
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("Booting up!");

    // Initialize radio stuff
    Serial7.begin(9600);
    while(!Serial7) delay(10);

    Serial7.println("hello");

    pinMode(XBEE_RESET, OUTPUT);
    pinMode(RSSI, INPUT);

    // Initialize GPS serial
    Serial1.begin(9600);

    // Initialize motor driver I2C
    Wire.setSCL(19);
    Wire.setSDA(18);
    Wire.setClock(100000);
    Wire.begin();

    Serial.println("I2C 0 initialized");

    if (!imu.begin(0x4A, Wire, IMU_INT, IMU_RST)) {
        Serial.println("Oops! Could not find IMU.");
        while (1) ;
    }
    Serial.println("IMU initialized");

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

// Helper functions

void driverSelect(uint8_t i) {
    // Note: i = 0 corresponds to driver 1.
    
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();

}