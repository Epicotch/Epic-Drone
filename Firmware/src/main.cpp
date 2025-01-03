/**
 * 
 * Cardinal V1 controller firmware
 * Author: Bernard Jin (bljin@mit.edu)
 * Created: 3 January 2025
 * 
 */

// includes
#include <Arduino.h>
#include <MCF8329A.h>
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

To write:
- Motor driver config stuff
- Motor startup script
- GPS stuff
- IMU integrator
- Kalman filter
- Barometer integration
- PID
- Motor control
- Drone control (in general)
- Return to home
- Go to waypoint
*/

/**
 * 
 * Various variables
 * 
 */

int motor1PWM = 0;
int motor2PWM = 0;
int motor3PWM = 0;
int motor4PWM = 0;


// TODO: Read this stuff from the EEPROM
float kP; 
float kI; 
float kD;

void setup() {

    // Initialize USB serial
    Serial.begin(115200);
    Serial.write("Booting up!");

    // Initialize radio stuff
    Serial7.begin(115200);
    Serial7.write("hello");

    pinMode(XBEE_RESET, OUTPUT);
    pinMode(RSSI, INPUT);

    // Initialize GPS serial
    Serial1.begin(9600);

    // Initialize motor driver I2C
    Wire.begin();
    Wire.setSCL(19);
    Wire.setSDA(18);
    Wire.setClock(100000);

    Serial.write("Motor driver I2C initialized");

    // Initialize IMU I2C (note: uses Wire object)
    pinMode(IMU_INT, INPUT);
    pinMode(RESET_IMU, OUTPUT);

    // Initialize I2C 1
    Wire1.begin();
    Wire1.setSCL(16);
    Wire1.setSDA(17);

    Serial.write("I2C 1 initialized");

    // Initialize I2C 2
    Wire2.begin();
    Wire2.setSCL(24);
    Wire2.setSDA(25);

    Serial.write("I2C 2 initialized");

    // Initialize UART 1
    // Serial2.begin(9600);

    // Initialize UART 2
    // Serial8.begin(9600);

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


    // Configure 
}

void loop() {
    
}