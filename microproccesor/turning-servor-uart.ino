#include <TMCStepper.h>     // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h> // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>      // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/

#define DIR_PIN 0           // Direction - WHITE
#define STEP_PIN 1          // Step - ORANGE
#define SW_RX 2             // SoftwareSerial transmit pin - YELLOW
#define SW_TX 3             // SoftwareSerial receive pin - BROWN
#define EN_PIN 4            // Enable - PURPLE
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

const byte MOTOR_PIN_OFFSETS[] = {2, 9}; // each offset is summed with the pin positions above to give the actual position on the arduino

SoftwareSerial SoftSerials[] = {
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[0], SW_TX + MOTOR_PIN_OFFSETS[0]),
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[1], SW_TX + MOTOR_PIN_OFFSETS[1]),
};

TMC2209Stepper TMCdrivers[] = {
    TMC2209Stepper((Stream *)&SoftSerials[0], R_SENSE, DRIVER_ADDRESS),
    TMC2209Stepper((Stream *)&SoftSerials[1], R_SENSE, DRIVER_ADDRESS),
};

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

void setup()
{
    int sizeMotors = sizeof(MOTOR_PIN_OFFSETS) / sizeof(byte);
    for (int i = 0; i < sizeMotors; i++)
    {
        SoftSerials[i].begin(11520);      // initialize software serial for UART motor control
        TMCdrivers[i].beginSerial(11520); // Initialize UART

        pinMode(EN_PIN + MOTOR_PIN_OFFSETS[i], OUTPUT); // Set pinmodes
        pinMode(STEP_PIN + MOTOR_PIN_OFFSETS[i], OUTPUT);
        pinMode(DIR_PIN + MOTOR_PIN_OFFSETS[i], OUTPUT);
        digitalWrite(EN_PIN + MOTOR_PIN_OFFSETS[i], LOW); // Enable TMC2209 board

        TMCdrivers[i].begin();          // UART: Init SW UART (if selected) with default 115200 baudrate
        TMCdrivers[i].toff(5);          // Enables driver in software
        TMCdrivers[i].rms_current(500); // Set motor RMS current
        TMCdrivers[i].microsteps(256);  // Set microsteps

        TMCdrivers[i].en_spreadCycle(false);
        TMCdrivers[i].pwm_autoscale(true); // Needed for stealthChop
    }
}

void loop()
{
    int sizeMotors = sizeof(MOTOR_PIN_OFFSETS) / sizeof(byte);
    for (int n = 0; n < sizeMotors; n++)
    {
        accel = 10000;          // Speed increase/decrease amount
        maxSpeed = 50000;       // Maximum speed to be reached
        speedChangeDelay = 100; // Delay between speed changes

        for (long i = 0; i <= maxSpeed; i = i + accel)
        {                             // Speed up to maxSpeed
            TMCdrivers[n].VACTUAL(i); // Set motor speed
            delay(100);
        }

        for (long i = maxSpeed; i >= 0; i = i - accel)
        { // Decrease speed to zero
            TMCdrivers[n].VACTUAL(i);
            delay(100);
        }

        dir = !dir;               // REVERSE DIRECTION
        TMCdrivers[n].shaft(dir); // SET DIRECTION
    }
}