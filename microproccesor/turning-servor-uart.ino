#define ARDUINO 101

#include <TMCStepper.h>     // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h> // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>      // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/
#include <MultiStepper.h>
#include <AccelStepper.h>

#define DIR_PIN 0           // Direction - WHITE
#define STEP_PIN 1          // Step - ORANGE
#define SW_RX 2             // SoftwareSerial transmit pin - YELLOW
#define SW_TX 3             // SoftwareSerial receive pin - BROWN
#define EN_PIN 4            // Enable - PURPLE
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

constexpr uint32_t steps_per_mm = 80;

const byte MOTOR_PIN_OFFSETS[] = {2, 9, 14}; // each offset is summed with the pin positions above to give the actual position on the arduino
const int sizeMotors = sizeof(MOTOR_PIN_OFFSETS) / sizeof(byte);

// simulate serial interfaces using digital pins because arduino doesn't have enough physical serial interfaces
SoftwareSerial SoftSerials[] = {
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[0], SW_TX + MOTOR_PIN_OFFSETS[0]),
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[1], SW_TX + MOTOR_PIN_OFFSETS[1]),
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[2], SW_TX + MOTOR_PIN_OFFSETS[2]),
};

// set tmc driver settings
TMC2209Stepper TMCdrivers[] = {
    TMC2209Stepper((Stream *)&SoftSerials[0], R_SENSE, DRIVER_ADDRESS),
    TMC2209Stepper((Stream *)&SoftSerials[1], R_SENSE, DRIVER_ADDRESS),
    TMC2209Stepper((Stream *)&SoftSerials[2], R_SENSE, DRIVER_ADDRESS),
};

// run motors in parallel
AccelStepper steppers[] = {
    AccelStepper(AccelStepper::DRIVER, MOTOR_PIN_OFFSETS[0] + STEP_PIN, MOTOR_PIN_OFFSETS[0] + DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, MOTOR_PIN_OFFSETS[1] + STEP_PIN, MOTOR_PIN_OFFSETS[1] + DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, MOTOR_PIN_OFFSETS[2] + STEP_PIN, MOTOR_PIN_OFFSETS[2] + DIR_PIN),
};

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

void setup()
{
    for (int i = 0; i < sizeMotors; i++)
    {
        SoftSerials[i].begin(11520);      // initialize software serial for UART motor control
        TMCdrivers[i].beginSerial(11520); // Initialize UART
        TMCdrivers[i].begin();            // UART: Init SW UART (if selected) with default 115200 baudrate
        TMCdrivers[i].toff(5);            // Enables driver in software
        TMCdrivers[i].rms_current(400);   // Set motor RMS current
        TMCdrivers[i].microsteps(16);     // Set microsteps

        TMCdrivers[i].en_spreadCycle(false);
        TMCdrivers[i].pwm_autoscale(true); // Needed for stealthChop

        steppers[i].setMaxSpeed(800 * steps_per_mm);      // 100mm/s @ 80 steps/mm
        steppers[i].setAcceleration(1000 * steps_per_mm); // 2000mm/s^2
        steppers[i].setEnablePin(EN_PIN + MOTOR_PIN_OFFSETS[i]);
        steppers[i].setPinsInverted(false, false, true);
        steppers[i].enableOutputs();
    }
}

void loop()
{
    for (int i = 0; i < sizeMotors; i++)
    {
        if (steppers[i].distanceToGo() == 0)
        {
            steppers[i].move(100 * steps_per_mm); // Move 100mm
        }
        steppers[i].run();
    }
}