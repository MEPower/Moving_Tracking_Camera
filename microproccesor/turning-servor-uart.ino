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

const byte MOTOR_PIN_OFFSETS[] = {2, 9}; // each offset is summed with the pin positions above to give the actual position on the arduino
const int sizeMotors = sizeof(MOTOR_PIN_OFFSETS) / sizeof(byte);

// simulate serial interfaces using digital pins because arduino doesn't have enough physical serial interfaces
SoftwareSerial SoftSerials[] = {
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[0], SW_TX + MOTOR_PIN_OFFSETS[0]),
    SoftwareSerial(SW_RX + MOTOR_PIN_OFFSETS[1], SW_TX + MOTOR_PIN_OFFSETS[1]),
};

// set tmc driver settings
TMC2209Stepper TMCdrivers[] = {
    TMC2209Stepper((Stream *)&SoftSerials[0], R_SENSE, DRIVER_ADDRESS),
    TMC2209Stepper((Stream *)&SoftSerials[1], R_SENSE, DRIVER_ADDRESS),
};

// run motors in parallel
AccelStepper steppers[] = {
    AccelStepper(AccelStepper::DRIVER, MOTOR_PIN_OFFSETS[0] + STEP_PIN, MOTOR_PIN_OFFSETS[0] + DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, MOTOR_PIN_OFFSETS[1] + STEP_PIN, MOTOR_PIN_OFFSETS[1] + DIR_PIN),
};

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

void setup()
{
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // turn the LED on

    for (int i = 0; i < sizeMotors; i++)
    {
        SoftSerials[i].begin(11520);      // initialize software serial for UART motor control
        TMCdrivers[i].beginSerial(11520); // Initialize UART
        TMCdrivers[i].begin();            // UART: Init SW UART (if selected) with default 115200 baudrate
        TMCdrivers[i].toff(5);            // Enables driver in software
        TMCdrivers[i].rms_current(1200);  // Set motor RMS current
        TMCdrivers[i].microsteps(8);      // Set microsteps

        TMCdrivers[i].en_spreadCycle(false);
        TMCdrivers[i].pwm_autoscale(true); // Needed for stealthChop

        steppers[i].setMaxSpeed(800 * steps_per_mm);     // 100mm/s @ 80 steps/mm
        steppers[i].setAcceleration(100 * steps_per_mm); // 2000mm/s^2
        steppers[i].setEnablePin(EN_PIN + MOTOR_PIN_OFFSETS[i]);
        steppers[i].setPinsInverted(false, false, true);
        steppers[i].enableOutputs();
    }

    // send ready
    Serial.write('r');
}

void loop()
{

    for (int i = 0; i < sizeMotors; i++)
    {
        steppers[i].run();
    }

    if (Serial.available() > 1)
    {

        // 0-9 allowed
        char x = Serial.read();
        char y = Serial.read();

        // debug
        // Serial.print(x);
        // Serial.print(y);

        // convert chars to int. Substract 5 to put 0 in the middle
        int xVelocity = (x - '0') - 5;
        int yVelocity = (y - '0') - 5;

        steppers[0].setMaxSpeed(10 * abs(yVelocity) * steps_per_mm);
        steppers[1].setMaxSpeed(10 * abs(xVelocity) * steps_per_mm);

        int sign0 = yVelocity > 0 ? 1 : -1;
        steppers[0].move(1 * steps_per_mm * sign0);
        int sign1 = xVelocity > 0 ? 1 : -1;
        steppers[1].move(10 * steps_per_mm * sign1);

        Serial.print(yVelocity);
        Serial.print(xVelocity);
    }
}
