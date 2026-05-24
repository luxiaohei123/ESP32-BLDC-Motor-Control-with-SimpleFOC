/*
  ============================================================
  V1_02_2804_OpenLoop_Test
  ------------------------------------------------------------
  Purpose:
  1. Test the motor driver without using encoder feedback
  2. Verify that ESP32 + SimpleFOC mini can drive the 2804 motor
  3. Control forward rotation, stop, and reverse rotation through serial commands

  Hardware:
  - MCU: DOIT ESP32 DevKit V1
  - Driver: SimpleFOC mini
  - Motor: 2804 BLDC gimbal motor, pole pairs = 7
  - Motor supply: 12V

  Wiring:
  SimpleFOC mini IN1 -> ESP32 GPIO32
  SimpleFOC mini IN2 -> ESP32 GPIO33
  SimpleFOC mini IN3 -> ESP32 GPIO25
  SimpleFOC mini EN  -> ESP32 GPIO12
  SimpleFOC mini GND -> ESP32 GND

  Power:
  - ESP32: powered through USB
  - SimpleFOC mini: powered by external 12V supply
  - ESP32 GND, SimpleFOC mini GND, and 12V negative terminal must share a common ground

  Serial Commands:
  T6.28   -> target speed about 6.28 rad/s, approximately 1 rotation/s
  T0      -> stop
  T-6.28  -> reverse rotation, approximately 1 rotation/s
  L4      -> set motor.voltage_limit to 4V
  L5      -> set motor.voltage_limit to 5V

  Notes:
  - Open-loop control does not use encoder feedback
  - Low target values such as T1 may not be enough to start this gimbal motor
  - T6.28 has been verified as a reliable open-loop startup command for this V1 setup
  - Do not run open-loop tests for a long time, because the motor may heat up
  ============================================================
*/

#include <SimpleFOC.h>

// The 2804 motor used in this project has 7 pole pairs.
BLDCMotor motor = BLDCMotor(7);

// Configure the 3 PWM input pins and the enable pin.
// Parameter order: IN1, IN2, IN3, EN
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

// Create the serial command parser.
Commander command = Commander(Serial);

// Serial command T: update the target velocity.
// Example: T6.28 sets motor.target = 6.28 rad/s.
void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
}

// Serial command L: update the motor voltage limit.
// Example: L4 sets motor.voltage_limit = 4V.
void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================================");
  Serial.println("V1_02_2804_OpenLoop_Test");
  Serial.println("2804 Open-loop Velocity Test");
  Serial.println("Motor pole pairs = 7");
  Serial.println("IN1=GPIO32, IN2=GPIO33, IN3=GPIO25, EN=GPIO12");
  Serial.println("==================================================");

  // Enable SimpleFOC debug output.
  // This prints more information during initialization.
  SimpleFOCDebug::enable(&Serial);

  // The SimpleFOC mini motor supply is 12V.
  driver.voltage_power_supply = 12;

  // Driver-side voltage limit.
  // The actual motor output limit is further constrained by motor.voltage_limit.
  driver.voltage_limit = 6;

  // Initialize PWM generation on the driver.
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  Serial.println("Driver init success.");

  // Link the motor object to the driver.
  motor.linkDriver(&driver);

  // Initial motor voltage limit for safe testing.
  motor.voltage_limit = 3;

  // Select open-loop velocity control.
  // No encoder feedback is used in this mode.
  motor.controller = MotionControlType::velocity_openloop;

  // Start from zero target speed to prevent immediate motion after boot.
  motor.target = 0;

  // Initialize the motor object.
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }
  Serial.println("Motor init success.");

  // Register serial commands.
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println();
  Serial.println("Motor ready.");
  Serial.println("Commands:");
  Serial.println("T6.28   -> forward, about 1 rotation/s");
  Serial.println("T0      -> stop");
  Serial.println("T-6.28  -> reverse, about 1 rotation/s");
  Serial.println("L4      -> set voltage limit to 4V");
  Serial.println("L5      -> set voltage limit to 5V");
}

void loop() {
  // Kept here for structural consistency with the closed-loop sketch.
  motor.loopFOC();

  // Run open-loop velocity control using motor.target.
  motor.move();

  // Parse incoming serial commands.
  command.run();
}
