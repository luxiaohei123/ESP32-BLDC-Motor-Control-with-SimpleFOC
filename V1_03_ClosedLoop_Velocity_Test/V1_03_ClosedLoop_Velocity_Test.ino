/*
  ============================================================
  V1_03_ClosedLoop_Velocity_Test
  ------------------------------------------------------------
  Purpose:
  1. Use AS5600 as position and velocity feedback
  2. Implement closed-loop velocity control for the 2804 motor with SimpleFOC
  3. Control forward rotation, stop, and reverse rotation through serial commands

  Hardware:
  - MCU: DOIT ESP32 DevKit V1
  - Driver: SimpleFOC mini
  - Sensor: AS5600 magnetic encoder
  - Motor: 2804 BLDC gimbal motor, pole pairs = 7
  - Motor supply: 12V

  AS5600 Wiring:
  AS5600 VCC -> ESP32 3.3V
  AS5600 GND -> ESP32 GND
  AS5600 SDA -> ESP32 GPIO21
  AS5600 SCL -> ESP32 GPIO22

  SimpleFOC mini Wiring:
  IN1 -> GPIO32
  IN2 -> GPIO33
  IN3 -> GPIO25
  EN  -> GPIO12
  GND -> ESP32 GND

  Power:
  - ESP32: powered through USB
  - SimpleFOC mini: powered by external 12V supply
  - ESP32 GND, SimpleFOC mini GND, and 12V negative terminal must share a common ground

  Recommended Power-up Sequence:
  1. Upload this sketch
  2. Open Serial Monitor at 115200 baud with line ending set to Newline
  3. Apply 12V power to the SimpleFOC mini board
  4. Press EN / RESET on the ESP32
  5. Wait until initFOC finishes

  Serial Commands:
  T3      -> slow closed-loop forward rotation
  T6.28   -> forward rotation, approximately 1 rotation/s
  T0      -> stop
  T-6.28  -> reverse rotation, approximately 1 rotation/s
  L4      -> set motor.voltage_limit to 4V
  L5      -> set motor.voltage_limit to 5V

  Verified in this V1 setup:
  - initFOC completed successfully
  - T6.28 forward rotation worked
  - T0 stop worked
  - T-6.28 reverse rotation worked
  ============================================================
*/

#include <SimpleFOC.h>
#include <Wire.h>

// Create the AS5600 I2C sensor object.
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Create the 2804 motor object.
// The motor used in this project has 7 pole pairs.
BLDCMotor motor = BLDCMotor(7);

// Configure the SimpleFOC mini 3PWM driver.
// Parameter order: IN1, IN2, IN3, EN
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

// Create the serial command parser.
Commander command = Commander(Serial);

// Serial command T: update the closed-loop target velocity.
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
  Serial.println("V1_03_ClosedLoop_Velocity_Test");
  Serial.println("Closed-loop Velocity Control");
  Serial.println("Hardware: ESP32 + AS5600 + SimpleFOC mini + 2804");
  Serial.println("SDA=GPIO21, SCL=GPIO22");
  Serial.println("IN1=GPIO32, IN2=GPIO33, IN3=GPIO25, EN=GPIO12");
  Serial.println("==================================================");

  // Enable SimpleFOC debug output.
  // This helps display initialization and sensor alignment messages.
  SimpleFOCDebug::enable(&Serial);

  // ============================================================
  // Initialize AS5600
  // ============================================================

  // Initialize ESP32 I2C pins: SDA = 21, SCL = 22.
  Wire.begin(21, 22);

  // Initialize the AS5600 encoder.
  sensor.init();

  // Link the sensor to the motor object.
  // Closed-loop control uses this sensor for rotor position and velocity feedback.
  motor.linkSensor(&sensor);

  Serial.println("Sensor init success.");

  // ============================================================
  // Initialize the driver
  // ============================================================

  // External motor supply voltage of the SimpleFOC mini board.
  driver.voltage_power_supply = 12;

  // Driver-side voltage limit.
  driver.voltage_limit = 6;

  // Initialize the motor driver.
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  Serial.println("Driver init success.");

  // Link the motor object to the driver.
  motor.linkDriver(&driver);

  // ============================================================
  // Configure closed-loop velocity control
  // ============================================================

  // Use voltage mode as torque control.
  // This V1 hardware setup does not use a current control loop.
  motor.torque_controller = TorqueControlType::voltage;

  // Enable closed-loop velocity control.
  motor.controller = MotionControlType::velocity;

  // Output voltage limit used during normal motor control.
  motor.voltage_limit = 3;

  // Velocity limit in rad/s.
  motor.velocity_limit = 20;

  // Sensor alignment voltage used during initFOC().
  motor.voltage_sensor_align = 3;

  // Velocity-loop PID parameters.
  // These values have been validated as a basic working set for this V1 setup.
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;

  // Low-pass filter for velocity estimation.
  motor.LPF_velocity.Tf = 0.02;

  // Start from zero target speed to prevent immediate motion after boot.
  motor.target = 0;

  // Initialize the motor object.
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }
  Serial.println("Motor init success.");

  // ============================================================
  // FOC alignment and closed-loop initialization
  // ============================================================

  Serial.println("Running initFOC...");

  // initFOC performs:
  // 1. Sensor direction detection
  // 2. Electrical zero-angle alignment
  // 3. Motor-sensor relationship calibration
  motor.initFOC();

  Serial.println("FOC init done.");

  // Register serial commands.
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println();
  Serial.println("Motor ready.");
  Serial.println("Commands:");
  Serial.println("T3      -> slow closed-loop rotate");
  Serial.println("T6.28   -> forward, about 1 rotation/s");
  Serial.println("T0      -> stop");
  Serial.println("T-6.28  -> reverse, about 1 rotation/s");
  Serial.println("L4      -> set voltage limit to 4V");
  Serial.println("L5      -> set voltage limit to 5V");
}

void loop() {
  // Main FOC loop:
  // Read sensor feedback, compute electrical angle, and update three-phase PWM.
  // This function should run as frequently as possible.
  motor.loopFOC();

  // Closed-loop velocity control:
  // Compute the control output from motor.target and encoder feedback.
  motor.move();

  // Parse incoming serial commands.
  command.run();
}
