/*
  ============================================================
  V1_01_AS5600_Sensor_Test
  ------------------------------------------------------------
  Purpose:
  1. Test whether the AS5600 magnetic encoder works correctly
  2. Read the rotor angle and angular velocity
  3. Print the sensor data to the Arduino Serial Monitor

  Hardware:
  - MCU: DOIT ESP32 DevKit V1
  - Sensor: AS5600 magnetic encoder

  Wiring:
  AS5600 VCC  -> ESP32 3.3V
  AS5600 GND  -> ESP32 GND
  AS5600 SDA  -> ESP32 GPIO21
  AS5600 SCL  -> ESP32 GPIO22

  Test Procedure:
  1. Power the ESP32 through USB only
  2. Do NOT apply 12V power to the SimpleFOC mini board
  3. Upload this sketch
  4. Open Serial Monitor at 115200 baud
  5. Rotate the 2804 motor rotor by hand
  6. Check whether Angle and Velocity change accordingly

  Pass Criteria:
  - Angle(rad) changes when the rotor is turned by hand
  - Velocity(rad/s) changes when the rotor is turned by hand
  - Velocity stays close to 0 when the rotor is stationary
  ============================================================
*/

#include <SimpleFOC.h>
#include <Wire.h>

// Create an AS5600 I2C magnetic sensor object.
// AS5600_I2C is predefined by the SimpleFOC library.
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  // Start serial communication.
  // The Arduino Serial Monitor must also be set to 115200 baud.
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================================");
  Serial.println("V1_01_AS5600_Sensor_Test");
  Serial.println("AS5600 I2C Sensor Test");
  Serial.println("SDA = GPIO21, SCL = GPIO22");
  Serial.println("Do NOT power the motor driver in this test.");
  Serial.println("Rotate the motor rotor by hand.");
  Serial.println("==================================================");

  // Initialize ESP32 I2C pins.
  // Wire.begin(SDA, SCL)
  Wire.begin(21, 22);

  // Initialize the AS5600 sensor.
  sensor.init();

  Serial.println("AS5600 init done.");
  Serial.println("Angle(rad)\tVelocity(rad/s)");
}

void loop() {
  // Update the sensor state before reading angle or velocity.
  sensor.update();

  // Print the accumulated angle in radians.
  Serial.print(sensor.getAngle());

  // Print a tab separator for cleaner serial output.
  Serial.print("\t");

  // Print the angular velocity in rad/s.
  Serial.println(sensor.getVelocity());

  // Print data every 100 ms to avoid excessive serial output.
  delay(100);
}
