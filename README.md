

# Bheater A DIY Battery ECU & firmware.

This project is designed for light electric vehicles to provide control of different battery uses by including charger status, battery heating, and an auxiliary heater. It also offers a few extra 3V3 tolerant GPIO for additional user-managed features.

(Status: Basic functionality around heating and charging is written and done. All the extra features are still missing)

Software Features
- Charger Status Control: Manages the status of the battery charger.
- Battery Heater Control: Provides control over a heater used to maintain the battery temperature.
- Auxiliary Heater Control: An additional heater control for optional features like a bench warmer or grip warmer.
- GPIO: Additional 3V3 tolerant GPIO for user-managed features or optional hardware functionality.

Hardware
- ESP32-D0WD MCU
- Dallas DS18B20 sensor
- Charger Control (P-fet): Controls the charging of the battery, with a capacity of around 5A.
- Ignition Control (P-fet): Manages the ignition system.
- 2 PID controlled PWM output (N-fet): Two outputs controlled by a PID (Proportional-Integral-Derivative) controller provide precise voltage and current control.
- The N-fet voltage range is 25V - 100V and the current capacity is around 5A.

Connectivity
- Wifi/MQTT: Connects to Home Assistant for smart home integration.
- BluetoothSerial: Provides Classical Bluetooth connectivity for basic command-line interface control.
- Android App (coming): An upcoming Android app will provide a user-friendly interface for controlling and monitoring the system. (Link to the Velluz app)

